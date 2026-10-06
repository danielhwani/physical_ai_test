"""규칙 기반 트롯 보행기 (학습 정책이 준비되기 전의 기준 컨트롤러).

목표 관절각을 계산한 뒤 제어 인터페이스(명세)의 action으로 바꿔 내보낸다.
ONNX 정책과 같은 인터페이스(reset/act)를 따르므로 runner에서 서로 바꿔 끼울 수 있다.

지형 인지(ctx.terrain, LiDAR 높이 지도)가 있으면 발마다:
  - 디딜 곳: 공중에 있는 동안 착지 예상 위치 주변에서 평탄한 곳을 고른다 (control/foothold.py, 앞뒤 ±6 cm, 옆 ±4 cm)
  - 높이 맞추기: 고른 자리의 지면 높이에 맞춰 발을 내린다. 몸통 높이는 네 발 지면 높이 평균을 천천히 따라간다
  - 턱 넘기: 들어 올린 곳과 디딜 곳 사이 최고 지면보다 발을 더 든다
지형 인지가 없으면 이전과 같은 계산이다 (결과가 비트 단위로 같음).
"""
import numpy as np

from .foothold import FootholdPlanner

# 대각 다리 쌍: FL+RR, FR+RL
PHASE_OFFSET = np.array([0.0, 0.5, 0.5, 0.0])
SIDE = np.array([1, -1, 1, -1])   # +1 = 왼쪽 다리


def leg_ik(x, z, l1, l2):
    """시상면 2링크 IK. 발 위치(x 전방, z 위, hip 기준) -> (thigh, calf)."""
    d2 = x * x + z * z
    c = np.clip((d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1.0, 1.0)
    q2 = -np.arccos(c)
    q1 = np.arctan2(-x, -z) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    return q1, q2


def leg_ik3(x, y, z, a, l1, l2):
    """발 위치(고관절 기준 몸통 좌표: x 전방, y 왼쪽, z 위) -> (hip, thigh, calf). a: 고관절 옆 간격 (왼쪽 다리 +, 오른쪽 -).
    기구학(control/kinematics.py): foot = Rx(q0) [x', a, z'] 이고 (x', z')는 시상면 2링크."""
    zp = -np.sqrt(max(y * y + z * z - a * a, 1e-6))
    q0 = np.arctan2(z, y) - np.arctan2(zp, a)
    q1, q2 = leg_ik(x, zp, l1, l2)
    return q0, q1, q2


class TrotController:
    name = "trot"

    def __init__(self, spec, cfg, interface):
        self.iface = interface
        leg = spec["robot"]["leg"]
        self.l1, self.l2 = leg["thigh_length"], leg["calf_length"]
        self.hip_xy = np.array(leg["hip_offsets"])
        self.track = 2 * (abs(self.hip_xy[0, 1]) + leg["abduction_offset"])     # 좌우 발 간격
        self.swing_h = cfg["swing_height"]
        self.stand_h = cfg["stand_height"]
        self.default = np.array(spec["robot"]["default_pose"], dtype=float)
        self.k_vel = cfg.get("velocity_feedback", 2.0)   # 보폭에 속도 오차 피드백 (P)
        self.ki_vel = cfg.get("velocity_integral", 2.0)  # 정상상태 오차 제거 (I)
        self.k_heading = cfg.get("heading_gain", 1.5)    # 목표 방위 유지
        self.abd = SIDE * leg["abduction_offset"]                          # 고관절 옆 간격 (다리별 부호)
        self.side_y = self.hip_xy[:, 1] + self.abd                         # 발의 몸통 y
        pcfg = cfg.get("perception") or {}
        self.planner = FootholdPlanner(pcfg.get("foothold"))
        self.plan_until = pcfg.get("plan_until", 0.7)       # 공중 단계 중 이 비율까지만 디딜 곳을 다시 고른다
        self.max_rel = pcfg.get("max_step_height", 0.10)    # 몸통 기준 지면 높이 차 한계 (다리 길이)
        self.h_tau = pcfg.get("body_height_tau", 0.3)       # 몸통 높이가 지면 평균을 따라가는 시간 상수 (s)
        # (시험용) 발마다 높이 맞추기를 높이 차 [lo, hi]에서 0 -> 전부로 줄인다. 기본 끔: 켜면 vehicle_crossing에서 넘어짐이 늘었다
        self.rel_band = pcfg.get("step_height_band")
        self.clear_band = pcfg.get("clearance_band", 0.02)
        self.reset()

    def reset(self):
        self.meas_f = np.zeros(2)       # 측정 [vx(몸통), yaw_rate] 저역통과
        self.err_i = np.zeros(2)
        self.heading_des = None
        # 지형 인지용 다리 상태 (odom 좌표 지면 높이, 진행 방향 발 옮김)
        self.in_swing = np.zeros(4, bool)
        self.dx, self.dx_lo, self.dx_td = np.zeros(4), np.zeros(4), np.zeros(4)
        self.dy, self.dy_lo, self.dy_td = np.zeros(4), np.zeros(4), np.zeros(4)
        self.h = self.h_lo = self.h_td = self.h_ref = None
        self.lo_xy = np.zeros((4, 2))
        self.extra = np.zeros(4)
        self.footholds = [None] * 4        # 마지막으로 고른 디딜 곳 (기록/화면용)
        self.nominal = np.full((4, 2), np.nan)   # 지형을 모를 때 디딜 위치 (화면용)

    def act(self, ctx):
        """StepContext -> action (인터페이스 규약). 위상과 명령은 ctx.clock에서 읽는다."""
        q = self.joint_targets(ctx.dt, ctx.clock, ctx.roll, ctx.pitch, ctx.yaw, ctx.v_body_x, ctx.wz_world,
                               terrain=ctx.terrain, est=ctx.est)
        return self.iface.action_from_targets(q)

    def _plan(self, i, s, stride, period, terrain, est, pose, first):
        """공중에 있는 다리 i의 디딜 곳을 고른다 (착지 예상 위치 = 지금 몸통 + 남은 시간 동안 이동)."""
        R, p = pose
        yaw = np.arctan2(R[1, 0], R[0, 0])
        fwd = np.array([np.cos(yaw), np.sin(yaw)])
        left = np.array([-fwd[1], fwd[0]])
        t_rem = (1.0 - s) * period / 2
        v_xy = (R @ est.v_body)[:2]
        nominal = p[:2] + (self.hip_xy[i, 0] + stride / 2) * fwd + self.side_y[i] * left + v_xy * t_rem
        prev = None if first else (self.dx_td[i], self.dy_td[i])
        fh = self.planner.choose(terrain.map, nominal, fwd, prev)
        self.footholds[i], self.nominal[i] = fh, nominal
        self.dx_td[i], self.dy_td[i] = fh.shift, fh.side
        self.h_td[i] = fh.height if np.isfinite(fh.height) else self.h_lo[i]   # 모르는 곳은 같은 높이로 본다
        top = self.planner.path_max(terrain.map, self.lo_xy[i], fh.xy)
        # 두 끝보다 clear_band 넘게 높은 턱만 더 넘는다 (작은 요철은 기본 swing_height로 충분. 지도 잡음에 매번 높이 들지 않게)
        self.extra[i] = max(0.0, top - max(self.h_lo[i], self.h_td[i]) - self.clear_band) if np.isfinite(top) else 0.0

    def joint_targets(self, dt, clock, roll, pitch, yaw, meas_vx, meas_wz, terrain=None, est=None):
        cmd_f, period = clock.cmd_f, clock.period
        self.meas_f += 0.1 * (np.array([meas_vx, meas_wz]) - self.meas_f)
        # 방위: 명령 yaw rate를 적분한 목표 방위와의 오차를 yaw rate 명령에 더한다
        if self.heading_des is None:
            self.heading_des = yaw
        self.heading_des += cmd_f[1] * dt
        heading_err = (self.heading_des - yaw + np.pi) % (2 * np.pi) - np.pi
        target = cmd_f + [0.0, self.k_heading * heading_err]
        # 개루프 보폭은 추종 지연·미끄러짐으로 속도가 모자라므로 측정 오차만큼 보폭을 키운다
        err = target - self.meas_f
        self.err_i = np.clip(self.err_i + err * dt, -0.5, 0.5)
        vx, wz = target + self.k_vel * err + self.ki_vel * self.err_i
        vx, wz = np.clip(vx, -1.0, 1.0), np.clip(wz, -1.5, 1.5)
        moving = clock.moving

        if terrain is not None:
            pose = terrain.pose(est)
            feet = terrain.feet_world(est)
            if self.h is None:                    # 처음: 서 있는 발의 접촉점 높이
                self.h = feet[:, 2] - terrain.kin.r
                self.h_lo, self.h_td = self.h.copy(), self.h.copy()
                self.h_ref = float(self.h.mean())
            ground_target = np.where(self.in_swing, self.h_td, self.h)
            self.h_ref += min(1.0, dt / self.h_tau) * (float(ground_target.mean()) - self.h_ref)

        q = self.default.copy()
        for i in range(4):
            stride = (vx - SIDE[i] * wz * self.track / 2) * period / 2
            p = (clock.phase + PHASE_OFFSET[i]) % 1.0
            swing = moving and p >= 0.5
            if terrain is not None:
                lifted = swing and not self.in_swing[i]
                if lifted:                                # 발을 듦: 지금 자리와 높이를 기억
                    self.dx_lo[i], self.dy_lo[i], self.h_lo[i] = self.dx[i], self.dy[i], self.h[i]
                    self.lo_xy[i] = feet[i, :2]
                elif not swing and self.in_swing[i]:      # 착지: 고른 자리가 이제 디딘 자리
                    self.dx[i], self.dy[i], self.h[i] = self.dx_td[i], self.dy_td[i], self.h_td[i]
                self.in_swing[i] = swing
                if swing and (p - 0.5) / 0.5 <= self.plan_until:
                    self._plan(i, (p - 0.5) / 0.5, stride, period, terrain, est, pose, lifted)
            rel = 0.0                                     # 이 발 지면 높이 - 몸통 기준 지면 높이
            y = self.dy[i]                                # 옆으로 옮긴 거리 (지형 인지일 때만)
            if not moving:
                x, lift = 0.0, 0.0
                if terrain is not None:
                    rel = self.h[i] - self.h_ref
            elif p < 0.5:                         # stance: 발을 뒤로 민다
                s = p / 0.5
                x, lift = stride * (0.5 - s), 0.0
                if terrain is not None:
                    x += self.dx[i]
                    rel = self.h[i] - self.h_ref
            else:                                 # swing: 들어서 앞으로
                s = (p - 0.5) / 0.5
                c = (1 - np.cos(np.pi * s)) / 2
                x = stride * (-0.5 + c)
                lift = self.swing_h * np.sin(np.pi * s)
                if terrain is not None:
                    x += self.dx_lo[i] * (1 - c) + self.dx_td[i] * c
                    y = self.dy_lo[i] * (1 - c) + self.dy_td[i] * c
                    lift += self.extra[i] * np.sin(np.pi * s)
                    dh = self.h_td[i] - self.h_lo[i]
                    # 올라갈 때는 앞 절반에 높이를 맞추고, 내려갈 때는 뒤 절반에 내린다 (턱에 발끝이 걸리지 않게)
                    u = min(1.0, s / 0.5) if dh > 0 else max(0.0, (s - 0.5) / 0.5)
                    ground = self.h_lo[i] + dh * (3 * u * u - 2 * u ** 3)
                    rel = ground - self.h_ref
            if terrain is not None and self.rel_band:
                lo, hi = self.rel_band
                rel = rel * float(np.clip((abs(rel) - lo) / (hi - lo), 0.0, 1.0))
            rel = float(np.clip(rel, -self.max_rel, self.max_rel))
            # 자세 보정: 기울어진 쪽 다리 길이를 조정해 몸통 수평 유지
            hx, hy = self.hip_xy[i]
            level = hx * np.tan(pitch) - hy * np.tan(roll)
            z = -(self.stand_h + level) + lift
            if terrain is not None:
                z += rel
                q[3 * i], q[3 * i + 1], q[3 * i + 2] = leg_ik3(x, self.abd[i] + y, z, self.abd[i], self.l1, self.l2)
                continue
            q[3 * i + 1], q[3 * i + 2] = leg_ik(x, z, self.l1, self.l2)
            q[3 * i] = 0.0
        return q
