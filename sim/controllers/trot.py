"""규칙 기반 트롯 보행기 (학습 정책이 준비되기 전의 기준 컨트롤러).

목표 관절각을 계산한 뒤 제어 인터페이스(명세)의 action으로 바꿔 내보낸다.
ONNX 정책과 같은 인터페이스(reset/act)를 따르므로 runner에서 서로 바꿔 끼울 수 있다.
"""
import numpy as np

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


class TrotController:
    name = "trot"

    def __init__(self, spec, cfg, interface):
        self.iface = interface
        leg = spec["robot"]["leg"]
        self.l1, self.l2 = leg["thigh_length"], leg["calf_length"]
        self.hip_xy = np.array(leg["hip_offsets"])
        self.track = 2 * (abs(self.hip_xy[0, 1]) + 0.0955)     # 좌우 발 간격
        self.swing_h = cfg["swing_height"]
        self.stand_h = cfg["stand_height"]
        self.default = np.array(spec["robot"]["default_pose"], dtype=float)
        self.k_vel = cfg.get("velocity_feedback", 2.0)   # 보폭에 속도 오차 피드백 (P)
        self.ki_vel = cfg.get("velocity_integral", 2.0)  # 정상상태 오차 제거 (I)
        self.k_heading = cfg.get("heading_gain", 1.5)    # 목표 방위 유지
        self.reset()

    def reset(self):
        self.meas_f = np.zeros(2)       # 측정 [vx(몸통), yaw_rate] 저역통과
        self.err_i = np.zeros(2)
        self.heading_des = None

    def act(self, ctx):
        """StepContext -> action (인터페이스 규약). 위상과 명령은 ctx.clock에서 읽는다."""
        q = self.joint_targets(ctx.dt, ctx.clock, ctx.roll, ctx.pitch, ctx.yaw, ctx.v_body_x, ctx.wz_world)
        return self.iface.action_from_targets(q)

    def joint_targets(self, dt, clock, roll, pitch, yaw, meas_vx, meas_wz):
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

        q = self.default.copy()
        for i in range(4):
            stride = (vx - SIDE[i] * wz * self.track / 2) * period / 2
            p = (clock.phase + PHASE_OFFSET[i]) % 1.0
            if not moving:
                x, lift = 0.0, 0.0
            elif p < 0.5:                         # stance: 발을 뒤로 민다
                s = p / 0.5
                x, lift = stride * (0.5 - s), 0.0
            else:                                 # swing: 들어서 앞으로
                s = (p - 0.5) / 0.5
                x = stride * (-0.5 + (1 - np.cos(np.pi * s)) / 2)
                lift = self.swing_h * np.sin(np.pi * s)
            # 자세 보정: 기울어진 쪽 다리 길이를 조정해 몸통 수평 유지
            hx, hy = self.hip_xy[i]
            level = hx * np.tan(pitch) - hy * np.tan(roll)
            z = -(self.stand_h + level) + lift
            q[3 * i + 1], q[3 * i + 2] = leg_ik(x, z, self.l1, self.l2)
            q[3 * i] = 0.0
        return q
