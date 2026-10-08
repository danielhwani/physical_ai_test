"""보행 알고리즘 노드 (알고리즘 쪽): 로봇 상태 메시지 -> 관절 명령 메시지.

실제 로봇의 탑재 소프트웨어와 같은 경계다. 입력은 로봇이 아는 값뿐이다.
  로봇 상태 (Unitree LowState를 본뜸): t, q[12], dq[12], imu{quat(x,y,z,w), gyro, accel}, foot_force[4]
  이동 명령 (운용자): vx, yaw_rate
  관절 명령 (Unitree LowCmd를 본뜸): q_des[12], dq_des[12], kp[12], kd[12], tau_ff[12]   (모델 관절 순서)
  LiDAR 점군 (선택, 트롯의 perception.sensor): 센서 좌표 점들 + 스캔 시각. 센서의 참 자세는 받지 않는다
스캔은 찍힌 시각의 추정 자세로 지도에 넣고, 찍힌 다음 제어 주기부터 쓴다. 그래서 ROS로 받을 때 상태 메시지와
점군의 도착 순서가 바뀌어도(한 주기 안이면) 같은 프로세스 실행과 결과가 같다.
시뮬레이터(sim/)와 MuJoCo를 import하지 않는다. 같은 프로세스에서 직접 부르거나 control/ros_node.py로 ROS2 노드가 된다.
"""
from collections import deque

import numpy as np

from .clock import GaitClock, StepContext
from .estimator import LegOdometryEstimator
from .interface import ControlInterface, load_card
from .observation import height_scan_grid
from .onnx_policy import OnnxPolicyController
from .terrain_map import TerrainPerception
from .trot import TrotController

# 일어서기: 보행을 멈추고 지금 관절 각도에서 서 있는 자세(기본 자세)까지 천천히 옮긴 뒤(관절 최대 RECOVER_RATE rad/s,
# 최소 RECOVER_MIN_S) RECOVER_HOLD_S 동안 서 있다가 보행을 처음 위상부터 다시 시작한다. 관절이 따라오는 만큼만 진행한다
# (명령과 측정 각도 차이가 RECOVER_TRACK rad를 넘으면 멈춤): 토크가 모자라 못 따라가는 동안 명령이 실제 자세에서 멀어지지 않게.
# 들어가는 조건 (모두 로봇이 아는 값):
#   - 로봇 상태가 RECOVER_GAP_S 넘게 오지 않다가 다시 옴 (링크 두절). 두절 중 주저앉았는데 그 자세에서 곧바로 트롯을 이으면
#     큰 PD 오차가 한꺼번에 걸려 고꾸라졌다 (확인함)
#   - 디딘 다리가 명령보다 SAG_M 넘게 눌린 상태가 SAG_S 동안 이어짐 (토크 부족, 예: 배터리 저하). 그대로 걸으려 하면
#     토크가 돌아오는 순간 다리가 튕겨 몸이 떠서 돌았다 (-73°). 정상 보행은 0.5초 지속 처짐이 최대 0.9 cm (시나리오 5종 측정)
# 정상 실행에서는 동작하지 않는다
RECOVER_GAP_S, RECOVER_RATE, RECOVER_MIN_S, RECOVER_HOLD_S, RECOVER_TRACK = 0.1, 1.0, 0.3, 0.5, 0.15
SAG_M, SAG_S, CONTACT_N = 0.02, 0.5, 20.0


class ControllerNode:
    def __init__(self, spec, controller_cfg=None, policy=None, lidar_defs=None):
        """controller_cfg: 시나리오의 controller 절 (트롯). policy: 정책 카드 경로 (ONNX).
        lidar_defs: specs/go2_sensors.yaml의 lidars (장착 보정값). 트롯의 perception.sensor가 있을 때 필요."""
        self.terrain, self.sensor = None, None
        if policy:
            card = load_card(policy)
            self.iface = ControlInterface(spec, card)
            self.ctrl = OnnxPolicyController(card, self.iface)
            self.desc, period = f"onnx:{card['name']}", self.iface.gait_period
            if self.iface.obs.needs_height_scan:             # 지형 인지 정책: 카드의 perception 절 (센서, 지도 설정)
                pcfg = card.get("perception") or {}
                self._make_terrain(spec, lidar_defs, pcfg.get("sensor", "front_lidar"), pcfg.get("map"))
        else:
            self.iface = ControlInterface(spec)
            self.ctrl = TrotController(spec, controller_cfg, self.iface)
            self.desc, period = f"trot:{controller_cfg}", controller_cfg["period"]
            pcfg = controller_cfg.get("perception")
            if pcfg:
                self._make_terrain(spec, lidar_defs, pcfg["sensor"], pcfg.get("map"))
        self.clock = GaitClock(period or 1.0)
        self.estimator = LegOdometryEstimator(spec)
        self.delay_steps = spec["action"].get("delay_steps", 0)
        self.on_action = None        # (ctx, action) -> action. 모방학습(DAgger)에서 실행 action 교체용
        self.aux_obs_spec = None     # 모방학습: 학생 정책의 관측 규약 (ctx.aux_obs로 같은 입력의 학생 관측을 받는다)
        self.reset()

    def _make_terrain(self, spec, lidar_defs, sensor, map_cfg):
        assert lidar_defs and sensor in lidar_defs, f"지형 인지 센서 {sensor}의 장착 명세가 없음"
        self.sensor = sensor
        self.terrain = TerrainPerception(spec, lidar_defs[sensor], map_cfg)

    def height_scan(self, obs_spec, est):
        """관측 height_scan 입력: 격자점(몸통 방향 기준)의 지도 높이 - 몸통 높이 (odom, 모르면 NaN)."""
        if self.terrain is None:
            return np.full(obs_spec.term("height_scan")["dim"], np.nan)
        _, p = self.terrain.pose(est)
        grid = height_scan_grid(obs_spec.term("height_scan"))
        c, s = np.cos(est.yaw), np.sin(est.yaw)
        pts = p[:2] + grid @ np.array([[c, s], [-s, c]])        # Rz(yaw) @ grid
        return self.terrain.map.lookup(pts) - p[2]

    def reset(self):
        self.ctrl.reset()
        self.clock.reset()
        self.estimator.reset()
        self.action = np.zeros(12)
        self.obs = np.zeros(self.iface.obs.dim)
        self.pending = deque([np.zeros(12)] * self.delay_steps)
        self.last_t, self.est = None, None
        self.q_des = self.iface.targets_from_action(self.action)
        self.recovery = None          # 일어서기 중: {"q0": 시작 관절 각도, "T": 옮기는 시간, "u": 진행 0~1, "hold": 서 있은 시간}
        self.sag_time = 0.0           # 디딘 다리가 눌린 상태가 이어진 시간
        self.last_q_des = None        # 지난 주기에 보낸 목표 관절 각도 (처짐 판단용)
        if self.terrain is not None:
            self.terrain.reset()
        self.scan_buf, self.pose_hist = [], {}

    def perception_snapshot(self):
        """화면용 지형 인지 상태 (odom 좌표). 지형 인지가 없으면 None.
        cells: 아는 칸 (N, 3) 중심 x, y, 높이 / h_ref: 몸통 기준 지면 높이 / legs: 다리별 고른 자리와 원래 자리."""
        if self.terrain is None or self.last_t is None:
            return None
        m, tr = self.terrain.map, self.ctrl
        rows, cols = np.nonzero(np.isfinite(m.h))
        xy = m.origin + (np.stack([cols, rows], 1) + 0.5) * m.res
        legs = []
        for i, fh in enumerate(getattr(tr, "footholds", [])):     # 디딜 곳은 트롯만 고른다 (정책은 지도만 표시)
            if fh is None:
                continue
            h = fh.height if np.isfinite(fh.height) else tr.h_td[i]
            legs.append({"leg": i, "swing": bool(tr.in_swing[i]), "chosen": [*fh.xy, h], "nominal": [*tr.nominal[i], h]})
        h_ref = getattr(tr, "h_ref", None)
        if h_ref is None:                                            # 정책: 몸통 높이 - 평지 기준 높이
            t = self.iface.obs.term("height_scan")
            h_ref = float(self.terrain.pose(self.est)[1][2] - (t["base_height_ref"] if t else 0.30))
        return {"t": self.last_t, "res": m.res, "cells": np.column_stack([xy, m.h[rows, cols]]),
                "h_ref": h_ref, "legs": legs}

    def on_scan(self, name, t, points):
        """LiDAR 점군 (센서 좌표, (N, 3)). 다음 제어 주기에 지도에 넣는다."""
        if self.terrain is not None and name == self.sensor:
            self.scan_buf.append((t, np.asarray(points, dtype=float)))

    def _integrate_scans(self, t, dt):
        ready = [sc for sc in self.scan_buf if sc[0] <= t - 0.5 * dt]
        self.scan_buf = [sc for sc in self.scan_buf if sc[0] > t - 0.5 * dt]
        for ts, pts in ready:
            key = min(self.pose_hist, key=lambda k: abs(k - ts)) if self.pose_hist else None
            if key is not None and abs(key - ts) < 0.5 * dt:
                self.terrain.add_scan(pts, *self.pose_hist[key])
        for k in [k for k in self.pose_hist if k < t - 1.0]:          # 1초 넘은 자세 기록은 버린다
            del self.pose_hist[k]

    def _start_recovery(self, q):
        q0 = np.asarray(q, dtype=float)
        stand = self.iface.targets_from_action(np.zeros(12))
        self.recovery = {"q0": q0, "T": float(np.clip(np.abs(stand - q0).max() / RECOVER_RATE, RECOVER_MIN_S, 2.0)),
                         "u": 0.0, "hold": 0.0}
        self.ctrl.reset()
        self.clock.cmd_f[:], self.clock.phase = 0.0, 0.0     # 명령 목표는 두고, 속도는 0에서 다시 가속
        self.sag_time = 0.0

    def _stance_sag(self, ls):
        """디딘 다리가 명령보다 눌린 정도 (m, 몸통 좌표 발 높이 차의 평균). 디딘 다리가 없으면 0."""
        q, q_cmd, kin = np.asarray(ls["q"], dtype=float), self.q_des, self.estimator.kin
        st = np.asarray(ls["foot_force"]) > CONTACT_N
        v = [kin.foot(i, q[3 * i:3 * i + 3])[2] - kin.foot(i, q_cmd[3 * i:3 * i + 3])[2] for i in range(4) if st[i]]
        return float(np.mean(v)) if v else 0.0

    def _recover_action(self, q, dt):
        r = self.recovery
        tracking = np.abs(self.q_des - np.asarray(q, dtype=float)).max() < RECOVER_TRACK
        if tracking:                                       # 따라오는 만큼만 진행
            if r["u"] < 1.0:
                r["u"] = min(1.0, r["u"] + dt / r["T"])
            else:
                r["hold"] += dt
        if r["hold"] >= RECOVER_HOLD_S:                    # 다 일어섰다: 다음 주기부터 보행
            self.recovery = None
        u = r["u"] * r["u"] * (3 - 2 * r["u"])            # 부드럽게 시작하고 멈춤
        stand = self.iface.targets_from_action(np.zeros(12))
        return self.iface.action_from_targets(r["q0"] + u * (stand - r["q0"]))

    def set_command(self, vx, yaw_rate):
        self.clock.set_command(vx, yaw_rate)

    def step(self, ls):
        """로봇 상태 메시지 -> 관절 명령 메시지 (dict)."""
        t = ls["t"]
        dt = self.iface.control_dt if self.last_t is None else t - self.last_t
        if self.last_t is not None and dt > RECOVER_GAP_S:      # 상태가 끊겼다가 다시 옴 (링크 두절): 일어서기부터
            self._start_recovery(ls["q"])
            # 공백 동안의 움직임은 모른다: 끊기기 전 속도로 공백을 적분하면 추정 위치가 튄다 (3초 두절에 0.7 m,
            # 지형 인지 보행의 지도·디딜 곳이 그만큼 어긋나 잘 걷지 못했다). 제자리에 있었다고 보고 속도 0에서 다시 시작
            self.estimator.restart_after_gap()
        self.last_t = t
        if self.recovery is None:
            self.clock.update(dt)
        est = self.est = self.estimator.update(ls)
        if self.terrain is not None:
            self.pose_hist[t] = self.terrain.update(est)
            self._integrate_scans(t, self.iface.control_dt)
        # 관측은 추정값으로 만든다 (MuJoCo 배치 규약의 qpos/qvel 자리에 추정 자세, 측정 각속도, 엔코더 값)
        qpos = np.concatenate([[0.0, 0.0, 0.0], est.quat_wxyz, est.q])
        qvel = np.concatenate([est.R @ est.v_body, est.gyro, est.dq])
        inputs = (qpos, qvel, self.clock.command_body(), self.action, self.clock.phase)
        hs = self.height_scan(self.iface.obs, est) if self.iface.obs.needs_height_scan else None
        self.obs = self.iface.obs.compute(*inputs, height_scan=hs)
        aux = None
        if self.aux_obs_spec is not None:
            ahs = self.height_scan(self.aux_obs_spec, est) if self.aux_obs_spec.needs_height_scan else None
            aux = self.aux_obs_spec.compute(*inputs, height_scan=ahs)
        ctx = StepContext(dt=dt, est=est, roll=est.roll, pitch=est.pitch, yaw=est.yaw, v_body_x=float(est.v_body[0]),
                          wz_world=est.wz_world, clock=self.clock, obs=self.obs, terrain=self.terrain, aux_obs=aux)
        if self.recovery is None and self.last_q_des is not None:   # 다리가 몸을 못 받침 (토크 부족): 일어서기부터
            # 처진 시간은 쌓고 괜찮은 시간만큼 뺀다 (띄엄띄엄 처지는 경우도 잡는다: 배터리 30%에서 처짐이 끊겨 감지 못하고
            # 토크가 돌아오는 순간 튕겨 돌았다)
            self.sag_time = max(0.0, self.sag_time + (dt if self._stance_sag(ls) > SAG_M else -dt))
            if self.sag_time >= SAG_S:
                self._start_recovery(ls["q"])
        if self.recovery is not None:
            action = self._recover_action(ls["q"], dt)
        else:
            action = self.ctrl.act(ctx)
        if self.on_action:
            action = self.on_action(ctx, action)
        self.action = np.clip(action, -self.iface.clip, self.iface.clip)
        if self.delay_steps:
            self.pending.append(self.action)
            applied = self.pending.popleft()
        else:
            applied = self.action
        self.q_des = self.last_q_des = self.iface.targets_from_action(applied)
        zeros = [0.0] * 12
        return {"t": t, "q_des": self.q_des.tolist(), "dq_des": zeros, "kp": self.iface.kp.tolist(),
                "kd": self.iface.kd.tolist(), "tau_ff": zeros}
