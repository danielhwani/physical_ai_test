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

    def set_command(self, vx, yaw_rate):
        self.clock.set_command(vx, yaw_rate)

    def step(self, ls):
        """로봇 상태 메시지 -> 관절 명령 메시지 (dict)."""
        t = ls["t"]
        dt = self.iface.control_dt if self.last_t is None else t - self.last_t
        self.last_t = t
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
        action = self.ctrl.act(ctx)
        if self.on_action:
            action = self.on_action(ctx, action)
        self.action = np.clip(action, -self.iface.clip, self.iface.clip)
        if self.delay_steps:
            self.pending.append(self.action)
            applied = self.pending.popleft()
        else:
            applied = self.action
        self.q_des = self.iface.targets_from_action(applied)
        zeros = [0.0] * 12
        return {"t": t, "q_des": self.q_des.tolist(), "dq_des": zeros, "kp": self.iface.kp.tolist(),
                "kd": self.iface.kd.tolist(), "tau_ff": zeros}
