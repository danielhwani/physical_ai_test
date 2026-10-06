"""보행 알고리즘 노드 (알고리즘 쪽): 로봇 상태 메시지 -> 관절 명령 메시지.

실제 로봇의 탑재 소프트웨어와 같은 경계다. 입력은 로봇이 아는 값뿐이다.
  로봇 상태 (Unitree LowState를 본뜸): t, q[12], dq[12], imu{quat(x,y,z,w), gyro, accel}, foot_force[4]
  이동 명령 (운용자): vx, yaw_rate
  관절 명령 (Unitree LowCmd를 본뜸): q_des[12], dq_des[12], kp[12], kd[12], tau_ff[12]   (모델 관절 순서)
시뮬레이터(sim/)와 MuJoCo를 import하지 않는다. 같은 프로세스에서 직접 부르거나 control/ros_node.py로 ROS2 노드가 된다.
"""
from collections import deque

import numpy as np

from .clock import GaitClock, StepContext
from .estimator import LegOdometryEstimator
from .interface import ControlInterface, load_card
from .onnx_policy import OnnxPolicyController
from .trot import TrotController


class ControllerNode:
    def __init__(self, spec, controller_cfg=None, policy=None):
        """controller_cfg: 시나리오의 controller 절 (트롯). policy: 정책 카드 경로 (ONNX)."""
        if policy:
            card = load_card(policy)
            self.iface = ControlInterface(spec, card)
            self.ctrl = OnnxPolicyController(card, self.iface)
            self.desc, period = f"onnx:{card['name']}", self.iface.gait_period
        else:
            self.iface = ControlInterface(spec)
            self.ctrl = TrotController(spec, controller_cfg, self.iface)
            self.desc, period = f"trot:{controller_cfg}", controller_cfg["period"]
        self.clock = GaitClock(period or 1.0)
        self.estimator = LegOdometryEstimator(spec)
        self.delay_steps = spec["action"].get("delay_steps", 0)
        self.on_action = None        # (ctx, action) -> action. 모방학습(DAgger)에서 실행 action 교체용
        self.reset()

    def reset(self):
        self.ctrl.reset()
        self.clock.reset()
        self.estimator.reset()
        self.action = np.zeros(12)
        self.obs = np.zeros(self.iface.obs.dim)
        self.pending = deque([np.zeros(12)] * self.delay_steps)
        self.last_t, self.est = None, None
        self.q_des = self.iface.targets_from_action(self.action)

    def set_command(self, vx, yaw_rate):
        self.clock.set_command(vx, yaw_rate)

    def step(self, ls):
        """로봇 상태 메시지 -> 관절 명령 메시지 (dict)."""
        t = ls["t"]
        dt = self.iface.control_dt if self.last_t is None else t - self.last_t
        self.last_t = t
        self.clock.update(dt)
        est = self.est = self.estimator.update(ls)
        # 관측은 추정값으로 만든다 (MuJoCo 배치 규약의 qpos/qvel 자리에 추정 자세, 측정 각속도, 엔코더 값)
        qpos = np.concatenate([[0.0, 0.0, 0.0], est.quat_wxyz, est.q])
        qvel = np.concatenate([est.R @ est.v_body, est.gyro, est.dq])
        self.obs = self.iface.obs.compute(qpos, qvel, self.clock.command_body(), self.action, self.clock.phase)
        ctx = StepContext(dt=dt, est=est, roll=est.roll, pitch=est.pitch, yaw=est.yaw, v_body_x=float(est.v_body[0]),
                          wz_world=est.wz_world, clock=self.clock, obs=self.obs)
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
