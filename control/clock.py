"""컨트롤러 공통 요소 (알고리즘 쪽).

명령 필터와 보행 위상 시계는 runner가 소유하고 모든 컨트롤러가 같이 쓴다.
정책 관측(command, gait_phase)과 규칙 기반 보행기가 같은 값을 보게 하기 위함이다.
"""
from dataclasses import dataclass

import numpy as np


class GaitClock:
    """명령 가속 제한 + 보행 위상 (0~1). 명령이 0이면 위상이 멈춘다."""

    def __init__(self, period, accel_limit=0.5):
        self.period = period
        self.accel_limit = accel_limit
        self.reset()

    def reset(self):
        self.cmd = np.zeros(2)       # 목표 [vx, yaw_rate]
        self.cmd_f = np.zeros(2)     # 가속 제한을 거친 명령
        self.phase = 0.0

    def set_command(self, vx, yaw_rate):
        self.cmd[:] = vx, yaw_rate

    @property
    def moving(self):
        return np.abs(self.cmd_f).max() > 1e-3

    def update(self, dt):
        a = self.accel_limit * dt
        self.cmd_f += np.clip(self.cmd - self.cmd_f, -a, a)
        if self.moving:
            self.phase = (self.phase + dt / self.period) % 1.0

    def command_body(self):
        """관측용 명령 [vx, vy, yaw_rate] (vy는 현재 미사용)."""
        return np.array([self.cmd_f[0], 0.0, self.cmd_f[1]])


@dataclass
class StepContext:
    """제어 주기마다 컨트롤러에 넘기는 값. 모두 로봇이 아는 값(측정 + 추정)이다. 참값은 없다."""
    dt: float
    est: object          # control.estimator.Estimate (자세, 각속도, 다리 주행거리계 속도, 접촉, 관절)
    roll: float
    pitch: float
    yaw: float
    v_body_x: float      # 몸통 좌표 전진 속도 (추정)
    wz_world: float
    clock: GaitClock
    obs: np.ndarray      # 이 컨트롤러의 인터페이스로 계산한 관측 (추정값 기반)
