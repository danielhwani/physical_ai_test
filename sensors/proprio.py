"""고유수용 센서 모델 (엔진 무관): IMU, 관절 엔코더, 발 힘 센서.

참값(물리엔진이 계산한 값)을 받아 잡음, 편향, 표류를 더해 "로봇이 실제로 아는 값"을 만든다.
보행 알고리즘은 이 출력만 받는다 (참값을 모른다). 난수는 (seed, 표본 번호)로 정해 같은 실행은 같은 값이다.
MuJoCo, ROS를 import하지 않는다.
"""
from dataclasses import dataclass

import numpy as np


def quat_mul(a, b):            # (w, x, y, z)
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_from_rpy(r, p, y):    # (w, x, y, z), Z-Y-X
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


@dataclass
class ProprioSpec:
    gyro_noise: float = 0.005          # rad/s (표본마다)
    gyro_bias: float = 0.002           # rad/s (실행마다 축별로 정해지는 편향의 표준편차)
    accel_noise: float = 0.05          # m/s^2
    accel_bias: float = 0.02           # m/s^2
    attitude_noise_deg: float = 0.2    # AHRS 롤/피치 잡음 (도)
    yaw_drift_deg_s: float = 0.05      # AHRS 방향 표류율의 표준편차 (도/초, 실행마다 정해짐)
    joint_pos_noise: float = 2e-4      # rad
    joint_vel_noise: float = 0.02      # rad/s
    foot_force_noise: float = 2.0      # N
    seed: int = 0

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: v for k, v in (d or {}).items() if k in cls.__dataclass_fields__})


class ProprioSensors:
    def __init__(self, spec: ProprioSpec):
        self.spec = spec
        rng = np.random.default_rng((spec.seed, 0xB1A5))
        self.gyro_bias = rng.normal(0, spec.gyro_bias, 3)
        self.accel_bias = rng.normal(0, spec.accel_bias, 3)
        self.yaw_drift = np.radians(rng.normal(0, spec.yaw_drift_deg_s))   # rad/s
        self.n = 0
        self.clear_fault()

    def set_fault(self, gyro_bias=(0, 0, 0), attitude_offset_deg=(0, 0, 0)):
        """고장 주입 (DIS 콘솔): 자이로 편향 추가(rad/s), AHRS 자세 출력 오프셋(롤, 피치, 방향, 도)."""
        self.fault_gyro = np.asarray(gyro_bias, dtype=float)
        self.fault_att = np.radians(np.asarray(attitude_offset_deg, dtype=float))

    def clear_fault(self):
        self.set_fault()

    NOISE_DIM = 36      # 표본 하나의 잡음: 자세 롤·피치 2, 관절각 12, 관절속도 12, 자이로 3, 가속도 3, 발 힘 4

    def draw_noise(self, n):
        """표본 번호 n의 잡음 (NOISE_DIM). 난수는 (seed, n)으로만 정해진다: C++ 실시간 코어(rt/)에 미리 만들어 넘겨도 같은 값."""
        s = self.spec
        rng = np.random.default_rng((s.seed, n))
        att = np.radians(s.attitude_noise_deg)
        return np.concatenate([[rng.normal(0, att), rng.normal(0, att)], rng.normal(0, s.joint_pos_noise, 12),
                               rng.normal(0, s.joint_vel_noise, 12), rng.normal(0, s.gyro_noise, 3),
                               rng.normal(0, s.accel_noise, 3), rng.normal(0, s.foot_force_noise, 4)])

    def measure(self, t, quat_wxyz, gyro, accel, q, dq, foot_force):
        """참값 -> 측정값 (로봇 상태 메시지의 내용). 쿼터니언 출력은 x, y, z, w (ROS 규약)."""
        z = self.draw_noise(self.n)
        self.n += 1
        fr, fp, fy = self.fault_att
        err = quat_from_rpy(z[0] + fr, z[1] + fp, self.yaw_drift * t + fy)   # 월드 기준 오차 회전
        qm = quat_mul(err, np.asarray(quat_wxyz, dtype=float))
        qm /= np.linalg.norm(qm)
        return {
            "t": float(t),
            "q": (np.asarray(q) + z[2:14]).tolist(),
            "dq": (np.asarray(dq) + z[14:26]).tolist(),
            "imu": {"quat": [qm[1], qm[2], qm[3], qm[0]],
                    "gyro": (np.asarray(gyro) + self.gyro_bias + z[26:29] + self.fault_gyro).tolist(),
                    "accel": (np.asarray(accel) + self.accel_bias + z[29:32]).tolist()},
            "foot_force": np.maximum(np.asarray(foot_force) + z[32:36], 0.0).tolist(),
        }
