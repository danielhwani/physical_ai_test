"""상태 추정기 (알고리즘 쪽): 로봇 상태 메시지(IMU, 관절 엔코더, 발 힘)만으로 몸통 상태를 추정한다.

- 자세(롤, 피치, 방향): IMU의 자세 출력(AHRS). 방향은 시간이 지나며 표류한다.
- 각속도: 자이로 (편향 포함).
- 몸통 선속도: 다리 주행거리계. 디딘 발(발 힘 > 문턱)의 "땅과 닿은 점"은 움직이지 않는다고 본다.
  Go2 발은 구라서 딛는 동안 굴러 발 중심은 움직인다 (중심 고정 가정은 약 10% 과소추정, 확인함).
      접촉점 c = p_foot - r n   (n: 월드 위 방향을 몸통 좌표로)
      0 = v_body + ω × p_foot + J(q) dq + (ω + Ω_rel) × (-r n)
      ->  v_body = -(J dq + ω × p_foot + (ω + Ω_rel) × (-r n))
  디딘 발들의 평균을 저역통과한다. 발이 미끄러지면 오차가 생긴다 (실제 로봇과 같은 한계).
참값을 받지 않는다. MuJoCo를 import하지 않는다.
"""
from dataclasses import dataclass

import numpy as np

from .kinematics import LegKinematics


def quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


@dataclass
class Estimate:
    quat_wxyz: np.ndarray
    R: np.ndarray            # 몸통 -> 월드 회전 (추정)
    roll: float
    pitch: float
    yaw: float
    gyro: np.ndarray         # 몸통 좌표 각속도 (측정)
    v_body: np.ndarray       # 몸통 좌표 선속도 (다리 주행거리계)
    wz_world: float          # 월드 z축 기준 방향 회전 속도
    contacts: np.ndarray     # 디딘 발 (bool, 4)
    q: np.ndarray            # 관절각 (측정)
    dq: np.ndarray           # 관절 속도 (측정)


class LegOdometryEstimator:
    def __init__(self, spec, contact_force=20.0, alpha=0.3):
        self.kin = LegKinematics(spec)
        self.contact_force = contact_force      # 이 이상 발 힘이면 디딤으로 본다 (N)
        self.alpha = alpha                      # 저역통과 계수 (제어 주기마다)
        self.reset()

    def reset(self):
        self.v = np.zeros(3)

    def update(self, ls):
        imu = ls["imu"]
        x, y, z, w = imu["quat"]
        qw = np.array([w, x, y, z])
        R = quat_xyzw_to_matrix(imu["quat"])
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arcsin(np.clip(-R[2, 0], -1, 1))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        gyro = np.asarray(imu["gyro"], dtype=float)
        q, dq = np.asarray(ls["q"], dtype=float), np.asarray(ls["dq"], dtype=float)
        contacts = np.asarray(ls["foot_force"]) > self.contact_force
        if contacts.any():
            est = []
            down = -self.kin.r * R[2, :]                       # 발 중심 -> 접촉점 (몸통 좌표): -r * (월드 z를 몸통 좌표로)
            for i in np.where(contacts)[0]:
                qi, dqi = q[3 * i:3 * i + 3], dq[3 * i:3 * i + 3]
                p = self.kin.foot(i, qi)
                omega_foot = gyro + self.kin.foot_angular_velocity(qi, dqi)
                est.append(-(self.kin.jacobian(i, qi) @ dqi + np.cross(gyro, p) + np.cross(omega_foot, down)))
            self.v += self.alpha * (np.mean(est, axis=0) - self.v)
        return Estimate(quat_wxyz=qw, R=R, roll=roll, pitch=pitch, yaw=yaw, gyro=gyro, v_body=self.v.copy(),
                        wz_world=float((R @ gyro)[2]), contacts=contacts, q=q, dq=dq)
