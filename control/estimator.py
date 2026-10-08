"""상태 추정기 (알고리즘 쪽): 로봇 상태 메시지(IMU, 관절 엔코더, 발 힘)만으로 몸통 상태를 추정한다.

- 자세(롤, 피치, 방향): IMU의 자세 출력(AHRS). 방향은 시간이 지나며 표류한다.
- 각속도: 자이로 (편향 포함).
- 몸통 선속도: 다리 주행거리계. 디딘 발(발 힘 > 문턱)의 "땅과 닿은 점"은 움직이지 않는다고 본다.
  Go2 발은 구라서 딛는 동안 굴러 발 중심은 움직인다 (중심 고정 가정은 약 10% 과소추정, 확인함).
      접촉점 c = p_foot - r n   (n: 월드 위 방향을 몸통 좌표로)
      0 = v_body + ω × p_foot + J(q) dq + (ω + Ω_rel) × (-r n)
      ->  v_body = -(J dq + ω × p_foot + (ω + Ω_rel) × (-r n))
  디딘 발들의 평균을 저역통과한다. 발이 미끄러지면 오차가 생긴다 (실제 로봇과 같은 한계).
- 위치(주행거리 좌표계 odom): 수평(x, y)은 추정 속도를 월드 방향으로 돌려 적분한다. 시작 자세가 원점, 시간이 지나며 표류한다.
  높이(z)는 디딘 발에 묶는다: 발이 닿은 순간 그 접촉점의 odom 높이를 기억하고, 디딘 동안 몸통 높이 =
  기억한 접촉점 높이 - (다리 기구학으로 구한 몸통 -> 접촉점 높이)의 평균. 속도 적분은 작은 치우침이 쌓여
  평지 60초 보행에 +0.28 m 표류했다 (RViz에서 지도가 떠올라 로봇이 가라앉아 보임). 발에 묶으면 걸음마다
  자세 잡음만큼의 작은 오차만 남는다. 모든 발이 떠 있는 동안만 속도로 적분한다.
  지형 지도(control/terrain_map.py)는 이 좌표계에 쌓는다.
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
    pos: np.ndarray          # 몸통 위치 (odom 좌표계, 적분)
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
        self.pos = np.zeros(3)
        self.last_t = None
        self.foot_z = {}                        # 디딘 발: 닿은 순간 접촉점의 odom 높이
        self.ground_z = 0.0                     # 마지막으로 안 지면 높이 (디딘 접촉점 평균, odom)
        self.reanchor = False                   # 상태 공백 뒤: 새로 디딘 발을 공백 전 지면 높이에 맞춘다

    def restart_after_gap(self):
        """로봇 상태가 한동안 오지 않았을 때 (링크 두절). 그 사이 움직임은 모르므로 제자리에 있었다고 보고 속도 0에서 다시 시작한다.
        로봇이 주저앉거나 엎드리면 발 하중이 사라져 기억한 접촉점을 잃으므로, 다시 디딘 발은 공백 전 지면 높이에 놓는다
        (지금 추정 몸통 높이로 잡으면 주저앉은 만큼 지면이 높다고 믿게 되어, 3초 두절 뒤 지형 인지 보행이 발을 뻗다 튀어 넘어졌다)."""
        self.v[:] = 0.0
        self.last_t = None
        self.foot_z = {}
        self.reanchor = True

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
        rel_z = {}                                             # 디딘 발: 몸통 -> 접촉점 높이 (월드 방향)
        if contacts.any():
            est = []
            down = -self.kin.r * R[2, :]                       # 발 중심 -> 접촉점 (몸통 좌표): -r * (월드 z를 몸통 좌표로)
            for i in np.where(contacts)[0]:
                qi, dqi = q[3 * i:3 * i + 3], dq[3 * i:3 * i + 3]
                p = self.kin.foot(i, qi)
                omega_foot = gyro + self.kin.foot_angular_velocity(qi, dqi)
                est.append(-(self.kin.jacobian(i, qi) @ dqi + np.cross(gyro, p) + np.cross(omega_foot, down)))
                rel_z[int(i)] = float((R @ (p + down))[2])
            self.v += self.alpha * (np.mean(est, axis=0) - self.v)
        if self.last_t is not None:
            self.pos += (R @ self.v) * (ls["t"] - self.last_t)
        for i in [i for i in self.foot_z if i not in rel_z]:   # 뗀 발은 잊는다
            del self.foot_z[i]
        if rel_z:
            for i, rz in rel_z.items():                        # 새로 디딘 발: 지금 추정 높이로 접촉점 높이를 정한다
                self.foot_z.setdefault(i, self.ground_z if self.reanchor else self.pos[2] + rz)
            self.reanchor = False
            self.pos[2] = float(np.mean([self.foot_z[i] - rz for i, rz in rel_z.items()]))
            self.ground_z = float(np.mean([self.foot_z[i] for i in rel_z]))
        self.last_t = ls["t"]
        return Estimate(quat_wxyz=qw, R=R, roll=roll, pitch=pitch, yaw=yaw, gyro=gyro, v_body=self.v.copy(),
                        wz_world=float((R @ gyro)[2]), contacts=contacts, pos=self.pos.copy(), q=q, dq=dq)
