"""다리 기구학 (알고리즘 쪽, 명세 숫자만 사용: MuJoCo 없음).

다리 i의 발 중심 위치(몸통 좌표계) = 고관절 위치 + Rx(q0) [ (0, ±a, 0) + Ry(q1) ( (0,0,-l1) + Ry(q2) (fx,0,-l2) ) ]
  q0: 고관절(옆, x축), q1: 허벅지(y축), q2: 무릎(y축). a = abduction_offset, 왼쪽 다리 +a.
conformance/test_estimator.py가 MuJoCo 모델의 발 위치와 비교한다.
"""
import numpy as np

SIDE = np.array([1, -1, 1, -1])        # FL, FR, RL, RR: +1 = 왼쪽


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class LegKinematics:
    def __init__(self, spec):
        leg = spec["robot"]["leg"]
        self.hips = np.array([[x, y, 0.0] for x, y in leg["hip_offsets"]])
        self.a, self.l1, self.l2 = leg["abduction_offset"], leg["thigh_length"], leg["calf_length"]
        self.fx = leg.get("foot_offset_x", 0.0)
        self.r = leg.get("foot_radius", 0.0)

    def foot_angular_velocity(self, q3, dq3):
        """발(종아리)의 몸통 대비 각속도 (몸통 좌표): 고관절(x축) + 허벅지·무릎(같은 y축, 고관절로 기울어짐)."""
        return np.array([dq3[0], 0.0, 0.0]) + _rx(q3[0]) @ np.array([0.0, dq3[1] + dq3[2], 0.0])

    def foot(self, i, q3):
        q0, q1, q2 = q3
        knee_to_foot = _ry(q2) @ np.array([self.fx, 0.0, -self.l2])
        thigh = _ry(q1) @ (np.array([0.0, 0.0, -self.l1]) + knee_to_foot)
        return self.hips[i] + _rx(q0) @ (np.array([0.0, SIDE[i] * self.a, 0.0]) + thigh)

    def feet(self, q):
        """관절각 12개(모델 순서) -> 네 발 중심 위치 (4, 3), 몸통 좌표계."""
        q = np.asarray(q).reshape(4, 3)
        return np.array([self.foot(i, q[i]) for i in range(4)])

    def jacobian(self, i, q3, eps=1e-6):
        """발 위치의 관절각 미분 (3x3), 수치 미분."""
        q3 = np.asarray(q3, dtype=float)
        J = np.zeros((3, 3))
        for k in range(3):
            d = np.zeros(3)
            d[k] = eps
            J[:, k] = (self.foot(i, q3 + d) - self.foot(i, q3 - d)) / (2 * eps)
        return J
