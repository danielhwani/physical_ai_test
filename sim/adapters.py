"""좌표계 변환은 이 파일 한 곳에서만 수행한다 (문서 §6.3, §8).

MuJoCo: 오른손계, Z-up, m, 쿼터니언 (w, x, y, z)
ROS2 (REP-103): 오른손계, Z-up, m, 쿼터니언 (x, y, z, w)  -> 축 변환 없음, 순서만 다름
UE5: 왼손계, Z-up, cm, X 전방 / Y 오른쪽       -> Y 반전, 100배
"""
import numpy as np


def mj_quat_to_ros(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([x, y, z, w])


def mj_pose_to_ue(pos, q_wxyz):
    """MuJoCo 월드 포즈 -> UE5 (위치 cm, 쿼터니언 x,y,z,w). Y축 반전 = 거울 변환."""
    w, x, y, z = q_wxyz
    ue_pos = np.array([pos[0], -pos[1], pos[2]]) * 100.0
    ue_quat = np.array([-x, y, -z, w])     # Y 거울: x,z 성분 부호 반전
    return ue_pos, ue_quat


def quat_to_roll_pitch(q_wxyz):
    w, x, y, z = q_wxyz
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def quat_to_yaw(q_wxyz):
    w, x, y, z = q_wxyz
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
