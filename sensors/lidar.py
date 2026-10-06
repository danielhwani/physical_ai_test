"""LiDAR 센서 모델 (엔진 무관, 문서 §8: 센서 로직은 엔진 밖 라이브러리, 엔진은 레이캐스트만 제공).

이 모듈은 MuJoCo, Chrono, ROS를 모른다. 광선 거리를 돌려주는 함수(raycast)만 받아서
스캔 패턴, 장착 위치, 거리 잡음, 누락을 적용해 점군을 만든다. 같은 모델을 UE5 등 다른 엔진의
레이캐스트에 붙여도 센서 출력 규약이 같다.

raycast(origin(3,), dirs(N,3) 월드 단위벡터, max_range) -> (dist(N,), hit_label(N,) int)
    맞지 않은 광선은 dist=inf. hit_label은 맞은 물체 종류 (0=지형, 1=다른 물체, -1=없음).
"""
from dataclasses import dataclass, field

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


@dataclass
class LidarSpec:
    name: str
    parent: str                      # 장착 바디 이름 (예: base)
    pos: list                        # 장착 위치 (부모 바디 좌표, m)
    rpy: list                        # 장착 자세 (부모 바디 좌표, rad): 센서 x축이 정면
    rate_hz: float = 10.0
    azimuth_deg: list = field(default_factory=lambda: [-90.0, 90.0])   # 수평 범위 [시작, 끝]
    azimuth_step_deg: float = 1.5
    elevation_deg: list = field(default_factory=lambda: [-60.0, 10.0])  # 수직 범위 [아래, 위]
    channels: int = 24
    range_min: float = 0.05
    range_max: float = 30.0
    noise_std: float = 0.01          # 거리 잡음 표준편차 (m)
    dropout: float = 0.0             # 무작위 누락 확률
    seed: int = 0

    @classmethod
    def from_dict(cls, name, d):
        return cls(name=name, **{k: v for k, v in d.items() if k in cls.__dataclass_fields__ and k != "name"})


class Lidar:
    def __init__(self, spec: LidarSpec):
        self.spec = spec
        az = np.radians(np.arange(spec.azimuth_deg[0], spec.azimuth_deg[1] + 1e-9, spec.azimuth_step_deg))
        el = np.radians(np.linspace(spec.elevation_deg[0], spec.elevation_deg[1], spec.channels))
        A, E = np.meshgrid(az, el)
        self.dirs_sensor = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
        self.ring = np.repeat(np.arange(spec.channels), len(az))          # 채널 번호 (점마다)
        self.R_mount = rpy_to_matrix(*spec.rpy)
        self.p_mount = np.asarray(spec.pos, dtype=float)
        self.period = 1.0 / spec.rate_hz
        self.frame = 0

    @property
    def n_rays(self):
        return len(self.dirs_sensor)

    def pose_world(self, R_parent, p_parent):
        """부모 바디 월드 자세 -> 센서 월드 자세 (R, p)."""
        return R_parent @ self.R_mount, p_parent + R_parent @ self.p_mount

    def scan(self, raycast, R_parent, p_parent):
        """한 프레임 스캔. 반환: dict(points=센서 좌표 (M,3), ring, label, R, p (센서 월드 자세), frame)."""
        s = self.spec
        R, p = self.pose_world(R_parent, p_parent)
        dist, label = raycast(p, self.dirs_sensor @ R.T, s.range_max)
        # 잡음과 누락은 프레임 번호로 정한 난수로 (같은 실행을 다시 돌리면 같은 점군)
        rng = np.random.default_rng((s.seed, self.frame))
        noisy = dist + rng.normal(0.0, s.noise_std, dist.shape)
        keep = np.isfinite(dist) & (dist >= s.range_min) & (dist <= s.range_max)
        if s.dropout > 0:
            keep &= rng.random(dist.shape) >= s.dropout
        pts = self.dirs_sensor[keep] * noisy[keep, None]
        out = {"points": pts.astype(np.float32), "ring": self.ring[keep].astype(np.uint16),
               "label": label[keep].astype(np.int8), "R": R, "p": p, "frame": self.frame,
               "range_true": dist[keep]}
        self.frame += 1
        return out


def to_world(scan):
    """센서 좌표 점군 -> 월드 좌표."""
    return scan["points"].astype(float) @ scan["R"].T + scan["p"]
