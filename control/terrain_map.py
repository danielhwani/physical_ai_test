"""지형 인지 (알고리즘 쪽): LiDAR 점군 -> 로봇 주변 높이 지도 (주행거리 좌표계 odom).

입력은 로봇이 아는 값뿐이다: 센서 좌표 점군(측정), 장착 위치(로봇 명세의 보정값), 추정 자세와 위치.
센서의 참 월드 자세나 지형 참값은 받지 않는다. 그래서 추정 위치가 표류하면 지도도 같이 어긋난다 (실제 로봇과 같음).

- 지도: 로봇을 따라 움직이는 격자 (기본 4 m x 4 m, 4 cm). 모르는 칸은 NaN.
  칸마다 이번 스캔 점들의 평균 높이를 넣고, 이전 값과 비슷하면 평균, 크게 다르면(지형이 바뀜, 차량이 지나감) 새 값으로 바꾼다.
- 높이 표류: 다리 주행거리계의 높이는 표류한다 (지형 인지 트롯 평지 16초에 약 1 cm). 보행에 영향을 주는 것은
  지면을 스캔한 뒤 그 위를 디디기까지(1~3초) 쌓인 표류뿐이라 보정하지 않는다.
  (디딘 발 아래 지도 높이로 몸통 높이를 당기는 보정을 시험했으나, 발이 지면에 몇 mm 파고드는 만큼이 스캔마다
  누적되어 오히려 초당 약 1 cm 표류했다 -> 뺐다)
- 앞쪽 LiDAR는 발밑을 보지 못한다. 앞에서 본 지면이 로봇이 걸어가면 발밑으로 온다 (지도가 기억한다).
MuJoCo, 시뮬레이터를 import하지 않는다.
"""
import numpy as np

from .kinematics import LegKinematics


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class ElevationMap:
    def __init__(self, half=2.0, res=0.04, change=0.04, recenter=0.5):
        self.res = res
        self.n = int(round(2 * half / res))
        self.change = change                 # 이전 값과 이만큼 넘게 다르면 새 값으로 바꾼다 (m)
        self.recenter_dist = recenter
        self.h = np.full((self.n, self.n), np.nan)
        self.origin = np.array([-half, -half])  # 칸 (0, 0)의 모서리 (odom xy)

    @property
    def center(self):
        return self.origin + self.n * self.res / 2

    def recenter(self, xy):
        """로봇이 지도 중심에서 멀어지면 칸 단위로 지도를 옮긴다 (밖으로 나간 칸은 버린다)."""
        shift = np.round((np.asarray(xy) - self.center) / self.res).astype(int)
        if np.abs(shift).max() * self.res < self.recenter_dist:
            return
        new = np.full_like(self.h, np.nan)
        sx, sy = shift
        src = self.h[max(sy, 0):self.n + min(sy, 0), max(sx, 0):self.n + min(sx, 0)]
        new[max(-sy, 0):max(-sy, 0) + src.shape[0], max(-sx, 0):max(-sx, 0) + src.shape[1]] = src
        self.h = new
        self.origin = self.origin + shift * self.res

    def index(self, xy):
        ij = np.floor((np.asarray(xy) - self.origin) / self.res).astype(int)
        return ij[..., 1], ij[..., 0]          # (행 = y, 열 = x)

    def add_points(self, pts):
        """odom 좌표 점들 (N, 3)을 지도에 넣는다."""
        r, c = self.index(pts[:, :2])
        ok = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        flat = r[ok] * self.n + c[ok]
        cnt = np.bincount(flat, minlength=self.n * self.n)
        hit = cnt > 0
        mean = np.zeros(self.n * self.n)
        mean[hit] = np.bincount(flat, weights=pts[ok, 2], minlength=self.n * self.n)[hit] / cnt[hit]
        h = self.h.ravel()
        old = h[hit]
        new = mean[hit]
        fuse = np.isfinite(old) & (np.abs(new - old) < self.change)
        h[hit] = np.where(fuse, 0.5 * (old + new), new)
        self.h = h.reshape(self.n, self.n)

    def window(self, xy, k):
        """칸 xy 주변 (2k+1)^2 높이. 지도 밖은 NaN."""
        r, c = self.index(xy)
        out = np.full((2 * k + 1, 2 * k + 1), np.nan)
        r0, r1, c0, c1 = r - k, r + k + 1, c - k, c + k + 1
        rr0, rr1, cc0, cc1 = max(r0, 0), min(r1, self.n), max(c0, 0), min(c1, self.n)
        if rr0 < rr1 and cc0 < cc1:
            out[rr0 - r0:rr1 - r0, cc0 - c0:cc1 - c0] = self.h[rr0:rr1, cc0:cc1]
        return out

    def lookup(self, xy):
        """여러 점 (N, 2)의 칸 값 그대로 (모르면 NaN). 정책 관측 height_scan용 (이웃 보간 없이 단순하게)."""
        r, c = self.index(np.asarray(xy))
        ok = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        out = np.full(len(r), np.nan)
        out[ok] = self.h[r[ok], c[ok]]
        return out

    def height(self, xy):
        """그 위치의 높이. 칸이 비어 있으면 바로 옆 칸들의 평균, 그것도 없으면 NaN."""
        w = self.window(xy, 1)
        if np.isfinite(w[1, 1]):
            return float(w[1, 1])
        return float(np.nanmean(w)) if np.isfinite(w).any() else np.nan


class TerrainPerception:
    """LiDAR 점군 + 추정 자세 -> 높이 지도. 스캔은 그 스캔 시각의 추정 자세로 odom 좌표에 옮긴다."""

    def __init__(self, spec, lidar_def, cfg=None):
        cfg = cfg or {}
        self.kin = LegKinematics(spec)
        self.R_mount = rpy_to_matrix(*lidar_def["rpy"])            # 장착 보정값 (로봇 명세)
        self.p_mount = np.asarray(lidar_def["pos"], dtype=float)
        self.map_cfg = {k: cfg[k] for k in ("half", "res") if k in cfg}
        self.body_box = np.array(cfg.get("body_box", [0.36, 0.2]))  # 몸통 좌표 |x|, |y| 안의 점은 자기 몸으로 보고 버린다
        self.reset()

    def reset(self):
        self.map = ElevationMap(**self.map_cfg)
        self.scans_used = 0

    def pose(self, est):
        """지도용 몸통 자세 (R, p) = 추정 자세와 odom 위치."""
        return est.R, est.pos

    def update(self, est):
        """제어 주기마다: 지도를 로봇 쪽으로 옮긴다. 반환: 이 시각의 지도용 자세 (스캔을 넣을 때 쓴다)."""
        R, p = self.pose(est)
        self.map.recenter(p[:2])
        return R, p.copy()

    def add_scan(self, points, R_base, p_base):
        """센서 좌표 점군 (N, 3) + 스캔 시각의 지도용 몸통 자세."""
        pts = np.asarray(points, dtype=float)
        body = pts @ self.R_mount.T + self.p_mount                 # 몸통 좌표
        own = (np.abs(body[:, 0]) < self.body_box[0]) & (np.abs(body[:, 1]) < self.body_box[1])
        world = body[~own] @ R_base.T + p_base
        self.map.add_points(world)
        self.scans_used += 1

    def feet_world(self, est):
        R, p = self.pose(est)
        return np.array([p + R @ self.kin.foot(i, est.q[3 * i:3 * i + 3]) for i in range(4)])
