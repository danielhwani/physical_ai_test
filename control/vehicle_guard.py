"""차 감시 (알고리즘 쪽): LiDAR로 앞쪽 길 양옆에 키 큰 물체 (차)가 보이면 서서 기다린다.

입력은 로봇이 아는 값뿐이다: 센서 좌표 점군 (측정), 장착 위치 (로봇 명세), 스캔 시각의 추정 자세. 차가 어디로 가는지, 시뮬레이터의
차 위치는 모른다. 점마다 "무엇에 맞았나" (label)도 받지 않는다 (실제 LiDAR처럼 높이로만 판단한다).
  - 점을 추정 자세로 odom 좌표로 바꾸고, 몸통보다 min_height (m) 넘게 높은 점만 본다 (땅에서 약 0.4 m 위. 요철·자국·작은 오르막은 걸리지 않는다).
  - 로봇 방향 기준 앞 [near, ahead] m, 좌우 ±side m 안에 그런 점이 min_points개 넘게 있으면 "막힘".
    좌우를 넓게 보는 것은 다가오는 차를 미리 보기 위해서다 (앞쪽 180° LiDAR가 옆까지 본다).
  - 막힌 동안은 걸음 시계에 속도 0을 넣고 (서서 기다림), clear_s 동안 안 보이면 운용자 명령으로 돌아간다.
한계: 차의 진로를 모르고 높이로만 본다. 길 양옆 ±side 안에 서 있는 차가 있으면 그 차가 떠날 때까지 기다린다.
MuJoCo, 시뮬레이터를 import하지 않는다.
"""
import numpy as np

DEFAULTS = {"near": 0.3, "ahead": 4.0, "side": 15.0, "min_height": 0.15, "min_points": 10, "clear_s": 1.5}


class VehicleGuard:
    def __init__(self, cfg, R_mount, p_mount, body_box):
        c = dict(DEFAULTS, **(cfg if isinstance(cfg, dict) else {}))
        self.near, self.ahead, self.side = float(c["near"]), float(c["ahead"]), float(c["side"])
        self.min_height, self.min_points, self.clear_s = float(c["min_height"]), int(c["min_points"]), float(c["clear_s"])
        self.R_mount, self.p_mount, self.body_box = R_mount, p_mount, body_box
        self.reset()

    def reset(self):
        self.last_seen = None          # 마지막으로 막힘을 본 스캔 시각
        self.last_count = 0
        self.holding = False
        self.holds, self.hold_s = 0, 0.0
        self.events = []               # (시각, "hold" | "go", 그때 본 점 수)

    def add_scan(self, ts, points, R_base, p_base):
        """센서 좌표 점군 (N, 3) + 스캔 시각의 지도용 몸통 자세 (TerrainPerception과 같은 자세)."""
        pts = np.asarray(points, dtype=float)
        body = pts @ self.R_mount.T + self.p_mount
        own = (np.abs(body[:, 0]) < self.body_box[0]) & (np.abs(body[:, 1]) < self.body_box[1])
        world = body[~own] @ R_base.T + p_base
        tall = world[world[:, 2] - p_base[2] > self.min_height]
        yaw = np.arctan2(R_base[1, 0], R_base[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        rel = tall[:, :2] - p_base[:2]
        fwd, lat = rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c
        n = int(((fwd > self.near) & (fwd < self.ahead) & (np.abs(lat) < self.side)).sum())
        self.last_count = n
        if n >= self.min_points:
            self.last_seen = ts

    def update(self, t, dt):
        """이번 제어 주기에 서서 기다려야 하면 True."""
        blocked = self.last_seen is not None and t - self.last_seen < self.clear_s
        if blocked and not self.holding:
            self.holds += 1
            self.events.append((round(t, 2), "hold", self.last_count))
        elif self.holding and not blocked:
            self.events.append((round(t, 2), "go", self.last_count))
        self.holding = blocked
        if blocked:
            self.hold_s += dt
        return blocked
