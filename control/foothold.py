"""디딜 곳 고르기 (알고리즘 쪽): 높이 지도에서 원래 디딜 위치 주변 후보를 평가해 가장 나은 곳을 고른다.

평가 기준은 "그 자리 자체가 평탄하고 모서리에서 먼가"다. 높이 자체는 벌점이 아니다.
바퀴 자국 바닥은 평탄하므로 좋은 자리이고, 자국 벽 근처(옆 칸과 높이 차가 큼)가 나쁜 자리다.
  비용 = w_edge1 * (바로 옆 칸 최대 높이 차) + w_edge2 * (두 칸 거리 최대 높이 차) + w_shift * |앞뒤| + w_side * |옆| + 모르는 칸 벌점
  높이 차는 edge_deadband(1.5 cm)를 뺀 값을 쓴다: 거리 잡음(1 cm)과 발자국 같은 작은 패임에는 반응하지 않는다.
후보는 진행 방향(몸통 x) ±max_shift, 옆 방향(몸통 y) ±max_side. 진행 방향을 가로지르는 자국은 앞뒤로,
나란한 자국(자국 안을 따라 걸을 때)은 옆으로 피한다.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Foothold:
    shift: float          # 진행 방향으로 옮긴 거리 (m)
    side: float           # 옆(왼쪽 +)으로 옮긴 거리 (m)
    xy: np.ndarray        # odom 좌표
    height: float         # 그 자리 지면 높이 (모르면 NaN)
    cost: float


class FootholdPlanner:
    def __init__(self, cfg=None):
        cfg = cfg or {}
        self.max_shift = cfg.get("max_shift", 0.06)
        self.max_side = cfg.get("max_side", 0.0)     # 옆 옮김 (기본 끔: 가로 자국에서 오히려 넘어짐이 늘었다. 0.04로 켜면 나란한 자국에서 조금 낫다)
        self.step = cfg.get("shift_step", 0.02)
        self.deadband = cfg.get("edge_deadband", 0.015)
        self.w_edge1 = cfg.get("w_edge1", 10.0)
        self.w_edge2 = cfg.get("w_edge2", 4.0)
        self.w_shift = cfg.get("w_shift", 0.5)
        self.w_side = cfg.get("w_side", 2.0)           # 옆으로 옮기기는 더 비싸게: 지도 칸의 들쭉날쭉함에 반응해 다리를 흔들지 않게
        self.unknown_cost = cfg.get("unknown_cost", 0.3)
        self.keep_bonus = cfg.get("keep_bonus", 0.02)   # 직전 선택을 유지하면 비용을 깎아 흔들림을 막는다
        xs = np.round(np.arange(-self.max_shift, self.max_shift + 1e-9, self.step), 6)
        ys = np.round(np.arange(-self.max_side, self.max_side + 1e-9, self.step), 6)
        self.candidates = [(float(x), float(y)) for x in xs for y in ys]

    def evaluate(self, emap, xy):
        w = emap.window(xy, 2)
        c = w[2, 2]
        if not np.isfinite(c):
            return np.nan, self.unknown_cost
        inner = w[1:4, 1:4]
        e1 = np.nanmax(np.abs(inner - c)) if np.isfinite(inner).sum() > 1 else 0.0
        e2 = np.nanmax(np.abs(w - c)) if np.isfinite(w).sum() > 1 else 0.0
        known = np.isfinite(inner).mean()
        cost = self.w_edge1 * max(0.0, e1 - self.deadband) + self.w_edge2 * max(0.0, e2 - self.deadband)
        same = inner[np.isfinite(inner) & (np.abs(inner - c) < self.deadband)]    # 같은 면의 옆 칸 평균 (잡음 완화)
        return float(same.mean()), cost + self.unknown_cost * (1 - known)

    def choose(self, emap, nominal_xy, forward_xy, prev=None):
        """prev: 직전에 고른 (앞, 옆) 옮김. 반환 Foothold."""
        left = np.array([-forward_xy[1], forward_xy[0]])
        best = None
        for sx, sy in self.candidates:
            xy = nominal_xy + sx * forward_xy + sy * left
            h, cost = self.evaluate(emap, xy)
            cost += self.w_shift * abs(sx) + self.w_side * abs(sy)
            if prev is not None and abs(sx - prev[0]) < 1e-9 and abs(sy - prev[1]) < 1e-9:
                cost -= self.keep_bonus
            if best is None or cost < best.cost - 1e-12:
                best = Foothold(sx, sy, xy, h, cost)
        return best

    @staticmethod
    def path_max(emap, a_xy, b_xy, n=8):
        """두 점 사이 직선 위 지면 최고 높이 (모르면 NaN). 발을 옮기는 동안 걸릴 턱 확인용."""
        hs = [emap.height(a_xy + (b_xy - a_xy) * k / (n - 1)) for k in range(n)]
        hs = [h for h in hs if np.isfinite(h)]
        return max(hs) if hs else np.nan
