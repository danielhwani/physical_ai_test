"""Terrain Map Service: 지형의 단일 원천 (문서 §5.1, §5.3).

- 절대 높이(m) 격자를 원본으로 보관하고, MuJoCo heightfield에는 정규화 값으로 배포한다.
- heightfield 크기/해상도는 모델 컴파일 시점에 고정되므로 시나리오 영역 전체를 미리 할당한다.
- 로봇 발 근처 셀은 갱신을 미루고, 발이 떠난 뒤 반영한다 (발밑 지형 급변 방지).
- 지금은 패치(요철, 경사, 홈)를 직접 생성하지만, 나중에 Chrono SCM 변형 결과를
  같은 apply_heights() 경로로 넣으면 된다.
"""
import numpy as np


class TerrainMapService:
    def __init__(self, size, resolution, z_range, seed=0):
        self.half_x, self.half_y = size
        self.z_min, self.z_max = z_range
        self.ncol = int(round(2 * self.half_x / resolution)) + 1
        self.nrow = int(round(2 * self.half_y / resolution)) + 1
        self.rng = np.random.default_rng(seed)
        # MuJoCo hfield 규약: data[r, c] -> x = (2c/(ncol-1) - 1) * half_x, y = (2r/(nrow-1) - 1) * half_y
        self.xs = np.linspace(-self.half_x, self.half_x, self.ncol)
        self.ys = np.linspace(-self.half_y, self.half_y, self.nrow)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.target = np.zeros((self.nrow, self.ncol))    # 최신 지형 (원천)
        self.applied = self.target.copy()                  # 물리엔진에 반영된 지형
        self.version = 0            # 원천(target)이 바뀔 때 증가
        self.applied_version = 0    # 물리엔진 반영(applied)이 바뀔 때 증가 (렌더러 갱신 기준)

    # ---- MuJoCo 모델 빌드용 파라미터 ----
    @property
    def hfield_size(self):
        return [self.half_x, self.half_y, self.z_max - self.z_min, 0.1]

    @property
    def geom_pos(self):
        return [0.0, 0.0, self.z_min]

    # ---- 패치 생성 ----
    def add_patch(self, p):
        kind = p["kind"]
        X, Y = self.X, self.Y
        if kind == "bump":
            cx, cy = p["center"]
            r2 = (X - cx) ** 2 + (Y - cy) ** 2
            delta = p["height"] * np.exp(-r2 / (2 * (p["radius"] / 2) ** 2))
        elif kind == "rough":
            x0, y0, x1, y1 = p["region"]
            noise = self.rng.uniform(-1, 1, X.shape) * p["amplitude"]
            delta = np.where(self._in(x0, y0, x1, y1), noise, 0.0)
        elif kind == "ramp":
            # 구간 [x0, x1]에서 rise만큼 오르고 이후 평탄 유지
            x0, y0, x1, y1 = p["region"]
            frac = np.clip((X - x0) / (x1 - x0), 0, 1)
            delta = np.where((Y >= y0) & (Y <= y1), frac * p["rise"], 0.0)
        elif kind == "rut":
            # 선분을 따라가는 홈 (cosine 단면)
            (ax, ay), (bx, by) = p["start"], p["end"]
            d = np.array([bx - ax, by - ay]); length = np.linalg.norm(d); d /= length
            px, py = X - ax, Y - ay
            along = px * d[0] + py * d[1]
            dist = np.abs(px * d[1] - py * d[0])
            half_w = p["width"] / 2
            inside = (dist < half_w) & (along >= 0) & (along <= length)
            delta = np.where(inside, -p["depth"] * 0.5 * (1 + np.cos(np.pi * dist / half_w)), 0.0)
        else:
            raise ValueError(f"unknown patch kind: {kind}")
        self.apply_heights(self.target + delta)

    def apply_heights(self, heights):
        """외부(예: Chrono SCM)에서 받은 절대 높이 격자를 원천에 반영."""
        self.target = np.clip(heights, self.z_min, self.z_max)
        self.version += 1

    def set_cells(self, rows, cols, heights):
        """외부 물리엔진(Chrono SCM)이 계산한 일부 셀의 절대 높이를 원천에 반영. 격자 밖 셀은 버린다."""
        rows, cols, heights = np.asarray(rows), np.asarray(cols), np.asarray(heights, dtype=float)
        ok = (rows >= 0) & (rows < self.nrow) & (cols >= 0) & (cols < self.ncol)
        if ok.any():
            self.target[rows[ok], cols[ok]] = np.clip(heights[ok], self.z_min, self.z_max)
            self.version += 1
        return int(ok.sum())

    def cell_of(self, x, y):
        """월드 좌표가 격자점에 정확히 놓이면 (row, col), 아니면 None."""
        c, r = (x + self.half_x) / (2 * self.half_x) * (self.ncol - 1), (y + self.half_y) / (2 * self.half_y) * (self.nrow - 1)
        if abs(c - round(c)) > 1e-6 or abs(r - round(r)) > 1e-6:
            return None
        return int(round(r)), int(round(c))

    def _in(self, x0, y0, x1, y1):
        return (self.X >= x0) & (self.X <= x1) & (self.Y >= y0) & (self.Y <= y1)

    # ---- 물리엔진 배포 ----
    def commit(self, model, hfield_id, feet_xy=(), guard_radius=0.15):
        """대기 중인 변경을 MuJoCo에 반영. 발 근처 셀은 보류. 변경이 있었으면 True."""
        pending = self.target != self.applied
        if not pending.any():
            return False
        for fx, fy in feet_xy:
            pending &= (self.X - fx) ** 2 + (self.Y - fy) ** 2 > guard_radius ** 2
        if not pending.any():
            return False
        self.applied[pending] = self.target[pending]
        self.applied_version += 1
        self.write_all(model, hfield_id)
        return True

    def write_all(self, model, hfield_id):
        adr = model.hfield_adr[hfield_id]
        norm = (self.applied - self.z_min) / (self.z_max - self.z_min)
        model.hfield_data[adr:adr + norm.size] = norm.ravel()

    def height_at(self, x, y):
        c = int(round((x + self.half_x) / (2 * self.half_x) * (self.ncol - 1)))
        r = int(round((y + self.half_y) / (2 * self.half_y) * (self.nrow - 1)))
        return float(self.applied[np.clip(r, 0, self.nrow - 1), np.clip(c, 0, self.ncol - 1)])
