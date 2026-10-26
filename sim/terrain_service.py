"""Terrain Map Service: 지형의 단일 원천 (문서 §5.1, §5.3).

- 절대 높이(m) 격자를 원본으로 보관하고, MuJoCo heightfield에는 정규화 값으로 배포한다.
- heightfield 크기/해상도는 모델 컴파일 시점에 고정된다. 세계 지도가 크면 MuJoCo에는 로봇 주변 "창"만
  할당하고, 로봇이 창 중심에서 멀어지면 창을 옮겨 다시 채운다 (recenter). 창은 snap(기본 1 m) 단위로만
  옮기므로 옮기기 전후 같은 월드 위치의 높이가 그대로다 (발밑 지형이 흔들리지 않음).
- 로봇 발 근처 셀은 갱신을 미루고, 발이 떠난 뒤 반영한다 (발밑 지형 급변 방지).
- 지금은 패치(요철, 경사, 홈)를 직접 생성하지만, 나중에 Chrono SCM 변형 결과를
  같은 apply_heights() 경로로 넣으면 된다.
- 맵 서버 (sim/map_server.py, 1단계): attach_server() 뒤로는 지형 변경을 맵 서버에 쓰고(홈 = 파임, 요철·경사·둔덕 = 편집 add,
  Chrono 노드 = 소유 영역), 서버가 합친 물리 높이를 target에 받아 쓴다. 결과는 서버 없이 할 때와 비트 단위로 같다.
"""
import numpy as np


class TerrainMapService:
    def __init__(self, size, resolution, z_range, seed=0, window=None, snap=1.0):
        self.half_x, self.half_y = size
        self.res = resolution
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
        self._cand = None   # 반영 대기일 수 있는 셀 (rows, cols). None이면 다음 반영 때 세계 지도 전체를 비교한다
        self.version = 0            # 원천(target)이 바뀔 때 증가
        self.server = None          # 맵 서버 (attach_server 뒤). target은 그 서버가 합친 물리 높이의 사본
        self.time = 0.0             # 변경 기록에 붙일 시뮬레이션 시각 (호출하는 쪽이 갱신)
        self.applied_version = 0    # 물리엔진 반영(applied)이 바뀔 때 증가 (렌더러 갱신 기준)
        self.received = 0           # 받은 다른 쪽 변경 수 (운용자 편집 등: 맵 서버에서 받았거나 재현 시나리오의 map_write)
        self.received_mask = None   # 그 변경이 닿은 칸 (변형 MOP에서 뺀다: 흙 변형과 섞이지 않게)

        # MuJoCo heightfield 창 (window=None이면 세계 지도 전체 = 예전 방식)
        wx, wy = window if window is not None else size
        self.win_half = (min(wx, self.half_x), min(wy, self.half_y))
        self.wncol = int(round(2 * self.win_half[0] / resolution)) + 1
        self.wnrow = int(round(2 * self.win_half[1] / resolution)) + 1
        self.snap = snap
        steps = [v / resolution for v in (*self.win_half, snap, self.half_x, self.half_y)]
        assert all(abs(v - round(v)) < 1e-6 for v in steps), "창 반폭, snap, 세계 반폭은 격자 간격의 정수배여야 한다"
        self.windowed = (self.wncol, self.wnrow) != (self.ncol, self.nrow)
        self.wr0, self.wc0 = self._window_origin(0.0, 0.0)

    @classmethod
    def from_scenario(cls, scn):
        """시나리오의 시작 지형 (terrain 블록 + 시작 패치). 시뮬레이터와 바탕 파일(sim/map_base.py)이 같은 함수를 쓴다."""
        t = scn["terrain"]
        # window: MuJoCo에 올릴 로봇 주변 창의 반폭 (없으면 세계 지도 전체). 창은 window_snap(m) 단위로 옮긴다
        terrain = cls(t["size"], t["resolution"], t["z_range"], scn.get("seed", 0), window=t.get("window"),
                      snap=t.get("window_snap", 1.0))
        for p in t.get("patches", []):
            terrain.add_patch(p)
        return terrain

    # ---- MuJoCo 모델 빌드용 파라미터 (창 기준) ----
    @property
    def hfield_size(self):
        return [self.win_half[0], self.win_half[1], self.z_max - self.z_min, 0.1]

    @property
    def window_center(self):
        return self.xs[self.wc0] + self.win_half[0], self.ys[self.wr0] + self.win_half[1]

    @property
    def geom_pos(self):
        return [*self.window_center, self.z_min]

    def _window_origin(self, cx, cy):
        """원하는 창 중심(snap 단위로 맞춤) -> 창 왼쪽 아래 격자 인덱스 (세계 지도 안으로 제한)."""
        cx, cy = round(cx / self.snap) * self.snap, round(cy / self.snap) * self.snap
        c0 = int(round((cx - self.win_half[0] + self.half_x) / self.res))
        r0 = int(round((cy - self.win_half[1] + self.half_y) / self.res))
        return int(np.clip(r0, 0, self.nrow - self.wnrow)), int(np.clip(c0, 0, self.ncol - self.wncol))

    def recenter(self, x, y, trigger=1.0):
        """로봇이 창 중심에서 trigger(m)보다 멀어지면 창을 로봇 쪽으로 옮긴다. 옮겼으면 True
        (호출한 쪽이 MuJoCo 지형 geom 위치를 geom_pos로 바꾸고 write_all로 다시 채운다)."""
        if not self.windowed:
            return False
        cx, cy = self.window_center
        if max(abs(x - cx), abs(y - cy)) <= trigger:
            return False
        origin = self._window_origin(x, y)
        if origin == (self.wr0, self.wc0):
            return False
        self.wr0, self.wc0 = origin
        return True

    def attach_server(self, remote=None):
        """지금 target(시작 패치까지 반영된 지형)을 바탕으로 맵 서버를 만들고, 이후 변경은 서버를 거친다.
        remote: 클라이언트 이름을 주면 별도 프로세스 맵 서버(sim/map_server_node.py)의 원격 클라이언트 (2단계)."""
        if remote:
            from .map_client import RemoteMap
            self.server = RemoteMap(self.target, (self.z_min, self.z_max), name=remote)
            if self.server.joined:                       # 나중에 붙음: 서버에 이미 있던 변경까지 받은 지도에서 시작
                self.target = self.server.compose(0, self.nrow, 0, self.ncol)
                self.version += 1
                self._cand = None
        else:
            from .map_server import MapServer
            self.server = MapServer(self.target, (self.z_min, self.z_max))
        return self.server

    def poll_server(self):
        """원격 맵 서버에서 받은 다른 쪽 변경을 반영. 반환: 사본에 넣은 다른 쪽 변경 목록 (바꾼 뒤의 메시지: 재현 시나리오에 적는다).
        commit 전에 부른다."""
        if self.server is None or not hasattr(self.server, "poll") or self.server.poll() == 0:
            if self.server is not None and hasattr(self.server, "applied_log"):
                self.server.applied_log.clear()
            return []
        got, self.server.applied_log = self.server.applied_log, []
        for m in got:
            self._mark_received(m)
        self.target = self.server.compose(0, self.nrow, 0, self.ncol)
        self.version += max(len(got), 1)            # 받은 변경마다 하나 (재현 시나리오에서 map_write 이벤트마다 하나씩 올리는 것과 같게)
        self._cand = None
        return got

    def apply_map_message(self, m):
        """재현 시나리오의 map_write 이벤트: 실행 때 다른 쪽(운용자 편집 등)에서 받아 넣은 변경을 같은 시각에 그대로 넣는다."""
        from .map_client import apply_message
        apply_message(self.server, m)
        self._mark_received(m)
        self.target = self.server.compose(0, self.nrow, 0, self.ncol)
        self.version += 1
        self._cand = None

    def _mark_received(self, m):
        """받은 변경이 바꾼 칸을 표시 (add는 0이 아닌 칸, 나머지는 NaN이 아닌 칸)."""
        from .map_client import msg_cells
        if m.get("type") not in ("write", "cells"):
            return
        rows, cols, vals = msg_cells(m)
        sel = (vals != 0) if m["op"] == "add" else ~np.isnan(vals)
        if self.received_mask is None:
            self.received_mask = np.zeros((self.nrow, self.ncol), bool)
        self.received_mask[rows[sel], cols[sel]] = True
        self.received += 1

    # ---- 패치 생성 ----
    def add_patch(self, p):
        self.apply_delta(self.patch_delta(p))

    def patch_delta(self, p):
        """패치가 원천 지형에 더할 높이 격자 (아직 반영하지 않음). 실시간 경로는 시각이 정해진 패치를 미리 계산해 둔다
        (rough만 난수를 쓰므로 반영 순서대로 계산해야 한다: 미리 계산하지 않는다)."""
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
            # 선분을 따라가는 홈. 단면 profile: cosine(기본, 완만) 또는 box(평평한 바닥, 가파른 벽)
            (ax, ay), (bx, by) = p["start"], p["end"]
            d = np.array([bx - ax, by - ay]); length = np.linalg.norm(d); d /= length
            half_w = p["width"] / 2
            # 홈은 선분에서 half_w 안쪽뿐이라 그 사각형 밖은 0이다. 사각형만 계산한다 (세계 지도 전체면 실시간에서 약 100 ms. 값은 같다)
            c0 = max(int(np.searchsorted(self.xs, min(ax, bx) - half_w)) - 1, 0)
            c1 = min(int(np.searchsorted(self.xs, max(ax, bx) + half_w, side="right")) + 1, self.ncol)
            r0 = max(int(np.searchsorted(self.ys, min(ay, by) - half_w)) - 1, 0)
            r1 = min(int(np.searchsorted(self.ys, max(ay, by) + half_w, side="right")) + 1, self.nrow)
            px, py = X[r0:r1, c0:c1] - ax, Y[r0:r1, c0:c1] - ay
            along = px * d[0] + py * d[1]
            dist = np.abs(px * d[1] - py * d[0])
            inside = (dist < half_w) & (along >= 0) & (along <= length)
            if p.get("profile", "cosine") == "box":   # 평평한 바닥 + 가파른 벽 (Chrono SCM 바퀴 자국과 같은 모양)
                block = np.where(inside, -p["depth"], 0.0)
            else:
                block = np.where(inside, -p["depth"] * 0.5 * (1 + np.cos(np.pi * dist / half_w)), 0.0)
            return (slice(r0, r1), slice(c0, c1)), block      # 사각형만 (바깥은 0)
        else:
            raise ValueError(f"unknown patch kind: {kind}")
        return delta

    def apply_delta(self, delta):
        """patch_delta 결과를 원천에 더한다. 사각형 패치(홈)는 그 사각형만 고친다: 세계 지도 전체를 더하고 자르면 실시간에서 약 50 ms
        (원천은 늘 범위 안이라 바깥 셀은 전체 계산과 같은 값이다)."""
        if not isinstance(delta, tuple):
            if self.server is None:
                self.apply_heights(self.target + delta)
                return
            self.server.write("add", "event", 0, 0, delta, t=self.time)            # 요철·경사·둔덕: 편집 (E와 D를 함께 옮김)
            self.target = self.server.compose(0, self.nrow, 0, self.ncol)
            self.version += 1
            self._cand = None
            return
        (rs, cs), block = delta
        if self.server is None:
            self.target[rs, cs] = np.clip(self.target[rs, cs] + block, self.z_min, self.z_max)
        elif (block <= 0).all():                     # 홈: 파임 (바뀐 뒤의 절대 높이, 안 바뀐 칸은 NaN)
            new = np.clip(self.target[rs, cs] + block, self.z_min, self.z_max)
            new[block == 0] = np.nan
            r0, r1, c0, c1 = self.server.write("dig", "event", rs.start, cs.start, new, t=self.time)
            self.target[r0:r1, c0:c1] = self.server.compose(r0, r1, c0, c1)
        else:                                        # 높이는 사각형 패치: 편집
            r0, r1, c0, c1 = self.server.write("add", "event", rs.start, cs.start, block, t=self.time)
            self.target[r0:r1, c0:c1] = self.server.compose(r0, r1, c0, c1)
        self.version += 1
        if self._cand is not None:                 # 바뀔 수 있는 셀: 보류 중이던 셀 + 패치가 0이 아닌 셀
            rows, cols = np.nonzero(block)
            self._cand = (np.concatenate([self._cand[0], rows + rs.start]), np.concatenate([self._cand[1], cols + cs.start]))

    def apply_heights(self, heights):
        """외부(예: Chrono SCM)에서 받은 절대 높이 격자를 원천에 반영."""
        if self.server is not None:                  # 높이 전체를 정함: 편집 set
            self.server.write("set", "external", 0, 0, np.clip(heights, self.z_min, self.z_max), t=self.time)
            self.target = self.server.compose(0, self.nrow, 0, self.ncol)
        else:
            self.target = np.clip(heights, self.z_min, self.z_max)
        self.version += 1
        self._cand = None

    def set_cells(self, rows, cols, heights, writer="chrono"):
        """외부 물리엔진(Chrono SCM)이 계산한 일부 셀의 절대 높이를 원천에 반영. 격자 밖 셀은 버린다."""
        rows, cols, heights = np.asarray(rows), np.asarray(cols), np.asarray(heights, dtype=float)
        ok = (rows >= 0) & (rows < self.nrow) & (cols >= 0) & (cols < self.ncol)
        if ok.any():
            r, c, h = rows[ok], cols[ok], np.clip(heights[ok], self.z_min, self.z_max)
            if self.server is None:
                self.target[r, c] = h
            else:                                    # 소유 영역(Chrono)이면 own, 아니면 높이를 정함 (set: 지금 동작과 같은 덮어쓰기)
                own = self.server.owned[r, c]
                if own.any():
                    self.server.write_cells("own", writer, r[own], c[own], h[own], t=self.time)
                if (~own).any():
                    self.server.write_cells("set", writer, r[~own], c[~own], h[~own], t=self.time)
                r0, r1, c0, c1 = r.min(), r.max() + 1, c.min(), c.max() + 1
                self.target[r, c] = self.server.compose(r0, r1, c0, c1)[r - r0, c - c0]
            self.version += 1
            if self._cand is not None:
                self._cand = (np.concatenate([self._cand[0], rows[ok]]), np.concatenate([self._cand[1], cols[ok]]))
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
        # 바뀐 셀에서만 본다. 세계 지도 전체로 비교·거리 계산을 하면 1201x1201에서 약 30 ms라 실시간 관리 프로세스가 한 주기를 넘긴다.
        # 전체 비교는 지형 전체가 바뀐 뒤(패치)에만, 그 밖에는 보류된 셀과 새로 바뀐 셀(Chrono)만 (결과는 전체 비교와 같다)
        if self._cand is None:
            rows, cols = np.nonzero(self.target != self.applied)
        else:
            idx = np.unique(self._cand[0] * self.ncol + self._cand[1])
            rows, cols = idx // self.ncol, idx % self.ncol
            diff = self.target[rows, cols] != self.applied[rows, cols]
            rows, cols = rows[diff], cols[diff]
        self._cand = (rows, cols)
        if rows.size == 0:
            return False
        x, y = self.X[rows, cols], self.Y[rows, cols]
        keep = np.ones(rows.size, bool)
        for fx, fy in feet_xy:
            keep &= (x - fx) ** 2 + (y - fy) ** 2 > guard_radius ** 2
        if not keep.any():
            return False
        self._cand = (rows[~keep], cols[~keep])           # 발 근처라 보류한 셀
        rows, cols = rows[keep], cols[keep]
        self.applied[rows, cols] = self.target[rows, cols]
        self.applied_version += 1
        self.write_all(model, hfield_id)
        return True

    def write_all(self, model, hfield_id):
        """물리엔진에 반영된 지형 중 창 영역을 MuJoCo heightfield에 쓴다."""
        adr = model.hfield_adr[hfield_id]
        win = self.applied[self.wr0:self.wr0 + self.wnrow, self.wc0:self.wc0 + self.wncol]
        norm = (win - self.z_min) / (self.z_max - self.z_min)
        model.hfield_data[adr:adr + norm.size] = norm.ravel()

    def height_at(self, x, y):
        c = int(round((x + self.half_x) / (2 * self.half_x) * (self.ncol - 1)))
        r = int(round((y + self.half_y) / (2 * self.half_y) * (self.nrow - 1)))
        return float(self.applied[np.clip(r, 0, self.nrow - 1), np.clip(c, 0, self.ncol - 1)])
