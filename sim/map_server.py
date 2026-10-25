"""맵 서버 (docs/map_server.md). 1단계: 같은 프로세스 안 모드. 지형의 원천을 들고, TerrainMapService(sim/terrain_service.py)가 그
클라이언트다. 결과는 맵 서버 이전과 비트 단위로 같다 (conformance/test_map_server.py, 모든 시나리오 비교).

  층 (물리 높이 height만, 1단계)        합치는 규칙 (RULES, 설계 5.1)
    E  편집 지형 = 바탕 + 운용자 편집    add, set: ordered (받은 순서대로)
    D  파임 지형, 처음엔 +∞             dig: min (NaN = 안 바꿈)
    O  소유 영역의 높이                 own: owned (소유자가 보낸 절대 높이)
  물리 높이: 공유 칸 = min(E, D), 소유 칸 = O   (compose, 합치는 계산은 여기 한 곳)

  격자: TerrainMapService와 같은 칸 (행 = y, 열 = x). 타일 = TILE × TILE 칸. D와 O는 건드린 타일에만 만든다 (지연 할당).
  소유 영역: 칸 인덱스 반열린 구간 [r0, r1) × [c0, c1)과 소유자. 소유 시작 때 O <- 지금 물리 높이.
  1단계 예외 (설계 5.7): 소유 영역에 다른 쪽이 쓴 변경(홈 이벤트 등)은 소유자에게 넘기지 않고 O에 바로 적용한다 (지금 동작과 같음).
  높이 범위: 지금 TerrainMapService가 매번 z_range로 자르므로 add도 저장할 때 자른다 (자르기가 단조라 결과가 같다).
  변경 기록: log (판, 시각, 쓴 쪽, 종류, 칸 수, 사각형). 높이 값은 싣지 않는다 (1단계).
"""
import numpy as np

RULES = {"dig": "min", "add": "ordered", "set": "ordered", "own": "owned"}
TILE = 200
FORMAT = 1


def merge_dig(d, new):
    """파임 합치기 (min). new의 NaN은 안 바꿈. 순서·중복과 무관."""
    return np.fmin(d, new)


def merge_own(o, new):
    """소유 영역 쓰기: new가 NaN이 아니면 그 값."""
    return np.where(np.isnan(new), o, new)


class MapServer:
    def __init__(self, base, z_range, tile=TILE):
        """base: 바탕 높이 격자 (nrow, ncol), 절대 높이 m (TerrainMapService.target의 시작 값)."""
        self.nrow, self.ncol = base.shape
        self.z_min, self.z_max = z_range
        self.tile = tile
        self.E = np.array(base, dtype=float)
        self.D = {}                     # (tr, tc) -> 타일 배열, +∞로 시작
        self.O = {}                     # (tr, tc) -> 타일 배열 (소유 칸만 의미)
        self.owners = []                # [(r0, r1, c0, c1, 쓴 쪽)]
        self.owned = np.zeros(base.shape, bool)
        self.version = 0
        self.tile_version = {}
        self.log = []
        self.counts = {}

    # ---- 타일 ----
    def _tiles(self, r0, r1, c0, c1):
        t = self.tile
        for tr in range(r0 // t, (r1 - 1) // t + 1):
            for tc in range(c0 // t, (c1 - 1) // t + 1):
                a0, a1 = max(r0, tr * t), min(r1, (tr + 1) * t)
                b0, b1 = max(c0, tc * t), min(c1, (tc + 1) * t)
                yield (tr, tc), (slice(a0 - tr * t, a1 - tr * t), slice(b0 - tc * t, b1 - tc * t)), \
                    (slice(a0 - r0, a1 - r0), slice(b0 - c0, b1 - c0)), (slice(a0, a1), slice(b0, b1))

    def _tile_shape(self, key):
        tr, tc = key
        return (min(self.tile, self.nrow - tr * self.tile), min(self.tile, self.ncol - tc * self.tile))

    def _get(self, layer, key, fill):
        if key not in layer:
            layer[key] = np.full(self._tile_shape(key), fill)
        return layer[key]

    # ---- 읽기 ----
    def compose(self, r0, r1, c0, c1):
        """사각형 [r0, r1) × [c0, c1)의 물리 높이."""
        out = self.E[r0:r1, c0:c1].copy()
        for key, ts, os_, gs in self._tiles(r0, r1, c0, c1):
            if key in self.D:
                out[os_] = np.minimum(out[os_], self.D[key][ts])
            if key in self.O:
                m = self.owned[gs]
                out[os_][m] = self.O[key][ts][m]
        return out

    # ---- 소유 영역 ----
    def set_owner(self, r0, r1, c0, c1, writer):
        """[r0, r1) × [c0, c1)을 writer가 소유한다. O <- 지금 물리 높이."""
        h = self.compose(r0, r1, c0, c1)
        for key, ts, os_, gs in self._tiles(r0, r1, c0, c1):
            self._get(self.O, key, np.nan)[ts] = h[os_]
        self.owned[r0:r1, c0:c1] = True
        self.owners.append((r0, r1, c0, c1, writer))

    # ---- 쓰기 ----
    def _record(self, op, writer, t, n, rect):
        self.version += 1
        self.log.append({"v": self.version, "t": t, "writer": writer, "op": op, "cells": int(n), "rect": [int(v) for v in rect]})
        k = f"{writer}/{op}"
        self.counts[k] = self.counts.get(k, 0) + 1
        r0, r1, c0, c1 = rect
        for key, *_ in self._tiles(r0, r1, c0, c1):
            self.tile_version[key] = self.tile_version.get(key, 0) + 1

    def write(self, op, writer, r0, c0, block, t=None):
        """사각형 쓰기. dig / own / set: block = 절대 높이 (NaN = 안 바꿈). add: block = 변화량 (0 = 안 바꿈).
        반환: 바뀌었을 수 있는 사각형 (r0, r1, c0, c1)."""
        assert op in RULES, op
        block = np.asarray(block, dtype=float)
        r1, c1 = r0 + block.shape[0], c0 + block.shape[1]
        for key, ts, os_, gs in self._tiles(r0, r1, c0, c1):
            b = block[os_]
            own = self.owned[gs]
            if op == "dig":
                if (~own).any():
                    d = self._get(self.D, key, np.inf)
                    d[ts] = np.where(own, d[ts], merge_dig(d[ts], b))
                if own.any():                                         # 1단계 예외: 소유 영역에 바로
                    o = self.O[key]
                    o[ts] = np.where(own, merge_own(o[ts], b), o[ts])
            elif op == "own":
                assert own[~np.isnan(b)].all(), "소유 영역 밖에 own으로 쓰려 했다"
                o = self._get(self.O, key, np.nan)
                o[ts] = merge_own(o[ts], b)
            elif op == "add":
                nz = b != 0.0
                e = self.E[gs]
                self.E[gs] = np.where(nz & ~own, np.clip(e + b, self.z_min, self.z_max), e)
                if key in self.D:
                    d = self.D[key]
                    shift = nz & ~own & np.isfinite(d[ts])
                    d[ts] = np.where(shift, np.clip(d[ts] + b, self.z_min, self.z_max), d[ts])
                if own.any():
                    o = self.O[key]
                    o[ts] = np.where(nz & own, np.clip(o[ts] + b, self.z_min, self.z_max), o[ts])
            elif op == "set":
                ok = ~np.isnan(b)
                self.E[gs] = np.where(ok & ~own, b, self.E[gs])
                if key in self.D:
                    self.D[key][ts] = np.where(ok & ~own, np.inf, self.D[key][ts])
                if own.any():
                    o = self.O[key]
                    o[ts] = np.where(ok & own, b, o[ts])
        self._record(op, writer, t, np.count_nonzero(~np.isnan(block)) if op != "add" else np.count_nonzero(block),
                     (r0, r1, c0, c1))
        return r0, r1, c0, c1

    def write_cells(self, op, writer, rows, cols, values, t=None):
        """흩어진 칸 쓰기 (Chrono SCM이 바뀐 노드만 보낼 때). 칸들을 감싸는 사각형에 NaN으로 채워 write()로."""
        rows, cols = np.asarray(rows), np.asarray(cols)
        if rows.size == 0:
            return None
        r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        block = np.full((r1 - r0, c1 - c0), np.nan)
        block[rows - r0, cols - c0] = values
        return self.write(op, writer, int(r0), int(c0), block, t=t)

    def summary(self):
        """변경 기록 요약 (실행 폴더 map_updates.json용)."""
        return {"format": FORMAT, "rules": RULES, "tile": self.tile, "grid": [self.nrow, self.ncol], "version": self.version,
                "counts": self.counts, "owners": [list(o) for o in self.owners],
                "tiles_with_dig": len(self.D), "tiles_with_owned": len(self.O)}
