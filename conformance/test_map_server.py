"""맵 서버 1단계 (sim/map_server.py, docs/map_server.md) 시험: 합치는 규칙과, 서버를 거쳐도 지형이 비트 단위로 같은지.

    python conformance/test_map_server.py          (mj_ros 환경, 몇 초)

  규칙: 파임 min은 순서·중복과 무관, NaN은 안 바꿈, +∞로 시작. add는 E와 D를 함께 옮김. set은 자국을 지움. 소유 영역은 O.
  비트 동일: 같은 패치·Chrono 노드 쓰기를 서버 없이 / 있게 적용한 TerrainMapService.target이 같다 (요철·경사·둔덕·홈, 높이 범위 끝 포함).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.map_server import MapServer, merge_dig  # noqa: E402
from sim.terrain_service import TerrainMapService  # noqa: E402


def test_dig_min_order_and_duplicates():
    rng = np.random.default_rng(0)
    base = rng.uniform(-0.05, 0.05, (300, 450))             # 타일 경계(200)를 넘는 크기
    writes = []
    for _ in range(40):
        r0, c0 = rng.integers(0, 250), rng.integers(0, 400)
        h, w = rng.integers(1, 50), rng.integers(1, 50)
        block = base[r0:r0 + h, c0:c0 + w] - rng.uniform(0, 0.03, (h, w))
        block[rng.random((h, w)) < 0.3] = np.nan            # 안 바꾼 칸
        writes.append((r0, c0, block))
    results = []
    for order in (writes, writes[::-1], writes + writes[:10]):   # 순서 바꿈, 중복
        s = MapServer(base, (-0.2, 0.2))
        for r0, c0, b in order:
            s.write("dig", "test", r0, c0, b)
        results.append(s.compose(0, 300, 0, 450))
    assert np.array_equal(results[0], results[1]) and np.array_equal(results[0], results[2])


def test_nan_and_inf():
    assert np.array_equal(merge_dig(np.array([np.inf, 1.0, 2.0]), np.array([np.nan, np.nan, 1.5])), [np.inf, 1.0, 1.5])
    s = MapServer(np.zeros((10, 10)), (-1, 1))
    assert np.array_equal(s.compose(0, 10, 0, 10), np.zeros((10, 10)))           # D 없음 = +∞
    s.write("dig", "t", 2, 2, np.full((2, 2), np.nan))
    assert np.array_equal(s.compose(0, 10, 0, 10), np.zeros((10, 10)))           # NaN만 = 안 바꿈
    s.write("dig", "t", 2, 2, np.full((2, 2), 0.3))
    assert np.array_equal(s.compose(0, 10, 0, 10), np.zeros((10, 10)))           # 높아지는 파임은 min이 버림


def test_add_moves_ruts_and_set_clears():
    s = MapServer(np.zeros((5, 5)), (-1, 1))
    s.write("dig", "t", 1, 1, np.full((1, 1), -0.1))
    s.write("add", "t", 0, 0, np.full((5, 5), 0.05))
    h = s.compose(0, 5, 0, 5)
    assert np.isclose(h[1, 1], -0.05) and np.isclose(h[0, 0], 0.05)              # 자국 모양 그대로 함께 옮김
    s.write("set", "t", 1, 1, np.full((1, 1), 0.2))
    assert np.isclose(s.compose(1, 2, 1, 2)[0, 0], 0.2)                           # set: 자국을 지우고 그 높이


def test_owned_region():
    s = MapServer(np.zeros((20, 20)), (-1, 1))
    s.write("dig", "t", 5, 5, np.full((1, 1), -0.1))
    s.set_owner(4, 10, 4, 10, "chrono")                                          # O <- 지금 물리 높이
    assert np.isclose(s.compose(5, 6, 5, 6)[0, 0], -0.1)
    s.write_cells("own", "chrono", [6, 7], [6, 7], [0.05, -0.2])                 # 오르내림 모두 (소유자 값 그대로)
    h = s.compose(0, 20, 0, 20)
    assert np.isclose(h[6, 6], 0.05) and np.isclose(h[7, 7], -0.2)
    s.write("dig", "event", 6, 6, np.full((1, 1), 0.0))                          # 1단계 예외: 소유 영역에 다른 쪽 쓰기는 바로
    assert np.isclose(s.compose(6, 7, 6, 7)[0, 0], 0.0)
    refused = False
    try:
        s.write_cells("own", "chrono", [15], [15], [0.1])
    except AssertionError:
        refused = True
    assert refused, "소유 영역 밖에 own으로 쓰는 것이 막히지 않았다"


def _terrain():
    t = TerrainMapService((5.0, 3.0), 0.05, (-0.2, 0.4), seed=3, window=(2.0, 2.0))
    t.add_patch({"kind": "rough", "amplitude": 0.015, "region": [1.0, -2.0, 4.0, 2.0]})
    t.add_patch({"kind": "bump", "center": [-1.0, 0.0], "height": 0.05, "radius": 0.4})
    return t


def test_bit_identical_with_server():
    """서버 없이 / 있게 같은 변경: target이 비트 단위로 같다. Chrono 소유 영역, 높이 범위 끝(z_min) 포함."""
    plain, served = _terrain(), _terrain()
    served.attach_server()
    served.server.set_owner(20, 60, 30, 90, "chrono")
    rng = np.random.default_rng(1)
    events = [{"kind": "rut", "start": [-4.0, -1.0], "end": [4.0, -1.0], "width": 0.3, "depth": 0.04},
              {"kind": "rut", "start": [0.0, -2.5], "end": [0.0, 2.5], "width": 0.25, "depth": 0.05, "profile": "box"},
              {"kind": "ramp", "region": [2.0, -1.0, 3.0, 1.0], "rise": 0.1},
              {"kind": "rut", "start": [-2.0, 0.5], "end": [3.0, 0.5], "width": 0.3, "depth": 0.5},   # z_min(-0.2)에서 잘림
              {"kind": "bump", "center": [1.5, -1.0], "height": 0.08, "radius": 0.6},
              {"kind": "rough", "amplitude": 0.01, "region": [-3.0, -1.0, 0.0, 1.0]}]
    for k, ev in enumerate(events):
        for t in (plain, served):
            t.add_patch(ev)
        rows, cols = rng.integers(20, 60, 50), rng.integers(30, 90, 50)            # Chrono 노드 쓰기 (소유 영역 안)
        h = rng.uniform(-0.25, 0.1, 50)
        for t in (plain, served):
            t.set_cells(rows, cols, h)
        assert np.array_equal(plain.target, served.target), f"이벤트 {k} ({ev['kind']}) 뒤 다름"
    assert served.server.counts, "서버에 쓴 것이 없다"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"{name}: OK")
