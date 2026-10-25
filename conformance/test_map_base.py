"""맵 서버 바탕 파일 (sim/map_base.py, docs/map_server.md 5.3, 2단계 2c) 시험. ROS2 없이 (몇 초).

    python conformance/test_map_base.py

  쓰고 읽기: 타일로 나눠 쓰고 다시 읽으면 비트 단위로 같다 (타일 경계 200칸을 넘는 크기, 끝 타일이 작은 격자).
  고친 파일 찾기: 타일 파일 하나의 한 칸을 바꾸면 읽을 때 그 타일을 적어 멈춘다. 매니페스트 해시를 바꿔도 마찬가지.
  시나리오 바탕: 시작 패치가 있는 시나리오의 바탕 = 시뮬레이터 시작 지형 (같은 함수), 두 번 만들어도 같다 (같은 해시, 다시 쓰지 않음).
"""
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.map_base import build_for_scenario, load_base, tile_shas, write_base  # noqa: E402
from sim.map_client import sha  # noqa: E402
from sim.runner import load_yaml  # noqa: E402
from sim.terrain_service import TerrainMapService  # noqa: E402


def test_roundtrip(tmp):
    h = np.random.default_rng(0).uniform(-0.1, 0.1, (433, 517))
    mf = write_base(h, tmp / "a", (-0.3, 0.4), name="a")
    assert len(mf["tiles"]) == 3 * 3 and mf["tiles"]["2,2"]["shape"] == [33, 117]
    g, mf2 = load_base(tmp / "a")
    assert np.array_equal(g, h) and mf2["base_sha256"] == sha(h) and {k: t["sha256"] for k, t in mf2["tiles"].items()} == tile_shas(h)


def test_tamper_found(tmp):
    h = np.zeros((401, 401))
    write_base(h, tmp / "b", (-0.3, 0.4))
    f = tmp / "b/height/1_2.npy"
    a = np.load(f)
    a[10, 0] += 1e-12                                        # 아주 작은 차이도 (끝 타일 1,2는 200 x 1칸)
    np.save(f, a)
    try:
        load_base(tmp / "b")
        raise SystemExit("FAIL: 고친 타일 파일을 못 찾았다")
    except RuntimeError as e:
        assert "1,2" in str(e) and "0,0" not in str(e), e
    write_base(h, tmp / "c", (-0.3, 0.4))
    mf = json.loads((tmp / "c/manifest.json").read_text())
    mf["tiles"]["0,1"]["sha256"] = "0" * 64
    (tmp / "c/manifest.json").write_text(json.dumps(mf))
    try:
        load_base(tmp / "c")
        raise SystemExit("FAIL: 매니페스트 해시가 틀린 것을 못 찾았다")
    except RuntimeError as e:
        assert "0,1" in str(e), e


def test_scenario_base(tmp):
    for name in ("rough_rut", "vehicle_crossing", "footprints"):
        scn = load_yaml(ROOT / f"scenarios/{name}.yaml")
        mf = build_for_scenario(scn, tmp / name)
        start = TerrainMapService.from_scenario(scn).target           # 시뮬레이터 시작 지형 (Simulation이 쓰는 함수)
        g, _ = load_base(tmp / name)
        assert np.array_equal(g, start), name
        f = tmp / name / "height/0_0.npy"
        t0 = f.stat().st_mtime_ns
        assert build_for_scenario(scn, tmp / name)["base_sha256"] == mf["base_sha256"]   # 두 번째: 같은 해시, 다시 쓰지 않음
        assert f.stat().st_mtime_ns == t0, f"{name}: 같은 바탕인데 다시 썼다"


if __name__ == "__main__":
    tmp = Path(tempfile.mkdtemp(prefix="map_base_test_"))
    try:
        for name, fn in list(globals().items()):
            if name.startswith("test_"):
                fn(tmp)
                print(f"{name}: OK")
    finally:
        shutil.rmtree(tmp)
