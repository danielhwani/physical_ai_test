"""무른 흙 발 상호작용 시험 (sim/foot_soil.py, 시나리오 foot_soil).

    python conformance/test_foot_soil.py

1. 지지력 식(발 구 아래 Bekker 압력 적분)의 정적 침하가 Chrono SCM 기준(conformance/reference/scm_foot_static.json, 0.25 cm 격자)과
   2.6 mm 안에서 맞는다 (기준 다시 만들기: chrono 환경에서 scripts/scm_foot_static.py)
2. 옵션을 켜지 않으면 모델이 그대로다 (받침판 없음, 발 충돌 설정 그대로)
3. 서 있는 로봇: 소성 침하가 일어서는 동안 받은 최대 하중에서의 SCM 침하와 맞는다 (단단한 흙, 보통 흙, 무른 흙)
4. 발을 든 자리에 구멍이 남는다 (소성 지도)
5. 단단한·보통 흙에서 트롯이 넘어지지 않고 걷고, 흙이 무를수록 디딤 침하가 커진다
6. 화면 발자국: 구멍은 화면 전용 토픽으로만 나가고, 물리 지형과 센서가 받는 지형은 그대로다
7. 쟁기질 (흙 속을 옆이나 위로 지나가는 발의 저항): 무른 흙에서 전진이 줄고 흙에 준 일이 생긴다 (발을 빼며 위로 든 일 포함).
   끄면(plow_ratio 0) 저항 없이 더 멀리 간다. 단단한 흙은 발이 반지름보다 얕게 빠져 위에 덮인 흙이 없으므로 위쪽 저항이 거의 0이다
8. 지역마다 다른 흙: 겹치면 나중 지역, 경계 포함, 같은 흙은 합침, 개수 상한. 진흙 띠(soil_strip)에서 띠 안 디딤이 더 깊고,
   받침판 마찰이 그 흙의 tanφ로 바뀐다
C++ 실시간 코어와 같은 결과인지는 conformance/test_rt_core.py test_foot_soil_matches_runner (lockstep 비트 동일)
"""
import contextlib
import copy
import io
import json
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dis_console.protocol import SOIL_PRESETS  # noqa: E402
from sim.foot_soil import bearing  # noqa: E402
from sim.runner import Simulation  # noqa: E402

BASE = yaml.safe_load((ROOT / "scenarios/flat_trot.yaml").read_text())
REF = json.loads((ROOT / "conformance/reference/scm_foot_static.json").read_text())


def _sink(fn, kphi):
    soil = {**REF["soil"], "bekker_kphi": kphi}
    lo, hi = 0.0, 0.3
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if bearing(mid, REF["radius_m"], soil)[0] < fn else (lo, mid)
    return mid


def _scm(fn, kphi):
    rows = sorted((r["fn"], r["sink_m"]) for r in REF["rows"] if r["bekker_kphi"] == kphi)
    return float(np.interp(fn, [r[0] for r in rows], [r[1] for r in rows]))


def _run(scn, seconds, each=None):
    sim = Simulation(scn)
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < seconds:
            sim.step_control()
            if each:
                each(sim)
    return sim


def test_bearing_matches_scm_reference():
    worst = 0.0
    for r in REF["rows"]:
        z = _sink(r["fn"], r["bekker_kphi"])
        worst = max(worst, abs(z - r["sink_m"]))
        assert abs(z - r["sink_m"]) < 0.0026, (r, z)
    print(f"    정적 침하: SCM 기준과 최대 차이 {1000 * worst:.1f} mm ({len(REF['rows'])}개)")


def test_off_keeps_model():
    sim = Simulation(BASE)
    names = [sim.model.body(i).name for i in range(sim.model.nbody)]
    assert sim.foot_soil is None and not any(n.startswith("soil_plate_") for n in names)
    assert all(sim.model.geom_contype[g] == 1 for g in sim.foot_ids)


def test_standing_sinkage_matches_scm():
    base = copy.deepcopy(BASE)
    base["events"] = []
    for preset in ("hard", "firm", "soft"):
        scn = copy.deepcopy(base)
        scn["foot_soil"] = {"preset": preset}
        peak = np.zeros(4)
        sim = _run(scn, 2.0, lambda s: np.maximum(peak, s.foot_soil.fn, out=peak))
        kphi = SOIL_PRESETS[preset]["bekker_kphi"]
        for i in range(4):
            ref = _scm(peak[i], kphi)
            zp = sim.foot_soil.zp[i]
            assert abs(zp - ref) < max(0.2 * ref, 0.0025), (preset, i, peak[i], zp, ref)
        print(f"    서 있기 {preset}: 소성 침하 {np.round(100 * np.array(sim.foot_soil.zp), 2)} cm, "
              f"최대 하중 {np.round(peak)} N에서 SCM {[round(100 * _scm(p, kphi), 2) for p in peak]} cm")


def test_holes_remain():
    scn = copy.deepcopy(BASE)
    scn["foot_soil"] = {"preset": "firm"}
    sim = _run(scn, 4.0)
    fs = sim.foot_soil
    assert len(fs.plastic) > 50 and fs.stance_n > 10
    deepest, sink = max(fs.plastic.values()), fs.sink_max
    assert 0.7 * sink < deepest <= sink + 1e-12, (deepest, sink)   # 구 모양 구멍: 칸 중심이 구 중심에서 벗어난 만큼 얕다


def test_walks_on_hard_and_firm_and_softer_sinks_more():
    out = {}
    for name, cfg in (("rigid", None), ("hard", {"preset": "hard"}), ("firm", {"preset": "firm"}),
                      ("2e6", {"preset": "firm", "soil": {"bekker_kphi": 2.0e6}})):
        scn = copy.deepcopy(BASE)
        if cfg:
            scn["foot_soil"] = cfg
        sim = _run(scn, 10.0)
        out[name] = (float(sim.data.qpos[0]), sim.foot_soil.mop() if sim.foot_soil else None)
    for name in ("hard", "firm"):
        assert out[name][0] > 2.5, out
    sinks = [out[n][1]["foot_sink_mean_m"] for n in ("hard", "firm", "2e6")]
    assert sinks[0] < sinks[1] < sinks[2], sinks
    print("    10초 전진 (m): " + ", ".join(f"{n} {v[0]:.2f}" for n, v in out.items())
          + " | 평균 침하 (cm): " + ", ".join(f"{100 * s:.2f}" for s in sinks))


def test_plowing_slows_soft_soil():
    res = {}
    for preset, ratio in (("soft", 0.0), ("soft", 1.0), ("hard", 1.0)):
        scn = copy.deepcopy(BASE)
        scn["foot_soil"] = {"preset": preset, "plow_ratio": ratio}
        sim = _run(scn, 8.0)
        fs = sim.foot_soil
        res[preset, ratio] = (float(sim.data.qpos[0]), fs.plow_work, fs.plow_work_up, float(sim.data.qpos[2]))
    off, on, hard = res["soft", 0.0], res["soft", 1.0], res["hard", 1.0]
    assert off[1] == 0.0 and on[1] > 50.0 and on[2] > 5.0, res
    assert on[0] < 0.6 * off[0], res                               # 저항이 없으면 무른 흙에서 오히려 멀리 갔다
    assert on[3] > 0.12, res                                       # 넘어지거나 주저앉지 않음 (몸통 높이)
    assert hard[2] < 0.05 * hard[1], res                           # 단단한 흙: 위쪽 저항 거의 없음
    print(f"    무른 흙 8초 전진: 쟁기질 없음 {off[0]:.2f} m, 있음 {on[0]:.2f} m (흙에 준 일 {on[1]:.0f} J, 그중 위로 {on[2]:.0f} J), "
          f"단단한 흙 위로 {hard[2]:.1f} / {hard[1]:.0f} J")


def test_footprints_shown_on_screen_only():
    """발자국 구멍은 화면 전용 토픽(/sim/terrain_overlay) 패치로만 나간다. 물리 지형과 센서가 받는 지형 패치는 그대로다."""
    from sim.stream import StreamTap, TerrainOverlayStream
    from viz.stream_decode import TerrainGrid
    scn = copy.deepcopy(BASE)
    scn["foot_soil"] = {"preset": "soft"}
    sim = Simulation(scn)
    fs = sim.foot_soil
    tap = StreamTap(sim)
    overlay = TerrainOverlayStream(fs.display_depth, fs.display.shape)
    shown = TerrainGrid(tap.manifest["terrain"])
    terrain = TerrainGrid(tap.manifest["terrain"])
    applied0 = sim.terrain.applied.copy()
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < 4.0:
            sim.step_control()
            for p in tap.patches():
                terrain.apply(p)
            if (p := overlay.update()) is not None:
                shown.apply(p)
    depth = fs.display_depth()
    assert depth.max() > 0.03 and (depth > 0).sum() > 20, (depth.max(), (depth > 0).sum())
    assert np.array_equal(shown.heights, depth)                    # 패치 누적 = 표시 깊이
    assert np.array_equal(sim.terrain.applied, applied0)           # 물리 지형은 그대로
    assert np.array_equal(terrain.heights, applied0.astype(np.float32))   # 센서가 받는 지형에도 구멍 없음
    print(f"    화면 발자국: 격자 점 {(depth > 0).sum()}개, 가장 깊은 곳 {100 * depth.max():.1f} cm")


def test_soil_regions():
    from sim.foot_soil import MAX_REGIONS, soil_list
    cfg = {"preset": "firm", "regions": [{"region": [0, 0, 2, 2], "preset": "mud"}, {"region": [1, 1, 3, 3], "preset": "soft"},
                                         {"name": "again", "region": [5, 5, 6, 6], "preset": "mud"},
                                         {"region": [7, 7, 8, 8], "soil": {"bekker_kphi": 2.0e6}}]}
    soils, regions = soil_list(cfg)
    assert [n for n, _, _ in soils] == ["firm", "mud", "soft", "region3"], soils      # 같은 흙(mud 두 번)은 하나
    assert soils[3][1]["friction_deg"] == 30.0 and soils[3][1]["bekker_kphi"] == 2.0e6     # 지역은 기본 흙에서 덮어쓴다
    scn = copy.deepcopy(BASE)
    scn["foot_soil"] = cfg
    fs = Simulation(scn).foot_soil
    assert [fs.soil_at(*p) for p in ((-1, -1), (0.5, 0.5), (1.5, 1.5), (2.0, 2.0), (3.0, 3.0), (5.5, 5.5))] == [0, 1, 2, 2, 2, 1]
    for bad in ({"preset": "firm", "regions": [{"region": [1, 0, 0, 1], "preset": "mud"}]},
                {"preset": "firm", "regions": [{"region": [k, 0, k + 1, 1], "preset": "mud"} for k in range(MAX_REGIONS + 1)]},
                {"preset": "firm", "regions": [{"region": [k, 0, k + 1, 1], "soil": {"bekker_kphi": 1e6 + k}} for k in range(4)]}):
        try:
            soil_list(bad)
        except ValueError:
            continue
        raise AssertionError(f"잘못된 지역 설정을 받았다: {bad}")


def test_mud_strip():
    scn = yaml.safe_load((ROOT / "scenarios/soil_strip.yaml").read_text())
    scn["duration"] = 10.0
    seen = set()

    def each(sim):
        fs = sim.foot_soil
        for i, g in enumerate(fs.feet):
            k = fs.sidx[i] if fs.on[i] else fs.soil_at(*sim.data.geom_xpos[g][:2])
            assert sim.model.geom_friction[fs.plates[i], 0] == fs.soils[k]["friction"]
            seen.add(k)
    sim = _run(scn, 10.0, each)
    by = sim.foot_soil.mop()["foot_sink_by_soil"]
    assert seen == {0, 1} and by["firm"]["stances"] > 20 and by["mud_strip"]["stances"] > 10, by
    assert by["mud_strip"]["sink_mean_m"] > 3 * by["firm"]["sink_mean_m"], by
    assert sim.data.qpos[0] > 2.0                                   # 띠까지는 간다
    print(f"    진흙 띠: 보통 흙 디딤 {by['firm']['stances']}번 평균 {100 * by['firm']['sink_mean_m']:.1f} cm, "
          f"진흙 {by['mud_strip']['stances']}번 {100 * by['mud_strip']['sink_mean_m']:.1f} cm, 전진 {sim.data.qpos[0]:.2f} m")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
