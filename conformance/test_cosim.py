"""MuJoCo + Chrono 연동 시험 (문서 §5).

    python conformance/test_cosim.py         (Chrono 프로세스를 띄우므로 1분 정도 걸린다)

1. 칸 정합: Chrono 바퀴(바퀴 축 바디) 위치 아래에 바퀴 자국이 MuJoCo 지형으로 들어오는지
   (인덱스 행/열 뒤바뀜, 원점 어긋남, 해상도 불일치를 잡는다)
2. 결정성: 같은 시나리오를 두 번 돌리면 로봇, 차량, 지형이 모두 같은지 (파이프라인 동기 포함)
3. 단방향: 로봇이 무엇을 하든 차량 궤적은 같은지 (로봇 -> 차량 물리 작용 없음)
4. 스트림: Chrono 바디가 physics_source=Chrono로 매니페스트와 TF 포즈에 들어가는지
5. 발자국: 로봇 발 하중으로 판 자국이 디딘 발 아래에 생기고, 깊이가 하중-흙 강도 관계를 따르며(무른 흙이 더 깊음),
   과하게 파이지 않는지(대리 구가 흙에 떨어져 충격으로 파는 문제 재발 방지), 결정적인지
"""
import contextlib
import copy
import io
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.runner import Simulation  # noqa: E402
from sim.stream import build_manifest  # noqa: E402

SCN = yaml.safe_load((ROOT / "scenarios/vehicle_crossing.yaml").read_text())
FOOT = yaml.safe_load((ROOT / "scenarios/footprints.yaml").read_text())


def _run(scn, seconds, on_step=None):
    sim = Simulation(scn)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < seconds:
                sim.step_control()
                if on_step:
                    on_step(sim)
        return (sim.data.qpos.copy(), sim.terrain.applied.copy(),
                [p.copy() for _, p, _ in sim.cosim.stream_poses()], sim)
    finally:
        sim.close()


def test_rut_lands_under_wheels():
    spindle_x = []

    def record(sim):
        # 차체가 y≈0을 지날 때 바퀴 축 바디의 x (자국이 생길 위치)
        if abs(sim.cosim.vehicle["pos"][1]) < 0.1:
            spindle_x.extend(p[0] for n, p, _ in sim.cosim.stream_poses() if "spindle" in n)

    _, H, _, sim = _run(SCN, 6.5, record)
    assert spindle_x, "차량이 y=0을 지나지 않음"
    ter = sim.terrain
    rows = np.abs(ter.ys - 0.0) < 1.0                          # y = -1..1 m 띠
    deep = (H < -0.02) & rows[:, None]
    xs = np.broadcast_to(ter.xs, H.shape)[deep]
    assert xs.size > 50, "자국이 MuJoCo 지형에 들어오지 않음"
    left, right = xs[xs < 3.0], xs[xs > 3.0]
    wl, wr = min(spindle_x), max(spindle_x)
    # 바퀴 축은 타이어 중심선이므로 자국 중심(x 평균)과 15 cm 안에서 맞아야 한다
    assert abs(left.mean() - wl) < 0.15 and abs(right.mean() - wr) < 0.15, \
        f"자국 중심 {left.mean():.2f}/{right.mean():.2f} vs 바퀴 {wl:.2f}/{wr:.2f}"
    assert not np.any((xs > wl + 0.3) & (xs < wr - 0.3)), "바퀴 사이(차체 아래)에 자국이 생김"


def test_determinism():
    q1, H1, v1, _ = _run(SCN, 5.0)
    q2, H2, v2, _ = _run(SCN, 5.0)
    assert np.array_equal(q1, q2) and np.array_equal(H1, H2)
    assert all(np.array_equal(a, b) for a, b in zip(v1, v2))


def test_one_way_coupling():
    standing = copy.deepcopy(SCN)
    standing["events"] = []                                    # 로봇은 제자리에 서 있기만
    _, _, v_walk, _ = _run(SCN, 5.0)
    _, _, v_stand, _ = _run(standing, 5.0)
    assert all(np.array_equal(a, b) for a, b in zip(v_walk, v_stand)), "로봇 동작이 차량에 영향을 줌"


def test_stream_includes_chrono_bodies():
    sim = Simulation(SCN)
    try:
        m = build_manifest(sim.model, sim.terrain, sim.cosim)
        chrono_bodies = [b for b in m["bodies"] if b["physics_source"] == "Chrono"]
        assert len(chrono_bodies) == 5, chrono_bodies                 # 차체 + 바퀴 축 4
        names = {b["name"] for b in chrono_bodies}
        vis = [v for v in m["visuals"] if v["body"] in names]
        assert len(vis) == 9 and all((ROOT / v["mesh"]).exists() for v in vis)
        assert [n for n, _, _ in sim.cosim.stream_poses()] == [b["name"] for b in chrono_bodies]
        assert len({b["id"] for b in m["bodies"]}) == len(m["bodies"]), "바디 id 중복"
    finally:
        sim.close()


def _footprints(scn, seconds):
    stance = []

    def record(sim):
        f = sim.foot_normal_forces()
        stance.extend(sim.data.geom_xpos[g][:2].copy() for i, g in enumerate(sim.foot_ids) if f[i] > 20)

    q, H, _, sim = _run(scn, seconds, record)
    prints = H < -0.005
    xs = np.broadcast_to(sim.terrain.xs, H.shape)[prints]
    ys = np.broadcast_to(sim.terrain.ys[:, None], H.shape)[prints]
    return q, H, prints, xs, ys, np.array(stance)


def test_footprints_under_stance_feet():
    # 12초: Chrono 시간 간격 2 ms일 때 9초쯤 대리 구가 순간적으로 박히던 문제(-15 cm)까지 지나도록
    _, H, prints, xs, ys, stance = _footprints(FOOT, 12.0)
    assert prints.sum() > 50, "발자국이 MuJoCo 지형에 들어오지 않음"
    d = np.min(np.hypot(xs[:, None] - stance[None, :, 0], ys[:, None] - stance[None, :, 1]), axis=1)
    assert np.median(d) < 0.03 and np.percentile(d, 95) < 0.06, f"발자국이 발 위치와 어긋남 (중앙 {np.median(d):.3f} m)"
    depth = H[prints]
    # 단독 측정(kphi 1e7: 50 N -> 1.8 cm, 80 N -> 2.8 cm)과 같은 범위. 과도한 침하(이전 문제: -15 cm) 재발 금지
    assert -0.03 < depth.mean() < -0.005 and depth.min() > -0.06, f"발자국 깊이 이상: 평균 {depth.mean():.3f}, 최대 {depth.min():.3f}"


def test_footprint_depth_follows_soil():
    firm = copy.deepcopy(FOOT)
    firm["chrono"]["scm"]["soil"]["bekker_kphi"] = 4.0e7           # 더 단단한 흙
    _, H_soft, p_soft, *_ = _footprints(FOOT, 4.0)
    _, H_firm, p_firm, *_ = _footprints(firm, 4.0)
    assert H_soft[p_soft].mean() < H_firm[p_firm].mean(), "무른 흙의 발자국이 더 깊어야 한다 (하중으로 누르는지 확인)"


def test_footprints_determinism():
    q1, H1, *_ = _footprints(FOOT, 3.0)
    q2, H2, *_ = _footprints(FOOT, 3.0)
    assert np.array_equal(q1, q2) and np.array_equal(H1, H2)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
