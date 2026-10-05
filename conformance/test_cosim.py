"""MuJoCo + Chrono 연동 시험 (문서 §5).

    python conformance/test_cosim.py         (Chrono 프로세스를 띄우므로 1분 정도 걸린다)

1. 칸 정합: Chrono 바퀴(바퀴 축 바디) 위치 아래에 바퀴 자국이 MuJoCo 지형으로 들어오는지
   (인덱스 행/열 뒤바뀜, 원점 어긋남, 해상도 불일치를 잡는다)
2. 결정성: 같은 시나리오를 두 번 돌리면 로봇, 차량, 지형이 모두 같은지 (파이프라인 동기 포함)
3. 단방향: 로봇이 무엇을 하든 차량 궤적은 같은지 (로봇 -> 차량 물리 작용 없음)
4. 스트림: Chrono 바디가 physics_source=Chrono로 매니페스트와 TF 포즈에 들어가는지
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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
