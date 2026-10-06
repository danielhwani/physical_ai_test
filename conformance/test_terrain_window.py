"""지형 창(로봇을 따라다니는 MuJoCo heightfield) 시험.

    python conformance/test_terrain_window.py

1. 평지에서 창 모드 = 세계 지도 전체 모드 (창을 여러 번 옮겨도 궤적이 부동소수점 수준으로 같다)
   - 지형 geom을 월드에 두고 위치만 바꾸면, MuJoCo가 모델 생성 때 계산한 충돌 경계 상자가 남아
     처음 창 밖으로 나간 발의 접촉을 놓쳤다 (8.92초에 발 접촉 누락, 확인함). mocap 바디로 옮겨 해결.
2. 창을 옮긴 직후 MuJoCo 지형 데이터와 위치 = 세계 지도의 창 영역 (요철 지형)
3. 창 밖 세계 지도로 걸어가도 넘어지지 않는다 (창 크기보다 먼 거리 보행)
"""
import contextlib
import copy
import io
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.runner import Simulation  # noqa: E402


def _load(name):
    return yaml.safe_load((ROOT / f"scenarios/{name}.yaml").read_text())


def _windowed(scn, half=3.0):
    scn = copy.deepcopy(scn)
    scn["terrain"]["window"] = [half, half]
    return scn


def _run(scn, on_step=None, seconds=None):
    sim = Simulation(scn)
    traj, shifts, last = [], 0, (sim.terrain.wr0, sim.terrain.wc0)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < (seconds or scn["duration"]):
                _, _, r, p = sim.step_control()
                traj.append(sim.data.qpos[:3].copy())
                if (sim.terrain.wr0, sim.terrain.wc0) != last:
                    shifts, last = shifts + 1, (sim.terrain.wr0, sim.terrain.wc0)
                    if on_step:
                        on_step(sim)
                assert not sim.fallen(r, p), f"넘어짐 t={sim.data.time:.2f}"
        return np.array(traj), shifts
    finally:
        sim.close()


def test_window_matches_full_world_on_flat():
    flat = _load("flat_trot")
    A, _ = _run(flat)
    B, shifts = _run(_windowed(flat))
    assert shifts >= 2, "창이 옮겨지지 않아 시험이 의미 없음"
    assert np.abs(A - B).max() < 1e-6, f"창 모드 궤적이 다름 (최대 {np.abs(A - B).max():.1e} m)"


def test_heights_after_shift_match_world_map():
    """창을 옮긴 직후: (1) heightfield 데이터 = 세계 지도의 창 영역, (2) 지형 geom 월드 위치 = 창 중심,
    (3) 레이캐스트 높이 = 세계 지도 (보조). mj_ray는 요철 heightfield에서 창과 무관하게 약 2%가 표면을
    빠져나가 바닥면에 맞으므로(창 없는 모드에서도 확인) 95% 일치를 기준으로 한다."""
    rng, gid = np.random.default_rng(0), np.zeros(1, np.int32)
    stats = {"data": 0.0, "pose": 0.0, "rays": 0, "ray_ok": 0}

    def check(sim):
        ter, m = sim.terrain, sim.model
        mujoco.mj_forward(m, sim.data)
        adr = m.hfield_adr[sim.hfield_id]
        data = m.hfield_data[adr:adr + ter.wnrow * ter.wncol].reshape(ter.wnrow, ter.wncol)
        heights = data * (ter.z_max - ter.z_min) + ter.z_min
        window = ter.applied[ter.wr0:ter.wr0 + ter.wnrow, ter.wc0:ter.wc0 + ter.wncol]
        stats["data"] = max(stats["data"], float(np.abs(heights - window).max()))
        expect = np.array([*ter.window_center, ter.z_min])
        stats["pose"] = max(stats["pose"], float(np.abs(sim.data.geom_xpos[sim.terrain_geom] - expect).max()))
        for _ in range(200):
            x, y = sim.data.qpos[:2] + rng.uniform(-2.0, 2.0, 2)
            c, r = int(round((x + ter.half_x) / ter.res)), int(round((y + ter.half_y) / ter.res))
            x, y = ter.xs[c] + 1e-6, ter.ys[r] + 1e-6
            dist = mujoco.mj_ray(m, sim.data, np.array([x, y, 2.0]), np.array([0, 0, -1.0]), None, 1, sim.base_id, gid)
            if gid[0] == sim.terrain_geom:
                stats["rays"] += 1
                stats["ray_ok"] += abs((2.0 - dist) - ter.applied[r, c]) < 1e-4

    _, shifts = _run(_windowed(_load("rough_rut")), check, seconds=12.0)
    assert shifts >= 2 and stats["rays"] > 100
    assert stats["data"] < 1e-6, f"창 데이터가 세계 지도와 다름 ({stats['data']:.2e} m)"
    assert stats["pose"] < 1e-9, f"지형 geom 위치가 창 중심과 다름 ({stats['pose']:.2e} m)"
    assert stats["ray_ok"] / stats["rays"] > 0.95, f"레이 일치율 {stats['ray_ok'] / stats['rays']:.1%}"


def test_walk_beyond_initial_window():
    scn = _windowed(_load("flat_trot"), half=1.5)       # 작은 창: 처음 창(±1.5 m) 밖으로 걸어 나간다
    scn["duration"] = 20.0
    traj, shifts = _run(scn)
    reach = np.abs(traj[:, :2]).max()                      # 실행 중 가장 멀리 간 거리 (축별)
    assert reach > 2.5 and shifts >= 3, (reach, shifts)     # 처음 창(±1.5 m) 밖까지 넘어지지 않고 걸었다


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
