"""LiDAR 센서 시험 (문서 §8).

    python conformance/test_lidar.py

1. 센서 모델(sensors/)은 엔진·ROS와 무관하고, 센서 렌더러(viz/sensor_renderer.py)는 시뮬레이터(sim/)를 모른다
   (스트림 메시지만 받음). 기록한 스트림 메시지만으로 같은 점군이 재현된다
2. 잡음 없는 LiDAR 점이 지형 표면 위에 있다 (heightfield 칸의 삼각형 면 기준 mm 정확도, 요철 + 바퀴 자국)
3. 다른 물리엔진의 물체(Chrono HMMWV)도 보인다 (중립 스트림 장면에 쏘므로)
4. 자기 몸에는 맞지 않는다
5. 주기, 결정성, 잡음 통계 (거리 잡음 표준편차 = 명세)
"""
import contextlib
import io
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sensors.lidar import Lidar, LidarSpec, to_world  # noqa: E402
from viz.sensor_renderer import SensorRenderer  # noqa: E402
from viz.stream_decode import quat_to_matrix  # noqa: E402
from sim.runner import Simulation  # noqa: E402

DEFS = yaml.safe_load((ROOT / "specs/go2_sensors.yaml").read_text())["lidars"]


def _load(name):
    return yaml.safe_load((ROOT / f"scenarios/{name}.yaml").read_text())


def _run(scn, seconds, sensors=("front_lidar",), each=None):
    sim = Simulation(scn, sensors=list(sensors))
    scans = []
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < seconds:
                sim.step_control()
                for n in sim.new_scans:
                    scans.append(sim.scans[n])
                    if each:
                        each(sim, sim.scans[n])
        return sim, scans
    finally:
        sim.close()


def _surface_error(P, ter):
    """점과 heightfield 표면의 높이 차. 칸을 나누는 대각선 방향 두 가지 중 작은 쪽 (MuJoCo 삼각형 분할)."""
    fx, fy = (P[:, 0] + ter.half_x) / ter.res, (P[:, 1] + ter.half_y) / ter.res
    c, r = fx.astype(int), fy.astype(int)
    u, v = fx - c, fy - r
    H = ter.applied
    h00, h10, h01, h11 = H[r, c], H[r, c + 1], H[r + 1, c], H[r + 1, c + 1]
    a = np.where(u >= v, h00 + u * (h10 - h00) + v * (h11 - h10), h00 + v * (h01 - h00) + u * (h11 - h01))
    b = np.where(u + v <= 1, h00 + u * (h10 - h00) + v * (h01 - h00), h11 + (1 - u) * (h01 - h11) + (1 - v) * (h10 - h11))
    return np.minimum(np.abs(P[:, 2] - a), np.abs(P[:, 2] - b))


def _imports(module):
    code = (f"import sys; import {module}; "
            "print(sorted({m.split('.')[0] for m in sys.modules} & {'mujoco', 'sim', 'cosim', 'viz', 'rclpy', 'pychrono'}))")
    return subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()


def test_sensor_model_is_engine_independent():
    assert _imports("sensors.lidar") == "[]"
    assert _imports("viz.sensor_renderer") == "['mujoco', 'viz']", "센서 렌더러가 시뮬레이터(sim/) 등을 불러옴"


def test_replay_from_stream_messages_only():
    """같은 프로세스 실행 때 센서 렌더러가 받은 스트림 메시지만 JSON으로 기록 -> 새 센서 렌더러에 다시 넣으면
    같은 점군이 나와야 한다 (시뮬레이터 내부를 읽지 않는다는 것의 실행 증거, ROS 전송과 같은 직렬화)."""
    log, scans = [], []
    sim = Simulation(_load("vehicle_crossing"), sensors=["front_lidar"])
    r = sim.sensor_renderer
    orig_patch, orig_poses, orig_render = r.apply_patch, r.set_poses, r.render
    r.apply_patch = lambda p: (log.append(("patch", json.dumps(p))), orig_patch(p))[1]
    r.set_poses = lambda ps: (log.append(("poses", json.dumps(ps))), orig_poses(ps))[1]
    r.render = lambda t: (log.append(("render", t)), orig_render(t))[1]
    manifest = json.dumps(sim.tap.manifest)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < 4.0:
                sim.step_control()
                scans += [sim.scans[n] for n in sim.new_scans]
    finally:
        sim.close()
    replay = SensorRenderer(json.loads(manifest), {"front_lidar": DEFS["front_lidar"]})
    got = []
    for kind, arg in log:
        if kind == "patch":
            replay.apply_patch(json.loads(arg))
        elif kind == "poses":
            replay.set_poses(json.loads(arg))
        else:
            got += [s for _, s in replay.render(arg)]
    assert len(got) == len(scans) > 30
    assert all(np.array_equal(a["points"], b["points"]) for a, b in zip(got, scans)), "스트림만으로 재현되지 않음"


def test_points_lie_on_terrain_surface():
    worst, n = [0.0], [0]

    def check(sim, scan):
        clean = Lidar(LidarSpec.from_dict("clean", {**DEFS["front_lidar"], "noise_std": 0.0}))
        r = sim.sensor_renderer                              # 방금 스캔한 장면 (스트림으로 재구성한 것)
        pos, quat = r.poses[clean.spec.parent]
        s = clean.scan(r.raycast, quat_to_matrix(quat), np.asarray(pos))
        P = to_world(s)[s["label"] == 0]
        err = _surface_error(P, sim.terrain)
        worst[0], n[0] = max(worst[0], float(err.max())), n[0] + len(P)

    scn = _load("rough_rut")
    scn["duration"] = 6.0                       # 4초에 생기는 바퀴 자국까지 포함
    _run(scn, 6.0, each=check)
    assert n[0] > 50_000 and worst[0] < 1e-3, f"지형 점 높이 오차 최대 {worst[0] * 1000:.2f} mm ({n[0]}점)"


def test_sees_other_engine_bodies_but_not_itself():
    seen, self_hits = [0], [0]

    def check(sim, scan):
        P = to_world(scan)
        other = scan["label"] == 1
        if other.any():
            vpos = np.array(sim.cosim.vehicle["pos"])
            seen[0] += int((np.linalg.norm(P[other, :2] - vpos[:2], axis=1) < 3.0).sum())   # HMMWV 근처 점
        # 장착 위치가 머리 메쉬 안쪽이라 몸에 맞으면 거리가 최소 거리보다 짧아 점으로 남지 않는다.
        # 그래서 점이 아니라 맞은 geom을 직접 본다: 로봇 몸(그룹 2)은 하나도 없어야 한다
        r = sim.sensor_renderer
        g = r.last_geomid
        self_hits[0] += int((r.model.geom_group[g[g >= 0]] == 2).sum())

    _run(_load("vehicle_crossing"), 4.5, each=check)            # 3.4초쯤 HMMWV가 2 m까지 접근
    assert self_hits[0] == 0, f"자기 몸에 맞은 점 {self_hits[0]}개"
    assert seen[0] > 100, f"HMMWV 점 {seen[0]}개"


def test_rate_determinism_and_noise():
    _, a = _run(_load("flat_trot"), 3.0)
    _, b = _run(_load("flat_trot"), 3.0)
    assert len(a) == int(3.0 * DEFS["front_lidar"]["rate_hz"]) + 1, len(a)
    assert all(np.array_equal(x["points"], y["points"]) for x, y in zip(a, b)), "같은 실행인데 점군이 다름"
    resid = np.concatenate([np.linalg.norm(s["points"], axis=1) - s["range_true"] for s in a])
    sigma = DEFS["front_lidar"]["noise_std"]
    assert abs(resid.std() - sigma) < 0.1 * sigma and abs(resid.mean()) < 0.1 * sigma, (resid.mean(), resid.std())


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
