"""지형 인지 보행 시험 (LiDAR 높이 지도 -> 디딜 곳, 발 높이).

    python conformance/test_perception.py

1. 높이 지도: 점을 넣은 칸 높이, 지도를 옮겨도 남은 칸 값이 그대로
2. 디딜 곳: 바퀴 자국 벽 근처면 벽에서 멀어지는 쪽으로 옮기고, 자국 바닥 가운데는 그대로 디딘다 (깊이는 벌점 아님)
3. LiDAR로 만든 지도가 참 지형과 맞는다 (시작 자세 기준으로 옮겨 비교, 높이 표류만큼의 일정한 차이는 뺀다)
4. 재현: 알고리즘이 받은 로봇 상태 메시지와 점군(센서 좌표)만 기록해 새 노드에 넣으면 같은 관절 명령이 나온다
   (센서의 참 자세나 지형 참값을 따로 읽지 않는다는 실행 증거)
5. 관측 규약 height_scan (specs/go2_control.yaml observation_perceptive): 격자 순서, 평지 = 0, 모르는 칸, 자르기
6. 걷는 동안 알고리즘이 뽑은 height_scan 입력이 참 지형(같은 격자, 참 자세)과 맞는다 -> 강화학습에서 시뮬레이터 지형으로
   뽑는 값과 배치 때 LiDAR 지도에서 뽑는 값이 같은 뜻이다
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
from control.foothold import FootholdPlanner  # noqa: E402
from control.interface import ControlInterface  # noqa: E402
from control.observation import height_scan_grid  # noqa: E402
from control.node import ControllerNode  # noqa: E402
from control.terrain_map import ElevationMap  # noqa: E402
from sim.runner import Simulation, apply_overrides  # noqa: E402

SPEC = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())
LIDARS = yaml.safe_load((ROOT / "specs/go2_sensors.yaml").read_text())["lidars"]
PERCEPTION = ["controller.perception.sensor=front_lidar"]


def _scenario(name, sets=()):
    return apply_overrides(yaml.safe_load((ROOT / f"scenarios/{name}.yaml").read_text()), list(sets))


def _run(sim, seconds):
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < seconds:
            sim.step_control()


def _rut_map(floor=-0.05, x0=1.0, width=0.25):
    m = ElevationMap(half=1.0, res=0.04)
    xs = np.arange(-0.98, 0.98, 0.01)
    X, Y = np.meshgrid(xs, xs)
    Z = np.where((X >= x0 - 1.0) & (X < x0 - 1.0 + width), floor, 0.0)    # 자국: x 0.0~0.25 m
    m.add_points(np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1))
    return m


def test_map_cells_and_recenter():
    m = _rut_map()
    assert abs(m.height(np.array([0.12, 0.0])) + 0.05) < 1e-9 and abs(m.height(np.array([0.5, 0.0]))) < 1e-9
    before = m.height(np.array([0.12, 0.3]))
    m.recenter(np.array([0.8, 0.0]))                   # 0.8 m 옮김 -> 자국 칸은 그대로 남아야 한다
    assert m.origin[0] > -1.0 and abs(m.height(np.array([0.12, 0.3])) - before) < 1e-12
    assert np.isnan(m.height(np.array([1.7, 0.0])))   # 새로 들어온 칸은 모름


def test_foothold_avoids_rut_wall_but_uses_rut_floor():
    m, fp = _rut_map(), FootholdPlanner()
    fwd = np.array([1.0, 0.0])
    near_wall = fp.choose(m, np.array([0.01, 0.0]), fwd)          # 자국 앞 벽 바로 안쪽
    _, wall_cost = fp.evaluate(m, np.array([0.01, 0.0]))
    assert wall_cost > 0.3 and abs(near_wall.shift) >= 0.04, near_wall
    center = fp.choose(m, np.array([0.13, 0.0]), fwd)              # 자국 바닥 가운데
    assert center.shift == 0.0 and center.side == 0.0 and abs(center.height + 0.05) < 1e-9, center


def test_lidar_map_matches_true_terrain():
    sim = Simulation(_scenario("rough_rut", PERCEPTION))
    try:
        x0, y0, z0 = sim.data.qpos[:3]
        _run(sim, 6.0)
        m = sim.controller.terrain.map
        rows, cols = np.nonzero(np.isfinite(m.h))
        xy = m.origin + (np.stack([cols, rows], 1) + 0.5) * m.res          # odom 좌표 칸 중심
        # 시작 방향은 0 (시나리오 시작 자세). odom 원점 = 시작 몸통 위치
        truth = np.array([sim.terrain.height_at(x + x0, y + y0) for x, y in xy]) - z0
        err = m.h[rows, cols] - truth
        near = np.linalg.norm(xy - sim.controller.est.pos[:2], axis=1) < 1.5
        err = err[near]
    finally:
        sim.close()
    assert near.sum() > 200, f"지도 칸이 너무 적음: {near.sum()}"
    spread = np.percentile(err, 90) - np.percentile(err, 10)
    assert spread < 0.03, f"지도 높이 오차 폭(10~90%) {spread * 100:.1f} cm (일정한 차이 {np.median(err) * 100:.1f} cm)"


def test_replay_from_low_state_and_points_only():
    sim = Simulation(_scenario("rough_rut", PERCEPTION))
    log, cmds = [], []
    node = sim.controller
    orig_step, orig_scan, orig_cmd = node.step, node.on_scan, node.set_command

    def step(ls):
        log.append(("state", copy.deepcopy(ls)))
        cmds.append(orig_step(ls))
        return cmds[-1]

    def on_scan(name, t, points):
        log.append(("scan", (name, t, np.array(points, dtype=np.float32))))
        orig_scan(name, t, points)

    def set_command(vx, wz):
        log.append(("cmd", (vx, wz)))
        orig_cmd(vx, wz)
    node.step, node.on_scan, node.set_command = step, on_scan, set_command
    try:
        _run(sim, 4.0)
        scenario = sim.scn
    finally:
        sim.close()
    fresh = ControllerNode(SPEC, scenario["controller"], lidar_defs=LIDARS)
    replay = []
    for kind, item in log:
        if kind == "state":
            replay.append(fresh.step(item))
        elif kind == "scan":
            fresh.on_scan(*item)
        else:
            fresh.set_command(*item)
    assert sum(k == "scan" for k, _ in log) >= 30 and fresh.terrain.scans_used > 0
    assert all(a["q_des"] == b["q_des"] for a, b in zip(replay, cmds)), "로봇 상태 + 점군만으로 재현되지 않음"


def _perceptive_obs():
    return ControlInterface(SPEC, {"observation": SPEC["observation_perceptive"]}).obs


def test_height_scan_contract():
    obs = _perceptive_obs()
    t = obs.term("height_scan")
    grid = height_scan_grid(t)
    assert grid.shape == (t["dim"], 2) and obs.dim == SPEC["observation_perceptive"]["dim"]
    assert tuple(grid[0]) == (-0.30, -0.25) and tuple(grid[1]) == (-0.30, -0.20) and tuple(grid[-1]) == (0.60, 0.25)
    sl = obs.slices()["height_scan"]
    base = (np.r_[0, 0, 0, 1, 0, 0, 0, np.zeros(12)], np.zeros(18), np.zeros(3), np.zeros(12), 0.0)
    flat = np.full(t["dim"], -t["base_height_ref"])                 # 평지에 서 있음 -> 0
    assert np.allclose(obs.compute(*base, height_scan=flat)[sl], 0.0)
    hs = flat.copy(); hs[0], hs[1], hs[2] = np.nan, -t["base_height_ref"] - 0.05, 1.0
    v = obs.compute(*base, height_scan=hs)[sl]
    assert v[0] == t["unknown"] * t["scale"] and abs(v[1] + 0.05 * t["scale"]) < 1e-12 and v[2] == t["clip_m"] * t["scale"]
    assert np.allclose(obs.compute(*base, height_scan=flat)[:47], SPEC_OBS().compute(*base))   # 앞 47개는 기존 규약과 같다


def SPEC_OBS():
    return ControlInterface(SPEC).obs


def test_height_scan_input_matches_true_terrain():
    sim = Simulation(_scenario("rough_rut", PERCEPTION))
    obs = _perceptive_obs()
    grid = height_scan_grid(obs.term("height_scan"))
    errs = []
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < 6.0:
                sim.step_control()
                if sim.data.time > 3.0 and round(sim.data.time / 0.02) % 10 == 0:
                    node = sim.controller
                    est = node.est
                    hs = node.height_scan(obs, est)
                    d = sim.data
                    yaw = np.arctan2(2 * (d.qpos[3] * d.qpos[6] + d.qpos[4] * d.qpos[5]), 1 - 2 * (d.qpos[5] ** 2 + d.qpos[6] ** 2))
                    c, s_ = np.cos(yaw), np.sin(yaw)
                    pts = d.qpos[:2] + grid @ np.array([[c, s_], [-s_, c]])
                    truth = np.array([sim.terrain.height_at(x, y) for x, y in pts]) - d.qpos[2]
                    ok = np.isfinite(hs)
                    errs.append((hs[ok] - truth[ok], ok.mean()))
    finally:
        sim.close()
    err = np.concatenate([e for e, _ in errs])
    known = np.mean([k for _, k in errs])
    spread = np.percentile(err, 90) - np.percentile(err, 10)
    # 앞쪽 LiDAR가 이미 본 곳만 값이 있다. 오차 = 위치 표류(일정한 차이) + 격자 칸 크기 + 거리 잡음
    assert known > 0.7, f"아는 격자점 비율 {known:.2f}"
    assert spread < 0.04, f"height_scan 오차 폭(10~90%) {spread * 100:.1f} cm, 일정한 차이 {np.median(err) * 100:.1f} cm"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
