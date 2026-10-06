"""지형 인지 보행 시험 (LiDAR 높이 지도 -> 디딜 곳, 발 높이).

    python conformance/test_perception.py

1. 높이 지도: 점을 넣은 칸 높이, 지도를 옮겨도 남은 칸 값이 그대로
2. 디딜 곳: 바퀴 자국 벽 근처면 벽에서 멀어지는 쪽으로 옮기고, 자국 바닥 가운데는 그대로 디딘다 (깊이는 벌점 아님)
3. LiDAR로 만든 지도가 참 지형과 맞는다 (시작 자세 기준으로 옮겨 비교, 높이 표류만큼의 일정한 차이는 뺀다)
4. 재현: 알고리즘이 받은 로봇 상태 메시지와 점군(센서 좌표)만 기록해 새 노드에 넣으면 같은 관절 명령이 나온다
   (센서의 참 자세나 지형 참값을 따로 읽지 않는다는 실행 증거)
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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
