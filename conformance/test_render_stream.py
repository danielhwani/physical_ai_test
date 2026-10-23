"""중립 렌더 스트림 계약 시험 (docs/render_interface.md).

    python conformance/test_render_stream.py

렌더러는 스트림만 보고 장면을 그린다. 그래서 "스트림만으로 재구성한 장면 = 시뮬레이터가 계산한 장면"이어야 한다.
1. 렌더러 패키지(viz/)가 물리 시뮬레이션(sim/)에 의존하지 않는다. RViz 쪽은 MuJoCo도 쓰지 않고,
   MuJoCo 렌더러 어댑터는 MuJoCo를 그리기 라이브러리로만 쓴다
2. 외형: 매니페스트 + 바디 포즈(TF와 같은 x,y,z,w)로 놓은 메쉬 정점 = MuJoCo의 월드 geom 정점
3. 지형: 높이 패치를 차례로 적용한 격자 = 물리엔진에 반영된 지형 (실행 중 변형 포함)
4. 상태/매니페스트가 JSON으로 왕복되고, RViz 어댑터가 매니페스트로 URDF를 만든다
5. MuJoCo 렌더러 어댑터: 매니페스트로 만든 그리기 전용 모델 + 포즈 = 시뮬레이터 외형, 지형도 같다
"""
import contextlib
import io
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.adapters import mj_quat_to_ros  # noqa: E402
from sim.runner import Simulation  # noqa: E402
from sim.stream import TerrainPatchStream, build_manifest, build_status  # noqa: E402
from viz.stream_decode import TerrainGrid, check_manifest, load_obj_vertices, quat_to_matrix  # noqa: E402

SCENARIO = ROOT / "scenarios/rough_rut.yaml"


def _roundtrip(obj):
    return json.loads(json.dumps(obj))         # 실제 전송과 같이 JSON 직렬화를 거친다


def _imported_roots(modules):
    code = (f"import sys; import {modules}; "
            "print(sorted({m.split('.')[0] for m in sys.modules} & {'mujoco', 'sim', 'cosim'}))")
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
    return out.stdout.strip()


def test_viz_does_not_depend_on_physics():
    assert _imported_roots("viz.stream_decode, viz.rviz_adapter") == "[]", "RViz 경로가 MuJoCo/sim을 불러옴"
    assert _imported_roots("viz.godot_bridge") == "[]", "Godot 중계기가 MuJoCo/sim을 불러옴"
    assert _imported_roots("viz.mujoco_adapter") == "['mujoco']", "MuJoCo 렌더러가 물리 시뮬레이션(sim/)을 불러옴"


def test_visuals_reconstruct_simulator_geometry():
    sim = Simulation(SCENARIO)
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(150):                    # 걷는 중의 임의 자세
            sim.step_control()
    m, d = sim.model, sim.data
    manifest = check_manifest(_roundtrip(build_manifest(m, sim.terrain)))
    body_id = {b["name"]: b["id"] for b in manifest["bodies"]}
    mesh_geoms = [g for g in range(m.ngeom) if m.geom_type[g] == 7 and m.geom_bodyid[g] != 0]   # 7 = mesh
    assert len(mesh_geoms) == len(manifest["visuals"])
    worst = 0.0
    for g, v in zip(mesh_geoms, manifest["visuals"]):
        # 렌더러가 하는 일: 바디 포즈(TF) ∘ 외형 오프셋(매니페스트)으로 OBJ 정점을 월드에 놓는다
        b = body_id[v["body"]]
        Rb, pb = quat_to_matrix(mj_quat_to_ros(d.xquat[b])), d.xpos[b]
        Rv, pv = quat_to_matrix(v["quat"]), np.array(v["pos"])
        verts = load_obj_vertices(ROOT / v["mesh"])
        rendered = (Rb @ (Rv @ verts.T + pv[:, None])).T + pb
        # 시뮬레이터 기준: MuJoCo의 월드 geom 포즈로 놓은 컴파일된 메쉬 정점
        mid = m.geom_dataid[g]
        ref = (d.geom_xmat[g].reshape(3, 3) @ m.mesh_vert[m.mesh_vertadr[mid]:][:m.mesh_vertnum[mid]].T).T + d.geom_xpos[g]
        worst = max(worst, float(np.abs(rendered - ref).max()))
    assert worst < 1e-5, f"외형 재구성 오차 {worst:.2e} m"


def test_terrain_patches_reconstruct_physics_terrain():
    sim = Simulation(SCENARIO)
    stream = TerrainPatchStream(sim.terrain)
    manifest = _roundtrip(build_manifest(sim.model, sim.terrain))
    grid = TerrainGrid(manifest["terrain"])
    grid.apply(_roundtrip(stream.full()))
    n_patches = 0
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < 8.0:              # 4초에 바퀴 자국 생성 + 발 근처 보류 후 반영
            sim.step_control()
            p = stream.update()
            if p:
                grid.apply(_roundtrip(p))
                n_patches += 1
            assert np.allclose(grid.heights, sim.terrain.applied, atol=1e-6), f"t={sim.data.time:.2f}"
    assert n_patches >= 1, "바퀴 자국 패치가 나오지 않음"
    assert grid.version == sim.terrain.applied_version


def test_status_and_urdf():
    from viz.rviz_adapter import build_urdf, status_lines
    sim = Simulation(SCENARIO)
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(100):
            sim.step_control()
    s = _roundtrip(build_status(sim, np.zeros(2), 10.0, 0))
    lines, alert = status_lines(s)
    assert lines and not alert and all(" " not in ln for ln in lines)
    manifest = _roundtrip(build_manifest(sim.model, sim.terrain))
    urdf = ET.fromstring(build_urdf(manifest))
    links = {lk.get("name") for lk in urdf.iter("link")}
    assert links == {"world"} | {b["name"] for b in manifest["bodies"]}
    for mesh in urdf.iter("mesh"):
        assert Path(mesh.get("filename").removeprefix("file://")).exists()


def test_mujoco_render_model_matches_simulator():
    import mujoco
    from viz.mujoco_adapter import apply_poses, build_render_model, write_terrain
    sim = Simulation(SCENARIO)
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(150):
            sim.step_control()
    m0, d0 = sim.model, sim.data
    manifest = _roundtrip(build_manifest(m0, sim.terrain))
    model, mocap, hid, win = build_render_model(manifest)
    data = mujoco.MjData(model)
    poses = {b["name"]: (d0.xpos[b["id"]].tolist(), mj_quat_to_ros(d0.xquat[b["id"]]).tolist()) for b in manifest["bodies"]}
    apply_poses(model, data, mocap, poses)

    def world_verts(m, d, g):
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        return (d.geom_xmat[g].reshape(3, 3) @ v.T).T + d.geom_xpos[g]

    src = [g for g in range(m0.ngeom) if m0.geom_type[g] == 7 and m0.geom_bodyid[g] != 0]
    dst = [g for g in range(model.ngeom) if model.geom_type[g] == 7]
    assert len(src) == len(dst)
    worst = 0.0
    for gs, gd in zip(src, dst):
        a, b = world_verts(m0, d0, gs), world_verts(model, data, gd)
        # 다시 컴파일하면 정점 순서가 바뀔 수 있으므로 중심과 축별 분포로 비교
        worst = max(worst, np.abs(a.mean(0) - b.mean(0)).max(), np.abs(np.sort(a, 0) - np.sort(b, 0)).max())
    assert worst < 1e-5, f"그리기 모델 외형 오차 {worst:.2e} m"

    grid = TerrainGrid(manifest["terrain"])
    grid.apply(_roundtrip(TerrainPatchStream(sim.terrain).full()))
    write_terrain(model, hid, grid, manifest["terrain"]["z_range"], win)
    zmin, zmax = manifest["terrain"]["z_range"]
    adr = model.hfield_adr[hid]
    h = model.hfield_data[adr:adr + grid.heights.size].reshape(grid.heights.shape) * (zmax - zmin) + zmin
    assert np.allclose(h, sim.terrain.applied, atol=1e-6)


def test_mujoco_overlay_path_segments():
    import mujoco
    from viz.mujoco_adapter import MAX_PATH, draw_overlays
    scn = mujoco.MjvScene(mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>"), maxgeom=10000)
    status = {"feet": {"FL": {"pos": [0, 0, 0], "contact": True}}, "base_pos": [1, 0, 0.3], "yaw": 0.0,
              "command": {"vx": 0.3}}
    for n in (2, 50, 1000):
        draw_overlays(scn, status, [np.array([x, 0, 0.3]) for x in np.linspace(0, 2, n)])
        caps = [scn.geoms[i] for i in range(scn.ngeom) if scn.geoms[i].type == mujoco.mjtGeom.mjGEOM_CAPSULE]
        assert len(caps) == min(n - 1, MAX_PATH) and min(g.size[2] for g in caps) > 0, n   # 길이 0 선분 금지


def test_soil_regions_on_screen():
    """무른 흙 지역(foot_soil.regions)은 매니페스트 soil_regions와 상태 soil로 나가고, 두 렌더러가 그린다 (화면 전용)."""
    import mujoco
    from sim.stream import build_status
    from viz.mujoco_adapter import draw_overlays, status_text
    from viz.rviz_adapter import status_lines
    scn = yaml.safe_load((ROOT / "scenarios/soil_strip.yaml").read_text())
    sim = Simulation(scn)
    regions = sim.foot_soil.display_regions()
    manifest = check_manifest(_roundtrip(build_manifest(sim.model, sim.terrain, soil_regions=regions)))
    assert manifest["soil_regions"] == regions and regions[0]["name"] == "mud_strip" and regions[0]["region"] == [2.0, -10.0, 3.5, 10.0]
    assert regions[0]["rgba"][0] < 0.45                              # 진흙은 기본 지형색(0.55)보다 짙다
    st = build_status(sim, sim.data.qpos[:2].copy(), 0.0, 0)
    assert st["soil"] == "firm"
    sim.data.qpos[0] = 2.5
    assert build_status(sim, sim.data.qpos[:2].copy(), 0.0, 0)["soil"] == "mud_strip"
    assert any("soil:mud_strip" in ln for ln in status_lines(_roundtrip(build_status(sim, sim.data.qpos[:2].copy(), 0.0, 0)))[0])
    assert "soil mud_strip" in status_text(build_status(sim, sim.data.qpos[:2].copy(), 0.0, 0))[1]
    vs = mujoco.MjvScene(mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>"), maxgeom=1000)
    status = {"feet": {}, "base_pos": [2.5, 0, 0.3], "yaw": 0.0, "command": {"vx": 0.3}}
    draw_overlays(vs, status, [], regions)
    caps = [vs.geoms[i] for i in range(vs.ngeom) if vs.geoms[i].type == mujoco.mjtGeom.mjGEOM_CAPSULE]
    assert len(caps) == 4 and min(g.size[2] for g in caps) > 0       # 테두리 네 변 (로봇 근처 ±3 m)
    assert build_manifest(sim.model, sim.terrain)["soil_regions"] == []   # 무른 흙이 없으면 빈 목록


def test_godot_bridge_messages():
    """Godot 중계기: 매니페스트·지형 패치·발자국으로 만든 지형 메시지가 스트림의 지형과 같고, 흙 지역은 그 색, 정보판 문구가 나온다.
    Godot 쪽(godot/main.gd)은 이 메시지로 메시만 만든다."""
    import base64
    import rclpy
    from sim.stream import TerrainPatchStream, build_status
    from std_msgs.msg import String
    from viz.godot_bridge import GodotBridge, status_lines
    scn = yaml.safe_load((ROOT / "scenarios/soil_strip.yaml").read_text())
    sim = Simulation(scn)
    sim.foot_soil.display[200, 240] = 0.05                       # 화면 발자국 깊이 하나 (격자점)
    from sim.stream import TerrainOverlayStream
    over = TerrainOverlayStream(sim.foot_soil.display_depth, sim.foot_soil.display.shape).update()
    rclpy.init()
    try:
        br = GodotBridge(0)                                       # 포트 0: 빈 포트
        br.on_manifest(String(data=json.dumps(build_manifest(sim.model, sim.terrain, soil_regions=sim.foot_soil.display_regions()))))
        br.on_patch(String(data=json.dumps(TerrainPatchStream(sim.terrain).full())))
        br.on_overlay(String(data=json.dumps(over)))
        t = br.terrain_msg()
        H = np.frombuffer(base64.b64decode(t["h"]), "<f4").reshape(t["nrow"], t["ncol"])
        rgb = np.frombuffer(base64.b64decode(t["rgb"]), "<f4").reshape(t["nrow"], t["ncol"], 3)
        xs = t["x0"] + np.arange(t["ncol"]) * t["dx"]
        ys = t["y0"] + np.arange(t["nrow"]) * t["dy"]
        r, c = int(np.argmin(np.abs(ys - sim.terrain.ys[200]))), int(np.argmin(np.abs(xs - sim.terrain.xs[240])))
        assert abs(H[r, c] - (sim.terrain.applied[200, 240] - 0.05)) < 1e-6        # 지형 - 발자국 깊이
        mud = rgb[:, (xs >= 2.0) & (xs <= 3.5)].mean(axis=(0, 1))
        firm = rgb[:, xs < 1.5].mean(axis=(0, 1))
        assert mud[0] < firm[0] - 0.15, (mud, firm)               # 진흙 띠는 짙다
        lines, alert = status_lines(_roundtrip(build_status(sim, sim.data.qpos[:2].copy(), 0.0, 0)))
        assert any(ln.startswith("soil firm") for ln in lines) and not alert
        br.destroy_node()
    finally:
        rclpy.shutdown()


def test_godot_bridge_points():
    """Godot 중계기 점군: 센서 좌표계 PointCloud2 + world -> 센서 TF를 월드 좌표로 바꾸고, 높이 색은 낮을수록 파랑."""
    import mujoco
    from viz.godot_bridge import cloud_to_world
    from viz.ros_msgs import scan_to_msgs
    rng = np.random.default_rng(0)
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, np.array([0.9, 0.1, -0.2, 0.3]) / np.linalg.norm([0.9, 0.1, -0.2, 0.3]))
    R = R.reshape(3, 3)
    p = np.array([1.0, -2.0, 0.3])
    pts = rng.normal(0, 2, (50, 3))
    msg, tf = scan_to_msgs("front_lidar", {"t": 1.0, "points": pts, "ring": np.zeros(50), "label": np.zeros(50), "p": p, "R": R})
    tr, q = tf.transform.translation, tf.transform.rotation
    xyz, rgb = cloud_to_world(msg, {"front_lidar": [tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w]})
    assert np.abs(xyz - (pts @ R.T + p)).max() < 1e-5
    assert rgb[np.argmin(xyz[:, 2])][2] == 1.0 and rgb[np.argmax(xyz[:, 2])][0] == 1.0
    assert cloud_to_world(msg, {}) is None                      # TF를 아직 모르면 보내지 않는다


def test_godot_bridge_footholds():
    """Godot 중계기 디딜 곳: perception_markers가 만든 MarkerArray(odom 좌표)를 world -> odom TF로 옮겨 구와 선분으로 넘긴다."""
    from builtin_interfaces.msg import Time
    from control.ros_viz import perception_markers
    from viz.godot_bridge import markers_to_world
    legs = [{"leg": i, "swing": i % 2 == 0, "chosen": [0.3 * i, 0.1, -0.02], "nominal": [0.3 * i - 0.05, 0.12, -0.02]}
            for i in range(4)]
    snap = {"cells": np.zeros((1, 3)), "h_ref": 0.0, "legs": legs}
    _, feet = perception_markers(snap, Time())
    off = [0.5, -0.2, 0.01]
    spheres, lines = markers_to_world(feet, {"odom": [*off, 0.0, 0.0, 0.0, 1.0]})
    assert len(spheres) == 8 and len(lines) == 4                   # 다리마다 고른 곳 + 원래 자리, 옮긴 선분
    chosen = [s for s in spheres if s[3] >= 0.035]
    for leg, sph in zip(legs, chosen):
        assert np.allclose(sph[:3], np.add(leg["chosen"], off), atol=1e-6)
        assert np.isclose(sph[3], 0.05 if leg["swing"] else 0.035)
    assert np.allclose(lines[0][:6], [*np.add(legs[0]["nominal"], off), *np.add(legs[0]["chosen"], off)], atol=1e-6)
    assert markers_to_world(feet, {}) == ([], [])                  # odom TF를 아직 모르면 보내지 않는다


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
