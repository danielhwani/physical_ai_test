"""중립 렌더 스트림 계약 시험 (docs/render_interface.md).

    python conformance/test_render_stream.py

렌더러는 스트림만 보고 장면을 그린다. 그래서 "스트림만으로 재구성한 장면 = 시뮬레이터가 계산한 장면"이어야 한다.
1. 렌더러 패키지(viz/)가 물리엔진(MuJoCo)에 의존하지 않는다
2. 외형: 매니페스트 + 바디 포즈(TF와 같은 x,y,z,w)로 놓은 메쉬 정점 = MuJoCo의 월드 geom 정점
3. 지형: 높이 패치를 차례로 적용한 격자 = 물리엔진에 반영된 지형 (실행 중 변형 포함)
4. 상태/매니페스트가 JSON으로 왕복되고, RViz 어댑터가 매니페스트로 URDF를 만든다
"""
import contextlib
import io
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.adapters import mj_quat_to_ros  # noqa: E402
from sim.runner import Simulation  # noqa: E402
from sim.stream import TerrainPatchStream, build_manifest, build_status  # noqa: E402
from viz.stream_decode import TerrainGrid, check_manifest, load_obj_vertices, quat_to_matrix  # noqa: E402

SCENARIO = ROOT / "scenarios/rough_rut.yaml"


def _roundtrip(obj):
    return json.loads(json.dumps(obj))         # 실제 전송과 같이 JSON 직렬화를 거친다


def test_viz_does_not_depend_on_physics():
    code = ("import sys; import viz.stream_decode, viz.rviz_adapter; "
            "bad = [m for m in sys.modules if m.split('.')[0] in ('mujoco', 'sim')]; print(bad)")
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "[]", f"viz/가 물리엔진 쪽 모듈을 불러옴: {out.stdout}"


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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
