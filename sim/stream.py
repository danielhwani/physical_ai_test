"""중립 렌더 스트림 인코더 (시뮬레이터 쪽). 계약: docs/render_interface.md

시뮬레이터는 "무엇이 어디에 있는가"만 내보내고, 그리는 방법(URDF, 마커, 액터)은 렌더러 어댑터가 정한다.
여기서 만드는 것은 모두 JSON 직렬화 가능한 dict이며 전송 수단(ROS2, UDP 등)과 무관하다.
  - 장면 매니페스트: 바디, 외형 메쉬, 지형 규격 (처음 한 번)
  - 지형 패치: 물리엔진에 반영된 높이 격자 중 바뀐 직사각형 영역
  - 상태: 시험 진행 값 (명령, 속도, 거리, 접촉, 이벤트, 최종 MOP)
바디 포즈는 표준 TF(/tf)로 보낸다.
"""
import base64
from pathlib import Path

import mujoco
import numpy as np

from .adapters import mj_quat_to_ros, quat_to_yaw

STREAM_VERSION = "1.0"
ROOT = Path(__file__).resolve().parent.parent
MESH_DIR = ROOT / "build/scene_meshes"
FOOT_NAMES = ("FL", "FR", "RL", "RR")


def export_meshes(model, mesh_dir=MESH_DIR):
    """MuJoCo가 컴파일한 메쉬를 OBJ로 내보낸다 (정점은 geom 좌표계).
    원본 OBJ는 MuJoCo의 메쉬 재중심화 때문에 geom 좌표계와 다르므로 쓰지 않는다."""
    mesh_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    for mid in range(model.nmesh):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
        path = mesh_dir / f"{name}.obj"
        if not path.exists():
            va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
            lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in model.mesh_vert[va:va + vn]]
            lines += [f"f {i + 1} {j + 1} {k + 1}" for i, j, k in model.mesh_face[fa:fa + fn]]
            path.write_text("\n".join(lines) + "\n")
        paths[mid] = path.relative_to(ROOT).as_posix()
    return paths


COSIM_ID_BASE = 1000     # 다른 물리엔진 바디의 id 시작값 (MuJoCo 바디 id와 겹치지 않게)


def build_manifest(model, terrain, cosim=None, physics_source="MuJoCo"):
    """cosim: 다른 물리엔진 연결(ChronoLink 등). 그 바디들도 같은 장면에 넣되 physics_source로 구분한다."""
    name = lambda t, i: mujoco.mj_id2name(model, t, i)  # noqa: E731
    meshes = export_meshes(model)
    bodies, visuals = [], []
    for b in range(1, model.nbody):
        parent = model.body_parentid[b]
        bodies.append({"id": b, "name": name(mujoco.mjtObj.mjOBJ_BODY, b),
                       "parent": "world" if parent == 0 else name(mujoco.mjtObj.mjOBJ_BODY, parent),
                       "physics_source": physics_source})
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_bodyid[g] == 0:
            continue
        rgba = model.mat_rgba[model.geom_matid[g]] if model.geom_matid[g] >= 0 else model.geom_rgba[g]
        visuals.append({"body": name(mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]),
                        "mesh": meshes[model.geom_dataid[g]],
                        "pos": model.geom_pos[g].tolist(), "quat": mj_quat_to_ros(model.geom_quat[g]).tolist(),
                        "rgba": [float(c) for c in rgba]})
    for k, b in enumerate(cosim.manifest_bodies() if cosim is not None else []):
        bodies.append({"id": COSIM_ID_BASE + k, "name": b["name"], "parent": "world",
                       "physics_source": b["physics_source"]})
        visuals += [dict(v, body=b["name"]) for v in b["visuals"]]
    return {
        "stream_version": STREAM_VERSION,
        "frame": "world",
        "conventions": {"coordinates": "REP-103 (x forward, y left, z up, right-handed)", "units": "m",
                        "quaternion": "x,y,z,w"},
        "asset_root": str(ROOT),
        "bodies": bodies,
        "visuals": visuals,
        "terrain": {"half_size": [terrain.half_x, terrain.half_y], "nrow": terrain.nrow, "ncol": terrain.ncol,
                    "z_range": [terrain.z_min, terrain.z_max],
                    "layout": "row-major; row i -> y = -half_y + i*dy, col j -> x = -half_x + j*dx"},
    }


def encode_heights(a):
    return base64.b64encode(np.ascontiguousarray(a, dtype="<f4").tobytes()).decode("ascii")


class TerrainPatchStream:
    """물리엔진에 반영된 지형(applied)의 변경분을 직사각형 패치로 만든다."""

    def __init__(self, terrain):
        self.terrain = terrain
        self.sent = None
        self.version = None

    def _patch(self, r0, r1, c0, c1):
        block = self.terrain.applied[r0:r1, c0:c1]
        return {"version": self.terrain.applied_version, "r0": r0, "c0": c0,
                "nrow": r1 - r0, "ncol": c1 - c0, "encoding": "base64-float32-le", "heights": encode_heights(block)}

    def full(self):
        self.sent = self.terrain.applied.copy()
        self.version = self.terrain.applied_version
        return self._patch(0, self.terrain.nrow, 0, self.terrain.ncol)

    def update(self):
        """반영 버전이 바뀌었으면 바뀐 셀을 감싸는 최소 직사각형 패치, 아니면 None."""
        if self.version == self.terrain.applied_version:
            return None
        self.version = self.terrain.applied_version
        diff = self.terrain.applied != self.sent
        if not diff.any():
            return None
        rows, cols = np.where(diff.any(1))[0], np.where(diff.any(0))[0]
        r0, r1, c0, c1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
        self.sent[r0:r1, c0:c1] = self.terrain.applied[r0:r1, c0:c1]
        return self._patch(int(r0), int(r1), int(c0), int(c1))


def build_status(sim, start_xy, energy, late, final_mop=None):
    """시험 상태 값. 표시 형식(문구, 색)은 렌더러 어댑터가 정한다."""
    d = sim.data
    R = d.xmat[sim.base_id].reshape(3, 3)
    v_now = float((R.T @ d.qvel[:3])[0])
    # 보행 주기 안에서 순간 속도가 출렁이므로 약 0.5 s 지수 평균 (상태 갱신 주기 0.1 s 기준)
    sim.display_speed = v_now if getattr(sim, "display_speed", None) is None else \
        sim.display_speed + 0.2 * (v_now - sim.display_speed)
    dist = float(np.linalg.norm(d.qpos[:2] - start_xy))
    cot = energy / (sim.total_mass * 9.81 * dist) if dist > 0.05 else None
    ctrl, _, _ = sim.controller_desc.partition(":")
    return {
        "t": float(d.time),
        "controller": ctrl, "variant": sim.variant,
        "command": {"vx": float(sim.clock.cmd_f[0]), "yaw_rate": float(sim.clock.cmd_f[1])},
        "speed_avg": sim.display_speed, "distance": dist, "cot": cot,
        "base_pos": d.qpos[:3].tolist(), "yaw": float(quat_to_yaw(d.qpos[3:7])),
        "feet": {n: {"pos": d.geom_xpos[g].tolist(), "contact": bool(c)}
                 for n, g, c in zip(FOOT_NAMES, sim.foot_ids, sim.foot_contacts())},
        "late_steps": late,
        "vehicles": [] if sim.cosim is None else [{
            "name": "hmmwv", "pos": sim.cosim.vehicle["pos"], "speed": sim.cosim.vehicle["speed"],
            "distance": float(np.linalg.norm(np.array(sim.cosim.vehicle["pos"][:2]) - d.qpos[:2]))}],
        "event": {"t": sim.last_event[0], "desc": sim.last_event[1]} if sim.last_event else None,
        "final_mop": final_mop,
    }
