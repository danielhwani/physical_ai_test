"""센서 렌더러: 중립 렌더 스트림만 받아 센서 출력을 만든다 (문서 §8: 레이캐스트는 렌더러 쪽).

시뮬레이터 내부를 읽지 않는다 (sim/ import 금지, 시험으로 확인). 입력은 스트림 메시지뿐이다:
  매니페스트(바디, 외형, 지형 규격, entity), 지형 패치, 바디 포즈(x,y,z + 쿼터니언 x,y,z,w), 시뮬레이션 시각.
같은 클래스를 두 방식으로 쓴다.
  - 같은 프로세스: 러너가 sim.stream.StreamTap으로 만든 메시지를 직접 건넨다 (결정적, 시험·폐루프용)
  - 별도 프로세스: viz/sensor_node.py가 ROS2 토픽으로 받는다 (실제 배치 형태, 결정성 없음)
장면 기하는 MuJoCo 그리기 전용 모델(build_render_model)로 만들고, MuJoCo는 레이캐스트 계산기로만 쓴다.
"""

import mujoco
import numpy as np

from sensors.lidar import Lidar, LidarSpec

from .mujoco_adapter import apply_poses, build_render_model, write_terrain
from .stream_decode import TerrainGrid, check_manifest, quat_to_matrix

TERRAIN, OTHER, MISS = 0, 1, -1
RAY_GROUPS = np.array([1, 1, 0, 0, 0, 0], np.uint8)     # 지형(0)과 다른 물체(1). 센서를 단 개체 자신(2)은 뺀다


class SensorRenderer:
    def __init__(self, manifest, lidar_specs, terrain_half=8.0):
        """lidar_specs: {이름: 명세 dict} (specs/go2_sensors.yaml의 lidars 항목)."""
        self.manifest = check_manifest(manifest)
        self.lidars = []
        for name, d in lidar_specs.items():
            lidar = Lidar(LidarSpec.from_dict(name, d))
            lidar.next_t = 0.0
            self.lidars.append(lidar)
        entity_of = {b["name"]: b.get("entity") for b in self.manifest["bodies"]}
        # 센서를 단 개체(예: go2)의 바디는 레이캐스트에서 뺀다 (자기 몸)
        owners = {entity_of[l.spec.parent] for l in self.lidars}
        self_bodies = {n for n, e in entity_of.items() if e in owners}
        self.model, self.mocap, self.hid, self.window = build_render_model(
            self.manifest, window_half=terrain_half, max_cells=0, self_bodies=self_bodies)
        self.data = mujoco.MjData(self.model)
        self.terrain_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
        self.grid = TerrainGrid(self.manifest["terrain"])
        self.z_range = self.manifest["terrain"]["z_range"]
        self.poses, self.terrain_dirty = {}, True
        self.last_geomid = np.zeros(0, np.int32)

    # ---- 스트림 입력 ----
    def apply_patch(self, patch):
        self.grid.apply(patch)
        self.terrain_dirty = True

    def set_poses(self, poses):
        """{바디 이름: (pos, quat_xyzw)} (TF와 같은 내용)."""
        self.poses.update(poses)

    # ---- 센서 출력 ----
    def due(self, t):
        return [l for l in self.lidars if t >= l.next_t - 1e-9]

    def render(self, t):
        """시각 t에 주기가 된 센서를 스캔한다. 반환: [(이름, 스캔 dict)]. 스캔 dict에 t가 들어간다."""
        due = self.due(t)
        if not due:
            return []
        self._update_scene(due)
        out = []
        for l in due:
            p, q = self.poses[l.spec.parent]
            scan = l.scan(self.raycast, quat_to_matrix(q), np.asarray(p, dtype=float))
            scan["t"] = t
            l.next_t += l.period
            out.append((l.spec.name, scan))
        return out

    def _update_scene(self, due):
        p_parent = np.asarray(self.poses[due[0].spec.parent][0], dtype=float)
        moved = self.window.recenter(*p_parent[:2])        # 지형 창은 센서를 단 바디를 따라간다
        if moved:
            self.data.mocap_pos[self.window.mocap] = [*self.window.center, 0.0]
        if moved or self.terrain_dirty:
            write_terrain(self.model, self.hid, self.grid, self.z_range, self.window)
            self.terrain_dirty = False
        apply_poses(self.model, self.data, self.mocap, self.poses)

    def raycast(self, origin, dirs, max_range):
        """sensors.lidar의 raycast 규약: (dist(N,) 맞지 않으면 inf, label(N,))."""
        n = len(dirs)
        geomid, dist = np.zeros(n, np.int32), np.zeros(n)
        mujoco.mj_multiRay(self.model, self.data, np.asarray(origin, float), np.ascontiguousarray(dirs, float).ravel(),
                           RAY_GROUPS, 1, -1, geomid, dist, None, n, max_range)
        self.last_geomid = geomid                     # 진단/시험용
        miss = geomid < 0
        dist[miss] = np.inf
        return dist, np.where(miss, MISS, np.where(geomid == self.terrain_geom, TERRAIN, OTHER))
