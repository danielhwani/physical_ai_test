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
OTHER_GROUPS = np.array([0, 1, 0, 0, 0, 0], np.uint8)   # 다른 물체만 (지형은 terrain_raycast가 직접 계산한다)


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
        """sensors.lidar의 raycast 규약: (dist(N,) 맞지 않으면 inf, label(N,)).
        지형은 직접 계산하고 (terrain_raycast), 다른 물체는 지형을 뺀 MuJoCo 레이캐스트로 구해 가까운 쪽을 쓴다."""
        origin = np.asarray(origin, float)
        dirs = np.ascontiguousarray(dirs, float)
        n = len(dirs)
        geomid, dist = np.zeros(n, np.int32), np.zeros(n)
        mujoco.mj_multiRay(self.model, self.data, origin, dirs.ravel(), OTHER_GROUPS, 1, -1, geomid, dist, None, n, max_range)
        dist[geomid < 0] = np.inf
        t_ter = terrain_raycast(origin, dirs, max_range, *self._terrain_surface())
        ter = t_ter < dist
        dist = np.where(ter, t_ter, dist)
        geomid = np.where(ter, self.terrain_geom, geomid)
        self.last_geomid = geomid                     # 진단/시험용
        miss = ~np.isfinite(dist)
        return dist, np.where(miss, MISS, np.where(ter, TERRAIN, OTHER))

    def _terrain_surface(self):
        """레이캐스트 모델의 heightfield 표면: (높이 (nrow, ncol), 창 왼쪽 아래 x, y, 칸 크기). MuJoCo가 쓰는 값 (float32) 그대로."""
        m, w = self.model, self.window
        nr, nc = m.hfield_nrow[self.hid], m.hfield_ncol[self.hid]
        adr = m.hfield_adr[self.hid]
        zmin, zmax = self.z_range
        H = zmin + m.hfield_data[adr:adr + nr * nc].astype(float).reshape(nr, nc) * (zmax - zmin)
        cx, cy = w.center if w.windowed else (0.0, 0.0)
        return H, cx - w.win_half[0], cy - w.win_half[1], w.res


def terrain_raycast(o, D, max_range, H, x0, y0, res):
    """광선과 heightfield (칸마다 삼각형 두 개: (00, 10, 11), (00, 11, 01)) 의 첫 교점 거리. 맞지 않으면 inf.
    광선이 지형 높이 범위 [H 최소, H 최대] 와 창 안에 있는 구간만 칸을 차례로 따라가며 (DDA) 검사한다.
    MuJoCo heightfield 레이캐스트는 광선이 지나는 범위의 칸을 넓게 검사해, 거의 수평인 광선과 위로 가는 광선이 매우 비싸다
    (광선 2904개 한 프레임에 약 100 ms. 이 함수는 그 일부)."""
    n = len(D)
    out = np.full(n, np.inf)
    nr, nc = H.shape
    lo, hi = float(H.min()) - 1e-9, float(H.max()) + 1e-9
    dx, dy, dz = D[:, 0], D[:, 1], D[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        # 높이 범위 안 구간 [ta, tb]
        if lo <= o[2] <= hi:
            ta = np.zeros(n)
            tb = np.where(dz < 0, (lo - o[2]) / dz, np.where(dz > 0, (hi - o[2]) / dz, np.inf))
        elif o[2] > hi:
            ta = np.where(dz < 0, (hi - o[2]) / dz, np.inf)
            tb = np.where(dz < 0, (lo - o[2]) / dz, -np.inf)
        else:
            ta = np.where(dz > 0, (lo - o[2]) / dz, np.inf)
            tb = np.where(dz > 0, (hi - o[2]) / dz, -np.inf)
        # 창 안 구간 (x, y 판)
        x1, y1 = x0 + (nc - 1) * res, y0 + (nr - 1) * res
        for d, a, b, p in ((dx, x0, x1, o[0]), (dy, y0, y1, o[1])):
            t1, t2 = (a - p) / d, (b - p) / d
            inside = (p >= a) & (p <= b)
            ta = np.where(d == 0, np.where(inside, ta, np.inf), np.maximum(ta, np.minimum(t1, t2)))
            tb = np.where(d == 0, tb, np.minimum(tb, np.maximum(t1, t2)))
        ta = np.maximum(ta, 0.0)
        tb = np.minimum(tb, max_range)
        act = np.nonzero(ta <= tb)[0]
        if act.size == 0:
            return out
        dx, dy, dz, ta, tb = dx[act], dy[act], dz[act], ta[act], tb[act]
        # 시작 칸과 다음 칸 경계까지의 거리 (Amanatides-Woo)
        px, py = o[0] + ta * dx, o[1] + ta * dy
        j = np.clip(np.floor((px - x0) / res).astype(np.int64), 0, nc - 2)
        i = np.clip(np.floor((py - y0) / res).astype(np.int64), 0, nr - 2)
        sx, sy = np.where(dx > 0, 1, -1), np.where(dy > 0, 1, -1)
        tdx, tdy = np.where(dx != 0, res / np.abs(dx), np.inf), np.where(dy != 0, res / np.abs(dy), np.inf)
        tmx = np.where(dx > 0, (x0 + (j + 1) * res - o[0]) / dx, np.where(dx < 0, (x0 + j * res - o[0]) / dx, np.inf))
        tmy = np.where(dy > 0, (y0 + (i + 1) * res - o[1]) / dy, np.where(dy < 0, (y0 + i * res - o[1]) / dy, np.inf))
    k = np.arange(act.size)                       # 아직 검사 중인 광선 (act 안의 번호)
    while k.size:
        ii, jj = i[k], j[k]
        cx, cy = x0 + jj * res, y0 + ii * res      # 칸 왼쪽 아래 (지역 좌표로 계산해 반올림 오차를 줄인다)
        h00, h10, h01, h11 = H[ii, jj], H[ii, jj + 1], H[ii + 1, jj], H[ii + 1, jj + 1]
        d = np.stack([dx[k], dy[k], dz[k]], 1)
        orig = np.stack([o[0] - cx, o[1] - cy, np.full(k.size, o[2])], 1)
        p00 = np.stack([np.zeros(k.size), np.zeros(k.size), h00], 1)
        p10 = np.stack([np.full(k.size, res), np.zeros(k.size), h10], 1)
        p01 = np.stack([np.zeros(k.size), np.full(k.size, res), h01], 1)
        p11 = np.stack([np.full(k.size, res), np.full(k.size, res), h11], 1)
        t = np.minimum(_ray_tri(orig, d, p00, p10, p11), _ray_tri(orig, d, p00, p11, p01))
        hit = np.isfinite(t)
        out[act[k[hit]]] = t[hit]
        k = k[~hit]
        if not k.size:
            break
        # 다음 칸
        gx = tmx[k] < tmy[k]
        tnext = np.where(gx, tmx[k], tmy[k])
        j[k] = np.where(gx, j[k] + sx[k], j[k])
        i[k] = np.where(gx, i[k], i[k] + sy[k])
        tmx[k] = np.where(gx, tmx[k] + tdx[k], tmx[k])
        tmy[k] = np.where(gx, tmy[k], tmy[k] + tdy[k])
        go = (tnext <= tb[k]) & (j[k] >= 0) & (j[k] <= nc - 2) & (i[k] >= 0) & (i[k] <= nr - 2)
        k = k[go]
    return out


def _ray_tri(o, d, a, b, c, eps=1e-12):
    """광선 (o, d) 과 삼각형 (a, b, c) 의 교점 거리 (Moller-Trumbore, 행마다). 없으면 inf. 모서리는 조금 넓게 (틈으로 빠지지 않게)."""
    e1, e2 = b - a, c - a
    pv = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, pv)
    ok = np.abs(det) > eps
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    tv = o - a
    u = np.einsum("ij,ij->i", tv, pv) * inv
    qv = np.cross(tv, e1)
    v = np.einsum("ij,ij->i", d, qv) * inv
    t = np.einsum("ij,ij->i", e2, qv) * inv
    tol = 1e-9
    good = ok & (u >= -tol) & (v >= -tol) & (u + v <= 1 + tol) & (t >= 0)
    return np.where(good, t, np.inf)
