"""촬영 드론 (경로 계산형, 물리 없음): 로봇 뒤 위쪽을 따라다니며 로봇을 바라본다.

    drone:
      offset: [-2.5, 0.0, 2.0]     # 목표 지점: 로봇 몸통 기준 (앞 x, 왼쪽 y, 위 z, m). 로봇 방향 (yaw)으로 돌린다
      max_speed: 3.0               # m/s
      max_accel: 2.0               # m/s^2
      gain: 1.5                    # 목표까지 거리 -> 원하는 속도 (1/s)
      start: [-3.0, 0.0, 1.5]      # 처음 위치 (월드, 빼면 처음 목표 지점)

움직임: 원하는 속도 = gain × (목표 - 위치) (max_speed로 자름), 속도는 그쪽으로 max_accel까지만 바뀐다. 몸체는 가속 방향으로
기운다 (쿼드로터가 수평 가속을 내려면 그만큼 기울어야 한다: 기울기 = atan(a / g)). 머리는 로봇 쪽을 본다.
물리 (프로펠러 추력, 자세 제어, 바람)는 없다. 위치를 만드는 부분 (update)만 바꾸면 나중에 쿼드로터 물리로 갈아 끼울 수 있다.
결정적이다 (로봇 몸통 참값과 제어 주기만 쓴다).

렌더 스트림에는 같은 프로세스 Chrono, 별도 프로세스 차와 같은 모양 (manifest_bodies, stream_poses)으로 나간다:
장면 목록의 바디 "drone_body" (physics_source "kinematic", entity "drone"), 외형 메쉬는 build/scene_meshes/drone/*.obj.
"""
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MESH_DIR = ROOT / "build/scene_meshes/drone"
G = 9.81
BODY = "drone_body"
DEFAULTS = {"offset": [-2.5, 0.0, 2.0], "max_speed": 3.0, "max_accel": 2.0, "gain": 1.5}
ARM = 0.17          # 몸체 중심에서 프로펠러 중심까지 (m, 대각선 방향)


def _box(sx, sy, sz):
    """중심 원점 상자의 꼭짓점, 면 (OBJ용 1부터)."""
    v = [(x * sx / 2, y * sy / 2, z * sz / 2) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    f = [(1, 3, 4, 2), (5, 6, 8, 7), (1, 2, 6, 5), (3, 7, 8, 4), (1, 5, 7, 3), (2, 4, 8, 6)]
    return v, [(a, b, c) for q in f for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _disc(r, h, n=20):
    """z축 납작한 원기둥 (프로펠러 회전면)."""
    v = [(0.0, 0.0, -h / 2), (0.0, 0.0, h / 2)]
    for k in range(n):
        a = 2 * np.pi * k / n
        v += [(r * np.cos(a), r * np.sin(a), -h / 2), (r * np.cos(a), r * np.sin(a), h / 2)]
    f = []
    for k in range(n):
        i0, i1 = 3 + 2 * k, 3 + 2 * ((k + 1) % n)
        f += [(1, i1, i0), (2, i0 + 1, i1 + 1), (i0, i1, i1 + 1), (i0, i1 + 1, i0 + 1)]
    return v, f


def _write_obj(path, vf):
    """원점 중심 볼록 모양: 삼각형마다 법선이 바깥 (원점 반대)을 향하게 꼭짓점 순서를 맞춘다 (렌더러가 뒷면을 그리지 않을 수 있다)."""
    v, f = vf
    P = np.asarray(v, float)
    out = []
    for a, b, c in f:
        pa, pb, pc = P[a - 1], P[b - 1], P[c - 1]
        out.append((a, b, c) if np.dot(np.cross(pb - pa, pc - pa), pa + pb + pc) >= 0 else (a, c, b))
    f = out
    path.write_text("\n".join([f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in v] + [f"f {a} {b} {c}" for a, b, c in f]) + "\n")


def export_meshes():
    """드론 외형 메쉬 (몸체, 팔, 프로펠러)를 OBJ로. 반환: 장면 목록 visuals (바디 좌표)."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    parts = {"drone_hull": _box(0.16, 0.10, 0.06), "drone_arm": _box(2 * ARM * 1.05, 0.025, 0.02), "drone_rotor": _disc(0.08, 0.008)}
    for name, vf in parts.items():
        p = MESH_DIR / f"{name}.obj"
        if not p.exists():
            _write_obj(p, vf)
    rel = lambda n: (MESH_DIR / f"{n}.obj").relative_to(ROOT).as_posix()      # noqa: E731
    c45, s45 = np.cos(np.pi / 8), np.sin(np.pi / 8)                            # z축 ±45° 쿼터니언 (x, y, z, w)
    vis = [{"mesh": rel("drone_hull"), "pos": [0, 0, 0], "quat": [0, 0, 0, 1], "rgba": [0.20, 0.22, 0.25, 1.0]},
           {"mesh": rel("drone_arm"), "pos": [0, 0, 0.01], "quat": [0, 0, s45, c45], "rgba": [0.12, 0.12, 0.12, 1.0]},
           {"mesh": rel("drone_arm"), "pos": [0, 0, 0.01], "quat": [0, 0, -s45, c45], "rgba": [0.12, 0.12, 0.12, 1.0]}]
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        d = ARM / np.sqrt(2)
        vis.append({"mesh": rel("drone_rotor"), "pos": [sx * d, sy * d, 0.03], "quat": [0, 0, 0, 1],
                    "rgba": [0.85, 0.35, 0.10, 1.0] if sx > 0 else [0.75, 0.75, 0.78, 1.0]})   # 앞 프로펠러는 주황
    return vis


def _quat_xyzw(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2),
                              np.cos(yaw / 2), np.sin(yaw / 2))
    return np.array([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])


class FollowDrone:
    def __init__(self, cfg):
        c = dict(DEFAULTS, **(cfg if isinstance(cfg, dict) else {}))
        self.offset = np.asarray(c["offset"], float)
        self.max_speed, self.max_accel, self.gain = float(c["max_speed"]), float(c["max_accel"]), float(c["gain"])
        self.start = None if c.get("start") is None else np.asarray(c["start"], float)
        self.visuals = export_meshes()
        self.pos, self.vel, self.acc = None, np.zeros(3), np.zeros(3)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.max_tilt_deg = self.max_speed_seen = 0.0
        self.err = []

    def target(self, robot_pos, robot_yaw):
        c, s = np.cos(robot_yaw), np.sin(robot_yaw)
        o = self.offset
        return np.asarray(robot_pos, float) + [c * o[0] - s * o[1], s * o[0] + c * o[1], o[2]]

    def update(self, dt, robot_pos, robot_yaw):
        """한 제어 주기. robot_pos: 로봇 몸통 (월드), robot_yaw (rad)."""
        tgt = self.target(robot_pos, robot_yaw)
        if self.pos is None:
            self.pos = tgt.copy() if self.start is None else self.start.copy()
        want = self.gain * (tgt - self.pos)
        n = np.linalg.norm(want)
        if n > self.max_speed:
            want *= self.max_speed / n
        dv = want - self.vel
        n = np.linalg.norm(dv)
        lim = self.max_accel * dt
        if n > lim:
            dv *= lim / n
        self.acc = dv / dt
        self.vel = self.vel + dv
        self.pos = self.pos + self.vel * dt
        # 머리는 로봇 쪽, 몸체는 수평 가속 방향으로 기운다 (몸체 좌표: 앞으로 가속하면 앞이 숙여진다)
        to = np.asarray(robot_pos, float)[:2] - self.pos[:2]
        yaw = float(np.arctan2(to[1], to[0])) if np.linalg.norm(to) > 1e-6 else 0.0
        c, s = np.cos(yaw), np.sin(yaw)
        a_fwd, a_left = c * self.acc[0] + s * self.acc[1], -s * self.acc[0] + c * self.acc[1]
        pitch, roll = np.arctan2(a_fwd, G), -np.arctan2(a_left, G)
        self.quat = _quat_xyzw(roll, pitch, yaw)
        self.max_tilt_deg = max(self.max_tilt_deg, float(np.degrees(np.hypot(roll, pitch))))
        self.max_speed_seen = max(self.max_speed_seen, float(np.linalg.norm(self.vel)))
        self.err.append(float(np.linalg.norm(tgt - self.pos)))

    def mop(self):
        e = np.asarray(self.err) if self.err else np.zeros(1)
        return {"drone_follow_err_mean_m": round(float(e.mean()), 3), "drone_follow_err_max_m": round(float(e.max()), 3),
                "drone_speed_max_mps": round(self.max_speed_seen, 3), "drone_tilt_max_deg": round(self.max_tilt_deg, 2)}

    # ---- 렌더 스트림 (sim/stream.py build_manifest, Ros2StreamPublisher) ----
    def manifest_bodies(self):
        return [{"name": BODY, "physics_source": "kinematic", "entity": "drone", "visuals": self.visuals}]

    def stream_poses(self):
        p = self.pos if self.pos is not None else np.zeros(3)
        return [(BODY, np.array(p), self.quat.copy())]


class RenderSources:
    """MuJoCo 밖 바디의 출처 여럿 (같은 프로세스 Chrono 또는 별도 프로세스 차, 드론)을 하나로 묶는다."""

    def __init__(self, sources):
        self.sources = [s for s in sources if s is not None]

    def manifest_bodies(self):
        return [b for s in self.sources for b in s.manifest_bodies()]

    def stream_poses(self):
        return [p for s in self.sources for p in s.stream_poses()]
