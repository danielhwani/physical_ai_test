"""차 한 대를 별도 Chrono 프로세스로 (맵 서버 4단계 운용, chrono conda 환경, Python 3.12).

    sim.runner가 시나리오의 vehicles 목록에서 차마다 띄운다: <chrono python> cosim/vehicle_peer.py --config '<JSON>'

시험 (conformance/probe_peer.py, probe_two_chrono.py)에서 확인한 규칙을 그대로 쓴다 (README "서로 궤적 받기", "늦게 붙기"):
  - 자기 SCM에 자기 HMMWV 한 대. 다른 차의 자국은 높이를 넣지 않고, 받은 바퀴 궤적을 고스트 바퀴로 재생해 자기 SCM이 직접 누른다 (다짐까지).
  - 자기 바퀴 궤적을 hz로 맵 서버에 보낸다 (traj). 출발 전과 끝에 닿아 선 뒤에도 보낸다 (서 있는 차도 흙을 누른다).
  - 지도에는 자기 SCM이 지도보다 THRESH 넘게 낮아진 칸만 dig로 쓴다 (0.2 s마다).
  - 다른 차 궤적은 delay만큼 늦춰 재생한다. 밀리면 catchup배로 따라잡고, 자기 앞바퀴 GUARD m 안의 고스트는 숨기고, NEAR m보다 먼 고스트는 쉬게 한다.
  - 시계: 모든 차가 같은 시뮬레이션 시각 0 (러너가 준 벽시계 epoch)에서 출발해 실시간 1배로 간다. 궤적의 스텝 번호 k가 곧 공용 시각이다.

러너와의 약속 (표준 입출력):
  준비되면 (맵 서버에 붙고 SCM과 차를 만든 뒤) 표준 출력에 "READY <바디 JSON>" 한 줄. 바디 JSON은 렌더러 매니페스트에 넣을
  [{"name": "<차 이름>_<바디>", "visuals": [{mesh, pos, quat, rgba}]}]. 러너가 "GO <epoch>" 한 줄을 주면 epoch (time.time)를 시각 0으로 달린다.
  달리는 동안 "POSE <JSON>" 줄을 50 Hz로: [t, [[x, y, z, qx, qy, qz, qw], ...]] (READY의 바디 순서, 외형 기준 프레임). 러너가 렌더러 TF로 보낸다.
  러너는 달리는 동안 로봇 몸통 위치를 "ROBOT <x> <y>" 줄로 20 Hz로 준다 (양보 판단용, 시뮬레이터 참값). 차는 50 ms마다 표준 입력을 비운다.
  표준 입력이 닫히면 (러너가 끝남) 마지막 자국과 궤적 끝을 보내고 결과를 out에 쓰고 끝낸다. 진행 기록은 표준 오류로.

config (JSON):
  name, peers (다른 차 이름들), tcp ("127.0.0.1:포트"), grid {"half": [hx, hy], "res"}, out (결과 JSON 경로),
  start [x, y], end [x, y], speed (m/s), t_start (s, 그때까지 제동), duration (s, 러너가 안 닫아도 이 시각 + 5 s에 끝낸다),
  soil {bekker_kphi, bekker_kc, bekker_n, cohesion, friction_deg, janosi_shear, elastic_k, damping_r}, hz, delay, catchup, threads, margin,
  yield_to_robot (차 앞 진로, 차체 중심 기준 앞 0 ~ yield_ahead m, 옆 ±yield_half_width m에 로봇이 있으면 제동하고 기다린다.
  같은 프로세스 Chrono (cosim/chrono_server.py)와 같은 규칙과 기본값 9 m, 1.8 m)
"""
import argparse
import bisect
import json
import math
import os
import select
import sys
import time
from pathlib import Path

import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.map_client import RemoteMap, dec  # noqa: E402
from cosim.chrono_server import body_poses, export_visual_bodies  # noqa: E402

STEP = 2e-3
TIRE_R, TIRE_W = 0.467, 0.254                      # HMMWV 단단한 타이어 (고스트 원통도 같은 크기)
WHEEL_DOMAIN = (1.1, 0.5, 1.1)                     # 차 바퀴마다 SCM 활성 영역 (m, README "실시간: 활성 영역 줄이기")
GHOST_DOMAIN = (1.1, 0.4, 1.1)
THRESH = 0.001                                     # 지도보다 이만큼 더 낮아진 칸만 쓴다 (m)
WRITE_EVERY = 100                                  # 지도에 쓰는 주기 (스텝, 0.2 s)
MAX_FILL = 1.0                                     # 받은 궤적 프레임 사이를 메우는 최대 시간 (s)
GUARD = 1.0                                        # 자기 앞바퀴 앞 이 거리 안의 고스트는 숨긴다 (m)
INPUT_EVERY = 25                                   # 표준 입력 (로봇 위치, 끝) 확인 주기 (스텝, 50 ms)
POSE_EVERY = 10                                    # 렌더러용 바디 포즈 주기 (스텝, 50 Hz)
NEAR = 15.0                                        # 이보다 먼 고스트는 쉬게 한다 (m)
SOIL = {"bekker_kphi": 5e6, "bekker_kc": 0.0, "bekker_n": 1.1, "cohesion": 0.0, "friction_deg": 30.0, "janosi_shear": 0.01,
        "elastic_k": 4e7, "damping_r": 3e4}


def log(*a):
    print(*a, file=sys.stderr, flush=True)


class StdinLines:
    """표준 입력을 직접 읽어 줄로 나눈다. sys.stdin.readline은 줄을 버퍼에 미리 읽어 두어 select가 남은 줄을 알려 주지 않는다."""

    def __init__(self):
        self.fd, self.buf, self.eof = sys.stdin.fileno(), b"", False

    def poll(self, timeout=0.0):
        """지금 읽을 수 있는 줄들 (timeout 초까지 기다린다). 입력이 닫히면 eof."""
        while not self.eof and select.select([self.fd], [], [], timeout)[0]:
            data = os.read(self.fd, 65536)
            if not data:
                self.eof = True
            self.buf += data
            timeout = 0.0
        *lines, self.buf = self.buf.split(b"\n")
        return [ln.decode().strip() for ln in lines if ln.strip()]


class Grid:
    """지도 격자: 행 r -> y = -hy + r*res, 열 c -> x = -hx + c*res (sim/terrain_service.py와 같은 규약)."""

    def __init__(self, g, nrow, ncol):
        self.hx, self.hy = g["half"]
        self.res, self.nrow, self.ncol = g["res"], nrow, ncol

    def cell(self, x, y):
        return int(round((y + self.hy) / self.res)), int(round((x + self.hx) / self.res))

    def xy(self, r, c):
        return -self.hx + c * self.res, -self.hy + r * self.res


class Car:
    """start에서 end 방향 직선을 speed로. end를 지나면 제동하고 선다. t_start 전에는 제동."""

    def __init__(self, s, ter, cfg):
        (x0, y0), (x1, y1) = cfg["start"], cfg["end"]
        self.start, self.length = (x0, y0), math.hypot(x1 - x0, y1 - y0)
        self.dir = ((x1 - x0) / self.length, (y1 - y0) / self.length)
        self.speed, self.t_start = float(cfg["speed"]), float(cfg.get("t_start", 0.0))
        car = veh.HMMWV_Full(s)
        car.SetContactMethod(chrono.ChContactMethod_SMC)
        car.SetChassisFixed(False)
        car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(x0, y0, 0.6), chrono.QuatFromAngleZ(math.atan2(*self.dir[::-1]))))
        car.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
        car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
        car.SetDriveType(veh.DrivelineTypeWV_AWD)
        car.SetTireType(veh.TireModelType_RIGID)
        car.SetTireStepSize(STEP)
        # 브레이크 잠금: 기본 브레이크 (SIMPLE)는 선 바퀴를 붙잡지 못해, 1단 기어의 엔진 브레이크 토크 (약 -5 Nm)에 밀려 평지에서 30초에 6 m 뒤로 굴렀다.
        # 흙 위에서는 떨며 수 cm 밀리고 자국을 1 cm 더 팠다 (README "제동한 차가 밀리는 원인"). 잠그면 바퀴가 거의 설 때 고정한다
        car.EnableBrakeLocking(bool(cfg.get("brake_lock", True)))   # 시험 (conformance/probe_brake_creep.py)만 끈다
        car.Initialize()
        for part, vt in (("Chassis", chrono.VisualizationType_MESH), ("Suspension", chrono.VisualizationType_NONE),
                         ("Steering", chrono.VisualizationType_NONE), ("Wheel", chrono.VisualizationType_MESH),
                         ("Tire", chrono.VisualizationType_MESH)):
            getattr(car, f"Set{part}VisualizationType")(vt)      # 렌더러에 보낼 외형 (chrono_server와 같음)
        self.car = car
        far = self.length + 8.0
        self.path = veh.StraightLinePath(chrono.ChVector3d(x0, y0, 0.5), chrono.ChVector3d(x0 + far * self.dir[0], y0 + far * self.dir[1], 0.5), 1)
        self.driver = veh.ChPathFollowerDriver(car.GetVehicle(), self.path, "d", self.speed)
        self.driver.GetSteeringController().SetLookAheadDistance(5.0)
        self.driver.GetSteeringController().SetGains(0.8, 0, 0)
        self.driver.GetSpeedController().SetGains(0.4, 0, 0)
        self.driver.Initialize()
        for axle in (0, 1):
            for side in (veh.LEFT, veh.RIGHT):
                ter.AddActiveDomain(car.GetVehicle().GetWheel(axle, side).GetSpindle(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(*WHEEL_DOMAIN))
        self.state = "wait"                        # wait (출발 전) -> drive -> stop (끝에 닿음)
        # 양보 (cosim/chrono_server.py와 같은 규칙): 차 앞 진로에 로봇이 있으면 제동하고 기다린다 (로봇과 물리 작용이 없어 뚫고 지나가는 것을 막음)
        self.yield_cfg = (float(cfg.get("yield_ahead", 9.0)), float(cfg.get("yield_half_width", 1.8))) if cfg.get("yield_to_robot") else None
        self.robot_xy, self.yielding, self.yields, self.yield_s = None, False, 0, 0.0

    def pos(self):
        p = self.car.GetVehicle().GetPos()
        return p.x, p.y, p.z

    def along(self):
        x, y, _ = self.pos()
        return (x - self.start[0]) * self.dir[0] + (y - self.start[1]) * self.dir[1]

    def check_yield(self):
        was = self.yielding
        self.yielding = False
        if self.yield_cfg and self.robot_xy is not None and self.state == "drive":
            x, y, _ = self.pos()
            rx, ry = self.robot_xy[0] - x, self.robot_xy[1] - y
            along = rx * self.dir[0] + ry * self.dir[1]
            lateral = abs(rx * self.dir[1] - ry * self.dir[0])
            self.yielding = 0.0 < along < self.yield_cfg[0] and lateral < self.yield_cfg[1]
        self.yields += int(self.yielding and not was)

    def step(self, t, ter):
        if self.state == "wait" and t >= self.t_start:
            self.state = "drive"
        if self.state == "drive" and self.along() > self.length:
            self.state = "stop"
        if self.yielding:
            self.yield_s += STEP
        if self.state != "drive" or self.yielding:
            di = veh.DriverInputs()
            di.m_throttle, di.m_steering, di.m_braking, di.m_clutch = 0.0, 0.0, 1.0, 0.0
            self.car.Synchronize(t, di, ter)
            self.car.Advance(STEP)
            return
        di = self.driver.GetInputs()
        self.driver.Synchronize(t)
        self.car.Synchronize(t, di, ter)
        self.driver.Advance(STEP)
        self.car.Advance(STEP)

    def wheel_frame(self):
        v = self.car.GetVehicle()
        out = []
        for axle in (0, 1):
            for side in (veh.LEFT, veh.RIGHT):
                p, q, u, w = v.GetSpindlePos(axle, side), v.GetSpindleRot(axle, side), v.GetSpindleLinVel(axle, side), v.GetSpindleAngVel(axle, side)
                out.append((p.x, p.y, p.z, q.e0, q.e1, q.e2, q.e3, u.x, u.y, u.z, w.x, w.y, w.z))
        return out


def hide(b):
    b.SetPos(chrono.ChVector3d(0, 0, 50))
    b.SetPosDt(chrono.ChVector3d(0, 0, 0))
    b.SetAngVelParent(chrono.ChVector3d(0, 0, 0))


def ghost_bodies(s, ter, count):
    """고스트 바퀴 몸체. SCM 활성 영역은 첫 스텝 전에 더해야 해서 처음에 다 만든다."""
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(0.8)
    out = []
    for _ in range(count):
        b = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, TIRE_R, TIRE_W, 1000, False, True, mat)
        b.SetFixed(True)
        b.SetPos(chrono.ChVector3d(0, 0, 50))
        s.Add(b)
        ter.AddActiveDomain(b, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(*GHOST_DOMAIN))
        out.append(b)
    return out


class Playback:
    """한 상대의 궤적 재생 (conformance/probe_peer.py Playback과 같은 규칙). k는 공용 시각 (스텝)."""

    def __init__(self, bodies, delay, catchup):
        self.bodies, self.delay, self.R = bodies, delay, catchup
        self.frames, self.keys, self.every, self.end_k, self.max_fill = {}, [], None, None, None
        self.cur = 0
        self.stats = {"placed": 0, "missed": 0, "hid": 0, "far": 0, "fast": 0, "behind_s": 0.0}

    def add(self, tm):
        self.every = tm["every"]
        self.max_fill = int(round(MAX_FILL / (self.every * STEP)))
        if tm["end"]:
            self.end_k = tm["k0"]
        elif tm["data"]:
            f = tm["k0"] // self.every
            if f not in self.frames:
                bisect.insort(self.keys, f)
            self.frames[f] = dec(tm["data"], tuple(tm["shape"]))[0]

    def advance(self, target):
        c, keys, every = self.cur, self.keys, self.every
        if c >= target:
            return target, 1
        last = keys[-1] * every
        if c >= last:
            return c, 0
        i = bisect.bisect_right(keys, c // every)
        if i == 0:
            return min(target, keys[0] * every), 0
        lo, hi = keys[i - 1], keys[i]
        if hi - lo > self.max_fill:
            return min(target, hi * every), 0
        rate = self.R if last - c >= 2 * every else 1
        return min(target, last, c + rate), rate

    def pose(self, idx):
        a0 = idx // self.every
        i = bisect.bisect_right(self.keys, a0)
        if i == 0 or i >= len(self.keys):
            return None
        lo, hi = self.keys[i - 1], self.keys[i]
        if hi - lo > self.max_fill:
            return None
        f = (idx - lo * self.every) / ((hi - lo) * self.every)
        fr = (1 - f) * self.frames[lo] + f * self.frames[hi]
        fr[:, 3:7] = fr[:, 3:7] / np.linalg.norm(fr[:, 3:7], axis=1, keepdims=True)
        return fr

    def step(self, k, car):
        st = self.stats
        if self.every is None or len(self.keys) < 2:
            for b in self.bodies:
                hide(b)
            return
        target = max(0, k - self.delay)
        self.cur, rate = self.advance(target)
        st["behind_s"] = max(st["behind_s"], (target - self.cur) * STEP)
        st["fast"] += rate > 1 and target - self.cur > 0
        if self.end_k is not None and self.cur >= self.end_k - self.every:
            for b in self.bodies:
                hide(b)
            return
        fr = self.pose(self.cur)
        if fr is None:
            for b in self.bodies:
                hide(b)
            st["missed"] += 1
            return
        x, y, _ = car.pos()
        if np.min(np.hypot(fr[:, 0] - x, fr[:, 1] - y)) > NEAR:
            for b in self.bodies:
                hide(b)
            st["far"] += 1
            return
        for b, r in zip(self.bodies, fr):
            b.SetPos(chrono.ChVector3d(*map(float, r[:3])))
            b.SetRot(chrono.ChQuaterniond(*map(float, r[3:7])))
            b.SetPosDt(chrono.ChVector3d(*map(float, r[7:10])))
            b.SetAngVelParent(chrono.ChVector3d(*map(float, r[10:13])))
        st["placed"] += 1
        st["hid"] += hide_late(car, self.bodies, fr)


def hide_late(car, bodies, frame):
    """자기 앞바퀴 앞 GUARD m 안까지 왔거나 지나간 고스트 바퀴를 숨긴다 (늦은 궤적: 고스트는 충돌 몸체라 겹치면 차에 부딪힌다)."""
    v = car.car.GetVehicle()
    pl, pr = v.GetSpindlePos(0, veh.LEFT), v.GetSpindlePos(0, veh.RIGHT)
    cx, cy = (pl.x + pr.x) / 2, (pl.y + pr.y) / 2
    fwd = chrono.ChMatrix33d(v.GetRot()).GetAxisX()
    n = 0
    for b, r in zip(bodies, frame):
        dx, dy = r[0] - cx, r[1] - cy
        if dx * fwd.x + dy * fwd.y < GUARD and abs(-dx * fwd.y + dy * fwd.x) < 2.0:
            hide(b)
            n += 1
    return n


def make_terrain(s, cfg, grid):
    """길을 덮는 축 정렬 SCM (길 양옆과 앞뒤로 margin m). 노드가 지도 칸과 겹치도록 기준점을 지도 칸에 둔다.
    반환: SCM, 기준 칸 (행, 열), 노드 반 개수 (가로, 세로)."""
    (x0, y0), (x1, y1) = cfg["start"], cfg["end"]
    mg = float(cfg.get("margin", 6.0))
    lo_x, hi_x = min(x0, x1) - mg, max(x0, x1) + mg
    lo_y, hi_y = min(y0, y1) - mg, max(y0, y1) + mg
    rc, cc = grid.cell((lo_x + hi_x) / 2, (lo_y + hi_y) / 2)
    ox, oy = grid.xy(rc, cc)
    nx = int(math.ceil(max(hi_x - ox, ox - lo_x) / grid.res))
    ny = int(math.ceil(max(hi_y - oy, oy - lo_y) / grid.res))
    soil = {k: float(v) for k, v in dict(SOIL, **(cfg.get("soil") or {})).items()}   # YAML은 5.0e6을 문자열로 읽는다
    t = veh.SCMTerrain(s)
    t.SetSoilParameters(soil["bekker_kphi"], soil["bekker_kc"], soil["bekker_n"], soil["cohesion"], soil["friction_deg"],
                        soil["janosi_shear"], soil["elastic_k"], soil["damping_r"])
    t.SetReferenceFrame(chrono.ChCoordsysd(chrono.ChVector3d(ox, oy, 0), chrono.QUNIT))
    t.Initialize(2 * nx * grid.res, 2 * ny * grid.res, grid.res)
    return t, (rc, cc), (nx, ny)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    cfg = json.loads(ap.parse_args().config)
    name, peers = cfg["name"], list(cfg.get("peers", []))
    hz, delay, catchup = float(cfg.get("hz", 100)), float(cfg.get("delay", 0.2)), int(cfg.get("catchup", 4))

    m = RemoteMap(None, name=name, tcp=cfg["tcp"], wait_s=60.0, keep_traj=True)
    grid = Grid(cfg["grid"], m.local.nrow, m.local.ncol)
    s = chrono.ChSystemSMC()
    s.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    s.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    s.SetNumThreads(int(cfg.get("threads", 2)))
    ter, (rc, cc), (nx, ny) = make_terrain(s, cfg, grid)
    # SCM은 평평한 새 흙 (높이 0)에서 시작한다: 지도와 겹치는 곳의 지금 높이도 0이어야 한다 (다른 쪽 자국을 높이로 넣지 않는다)
    r0, r1 = max(rc - ny, 0), min(rc + ny + 1, grid.nrow)
    c0, c1 = max(cc - nx, 0), min(cc + nx + 1, grid.ncol)
    if r0 < r1 and c0 < c1:
        z = m.local.compose(r0, r1, c0, c1)
        if np.any(z != 0.0):
            raise SystemExit(f"{name}: SCM 영역 (지도 행 {r0}~{r1}, 열 {c0}~{c1})의 지도 높이가 0이 아니다 (최대 |z| {np.abs(z).max():.3f} m). "
                             "차가 지나는 곳은 평평해야 한다 (요철 패치를 옮기거나 margin을 줄인다)")
    plays = {p: Playback(ghost_bodies(s, ter, 4), int(round(delay / STEP)), catchup) for p in peers}
    car = Car(s, ter, cfg)
    n_hist = m.fetch_traj_history()                    # 늦게 붙었으면 지난 궤적 (공용 시각이라 재생 규칙이 그대로 따라잡는다)
    every = max(1, int(round(1.0 / (hz * STEP))))
    tire = (TIRE_R, TIRE_W)
    sent = {}

    def receive():
        m.poll()
        for tm in m.traj_in:
            pb = plays.get(tm["writer"])
            if pb is not None and not (tm["k0"] == 0 and not tm["data"] and not tm["end"] and tm["every"] == 1):
                pb.add(tm)
        m.traj_in.clear()

    def write(t):
        i, j, h = [], [], []
        for n, v in ter.GetModifiedNodes(True):
            if sent.get((n.x, n.y)) != v:
                sent[(n.x, n.y)] = v
                i.append(n.x), j.append(n.y), h.append(v)
        if not i:
            return
        r, c, h = rc + np.array(j), cc + np.array(i), np.array(h)
        ok = (r >= 0) & (r < grid.nrow) & (c >= 0) & (c < grid.ncol)
        r, c, h = r[ok], c[ok], np.clip(h[ok], m.local.z_min, m.local.z_max)
        if r.size:
            r0, c0 = int(r.min()), int(c.min())                  # 쓰는 칸을 감싸는 범위만 합성한다 (지도 전체는 87만 칸일 수 있다)
            cur = m.local.compose(r0, int(r.max()) + 1, c0, int(c.max()) + 1)[r - r0, c - c0]
            low = h < cur - THRESH
            if low.any():
                m.write_cells("dig", name, r[low], c[low], h[low], t=t)

    log(f"{name}: SCM {2 * nx * grid.res:.1f} x {2 * ny * grid.res:.1f} m, 다른 차 {peers}, 지난 궤적 {n_hist}")
    vis_bodies, described = export_visual_bodies(s, ROOT, prefix=f"{name}_")

    def poses_line(t):
        return "POSE " + json.dumps([round(t, 3), [[round(v, 4) for v in p + q] for p, q in body_poses(vis_bodies)]])

    print("READY " + json.dumps(described), flush=True)      # 렌더러 매니페스트에 넣을 바디와 외형
    print(poses_line(0.0), flush=True)
    inp = StdinLines()
    lines = []
    while not lines and not inp.eof:
        lines = inp.poll(timeout=1.0)
    line = lines[0].split() if lines else []
    if not line or line[0] != "GO":
        log(f"{name}: 시작 신호 없이 끝남")
        m.close()
        return
    epoch = float(line[1])
    while time.time() < epoch:
        time.sleep(min(0.01, epoch - time.time()))
    wall0 = time.perf_counter() - (time.time() - epoch)
    t, k, lag, closed = 0.0, 0, 0.0, False
    track = []
    t_end = float(cfg.get("duration", 600.0)) + 5.0
    while not closed and t < t_end:
        receive()
        for pb in plays.values():
            pb.step(k, car)
        ter.Synchronize(t)
        car.step(t, ter)
        ter.Advance(STEP)
        s.DoStepDynamics(STEP)
        t = s.GetChTime()
        if k % POSE_EVERY == 0:
            print(poses_line(t), flush=True)
        if k % every == 0:
            m.send_traj(name, k, every, [car.wheel_frame()], tire)
        if (k + 1) % WRITE_EVERY == 0:
            write(t)
            x, y, z = car.pos()
            track.append((t, x, y, z, car.car.GetVehicle().GetSpeed(), time.perf_counter() - wall0 - t, float(car.yielding)))   # 늦음 (s), 양보 중
        if k % INPUT_EVERY == 0:
            for ln in inp.poll():
                f = ln.split()
                if f[0] == "ROBOT":
                    car.robot_xy = (float(f[1]), float(f[2]))
            closed = inp.eof                               # 러너가 끝났다
            car.check_yield()
        k += 1
        now = time.perf_counter() - wall0
        if now < t:
            time.sleep(t - now)
        else:
            lag = max(lag, now - t)
    write(t)
    m.send_traj(name, k, every, np.zeros((0, 4, 13)), tire, end=True)
    au = m.audit(wait_s=20.0)
    out = {"name": name, "sim_s": round(t, 3), "max_lag_s": round(lag, 4), "state": car.state, "along_m": round(car.along(), 3),
           "history_msgs": n_hist, "audit": au, "ghosts": {p: pb.stats for p, pb in plays.items()},
           "yields": car.yields, "yield_s": round(car.yield_s, 2),
           "track_columns": ["t", "x", "y", "z", "speed", "lag", "yielding"], "track": [[round(v, 4) for v in row] for row in track]}
    Path(cfg["out"]).write_text(json.dumps(out))
    log(f"{name}: 끝 t {t:.1f} s, 늦음 최대 {1000 * lag:.0f} ms, 상태 {car.state}, 양보 {car.yields}번 {car.yield_s:.1f} s, 지도 대조 {au.get('hash_mismatch')}")
    m.close()


if __name__ == "__main__":
    prof = os.environ.get("VEHICLE_PEER_PROFILE")        # 진단용: 이 폴더에 차마다 cProfile 결과 (<이름>.prof)
    if prof:
        import cProfile
        name = json.loads(sys.argv[sys.argv.index("--config") + 1])["name"]
        cProfile.run("main()", str(Path(prof) / f"{name}.prof"))
    else:
        main()
