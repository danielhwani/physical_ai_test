"""Chrono 차량 + SCM 변형 지형 서버 (chrono conda 환경, Python 3.12에서 실행).

    sim.runner가 자동으로 띄운다: <chrono python> cosim/chrono_server.py --fd <socket fd> --root <repo>

역할 (문서 §5.1): 차량 동역학과 변형 지면(SCM) 계산. 로봇은 모른다 (단방향 연결).
러너가 정한 시각까지 진행한 뒤 차량 바디 포즈와 지형 높이 변경분을 돌려준다.

요청/응답 (cosim/wire.py)
  {"cmd": "init", "config": {...}}            -> {"bodies": [...], "scm": {...}}
  {"cmd": "advance", "t": T, "terrain": bool}  -> {"t": T, "poses": [...], "vehicle": {...}, "terrain": {...}|null}
  {"cmd": "close"}
"""
import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wire  # noqa: E402

COLORS = {"hmmwv_chassis": [0.36, 0.40, 0.30, 1.0], "hmmwv_rim": [0.25, 0.27, 0.22, 1.0]}
TIRE_COLOR = [0.08, 0.08, 0.08, 1.0]


def vec(v):
    return [v.x, v.y, v.z]


def quat_xyzw(q):
    return [q.e1, q.e2, q.e3, q.e0]


class ChronoServer:
    def __init__(self, root):
        self.root = Path(root)

    # ---------------- 초기화 ----------------
    def init(self, cfg):
        v, scm = cfg["vehicle"], cfg["scm"]
        self.step = cfg.get("step", 2e-3)
        start, end = np.array(v["start"], float), np.array(v["end"], float)
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        q = chrono.QuatFromAngleZ(heading)

        car = veh.HMMWV_Full()
        car.SetContactMethod(chrono.ChContactMethod_SMC)
        car.SetChassisFixed(False)
        car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(start[0], start[1], 0.6), q))
        car.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
        car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
        car.SetDriveType(veh.DrivelineTypeWV_AWD)
        car.SetTireType(veh.TireModelType_RIGID)        # SCM과 함께 쓰는 권장 타이어
        car.SetTireStepSize(self.step)
        car.Initialize()
        for part, vt in (("Chassis", chrono.VisualizationType_MESH), ("Suspension", chrono.VisualizationType_NONE),
                         ("Steering", chrono.VisualizationType_NONE), ("Wheel", chrono.VisualizationType_MESH),
                         ("Tire", chrono.VisualizationType_MESH)):
            getattr(car, f"Set{part}VisualizationType")(vt)
        self.car, self.sys = car, car.GetSystem()
        self.sys.SetNumThreads(1)                      # 결정성 (같은 입력 -> 같은 결과)

        # SCM: 기준 평면 원점 = 영역 중심, 격자 간격 = MuJoCo 지형 해상도 (칸 단위 정합, 문서 §5.3)
        self.terrain = veh.SCMTerrain(self.sys)
        sp = {k: float(v) for k, v in scm["soil"].items()}     # YAML은 1.0e6을 문자열로 읽으므로 숫자로 변환
        self.terrain.SetSoilParameters(sp["bekker_kphi"], sp["bekker_kc"], sp["bekker_n"], sp["cohesion"],
                                       sp["friction_deg"], sp["janosi_shear"], sp["elastic_k"], sp["damping_r"])
        cx, cy = scm["center"]
        self.terrain.SetReferenceFrame(chrono.ChCoordsysd(chrono.ChVector3d(cx, cy, 0.0), chrono.QUNIT))
        self.delta = scm["resolution"]
        self.terrain.Initialize(scm["size"][0], scm["size"][1], self.delta)
        self.terrain.AddActiveDomain(car.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
        self.scm_center = (cx, cy)
        self.sent = {}                                  # 마지막으로 보낸 노드 높이

        path = veh.StraightLinePath(chrono.ChVector3d(*start, 0.5), chrono.ChVector3d(*end, 0.5), 1)
        self.driver = veh.ChPathFollowerDriver(car.GetVehicle(), path, "crossing", v["speed"])
        self.driver.GetSteeringController().SetLookAheadDistance(5.0)
        self.driver.GetSteeringController().SetGains(0.8, 0, 0)
        self.driver.GetSpeedController().SetGains(0.4, 0, 0)
        self.driver.Initialize()
        self.t_start, self.end, self.stopped = v.get("t_start", 0.0), end[:2], False
        self.direction = (end[:2] - start[:2]) / np.linalg.norm(end[:2] - start[:2])
        return {"bodies": self.export_bodies(),
                "scm": {"center": [cx, cy], "size": scm["size"], "resolution": self.delta}}

    def export_bodies(self):
        """외형 메쉬가 있는 바디: 메쉬를 OBJ(외형 좌표계)로 내보내고 바디별 외형 오프셋을 알려준다.
        포즈는 각 바디의 외형 기준 프레임(GetVisualModelFrame)으로 보낸다."""
        out_dir = self.root / "build/scene_meshes/chrono"
        out_dir.mkdir(parents=True, exist_ok=True)
        self.bodies, described = [], []
        for b in self.sys.GetBodies():
            vm = b.GetVisualModel()
            if vm is None:
                continue
            visuals = []
            for inst in vm.GetShapeInstances():
                tm = chrono.CastToChVisualShapeTriangleMesh(inst.shape)
                if tm is None:
                    continue
                name = tm.GetName()
                path = out_dir / f"{name}.obj"
                if not path.exists():
                    m = tm.GetMesh()
                    lines = [f"v {p.x:.5f} {p.y:.5f} {p.z:.5f}" for p in m.GetCoordsVertices()]
                    lines += [f"f {f.x + 1} {f.y + 1} {f.z + 1}" for f in m.GetIndicesVertices()]
                    path.write_text("\n".join(lines) + "\n")
                rgba = COLORS.get(name, TIRE_COLOR if "tire" in name else [0.5, 0.5, 0.5, 1.0])
                visuals.append({"mesh": path.relative_to(self.root).as_posix(), "pos": vec(inst.frame.GetPos()),
                                "quat": quat_xyzw(inst.frame.GetRot()), "rgba": rgba})
            if visuals:
                self.bodies.append(b)
                described.append({"name": "hmmwv_" + b.GetName().replace(" ", "_"), "visuals": visuals})
        return described

    # ---------------- 진행 ----------------
    def inputs(self, t):
        """출발 전과 끝점 통과 후에는 정지 (제동). 그 사이는 경로 추종 운전자."""
        p = self.car.GetVehicle().GetPos()
        if not self.stopped and np.dot([p.x - self.end[0], p.y - self.end[1]], self.direction) > 0:
            self.stopped = True
        if t < self.t_start or self.stopped:
            di = veh.DriverInputs()
            di.m_throttle, di.m_steering, di.m_braking, di.m_clutch = 0.0, 0.0, 1.0, 0.0
            return di, False
        return self.driver.GetInputs(), True

    def advance(self, t_target, want_terrain):
        while self.sys.GetChTime() < t_target - 1e-9:
            t = self.sys.GetChTime()
            di, driving = self.inputs(t)
            if driving:
                self.driver.Synchronize(t)
            self.terrain.Synchronize(t)
            self.car.Synchronize(t, di, self.terrain)
            if driving:
                self.driver.Advance(self.step)
            self.terrain.Advance(self.step)
            self.car.Advance(self.step)
        poses = []
        for b in self.bodies:
            fr = b.GetVisualModelFrame()
            poses.append({"pos": vec(fr.GetPos()), "quat": quat_xyzw(fr.GetRot())})
        v = self.car.GetVehicle()
        reply = {"t": self.sys.GetChTime(), "poses": poses,
                 "vehicle": {"speed": v.GetSpeed(), "pos": vec(v.GetPos()), "stopped": self.stopped},
                 "terrain": self.terrain_changes() if want_terrain else None}
        return reply

    def terrain_changes(self):
        """지난 응답 이후 높이가 바뀐 SCM 노드 (노드 인덱스는 기준 평면 원점 기준, 높이는 평면 기준)."""
        ii, jj, hh = [], [], []
        for node, h in self.terrain.GetModifiedNodes(True):
            key = (node.x, node.y)
            if self.sent.get(key) != h:
                self.sent[key] = h
                ii.append(node.x); jj.append(node.y); hh.append(h)
        if not ii:
            return None
        return {"n": len(ii), "i": wire.b64(np.array(ii, "<i4").tobytes()),
                "j": wire.b64(np.array(jj, "<i4").tobytes()), "h": wire.b64(np.array(hh, "<f4").tobytes())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fd", type=int, required=True)
    ap.add_argument("--root", required=True)
    args = ap.parse_args()
    sock = socket.socket(fileno=args.fd)
    server = ChronoServer(args.root)
    busy = 0.0
    while True:
        msg = wire.recv(sock)
        t0 = time.perf_counter()
        if msg["cmd"] == "init":
            reply = server.init(msg["config"])
        elif msg["cmd"] == "advance":
            reply = server.advance(msg["t"], msg.get("terrain", False))
        elif msg["cmd"] == "close":
            wire.send(sock, {"busy_s": busy})
            break
        else:
            reply = {"error": f"unknown cmd {msg['cmd']}"}
        busy += time.perf_counter() - t0
        wire.send(sock, reply)
    sock.close()


if __name__ == "__main__":
    main()
