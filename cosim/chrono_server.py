"""Chrono 차량 + SCM 변형 지형 서버 (chrono conda 환경, Python 3.12에서 실행).

    sim.runner가 자동으로 띄운다: <chrono python> cosim/chrono_server.py --fd <socket fd> --root <repo>

역할 (문서 §5.1): 변형 지면(SCM) 계산. 선택적으로 HMMWV 차량 동역학과, 로봇 발 하중에 의한 흙 눌림(발자국).
로봇의 동역학은 모른다: 발자국은 러너가 보낸 발 위치와 수직 하중으로만 계산한다.
러너가 정한 시각까지 진행한 뒤 차량 바디 포즈와 지형 높이 변경분을 돌려준다.

요청/응답 (cosim/wire.py)
  {"cmd": "init", "config": {...}}            -> {"bodies": [...], "scm": {...}}
  {"cmd": "advance", "t": T, "terrain": bool, "feet": [...]|null, "vehicle_cmd": {...}|null, "soil": {...}|null}
      robot_xy: 로봇 몸통 위치 (vehicle.yield_to_robot이면 차 앞 진로에 로봇이 있는 동안 제동하고 기다린다)
      vehicle_cmd: {"action": "start", "speed": m/s|null} 출발 (vehicle.t_start가 null이면 이 명령까지 제동하고 기다린다),
                   {"action": "stop"} 제동. soil: SCM 흙 값 (이번 구간부터)
                                              -> {"t": T, "poses": [...], "vehicle": {...}|null, "feet": [...], "terrain": {...}|null}
  {"cmd": "close"}
"""
import argparse
import math
import os
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
        """cfg: vehicle(선택, HMMWV), scm(필수), robot_feet(선택, 로봇 발 하중으로 흙 누르기)."""
        scm = cfg["scm"]
        # 발자국 연결은 작은 접촉이라 1 ms가 필요하다 (2 ms에서 수치 불안정 확인). HMMWV만이면 2 ms
        self.step = float(cfg.get("step", 1e-3 if cfg.get("robot_feet") else 2e-3))
        self.car = None
        if "vehicle" in cfg:
            self._init_vehicle(cfg["vehicle"])         # 차량 시스템(self.sys)을 만든다
        else:
            self.sys = chrono.ChSystemSMC()
            self.sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # SCM 레이캐스트에 필요
            self.sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
        self.sys.SetNumThreads(1)                      # 결정성 (같은 입력 -> 같은 결과)

        # SCM: 기준 평면 원점 = 영역 중심, 격자 간격 = MuJoCo 지형 해상도 (칸 단위 정합, 문서 §5.3)
        self.terrain = veh.SCMTerrain(self.sys)
        self.set_soil(scm["soil"])
        cx, cy = scm["center"]
        self.terrain.SetReferenceFrame(chrono.ChCoordsysd(chrono.ChVector3d(cx, cy, 0.0), chrono.QUNIT))
        self.delta = scm["resolution"]
        self.terrain.Initialize(scm["size"][0], scm["size"][1], self.delta)
        if self.car is not None:
            self.terrain.AddActiveDomain(self.car.GetChassisBody(), chrono.ChVector3d(0, 0, 0),
                                         chrono.ChVector3d(5, 3, 1))
        self.sent = {}                                  # 마지막으로 보낸 노드 높이
        self.feet = self._init_feet(cfg["robot_feet"]) if cfg.get("robot_feet") else []
        return {"bodies": self.export_bodies() if self.car is not None else [],
                "scm": {"center": [cx, cy], "size": scm["size"], "resolution": self.delta},
                "feet": [f["name"] for f in self.feet]}

    def set_soil(self, soil):
        sp = {k: float(v) for k, v in soil.items()}     # YAML은 1.0e6을 문자열로 읽으므로 숫자로 변환
        self.terrain.SetSoilParameters(sp["bekker_kphi"], sp["bekker_kc"], sp["bekker_n"], sp["cohesion"],
                                       sp["friction_deg"], sp["janosi_shear"], sp["elastic_k"], sp["damping_r"])

    def vehicle_command(self, cmd):
        """DIS 콘솔 개체 투입/제거. 차량은 시작 때 만들어 둔다 (실행 중 생성은 Chrono SCM에서 비정상 침하·충돌 확인)."""
        if self.car is None:
            return
        if cmd["action"] == "start":
            self.t_start, self.stopped = self.sys.GetChTime(), False
            if cmd.get("speed"):
                self.driver.SetDesiredSpeed(float(cmd["speed"]))
        elif cmd["action"] == "stop":
            self.stopped = True

    def _init_vehicle(self, v):
        start, end = np.array(v["start"], float), np.array(v["end"], float)
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        car = veh.HMMWV_Full()
        car.SetContactMethod(chrono.ChContactMethod_SMC)
        car.SetChassisFixed(False)
        car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(start[0], start[1], 0.6), chrono.QuatFromAngleZ(heading)))
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
        path = veh.StraightLinePath(chrono.ChVector3d(*start, 0.5), chrono.ChVector3d(*end, 0.5), 1)
        self.driver = veh.ChPathFollowerDriver(car.GetVehicle(), path, "crossing", v["speed"])
        self.driver.GetSteeringController().SetLookAheadDistance(5.0)
        self.driver.GetSteeringController().SetGains(0.8, 0, 0)
        self.driver.GetSpeedController().SetGains(0.4, 0, 0)
        self.driver.Initialize()
        self.t_start, self.end, self.stopped = v.get("t_start", 0.0), end[:2], False   # t_start None: 출발 명령까지 대기
        self.direction = (end[:2] - start[:2]) / np.linalg.norm(end[:2] - start[:2])
        # 양보: 차 앞 진로(앞 0~yield_ahead m, 옆 ±yield_half_width m)에 로봇이 있으면 제동 (로봇과 물리 작용이 없어 뚫고 지나가는 것을 막음)
        self.yield_cfg = (v.get("yield_ahead", 9.0), v.get("yield_half_width", 1.8)) if v.get("yield_to_robot") else None
        self.robot_xy, self.yielding = None, False

    def _init_feet(self, fc):
        """로봇 발 대리 물체: MuJoCo 발과 같은 반지름의 구. MuJoCo가 보낸 수직 하중으로 흙을 누른다.
        위치만 따라가게 하면 침하 깊이를 계산할 수 없으므로 '힘'으로 누른다 (깊이 = 하중과 흙 강도로 결정)."""
        feet = []
        mat = chrono.ChContactMaterialSMC()
        for name in fc.get("names", ["FL", "FR", "RL", "RR"]):
            b = chrono.ChBodyEasySphere(fc["radius"], 1000, False, True, mat)
            b.SetMass(fc.get("mass", 2.0))
            b.SetPos(chrono.ChVector3d(0, 0, 5.0))            # 첫 하중이 올 때까지 흙 위에 둔다
            # 충돌 마스크로 다른 물체와의 충돌을 끄면 SCM 레이캐스트에서도 빠져 흙을 뚫고 떨어진다 (확인함).
            # 그래서 기본 충돌 설정을 유지한다. 로봇 발과 HMMWV가 겹칠 일은 거의 없다.
            self.sys.Add(b)
            self.terrain.AddActiveDomain(b, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0.2, 0.2, 0.2))
            feet.append({"name": name, "body": b, "acc": b.AddAccumulator(), "lifted": True, "target": None})
        # 질량과 감쇠: 하중이 걸린 구가 흙에 "떨어지며" 충격으로 과하게 파이지 않도록 천천히 정적 평형에 이르게 한다.
        # 감쇠력은 명시적으로 걸리므로 damping*dt/mass < 2 여야 안정 (2 kg, 300 N s/m, 2 ms -> 0.3)
        self.foot_cfg = {"fn_min": fc.get("fn_min", 5.0), "damping": fc.get("damping", 300.0),
                         "lift": fc.get("lift", 0.05), "mass": fc.get("mass", 2.0), "radius": fc["radius"]}
        return feet

    def _apply_feet(self):
        if not self.feet:                 # 발자국 연결을 쓰지 않는 시나리오 (HMMWV만)
            return
        c = self.foot_cfg
        for f in self.feet:
            tgt, b = f["target"], f["body"]
            if tgt is None:
                continue
            x, y, z = tgt["pos"]
            b.EmptyAccumulator(f["acc"])
            if tgt["fn"] > c["fn_min"]:                       # 디딤: MuJoCo 하중으로 누른다
                if f["lifted"]:                               # 착지: Chrono 흙 표면에 바로 놓고 시작
                    # MuJoCo 발 높이를 쓰면 (MuJoCo 지형은 발 근처 갱신을 보류하므로) 이미 패인 자리 위에서
                    # 떨어지며 충격으로 과하게 판다. 그래서 흙 표면 높이에서 시작한다.
                    zs = self.terrain.GetHeight(chrono.ChVector3d(x, y, 0.0)) + c["radius"]
                    b.SetPos(chrono.ChVector3d(x, y, zs))
                    b.SetPosDt(chrono.ChVector3d(0, 0, 0))
                    f["lifted"] = False
                else:                                         # 수평 위치만 MuJoCo 발을 따른다
                    p, v = b.GetPos(), b.GetPosDt()
                    b.SetPos(chrono.ChVector3d(x, y, p.z))
                    b.SetPosDt(chrono.ChVector3d(0, 0, v.z))
                if os.environ.get("CHRONO_DEBUG_FEET"):
                    p = b.GetPos()
                    print(f"DBG t={self.sys.GetChTime():.3f} {f['name']} fn={tgt['fn']:.0f} x={p.x:.3f} y={p.y:.3f} "
                          f"z={p.z:.4f} vz={b.GetPosDt().z:.3f} surf={self.terrain.GetHeight(chrono.ChVector3d(p.x, p.y, 0)):.4f}",
                          flush=True)
                fz = -tgt["fn"] + c["mass"] * 9.81 - c["damping"] * b.GetPosDt().z   # 자중은 상쇄, 진동 감쇠
                b.AccumulateForce(f["acc"], chrono.ChVector3d(0, 0, fz), b.GetPos(), False)
            else:                                             # 들림: 흙 위로 올려 둔다 (흙 변형은 남는다)
                b.SetPos(chrono.ChVector3d(x, y, z + c["lift"]))
                b.SetPosDt(chrono.ChVector3d(0, 0, 0))
                f["lifted"] = True

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
        self.yielding = False
        if self.yield_cfg and self.robot_xy is not None and not (self.t_start is None or t < self.t_start or self.stopped):
            rel = np.asarray(self.robot_xy) - [p.x, p.y]
            along = float(rel @ self.direction)
            lateral = abs(float(rel[0] * self.direction[1] - rel[1] * self.direction[0]))
            self.yielding = 0.0 < along < self.yield_cfg[0] and lateral < self.yield_cfg[1]
        if self.t_start is None or t < self.t_start or self.stopped or self.yielding:
            di = veh.DriverInputs()
            di.m_throttle, di.m_steering, di.m_braking, di.m_clutch = 0.0, 0.0, 1.0, 0.0
            return di, False
        return self.driver.GetInputs(), True

    def advance(self, t_target, want_terrain, feet=None, vehicle_cmd=None, soil=None, robot_xy=None):
        """feet: 이번 구간 동안 쓸 로봇 발 목표 [{"pos": [x,y,z], "fn": N}, ...] (robot_feet 설정 시).
        vehicle_cmd, soil: DIS 콘솔 명령 (이번 구간 시작부터)."""
        if robot_xy is not None:
            self.robot_xy = robot_xy
        if vehicle_cmd:
            self.vehicle_command(vehicle_cmd)
        if soil:
            self.set_soil(soil)
        if feet:
            for f, tgt in zip(self.feet, feet):
                f["target"] = tgt
        while self.sys.GetChTime() < t_target - 1e-9:
            t = self.sys.GetChTime()
            self._apply_feet()
            if self.car is not None:
                di, driving = self.inputs(t)
                if driving:
                    self.driver.Synchronize(t)
                self.terrain.Synchronize(t)
                self.car.Synchronize(t, di, self.terrain)
                if driving:
                    self.driver.Advance(self.step)
                self.terrain.Advance(self.step)
                self.car.Advance(self.step)               # 차량 Advance가 시스템 전체를 한 스텝 진행
            else:
                self.terrain.Synchronize(t)
                self.terrain.Advance(self.step)
                self.sys.DoStepDynamics(self.step)
        poses = []
        for b in getattr(self, "bodies", []):
            # 프레임을 변수에 잡아 둔다: b.GetVisualModelFrame().GetPos()처럼 한 식에서 이어 부르면
            # SWIG 임시 객체가 먼저 해제되어 값이 깨진다 (x가 0으로 읽힘, 확인함)
            fr = b.GetVisualModelFrame()
            poses.append({"pos": vec(fr.GetPos()), "quat": quat_xyzw(fr.GetRot())})
        vehicle = None
        if self.car is not None:
            v = self.car.GetVehicle()
            vehicle = {"speed": v.GetSpeed(), "pos": vec(v.GetPos()), "stopped": self.stopped, "yielding": self.yielding}
        return {"t": self.sys.GetChTime(), "poses": poses, "vehicle": vehicle,
                "feet": [{"z": f["body"].GetPos().z, "loaded": not f["lifted"]} for f in self.feet],
                "terrain": self.terrain_changes() if want_terrain else None}

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
            reply = server.advance(msg["t"], msg.get("terrain", False), msg.get("feet"), msg.get("vehicle_cmd"), msg.get("soil"),
                                   msg.get("robot_xy"))
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
