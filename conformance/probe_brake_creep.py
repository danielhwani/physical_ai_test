"""제동한 차의 밀림 (README "제동한 차가 밀리는 원인"). chrono 환경:

    python conformance/probe_brake_creep.py <땅> [세워 둘 초] [고침]
      땅: rigid (단단한 평지) | firm (SCM 5e6) | soft (SCM 1e6)
      고침: none (기본 브레이크 SIMPLE) | shafts (BrakeType_SHAFTS) | lock (EnableBrakeLocking) | neutral (변속기 중립, 지금 Python에서 안 됨)

차 프로세스 (cosim/vehicle_peer.py)의 Car로 차 한 대를 10 m 달리고 제동한 뒤 세워 두고, 선 뒤 2초부터 끝까지 움직인 양을 출력한다.
vehicle_peer는 2026-11-22부터 브레이크 잠금을 켠다 (설정 brake_lock, 기본 켬). 이 시험은 lock일 때만 켠다.
"""
import json, math, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import pychrono as chrono
import pychrono.vehicle as veh
import cosim.vehicle_peer as vp

case = sys.argv[1]
hold = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
fix = sys.argv[3] if len(sys.argv) > 3 else "none"     # none | shafts | lock | neutral
_orig = veh.HMMWV_Full
def _mk(sys_):
    c = _orig(sys_)
    if fix == "shafts":
        c.SetBrakeType(veh.BrakeType_SHAFTS)
    return c
veh.HMMWV_Full = _mk
s = chrono.ChSystemSMC()
s.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
s.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
s.SetNumThreads(2)
cfg = {"start": [0.0, -10.0], "end": [0.0, 0.0], "speed": 3.0, "t_start": 0.0, "margin": 6.0, "brake_lock": fix == "lock"}

class NoDomain:
    def AddActiveDomain(self, *a): pass

if case == "rigid":
    ter = veh.RigidTerrain(s)
    mat = chrono.ChContactMaterialSMC(); mat.SetFriction(0.9); mat.SetRestitution(0.01); mat.SetYoungModulus(2e7)
    ter.AddPatch(mat, chrono.CSYSNORM, 60, 60)
    ter.Initialize()
    car = vp.Car(s, NoDomain(), cfg)
else:
    soil = {"rigid": None, "firm": {"bekker_kphi": 5e6}, "soft": {"bekker_kphi": 1e6}}[case]
    grid = vp.Grid({"half": [40, 40], "res": 0.05}, 1601, 1601)
    ter, _, _ = vp.make_terrain(s, dict(cfg, soil=soil), grid)
    car = vp.Car(s, ter, cfg)

t, k = 0.0, 0
rows, t_stop = [], None
end = None
w0 = time.perf_counter()
while True:
    ter.Synchronize(t)
    car.step(t, ter)
    ter.Advance(vp.STEP)
    s.DoStepDynamics(vp.STEP)
    t = s.GetChTime(); k += 1
    if car.state == "stop" and t_stop is None:
        t_stop = t
        if fix == "neutral":
            car.car.GetVehicle().GetTransmission().SetDriveMode(veh.ChAutomaticTransmission.DriveMode_NEUTRAL)
    if t_stop is not None and end is None:
        end = t_stop + hold
    if k % 250 == 0:
        v = car.car.GetVehicle()
        p, q = v.GetPos(), v.GetRot()
        yaw = math.degrees(math.atan2(2 * (q.e0 * q.e3 + q.e1 * q.e2), 1 - 2 * (q.e2 ** 2 + q.e3 ** 2)))
        w = [v.GetSpindleAngVel(a, sd).Length() for a in (0, 1) for sd in (veh.LEFT, veh.RIGHT)]
        rows.append([round(t, 2), round(p.x, 4), round(p.y, 4), round(p.z, 4), round(yaw, 3), round(v.GetSpeed(), 4), round(max(w), 3)])
        if k % 2500 == 0:
            e, tr = v.GetEngine(), v.GetTransmission()
            print(f"    t {t:5.1f} gear {tr.GetCurrentGear()} engine rpm {e.GetMotorSpeed() * 60 / (2 * math.pi):7.1f} torque {e.GetOutputMotorshaftTorque():7.1f}", flush=True)
    if end is not None and t >= end:
        break
r = [row for row in rows if row[0] >= t_stop + 2.0]
a, b = r[0], r[-1]
print(json.dumps({"case": case, "fix": fix, "t_stop": round(t_stop, 2), "wall_s": round(time.perf_counter() - w0, 1),
                  "from_t": a[0], "to_t": b[0], "dx_cm": round(100 * (b[1] - a[1]), 2), "dy_cm": round(100 * (b[2] - a[2]), 2),
                  "dz_cm": round(100 * (b[3] - a[3]), 2), "dyaw_deg": round(b[4] - a[4], 3),
                  "speed_absmax": max(abs(x[5]) for x in r), "spin_max": max(x[6] for x in r)}))
print("  t x y z yaw speed spin (5초마다):", [x for x in r[::10]])
