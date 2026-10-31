"""HMMWV 두 대가 같은 길을 달릴 때 뒤 차가 앞 차 자국에서 받는 효과의 기준 측정 (docs/map_server.md 5.5). 시뮬레이터, 맵 서버 없이 Chrono만.

    python conformance/probe_scm_two_hmmwv.py [--kphi 5e6 1e6] [--mults 8] [--stride 1 5] [--record build/scm_two_hmmwv.npz]
    (어느 환경이든: pychrono가 없으면 chrono 환경으로 다시 실행)

  세 경우 (모두 같은 차, 같은 흙, 뒤 차는 같은 출발):
    A 이어 지나감 (정답)  한 SCM에서 두 대가 GAP m 간격으로 같은 길을 달린다. 뒤 차는 앞 차가 다진 자국을 밟는다.
    B 높이만             앞 차 혼자 달려 만든 자국의 높이를 새 SCM에 SetModifiedNodes로 넣고 뒤 차만 달린다. 지금 맵 서버 방식 (넣은 노드는 새 흙).
    C 평지               자국 없이 뒤 차만 달린다. 자국의 효과가 얼마나 큰지 보려고.
    D 단단한 흙          B + 자국 칸(앞 차가 RUT_MIN보다 판 칸)의 kphi를 콜백으로 올린다 (높이 + 단단함).
       D 계산값: 정답을 보지 않고 칸마다 자국 깊이 s에서. 받았던 압력 p = kphi s^n, 다져진 흙은 p까지 탄성으로만 눌린다 -> 다시 눌리는 깊이 s_e = p / elastic_k
                 -> 그 깊이에서 p를 받치는 kphi' = p / s_e^n.
       D 배수:   자국 칸의 kphi를 모두 m배. 정답에 맞는 값이 있는지 (맞춘 값을 쓰면 정답을 본 것이다).
    E 가상 다짐          앞 차 네 바퀴의 궤적(위치, 회전, 속도, 각속도)을 받아, 그 위치로 움직이는 유령 바퀴(타이어 크기 원통, 고정 몸체)로 새 SCM을 눌러 자국과 다짐을 다시 만든다.
                         하중이 아니라 위치로 누른다 (차체 동역학이 필요 없다). 그 뒤 유령을 치우고 뒤 차만 달린다. stride: 궤적을 몇 스텝마다 쓸지 (클수록 빠르다).
  재는 구간 (뒤 차 중심 y WINDOW, 뒤 차가 자국 안에서 좌우로 흔들려 넓게 평균한다): 추가 침하 (앞 차 자국 바닥 - 뒤 차가 지난 뒤 바닥), 끝 자국 깊이, 바퀴 중심 높이, 차체 높이, 차체 기울기 흔들림,
  옆으로 벗어남, 스로틀. 차 두 대는 충돌하지 않게 떨어져 있고, 서로 지형으로만 영향을 준다.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

try:
    import pychrono as chrono
    import pychrono.vehicle as veh
except ImportError:                                # mj_ros 등: chrono 환경으로 다시 실행
    py = Path.home() / "miniconda3/envs/chrono/bin/python"
    os.execv(str(py), [str(py), *sys.argv])

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scm_record  # noqa: E402

DELTA, STEP = 0.05, 2e-3
SPEED, GAP = 3.0, 12.0
Y_FRONT, Y_END = -14.0, 18.0                       # 앞 차 출발 y, 끝 y. 뒤 차는 GAP 뒤에서 출발
SCM_Y = (-30.0, 22.0)
SCM_W = 4.0
WINDOW = (-8.0, 12.0)                              # 재는 구간 (뒤 차 중심 y)
BIN = 4.0                                          # 구간 안에서 값이 얼마나 흔들리는지 볼 칸 (m)
TIRE_R, TIRE_W = 0.467, 0.254                      # HMMWV 단단한 타이어 (접촉 원통)
RUT_MIN = 0.005                                    # 이보다 판 칸만 단단하게 (m)
SOIL = dict(kc=0.0, n=1.1, cohesion=0.0, friction_deg=30.0, janosi=0.01, elastic_k=4e7, damping_r=3e4)


def make_system():
    s = chrono.ChSystemSMC()
    s.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    s.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    s.SetNumThreads(1)
    return s


def make_terrain(s, kphi):
    t = veh.SCMTerrain(s)
    t.SetSoilParameters(kphi, SOIL["kc"], SOIL["n"], SOIL["cohesion"], SOIL["friction_deg"], SOIL["janosi"], SOIL["elastic_k"], SOIL["damping_r"])
    cy = (SCM_Y[0] + SCM_Y[1]) / 2
    t.SetReferenceFrame(chrono.ChCoordsysd(chrono.ChVector3d(0, cy, 0), chrono.QUNIT))
    t.Initialize(SCM_W, SCM_Y[1] - SCM_Y[0], DELTA)
    return t, cy


class Car:
    def __init__(self, s, terrain, y0):
        car = veh.HMMWV_Full(s)
        car.SetContactMethod(chrono.ChContactMethod_SMC)
        car.SetChassisFixed(False)
        car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, y0, 0.6), chrono.QuatFromAngleZ(math.pi / 2)))
        car.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
        car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
        car.SetDriveType(veh.DrivelineTypeWV_AWD)
        car.SetTireType(veh.TireModelType_RIGID)
        car.SetTireStepSize(STEP)
        car.Initialize()
        self.car = car
        self.path = veh.StraightLinePath(chrono.ChVector3d(0, y0, 0.5), chrono.ChVector3d(0, Y_END, 0.5), 1)
        self.driver = veh.ChPathFollowerDriver(car.GetVehicle(), self.path, "d", SPEED)
        self.driver.GetSteeringController().SetLookAheadDistance(5.0)
        self.driver.GetSteeringController().SetGains(0.8, 0, 0)
        self.driver.GetSpeedController().SetGains(0.4, 0, 0)
        self.driver.Initialize()
        terrain.AddActiveDomain(car.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

    def pos(self):
        p = self.car.GetVehicle().GetPos()
        return p.x, p.y, p.z

    def brake(self, t, terrain):
        di = veh.DriverInputs()
        di.m_throttle, di.m_steering, di.m_braking, di.m_clutch = 0.0, 0.0, 1.0, 0.0
        self.car.Synchronize(t, di, terrain)
        self.car.Advance(STEP)
        return di

    def step_in(self, t, terrain):
        v = self.car.GetVehicle()
        if v.GetPos().y > Y_END:
            return self.brake(t, terrain)
        di = self.driver.GetInputs()
        self.driver.Synchronize(t)
        self.car.Synchronize(t, di, terrain)
        self.driver.Advance(STEP)
        self.car.Advance(STEP)
        return di


def heights(terrain):
    return {(n.x, n.y): h for n, h in terrain.GetModifiedNodes(True)}


def track_profile(levels, cy):
    """바퀴 자리 단면: y마다 왼쪽(x<0), 오른쪽(x>0) 가장 낮은 높이. 반환 {y칸: (왼쪽, 오른쪽)}."""
    out = {}
    for (i, j), h in levels.items():
        side = 0 if i < 0 else 1
        cur = out.setdefault(j, [0.0, 0.0])
        cur[side] = min(cur[side], h)
    return out


class StiffRuts(veh.SoilParametersCallback):
    """칸마다 흙 성질: stiff {(i, j): kphi}에 있는 칸은 그 kphi, 나머지는 기본 흙. 답은 C++ 칸에 doublep_assign으로 (probe_scm_compaction.py와 같다)."""

    def __init__(self, stiff, kphi, cy):
        super().__init__()
        self.stiff, self.kphi, self.cy = stiff, kphi, cy

    def Set(self, loc, kphi, kc, n, coh, phi, js, ek, dr):
        k = self.stiff.get((int(round(loc.x / DELTA)), int(round((loc.y - self.cy) / DELTA))), self.kphi)
        for ptr, v in zip((kphi, kc, n, coh, phi, js, ek, dr), (k, SOIL["kc"], SOIL["n"], SOIL["cohesion"], SOIL["friction_deg"], SOIL["janosi"], SOIL["elastic_k"], SOIL["damping_r"])):
            veh.doublep_assign(ptr, float(v))


def stiff_computed(levels, kphi):
    """D 계산값: 칸마다 자국 깊이에서 kphi' (정답을 보지 않음)."""
    out = {}
    for key, h in levels.items():
        if -h > RUT_MIN:
            p = kphi * (-h) ** SOIL["n"]
            out[key] = p / (p / SOIL["elastic_k"]) ** SOIL["n"]
    return out


def stiff_mult(levels, kphi, m):
    return {key: kphi * m for key, h in levels.items() if -h > RUT_MIN}


def ghost_bodies(s, terrain, count):
    """유령 바퀴 몸체. SCM 활성 영역은 첫 스텝 전에 더해야 해서 (뒤에 더한 차는 땅에 묻혔다: 확인함) 차와 함께 처음에 만든다."""
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(0.8)
    bodies = []
    for _ in range(count):
        b = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, TIRE_R, TIRE_W, 1000, False, True, mat)
        b.SetFixed(True)
        b.SetPos(chrono.ChVector3d(0, 0, 50))
        s.Add(b)
        terrain.AddActiveDomain(b, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1.4, 0.8, 1.4))
        bodies.append(b)
    return bodies


def ghost_press(s, terrain, bodies, traj, stride, hold, with_vel=True):
    """E: 기록된 네 바퀴 궤적대로 유령 바퀴를 움직여 SCM을 누른다. 위치로 누르므로 차체 동역학이 필요 없다. 끝나면 유령을 위로 치운다.
    hold: 그동안 제동으로 세워 둘 차."""
    def place(frame):
        for b, r in zip(bodies, frame):
            b.SetPos(chrono.ChVector3d(r[0], r[1], r[2]))
            b.SetRot(chrono.ChQuaterniond(r[3], r[4], r[5], r[6]))
            if len(r) > 7 and with_vel:                    # 속도도 준다: SCM은 접촉점 속도로 미끄럼과 전단 변위를 계산한다 (위치만 옮기면 0으로 본다)
                b.SetPosDt(chrono.ChVector3d(r[7], r[8], r[9]))
                b.SetAngVelParent(chrono.ChVector3d(r[10], r[11], r[12]))
    t = s.GetChTime()
    def step(t):
        terrain.Synchronize(t)
        hold.brake(t, terrain)
        terrain.Advance(STEP)
        s.DoStepDynamics(STEP)
        return s.GetChTime()
    for k in range(0, len(traj), stride):
        place(traj[k])
        t = step(t)
    for b in bodies:                                       # 치운다 (SCM 활성 영역이 몸체를 가리키므로 지우지 않는다)
        b.SetPos(chrono.ChVector3d(0, 0, 50))
    step(t)


def run(kphi, front, rear, levels=None, until=None, stiff=None, ghost=None, stride=1, with_vel=True):
    """front / rear: 그 차를 달릴지. levels: 시작 때 넣을 자국 {(i, j): h}. stiff: 칸별 kphi (콜백). ghost: E 유령 바퀴 궤적 (스텝, 바퀴, 7).
    반환: 뒤 차 기록, 끝 높이, 앞 차 궤적 (앞 차만 달릴 때), 시간."""
    s = make_system()
    terrain, cy = make_terrain(s, kphi)
    cb = None
    if stiff:
        cb = StiffRuts(stiff, kphi, cy)                         # 함수가 끝날 때까지 살아 있어야 한다 (파이썬 객체를 SCM이 부름)
        terrain.RegisterSoilParametersCallback(cb)
    if levels:
        lst = veh.ChSCMTerrainNodeLevelList()
        for (i, j), h in levels.items():
            lst.append((chrono.ChVector2i(i, j), h))
        terrain.SetModifiedNodes(lst)
    ghost_s, after_ghost = 0.0, None
    bodies = ghost_bodies(s, terrain, ghost.shape[1]) if ghost is not None else None
    cars = []
    fc = Car(s, terrain, Y_FRONT) if front else None
    rc = Car(s, terrain, Y_FRONT - GAP) if rear else None
    if ghost is not None:                                       # 유령이 다 누를 때까지 뒤 차는 출발점에 세워 둔다
        w0 = time.perf_counter()
        ghost_press(s, terrain, bodies, ghost, stride, rc, with_vel)
        ghost_s = time.perf_counter() - w0
        after_ghost = heights(terrain)
    cars = [c for c in (fc, rc) if c is not None]
    t, log = s.GetChTime(), []
    traj = []
    wall = time.perf_counter()
    while True:
        terrain.Synchronize(t)
        inputs = [c.step_in(t, terrain) for c in cars]
        terrain.Advance(STEP)
        s.DoStepDynamics(STEP)
        t = s.GetChTime()
        if fc is not None and rc is None:                       # 앞 차 네 바퀴 궤적 (E 가상 다짐이 받는 것)
            v = fc.car.GetVehicle()
            frame = []
            for axle in (0, 1):
                for side in (veh.LEFT, veh.RIGHT):
                    p_, q_ = v.GetSpindlePos(axle, side), v.GetSpindleRot(axle, side)
                    u_, w_ = v.GetSpindleLinVel(axle, side), v.GetSpindleAngVel(axle, side)
                    frame.append((p_.x, p_.y, p_.z, q_.e0, q_.e1, q_.e2, q_.e3, u_.x, u_.y, u_.z, w_.x, w_.y, w_.z))
            traj.append(frame)
        if rc is not None:
            x, y, z = rc.pos()
            if WINDOW[0] <= y <= WINDOW[1]:
                v = rc.car.GetVehicle()
                q = v.GetRot()
                rot = chrono.ChMatrix33d(q)
                # 차체 축: x = 앞. 앞뒤 기울기 = 앞 축의 z 성분, 옆 기울기 = 옆 축의 z 성분
                fwd, left = rot.GetAxisX(), rot.GetAxisY()
                wz = [v.GetSpindlePos(a, side).z for a in (0, 1) for side in (veh.LEFT, veh.RIGHT)]
                log.append((t, x, y, z, math.degrees(math.asin(fwd.z)), math.degrees(math.asin(left.z)), np.mean(wz), v.GetSpeed(),
                            inputs[-1].m_throttle, inputs[-1].m_braking))
            if y > WINDOW[1] + 3.0:
                break
        elif fc.pos()[1] > (until if until is not None else Y_END):
            break
        if t > 60:
            raise RuntimeError(f"차가 {t:.0f} s 동안 끝에 닿지 못했다: 앞 {fc.pos() if fc else None}, 뒤 {rc.pos() if rc else None}")
    return {"log": np.array(log) if log else None, "final": heights(terrain), "cy": cy, "wall": time.perf_counter() - wall, "t": t,
            "traj": np.array(traj) if traj else None, "ghost_s": ghost_s, "after_ghost": after_ghost}


def window_stats(res, front_levels):
    """가운데 구간의 수치. front_levels: 뒤 차가 오기 전 자국 (없으면 평지). 깊이 단위 cm, 각도 deg."""
    log, cy = res["log"], res["cy"]
    j0, j1 = int(round((WINDOW[0] - cy) / DELTA)), int(round((WINDOW[1] - cy) / DELTA))
    pf = track_profile(res["final"], cy)
    pb = track_profile(front_levels or {}, cy)
    final, added = [], []
    bins = {}
    for j in range(j0, j1 + 1):
        a = pf.get(j, [0.0, 0.0])
        b = pb.get(j, [0.0, 0.0])
        for k in (0, 1):
            final.append(a[k])
            added.append(b[k] - a[k])
            bins.setdefault(int((j - j0) * DELTA // BIN), []).append(b[k] - a[k])
    bm = [100 * float(np.mean(v)) for v in bins.values()]
    return {"final_depth_cm": -100 * float(np.mean(final)), "added_sink_cm": 100 * float(np.mean(added)), "added_sink_std_cm": 100 * float(np.std(added)),
            "added_sink_bin_min_cm": min(bm), "added_sink_bin_max_cm": max(bm),
            "spindle_z_m": float(log[:, 6].mean()), "chassis_z_m": float(log[:, 3].mean()), "pitch_rms_deg": float(np.std(log[:, 4])), "roll_rms_deg": float(np.std(log[:, 5])),
            "lateral_m": float(np.abs(log[:, 1]).mean()), "speed_mps": float(log[:, 7].mean()), "throttle": float(log[:, 8].mean())}


ROWS = [("추가 침하 (cm)", "added_sink_cm", 2), ("끝 자국 깊이 (cm)", "final_depth_cm", 2), ("바퀴 중심 높이 (m)", "spindle_z_m", 4), ("차체 높이 (m)", "chassis_z_m", 4),
        ("앞뒤 기울기 흔들림 (deg)", "pitch_rms_deg", 3), ("옆 기울기 흔들림 (deg)", "roll_rms_deg", 3), ("옆 벗어남 (m)", "lateral_m", 3), ("속도 (m/s)", "speed_mps", 3), ("스로틀 (0~1)", "throttle", 3)]


def main():
    ap = argparse.ArgumentParser(description="HMMWV 두 대 기준 측정")
    ap.add_argument("--kphi", type=float, nargs="+", default=[5e6, 1e6])
    ap.add_argument("--mults", type=float, nargs="*", default=[8], help="D 배수")
    ap.add_argument("--stride", type=int, nargs="*", default=[1, 5], help="E 유령 궤적을 몇 스텝마다 쓸지")
    ap.add_argument("--record")
    a = ap.parse_args()
    analysis = {}
    for kphi in a.kphi:
        t0 = time.perf_counter()
        ca = run(kphi, True, True)                              # A 이어 지나감
        fa = run(kphi, True, False)                             # 앞 차 혼자: B에 넣을 자국
        cb = run(kphi, False, True, levels=fa["final"])         # B 높이만
        cc = run(kphi, False, True)                             # C 평지
        sa = window_stats(ca, fa["final"])                      # 뒤 차가 오기 전 자국 = 앞 차 혼자 자국 (A의 앞 차 자국과 평균 0.07 cm 차이)
        sb = window_stats(cb, fa["final"])
        sc = window_stats(cc, None)
        analysis[f"{kphi:g}"] = {"A": sa, "B": sb, "C": sc}
        comp = stiff_computed(fa["final"], kphi)
        rows_d = [("D 계산값", window_stats(run(kphi, False, True, levels=fa["final"], stiff=comp), fa["final"]))]
        for m in a.mults:
            rows_d.append((f"D x{m:g}", window_stats(run(kphi, False, True, levels=fa["final"], stiff=stiff_mult(fa["final"], kphi, m)), fa["final"])))
        analysis[f"{kphi:g}"]["D"] = {name: st for name, st in rows_d}
        analysis[f"{kphi:g}"]["computed_kphi"] = {"n": len(comp), "median": float(np.median(list(comp.values())))}
        rows_e = []
        for st_, vel in [(x, True) for x in a.stride] + [(a.stride[0], False)]:
            ce = run(kphi, False, True, ghost=fa["traj"], stride=st_, with_vel=vel)
            g = ce["after_ghost"]
            keys = set(g) | set(fa["final"])
            j0, j1 = int(round((WINDOW[0] - ce["cy"]) / DELTA)), int(round((WINDOW[1] - ce["cy"]) / DELTA))
            dh = np.array([100 * (g.get(k_, 0.0) - fa["final"].get(k_, 0.0)) for k_ in keys if j0 <= k_[1] <= j1])   # 구간 안 (뒤 차 출발점은 밖)
            se = window_stats(ce, g)
            se.update(ghost_s=ce["ghost_s"], ghost_rut_diff_mean_cm=float(np.abs(dh).mean()), ghost_rut_diff_max_cm=float(np.abs(dh).max()), ghost_rut_bias_cm=float(dh.mean()))
            rows_e.append((f"E 1/{st_}" + ("" if vel else " 위치만"), se))
        analysis[f"{kphi:g}"]["E"] = {name: st for name, st in rows_e}
        print(f"\n== kphi {kphi:g}: 뒤 차가 구간 y {WINDOW[0]:g} ~ {WINDOW[1]:g} m를 지날 때 ({time.perf_counter() - t0:.0f} s)", flush=True)
        print(f"{'':28s}{'A 이어 (정답)':>14s}{'B 높이만':>12s}{'C 평지':>12s}{'B - A':>10s}")
        for name, k, dg in ROWS:
            print(f"{name:28s}{sa[k]:14.{dg}f}{sb[k]:12.{dg}f}{sc[k]:12.{dg}f}{sb[k] - sa[k]:10.{dg}f}")
        print(f"\nD 단단한 흙 (자국 칸 {len(comp)}개, 계산값 kphi 중앙값 {np.median(list(comp.values())):.3g} = 기본의 {np.median(list(comp.values())) / kphi:.1f}배)")
        print(f"{'':12s}{'추가 침하':>10s}{'A와 차이':>10s}{'끝 깊이':>9s}{'A와 차이':>10s}{'바퀴 높이':>10s}{'A와 차이':>10s}{'스로틀':>8s}{'A와 차이':>10s}")
        for name, st in [("A 정답", sa), ("B 높이만", sb), ("C 평지", sc)] + rows_d + rows_e:
            print(f"{name:12s}{st['added_sink_cm']:10.2f}{st['added_sink_cm'] - sa['added_sink_cm']:+10.2f}{st['final_depth_cm']:9.2f}{st['final_depth_cm'] - sa['final_depth_cm']:+10.2f}"
                  f"{st['spindle_z_m'] * 100:10.2f}{(st['spindle_z_m'] - sa['spindle_z_m']) * 100:+10.2f}{st['throttle']:8.3f}{st['throttle'] - sa['throttle']:+10.3f}")
        print(f"A의 추가 침하, {BIN:g} m 칸별 범위: {sa['added_sink_bin_min_cm']:.2f} ~ {sa['added_sink_bin_max_cm']:.2f} cm (이보다 작은 차이는 구별 못 한다)")
        print(f"앞 차 혼자 달린 시간 {fa['wall']:.1f} s (시뮬레이션 {fa['t']:.1f} s)")
        for name, st in rows_e:
            print(f"{name}: 유령이 만든 자국과 실제 자국 차이 평균 {st['ghost_rut_diff_mean_cm']:.3f}, 최대 {st['ghost_rut_diff_max_cm']:.2f}, 치우침 {st['ghost_rut_bias_cm']:+.3f} cm, "
                  f"다시 누른 시간 {st['ghost_s']:.1f} s, 옆 벗어남 {st['lateral_m'] * 100:.1f} cm, 옆 흔들림 {st['roll_rms_deg']:.3f}")
        print(f"(A 옆 벗어남 {sa['lateral_m'] * 100:.1f} cm, 옆 흔들림 {sa['roll_rms_deg']:.3f})")
        if sa["added_sink_cm"] > 1e-3:
            print(f"추가 침하 비 B / A = {sb['added_sink_cm'] / sa['added_sink_cm']:.1f}배, 평지 첫 지나감(앞 차 자국 깊이 대신 C) = {sc['added_sink_cm']:.2f} cm")
    if a.record:
        scm_record.save(a.record, "two_hmmwv", {"gap": GAP, "speed": SPEED, "window": WINDOW, "soil": SOIL, "kphi": a.kphi, "delta": DELTA}, analysis)
        print("기록:", a.record)
    print("\n" + json.dumps({"ok": True}))


if __name__ == "__main__":
    main()
