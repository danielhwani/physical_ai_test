"""뒤 차 한 프로세스의 빠르기: 저장한 앞 차 궤적을 고스트로 재생하며 어디에 시간을 쓰는지 잰다 (지도와 통신 없음). chrono 환경에서 저장소 맨 위에서 돌린다.

    python conformance/probe_rear_speed.py --traj /tmp/t.npy --record                                     앞 차 궤적을 만든다
    python conformance/probe_rear_speed.py --traj /tmp/t.npy --gdomain 1.1 0.4 1.1 --wdomain 1.1 0.5 1.1 --threads 2
    python conformance/probe_rear_speed.py --traj /tmp/t.npy --steps 9000 --log /tmp/a.npy ...             스텝별 기록을 남긴다
    python conformance/probe_rear_speed.py --compare /tmp/a.npy /tmp/b.npy                                 두 기록이 어디서 처음 갈라지나 (활성 영역이 접촉을 다 덮는지 확인)
"""
import argparse, hashlib, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, "conformance")
import probe_scm_two_hmmwv as P
ap = argparse.ArgumentParser()
ap.add_argument("--kphi", type=float, default=1e6)
ap.add_argument("--threads", type=int, default=1)
ap.add_argument("--ghost", type=int, default=1)
ap.add_argument("--gdomain", type=float, nargs=3, default=[1.4, 0.8, 1.4])
ap.add_argument("--cdomain", type=float, nargs=3, default=[5.0, 3.0, 1.0])
ap.add_argument("--wdomain", type=float, nargs=3)
ap.add_argument("--route", default="straight")
ap.add_argument("--log")
ap.add_argument("--steps", type=int, default=3000)
ap.add_argument("--traj")
ap.add_argument("--compare", nargs=2)
ap.add_argument("--record", action="store_true")
a = ap.parse_args()
if a.compare:
    la, lb = np.load(a.compare[0]), np.load(a.compare[1])
    n = min(len(la), len(lb))
    d = np.abs(la[:n, :3] - lb[:n, :3]).max(axis=1)
    dh = np.abs(la[:n, 3] - lb[:n, 3])
    first = int(np.argmax(d > 0)) if (d > 0).any() else -1
    print(f"위치 처음 다른 스텝 {first}" + (f" (y {la[first, 1]:.2f}, 차이 {d[first]:.1e} m)" if first >= 0 else "") + f", 최대 {d.max() * 100:.3f} cm | 맞은 칸 수가 다른 스텝 {int((dh > 0).sum())}개, 최대 {dh.max():.0f}칸")
    raise SystemExit
P.Y_END, P.SCM_Y = 26.0, (-30.0, 36.0)
P.THREADS, P.GHOST_DOMAIN, P.CAR_DOMAIN = a.threads, tuple(a.gdomain), tuple(a.cdomain)
P.WHEEL_DOMAIN = tuple(a.wdomain) if a.wdomain else None
P.ROUTE = a.route
EVERY = 20
def wheel_frame(car):
    veh, v = P.veh, car.car.GetVehicle()
    out = []
    for axle in (0, 1):
        for side in (veh.LEFT, veh.RIGHT):
            p, q, u, w = v.GetSpindlePos(axle, side), v.GetSpindleRot(axle, side), v.GetSpindleLinVel(axle, side), v.GetSpindleAngVel(axle, side)
            out.append((p.x, p.y, p.z, q.e0, q.e1, q.e2, q.e3, u.x, u.y, u.z, w.x, w.y, w.z))
    return out
s = P.make_system()
ter, cy = P.make_terrain(s, a.kphi)
if a.record:
    car = P.Car(s, ter, P.Y_FRONT)
    fr, t, k = [], 0.0, 0
    while k < 10000:
        ter.Synchronize(t); car.step_in(t, ter); ter.Advance(P.STEP); s.DoStepDynamics(P.STEP); t = s.GetChTime()
        if k % EVERY == 0:
            fr.append(wheel_frame(car))
        k += 1
    np.save(a.traj, np.array(fr))
    print("기록", len(fr), "프레임, 앞 차 y", car.pos()[1])
    raise SystemExit
traj = np.load(a.traj)
bodies = P.ghost_bodies(s, ter, 4) if a.ghost else []
car = P.Car(s, ter, P.Y_FRONT - 12.0)
t, h = 0.0, hashlib.sha1()
DELAY = 500
START = 500
tm = dict(step=0.0, coll=0.0, ray_test=0.0, ray_cast=0.0, patch=0.0, force=0.0, dom=0.0, hits=0, casts=0, place=0.0, drv=0.0)
w0 = None
LOG = []
for k in range(START + a.steps):
    if k == START:
        w0 = time.perf_counter()
    c0 = time.perf_counter()
    if bodies:
        i = max(0, k - DELAY)
        q, f = divmod(i, EVERY)
        f /= EVERY
        r = (1 - f) * traj[q] + f * traj[q + 1]
        r[:, 3:7] /= np.linalg.norm(r[:, 3:7], axis=1, keepdims=True)
        P.place_ghosts(bodies, r)
    c1 = time.perf_counter()
    ter.Synchronize(t)
    car.step_in(t, ter)
    ter.Advance(P.STEP)
    c2 = time.perf_counter()
    s.DoStepDynamics(P.STEP)
    t = s.GetChTime()
    x, y, z = car.pos()
    h.update(np.array([x, y, z]).tobytes())
    LOG.append((x, y, z, ter.GetNumRayHits(), ter.GetNumContactPatches()))
    if k >= START:
        tm["place"] += c1 - c0
        tm["drv"] += c2 - c1
        tm["step"] += s.GetTimerStep()
        tm["coll"] += s.GetTimerCollision()
        tm["ray_test"] += ter.GetTimerRayTesting()
        tm["ray_cast"] += ter.GetTimerRayCasting()
        tm["patch"] += ter.GetTimerContactPatches()
        tm["force"] += ter.GetTimerContactForces()
        tm["dom"] += ter.GetTimerActiveDomains()
        tm["hits"] += ter.GetNumRayHits()
        tm["casts"] += ter.GetNumRayCasts()
wall = time.perf_counter() - w0
if a.log:
    np.save(a.log, np.array(LOG))
sim = a.steps * P.STEP
n = a.steps
print(f"kphi {a.kphi:g} 스레드 {a.threads} 고스트 {a.ghost} 고스트영역 {a.gdomain[:2]} 차영역 {a.wdomain or a.cdomain[:2]}: {wall:6.2f} s = {sim / wall:.2f}배 | 스텝당 ms: 전체 {1000 * wall / n:.2f}, 물리 {1000 * tm['step'] / n:.2f}, 충돌 {1000 * tm['coll'] / n:.2f}, "
      f"SCM 영역 {tm['dom'] / n:.3f}, 시험 {tm['ray_test'] / n:.3f}, 레이 {tm['ray_cast'] / n:.3f}, 패치 {tm['patch'] / n:.3f}, 힘 {tm['force'] / n:.3f}, 고스트 놓기 {1000 * tm['place'] / n:.3f}, 차 입력 {1000 * tm['drv'] / n:.3f} | 레이 {tm['casts'] // n}, 맞음 {tm['hits'] // n} | y {y:.2f}, {h.hexdigest()[:10]}", flush=True)
