"""교차 시험: 앞 차 자국을 다른 차가 90도로 건넌다 (맵 서버 4단계, 고스트 방식의 약점 찾기).

    python conformance/probe_cross.py [--kphi 5e6 1e6] [--starts -24.1 -24 -23.9]      (mj_ros. Chrono 프로세스는 chrono 환경으로 띄운다)

  앞 차: x = 0을 y 방향으로. 교차 차량: y = 0을 x 방향으로, 앞 차가 지나간 뒤 건넌다.
  넷을 비교한다 (건너는 구간 x -6 ~ 6 m):
    정답         한 SCM에서 두 대
    두 프로세스   앞 차와 교차 차량이 별도 프로세스. 앞 차 궤적을 고스트로 재생, 맵 서버로 주고받음
    높이만       앞 차의 끝 높이를 교차 차량 SCM에 넣음 (다짐 없음)
    자국 없음     교차 차량 혼자
"""
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "conformance"))
import probe_two_chrono as T  # noqa: E402

X_END = 8.0                                        # 교차 차량이 멈추기 시작하는 x (m)
WIN = (-6.0, 6.0)                                  # 재는 구간 (교차 차량 중심 x)
CELLS = (6.0, 4.0)                                 # 지도 비교 영역 반폭 (x, y m)
CROSS = 1.5                                        # 두 자국이 만나는 칸 반폭 (m)


def setup(P):
    P.Y_END, P.SCM_Y, P.SCM_W = 18.0, (-20.0, 22.0), 56.0
    return P


def cross_car(P, s, ter, a):
    return P.Car(s, ter, 0.0, a.start, heading=0.0, length=X_END - a.start)


def row(P, car, t, di):
    """시각, x, y, z, 앞뒤 기울기, 옆 기울기, 방향, 속도, 스로틀, 조향, 수직 속도, 바퀴 중심 높이 평균."""
    v = car.car.GetVehicle()
    p = v.GetPos()
    rot = P.chrono.ChMatrix33d(v.GetRot())
    fwd, left = rot.GetAxisX(), rot.GetAxisY()
    wz = [v.GetSpindlePos(a, side).z for a in (0, 1) for side in (P.veh.LEFT, P.veh.RIGHT)]
    return (t, p.x, p.y, p.z, math.degrees(math.asin(fwd.z)), math.degrees(math.asin(left.z)), math.degrees(math.atan2(fwd.y, fwd.x)), v.GetSpeed(),
            di.m_throttle, di.m_steering, car.car.GetChassisBody().GetPosDt().z, float(np.mean(wz)))


def final_heights(ter):
    return {(n.x, n.y): v for n, v in ter.GetModifiedNodes(True)}


def role_front(a):
    import probe_scm_two_hmmwv as P
    setup(P)
    m, to_cells, _, _ = T.client(P, "front", a)
    s = P.make_system()
    ter, _ = P.make_terrain(s, a.kphi)
    car = P.Car(s, ter, P.Y_FRONT)
    every = max(1, int(round(1.0 / (a.hz * P.STEP))))
    sent, t, k = {}, 0.0, 0
    t0 = time.monotonic()
    while not any(tm["writer"] == "rear" for tm in m.traj_in):
        m.poll()
        time.sleep(0.01)
        if time.monotonic() - t0 > 120:
            raise RuntimeError("교차 차량이 붙지 않는다")
    tire = (P.TIRE_R, P.TIRE_W)
    wall = time.perf_counter()
    t_pass = None
    while car.pos()[1] <= P.Y_END:
        ter.Synchronize(t)
        car.step_in(t, ter)
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        if t_pass is None and car.pos()[1] > 0:
            t_pass = t
        if k % every == 0:
            m.send_traj("front", k, every, [T.wheel_frame(P, car)], tire)
        if (k + 1) % T.WRITE_EVERY == 0:
            r, c, h = to_cells(*T.changed_nodes(ter, sent))
            if r.size:
                m.write_cells("dig", "front", r, c, h, t=t)
            m.poll()
        k += 1
    r, c, h = to_cells(*T.changed_nodes(ter, sent))
    if r.size:
        m.write_cells("dig", "front", r, c, h, t=t)
    m.send_traj("front", k, every, np.zeros((0, 4, 13)), tire, end=True)
    wall = time.perf_counter() - wall
    au = m.audit(wait_s=120.0)
    T.save(a.out, heights=final_heights(ter), wall=wall, sim=t, audit=json.dumps(au), t_pass=t_pass)
    m.close()


def role_rear(a):
    from sim.map_client import dec
    import probe_scm_two_hmmwv as P
    setup(P)
    m, to_cells, i0, j0 = T.client(P, "rear", a)
    s = P.make_system()
    ter, _ = P.make_terrain(s, a.kphi)
    bodies = P.ghost_bodies(s, ter, 4)
    car = cross_car(P, s, ter, a)
    delay = int(round(a.delay / P.STEP))
    frames, ended, every = [], False, None
    sent, log, t, k = {}, [], 0.0, 0
    ghost_on, waited = True, 0.0
    m.send_traj("rear", 0, 1, np.zeros((0, 4, 13)), (P.TIRE_R, P.TIRE_W))

    def receive():
        nonlocal ended, every
        m.poll()
        for tm in m.traj_in:
            every = tm["every"]
            if tm["end"]:
                ended = True
            elif tm["data"]:
                if tm["k0"] != len(frames) * every:
                    raise RuntimeError(f"궤적 프레임이 빠졌다: k0 {tm['k0']}, 받은 {len(frames)}개 x {every}")
                frames.extend(dec(tm["data"], tuple(tm["shape"])))
        m.traj_in.clear()

    def write():
        r, c, h = to_cells(*T.changed_nodes(ter, sent))
        if r.size:
            low = h < m.local.compose(0, m.local.nrow, 0, m.local.ncol)[r, c] - T.THRESH
            if low.any():
                m.write_cells("dig", "rear", r[low], c[low], h[low], t=t)

    wall = time.perf_counter()
    while True:
        if ghost_on:
            idx = max(0, k - delay)
            w0 = time.perf_counter()
            while True:
                receive()
                if every is not None and (ended or idx // every + 1 < len(frames)):
                    break
                time.sleep(0.0005)
            waited += time.perf_counter() - w0
            a0 = idx // every
            if a0 + 1 >= len(frames):
                for b in bodies:
                    b.SetPos(P.chrono.ChVector3d(0, 0, 50))
                    b.SetPosDt(P.chrono.ChVector3d(0, 0, 0))
                    b.SetAngVelParent(P.chrono.ChVector3d(0, 0, 0))
                ghost_on = False
            else:
                f = (idx - a0 * every) / every
                fr = (1 - f) * frames[a0] + f * frames[a0 + 1]
                fr[:, 3:7] = fr[:, 3:7] / np.linalg.norm(fr[:, 3:7], axis=1, keepdims=True)
                P.place_ghosts(bodies, fr)
        ter.Synchronize(t)
        di = car.step_in(t, ter)
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        log.append(row(P, car, t, di))
        if (k + 1) % T.WRITE_EVERY == 0:
            write()
        k += 1
        if car.pos()[0] > WIN[1] + 1.0:
            break
    write()
    wall = time.perf_counter() - wall
    au = m.audit(wait_s=120.0)
    T.save(a.out, heights=final_heights(ter), wall=wall, sim=t, audit=json.dumps(au), log=np.array(log), final_map=m.local.compose(0, m.local.nrow, 0, m.local.ncol),
           waited=waited, i0=i0, j0=j0)
    m.close()


def role_solo(a):
    """한 SCM: truth (두 대), flat (교차 차량만), heights (앞 차 끝 높이를 넣고 교차 차량만)."""
    import probe_scm_two_hmmwv as P
    setup(P)
    s = P.make_system()
    ter, _ = P.make_terrain(s, a.kphi)
    if a.mode == "heights":
        _, fh = T.load(a.front)
        lst = P.veh.ChSCMTerrainNodeLevelList()
        for (i, j), h in fh.items():
            lst.append((P.chrono.ChVector2i(i, j), h))
        ter.SetModifiedNodes(lst)
    fc = P.Car(s, ter, P.Y_FRONT) if a.mode == "truth" else None
    car = cross_car(P, s, ter, a)
    log, t, gap, t_pass = [], s.GetChTime(), float("inf"), None
    wall = time.perf_counter()
    while True:
        ter.Synchronize(t)
        if fc is not None:
            fc.step_in(t, ter)
        di = car.step_in(t, ter)
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        log.append(row(P, car, t, di))
        if fc is not None:
            fx, fy, _ = fc.pos()
            cx, cy_, _ = car.pos()
            gap = min(gap, math.hypot(fx - cx, fy - cy_))
            if t_pass is None and fy > 0:
                t_pass = t
        if car.pos()[0] > WIN[1] + 1.0:
            break
    T.save(a.out, heights=final_heights(ter), log=np.array(log), wall=time.perf_counter() - wall, sim=t, min_gap=gap, t_pass=t_pass or -1.0)


def stats(log):
    """건너는 구간의 값. 각도 deg, 가속도 m/s^2, 속도 m/s, 옆 위치 cm."""
    sel = (log[:, 1] >= WIN[0]) & (log[:, 1] <= WIN[1])
    lg = log[sel]
    vz = np.convolve(lg[:, 10], np.ones(25) / 25, mode="valid")              # 50 ms 평균 뒤 미분 (접촉 잡음 빼기)
    az = np.diff(vz) / np.diff(lg[:len(vz), 0])
    entry = lg[lg[:, 1] < WIN[0] + 1.0]
    exit_ = lg[lg[:, 1] > WIN[1] - 1.0]
    return {"앞뒤 기울기 폭": float(lg[:, 4].max() - lg[:, 4].min()), "옆 기울기 최대": float(np.abs(lg[:, 5]).max()), "수직 가속도 최대": float(np.abs(az).max()),
            "속도 손실": float(entry[:, 7].mean() - lg[:, 7].min()), "방향 변화 최대": float(np.abs(lg[:, 6]).max()), "나갈 때 옆 위치": 100 * float(exit_[:, 2].mean()),
            "구간 스로틀": float(lg[:, 8].mean())}


def cell_diff(a_h, b_h, x, y, res=0.05):
    ix, iy = int(round(x / res)), int(round(y / res))
    cy = 1.0
    jc = int(round(-cy / res))
    out = []
    for i in range(-ix, ix + 1):
        for j in range(jc - iy, jc + iy + 1):
            u, v = a_h.get((i, j), 0.0), b_h.get((i, j), 0.0)
            if abs(u) > 0.002 or abs(v) > 0.002:
                out.append(100 * (v - u))
    return np.array(out)


def rng(vals, f=".2f"):
    return f"{min(vals):{f}} ~ {max(vals):{f}}"


def report(res, starts, kphi):
    print(f"\n== kphi {kphi:g}: 교차 차량 값, x {WIN[0]:g} ~ {WIN[1]:g} m (출발 {starts})")
    names = ["truth", "rear", "heights", "flat"]
    labels = {"truth": "정답 (한 SCM)", "rear": "두 프로세스", "heights": "높이만", "flat": "자국 없음"}
    keys = list(stats(res[starts[0]]["truth"][0]).keys())
    for k in keys:
        print(f"  {k:12s}" + "  ".join(f"{labels[n]} {rng([stats(res[g][n][0])[k] for g in starts])}" for n in names))
    gs = list(starts)
    for lab, (x, y) in (("주변 영역", CELLS), ("만나는 칸", (CROSS, CROSS))):
        tp = [cell_diff(res[p]["truth"][1], res[q]["truth"][1], x, y) for ia, p in enumerate(gs) for q in gs[ia + 1:]]
        mp = [cell_diff(res[g]["truth"][1], res[g]["map"], x, y) for g in gs]
        fmt = lambda ds: f"|평균| {rng([float(np.abs(d).mean()) for d in ds], '.3f')}, 평균 {rng([float(d.mean()) for d in ds], '+.3f')}, 최대 {max(float(np.abs(d).max()) for d in ds):.2f}" if ds else "없음"  # noqa: E731
        print(f"  {lab} 칸별 (cm): 정답끼리 {fmt(tp)}")
        print(f"  {lab} 칸별 (cm): 지도-정답 {fmt(mp)}")
    tr = res[gs[0]]["truth"]
    x_t = tr[0][:, 1]
    print(f"  앞 차가 y = 0을 지난 때 t = {tr[2]:.1f} s, 교차 차량이 x = -3 m를 지난 때 t = {tr[0][np.searchsorted(x_t, -3.0), 0]:.1f} s, 두 차 최소 거리 {tr[3]:.1f} m")
    au = [res[g]["audit"] for g in gs]
    print(f"  해시 불일치: {[x.get('hash_mismatch', x) for x in au]}, 교차 차량 빠르기 {rng([res[g]['speed'] for g in gs])}배")


def orchestrate(a):
    try:
        import rclpy  # noqa: F401
    except ImportError:
        raise SystemExit("맵 서버에 ROS2가 필요하다: conda activate mj_ros 후 실행")
    chrono_py = str(Path.home() / "miniconda3/envs/chrono/bin/python")
    me = [chrono_py, str(Path(__file__).resolve())]
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH")}
    for kphi in a.kphi:
        res = {}
        with tempfile.TemporaryDirectory(prefix="cross_") as tmp:
            dirn = Path(tmp)
            for g in a.starts:
                so = socket.socket()
                so.bind(("127.0.0.1", 0))
                port = so.getsockname()[1]
                so.close()
                server = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--tcp-port", str(port)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                common = ["--kphi", str(kphi), "--start", str(g), "--hz", str(a.hz), "--delay", str(a.delay)]
                tcp = ["--tcp", f"127.0.0.1:{port}"]
                try:
                    front = subprocess.Popen(me + ["--role", "front", "--out", str(dirn / f"front_{g}.npz")] + common + tcp, env=env, cwd=ROOT)
                    time.sleep(1.0)
                    rear = subprocess.Popen(me + ["--role", "rear", "--out", str(dirn / f"rear_{g}.npz")] + common + tcp, env=env, cwd=ROOT)
                    for name, p in (("교차 차량", rear), ("앞 차", front)):
                        if p.wait(timeout=1800) != 0:
                            raise SystemExit(f"{name} 프로세스가 실패했다")
                finally:
                    server.terminate()
                    server.wait(timeout=10)
                for mode in ("truth", "flat", "heights"):
                    subprocess.run(me + ["--role", "solo", "--mode", mode, "--front", str(dirn / f"front_{g}.npz"), "--out", str(dirn / f"{mode}_{g}.npz")] + common, env=env, cwd=ROOT, check=True, timeout=1800)
                rd, _ = T.load(dirn / f"rear_{g}.npz")
                final, i0, j0 = rd["final_map"], int(rd["i0"]), int(rd["j0"])
                mp = {(int(i - i0), int(j - j0)): float(final[j, i]) for j, i in zip(*np.nonzero(final))}
                res[g] = {"rear": (rd["log"],), "map": mp, "audit": json.loads(str(rd["audit"])), "speed": float(rd["sim"]) / float(rd["wall"])}
                for mode in ("truth", "flat", "heights"):
                    d, h = T.load(dirn / f"{mode}_{g}.npz")
                    res[g][mode] = (d["log"], h, float(d["t_pass"]), float(d["min_gap"]))
                print(f"[kphi {kphi:g} 출발 {g} 끝]", flush=True)
        report(res, a.starts, kphi)


def main():
    ap = argparse.ArgumentParser(description="앞 차 자국을 90도로 건너기")
    ap.add_argument("--kphi", type=float, nargs="+", default=[5e6, 1e6])
    ap.add_argument("--starts", type=float, nargs="+", default=[-24.1, -24.0, -23.9], help="교차 차량 출발 x (m)")
    ap.add_argument("--start", type=float)
    ap.add_argument("--hz", type=float, default=25.0)
    ap.add_argument("--delay", type=float, default=0.2)
    ap.add_argument("--role", choices=("front", "rear", "solo"))
    ap.add_argument("--mode", choices=("truth", "flat", "heights"))
    ap.add_argument("--front")
    ap.add_argument("--tcp")
    ap.add_argument("--out")
    a = ap.parse_args()
    if a.role is None:
        return orchestrate(a)
    a.kphi, a.lane = a.kphi[0], None
    {"front": role_front, "rear": role_rear, "solo": role_solo}[a.role](a)


if __name__ == "__main__":
    main()
