"""맵 서버 4단계 시제품: Chrono 프로세스 둘이 맵 서버로 자국과 바퀴 궤적을 주고받아, 뒤 차가 앞 차 자국을 밟는다 (docs/map_server.md 8절, 4단계).
시뮬레이터 없이 맵 서버와 Chrono 프로세스만. 정답은 한 SCM에서 두 대를 달린 것 (conformance/probe_scm_two_hmmwv.py의 A).

    python conformance/probe_two_chrono.py [--kphi 5e6 1e6] [--hz 25] [--delay 0.2]       (mj_ros 환경. Chrono 프로세스는 chrono 환경으로 띄운다)
    python conformance/probe_two_chrono.py --spread 11.8 11.9 12.0 12.1 12.2              뒤 차 출발 위치별로 정답끼리의 칸별 범위와 두 프로세스를 비교

  앞 차 프로세스: 자기 SCM에서 달리며 바뀐 노드를 dig로, 네 바퀴 궤적을 traj로 맵 서버(TCP)에 보낸다.
  뒤 차 프로세스: 받은 궤적을 고스트 바퀴로 재생해 자기 SCM을 누르고 (높이는 넣지 않는다), 뒤 차를 달리며 지도보다 THRESH 넘게 낮아진 칸만 dig로 보낸다.
                  궤적이 아직 안 왔으면 물리를 멈추고 기다린다 (시각을 맞추려고. 실시간 동작이 아니다).
  지도: 소유자 없이 공유 영역. 칸마다 더 낮은 값이 남는다 (dig = min).
  비교: 뒤 차의 침하, 바퀴 높이, 스로틀, 옆 벗어남을 정답과. 끝난 지도 높이를 정답 SCM의 끝 높이와. 두 클라이언트와 서버의 해시 불일치.
"""
import argparse
import json
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
THRESH = 0.001                                     # 뒤 차: 지도보다 이만큼 더 낮아진 칸만 보낸다 (m)
Z_RANGE = (-0.5, 0.5)
WRITE_EVERY = 100                                  # 지도에 쓰는 주기 (물리 스텝, 0.2 s)


def setup(P):
    """앞 차가 멀리 선다 (뒤 차가 구간을 다 지나도 부딪히지 않게: 기본 길이면 끝에서 중심 거리 4.1 m)."""
    P.Y_END, P.SCM_Y = 26.0, (-30.0, 36.0)
    return P


def grid_shape(P):
    """SCM 노드 (i, j) -> 지도 칸 (행 j + J0, 열 i + I0)."""
    i0, j0 = int(round(P.SCM_W / 2 / P.DELTA)), int(round((P.SCM_Y[1] - P.SCM_Y[0]) / 2 / P.DELTA))
    return 2 * j0 + 1, 2 * i0 + 1, i0, j0


def changed_nodes(terrain, sent):
    i, j, h = [], [], []
    for n, v in terrain.GetModifiedNodes(True):
        if sent.get((n.x, n.y)) != v:
            sent[(n.x, n.y)] = v
            i.append(n.x), j.append(n.y), h.append(v)
    return np.array(i, int), np.array(j, int), np.array(h, float)


def client(P, name, tcp):
    from sim.map_client import RemoteMap
    nrow, ncol, i0, j0 = grid_shape(P)
    return RemoteMap(np.zeros((nrow, ncol)), Z_RANGE, name=name, tcp=tcp), i0, j0


def wheel_frame(P, car):
    veh, v = P.veh, car.car.GetVehicle()
    out = []
    for axle in (0, 1):
        for side in (veh.LEFT, veh.RIGHT):
            p, q, u, w = v.GetSpindlePos(axle, side), v.GetSpindleRot(axle, side), v.GetSpindleLinVel(axle, side), v.GetSpindleAngVel(axle, side)
            out.append((p.x, p.y, p.z, q.e0, q.e1, q.e2, q.e3, u.x, u.y, u.z, w.x, w.y, w.z))
    return out


def save(path, **kw):
    heights = kw.pop("heights")
    arr = np.array([(i, j, h) for (i, j), h in heights.items()], float) if heights else np.zeros((0, 3))
    np.savez(path, heights=arr, **kw)


def role_front(a):
    import probe_scm_two_hmmwv as P
    setup(P)
    m, i0, j0 = client(P, "front", a.tcp)
    s = P.make_system()
    ter, cy = P.make_terrain(s, a.kphi)
    car = P.Car(s, ter, P.Y_FRONT)
    every = max(1, int(round(1.0 / (a.hz * P.STEP))))
    sent, t, k = {}, 0.0, 0
    t_wait = time.monotonic()
    while "rear" not in (m.info or {}).get("joined", []):       # 뒤 차가 붙을 때까지 출발하지 않는다 (붙기 전 궤적은 받지 못한다: 늦게 붙기는 아직 없다)
        m.poll()
        time.sleep(0.01)
        if time.monotonic() - t_wait > 120:
            raise RuntimeError("뒤 차가 붙지 않는다")
    wall = time.perf_counter()
    tire = (P.TIRE_R, P.TIRE_W)
    while car.pos()[1] <= P.Y_END:
        ter.Synchronize(t)
        car.step_in(t, ter)
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        if k % every == 0:
            m.send_traj("front", k, every, [wheel_frame(P, car)], tire)
        if (k + 1) % WRITE_EVERY == 0:
            i, j, h = changed_nodes(ter, sent)
            if i.size:
                m.write_cells("dig", "front", j + j0, i + i0, h, t=t)
            m.poll()
        k += 1
    i, j, h = changed_nodes(ter, sent)
    if i.size:
        m.write_cells("dig", "front", j + j0, i + i0, h, t=t)
    m.send_traj("front", k, every, np.zeros((0, 4, 13)), tire, end=True)
    wall = time.perf_counter() - wall
    au = m.audit(wait_s=120.0)
    save(a.out, heights={(n.x, n.y): v for n, v in ter.GetModifiedNodes(True)}, wall=wall, sim=t, audit=json.dumps(au), steps=k)
    m.close()


def role_rear(a):
    import probe_scm_two_hmmwv as P
    setup(P)
    P.GAP = a.gap
    m, i0, j0 = client(P, "rear", a.tcp)
    s = P.make_system()
    ter, cy = P.make_terrain(s, a.kphi)
    bodies = P.ghost_bodies(s, ter, 4)
    car = P.Car(s, ter, P.Y_FRONT - P.GAP)
    delay = int(round(a.delay / P.STEP))
    frames, ended, every = [], False, None
    sent, log, t, k = {}, [], 0.0, 0
    ghost_on, waited = True, 0.0

    def receive():
        nonlocal ended, every
        m.poll()
        for tm in m.traj_in:
            every = tm["every"]
            if tm["end"]:
                ended = True
            elif tm["data"]:
                from sim.map_client import dec
                if tm["k0"] != len(frames) * every:             # 궤적은 빠지면 안 된다 (자리가 밀린다)
                    raise RuntimeError(f"궤적 프레임이 빠졌다: k0 {tm['k0']}, 받은 {len(frames)}개 x {every}")
                frames.extend(dec(tm["data"], tuple(tm["shape"])))
        m.traj_in.clear()

    def write():
        i, j, h = changed_nodes(ter, sent)
        if i.size:
            r, c = j + j0, i + i0
            h2 = np.clip(h, *Z_RANGE)
            low = h2 < m.local.compose(0, m.local.nrow, 0, m.local.ncol)[r, c] - THRESH   # 고스트가 만든 칸은 지도에 이미 있다
            if low.any():
                m.write_cells("dig", "rear", r[low], c[low], h2[low], t=t)

    wall = time.perf_counter()
    while True:
        if ghost_on:
            idx = max(0, k - delay)
            w0 = time.perf_counter()
            while True:                                         # 이 스텝의 고스트 자세가 올 때까지
                receive()
                if every is not None and (ended or idx // every + 1 < len(frames)):
                    break
                time.sleep(0.0005)
            waited += time.perf_counter() - w0
            a0 = idx // every
            if a0 + 1 >= len(frames):                           # 궤적 끝
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
        x, y, z = car.pos()
        if P.WINDOW[0] <= y <= P.WINDOW[1]:
            log.append(P.log_row(car, t, x, y, z, di))
        if (k + 1) % WRITE_EVERY == 0:
            write()
        k += 1
        if y > P.WINDOW[1] + 3.0:
            break
    write()
    wall = time.perf_counter() - wall
    au = m.audit(wait_s=120.0)
    final = m.local.compose(0, m.local.nrow, 0, m.local.ncol)
    save(a.out, heights={(n.x, n.y): v for n, v in ter.GetModifiedNodes(True)}, wall=wall, sim=t, audit=json.dumps(au), log=np.array(log), final_map=final,
         waited=waited, i0=i0, j0=j0, frames=np.array(frames))
    m.close()


def role_truth(a):
    import probe_scm_two_hmmwv as P
    setup(P)
    P.GAP = a.gap
    ca = P.run(a.kphi, True, True)
    save(a.out, heights=ca["final"], log=ca["log"], cy=ca["cy"], wall=ca["wall"], sim=ca["t"], min_gap=ca["min_gap"])


def load(path):
    d = np.load(path, allow_pickle=False)
    return d, {(int(i), int(j)): h for i, j, h in d["heights"]}



def compare(a, dirn, kphi):
    """chrono 환경에서 돈다 (probe_scm_two_hmmwv의 계산을 쓴다)."""
    import probe_scm_two_hmmwv as P
    setup(P)
    fd, fh = load(dirn / "front.npz")
    rd, rh = load(dirn / "rear.npz")
    td, th = load(dirn / "truth.npz")
    cy = float(td["cy"])
    st_t = P.window_stats({"log": td["log"], "final": th, "cy": cy}, fh)
    st_r = P.window_stats({"log": rd["log"], "final": rh, "cy": cy}, fh)
    final, i0, j0 = rd["final_map"], int(rd["i0"]), int(rd["j0"])
    j_lo, j_hi = int(round((P.WINDOW[0] - cy) / P.DELTA)), int(round((P.WINDOW[1] - cy) / P.DELTA))
    dm, dr = [], []
    for i in range(-i0, i0 + 1):
        for j in range(j_lo, j_hi + 1):
            t_ = th.get((i, j), 0.0)
            m_ = final[j + j0, i + i0]
            if abs(t_) > 0.002 or abs(m_) > 0.002:
                dm.append(100 * (m_ - t_))
                dr.append(100 * (rh.get((i, j), 0.0) - t_))
    dm, dr = np.array(dm), np.array(dr)
    print(f"\n== kphi {kphi:g}: 뒤 차, y {P.WINDOW[0]:g} ~ {P.WINDOW[1]:g} m (궤적 {a.hz:g} Hz, 지연 {a.delay:g} s)")
    print(f"{'':10s}{'추가 침하':>9s}{'끝 깊이':>8s}{'바퀴 높이':>9s}{'스로틀':>7s}{'벗어남cm':>9s}{'옆흔들림':>9s}")
    for name, st in (("정답 A", st_t), ("두 프로세스", st_r)):
        print(f"{name:10s}{st['added_sink_cm']:9.2f}{st['final_depth_cm']:8.2f}{st['spindle_z_m'] * 100:9.2f}{st['throttle']:7.3f}{st['lateral_m'] * 100:9.1f}{st['roll_rms_deg']:9.3f}")
    print(f"차이      {st_r['added_sink_cm'] - st_t['added_sink_cm']:+9.2f}{st_r['final_depth_cm'] - st_t['final_depth_cm']:+8.2f}{(st_r['spindle_z_m'] - st_t['spindle_z_m']) * 100:+9.2f}{st_r['throttle'] - st_t['throttle']:+7.3f}")
    print(f"A 칸별 추가 침하 {st_t['added_sink_bin_min_cm']:.2f} ~ {st_t['added_sink_bin_max_cm']:.2f} cm, 두 차 최소 거리 {float(td['min_gap']):.1f} m")
    _, tb = load(dirn / "truth_b.npz")
    d0 = np.array([100 * (tb.get((i, j), 0.0) - th.get((i, j), 0.0)) for i in range(-i0, i0 + 1) for j in range(j_lo, j_hi + 1)
                   if abs(th.get((i, j), 0.0)) > 0.002 or abs(tb.get((i, j), 0.0)) > 0.002])
    print(f"끝난 지도 - 정답 (자국 칸 {dm.size}개): 평균 {dm.mean():+.3f}, |평균| {np.abs(dm).mean():.3f}, 최대 {np.abs(dm).max():.2f} cm")
    print(f"정답 (출발 10 cm 다름) - 정답   (기준): 평균 {d0.mean():+.3f}, |평균| {np.abs(d0).mean():.3f}, 최대 {np.abs(d0).max():.2f} cm")
    print(f"뒤 차 SCM - 정답                : 평균 {dr.mean():+.3f}, |평균| {np.abs(dr).mean():.3f}, 최대 {np.abs(dr).max():.2f} cm")
    for name, d in (("앞 차", fd), ("뒤 차", rd)):
        au = json.loads(str(d["audit"]))
        print(f"{name}: 불일치 {au.get('hash_mismatch', au)}, 보냄 {au.get('sent')}, 빠르기 {float(d['sim']) / float(d['wall']):.2f}배"
              + (f" (궤적 기다림 {float(d['waited']):.1f} s)" if "waited" in d else ""))


def cell_diff(a_h, b_h, i0, j_lo, j_hi):
    """두 높이 사전의 자국 칸(둘 중 하나가 2 mm 넘게 파인 칸) 차이 (cm)."""
    return np.array([100 * (b_h.get((i, j), 0.0) - a_h.get((i, j), 0.0)) for i in range(-i0, i0 + 1) for j in range(j_lo, j_hi + 1)
                     if abs(a_h.get((i, j), 0.0)) > 0.002 or abs(b_h.get((i, j), 0.0)) > 0.002])


def trace_diff(la, lb):
    """뒤 차 옆 위치 궤적의 차이 (같은 y에서, cm RMS)."""
    ys = np.arange(-8.0, 12.0, 0.25)
    xa = np.interp(ys, la[:, 2], la[:, 1])
    xb = np.interp(ys, lb[:, 2], lb[:, 1])
    return 100 * float(np.sqrt(np.mean((xa - xb) ** 2)))


def spread_compare(a, dirn, kphi):
    """chrono 환경에서: 정답끼리의 범위와 두 프로세스 - 정답."""
    import probe_scm_two_hmmwv as P
    setup(P)
    nrow, ncol, i0, j0 = grid_shape(P)
    cy = (P.SCM_Y[0] + P.SCM_Y[1]) / 2
    j_lo, j_hi = int(round((P.WINDOW[0] - cy) / P.DELTA)), int(round((P.WINDOW[1] - cy) / P.DELTA))
    T, R = {}, {}
    fh = None
    for g in a.spread:
        td, th = load(dirn / f"truth_{g}.npz")
        rd, rh = load(dirn / f"rear_{g}.npz")
        fd, fh = load(dirn / f"front_{g}.npz")
        T[g] = (td["log"], th, P.window_stats({"log": td["log"], "final": th, "cy": cy}, fh))
        final = rd["final_map"]
        mp = {(int(i - i0), int(j - j0)): float(final[j, i]) for j in range(nrow) for i in range(ncol) if final[j, i] != 0.0}
        R[g] = (rd["log"], rh, P.window_stats({"log": rd["log"], "final": rh, "cy": cy}, fh), mp)
        au = json.loads(str(rd["audit"]))
        if au.get("hash_mismatch") != 0:
            print(f"간격 {g}: 뒤 차 해시 불일치 {au}")
    print(f"\n== kphi {kphi:g}. 각 쌍: 칸별 |평균| (cm), 평균 (cm), 뒤 차 옆 궤적 차이 RMS (cm)")
    gs = list(a.spread)
    pairs = []
    for ia, ga in enumerate(gs):
        for gb in gs[ia + 1:]:
            d = cell_diff(T[ga][1], T[gb][1], i0, j_lo, j_hi)
            pairs.append((float(np.abs(d).mean()), float(d.mean()), trace_diff(T[ga][0], T[gb][0]), ga, gb))
            print(f"  정답 {ga} - 정답 {gb}: |평균| {pairs[-1][0]:.3f}, 평균 {pairs[-1][1]:+.3f}, 궤적 {pairs[-1][2]:.1f}", flush=True)
    own = []
    for g in gs:
        d = cell_diff(T[g][1], R[g][3], i0, j_lo, j_hi)
        own.append((float(np.abs(d).mean()), float(d.mean()), trace_diff(T[g][0], R[g][0])))
        print(f"  지도 {g} - 정답 {g}: |평균| {own[-1][0]:.3f}, 평균 {own[-1][1]:+.3f}, 궤적 {own[-1][2]:.1f}", flush=True)
    pa, oa = np.array([p[:3] for p in pairs]), np.array(own)
    print(f"  정답끼리 {len(pairs)}쌍: |평균| {pa[:, 0].min():.3f} ~ {pa[:, 0].max():.3f}, 평균 {pa[:, 1].min():+.3f} ~ {pa[:, 1].max():+.3f}, 궤적 {pa[:, 2].min():.1f} ~ {pa[:, 2].max():.1f}")
    print(f"  지도 - 정답 {len(own)}쌍: |평균| {oa[:, 0].min():.3f} ~ {oa[:, 0].max():.3f}, 평균 {oa[:, 1].min():+.3f} ~ {oa[:, 1].max():+.3f}, 궤적 {oa[:, 2].min():.1f} ~ {oa[:, 2].max():.1f}")
    allp = np.vstack([pa, oa])
    print(f"  칸별 |평균|과 궤적 차이의 상관 (15쌍): {np.corrcoef(allp[:, 0], allp[:, 2])[0, 1]:+.2f}")
    keys = ("added_sink_cm", "final_depth_cm", "spindle_z_m", "throttle", "lateral_m", "speed_mps")
    for name, rows in (("정답", [T[g][2] for g in gs]), ("두 프로세스", [R[g][2] for g in gs])):
        print(f"  {name:8s}" + "  ".join(f"{k.split('_')[0]} {min(r[k] for r in rows) * (100 if k in ('spindle_z_m', 'lateral_m') else 1):.3f} ~ {max(r[k] for r in rows) * (100 if k in ('spindle_z_m', 'lateral_m') else 1):.3f}" for k in keys))


def spread(a, me, env):
    for kphi in a.kphi:
        with tempfile.TemporaryDirectory(prefix="two_chrono_") as tmp:
            dirn = Path(tmp)
            for g in a.spread:
                s = socket.socket()
                s.bind(("127.0.0.1", 0))
                port = s.getsockname()[1]
                s.close()
                server = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--tcp-port", str(port)], cwd=ROOT,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                common = ["--kphi", str(kphi), "--hz", str(a.hz), "--delay", str(a.delay), "--tcp", f"127.0.0.1:{port}", "--gap", str(g)]
                try:
                    front = subprocess.Popen(me + ["--role", "front", "--out", str(dirn / f"front_{g}.npz")] + common, env=env, cwd=ROOT)
                    time.sleep(1.0)
                    rear = subprocess.Popen(me + ["--role", "rear", "--out", str(dirn / f"rear_{g}.npz")] + common, env=env, cwd=ROOT)
                    for p in (front, rear):
                        assert p.wait(timeout=1800) == 0
                finally:
                    server.terminate()
                    server.wait(timeout=10)
                subprocess.run(me + ["--role", "truth", "--out", str(dirn / f"truth_{g}.npz"), *common], env=env, cwd=ROOT, check=True, timeout=1800)
                print(f"[kphi {kphi:g} 간격 {g} 끝]", flush=True)
            subprocess.run(me + ["--role", "spread_compare", "--dir", str(dirn), "--kphi", str(kphi), "--spread", *map(str, a.spread)], env=env, cwd=ROOT, check=True, timeout=900)


def orchestrate(a):
    try:
        import rclpy  # noqa: F401
    except ImportError:
        raise SystemExit("맵 서버에 ROS2가 필요하다: conda activate mj_ros 후 실행")
    chrono_py = str(Path.home() / "miniconda3/envs/chrono/bin/python")
    me = [chrono_py, str(Path(__file__).resolve())]
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH")}
    if a.spread:
        return spread(a, me, env)
    for kphi in a.kphi:
        with tempfile.TemporaryDirectory(prefix="two_chrono_") as tmp:
            dirn = Path(tmp)
            s = socket.socket()
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
            s.close()
            server = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--tcp-port", str(port)], cwd=ROOT,
                                      stdout=open(dirn / "server.log", "w"), stderr=subprocess.STDOUT)
            common = ["--kphi", str(kphi), "--hz", str(a.hz), "--delay", str(a.delay), "--tcp", f"127.0.0.1:{port}"]
            try:
                t0 = time.perf_counter()
                front = subprocess.Popen(me + ["--role", "front", "--out", str(dirn / "front.npz")] + common, env=env, cwd=ROOT)
                time.sleep(1.0)                                 # 앞 차가 먼저 지도를 만든다 (앞 차는 뒤 차가 붙을 때까지 기다린다)
                rear = subprocess.Popen(me + ["--role", "rear", "--out", str(dirn / "rear.npz")] + common, env=env, cwd=ROOT)
                for name, p in (("앞 차", front), ("뒤 차", rear)):
                    if p.wait(timeout=1800) != 0:
                        raise SystemExit(f"{name} 프로세스가 실패했다")
                print(f"[두 프로세스 {time.perf_counter() - t0:.0f} s]", flush=True)
            finally:
                server.terminate()
                server.wait(timeout=10)
            subprocess.run(me + ["--role", "truth", "--out", str(dirn / "truth.npz"), *common], env=env, cwd=ROOT, check=True, timeout=1800)
            subprocess.run(me + ["--role", "truth", "--gap", "12.1", "--out", str(dirn / "truth_b.npz"), *common], env=env, cwd=ROOT, check=True, timeout=1800)
            subprocess.run(me + ["--role", "compare", "--dir", str(dirn), *common], env=env, cwd=ROOT, check=True, timeout=600)


def main():
    ap = argparse.ArgumentParser(description="Chrono 프로세스 둘 + 맵 서버 시제품")
    ap.add_argument("--kphi", type=float, nargs="+", default=[5e6, 1e6])
    ap.add_argument("--hz", type=float, default=25.0)
    ap.add_argument("--delay", type=float, default=0.2)
    ap.add_argument("--role", choices=("front", "rear", "truth", "compare", "spread_compare"))
    ap.add_argument("--spread", type=float, nargs="*", help="뒤 차 출발 간격들 (m)")
    ap.add_argument("--gap", type=float, default=12.0)
    ap.add_argument("--tcp")
    ap.add_argument("--out")
    ap.add_argument("--dir")
    a = ap.parse_args()
    if a.role is None:
        return orchestrate(a)
    a.kphi = a.kphi[0]
    if a.role == "spread_compare":
        return spread_compare(a, Path(a.dir), a.kphi)
    if a.role == "compare":
        return compare(a, Path(a.dir), a.kphi)
    {"front": role_front, "rear": role_rear, "truth": role_truth}[a.role](a)


if __name__ == "__main__":
    main()
