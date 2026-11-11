"""서로 궤적 받기: 차마다 같은 코드 (peer)로 자기 궤적을 보내고, 받은 남의 궤적을 고스트로 재생한다. 어느 차가 먼저 지나갈지 미리 정하지 않는다.
길은 교차 시험 (probe_cross.py): A는 x = 0을 +y로, B는 --angle 도로 (0, 0)을 지난다. B의 출발 (--starts, B 길을 따라 (0, 0)까지 거리, 음수)로 순서가 바뀐다.

    python conformance/probe_peer.py --starts -24.1 -24 -23.9 --kphi 1e6      A가 먼저 지나간다 (B가 A 자국을 건넌다)
    python conformance/probe_peer.py --starts -8.1 -8 -7.9 --kphi 1e6         B가 먼저 지나간다 (A가 B 자국을 건넌다)
    python conformance/probe_peer.py --role truth --start -8 --kphi 1e6 --out /tmp/t.npz    정답만 (chrono 환경)

  peer: 자기 SCM에서 자기 차를 달리고, 궤적을 traj로 보내고, 지도보다 THRESH 넘게 낮아진 칸만 dig로 쓴다 (첫 차와 뒤 차를 나누지 않는다).
        받은 궤적마다 고스트 바퀴 넷을 둔다. 재생은 probe_two_chrono.py의 뒤 차와 같다 (지연 버퍼, 따라잡기, 빠진 구멍 건너뛰기).
        자기 차 앞바퀴 앞 GUARD m 안이거나 지나간 고스트 바퀴는 숨기고 (T.hide_late_ghosts), 자기 차에서 NEAR m 넘게 먼 고스트는 쉬게 한다.
  정답: 한 SCM에서 두 대. 두 차의 기록을 모두 남긴다.
  비교: 차마다 자기 교차 구간 (자기 길을 따라 (0, 0)에서 ±6 m)의 바퀴 높이, 옆 위치, 같은 출발의 정답과 옆 궤적 차이. 교차점 둘레 칸의 지도와 정답 높이.
"""
import argparse
import bisect
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
import probe_cross as C  # noqa: E402
import probe_two_chrono as T  # noqa: E402

CARS = ("A", "B")
WIN = 6.0                                          # 교차 구간: 자기 길을 따라 (0, 0)에서 ±WIN m
END = WIN + 2.0                                    # 이만큼 지나면 그 차는 끝
NEAR = 15.0                                        # 이보다 먼 고스트는 쉬게 한다 (m, 자기 차 중심에서 가장 가까운 고스트 바퀴까지)
BOX = 4.0                                          # 지도 비교: 교차점 둘레 |x|, |y| < BOX m


def setup(P, a):
    C.setup(P, a)
    T.apply_fast(P, a, a.threads)


def make_car(P, s, ter, a, who):
    return P.Car(s, ter, P.Y_FRONT) if who == "A" else C.cross_car(P, s, ter, a)


def along(car, a, who):
    """자기 길을 따라 (0, 0)까지의 거리 (지나면 +)."""
    return car.pos()[1] if who == "A" else C.along(car, a)


def side(x, y, a, who):
    """자기 길에서 옆으로 벗어난 거리 (m, 왼쪽 +)."""
    if who == "A":
        return -x
    th = C.heading(a.angle)
    return -x * math.sin(th) + y * math.cos(th)


def log_row(P, car, t, di, a, who):
    """시각, 길을 따라 거리, 옆 위치, 바퀴 중심 높이 평균, 속도, 조향."""
    x, y, _ = car.pos()
    v = car.car.GetVehicle()
    wz = [v.GetSpindlePos(k, sd).z for k in (0, 1) for sd in (P.veh.LEFT, P.veh.RIGHT)]
    return (t, along(car, a, who), side(x, y, a, who), float(np.mean(wz)), v.GetSpeed(), di.m_steering)


class Playback:
    """한 상대의 궤적 재생 (probe_two_chrono.role_rear_free와 같은 규칙)."""

    def __init__(self, P, bodies, delay, catchup):
        self.P, self.bodies, self.delay, self.R = P, bodies, delay, catchup
        self.frames, self.keys, self.every, self.end_k, self.max_fill = {}, [], None, None, None
        self.cur, self.missed, self.hid, self.far, self.behind, self.fast = 0, 0, 0, 0, 0, 0
        self.trace = []                                    # 스텝마다 (k, 재생 위치, 상태: 0 놓음, 1 빠짐, 2 멂, 3 끝, 4 준비 전, 숨긴 바퀴 수)

    def add(self, tm, dec):
        self.every = tm["every"]
        self.max_fill = int(round(T.MAX_FILL / (self.every * self.P.STEP)))
        if tm["end"]:
            self.end_k = tm["k0"]
        elif tm["data"]:
            f = tm["k0"] // self.every
            self.frames[f] = dec(tm["data"], tuple(tm["shape"]))[0]
            bisect.insort(self.keys, f)

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
        P = self.P
        if self.every is None or len(self.keys) < 2:
            T.hide_ghosts(P, self.bodies)
            self.trace.append((k, self.cur, 4, 0))
            return
        target = max(0, k - self.delay)
        self.cur, rate = self.advance(target)
        self.behind = max(self.behind, target - self.cur)
        self.fast += rate > 1 and target - self.cur > 0
        if self.end_k is not None and self.cur >= self.end_k - self.every:
            T.hide_ghosts(P, self.bodies)
            self.trace.append((k, self.cur, 3, 0))
            return
        fr = self.pose(self.cur)
        if fr is None:
            T.hide_ghosts(P, self.bodies)
            self.missed += 1
            self.trace.append((k, self.cur, 1, 0))
            return
        x, y, _ = car.pos()
        if np.min(np.hypot(fr[:, 0] - x, fr[:, 1] - y)) > NEAR:
            T.hide_ghosts(P, self.bodies)
            self.far += 1
            self.trace.append((k, self.cur, 2, 0))
            return
        P.place_ghosts(self.bodies, fr)
        n = T.hide_late_ghosts(P, car, self.bodies, fr)
        self.hid += n
        self.trace.append((k, self.cur, 0, n))


def role_peer(a):
    from sim.map_client import dec
    import probe_scm_two_hmmwv as P
    setup(P, a)
    who = a.who
    other = [c for c in CARS if c != who][0]
    m, to_cells, i0, j0 = T.client(P, who, a)
    s = P.make_system()
    ter, _ = P.make_terrain(s, a.kphi)
    pb = Playback(P, P.ghost_bodies(s, ter, 4), int(round(a.delay / P.STEP)), a.catchup)
    car = make_car(P, s, ter, a, who)
    every = max(1, int(round(1.0 / (a.hz * P.STEP))))
    tire = (P.TIRE_R, P.TIRE_W)
    sent, log, t, k = {}, [], 0.0, 0
    hello, other_end = set(), False

    def receive():
        nonlocal other_end
        m.poll()
        for tm in m.traj_in:
            if tm["writer"] != other:
                continue
            if tm["k0"] == 0 and not tm["data"] and not tm["end"] and tm["every"] == 1:
                hello.add(other)                               # 준비 끝 알림 (빈 궤적)
                continue
            if tm["end"]:
                other_end = True
            pb.add(tm, dec)
        m.traj_in.clear()

    def write():
        r, c, h = to_cells(*T.changed_nodes(ter, sent))
        if r.size:
            low = h < m.local.compose(0, m.local.nrow, 0, m.local.ncol)[r, c] - T.THRESH
            if low.any():
                m.write_cells("dig", who, r[low], c[low], h[low], t=t)

    t0 = time.monotonic()                                      # 둘 다 준비될 때까지 준비 알림을 되풀이한다 (붙기 전에 보낸 것은 서버가 넘기지 못한다)
    while other not in hello:
        m.send_traj(who, 0, 1, np.zeros((0, 4, 13)), tire)
        for _ in range(20):
            receive()
            time.sleep(0.01)
        if time.monotonic() - t0 > 120:
            raise RuntimeError(f"{other}가 붙지 않는다")
    for _ in range(5):                                         # 상대도 내 알림을 받도록 몇 번 더
        m.send_traj(who, 0, 1, np.zeros((0, 4, 13)), tire)
        time.sleep(0.02)
    wall = time.perf_counter()
    lag, plog = 0.0, []
    while along(car, a, who) <= END:
        receive()
        pb.step(k, car)
        ter.Synchronize(t)
        di = car.step_in(t, ter)
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        log.append(log_row(P, car, t, di, a, who))
        if k % every == 0:
            m.send_traj(who, k, every, [T.wheel_frame(P, car)], tire)
        if (k + 1) % T.WRITE_EVERY == 0:
            write()
        k += 1
        lag = max(lag, T.pace_wait(wall, t, a.pace))
    write()
    m.send_traj(who, k, every, np.zeros((0, 4, 13)), tire, end=True)
    wall = time.perf_counter() - wall
    t_wait = time.monotonic()                                  # 상대가 끝날 때까지 지도를 받는다 (끝난 지도에 두 차가 다 들어가게)
    while not other_end and time.monotonic() - t_wait < 300:
        receive()
        time.sleep(0.01)
    time.sleep(0.5)
    m.poll()
    au = m.audit(wait_s=120.0)
    T.save(a.out, heights=C.final_heights(ter), log=np.array(log), wall=wall, sim=t, audit=json.dumps(au), final_map=m.local.compose(0, m.local.nrow, 0, m.local.ncol),
           i0=i0, j0=j0, missed=pb.missed, hid=pb.hid, far=pb.far, behind=pb.behind * P.STEP, fast=pb.fast, lag=lag, steps=k,
           trace=np.array(pb.trace), ghost_frames=np.array([pb.frames[f] for f in pb.keys]) if pb.keys else np.zeros((0, 4, 13)), ghost_keys=np.array(pb.keys), ghost_every=pb.every or 0)
    m.close()


def role_truth(a):
    import probe_scm_two_hmmwv as P
    setup(P, a)
    if not a.truth_fast:
        P.WHEEL_DOMAIN, P.GHOST_DOMAIN, P.THREADS = None, (1.4, 0.8, 1.4), 1      # 정답은 기본 활성 영역
    s = P.make_system()
    ter, _ = P.make_terrain(s, a.kphi)
    cars = {w: make_car(P, s, ter, a, w) for w in CARS}
    logs, t_pass = {w: [] for w in CARS}, {w: None for w in CARS}
    t, gap = 0.0, float("inf")
    while any(along(cars[w], a, w) <= END for w in CARS):
        ter.Synchronize(t)
        di = {w: cars[w].step_in(t, ter) for w in CARS}
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        for w in CARS:
            logs[w].append(log_row(P, cars[w], t, di[w], a, w))
            if t_pass[w] is None and along(cars[w], a, w) > 0:
                t_pass[w] = t
        (ax, ay, _), (bx, by, _) = cars["A"].pos(), cars["B"].pos()
        gap = min(gap, math.hypot(ax - bx, ay - by))
    T.save(a.out, heights=C.final_heights(ter), log_A=np.array(logs["A"]), log_B=np.array(logs["B"]), t_pass_A=t_pass["A"], t_pass_B=t_pass["B"], min_gap=gap, sim=t)


def window_stats(lg):
    w = (lg[:, 1] >= -WIN) & (lg[:, 1] <= WIN)
    return 100 * float(lg[w, 3].mean()), 100 * float(np.sqrt(np.mean(lg[w, 2] ** 2))), 100 * float(lg[w, 2][-1])


def trace_diff(la, lb):
    u = np.arange(-WIN, WIN, 0.25)
    return 100 * float(np.sqrt(np.mean((np.interp(u, la[:, 1], la[:, 2]) - np.interp(u, lb[:, 1], lb[:, 2])) ** 2)))


def box_diff(h_truth, mp, delta):
    n = int(round(BOX / delta))
    keys = [(i, j) for i in range(-n, n + 1) for j in range(-n, n + 1)]
    d = np.array([100 * (mp.get(k, 0.0) - h_truth.get(k, 0.0)) for k in keys if abs(h_truth.get(k, 0.0)) > 0.002 or abs(mp.get(k, 0.0)) > 0.002])
    return d


def report(dirn, a, kphi):
    import probe_scm_two_hmmwv as P
    setup(P, a)
    rows = {}
    for g in a.starts:
        td, th = T.load(dirn / f"truth_{g}.npz")
        rows[g] = {"truth": td, "th": th}
        for w in CARS:
            pd, _ = T.load(dirn / f"{w}_{g}.npz")
            rows[g][w] = pd
    g0 = a.starts[0]
    tp = {w: [float(rows[g]["truth"][f"t_pass_{w}"]) for g in a.starts] for w in CARS}
    first = "A" if np.mean(tp["A"]) < np.mean(tp["B"]) else "B"
    later = [c for c in CARS if c != first][0]
    margin = []                                                # 나중 차 앞바퀴가 교차 칸에 닿은 시각 - 먼저 차 뒷바퀴가 그 칸을 떠난 시각 (정답 기록, 축 사이 1.65 m, 바퀴 칸 반폭 1.1 m)
    for g in a.starts:
        lf, ll = rows[g]["truth"][f"log_{first}"], rows[g]["truth"][f"log_{later}"]
        t_leave = lf[np.argmax(lf[:, 1] > 1.65 + 1.1), 0]
        t_arrive = ll[np.argmax(ll[:, 1] > -(1.65 + 1.1)), 0]
        margin.append(t_arrive - t_leave)
    print(f"\n== kphi {kphi:g}, 각도 {a.angle:g}, B 출발 {a.starts}: 교차점 지나는 시각 A {min(tp['A']):.2f} ~ {max(tp['A']):.2f} s, B {min(tp['B']):.2f} ~ {max(tp['B']):.2f} s "
          f"-> {first}가 먼저. 여유 (나중 차 앞바퀴 도착 - 먼저 차 뒷바퀴 떠남) {min(margin):.2f} ~ {max(margin):.2f} s. 두 차 최소 거리 {min(float(rows[g]['truth']['min_gap']) for g in a.starts):.1f} m")
    for g in a.starts:
        line = []
        for w in CARS:
            pd = rows[g][w]
            au = json.loads(str(pd["audit"]))
            line.append(f"{w}: 쉼 {int(pd['missed'])}, 멀어서 쉼 {int(pd['far'])}/{int(pd['steps'])}, 숨긴 바퀴 {int(pd['hid'])}, 늦음 최대 {1000 * float(pd['lag']):.0f} ms, "
                        f"빠르기 {float(pd['sim']) / float(pd['wall']):.2f}배{', 해시 불일치 ' + str(au.get('hash_mismatch')) if au.get('hash_mismatch') else ''}")
        print(f"  [B 출발 {g}] " + " | ".join(line))
    for w in CARS:
        role = "먼저" if w == first else "나중 (상대 자국을 건넌다)"
        tr = [window_stats(rows[g]["truth"][f"log_{w}"]) for g in a.starts]
        pr = [window_stats(rows[g][w]["log"]) for g in a.starts]
        dd = [trace_diff(rows[g]["truth"][f"log_{w}"], rows[g][w]["log"]) for g in a.starts]
        base = [trace_diff(rows[ga]["truth"][f"log_{w}"], rows[gb]["truth"][f"log_{w}"]) for i, ga in enumerate(a.starts) for gb in a.starts[i + 1:]]
        r = lambda v, f=".2f": f"{min(v):{f}} ~ {max(v):{f}}"
        print(f"  {w} ({role}): 교차 구간 바퀴 높이 정답 {r([x[0] for x in tr])}, 두 프로세스 {r([x[0] for x in pr])} cm | 옆 위치 끝 정답 {r([x[2] for x in tr], '.1f')}, "
              f"두 프로세스 {r([x[2] for x in pr], '.1f')} cm | 옆 궤적 차이: 같은 출발 정답과 {r(dd, '.1f')} cm, 정답끼리 {r(base, '.1f') if base else '-'} cm")
    for name, mk in (("A 지도", "A"), ("B 지도", "B")):
        dm = []
        for g in a.starts:
            pd = rows[g][mk]
            final, i0, j0 = pd["final_map"], int(pd["i0"]), int(pd["j0"])
            mp = {(int(i - i0), int(j - j0)): float(final[j, i]) for j, i in zip(*np.nonzero(final))}
            d = box_diff(rows[g]["th"], mp, P.DELTA)
            dm.append((float(np.abs(d).mean()), float(d.mean()), d.size))
        print(f"  {name} - 정답 (교차점 둘레 {2 * BOX:g} m 칸, 자국 칸 {dm[0][2]}개 안팎): |평균| {min(x[0] for x in dm):.3f} ~ {max(x[0] for x in dm):.3f}, 평균 {min(x[1] for x in dm):+.3f} ~ {max(x[1] for x in dm):+.3f} cm")
    bd = []
    for i, ga in enumerate(a.starts):
        for gb in a.starts[i + 1:]:
            d = box_diff(rows[ga]["th"], rows[gb]["th"], P.DELTA)
            bd.append((float(np.abs(d).mean()), float(d.mean())))
    if bd:
        print(f"  정답끼리 (같은 칸): |평균| {min(x[0] for x in bd):.3f} ~ {max(x[0] for x in bd):.3f}, 평균 {min(x[1] for x in bd):+.3f} ~ {max(x[1] for x in bd):+.3f} cm")


def orchestrate(a):
    try:
        import rclpy  # noqa: F401
    except ImportError:
        raise SystemExit("맵 서버에 ROS2가 필요하다: conda activate mj_ros 후 실행")
    me = [str(Path.home() / "miniconda3/envs/chrono/bin/python"), str(Path(__file__).resolve())]
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH")}
    for kphi in a.kphi:
        with tempfile.TemporaryDirectory(prefix="peer_") as tmp:
            dirn = Path(a.keep) if a.keep else Path(tmp)
            dirn.mkdir(parents=True, exist_ok=True)
            for g in a.starts:
                so = socket.socket()
                so.bind(("127.0.0.1", 0))
                port = so.getsockname()[1]
                so.close()
                server = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--tcp-port", str(port)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                common = ["--angle", str(a.angle), "--kphi", str(kphi), "--start", str(g), "--hz", str(a.hz), "--delay", str(a.delay), "--pace", str(a.pace),
                          "--catchup", str(a.catchup), "--threads", str(a.threads)] + (["--fast"] if a.fast else [])
                try:
                    peers = [subprocess.Popen(me + ["--role", "peer", "--who", w, "--tcp", f"127.0.0.1:{port}", "--out", str(dirn / f"{w}_{g}.npz")] + common, env=env, cwd=ROOT) for w in CARS]
                    for w, p in zip(CARS, peers):
                        if p.wait(timeout=1800) != 0:
                            raise SystemExit(f"{w} 프로세스가 실패했다")
                finally:
                    server.terminate()
                    server.wait(timeout=10)
                subprocess.run(me + ["--role", "truth", "--out", str(dirn / f"truth_{g}.npz")] + common + (["--truth-fast"] if a.truth_fast else []), env=env, cwd=ROOT, check=True, timeout=1800)
                print(f"[kphi {kphi:g} B 출발 {g} 끝]", flush=True)
            subprocess.run(me + ["--role", "report", "--dir", str(dirn), "--kphi", str(kphi), "--angle", str(a.angle), "--starts", *map(str, a.starts)]
                           + (["--fast"] if a.fast else []), env=env, cwd=ROOT, check=True, timeout=600)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=float, default=90.0)
    ap.add_argument("--kphi", type=float, nargs="+", default=[5e6, 1e6])
    ap.add_argument("--starts", type=float, nargs="+", default=[-24.1, -24.0, -23.9], help="B 출발 (B 길을 따라 (0, 0)까지 거리, m)")
    ap.add_argument("--start", type=float)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--delay", type=float, default=0.2, help="고스트 재생 지연 버퍼 (s). B가 먼저인 교차는 여유가 약 0.8초라 0.5초면 고스트가 A 앞바퀴 GUARD 안에 들어간다")
    ap.add_argument("--pace", type=float, default=1.0)
    ap.add_argument("--catchup", type=int, default=4)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--fast", action="store_true", default=True)
    ap.add_argument("--truth-fast", action="store_true")
    ap.add_argument("--role", choices=("peer", "truth", "report"))
    ap.add_argument("--who", choices=CARS)
    ap.add_argument("--tcp")
    ap.add_argument("--out")
    ap.add_argument("--dir")
    ap.add_argument("--keep", help="결과 파일을 이 폴더에 남긴다")
    a = ap.parse_args()
    a.lane = None
    if a.role in ("peer", "truth"):
        a.kphi = a.kphi[0]
    if a.role == "peer":
        return role_peer(a)
    if a.role == "truth":
        return role_truth(a)
    if a.role == "report":
        return report(Path(a.dir), a, a.kphi[0])
    orchestrate(a)


if __name__ == "__main__":
    main()
