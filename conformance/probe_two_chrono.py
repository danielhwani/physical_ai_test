"""맵 서버 4단계 시제품: Chrono 프로세스 둘이 맵 서버로 자국과 바퀴 궤적을 주고받아, 뒤 차가 앞 차 자국을 밟는다 (docs/map_server.md 8절, 4단계).
시뮬레이터 없이 맵 서버와 Chrono 프로세스만. 정답은 한 SCM에서 두 대를 달린 것 (conformance/probe_scm_two_hmmwv.py의 A).

    python conformance/probe_two_chrono.py [--kphi 5e6 1e6] [--hz 25] [--delay 0.2]       (mj_ros 환경. Chrono 프로세스는 chrono 환경으로 띄운다)
    python conformance/probe_two_chrono.py --spread 11.8 11.9 12.0 12.1 12.2              뒤 차 출발 위치별로 정답끼리의 칸별 범위와 두 프로세스를 비교
    python conformance/probe_two_chrono.py --spread 11.8 11.9 12.0 12.1 12.2 --rear-x 0.1 0.2 0.3   자국 벽 시험: 뒤 차를 앞 차 길에서 옆으로 옮겨서
    python conformance/probe_two_chrono.py --spread 11.8 11.9 12.0 12.1 12.2 --delays 1 2 3 4 5     궤적이 얼마나 늦어도 되는가 (정답은 한 번만 돌린다)
    python conformance/probe_two_chrono.py --spread 11.8 11.9 12.0 12.1 12.2 --delay 1 --cases base drop0.5 out0.5 out2 late4   뒤 차가 기다리지 않는다 (같은 배속 --pace)
        base 그대로, dropP 궤적 프레임을 확률 P로 뺀다, outD 앞 차 t = 6 s (--outage-at)부터 D초 끊긴다 (구멍 앞, 안, 뒤로 나눠 출력), lateW 궤적이 실제 시간 W초 늦게 도착한다, stallD 궤적이 D초 끊겼다가 몰려서 온다 (밀린 궤적은 --catchup 배로 재생)
    python conformance/probe_two_chrono.py --spread 11.8 11.9 12.0 12.1 12.2 --delay 1 --cases base --fast --pace 1   실시간: 앞 차와 뒤 차의 SCM 활성 영역을 줄이고 1배속으로 (--truth-fast면 정답도 그 영역으로)
    python conformance/probe_two_chrono.py --route curve --spread 11.8 11.9 12.0 12.1 12.2 --delay 1 --fast --pace 1 --hz 100 --cases base   곡선 (무른 흙은 100 Hz가 필요하다. 운용 권장 100 Hz, 기본값은 예전 결과 재현용 25 Hz)
    python conformance/probe_two_chrono.py --robot scenarios/two_cars_crossing.yaml [--godot]   두 차가 낸 자국을 시뮬레이터의 로봇이 넘는다 (차는 지도 x = --lane m를 지난다)

  앞 차 프로세스: 자기 SCM에서 달리며 바뀐 노드를 dig로, 네 바퀴 궤적을 traj로 맵 서버(TCP)에 보낸다.
  뒤 차 프로세스: 받은 궤적을 고스트 바퀴로 재생해 자기 SCM을 누르고 (높이는 넣지 않는다), 뒤 차를 달리며 지도보다 THRESH 넘게 낮아진 칸만 dig로 보낸다.
                  궤적이 아직 안 왔으면 물리를 멈추고 기다린다 (시각을 맞추려고. 실시간 동작이 아니다).
  지도: 소유자 없이 공유 영역. 칸마다 더 낮은 값이 남는다 (dig = min).
  비교: 뒤 차의 침하, 바퀴 높이, 스로틀, 옆 벗어남을 정답과. 끝난 지도 높이를 정답 SCM의 끝 높이와. 두 클라이언트와 서버의 해시 불일치.
"""
import argparse
import json
import os
import shutil
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
PACE_LOG = 250                                                 # 뒤 차 빠르기 기록 간격 (스텝, 0.5 s)
MAX_FILL = 1.0                                     # 받은 궤적 프레임 사이를 메우는 최대 시간 (s, 시뮬레이션)
GUARD = 1.0                                        # 고스트를 숨기는 거리: 자기 차 앞바퀴 앞 (m)
WRITE_EVERY = 100                                  # 지도에 쓰는 주기 (물리 스텝, 0.2 s)


ROUTE = "straight"


def setup(P):
    """앞 차가 멀리 선다 (뒤 차가 구간을 다 지나도 부딪히지 않게: 기본 길이면 끝에서 중심 거리 4.1 m). curve: S자 길 (SCM 폭 8 m)."""
    P.Y_END, P.SCM_Y = 26.0, (-30.0, 36.0)
    P.ROUTE = ROUTE
    if ROUTE == "curve":
        P.SCM_W = 8.0                                          # make_terrain이 곡선에 쓰는 폭과 지도 칸을 맞춘다
    return P


def apply_fast(P, a, threads=1):
    """--fast: 차는 바퀴마다, 고스트도 작은 SCM 활성 영역 (레이 수를 줄인다). 접촉을 다 덮으면 영역 크기는 결과를 바꾸지 않는다. threads: 이 프로세스의 Chrono 스레드 수."""
    if a.fast:
        P.WHEEL_DOMAIN, P.GHOST_DOMAIN, P.THREADS = (1.1, 0.5, 1.1), (1.1, 0.4, 1.1), threads


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


def client(P, name, a):
    """지도에 붙는다. 반환: 클라이언트, to_cells(i, j, h) -> 지도 칸 (행, 열, 높이), 범위 밖 칸은 뺀다.
    --robot: 지도는 시뮬레이터 것 (관찰자로 받는다). SCM 노드 (i, j)의 월드 좌표 = (lane + i 칸, SCM 가운데 y + j 칸)."""
    from sim.map_client import RemoteMap
    if a.lane is None:
        nrow, ncol, i0, j0 = grid_shape(P)
        m = RemoteMap(np.zeros((nrow, ncol)), Z_RANGE, name=name, tcp=a.tcp)
    else:
        m = RemoteMap(None, name=name, tcp=a.tcp, wait_s=60.0)
        nrow, ncol = m.local.nrow, m.local.ncol
        cy = (P.SCM_Y[0] + P.SCM_Y[1]) / 2
        i0 = int(round((a.lane + (ncol - 1) * P.DELTA / 2) / P.DELTA))
        j0 = int(round((cy + (nrow - 1) * P.DELTA / 2) / P.DELTA))
    m.keep_traj = True

    def to_cells(i, j, h):
        r, c = j + j0, i + i0
        ok = (r >= 0) & (r < nrow) & (c >= 0) & (c < ncol)
        return r[ok], c[ok], np.clip(h[ok], m.local.z_min, m.local.z_max)
    return m, to_cells, i0, j0


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
    apply_fast(P, a)
    m, to_cells, i0, j0 = client(P, "front", a)
    s = P.make_system()
    ter, cy = P.make_terrain(s, a.kphi)
    car = P.Car(s, ter, P.Y_FRONT)
    every = max(1, int(round(1.0 / (a.hz * P.STEP))))
    sent, t, k = {}, 0.0, 0
    t_wait = time.monotonic()
    while not any(tm["writer"] == "rear" for tm in m.traj_in):  # 뒤 차가 지도를 다 받고 알릴 때까지 출발하지 않는다 (붙기 전 궤적은 받지 못한다: 늦게 붙기는 아직 없다)
        m.poll()
        time.sleep(0.01)
        if time.monotonic() - t_wait > 120:
            raise RuntimeError("뒤 차가 붙지 않는다")
    wall = time.perf_counter()
    tire = (P.TIRE_R, P.TIRE_W)
    rng = np.random.default_rng(1)
    ys, held = [], []
    while car.pos()[1] <= P.Y_END:
        ter.Synchronize(t)
        car.step_in(t, ter)
        ter.Advance(P.STEP)
        s.DoStepDynamics(P.STEP)
        t = s.GetChTime()
        ys.append((t, car.pos()[1]))
        out = a.outage is not None and a.outage[0] <= t < a.outage[0] + a.outage[1]
        if k % every == 0 and not out and (k < 2 * every or rng.random() >= a.drop_traj):   # 처음 두 프레임은 시작 맞추기에 쓴다
            if a.stall is not None and a.stall[0] <= t < a.stall[0] + a.stall[1]:    # 사라지지 않고 늦는다 (끊김이 풀리면 몰아서 보낸다)
                held.append((k, wheel_frame(P, car)))
            else:
                for hk, hf in held:
                    m.send_traj("front", hk, every, [hf], tire)
                held.clear()
                m.send_traj("front", k, every, [wheel_frame(P, car)], tire)
        if (k + 1) % WRITE_EVERY == 0:
            r, c, h = to_cells(*changed_nodes(ter, sent))
            if r.size:
                m.write_cells("dig", "front", r, c, h, t=t)
            m.poll()
        k += 1
        if a.pace:
            pace_wait(wall, t, a.pace)
    r, c, h = to_cells(*changed_nodes(ter, sent))
    if r.size:
        m.write_cells("dig", "front", r, c, h, t=t)
    m.send_traj("front", k, every, np.zeros((0, 4, 13)), tire, end=True)
    wall = time.perf_counter() - wall
    au = m.audit(wait_s=120.0)
    save(a.out, heights={(n.x, n.y): v for n, v in ter.GetModifiedNodes(True)}, wall=wall, sim=t, audit=json.dumps(au), steps=k, ys=np.array(ys))
    m.close()


def pace_wait(wall0, t, pace):
    """시뮬레이션 시각 t가 실제 시간의 pace배를 앞서면 기다린다. 반환: 늦은 시간 (s, 0 이상)."""
    now = time.perf_counter() - wall0
    target = t / pace
    if now < target:
        time.sleep(target - now)
        return 0.0
    return now - target


def hide_ghosts(P, bodies):
    for b in bodies:
        b.SetPos(P.chrono.ChVector3d(0, 0, 50))
        b.SetPosDt(P.chrono.ChVector3d(0, 0, 0))
        b.SetAngVelParent(P.chrono.ChVector3d(0, 0, 0))


def role_rear_free(a):
    """기다리지 않는 뒤 차: delay 전 자세를 재생한다 (지터 버퍼). 그때 자세가 없으면 고스트를 쉬게 한다 (받은 프레임 사이가 MAX_FILL 이하면 메운다).
    늦게 온 궤적은 버리지 않는다: 재생 위치 cur가 목표 (k - delay)보다 밀렸으면 R배로 따라잡고, 받은 궤적 끝에 가까우면 (2프레임 미만) 1배로 간다. 프레임이 실제로 빠진 구멍은 건너뛴다.
    늦지 않으면 cur는 항상 목표와 같다."""
    import bisect
    import probe_scm_two_hmmwv as P
    from sim.map_client import dec
    setup(P)
    apply_fast(P, a, a.threads)
    P.GAP = a.gap
    m, to_cells, i0, j0 = client(P, "rear", a)
    s = P.make_system()
    ter, cy = P.make_terrain(s, a.kphi)
    bodies = P.ghost_bodies(s, ter, 4)
    car = P.Car(s, ter, P.Y_FRONT - P.GAP, a.rear_x[0])
    delay = int(round(a.delay / P.STEP))
    frames, keys, queue, every, end_k = {}, [], [], None, None
    sent, log, t, k = {}, [], 0.0, 0
    hid, missed, late_frames, lag = 0, 0, 0, 0.0
    max_fill = None
    R = int(a.catchup)
    plog = []
    cur, behind, fast = 0, 0, 0
    m.send_traj("rear", 0, 1, np.zeros((0, 4, 13)), (P.TIRE_R, P.TIRE_W))

    def receive(idx):
        nonlocal every, end_k, late_frames, max_fill
        m.poll()
        queue.extend(m.traj_in)
        m.traj_in.clear()
        now = time.time()
        rest = []
        for tm in queue:
            start = tm["k0"] < 2 * tm["every"] or tm["end"]               # 처음 두 프레임과 끝 표시는 늦추지 않는다 (시작 맞추기와 끝내기)
            if not start and now - tm["wall"] < a.net_delay:
                rest.append(tm)
                continue
            every = tm["every"]
            max_fill = int(round(MAX_FILL / (every * P.STEP)))
            if tm["end"]:
                end_k = tm["k0"]
            elif tm["data"]:
                f = tm["k0"] // every
                frames[f] = dec(tm["data"], tuple(tm["shape"]))[0]
                bisect.insort(keys, f)
                if tm["k0"] < idx:
                    late_frames += 1
        queue[:] = rest

    def pose(idx):
        """이 스텝의 자세. 없으면 None."""
        a0 = idx // every
        i = bisect.bisect_right(keys, a0)
        if i == 0 or i >= len(keys):
            return None
        lo, hi = keys[i - 1], keys[i]
        if hi - lo > max_fill:
            return None
        f = (idx - lo * every) / ((hi - lo) * every)
        fr = (1 - f) * frames[lo] + f * frames[hi]
        fr[:, 3:7] = fr[:, 3:7] / np.linalg.norm(fr[:, 3:7], axis=1, keepdims=True)
        return fr

    def write():
        r, c, h = to_cells(*changed_nodes(ter, sent))
        if r.size:
            low = h < m.local.compose(0, m.local.nrow, 0, m.local.ncol)[r, c] - THRESH
            if low.any():
                m.write_cells("dig", "rear", r[low], c[low], h[low], t=t)

    def advance(c, target):
        """다음 재생 위치. 반환: (위치, 몇 배로 갔나)."""
        if c >= target:
            return target, 1
        last = keys[-1] * every
        if c >= last:                                          # 앞에 받은 궤적이 없다
            return c, 0
        i = bisect.bisect_right(keys, c // every)
        if i == 0:
            return min(target, keys[0] * every), 0
        lo, hi = keys[i - 1], keys[i]
        if hi - lo > max_fill:                                 # 뒤 프레임이 이미 왔으니 (순서대로 온다) 이 구멍은 안 온다
            return min(target, hi * every), 0
        rate = R if last - c >= 2 * every else 1
        return min(target, last, c + rate), rate

    t0 = time.monotonic()
    while len(keys) < 2:                                       # 시작 맞추기만 기다린다 (첫 두 프레임)
        receive(0)
        time.sleep(0.001)
        if time.monotonic() - t0 > 120:
            raise RuntimeError("앞 차 궤적이 오지 않는다")
    wall = time.perf_counter()
    while True:
        target = max(0, k - delay)
        receive(cur)
        cur, rate = advance(cur, target)
        behind = max(behind, target - cur)
        fast += rate > 1 and target - cur > 0
        idx = cur
        if end_k is None or idx < end_k - every:
            fr = pose(idx)
            if fr is None:
                hide_ghosts(P, bodies)
                missed += 1
            else:
                P.place_ghosts(bodies, fr)
                hid += hide_late_ghosts(P, car, bodies, fr)
        else:
            hide_ghosts(P, bodies)
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
        lag = max(lag, pace_wait(wall, t, a.pace))
        if k % PACE_LOG == 0:                                  # 0.5 s마다: 시뮬레이션 시각, 실제 경과 시간, 뒤 차 y
            plog.append((t, time.perf_counter() - wall, y))
        if y > P.WINDOW[1] + 3.0:
            break
    write()
    wall = time.perf_counter() - wall
    au = m.audit(wait_s=120.0)
    final = m.local.compose(0, m.local.nrow, 0, m.local.ncol)
    save(a.out, heights={(n.x, n.y): v for n, v in ter.GetModifiedNodes(True)}, wall=wall, sim=t, audit=json.dumps(au), log=np.array(log), final_map=final,
         waited=0.0, pace=a.pace, i0=i0, j0=j0, frames=np.array([frames[f] for f in keys]), hid=hid, missed=missed, late_frames=late_frames, lag=lag, steps=k, behind=behind * P.STEP, fast=fast, plog=np.array(plog))
    m.close()


def hide_late_ghosts(P, car, bodies, frame):
    """자기 차 앞바퀴에 GUARD m 안까지 왔거나 지나간 고스트 바퀴를 숨긴다 (이 궤적은 늦었다. 고스트는 충돌 몸체라 겹치면 차에 부딪힌다). 반환: 숨긴 바퀴 수."""
    v = car.car.GetVehicle()
    pl, pr = v.GetSpindlePos(0, P.veh.LEFT), v.GetSpindlePos(0, P.veh.RIGHT)
    cx, cy = (pl.x + pr.x) / 2, (pl.y + pr.y) / 2
    fwd = P.chrono.ChMatrix33d(v.GetRot()).GetAxisX()
    n = 0
    for b, r in zip(bodies, frame):
        dx, dy = r[0] - cx, r[1] - cy
        if dx * fwd.x + dy * fwd.y < GUARD and abs(-dx * fwd.y + dy * fwd.x) < 2.0:
            b.SetPos(P.chrono.ChVector3d(0, 0, 50))
            b.SetPosDt(P.chrono.ChVector3d(0, 0, 0))
            b.SetAngVelParent(P.chrono.ChVector3d(0, 0, 0))
            n += 1
    return n


def role_rear(a):
    import probe_scm_two_hmmwv as P
    setup(P)
    P.GAP = a.gap
    m, to_cells, i0, j0 = client(P, "rear", a)
    s = P.make_system()
    ter, cy = P.make_terrain(s, a.kphi)
    bodies = P.ghost_bodies(s, ter, 4)
    car = P.Car(s, ter, P.Y_FRONT - P.GAP, a.rear_x[0])
    delay = int(round(a.delay / P.STEP))
    frames, ended, every = [], False, None
    sent, log, t, k = {}, [], 0.0, 0
    ghost_on, waited, hid = True, 0.0, 0
    m.send_traj("rear", 0, 1, np.zeros((0, 4, 13)), (P.TIRE_R, P.TIRE_W))   # 준비 끝: 앞 차가 이것을 보고 출발한다

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
        r, c, h = to_cells(*changed_nodes(ter, sent))
        if r.size:
            low = h < m.local.compose(0, m.local.nrow, 0, m.local.ncol)[r, c] - THRESH   # 고스트가 만든 칸은 지도에 이미 있다
            if low.any():
                m.write_cells("dig", "rear", r[low], c[low], h[low], t=t)

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
                hid += hide_late_ghosts(P, car, bodies, fr)
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
         waited=waited, i0=i0, j0=j0, frames=np.array(frames), hid=hid)
    m.close()


def role_truth(a):
    import probe_scm_two_hmmwv as P
    setup(P)
    if a.truth_fast:                                           # 정답도 작은 활성 영역으로 (아주 작은 교란에 정답 범위가 얼마나 넓어지나)
        a.fast = True
        apply_fast(P, a)
    P.GAP = a.gap
    P.REAR_X = a.rear_x[0]
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
    print(f"\n== kphi {kphi:g}: 뒤 차 값, y {P.WINDOW[0]:g} ~ {P.WINDOW[1]:g} m (궤적 {a.hz:g} Hz, 지연 {a.delay:g} s)")
    print(f"{'':20s}{'추가 침하':>9s}{'끝 깊이':>8s}{'바퀴 높이':>9s}{'스로틀':>7s}{'벗어남cm':>9s}{'옆흔들림':>9s}")
    for name, st in (("뒤 차 (정답 A)", st_t), ("뒤 차 (두 프로세스)", st_r)):
        print(f"{name:20s}{st['added_sink_cm']:9.2f}{st['final_depth_cm']:8.2f}{st['spindle_z_m'] * 100:9.2f}{st['throttle']:7.3f}{st['lateral_m'] * 100:9.1f}{st['roll_rms_deg']:9.3f}")
    print(f"{'뒤 차 차이':20s}{st_r['added_sink_cm'] - st_t['added_sink_cm']:+9.2f}{st_r['final_depth_cm'] - st_t['final_depth_cm']:+8.2f}{(st_r['spindle_z_m'] - st_t['spindle_z_m']) * 100:+9.2f}{st_r['throttle'] - st_t['throttle']:+7.3f}")
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


def pace_summary(rd, window=(-8.0, 12.0)):
    """0.5 s 구간별 뒤 차 빠르기 (기다린 시간 포함이라 --pace 1이면 1배 근처). 가장 느린 구간과, 실제 시각이 시뮬레이션보다 늦은 정도.
    비교 구간 (뒤 차 y가 window 안) 따로: 그 뒤에는 앞 차가 끝나며 마무리 작업 (지도 해시 검사 등)을 한다."""
    if "plog" not in rd or len(rd["plog"]) < 3:
        return ""
    pl = rd["plog"]
    sp = np.diff(pl[:, 0]) / np.maximum(np.diff(pl[:, 1]), 1e-9)
    late = pl[:, 1] - pl[:, 0] / max(float(rd["pace"]), 1e-9)
    inw = (pl[:, 2] >= window[0]) & (pl[:, 2] <= window[1])
    i = int(np.argmin(sp))
    j = int(np.argmax(late))
    lw = late[inw].max() if inw.any() else float("nan")
    return (f" | 0.5 s 구간 빠르기 최소 {sp[i]:.2f}배 (y {pl[i + 1, 2]:.1f} m), 0.9배 미만 {int((sp < 0.9).sum())}/{len(sp)}, "
            f"가장 늦음 {1000 * late[j]:.0f} ms (y {pl[j, 2]:.1f} m), 비교 구간 안 가장 늦음 {1000 * lw:.0f} ms")


def hole_profile(P, T, R, F, gs, y0, y1, i0, cy, j_lo, j_hi):
    """끊긴 궤적의 영향을 y로 나눠 본다. 바퀴는 차 중심 앞뒤로 있어 구멍 양쪽 2 m까지 영향권으로 둔다."""
    def cells(h_a, h_b, j_a, j_b):
        return cell_diff(h_a, h_b, i0, j_a, j_b)
    def j_of(y):
        return int(round((y - cy) / P.DELTA))
    def depth(h, j_a, j_b):
        pr = P.track_profile(h, cy)
        v = [-100 * x for j in range(j_a, j_b) for x in pr.get(j, [0.0, 0.0])]
        return float(np.mean(v)) if v else 0.0
    def wheel(lg, y_a, y_b, col):
        m = (lg[:, 2] >= y_a) & (lg[:, 2] < y_b)
        return 100 * float(lg[m, col].mean()) if m.any() else float("nan")
    def rng(v, f="+.2f"):
        return f"{min(v):{f}} ~ {max(v):{f}}"

    print(f"  y 칸별 (m). 차이는 지도 - 정답 (cm, + = 지도가 얕음). 깊이는 왼쪽, 오른쪽 자국의 가장 깊은 곳 평균, 출발 위치 {len(gs)}개 평균")
    print(f"  {'y':>6s} {'정답끼리|평균|최대':>16s} {'지도-정답 평균':>18s} {'뒤SCM-정답 평균':>18s} {'앞차':>6s} {'정답':>6s} {'지도':>6s} {'뒤SCM':>6s} {'바퀴높이 차':>16s} {'옆 위치 차':>16s}")
    TH = {g: T[g][1] for g in gs}
    MP = {g: R[g][3] for g in gs}
    RH = {g: R[g][1] for g in gs}
    for e in np.arange(P.WINDOW[0], P.WINDOW[1], 1.0):
        ja, jb = j_of(e), j_of(e + 1.0)
        base = [float(np.abs(cells(TH[ga], TH[gb], ja, jb)).mean()) for i, ga in enumerate(gs) for gb in gs[i + 1:]]
        dm = [float(cells(TH[g], MP[g], ja, jb).mean()) for g in gs]
        dr = [float(cells(TH[g], RH[g], ja, jb).mean()) for g in gs]
        dep = [np.mean([depth(src[g], ja, jb) for g in gs]) for src in (F, TH, MP, RH)]
        dz = [wheel(R[g][0], e, e + 1.0, 6) - wheel(T[g][0], e, e + 1.0, 6) for g in gs]
        dx = [wheel(R[g][0], e, e + 1.0, 1) - wheel(T[g][0], e, e + 1.0, 1) for g in gs]
        mark = "*" if y0 <= e + 1.0 and e <= y1 else " "
        print(f" {mark}{e:6.1f} {max(base):16.3f} {rng(dm):>18s} {rng(dr):>18s} {dep[0]:6.1f} {dep[1]:6.1f} {dep[2]:6.1f} {dep[3]:6.1f} {rng(dz):>16s} {rng(dx, '+.1f'):>16s}", flush=True)

    print(f"  세 구역 (구멍 y {y0:.1f} ~ {y1:.1f} m, 양쪽 2 m까지 영향권). 몫 = 그 구역이 전체 지도 평균에 보탠 값 (cm)")
    zones = (("구멍 앞", P.WINDOW[0], y0 - 2.0), ("구멍과 영향권", y0 - 2.0, y1 + 2.0), ("구멍 뒤", y1 + 2.0, P.WINDOW[1]))
    for name, za, zb in zones:
        ja, jb = j_of(za), j_of(zb)
        base = [float(np.abs(cells(TH[ga], TH[gb], ja, jb)).mean()) for i, ga in enumerate(gs) for gb in gs[i + 1:]]
        bm = [float(cells(TH[ga], TH[gb], ja, jb).mean()) for i, ga in enumerate(gs) for gb in gs[i + 1:]]
        dm, da, sh, dz, dx = [], [], [], [], []
        for g in gs:
            d = cells(TH[g], MP[g], ja, jb)
            n = cell_diff(TH[g], MP[g], i0, j_lo, j_hi).size
            dm.append(float(d.mean()))
            da.append(float(np.abs(d).mean()))
            sh.append(float(d.sum()) / n)
            dz.append(wheel(R[g][0], za, zb, 6) - wheel(T[g][0], za, zb, 6))
            lr, lt = R[g][0], T[g][0]
            yy = np.arange(za, zb, 0.25)
            dx.append(100 * float(np.sqrt(np.mean((np.interp(yy, lr[:, 2], lr[:, 1]) - np.interp(yy, lt[:, 2], lt[:, 1])) ** 2))))
        print(f"    {name} y {za:.1f} ~ {zb:.1f}: 지도-정답 평균 {rng(dm)}, |평균| {rng(da, '.2f')}, 몫 {rng(sh)} | 정답끼리 평균 {rng(bm)}, |평균| {rng(base, '.2f')} | 바퀴높이 차 {rng(dz)} | 옆 궤적 차이 RMS {rng(dx, '.1f')}", flush=True)


def spread_compare(a, dirn, kphi):
    """chrono 환경에서: 정답끼리의 범위와 두 프로세스 - 정답."""
    import probe_scm_two_hmmwv as P
    setup(P)
    nrow, ncol, i0, j0 = grid_shape(P)
    cy = (P.SCM_Y[0] + P.SCM_Y[1]) / 2
    j_lo, j_hi = int(round((P.WINDOW[0] - cy) / P.DELTA)), int(round((P.WINDOW[1] - cy) / P.DELTA))
    T, R, F = {}, {}, {}
    fh = None
    for g in a.spread:
        td, th = load(dirn / f"truth_{g}.npz")
        rd, rh = load(dirn / f"rear_{g}.npz")
        fd, fh = load(dirn / f"front_{g}.npz")
        F[g] = fh
        T[g] = (td["log"], th, P.window_stats({"log": td["log"], "final": th, "cy": cy}, fh))
        final = rd["final_map"]
        mp = {(int(i - i0), int(j - j0)): float(final[j, i]) for j in range(nrow) for i in range(ncol) if final[j, i] != 0.0}
        R[g] = (rd["log"], rh, P.window_stats({"log": rd["log"], "final": rh, "cy": cy}, fh), mp)
        sha = __import__("hashlib").sha1(rd["log"].tobytes()).hexdigest()[:10]
        print(f"  [간격 {g}] 뒤 차 기록 {sha}, 숨긴 고스트 바퀴 {int(rd['hid'])} 스텝 x 바퀴 ({100 * int(rd['hid']) / max(1, 4 * len(rd['frames']) * 20):.0f} %)"
              + (f", 고스트 쉼 {int(rd['missed'])}/{int(rd['steps'])} 스텝, 재생 뒤 도착한 프레임 {int(rd['late_frames'])}, 배속 늦음 최대 {1000 * float(rd['lag']):.0f} ms, 재생 밀림 최대 {float(rd['behind']):.2f} s, 빨리 재생 {int(rd['fast'])} 스텝" if "missed" in rd else "")
              + f", 빠르기 앞 차 {float(fd['sim']) / float(fd['wall']):.2f}배, 뒤 차 {float(rd['sim']) / float(rd['wall']):.2f}배" + pace_summary(rd))
        au = json.loads(str(rd["audit"]))
        if au.get("hash_mismatch") != 0:
            print(f"간격 {g}: 뒤 차 해시 불일치 {au}")
    print(f"\n== kphi {kphi:g}, 뒤 차 옆으로 {100 * a.rear_x[0]:g} cm, 지연 {a.delay:g} s{', 경우 ' + a.case if a.case else ''}. 각 쌍: 칸별 |평균| (cm), 평균 (cm), 뒤 차 옆 궤적 차이 RMS (cm)")
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
    if a.outage:                                               # 끊긴 궤적이 누르지 못한 구간만
        ys = np.load(dirn / f"front_{gs[0]}.npz")["ys"]
        y0, y1 = np.interp(a.outage[0], ys[:, 0], ys[:, 1]), np.interp(a.outage[0] + a.outage[1], ys[:, 0], ys[:, 1])
        for name, src in (("정답 A", T), ("두 프로세스", R)):
            v = [100 * float(src[g][0][(src[g][0][:, 2] >= y0) & (src[g][0][:, 2] <= y1), 6].mean()) for g in gs]
            print(f"  구멍 구간 y {y0:.1f} ~ {y1:.1f} m 바퀴 높이 {name}: {min(v):.2f} ~ {max(v):.2f} cm")
        hole_profile(P, T, R, F, gs, float(y0), float(y1), i0, cy, j_lo, j_hi)
    pa, oa = np.array([p[:3] for p in pairs]), np.array(own)
    print(f"  정답끼리 {len(pairs)}쌍: |평균| {pa[:, 0].min():.3f} ~ {pa[:, 0].max():.3f}, 평균 {pa[:, 1].min():+.3f} ~ {pa[:, 1].max():+.3f}, 궤적 {pa[:, 2].min():.1f} ~ {pa[:, 2].max():.1f}")
    print(f"  지도 - 정답 {len(own)}쌍: |평균| {oa[:, 0].min():.3f} ~ {oa[:, 0].max():.3f}, 평균 {oa[:, 1].min():+.3f} ~ {oa[:, 1].max():+.3f}, 궤적 {oa[:, 2].min():.1f} ~ {oa[:, 2].max():.1f}")
    allp = np.vstack([pa, oa])
    print(f"  칸별 |평균|과 궤적 차이의 상관 ({len(allp)}쌍): {np.corrcoef(allp[:, 0], allp[:, 2])[0, 1]:+.2f}")
    keys = ("added_sink_cm", "final_depth_cm", "spindle_z_m", "throttle", "lateral_m", "speed_mps")
    for name, rows in (("뒤 차 (정답 A)", [T[g][2] for g in gs]), ("뒤 차 (두 프로세스)", [R[g][2] for g in gs])):
        print(f"  {name:20s}" + "  ".join(f"{k.split('_')[0]} {min(r[k] for r in rows) * (100 if k in ('spindle_z_m', 'lateral_m') else 1):.3f} ~ {max(r[k] for r in rows) * (100 if k in ('spindle_z_m', 'lateral_m') else 1):.3f}" for k in keys))
    for name, logs in (("뒤 차 (정답 A)", [T[g][0] for g in gs]), ("뒤 차 (두 프로세스)", [R[g][0] for g in gs])):
        xm = [100 * float(lg[:, 1].mean()) for lg in logs]                        # 평균 옆 위치 (앞 차 길 기준, cm)
        xe = [100 * float(lg[lg[:, 2] > P.WINDOW[1] - 2.0, 1].mean()) for lg in logs]   # 구간 끝 2 m의 옆 위치
        st = [float(np.sqrt(np.mean(lg[:, 10] ** 2))) for lg in logs]             # 조향 입력 RMS
        rl = [float(np.std(lg[:, 5])) for lg in logs]
        print(f"  {name:20s}옆 위치 평균 {min(xm):.1f} ~ {max(xm):.1f} cm  구간 끝 {min(xe):.1f} ~ {max(xe):.1f} cm  조향 RMS {min(st):.4f} ~ {max(st):.4f}  옆기울기 {min(rl):.3f} ~ {max(rl):.3f} deg")


def spread(a, me, env):
    cache = tempfile.TemporaryDirectory(prefix="two_chrono_truth_")      # 정답은 지연과 상관없다
    def case_flags(c):
        if c == "wait":
            return []
        out = ["--pace", str(a.pace or 0.4), "--catchup", str(a.catchup), "--threads", str(a.threads)] + (["--fast"] if a.fast else []) + (["--truth-fast"] if a.truth_fast else [])
        if c.startswith("drop"):
            out += ["--drop-traj", c[4:]]
        elif c.startswith("out"):
            out += ["--outage", str(a.outage_at), c[3:]]
        elif c.startswith("stall"):
            out += ["--stall", str(a.outage_at), c[5:]]
        elif c.startswith("late"):
            out += ["--net-delay", c[4:]]
        else:
            assert c == "base", c
        return out

    for kphi, off, delay, case in [(k, x, d, c) for k in a.kphi for x in a.rear_x for d in (a.delays or [a.delay]) for c in (a.cases or ["wait"])]:
        with tempfile.TemporaryDirectory(prefix="two_chrono_") as tmp:
            dirn = Path(tmp)
            for g in a.spread:
                s = socket.socket()
                s.bind(("127.0.0.1", 0))
                port = s.getsockname()[1]
                s.close()
                server = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--tcp-port", str(port)], cwd=ROOT,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                common = ["--kphi", str(kphi), "--hz", str(a.hz), "--delay", str(delay), "--tcp", f"127.0.0.1:{port}", "--gap", str(g), "--rear-x", str(off), "--route", a.route] + case_flags(case)
                try:
                    front = subprocess.Popen(me + ["--role", "front", "--out", str(dirn / f"front_{g}.npz")] + common, env=env, cwd=ROOT)
                    time.sleep(1.0)
                    rear = subprocess.Popen(me + ["--role", "rear", "--out", str(dirn / f"rear_{g}.npz")] + common, env=env, cwd=ROOT)
                    for p in (front, rear):
                        assert p.wait(timeout=1800) == 0
                finally:
                    server.terminate()
                    server.wait(timeout=10)
                cached = Path(cache.name) / f"truth_{a.route}_{kphi:g}_{off:g}_{g}{'_fast' if a.truth_fast else ''}.npz"
                if not cached.exists():
                    subprocess.run(me + ["--role", "truth", "--out", str(cached), *common], env=env, cwd=ROOT, check=True, timeout=1800)
                shutil.copy(cached, dirn / f"truth_{g}.npz")
                print(f"[kphi {kphi:g} 옆 {off:g} 지연 {delay:g} 간격 {g} 끝]", flush=True)
            subprocess.run(me + ["--role", "spread_compare", "--dir", str(dirn), "--route", a.route, "--kphi", str(kphi), "--rear-x", str(off), "--delay", str(delay), "--spread", *map(str, a.spread), "--case", case if case != "wait" else ""] + (["--outage", str(a.outage_at), case[3:]] if case.startswith("out") else ["--outage", str(a.outage_at), case[5:]] if case.startswith("stall") else []), env=env, cwd=ROOT, check=True, timeout=900)


def robot_series(run):
    import pandas as pd
    ts = pd.read_parquet(run / "timeseries.parquet")
    pos = np.column_stack([ts[f"base_pos_{i}"] for i in range(3)]) if "base_pos_0" in ts else np.stack(ts["base_pos"].to_numpy())
    quat = np.column_stack([ts[f"base_quat_wxyz_{i}"] for i in range(4)]) if "base_quat_wxyz_0" in ts else np.stack(ts["base_quat_wxyz"].to_numpy())
    return ts["t"].to_numpy(), pos, quat


def robot_row(name, run, t_cmd):
    t, pos, q = robot_series(run)
    sel = t > t_cmd
    w, x, y, z = q[sel].T
    roll, pitch = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))), np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    su = json.loads((run / "summary.json").read_text())["mop"]
    print(f"{name:22s}{pos[-1, 0]:8.2f}{pos[-1, 1]:8.2f}{pos[sel, 2].min():8.3f}{np.abs(roll).max():8.1f}{np.abs(pitch).max():8.1f}{su.get('map_received_changes', 0):7d}")


def runner_once(path):
    """맵 서버 없이 시뮬레이터 한 번 (lockstep). 반환: 실행 폴더."""
    runs = ROOT / "runs"
    before = set(os.listdir(runs))
    subprocess.run([sys.executable, "-m", "sim.runner", str(path)], cwd=ROOT, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
    new = sorted(set(os.listdir(runs)) - before)
    assert len(new) == 1, new
    return runs / new[0]


def robot_report(a, run, dirn, t_cmd, me, env):
    """받은 자국과 로봇 위치, 다시 돌리기(비트 비교), 정답 지도(한 SCM)와 평평한 땅 위의 로봇."""
    import shutil
    import yaml
    from sim.map_client import enc, msg_cells
    replay = yaml.safe_load((run / "scenario_replay.yaml").read_text())
    wr = [e for e in replay["events"] if e["action"] == "map_write"]
    print(f"받은 변경 {len(wr)}개: " + ", ".join(f"{w} {sum(e['msg']['writer'] == w for e in wr)}개" for w in sorted({e['msg']['writer'] for e in wr})))
    final = np.load(dirn / "rear.npz")["final_map"]
    nrow, ncol = final.shape
    res = replay["terrain"]["resolution"]
    half = (nrow - 1) * res / 2                                     # terrain.size는 반폭이다
    last, low = 0.0, 0.0
    for e in wr:
        rows, _, vals = msg_cells(e["msg"])
        s = np.abs(rows * res - half) < 0.6
        if s.any():
            last, low = max(last, e["t"]), min(low, float(np.nanmin(vals[s])))
    t, pos, _ = robot_series(run)
    print(f"로봇 길(|y| < 0.6 m)의 마지막 변경 t = {last:.1f} s, 그때 로봇 x = {pos[np.searchsorted(t, last), 0]:.2f} m (출발 t = {t_cmd:g} s), 가장 깊은 값 {100 * low:.1f} cm")

    aux = []
    rep = runner_once(run / "scenario_replay.yaml")
    aux.append(rep)
    t2, pos2, _ = robot_series(rep)
    same = len(t) == len(t2) and np.array_equal(pos, pos2)
    print(f"다시 돌리기 (차 없이): 로봇 위치 비트 단위로 {'같다' if same else '다르다'}" + ("" if same else f" (최대 차이 {np.abs(pos[:min(len(t), len(t2))] - pos2[:min(len(t), len(t2))]).max():.4f} m)"))

    subprocess.run(me + ["--role", "truth", "--out", str(dirn / "truth.npz"), "--kphi", str(a.kphi[0])], env=env, cwd=ROOT, check=True, timeout=600)
    _, th = load(dirn / "truth.npz")
    cy = float(np.load(dirn / "truth.npz")["cy"])
    i0, j0 = int(round((a.lane_or3 + (ncol - 1) * res / 2) / res)), int(round((cy + (nrow - 1) * res / 2) / res))
    cells = [(j + j0, i + i0, h) for (i, j), h in th.items() if 0 <= j + j0 < nrow and 0 <= i + i0 < ncol and abs(h) > 1e-9]
    rows, cols, vals = (np.array(v) for v in zip(*cells))
    vals = np.clip(vals, *replay["terrain"]["z_range"])
    truth_map = np.zeros((nrow, ncol))
    truth_map[rows, cols] = vals
    mask = (np.abs(final) > 0.002) | (np.abs(truth_map) > 0.002)
    mask[:, int(round((5.0 + half) / res)):] = False                # 요철 (바탕) 영역은 뺀다
    diff = 100 * (final - truth_map)[mask]
    print(f"끝난 지도 - 정답 (지도 안 자국 칸 {diff.size}개): 평균 {diff.mean():+.3f}, |평균| {np.abs(diff).mean():.3f}, 최대 {np.abs(diff).max():.2f} cm")
    msg = {"type": "cells", "op": "dig", "writer": "truth", "t": t_cmd / 2, "n_cells": int(rows.size),
           "rows": __import__("base64").b64encode(rows.astype("<i4").tobytes()).decode(),
           "cols": __import__("base64").b64encode(cols.astype("<i4").tobytes()).decode(), "values": enc(vals)}
    tr = dict(replay)
    tr["name"] = replay["name"] + "_truth"
    tr["events"] = [e for e in replay["events"] if e["action"] != "map_write"] + [{"t": t_cmd / 2, "action": "map_write", "source": "map", "msg": msg}]
    tr["events"].sort(key=lambda e: e["t"])
    fl = dict(replay)
    fl["name"] = replay["name"] + "_flat"
    fl["events"] = [e for e in replay["events"] if e["action"] != "map_write"]
    (dirn / "truth.yaml").write_text(yaml.safe_dump(tr))
    (dirn / "flat.yaml").write_text(yaml.safe_dump(fl))
    run_t, run_f = runner_once(dirn / "truth.yaml"), runner_once(dirn / "flat.yaml")
    aux += [run_t, run_f]
    print(f"\n로봇 (출발 뒤)        {'끝 x':>6s}{'끝 y':>6s}{'몸통최저':>6s}{'옆기움':>6s}{'앞뒤기움':>5s}{'변경':>5s}")
    robot_row("두 프로세스 지도", run, t_cmd)
    robot_row("정답 지도 (한 SCM)", run_t, t_cmd)
    robot_row("자국 없음", run_f, t_cmd)
    for d in aux:
        shutil.rmtree(d)


def run_robot(a, env, me):
    """시뮬레이터(맵 서버 포함)를 띄우고, 지도가 생기면 차 두 프로세스를 붙인다."""
    from sim.map_client import RemoteMap
    import yaml
    t_cmd = next(e["t"] for e in yaml.safe_load(Path(a.robot).read_text())["events"] if e["action"] == "set_command")
    runs = ROOT / "runs"
    before = set(os.listdir(runs))
    with tempfile.TemporaryDirectory(prefix="two_chrono_") as tmp:
        dirn = Path(tmp)
        runner = subprocess.Popen([sys.executable, "-m", "sim.runner", a.robot, "--map-server", "--realtime"] + (["--godot"] if a.godot else []),
                                  cwd=ROOT, stdout=open(dirn / "runner.log", "w"), stderr=subprocess.STDOUT)
        try:
            ob = RemoteMap(None, name="probe_robot", wait_s=120.0)
            port = ob.info["tcp_port"]
            ob.close()
            common = ["--kphi", str(a.kphi[0]), "--hz", str(a.hz), "--delay", str(a.delay), "--tcp", f"127.0.0.1:{port}", "--lane", str(a.lane or 3.0)]
            front = subprocess.Popen(me + ["--role", "front", "--out", str(dirn / "front.npz")] + common, env=env, cwd=ROOT)
            time.sleep(1.0)
            rear = subprocess.Popen(me + ["--role", "rear", "--out", str(dirn / "rear.npz")] + common, env=env, cwd=ROOT)
            for name, p in (("앞 차", front), ("뒤 차", rear)):
                if p.wait(timeout=600) != 0:
                    raise SystemExit(f"{name} 프로세스가 실패했다")
            for name in ("front", "rear"):
                d = np.load(dirn / f"{name}.npz")
                au = json.loads(str(d["audit"]))
                print(f"{name}: 불일치 {au.get('hash_mismatch', au)}, 보냄 {au.get('sent')}, 빠르기 {float(d['sim']) / float(d['wall']):.2f}배, 걸린 시간 {float(d['wall']):.1f} s", flush=True)
            if runner.wait(timeout=300) != 0:
                print((dirn / "runner.log").read_text()[-2000:])
                raise SystemExit("시뮬레이터가 실패했다")
        finally:
            if runner.poll() is None:
                runner.terminate()
                runner.wait(timeout=30)
        new = sorted(set(os.listdir(runs)) - before)
        assert len(new) == 1, new
        run = runs / new[0]
        a.lane_or3 = a.lane or 3.0
        robot_report(a, run, dirn, t_cmd, me, env)
    print(f"실행 폴더: {run.relative_to(ROOT)}")


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
    if a.robot:
        return run_robot(a, env, me)
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
    ap.add_argument("--delays", type=float, nargs="+", help="--spread: 궤적 지연 여러 값 (s)")
    ap.add_argument("--cases", nargs="+", help="--spread: 기다리지 않는 뒤 차의 경우들 (위 설명). wait은 기다리는 방식")
    ap.add_argument("--case", default="")
    ap.add_argument("--pace", type=float, help="두 프로세스의 배속 (실제 시간 대비). 있으면 뒤 차가 기다리지 않는다")
    ap.add_argument("--drop-traj", type=float, default=0.0)
    ap.add_argument("--outage", type=float, nargs=2)
    ap.add_argument("--stall", type=float, nargs=2, help="이 구간의 궤적을 보내지 않고 모았다가 끝나면 몰아서 보낸다 (시작, 길이 s)")
    ap.add_argument("--route", default="straight", choices=["straight", "curve"], help="길 (curve: S자, 진폭 1 m, 파장 16 m)")
    ap.add_argument("--fast", action="store_true", help="앞 차와 뒤 차의 SCM 활성 영역을 줄인다 (정답은 그대로)")
    ap.add_argument("--truth-fast", action="store_true", help="정답도 --fast의 활성 영역으로 돌린다")
    ap.add_argument("--threads", type=int, default=2, help="--fast일 때 뒤 차 Chrono 스레드 수")
    ap.add_argument("--catchup", type=int, default=4, help="밀린 궤적을 몇 배로 재생하나")
    ap.add_argument("--outage-at", type=float, default=6.0, help="--cases outN에서 끊김이 시작되는 시뮬레이션 시각 (s)")
    ap.add_argument("--net-delay", type=float, default=0.0)
    ap.add_argument("--role", choices=("front", "rear", "truth", "compare", "spread_compare"))
    ap.add_argument("--spread", type=float, nargs="*", help="뒤 차 출발 간격들 (m)")
    ap.add_argument("--gap", type=float, default=12.0)
    ap.add_argument("--rear-x", type=float, nargs="+", default=[0.0], help="뒤 차 길을 앞 차 길에서 옆으로 옮기는 거리 (m). --spread는 여러 값")
    ap.add_argument("--robot", help="이 시나리오의 시뮬레이터(로봇)에 지도를 붙인다")
    ap.add_argument("--godot", action="store_true", help="--robot: 화면으로 보기")
    ap.add_argument("--lane", type=float, help="차가 지나는 지도 x (m). --robot이면 기본 3")
    ap.add_argument("--tcp")
    ap.add_argument("--out")
    ap.add_argument("--dir")
    a = ap.parse_args()
    global ROUTE
    ROUTE = a.route
    if a.role is None:
        return orchestrate(a)
    a.kphi = a.kphi[0]
    if a.role == "spread_compare":
        return spread_compare(a, Path(a.dir), a.kphi)
    if a.role == "compare":
        return compare(a, Path(a.dir), a.kphi)
    {"front": role_front, "rear": role_rear_free if a.pace else role_rear, "truth": role_truth}[a.role](a)


if __name__ == "__main__":
    main()
