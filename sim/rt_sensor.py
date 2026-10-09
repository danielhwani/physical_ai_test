"""센서(LiDAR) 계산 작업 프로세스: C++ 실시간 코어 경로에서 관리 프로세스(sim/rt_link.py)의 짐을 덜어 준다.

LiDAR 한 번에 이 PC에서 약 98 ms(최대 107 ms)라, 관리 프로세스 안에서 계산하면 10 Hz 스캔만으로 실시간을 넘쳐 같은 프로세스의 두뇌가
밀렸다 (지형 인지 보행이 실시간에서 거의 걷지 못함). 작업 프로세스 하나로도 한 코어를 98% 써서 조금만 밀리면 스캔이 쌓였다
(두뇌에 도착하기까지 평균 0.4~0.56초). 그래서 작업 프로세스 여러 개가 스캔을 번갈아 계산한다.
  관리 프로세스: 스캔할 차례인지 판단 (viz/sensor_renderer.py와 같은 규칙), 지형 패치는 모든 작업 프로세스에, 스캔 작업(시각, 바디 포즈,
                 스캔 번호)은 하나에 보낸다. 스캔 번호로 잡음이 정해지므로 어느 작업 프로세스가 계산해도 같은 점군이다
  작업 프로세스: 같은 SensorRenderer로 계산해 돌려준다
lockstep에서는 결과를 기다리므로 sim.runner와 같은 스캔이 같은 스텝에 들어간다. 실시간에서는 계산 시간만큼 늦게 도착하고,
작업 프로세스가 모두 밀려 있으면 그 스캔은 건너뛴다 (실제 LiDAR 드라이버의 프레임 버림과 같다. dropped로 센다).
"""
import multiprocessing as mp


def _worker(conn, manifest, specs, terrain_half):
    from viz.sensor_renderer import SensorRenderer
    r = SensorRenderer(manifest, specs, terrain_half=terrain_half)
    lidars = {l.spec.name: l for l in r.lidars}
    while True:
        msg = conn.recv()
        if msg is None:
            break
        if msg[0] == "patch":
            for p in msg[1]:
                r.apply_patch(p)
            continue
        _, t, poses, frames, due = msg
        r.set_poses(poses)
        for name, l in lidars.items():            # 이번 차례인 센서만, 관리 프로세스가 정한 스캔 번호로
            l.next_t = t if name in due else float("inf")
            l.frame = frames.get(name, l.frame)
        conn.send([(name, scan) for name, scan in r.render(t)])


class SensorWorker:
    def __init__(self, sim, workers=1, max_backlog=1):
        r = sim.sensor_renderer                   # 관리 프로세스 쪽은 스캔 차례 판단에만 쓴다 (계산은 작업 프로세스)
        self.sim, self.max_backlog = sim, max_backlog
        self.timers = {l.spec.name: [l.next_t, l.period, 0] for l in r.lidars}     # 다음 시각, 주기, 스캔 번호
        ctx = mp.get_context("spawn")             # rclpy, MuJoCo 상태를 물려받지 않게
        defs = sim.sensor_defs["lidars"]
        args = (sim.tap.manifest, {n: defs[n] for n in self.timers}, min(8.0, max(defs[n]["range_max"] for n in self.timers)))
        self.conns, self.procs = [], []
        for _ in range(workers):
            conn, child = ctx.Pipe()
            proc = ctx.Process(target=_worker, args=(child, *args), daemon=True)
            proc.start()
            self.conns.append(conn); self.procs.append(proc)
        self.inflight = []                        # [(작업 프로세스 번호, 덧붙임)] 보낸 순서
        self.next_worker, self.dropped = 0, 0

    def submit(self, t, tag):
        """스캔 차례면 렌더 스트림 메시지를 보낸다. tag는 결과와 함께 돌려준다 (고장 상태 등)."""
        due = [n for n, (nt, _, _) in self.timers.items() if t >= nt - 1e-9]
        if not due:
            return False
        frames = {}
        for n in due:                             # SensorRenderer.render와 같은 갱신
            frames[n] = self.timers[n][2]
            self.timers[n][0] += self.timers[n][1]
            self.timers[n][2] += 1
        tap = self.sim.tap
        patches = tap.patches()
        if patches:                               # 지형은 모든 작업 프로세스가 같아야 한다
            for c in self.conns:
                c.send(("patch", patches))
        w = self.next_worker
        if sum(1 for i, _ in self.inflight if i == w) >= self.max_backlog:
            self.dropped += 1                     # 밀려 있음: 이 스캔은 건너뛴다
            return False
        self.next_worker = (w + 1) % len(self.conns)
        self.conns[w].send(("scan", t, tap.poses(), frames, due))
        self.inflight.append((w, tag))
        return True

    def results(self, wait=False):
        """끝난 작업 [(tag, [(이름, 스캔)])] (보낸 순서). wait면 보낸 것을 모두 기다린다."""
        out = []
        while self.inflight:
            w, tag = self.inflight[0]
            if not (wait or self.conns[w].poll()):
                break
            out.append((tag, self.conns[w].recv()))
            self.inflight.pop(0)
        return out

    def close(self):
        for c in self.conns:
            try:
                c.send(None)
            except (BrokenPipeError, OSError):
                pass
        for p in self.procs:
            p.join(timeout=3)
            if p.is_alive():
                p.terminate()
