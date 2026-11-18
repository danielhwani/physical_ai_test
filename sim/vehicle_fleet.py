"""시나리오의 vehicles: 차마다 별도 Chrono 프로세스 (cosim/vehicle_peer.py)를 띄우고 맵 서버로 자국을 주고받게 한다 (맵 서버 4단계 운용).

    vehicles:
      soil: {bekker_kphi: 5.0e6, ...}     # SCM 흙 (모든 차 같음, 빼면 단단한 흙)
      hz: 100                             # 바퀴 궤적 빈도
      delay: 0.2                          # 받은 궤적을 늦춰 재생하는 시간 (s)
      catchup: 4                          # 밀린 궤적 재생 배속
      threads: 2                          # 차마다 Chrono 스레드
      margin: 6.0                         # 차 SCM이 길 양옆, 앞뒤로 덮는 거리 (m)
      yield_to_robot: true                # 차 앞 진로 (앞 9 m, 옆 ±1.8 m)에 로봇이 있으면 제동하고 기다린다 (yield_ahead, yield_half_width)
      cars:
        - {name: car_a, start: [3.0, -14.0], end: [3.0, 26.0], speed: 3.0, t_start: 0.0}

로봇 시뮬레이터는 지도 높이만 받는다 (차와 물리 작용 없음). 맵 서버 (--map-server)와 실시간 (--realtime)이 필요해 러너가 켠다.
차 프로세스는 시뮬레이션 시각 0을 러너와 같은 벽시계로 맞추고 실시간 1배로 달린다. 결과는 결정적이지 않다 (실시간).
차 결과는 실행 폴더 vehicles/<이름>.json (궤적 0.2 s마다, 늦음, 고스트 통계, 지도 대조), 진행 기록은 vehicles/<이름>.log.

렌더러: 차 프로세스가 READY 때 알린 바디와 외형을 장면 매니페스트에 넣고 (manifest_bodies), 50 Hz로 보내는 바디 포즈의 최신 값을
TF로 보낸다 (stream_poses). 같은 프로세스 Chrono (ChronoLink)와 같은 모양이라 렌더 스트림은 둘을 구분하지 않는다.
포즈는 차의 지금 (벽시계) 위치다. 로봇 시뮬레이터가 실시간보다 느리면 화면의 로봇 시각과 어긋난다 (지도 자국도 차 시각으로 들어온다).
"""
import json
import os
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

from .chrono_link import find_chrono_python

ROOT = Path(__file__).resolve().parent.parent
KEYS = ("soil", "hz", "delay", "catchup", "threads", "margin", "yield_to_robot", "yield_ahead", "yield_half_width")   # 모든 차에 (차마다 덮어쓸 수 있다)


class VehicleFleet:
    def __init__(self, cfg, terrain, out_dir, duration):
        cars = cfg.get("cars") or []
        names = [c["name"] for c in cars]
        assert len(set(names)) == len(names), f"차 이름이 겹친다: {names}"
        port = terrain.server.info.get("tcp_port")
        assert port, "맵 서버에 TCP 입구가 없다 (sim.map_server_node --tcp-port)"
        self.dir = Path(out_dir) / "vehicles"
        self.dir.mkdir(parents=True, exist_ok=True)
        py = find_chrono_python(cfg)
        env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH")}   # ROS2 환경의 경로가 chrono 파이썬에 섞이지 않게
        common = {k: cfg[k] for k in KEYS if k in cfg}
        self.procs, self.ready, self.bodies, self.poses = {}, {}, {}, {}
        self.lock = threading.Lock()
        for c in cars:
            n = c["name"]
            conf = dict(common, **c, peers=[x for x in names if x != n], tcp=f"127.0.0.1:{port}", duration=duration,
                        grid={"half": [terrain.half_x, terrain.half_y], "res": terrain.res}, out=str(self.dir / f"{n}.json"))
            self.procs[n] = subprocess.Popen([py, str(ROOT / "cosim/vehicle_peer.py"), "--config", json.dumps(conf)], cwd=ROOT, env=env,
                                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=open(self.dir / f"{n}.log", "w"),
                                             text=True, bufsize=1)
            self.ready[n] = threading.Event()
            # 표준 출력은 늘 비운다 (포즈가 50 Hz로 오므로 안 읽으면 파이프가 차서 차 프로세스가 멈춘다)
            threading.Thread(target=self._read, args=(n,), daemon=True).start()

    def _read(self, n):
        for line in self.procs[n].stdout:
            head, _, body = line.partition(" ")
            if head == "POSE":
                t, poses = json.loads(body)
                with self.lock:
                    self.poses[n] = poses
            elif head.strip() == "READY":
                self.bodies[n] = json.loads(body) if body.strip() else []
                self.ready[n].set()

    def wait_ready(self, timeout=240.0):
        """모든 차가 맵 서버에 붙고 SCM을 만들 때까지."""
        t0 = time.monotonic()
        for n, p in self.procs.items():
            while not self.ready[n].wait(0.05):
                if p.poll() is not None:
                    raise RuntimeError(f"차 {n} 프로세스가 시작하지 못했다:\n{(self.dir / f'{n}.log').read_text()[-1500:]}")
                if time.monotonic() - t0 > timeout:
                    raise RuntimeError(f"차가 준비되지 않는다: {sorted(k for k, e in self.ready.items() if not e.is_set())}")

    # ---- 렌더 스트림 (sim/stream.py build_manifest, Ros2StreamPublisher가 ChronoLink와 같은 방식으로 부른다) ----
    def manifest_bodies(self):
        return [dict(b, physics_source="Chrono", entity=n) for n in self.procs for b in self.bodies.get(n, [])]

    def stream_poses(self):
        out = []
        with self.lock:
            for n in self.procs:
                for b, v in zip(self.bodies.get(n, []), self.poses.get(n, [])):
                    out.append((b["name"], np.array(v[:3]), np.array(v[3:])))
        return out

    def send_robot(self, x, y):
        """로봇 몸통 위치 (양보 판단용). 러너가 20 Hz로 부른다. 이미 끝난 차는 건너뛴다."""
        for p in self.procs.values():
            if p.poll() is None:
                try:
                    p.stdin.write(f"ROBOT {x:.3f} {y:.3f}\n")
                    p.stdin.flush()
                except (BrokenPipeError, OSError, ValueError):
                    pass

    def go(self, lead=1.0):
        """lead 초 뒤를 시뮬레이션 시각 0으로 알린다. 반환: 그 순간의 time.perf_counter (러너의 실시간 기준점)."""
        epoch = time.time() + lead
        base = time.perf_counter() + lead
        for p in self.procs.values():
            p.stdin.write(f"GO {epoch:.6f}\n")
            p.stdin.flush()
        while time.perf_counter() < base:
            time.sleep(min(0.01, base - time.perf_counter()))
        return base

    def close(self, timeout=60.0):
        """표준 입력을 닫아 끝내게 하고 결과를 모은다. 반환: {이름: 결과 요약}."""
        for p in self.procs.values():
            try:
                p.stdin.close()
            except OSError:
                pass
        out = {}
        for n, p in self.procs.items():
            try:
                p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                p.kill()
            f = self.dir / f"{n}.json"
            if f.exists():
                r = json.loads(f.read_text())
                out[n] = {"sim_s": r["sim_s"], "max_lag_s": r["max_lag_s"], "state": r["state"], "along_m": r["along_m"],
                          "yields": r.get("yields", 0), "yield_s": r.get("yield_s", 0.0), "hash_mismatch": r["audit"].get("hash_mismatch")}
            else:
                out[n] = {"error": f"결과 없음 (종료 코드 {p.returncode}, {self.dir / f'{n}.log'})"}
        return out

    def kill(self):
        for p in self.procs.values():
            if p.poll() is None:
                p.kill()
