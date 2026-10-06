"""Chrono 연동 (문서 §5): Chrono는 변형 지면(SCM)과 선택적으로 HMMWV, MuJoCo는 4족 로봇.

  Chrono SCM 높이 변화 ──▶ Terrain Map Service ──▶ MuJoCo heightfield   (지형, 5 Hz)
  Chrono HMMWV 바디 포즈 ──▶ 중립 렌더 스트림(physics_source: Chrono)      (25 Hz)
  MuJoCo 로봇 발 위치 + 수직 하중 ──▶ Chrono 발 대리 구가 흙을 누름 (발자국, robot_feet 설정 시)
  로봇과 HMMWV 사이 물리 작용은 없다. 근접은 논리 이벤트로만 판정한다 (문서 §5.4).
  발자국은 로봇이 흙에 빠지는 힘을 MuJoCo로 되돌리지 않는다: 패인 모양만 지형으로 돌아와 이후 걸음이 밟는다.

Chrono는 별도 프로세스(chrono conda 환경, Python 3.12)로 돌고 socketpair로 연결한다 (cosim/wire.py).
동기는 파이프라인 방식: 시각 T_k에 T_k 결과를 반영하고 곧바로 T_k+1을 요청하므로, 두 엔진이 같은 구간을
동시에 계산한다. 교환 시각이 시뮬레이션 시간으로 고정돼 있어 결과는 결정적이다.
"""
import atexit
import os
import socket
import subprocess
from pathlib import Path

import numpy as np

from cosim import wire

ROOT = Path(__file__).resolve().parent.parent


def find_chrono_python(cfg):
    """chrono 환경의 python: 시나리오 설정 > 환경변수 CHRONO_PYTHON > conda envs/chrono."""
    cands = [cfg.get("python"), os.environ.get("CHRONO_PYTHON")]
    for base in (os.environ.get("CONDA_EXE"), os.environ.get("CONDA_PYTHON_EXE")):
        if base:
            cands.append(str(Path(base).resolve().parents[1] / "envs/chrono/bin/python"))
    cands.append(str(Path.home() / "miniconda3/envs/chrono/bin/python"))
    for c in cands:
        if c and Path(c).exists():
            return c
    raise FileNotFoundError("chrono 환경의 python을 찾지 못했다. CHRONO_PYTHON 환경변수로 지정할 것")


class ChronoLink:
    def __init__(self, cfg, terrain, log_path=ROOT / "build/chrono_server.log"):
        self.cfg, self.terrain = cfg, terrain
        self.sync_dt = cfg.get("sync_dt", 0.04)
        self.terrain_every = max(1, round(cfg.get("terrain_dt", 0.2) / self.sync_dt))
        self.near_dist = cfg.get("near_distance", 3.0)
        self.has_vehicle = "vehicle" in cfg
        self.feet_enabled = bool(cfg.get("robot_feet"))
        self._fn_sum, self._n_obs, self._feet_pos = np.zeros(4), 0, np.zeros((4, 3))

        # 칸 단위 정합 확인 (문서 §5.3 해상도 정합): SCM 격자 = MuJoCo 지형 격자
        scm = cfg["scm"]
        res = 2 * terrain.half_x / (terrain.ncol - 1)
        assert abs(scm["resolution"] - res) < 1e-9, f"SCM 해상도 {scm['resolution']} != 지형 해상도 {res}"
        cell = terrain.cell_of(*scm["center"])
        assert cell is not None, "SCM 중심이 지형 격자점에 놓이지 않는다"
        self.r0, self.c0 = cell
        # SCM은 평평한 평면(z=0)에서 시작하므로 그 영역의 기본 지형도 0이어야 한다 (지형 단일 원천 유지)
        hx, hy = scm["size"][0] / 2, scm["size"][1] / 2
        region = (np.abs(terrain.X - scm["center"][0]) <= hx) & (np.abs(terrain.Y - scm["center"][1]) <= hy)
        assert np.all(terrain.target[region] == 0.0), "SCM 영역의 기본 지형은 평평(높이 0)해야 한다"

        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.sock, child = socket.socketpair()
        self.proc = subprocess.Popen(
            [find_chrono_python(cfg), str(ROOT / "cosim/chrono_server.py"), "--fd", str(child.fileno()),
             "--root", str(ROOT)], pass_fds=[child.fileno()],
            stdout=open(log_path, "w"), stderr=subprocess.STDOUT)
        child.close()
        atexit.register(self._kill)

        self.log_path = log_path
        try:
            wire.send(self.sock, {"cmd": "init", "config": cfg})
            info = wire.recv(self.sock)
        except ConnectionError:
            tail = "".join(open(log_path).readlines()[-5:])
            raise RuntimeError(f"Chrono 서버 시작 실패 (로그 {log_path}):\n{tail}") from None
        self.bodies = info["bodies"]
        self.names = [b["name"] for b in self.bodies]
        wire.send(self.sock, {"cmd": "advance", "t": 0.0, "terrain": False})
        self._apply(wire.recv(self.sock))
        self.k = 0
        self.min_dist, self.near_event_sent = np.inf, False
        self._request(1)

    # ---- 동기 ----
    def _request(self, k):
        msg = {"cmd": "advance", "t": round(k * self.sync_dt, 9), "terrain": k % self.terrain_every == 0}
        if self.feet_enabled and self._n_obs:
            # 지난 교환 이후 제어 주기들의 평균 수직 하중과 마지막 발 위치 (다음 구간 동안 Chrono가 사용)
            fn = self._fn_sum / self._n_obs
            msg["feet"] = [{"pos": p.tolist(), "fn": float(f)} for p, f in zip(self._feet_pos, fn)]
            self._fn_sum[:], self._n_obs = 0.0, 0
        wire.send(self.sock, msg)
        self.pending_t = k * self.sync_dt

    def observe_feet(self, positions, normal_forces):
        """제어 주기마다 호출: 발 중심 월드 위치(4x3)와 지면 수직 하중(N, 4)."""
        self._feet_pos[:] = positions
        self._fn_sum += normal_forces
        self._n_obs += 1

    def sync(self, t):
        """제어 주기 시작마다 호출. 교환 시각에 도달했으면 결과를 반영하고 다음 구간을 요청."""
        while t >= self.pending_t - 1e-9:
            self._apply(wire.recv(self.sock))
            self.k += 1
            self._request(self.k + 1)

    def _apply(self, r):
        self.t = r["t"]
        self.poses = [(np.array(p["pos"]), np.array(p["quat"])) for p in r["poses"]]
        self.vehicle = r["vehicle"]
        self.foot_state = r.get("feet", [])
        tr = r["terrain"]
        if tr:
            i = np.frombuffer(wire.unb64(tr["i"]), "<i4")
            j = np.frombuffer(wire.unb64(tr["j"]), "<i4")
            h = np.frombuffer(wire.unb64(tr["h"]), "<f4").astype(float)
            # SCM 노드 (i, j) = 기준 평면 원점에서 (i*δ, j*δ). 격자 행 = y, 열 = x
            self.terrain.set_cells(self.r0 + j, self.c0 + i, h)

    # ---- 로봇과의 관계 (물리 작용 없음, 논리 판정만) ----
    def distance_to(self, xy):
        if not self.has_vehicle:
            return np.inf
        d = float(np.linalg.norm(np.array(self.vehicle["pos"][:2]) - xy))
        self.min_dist = min(self.min_dist, d)
        return d

    # ---- 스트림용 ----
    def manifest_bodies(self):
        return [dict(b, physics_source="Chrono", entity="hmmwv") for b in self.bodies]

    def stream_poses(self):
        return [(n, p, q) for n, (p, q) in zip(self.names, self.poses)]

    def close(self):
        if self.proc.poll() is not None:
            return None
        try:
            wire.recv(self.sock)                    # 요청해 둔 구간의 응답을 비운다
            wire.send(self.sock, {"cmd": "close"})
            stats = wire.recv(self.sock)
            self.proc.wait(timeout=10)
            return stats
        except (ConnectionError, OSError, subprocess.TimeoutExpired):
            self._kill()
            return None
        finally:
            self.sock.close()

    def _kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
