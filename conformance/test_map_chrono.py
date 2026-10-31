"""Chrono를 맵 서버의 별도 클라이언트(소유자)로 (docs/map_server.md 3단계 3c) 시험. ROS2 필요 (mj_ros) + chrono 환경, 약 1분.

    python conformance/test_map_chrono.py          (ROS_DOMAIN_ID 79)

  TCP 입구: ROS2 클라이언트와 TCP 클라이언트가 같은 서버에서 서로의 변경을 받아 같은 사본이 된다.
  ROS2 없는 클라이언트: chrono 환경(Python 3.12, rclpy 없음)의 프로세스가 TCP로 붙어 지금 지도를 받고 쓴다.
  Chrono 소유자: vehicle_crossing (6 s)을 --map-server로 돌리면 Chrono 프로세스가 SCM 영역을 직접 소유하고 쓰며(시뮬레이터는 init만 보냄),
  맵 서버 없이 돌린 것과 시계열이 비트 단위로 같다. 시뮬레이터가 이 쓰기를 받기까지 기다린 시간(map_partner_wait_max_ms)을 낸다.
  SCM에 넣기: 받은 편집(재현 시나리오의 map_write)을 다음 교환 구간부터 SCM에도 넣는다. 바퀴 자리에 올린 5 cm 판을 HMMWV가 밟아 차체가 오르고 자국이
  판 위에서 파인다. SCM에 안 넣으면 차는 판을 모르고 Chrono 쓰기가 판을 덮는다.
  실행 중 편집: --map-server 실행 중 sim.map_edit로 SCM 영역에 편집을 넣으면 불일치 0이고, 재현 시나리오를 맵 서버 없이 돌리면 비트 단위로 같다.
  이 시험이 만든 실행 폴더만 지운다.
"""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ROS_DOMAIN_ID", "79")
WALL = ("wall", "lag", "late", "wake", "compute")
CHRONO_ENV_CLIENT = '''
import sys, time
sys.path.insert(0, sys.argv[2])
import numpy as np
from sim.map_client import RemoteMap
c = RemoteMap(None, name="chrono_env_test", tcp=f"127.0.0.1:{sys.argv[1]}")
c.write_cells("set", "chrono_env", np.arange(200, 210), np.full(10, 100), np.full(10, -0.03))
au = c.audit()
print(sys.version.split()[0], "rclpy" in sys.modules, c.joined["version"], au["hash_mismatch"])
c.close()
'''


def runs():
    return set(p.name for p in (ROOT / "runs").iterdir())


def tcp_clients():
    from sim.chrono_link import find_chrono_python
    from sim.map_client import RemoteMap
    proc = subprocess.Popen([sys.executable, "-m", "sim.map_server_node"], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        rng = np.random.default_rng(0)
        base = rng.uniform(-0.02, 0.02, (300, 420))
        a = RemoteMap(base, (-0.2, 0.2), name="ros_a")
        port = a.info["tcp_port"]
        b = RemoteMap(base, (-0.2, 0.2), name="tcp_b", tcp=f"127.0.0.1:{port}")
        for k in range(50):
            a.write("dig", "a", int(rng.integers(0, 100)), int(rng.integers(0, 380)), base[0:10, 0:30] - 0.01 * (k % 5))
            b.write_cells("set", "b", rng.integers(150, 300, 30), rng.integers(0, 420, 30), rng.uniform(-0.05, 0.05, 30))
        aa, ab = a.audit(), b.audit()
        while aa["server_version"] != ab["server_version"]:
            aa, ab = a.audit(), b.audit()
        assert aa["hash_mismatch"] == ab["hash_mismatch"] == 0 and np.array_equal(a.compose(0, 300, 0, 420), b.compose(0, 300, 0, 420))
        print(f"test_tcp_and_ros_clients: OK (판 {aa['server_version']}, TCP 쪽이 서버에 닿기까지 p95 {ab['latency_ms']['recv']['p95']} ms)")

        r = subprocess.run([find_chrono_python({}), "-c", CHRONO_ENV_CLIENT, str(port), str(ROOT)], capture_output=True, text=True,
                           timeout=120)
        ver, has_rclpy, joined, bad = r.stdout.split()
        assert ver.startswith("3.12") and has_rclpy == "False" and int(joined) == 100 and bad == "0", r.stdout + r.stderr[-1000:]
        t0 = time.monotonic()
        while a.local.version < 101 and time.monotonic() - t0 < 5:
            a.poll()
            time.sleep(0.01)
        assert a.local.counts.get("chrono_env/set") == 1 and a.audit()["hash_mismatch"] == 0
        print(f"test_client_without_ros2: OK (Python {ver}, rclpy 없음, 판 {joined}에서 붙어 씀, ROS2 쪽이 받음)")
        a.close(), b.close()
    finally:
        proc.terminate()
        proc.wait(timeout=10)


def chrono_owner():
    before = runs()
    py = [sys.executable, "-m", "sim.runner", "scenarios/vehicle_crossing.yaml", "--set", "duration=6"]
    try:
        out = {}
        for mode in ("plain", "map"):
            b = runs()
            r = subprocess.run(py + (["--map-server"] if mode == "map" else []), cwd=ROOT, capture_output=True, text=True, timeout=600)
            assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
            out[mode] = ROOT / "runs" / sorted(runs() - b)[-1]
        ta, tb = (pq.read_table(out[k] / "timeseries.parquet") for k in ("plain", "map"))
        cols = [c for c in ta.column_names if not any(w in c for w in WALL)]
        diff = [c for c in cols if not np.array_equal(np.asarray(ta[c]), np.asarray(tb[c]), equal_nan=True)]
        assert ta.num_rows == tb.num_rows and not diff, diff
        u = json.loads((out["map"] / "map_updates.json").read_text())
        mop = json.loads((out["map"] / "summary.json").read_text())["mop"]
        assert u["counts"].get("chrono/own", 0) > 0 and u["audit"]["sent"]["total"] == 1, u      # 시뮬레이터는 init만, 쓰기는 Chrono가
        assert mop["map_hash_mismatch"] == 0 and mop["map_received_changes"] == 0 and "map_partner_wait_max_ms" in mop, mop
        print(f"test_chrono_owner_client: OK (맵 서버 없이 = 있게 비트 동일, Chrono가 쓴 것 {u['counts']}, 시뮬레이터 보냄 init 1개, "
              f"기다린 시간 최대 {mop['map_partner_wait_max_ms']} ms)")
    finally:
        for r in runs() - before:                                # 이 시험이 만든 실행 폴더만
            shutil.rmtree(ROOT / "runs" / r)


def plate_run(inject):
    """t = 2 s에 왼쪽 바퀴 자리(x 1.9 ~ 2.4, y 2.5 ~ 4)에 5 cm 판 (맵 서버 없이: 재현 시나리오와 같은 map_write 이벤트). 차는 y = 2.5에 t = 4.5 s쯤 닿는다."""
    import yaml
    from sim.map_client import enc
    from sim.runner import Simulation
    from sim.terrain_service import TerrainMapService
    scn = yaml.safe_load((ROOT / "scenarios/vehicle_crossing.yaml").read_text())
    scn["duration"] = 6.5
    t = TerrainMapService.from_scenario(scn)
    rows, cols = np.nonzero(t._in(1.9, 2.5, 2.4, 4.0))
    r0, c0 = int(rows.min()), int(cols.min())
    block = np.full((rows.max() - r0 + 1, cols.max() - c0 + 1), 0.05)
    scn["events"].append({"t": 2.0, "action": "map_write", "msg": {"type": "write", "op": "add", "writer": "operator", "t": 2.0, "r0": r0, "c0": c0,
                                                                   "nrow": block.shape[0], "ncol": block.shape[1], "data": enc(block)}})
    scn["events"].sort(key=lambda e: e["t"])
    sim = Simulation(scn)
    if not inject:
        sim.cosim.inject = lambda m: None
    zs = []
    while sim.data.time < scn["duration"]:
        sim.step_control(verbose=False)
        if sim.cosim.vehicle:
            zs.append((sim.cosim.vehicle["pos"][1], sim.cosim.vehicle["pos"][2]))
    zs = np.array(zs)
    T = sim.terrain
    h = T.target.copy()
    cell = lambda x, y: h[int(np.argmin(np.abs(T.ys - y))), int(np.argmin(np.abs(T.xs - x)))]  # noqa: E731
    out = (zs[(zs[:, 0] > 2.0) & (zs[:, 0] < 4.5), 1].max() - zs[zs[:, 0] < 0, 1].mean(), cell(2.15, 3.25), cell(1.95, 3.25))
    sim.close()
    return out


def inject_into_scm():
    rise, rut, side = plate_run(True)
    rise0, rut0, side0 = plate_run(False)
    # 넣은 노드는 Chrono 응답(float32)으로 돌아오므로 1e-6 안에서 비교한다
    assert rise > 0.005 and rut > -0.02 and abs(side - 0.05) < 1e-6, (rise, rut, side)          # 차가 판을 밟아 오르고, 자국은 판 위에서 파인다
    assert rise0 < 0.001 and rut0 < -0.03 and abs(side0 - 0.05) < 1e-6, (rise0, rut0, side0)    # SCM이 모르면 차는 평지를 달리고 쓰기가 판을 덮는다
    print(f"test_inject_into_scm: OK (차체 오름 {rise * 100:.1f} cm, 자국 {rut * 100:.1f} cm. 안 넣으면 오름 {rise0 * 100:.1f} cm, 자국 {rut0 * 100:.1f} cm)")


def live_edit_into_scm():
    from sim.map_edit import main as edit
    before = runs()
    try:
        live = subprocess.Popen([sys.executable, "-m", "sim.runner", "scenarios/vehicle_crossing.yaml", "--map-server", "--realtime"], cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(7.0)
        assert edit(["set", "--region", "1.9", "2.5", "2.4", "4.0", "--height", "0.05"]) == 0
        assert edit(["bump", "--center", "3.85", "4", "--height", "0.05", "--radius", "0.5"]) == 0
        out, _ = live.communicate(timeout=300)
        assert "map_hash_mismatch    0" in out and "map_lost_changes     0" in out, out[-1500:]
        d = ROOT / "runs" / sorted(runs() - before)[-1]
        b2 = runs()
        subprocess.run([sys.executable, "-m", "sim.runner", str(d / "scenario_replay.yaml")], cwd=ROOT, check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.STDOUT, timeout=600)
        r = ROOT / "runs" / sorted(runs() - b2)[-1]
        ta, tb = (pq.read_table(x / "timeseries.parquet") for x in (d, r))
        cols = [c for c in ta.column_names if not any(w in c for w in WALL)]
        diff = [c for c in cols if not np.array_equal(np.asarray(ta[c]), np.asarray(tb[c]), equal_nan=True)]
        assert ta.num_rows == tb.num_rows and not diff, diff
        m = json.loads((d / "summary.json").read_text())["mop"]
        print(f"test_live_edit_into_scm: OK (받은 변경 {m['map_received_changes']}개, 불일치 0, 실행 = 재현 비트 동일, 끝 시각 {m['sim_time_s']} s, "
              f"넘어짐 {m['fell']}, forward_x {m['forward_x_m']})")
    finally:
        for x in runs() - before:
            shutil.rmtree(ROOT / "runs" / x)


if __name__ == "__main__":
    tcp_clients()
    chrono_owner()
    inject_into_scm()
    live_edit_into_scm()
