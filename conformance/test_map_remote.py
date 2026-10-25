"""맵 서버 2단계 (sim/map_server_node.py + sim/map_client.py) 시험: 별도 프로세스 서버와 원격 클라이언트. ROS2 필요 (mj_ros).

    python conformance/test_map_remote.py          (약 20초. ROS_DOMAIN_ID는 79를 쓴다)

  주고받기: 두 클라이언트가 서로 다른 칸을 쓰면 서로의 변경을 받아 두 사본과 서버가 같다 (타일 해시 대조).
  바탕이 다른 클라이언트는 거부된다.
  감사가 어긋남을 잡는다: A의 파임이 서버에 먼저 들어간 뒤 B가 그걸 받기 전에 같은 칸을 add로 편집하면, B의 사본은 적용 순서가 서버와
  달라 어긋난다 (설계 5.6 "늦게 온 이른 파임", 아직 구현 안 함). 감사가 그 타일을 불일치로 보고해야 한다.
  늦게 붙기 (2b): 관찰자가 A가 쓰는 도중에 붙어도(스냅샷 + 그 뒤 변경) 서버와 같다. 바탕이 같은 클라이언트가 나중에 붙으면 바탕이
  아니라 지금 지도에서 시작한다. 지연: 자기 변경 왕복(echo), 다른 쪽 변경 반영(apply).
  바탕 파일 (2c): 서버를 바탕 파일로 띄우면 클라이언트는 격자 대신 타일 해시만 보내고 붙는다 (init이 작다). 타일 하나가 다른
  클라이언트는 그 타일을 적어 거부된다.
"""
import shutil
import tempfile
import threading
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ROS_DOMAIN_ID", "79")


def settle(*clients, version, wait_s=20.0):  # noqa: E302
    """모든 클라이언트 사본의 판이 version(지금까지 쓴 변경 수)이 될 때까지 받는다. 걸린 시간을 돌려준다."""
    t0 = time.monotonic()
    while any(c.local.version < version for c in clients):
        if time.monotonic() - t0 > wait_s:
            raise AssertionError(f"변경이 다 오지 않았다: {[c.local.version for c in clients]} / {version}")
        for c in clients:
            c.poll()
        time.sleep(0.01)
    return time.monotonic() - t0


def main():
    from sim.map_client import RemoteMap
    proc = subprocess.Popen([sys.executable, "-m", "sim.map_server_node"], cwd=ROOT,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        rng = np.random.default_rng(0)
        base = rng.uniform(-0.02, 0.02, (300, 420))
        a = RemoteMap(base, (-0.2, 0.2), name="a")
        b = RemoteMap(base, (-0.2, 0.2), name="b")

        # 1. 주고받기 (A: 위쪽 반에 파임, B: 아래쪽 반을 소유하고 오르내림)
        for _ in range(30):
            r0, c0 = rng.integers(0, 120), rng.integers(0, 380)
            blk = base[r0:r0 + 20, c0:c0 + 30] - rng.uniform(0, 0.03, (20, 30))
            a.write("dig", "a_wheels", int(r0), int(c0), blk)
        b.set_owner(160, 300, 0, 420, "b_scm")
        for _ in range(30):
            rows, cols = rng.integers(160, 300, 40), rng.integers(0, 420, 40)
            b.write_cells("own", "b_scm", rows, cols, rng.uniform(-0.1, 0.05, 40))
        took = settle(a, b, version=60)
        ha, hb = a.compose(0, 300, 0, 420), b.compose(0, 300, 0, 420)
        assert np.array_equal(ha, hb), (f"두 사본이 다르다: 다른 칸 {np.count_nonzero(ha != hb)}, "
                                        f"A {a.local.counts} 판 {a.local.version}, B {b.local.counts} 판 {b.local.version}")
        for c in (a, b):
            au = c.audit()
            assert au["hash_mismatch"] == 0, au
        au = a.audit()
        assert au["server_version"] == a.sent["write"] + b.sent["write"], au      # 판 = 지도 변경 메시지 수 (init, owner 제외)
        print(f"test_exchange: OK (A 보냄 {a.sent}, B 보냄 {b.sent}, 서버 판 {au['server_version']} = 지도 변경 "
              f"{a.sent['write']} + {b.sent['write']}, 다 받기까지 {took:.2f} s)")

        # 2. 바탕이 다른 클라이언트
        try:
            RemoteMap(base + 0.001, (-0.2, 0.2), name="c", wait_s=5.0)
            raise SystemExit("FAIL: 바탕이 다른 클라이언트가 붙었다")
        except RuntimeError as e:
            assert "바탕이 다르다" in str(e), e
        print("test_refuse_other_base: OK")

        # 3. 감사가 어긋남을 잡는다
        r0, c0 = 50, 50
        blk = a.compose(r0, r0 + 4, c0, c0 + 4) - 0.05
        a.write("dig", "a_wheels", r0, c0, blk)
        assert a.audit()["hash_mismatch"] == 0                  # 서버가 A의 파임을 받음 (B는 아직 poll 안 함)
        b.write("add", "b_operator", r0, c0, np.full((4, 4), 0.02))
        settle(a, b, version=62)
        au_a, au_b = a.audit(), b.audit()
        assert au_a["hash_mismatch"] == 0, au_a                 # A: 파임 -> add (서버와 같은 순서)
        assert au_b["hash_mismatch"] >= 1 and "0,0" in au_b["mismatched_tiles"], au_b   # B: add -> 파임 (순서가 다름)
        print(f"test_audit_catches_order: OK (B 불일치 타일 {au_b['mismatched_tiles']})")

        # 4. 관찰자가 A가 쓰는 도중에 붙는다 (A는 다른 스레드에서 계속 쓴다: 스냅샷과 스트림 사이 경합)
        stop = threading.Event()
        def writer():
            k = 0
            while not stop.is_set() and k < 300:
                r0, c0 = rng.integers(0, 120), rng.integers(0, 380)
                a.write("dig", "a_wheels", int(r0), int(c0), base[r0:r0 + 20, c0:c0 + 30] - 0.04 - 0.0001 * k)
                k += 1
                time.sleep(0.002)
        th = threading.Thread(target=writer)
        th.start()
        time.sleep(0.1)
        obs = RemoteMap(None, name="observer")
        th.join()
        assert obs.joined["version"] > 62, obs.joined                # A가 쓰던 도중에 붙었다
        v = a.local.version
        settle(obs, version=v)
        au_o = obs.audit()
        assert au_o["hash_mismatch"] == 0 and au_o["client_version"] == v, au_o
        assert np.array_equal(obs.compose(0, 300, 0, 420), a.compose(0, 300, 0, 420))
        print(f"test_late_join_observer: OK (붙은 판 {obs.joined['version']}, 스냅샷 {obs.joined['s']} s, 끝 판 {v}, "
              f"그 뒤 받은 변경 {au_o['latency_ms']['apply']['n']})")

        # 5. 바탕이 같은 클라이언트가 나중에 붙으면 지금 지도에서 시작한다 (2a에서는 바탕에서 시작했다)
        d = RemoteMap(base, (-0.2, 0.2), name="d")
        assert d.joined and np.array_equal(d.compose(0, 300, 0, 420), a.compose(0, 300, 0, 420)), "늦게 붙은 클라이언트가 바탕에서 시작"
        assert d.audit()["hash_mismatch"] == 0
        print(f"test_late_join_writer: OK (판 {d.local.version}에서 시작)")

        # 6. 지연 (같은 PC): A가 50 Hz로 쓰고, A와 관찰자가 100 Hz로 poll (사본 반영 지연에는 poll 주기가 들어간다)
        for c in (a, obs):
            c.poll()                                              # 앞 시험에서 쌓인 메아리를 먼저 비운다
            c.lat = {"echo": [], "apply": []}
        t0, k = time.monotonic(), 0
        while time.monotonic() - t0 < 2.0:
            if time.monotonic() - t0 >= k / 50.0:
                r0, c0 = rng.integers(0, 120), rng.integers(0, 380)
                a.write("dig", "a_wheels", int(r0), int(c0), base[r0:r0 + 20, c0:c0 + 30] - 0.05)
                k += 1
            a.poll(), obs.poll()
            time.sleep(0.01)
        settle(obs, version=a.local.version)
        la, lo = a.audit()["latency_ms"], obs.audit()["latency_ms"]
        assert lo["apply"]["n"] == k and lo["apply"]["p95"] < 100 and la["echo"]["n"] == k, (la, lo)
        print(f"test_latency: OK (변경 {k}개, A -> 서버 recv {la['recv']}, 관찰자 반영 apply {lo['apply']} ms)")
        for c in (a, b, obs, d):
            c.close()
    finally:
        proc.terminate()
        proc.wait(timeout=10)
    base_files()


def base_files():
    """7. 바탕 파일로 띄운 서버 (2c)."""
    from sim.map_base import write_base
    from sim.map_client import RemoteMap
    tmp = Path(tempfile.mkdtemp(prefix="map_remote_base_"))
    base = np.random.default_rng(5).uniform(-0.02, 0.02, (401, 401))
    write_base(base, tmp / "m", (-0.2, 0.2), name="m")
    proc = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--base", str(tmp / "m")], cwd=ROOT,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        a = RemoteMap(base, (-0.2, 0.2), name="a")
        assert a.info.get("base_path") == str(tmp / "m") and a.joined is None, a.info.get("base_path")   # 판 0: 스냅샷 없이
        assert a.init_bytes < 10000, a.init_bytes                     # 격자(약 1.7 MB) 대신 타일 해시 9개
        a.write("dig", "w", 100, 100, base[100:110, 100:110] - 0.01)
        assert a.audit()["hash_mismatch"] == 0
        print(f"test_base_file_join: OK (init {a.init_bytes} 바이트, 격자를 보냈다면 약 {len(base.tobytes()) * 4 // 3} 바이트)")
        other = base.copy()
        other[250, 390] += 0.001                                      # 타일 1,1 (행 200~399, 열 200~399)의 한 칸
        try:
            RemoteMap(other, (-0.2, 0.2), name="c", wait_s=5.0)
            raise SystemExit("FAIL: 타일 하나가 다른 클라이언트가 붙었다")
        except RuntimeError as e:
            assert "'1,1'" in str(e) and "'0,0'" not in str(e), e
        print("test_base_file_refuse_tile: OK (다른 타일 ['1,1'])")
        a.close()
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        shutil.rmtree(tmp)


if __name__ == "__main__":
    main()
