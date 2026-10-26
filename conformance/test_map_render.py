"""렌더러를 맵 클라이언트로 (viz/godot_bridge.py --map-server, docs/map_server.md 3단계 3b) 시험. ROS2 필요 (mj_ros), 약 20초.

    python conformance/test_map_render.py          (ROS_DOMAIN_ID 79)

  늦게 붙는 렌더러: 변경 700개 뒤에 Godot 중계기를 붙인다.
    패치 토픽(/sim/terrain_patch, 늦게 붙으면 최근 500개만 다시 줌)으로 받는 예전 방식은 앞의 200개를 놓쳐 서버와 다르다 (문제가 실제로 있다).
    맵 서버 관찰자로 받으면 스냅샷 + 그 뒤 변경이라 서버와 비트 단위로 같고, 붙은 뒤의 변경도 따라간다.
  Godot 쪽은 띄우지 않는다 (중계기가 Godot에 보낼 지형 격자를 본다).
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ROS_DOMAIN_ID", "79")


def spin(nodes, s):
    import rclpy
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.005)


def main():
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from sim.map_base import write_base
    from sim.map_client import RemoteMap
    from sim.runner import Simulation
    from sim.stream import build_manifest, encode_heights
    from viz.godot_bridge import GodotBridge

    sim = Simulation(yaml.safe_load((ROOT / "scenarios/flat_trot.yaml").read_text()))
    T = sim.terrain
    base = T.target.copy()
    tmp = Path(tempfile.mkdtemp(prefix="map_render_"))
    write_base(base, tmp / "m", (T.z_min, T.z_max), res=T.res, size=[T.half_x, T.half_y], name="m")
    proc = subprocess.Popen([sys.executable, "-m", "sim.map_server_node", "--base", str(tmp / "m")], cwd=ROOT,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        writer = RemoteMap(base, (T.z_min, T.z_max), name="sim")
        pubn = rclpy.create_node("fake_sim_stream")             # 시뮬레이터의 스트림 발행기 대신 (같은 QoS)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub_m = pubn.create_publisher(String, "/sim/scene_manifest", latched)
        pub_p = pubn.create_publisher(String, "/sim/terrain_patch", QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub_m.publish(String(data=json.dumps(build_manifest(sim.model, T))))
        mirror = base.astype(np.float32)
        pub_p.publish(String(data=json.dumps({"version": 0, "r0": 0, "c0": 0, "nrow": T.nrow, "ncol": T.ncol,
                                              "encoding": "base64-float32-le", "heights": encode_heights(mirror)})))
        rng = np.random.default_rng(4)
        for k in range(700):                                     # 변경 700개: 맵 서버로, 그리고 지금처럼 패치 토픽으로
            r0, c0 = int(rng.integers(0, T.nrow - 10)), int(rng.integers(0, T.ncol - 10))
            blk = writer.compose(r0, r0 + 10, c0, c0 + 10) - rng.uniform(0.001, 0.01, (10, 10))
            writer.write("dig", "wheels", r0, c0, blk)
            h = writer.compose(r0, r0 + 10, c0, c0 + 10).astype(np.float32)
            pub_p.publish(String(data=json.dumps({"version": k + 1, "r0": r0, "c0": c0, "nrow": 10, "ncol": 10,
                                                  "encoding": "base64-float32-le", "heights": encode_heights(h)})))
            if k % 50 == 0:
                writer.poll()
        assert writer.audit()["hash_mismatch"] == 0
        truth = writer.compose(0, T.nrow, 0, T.ncol).astype(np.float32)

        old = GodotBridge(0)                                     # 예전 방식: 패치 토픽 (최근 500개만 다시 받음)
        spin([old, pubn], 3.0)
        assert old.grid is not None
        bad_old = int(np.count_nonzero(old.grid.heights != truth))
        assert bad_old > 0, "패치 토픽으로도 맞았다 (500개를 넘겼는데)"
        print(f"test_late_renderer_patch_topic: OK (문제가 있다: 늦게 붙은 예전 방식은 {bad_old}칸이 서버와 다르다)")

        new = GodotBridge(0, map_server=True)                    # 맵 서버 관찰자
        spin([new, pubn], 3.0)
        assert new.map is not None and np.array_equal(new.grid.heights, truth), "맵 서버로 받은 지형이 서버와 다르다"
        joined = new.map.joined
        for _ in range(20):                                      # 붙은 뒤의 변경도 따라간다
            r0, c0 = int(rng.integers(0, T.nrow - 10)), int(rng.integers(0, T.ncol - 10))
            writer.write("dig", "wheels", r0, c0, writer.compose(r0, r0 + 10, c0, c0 + 10) - 0.02)
        t0 = time.monotonic()
        truth = writer.compose(0, T.nrow, 0, T.ncol).astype(np.float32)
        while not np.array_equal(new.grid.heights, truth) and time.monotonic() - t0 < 10:
            spin([new], 0.05)
            new.poll_map()
        assert np.array_equal(new.grid.heights, truth) and new.terrain_dirty
        assert new.map.audit()["hash_mismatch"] == 0
        msg = new.terrain_msg()                                  # Godot에 보낼 메시지도 만들어진다
        assert msg["type"] == "terrain" and msg["nrow"] > 0
        print(f"test_late_renderer_map_server: OK (판 {joined['version']}에서 붙어 서버와 같음, 그 뒤 변경 20개 "
              f"{time.monotonic() - t0:.2f} s 안에 따라감)")
        for n in (old, new, pubn):
            if getattr(n, "map", None) is not None:
                n.map.close()
            n.destroy_node()
        writer.close()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        proc.terminate()
        proc.wait(timeout=10)
        shutil.rmtree(tmp)


if __name__ == "__main__":
    main()
