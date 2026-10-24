"""SCM 시험 기록(conformance/probe_scm_*.py --record)을 중립 렌더 스트림으로 재생해 Godot로 본다.
mj_ros 환경 (ROS2). 렌더러는 출처를 모른다: MuJoCo 시뮬레이터 대신 이 재생기가 같은 토픽을 낸다.

    python conformance/replay_scm_probe.py [--probe press|rolling]       # 기록이 없으면 chrono 환경으로 시험부터 돌린다
    python conformance/replay_scm_probe.py build/scm_probe.npz --speed 0.5 [--loop]

  press    probe_scm_injection.py: 구를 네 자리에 누름 (1 새 흙, 2 높이만 3 cm 낮춤, 3 이어 누른 자리, 4 3의 높이만 베낌)
  rolling  probe_scm_rolling.py: 구를 세 줄에 굴림 (A 이어 굴린 줄 = 기준, B 높이만 베낀 줄, C 가상 다짐 줄)
  출력  /sim/scene_manifest (구 + 회전이 보이는 표시점, 자리·줄 = soil_regions로 색과 이름), /sim/terrain_patch (SCM 흙 높이,
        바뀐 사각형), /tf (world -> ball), /sim/status (info_lines: 제목, 단계, 가라앉은 양, 끄는 힘, 결과)
기록 규약: scm_record.py (분석 그래프 plot_scm_probe.py와 같은 기록). 렌더용 프레임이 있는 기록(시험을 --record로 돌림)만 재생한다.
화면 글자는 영어와 숫자만 (Godot 기본 글꼴에 한글이 없어 한글을 쓰면 화면이 멈춘다). Godot 창을 닫으면 끝난다.
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.stream import STREAM_VERSION, encode_heights  # noqa: E402

sys.path.insert(0, str(ROOT / "conformance"))
import scm_record  # noqa: E402

PROBES = {"press": ("probe_scm_injection.py", "build/scm_press.npz"), "rolling": ("probe_scm_rolling.py", "build/scm_rolling.npz")}


def sphere_obj(path, r, nu=32, nv=16):
    """UV 구 OBJ (바깥이 반시계 감김: godot/main.gd _load_obj 규약)."""
    lines = []
    for i in range(nv + 1):
        th = math.pi * i / nv
        for j in range(nu):
            ph = 2 * math.pi * j / nu
            lines.append(f"v {r * math.sin(th) * math.cos(ph):.5f} {r * math.sin(th) * math.sin(ph):.5f} {r * math.cos(th):.5f}")
    idx = lambda i, j: i * nu + (j % nu) + 1  # noqa: E731
    for i in range(nv):
        for j in range(nu):
            a, b, c, d = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
            lines += [f"f {a} {b} {c}", f"f {a} {c} {d}"]
    path.write_text("\n".join(lines) + "\n")


def manifest(rec, asset_dir):
    W, H = rec["size"]
    nrow, ncol = rec["heights"].shape[1:]
    regions = [dict(rg, soil=rg["name"]) for rg in json.loads(str(rec["regions"]))]
    r = float(rec["radius"])
    return {"stream_version": STREAM_VERSION, "frame": "world",
            "conventions": {"coordinates": "REP-103 (x forward, y left, z up, right-handed)", "units": "m", "quaternion": "x,y,z,w"},
            "asset_root": str(asset_dir),
            "bodies": [{"id": 1, "name": "ball", "parent": "world", "physics_source": "Chrono", "entity": "probe_ball"}],
            "visuals": [{"body": "ball", "mesh": "ball.obj", "pos": [0, 0, 0], "quat": [0, 0, 0, 1], "rgba": [0.78, 0.78, 0.80, 1.0]},
                        {"body": "ball", "mesh": "mark.obj", "pos": [0, 0, r], "quat": [0, 0, 0, 1], "rgba": [0.15, 0.15, 0.18, 1.0]},
                        {"body": "ball", "mesh": "mark.obj", "pos": [r, 0, 0], "quat": [0, 0, 0, 1], "rgba": [0.85, 0.25, 0.20, 1.0]}],
            "soil_regions": regions,
            "terrain": {"half_size": [W / 2, H / 2], "nrow": int(nrow), "ncol": int(ncol), "z_range": [-0.2, 0.2],
                        "layout": "row-major; row i -> y = -half_y + i*dy, col j -> x = -half_x + j*dx"}}


def patch(version, grid, r0, r1, c0, c1):
    return {"version": version, "r0": int(r0), "c0": int(c0), "nrow": int(r1 - r0), "ncol": int(c1 - c0),
            "encoding": "base64-float32-le", "heights": encode_heights(grid[r0:r1, c0:c1])}


def info_lines(rec, k):
    now = f"sinkage {rec['sink'][k] * 100:.2f} cm"
    if "force" in rec:
        now += f"   rolling resistance {rec['force'][k]:.1f} N"
    return [str(rec["title"]), f"[{rec['phase'][k]}]", now, *json.loads(str(rec["result_lines"]))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("record", nargs="?", help="기록 (없으면 --probe의 기본 기록)")
    ap.add_argument("--probe", choices=list(PROBES), default="press", help="기록이 없을 때 돌릴 시험")
    ap.add_argument("--speed", type=float, default=0.5, help="재생 속도 (1 = 시뮬레이션 시간)")
    ap.add_argument("--loop", action="store_true", help="끝나면 처음부터 다시")
    a = ap.parse_args()
    script, default = PROBES[a.probe]
    rec_path = Path(a.record or ROOT / default)
    try:
        rec = scm_record.load(rec_path) if rec_path.exists() else {}
    except ValueError as e:                               # 옛 기록
        print(e)
        rec = {}
    if not rec or not scm_record.has_render(rec):         # 없거나 렌더용 프레임이 없는 기록: 시험부터 다시
        from sim.chrono_link import find_chrono_python
        print(f"기록이 없어 시험부터 돌린다 ({script}, chrono 환경)")
        rec_path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([find_chrono_python({}), str(ROOT / "conformance" / script), "--record", str(rec_path)], cwd=ROOT, check=True)
        rec = scm_record.load(rec_path)
    out = ROOT / "build/scm_probe_replay"
    out.mkdir(parents=True, exist_ok=True)
    sphere_obj(out / "ball.obj", float(rec["radius"]))
    sphere_obj(out / "mark.obj", 0.022, nu=12, nv=6)        # 구 표면의 표시점 (구르는 회전이 보이게)

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from sim.ros2_bridge import launch_godot
    rclpy.init()
    node = rclpy.create_node("scm_probe_replay")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_man = node.create_publisher(String, "/sim/scene_manifest", latched)
    pub_ter = node.create_publisher(String, "/sim/terrain_patch", QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub_st = node.create_publisher(String, "/sim/status", latched)
    pub_tf = node.create_publisher(TFMessage, "/tf", 50)
    godot, bridge = launch_godot(out)
    pub_man.publish(String(data=json.dumps(manifest(rec, out))))
    H = rec["heights"]
    version = 1
    pub_ter.publish(String(data=json.dumps(patch(version, H[0], 0, H.shape[1], 0, H.shape[2]))))
    print("Godot 대기 중...")
    t0 = time.monotonic()
    while pub_tf.get_subscription_count() == 0 and time.monotonic() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(2.0)                                       # Godot가 중계기에 붙어 장면을 만들 시간
    print("재생 (창을 닫으면 끝)")
    try:
        while godot.poll() is None:
            sent = H[0].copy()
            for k in range(len(H)):
                if godot.poll() is not None:
                    break
                diff = H[k] != sent
                if diff.any():
                    rows, cols = np.where(diff.any(1))[0], np.where(diff.any(0))[0]
                    version += 1
                    pub_ter.publish(String(data=json.dumps(patch(version, H[k], rows[0], rows[-1] + 1, cols[0], cols[-1] + 1))))
                    sent = H[k].copy()
                c = rec["center"][k]
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = "world", "ball"
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, c)
                if "quat" in rec:
                    q = rec["quat"][k]
                    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
                else:
                    tf.transform.rotation.w = 1.0
                pub_tf.publish(TFMessage(transforms=[tf]))
                pub_st.publish(String(data=json.dumps({"info_lines": info_lines(rec, k), "base_pos": [float(v) for v in c]})))
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep((float(rec["dt"]) + float(rec["hold"][k])) / a.speed)
            if not a.loop:
                break
            version += 1                                  # 처음 흙으로 되돌리고 다시
            pub_ter.publish(String(data=json.dumps(patch(version, H[0], 0, H.shape[1], 0, H.shape[2]))))
            time.sleep(1.0)
        if godot.poll() is None:
            print("끝 (창을 닫으면 종료)")
            godot.wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in (godot, bridge):
            if p.poll() is None:
                p.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
