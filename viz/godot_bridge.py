"""Godot 4 어댑터의 중계기: 중립 렌더 스트림(ROS2) -> TCP (127.0.0.1, 한 줄에 JSON 하나) -> Godot 프로젝트(godot/).

    python -m viz.godot_bridge [--port 4243] [--points] [--map-server]   (sim.runner / sim.rt_link --godot,
                                                                          또는 python -m viz.godot_launch가 Godot와 함께 실행)

GDScript는 ROS2를 직접 받지 못하므로 이 중계기가 받아 Godot가 바로 쓸 형태로 넘긴다. 좌표는 ROS(REP-103) 그대로 보내고,
Godot 쪽에서 장면 맨 위 노드 하나를 X축 -90° 돌려(ROS (x, y, z) -> Godot (x, z, -y)) 변환을 한 곳에서 끝낸다.

  입력  /sim/scene_manifest, /sim/terrain_patch, /sim/terrain_overlay, /tf, /sim/status
        --map-server: 지형은 /sim/terrain_patch 대신 맵 서버에서 (관찰자로 붙어 스냅샷 + 그 뒤 변경, docs/map_server.md 3b).
        실행 중 언제 붙어도 지금 지형 전체를 받는다 (/sim/terrain_patch는 최근 500개만 다시 준다). 화면은 서버 지도(원천)라 물리 반영보다
        조금 앞설 수 있다 (물리는 5 Hz로 반영하고 발 근처 칸은 발이 떠날 때까지 미룬다).
  출력  {"type": "scene", ...}    바디 이름, 외형(OBJ 절대 경로, 바디 좌표 pos/quat, 색), 흙 지역
        {"type": "terrain", ...}  로봇 주변 창의 표시 높이 격자(지형 - 화면 발자국 깊이)와 꼭짓점 색 (RViz와 같은 색 규칙), base64 float32
        {"type": "poses", ...}    바디 월드 포즈 {이름: [x, y, z, qx, qy, qz, qw]} (약 30 Hz)
        {"type": "status", ...}   상태 JSON + 정보판 문구
        {"type": "points", ...}   --points일 때: 점군을 월드 좌표로 (LiDAR /sensors/<이름>/points는 높이 색,
                                  알고리즘 지형 지도 /control/terrain_map은 자기 rgb). 약 10 Hz, base64 float32
        {"type": "markers", ...}  디딜 곳 (/control/footholds, 지형 인지 보행기가 있을 때): 구 [x, y, z, 지름, r, g, b, a]와
                                  선분 [x1, y1, z1, x2, y2, z2, r, g, b, a] (월드 좌표). 약 10 Hz
물리엔진은 import하지 않는다 (viz 규칙).
"""
import argparse
import base64
import json
import os
import re
import socket
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from .stream_decode import TerrainGrid, TerrainWindow, check_manifest

PORT = 4243
TERRAIN_HALF = 6.0          # 표시 창 반폭 (m). RViz 어댑터와 같다
WINDOW_TRIGGER = 2.0
MAX_CELLS = 100_000         # 표시 셀 수 상한 (넘으면 성기게)
TERRAIN_RGB = np.array([0.55, 0.50, 0.42])
SHADE_M = 0.08
POSE_HZ, TERRAIN_HZ, POINTS_HZ = 30.0, 4.0, 10.0
MARKER_TOPIC = "/control/footholds"
CLOUD_TOPIC = re.compile(r"^/sensors/[^/]+/points$|^/control/terrain_map$")   # RViz 설정(config/go2.rviz)과 같은 점군


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a, dtype="<f4").tobytes()).decode("ascii")


def status_lines(s):
    """정보판 문구 (Godot Label). 로봇 시험이 아닌 출처(예: conformance/replay_scm_probe.py)는 info_lines로 문구를 직접 준다."""
    if "info_lines" in s:
        return list(s["info_lines"]), bool(s.get("alert", False))
    m = s.get("final_mop")
    if m:
        state = "FELL" if m["fell"] else ("INTERRUPTED" if m.get("interrupted") else "FINISHED")
        lines = [f"== {state} ==  t {m['sim_time_s']:.2f} s", f"forward {m['forward_x_m']:.2f} m   lateral {m['lateral_drift_m']:+.2f} m",
                 f"mean speed {m['mean_speed_mps']:.2f} m/s   CoT {m['cost_of_transport']:.2f}", "(close window to exit)"]
        return lines, bool(m["fell"])
    cot = "--" if s.get("cot") is None else f"{s['cot']:.2f}"
    lines = [f"[{s['controller']}|{s['variant']}]  t {s['t']:.2f} s",
             f"cmd vx {s['command']['vx']:.2f} m/s  yaw {s['command']['yaw_rate']:+.2f} rad/s",
             f"speed {s['speed_avg']:.2f} m/s  dist {s['distance']:.2f} m  CoT {cot}",
             "contact " + " ".join(n if f["contact"] else "--" for n, f in s["feet"].items()),
             f"late steps {s['late_steps']}"]
    if s.get("soil"):
        lines.append(f"soil {s['soil']}")
    for v in s.get("vehicles", []):
        lines.append(f"{v['name']}  dist {v['distance']:.1f} m  speed {v['speed']:.1f} m/s")
    ev = s.get("event")
    if ev and s["t"] - ev["t"] < 3.0:
        lines.append(f"event @{ev['t']:.1f}s {ev['desc']}")
    faults = s.get("faults") or {}
    lines += [f"FAULT {n}" + ("" if left is None else f" ({left:.1f}s)") for n, left in faults.items()]
    if s.get("frozen"):
        lines.append("== FROZEN ==")
    return lines, bool(faults) or bool(s.get("frozen"))


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def height_colors(z):
    """높이 색 (RViz AxisColor Z처럼 이 점군의 최저 -> 최고: 파랑 -> 초록 -> 빨강)."""
    u = (z - z.min()) / max(float(np.ptp(z)), 1e-6) if len(z) else z
    return np.stack([np.clip(2 * u - 1, 0, 1), 1 - np.abs(2 * u - 1), np.clip(1 - 2 * u, 0, 1)], 1)


def cloud_to_world(msg, poses):
    """PointCloud2 -> (월드 xyz (N, 3), rgb (N, 3)) 또는 좌표계 포즈를 아직 모르면 None.
    좌표계는 /tf의 world -> 프레임 (센서 이름, odom). rgb 필드(RViz 규약: 0x00RRGGBB)가 있으면 그 색, 없으면 높이 색."""
    frame = msg.header.frame_id
    if frame not in ("world", "") and frame not in poses:
        return None
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(n, msg.point_step)
    xyz = np.stack([raw[:, off[k]:off[k] + 4].copy().view("<f4")[:, 0] for k in "xyz"], 1).astype(np.float64)
    if frame not in ("world", ""):
        x, y, z, qx, qy, qz, qw = poses[frame]
        xyz = xyz @ quat_matrix(qx, qy, qz, qw).T + [x, y, z]
    ok = np.isfinite(xyz).all(1)
    if "rgb" in off:
        c = raw[ok, off["rgb"]:off["rgb"] + 4].copy().view("<u4")[:, 0]
        return xyz[ok], np.stack([(c >> 16) & 255, (c >> 8) & 255, c & 255], 1) / 255.0
    return xyz[ok], height_colors(xyz[ok, 2])


def frame_transform(frame, poses):
    """world -> frame (R, p), 모르면 None."""
    if frame in ("world", ""):
        return np.eye(3), np.zeros(3)
    if frame not in poses:
        return None
    x, y, z, qx, qy, qz, qw = poses[frame]
    return quat_matrix(qx, qy, qz, qw), np.array([x, y, z])


def markers_to_world(msg, poses):
    """MarkerArray의 SPHERE와 LINE_LIST -> (구 목록, 선분 목록) 월드 좌표. 좌표계 포즈를 모르는 마커는 건너뛴다."""
    spheres, lines = [], []
    for m in msg.markers:
        T = frame_transform(m.header.frame_id, poses)
        if T is None or m.action != Marker.ADD:
            continue
        R, p = T
        o = m.pose.orientation
        Rm = R @ quat_matrix(o.x, o.y, o.z, o.w)
        pm = R @ [m.pose.position.x, m.pose.position.y, m.pose.position.z] + p
        rgba = [m.color.r, m.color.g, m.color.b, m.color.a]
        if m.type == Marker.SPHERE:
            spheres.append([*map(float, pm), float(m.scale.x), *map(float, rgba)])
        elif m.type == Marker.LINE_LIST:
            pts = [Rm @ [q.x, q.y, q.z] + pm for q in m.points]
            for a, b in zip(pts[0::2], pts[1::2]):
                lines.append([*map(float, a), *map(float, b), *map(float, rgba)])
    return spheres, lines


class GodotBridge(Node):
    def __init__(self, port, points=False, map_server=False):
        super().__init__("godot_bridge")
        self.use_map, self.map = map_server, None       # 맵 서버 관찰자 (manifest를 받은 뒤 붙는다)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        history = QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/sim/scene_manifest", self.on_manifest, latched)
        self.create_subscription(String, "/sim/terrain_patch", self.on_patch, history)
        self.create_subscription(String, "/sim/terrain_overlay", self.on_overlay, history)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.create_subscription(String, "/sim/status", self.on_status, 10)
        self.manifest = self.grid = self.overlay = self.window = None
        self.pending, self.pending_overlay = [], []
        self.poses, self.status = {}, None
        self.terrain_dirty = self.poses_dirty = self.status_dirty = False
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", port))     # 같은 PC의 Godot만
        self.server.listen(1)
        self.server.setblocking(False)
        self.client = None
        self.points = points
        self.clouds, self.cloud_subs = {}, {}           # 토픽 -> 아직 안 보낸 최신 PointCloud2
        self.markers, self.marker_sub = None, None      # 아직 안 보낸 최신 디딜 곳 MarkerArray
        self.get_logger().info(f"Godot 대기: 127.0.0.1:{port}" + ("  (점군 포함)" if points else ""))

    # ---- 스트림 입력 ----
    def on_manifest(self, msg):
        self.manifest = check_manifest(json.loads(msg.data))
        t = self.manifest["terrain"]
        self.grid, self.overlay = TerrainGrid(t), TerrainGrid(t)
        res = 2 * t["half_size"][0] / (t["ncol"] - 1)
        half = min(TERRAIN_HALF, max(1.0, np.floor(res * np.sqrt(MAX_CELLS) / 2)))
        self.window = TerrainWindow(t, half=half, max_cells=0)
        if self.use_map:                                 # 지형은 맵 서버에서 (패치 토픽은 쓰지 않는다)
            self.pending = []
            self.attach_map()
        for p in self.pending:
            self.grid.apply(p)
        for p in self.pending_overlay:
            self.overlay.apply(p)
        self.pending = self.pending_overlay = []
        self.terrain_dirty = True
        self.send_scene()

    def attach_map(self):
        from sim.map_client import RemoteMap                 # 물리엔진이 아니라 지도 사본 (numpy만)
        if self.map is None:
            self.map = RemoteMap(None, name=f"godot_{os.getpid()}")
            j = self.map.joined
            self.get_logger().info(f"맵 서버에 붙음: 판 {self.map.local.version} (스냅샷 {j['s']} s)")
        g = self.grid
        assert (self.map.local.nrow, self.map.local.ncol) == (g.nrow, g.ncol), "맵 서버 격자와 장면 매니페스트 격자가 다르다"
        g.heights = self.map.local.compose(0, g.nrow, 0, g.ncol).astype(np.float32)
        g.version = self.map.local.version

    def poll_map(self):
        if self.map is not None and self.map.poll() > 0:
            g = self.grid
            g.heights = self.map.local.compose(0, g.nrow, 0, g.ncol).astype(np.float32)
            g.version = self.map.local.version
            self.terrain_dirty = True

    def on_patch(self, msg):
        if self.use_map:
            return
        p = json.loads(msg.data)
        if self.grid is None:
            self.pending.append(p)
        else:
            self.grid.apply(p)
            self.terrain_dirty = True

    def on_overlay(self, msg):
        p = json.loads(msg.data)
        if self.overlay is None:
            self.pending_overlay.append(p)
        else:
            self.overlay.apply(p)
            self.terrain_dirty = True

    def on_tf(self, msg):
        for t in msg.transforms:
            tr, r = t.transform.translation, t.transform.rotation
            self.poses[t.child_frame_id] = [tr.x, tr.y, tr.z, r.x, r.y, r.z, r.w]
        self.poses_dirty = True

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_dirty = True
        if self.window is not None and "base_pos" in self.status and \
                self.window.recenter(*self.status["base_pos"][:2], WINDOW_TRIGGER):
            self.terrain_dirty = True

    def find_topics(self):
        """디딜 곳과 (--points일 때) 점군 토픽을 찾아 구독한다 (센서 이름을 미리 몰라도 된다)."""
        for topic, types in self.get_topic_names_and_types():
            if topic == MARKER_TOPIC and self.marker_sub is None and "visualization_msgs/msg/MarkerArray" in types:
                self.marker_sub = self.create_subscription(MarkerArray, topic, lambda m: setattr(self, "markers", m), 5)
                self.get_logger().info(f"디딜 곳 구독: {topic}")
            if not self.points:
                continue
            if topic not in self.cloud_subs and CLOUD_TOPIC.match(topic) and "sensor_msgs/msg/PointCloud2" in types:
                self.cloud_subs[topic] = self.create_subscription(
                    PointCloud2, topic, lambda m, t=topic: self.clouds.__setitem__(t, m), 5)
                self.get_logger().info(f"점군 구독: {topic}")

    def send_clouds(self):
        for topic, msg in list(self.clouds.items()):
            got = cloud_to_world(msg, self.poses)
            if got is None:
                continue                                    # 센서 좌표계 TF가 아직 없다. 다음 점군에서
            del self.clouds[topic]
            xyz, rgb = got
            self.send({"type": "points", "name": topic, "kind": "map" if topic == "/control/terrain_map" else "lidar",
                       "n": len(xyz), "xyz": b64(xyz), "rgb": b64(rgb)})

    def send_markers(self):
        spheres, lines = markers_to_world(self.markers, self.poses)
        if spheres or lines:
            self.markers = None
            self.send({"type": "markers", "name": MARKER_TOPIC, "spheres": spheres, "lines": lines})

    # ---- Godot로 ----
    def send(self, obj):
        if self.client is None:
            return
        try:
            self.client.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        except OSError:
            self.get_logger().info("Godot 연결 끊김")
            self.client.close()
            self.client = None

    def send_scene(self):
        if self.manifest is None:
            return
        root = Path(self.manifest["asset_root"])
        self.send({"type": "scene", "bodies": [b["name"] for b in self.manifest["bodies"]],
                   "visuals": [dict(v, mesh=str(root / v["mesh"])) for v in self.manifest["visuals"]],
                   "soil_regions": self.manifest.get("soil_regions", [])})

    def terrain_msg(self):
        """표시 창 높이 격자와 꼭짓점 색 (RViz 어댑터와 같은 규칙: 흙 지역 색, 높이에 따른 명암)."""
        w, g = self.window, self.grid
        k = 1
        while (w.wncol // k) * (w.wnrow // k) > MAX_CELLS:
            k += 1
        rows = np.arange(w.r0, w.r0 + w.wnrow, k)
        cols = np.arange(w.c0, w.c0 + w.wncol, k)
        H = (g.heights - self.overlay.heights)[np.ix_(rows, cols)].astype(np.float32)
        X, Y = np.meshgrid(-g.half_x + cols * w.res, -g.half_y + rows * w.res)
        base = np.broadcast_to(TERRAIN_RGB, X.shape + (3,)).copy()
        for rg in self.manifest.get("soil_regions", []):
            x0, y0, x1, y1 = rg["region"]
            base[(X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)] = rg["rgba"][:3]
        rgb = np.clip(base * np.clip(1.0 + H[..., None] / SHADE_M, 0.3, 1.5), 0.0, 1.0)
        return {"type": "terrain", "x0": float(X[0, 0]), "y0": float(Y[0, 0]), "dx": float(k * w.res), "dy": float(k * w.res),
                "nrow": int(len(rows)), "ncol": int(len(cols)), "h": b64(H), "rgb": b64(rgb)}

    def accept(self):
        try:
            conn, _ = self.server.accept()
        except BlockingIOError:
            return
        if self.client is not None:
            self.client.close()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.client = conn
        self.get_logger().info("Godot 접속")
        self.send_scene()                                  # 늦게 붙어도 지금 장면 전체
        self.terrain_dirty = self.poses_dirty = self.status_dirty = True

    def run(self):
        next_pose = next_terrain = next_points = next_find = next_markers = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)
            self.poll_map()
            self.accept()
            now = time.monotonic()
            if now >= next_find:
                self.find_topics()
                next_find = now + 1.0
            if self.client is None:
                continue
            if self.poses_dirty and now >= next_pose:
                self.send({"type": "poses", "p": self.poses})
                self.poses_dirty, next_pose = False, now + 1.0 / POSE_HZ
            if self.status_dirty and self.status is not None:
                lines, alert = status_lines(self.status)
                s = self.status
                self.send({"type": "status", "lines": lines, "alert": alert,
                           **{k: s[k] for k in ("base_pos", "yaw", "command", "feet") if k in s}})
                self.status_dirty = False
            if self.terrain_dirty and self.window is not None and now >= next_terrain:
                self.send(self.terrain_msg())
                self.terrain_dirty, next_terrain = False, now + 1.0 / TERRAIN_HZ
            if self.clouds and now >= next_points:
                self.send_clouds()
                next_points = now + 1.0 / POINTS_HZ
            if self.markers is not None and now >= next_markers:
                self.send_markers()
                next_markers = now + 1.0 / POINTS_HZ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--points", action="store_true", help="LiDAR 점군과 알고리즘 지형 지도도 넘긴다")
    ap.add_argument("--map-server", action="store_true", help="지형을 맵 서버에서 (언제 붙어도 지금 지형 전체)")
    a = ap.parse_args()
    rclpy.init()
    node = GodotBridge(a.port, points=a.points, map_server=a.map_server)
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):     # 실행기가 끝나며 멈춤 신호를 보냄
        pass
    finally:
        if node.map is not None:
            node.map.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
