"""RViz 어댑터: 중립 렌더 스트림 -> RViz 표현.

    python -m viz.rviz_adapter          (sim.runner --rviz가 자동 실행)

  입력  /sim/scene_manifest, /sim/terrain_patch, /sim/status   (+ RViz가 직접 읽는 /tf, /clock)
  출력  /robot_description   매니페스트의 외형으로 만든 URDF (RobotModel 표시용)
        /viz/terrain         지형 메쉬 (0.5 m 타일 MarkerArray, 바뀐 타일만 다시 발행)
        /viz/status_markers  정보판, 발 접촉, 명령 화살표, 경로

UE5 등 다른 렌더러는 이 파일과 같은 위치에 자기 어댑터를 둔다 (같은 입력, 다른 출력).
물리엔진(MuJoCo)은 import하지 않는다.
"""
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .stream_decode import TerrainGrid, TerrainWindow, check_manifest, quat_to_rpy

TERRAIN_HALF = 6.0      # 표시 범위 (m, 로봇 주변 창의 반폭). 저사양 RViz를 위해 제한
WINDOW_TRIGGER = 2.0    # 로봇이 표시 창 중심에서 이만큼 멀어지면 창을 옮겨 다시 그린다 (m)
MAX_CELLS = 100_000     # 표시 셀 수 상한: 지형이 작으면 원래 해상도(발자국 등), 크면 성기게
TILE = 5                # 타일 한 변의 셀 수


def build_urdf(manifest):
    """매니페스트 -> URDF. 링크 포즈는 RViz가 /tf에서 직접 읽으므로 관절은 고정 관절로 트리만 구성."""
    root = Path(manifest["asset_root"])
    vis_by_body = {}
    for i, v in enumerate(manifest["visuals"]):
        xyz = " ".join(f"{a:.6f}" for a in v["pos"])
        rpy = " ".join(f"{a:.6f}" for a in quat_to_rpy(v["quat"]))
        rgba = " ".join(f"{c:.3f}" for c in v["rgba"])
        vis_by_body.setdefault(v["body"], []).append(
            f'<visual><origin xyz="{xyz}" rpy="{rpy}"/>'
            f'<geometry><mesh filename="file://{root / v["mesh"]}"/></geometry>'
            f'<material name="m{i}"><color rgba="{rgba}"/></material></visual>')
    links = [f'<link name="{b["name"]}">{"".join(vis_by_body.get(b["name"], []))}</link>' for b in manifest["bodies"]]
    joints = [f'<joint name="{b["name"]}_fixed" type="fixed"><parent link="{b["parent"]}"/>'
              f'<child link="{b["name"]}"/></joint>' for b in manifest["bodies"]]
    return f'<?xml version="1.0"?><robot name="scene"><link name="{manifest["frame"]}"/>' \
           + "".join(links) + "".join(joints) + "</robot>"


def status_lines(s):
    """정보판 문구. RViz 글꼴은 공백 폭이 비정상적으로 넓어 공백 없이 구분자로 쓴다."""
    m = s.get("final_mop")
    if m:
        state = "FELL" if m["fell"] else ("INTERRUPTED" if m.get("interrupted") else "FINISHED")
        return [f"=={state}==|t={m['sim_time_s']:.2f}s",
                f"forward={m['forward_x_m']:.2f}m|lateral={m['lateral_drift_m']:+.2f}m",
                f"mean_speed={m['mean_speed_mps']:.2f}m/s|CoT={m['cost_of_transport']:.2f}",
                f"late_steps={m['late_control_steps']}"
                + (f"|min_vehicle_dist={m['min_vehicle_distance_m']:.2f}m" if m.get("min_vehicle_distance_m") else ""),
                "(close_RViz_to_exit)"], (m["fell"] or bool(m.get("interrupted")))
    cot = "--" if s["cot"] is None else f"{s['cot']:.2f}"
    lines = [f"[{s['controller']}|{s['variant']}]|t={s['t']:.2f}s",
             f"cmd:vx={s['command']['vx']:.2f}m/s,yaw={s['command']['yaw_rate']:+.2f}rad/s",
             f"speed={s['speed_avg']:.2f}m/s|dist={s['distance']:.2f}m|CoT={cot}",
             "contact:" + ",".join(n if f["contact"] else "--" for n, f in s["feet"].items()),
             f"late_steps={s['late_steps']}"]
    for v in s.get("vehicles", []):
        lines.append(f"{v['name']}:dist={v['distance']:.1f}m,speed={v['speed']:.1f}m/s")
    ev = s.get("event")
    if ev and s["t"] - ev["t"] < 3.0:
        lines.append(f"event@{ev['t']:.1f}s:{ev['desc']}")
    return [ln.replace(" ", "_") for ln in lines], False


class RvizAdapter(Node):
    def __init__(self):
        super().__init__("rviz_adapter")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        history = QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_desc = self.create_publisher(String, "/robot_description", latched)
        self.pub_terrain = self.create_publisher(MarkerArray, "/viz/terrain", history)
        self.pub_status = self.create_publisher(MarkerArray, "/viz/status_markers", 10)
        self.create_subscription(String, "/sim/scene_manifest", self.on_manifest, latched)
        self.create_subscription(String, "/sim/terrain_patch", self.on_patch, history)
        self.create_subscription(String, "/sim/status", self.on_status, 10)
        self.manifest, self.grid, self.pending, self.shown, self.path = None, None, [], None, []
        self.frame = "world"

    # ---- 장면/로봇 ----
    def on_manifest(self, msg):
        self.manifest = check_manifest(json.loads(msg.data))
        self.frame = self.manifest["frame"]
        self.grid = TerrainGrid(self.manifest["terrain"])
        # 창 반폭: 원래 해상도로 MAX_CELLS 안에 들어가는 크기 (2 cm 격자 -> 약 3 m, 5 cm -> 6 m). 세계가 창보다 크면 창을 옮긴다
        t = self.manifest["terrain"]
        res = 2 * t["half_size"][0] / (t["ncol"] - 1)
        half = min(TERRAIN_HALF, max(1.0, np.floor(res * np.sqrt(MAX_CELLS) / 2)))
        self.window = TerrainWindow(t, half=half, max_cells=0)
        self.pub_desc.publish(String(data=build_urdf(self.manifest)))
        self.get_logger().info(f"manifest: {len(self.manifest['bodies'])} bodies, {len(self.manifest['visuals'])} visuals")
        for p in self.pending:                      # 매니페스트보다 먼저 온 지형 패치
            self.grid.apply(p)
        if self.pending:
            self.publish_terrain()
        self.pending = []

    # ---- 지형 ----
    def on_patch(self, msg):
        patch = json.loads(msg.data)
        if self.grid is None:
            self.pending.append(patch)
            return
        self.grid.apply(patch)
        self.publish_terrain()

    def sampled(self):
        """표시용 격자: 로봇 주변 창에서 k칸마다 한 점 (k는 셀 수가 MAX_CELLS 이하가 되는 가장 작은 값)."""
        w = self.window
        k = 1
        while (w.wncol // k) * (w.wnrow // k) > MAX_CELLS:
            k += 1
        return w.xs()[::k], w.ys()[::k], w.heights(self.grid)[::k, ::k].astype(float)

    def publish_terrain(self):
        xs, ys, H = self.sampled()
        nr, nc = len(ys) - 1, len(xs) - 1
        arr = MarkerArray()
        for r0 in range(0, nr, TILE):
            for c0 in range(0, nc, TILE):
                r1, c1 = min(r0 + TILE, nr), min(c0 + TILE, nc)
                block = H[r0:r1 + 1, c0:c1 + 1]
                if self.shown is not None and np.array_equal(block, self.shown[r0:r1 + 1, c0:c1 + 1]):
                    continue
                X, Y = np.meshgrid(xs[c0:c1 + 1], ys[r0:r1 + 1])
                V = np.stack([X, Y, block], -1)
                a, b, d, e = V[:-1, :-1], V[:-1, 1:], V[1:, :-1], V[1:, 1:]
                tri = np.stack([a, b, e, a, e, d], 2).reshape(-1, 3)
                m = self.marker("terrain", r0 * (nc + 1) + c0, Marker.TRIANGLE_LIST, (0.55, 0.50, 0.42, 1.0), (1, 1, 1))
                m.points = [Point(x=x, y=y, z=z) for x, y, z in tri.tolist()]
                arr.markers.append(m)
        if arr.markers:
            self.pub_terrain.publish(arr)
        self.shown = H

    # ---- 상태 ----
    def marker(self, ns, mid, mtype, rgba, scale):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        return m

    def on_status(self, msg):
        s = json.loads(msg.data)
        bx, by, bz = s["base_pos"]
        if self.grid is not None and self.window.recenter(bx, by, WINDOW_TRIGGER):
            clear = Marker(); clear.header.frame_id = self.frame; clear.ns = "terrain"; clear.action = Marker.DELETEALL
            self.pub_terrain.publish(MarkerArray(markers=[clear]))   # 이전 창 지형 지우고 새 창 전체를 다시 보낸다
            self.shown = None
            self.publish_terrain()
        arr = MarkerArray()
        lines, alert = status_lines(s)
        txt = self.marker("info", 0, Marker.TEXT_VIEW_FACING,
                          (1.0, 0.35, 0.3, 1.0) if alert else (1.0, 1.0, 1.0, 1.0), (0, 0, 0.055))
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = bx, by, bz + 0.55
        txt.text = "\n".join(lines)
        arr.markers.append(txt)
        for i, f in enumerate(s["feet"].values()):
            m = self.marker("contact", i, Marker.SPHERE,
                            (0.2, 0.9, 0.3, 0.9) if f["contact"] else (0.6, 0.6, 0.6, 0.5), (0.045,) * 3)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, f["pos"])
            arr.markers.append(m)
        # 명령 화살표: 길이 = 명령 전진 속도(1 m/s -> 1 m), 방향 = 몸통 방향
        a = self.marker("command", 0, Marker.ARROW, (0.25, 0.6, 1.0, 0.9), (0.02, 0.04, 0.05))
        L, yaw = max(s["command"]["vx"], 0.02), s["yaw"]
        a.points = [Point(x=bx, y=by, z=bz + 0.15), Point(x=bx + L * np.cos(yaw), y=by + L * np.sin(yaw), z=bz + 0.15)]
        arr.markers.append(a)
        if not self.path or np.hypot(bx - self.path[-1][0], by - self.path[-1][1]) > 0.03:
            self.path.append((bx, by, bz))
        if len(self.path) > 1:
            pm = self.marker("path", 0, Marker.LINE_STRIP, (1.0, 0.85, 0.2, 0.9), (0.015, 0, 0))
            pm.points = [Point(x=x, y=y, z=z - 0.2) for x, y, z in self.path]
            arr.markers.append(pm)
        self.pub_status.publish(arr)


def main():
    rclpy.init()
    node = RvizAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
