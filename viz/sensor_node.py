"""별도 프로세스 센서 노드: ROS2 중립 스트림만 구독해 센서 출력을 발행한다 (실제 배치 형태, 문서 §8).

    python -m viz.sensor_node front_lidar        (sim.runner --sensor-node --sensor front_lidar 가 자동 실행)

  입력  /sim/scene_manifest, /sim/terrain_patch, /tf, /clock
  출력  /sensors/<이름>/points (PointCloud2), /tf (world -> <이름>)

UE5 같은 렌더러가 센서를 맡는 구조와 같다. 시뮬레이터와 시각을 맞추지 않고 받은 최신 상태로 그리므로
결정성은 없다 (실시간 실행용). 결정적인 결과가 필요하면 같은 프로세스 방식(--sensor만)을 쓴다.
"""
import json
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .ros_msgs import scan_to_msgs
from .sensor_renderer import SensorRenderer

ROOT = Path(__file__).resolve().parent.parent


class SensorNode:
    def __init__(self, names):
        defs = yaml.safe_load((ROOT / "specs/go2_sensors.yaml").read_text())["lidars"]
        self.specs = {n: defs[n] for n in names}
        rclpy.init()
        self.node = rclpy.create_node("sensor_node")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        history = QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/sim/scene_manifest", self.on_manifest, latched)
        self.node.create_subscription(String, "/sim/terrain_patch", self.on_patch, history)
        self.node.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.node.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.pub_tf = self.node.create_publisher(TFMessage, "/tf", 10)
        self.pubs = {n: self.node.create_publisher(PointCloud2, f"/sensors/{n}/points", 5) for n in names}
        self.renderer, self.pending, self.bodies = None, [], set()

    def on_manifest(self, msg):
        manifest = json.loads(msg.data)
        self.renderer = SensorRenderer(manifest, self.specs,
                                       terrain_half=min(8.0, max(s["range_max"] for s in self.specs.values())))
        self.bodies = {b["name"] for b in manifest["bodies"]}
        for p in self.pending:
            self.renderer.apply_patch(p)
        self.pending = []
        self.node.get_logger().info(f"sensors {list(self.specs)} ready ({len(self.bodies)} bodies)")

    def on_patch(self, msg):
        p = json.loads(msg.data)
        (self.renderer.apply_patch(p) if self.renderer else self.pending.append(p))

    def on_tf(self, msg):
        if self.renderer is None:
            return
        poses = {}
        for t in msg.transforms:
            if t.child_frame_id in self.bodies:               # 장면 바디만 (센서 좌표계 등은 무시)
                tr, r = t.transform.translation, t.transform.rotation
                poses[t.child_frame_id] = ([tr.x, tr.y, tr.z], [r.x, r.y, r.z, r.w])
        self.renderer.set_poses(poses)

    def on_clock(self, msg):
        if self.renderer is None or not all(s["parent"] in self.renderer.poses for s in self.specs.values()):
            return
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        for name, scan in self.renderer.render(t):
            pc, tf = scan_to_msgs(name, scan)
            self.pub_tf.publish(TFMessage(transforms=[tf]))
            self.pubs[name].publish(pc)


def main():
    node = SensorNode(sys.argv[1:] or ["front_lidar"])
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    finally:
        node.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
