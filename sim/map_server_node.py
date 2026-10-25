"""맵 서버 프로세스 (docs/map_server.md 2단계 2a). ROS2 노드 안에 sim/map_server.py의 MapServer를 들고, 클라이언트(sim/map_client.py)가
보낸 변경을 같은 규칙으로 적용해 판을 매기고 다른 클라이언트에 다시 내보낸다. 메시지 규약은 sim/map_client.py 첫머리.

    python -m sim.map_server_node            (sim.runner / sim.rt_link --map-server가 띄운다. 이 PC 안에서만, 같은 ROS_DOMAIN_ID)

  시작: 첫 init 메시지의 바탕 격자로 지도를 만든다 (2c에서 바탕 파일·해시로 바꿀 예정). init 전의 변경은 버린다 (세어서 info에 싣는다).
  감사: /map/request {"type": "audit"} -> /map/reply에 지금 판, 클라이언트별 마지막 seq, 타일 해시.
"""
import base64
import hashlib
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .map_client import apply_message, dec, tile_hashes
from .map_server import FORMAT, RULES, MapServer


class MapServerNode:
    def __init__(self):
        self.node = rclpy.create_node("map_server")
        rel = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_up = self.node.create_publisher(String, "/map/updates", rel)
        self.pub_info = self.node.create_publisher(String, "/map/info", latched)
        self.pub_reply = self.node.create_publisher(String, "/map/reply", rel)
        self.node.create_subscription(String, "/map/write", self.on_write, rel)
        self.node.create_subscription(String, "/map/request", self.on_request, rel)
        self.server, self.seq, self.dropped, self.gaps = None, {}, 0, 0
        self.base_hash, self.joined, self.refused = None, [], []    # 첫 init의 바탕 해시, 같은 바탕으로 붙은 / 다른 바탕이라 거부한 클라이언트
        self.node.get_logger().info("맵 서버 대기 (init)")

    def publish_info(self):
        s = self.server
        info = {"format": FORMAT, "rules": RULES, "dropped_before_init": self.dropped, "seq_gaps": self.gaps,
                "writers": self.seq, "base_sha256": self.base_hash, "joined": self.joined, "refused": self.refused}
        if s is not None:
            info.update(tile=s.tile, grid=[s.nrow, s.ncol], z_range=[s.z_min, s.z_max], version=s.version,
                        owners=[list(o) for o in s.owners])
        self.pub_info.publish(String(data=json.dumps(info)))

    def on_write(self, msg):
        m = json.loads(msg.data)
        sender = m.get("from", "?")
        last = self.seq.get(sender, 0)
        if m["seq"] != last + 1:
            self.gaps += 1                                    # 빈칸 (2d에서 다시 받기)
        self.seq[sender] = m["seq"]
        if m["type"] == "init":
            h = hashlib.sha256(base64.b64decode(m["base"])).hexdigest()
            if self.server is None:                           # 첫 init: 이 바탕으로 지도를 만든다
                self.server = MapServer(dec(m["base"], tuple(m["shape"])), tuple(m["z_range"]))
                self.base_hash = h
                self.node.get_logger().info(f"지도 시작: {m['shape']} 격자, {sender}")
            elif h == self.base_hash:                         # 같은 바탕: 붙기만 (지도를 다시 만들지 않는다)
                self.joined.append(sender)
                self.node.get_logger().info(f"클라이언트 붙음: {sender}")
            else:                                             # 다른 바탕: 거부 (지형 상관이 깨진다)
                self.refused.append(sender)
                self.node.get_logger().error(f"바탕이 다른 클라이언트 거부: {sender}")
            self.publish_info()
            return
        if self.server is None or sender in self.refused:     # init 전, 또는 바탕이 달라 거부한 클라이언트의 변경은 버린다
            self.dropped += 1
            return
        apply_message(self.server, m)
        m["v"] = self.server.version
        self.pub_up.publish(String(data=json.dumps(m)))
        if m["type"] == "owner":
            self.publish_info()

    def on_request(self, msg):
        m = json.loads(msg.data)
        if m["type"] == "audit":
            r = {"type": "audit", "client": m["client"], "id": m["id"], "version": None if self.server is None else self.server.version,
                 "seq": self.seq, "hashes": {} if self.server is None else tile_hashes(self.server), "gaps": self.gaps}
            self.pub_reply.publish(String(data=json.dumps(r)))

    def spin(self):
        last_info = 0.0
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - last_info > 2.0:              # 판이 바뀐 것을 가끔 알림 (늦게 붙는 쪽용)
                self.publish_info()
                last_info = time.monotonic()


def main():
    rclpy.init()
    n = MapServerNode()
    try:
        n.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
