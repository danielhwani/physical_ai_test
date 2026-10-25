"""맵 서버 프로세스 (docs/map_server.md 2단계). ROS2 노드 안에 sim/map_server.py의 MapServer를 들고, 클라이언트(sim/map_client.py)가
보낸 변경을 같은 규칙으로 적용해 판을 매기고 다른 클라이언트에 다시 내보낸다. 메시지 규약은 sim/map_client.py 첫머리.

    python -m sim.map_server_node [--base build/maps/<이름>]   (sim.runner / sim.rt_link --map-server가 띄운다. 이 PC 안에서만, 같은 ROS_DOMAIN_ID)

  시작: --base가 있으면 바탕 파일(sim/map_base.py)을 읽고 해시를 확인해 바로 지도를 만든다 (2c). 없으면 첫 init의 바탕 격자로 만든다.
  init: 클라이언트는 타일 해시를 보낸다 (지도가 아직 없을 때만 격자도). 타일마다 대조해 다 같으면 붙이고, 다르면 그 타일들을 적어 거부한다.
  지도가 생기기 전의 변경은 버린다 (세어서 info에 싣는다).
  감사: /map/request {"type": "audit"} -> /map/reply에 지금 판, 클라이언트별 마지막 seq, 타일 해시.
  스냅샷: /map/request {"type": "snapshot"} -> /map/reply에 상태 전체와 그때의 n (늦게 붙는 클라이언트, 설계 6.1).
"""
import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .map_base import load_base, tile_shas
from .map_client import apply_message, dec, enc_state, sha, stats_ms, tile_hashes
from .map_server import FORMAT, RULES, TILE, MapServer


class MapServerNode:
    def __init__(self, base=None):
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
        self.inits = {}                                             # 클라이언트 -> 마지막 init의 {nonce, result} (클라이언트가 답을 알아봄)
        self.n = 0                                                  # /map/updates로 내보낸 메시지 번호 (붙는 절차가 스냅샷 뒤를 가른다)
        self.lat = {}                                               # 클라이언트 -> 보낸 때부터 서버가 받기까지 (s) 목록
        self.base_tiles, self.map_name, self.base_path = {}, None, None   # 타일별 바탕 해시, 지도 이름, 바탕 폴더
        if base:                                                    # 바탕 파일에서 바로 시작 (해시가 어긋나면 여기서 멈춘다)
            h, mf = load_base(base)
            assert mf["tile"] == TILE, f"바탕 타일 크기 {mf['tile']} != {TILE}"
            self.server = MapServer(h, tuple(mf["z_range"]))
            self.base_hash, self.map_name, self.base_path = mf["base_sha256"], mf["name"], str(base)
            self.base_tiles = {k: t["sha256"] for k, t in mf["tiles"].items()}
            self.node.get_logger().info(f"지도 시작: 바탕 파일 {base} ({mf['grid']} 격자, 타일 {len(self.base_tiles)}개)")
        else:
            self.node.get_logger().info("맵 서버 대기 (첫 클라이언트의 init)")
        self.publish_info()

    def publish_info(self):
        s = self.server
        info = {"format": FORMAT, "rules": RULES, "dropped_before_init": self.dropped, "seq_gaps": self.gaps,
                "writers": self.seq, "base_sha256": self.base_hash, "joined": self.joined, "refused": self.refused,
                "inits": self.inits, "n": self.n, "map_name": self.map_name, "base_path": self.base_path,
                "base_tiles": self.base_tiles}
        if s is not None:
            info.update(tile=s.tile, grid=[s.nrow, s.ncol], z_range=[s.z_min, s.z_max], version=s.version,
                        owners=[list(o) for o in s.owners])
        self.pub_info.publish(String(data=json.dumps(info)))

    def on_write(self, msg):
        m = json.loads(msg.data)
        sender = m.get("from", "?")
        if "wall" in m and m["type"] != "init":                # init(바탕 격자 전체, 접속 직후)은 지연에서 뺀다: 401 x 401에 약 1.7 MB
            self.lat.setdefault(sender, []).append(time.time() - m["wall"])
        last = self.seq.get(sender, 0)
        if m["type"] != "init" and m["seq"] != last + 1:     # init은 seq 1부터 다시 (같은 이름이 다시 붙을 때)
            self.gaps += 1                                    # 빈칸 (2d에서 다시 받기)
        self.seq[sender] = m["seq"]
        if m["type"] == "init":
            bad = []
            if self.server is None and "base" in m:           # 바탕 파일 없이 시작한 서버의 첫 init: 이 격자로 지도를 만든다
                h = dec(m["base"], tuple(m["shape"]))
                self.server = MapServer(h, tuple(m["z_range"]))
                self.base_hash, self.base_tiles, result = sha(h), tile_shas(h), "first"
                self.node.get_logger().info(f"지도 시작: {m['shape']} 격자, {sender}")
            elif self.server is None:
                result = "no_map"
            else:                                             # 타일 해시 대조 (격자 크기가 다르면 모두 다름)
                theirs = m.get("tiles", {})
                if list(m["shape"]) != [self.server.nrow, self.server.ncol]:
                    bad = ["grid"]
                else:
                    bad = sorted(k for k in self.base_tiles if theirs.get(k) != self.base_tiles[k])
                result = "refused" if bad else "joined"
                if bad:                                       # 다른 바탕: 거부 (지형 상관이 깨진다)
                    self.refused.append(sender)
                    self.node.get_logger().error(f"바탕이 다른 클라이언트 거부: {sender}, 타일 {bad[:10]}")
                else:                                         # 같은 바탕: 붙기만 (클라이언트가 스냅샷을 받음)
                    self.joined.append(sender)
                    self.node.get_logger().info(f"클라이언트 붙음: {sender} (판 {self.server.version})")
            if result != "refused" and sender in self.refused:
                self.refused.remove(sender)
            self.inits[sender] = {"nonce": m.get("nonce"), "result": result, "bad_tiles": bad[:50], "n": self.n}
            self.publish_info()
            return
        if self.server is None or sender in self.refused:     # init 전, 또는 바탕이 달라 거부한 클라이언트의 변경은 버린다
            self.dropped += 1
            return
        apply_message(self.server, m)
        self.n += 1
        m["v"], m["n"] = self.server.version, self.n
        self.pub_up.publish(String(data=json.dumps(m)))
        if m["type"] == "owner":
            self.publish_info()

    def on_request(self, msg):
        m = json.loads(msg.data)
        if m["type"] == "audit":
            r = {"type": "audit", "client": m["client"], "id": m["id"], "version": None if self.server is None else self.server.version,
                 "seq": self.seq, "hashes": {} if self.server is None else tile_hashes(self.server), "gaps": self.gaps,
                 "recv_latency_ms": stats_ms(self.lat.get(m["client"], []))}
            self.pub_reply.publish(String(data=json.dumps(r)))
        elif m["type"] == "snapshot" and self.server is not None:   # 붙는 절차 2: 지금 상태 전체와 그때의 n (서버는 한 줄로 돌아 사이에 끼는 변경이 없다)
            r = {"type": "snapshot", "client": m["client"], "id": m["id"], "n": self.n, "state": enc_state(self.server.state())}
            self.pub_reply.publish(String(data=json.dumps(r)))
            self.node.get_logger().info(f"스냅샷: {m['client']} (판 {self.server.version}, n {self.n})")

    def spin(self):
        last_info = 0.0
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - last_info > 2.0:              # 판이 바뀐 것을 가끔 알림 (늦게 붙는 쪽용)
                self.publish_info()
                last_info = time.monotonic()


def main():
    ap = argparse.ArgumentParser(description="맵 서버 (ROS2)")
    ap.add_argument("--base", help="바탕 파일 폴더 (python -m sim.map_base build). 없으면 첫 클라이언트의 init 격자로 시작")
    a = ap.parse_args()
    rclpy.init()
    n = MapServerNode(base=a.base)
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
