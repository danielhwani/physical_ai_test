"""맵 서버 프로세스 (docs/map_server.md 2단계). ROS2 노드 안에 sim/map_server.py의 MapServer를 들고, 클라이언트(sim/map_client.py)가
보낸 변경을 같은 규칙으로 적용해 판을 매기고 다른 클라이언트에 다시 내보낸다. 메시지 규약은 sim/map_client.py 첫머리.

    python -m sim.map_server_node [--base build/maps/<이름>] [--save-dir DIR] [--resume DIR]
        (sim.runner / sim.rt_link --map-server가 띄운다. 이 PC 안에서만, 같은 ROS_DOMAIN_ID)

  시작: --base가 있으면 바탕 파일(sim/map_base.py)을 읽고 해시를 확인해 바로 지도를 만든다 (2c). 없으면 첫 init의 바탕 격자로 만든다.
        --resume DIR: 저장한 상태(DIR/state.json)에서 이어 시작한다 (판, n, 클라이언트별 seq까지. 서버를 다시 띄워도 클라이언트가 이어 간다).
  init: 클라이언트는 타일 해시를 보낸다 (지도가 아직 없을 때만 격자도). 타일마다 대조해 다 같으면 붙이고, 다르면 그 타일들을 적어 거부한다.
  지도가 생기기 전의 변경은 버린다 (세어서 info에 싣는다).
  빈칸 (2d, 설계 7절): 클라이언트마다 seq 순서대로만 적용한다. 빈칸 뒤에 온 것은 쌓아 두고 /map/reply {"type": "resend"}로 빠진 seq를
        다시 달라고 한다 (0.5 s마다, 메워질 때까지). 이미 받은 seq는 버린다 (다시 보낸 것). 보낸 /map/updates는 최근 RING개를 들고 있다가
        클라이언트가 /map/request {"type": "replay", "from", "to"}로 달라면 다시 보낸다 (그보다 오래되면 "too_old": 클라이언트가 스냅샷).
  감사: /map/request {"type": "audit"} -> /map/reply에 지금 판, 클라이언트별 마지막 seq, 타일 해시.
  스냅샷: /map/request {"type": "snapshot"} -> /map/reply에 상태 전체와 그때의 n, 클라이언트별 seq (늦게 붙기 6.1, 빈칸 복구).
  저장 (5.8): --save-dir DIR이면 끝날 때(SIGTERM, Ctrl+C), 또는 /map/request {"type": "save"}에 DIR/state.json (상태 전체)과
        DIR/final/ (합친 물리 높이를 바탕 파일 꼴로: 다음 서버의 --base로 쓸 수 있다)을 쓴다.
"""
import argparse
import collections
import json
import signal
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .map_base import load_base, tile_shas, write_base
from .map_client import apply_message, dec, dec_state, enc_state, sha, stats_ms, tile_hashes
from .map_server import FORMAT, RULES, TILE, MapServer

RING = 20000          # 다시 보내 줄 수 있게 들고 있는 /map/updates 수
RESEND_S = 0.5        # 빈칸을 다시 달라고 하는 주기 (s)


class MapServerNode:
    def __init__(self, base=None, save_dir=None, resume=None, ring=RING):
        self.node = rclpy.create_node("map_server")
        rel = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_up = self.node.create_publisher(String, "/map/updates", rel)
        self.pub_info = self.node.create_publisher(String, "/map/info", latched)
        self.pub_reply = self.node.create_publisher(String, "/map/reply", rel)
        self.node.create_subscription(String, "/map/write", self.on_write, rel)
        self.node.create_subscription(String, "/map/request", self.on_request, rel)
        self.server, self.seq, self.dropped = None, {}, 0
        self.base_hash, self.joined, self.refused = None, [], []    # 첫 init의 바탕 해시, 같은 바탕으로 붙은 / 다른 바탕이라 거부한 클라이언트
        self.inits = {}                                             # 클라이언트 -> 마지막 init의 {nonce, result} (클라이언트가 답을 알아봄)
        self.n = 0                                                  # /map/updates로 내보낸 메시지 번호 (붙는 절차, 빈칸 찾기)
        self.ring = collections.deque(maxlen=ring)                  # 최근에 내보낸 메시지 (n, JSON)
        self.lat = {}                                               # 클라이언트 -> 보낸 때부터 서버가 받기까지 (s) 목록
        self.held = {}                                              # 클라이언트 -> {seq: 메시지} (빈칸 뒤에 온 것)
        self.gap_since = {}                                         # 클라이언트 -> {빠진 seq: 처음 본 시각} (monotonic)
        self.gap_log = {}                                           # 클라이언트 -> [빠진 seq를 알아챈 때부터 받기까지 s]
        self.gap_delay = {}                                         # 클라이언트 -> [빈칸 때문에 늦게 적용된 변경의 보낸 때부터 적용까지 s]
        self.dups = 0                                               # 이미 받은 seq (다시 보낸 것)
        self.base_tiles, self.map_name, self.base_path = {}, None, None   # 타일별 바탕 해시, 지도 이름, 바탕 폴더
        self.save_dir = Path(save_dir) if save_dir else None
        log = self.node.get_logger()
        if resume:                                                  # 저장한 상태에서 이어 시작
            j = json.loads((Path(resume) / "state.json").read_text())
            self.server = MapServer.from_state(dec_state(j["state"]))
            self.n, self.seq = j["n"], j["seq"]
            self.base_hash, self.base_tiles, self.map_name, self.base_path = j["base_sha256"], j["base_tiles"], j["map_name"], j["base_path"]
            log.info(f"이어 시작: {resume} (판 {self.server.version}, n {self.n}, 클라이언트 {list(self.seq)})")
        elif base:                                                  # 바탕 파일에서 바로 시작 (해시가 어긋나면 여기서 멈춘다)
            h, mf = load_base(base)
            assert mf["tile"] == TILE, f"바탕 타일 크기 {mf['tile']} != {TILE}"
            self.server = MapServer(h, tuple(mf["z_range"]))
            self.base_hash, self.map_name, self.base_path = mf["base_sha256"], mf["name"], str(base)
            self.base_tiles = {k: t["sha256"] for k, t in mf["tiles"].items()}
            log.info(f"지도 시작: 바탕 파일 {base} ({mf['grid']} 격자, 타일 {len(self.base_tiles)}개)")
        else:
            log.info("맵 서버 대기 (첫 클라이언트의 init)")
        self.publish_info()

    def gap_stats(self):
        return {"held": {c: sorted(h) for c, h in self.held.items() if h}, "dups": self.dups,
                "recovered": {c: stats_ms(v) for c, v in self.gap_log.items()}}

    def publish_info(self):
        s = self.server
        info = {"format": FORMAT, "rules": RULES, "dropped_before_init": self.dropped, "gaps": self.gap_stats(),
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
        if m["type"] == "init":
            self.seq[sender] = m["seq"]                           # init은 seq 1부터 다시 (같은 이름이 다시 붙을 때)
            self.held.pop(sender, None)
            self.gap_since.pop(sender, None)
            self.on_init(m, sender)
            return
        if "wall" in m:                                           # init(바탕 격자 전체, 접속 직후)은 지연에서 뺀다: 401 x 401에 약 1.7 MB
            self.lat.setdefault(sender, []).append(time.time() - m["wall"])
        last = self.seq.get(sender, 0)
        if m["seq"] <= last:                                      # 이미 받음 (클라이언트가 다시 보낸 것)
            self.dups += 1
            return
        if m["seq"] > last + 1:                                   # 빈칸: 쌓아 두고 빠진 것을 달라고 한다
            held = self.held.setdefault(sender, {})
            held[m["seq"]] = m
            missing = self.gap_since.setdefault(sender, {})
            now = time.monotonic()
            new = [s for s in range(last + 1, m["seq"]) if s not in held and s not in missing]
            for s in new:
                missing[s] = now
            if new:
                self.ask_resend(sender)
            return
        self.accept(sender, m, late=bool(self.gap_since.get(sender)) and m["seq"] in self.gap_since[sender])
        held = self.held.get(sender, {})
        while self.seq[sender] + 1 in held:                       # 빈칸이 메워졌다: 쌓아 둔 것을 순서대로 (기다린 만큼 늦었다)
            self.accept(sender, held.pop(self.seq[sender] + 1), late=True)
        if not self.gap_since.get(sender):
            self.gap_since.pop(sender, None)

    def ask_resend(self, sender):
        held = self.held.get(sender, {})
        if not held:
            return
        r = {"type": "resend", "client": sender, "from": self.seq.get(sender, 0) + 1, "to": min(held) - 1}   # 첫 빈칸 (메워지면 다음 것)
        self.pub_reply.publish(String(data=json.dumps(r)))

    def accept(self, sender, m, late=False):
        """seq 순서대로 온 변경 하나: 적용하고 번호를 매겨 내보낸다. late: 빈칸 때문에 늦게 적용 (빠졌던 것, 그 뒤에 기다린 것)."""
        if late and "wall" in m:
            self.gap_delay.setdefault(sender, []).append(time.time() - m["wall"])
        self.seq[sender] = m["seq"]
        t = self.gap_since.get(sender, {}).pop(m["seq"], None)
        if t is not None:                                         # 빠졌던 seq가 왔다 (빈칸마다 잰다)
            self.gap_log.setdefault(sender, []).append(time.monotonic() - t)
        if self.server is None or sender in self.refused:         # 지도 전, 또는 바탕이 달라 거부한 클라이언트의 변경은 버린다
            self.dropped += 1
            return
        apply_message(self.server, m)
        self.n += 1
        m["v"], m["n"] = self.server.version, self.n
        data = json.dumps(m)
        self.ring.append((self.n, data))
        self.pub_up.publish(String(data=data))
        if m["type"] == "owner":
            self.publish_info()

    def on_init(self, m, sender):
        bad = []
        if self.server is None and "base" in m:                   # 바탕 파일 없이 시작한 서버의 첫 init: 이 격자로 지도를 만든다
            h = dec(m["base"], tuple(m["shape"]))
            self.server = MapServer(h, tuple(m["z_range"]))
            self.base_hash, self.base_tiles, result = sha(h), tile_shas(h), "first"
            self.node.get_logger().info(f"지도 시작: {m['shape']} 격자, {sender}")
        elif self.server is None:
            result = "no_map"
        else:                                                     # 타일 해시 대조 (격자 크기가 다르면 모두 다름)
            theirs = m.get("tiles", {})
            if list(m["shape"]) != [self.server.nrow, self.server.ncol]:
                bad = ["grid"]
            else:
                bad = sorted(k for k in self.base_tiles if theirs.get(k) != self.base_tiles[k])
            result = "refused" if bad else "joined"
            if bad:                                               # 다른 바탕: 거부 (지형 상관이 깨진다)
                self.refused.append(sender)
                self.node.get_logger().error(f"바탕이 다른 클라이언트 거부: {sender}, 타일 {bad[:10]}")
            else:                                                 # 같은 바탕: 붙기만 (변경이 있으면 클라이언트가 스냅샷을 받음)
                self.joined.append(sender)
                self.node.get_logger().info(f"클라이언트 붙음: {sender} (판 {self.server.version})")
        if result != "refused" and sender in self.refused:
            self.refused.remove(sender)
        self.inits[sender] = {"nonce": m.get("nonce"), "result": result, "bad_tiles": bad[:50], "n": self.n}
        self.publish_info()

    def on_request(self, msg):
        m = json.loads(msg.data)
        reply = lambda r: self.pub_reply.publish(String(data=json.dumps(dict(r, client=m["client"], id=m.get("id")))))  # noqa: E731
        if m["type"] == "audit":
            reply({"type": "audit", "version": None if self.server is None else self.server.version,
                   "seq": self.seq, "hashes": {} if self.server is None else tile_hashes(self.server), "gaps": self.gap_stats(),
                   "recv_latency_ms": stats_ms(self.lat.get(m["client"], [])),
                   "gap_recovery_ms": stats_ms(self.gap_log.get(m["client"], [])),
                   "gap_delay_ms": stats_ms(self.gap_delay.get(m["client"], []))})
        elif m["type"] == "snapshot" and self.server is not None:   # 지금 상태 전체와 그때의 n (서버는 한 줄로 돌아 사이에 끼는 변경이 없다)
            reply({"type": "snapshot", "n": self.n, "seq": self.seq, "state": enc_state(self.server.state())})
            self.node.get_logger().info(f"스냅샷: {m['client']} (판 {self.server.version}, n {self.n})")
        elif m["type"] == "replay":                                 # 빈칸 다시 보내기 (클라이언트가 못 받은 n 구간)
            lo, hi = m["from"], m["to"]
            if not self.ring or lo < self.ring[0][0]:
                reply({"type": "replay", "too_old": True, "from": lo, "to": hi})
            else:
                k0 = self.ring[0][0]
                msgs = [self.ring[k - k0][1] for k in range(lo, min(hi, self.n) + 1)]
                reply({"type": "replay", "from": lo, "to": hi, "msgs": msgs})
        elif m["type"] == "save":
            reply({"type": "saved", "dir": self.save(m.get("dir")), "version": None if self.server is None else self.server.version})

    def save(self, out=None):
        """상태 전체(state.json)와 합친 물리 높이(final/, 바탕 파일 꼴)를 쓴다. 반환: 폴더 (없으면 None)."""
        out = Path(out) if out else self.save_dir
        if out is None or self.server is None:
            return None
        out.mkdir(parents=True, exist_ok=True)
        s = self.server
        j = {"format": FORMAT, "saved_wall": time.time(), "n": self.n, "seq": self.seq, "base_sha256": self.base_hash,
             "base_tiles": self.base_tiles, "map_name": self.map_name, "base_path": self.base_path, "state": enc_state(s.state())}
        tmp = out / "state.json.tmp"
        tmp.write_text(json.dumps(j))
        tmp.replace(out / "state.json")                           # 쓰는 도중에 끝나도 예전 것이 남게
        write_base(s.compose(0, s.nrow, 0, s.ncol), out / "final", (s.z_min, s.z_max), name=f"{self.map_name or 'map'}_final",
                   source={"from": self.base_path, "version": s.version, "counts": s.counts})
        self.node.get_logger().info(f"저장: {out} (판 {s.version}, n {self.n})")
        return str(out)

    def spin(self):
        last_info = last_resend = 0.0
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_resend > RESEND_S:                       # 아직 안 메워진 빈칸을 다시 달라고
                for c in list(self.gap_since):
                    self.ask_resend(c)
                last_resend = now
            if now - last_info > 2.0:                              # 판이 바뀐 것을 가끔 알림 (늦게 붙는 쪽용)
                self.publish_info()
                last_info = now


def main():
    ap = argparse.ArgumentParser(description="맵 서버 (ROS2)")
    ap.add_argument("--base", help="바탕 파일 폴더 (python -m sim.map_base build). 없으면 첫 클라이언트의 init 격자로 시작")
    ap.add_argument("--save-dir", help="끝날 때 상태(state.json)와 끝난 지형(final/, 바탕 파일 꼴)을 쓸 폴더")
    ap.add_argument("--resume", help="저장한 상태(state.json이 있는 폴더)에서 이어 시작")
    ap.add_argument("--ring", type=int, default=RING, help="다시 보내 줄 수 있게 들고 있는 변경 수 (시험용으로 줄임)")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))   # terminate()에도 저장하고 끝내게
    rclpy.init()
    n = MapServerNode(base=a.base, save_dir=a.save_dir, resume=a.resume, ring=a.ring)
    try:
        n.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.save()
        n.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
