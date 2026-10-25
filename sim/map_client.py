"""맵 서버 원격 클라이언트 (docs/map_server.md 2단계 2a). sim/map_server.py의 MapServer와 같은 함수를 가지고, 안에 사본(MapServer)을 둔다.
자기 변경은 사본에 바로 반영한 뒤 서버로 보낸다 (물리 결과가 서버와 무관: 원칙 3 "실시간 루프는 서버를 기다리지 않는다").
다른 쪽 변경은 poll()에서 받아 사본에 반영한다.

  전송 (ROS2 std_msgs/String, JSON. 높이는 float64 little-endian base64: 사본이 비트 단위로 같아야 해서)
    /map/write    클라이언트 -> 서버   {"type": "init" | "write" | "cells" | "owner", "writer", "seq", ...}
    /map/updates  서버 -> 클라이언트   위 메시지에 서버가 매긴 판 "v"를 붙여 그대로 (init 제외)
    /map/info     서버 (늦게 붙어도 받음) {"format", "rules", "tile", "grid", "z_range", "version", "owners", "writers"}
    /map/request  클라이언트 -> 서버   {"type": "audit", "client", "id"}
    /map/reply    서버 -> 클라이언트   {"type": "audit", "client", "id", "version", "seq": {쓴 쪽: 마지막 seq}, "hashes": {타일: sha256}}
  보낸 클라이언트(from)마다 seq를 1부터 붙인다 (모든 메시지: init, owner, write, cells). 서버는 받은 순서로 판을 매긴다.
  판 = 적용한 지도 변경 메시지 수 (write, cells). init(접속·바탕)과 owner(소유 영역 선언)는 판을 올리지 않는다. 값이 실제로 안 바뀐
  변경(예: min에서 더 낮지 않은 값만)도 판을 하나 올린다. 그래서 seq = 판 + init + owner (이 클라이언트만 쓴 경우).
"""
import base64
import hashlib
import json
import time

import numpy as np

from .map_server import MapServer

TOPICS = ("/map/write", "/map/updates", "/map/info", "/map/request", "/map/reply")


def enc(a):
    return base64.b64encode(np.ascontiguousarray(a, dtype="<f8").tobytes()).decode("ascii")


def dec(s, shape):
    return np.frombuffer(base64.b64decode(s), dtype="<f8").reshape(shape).copy()


def tile_hashes(server):
    """타일마다 물리 높이(compose)의 sha256. 사본끼리 같으면 같은 지형."""
    t, out = server.tile, {}
    for tr in range(0, (server.nrow + t - 1) // t):
        for tc in range(0, (server.ncol + t - 1) // t):
            r0, c0 = tr * t, tc * t
            h = server.compose(r0, min(r0 + t, server.nrow), c0, min(c0 + t, server.ncol))
            out[f"{tr},{tc}"] = hashlib.sha256(np.ascontiguousarray(h, dtype="<f8").tobytes()).hexdigest()
    return out


def apply_message(server, m):
    """서버와 클라이언트가 같은 방식으로 메시지를 사본에 적용한다 (한 곳)."""
    if m["type"] == "write":
        server.write(m["op"], m["writer"], m["r0"], m["c0"], dec(m["data"], (m["nrow"], m["ncol"])), t=m.get("t"))
    elif m["type"] == "cells":
        n = m["n"]
        server.write_cells(m["op"], m["writer"], np.frombuffer(base64.b64decode(m["rows"]), "<i4"),
                           np.frombuffer(base64.b64decode(m["cols"]), "<i4"), dec(m["values"], (n,)), t=m.get("t"))
    elif m["type"] == "owner":
        server.set_owner(m["r0"], m["r1"], m["c0"], m["c1"], m["writer"])
    else:
        raise ValueError(f"모르는 메시지: {m['type']}")


class RemoteMap:
    """MapServer처럼 쓴다 (TerrainMapService.attach_server(remote=...)). name = 쓴 쪽 이름."""

    def __init__(self, base, z_range, name="sim", wait_s=30.0):
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self._String = String
        self.local = MapServer(base, z_range)
        self.name, self.seq = name, 0
        self.sent = {"init": 0, "owner": 0, "write": 0}              # 보낸 메시지 종류별 수 (write = write + cells, 지도 변경)
        self.node = rclpy.create_node(f"map_client_{name}")
        rel = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.node.create_publisher(String, "/map/write", rel)
        self.pub_req = self.node.create_publisher(String, "/map/request", rel)
        self.foreign = []                                        # 받은 다른 쪽 변경 (poll에서 적용)
        self.rx = 0                                              # 받은 /map/updates 수 (자기 메아리 포함)
        self.replies, self.info = [], None
        self.req_id = 0                                          # 요청 번호 (이 클라이언트 안에서 계속 증가: 예전 응답을 잘못 집지 않게)
        self.node.create_subscription(String, "/map/updates", self._on_update, rel)
        self.node.create_subscription(String, "/map/reply", lambda m: self.replies.append(json.loads(m.data)), rel)
        self.node.create_subscription(String, "/map/info", lambda m: setattr(self, "info", json.loads(m.data)), latched)
        t0 = time.monotonic()
        while self.pub.get_subscription_count() == 0:            # 서버가 뜰 때까지 (안 기다리면 init이 사라진다)
            if time.monotonic() - t0 > wait_s:
                raise RuntimeError("맵 서버가 /map/write를 구독하지 않는다 (python -m sim.map_server_node)")
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._send({"type": "init", "writer": name, "shape": list(base.shape), "z_range": list(z_range), "base": enc(base)})
        mine = hashlib.sha256(np.ascontiguousarray(base, dtype="<f8").tobytes()).hexdigest()
        t0 = time.monotonic()
        while not (self.info and (self.info.get("base_sha256") == mine or name in self.info.get("refused", []))):
            if time.monotonic() - t0 > wait_s:
                raise RuntimeError("맵 서버가 init에 답하지 않는다 (/map/info)")
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.info.get("base_sha256") != mine:                 # 서버 지도의 바탕이 이 클라이언트의 시작 지형과 다르다
            raise RuntimeError(f"맵 서버의 바탕이 다르다 (서버 {self.info.get('base_sha256', '')[:12]}, 이쪽 {mine[:12]})")

    def __getattr__(self, k):                                    # owned, owners, counts, version, compose ... 는 사본에서
        return getattr(self.local, k)

    def _send(self, m):
        self.seq += 1
        self.sent["write" if m["type"] in ("write", "cells") else m["type"]] += 1
        m["seq"], m["from"] = self.seq, self.name                 # from = 보낸 클라이언트 (seq는 클라이언트마다), writer = 변경의 쓴 쪽
        self.pub.publish(self._String(data=json.dumps(m)))

    def _on_update(self, msg):
        self.rx += 1
        m = json.loads(msg.data)
        if m.get("from") != self.name:                           # 자기 변경은 이미 사본에 있다
            self.foreign.append(m)

    def poll(self):
        """받은 다른 쪽 변경을 사본에 반영. 반환: 반영한 개수."""
        import rclpy
        for _ in range(100000):                                  # spin_once는 콜백을 하나만 부른다: 더 받을 것이 없을 때까지
            before = self.rx
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self.rx == before:
                break
        n = len(self.foreign)
        for m in self.foreign:
            apply_message(self.local, m)
        self.foreign = []
        return n

    # ---- MapServer와 같은 쓰기 (사본에 먼저, 그다음 서버로) ----
    def write(self, op, writer, r0, c0, block, t=None):
        out = self.local.write(op, writer, r0, c0, block, t=t)
        block = np.asarray(block, dtype=float)
        self._send({"type": "write", "op": op, "writer": writer, "from": self.name, "t": t, "r0": int(r0), "c0": int(c0),
                    "nrow": int(block.shape[0]), "ncol": int(block.shape[1]), "data": enc(block)})
        return out

    def write_cells(self, op, writer, rows, cols, values, t=None):
        out = self.local.write_cells(op, writer, rows, cols, values, t=t)
        rows, cols = np.asarray(rows, "<i4"), np.asarray(cols, "<i4")
        if rows.size:
            self._send({"type": "cells", "op": op, "writer": writer, "from": self.name, "t": t, "n": int(rows.size),
                        "rows": base64.b64encode(rows.tobytes()).decode(), "cols": base64.b64encode(cols.tobytes()).decode(),
                        "values": enc(np.asarray(values, float))})
        return out

    def set_owner(self, r0, r1, c0, c1, writer):
        self.local.set_owner(r0, r1, c0, c1, writer)
        self._send({"type": "owner", "writer": writer, "from": self.name, "r0": int(r0), "r1": int(r1), "c0": int(c0), "c1": int(c1)})

    # ---- 끝: 서버가 다 받았는지, 사본이 같은지 ----
    def audit(self, wait_s=30.0):
        """서버가 이 클라이언트의 마지막 seq까지 받고, 이 클라이언트가 서버의 판까지 다 받을 때까지(같은 판에서 비교) 기다린 뒤
        타일 해시를 대조. 반환: dict (summary에 넣는다)."""
        import rclpy
        t0 = time.monotonic()
        while True:
            self.req_id += 1
            k = self.req_id
            self.pub_req.publish(self._String(data=json.dumps({"type": "audit", "client": self.name, "id": k})))
            t1 = time.monotonic()
            got = None
            while got is None and time.monotonic() - t1 < 1.0:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                got = next((r for r in self.replies if r.get("client") == self.name and r.get("id") == k), None)
            self.replies = [r for r in self.replies if r.get("id", 0) > k]   # 이번 것까지 다 본 응답은 버린다
            self.poll()
            if got and got["seq"].get(self.name, 0) >= self.seq and got["version"] == self.local.version:
                break
            if time.monotonic() - t0 > wait_s:
                return {"server": "no reply" if got is None else
                        f"server got seq {got['seq'].get(self.name, 0)} / {self.seq}, version {got['version']} / {self.local.version}"}
        mine = tile_hashes(self.local)
        bad = sorted(k for k in mine if got["hashes"].get(k) != mine[k])
        return {"server_version": got["version"], "client_version": self.local.version, "sent": dict(self.sent, total=self.seq),
                "tiles": len(mine), "hash_mismatch": len(bad), "mismatched_tiles": bad[:20],
                "wait_s": round(time.monotonic() - t0, 3)}

    def summary(self):
        s = self.local.summary()
        s["mode"] = "process (ros2)"
        return s

    def close(self):
        self.node.destroy_node()
