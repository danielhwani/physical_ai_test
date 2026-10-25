"""맵 서버 원격 클라이언트 (docs/map_server.md 2단계). sim/map_server.py의 MapServer와 같은 함수를 가지고, 안에 사본(MapServer)을 둔다.
자기 변경은 사본에 바로 반영한 뒤 서버로 보낸다 (물리 결과가 서버와 무관: 원칙 3 "실시간 루프는 서버를 기다리지 않는다").
다른 쪽 변경은 poll()에서 받아 사본에 반영한다.

    python -m sim.map_client --observe        늦게 붙는 관찰자: 지금 지도를 받아 따라가며 판, 지연을 찍는다 (Ctrl+C로 끝내면 감사)

  전송 (ROS2 std_msgs/String, JSON. 높이는 float64 little-endian base64: 사본이 비트 단위로 같아야 해서)
    /map/write    클라이언트 -> 서버   {"type": "init" | "write" | "cells" | "owner", "writer", "from", "seq", "wall", ...}
    /map/updates  서버 -> 클라이언트   위 메시지에 서버가 매긴 판 "v"와 내보낸 번호 "n"을 붙여 그대로 (init 제외)
    /map/info     서버 (늦게 붙어도 받음) {"format", "rules", "tile", "grid", "z_range", "version", "n", "owners", "writers", ...}
    /map/request  클라이언트 -> 서버   {"type": "audit" | "snapshot", "client", "id"}
    /map/reply    서버 -> 클라이언트   audit: {"version", "seq": {클라이언트: 마지막 seq}, "hashes": {타일: sha256}}
                                       snapshot: {"n", "state": 서버 상태 전체 (MapServer.state, 아래 enc_state)}
  보낸 클라이언트(from)마다 seq를 1부터 붙인다 (모든 메시지: init, owner, write, cells). 서버는 받은 순서로 판을 매긴다.
  판 = 적용한 지도 변경 메시지 수 (write, cells). init(접속·바탕)과 owner(소유 영역 선언)는 판을 올리지 않는다. 값이 실제로 안 바뀐
  변경(예: min에서 더 낮지 않은 값만)도 판을 하나 올린다. 그래서 seq = 판 + init + owner (이 클라이언트만 쓴 경우).
  n = 서버가 /map/updates로 내보낸 메시지 번호 (owner 포함). 붙는 절차는 판이 아니라 n으로 가른다 (owner는 판이 같아서).

  붙는 절차 (설계 6.1): /map/updates를 먼저 구독해 쌓아 두고, 스냅샷(상태와 그때의 n)을 받아 사본을 만든 뒤, 쌓인 것 중 n이 더 큰
  것만 적용한다. 바탕이 같은 클라이언트가 나중에 붙을 때와 관찰자(base=None)가 이 절차를 쓴다 (첫 클라이언트는 자기 바탕이 곧 지도).
  지연 (wall 시각, 같은 PC): recv = 이 클라이언트가 보낸 때부터 서버가 받기까지 (서버가 잰다), apply = 다른 쪽 변경을 보낸 때부터 이
  사본에 반영하기까지 (poll 주기가 들어간다: 시뮬레이터는 지형 반영 5 Hz, 물리가 그 변경을 보는 시각). echo = 자기 변경이 서버를
  거쳐 돌아와 poll에서 확인되기까지 (poll 주기가 대부분이라 MOP에는 쓰지 않는다).
"""
import argparse
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


def sha(a):
    return hashlib.sha256(np.ascontiguousarray(a, dtype="<f8").tobytes()).hexdigest()


def enc_state(st):
    """MapServer.state() -> JSON에 실을 수 있는 dict (타일 키 "tr,tc")."""
    key = lambda k: f"{k[0]},{k[1]}"   # noqa: E731
    return {"grid": st["grid"], "z_range": st["z_range"], "tile": st["tile"], "version": st["version"], "E": enc(st["E"]),
            "D": {key(k): [list(v.shape), enc(v)] for k, v in st["D"].items()},
            "O": {key(k): [list(v.shape), enc(v)] for k, v in st["O"].items()},
            "owners": st["owners"], "tile_version": {key(k): v for k, v in st["tile_version"].items()}, "counts": st["counts"]}


def dec_state(j):
    key = lambda s: tuple(int(v) for v in s.split(","))   # noqa: E731
    return {"grid": j["grid"], "z_range": j["z_range"], "tile": j["tile"], "version": j["version"], "E": dec(j["E"], tuple(j["grid"])),
            "D": {key(k): dec(v, tuple(sh)) for k, (sh, v) in j["D"].items()},
            "O": {key(k): dec(v, tuple(sh)) for k, (sh, v) in j["O"].items()},
            "owners": j["owners"], "tile_version": {key(k): v for k, v in j["tile_version"].items()}, "counts": j["counts"]}


def tile_hashes(server):
    """타일마다 물리 높이(compose)의 sha256. 사본끼리 같으면 같은 지형."""
    t, out = server.tile, {}
    for tr in range(0, (server.nrow + t - 1) // t):
        for tc in range(0, (server.ncol + t - 1) // t):
            r0, c0 = tr * t, tc * t
            out[f"{tr},{tc}"] = sha(server.compose(r0, min(r0 + t, server.nrow), c0, min(c0 + t, server.ncol)))
    return out


def stats_ms(xs):
    """지연 분포 (s 목록 -> ms 요약)."""
    if not xs:
        return {"n": 0}
    a = 1e3 * np.asarray(xs)
    return {"n": int(a.size), "p50": round(float(np.percentile(a, 50)), 2), "p95": round(float(np.percentile(a, 95)), 2),
            "max": round(float(a.max()), 2)}


def audit_mop(au):
    """감사 결과 -> MOP (설계 부록: 클라이언트 간 지형 일관성, 지형 동기화 지연). 서버가 답하지 않았으면 불일치 None."""
    lat = au.get("latency_ms", {})
    return {"map_hash_mismatch": au.get("hash_mismatch"),
            "map_recv_latency_p95_ms": lat.get("recv", {}).get("p95"),      # 이 클라이언트가 보낸 변경이 서버에 닿기까지
            "map_apply_latency_p95_ms": lat.get("apply", {}).get("p95")}    # 다른 쪽 변경이 이 사본(물리)에 들어가기까지


def apply_message(server, m):
    """서버와 클라이언트가 같은 방식으로 메시지를 사본에 적용한다 (한 곳)."""
    if m["type"] == "write":
        server.write(m["op"], m["writer"], m["r0"], m["c0"], dec(m["data"], (m["nrow"], m["ncol"])), t=m.get("t"))
    elif m["type"] == "cells":
        n = m["n_cells"]
        server.write_cells(m["op"], m["writer"], np.frombuffer(base64.b64decode(m["rows"]), "<i4"),
                           np.frombuffer(base64.b64decode(m["cols"]), "<i4"), dec(m["values"], (n,)), t=m.get("t"))
    elif m["type"] == "owner":
        server.set_owner(m["r0"], m["r1"], m["c0"], m["c1"], m["writer"])
    else:
        raise ValueError(f"모르는 메시지: {m['type']}")


class RemoteMap:
    """MapServer처럼 쓴다 (TerrainMapService.attach_server(remote=...)). name = 이 클라이언트 이름 (서버 안에서 하나뿐이어야 함).
    base=None: 관찰자 (바탕을 보내지 않고 서버의 지금 지도를 받아 붙는다)."""

    def __init__(self, base, z_range=None, name="sim", wait_s=30.0):
        import rclpy
        import rclpy.executors
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self._String = String
        self.name, self.seq, self.wait_s = name, 0, wait_s
        self.sent = {"init": 0, "owner": 0, "write": 0}              # 보낸 메시지 종류별 수 (write = write + cells, 지도 변경)
        self.node = rclpy.create_node(f"map_client_{name}")
        # 자기 실행기: rclpy.spin_once(node)는 전역 실행기라 한 프로세스의 다른 노드(클라이언트 둘, 브리지 등) 콜백도 돌린다
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.n_cb = 0                                            # 이 노드의 콜백 수 (poll이 더 받을 것이 없을 때까지 돌기 위해)
        rel = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.node.create_publisher(String, "/map/write", rel)
        self.pub_req = self.node.create_publisher(String, "/map/request", rel)
        self.foreign = []                                        # 받은 다른 쪽 변경 (poll에서 적용)
        self.rx = 0                                              # 받은 /map/updates 수 (자기 메아리 포함)
        self.applied_n = 0                                       # 사본에 들어간 다른 쪽 변경의 마지막 서버 번호 n (스냅샷이면 그때의 n)
        self.lat = {"echo": [], "apply": []}                     # 지연 (s)
        self.replies, self.info = [], None
        self.req_id = 0                                          # 요청 번호 (이 클라이언트 안에서 계속 증가: 예전 응답을 잘못 집지 않게)
        self.joined = None                                       # 붙는 절차를 썼으면 {"version", "n", "s"}
        self.sub_up = self.node.create_subscription(String, "/map/updates", self._on_update, rel)   # 붙는 절차 1: 먼저 구독
        self.node.create_subscription(String, "/map/reply", self._on_reply, rel)
        self.node.create_subscription(String, "/map/info", self._on_info, latched)
        self._wait(lambda: self.pub.get_subscription_count() > 0 and self.node.count_publishers("/map/updates") > 0,
                   "맵 서버가 없다 (python -m sim.map_server_node)")     # 안 기다리면 init이 사라진다
        if base is None:                                         # 관찰자: 서버에 지도가 있을 때까지
            self._wait(lambda: self.info and "grid" in self.info, "맵 서버에 아직 지도가 없다 (첫 클라이언트의 init 전)")
            self._join()
            return
        self.local = MapServer(base, z_range)
        nonce = hashlib.sha256(f"{name} {time.time_ns()} {id(self)}".encode()).hexdigest()[:16]   # 이번 init의 답을 알아보는 표
        self._send({"type": "init", "writer": name, "shape": list(base.shape), "z_range": list(z_range), "base": enc(base),
                    "nonce": nonce})
        mine = sha(base)
        ack = lambda: (self.info or {}).get("inits", {}).get(name, {})   # noqa: E731
        self._wait(lambda: ack().get("nonce") == nonce, "맵 서버가 init에 답하지 않는다 (/map/info)")
        result = ack()["result"]                                 # first (이 바탕으로 지도를 만듦) | joined | refused
        if result == "refused":                                  # 서버 지도의 바탕이 이 클라이언트의 시작 지형과 다르다
            raise RuntimeError(f"맵 서버의 바탕이 다르다 (서버 {self.info.get('base_sha256', '')[:12]}, 이쪽 {mine[:12]})")
        if result == "joined":                                   # 나중에 붙음: 그 사이 서버에 들어간 변경을 받는다
            self._join()

    def __getattr__(self, k):                                    # owned, owners, counts, version, compose ... 는 사본에서
        if k == "local":
            raise AttributeError(k)
        return getattr(self.local, k)

    def _wait(self, cond, msg):
        t0 = time.monotonic()
        while not cond():
            if time.monotonic() - t0 > self.wait_s:
                raise RuntimeError(msg)
            self.executor.spin_once(timeout_sec=0.05)

    def _request(self, kind, timeout=1.0):
        """요청 하나 보내고 그 응답을 기다린다 (없으면 None)."""
        self.req_id += 1
        k = self.req_id
        self.pub_req.publish(self._String(data=json.dumps({"type": kind, "client": self.name, "id": k})))
        t1 = time.monotonic()
        got = None
        while got is None and time.monotonic() - t1 < timeout:
            self.executor.spin_once(timeout_sec=0.05)
            got = next((r for r in self.replies if r.get("id") == k), None)
        self.replies = [r for r in self.replies if r.get("id", 0) > k]   # 이번 것까지 다 본 응답은 버린다
        return got

    def _join(self):
        """붙는 절차 2, 3: 스냅샷으로 사본을 만들고, 쌓인 변경 중 스냅샷 뒤(n이 더 큰) 것만 poll에서 적용한다."""
        t0 = time.monotonic()
        snap = None
        while snap is None:
            snap = self._request("snapshot", timeout=5.0)
            if snap is None and time.monotonic() - t0 > self.wait_s:
                raise RuntimeError("맵 서버가 스냅샷을 주지 않는다")
        self.local = MapServer.from_state(dec_state(snap["state"]))
        self.applied_n = snap["n"]
        self.joined = {"version": self.local.version, "n": snap["n"], "s": round(time.monotonic() - t0, 3)}
        self.poll()

    def _send(self, m):
        self.seq += 1
        self.sent["write" if m["type"] in ("write", "cells") else m["type"]] += 1
        m["seq"], m["from"], m["wall"] = self.seq, self.name, time.time()   # from = 보낸 클라이언트, writer = 변경의 쓴 쪽
        self.pub.publish(self._String(data=json.dumps(m)))

    def _on_reply(self, msg):
        self.n_cb += 1
        m = json.loads(msg.data)
        if m.get("client") == self.name:                         # 다른 클라이언트의 응답(스냅샷 등)은 바로 버린다
            self.replies.append(m)

    def _on_info(self, msg):
        self.n_cb += 1
        self.info = json.loads(msg.data)

    def _on_update(self, msg):
        self.n_cb += 1
        self.rx += 1
        m = json.loads(msg.data)
        if m.get("from") == self.name:                           # 자기 변경은 이미 사본에 있다
            self.lat["echo"].append(time.time() - m["wall"])   # applied_n은 건드리지 않는다 (아직 안 받은 앞 번호의 다른 쪽 변경이 걸러진다)
        else:
            self.foreign.append(m)

    def poll(self):
        """받은 다른 쪽 변경을 사본에 반영. 반환: 반영한 개수."""
        # spin_once는 콜백을 하나만 부르고, timeout 0이면 남은 것이 있어도 번갈아 빈손으로 돌아온다 (Humble: 1, 0, 1, 0 ...).
        # 그래서 3번 연속 빈손이면 다 받은 것으로 본다 (기다리지 않음: 그 뒤에 온 것은 다음 poll에서)
        empty = 0
        for _ in range(100000):
            before = self.n_cb
            self.executor.spin_once(timeout_sec=0.0)
            empty = 0 if self.n_cb != before else empty + 1
            if empty >= 3:
                break
        if not hasattr(self, "local"):                           # 붙기 전 (스냅샷을 기다리는 중): 쌓아 두기만
            return 0
        n, now = 0, time.time()
        for m in sorted(self.foreign, key=lambda m: m["n"]):
            if m["n"] <= self.applied_n:                         # 스냅샷에 이미 들어 있다
                continue
            apply_message(self.local, m)
            self.applied_n = m["n"]
            self.lat["apply"].append(now - m["wall"])
            n += 1
        self.foreign = []
        return n

    # ---- MapServer와 같은 쓰기 (사본에 먼저, 그다음 서버로) ----
    def write(self, op, writer, r0, c0, block, t=None):
        out = self.local.write(op, writer, r0, c0, block, t=t)
        block = np.asarray(block, dtype=float)
        self._send({"type": "write", "op": op, "writer": writer, "t": t, "r0": int(r0), "c0": int(c0),
                    "nrow": int(block.shape[0]), "ncol": int(block.shape[1]), "data": enc(block)})
        return out

    def write_cells(self, op, writer, rows, cols, values, t=None):
        out = self.local.write_cells(op, writer, rows, cols, values, t=t)
        rows, cols = np.asarray(rows, "<i4"), np.asarray(cols, "<i4")
        if rows.size:
            self._send({"type": "cells", "op": op, "writer": writer, "t": t, "n_cells": int(rows.size),
                        "rows": base64.b64encode(rows.tobytes()).decode(), "cols": base64.b64encode(cols.tobytes()).decode(),
                        "values": enc(np.asarray(values, float))})
        return out

    def set_owner(self, r0, r1, c0, c1, writer):
        self.local.set_owner(r0, r1, c0, c1, writer)
        self._send({"type": "owner", "writer": writer, "r0": int(r0), "r1": int(r1), "c0": int(c0), "c1": int(c1)})

    # ---- 끝: 서버가 다 받았는지, 사본이 같은지 ----
    def audit(self, wait_s=30.0):
        """서버가 이 클라이언트의 마지막 seq까지 받고, 이 클라이언트가 서버의 판까지 다 받을 때까지(같은 판에서 비교) 기다린 뒤
        타일 해시를 대조. 반환: dict (summary에 넣는다)."""
        t0 = time.monotonic()
        while True:
            got = self._request("audit")
            self.poll()
            if got and got["seq"].get(self.name, 0) >= self.seq and got["version"] == self.local.version:
                break
            if time.monotonic() - t0 > wait_s:
                return {"server": "no reply (서버가 끝났거나 없다)" if got is None else
                        f"server got seq {got['seq'].get(self.name, 0)} / {self.seq}, version {got['version']} / {self.local.version}"}
        mine = tile_hashes(self.local)
        bad = sorted(k for k in mine if got["hashes"].get(k) != mine[k])
        out = {"server_version": got["version"], "client_version": self.local.version, "sent": dict(self.sent, total=self.seq),
               "tiles": len(mine), "hash_mismatch": len(bad), "mismatched_tiles": bad[:20],
               "wait_s": round(time.monotonic() - t0, 3),
               "latency_ms": {"recv": got.get("recv_latency_ms", {"n": 0}), **{k: stats_ms(v) for k, v in self.lat.items()}}}
        if self.joined:
            out["joined"] = self.joined
        return out

    def summary(self):
        s = self.local.summary()
        s["mode"] = "process (ros2)"
        return s

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()


def main():
    ap = argparse.ArgumentParser(description="맵 서버 클라이언트 (관찰자)")
    ap.add_argument("--observe", action="store_true", help="늦게 붙어 지금 지도를 받고 따라간다 (쓰지 않음)")
    ap.add_argument("--name", default="observer")
    ap.add_argument("--duration", type=float, default=0.0, help="이 시간(s) 뒤 감사하고 끝 (0: Ctrl+C까지)")
    a = ap.parse_args()
    if not a.observe:
        ap.error("지금은 --observe만 있다")
    m = RemoteMap(None, name=a.name)
    print(f"붙음: 판 {m.local.version}, 서버 번호 n {m.joined['n']}, 스냅샷 {m.joined['s']} s, 격자 {m.local.nrow} x {m.local.ncol}, "
          f"소유 영역 {len(m.local.owners)}", flush=True)
    t0, last = time.monotonic(), 0.0
    try:
        while a.duration <= 0 or time.monotonic() - t0 < a.duration:
            m.poll()
            if time.monotonic() - last >= 1.0:
                last = time.monotonic()
                s = stats_ms(m.lat["apply"][-200:])
                print(f"판 {m.local.version:5d}  받은 변경 {len(m.lat['apply']):5d}  반영 지연 p50 {s.get('p50', '-')} ms "
                      f"p95 {s.get('p95', '-')} ms", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    au = m.audit(wait_s=10.0)
    print(f"감사: {au}", flush=True)
    m.close()


if __name__ == "__main__":
    main()
