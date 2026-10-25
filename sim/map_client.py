"""맵 서버 원격 클라이언트 (docs/map_server.md 2단계). sim/map_server.py의 MapServer와 같은 함수를 가지고, 안에 사본(MapServer)을 둔다.
자기 변경은 사본에 바로 반영한 뒤 서버로 보낸다 (물리 결과가 서버와 무관: 원칙 3 "실시간 루프는 서버를 기다리지 않는다").
다른 쪽 변경은 poll()에서 받아 사본에 반영한다.

    python -m sim.map_client --observe        늦게 붙는 관찰자: 지금 지도를 받아 따라가며 판, 지연을 찍는다 (Ctrl+C로 끝내면 감사)

  전송 (ROS2 std_msgs/String, JSON. 높이는 float64 little-endian base64: 사본이 비트 단위로 같아야 해서)
    /map/write    클라이언트 -> 서버   {"type": "init" | "write" | "cells" | "owner", "writer", "from", "seq", "wall", ...}
    /map/updates  서버 -> 클라이언트   위 메시지에 서버가 매긴 판 "v"와 내보낸 번호 "n"을 붙여 그대로 (init 제외)
    /map/info     서버 (늦게 붙어도 받음) {"format", "rules", "tile", "grid", "z_range", "version", "n", "owners", "writers", ...}
    /map/request  클라이언트 -> 서버   {"type": "audit" | "snapshot" | "replay" | "save", "client", "id", ...}
    /map/reply    서버 -> 클라이언트   audit: {"version", "seq": {클라이언트: 마지막 seq}, "hashes": {타일: sha256}}
                                       snapshot: {"n", "seq", "state": 서버 상태 전체 (MapServer.state, 아래 enc_state)}
                                       replay: {"from", "to", "msgs" | "too_old"}, resend: {"from", "to"} (서버가 빈칸을 봄)
  보낸 클라이언트(from)마다 seq를 1부터 붙인다 (모든 메시지: init, owner, write, cells). 서버는 받은 순서로 판을 매긴다.
  판 = 적용한 지도 변경 메시지 수 (write, cells). init(접속·바탕)과 owner(소유 영역 선언)는 판을 올리지 않는다. 값이 실제로 안 바뀐
  변경(예: min에서 더 낮지 않은 값만)도 판을 하나 올린다. 그래서 seq = 판 + init + owner (이 클라이언트만 쓴 경우).
  n = 서버가 /map/updates로 내보낸 메시지 번호 (owner 포함). 붙는 절차는 판이 아니라 n으로 가른다 (owner는 판이 같아서).

  붙는 절차 (설계 6.1): /map/updates를 먼저 구독해 쌓아 두고, 스냅샷(상태와 그때의 n)을 받아 사본을 만든 뒤, 쌓인 것 중 n이 더 큰
  것만 적용한다. 바탕이 같은 클라이언트가 나중에 붙을 때와 관찰자(base=None)가 이 절차를 쓴다 (첫 클라이언트는 자기 바탕이 곧 지도).
  지연 (wall 시각, 같은 PC): recv = 이 클라이언트가 보낸 때부터 서버가 받기까지 (서버가 잰다), apply = 다른 쪽 변경을 보낸 때부터 이
  사본에 반영하기까지 (poll 주기가 들어간다: 시뮬레이터는 지형 반영 5 Hz, 물리가 그 변경을 보는 시각). echo = 자기 변경이 서버를
  거쳐 돌아와 poll에서 확인되기까지 (poll 주기가 대부분이라 MOP에는 쓰지 않는다).
  빈칸 (2d, 설계 7절): 보낸 것은 메아리가 올 때까지 들고 있다가 서버가 resend로 달라면, 또는 RESEND_S 동안 메아리가 없으면 다시 보낸다.
  받는 쪽은 n이 이어질 때만 적용하고, 빈칸이 있으면(뒤 번호가 와 있거나 /map/info의 n보다 덜 받음) GAP_ASK_S마다 replay로 다시 달라고
  한다. 서버가 그 구간을 더 들고 있지 않으면(too_old) 스냅샷으로 다시 맞추고, 서버가 아직 못 받은 자기 변경을 그 위에 다시 넣는다.
  빈칸 지연: recovery = 빈칸을 알아챈 때부터 메워지기까지 (알아채는 데는 다음 메시지가 와야 해서 그 시간이 빠진다),
  apply_late / server_side_delay = 빈칸 때문에 늦게 반영된 변경의 보낸 때부터 반영까지 (물리가 실제로 늦게 본 시간, MOP).
  다시 받을 수 없는 빈칸: 받는 쪽은 스냅샷으로 다시 맞추므로 잃지 않는다. 보내는 쪽(이 클라이언트가 죽었거나 UNACKED_MAX를 넘어 버림)은
  서버가 GIVEUP_S 뒤 건너뛰고 잃은 변경으로 센다 (MOP map_lost_changes). 소유 영역이 있으면 서버가 refresh_owned로 영역의 지금
  높이를 통째로 달라고 하고, 이 클라이언트는 사본의 그 영역을 own 쓰기 하나로 보낸다 (절대 높이라 빠진 것과 상관없이 맞는다).
"""
import argparse
import base64
import hashlib
import json
import time

import numpy as np

from .map_server import MapServer

TOPICS = ("/map/write", "/map/updates", "/map/info", "/map/request", "/map/reply")
GAP_ASK_S = 0.2       # 받는 쪽 빈칸: 이 주기로 다시 달라고 한다 (s)
RESEND_S = 1.0        # 보낸 것의 메아리가 이만큼 없으면 다시 보낸다 (s)
UNACKED_MAX = 10000   # 메아리를 기다리며 들고 있는 보낸 것의 상한 (넘으면 오래된 것부터 버린다: 서버가 오래 없을 때 메모리)


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
    lat, g = au.get("latency_ms", {}), au.get("gaps", {})
    rec = [v for v in (lat.get("apply_late", {}).get("max"), g.get("server_side_delay_ms", {}).get("max")) if v is not None]
    return {"map_hash_mismatch": au.get("hash_mismatch"),
            "map_seq_gaps": None if not g else g["recv_side"] + g.get("server_side_recovery_ms", {}).get("n", 0),   # 빈칸 수 (받는 쪽 + 서버 쪽)
            "map_gap_delay_max_ms": max(rec) if rec else None,           # 빈칸 때문에 늦은 변경: 보낸 때부터 반영까지 (알아채기까지 포함)
            "map_lost_changes": None if not g else sum(d.get("lost", 0) for d in g.get("server_lost", {}).values()),   # 서버가 끝내 적용 못 한 변경 (모든 클라이언트)
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
    base=None: 관찰자 (바탕을 보내지 않고 서버의 지금 지도를 받아 붙는다).
    drop: (보내기, 받기) 일부러 버릴 비율 (빈칸 복구 시험용, 0이면 없음). 버린 것은 다시 보내기·다시 받기로 메워져야 한다."""

    def __init__(self, base, z_range=None, name="sim", wait_s=30.0, drop=(0.0, 0.0), drop_seed=0, unacked_max=UNACKED_MAX):
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
        self.pending = {}                                        # n -> 받은 /map/updates (자기 메아리 포함), n 순서대로 소비
        self.next_n = 1                                          # 다음에 받을 서버 번호 n (그 앞은 사본에 다 들어 있다)
        self.unacked = {}                                        # seq -> [JSON, 보낸 시각]: 메아리가 올 때까지 (다시 보내기)
        self.unacked_max = unacked_max
        self.my_owners = []                                      # 이 클라이언트가 선언한 소유 영역 (r0, r1, c0, c1, 쓴 쪽): 다시 보내기용
        self.gap = None                                          # 받는 쪽 빈칸 {"n": 빠진 첫 번호, "since", "asked"} (monotonic)
        self.need_resync = False                                 # 서버가 빈칸 구간을 더 들고 있지 않다 (too_old): 다음 poll에서 스냅샷
        self.stats = {"gaps": 0, "replayed": 0, "resyncs": 0, "resent": 0, "dropped_out": 0, "dropped_in": 0, "recovery": [],
                      "evicted": 0, "refreshed": 0}
        self.drop, self._rng = drop, np.random.default_rng(drop_seed)
        self.rx = 0                                              # 받은 /map/updates 수 (자기 메아리 포함)
        self.lat = {"echo": [], "apply": [], "apply_late": []}   # 지연 (s). apply_late: 빈칸 때문에 늦게 반영된 것 (다시 받은 것 + 그 뒤에 기다린 것)
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
        from .map_base import tile_shas
        self.local = MapServer(base, z_range)
        self._wait(lambda: self.info is not None, "맵 서버가 /map/info를 내지 않는다")
        nonce = hashlib.sha256(f"{name} {time.time_ns()} {id(self)}".encode()).hexdigest()[:16]   # 이번 init의 답을 알아보는 표
        m = {"type": "init", "writer": name, "shape": list(base.shape), "z_range": list(z_range), "nonce": nonce,
             "base_sha256": sha(base), "tiles": tile_shas(base)}           # 바탕은 해시로 (2c)
        if "grid" not in self.info:                              # 서버에 아직 지도가 없다 (바탕 파일 없이 띄운 서버): 격자도 보낸다
            m["base"] = enc(base)
        self.init_bytes = len(json.dumps(m))
        self._send(m)
        ack = lambda: (self.info or {}).get("inits", {}).get(name, {})   # noqa: E731
        self._wait(lambda: ack().get("nonce") == nonce, "맵 서버가 init에 답하지 않는다 (/map/info)")
        a = ack()                                                # result: first (이 격자로 지도를 만듦) | joined | refused | no_map
        if a["result"] == "refused":                             # 서버 지도의 바탕이 이 클라이언트의 시작 지형과 다르다
            raise RuntimeError(f"맵 서버의 바탕이 다르다 (서버 {self.info.get('base_path') or '첫 클라이언트의 격자'}, "
                               f"다른 타일 {a.get('bad_tiles')})")
        if a["result"] == "no_map":
            raise RuntimeError("맵 서버에 지도가 없다 (바탕 파일 없이 띄웠고 격자를 받지 못했다)")
        self.next_n = a.get("n", 0) + 1                          # init을 받은 때까지 서버가 내보낸 것은 바탕 + 아래 스냅샷에 있다
        if a["result"] == "joined" and a.get("n", 0) > 0:      # 서버에 이미 변경이 있다: 그것까지 받는다 (없으면 바탕이 곧 지금 지도)
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

    def _publish_req(self, r):
        self.req_id += 1
        self.pub_req.publish(self._String(data=json.dumps(dict(r, client=self.name, id=self.req_id))))
        return self.req_id

    def _request(self, kind, timeout=1.0, **extra):
        """요청 하나 보내고 그 응답을 기다린다 (없으면 None)."""
        k = self._publish_req({"type": kind, **extra})
        t1 = time.monotonic()
        got = None
        while got is None and time.monotonic() - t1 < timeout:
            self.executor.spin_once(timeout_sec=0.05)
            got = next((r for r in self.replies if r.get("id") == k), None)
        self.replies = [r for r in self.replies if r.get("id", 0) > k]   # 이번 것까지 다 본 응답은 버린다
        return got

    def _snapshot(self):
        t0 = time.monotonic()
        snap = None
        while snap is None:
            snap = self._request("snapshot", timeout=5.0)
            if snap is None and time.monotonic() - t0 > self.wait_s:
                raise RuntimeError("맵 서버가 스냅샷을 주지 않는다")
        self.local = MapServer.from_state(dec_state(snap["state"]))
        self.next_n = snap["n"] + 1
        self.pending = {k: v for k, v in self.pending.items() if k >= self.next_n}
        return snap, time.monotonic() - t0

    def _join(self):
        """붙는 절차 2, 3: 스냅샷으로 사본을 만들고, 쌓인 변경 중 스냅샷 뒤(n이 더 큰) 것만 poll에서 적용한다."""
        snap, took = self._snapshot()
        self.joined = {"version": self.local.version, "n": snap["n"], "s": round(took, 3)}
        self.poll()

    def _resync(self):
        """다시 받을 수 없는 빈칸 (서버가 들고 있는 것보다 오래됨): 스냅샷으로 다시 맞추고, 서버가 아직 못 받은 자기 변경을 그 위에 다시 넣는다."""
        snap, _ = self._snapshot()
        done = snap["seq"].get(self.name, 0)
        for s in [k for k in self.unacked if k <= done]:        # 서버가 이미 받았다 (메아리만 못 받음): 스냅샷에 들어 있다
            del self.unacked[s]
        for s in sorted(self.unacked):                           # 서버가 아직 못 받은 자기 변경: 스냅샷 위에 다시
            apply_message(self.local, json.loads(self.unacked[s][0]))
        if self.gap is not None and not self.pending:           # 빈칸이 스냅샷으로 메워졌다
            self.stats["recovery"].append(time.monotonic() - self.gap["since"])
            self.gap = None
        self.stats["resyncs"] += 1

    def _send(self, m):
        self.seq += 1
        self.sent["write" if m["type"] in ("write", "cells") else m["type"]] += 1
        m["seq"], m["from"], m["wall"] = self.seq, self.name, time.time()   # from = 보낸 클라이언트, writer = 변경의 쓴 쪽
        data = json.dumps(m)
        if m["type"] != "init":                                  # 메아리가 올 때까지 들고 있다 (init은 답을 기다린다)
            self.unacked[self.seq] = [data, time.monotonic()]
            while len(self.unacked) > self.unacked_max:          # 상한: 가장 오래된 것을 버린다 (다시 보낼 수 없게 됨 -> 서버가 건너뜀)
                del self.unacked[min(self.unacked)]
                self.stats["evicted"] += 1
            if self.drop[0] and self._rng.random() < self.drop[0]:
                self.stats["dropped_out"] += 1                   # 시험: 보내지 않음 (서버가 빈칸을 보고 다시 달라거나, 시간이 지나 다시 보냄)
                return
        self.pub.publish(self._String(data=data))

    def _resend(self, seqs):
        for s in seqs:
            if s in self.unacked:
                self.pub.publish(self._String(data=self.unacked[s][0]))
                self.unacked[s][1] = time.monotonic()
                self.stats["resent"] += 1

    def _take(self, m):
        """/map/updates 하나 (또는 다시 받은 것): n 순서대로 소비하려고 쌓는다. 자기 메아리면 보낸 것을 내려놓는다."""
        if m.get("from") == self.name:
            if self.unacked.pop(m["seq"], None) is not None:
                self.lat["echo"].append(time.time() - m["wall"])
        if m["n"] >= self.next_n:
            self.pending[m["n"]] = m

    def _on_reply(self, msg):
        self.n_cb += 1
        m = json.loads(msg.data)
        if m.get("client") != self.name:                         # 다른 클라이언트의 응답(스냅샷 등)은 바로 버린다
            return
        if m["type"] == "resend":                                # 서버가 빈칸을 봄: 빠진 seq를 다시 보낸다
            self._resend(range(m["from"], m["to"] + 1))
        elif m["type"] == "refresh_owned":                       # 서버가 이 클라이언트의 변경을 건너뜀: 소유 영역을 통째로
            self.refresh_owned()
        elif m["type"] == "replay":                              # 다시 받은 /map/updates (또는 너무 오래돼 없음)
            if m.get("too_old"):
                self.need_resync = True
                return
            for d in m["msgs"]:
                self._take(json.loads(d))
            self.stats["replayed"] += len(m["msgs"])
        else:
            self.replies.append(m)

    def _on_info(self, msg):
        self.n_cb += 1
        self.info = json.loads(msg.data)

    def _on_update(self, msg):
        self.n_cb += 1
        self.rx += 1
        if self.drop[1] and self._rng.random() < self.drop[1]:
            self.stats["dropped_in"] += 1                        # 시험: 받지 않은 것으로 (빈칸 -> 다시 받기)
            return
        self._take(json.loads(msg.data))

    def poll(self):
        """받은 다른 쪽 변경을 n 순서대로 사본에 반영하고, 빈칸이 있으면 다시 달라고 한다. 반환: 반영한 개수 (다시 맞췄으면 1 더)."""
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
        n, now, mono = 0, time.time(), time.monotonic()
        if self.need_resync:
            self.need_resync = False
            self._resync()
            n += 1
        late = self.gap is not None                              # 빈칸이 열려 있었다: 이번에 반영하는 것은 그 때문에 늦었다
        while self.next_n in self.pending:                       # n 순서대로 (자기 메아리는 이미 사본에 있어 건너뛴다)
            m = self.pending.pop(self.next_n)
            if m.get("from") != self.name:
                apply_message(self.local, m)
                self.lat["apply"].append(now - m["wall"])
                if late:                                         # 보낸 때부터 (빈칸을 알아채기까지 걸린 시간 포함)
                    self.lat["apply_late"].append(now - m["wall"])
                n += 1
            self.next_n += 1
        if self.gap is not None and self.next_n > self.gap["n"]:   # 그 빈칸이 메워졌다 (빈칸마다 잰다)
            self.stats["recovery"].append(mono - self.gap["since"])
            self.gap = None
        # 빈칸: 뒤 번호가 와 있거나, 서버가 알린 n보다 덜 받음 (마지막 것이 사라진 경우)
        top = max(max(self.pending, default=0), (self.info or {}).get("n", 0))
        if top >= self.next_n:
            if self.gap is None:
                self.gap = {"n": self.next_n, "since": mono, "asked": 0.0}
                self.stats["gaps"] += 1
            if mono - self.gap["asked"] > GAP_ASK_S:
                hi = min(self.pending) - 1 if self.pending else top
                self._publish_req({"type": "replay", "from": self.next_n, "to": hi})
                self.gap["asked"] = mono
        done = (self.info or {}).get("writers", {}).get(self.name, 0)   # 서버가 받았다고 알린 seq: 메아리를 못 받아도 내려놓는다
        for s in [k for k in self.unacked if k <= done]:
            del self.unacked[s]
        old = [s for s, (_, t) in self.unacked.items() if mono - t > RESEND_S]   # 오래 메아리가 없다 (마지막 것이 사라졌을 수 있다)
        self._resend(old)
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

    def refresh_owned(self):
        """소유 영역마다 사본의 지금 높이를 own 쓰기 하나로 보낸다 (서버가 건너뛴 소유 영역 쓰기를 메운다)."""
        for r0, r1, c0, c1, writer in self.my_owners:
            self.write("own", writer, r0, c0, self.local.compose(r0, r1, c0, c1))
            self.stats["refreshed"] += 1

    def set_owner(self, r0, r1, c0, c1, writer):
        self.my_owners.append((int(r0), int(r1), int(c0), int(c1), writer))
        self.local.set_owner(r0, r1, c0, c1, writer)
        self._send({"type": "owner", "writer": writer, "r0": int(r0), "r1": int(r1), "c0": int(c0), "c1": int(c1)})

    # ---- 끝: 서버가 다 받았는지, 사본이 같은지 ----
    def audit(self, wait_s=30.0):
        """서버가 이 클라이언트의 마지막 seq까지 받고(다시 보내기·건너뛰기 포함), 이 클라이언트가 서버가 내보낸 번호 n까지 빈칸 없이 다 받을
        때까지 기다린 뒤 타일 해시를 대조. 판이 아니라 n으로 기다린다: 서버가 이 클라이언트의 변경을 건너뛰었으면 판이 끝내 같아지지
        않는다 (그 차이를 해시 불일치로 드러낸다). 반환: dict (summary에 넣는다)."""
        t0 = time.monotonic()
        while True:
            got = self._request("audit")
            self.poll()
            if (got and got["seq"].get(self.name, 0) >= self.seq and self.next_n > got["n"]
                    and not self.unacked and self.gap is None):
                break
            if time.monotonic() - t0 > wait_s:
                return {"server": "no reply (서버가 끝났거나 없다)" if got is None else
                        f"server got seq {got['seq'].get(self.name, 0)} / {self.seq}, version {got['version']} / {self.local.version}, "
                        f"unacked {len(self.unacked)}, gap {self.gap is not None}"}
        mine = tile_hashes(self.local)
        bad = sorted(k for k in mine if got["hashes"].get(k) != mine[k])
        st = self.stats
        out = {"server_version": got["version"], "client_version": self.local.version, "sent": dict(self.sent, total=self.seq),
               "tiles": len(mine), "hash_mismatch": len(bad), "mismatched_tiles": bad[:20],
               "wait_s": round(time.monotonic() - t0, 3),
               "latency_ms": {"recv": got.get("recv_latency_ms", {"n": 0}), **{k: stats_ms(v) for k, v in self.lat.items()}},
               "gaps": {"recv_side": st["gaps"], "recv_recovery_ms": stats_ms(st["recovery"]), "replayed": st["replayed"],
                        "resyncs": st["resyncs"], "resent": st["resent"], "server_side_recovery_ms": got.get("gap_recovery_ms", {"n": 0}),
                        "server_side_delay_ms": got.get("gap_delay_ms", {"n": 0}),
                        "evicted": st["evicted"], "refreshed": st["refreshed"], "server_lost": got.get("lost", {}),
                        "dropped_test": {"out": st["dropped_out"], "in": st["dropped_in"]}}}
        if self.joined:
            out["joined"] = self.joined
        return out

    def save(self, out):
        """서버에 상태 저장을 요청한다 (out/state.json, out/final/ 바탕 파일 꼴: 다음 서버의 --resume / --base). 반환: 폴더 또는 None."""
        r = self._request("save", timeout=30.0, dir=str(out))
        return None if r is None else r.get("dir")

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
    ap.add_argument("--drop", type=float, default=0.0, help="받은 변경을 일부러 버릴 비율 (빈칸 복구 보기, 예 0.2)")
    a = ap.parse_args()
    if not a.observe:
        ap.error("지금은 --observe만 있다")
    m = RemoteMap(None, name=a.name, drop=(0.0, a.drop))
    print(f"붙음: 판 {m.local.version}, 서버 번호 n {m.joined['n']}, 스냅샷 {m.joined['s']} s, 격자 {m.local.nrow} x {m.local.ncol}, "
          f"소유 영역 {len(m.local.owners)}", flush=True)
    t0, last = time.monotonic(), 0.0
    try:
        while a.duration <= 0 or time.monotonic() - t0 < a.duration:
            m.poll()
            if time.monotonic() - last >= 1.0:
                last = time.monotonic()
                s = stats_ms(m.lat["apply"][-200:])
                g = m.stats
                late = stats_ms(m.lat["apply_late"])
                print(f"판 {m.local.version:5d}  받은 변경 {len(m.lat['apply']):5d}  반영 지연 p50 {s.get('p50', '-')} ms "
                      f"p95 {s.get('p95', '-')} ms  빈칸 {g['gaps']} (버림 {g['dropped_in']}, 다시 받음 {g['replayed']}, "
                      f"빈칸 때문에 늦은 것 max {late.get('max', '-')} ms)", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    au = m.audit(wait_s=10.0)
    print(f"감사: {au}", flush=True)
    m.close()


if __name__ == "__main__":
    main()
