"""DIS 시나리오 콘솔의 시뮬레이터 쪽 서버 (문서 §10). 규약은 dis_console/protocol.py.

콘솔 요청을 시나리오 이벤트(sim.events)로 바꿔 넣는다. 시나리오 파일의 events와 같은 경로로 적용되므로
콘솔로 넣은 이벤트도 "적용된 시각 + 이벤트"로 기록하면 그대로 다시 실행할 수 있다 (재현성, 문서 §10.1).
  - 스레드 없이 제어 주기마다 poll()로 받는다 (시뮬레이션 루프 안에서 적용 시각이 정해진다)
  - 고정 포트에서 대기하고 요청을 보낸 주소로 응답한다. 기본은 이 PC 안(127.0.0.1)에서만 받는다
  - 승인되지 않은 Exercise ID, 다른 수신자 앞 PDU는 응답 없이 버린다
  - (콘솔 Entity ID, Request ID)로 처리한 요청을 기억해 중복이면 다시 넣지 않고 마지막 응답만 다시 보낸다
  - 받으면 ACCEPTED(Pending, 적용 예정 시각), 적용되면 COMPLETED(Complete, 적용 시각)
  - 고장 주입·해제, 시험 종료, 일시정지는 이벤트로 넣는다 (일시정지·종료는 다시 실행할 시나리오에서 빠진다). 재개는 바로 처리한다
  - 제어권: 명령할 수 있는 콘솔은 하나 (Request_Connection role=control). 다른 콘솔은 관찰(role=observe, 주기 보고만)하거나
    거부된다 (CONTROL_BUSY). Request_ReleaseControl로 내놓는다
  - 링크 감시: 제어권을 가진 콘솔에게서 comm_lost_s 동안 아무것도 오지 않으면 통신 두절. 단절 시 동작 STOP이면
    로봇 이동 명령 0을 이벤트로 넣는다 (source: dis_comm_lost, 기록되므로 다시 실행해도 같다). 제어권은 풀린다 (다른 콘솔이 받을 수 있게).
    그 콘솔에게서 다시 받으면 링크는 OK로 되돌리지만 제어권은 다시 접속해야 받는다
  - 인증 (key가 있으면): 모든 PDU에 HMAC 서명을 붙이고, 받은 PDU는 서명과 카운터(재전송 공격)를 확인해 아니면 응답 없이 버린다
    (dis_console/auth.py)
  - 기록: <실행 폴더>/dis_events.jsonl (요청과 결과, 링크 상태 변화), dis_pdus.jsonl·dis_pdus.pcap (원본 PDU, DIS_test 형식)
"""
import bisect
import json
import socket
import time

import numpy as np

from dis_console import auth
from dis_console import envelope as E
from dis_console import protocol as P
from dis_console.pdulog import PduRecorder

REPORT_DT = 0.2          # 주기 보고 (벽시계 s). 콘솔이 로봇 기준 위치(ahead)를 계산할 수 있게 짧게
RETAIN_S = 60.0          # 끝난 요청 기억 시간 (벽시계 s)


class DisScenarioServer:
    def __init__(self, sim, port=P.DEFAULT_PORT, host="127.0.0.1", exercise_id=P.EXERCISE_ID, log_path=None,
                 comm_lost_s=P.COMM_LOST_S, comm_lost_behavior="STOP", key=None, pdu_dir=None):
        """key: 인증 공유 키 (bytes, 없으면 인증 안 함). pdu_dir: 원본 PDU 기록 폴더."""
        assert comm_lost_behavior in ("STOP", "CONTINUE")
        self.signer = auth.Signer(key) if key else None
        self.verifier = auth.Verifier(key) if key else None
        self.owner, self.lost_owner = None, None     # 제어권을 가진 콘솔, 통신 두절로 제어권을 잃은 콘솔
        self.auth_drops = 0
        self.sim, self.exercise_id = sim, exercise_id
        self.comm_lost_s, self.comm_lost_behavior = comm_lost_s, comm_lost_behavior
        self.last_rx = {}        # 콘솔 -> 마지막으로 무엇이든 받은 벽시계 시각
        self.timed = {}          # (콘솔, Request ID) -> 출발 시각을 기다리는 투입 {gap, speed}
        self.comm_state = "OK"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]
        self.pdus = PduRecorder(pdu_dir, (host, self.port)) if pdu_dir else None
        self.consoles = {}       # EntityId -> 주소 (접속한 콘솔. 주기 보고를 받는다)
        self.handled = {}        # (콘솔, Request ID) -> {addr, resp(bytes), event, done(벽시계)}
        self.log = open(log_path, "a", encoding="utf-8") if log_path else None
        self.next_report = 0.0

    def close(self):
        self.sock.close()
        if self.log:
            self.log.close()
        if self.pdus:
            self.pdus.close()

    def _send(self, raw, addr):
        """서명(키가 있으면)하고 보낸다. 보낸 바이트를 돌려준다 (그대로 다시 보낼 수 있게)."""
        if self.signer:
            raw = self.signer.sign(raw)
        self._sendraw(raw, addr)
        return raw

    def _sendraw(self, raw, addr):
        self.sock.sendto(raw, addr)
        if self.pdus:
            self.pdus.record(False, addr, raw)

    # ---- 시뮬레이션 루프에서 부른다 ----
    def poll(self):
        """제어 주기 시작: 받은 요청 처리 (이벤트 예약)."""
        while True:
            try:
                data, addr = self.sock.recvfrom(E.MAX_PDU_SIZE)
            except (BlockingIOError, InterruptedError):
                break
            self._handle(data, addr)
        now = time.monotonic()
        for k in [k for k, h in self.handled.items() if h["done"] and now - h["done"] > RETAIN_S]:
            del self.handled[k]

    def after_events(self):
        """이벤트 적용 직후: 적용된 요청에 Complete 응답. 주기 보고."""
        for (orig, req_id), h in self.handled.items():
            e = h["event"]
            if e is not None and not h["done"] and "applied_t" in e:
                body = {"applied_t": round(e["applied_t"], 6)}
                if e["action"] == "add_patch":           # 적용 순간 로봇 기준 위치 (앞 +, 왼쪽 +). 로봇이 이미 지나간 곳인지 콘솔이 알린다
                    body["robot_rel"] = self.robot_relative(e["patch"])
                self._respond(h, orig, req_id, "COMPLETED", body, final=True)
        self._check_timed()
        self._check_link()
        if self.consoles and time.monotonic() >= self.next_report:
            self.next_report = time.monotonic() + REPORT_DT
            self._report()

    def _check_link(self):
        now = time.monotonic()
        for c in [c for c in self.consoles if c != self.owner and now - self.last_rx.get(c, 0.0) > 3 * self.comm_lost_s]:
            del self.consoles[c]                      # 오래 조용한 관찰 콘솔에는 주기 보고를 그만 보낸다
        if self.comm_state == "OK" and self.owner is not None and now - self.last_rx.get(self.owner, 0.0) > self.comm_lost_s:
            silent = now - self.last_rx.get(self.owner, 0.0)
            self.comm_state = "LOST"
            self.lost_owner, self.owner = self.owner, None
            sim = self.sim
            stop = self.comm_lost_behavior == "STOP"
            print(f"[t={sim.data.time:6.2f}] DIS 통신 두절 (제어 콘솔 {self.lost_owner} {silent:.1f} s 무응답) -> "
                  f"단절 시 동작 {self.comm_lost_behavior}, 제어권 해제")
            self._write({"comm_state": "LOST", "silent_s": round(silent, 2), "behavior": self.comm_lost_behavior,
                         "released": str(self.lost_owner)})
            if stop and sim.command != (0.0, 0.0):          # 로봇을 세운다: 다른 이벤트처럼 넣어 기록·재현되게
                event = {"action": "set_command", "vx": 0.0, "yaw_rate": 0.0, "t": sim.data.time, "source": "dis_comm_lost"}
                sim.events.insert(bisect.bisect_right([e["t"] for e in sim.events], event["t"]), event)
        elif self.comm_state == "LOST" and (self.owner is not None or now - self.last_rx.get(self.lost_owner, 0.0) <= self.comm_lost_s):
            self.comm_state = "OK"
            print(f"[t={self.sim.data.time:6.2f}] DIS 링크 복구 (명령은 자동으로 되돌리지 않는다. 제어권은 다시 접속해야 받는다)")
            self._write({"comm_state": "OK"})

    # ---- 요청 처리 ----
    def _handle(self, data, addr):
        if self.verifier:                             # 인증: 서명·카운터가 맞지 않으면 응답 없이 버린다
            try:
                unsigned = self.verifier.verify(data)
            except (auth.AuthError, E.DecodeError) as e:
                self.auth_drops += 1
                if self.pdus:
                    self.pdus.record(True, addr, data, dropped="auth", reason=str(e))
                self._write({"from": f"{addr[0]}:{addr[1]}", "dropped": "auth", "reason": str(e)})
                if self.auth_drops in (1, 10, 100):
                    print(f"[t={self.sim.data.time:6.2f}] DIS 인증 실패로 버림 ({self.auth_drops}번째): {e}")
                return
        else:
            unsigned = data
        if self.pdus:
            self.pdus.record(True, addr, data)
        data = unsigned
        try:
            pdu = E.decode(data)
        except E.DecodeError as e:
            self._write({"from": f"{addr[0]}:{addr[1]}", "error": f"decode: {e}"})
            return
        if pdu is None or pdu.exercise_id != self.exercise_id or not pdu.receiving.matches(P.SIM_ENTITY):
            return                                   # 다른 훈련, 모르는 PDU, 다른 수신자 앞은 무시
        if pdu.originating in self.consoles:         # 접속한 콘솔에서 온 것은 무엇이든 생존 신호 (heartbeat, 요청, 재전송)
            self.last_rx[pdu.originating] = time.monotonic()
        if not isinstance(pdu, E.ActionRequestR):
            return
        key = (pdu.originating, pdu.request_id)
        if key in self.handled:                      # 중복 (재전송, 상태 재질의): 다시 실행하지 않고 마지막 응답
            h = self.handled[key]
            h["addr"] = addr
            self._sendraw(h["resp"], addr)                # 서명까지 같은 바이트 (콘솔이 재전송으로 알아본다)
            return
        h = self.handled[key] = {"addr": addr, "resp": b"", "event": None, "done": None}
        p = pdu.payload
        self._write({"from": str(pdu.originating), "request_id": pdu.request_id, "type": p.type, "body": p.body})
        if p.lang != E.LANG_SCENARIO or p.type not in P.SUPPORTED:
            return self._reject(h, key, "UNSUPPORTED", "UNSUPPORTED_MESSAGE")
        orig = pdu.originating
        if p.type == P.CONNECT:
            role = p.body.get("role", "control")
            if role not in ("control", "observe"):
                return self._reject(h, key, "FAILED", "INVALID_VALUE", "role: control 또는 observe")
            if role == "control" and self.owner not in (None, orig):
                return self._reject(h, key, "DENIED", "CONTROL_BUSY", f"제어권은 {self.owner}에게 있다 (관찰만 하려면 role=observe)")
            if role == "control":
                self.owner = orig
            elif self.owner == orig:
                self.owner = None                     # 제어 콘솔이 관찰로 다시 접속: 제어권을 내놓는다
            self.consoles[orig] = addr
            self.last_rx[orig] = time.monotonic()
            self.next_report = 0.0
            info = {"scenario": self.sim.scn["name"], "t_sim": round(self.sim.data.time, 6),
                    "duration": self.sim.scn["duration"], "mode": "virtual", "supported": P.SUPPORTED,
                    "comm_lost_s": self.comm_lost_s, "comm_lost_behavior": self.comm_lost_behavior,
                    "role": role, "control_owner": None if self.owner is None else str(self.owner),
                    "auth": self.verifier is not None}
            return self._respond(h, *key, "COMPLETED", info, final=True)
        if orig not in self.consoles:
            return self._reject(h, key, "DENIED", "NOT_CONNECTED")
        if p.type == P.RELEASE_CONTROL:
            if orig != self.owner:
                return self._reject(h, key, "DENIED", "NOT_IN_CONTROL")
            self.owner = None
            return self._respond(h, *key, "COMPLETED", {"role": "observe", "control_owner": None}, final=True)
        if orig != self.owner:                        # 관찰 콘솔은 명령할 수 없다
            return self._reject(h, key, "DENIED", "NOT_IN_CONTROL",
                                f"제어권은 {self.owner}에게 있다" if self.owner else "제어권이 비어 있다 (control로 다시 접속)")
        if p.type == P.RESUME:                       # 재개는 바로 (일시정지 중에는 시뮬레이션 시각이 멈춰 이벤트로 넣을 수 없다)
            if not self.sim.frozen:
                return self._reject(h, key, "DENIED", "NOT_FROZEN")
            self.sim.frozen = False
            self.sim.last_event = (self.sim.data.time, "resume")
            return self._respond(h, *key, "COMPLETED", {"applied_t": round(self.sim.data.time, 6)}, final=True)
        if p.type == P.CLEAR_FAULT and p.body.get("fault") not in self.sim.faults and not any(
                e["action"] == "inject_fault" and e["fault"] == p.body.get("fault") for e in self.sim.events):
            return self._reject(h, key, "DENIED", "FAULT_NOT_ACTIVE")
        cosim = self.sim.cosim
        if p.type in (P.CREATE_ENTITY, P.REMOVE_ENTITY, P.SET_SOIL) and cosim is None:
            return self._reject(h, key, "DENIED", "NO_CHRONO", "개체·흙은 Chrono 시나리오에서만 (예: scenarios/dis_chrono.yaml)")
        if p.type in (P.CREATE_ENTITY, P.REMOVE_ENTITY):
            if not cosim.has_vehicle:
                return self._reject(h, key, "DENIED", "NO_ENTITY_SLOT", "시나리오 chrono.vehicle이 없다")
            if p.type == P.CREATE_ENTITY and cosim.vehicle_visible:
                return self._reject(h, key, "DENIED", "ENTITY_ACTIVE")
            if p.type == P.REMOVE_ENTITY and not cosim.vehicle_visible:
                return self._reject(h, key, "DENIED", "ENTITY_NOT_ACTIVE")
        if p.type == P.CREATE_ENTITY and p.body.get("cross_ahead") is not None:
            return self._accept_timed(h, key, p.body, pdu)
        try:
            event = self._to_event(p)
        except (KeyError, TypeError, ValueError) as e:
            return self._reject(h, key, "FAILED", "INVALID_VALUE", str(e))
        now = self.sim.data.time
        t = now if p.body.get("t_apply") is None else float(p.body["t_apply"])
        if t < now - 1e-9:
            return self._reject(h, key, "DENIED", "T_APPLY_PASSED", f"t_apply {t:.3f} < t_sim {now:.3f}")
        event.update(t=t, source="dis", request=f"{pdu.originating}#{pdu.request_id}")
        events = self.sim.events                     # 같은 시각이면 먼저 있던 이벤트 뒤에
        events.insert(bisect.bisect_right([e["t"] for e in events], t), event)
        h["event"] = event
        self._respond(h, *key, "ACCEPTED", {"scheduled_t": round(t, 6), **({"frozen": True} if self.sim.frozen else {})})

    # ---- 개체 투입 시각 맞추기 (cross_ahead): 차가 로봇 진행선의 로봇 앞 gap m 지점을 로봇보다 먼저 지나가게 ----
    LEAD = 0.3            # 차 꼬리가 지나간 뒤 로봇이 그 지점에 오기까지 여유 (s)
    CLEAR = 3.0           # 교차점을 완전히 지나기까지 더 갈 거리 = HMMWV 반 길이 2.4 m + 0.6 m

    def _crossing(self, gap, speed):
        """(로봇이 gap 지점에 오기까지 남은 시간 - 차가 교차점을 다 지나는 데 걸리는 시간, 설명). 교차가 없으면 (None, 이유)."""
        sim = self.sim
        v = sim.cosim.cfg["vehicle"]
        s, e = np.asarray(v["start"], float), np.asarray(v["end"], float)
        u = (e - s) / np.linalg.norm(e - s)
        x, y, yaw = self.robot_pose()
        hd = np.array([np.cos(yaw), np.sin(yaw)])
        A = np.column_stack([hd, -u])
        if abs(np.linalg.det(A)) < 0.2:
            return None, "로봇 진행 방향이 차량 경로와 거의 나란하다"
        a, b = np.linalg.solve(A, s - [x, y])          # 로봇 -> 교차점 거리 a, 차량 출발점 -> 교차점 거리 b
        if a < 0 or not 0 <= b <= np.linalg.norm(e - s):
            return None, "로봇 진행선이 차량 경로와 앞에서 만나지 않는다"
        vel = speed or v["speed"]
        t_vehicle = (b + self.CLEAR) / (0.95 * vel) + 0.15 + 0.08 * vel     # 이 PC 측정: 목표 속도의 95%, 출발 지연
        v_robot = max(sim.command[0], 0.0)
        if v_robot < 0.05:                               # 로봇이 서 있음: gap보다 멀면 언제든 앞을 지난다
            return (np.inf if a > gap else -np.inf), f"로봇 정지, 교차점까지 {a:.1f} m"
        return (a - gap) / v_robot - t_vehicle, f"교차점까지 {a:.1f} m, 차량 {t_vehicle:.1f} s"

    def _accept_timed(self, h, key, body, pdu):
        cosim = self.sim.cosim
        gap = float(body["cross_ahead"])
        speed = None if body.get("speed") is None else float(body["speed"])
        if not 0.5 <= gap <= 10.0 or (speed is not None and not 0 < speed <= 15.0):
            return self._reject(h, key, "FAILED", "INVALID_VALUE", "cross_ahead 0.5~10 m, speed 0~15 m/s")
        slack, why = self._crossing(gap, speed)
        if slack is None:
            return self._reject(h, key, "DENIED", "NO_CROSSING", why)
        if slack < 0:
            return self._reject(h, key, "DENIED", "TOO_LATE", f"로봇이 너무 가깝다 ({why})")
        self.timed[key] = {"gap": gap, "speed": speed, "request": f"{pdu.originating}#{pdu.request_id}"}
        wait = 0.0 if np.isinf(slack) else max(0.0, slack - self.LEAD)
        self._respond(h, *key, "ACCEPTED", {"scheduled_t": round(self.sim.data.time + wait, 6), "timing": "cross_ahead",
                                           "detail": why})

    def _check_timed(self):
        for key, w in list(self.timed.items()):
            h = self.handled[key]
            slack, why = self._crossing(w["gap"], w["speed"])
            if slack is None or slack < 0:              # 기다리는 동안 로봇이 방향을 바꾸거나 너무 가까워짐
                del self.timed[key]
                self._reject(h, key, "FAILED", "TOO_LATE" if slack is not None else "NO_CROSSING", why)
            elif slack <= self.LEAD or np.isinf(slack):  # 지금 출발하면 꼬리가 LEAD초 여유로 로봇 앞을 지나간다
                del self.timed[key]
                event = {"action": "create_entity", "entity_type": "HMMWV", "speed": w["speed"], "t": self.sim.data.time,
                         "source": "dis", "request": w["request"]}
                self.sim.events.insert(bisect.bisect_right([e["t"] for e in self.sim.events], event["t"]), event)
                h["event"] = event

    def announce_end(self, info):
        """시험이 끝났음을 접속한 콘솔에 알린다 (마지막 주기 보고, ended). 콘솔은 이후 링크 감시를 멈춘다."""
        self.sim.last_event = (self.sim.data.time, f"end {info['reason']}")
        body = {"t_sim": round(self.sim.data.time, 3), "duration": self.sim.scn["duration"], "ended": info}
        for ent, addr in self.consoles.items():
            for _ in range(3):                        # 확인 없는 Data PDU라 몇 번 보낸다 (콘솔은 처음 것만 쓴다)
                self._send(E.encode(E.DataPdu(self.exercise_id, P.SIM_ENTITY, ent, P.payload(P.REPORT, body))), addr)

    def robot_pose(self):
        """로봇 몸통 위치와 방향 (시험 판정자 쪽 참값): x, y, yaw(rad)."""
        d = self.sim.data
        w, x, y, z = d.qpos[3:7]
        return float(d.qpos[0]), float(d.qpos[1]), float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    def robot_relative(self, patch):
        """패치 기준점의 로봇 좌표 [앞, 왼쪽] (m). 둔덕: 중심, 홈: 선분 위 가장 가까운 점, 영역: 가운데."""
        x, y, yaw = self.robot_pose()
        kind = patch["kind"]
        if kind == "bump":
            q = np.asarray(patch["center"], float)
        elif kind == "rut":
            a, b = np.asarray(patch["start"], float), np.asarray(patch["end"], float)
            u = np.clip(np.dot([x, y] - a, b - a) / np.dot(b - a, b - a), 0, 1)
            q = a + u * (b - a)
        else:
            x0, y0, x1, y1 = patch["region"]
            q = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
        dx, dy = q - [x, y]
        c, s_ = np.cos(yaw), np.sin(yaw)
        return [round(float(c * dx + s_ * dy), 3), round(float(-s_ * dx + c * dy), 3)]

    @staticmethod
    def _to_event(p):
        b = p.body
        if p.type in (P.CREATE_ENTITY, P.REMOVE_ENTITY):
            etype = str(b.get("entity_type", "HMMWV")).upper()
            if etype not in P.ENTITY_TYPES:
                raise ValueError(f"모르는 개체: {etype} (지원: {', '.join(P.ENTITY_TYPES)})")
            if p.type == P.REMOVE_ENTITY:
                return {"action": "remove_entity", "entity_type": etype}
            speed = b.get("speed")
            if speed is not None and not 0 < float(speed) <= 15.0:
                raise ValueError("speed는 0 < v <= 15 m/s")
            return {"action": "create_entity", "entity_type": etype, "speed": None if speed is None else float(speed)}
        if p.type == P.SET_SOIL:
            if "preset" in b:
                if b["preset"] not in P.SOIL_PRESETS:
                    raise ValueError(f"preset: {', '.join(P.SOIL_PRESETS)}")
                return {"action": "set_soil", "preset": b["preset"], "soil": dict(P.SOIL_PRESETS[b["preset"]])}
            soil = {k: float(v) for k, v in b["soil"].items()}
            missing = [k for k in P.SOIL_KEYS if k not in soil]
            if missing:
                raise ValueError(f"soil에 없는 값: {missing}")
            if soil["bekker_kphi"] <= 0:
                raise ValueError("bekker_kphi > 0")
            return {"action": "set_soil", "soil": soil}
        if p.type in (P.FREEZE, P.STOP):
            return {"action": "freeze" if p.type == P.FREEZE else "stop"}
        if p.type in (P.INJECT_FAULT, P.CLEAR_FAULT):
            fault = b["fault"]
            if fault not in P.FAULTS:
                raise ValueError(f"모르는 고장: {fault} (지원: {', '.join(P.FAULTS)})")
            if p.type == P.CLEAR_FAULT:
                return {"action": "clear_fault", "fault": fault}
            dur = b.get("duration")
            if dur is not None and not 0 < float(dur) <= P.MAX_FAULT_S:
                raise ValueError(f"duration은 0 < d <= {P.MAX_FAULT_S:g} s")
            params = dict(b.get("params") or {})
            if fault == "link_loss" and params.get("robot_behavior", "hold_last") not in ("hold_last", "damp"):
                raise ValueError("robot_behavior: hold_last 또는 damp")
            if fault == "imu_bias":
                g = [float(v) for v in params.get("gyro_bias", (0, 0, 0))]
                a = [float(v) for v in params.get("attitude_offset_deg", (0, 0, 0))]
                if len(g) != 3 or len(a) != 3 or max(map(abs, g)) > 1.0 or max(map(abs, a)) > 30.0:
                    raise ValueError("gyro_bias 3개 (|값| <= 1 rad/s), attitude_offset_deg 3개 (|값| <= 30도)")
                params = {"gyro_bias": g, "attitude_offset_deg": a}
            if fault == "battery_low":
                params["torque_scale"] = float(params.get("torque_scale", 0.6))
                if not 0.1 <= params["torque_scale"] <= 1.0:
                    raise ValueError("torque_scale은 0.1 ~ 1.0")
            return {"action": "inject_fault", "fault": fault, "duration": None if dur is None else float(dur), "params": params}
        if p.type == P.SET_COMMAND:
            vx, wz = float(b["vx"]), float(b.get("yaw_rate", 0.0))
            if not (abs(vx) <= P.MAX_VX and abs(wz) <= P.MAX_YAW_RATE):
                raise ValueError(f"범위 밖: |vx| <= {P.MAX_VX}, |yaw_rate| <= {P.MAX_YAW_RATE}")
            return {"action": "set_command", "vx": vx, "yaw_rate": wz}
        patch = dict(b["patch"])
        missing = [k for k in P.PATCH_KEYS[patch["kind"]] if k not in patch]
        if missing:
            raise ValueError(f"{patch['kind']} 패치에 없는 값: {missing}")
        if patch["kind"] == "rut" and np.allclose(patch["start"], patch["end"]):
            raise ValueError("rut start == end")
        return {"action": "add_patch", "patch": patch}

    def _reject(self, h, key, result, reason, detail=None):
        body = {"reason_code": reason, **({"detail": detail} if detail else {})}
        self._respond(h, *key, result, body, final=True)

    def _respond(self, h, orig, req_id, result, body, final=False):
        resp = E.ActionResponseR(self.exercise_id, P.SIM_ENTITY, orig, req_id, E.RESULT_TO_STATUS[result],
                                 P.payload("Response_Result", {"result": result, "t_sim": round(self.sim.data.time, 6), **body}))
        if final:
            h["done"] = time.monotonic()
        h["resp"] = self._send(E.encode(resp), h["addr"])
        self._write({"to": str(orig), "request_id": req_id, "result": result, **body})

    def _report(self):
        sim, d = self.sim, self.sim.data
        x, y, yaw = self.robot_pose()
        body = {"t_sim": round(d.time, 3), "duration": sim.scn["duration"],
                "robot": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(float(np.degrees(yaw)), 1)},
                "command": {"vx": sim.command[0], "yaw_rate": sim.command[1]},
                "last_event": sim.last_event[1] if sim.last_event else None,
                "faults": {n: (None if f["until"] is None else round(f["until"] - d.time, 2)) for n, f in sim.faults.items()},
                "frozen": sim.frozen, "comm_state": self.comm_state,
                "control_owner": None if self.owner is None else str(self.owner),
                "vehicles": [] if sim.cosim is None or not sim.cosim.vehicle_visible else [
                    {"type": "HMMWV", "x": round(sim.cosim.vehicle["pos"][0], 2), "y": round(sim.cosim.vehicle["pos"][1], 2),
                     "speed": round(sim.cosim.vehicle["speed"], 2)}]}
        for ent, addr in self.consoles.items():
            pdu = E.DataPdu(self.exercise_id, P.SIM_ENTITY, ent, P.payload(P.REPORT, body))
            self._send(E.encode(pdu), addr)

    def _write(self, rec):
        if self.log:
            self.log.write(json.dumps({"wall": round(time.time(), 3), "t_sim": round(self.sim.data.time, 6), **rec},
                                      ensure_ascii=False) + "\n")
            self.log.flush()
