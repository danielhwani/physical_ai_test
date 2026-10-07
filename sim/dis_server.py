"""DIS 시나리오 콘솔의 시뮬레이터 쪽 서버 (문서 §10). 규약은 dis_console/protocol.py.

콘솔 요청을 시나리오 이벤트(sim.events)로 바꿔 넣는다. 시나리오 파일의 events와 같은 경로로 적용되므로
콘솔로 넣은 이벤트도 "적용된 시각 + 이벤트"로 기록하면 그대로 다시 실행할 수 있다 (재현성, 문서 §10.1).
  - 스레드 없이 제어 주기마다 poll()로 받는다 (시뮬레이션 루프 안에서 적용 시각이 정해진다)
  - 고정 포트에서 대기하고 요청을 보낸 주소로 응답한다. 기본은 이 PC 안(127.0.0.1)에서만 받는다
  - 승인되지 않은 Exercise ID, 다른 수신자 앞 PDU는 응답 없이 버린다
  - (콘솔 Entity ID, Request ID)로 처리한 요청을 기억해 중복이면 다시 넣지 않고 마지막 응답만 다시 보낸다
  - 받으면 ACCEPTED(Pending, 적용 예정 시각), 적용되면 COMPLETED(Complete, 적용 시각)
  - 기록: <실행 폴더>/dis_events.jsonl (요청과 결과)
"""
import bisect
import json
import socket
import time

import numpy as np

from dis_console import envelope as E
from dis_console import protocol as P

REPORT_DT = 0.2          # 주기 보고 (벽시계 s). 콘솔이 로봇 기준 위치(ahead)를 계산할 수 있게 짧게
RETAIN_S = 60.0          # 끝난 요청 기억 시간 (벽시계 s)


class DisScenarioServer:
    def __init__(self, sim, port=P.DEFAULT_PORT, host="127.0.0.1", exercise_id=P.EXERCISE_ID, log_path=None):
        self.sim, self.exercise_id = sim, exercise_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]
        self.consoles = {}       # EntityId -> 주소 (접속한 콘솔. 주기 보고를 받는다)
        self.handled = {}        # (콘솔, Request ID) -> {addr, resp(bytes), event, done(벽시계)}
        self.log = open(log_path, "a", encoding="utf-8") if log_path else None
        self.next_report = 0.0

    def close(self):
        self.sock.close()
        if self.log:
            self.log.close()

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
        if self.consoles and time.monotonic() >= self.next_report:
            self.next_report = time.monotonic() + REPORT_DT
            self._report()

    # ---- 요청 처리 ----
    def _handle(self, data, addr):
        try:
            pdu = E.decode(data)
        except E.DecodeError as e:
            self._write({"from": f"{addr[0]}:{addr[1]}", "error": f"decode: {e}"})
            return
        if pdu is None or pdu.exercise_id != self.exercise_id or not isinstance(pdu, E.ActionRequestR):
            return                                   # 다른 훈련, 모르는 PDU, 콘솔 heartbeat 등은 무시
        if not pdu.receiving.matches(P.SIM_ENTITY):
            return
        key = (pdu.originating, pdu.request_id)
        if key in self.handled:                      # 중복 (재전송, 상태 재질의): 다시 실행하지 않고 마지막 응답
            h = self.handled[key]
            h["addr"] = addr
            self.sock.sendto(h["resp"], addr)
            return
        h = self.handled[key] = {"addr": addr, "resp": b"", "event": None, "done": None}
        p = pdu.payload
        self._write({"from": str(pdu.originating), "request_id": pdu.request_id, "type": p.type, "body": p.body})
        if p.lang != E.LANG_SCENARIO or p.type not in P.SUPPORTED:
            return self._reject(h, key, "UNSUPPORTED", "UNSUPPORTED_MESSAGE")
        if p.type == P.CONNECT:
            self.consoles[pdu.originating] = addr
            self.next_report = 0.0
            info = {"scenario": self.sim.scn["name"], "t_sim": round(self.sim.data.time, 6),
                    "duration": self.sim.scn["duration"], "mode": "virtual", "supported": P.SUPPORTED}
            return self._respond(h, *key, "COMPLETED", info, final=True)
        if pdu.originating not in self.consoles:
            return self._reject(h, key, "DENIED", "NOT_CONNECTED")
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
        self._respond(h, *key, "ACCEPTED", {"scheduled_t": round(t, 6)})

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
        h["resp"] = E.encode(resp)
        if final:
            h["done"] = time.monotonic()
        self.sock.sendto(h["resp"], h["addr"])
        self._write({"to": str(orig), "request_id": req_id, "result": result, **body})

    def _report(self):
        sim, d = self.sim, self.sim.data
        x, y, yaw = self.robot_pose()
        body = {"t_sim": round(d.time, 3), "duration": sim.scn["duration"],
                "robot": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(float(np.degrees(yaw)), 1)},
                "command": {"vx": sim.command[0], "yaw_rate": sim.command[1]},
                "last_event": sim.last_event[1] if sim.last_event else None}
        for ent, addr in self.consoles.items():
            pdu = E.DataPdu(self.exercise_id, P.SIM_ENTITY, ent, P.payload(P.REPORT, body))
            self.sock.sendto(E.encode(pdu), addr)

    def _write(self, rec):
        if self.log:
            self.log.write(json.dumps({"wall": round(time.time(), 3), "t_sim": round(self.sim.data.time, 6), **rec},
                                      ensure_ascii=False) + "\n")
            self.log.flush()
