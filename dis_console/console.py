"""DIS 시나리오 콘솔 (문서 §10, 가장 기본 구성): 실행 중인 시뮬레이터에 시나리오 이벤트를 넣는다.

    python -m sim.runner scenarios/dis_console.yaml --dis-port 3000 --rviz      # 시뮬레이터 (터미널 1)
    python -m dis_console.console --sim 127.0.0.1:3000                           # 콘솔 (터미널 2)
    python -m dis_console.console --send "cmd 0.3" --send "rut 2.0 @8"           # 명령을 차례로 보내고 끝내기

핸드셰이크 (DIS_test와 같음): 요청마다 Request ID. 1초 안에 응답이 없으면 같은 Request ID, 같은 바이트로 3번까지 재전송.
Pending을 받은 뒤 3초 동안 아무 응답이 없으면 같은 Request ID로 상태 재질의 (시뮬레이터는 다시 실행하지 않고 마지막 응답을 보낸다).
좌표는 시뮬레이터 세계 좌표 (m), 시각은 시뮬레이션 시각 (s).
"""
import argparse
import json
import math
import re
import socket
import sys
import threading
import time

from . import envelope as E
from . import protocol as P

RESPONSE_TIMEOUT, RETRIES = 1.0, 3
POLL_S, FINAL_TIMEOUT = 3.0, 30.0
RESULT_NAMES = {v: k for k, v in E.RESULT_TO_STATUS.items() if k not in ("UNSUPPORTED", "FAILED")}

HELP = """명령 (좌표는 세계 좌표 m: 원점 = 로봇 출발점, +x = 처음 바라본 방향. 끝에 @T를 붙이면 시뮬레이션 시각 T에 적용, 없으면 바로)
  cmd <vx> [yaw_rate]            로봇 이동 명령 (전진 m/s, 회전 rad/s)
  stop                           cmd 0 0
  rut <x> [width] [depth]        x 위치에 가로지르는 바퀴 자국 (기본 폭 0.25, 깊이 0.05, 평평한 바닥)
  ridge <x> [width] [height]     x 위치에 가로지르는 턱 (과속방지턱 모양, 기본 폭 0.4, 높이 0.05)
  bump <x> <y> [height] [radius] 한 점 둔덕 (기본 높이 0.05, 지름 0.4)
  patch <JSON>                   지형 패치 그대로 (시나리오 patches 형식)
  ahead bump <앞> [왼쪽] [height] [radius]   로봇 기준 위치에 둔덕 (적용 시점의 로봇 위치를 예측해 세계 좌표로 보냄)
  ahead rut <앞> [width] [depth]           로봇 정면 <앞> m에 진행 방향을 가로지르는 바퀴 자국
  ahead ridge <앞> [width] [height]        로봇 정면 <앞> m에 진행 방향을 가로지르는 턱
  status                         마지막 주기 보고
  watch                          주기 보고 계속 표시 켜기/끄기
  help / quit"""


class Console:
    def __init__(self, sim_addr, entity=P.CONSOLE_ENTITY, exercise_id=P.EXERCISE_ID, out=print):
        self.entity, self.exercise_id, self.out = entity, exercise_id, out
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect(sim_addr)          # 시뮬레이터에서 온 패킷만 받는다
        self.sock.settimeout(0.1)
        self.next_id = int(time.time() * 10) % 1_000_000_000      # 시각 기반 시작값: 다시 켜도 옛 번호와 잘 안 겹친다
        self.reqs, self.lock = {}, threading.Lock()
        self.report, self.report_rx, self.next_watch = None, None, 0.0
        self.watch, self.running = False, True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.sock.close()

    # ---- 요청 ----
    def request(self, type_, body=None):
        with self.lock:
            req_id, self.next_id = self.next_id, self.next_id + 1
            pdu = E.ActionRequestR(self.exercise_id, self.entity, P.SIM_ENTITY, req_id, P.payload(type_, body))
            raw = E.encode(pdu)
            now = time.monotonic()
            self.reqs[req_id] = {"type": type_, "raw": raw, "sent": now, "tries": 1, "last_rx": None, "created": now,
                                 "status": None, "body": None, "final": threading.Event(), "deadline": now + FINAL_TIMEOUT}
        self.sock.send(raw)
        return req_id

    def wait(self, req_id, timeout=60.0):
        """최종 응답 (결과 이름, body). 포기했으면 (None, 사유)."""
        r = self.reqs[req_id]
        r["final"].wait(timeout)
        return r["status"], r["body"]

    def call(self, type_, body=None, timeout=60.0):
        return self.wait(self.request(type_, body), timeout)

    # ---- 수신, 재전송 ----
    def _loop(self):
        while self.running:
            try:
                self._handle(self.sock.recv(E.MAX_PDU_SIZE))
            except socket.timeout:
                pass
            except ConnectionRefusedError:      # 시뮬레이터가 아직 안 떴거나 끝남 (재전송으로 처리)
                time.sleep(0.1)
            except E.DecodeError:
                pass
            self._timers()

    def _handle(self, data):
        pdu = E.decode(data)
        if pdu is None or pdu.exercise_id != self.exercise_id or not pdu.receiving.matches(self.entity):
            return
        if isinstance(pdu, E.DataPdu):
            if pdu.payload.type == P.REPORT:
                self.report, self.report_rx = pdu.payload.body, time.monotonic()
                if self.watch and self.report_rx >= self.next_watch:      # 보고는 0.2 s마다, 표시는 1 s마다
                    self.next_watch = self.report_rx + 1.0
                    self.out(self.format_report())
            return
        if not isinstance(pdu, E.ActionResponseR):
            return
        with self.lock:
            r = self.reqs.get(pdu.request_id)
            if r is None or r["final"].is_set():
                return                          # 모르는 번호, 이미 끝낸 요청의 늦은 응답
            r["last_rx"] = time.monotonic()
            body = pdu.payload.body
            result = body.get("result", RESULT_NAMES.get(pdu.request_status, str(pdu.request_status)))
            if result == r["status"]:
                return                          # 같은 상태의 중복 응답
            r["status"], r["body"] = result, body
            if result == "ACCEPTED" and "scheduled_t" in body:   # 먼 시각에 적용할 이벤트는 그만큼 더 기다린다
                r["deadline"] = time.monotonic() + FINAL_TIMEOUT + max(0.0, body["scheduled_t"] - body.get("t_sim", 0.0))
            final = pdu.request_status in E.FINAL_STATUSES
        self.out(f"  [{pdu.request_id}] {r['type']}: {self.format_result(result, body)}")
        if final:
            r["final"].set()

    def _timers(self):
        now, resend, give_up = time.monotonic(), [], []
        with self.lock:
            for req_id, r in self.reqs.items():
                if r["final"].is_set():
                    continue
                if r["status"] is None:         # 아직 응답 없음: 재전송, 3번 넘으면 포기
                    if now - r["sent"] > RESPONSE_TIMEOUT:
                        if r["tries"] > RETRIES:
                            give_up.append((req_id, "응답 없음"))
                        else:
                            r["tries"] += 1; r["sent"] = now; resend.append(r["raw"])
                elif now > r["deadline"]:
                    give_up.append((req_id, "최종 응답 시간 초과"))
                elif now - max(r["last_rx"], r["sent"]) > POLL_S:   # Pending 뒤 조용함: 같은 Request ID로 상태 재질의
                    r["sent"] = now; resend.append(r["raw"])
        for raw in resend:
            try:
                self.sock.send(raw)
            except ConnectionRefusedError:
                pass
        for req_id, why in give_up:
            r = self.reqs[req_id]
            r["status"], r["body"] = None, {"reason": why}
            self.out(f"  [{req_id}] {r['type']}: 포기 ({why})")
            r["final"].set()

    def predict_pose(self, t_apply=None):
        """시뮬레이션 시각 t_apply(없으면 지금)의 로봇 위치 예측 (x, y, yaw). 마지막 보고에서 명령 속도로 원호를 따라 간다.
        실시간 실행 가정 (지금 시각 = 보고 시각 + 받은 뒤 지난 벽시계 시간). 실제 속도가 명령보다 느리면 그만큼 앞에 놓인다."""
        r = self.report
        if r is None:
            raise ValueError("상태 보고를 아직 못 받았다 (잠시 뒤 다시)")
        x, y, yaw = r["robot"]["x"], r["robot"]["y"], math.radians(r["robot"]["yaw_deg"])
        v, w = r["command"]["vx"], r["command"]["yaw_rate"]
        t = t_apply if t_apply is not None else r["t_sim"] + (time.monotonic() - self.report_rx)
        dt = max(0.0, t - r["t_sim"])
        if abs(w) < 1e-6:
            return x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw
        return (x + v / w * (math.sin(yaw + w * dt) - math.sin(yaw)), y - v / w * (math.cos(yaw + w * dt) - math.cos(yaw)),
                yaw + w * dt)

    # ---- 표시 ----
    @staticmethod
    def format_result(result, body):
        if result == "ACCEPTED":
            return f"접수, t={body['scheduled_t']:.2f}에 적용 예정 (지금 t={body['t_sim']:.2f})"
        if result == "COMPLETED":
            if "applied_t" in body:
                text = f"적용 완료 t={body['applied_t']:.2f}"
                if "robot_rel" in body:
                    fwd, left = body["robot_rel"]
                    side = f"왼쪽 {left:.2f} m" if left >= 0 else f"오른쪽 {-left:.2f} m"
                    text += f" (로봇 기준 {'앞' if fwd >= 0 else '뒤'} {abs(fwd):.2f} m, {side})"
                    if fwd < -0.3:
                        text += "  ※ 로봇이 이미 지나간 곳"
                return text
            if "scenario" in body:
                return (f"접속 완료: 시나리오 {body['scenario']}, t={body['t_sim']:.2f}/{body['duration']} s, "
                        f"지원 {', '.join(body['supported'])}")
        return f"{result} {body.get('reason_code', '')} {body.get('detail', '')}".rstrip()

    def format_report(self):
        r = self.report
        if r is None:
            return "  주기 보고 없음 (접속 전이거나 시뮬레이터가 멈춤)"
        rb, c = r["robot"], r["command"]
        return (f"  t={r['t_sim']:.1f}/{r['duration']} s  로봇 x={rb['x']:.2f} y={rb['y']:.2f} 방향 {rb['yaw_deg']:.0f}°  "
                f"명령 vx={c['vx']:.2f} yaw={c['yaw_rate']:.2f}  마지막 이벤트: {r['last_event']}")


def parse_command(line, pose=None):
    """콘솔 한 줄 -> (메시지 종류, body). 표시만 하는 명령이면 (이름, None).
    pose: ahead 명령용 로봇 위치 예측 함수 (t_apply -> x, y, yaw). Console.predict_pose"""
    line, t_apply = line.strip(), None
    m = re.search(r"\s@(\S+)$", line)
    if m:
        t_apply, line = float(m.group(1)), line[:m.start()].rstrip()
    if not line:
        return None, None
    op, _, rest = line.partition(" ")
    a = rest.split()
    if op in ("status", "watch", "help", "quit", "exit"):
        return op, None
    if op == "cmd":
        body = {"vx": float(a[0]), "yaw_rate": float(a[1]) if len(a) > 1 else 0.0}
        kind = P.SET_COMMAND
    elif op == "stop":
        body, kind = {"vx": 0.0, "yaw_rate": 0.0}, P.SET_COMMAND
    elif op == "rut":
        x = float(a[0])
        body = {"patch": {"kind": "rut", "start": [x, -50.0], "end": [x, 50.0], "width": float(a[1]) if len(a) > 1 else 0.25,
                          "depth": float(a[2]) if len(a) > 2 else 0.05, "profile": "box"}}
        kind = P.ADD_PATCH
    elif op == "ridge":                             # 솟은 턱 = 깊이가 음수인 완만한(cosine) 홈
        x = float(a[0])
        body = {"patch": {"kind": "rut", "start": [x, -50.0], "end": [x, 50.0], "width": float(a[1]) if len(a) > 1 else 0.4,
                          "depth": -(float(a[2]) if len(a) > 2 else 0.05), "profile": "cosine"}}
        kind = P.ADD_PATCH
    elif op == "bump":
        body = {"patch": {"kind": "bump", "center": [float(a[0]), float(a[1])], "height": float(a[2]) if len(a) > 2 else 0.05,
                          "radius": float(a[3]) if len(a) > 3 else 0.4}}
        kind = P.ADD_PATCH
    elif op == "patch":
        body, kind = {"patch": json.loads(rest)}, P.ADD_PATCH
    elif op == "ahead":
        if pose is None:
            raise ValueError("ahead는 콘솔 접속 후에만 쓸 수 있다")
        sub, fwd = a[0], float(a[1])
        x, y, yaw = pose(t_apply)
        c, s_ = math.cos(yaw), math.sin(yaw)
        if sub == "bump":
            left = float(a[2]) if len(a) > 2 else 0.0
            body = {"patch": {"kind": "bump", "center": [round(x + fwd * c - left * s_, 3), round(y + fwd * s_ + left * c, 3)],
                              "height": float(a[3]) if len(a) > 3 else 0.05, "radius": float(a[4]) if len(a) > 4 else 0.4}}
        elif sub in ("rut", "ridge"):               # 진행 방향에 수직인 선 (양쪽 50 m). ridge는 깊이가 음수인 완만한 홈
            px, py = x + fwd * c, y + fwd * s_
            ridge = sub == "ridge"
            body = {"patch": {"kind": "rut", "start": [round(px + 50 * s_, 3), round(py - 50 * c, 3)],
                              "end": [round(px - 50 * s_, 3), round(py + 50 * c, 3)],
                              "width": float(a[2]) if len(a) > 2 else (0.4 if ridge else 0.25),
                              "depth": (-1 if ridge else 1) * (float(a[3]) if len(a) > 3 else 0.05),
                              "profile": "cosine" if ridge else "box"}}
        else:
            raise ValueError("ahead 다음은 bump, rut, ridge")
        kind = P.ADD_PATCH
    else:
        raise ValueError(f"모르는 명령: {op} (help)")
    if t_apply is not None:
        body["t_apply"] = t_apply
    return kind, body


def main():
    ap = argparse.ArgumentParser(description="DIS 시나리오 콘솔")
    ap.add_argument("--sim", default=f"127.0.0.1:{P.DEFAULT_PORT}", help="시뮬레이터 주소 host:port")
    ap.add_argument("--entity", default=str(P.CONSOLE_ENTITY), help="콘솔 Entity ID site/app/entity")
    ap.add_argument("--exercise", type=int, default=P.EXERCISE_ID)
    ap.add_argument("--send", action="append", metavar="CMD", help="이 명령들을 차례로 보내고(각각 완료까지) 끝낸다")
    args = ap.parse_args()
    host, port = args.sim.rsplit(":", 1)
    con = Console((host, int(port)), E.EntityId.parse(args.entity), args.exercise)
    try:
        print(f"시뮬레이터 {args.sim}에 접속...")
        status, _ = con.call(P.CONNECT)
        if status != "COMPLETED":
            sys.exit(1)
        lines = args.send
        if lines is None:
            print(HELP)
            try:
                import readline  # noqa: F401  (명령 이력, 화살표 키)
            except ImportError:
                pass
        while True:
            if lines is None:
                try:
                    line = input("dis> ")
                except EOFError:
                    break
            elif lines:
                line = lines.pop(0)
                print(f"dis> {line}")
            else:
                break
            try:
                kind, body = parse_command(line, con.predict_pose)
            except (ValueError, IndexError, json.JSONDecodeError) as e:
                print(f"  {e}")
                continue
            if kind is None:
                continue
            if kind in ("quit", "exit"):
                break
            if kind == "help":
                print(HELP)
            elif kind == "status":
                print(con.format_report())
            elif kind == "watch":
                con.watch = not con.watch
                print(f"  주기 보고 표시 {'켬' if con.watch else '끔'}")
            elif lines is not None:
                con.call(kind, body)               # 스크립트: 완료까지 기다린다
            else:
                con.request(kind, body)            # 대화형: 결과는 받는 대로 표시
    except KeyboardInterrupt:
        print()
    finally:
        con.close()


if __name__ == "__main__":
    main()
