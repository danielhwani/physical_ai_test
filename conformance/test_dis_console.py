"""DIS 시나리오 콘솔 시험 (문서 §10).

    python conformance/test_dis_console.py

1. 봉투: Action Request-R 인코딩·디코딩 왕복 (한글 포함), 시나리오 관리 언어
2. 핸드셰이크: 접속 -> 이벤트 Pending(예정 시각) -> Complete(적용 시각, 지형이면 로봇 기준 위치). 같은 Request ID 재전송은
   다시 넣지 않고 마지막 응답만. ahead(로봇 기준)로 넣은 패치가 로봇 앞 지정 거리에 생긴다.
   접속 전 요청 거부, 지난 시각 거부, 범위 밖 값 거부, 모르는 메시지 UNSUPPORTED, 다른 Exercise ID는 응답 없음
3. 재현성: 콘솔로 넣은 이벤트가 적용 시각과 함께 기록되고, 그 기록(scenario_replay)으로 다시 돌리면
   콘솔 없이도 로봇 궤적이 비트 단위로 같다 (즉흥 투입 이벤트도 재실행 가능한 시나리오가 된다)
시뮬레이터 서버는 이 PC 안(127.0.0.1)의 빈 포트를 쓴다.
"""
import contextlib
import io
import socket
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dis_console import envelope as E  # noqa: E402
from dis_console import protocol as P  # noqa: E402
from dis_console.console import Console, parse_command  # noqa: E402
from sim.dis_server import DisScenarioServer  # noqa: E402
from sim.runner import Simulation  # noqa: E402

SCENARIO = yaml.safe_load((ROOT / "scenarios/dis_console.yaml").read_text())


def _sim():
    sim = Simulation(SCENARIO)
    sim.dis = DisScenarioServer(sim, port=0)
    return sim


def _step(sim, n=1):
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(n):
            sim.step_control()


def _drive(sim, con, req_id, max_steps=500):
    """콘솔 요청의 최종 응답이 올 때까지 시뮬레이션을 진행한다."""
    r = con.reqs[req_id]
    for _ in range(max_steps):
        if r["final"].wait(0.002):
            return r["status"], r["body"]
        _step(sim)
    raise AssertionError(f"최종 응답 없음: {r['type']} {r['status']}")


def _console(sim, **kw):
    return Console(("127.0.0.1", sim.dis.port), out=lambda *_: None, **kw)


def test_envelope_roundtrip():
    pdu = E.ActionRequestR(P.EXERCISE_ID, P.CONSOLE_ENTITY, P.SIM_ENTITY, 1234,
                           P.payload(P.ADD_PATCH, {"patch": {"kind": "rut"}, "메모": "바퀴 자국"}))
    raw = E.encode(pdu)
    back = E.decode(raw)
    assert back.request_id == 1234 and back.payload.lang == E.LANG_SCENARIO
    assert back.payload.type == P.ADD_PATCH and back.payload.body["메모"] == "바퀴 자국"
    assert back.receiving == P.SIM_ENTITY and back.originating == P.CONSOLE_ENTITY


def test_handshake():
    sim = _sim()
    con = _console(sim)
    other = _console(sim, entity=E.EntityId(2, 1, 9))
    try:
        # 접속 전 요청은 거부
        st, body = _drive(sim, other, other.request(P.SET_COMMAND, {"vx": 0.3}))
        assert st == "DENIED" and body["reason_code"] == "NOT_CONNECTED", (st, body)
        st, body = _drive(sim, con, con.request(P.CONNECT))
        assert st == "COMPLETED" and body["scenario"] == "dis_console" and P.SET_COMMAND in body["supported"]
        # 바로 적용: Pending -> Complete, 적용 시각 = 받은 제어 주기
        req = con.request(P.SET_COMMAND, {"vx": 0.3, "yaw_rate": 0.1})
        st, body = _drive(sim, con, req)
        assert st == "COMPLETED" and sim.command == (0.3, 0.1), (st, body, sim.command)
        n_events = len(sim.event_log)
        # 중복 (같은 Request ID, 같은 바이트): 다시 넣지 않고 마지막 응답(Complete)만
        raw = con.reqs[req]["raw"]
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("127.0.0.1", sim.dis.port)); probe.settimeout(1.0)
        probe.send(raw)
        _step(sim)
        resp = E.decode(probe.recv(E.MAX_PDU_SIZE))
        assert resp.request_id == req and resp.request_status == E.STATUS_COMPLETE
        _step(sim, 3)
        assert len(sim.event_log) == n_events and not sim.events
        # 다른 Exercise ID: 응답 없음
        probe.send(E.encode(E.ActionRequestR(99, P.CONSOLE_ENTITY, P.SIM_ENTITY, 7, P.payload(P.CONNECT))))
        _step(sim)
        probe.settimeout(0.3)
        try:
            probe.recv(E.MAX_PDU_SIZE)
            raise AssertionError("다른 Exercise ID에 응답함")
        except socket.timeout:
            pass
        probe.close()
        # 미래 시각 적용: Pending(예정 시각) 뒤 그 시각에 Complete
        t_apply = round(sim.data.time + 0.5, 2)
        kind, b = parse_command(f"rut 3.0 @{t_apply}")
        req = con.request(kind, b)
        st, body = _drive(sim, con, req)
        assert st == "COMPLETED" and abs(body["applied_t"] - t_apply) < sim.decim * sim.model.opt.timestep, body
        assert sim.event_log[-1]["action"] == "add_patch" and sim.event_log[-1]["source"] == "dis"
        # 거부: 지난 시각, 범위 밖, 패치 값 누락, 모르는 메시지
        for type_, b, want, reason in [
                (P.SET_COMMAND, {"vx": 0.2, "t_apply": 0.0}, "DENIED", "T_APPLY_PASSED"),
                (P.SET_COMMAND, {"vx": 5.0}, "FAILED", "INVALID_VALUE"),
                (P.ADD_PATCH, {"patch": {"kind": "bump", "center": [1, 1]}}, "FAILED", "INVALID_VALUE"),
                ("Event_Weather", {"rain": 1}, "UNSUPPORTED", "UNSUPPORTED_MESSAGE")]:
            st, body = _drive(sim, con, con.request(type_, b))
            assert st == want and body["reason_code"] == reason, (type_, st, body)
        assert sim.command == (0.3, 0.1)
        # 로봇 기준 위치: ahead로 넣은 패치는 Complete의 robot_rel이 지정한 앞/옆 거리, 지나간 곳은 음수
        pose = lambda t: sim.dis.robot_pose()          # 시험은 실시간이 아니므로 예측 대신 지금 자세
        for line, want in [("ahead bump 1.0 0.3", (1.0, 0.3)), ("ahead rut 2.0", (2.0, 0.0)), ("ahead ridge 3.0", (3.0, 0.0))]:
            st, body = _drive(sim, con, con.request(*parse_command(line, pose)))
            assert st == "COMPLETED" and np.allclose(body["robot_rel"], want, atol=0.05), (line, body)
        x, y, _ = sim.dis.robot_pose()
        st, body = _drive(sim, con, con.request(*parse_command(f"bump {x - 1.5:.3f} {y:.3f}")))
        assert body["robot_rel"][0] < -1.0 and "지나간 곳" in Console.format_result(st, body), body
    finally:
        con.close(); other.close(); sim.dis.close(); sim.close()


def test_console_events_replay_bit_identical():
    sim = _sim()
    con = _console(sim)
    try:
        _drive(sim, con, con.request(P.CONNECT))
        _step(sim, 20)
        _drive(sim, con, con.request(*parse_command("cmd 0.35")))
        _drive(sim, con, con.request(*parse_command("rut 1.2 0.25 0.05 @3.0")))
        _drive(sim, con, con.request(*parse_command("bump 0.6 0.0 0.04")))
        while sim.data.time < 4.0:
            _step(sim)
        _drive(sim, con, con.request(*parse_command("cmd 0.3 0.3")))
        while sim.data.time < 6.0 - 1e-9:
            _step(sim)
        qpos, t_end = sim.data.qpos.copy(), sim.data.time
        replay = sim.replay_scenario()
    finally:
        con.close(); sim.dis.close(); sim.close()
    dis_events = [e for e in replay["events"] if e.get("source") == "dis"]
    assert [e["action"] for e in dis_events] == ["set_command", "add_patch", "add_patch", "set_command"], dis_events
    replay = yaml.safe_load(yaml.safe_dump(replay, allow_unicode=True, sort_keys=False))   # 파일로 저장했다 읽은 것과 같게
    sim2 = Simulation(replay)
    try:
        while sim2.data.time < t_end - 1e-9:
            _step(sim2)
        assert np.allclose([e["applied_t"] for e in sim2.event_log], [e["t"] for e in replay["events"]], atol=1e-6)
        assert np.array_equal(sim2.data.qpos, qpos), np.abs(sim2.data.qpos - qpos).max()
    finally:
        sim2.close()
    assert qpos[0] > 1.0, f"로봇이 콘솔 명령으로 걷지 않았다: x={qpos[0]:.2f}"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
