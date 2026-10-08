"""DIS 시나리오 콘솔 시험 (문서 §10).

    python conformance/test_dis_console.py

1. 봉투: Action Request-R 인코딩·디코딩 왕복 (한글 포함), 시나리오 관리 언어
2. 핸드셰이크: 접속 -> 이벤트 Pending(예정 시각) -> Complete(적용 시각, 지형이면 로봇 기준 위치). 같은 Request ID 재전송은
   다시 넣지 않고 마지막 응답만. ahead(로봇 기준)로 넣은 패치가 로봇 앞 지정 거리에 생긴다.
   접속 전 요청 거부, 지난 시각 거부, 범위 밖 값 거부, 모르는 메시지 UNSUPPORTED, 다른 Exercise ID는 응답 없음
3. 재현성: 콘솔로 넣은 이벤트(고장 포함)가 적용 시각과 함께 기록되고, 그 기록(scenario_replay)으로 다시 돌리면
   콘솔 없이도 로봇 궤적이 비트 단위로 같다 (즉흥 투입 이벤트도 재실행 가능한 시나리오가 된다)
4. 고장 주입 (로봇 쪽 경계): LiDAR 끊김(점군 안 옴), 링크 두절(알고리즘에 상태 안 가고 로봇은 마지막 명령 유지),
   IMU 편향, 배터리 저하(토크 한계). 지정 시간 뒤 저절로 풀리고, 해제 명령으로도 풀린다.
   링크 두절 중 감쇠 모드로 주저앉아도 복구 뒤 알고리즘이 일어서기부터 해서 다시 걷는다.
   배터리 저하로 다리가 처지면 일어서기로 들어가 토크가 돌아올 때 튕겨 돌지 않는다
5. 실행 제어: 일시정지(예약 가능) -> 재개, 종료(예약 시각에 멈추고 다시 실행할 시나리오의 duration이 됨)
6. 링크 감시: 콘솔이 조용해지면 통신 두절 -> 로봇 이동 명령 0 이벤트 (기록됨)
7. 개체·흙 (Chrono, scenarios/dis_chrono.yaml): 숨겨 둔 HMMWV 투입 -> 보이고 출발, 흙을 무르게 -> 자국이 깊다,
   Chrono 없는 시나리오는 거부. 이 세션도 기록으로 다시 돌리면 로봇과 차량이 비트 단위로 같다.
   spawn ahead: 차가 로봇 앞을 먼저 지나가도록 출발 시각을 맞춘다 (늦으면 TOO_LATE). 시점이 어긋나면 차가 로봇에게 양보한다.
   dis_footprints: 흙을 무르게 하면 로봇 발자국이 깊어진다
시뮬레이터 서버는 이 PC 안(127.0.0.1)의 빈 포트를 쓴다.
"""
import contextlib
import io
import socket
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dis_console import envelope as E  # noqa: E402
from dis_console import protocol as P  # noqa: E402
from dis_console.console import Console, parse_command  # noqa: E402
from sim.dis_server import DisScenarioServer  # noqa: E402
from sim.runner import Simulation, apply_overrides  # noqa: E402

SCENARIO = yaml.safe_load((ROOT / "scenarios/dis_console.yaml").read_text())


def _sim(sets=(), **kw):
    sim = Simulation(apply_overrides(SCENARIO, list(sets)))
    sim.dis = DisScenarioServer(sim, port=0, **kw)
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
        _drive(sim, con, con.request(*parse_command("fault imu 0.6 gyro=0,0,0.1 att=0,2,0")))
        _drive(sim, con, con.request(*parse_command("fault link 0.1 @5.0")))
        _drive(sim, con, con.request(*parse_command("fault battery - 0.7")))
        while sim.data.time < 6.0 - 1e-9:
            _step(sim)
        qpos, t_end = sim.data.qpos.copy(), sim.data.time
        replay = sim.replay_scenario()
    finally:
        con.close(); sim.dis.close(); sim.close()
    dis_events = [e for e in replay["events"] if e.get("source") == "dis"]
    assert [e["action"] for e in dis_events] == ["set_command", "add_patch", "add_patch", "set_command", "inject_fault",
                                                 "inject_fault", "inject_fault"], dis_events
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


def test_faults_at_robot_boundary():
    sim = _sim(["controller.perception.sensor=front_lidar"])
    con = _console(sim)
    try:
        _drive(sim, con, con.request(P.CONNECT))
        _drive(sim, con, con.request(*parse_command("cmd 0.3")))
        _step(sim, 50)
        # LiDAR 끊김: 그동안 알고리즘이 쓴 스캔 수가 늘지 않고, 풀리면 다시 는다
        used = sim.controller.terrain.scans_used
        st, body = _drive(sim, con, con.request(*parse_command("fault lidar 1.0")))
        assert st == "COMPLETED" and "lidar_blackout" in sim.faults
        t_end = body["applied_t"] + 1.0
        while sim.data.time < t_end - 0.05:
            _step(sim)
        assert sim.controller.terrain.scans_used == used and "lidar_blackout" in sim.faults
        _step(sim, 25)
        assert "lidar_blackout" not in sim.faults and sim.controller.terrain.scans_used > used
        # 링크 두절: 알고리즘이 상태를 받지 않고(마지막 시각 그대로) 로봇은 마지막 명령을 계속 실행
        _drive(sim, con, con.request(*parse_command("fault link 0.3")))
        t_ctrl, cmd = sim.controller.last_t, sim.low_cmd
        _step(sim, 5)
        assert sim.controller.last_t == t_ctrl and sim.low_cmd is cmd
        _step(sim, 15)
        assert "link_loss" not in sim.faults and sim.controller.last_t > t_ctrl
        # IMU 편향: 측정 각속도 - 참 각속도 평균이 넣은 편향만큼
        _drive(sim, con, con.request(*parse_command("fault imu 0.5 gyro=0,0,0.2")))
        diff = []
        for _ in range(10):
            truth = sim.data.qvel[3:6].copy()     # 로봇 상태는 제어 주기 시작에 측정한다
            _step(sim)
            diff.append(np.array(sim.low_state["imu"]["gyro"]) - truth)
        assert abs(np.mean(diff, axis=0)[2] - 0.2) < 0.02, np.mean(diff, axis=0)
        _step(sim, 25)
        assert not sim.robot.sensors.fault_gyro.any()
        # 배터리 저하: 해제 명령 전까지 토크 한계 축소. 없는 고장 해제는 거부
        _drive(sim, con, con.request(*parse_command("fault battery - 0.5")))
        assert sim.robot.torque_scale == 0.5 and sim.faults["battery_low"]["until"] is None
        st, _ = _drive(sim, con, con.request(*parse_command("clear battery")))
        assert st == "COMPLETED" and sim.robot.torque_scale == 1.0 and not sim.faults
        st, body = _drive(sim, con, con.request(*parse_command("clear lidar")))
        assert st == "DENIED" and body["reason_code"] == "FAULT_NOT_ACTIVE"
        for line, reason in [("fault battery 5 2.0", "INVALID_VALUE"), ("fault gps 5", "INVALID_VALUE")]:
            st, body = _drive(sim, con, con.request(*parse_command(line)))
            assert st == "FAILED" and body["reason_code"] == reason, (line, body)
        assert not sim.fallen(0.0, 0.0)
    finally:
        con.close(); sim.dis.close(); sim.close()


def test_freeze_resume_stop():
    sim = _sim()
    con = _console(sim)
    try:
        _drive(sim, con, con.request(P.CONNECT))
        st, body = _drive(sim, con, con.request(P.RESUME))
        assert st == "DENIED" and body["reason_code"] == "NOT_FROZEN"
        _drive(sim, con, con.request(*parse_command("freeze @0.5")))
        assert sim.frozen and abs(sim.data.time - 0.5) < 0.03
        req = con.request(P.RESUME)
        for _ in range(200):                    # 일시정지 중: 러너처럼 시뮬레이션은 진행하지 않고 콘솔 요청만 받는다
            sim.dis.poll(); sim.dis.after_events()
            if con.reqs[req]["final"].wait(0.005):
                break
        assert con.reqs[req]["status"] == "COMPLETED" and not sim.frozen
        _drive(sim, con, con.request(*parse_command("cmd 0.3")))
        st, body = _drive(sim, con, con.request(*parse_command("end @1.0")))
        assert st == "COMPLETED" and sim.stop_requested and abs(body["applied_t"] - 1.0) < 1e-6
        replay = sim.replay_scenario()
        t_end = sim.data.time                   # 종료 예약 시각의 제어 주기까지 실행하고 멈춘다
        assert abs(t_end - 1.02) < 1e-6 and t_end - 0.02 < replay["duration"] < t_end
        sim2 = Simulation(replay)               # 다시 실행해도 같은 스텝에서 끝난다
        while sim2.data.time < replay["duration"]:
            _step(sim2)
        assert abs(sim2.data.time - t_end) < 1e-9, (sim2.data.time, t_end)
        sim2.close()
        assert not any(e["action"] in ("freeze", "stop") for e in replay["events"])
    finally:
        con.close(); sim.dis.close(); sim.close()


def test_comm_lost_stops_robot():
    sim = _sim(comm_lost_s=0.5)
    con = _console(sim)
    try:
        _drive(sim, con, con.request(P.CONNECT))
        _drive(sim, con, con.request(*parse_command("cmd 0.3")))
        _step(sim, 10)
        assert sim.dis.comm_state == "OK"        # heartbeat가 오는 동안은 정상
        con.close()                              # 콘솔이 꺼짐 -> heartbeat 끊김
        for _ in range(200):
            _step(sim)
            time.sleep(0.005)
            if sim.dis.comm_state == "LOST" and sim.command == (0.0, 0.0):
                break
        assert sim.dis.comm_state == "LOST" and sim.command == (0.0, 0.0)
        assert sim.event_log[-1]["source"] == "dis_comm_lost"
        assert any(e.get("source") == "dis_comm_lost" for e in sim.replay_scenario()["events"])
    finally:
        sim.dis.close(); sim.close()


def test_entity_and_soil_with_chrono():
    sim = Simulation(yaml.safe_load((ROOT / "scenarios/dis_chrono.yaml").read_text()))
    sim.dis = DisScenarioServer(sim, port=0)
    con = _console(sim)
    try:
        _drive(sim, con, con.request(P.CONNECT))
        assert not sim.cosim.vehicle_visible and sim.cosim.stream_poses()[0][1][2] < -50     # 숨김: 땅속
        st, body = _drive(sim, con, con.request(*parse_command("despawn")))
        assert st == "DENIED" and body["reason_code"] == "ENTITY_NOT_ACTIVE"
        assert _drive(sim, con, con.request(*parse_command("soil soft")))[0] == "COMPLETED"
        assert _drive(sim, con, con.request(*parse_command("spawn 4 @0.5")))[0] == "COMPLETED"
        assert sim.cosim.vehicle_visible
        st, body = _drive(sim, con, con.request(*parse_command("spawn")))
        assert st == "DENIED" and body["reason_code"] == "ENTITY_ACTIVE"
        while sim.data.time < 3.0:
            _step(sim)
        v = sim.cosim.vehicle
        assert v["pos"][1] > -4.0 and v["speed"] > 2.0, v          # 출발점 y = -8에서 달려왔다
        assert sim.terrain.applied.min() < -0.10, sim.terrain.applied.min()   # 무른 흙: 단단한 흙(약 5.5 cm)보다 깊은 자국
        qpos, vpos, replay = sim.data.qpos.copy(), list(v["pos"]), sim.replay_scenario()
    finally:
        con.close(); sim.dis.close(); sim.close()
    sim2 = Simulation(yaml.safe_load(yaml.safe_dump(replay, allow_unicode=True)))
    try:
        while sim2.data.time < 3.0:
            _step(sim2)
        assert np.array_equal(sim2.data.qpos, qpos) and list(sim2.cosim.vehicle["pos"]) == vpos
    finally:
        sim2.close()
    sim = _sim()                                    # Chrono 없는 시나리오: 거부
    con = _console(sim)
    try:
        _drive(sim, con, con.request(P.CONNECT))
        for line in ("spawn", "soil mud"):
            st, body = _drive(sim, con, con.request(*parse_command(line)))
            assert st == "DENIED" and body["reason_code"] == "NO_CHRONO", (line, body)
    finally:
        con.close(); sim.dis.close(); sim.close()

def test_stand_up_after_link_loss():
    """감쇠 모드로 1초(주저앉음), 3초(배를 대고 엎드림) 끊긴 뒤 일어서서 다시 걷는다. 넘어짐 판정 없음.
    지형 인지 보행도: 공백 동안 추정 위치가 튀지 않고(공백 전 속도로 적분하지 않음) 지면 높이를 잃지 않아야 걷는다."""
    for dur, sets in ((1.0, []), (3.0, []), (3.0, ["controller.perception.sensor=front_lidar"])):
        scn = apply_overrides(dict(SCENARIO, events=[{"t": 0.0, "action": "set_command", "vx": 0.3, "yaw_rate": 0.0},
                                                     {"t": 1.0, "action": "inject_fault", "fault": "link_loss", "duration": dur,
                                                      "params": {"robot_behavior": "damp"}}]), sets)
        sim = Simulation(scn)
        d, z_min, p0 = sim.data, 1.0, sim.data.qpos[:3].copy()
        try:
            while d.time < 1.0 + dur:
                _, _, roll, pitch = _step_out(sim)
                z_min = min(z_min, d.qpos[2])
                assert not sim.fallen(roll, pitch), f"두절 중 넘어짐 판정 t={d.time:.2f}"
            x0 = d.qpos[0]
            while d.time < 1.0 + dur + 6.0:
                _, _, roll, pitch = _step_out(sim)
                assert not sim.fallen(roll, pitch), f"복구 뒤 넘어짐 t={d.time:.2f} {sets}"
            err = np.abs(sim.controller.est.pos - (d.qpos[:3] - p0)).max()
            assert z_min < 0.16 and d.qpos[2] > 0.24 and d.qpos[0] - x0 > 1.0 and err < 0.1, \
                (dur, sets, z_min, d.qpos[2], d.qpos[0] - x0, err)
        finally:
            sim.close()


def _step_out(sim):
    with contextlib.redirect_stdout(io.StringIO()):
        return sim.step_control()

def _chrono_session(walk_to, line, seconds):
    """로봇을 0.35 m/s로 걷게 하고 x가 walk_to가 되면 콘솔 명령, 그 뒤 seconds초 진행. 반환: (최종 상태, body, 기록)."""
    sim = Simulation(yaml.safe_load((ROOT / "scenarios/dis_chrono.yaml").read_text()))
    sim.dis = DisScenarioServer(sim, port=0)
    con = _console(sim)
    log = {"x_at_pass": None, "dmin": np.inf, "yield_steps": 0}
    try:
        _drive(sim, con, con.request(P.CONNECT))
        _drive(sim, con, con.request(*parse_command("cmd 0.35")))
        while sim.data.qpos[0] < walk_to:
            _step(sim)
        req = con.request(*parse_command(line))
        r, t_end = con.reqs[req], sim.data.time + seconds
        while sim.data.time < t_end:
            _step(sim)
            if r["status"] == "DENIED":
                break
            if sim.cosim.vehicle_visible:
                v, (rx, ry) = sim.cosim.vehicle, sim.data.qpos[:2]
                log["dmin"] = min(log["dmin"], float(np.hypot(v["pos"][0] - rx, v["pos"][1] - ry)))
                log["yield_steps"] += bool(v.get("yielding"))
                if log["x_at_pass"] is None and v["pos"][1] - ry > 2.4:      # 차 꼬리가 로봇 진행선을 지남
                    log["x_at_pass"] = float(rx)
        r["final"].wait(1.0)
        return r["status"], r["body"], log
    finally:
        con.close(); sim.dis.close(); sim.close()


def test_vehicle_passes_ahead_or_yields():
    st, body, log = _chrono_session(0.0, "spawn ahead 2.5", 9.0)
    assert st == "COMPLETED" and log["x_at_pass"] is not None and log["x_at_pass"] < 5.0 - 2.0, (body, log)
    assert log["yield_steps"] == 0 and log["dmin"] > 2.5, log
    st, body, _ = _chrono_session(3.8, "spawn ahead 2.5", 1.0)
    assert st == "DENIED" and body["reason_code"] == "TOO_LATE", body
    st, body, log = _chrono_session(3.3, "spawn", 14.0)       # 시점이 어긋남: 차가 기다렸다가 로봇 뒤로 지나간다
    assert st == "COMPLETED" and log["yield_steps"] > 100 and log["dmin"] > 2.5, log
    assert log["x_at_pass"] is not None and log["x_at_pass"] > 5.0 + 1.5, log

def test_soil_changes_robot_footprints():
    """dis_footprints: 콘솔로 흙을 무르게 하면 로봇 발자국이 깊어진다 (hard 평균 약 0.7 cm, soft 약 3 cm)."""
    depth = {}
    for preset in ("hard", "soft"):
        sim = Simulation(yaml.safe_load((ROOT / "scenarios/dis_footprints.yaml").read_text()))
        sim.dis = DisScenarioServer(sim, port=0)
        con = _console(sim)
        try:
            _drive(sim, con, con.request(P.CONNECT))
            assert _drive(sim, con, con.request(*parse_command(f"soil {preset}")))[0] == "COMPLETED"
            _drive(sim, con, con.request(*parse_command("cmd 0.35")))
            while sim.data.time < 9.0:
                _step(sim)
            T = sim.terrain
            m = (T.X > 1.2) & (T.X < sim.data.qpos[0] - 0.4) & (np.abs(T.Y) < 0.4)
            dents = T.applied[m][T.applied[m] < -0.003]
            assert dents.size > 50, (preset, dents.size)
            depth[preset] = -dents.mean()
            st, body = _drive(sim, con, con.request(*parse_command("spawn")))
            assert st == "DENIED" and body["reason_code"] == "NO_ENTITY_SLOT"
        finally:
            con.close(); sim.dis.close(); sim.close()
    assert depth["soft"] > 2 * depth["hard"], depth

def test_battery_low_recovery():
    """배터리 저하로 토크가 모자라면 (20%, 30%) 알고리즘이 다리 처짐을 보고 일어서기로 들어가, 토크가 돌아올 때 튕겨 돌지 않는다.
    고치기 전: 해제 순간 0.5초 만에 -40° (20%), 30%에서는 계속 돌아 -110°. 40%는 평지 보행에 영향이 없어 일어서기도 없다."""
    from sim.adapters import quat_to_yaw
    for scale, want_rec in ((0.2, True), (0.3, True), (0.4, False)):
        scn = dict(SCENARIO, events=[{"t": 0.0, "action": "set_command", "vx": 0.3, "yaw_rate": 0.0},
                                     {"t": 2.0, "action": "inject_fault", "fault": "battery_low", "duration": 6.0,
                                      "params": {"torque_scale": scale}}])
        sim = Simulation(scn)
        d, n_rec, prev, yaw_dev = sim.data, 0, None, 0.0
        try:
            while d.time < 8.0:
                _step_out(sim)
                r = sim.controller.recovery
                n_rec += r is not None and prev is None
                prev = r
            x0 = d.qpos[0]
            while d.time < 14.0:
                _, _, roll, pitch = _step_out(sim)
                yaw_dev = max(yaw_dev, abs(np.degrees(quat_to_yaw(d.qpos[3:7]))))
                assert not sim.fallen(roll, pitch), (scale, d.time)
            assert (n_rec > 0) == want_rec and yaw_dev < 15.0 and d.qpos[0] - x0 > 1.2, (scale, n_rec, yaw_dev, d.qpos[0] - x0)
        finally:
            sim.close()

if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
