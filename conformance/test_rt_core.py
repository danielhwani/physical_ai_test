"""C++ 실시간 코어 시험 (rt/, sim/rt_link.py, 문서 §11).

    python conformance/test_rt_core.py

1. 공유 메모리 규약: C++ (rt/shm_layout.h)과 Python ctypes 정의의 위치·크기가 같다
2. 물리: Python 시뮬레이터가 실행한 관절 명령을 그대로 C++ 코어에 넣으면 매 스텝 몸통·관절 상태가 비트 단위로 같다
   (같은 libmujoco, 같은 PD 식과 순서, FMA 축약 금지). 로봇 상태(센서 모델) 출력 차이도 잰다
3. 닫힌 고리 (lockstep): 같은 보행 알고리즘을 붙이면 Python 실행과 끝 상태가 같다. 두뇌(상위 제어기) 연결 방식
   (같은 프로세스 / 같은 PC 별도 프로세스·공유 메모리 / ROS2 노드)과 고장(링크 두절 감쇠·유지, 배터리 저하)이 섞여도 같다
4. 기록: lockstep 실행의 timeseries.parquet가 sim.runner와 모든 공통 열에서 비트 단위로 같다 (MOP도 같다). 모델 설정 mjx도 같다.
   지형이 바뀌는 시나리오(자국 이벤트, 지형 창 이동 + 실행 중 자국 추가)도 같다.
   LiDAR·지형 인지 보행 (LiDAR 끊김 고장 포함): 세 두뇌 연결 방식 모두 기록과 스캔 점이 비트 단위로 같다.
   Chrono (lockstep): 발자국, HMMWV 투입·제거, 흙 바꾸기, LiDAR가 차량을 보는 지형 인지 보행도 같다.
   Chrono 실시간 (기다리지 않는 교환, 뒤처짐 상한): DIS 콘솔로 흙·차량 투입
   무른 흙 (foot_soil): 지형 패치 이벤트(세 두뇌 연결 방식), 지형 창 + 실행 중 자국 추가도 기록·MOP가 비트 단위로 같다
5. DIS 콘솔 (실시간, 별도 프로세스 두뇌): 콘솔 명령(이동, 자국, 링크 두절, 종료)이 접수·적용되고, 남은 scenario_replay.yaml을
   Python 시뮬레이터(sim.runner)로 다시 돌리면 같은 결과다 (실시간 실행에서 늦은 명령·늦게 걸린 변경이 없었을 때).
   실행 도중 시각 없이 보낸 요청(이동, 자국, 링크 두절 감쇠)도 같다
6. 실시간: 몇 초 돌려 잡음이 모자라지 않고, 모든 스텝을 돌고, 타이밍이 기록된다 (주기 초과 수는 시스템 설정에 달려 판정하지 않음)
"""
import contextlib
import io
import os
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
os.environ.setdefault("ROS_DOMAIN_ID", "79")      # 다른 ROS2 실행과 섞이지 않게
sys.path.insert(0, str(ROOT))
from sim import rt_link  # noqa: E402
from sim.runner import Simulation  # noqa: E402

SCENARIO = yaml.safe_load((ROOT / "scenarios/flat_trot.yaml").read_text())


def _python_run(seconds):
    """Python 시뮬레이터 (같은 프로세스): 스텝마다 받은 로봇 상태, 실행한 명령, 진행 뒤 qpos."""
    sim = Simulation(dict(SCENARIO, duration=seconds))
    states, cmds, qpos = [], [], []
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < seconds:              # sim.runner와 같은 조건
                sim.step_control()
                states.append(sim.low_state); cmds.append(sim.low_cmd); qpos.append(sim.data.qpos.copy())
    finally:
        sim.close()
    return states, cmds, qpos


def test_layout_matches_cpp():
    if not rt_link.CORE.exists():
        rt_link.build()
    c = json.loads(subprocess.run([str(rt_link.CORE), "--layout"], capture_output=True, text=True, check=True).stdout)
    assert c == rt_link.layout(), {k: (c.get(k), rt_link.layout().get(k)) for k in c}


def test_physics_bit_identical_given_same_commands():
    seconds = 4.0
    states, cmds, qpos = _python_run(seconds)
    sim = Simulation(dict(SCENARIO, duration=seconds))
    link = rt_link.RtLink(sim, lockstep=True)
    sensor_err = 0.0
    try:
        link.start()
        for k in range(len(cmds)):
            if "t" in cmds[k]:                          # 0번째는 기본 자세 (명령 없음): 코어도 기본 자세
                link.write_cmd(cmds[k])
            link.shm.ctl.lockstep_target = k
            while link.shm.stats.steps < k + 1:
                time.sleep(0.00002)
            e = link.truth(k)
            assert np.array_equal(np.array(e.qpos), qpos[k]), (k, np.abs(np.array(e.qpos) - qpos[k]).max())
            ls, ref = link.read_state(), states[k]
            v = np.concatenate([ls["q"], ls["dq"], ls["imu"]["quat"], ls["imu"]["gyro"], ls["imu"]["accel"], ls["foot_force"]])
            w = np.concatenate([ref["q"], ref["dq"], ref["imu"]["quat"], ref["imu"]["gyro"], ref["imu"]["accel"], ref["foot_force"]])
            sensor_err = max(sensor_err, float(np.abs(v - w).max()))
        assert link.shm.stats.noise_underruns == 0
    finally:
        link.close(); sim.close()
    assert sensor_err < 1e-12, sensor_err
    print(f"    {len(cmds)} 스텝 qpos 비트 동일, 로봇 상태 최대 차이 {sensor_err:.1e}")


def test_closed_loop_lockstep_matches_python():
    seconds = 6.0
    _, _, qpos = _python_run(seconds)
    with contextlib.redirect_stdout(io.StringIO()):
        mop, q = rt_link.run(dict(SCENARIO, duration=seconds), lockstep=True, verbose=False)
    assert mop["steps"] == len(qpos) and np.array_equal(q, qpos[-1]), np.abs(q - qpos[-1]).max()
    assert mop["cmd_latency_ms_mean"] == 20.0 and mop["noise_underruns"] == 0


def test_realtime_runs_and_reports_timing():
    with contextlib.redirect_stdout(io.StringIO()):
        mop, _ = rt_link.run(dict(SCENARIO, duration=3.0), lockstep=False, verbose=False)
    assert mop["steps"] == rt_link.steps_for(3.0, 0.002, 10) and mop["noise_underruns"] == 0 and mop["mlockall"], mop
    assert mop["wake_us_max"] > 0 and mop["compute_us_mean"] > 0 and not mop["fell"], mop
    print(f"    실시간 3초: 깨어남 지연 최대 {mop['wake_us_max']:.0f} us, 계산 평균 {mop['compute_us_mean']:.0f} us, "
          f"최대 {mop['compute_us_max']:.0f} us, 주기 초과 {mop['overruns']}, SCHED_FIFO {mop['sched_fifo']}")


def test_brain_transports_and_faults_match_python():
    scn = dict(SCENARIO, duration=7.0, events=sorted(SCENARIO["events"] + [
        {"t": 2.0, "action": "inject_fault", "fault": "link_loss", "duration": 1.0, "params": {"robot_behavior": "damp"}},
        {"t": 4.0, "action": "inject_fault", "fault": "battery_low", "duration": 1.0, "params": {"torque_scale": 0.5}},
        {"t": 4.5, "action": "inject_fault", "fault": "link_loss", "duration": 0.5, "params": {}}], key=lambda e: e["t"]))
    sim = Simulation(scn)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < 7.0:
                sim.step_control()
        ref = sim.data.qpos.copy()
    finally:
        sim.close()
    for brain in ("inproc", "shm", "ros"):
        with contextlib.redirect_stdout(io.StringIO()):
            mop, q = rt_link.run(dict(scn), lockstep=True, verbose=False, brain=brain)
        assert np.array_equal(q, ref), (brain, np.abs(q - ref).max())
        assert mop["brain"] == brain and mop["noise_underruns"] == 0


def test_realtime_shm_brain():
    with contextlib.redirect_stdout(io.StringIO()):
        mop, _ = rt_link.run(dict(SCENARIO, duration=3.0), lockstep=False, verbose=False, brain="shm")
    assert mop["steps"] == rt_link.steps_for(3.0, 0.002, 10) and not mop["fell"] and mop["cmd_latency_ms_mean"] < 25, mop
    print(f"    별도 프로세스 두뇌 (공유 메모리) 실시간 3초: 명령 지연 최대 {mop['cmd_latency_ms_max']} ms, 늦은 주기 {mop['cmd_late_steps']}")

def _window_scenario():
    """지형 창(로봇을 따라 옮김) + 실행 중 자국 추가."""
    s = yaml.safe_load((ROOT / "scenarios/dis_console.yaml").read_text())
    s.update(name="rt_window_check", duration=10.0, events=[
        {"t": 0.5, "action": "set_command", "vx": 0.5, "yaw_rate": 0.0},
        {"t": 2.0, "action": "add_patch", "patch": {"kind": "rut", "start": [3.0, -30], "end": [3.0, 30], "width": 0.25,
                                                    "depth": 0.04, "profile": "box"}}])
    return s


def test_recording_matches_runner():
    import pyarrow.parquet as pq
    import re
    import shutil
    import tempfile
    cases = [("flat_trot", SCENARIO, "cpu"), ("flat_trot", SCENARIO, "mjx"),
             ("rough_rut", yaml.safe_load((ROOT / "scenarios/rough_rut.yaml").read_text()), "cpu"),
             ("window", _window_scenario(), "cpu")]
    for name, scn, variant in cases:
        tmp = Path(tempfile.mkdtemp())
        scn_file = tmp / "scenario.yaml"
        scn_file.write_text(yaml.safe_dump(scn, allow_unicode=True))
        out = subprocess.run([sys.executable, "-m", "sim.runner", str(scn_file), "--variant", variant],
                             cwd=ROOT, capture_output=True, text=True, check=True).stdout
        py_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                mop, _ = rt_link.run(dict(scn), lockstep=True, verbose=False, out_dir=tmp / "rt", variant=variant)
            a = pq.read_table(py_dir / "timeseries.parquet").to_pandas()
            b = pq.read_table(tmp / "rt" / "timeseries.parquet").to_pandas()
            assert len(a) == len(b) and set(a.columns) <= set(b.columns), (name, variant, len(a), len(b))
            diff = [c for c in a.columns if not np.array_equal(a[c].to_numpy(), b[c].to_numpy(), equal_nan=True)]
            assert not diff, (name, variant, diff)
            ma = json.loads((py_dir / "summary.json").read_text())["mop"]
            for k in ("forward_x_m", "distance_m", "cost_of_transport", "touchdowns", "edge_touchdowns", "est_speed_rmse_mps"):
                assert ma[k] == mop[k], (name, variant, k, ma[k], mop[k])
        finally:
            shutil.rmtree(py_dir, ignore_errors=True)
            shutil.rmtree(tmp, ignore_errors=True)

def test_dis_console_on_rt_core():
    import re
    import shutil
    import socket
    from dis_console import protocol as P
    from dis_console.console import Console, parse_command
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sk:            # 빈 포트
        sk.bind(("127.0.0.1", 0)); port = sk.getsockname()[1]
    proc = subprocess.Popen([sys.executable, "-m", "sim.rt_link", "scenarios/dis_console.yaml", "--dis-port", str(port),
                             "--dis-wait", "--brain", "shm"], cwd=ROOT, stdout=subprocess.PIPE, text=True)
    con = Console(("127.0.0.1", port), out=lambda *_: None)
    dirs = []
    try:
        assert con.call(P.CONNECT, {"role": "control"}, timeout=30)[0] == "COMPLETED"
        for line in ("cmd 0.4", "rut 1.0 @2", "fault link 0.5 damp @3", "end @5"):
            st, body = con.call(*parse_command(line), timeout=30)
            assert st == "COMPLETED", (line, st, body)
        out, _ = proc.communicate(timeout=60)
        run_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
        dirs.append(run_dir)
        mop = json.loads((run_dir / "summary.json").read_text())["mop"]
        assert mop["stopped_by_console"] and mop["brain"] == "shm" and (run_dir / "dis_pdus.jsonl").exists(), mop
        rep = subprocess.run([sys.executable, "-m", "sim.runner", str(run_dir / "scenario_replay.yaml")], cwd=ROOT,
                             capture_output=True, text=True, check=True).stdout
        dirs.append(ROOT / re.search(r"기록: (\S+)", rep).group(1))
        replay_x = float(re.search(r"forward_x_m\s+(\S+)", rep).group(1))
        if mop["cmd_late_steps"] == 0 and mop["late_applies"] == 0:   # 실시간에서 늦은 명령·늦게 걸린 변경이 없었으면 Python 재현과 같다
            assert replay_x == mop["forward_x_m"], (replay_x, mop["forward_x_m"])
        print(f"    DIS 콘솔 (C++ 코어, 실시간): 전진 {mop['forward_x_m']} m, Python 재현 {replay_x} m, 늦은 명령 {mop['cmd_late_steps']}")
    finally:
        con.close()
        if proc.poll() is None:
            proc.kill()
        for d in dirs:
            shutil.rmtree(d, ignore_errors=True)

def test_dis_immediate_events_replay_on_rt_core():
    """실시간 도중 시각 없이 보낸 콘솔 요청(이동 명령, 홈, 링크 두절 감쇠)도 기록대로 다시 실행하면 같다.
    고장 끝 시각은 적용 시각부터 (요청 시각부터 세면 한 주기 일찍 끝났다), 지난 시각이 붙은 명령은 다음 스텝 시각으로 정한다."""
    import re
    import shutil
    import socket
    import time
    import pandas as pd
    from dis_console import protocol as P
    from dis_console.console import Console, parse_command
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sk:
        sk.bind(("127.0.0.1", 0)); port = sk.getsockname()[1]
    proc = subprocess.Popen([sys.executable, "-m", "sim.rt_link", "scenarios/dis_console.yaml", "--dis-port", str(port),
                             "--dis-wait", "--brain", "shm"], cwd=ROOT, stdout=subprocess.PIPE, text=True)
    con = Console(("127.0.0.1", port), out=lambda *_: None)
    dirs = []
    try:
        assert con.call(P.CONNECT, {"role": "control"}, timeout=30)[0] == "COMPLETED"
        time.sleep(1.0)                                    # 실행 도중에
        for line in ("cmd 0.4", "rut 1.0", "fault link 0.5 damp", "end @4"):
            st, body = con.call(*parse_command(line), timeout=30)
            assert st == "COMPLETED", (line, st, body)
        out, _ = proc.communicate(timeout=60)
        run_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
        dirs.append(run_dir)
        mop = json.loads((run_dir / "summary.json").read_text())["mop"]
        rep = subprocess.run([sys.executable, "-m", "sim.runner", str(run_dir / "scenario_replay.yaml")], cwd=ROOT,
                             capture_output=True, text=True, check=True).stdout
        dirs.append(ROOT / re.search(r"기록: (\S+)", rep).group(1))
        replay_x = float(re.search(r"forward_x_m\s+(\S+)", rep).group(1))
        damp = int((pd.read_parquet(run_dir / "timeseries.parquet").cmd_state_t == -2).sum())
        on_time = mop["cmd_late_steps"] == 0 and mop["late_applies"] == 0
        if on_time:
            assert damp == 26, damp                        # 0.5 s = 25주기 + 복구 주기 (새 명령 전까지 감쇠 유지)
            assert replay_x == mop["forward_x_m"], (replay_x, mop["forward_x_m"])
        print(f"    즉시 요청 (C++ 코어, 실시간): 전진 {mop['forward_x_m']} m, Python 재현 {replay_x} m, 감쇠 {damp}주기, "
              f"늦은 명령 {mop['cmd_late_steps']}, 늦게 걸린 변경 {mop['late_applies']}{'' if on_time else ' (비교 생략)'}")
    finally:
        con.close()
        if proc.poll() is None:
            proc.kill()
        for d in dirs:
            shutil.rmtree(d, ignore_errors=True)


def test_dis_chrono_realtime_on_rt_core():
    """실시간 Chrono (기다리지 않는 교환): 콘솔로 흙·차량 투입이 되고, Chrono 뒤처짐은 상한(0.2 s) 근처로 묶인다."""
    import re
    import shutil
    import socket
    from dis_console import protocol as P
    from dis_console.console import Console, parse_command
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sk:
        sk.bind(("127.0.0.1", 0)); port = sk.getsockname()[1]
    proc = subprocess.Popen([sys.executable, "-m", "sim.rt_link", "scenarios/dis_chrono.yaml", "--dis-port", str(port),
                             "--dis-wait", "--brain", "shm"], cwd=ROOT, stdout=subprocess.PIPE, text=True)
    con = Console(("127.0.0.1", port), out=lambda *_: None)
    run_dir = None
    try:
        assert con.call(P.CONNECT, {"role": "control"}, timeout=60)[0] == "COMPLETED"
        for line in ("cmd 0.35", "soil soft @1", "spawn @1.5", "end @8"):
            st, body = con.call(*parse_command(line), timeout=60)
            assert st == "COMPLETED", (line, st, body)
        out, _ = proc.communicate(timeout=120)
        run_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
        mop = json.loads((run_dir / "summary.json").read_text())["mop"]
        assert mop["stopped_by_console"] and not mop["fell"] and mop["cmd_late_steps"] == 0, mop   # 주기 초과는 절전 상태에 따라 가끔 (README 실시간 측정)
        assert mop["min_vehicle_distance_m"] is not None and mop["deformed_cells"] > 0 and mop["chrono_lag_s_max"] < 0.3, mop
        print(f"    DIS + Chrono (실시간): 전진 {mop['forward_x_m']} m, 차량 최소 {mop['min_vehicle_distance_m']} m, "
              f"Chrono 뒤처짐 최대 {mop['chrono_lag_s_max']} s (멈춤 {mop['chrono_holds']}번 {mop['chrono_hold_s']} s), 주기 초과 {mop['overruns']}")
    finally:
        con.close()
        if proc.poll() is None:
            proc.kill()
        if run_dir is not None:
            shutil.rmtree(run_dir, ignore_errors=True)


def test_perception_matches_runner():
    import pyarrow.parquet as pq
    import re
    import shutil
    import tempfile
    from sim.runner import apply_overrides
    scn = apply_overrides(yaml.safe_load((ROOT / "scenarios/rut_crossing.yaml").read_text()),
                          ["controller.perception.sensor=front_lidar", "duration=8"])
    scn["events"] = scn["events"] + [{"t": 4.0, "action": "inject_fault", "fault": "lidar_blackout", "duration": 1.0,
                                      "params": {}}]
    tmp = Path(tempfile.mkdtemp())
    (tmp / "scenario.yaml").write_text(yaml.safe_dump(scn, allow_unicode=True))
    out = subprocess.run([sys.executable, "-m", "sim.runner", str(tmp / "scenario.yaml")], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    py_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
    try:
        a = pq.read_table(py_dir / "timeseries.parquet").to_pandas()
        la = np.load(py_dir / "front_lidar.npz")
        for brain in ("inproc", "shm", "ros"):
            with contextlib.redirect_stdout(io.StringIO()):
                rt_link.run(dict(scn), lockstep=True, verbose=False, brain=brain, out_dir=tmp / brain)
            b = pq.read_table(tmp / brain / "timeseries.parquet").to_pandas()
            diff = [c for c in a.columns if len(a) != len(b) or not np.array_equal(a[c].to_numpy(), b[c].to_numpy(), equal_nan=True)]
            assert not diff, (brain, diff)
            lb = np.load(tmp / brain / "front_lidar.npz")
            assert np.array_equal(la["t"], lb["t"]) and np.array_equal(la["points"], lb["points"]), brain
        assert not np.any((la["t"] > 4.0) & (la["t"] < 5.0 - 1e-9))     # LiDAR 끊김 동안 스캔 없음
    finally:
        shutil.rmtree(py_dir, ignore_errors=True)
        shutil.rmtree(tmp, ignore_errors=True)

def _runner_vs_rt(scn, brains, tmp, extra_npz=()):
    import pyarrow.parquet as pq
    import re
    import shutil
    tmp.mkdir(parents=True, exist_ok=True)
    (tmp / "scenario.yaml").write_text(yaml.safe_dump(scn, allow_unicode=True))
    out = subprocess.run([sys.executable, "-m", "sim.runner", str(tmp / "scenario.yaml")], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    py_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
    try:
        a = pq.read_table(py_dir / "timeseries.parquet").to_pandas()
        ma = json.loads((py_dir / "summary.json").read_text())["mop"]
        for brain in brains:
            with contextlib.redirect_stdout(io.StringIO()):
                mb, _ = rt_link.run(dict(scn), lockstep=True, verbose=False, brain=brain, out_dir=tmp / brain)
            b = pq.read_table(tmp / brain / "timeseries.parquet").to_pandas()
            diff = [c for c in a.columns if len(a) != len(b) or not np.array_equal(a[c].to_numpy(), b[c].to_numpy(), equal_nan=True)]
            assert not diff, (brain, diff)
            for k in ("min_vehicle_distance_m", "deformed_cells", "deform_mean_m", "deform_max_m", "forward_x_m",
                      "foot_sink_mean_m", "foot_sink_max_m", "foot_slip_mean_m", "plow_work_j", "plow_work_up_j",
                      "foot_sink_by_soil"):
                assert ma.get(k) == mb.get(k), (brain, k, ma.get(k), mb.get(k))
            for name in extra_npz:
                la, lb = np.load(py_dir / f"{name}.npz"), np.load(tmp / brain / f"{name}.npz")
                assert np.array_equal(la["points"], lb["points"]) and np.array_equal(la["label"], lb["label"]), brain
        return ma
    finally:
        shutil.rmtree(py_dir, ignore_errors=True)


def test_foot_soil_matches_runner():
    """무른 흙 (foot_soil, C++ 코어의 같은 알고리즘): 지형 패치 이벤트가 있는 rough_rut(세 두뇌 연결 방식)과
    지형 창 + 실행 중 자국 추가, 지역마다 다른 흙(진흙 띠), 실행 중 흙 바꾸기(set_soil)가 sim.runner와 기록·MOP까지 비트 단위로 같다."""
    import shutil
    import tempfile
    tmp = Path(tempfile.mkdtemp())
    try:
        rr = yaml.safe_load((ROOT / "scenarios/rough_rut.yaml").read_text())
        rr["duration"], rr["foot_soil"] = 5.0, {"preset": "firm"}
        ma = _runner_vs_rt(rr, ("inproc", "shm", "ros"), tmp / "rr")
        win = yaml.safe_load((ROOT / "scenarios/dis_console.yaml").read_text())
        win["duration"], win["foot_soil"] = 8.0, {"preset": "soft", "plow_ratio": 0.5}
        win["events"] = [{"t": 0.5, "action": "set_command", "vx": 0.5, "yaw_rate": 0.0},
                         {"t": 2.0, "action": "add_patch",
                          "patch": {"kind": "rut", "start": [1.0, -50.0], "end": [1.0, 50.0], "width": 0.25, "depth": 0.05,
                                    "profile": "box"}}]
        mb = _runner_vs_rt(win, ("inproc",), tmp / "win")
        strip = yaml.safe_load((ROOT / "scenarios/soil_strip.yaml").read_text())
        strip["duration"] = 8.0
        mc = _runner_vs_rt(strip, ("inproc",), tmp / "strip")                      # 지역마다 다른 흙 (진흙 띠)
        from dis_console.protocol import SOIL_PRESETS
        chg = yaml.safe_load((ROOT / "scenarios/flat_trot.yaml").read_text())     # 실행 중 흙 바꾸기 (DIS 콘솔 soil)
        chg["duration"], chg["foot_soil"] = 6.0, {"preset": "firm"}
        chg["events"] = chg["events"] + [{"t": 3.0, "action": "set_soil", "preset": "mud", "soil": dict(SOIL_PRESETS["mud"])}]
        md = _runner_vs_rt(chg, ("inproc", "shm"), tmp / "chg")
        assert md["foot_sink_max_m"] > 0.06, md                                     # 진흙으로 바뀐 뒤 깊이 빠진다 (보통 흙은 약 3.5 cm)
        assert ma["foot_sink_mean_m"] > 0.005 and mb["plow_work_j"] > 0 and mc["foot_sink_by_soil"]["mud_strip"]["stances"] > 0, (ma, mb, mc)
        print(f"    무른 흙 (C++ 코어 lockstep = sim.runner): rough_rut firm 침하 {100 * ma['foot_sink_mean_m']:.1f} cm, "
              f"지형 창 soft 쟁기질 {mb['plow_work_j']:.0f} J")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_chrono_matches_runner():
    import shutil
    import tempfile
    from dis_console import protocol as P
    from sim.runner import apply_overrides
    tmp = Path(tempfile.mkdtemp())
    try:
        fp = apply_overrides(yaml.safe_load((ROOT / "scenarios/footprints.yaml").read_text()), ["duration=4"])
        assert _runner_vs_rt(fp, ("inproc",), tmp / "fp")["deformed_cells"] > 0          # 발자국 (발 하중 -> 흙)
        ev = apply_overrides(yaml.safe_load((ROOT / "scenarios/dis_chrono.yaml").read_text()),
                             ["duration=8", "controller.perception.sensor=front_lidar"])
        ev["events"] = [{"t": 0.5, "action": "set_command", "vx": 0.35, "yaw_rate": 0.0},
                        {"t": 1.0, "action": "set_soil", "soil": dict(P.SOIL_PRESETS["soft"])},
                        {"t": 1.5, "action": "create_entity", "entity_type": "HMMWV", "speed": 4.0},
                        {"t": 6.5, "action": "remove_entity", "entity_type": "HMMWV"}]
        m = _runner_vs_rt(ev, ("inproc", "shm"), tmp / "ev", extra_npz=("front_lidar",))
        assert m["min_vehicle_distance_m"] is not None and m["deformed_cells"] > 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
