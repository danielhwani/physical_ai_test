"""C++ 실시간 코어 시험 (rt/, sim/rt_link.py, 문서 §11).

    python conformance/test_rt_core.py

1. 공유 메모리 규약: C++ (rt/shm_layout.h)과 Python ctypes 정의의 위치·크기가 같다
2. 물리: Python 시뮬레이터가 실행한 관절 명령을 그대로 C++ 코어에 넣으면 매 스텝 몸통·관절 상태가 비트 단위로 같다
   (같은 libmujoco, 같은 PD 식과 순서, FMA 축약 금지). 로봇 상태(센서 모델) 출력 차이도 잰다
3. 닫힌 고리 (lockstep): 같은 보행 알고리즘을 붙이면 Python 실행과 끝 상태가 같다
4. 실시간: 몇 초 돌려 잡음이 모자라지 않고, 모든 스텝을 돌고, 타이밍이 기록된다 (주기 초과 수는 시스템 설정에 달려 판정하지 않음)
"""
import contextlib
import io
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
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
            while sim.data.time < seconds - 1e-9:
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
    assert mop["steps"] == 150 and mop["noise_underruns"] == 0 and mop["mlockall"], mop
    assert mop["wake_us_max"] > 0 and mop["compute_us_mean"] > 0 and not mop["fell"], mop
    print(f"    실시간 3초: 깨어남 지연 최대 {mop['wake_us_max']:.0f} us, 계산 평균 {mop['compute_us_mean']:.0f} us, "
          f"최대 {mop['compute_us_max']:.0f} us, 주기 초과 {mop['overruns']}, SCHED_FIFO {mop['sched_fifo']}")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
