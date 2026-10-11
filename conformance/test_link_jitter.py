"""lockstep의 통계적 명령 지연 시험 (sim/link_jitter.py, 시나리오 control_link).

    python conformance/test_link_jitter.py

1. 분포를 주지 않으면 고정 지연 경로 그대로 (지연 객체 없음)
2. 같은 시드 = 같은 지연 순서, 다른 시드 = 다른 순서. 뽑힌 비율이 분포를 따른다
3. 로봇 쪽 규칙: 도착한 것 중 가장 새 명령, 추월당한 명령은 버림, 손실이면 마지막 명령 유지
4. 시뮬레이션: 같은 시드면 비트 단위로 같고, 실행한 명령의 나이(cmd_latency)가 뽑힌 지연과 맞는다
5. C++ 실시간 코어 경로는 이 설정을 받지 않는다 (실제 지연을 쓴다)
"""
import contextlib
import copy
import io
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.link_jitter import make_link_jitter  # noqa: E402
from sim.runner import Simulation  # noqa: E402

BASE = yaml.safe_load((ROOT / "scenarios/flat_trot.yaml").read_text())


def _cmd(t):
    return {"t": t}


def test_no_distribution_keeps_fixed_path():
    assert make_link_jitter({"latency_steps": 1}, 0) is None
    assert Simulation(BASE).link_jitter is None


def test_seeded_and_follows_distribution():
    def draws(seed):
        j = make_link_jitter({"extra_steps_probs": [0.1, 0.05], "drop_probability": 0.02}, seed)
        for k in range(20000):
            j.deliver(k, _cmd(k * 0.02), 1)
        return j
    a, b, c = draws(0), draws(0), draws(1)
    assert a.hist.tolist() == b.hist.tolist() and a.dropped == b.dropped
    assert a.hist.tolist() != c.hist.tolist()
    n = a.sent
    assert abs(a.hist[1] / n - 0.1) < 0.01 and abs(a.hist[2] / n - 0.05) < 0.007 and abs(a.dropped / n - 0.02) < 0.005
    u = make_link_jitter({"late_probability": 0.06, "max_extra_steps": 3}, 0)   # 균등 약식
    assert np.allclose(u.probs, [0.02, 0.02, 0.02])
    for bad in ({"extra_steps_probs": [0.5], "late_probability": 0.1}, {"extra_steps_probs": [0.8], "drop_probability": 0.3}):
        try:
            make_link_jitter(bad, 0)
        except ValueError:
            continue
        raise AssertionError(f"잘못된 설정을 받았다: {bad}")


def test_robot_side_rules():
    j = make_link_jitter({"drop_probability": 1e-9}, 0)
    j.inflight = [(5, _cmd(0.06)), (5, _cmd(0.08))]                 # 같은 스텝에 둘 도착: 새 것만
    j.rng = np.random.default_rng(0)
    out = j.deliver(5, _cmd(0.10), 1)                               # 0.10은 스텝 6에 도착
    assert out["t"] == 0.08 and j.stale == 1
    j.inflight.append((6, _cmd(0.04)))                              # 늦게 온 오래된 명령: 버림
    out = j.deliver(6, _cmd(0.12), 1)
    assert out["t"] == 0.10 and j.stale == 2
    j.inflight = [(7, _cmd(0.02))]
    assert j.deliver(7, _cmd(0.14), 1) is None                      # 실행 중인 것보다 오래됨 -> 마지막 명령 유지


def _run(scn, seconds=6.0):
    sim = Simulation(scn)
    ages = []
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < seconds:
            sim.step_control()
            if "t" in sim.low_cmd:
                ages.append(round((sim.low_state["t"] - sim.low_cmd["t"]) / 0.02))
    return sim, np.bincount(ages)


def test_simulation_seeded_and_ages_match():
    scn = copy.deepcopy(BASE)
    scn["control_link"] = {"latency_steps": 1, "extra_steps_probs": [0.05, 0.02], "drop_probability": 0.03}
    s1, a1 = _run(scn)
    s2, a2 = _run(scn)
    assert np.array_equal(s1.data.qpos, s2.data.qpos) and a1.tolist() == a2.tolist()
    j = s1.link_jitter
    assert j.late > 0 and j.dropped > 0 and j.stale > 0
    assert a1[0] == 0 and a1[1] > 0.85 * a1.sum() and len(a1) > 2   # 대부분 한 주기, 가끔 더 오래된 명령
    s0, _ = _run(BASE)
    assert not np.array_equal(s0.data.qpos, s1.data.qpos)           # 지연이 실제로 움직임을 바꾼다


def test_rt_core_rejects_jitter():
    from sim.rt_link import run
    scn = copy.deepcopy(BASE)
    scn["control_link"] = {"extra_steps_probs": [0.01]}
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            run(scn, lockstep=True, verbose=False)
    except SystemExit as e:
        assert "sim.runner lockstep 전용" in str(e)
        return
    raise AssertionError("실시간 코어 경로가 통계적 지연 설정을 받았다")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
