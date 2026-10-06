"""보행 알고리즘 분리 시험 (실제 로봇과 같은 경계).

    python conformance/test_control.py

1. 알고리즘(control/)은 시뮬레이터·물리엔진·렌더러를 모른다 (sim, mujoco, cosim, viz를 불러오지 않음)
2. 로봇 상태 메시지에는 로봇이 아는 값만 있다 (몸통 위치·속도 같은 참값 없음)
3. 알고리즘 쪽 다리 기구학(명세 숫자만) = MuJoCo 모델의 발 위치
4. 상태 추정 정확도: 평지에서 다리 주행거리계 속도 편향이 작다 (발 구름 보정 포함)
5. 재현: 같은 프로세스 실행 때 알고리즘이 받은 로봇 상태 메시지만 JSON으로 기록해 새 알고리즘 노드에 넣으면
   같은 관절 명령이 나온다 (참값을 따로 읽지 않는다는 실행 증거)
6. 센서 잡음: 같은 시드는 같은 값, 다른 시드는 다른 값
"""
import contextlib
import copy
import io
import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from control.kinematics import LegKinematics  # noqa: E402
from control.node import ControllerNode  # noqa: E402
from sim.runner import Simulation  # noqa: E402

SPEC = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())


def _load(name):
    return yaml.safe_load((ROOT / f"scenarios/{name}.yaml").read_text())


def test_controller_does_not_import_simulator():
    code = ("import sys; import control.node; "
            "print(sorted({m.split('.')[0] for m in sys.modules} & {'mujoco', 'sim', 'cosim', 'viz', 'pychrono'}))")
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "[]", f"알고리즘이 시뮬레이터 쪽 모듈을 불러옴: {out.stdout}"


def test_low_state_has_only_robot_known_values():
    sim = Simulation(_load("flat_trot"))
    try:
        ls = sim.robot.read()
        assert set(ls) == {"t", "q", "dq", "imu", "foot_force"}, set(ls)
        assert set(ls["imu"]) == {"quat", "gyro", "accel"}, set(ls["imu"])
        assert len(ls["q"]) == len(ls["dq"]) == 12 and len(ls["foot_force"]) == 4
    finally:
        sim.close()


def test_kinematics_matches_mujoco():
    sim = Simulation(_load("flat_trot"))
    m, d = sim.model, sim.data
    kin = LegKinematics(SPEC)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(50):
        d.qpos[7:] = np.tile([rng.uniform(-0.5, 0.5), rng.uniform(0.2, 1.6), rng.uniform(-2.5, -1.0)], 4)
        mujoco.mj_kinematics(m, d)
        R, p = d.xmat[sim.base_id].reshape(3, 3), d.xpos[sim.base_id]
        ref = np.array([R.T @ (d.geom_xpos[g] - p) for g in sim.foot_ids])       # 몸통 좌표의 발 중심
        worst = max(worst, float(np.abs(kin.feet(d.qpos[7:]) - ref).max()))
    sim.close()
    assert worst < 1e-6, f"기구학 오차 {worst:.2e} m"


def test_leg_odometry_on_flat_ground():
    sim = Simulation(_load("flat_trot"))
    err, a, tf = [], 0.3, None
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < 8.0:
                sim.step_control()
                d = sim.data
                v_true = (d.xmat[sim.base_id].reshape(3, 3).T @ d.qvel[:3])[0]
                tf = v_true if tf is None else tf + a * (v_true - tf)      # 추정기와 같은 저역통과를 거친 참값
                if d.time > 2.0:
                    err.append(sim.controller.est.v_body[0] - tf)
    finally:
        sim.close()
    err = np.array(err)
    # 발 구름 보정 전에는 편향 약 -0.038 m/s (발 중심 고정 가정). 보정 후 0.01 m/s 이내
    assert abs(err.mean()) < 0.01 and err.std() < 0.05, (err.mean(), err.std())


def test_replay_from_low_state_messages_only():
    states, cmds = [], []
    sim = Simulation(_load("rough_rut"))
    orig = sim.controller.step
    def logged_step(ls):
        states.append(json.dumps(ls))
        cmds.append(orig(ls))
        return cmds[-1]
    sim.controller.step = logged_step
    events = []
    orig_cmd = sim.controller.set_command
    sim.controller.set_command = lambda vx, wz: (events.append((len(states), vx, wz)), orig_cmd(vx, wz))[1]
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            while sim.data.time < 5.0:
                sim.step_control()
    finally:
        sim.close()
    fresh = ControllerNode(SPEC, _load("rough_rut")["controller"])
    replay = []
    for k, s in enumerate(states):
        for n, vx, wz in events:
            if n == k:
                fresh.set_command(vx, wz)
        replay.append(fresh.step(json.loads(s)))
    assert len(replay) == len(cmds) > 200
    assert all(a["q_des"] == b["q_des"] for a, b in zip(replay, cmds)), "로봇 상태 메시지만으로 재현되지 않음"


def test_sensor_noise_is_seeded():
    def first_states(seed):
        scn = copy.deepcopy(_load("flat_trot"))
        scn["seed"] = seed
        sim = Simulation(scn)
        out = [sim.robot.read() for _ in range(3)]
        sim.close()
        return out

    a, b, c = first_states(1), first_states(1), first_states(2)
    assert a == b, "같은 시드인데 측정값이 다름"
    assert a[0]["imu"]["gyro"] != c[0]["imu"]["gyro"], "시드가 달라도 측정값이 같음"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
