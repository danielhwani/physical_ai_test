"""적합성 시험 (문서 §12.3). 실행: python -m pytest conformance -q  또는  python conformance/test_conformance.py

1. 결정성: 같은 시나리오/시드를 두 번 돌리면 상태가 비트 단위로 같아야 한다 (CPU MuJoCo).
2. 명세-모델 일치: specs의 다리 치수로 푼 IK가 MJCF 순기구학과 일치해야 한다.
   (명세 파일과 모델이 따로 놀기 시작하면 여기서 잡힌다.)
"""
import contextlib
import io
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from control.trot import leg_ik  # noqa: E402
from sim.runner import Simulation  # noqa: E402

SCENARIO = Path(__file__).resolve().parent.parent / "scenarios/rough_rut.yaml"


def _run(seconds):
    sim = Simulation(SCENARIO)
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < seconds:
            sim.step_control()
    return sim.data.qpos.copy(), sim.data.qvel.copy()


def test_determinism():
    (q1, v1), (q2, v2) = _run(5.0), _run(5.0)
    assert np.array_equal(q1, q2) and np.array_equal(v1, v2)


def test_ik_matches_mjcf():
    sim = Simulation(SCENARIO)
    m, d = sim.model, sim.data
    leg = sim.spec["robot"]["leg"]
    rng = np.random.default_rng(0)
    for _ in range(50):
        # 몸통을 원점에 고정하고 무작위 시상면 관절각 적용
        d.qpos[:7] = [0, 0, 1, 1, 0, 0, 0]
        thigh, calf = rng.uniform(0.3, 1.5), rng.uniform(-2.5, -1.0)
        d.qpos[7:] = np.tile([0.0, thigh, calf], 4)
        mujoco.mj_kinematics(m, d)
        for i, g in enumerate(sim.foot_ids):
            hip = m.body(f"{sim.spec['robot']['feet'][i]}_thigh").id
            rel = d.geom_xpos[g] - d.xpos[hip]          # thigh 관절 원점 기준 발 중심
            q1, q2 = leg_ik(rel[0], rel[2], leg["thigh_length"], leg["calf_length"])
            # foot geom의 x 오프셋(-2 mm)만큼 허용
            assert abs(q1 - thigh) < 0.02 and abs(q2 - calf) < 0.02, (i, thigh, calf, q1, q2)


if __name__ == "__main__":
    test_determinism(); print("determinism: OK")
    test_ik_matches_mjcf(); print("ik vs mjcf: OK")
