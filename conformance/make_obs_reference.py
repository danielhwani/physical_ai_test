"""관측 기준 벡터 생성 (문서 §12.3 "기준 벡터 시험").

    python conformance/make_obs_reference.py

conformance/reference/obs_reference.npz 에 (qpos, qvel, command, last_action, gait_phase) 입력과
control/observation.py 기준 구현의 출력(obs)을 저장한다. JAX/C++ 구현은 같은 입력으로 계산해
명세의 tolerance 안에서 obs와 일치해야 한다.

입력 상태 구성:
  - 실제 보행 궤적 샘플 (rough_rut, 두 모델 변형)  -> 현실적인 값 범위
  - 무작위 극단 상태 (임의 자세, 큰 속도, clip 경계)  -> 부호/순서/clip 실수 검출
명세의 observation 절이 바뀌면 spec_hash가 달라져 시험이 재생성을 요구한다.
"""
import contextlib
import hashlib
import io
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from control.interface import ControlInterface  # noqa: E402
from control.observation import INPUT_KEYS  # noqa: E402
from sim.runner import Simulation, load_yaml  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "conformance/reference/obs_reference.npz"


def spec_hash(spec):
    section = {k: spec[k] for k in ("observation",)}
    section["default_pose"] = spec["robot"]["default_pose"]
    return hashlib.sha256(json.dumps(section, sort_keys=True).encode()).hexdigest()[:16]


def trajectory_samples(variant, every=10):
    """제어 주기마다 runner가 관측 계산에 넘긴 입력을 그대로 가로채 저장."""
    sim = Simulation(ROOT / "scenarios/rough_rut.yaml", variant=variant)
    rows, k = [], 0
    compute = sim.iface.obs.compute

    def capture(**inp):
        nonlocal k
        if k % every == 0:
            rows.append(tuple(np.array(inp[key], dtype=float).copy() for key in INPUT_KEYS))
        k += 1
        return compute(**inp)

    sim.iface.obs.compute = capture
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < 10.0:
            sim.step_control()
    return rows


def random_samples(n, rng):
    rows = []
    for _ in range(n):
        qpos = np.zeros(19); qvel = np.zeros(18)
        qpos[:3] = rng.uniform(-5, 5, 3)
        q = rng.normal(size=4); qpos[3:7] = q / np.linalg.norm(q)        # 임의 자세 (뒤집힘 포함)
        qpos[7:] = rng.uniform(-3, 3, 12)
        qvel[:] = rng.normal(scale=[2] * 3 + [10] * 3 + [40] * 12)        # 큰 관절 속도 -> 일부 clip
        cmd = rng.uniform([-1, -0.5, -1.5], [1, 0.5, 1.5])
        act = rng.uniform(-4, 4, 12)
        rows.append((qpos, qvel, cmd, act, np.array(rng.uniform(0, 1))))
    # clip 경계 확인용: 관절 속도 극단값
    qpos = np.zeros(19); qpos[3] = 1.0; qvel = np.zeros(18); qvel[6:] = 5000.0
    rows.append((qpos, qvel, np.zeros(3), np.zeros(12), np.array(0.25)))
    return rows


def main():
    rng = np.random.default_rng(42)
    spec = load_yaml(ROOT / "specs/go2_control.yaml")
    obs_spec = ControlInterface(spec).obs
    rows = trajectory_samples("cpu") + trajectory_samples("mjx") + random_samples(200, rng)
    inputs = {key: np.array(col) for key, col in zip(INPUT_KEYS, zip(*rows))}
    obs = np.array([obs_spec.compute(*r) for r in rows])
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT, **inputs, obs=obs,
                        spec_hash=np.array(spec_hash(spec)),
                        term_names=np.array([t["name"] for t in obs_spec.terms]))
    print(f"{OUT.relative_to(ROOT)}: {len(rows)} samples, obs dim {obs.shape[1]}, "
          f"clip 발생 샘플 {int((np.abs(obs) >= obs_spec.clip).any(axis=1).sum())}개")


if __name__ == "__main__":
    main()
