"""명세 적합성 시험: 관측 기준 구현과 MJX 오버라이드.

    python conformance/test_specs.py      (pytest로도 실행 가능)
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from conformance.make_obs_reference import OUT as REF_PATH, spec_hash  # noqa: E402
from control.interface import ControlInterface  # noqa: E402
from sim.model_builder import COLLISION_GROUP, MJX_OVERRIDE, robot_spec  # noqa: E402
from control.observation import INPUT_KEYS  # noqa: E402

SPEC = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())
OBS = ControlInterface(SPEC).obs


def _inputs(ref, i, cast=lambda a: a):
    return {k: cast(ref[k][i]) for k in INPUT_KEYS}


def _ref():
    ref = np.load(REF_PATH)
    assert str(ref["spec_hash"]) == spec_hash(SPEC), \
        "명세 observation 절이 바뀌었다. 의도한 변경이면 conformance/make_obs_reference.py 를 다시 실행"
    return ref


# ---------- 관측 ----------

def test_obs_matches_reference():
    """기준 구현 회귀 시험. JAX/C++ 구현도 이 함수와 같은 방식으로 비교하면 된다."""
    ref = _ref()
    tol = OBS.tolerance
    for i in range(len(ref["obs"])):
        got = OBS.compute(**_inputs(ref, i))
        if not np.allclose(got, ref["obs"][i], **tol):
            bad = [n for n, s in OBS.slices().items() if not np.allclose(got[s], ref["obs"][i][s], **tol)]
            raise AssertionError(f"sample {i}: 불일치 항목 {bad}")


def test_obs_float32_within_tolerance():
    """MJX 기본 float32 계산을 흉내 내도 명세 허용 오차 안에 드는지 (허용 오차가 현실적인지 확인)."""
    ref = _ref()
    f32 = lambda a: np.asarray(a, dtype=np.float32)  # noqa: E731
    for i in range(len(ref["obs"])):
        got = OBS.compute(**_inputs(ref, i, f32)).astype(np.float32)
        assert np.allclose(got, ref["obs"][i], **OBS.tolerance), i


def test_obs_definitions_match_mujoco():
    """명세의 정의(def)가 MuJoCo가 계산한 물리량과 같은지 독립 확인.
    - projected_gravity: mju_quat2Mat 기반 회전
    - base_ang_vel: imu site의 로컬 각속도 (실기 gyro에 해당)"""
    m = robot_spec(SPEC).compile()
    d = mujoco.MjData(m)
    imu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu")
    sl = OBS.slices()
    scale = {t["name"]: np.asarray(t["scale"], dtype=float) for t in OBS.terms}
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4); d.qpos[3:7] = q / np.linalg.norm(q)
        d.qpos[7:] = rng.uniform(-1, 1, 12)
        d.qvel[:] = rng.normal(size=18)
        mujoco.mj_forward(m, d)
        obs = OBS.compute(d.qpos, d.qvel, np.zeros(3), np.zeros(12), 0.0)

        R = np.zeros(9); mujoco.mju_quat2Mat(R, d.qpos[3:7])
        g = R.reshape(3, 3).T @ [0, 0, -1]
        assert np.allclose(obs[sl["projected_gravity"]] / scale["projected_gravity"], g, atol=1e-9)

        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, imu, vel, 1)   # [ang, lin], 로컬 좌표
        assert np.allclose(obs[sl["base_ang_vel"]] / scale["base_ang_vel"], vel[:3], atol=1e-9)


def test_card_joint_reordering():
    """정책 카드가 다른 관절 순서를 쓰면 관측/행동이 정확히 재배열되는지 (2번 경우: 카드만으로 흡수)."""
    order = SPEC["robot"]["joints"]
    perm = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]                  # 예: FR, FL, RR, RL 순서 정책
    card = {"joints": [order[i] for i in perm],
            "default_pose": [SPEC["robot"]["default_pose"][i] for i in perm],
            "action": {"pd": {"kp": [SPEC["action"]["pd"]["kp"][i] for i in perm],
                              "kd": [SPEC["action"]["pd"]["kd"][i] for i in perm]}}}
    base, re = ControlInterface(SPEC), ControlInterface(SPEC, card)
    rng = np.random.default_rng(1)
    qpos, qvel = np.r_[0, 0, 0.3, 1, 0, 0, 0, rng.normal(size=12)], rng.normal(size=18)
    a = rng.uniform(-1, 1, 12)
    o1, o2 = base.obs.compute(qpos, qvel, np.zeros(3), a, 0.3), re.obs.compute(qpos, qvel, np.zeros(3), a[perm], 0.3)
    for name in ("joint_pos_rel", "joint_vel", "last_action"):
        sl = base.obs.slices()[name]
        assert np.allclose(o1[sl][perm], o2[sl]), name
    assert np.allclose(base.targets_from_action(a), re.targets_from_action(a[perm]))   # 모델 순서 목표각 동일
    assert np.allclose(base.kp, re.kp) and np.allclose(base.kd, re.kd)


# ---------- MJX 오버라이드 ----------

def _collision(m):
    out = []
    for g in range(m.ngeom):
        if m.geom_group[g] == COLLISION_GROUP:
            out.append((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]),
                        int(m.geom_type[g]), m.geom_size[g].copy(), m.geom_pos[g].copy(),
                        m.geom_solimp[g].copy()))
    return out


def test_mjx_override_reproduces_upstream():
    """오버라이드를 적용한 모델의 adopt 항목이 Menagerie go2_mjx.xml과 일치하는지."""
    ours = robot_spec(SPEC, "mjx").compile()
    ref_path = yaml.safe_load(MJX_OVERRIDE.read_text())["reference_model"]
    up = mujoco.MjModel.from_xml_path(str(ROOT / ref_path))

    for f in ("cone", "iterations", "ls_iterations", "disableflags", "impratio", "timestep"):
        assert getattr(ours.opt, f) == getattr(up.opt, f), f"option.{f}"
    assert np.allclose(ours.dof_frictionloss, up.dof_frictionloss)

    a, b = _collision(ours), _collision(up)
    assert len(a) == len(b)
    for x, y in zip(a, b):
        assert x[0] == y[0] and x[1] == y[1], (x[0], x[1], y[1])
        for k, what in ((2, "size"), (3, "pos"), (4, "solimp")):
            assert np.allclose(x[k], y[k], atol=1e-6), (x[0], what, x[k], y[k])


def test_mjx_override_keeps_rejected_items():
    """reject 항목(실기 성능 관련)은 원본 그대로 남아 있어야 한다."""
    cpu, mjx = robot_spec(SPEC, "cpu").compile(), robot_spec(SPEC, "mjx").compile()
    assert np.array_equal(cpu.jnt_range, mjx.jnt_range)                 # 뒷다리 hip 범위
    assert np.array_equal(cpu.actuator_gainprm, mjx.actuator_gainprm)   # 토크 모터 유지
    assert np.array_equal(cpu.actuator_biastype, mjx.actuator_biastype)
    assert np.array_equal(cpu.actuator_ctrlrange, mjx.actuator_ctrlrange)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
