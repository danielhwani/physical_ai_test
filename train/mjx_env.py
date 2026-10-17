"""Go2 MJX 강화학습 환경 (평지 걷기, 실험적). conda 환경 mjx에서 쓴다 (jax, mujoco-mjx, brax).

배치 규약을 그대로 따른다 (README "강화학습 준비"):
  모델     명세의 mjx 설정 모델 (sim/model_builder.py robot_spec(variant="mjx")) + 평지
  행동     q_des = default_pose + scale(0.25) * clip(action, ±4), PD는 물리 스텝마다 (명세 action 절. kp/kd는 이 파일의 KP/KD -> 카드에 적음)
           정책 출력(−1~1, Brax PPO의 tanh) × ACT_GAIN = 카드 단위 action (ONNX도 같은 곱을 한다)
  actor    배치용 관측 observation 47개 (control/observation.py와 같은 식. obs_actor는 obs_reference.npz와 1e-5 안에서 같다)
           + 배치 흉내: 관측 잡음, 연결 지연 0~2주기 무작위, 명령 가속 제한과 보행 위상 (control/clock.py GaitClock과 같은 규칙)
  critic   특권 정보 추가 (비대칭 actor-critic): 잡음 없는 관측 + 참 몸통 속도(몸통 좌표) + 몸통 높이 + 발 접촉
보상은 legged_gym 계열의 단순한 묶음 (속도 추종, 자세·진동 벌점, 발 공중 시간). 지형·마찰 무작위화는 아직 없다.
"""
import functools
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
import yaml
from brax.envs.base import Env, State
from mujoco import mjx

ROOT = Path(__file__).resolve().parent.parent
SPEC = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())

KP, KD = 40.0, 1.0                 # 학습 정책용 PD (명세의 규칙 기반 보행기용 100~150/4보다 낮게)
ACT_GAIN = 2.0                     # 정책 출력(−1~1) -> 카드 단위 action
GAIT_PERIOD = 0.36                 # 보행 위상 주기 (카드 gait_clock.period)
ACCEL_LIMIT = 0.5                  # 명령 가속 제한 (control/clock.py GaitClock 기본값)
DECIM = 10                         # 제어 주기당 물리 스텝 (20 ms / 2 ms)
MAX_LATENCY = 2                    # 연결 지연 0..2 주기 무작위
EPISODE_STEPS = 1000               # 20 s


def build_mj_model():
    """명세의 mjx 설정 Go2 + 평지 (heightfield 대신 평면)."""
    import sys
    sys.path.insert(0, str(ROOT))
    from sim.model_builder import robot_spec
    s = robot_spec(SPEC, variant="mjx")
    s.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], friction=[0.8, 0.02, 0.01])
    return s.compile()


def quat_rotate_inverse(q, v):
    """R(q)^T @ v, q = (w, x, y, z). control/observation.py와 같은 전개."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    R = jnp.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])
    return R.T @ v


_OBS = SPEC["observation"]
_SCALES = {t["name"]: jnp.asarray(t["scale"], jnp.float32) for t in _OBS["terms"]}
DEFAULT_POSE = jnp.asarray(SPEC["robot"]["default_pose"], jnp.float32)


def obs_actor(qpos, qvel, command, last_action, gait_phase):
    """배치용 관측 observation (47). 입력은 control/observation.py ObservationSpec.compute와 같다."""
    g = quat_rotate_inverse(qpos[3:7], jnp.array([0.0, 0.0, -1.0]))
    parts = [qvel[3:6] * _SCALES["base_ang_vel"], g * _SCALES["projected_gravity"], command * _SCALES["command"],
             (qpos[7:19] - DEFAULT_POSE) * _SCALES["joint_pos_rel"], qvel[6:18] * _SCALES["joint_vel"],
             last_action * _SCALES["last_action"],
             jnp.array([jnp.sin(2 * jnp.pi * gait_phase), jnp.cos(2 * jnp.pi * gait_phase)]) * _SCALES["gait_phase"]]
    return jnp.clip(jnp.concatenate(parts), -_OBS["clip"], _OBS["clip"])


class Go2FlatEnv(Env):
    def __init__(self, obs_noise=1.0):
        self.mj_model = build_mj_model()
        self.mjx_model = mjx.put_model(self.mj_model)
        m = self.mj_model
        self.foot_ids = jnp.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f) for f in SPEC["robot"]["feet"]])
        self.foot_r = float(m.geom_size[int(self.foot_ids[0])][0])
        self.base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, SPEC["robot"]["base_body"])
        self.qpos0 = jnp.asarray(m.key_qpos[0])
        self.torque_limit = jnp.asarray(SPEC["action"]["torque_limit"], jnp.float32)
        self.obs_noise = obs_noise
        self.dt = DECIM * m.opt.timestep

    @property
    def observation_size(self):
        return {"state": (47,), "privileged_state": (55,)}

    @property
    def action_size(self):
        return 12

    @property
    def backend(self):
        return "mjx"

    def _sample_command(self, rng):
        k1, k2, k3 = jax.random.split(rng, 3)
        vx = jax.random.uniform(k1, (), minval=-0.2, maxval=0.8)
        wz = jax.random.uniform(k2, (), minval=-0.6, maxval=0.6)
        still = jax.random.uniform(k3, ()) < 0.1                       # 10 %는 제자리
        return jnp.where(still, jnp.zeros(2), jnp.array([vx, wz]))

    def reset(self, rng):
        rng, k1, k2, k3 = jax.random.split(rng, 4)
        qpos = self.qpos0.at[7:].add(jax.random.uniform(k1, (12,), minval=-0.1, maxval=0.1))
        data = mjx.make_data(self.mjx_model).replace(qpos=qpos, qvel=jnp.zeros(self.mjx_model.nv))
        data = mjx.forward(self.mjx_model, data)
        info = {"rng": rng, "cmd": self._sample_command(k2), "cmd_f": jnp.zeros(2), "phase": jnp.zeros(()),
                "last_action": jnp.zeros(12), "hist": jnp.zeros((MAX_LATENCY + 1, 12)),
                "latency": jax.random.randint(k3, (), 0, MAX_LATENCY + 1), "step": jnp.zeros((), jnp.int32),
                "air_time": jnp.zeros(4), "last_contact": jnp.ones(4, bool)}
        obs = self._obs(data, info, jax.random.PRNGKey(0), noise=False)
        metrics = {"vx": jnp.zeros(()), "track": jnp.zeros(())}
        return State(data, obs, jnp.zeros(()), jnp.zeros(()), metrics, info)

    def _contacts(self, data):
        return data.geom_xpos[self.foot_ids, 2] - self.foot_r < 0.005   # 평지: 발 구 바닥 높이

    def _obs(self, data, info, rng, noise=True):
        cmd3 = jnp.array([info["cmd_f"][0], 0.0, info["cmd_f"][1]])
        clean = obs_actor(data.qpos, data.qvel, cmd3, info["last_action"], info["phase"])
        # 배치 관측은 센서 잡음 + 상태 추정: 자이로·자세·관절에 잡음 (크기는 sensors/proprio.py 정도)
        sigma = jnp.concatenate([jnp.full(3, 0.05), jnp.full(3, 0.03), jnp.zeros(3), jnp.full(12, 0.01), jnp.full(12, 0.1),
                                 jnp.zeros(12), jnp.zeros(2)]) * self.obs_noise
        actor = clean + jnp.where(noise, sigma * jax.random.normal(rng, (47,)), 0.0)
        R_v = quat_rotate_inverse(data.qpos[3:7], data.qvel[0:3])     # 몸통 좌표 선속도 (참값)
        priv = jnp.concatenate([clean, R_v * 2.0, data.qpos[2:3] - 0.27, self._contacts(data).astype(jnp.float32)])
        return {"state": actor, "privileged_state": priv}

    def step(self, state, action):
        info = dict(state.info)
        rng, k_obs, k_cmd = jax.random.split(info["rng"], 3)
        a_card = jnp.clip(ACT_GAIN * action, -SPEC["action"]["clip"], SPEC["action"]["clip"])
        hist = jnp.concatenate([a_card[None], info["hist"][:-1]])     # hist[0] = 이번, hist[L] = L주기 전
        executed = hist[info["latency"]]                               # 연결 지연 (실제 로봇: 지난 명령 실행)
        q_des = DEFAULT_POSE + SPEC["action"]["scale"] * executed

        def substep(_, d):
            tau = jnp.clip(KP * (q_des - d.qpos[7:]) - KD * d.qvel[6:], -self.torque_limit, self.torque_limit)
            return mjx.step(self.mjx_model, d.replace(ctrl=tau))
        data = jax.lax.fori_loop(0, DECIM, substep, state.pipeline_state)

        # 명령: 가속 제한 + 보행 위상 (GaitClock), 10초마다 새 명령
        a = ACCEL_LIMIT * self.dt
        cmd_f = info["cmd_f"] + jnp.clip(info["cmd"] - info["cmd_f"], -a, a)
        moving = jnp.abs(cmd_f).max() > 1e-3
        phase = jnp.where(moving, (info["phase"] + self.dt / GAIT_PERIOD) % 1.0, info["phase"])
        step = info["step"] + 1
        cmd = jnp.where(step % 500 == 0, self._sample_command(k_cmd), info["cmd"])

        # 보상
        v_body = quat_rotate_inverse(data.qpos[3:7], data.qvel[0:3])
        w_body = data.qvel[3:6]
        g = quat_rotate_inverse(data.qpos[3:7], jnp.array([0.0, 0.0, -1.0]))
        contact = self._contacts(data)
        first = contact & ~info["last_contact"]
        air = info["air_time"] + self.dt
        r_air = jnp.sum((air - 0.12) * first) * (jnp.abs(cmd_f[0]) > 0.05)
        air = jnp.where(contact, 0.0, air)
        err_v = jnp.sum(jnp.square(jnp.array([cmd_f[0], 0.0]) - v_body[:2]))
        r = (1.0 * jnp.exp(-err_v / 0.25) + 0.5 * jnp.exp(-jnp.square(cmd_f[1] - w_body[2]) / 0.25)
             - 2.0 * jnp.square(v_body[2]) - 0.05 * jnp.sum(jnp.square(w_body[:2])) - 2.0 * jnp.sum(jnp.square(g[:2]))
             - 2e-4 * jnp.sum(jnp.square(data.ctrl)) - 0.01 * jnp.sum(jnp.square(a_card - info["last_action"]))
             + 0.5 * r_air
             - 0.5 * jnp.sum(jnp.square(data.qpos[7:] - DEFAULT_POSE)) * (jnp.abs(cmd_f).max() < 0.05))
        fallen = (data.qpos[2] < 0.15) | (g[2] > -0.5)
        done = fallen | (step >= EPISODE_STEPS)
        r = jnp.where(fallen, -2.0, r) * self.dt

        info.update(rng=rng, cmd=cmd, cmd_f=cmd_f, phase=phase, last_action=a_card, hist=hist, step=step,
                    air_time=air, last_contact=contact)
        obs = self._obs(data, info, k_obs)
        metrics = dict(state.metrics)              # Brax 감싸개가 넣은 항목은 그대로 두고 갱신
        metrics.update(vx=v_body[0], track=jnp.exp(-err_v / 0.25))
        return state.replace(pipeline_state=data, obs=obs, reward=r, done=done.astype(jnp.float32), metrics=metrics, info=info)
