"""Go2 평지 걷기 MJX 강화학습 (Brax PPO, 비대칭 actor-critic) -> ONNX + 정책 카드. 실험적. conda 환경 mjx에서 실행한다.

    JAX_PLATFORMS=cpu python -m train.train_mjx --smoke                  # RT 커널(GPU 없음): 전체 경로가 도는지만 (몇 분)
    python -m train.train_mjx --envs 1024 --steps 20000000 --name go2_mjx_flat   # 일반 커널 + GPU

결과: policies/<이름>/{policy.onnx, card.yaml, train_log.json}. 그다음 mj_ros 환경에서
    python conformance/make_policy_reference.py policies/<이름>/card.yaml    # 입출력 기준 (test_policy.py용)
    python -m sim.runner scenarios/flat_trot.yaml --policy policies/<이름>/card.yaml --rviz
ONNX 그래프: 관측 정규화 (x - 평균) / 표준편차 -> MLP(silu) -> tanh -> × ACT_GAIN = 카드 단위 action. 저장 전에 Brax 추론 함수와
같은 출력인지 확인한다.
"""
import argparse
import datetime as dt
import functools
import json
import sys
import time
from pathlib import Path

import jax
import jax.numpy as jnp
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from train import mjx_env as E  # noqa: E402

HIDDEN = (128, 128, 128)          # 정책 MLP (배치 추론이 가벼워야 한다)
VALUE_HIDDEN = (256, 256, 256)


def export_onnx(params, path, obs_dim=47, act_dim=12):
    """Brax PPO 정책 -> ONNX (관측 정규화 포함). 반환: (평균, 표준편차, 층 목록)."""
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    normalizer, policy = params[0], params[1]
    mean = np.asarray(normalizer.mean["state"], np.float32)
    std = np.asarray(normalizer.std["state"], np.float32)
    p = policy["params"]
    names = sorted([k for k in p if k.startswith("hidden_")], key=lambda k: int(k.split("_")[1]))
    layers = [(np.asarray(p[k]["kernel"], np.float32), np.asarray(p[k]["bias"], np.float32)) for k in names]
    inits = [numpy_helper.from_array(mean, "mean"), numpy_helper.from_array(std, "std"),
             numpy_helper.from_array(np.array(E.ACT_GAIN, np.float32), "gain")]
    nodes = [helper.make_node("Sub", ["obs", "mean"], ["x0a"]), helper.make_node("Div", ["x0a", "std"], ["x0"])]
    x = "x0"
    for i, (W, b) in enumerate(layers):
        inits += [numpy_helper.from_array(W, f"W{i}"), numpy_helper.from_array(b, f"b{i}")]
        nodes += [helper.make_node("MatMul", [x, f"W{i}"], [f"m{i}"]), helper.make_node("Add", [f"m{i}", f"b{i}"], [f"z{i}"])]
        if i < len(layers) - 1:                                   # silu = x * sigmoid(x)
            nodes += [helper.make_node("Sigmoid", [f"z{i}"], [f"s{i}"]), helper.make_node("Mul", [f"z{i}", f"s{i}"], [f"h{i}"])]
            x = f"h{i}"
        else:
            x = f"z{i}"
    if layers[-1][0].shape[1] != act_dim:                         # (loc, scale) 출력이면 loc만
        inits += [numpy_helper.from_array(np.array([0], np.int64), "loc_start"),
                  numpy_helper.from_array(np.array([act_dim], np.int64), "loc_end"),
                  numpy_helper.from_array(np.array([1], np.int64), "loc_axis")]
        nodes.append(helper.make_node("Slice", [x, "loc_start", "loc_end", "loc_axis"], ["loc"]))
        x = "loc"
    nodes += [helper.make_node("Tanh", [x], ["t"]), helper.make_node("Mul", ["t", "gain"], ["actions"])]
    graph = helper.make_graph(nodes, "go2_mjx_policy", [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, obs_dim])],
                              [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, act_dim])], inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, str(path))
    return mean, std, layers


def check_onnx(path, inference_fn, n=64, seed=0):
    """ONNX 출력 = Brax 결정적 추론 × ACT_GAIN (무작위 관측으로)."""
    import onnxruntime as ort
    sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(seed)
    worst = 0.0
    for _ in range(n):
        obs = rng.normal(0, 1, (1, 47)).astype(np.float32)
        a_onnx = sess.run(["actions"], {"obs": obs})[0][0]
        a_brax, _ = inference_fn({"state": jnp.asarray(obs[0]), "privileged_state": jnp.zeros(55)}, jax.random.PRNGKey(0))
        worst = max(worst, float(np.abs(a_onnx - E.ACT_GAIN * np.asarray(a_brax)).max()))
    return worst


def write_card(out, name, info):
    base = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())
    card = {
        "name": name, "onnx": "policy.onnx", "io": {"input": "obs", "output": "actions"},
        "joints": base["robot"]["joints"], "default_pose": base["robot"]["default_pose"],
        "action": {"scale": base["action"]["scale"], "clip": base["action"]["clip"], "control_dt": base["action"]["control_dt"],
                   "pd": {"kp": [E.KP] * 12, "kd": [E.KD] * 12, "rate": "physics"}},
        "observation": base["observation"],
        "gait_clock": {"period": E.GAIT_PERIOD},
        "provenance": {"method": "Brax PPO, asymmetric actor-critic (critic: true body velocity, base height, foot contact)",
                       "observation_source": "training: true state + noise, link latency 0-2 steps; deploy: estimator",
                       "link_latency_steps_range": [0, E.MAX_LATENCY], "spec_version": base["spec_version"],
                       "date": dt.date.today().isoformat(), "trained_on": "MJX, model variant mjx, flat plane",
                       "network": f"MLP 47-{'-'.join(map(str, HIDDEN))}-12 silu, tanh x {E.ACT_GAIN}, obs normalization in graph",
                       **info},
    }
    (out / "card.yaml").write_text("# 자동 생성: train/train_mjx.py (실험적)\n" + yaml.safe_dump(card, allow_unicode=True, sort_keys=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true", help="아주 작게 (CPU에서 경로 확인)")
    ap.add_argument("--envs", type=int, default=1024)
    ap.add_argument("--steps", type=int, default=20_000_000)
    ap.add_argument("--name", default="go2_mjx_flat")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    from brax.training.agents.ppo import networks as ppo_networks
    from brax.training.agents.ppo import train as ppo

    if a.smoke:
        a.envs, a.steps, a.name = 16, 16 * 20 * 4 * 2, a.name + "_smoke"
    cfg = dict(num_timesteps=a.steps, num_envs=a.envs, episode_length=E.EPISODE_STEPS, action_repeat=1, unroll_length=20,
               batch_size=max(a.envs // 4, 4), num_minibatches=4, num_updates_per_batch=4, learning_rate=3e-4,
               entropy_cost=1e-2, discounting=0.97, reward_scaling=1.0, normalize_observations=True, num_evals=3 if a.smoke else 10,
               num_eval_envs=min(a.envs, 128), seed=a.seed, max_grad_norm=1.0, clipping_epsilon=0.2)
    env = E.Go2FlatEnv()
    net = functools.partial(ppo_networks.make_ppo_networks, policy_hidden_layer_sizes=HIDDEN, value_hidden_layer_sizes=VALUE_HIDDEN,
                            policy_obs_key="state", value_obs_key="privileged_state")
    log, t0 = [], time.time()

    def progress(step, metrics):
        row = {"step": int(step), "wall_s": round(time.time() - t0, 1),
               "reward": float(metrics.get("eval/episode_reward", np.nan)),
               "vx": float(metrics.get("eval/episode_vx", np.nan)), "track": float(metrics.get("eval/episode_track", np.nan))}
        log.append(row)
        print(json.dumps(row), flush=True)

    print(f"devices {jax.devices()}  envs {a.envs}  steps {a.steps}", flush=True)
    make_inference, params, _ = ppo.train(environment=env, network_factory=net, progress_fn=progress, **cfg)
    wall = time.time() - t0
    out = ROOT / "policies" / a.name
    out.mkdir(parents=True, exist_ok=True)
    import pickle
    (out / "params.pkl").write_bytes(pickle.dumps(jax.device_get(params)))   # 내보내기를 다시 할 때 (학습 없이)
    export_onnx(params, out / "policy.onnx")
    worst = check_onnx(out / "policy.onnx", jax.jit(make_inference(params, deterministic=True)))
    assert worst < 1e-4, f"ONNX 출력이 Brax와 다르다 ({worst})"
    info = {"train_seconds": round(wall), "env_steps": a.steps, "num_envs": a.envs, "devices": str(jax.devices()),
            "onnx_vs_brax_max_err": worst, "final_eval": log[-1] if log else None}
    write_card(out, a.name, info)
    (out / "train_log.json").write_text(json.dumps({"config": {k: v for k, v in cfg.items()}, "log": log, **info}, indent=1))
    print(f"저장: {out.relative_to(ROOT)} (학습 {wall:.0f} s, ONNX = Brax 최대 차이 {worst:.1e})")


if __name__ == "__main__":
    main()
