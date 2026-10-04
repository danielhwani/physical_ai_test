"""규칙 기반 트롯 보행기를 MLP로 모방(DAgger)하고 ONNX + 정책 카드로 내보낸다.

    python train/imitate_trot.py                  # 기본: 5 프로세스, DAgger 6회
    python train/imitate_trot.py --workers 3      # 메모리가 부족하면 줄인다

목적은 성능 향상이 아니라 ONNX 탑재 경로 시험용 정책을 만드는 것이다.
CPU만으로 동작하도록 numpy로 MLP를 학습하고, ONNX 그래프를 직접 구성한다 (PyTorch 불필요).

DAgger: 정책이 직접 움직인 상태에서도 전문가(트롯 보행기)의 정답을 받아 데이터에 추가한다.
단순 모방(전문가 상태만 학습)은 작은 오차가 쌓여 학습 때 못 본 상태로 흘러가 넘어지기 쉽다.
"""
import argparse
import datetime as dt
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.runner import Simulation, load_yaml  # noqa: E402

OUT_DIR = ROOT / "policies/go2_trot_bc"
EPISODE_S = 12.0
HIDDEN = (128, 128)


# ---------------- MLP (numpy) ----------------

def elu(x):
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0)))


class MLP:
    def __init__(self, sizes, rng):
        self.W = [rng.normal(0, np.sqrt(2 / a), (a, b)) for a, b in zip(sizes[:-1], sizes[1:])]
        self.b = [np.zeros(b) for b in sizes[1:]]
        self.mean = np.zeros(sizes[0]); self.std = np.ones(sizes[0])

    def __call__(self, x):
        h = (x - self.mean) / self.std
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            h = h @ W + b
            if i < len(self.W) - 1:
                h = elu(h)
        return h

    def params(self):
        return {"W": self.W, "b": self.b, "mean": self.mean, "std": self.std}

    @classmethod
    def from_params(cls, p):
        m = cls.__new__(cls)
        m.W, m.b, m.mean, m.std = p["W"], p["b"], p["mean"], p["std"]
        return m

    def fit(self, X, Y, epochs, rng, lr=1e-3, batch=512):
        self.mean, self.std = X.mean(0), X.std(0) + 1e-3
        Xn = (X - self.mean) / self.std
        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]; v = [np.zeros_like(p) for p in params]
        t = 0
        for _ in range(epochs):
            order = rng.permutation(len(X))
            for s in range(0, len(X), batch):
                idx = order[s:s + batch]
                # forward (활성화 저장)
                hs, zs = [Xn[idx]], []
                for i, (W, b) in enumerate(zip(self.W, self.b)):
                    z = hs[-1] @ W + b
                    zs.append(z)
                    hs.append(elu(z) if i < len(self.W) - 1 else z)
                g = 2 * (hs[-1] - Y[idx]) / len(idx)          # dMSE/dout
                gW, gb = [None] * len(self.W), [None] * len(self.W)
                for i in reversed(range(len(self.W))):
                    gW[i], gb[i] = hs[i].T @ g, g.sum(0)
                    if i > 0:
                        g = (g @ self.W[i].T) * np.where(zs[i - 1] > 0, 1.0, np.exp(np.minimum(zs[i - 1], 0)))
                t += 1
                for k, (p, gp) in enumerate(zip(params, gW + gb)):
                    m[k] = 0.9 * m[k] + 0.1 * gp
                    v[k] = 0.999 * v[k] + 0.001 * gp * gp
                    p -= lr * (m[k] / (1 - 0.9 ** t)) / (np.sqrt(v[k] / (1 - 0.999 ** t)) + 1e-8)
        return float(np.mean((self(X) - Y) ** 2))


# ---------------- 데이터 수집 ----------------

def random_scenario(rng):
    rough = rng.random() < 0.6
    patches = []
    if rough:
        patches.append({"kind": "rough", "amplitude": float(rng.uniform(0.005, 0.02)), "region": [-8, -8, 8, 8]})
        for _ in range(rng.integers(0, 4)):
            patches.append({"kind": "bump", "center": rng.uniform(-4, 4, 2).tolist(),
                            "height": float(rng.uniform(0.02, 0.06)), "radius": float(rng.uniform(0.3, 0.6))})
    events, t = [], float(rng.uniform(0.3, 1.0))
    while t < EPISODE_S:
        if rng.random() < 0.15:
            vx, wz = 0.0, 0.0                                         # 정지 명령도 포함
        else:
            vx, wz = float(rng.uniform(0.0, 0.5)), float(rng.uniform(-0.5, 0.5))
        events.append({"t": round(t, 2), "action": "set_command", "vx": vx, "yaw_rate": wz})
        t += float(rng.uniform(2.0, 4.0))
    return {"name": "bc_episode", "duration": EPISODE_S, "seed": int(rng.integers(1 << 30)),
            "terrain": {"size": [8.0, 8.0], "resolution": 0.05, "z_range": [-0.2, 0.4], "patches": patches},
            "controller": {"type": "trot", "period": 0.36, "swing_height": 0.08, "stand_height": 0.27},
            "events": events}


def run_episode(job):
    """전문가(트롯)를 함께 돌리며 (관측, 전문가 action)을 모은다.
    beta 확률로 전문가가, 나머지는 정책이 실제로 로봇을 움직인다."""
    scenario, beta, params, noise, seed = job
    rng = np.random.default_rng(seed)
    policy = MLP.from_params(params) if params else None
    sim = Simulation(scenario)
    X, Y = [], []

    def hook(ctx, expert_action):
        X.append(ctx.obs.copy()); Y.append(expert_action.copy())
        if policy is not None and rng.random() >= beta:
            a = policy(ctx.obs[None])[0]
        else:
            a = expert_action
        return a + rng.normal(0, noise, 12) if noise else a

    sim.on_action = hook
    fell = False
    while sim.data.time < scenario["duration"]:
        _, _, r, p = sim.step_control(verbose=False)
        if sim.fallen(r, p):
            fell = True
            break
    return np.array(X), np.array(Y), fell, float(sim.data.time)


def evaluate(params, scenarios):
    """정책 단독 실행: 전진 거리와 넘어짐."""
    out = []
    for path in scenarios:
        sim = Simulation(path)
        policy = MLP.from_params(params)
        sim.on_action = lambda ctx, a: policy(ctx.obs[None])[0]
        x0 = sim.data.qpos[0]
        fell = None
        while sim.data.time < sim.scn["duration"]:
            _, _, r, p = sim.step_control(verbose=False)
            if sim.fallen(r, p):
                fell = round(sim.data.time, 2)
                break
        out.append({"scenario": Path(path).stem, "forward_m": round(float(sim.data.qpos[0] - x0), 3), "fell_at": fell})
    return out


# ---------------- ONNX 내보내기 ----------------

def export_onnx(params, path):
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    f32 = lambda a: np.asarray(a, dtype=np.float32)  # noqa: E731
    inits = [numpy_helper.from_array(f32(params["mean"]), "obs_mean"),
             numpy_helper.from_array(f32(params["std"]), "obs_std")]
    nodes = [helper.make_node("Sub", ["obs", "obs_mean"], ["x_c"]),
             helper.make_node("Div", ["x_c", "obs_std"], ["h0"])]   # 관측 정규화를 그래프 안에 포함
    n = len(params["W"])
    for i, (W, b) in enumerate(zip(params["W"], params["b"])):
        inits += [numpy_helper.from_array(f32(W), f"W{i}"), numpy_helper.from_array(f32(b), f"b{i}")]
        out = "actions" if i == n - 1 else f"z{i + 1}"
        nodes.append(helper.make_node("Gemm", [f"h{i}", f"W{i}", f"b{i}"], [out]))
        if i < n - 1:
            nodes.append(helper.make_node("Elu", [out], [f"h{i + 1}"], alpha=1.0))
    d_in, d_out = params["W"][0].shape[0], params["W"][-1].shape[1]
    graph = helper.make_graph(
        nodes, "go2_trot_bc",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", d_in])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, ["batch", d_out])], inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], producer_name="imitate_trot")
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, path)


def write_card(spec, info, path):
    card = {
        "name": "go2_trot_bc",
        "onnx": "policy.onnx",
        "io": {"input": "obs", "output": "actions"},
        # 아래 규약은 학습 당시 명세를 그대로 복사한다 (카드만 보고 탑재할 수 있도록 자기완결적으로)
        "joints": spec["robot"]["joints"],
        "default_pose": spec["robot"]["default_pose"],
        "action": {k: spec["action"][k] for k in ("scale", "clip", "control_dt", "pd")},
        "observation": spec["observation"],
        "gait_clock": {"period": 0.36},
        "provenance": info,
    }
    path.write_text("# 자동 생성: train/imitate_trot.py\n" + yaml.safe_dump(card, allow_unicode=True, sort_keys=False))


# ---------------- 메인 ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=5)
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    spec = load_yaml(ROOT / "specs/go2_control.yaml")
    dim_in, dim_out = spec["observation"]["dim"], 12
    net = MLP((dim_in, *HIDDEN, dim_out), rng)
    betas = [1.0, 0.5, 0.25, 0.1, 0.0, 0.0, 0.0, 0.0][:args.iters]
    eval_set = [ROOT / "scenarios/flat_trot.yaml", ROOT / "scenarios/rough_rut.yaml"]
    X_all, Y_all, log = [], [], []
    t_start = time.time()

    with mp.get_context("fork").Pool(args.workers) as pool:
        for it, beta in enumerate(betas):
            t0 = time.time()
            params = None if it == 0 else net.params()
            n_ep = args.episodes + (10 if it == 0 else 0)
            noise = 0.15 if it == 0 else 0.0          # 첫 회: 전문가 행동에 잡음 -> 회복 동작 데이터 확보
            jobs = [(random_scenario(rng), beta, params, noise, int(rng.integers(1 << 30))) for _ in range(n_ep)]
            results = pool.map(run_episode, jobs)
            X_all += [r[0] for r in results if len(r[0])]
            Y_all += [r[1] for r in results if len(r[1])]
            falls = sum(r[2] for r in results)
            X, Y = np.concatenate(X_all), np.concatenate(Y_all)
            mse = net.fit(X, Y, epochs=40 if it == 0 else 15, rng=rng)
            ev = evaluate(net.params(), eval_set)
            entry = {"iter": it, "beta": beta, "episodes": n_ep, "falls_during_collection": int(falls),
                     "samples": int(len(X)), "train_mse": round(mse, 5), "eval": ev, "sec": round(time.time() - t0, 1)}
            log.append(entry)
            print(json.dumps(entry, ensure_ascii=False))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    export_onnx(net.params(), OUT_DIR / "policy.onnx")
    info = {"method": "DAgger imitation of rule-based trot (sim/controllers/trot.py)",
            "spec_version": spec["spec_version"], "date": dt.date.today().isoformat(),
            "network": f"MLP {dim_in}-{'-'.join(map(str, HIDDEN))}-{dim_out} ELU, obs normalization in graph",
            "samples": int(len(X)), "dagger_iters": len(betas), "train_seconds": round(time.time() - t_start),
            "trained_on": "CPU MuJoCo, model variant cpu", "final_eval": log[-1]["eval"]}
    write_card(spec, info, OUT_DIR / "card.yaml")
    (OUT_DIR / "train_log.json").write_text(json.dumps(log, indent=2, ensure_ascii=False))
    print(f"\n저장: {OUT_DIR.relative_to(ROOT)}/policy.onnx, card.yaml, train_log.json")


if __name__ == "__main__":
    main()
