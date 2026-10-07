"""규칙 기반 트롯 보행기를 MLP로 모방(DAgger)하고 ONNX + 정책 카드로 내보낸다.

    python train/imitate_trot.py                  # 기본: 5 프로세스, DAgger 6회
    python train/imitate_trot.py --workers 3      # 메모리가 부족하면 줄인다
    python train/imitate_trot.py --perception     # 지형 인지 트롯(LiDAR 높이 지도)을 모방 -> policies/go2_trot_bc_terrain

목적은 성능 향상이 아니라 ONNX 탑재 경로 시험용 정책을 만드는 것이다.
CPU만으로 동작하도록 numpy로 MLP를 학습하고, ONNX 그래프를 직접 구성한다 (PyTorch 불필요).

DAgger: 정책이 직접 움직인 상태에서도 전문가(트롯 보행기)의 정답을 받아 데이터에 추가한다.
단순 모방(전문가 상태만 학습)은 작은 오차가 쌓여 학습 때 못 본 상태로 흘러가 넘어지기 쉽다.

--perception: 전문가 = 지형 인지 트롯 (perception.sensor = front_lidar). 학생 관측 = 명세의 observation_perceptive
(47개 항목 + height_scan 209개). 학생도 전문가와 같은 LiDAR 추정 높이 지도에서 격자를 뽑으므로 배치 때와 같은 오차를 본다.
학습 지형은 Chrono 없이 MuJoCo 지형에 바닥이 평평한 바퀴 자국(profile: box)을 무작위로 넣는다 (빠르고 메모리가 적다).
LiDAR 스캔 계산 때문에 데이터 수집이 약 5배 느리다.
"""
import argparse
import datetime as dt
import gc
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from control.interface import ControlInterface  # noqa: E402
from sim.runner import Simulation, apply_overrides, load_yaml  # noqa: E402

OUT_DIR = ROOT / "policies/go2_trot_bc_est"
OUT_DIR_TERRAIN = ROOT / "policies/go2_trot_bc_terrain"
PERCEPTION = {"sensor": "front_lidar"}
LATENCY_RANGE = (0, 2)       # 에피소드마다 연결 지연 0~2 제어 주기 (배치 측정값 1주기 = 20 ms 주변으로 여유)
EPISODE_S = 12.0
HIDDEN = (128, 128)
HIDDEN_TERRAIN = (256, 128)   # 입력 256 (height_scan 209 포함)


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

    def fit(self, X, Y, epochs, rng, lr=1e-3, batch=512, std_floor=None):
        self.mean, self.std = X.mean(0), X.std(0) + 1e-3
        if std_floor is not None:   # 거의 상수인 입력(평지의 높이 격자)의 작은 잡음이 정규화로 수백 배 커지지 않게
            self.std = np.maximum(self.std, std_floor)
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

def random_ruts(rng):
    """진행 방향(x)을 가로지르는 바퀴 자국 1~3줄. Chrono 자국과 같은 평평한 바닥.
    교사(지형 인지 트롯)가 넘을 수 있는 범위로 제한한다: 비스듬한 자국(14~39도)과 6.5 cm 넘는 깊이에서 교사도 넘어졌다
    (무작위 12개 중 3개). 교사가 못 하는 곳의 정답은 학생에게 나쁜 예제다."""
    # 자국을 촘촘하게(0.5~1.1 m 간격, 3~6줄): 드문드문 두면 자국을 건너는 데이터가 2~5%뿐이라 학생이 평지 보행만 배웠다
    ruts, x = [], float(rng.uniform(0.6, 1.2))
    for _ in range(rng.integers(3, 7)):
        ang = float(np.clip(rng.normal(0.0, 0.15), -0.3, 0.3))   # 0 = 진행 방향에 직각. ±17도 이내
        dx, dy = 6.0 * np.sin(ang), 6.0 * np.cos(ang)
        ruts.append({"kind": "rut", "start": [x - dx, -dy], "end": [x + dx, dy], "profile": "box",
                     "width": float(rng.uniform(0.18, 0.35)), "depth": float(rng.uniform(0.03, 0.06))})
        x += float(rng.uniform(0.5, 1.1))
    return ruts


def random_scenario(rng, latency_range=LATENCY_RANGE, perception=False):
    rough = rng.random() < (0.4 if perception else 0.6)
    patches = []
    if rough:
        patches.append({"kind": "rough", "amplitude": float(rng.uniform(0.005, 0.02)), "region": [-8, -8, 8, 8]})
        for _ in range(rng.integers(0, 4)):
            patches.append({"kind": "bump", "center": rng.uniform(-4, 4, 2).tolist(),
                            "height": float(rng.uniform(0.02, 0.06)), "radius": float(rng.uniform(0.3, 0.6))})
    if perception and rng.random() < 0.9:
        patches += random_ruts(rng)
    events, t = [], float(rng.uniform(0.3, 1.0))
    while t < EPISODE_S:
        if rng.random() < 0.15:
            vx, wz = 0.0, 0.0                                         # 정지 명령도 포함
        elif perception:                                              # 자국을 실제로 건너가도록 주로 앞으로
            vx, wz = float(rng.uniform(0.15, 0.5)), float(rng.uniform(-0.25, 0.25))
        else:
            vx, wz = float(rng.uniform(0.0, 0.5)), float(rng.uniform(-0.5, 0.5))
        events.append({"t": round(t, 2), "action": "set_command", "vx": vx, "yaw_rate": wz})
        t += float(rng.uniform(2.0, 4.0))
    return {"name": "bc_episode", "duration": EPISODE_S, "seed": int(rng.integers(1 << 30)),
            "terrain": {"size": [8.0, 8.0], "resolution": 0.05, "z_range": [-0.2, 0.4], "patches": patches},
            "controller": {"type": "trot", "period": 0.36, "swing_height": 0.08, "stand_height": 0.27,
                           "perception": dict(PERCEPTION) if perception else None},
            # 배치(ROS 노드)와 같은 연결 지연. 관측은 로봇 경계(센서 잡음 + 상태 추정)를 그대로 거친다
            "control_link": {"latency_steps": int(rng.integers(latency_range[0], latency_range[1] + 1))},
            "events": events}


def student_obs_spec(spec, perception):
    """학생 정책의 관측 규약 (ControlInterface와 같은 관절 순서/기본 자세)."""
    obs = spec["observation_perceptive"] if perception else spec["observation"]
    return ControlInterface(spec, {"observation": obs}).obs


def attach_student(sim, spec, perception):
    """ctx.aux_obs에 학생 관측이 오도록 한다. 반환: 관측을 꺼내는 함수."""
    if not perception:
        return lambda ctx: ctx.obs
    sim.controller.aux_obs_spec = student_obs_spec(spec, True)
    return lambda ctx: ctx.aux_obs


def run_episode(job):
    """전문가(트롯)를 함께 돌리며 (학생 관측, 전문가 action)을 모은다.
    beta 확률로 전문가가, 나머지는 정책이 실제로 로봇을 움직인다."""
    scenario, beta, params, noise, seed, perception = job
    rng = np.random.default_rng(seed)
    policy = MLP.from_params(params) if params else None
    sim = Simulation(scenario)
    obs_of = attach_student(sim, sim.spec, perception)
    X, Y = [], []

    def hook(ctx, expert_action):
        o = obs_of(ctx)
        X.append(o.copy()); Y.append(expert_action.copy())
        if policy is not None and rng.random() >= beta:
            a = policy(o[None])[0]
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
    t_end = float(sim.data.time)
    if fell:                 # 넘어지기 직전 1초는 교사도 회복할 수 없는 상태라 정답이 의미 없다
        keep = max(0, len(X) - int(round(1.0 / sim.controller.iface.control_dt)))
        X, Y = X[:keep], Y[:keep]
    sim.close()
    del sim, hook
    gc.collect()     # 시뮬레이터·센서 렌더러의 MuJoCo 모델이 순환 참조로 남아 에피소드마다 약 100 MB씩 쌓였다 (확인함)
    return np.array(X), np.array(Y), fell, t_end


def evaluate(params, scenarios, perception=False):
    """정책 단독 실행: 전진 거리와 넘어짐. 지형 인지면 같은 높이 지도를 쓰도록 전문가 쪽 지형 인지를 켠다 (행동은 정책만)."""
    out = []
    for path in scenarios:
        scn = apply_overrides(load_yaml(path), ["controller.perception.sensor=front_lidar"] if perception else [])
        sim = Simulation(scn)
        obs_of = attach_student(sim, sim.spec, perception)
        policy = MLP.from_params(params)
        sim.on_action = lambda ctx, a: policy(obs_of(ctx)[None])[0]
        x0 = sim.data.qpos[0]
        fell = None
        while sim.data.time < sim.scn["duration"]:
            _, _, r, p = sim.step_control(verbose=False)
            if sim.fallen(r, p):
                fell = round(sim.data.time, 2)
                break
        out.append({"scenario": Path(path).stem, "forward_m": round(float(sim.data.qpos[0] - x0), 3), "fell_at": fell})
        sim.close()
        del sim
        gc.collect()
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


def write_card(spec, info, path, perception=False):
    card = {
        "name": path.parent.name,
        "onnx": "policy.onnx",
        "io": {"input": "obs", "output": "actions"},
        # 아래 규약은 학습 당시 명세를 그대로 복사한다 (카드만 보고 탑재할 수 있도록 자기완결적으로)
        "joints": spec["robot"]["joints"],
        "default_pose": spec["robot"]["default_pose"],
        "action": {k: spec["action"][k] for k in ("scale", "clip", "control_dt", "pd")},
        "observation": spec["observation_perceptive"] if perception else spec["observation"],
        "gait_clock": {"period": 0.36},
        "provenance": info,
    }
    if perception:          # 배치 때 높이 지도를 만들 센서 (control/node.py가 TerrainPerception을 만든다)
        card["perception"] = dict(PERCEPTION)
    path.write_text("# 자동 생성: train/imitate_trot.py\n" + yaml.safe_dump(card, allow_unicode=True, sort_keys=False))


# ---------------- 메인 ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=3)   # 작업자당 약 0.55 GB. 5개는 이 PC에서 메모리 부족
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", help="정책 저장 폴더")
    ap.add_argument("--perception", action="store_true", help="지형 인지 트롯 모방 (학생 관측 observation_perceptive)")
    args = ap.parse_args()
    perc = args.perception
    out_dir = Path(args.out or (OUT_DIR_TERRAIN if perc else OUT_DIR))

    rng = np.random.default_rng(args.seed)
    spec = load_yaml(ROOT / "specs/go2_control.yaml")
    dim_in, dim_out = student_obs_spec(spec, perc).dim, 12
    hidden = HIDDEN_TERRAIN if perc else HIDDEN
    net = MLP((dim_in, *hidden, dim_out), rng)
    betas = [1.0, 0.5, 0.25, 0.1, 0.0, 0.0, 0.0, 0.0][:args.iters]
    eval_set = [ROOT / "scenarios/flat_trot.yaml",
                ROOT / ("scenarios/rut_crossing.yaml" if perc else "scenarios/rough_rut.yaml")]
    X_all, Y_all, log = [], [], []
    t_start = time.time()
    best = None
    std_floor = None
    if perc:   # height_scan 부분: 원래 단위 5 cm (scale 5 -> 0.25)
        o = student_obs_spec(spec, True)
        t_hs, sl = o.term("height_scan"), o.slices()["height_scan"]
        std_floor = np.zeros(dim_in)
        std_floor[sl] = 0.05 * t_hs["scale"]

    with mp.get_context("fork").Pool(args.workers, maxtasksperchild=10) as pool:   # 작업자를 주기적으로 새로 띄워 메모리 조각화 방지
        for it, beta in enumerate(betas):
            t0 = time.time()
            params = None if it == 0 else net.params()
            n_ep = args.episodes + (10 if it == 0 else 0)
            noise = (0.08 if perc else 0.15) if it == 0 else 0.0   # 첫 회: 전문가 행동에 잡음 -> 회복 동작 데이터 확보 (지형이 있으면 작게: 0.15는 40회 중 15회 넘어짐)
            jobs = [(random_scenario(rng, perception=perc), beta, params, noise, int(rng.integers(1 << 30)), perc)
                    for _ in range(n_ep)]
            results = pool.map(run_episode, jobs)
            X_all += [r[0] for r in results if len(r[0])]
            Y_all += [r[1] for r in results if len(r[1])]
            falls = sum(r[2] for r in results)
            X, Y = np.concatenate(X_all), np.concatenate(Y_all)
            mse = net.fit(X, Y, epochs=40 if it == 0 else (25 if perc else 15), rng=rng, std_floor=std_floor)
            ev = evaluate(net.params(), eval_set, perc)
            score = sum(e["forward_m"] for e in ev) - 3.0 * sum(e["fell_at"] is not None for e in ev)
            if best is None or score > best[0]:     # 반복마다 결과가 흔들리므로 평가가 가장 좋은 반복을 저장한다
                best = (score, it, {k: [a.copy() for a in v] if isinstance(v, list) else v.copy()
                                    for k, v in net.params().items()}, ev)
            entry = {"iter": it, "beta": beta, "episodes": n_ep, "falls_during_collection": int(falls),
                     "samples": int(len(X)), "train_mse": round(mse, 5), "eval": ev, "sec": round(time.time() - t0, 1)}
            log.append(entry)
            print(json.dumps(entry, ensure_ascii=False))

    out_dir.mkdir(parents=True, exist_ok=True)
    best_score, best_it, best_params, best_eval = best
    export_onnx(best_params, out_dir / "policy.onnx")
    info = {"method": "DAgger imitation of rule-based trot (control/trot.py)"
                      + (" with terrain perception (LiDAR elevation map, control/foothold.py)" if perc else ""),
            "observation_source": "robot boundary: sensor noise + state estimator (no ground truth)",
            "link_latency_steps_range": list(LATENCY_RANGE),
            "spec_version": spec["spec_version"], "date": dt.date.today().isoformat(),
            "network": f"MLP {dim_in}-{'-'.join(map(str, hidden))}-{dim_out} ELU, obs normalization in graph",
            "samples": int(len(X)), "dagger_iters": len(betas), "train_seconds": round(time.time() - t_start),
            "selected_iter": best_it,
            "trained_on": "CPU MuJoCo, model variant cpu"
                          + ("; random box ruts (no Chrono), height map from simulated LiDAR" if perc else ""),
            "final_eval": best_eval}
    write_card(spec, info, out_dir / "card.yaml", perc)
    (out_dir / "train_log.json").write_text(json.dumps(log, indent=2, ensure_ascii=False))
    print(f"\n저장: {out_dir}/policy.onnx, card.yaml, train_log.json (평가는 연결 지연 1주기 = ROS 배치와 같음)")


if __name__ == "__main__":
    main()
