"""차이의 계층 분리 (문서 §13.2): 모델 설정 차이만의 영향.

    python conformance/compare_variants.py [--seeds 8] [--scenario scenarios/rough_rut.yaml] [--policy card.yaml]

같은 CPU MuJoCo에서 원본 모델(cpu)과 MJX 오버라이드 모델(mjx)을 같은 시나리오/시드로 돌려
MOP 분포를 비교한다. 여기서 나오는 차이 = "MJX용 물리 설정 단순화"의 영향.
나중에 실제 MJX로 같은 시험을 하면, 그 결과와 mjx 변형의 차이 = "구현(JAX/float32) 차이"가 된다.
"""
import argparse
import contextlib
import io
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim.runner import Simulation  # noqa: E402


def run_once(scenario, variant, seed, policy=None):
    sim = Simulation(scenario, variant=variant, seed=seed, policy=policy)
    d = sim.data
    x0 = d.qpos[:2].copy()
    energy, fell, min_h = 0.0, False, np.inf
    with contextlib.redirect_stdout(io.StringIO()):
        while d.time < sim.scn["duration"]:
            _, e, r, p = sim.step_control()
            energy += e
            min_h = min(min_h, d.qpos[2] - sim.terrain.height_at(*d.qpos[:2]))
            if sim.fallen(r, p):
                fell = True
                break
    dist = float(np.linalg.norm(d.qpos[:2] - x0))
    return {"fell": float(fell), "forward_m": d.qpos[0] - x0[0], "lateral_m": abs(d.qpos[1] - x0[1]),
            "CoT": energy / (sim.total_mass * 9.81 * max(dist, 1e-6)), "min_body_h": min_h}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default=str(ROOT / "scenarios/rough_rut.yaml"))
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--policy", help="정책 카드 (없으면 트롯 보행기)")
    args = ap.parse_args()

    res = {v: [run_once(args.scenario, v, s, args.policy) for s in range(args.seeds)] for v in ("cpu", "mjx")}
    keys = list(res["cpu"][0])
    ctrl = Path(args.policy).parent.name if args.policy else "trot"
    print(f"시나리오 {Path(args.scenario).name}, 컨트롤러 {ctrl}, 시드 {args.seeds}개 (mean ± std)\n")
    print(f"{'MOP':12s} {'cpu':>18s} {'mjx-override':>18s} {'차이(mjx-cpu)':>14s}")
    for k in keys:
        a = np.array([r[k] for r in res["cpu"]]); b = np.array([r[k] for r in res["mjx"]])
        print(f"{k:12s} {a.mean():9.3f} ± {a.std():6.3f} {b.mean():9.3f} ± {b.std():6.3f} {b.mean() - a.mean():+14.3f}")


if __name__ == "__main__":
    main()
