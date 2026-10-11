"""실시간 명령 지연과 lockstep 통계적 지연 비교 (README "통계적 명령 지연"의 검증).

    python conformance/compare_link_timing.py scenarios/rough_rut.yaml --seeds 30 [--brain ros] [--keep]

시드마다 세 가지를 돌린다.
  A: sim.runner lockstep, 고정 한 주기 지연
  C: sim.rt_link 실시간 (C++ 코어 + 두뇌, 실제 지연). 기록의 cmd_state_t로 명령이 몇 주기 더 늦었는지 센다 (시작 0.5초 제외)
  B: sim.runner lockstep + C 전체에서 잰 분포 (control_link.extra_steps_probs)
늦은 명령이 없던 C는 A와 같아야 한다 (실시간과 lockstep의 차이는 명령 시각뿐). MOP 평균·퍼짐의 차이를 부트스트랩 95% 구간으로 보인다.
실행 폴더는 --keep이 없으면 지운다. 이 PC에서 시드당 약 25초 (실시간 15초 포함, rough_rut).
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
MOPS = (("forward_x_m", lambda m: m["forward_x_m"]), ("|lateral_drift_m|", lambda m: abs(m["lateral_drift_m"])),
        ("cost_of_transport", lambda m: m["cost_of_transport"]))


def _run(args, made):
    r = subprocess.run([sys.executable, "-m", *args], cwd=ROOT, capture_output=True, text=True)
    m = re.search(r"기록: (\S+)", r.stdout)
    if not m:
        raise RuntimeError(f"{args}: 기록 폴더 없음\n{r.stdout[-600:]}\n{r.stderr[-600:]}")
    d = ROOT / m.group(1)
    made.append(d)
    return d, json.loads((d / "summary.json").read_text())["mop"]


def _extra_steps(run_dir, dt):
    df = pd.read_parquet(run_dir / "timeseries.parquet")
    df = df[df.t > 0.5]
    ex = np.round((df.t - dt - df.cmd_state_t) / dt).astype(int) - 1
    return np.bincount(ex.clip(0, 9), minlength=10)


def _ci(rng, x, y, stat):
    d = [stat(rng.choice(x, len(x))) - stat(rng.choice(y, len(y))) for _ in range(5000)]
    return np.percentile(d, [2.5, 97.5])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--brain", choices=["inproc", "shm", "ros"], default="ros")
    ap.add_argument("--keep", action="store_true", help="실행 폴더를 남긴다")
    a = ap.parse_args()
    seeds, made, res = range(a.start, a.start + a.seeds), [], {}
    hist = np.zeros(10, int)
    try:
        for s in seeds:
            _, A = _run(["sim.runner", a.scenario, "--set", f"seed={s}"], made)
            d, C = _run(["sim.rt_link", a.scenario, "--brain", a.brain, "--set", f"seed={s}"], made)
            h = _extra_steps(d, 0.02)
            hist += h
            res[s] = {"A": A, "C": C, "late": int(h[1:].sum())}
            print(f"seed {s}: A {A['forward_x_m']} m, C {C['forward_x_m']} m (늦은 명령 {res[s]['late']}, 늦게 걸린 변경 {C['late_applies']})",
                  flush=True)
        probs = [round(float(p), 4) for p in hist[1:] / hist.sum()]
        while probs and probs[-1] == 0:
            probs.pop()
        print(f"실측 분포: +1주기 이상 늦은 명령 {int(hist[1:].sum())} / {int(hist.sum())}, extra_steps_probs {probs}")
        for s in seeds:
            _, res[s]["B"] = _run(["sim.runner", a.scenario, "--set", f"seed={s}",
                                   "--set", f"control_link.extra_steps_probs={probs or [0.0]}"], made)
    finally:
        if not a.keep:
            for d in made:
                shutil.rmtree(d, ignore_errors=True)

    rng = np.random.default_rng(0)
    zero = [s for s in seeds if res[s]["late"] == 0]
    print(f"\n늦은 명령이 없던 실시간 실행 {len(zero)}개: lockstep과 같음 "
          f"{sum(res[s]['C']['forward_x_m'] == res[s]['A']['forward_x_m'] for s in zero)}/{len(zero)}")
    print(f"넘어짐: A {sum(res[s]['A']['fell'] for s in seeds)}, C {sum(res[s]['C']['fell'] for s in seeds)}, "
          f"B {sum(res[s]['B']['fell'] for s in seeds)}")
    for name, f in MOPS:
        v = {k: np.array([f(res[s][k]) for s in seeds]) for k in ("A", "C", "B")}
        print(f"\n{name}")
        for k in ("A", "C", "B"):
            print(f"  {k}: 평균 {v[k].mean():.3f}, 표준편차 {v[k].std(ddof=1):.3f}, A와 차이 평균 {np.abs(v[k] - v['A']).mean():.3f}")
        for k in ("A", "B"):
            m = _ci(rng, v["C"], v[k], np.mean)
            sd = _ci(rng, v["C"], v[k], lambda x: np.std(x, ddof=1))
            print(f"  C-{k}: 평균 차이 95% [{m[0]:.3f}, {m[1]:.3f}], 표준편차 차이 95% [{sd[0]:.3f}, {sd[1]:.3f}]")
    late = np.array([res[s]["late"] for s in seeds])
    print(f"\n실행별 늦은 명령 수: 평균 {late.mean():.2f}, 분산 {late.var(ddof=1):.2f} (명령마다 독립이면 분산 ≈ 평균)")


if __name__ == "__main__":
    main()
