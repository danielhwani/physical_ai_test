"""파라미터 스윕: 시나리오 값 하나를 바꿔 가며 실행하고 MOP를 표로 비교한다.

    python -m sim.sweep scenarios/footprints.yaml --param chrono.scm.soil.bekker_kphi --values 5e6 1e7 2e7 4e7
    python -m sim.sweep scenarios/hmmwv_follow.yaml --param chrono.scm.soil.bekker_kphi --values 2e6 5e6 2e7 --set duration=12

각 실행은 runs/에 일반 실행과 같이 기록되고(메타데이터에 바꾼 값), 비교표는 runs/sweep_<시각>.csv로 저장된다.
화면 없이 시뮬레이션 시간(lockstep)으로 돌므로 결과는 결정적이다.
"""
import argparse
import contextlib
import csv
import datetime as dt
import io

from .model_builder import ROOT
from .runner import load_yaml, run

COLUMNS = ["fell", "fell_at_s", "forward_x_m", "lateral_drift_m", "mean_speed_mps", "cost_of_transport", "edge_touchdowns",
           "deformed_cells", "deform_mean_m", "deform_max_m", "min_vehicle_distance_m", "wall_time_s"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--param", required=True, help="바꿀 시나리오 키 (점으로 구분)")
    ap.add_argument("--values", nargs="+", required=True)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="모든 실행에 공통으로 바꿀 값")
    ap.add_argument("--variant", choices=["cpu", "mjx"], default="cpu")
    ap.add_argument("--policy")
    args = ap.parse_args()

    base = load_yaml(args.scenario)
    rows = []
    for v in args.values:
        sets = args.set + [f"{args.param}={v}"]
        run_args = argparse.Namespace(scenario=base, set=sets, variant=args.variant, policy=args.policy,
                                      realtime=False, view=False, start_paused=False, ros2=False, rviz=False,
                                      mjviz=False, rt=False)
        print(f"{args.param} = {v} ...", flush=True)
        with contextlib.redirect_stdout(io.StringIO()):
            mop = run(run_args)
        rows.append({"value": v, **{c: mop.get(c) for c in COLUMNS}})

    shown = [c for c in COLUMNS if any(r[c] not in (None, 0, 0.0, False) for r in rows) or c in ("fell", "forward_x_m")]
    print(f"\n{base['name']}: {args.param}")
    print(f"{'value':>10s} " + " ".join(f"{c:>16s}" for c in shown))
    for r in rows:
        print(f"{r['value']:>10s} " + " ".join(f"{str(r[c]):>16s}" for c in shown))
    out = ROOT / "runs" / f"sweep_{base['name']}_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    out.parent.mkdir(exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["value"] + COLUMNS)
        w.writeheader()
        w.writerows(rows)
    print(f"\n저장: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
