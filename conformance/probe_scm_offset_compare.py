"""가상 다짐의 두 높이 보정 비교 (docs/map_server.md 5.5 "보정 방식 비교"): 되풀이 보정과 sinkage 보정을 누르기와 굴리기에서 같은 조건으로
돌려 단일 SCM 기준과 비교한다. 결과는 기록 하나에 담고, 그 기록으로 그림을 그린다.

    python conformance/probe_scm_offset_compare.py [--record build/scm_offset_compare.npz] [--plot docs/img/scm_offset_compare.png]
    (어느 환경이든: pychrono가 없으면 chrono 환경으로 다시 실행. 이 PC에서 약 1분 10초)

  되풀이 보정  눌러(굴려) 둔 뒤 목표 높이가 되는 시작 높이를 시험을 되풀이해 찾는다 (누르기 4회, 굴리기 3회)
  sinkage 보정 앞쪽 SCM의 노드 sinkage만큼 올려 넣는다 (= 자국 전 높이를 넣고 같은 일을 다시 한다), 되풀이 없음
누르기는 probe_scm_virtual_compaction.py, 굴리기는 probe_scm_rolling.py의 함수를 그대로 쓴다.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_scm_rolling as RL  # noqa: E402  (pychrono가 없으면 여기서 chrono 환경으로 다시 실행된다)
import probe_scm_virtual_compaction as V  # noqa: E402

PRESS_MASSES = [40.0, 60.0, 90.0]
ROLL_MASSES = [60.0, 90.0]
PRESS_ROUNDS, ROLL_ROUNDS = 4, 3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", metavar="NPZ", help="시험 기록 (scm_record.py)")
    ap.add_argument("--plot", metavar="PNG", help="그림 (기록을 저장한 뒤 plot_scm_probe.py로 그림)")
    a = ap.parse_args()

    print(f"누르기: 되풀이 보정 {PRESS_ROUNDS}회 찾기", flush=True)
    press_off = V.calibrate(PRESS_ROUNDS)
    press_hist = list(V.calibrate.history)
    press = {}
    for method, off in (("iterate", press_off), ("sinkage", "sinkage")):
        for m in PRESS_MASSES:
            s3, d3, s4, d4, e = V.run(m, off)
            press[f"{method}/{m:g}"] = {"ref": s3, "ref_depth": d3, "sink": s4, "depth": d4, "shape_err": e}
            print(f"  {method:8} {m:3.0f} kg: 기준 {s3 * 100:5.2f} cm | 가상 다짐 {s4 * 100:5.2f} cm | 모양 차이 {e * 100:.3f} cm",
                  flush=True)

    print(f"굴리기: 되풀이 보정 {ROLL_ROUNDS}회 찾기", flush=True)
    roll_off = RL.calibrate(ROLL_ROUNDS)
    roll_hist = list(RL.calibrate.history)
    roll = {}
    for method, off in (("iterate", roll_off), ("sinkage", "sinkage")):
        for m in ROLL_MASSES:
            _, A, _, C, _ = RL.run(m, off)
            roll[f"{method}/{m:g}"] = {"ref_sink": A[0], "ref_force": A[1], "sink": C[0], "force": C[1],
                                       "shape_err": RL.run.shape_err[m]}
            print(f"  {method:8} {m:3.0f} kg: 기준 {A[0] * 100:5.2f} cm / {A[1]:6.1f} N | 가상 다짐 {C[0] * 100:5.2f} cm / {C[1]:6.1f} N "
                  f"| 모양 차이 {RL.run.shape_err[m] * 100:.3f} cm", flush=True)

    if a.record or a.plot:
        import scm_record
        config = {"soil": V.P.SOIL, "radius": V.P.R, "delta": V.P.DELTA, "compact_mass": V.COMPACT_MASS,
                  "press_rounds": PRESS_ROUNDS, "roll_rounds": ROLL_ROUNDS}
        analysis = {"press_masses": PRESS_MASSES, "roll_masses": ROLL_MASSES, "press": press, "roll": roll,
                    "press_history": press_hist, "roll_history": roll_hist}
        path = a.record or str(Path(__file__).resolve().parent.parent / "build/scm_offset_compare_plot.npz")
        scm_record.save(path, "offset_compare", config, analysis)
        print(f"기록: {path}")
        if a.plot:
            from plot_scm_probe import plot
            print(f"그림: {plot(path, a.plot)}")


if __name__ == "__main__":
    main()
