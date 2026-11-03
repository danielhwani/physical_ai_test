"""단일 프로세스: 고스트 재생 빈도별로 뒤 차가 정답 (A, 한 SCM에서 두 대)과 얼마나 다른가. 출발 위치 (간격)마다 같은 출발끼리 비교한다.
chrono 환경에서 저장소 맨 위에서 돌린다. 간격마다 정답, 앞 차, 빈도별 F를 함께 띄운다 (빈도 수 + 2 프로세스).

    python conformance/probe_ghost_hz.py --route curve --kphi 1e6 --gaps 11.8 12.1 --hz 500 100 50 25    약 4분
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def one(a):
    import probe_scm_two_hmmwv as P
    P.ROUTE = a.route
    P.Y_END, P.SCM_Y = 26.0, (-30.0, 36.0)                      # probe_two_chrono.py와 같은 길이
    if a.route == "curve":
        P.SCM_W = 8.0
    P.GAP = a.gap
    if a.what == "A":
        np.save(a.out, P.run(a.kphi, True, True)["log"])
    elif a.what == "front":
        np.save(a.out, P.run(a.kphi, True, False)["traj"])
    else:
        np.save(a.out, P.run(a.kphi, False, True, ghost=np.load(a.traj), live=(a.delay, float(a.what)))["log"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default="curve", choices=["straight", "curve"])
    ap.add_argument("--kphi", type=float, default=1e6)
    ap.add_argument("--gaps", type=float, nargs="+", default=[11.8, 12.1])
    ap.add_argument("--hz", nargs="+", default=["500", "100", "50", "25"])
    ap.add_argument("--delay", type=float, default=0.2)
    ap.add_argument("--what"), ap.add_argument("--gap", type=float), ap.add_argument("--out"), ap.add_argument("--traj")
    a = ap.parse_args()
    if a.what:
        return one(a)
    me = [sys.executable, str(Path(__file__).resolve())]
    base = ["--route", a.route, "--kphi", str(a.kphi), "--delay", str(a.delay)]
    ys = np.arange(-8.0, 12.0, 0.25)
    with tempfile.TemporaryDirectory(prefix="ghost_hz_") as tmp:
        d = Path(tmp)
        for g in a.gaps:
            jobs = [subprocess.Popen(me + base + ["--what", w, "--gap", str(g), "--out", str(d / f"{w}.npy")]) for w in ("A", "front")]
            assert all(j.wait() == 0 for j in jobs)
            jobs = [subprocess.Popen(me + base + ["--what", h, "--gap", str(g), "--out", str(d / f"F{h}.npy"), "--traj", str(d / "front.npy")]) for h in a.hz]
            assert all(j.wait() == 0 for j in jobs)
            la = np.load(d / "A.npy")
            xa = np.interp(ys, la[:, 2], la[:, 1])
            for h in a.hz:
                lb = np.load(d / f"F{h}.npy")
                dx = 100 * (np.interp(ys, lb[:, 2], lb[:, 1]) - xa)
                print(f"{a.route} kphi {a.kphi:g} 간격 {g}: F {h:>4s} Hz - A 뒤 차 옆 궤적 차이 RMS {np.sqrt(np.mean(dx ** 2)):5.1f} cm, 최대 {np.abs(dx).max():5.1f} cm, "
                      f"바퀴 높이 {100 * la[:, 6].mean():.2f} vs {100 * lb[:, 6].mean():.2f} cm", flush=True)


if __name__ == "__main__":
    main()
