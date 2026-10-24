"""SCM 다짐 우회 시험 "다짐 -> 단단한 흙" (docs/map_server.md 5.5): 높이만 넣은 자리(다짐 이력 없음)에서 위치마다 흙 성질 콜백이
더 단단한 흙(kphi × 배수)을 답하면, 같은 SCM에서 이어 누른 자리처럼 덜 가라앉는가.

    python conformance/probe_scm_compaction.py [--mults ...] [--masses 60 40 90] [--plot docs/img/scm_compaction.png]
    (어느 환경이든: pychrono가 없으면 chrono 환경으로 다시 실행. 기본 값으로 이 PC에서 약 1분 30초)

probe_scm_injection.py와 같은 흙·구. 두 자리 (그 시험의 3, 4번 자리와 같다). 한 번 실행마다 새 SCM:
  이어 누른 자리  60 kg으로 한 번 눌러 다진다 (같은 SCM 안에서) -> 그 높이를 베껴 두고 -> 같은 자리를 하중 M으로 다시 누름
  베낀 자리  이어 누른 자리의 높이만 새 흙에 넣는다 -> 콜백이 이 자리(0.42 m 사각형)에서만 kphi × 배수를 답한다 -> 하중 M으로 누름
M = 60 kg (다질 때와 같은 하중) 과 90 kg (더 무거움). 같은 SCM 안에서 다져진 흙은 예전 하중보다 센 하중에서 다시 무른 흙처럼 꺼지지만,
"단단한 흙"은 어떤 하중에서나 단단하다. 그래서 60 kg에서 맞춘 배수가 90 kg에서도 맞는지 본다.
이 PC (2026-10-24, 60 kg으로 다짐): 이어 누른 자리 다시 누름 0.02 cm (60 kg), 0.22 cm (90 kg), -0.02 cm (40 kg).
베낀 자리: ×1 1.44, ×3 0.64, ×10 0.20, ×20 0.05, ×25 0.01, ×30 -0.03 cm (60 kg). ×25로 90 kg 0.00, 40 kg 0.01 cm.
음수 = 구가 누르기 전 가운데 높이보다 위에서 멈춤 (단단한 흙에서 구가 홈 벽에 얹힘).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_scm_injection as P  # noqa: E402  (pychrono가 없으면 여기서 chrono 환경으로 다시 실행된다)

veh = P.veh
MULTS = [1, 3, 10, 20, 25, 30, 100]
MASSES = [60.0, 40.0, 90.0]          # 첫 값(다질 때와 같은 하중)에서 이어 누른 자리에 맞는 배수를 고른다
COMPACT_MASS = 60.0


class StiffWhere(veh.SoilParametersCallback):
    """위치마다 흙 성질: 다짐 영역 안이면 kphi × mult, 밖이면 시험 기본 흙. SCM이 노드를 계산할 때마다 부른다.
    답은 C++ 칸(double&)에 veh.doublep_assign으로 적는다 (그냥 대입하면 효과가 없다)."""

    def __init__(self, center, half_m, mult):
        super().__init__()
        self.cx, self.cy, self.h, self.mult = center[0], center[1], half_m, mult
        self.calls = 0

    def Set(self, loc, kphi, kc, n, coh, phi, js, ek, dr):
        self.calls += 1
        inside = abs(loc.x - self.cx) <= self.h and abs(loc.y - self.cy) <= self.h
        s = P.SOIL
        vals = (s["kphi"] * (self.mult if inside else 1.0), s["kc"], s["n"], s["coh"], s["phi"], s["js"], s["ek"], s["dr"])
        for ptr, v in zip((kphi, kc, n, coh, phi, js, ek, dr), vals):
            veh.doublep_assign(ptr, float(v))


def run(mult, mass):
    """반환: (이어 누른 자리 다시 누름 침하, 베낀 자리 침하, 베낀 자리 처음 흙 기준 깊이, 콜백 호출 수)."""
    p = P.Probe()
    cb = StiffWhere(P.SPOTS[4], P.HALF * P.DELTA, mult)
    p.ter.RegisterSoilParametersCallback(cb)           # cb는 이 함수가 끝날 때까지 살아 있어야 한다 (파이썬 객체를 SCM이 부름)
    p.ball.SetMass(COMPACT_MASS)
    p.press(P.SPOTS[3])                                 # 같은 SCM 안에서 다짐 (기준)
    c3 = P.node(P.SPOTS[3])
    copied = {(nd.x - c3[0], nd.y - c3[1]): h for nd, h in p.ter.GetModifiedNodes(True)
              if abs(nd.x - c3[0]) <= P.HALF and abs(nd.y - c3[1]) <= P.HALF}
    p.ball.SetMass(mass)
    h0, b3 = p.press(P.SPOTS[3])
    c4 = P.node(P.SPOTS[4])
    p.inject({(c4[0] + di, c4[1] + dj): h for (di, dj), h in copied.items()}, P.SPOTS[4])
    h4, b4 = p.press(P.SPOTS[4])
    return h0 - b3, h4 - b4, -b4, cb.calls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mults", type=float, nargs="+", default=MULTS, help="kphi 배수")
    ap.add_argument("--masses", type=float, nargs="+", default=MASSES, help="다시 누르는 하중 (kg). 첫 값이 배수를 맞추는 하중")
    ap.add_argument("--plot", metavar="PNG", help="그림으로 저장 (기록을 저장한 뒤 plot_scm_probe.py로 그림)")
    ap.add_argument("--record", metavar="NPZ", help="시험 기록 (scm_record.py)")
    a = ap.parse_args()
    rows = {}
    for mass in a.masses:
        for m in a.mults:
            rows[mass, m] = run(m, mass)
            s3, s4, d4, calls = rows[mass, m]
            print(f"하중 {mass:3.0f} kg  kphi ×{m:<4g} 이어 누른 자리 {s3 * 100:5.2f} cm | 베낀 자리 {s4 * 100:5.2f} cm "
                  f"(처음 흙 기준 깊이 {d4 * 100:.2f} cm) | 콜백 {calls}번", flush=True)
    m0 = a.masses[0]
    best = min(a.mults, key=lambda m: abs(rows[m0, m][1] - rows[m0, m][0]))      # 베낀 자리의 침하가 이어 누른 자리와 가장 가까운 배수
    print(f"\n{m0:.0f} kg에서 이어 누른 자리에 가장 가까운 배수: ×{best:g} (이어 누른 자리 {rows[m0, best][0] * 100:.2f} cm, "
          f"베낀 자리 {rows[m0, best][1] * 100:.2f} cm)")
    for mass in a.masses[1:]:
        print(f"같은 배수로 {mass:.0f} kg: 이어 누른 자리 {rows[mass, best][0] * 100:.2f} cm, 베낀 자리 {rows[mass, best][1] * 100:.2f} cm "
              f"(같은 SCM 안에서 다져진 흙은 다질 때 하중을 넘으면 다시 가라앉는다)")
    if a.record or a.plot:
        import scm_record
        config = {"soil": P.SOIL, "radius": P.R, "delta": P.DELTA, "compact_mass": COMPACT_MASS}
        analysis = {"rows": [[mass, m, *v] for (mass, m), v in rows.items()], "mults": a.mults, "masses": a.masses, "best": best}
        path = a.record or str(Path(__file__).resolve().parent.parent / "build/scm_compaction_plot.npz")
        scm_record.save(path, "compaction", config, analysis)
        print(f"기록: {path}")
        if a.plot:
            from plot_scm_probe import plot
            print(f"그림: {plot(path, a.plot)}")


if __name__ == "__main__":
    main()
