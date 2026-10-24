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
    ap.add_argument("--plot", metavar="PNG", help="그림으로 저장")
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
    if a.plot:
        plot(a.plot, rows, a.mults, a.masses, best)
        print(f"그림: {a.plot}")


LOAD_COLORS = {40.0: "#2a78d6", 60.0: "#eb6834", 90.0: "#1baf7a"}     # dataviz 기본 범주 색 1~3 (하중)


def plot(path, rows, mults, masses, best):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": ["Noto Sans CJK KR", "NanumGothic", "sans-serif"], "font.size": 10,
                         "axes.edgecolor": P.GRID, "axes.labelcolor": P.INK2, "xtick.color": P.INK2, "ytick.color": P.INK2,
                         "axes.facecolor": P.SURFACE, "figure.facecolor": P.SURFACE, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(10, 5.6))
    ax.axhline(0, color=P.GRID, lw=1, zorder=0)
    ax.axvline(best, color=P.MUTED, lw=1, ls=":", zorder=0)
    ax.text(best / 1.04, 0.03, f"×{best:g}: {masses[0]:.0f} kg에서 이어 누른 자리와 맞음", transform=ax.get_xaxis_transform(), ha="right",
            va="bottom", color=P.INK2, fontsize=9)
    for mass in sorted(masses):
        c = LOAD_COLORS.get(mass, P.INK2)
        xs = sorted(mults)
        ys = [rows[mass, m][1] * 100 for m in xs]
        real = rows[mass, xs[0]][0] * 100
        ax.plot(xs, ys, color=c, lw=2, marker="o", ms=5, label=f"{mass:.0f} kg: 베낀 자리 + 단단한 흙", zorder=3)
        ax.axhline(real, color=c, lw=1.5, ls=(0, (4, 3)), zorder=2)
        ax.annotate(f"{ys[0]:.2f}", (xs[0], ys[0]), xytext=(7, 0), textcoords="offset points", va="center",
                    color=P.INK, fontsize=8.5)
    from matplotlib.lines import Line2D
    reals = {mass: rows[mass, min(mults)][0] * 100 for mass in masses}
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Line2D([], [], color=P.MUTED, lw=1.5, ls=(0, (4, 3))))
    labels.append("점선 = 같은 하중의 이어 누른 자리 (단일 SCM 기준):\n" + ", ".join(f"{m:.0f} kg {reals[m]:.2f}" for m in sorted(masses)) + " cm")
    ax.set_xscale("log")
    ticks = [m for m in sorted(mults) if m != best]
    ax.set_xticks(ticks)
    ax.set_xticklabels([f"×{m:g}" for m in ticks])
    ax.set_xlabel("베낀 자리에서 콜백이 답하는 kphi 배수 (×1 = 콜백 효과 없음)")
    ax.set_ylabel("다시 누를 때 가라앉은 양 (cm)")
    ax.grid(axis="y", color=P.GRID, lw=0.6)
    ax.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.28, 0.97), frameon=False, fontsize=9)
    ax.set_title(f"다짐 -> 단단한 흙: kphi 약 ×{best:g}에서 베낀 자리(실선)가 이어 누른 자리(점선, 단일 SCM 기준)와 맞음",
                 loc="left", color=P.INK, fontsize=11)
    fig.text(0.01, -0.02, "이어 누른 자리 = 같은 SCM에서 60 kg 구로 한 번 누른 뒤 이어서 다시 누름 (단일 SCM 기준, 실측 아님), 베낀 자리 = 그 높이만 새 흙에 넣은 자리 (첫 시험 probe_scm_injection.py의 3, 4번). 같은 하중으로 누름.\n90 kg은 다질 때보다 무거워 "
             "같은 SCM 안에서 다져진 흙은 다시 가라앉지만, 단단한 흙은 하중과 상관없이 단단하다.\n"
             "음수 = 구가 누르기 전 가운데 높이보다 위에서 멈춤 (홈 벽에 얹힘). Chrono SCM, kphi 5e6, 2 cm 격자 "
             "(conformance/probe_scm_compaction.py)", color=P.INK2, fontsize=8, va="top")
    fig.savefig(path, dpi=130, bbox_inches="tight")


if __name__ == "__main__":
    main()
