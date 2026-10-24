"""SCM 다짐 우회 시험 "가상 다짐" (docs/map_server.md 5.5): 높이를 넣은 자리를 다질 때와 같은 하중으로 한 번 눌러 두면,
그 SCM이 스스로 다짐 상태(sigma_yield)를 갖게 되어 이후에는 같은 SCM에서 이어 누른 자리처럼 움직이는가. 흙 성질은 원래 그대로 (콜백 없음).

    python conformance/probe_scm_virtual_compaction.py [--masses 40 60 90] [--rounds 4] [--plot docs/img/scm_virtual_compaction.png]
    (어느 환경이든: pychrono가 없으면 chrono 환경으로 다시 실행)

probe_scm_injection.py와 같은 흙·구. 두 자리 (그 시험의 3, 4번 자리와 같다). 한 번 실행마다 새 SCM:
  이어 누른 자리      60 kg으로 한 번 눌러 다진다 (같은 SCM 안에서) -> 그 높이를 베껴 두고 -> 같은 자리를 하중 M으로 다시 누름
  가상 다짐 자리  이어 누른 자리의 높이를 새 흙에 넣는다 -> 60 kg으로 한 번 눌러 둔다 (가상 다짐) -> 하중 M으로 누름
가상 다짐은 눌러 두는 동안 흙이 더 내려간다. 그래서 두 방식:
  보정 없음  베낀 높이를 그대로 넣는다 (가상 다짐 뒤 이어 누른 자리보다 깊어진다)
  보정 있음  눌러 둘 때 내려가는 양을 미리 재서(따로 돌림) 그만큼 올려 넣는다. --rounds번 되풀이해 맞춘다
--plot: 세 방법(높이만, 단단한 흙 ×25, 가상 다짐)을 하중별로 이어 누른 자리와 비교 (단단한 흙은 probe_scm_compaction.py로 함께 잰다, 약 40초 더).
이 PC (2026-10-24, 보정 4회, 약 15초): 이어 누른 자리 / 가상 다짐 자리 침하 40 kg -0.02 / -0.01, 60 kg 0.02 / 0.07, 90 kg 0.22 / 0.35 cm.
깊이(처음 흙 기준) 60 kg 3.22 / 3.28 cm. 보정 없으면 깊이 4.69 cm (눌러 두는 동안 1.42 cm 더 내려감).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_scm_injection as P  # noqa: E402  (pychrono가 없으면 여기서 chrono 환경으로 다시 실행된다)

chrono = P.chrono
COMPACT_MASS = 60.0
MASSES = [40.0, 60.0, 90.0]


def compact_and_copy(p):
    """이어 누른 자리를 60 kg으로 다지고, 그 자리 노드 높이를 (가운데 기준 칸 차이 -> 높이)로 돌려준다."""
    p.ball.SetMass(COMPACT_MASS)
    p.press(P.SPOTS[3])
    c3 = P.node(P.SPOTS[3])
    return {(nd.x - c3[0], nd.y - c3[1]): h for nd, h in p.ter.GetModifiedNodes(True)
            if abs(nd.x - c3[0]) <= P.HALF and abs(nd.y - c3[1]) <= P.HALF}


def heights(p, center, keys):
    c = P.node(center)
    return {k: p.ter.GetHeight(chrono.ChVector3d((c[0] + k[0]) * P.DELTA, (c[1] + k[1]) * P.DELTA, 0)) for k in keys}


def virtual_compact(p, copied, offset):
    """가상 다짐 자리에 (베낀 높이 + offset)을 넣고 60 kg으로 눌러 둔다. 반환: 눌러 둔 뒤 노드 높이."""
    c4 = P.node(P.SPOTS[4])
    p.inject({(c4[0] + di, c4[1] + dj): h + offset.get((di, dj), 0.0) for (di, dj), h in copied.items()}, P.SPOTS[4])
    p.ball.SetMass(COMPACT_MASS)
    p.press(P.SPOTS[4])
    return heights(p, P.SPOTS[4], copied.keys())


def calibrate(rounds=2):
    """눌러 둔 뒤 베낀 높이와 같아지도록 미리 올려 넣을 양 (노드마다). 매번 새 SCM으로 돌려 맞춘다."""
    offset = {}
    for r in range(rounds):
        p = P.Probe()
        copied = compact_and_copy(p)
        after = virtual_compact(p, copied, offset)
        err = {k: copied[k] - after[k] for k in copied}             # 눌러 둔 뒤 아직 더 내려간 양
        offset = {k: offset.get(k, 0.0) + err[k] for k in copied}
        print(f"  보정 {r + 1}회: 눌러 둔 뒤 베낀 높이와의 차이 최대 {max(abs(v) for v in err.values()) * 100:.2f} cm, "
              f"가운데 {err[(0, 0)] * 100:+.2f} cm", flush=True)
    return offset


def run(mass, offset):
    """반환: (이어 누른 자리 침하, 이어 누른 자리 깊이, 가상 다짐 자리 침하, 가상 다짐 자리 깊이, 눌러 둔 뒤 모양 차이 최대)."""
    p = P.Probe()
    copied = compact_and_copy(p)
    p.ball.SetMass(mass)
    h3, b3 = p.press(P.SPOTS[3])
    after = virtual_compact(p, copied, offset)
    shape_err = max(abs(after[k] - copied[k]) for k in copied)
    p.ball.SetMass(mass)
    h4, b4 = p.press(P.SPOTS[4])
    return h3 - b3, -b3, h4 - b4, -b4, shape_err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--masses", type=float, nargs="+", default=MASSES, help="다시 누르는 하중 (kg)")
    ap.add_argument("--rounds", type=int, default=4, help="높이 보정 되풀이 횟수 (4회면 모양 차이 0.06 cm)")
    ap.add_argument("--plot", metavar="PNG", help="세 방법 비교 그림")
    a = ap.parse_args()
    print("보정 (눌러 둘 때 내려가는 양 재기):")
    offset = calibrate(a.rounds)
    virtual = {}
    for name, off in (("보정 없음", {}), ("보정 있음", offset)):
        print(f"\n{name}:")
        for mass in a.masses:
            s3, d3, s4, d4, e = run(mass, off)
            if off:
                virtual[mass] = (s3, s4)
            print(f"  하중 {mass:3.0f} kg  이어 누른 자리 {s3 * 100:5.2f} cm (깊이 {d3 * 100:.2f}) | 가상 다짐 자리 {s4 * 100:5.2f} cm "
                  f"(깊이 {d4 * 100:.2f}) | 눌러 둔 뒤 모양 차이 최대 {e * 100:.2f} cm", flush=True)
    if a.plot:
        plot(a.plot, a.masses, virtual)
        print(f"그림: {a.plot}")


def plot(path, masses, virtual, stiff_mult=25):
    """하중별: 이어 누른 자리(기준), 높이만, 단단한 흙 ×stiff_mult, 가상 다짐. 왼쪽은 침하, 오른쪽은 이어 누른 자리 대비 차이."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import probe_scm_compaction as C
    print("비교용: 높이만, 단단한 흙", flush=True)
    only = {m: C.run(1, m)[1] for m in masses}
    stiff = {m: C.run(stiff_mult, m)[1] for m in masses}
    real = {m: virtual[m][0] for m in masses}
    virt = {m: virtual[m][1] for m in masses}
    methods = [("높이만 넣음", only, "#2a78d6"), (f"높이 + 단단한 흙 ×{stiff_mult}", stiff, "#eb6834"),
               ("높이 + 가상 다짐", virt, "#1baf7a")]                        # dataviz 기본 범주 색 1~3
    plt.rcParams.update({"font.family": ["Noto Sans CJK KR", "NanumGothic", "sans-serif"], "font.size": 10,
                         "axes.edgecolor": P.GRID, "axes.labelcolor": P.INK2, "xtick.color": P.INK2, "ytick.color": P.INK2,
                         "axes.facecolor": P.SURFACE, "figure.facecolor": P.SURFACE, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={"width_ratios": [1.15, 1], "wspace": 0.25})
    x = np.arange(len(masses))
    w = 0.2
    groups = [("이어 누른 자리 (단일 SCM 기준)", real, P.MUTED)] + methods
    for k, (name, vals, c) in enumerate(groups):
        ys = [vals[m] * 100 for m in masses]
        xs = x + (k - 1.5) * w
        ax.bar(xs, ys, w * 0.9, color=c, label=name, zorder=2)
        for xi, yi in zip(xs, ys):
            ax.text(xi, max(yi, 0) + 0.03, f"{yi:.2f}", ha="center", va="bottom", fontsize=8, color=P.INK)
    ax.axhline(0, color=P.INK2, lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{m:.0f} kg" + (" (다질 때와 같음)" if m == COMPACT_MASS else "") for m in masses])
    ax.set_ylabel("다시 누를 때 가라앉은 양 (cm)")
    ax.grid(axis="y", color=P.GRID, lw=0.6, zorder=0)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.set_title("하중별 침하 (같은 SCM에서 60 kg으로 다진 뒤 이어 누른 결과를 세 방법으로 흉내)", loc="left", color=P.INK, fontsize=11)
    for k, (name, vals, c) in enumerate(methods[1:]):
        ys = [(vals[m] - real[m]) * 100 for m in masses]
        xs = x + (k - 0.5) * 0.28
        bx.bar(xs, ys, 0.25, color=c, label=name, zorder=2)
        for xi, yi in zip(xs, ys):
            bx.text(xi, yi + (0.008 if yi >= 0 else -0.008), f"{yi:+.2f}", ha="center", va="bottom" if yi >= 0 else "top",
                    fontsize=8.5, color=P.INK)
    bx.axhline(0, color=P.INK2, lw=0.8)
    lim = max(abs((vals[m] - real[m]) * 100) for _, vals, _ in methods[1:] for m in masses) * 1.45
    bx.set_ylim(-lim, lim)
    bx.text(1.0, 0.98, "더 가라앉음 (보수적)", transform=bx.transAxes, ha="right", va="top", color=P.INK2, fontsize=9)
    bx.text(1.0, 0.02, "덜 가라앉음 (낙관적)", transform=bx.transAxes, ha="right", va="bottom", color=P.INK2, fontsize=9)
    bx.set_xticks(x)
    bx.set_xticklabels([f"{m:.0f} kg" for m in masses])
    bx.set_ylabel("이어 누른 자리 대비 차이 (cm)")
    bx.grid(axis="y", color=P.GRID, lw=0.6, zorder=0)
    bx.legend(loc="upper left", frameon=False, fontsize=9)
    only_err = ", ".join(f"{m:.0f} kg {(only[m] - real[m]) * 100:+.2f}" for m in masses)
    bx.set_title("이어 누른 자리 대비 차이 (0에 가까울수록 맞음)", loc="left", color=P.INK, fontsize=11)
    bx.text(0.02, 0.80, f"높이만 넣음은 범위 밖:\n{only_err} cm", transform=bx.transAxes, ha="left", va="top",
            color=P.INK2, fontsize=8.5)
    fig.text(0.01, -0.02, "이어 누른 자리 = 같은 SCM에서 60 kg 구로 한 번 누른 뒤 이어서 다시 누름 (단일 SCM 기준, 실측 아님). 나머지는 그 높이를 새 흙에 넣은 자리: 높이만 / 콜백이 kphi를 키움 / "
             "60 kg으로 한 번 눌러 둠 (높이는 미리 올려 넣어 보정).\n가상 다짐은 SCM이 눌러 둔 압력을 스스로 기억해 다질 때 하중을 넘으면 "
             "다시 파인다 (90 kg). Chrono SCM, kphi 5e6, 2 cm 격자 (conformance/probe_scm_virtual_compaction.py)",
             color=P.INK2, fontsize=8, va="top")
    fig.savefig(path, dpi=130, bbox_inches="tight")


if __name__ == "__main__":
    main()
