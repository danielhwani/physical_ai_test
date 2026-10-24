"""SCM 시험 기록(.npz, scm_record.py 규약)에서 분석 그래프를 그린다. 시뮬레이션을 다시 돌리지 않는다. pychrono 없이 돈다.

    python conformance/plot_scm_probe.py build/scm_rolling.npz                       # 그림: docs/img/<시험 기본 이름>.png
    python conformance/plot_scm_probe.py build/scm_compaction.npz -o /tmp/x.png

기록 만들기: python conformance/probe_scm_<시험>.py --record build/<이름>.npz (--plot을 주면 기록을 저장한 뒤 이 모듈로 그린다).
같은 기록을 Godot로 재생: python conformance/replay_scm_probe.py build/<이름>.npz (누르기·굴리기 기록만).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scm_record  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
INK, INK2, MUTED, SURFACE, GRID = "#0b0b0b", "#52514e", "#8f8d86", "#fcfcfb", "#e4e3df"
DEFAULT_PNG = {"press": "docs/img/scm_injection.png", "compaction": "docs/img/scm_compaction.png",
               "virtual_compaction": "docs/img/scm_virtual_compaction.png", "rolling": "docs/img/scm_rolling.png"}


def _korean_font():
    """한글 글꼴 이름. matplotlib이 시스템 글꼴을 모르는 환경(예: mj_ros)이 있어 fontconfig로 파일을 찾아 등록한다."""
    import subprocess
    from matplotlib import font_manager
    for family in ("Noto Sans CJK KR", "NanumGothic"):
        try:
            path = subprocess.run(["fc-match", "-f", "%{file}", f"{family}:lang=ko"], capture_output=True, text=True,
                                  timeout=5).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            continue
        if path and Path(path).exists():
            font_manager.fontManager.addfont(path)
            return font_manager.FontProperties(fname=path).get_name()
    return "sans-serif"


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": [_korean_font(), "sans-serif"], "font.size": 10,
                         "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "axes.facecolor": SURFACE, "figure.facecolor": SURFACE, "axes.spines.top": False,
                         "axes.spines.right": False})
    return plt


PRESS_COLORS = {1: "#2a78d6", 2: "#eb6834", 3: "#1baf7a", 4: "#eda100"}      # 범주 순서 고정 (dataviz 기본 팔레트 1~4)


def plot_press(rec, path):
    """probe_scm_injection.py: 위는 누르는 동안 가라앉은 양(시간), 아래는 자리마다 단면."""
    plt = _pyplot()
    a, cfg = rec["analysis"], rec["config"]
    R, HOLE = cfg["radius"], cfg["hole"]
    traces = [(np.array(t), np.array(z)) for t, z in a["traces"]]
    prof = {int(k): ((np.array(v[0]), np.array(v[1])), (np.array(v[2]), np.array(v[3])), v[4], v[5]) for k, v in a["prof"].items()}
    LABELS = {1: "1 새 흙", 2: f"2 높이만 {HOLE * 100:.0f} cm 낮춤", 3: "3 이어 누른 자리", 4: "4 3의 높이만 베낌"}
    COLORS = PRESS_COLORS
    fig = plt.figure(figsize=(11, 7.2))
    gs = fig.add_gridspec(2, 4, height_ratios=[1.2, 1], hspace=0.55, wspace=0.08)
    ax = fig.add_subplot(gs[0, :])
    for k, (t, z) in zip((1, 2, 3, 4), traces):      # 2는 1과 겹치므로 점선으로 위에 그린다
        ax.plot(t, -z * 100, color=COLORS[k], lw=3.5 if k == 1 else 2, label=LABELS[k], zorder=3 if k == 2 else 2,
                ls=(0, (4, 3)) if k == 2 else "-")
        if k != 2:
            ax.annotate(("1, 2: " if k == 1 else "") + f"{-z[-1] * 100:.2f} cm", (t[-1], -z[-1] * 100), xytext=(6, 0),
                        textcoords="offset points", va="center", color=INK, fontsize=9)
    ax.invert_yaxis()
    ax.set_xlim(0, traces[0][0][-1] * 1.15)
    ax.set_xlabel("누른 시간 (s)")
    ax.set_ylabel("가라앉은 양 (cm)")
    ax.grid(axis="y", color=GRID, lw=0.8)
    ax.legend(loc="upper right", bbox_to_anchor=(0.87, 0.9), frameon=False, ncol=2)
    ax.set_title("같은 무게(60 kg 구)로 눌렀을 때 가라앉은 양: 1과 2는 겹친다 (넣은 높이를 접촉이 따라감), 4는 3의 약 70배",
                 loc="left", color=INK, fontsize=11)
    lo = min(min(a[1].min(), bot) for (b, a, bot, _) in prof.values()) * 100 - 2.2
    th = np.linspace(0, 2 * np.pi, 200)
    for k in (1, 2, 3, 4):
        (xb, hb), (xa, ha), bot, steps = prof[k]
        sx = fig.add_subplot(gs[1, k - 1])
        sx.axhline(0, color=GRID, lw=1, zorder=0)                                       # 처음 흙 (평평)
        sx.fill_between(xa * 100, hb * 100, ha * 100, where=ha < hb - 1e-5, color=COLORS[k], alpha=0.45, lw=0,
                        interpolate=True)                                               # 이번 누름이 더 판 부분
        sx.plot(xb * 100, hb * 100, color=INK2, lw=1.3, ls=(0, (3, 2)))
        sx.plot(xa * 100, ha * 100, color=COLORS[k], lw=2)
        sx.plot((R * np.cos(th)) * 100, (bot + R + R * np.sin(th)) * 100, color=MUTED, lw=1)   # 멈춘 구 (세로가 늘어나 보임)
        sx.set_xlim(-25, 25)
        sx.set_ylim(lo, 2.5)
        sx.set_title(LABELS[k], loc="left", color=INK, fontsize=10, pad=18)
        sx.text(0, 1.02, " → ".join("0" if v == 0 else f"{v * 100:.2f}".replace("-", "−") for v in steps) + " cm",
                transform=sx.transAxes, ha="left", va="bottom", color=INK2, fontsize=8.5)
        sx.text(-24, lo + 0.3, f"이번 누름 +{(steps[-2] - bot) * 100:.2f} cm\n처음 흙 기준 깊이 {-bot * 100:.2f} cm",
                ha="left", va="bottom", color=INK, fontsize=9.5, fontweight="bold", linespacing=1.4)
        sx.set_xlabel("x (cm)")
        sx.grid(axis="y", color=GRID, lw=0.6)
        if k == 1:
            sx.set_ylabel("높이 (cm, 세로 확대)")
        else:
            sx.tick_params(labelleft=False)
    fig.text(0.01, -0.02, "아래: 구를 지나는 x 방향 단면 (세로 확대). 회색 가로선 = 처음 흙, 회색 점선 = 이번 누름 직전, 실선 = 누른 뒤, "
             "색 띠 = 이번 누름이 더 판 부분, 원 = 멈춘 구. 3과 4는 누르기 직전 모양이 같다.\n"
             "제목 아래 = 처음 흙 기준 높이 변화 (1: 누름, 2: 넣음 -> 누름, 3: 첫 누름 -> 되튐 -> 다시 누름, 4: 3의 되튄 표면을 넣음 -> 누름).\n"
             "Chrono SCM, kphi 5e6 (vehicle_crossing.yaml 흙), 2 cm 격자 (conformance/probe_scm_injection.py)",
             color=INK2, fontsize=8, va="top")
    fig.savefig(path, dpi=130, bbox_inches="tight")


LOAD_COLORS = {40.0: "#2a78d6", 60.0: "#eb6834", 90.0: "#1baf7a"}     # dataviz 기본 범주 색 1~3 (하중)


def plot_compaction(rec, path):
    """probe_scm_compaction.py: kphi 배수에 따른 베낀 자리 침하, 하중별 (점선 = 이어 누른 자리)."""
    plt = _pyplot()
    a = rec["analysis"]
    rows = {(r[0], r[1]): r[2:] for r in a["rows"]}
    mults, masses, best = a["mults"], a["masses"], a["best"]
    fig, ax = plt.subplots(figsize=(10, 5.6))
    ax.axhline(0, color=GRID, lw=1, zorder=0)
    ax.axvline(best, color=MUTED, lw=1, ls=":", zorder=0)
    ax.text(best / 1.04, 0.03, f"×{best:g}: {masses[0]:.0f} kg에서 이어 누른 자리와 맞음", transform=ax.get_xaxis_transform(), ha="right",
            va="bottom", color=INK2, fontsize=9)
    for mass in sorted(masses):
        c = LOAD_COLORS.get(mass, INK2)
        xs = sorted(mults)
        ys = [rows[mass, m][1] * 100 for m in xs]
        real = rows[mass, xs[0]][0] * 100
        ax.plot(xs, ys, color=c, lw=2, marker="o", ms=5, label=f"{mass:.0f} kg: 베낀 자리 + 단단한 흙", zorder=3)
        ax.axhline(real, color=c, lw=1.5, ls=(0, (4, 3)), zorder=2)
        ax.annotate(f"{ys[0]:.2f}", (xs[0], ys[0]), xytext=(7, 0), textcoords="offset points", va="center",
                    color=INK, fontsize=8.5)
    from matplotlib.lines import Line2D
    reals = {mass: rows[mass, min(mults)][0] * 100 for mass in masses}
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Line2D([], [], color=MUTED, lw=1.5, ls=(0, (4, 3))))
    labels.append("점선 = 같은 하중의 이어 누른 자리 (단일 SCM 기준):\n" + ", ".join(f"{m:.0f} kg {reals[m]:.2f}" for m in sorted(masses)) + " cm")
    ax.set_xscale("log")
    ticks = [m for m in sorted(mults) if m != best]
    ax.set_xticks(ticks)
    ax.set_xticklabels([f"×{m:g}" for m in ticks])
    ax.set_xlabel("베낀 자리에서 콜백이 답하는 kphi 배수 (×1 = 콜백 효과 없음)")
    ax.set_ylabel("다시 누를 때 가라앉은 양 (cm)")
    ax.grid(axis="y", color=GRID, lw=0.6)
    ax.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.28, 0.97), frameon=False, fontsize=9)
    ax.set_title(f"다짐 -> 단단한 흙: kphi 약 ×{best:g}에서 베낀 자리(실선)가 이어 누른 자리(점선, 단일 SCM 기준)와 맞음",
                 loc="left", color=INK, fontsize=11)
    fig.text(0.01, -0.02, "이어 누른 자리 = 같은 SCM에서 60 kg 구로 한 번 누른 뒤 이어서 다시 누름 (단일 SCM 기준, 실측 아님), 베낀 자리 = 그 높이만 새 흙에 넣은 자리 (첫 시험 probe_scm_injection.py의 3, 4번). 같은 하중으로 누름.\n90 kg은 다질 때보다 무거워 "
             "같은 SCM 안에서 다져진 흙은 다시 가라앉지만, 단단한 흙은 하중과 상관없이 단단하다.\n"
             "음수 = 구가 누르기 전 가운데 높이보다 위에서 멈춤 (홈 벽에 얹힘). Chrono SCM, kphi 5e6, 2 cm 격자 "
             "(conformance/probe_scm_compaction.py)", color=INK2, fontsize=8, va="top")
    fig.savefig(path, dpi=130, bbox_inches="tight")


def plot_virtual(rec, path):
    """probe_scm_virtual_compaction.py: 하중별 이어 누른 자리(기준), 높이만, 단단한 흙 ×N, 가상 다짐. 왼쪽 침하, 오른쪽 기준 대비 차이."""
    plt = _pyplot()
    a, cfg = rec["analysis"], rec["config"]
    COMPACT_MASS = cfg["compact_mass"]
    masses, stiff_mult = a["masses"], a["stiff_mult"]
    real, virt, only, stiff = ({float(m): x for m, x in a[k].items()} for k in ("reference", "virtual", "heights_only", "stiff"))
    methods = [("높이만 넣음", only, "#2a78d6"), (f"높이 + 단단한 흙 ×{stiff_mult}", stiff, "#eb6834"),
               ("높이 + 가상 다짐", virt, "#1baf7a")]                        # dataviz 기본 범주 색 1~3
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={"width_ratios": [1.15, 1], "wspace": 0.25})
    x = np.arange(len(masses))
    w = 0.2
    groups = [("이어 누른 자리 (단일 SCM 기준)", real, MUTED)] + methods
    for k, (name, vals, c) in enumerate(groups):
        ys = [vals[m] * 100 for m in masses]
        xs = x + (k - 1.5) * w
        ax.bar(xs, ys, w * 0.9, color=c, label=name, zorder=2)
        for xi, yi in zip(xs, ys):
            ax.text(xi, max(yi, 0) + 0.03, f"{yi:.2f}", ha="center", va="bottom", fontsize=8, color=INK)
    ax.axhline(0, color=INK2, lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{m:.0f} kg" + (" (다질 때와 같음)" if m == COMPACT_MASS else "") for m in masses])
    ax.set_ylabel("다시 누를 때 가라앉은 양 (cm)")
    ax.grid(axis="y", color=GRID, lw=0.6, zorder=0)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.set_title("하중별 침하 (같은 SCM에서 60 kg으로 다진 뒤 이어 누른 결과를 세 방법으로 흉내)", loc="left", color=INK, fontsize=11)
    for k, (name, vals, c) in enumerate(methods[1:]):
        ys = [(vals[m] - real[m]) * 100 for m in masses]
        xs = x + (k - 0.5) * 0.28
        bx.bar(xs, ys, 0.25, color=c, label=name, zorder=2)
        for xi, yi in zip(xs, ys):
            bx.text(xi, yi + (0.008 if yi >= 0 else -0.008), f"{yi:+.2f}", ha="center", va="bottom" if yi >= 0 else "top",
                    fontsize=8.5, color=INK)
    bx.axhline(0, color=INK2, lw=0.8)
    lim = max(abs((vals[m] - real[m]) * 100) for _, vals, _ in methods[1:] for m in masses) * 1.45
    bx.set_ylim(-lim, lim)
    bx.text(1.0, 0.98, "더 가라앉음 (보수적)", transform=bx.transAxes, ha="right", va="top", color=INK2, fontsize=9)
    bx.text(1.0, 0.02, "덜 가라앉음 (낙관적)", transform=bx.transAxes, ha="right", va="bottom", color=INK2, fontsize=9)
    bx.set_xticks(x)
    bx.set_xticklabels([f"{m:.0f} kg" for m in masses])
    bx.set_ylabel("이어 누른 자리 대비 차이 (cm)")
    bx.grid(axis="y", color=GRID, lw=0.6, zorder=0)
    bx.legend(loc="upper left", frameon=False, fontsize=9)
    only_err = ", ".join(f"{m:.0f} kg {(only[m] - real[m]) * 100:+.2f}" for m in masses)
    bx.set_title("이어 누른 자리 대비 차이 (0에 가까울수록 맞음)", loc="left", color=INK, fontsize=11)
    bx.text(0.02, 0.80, f"높이만 넣음은 범위 밖:\n{only_err} cm", transform=bx.transAxes, ha="left", va="top",
            color=INK2, fontsize=8.5)
    fig.text(0.01, -0.02, "이어 누른 자리 = 같은 SCM에서 60 kg 구로 한 번 누른 뒤 이어서 다시 누름 (단일 SCM 기준, 실측 아님). 나머지는 그 높이를 새 흙에 넣은 자리: 높이만 / 콜백이 kphi를 키움 / "
             "60 kg으로 한 번 눌러 둠 (높이는 미리 올려 넣어 보정).\n가상 다짐은 SCM이 눌러 둔 압력을 스스로 기억해 다질 때 하중을 넘으면 "
             "다시 파인다 (90 kg). Chrono SCM, kphi 5e6, 2 cm 격자 (conformance/probe_scm_virtual_compaction.py)",
             color=INK2, fontsize=8, va="top")
    fig.savefig(path, dpi=130, bbox_inches="tight")


ROLL_NAMES = ["A 이어 굴린 줄 (단일 SCM 기준)", "B 높이만 베낀 줄", "C 가상 다짐 줄 (굴려서 다짐)"]
ROLL_COLORS = [MUTED, "#2a78d6", "#1baf7a"]          # 기준은 회색, 나머지는 dataviz 범주 색 (probe_scm_virtual_compaction 그림과 같은 뜻의 색)


def plot_rolling(rec, path):
    """probe_scm_rolling.py: 위는 두 번째 굴림을 x로 따라간 추가 침하와 끄는 힘, 아래는 하중별 구간 평균."""
    plt = _pyplot()
    a, cfg = rec["analysis"], rec["config"]
    MID, COMPACT_MASS = tuple(cfg["mid"]), cfg["compact_mass"]
    NAMES, COLORS = ROLL_NAMES, ROLL_COLORS
    results = {float(m): x for m, x in a["results"].items()}
    traces = {float(m): x for m, x in a["traces"].items()}
    masses = sorted(results)
    m0 = masses[0]
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), gridspec_kw={"hspace": 0.45, "wspace": 0.22})
    (ax, fx), (sx, rx) = axs
    for k, tr in enumerate(traces[m0]):
        x, bot, before, f = (np.array(v) for v in zip(*tr))
        ax.plot(x, (before - bot) * 100, color=COLORS[k], lw=2.5 if k == 0 else 1.8, label=NAMES[k], zorder=3 - k * 0.1)
        fx.plot(x, f, color=COLORS[k], lw=2.5 if k == 0 else 1.6, label=NAMES[k])
    for a_ in (ax, fx):
        a_.axvspan(*MID, color=GRID, alpha=0.5, lw=0, zorder=0)
        a_.set_xlabel("x (m)")
        a_.grid(axis="y", color=GRID, lw=0.6)
    ax.text(np.mean(MID), 0.02, "재는 구간", transform=ax.get_xaxis_transform(), ha="center", va="bottom", color=INK2, fontsize=8.5)
    ax.set_ylabel("추가 침하 (cm)")
    ax.set_title(f"두 번째 굴림 ({m0:.0f} kg): 굴리기 전 표면 - 구 바닥", loc="left", color=INK, fontsize=11)
    ax.legend(loc="center right", bbox_to_anchor=(1.0, 0.55), frameon=False, fontsize=9)
    fx.set_ylabel("끄는 힘 = 구름 저항 (N)")
    fx.set_title(f"두 번째 굴림 ({m0:.0f} kg): 끄는 힘", loc="left", color=INK, fontsize=11)
    xs = np.arange(len(masses))
    w = 0.26
    for k in range(3):
        sinks = [results[m][2 + k][0] * 100 for m in masses]
        forces = [results[m][2 + k][1] for m in masses]
        for bx_, vals, fmt in ((sx, sinks, "{:.2f}"), (rx, forces, "{:.1f}")):
            pos = xs + (k - 1) * w
            bx_.bar(pos, vals, w * 0.9, color=COLORS[k], label=NAMES[k], zorder=2)
            for p_, v in zip(pos, vals):
                bx_.text(p_, v, fmt.format(v), ha="center", va="bottom", fontsize=8.5, color=INK)
    first = results[m0]
    for bx_, ylabel, title, ref in ((sx, "추가 침하 (cm)", f"가운데 구간 평균 추가 침하 (점선 = 첫 굴림 60 kg 새 흙 {first[0] * 100:.2f} cm)",
                                     first[0] * 100),
                                    (rx, "구름 저항 (N)", f"가운데 구간 평균 구름 저항 (점선 = 첫 굴림 60 kg 새 흙 {first[1]:.1f} N)", first[1])):
        bx_.axhline(ref, color=INK2, lw=1, ls=":", zorder=1)
        bx_.set_xticks(xs)
        bx_.set_xticklabels([f"두 번째 굴림 {m:.0f} kg" + (" (첫 굴림과 같음)" if m == COMPACT_MASS else "") for m in masses])
        bx_.set_ylabel(ylabel)
        bx_.set_title(title, loc="left", color=INK, fontsize=10.5)
        bx_.grid(axis="y", color=GRID, lw=0.6, zorder=0)
    fig.text(0.01, 0.0, "구(반지름 10 cm)를 0.3 m/s로 끌어 굴림 (위아래·회전 자유). A = 같은 SCM에서 60 kg으로 한 번 굴린 자국을 이어서 다시 굴림 "
             "(단일 SCM 기준, 실측 아님), B = 그 자국 높이만 새 흙에 넣음,\nC = 높이(+ 보정)를 넣고 60 kg으로 한 번 굴려 다진 뒤 굴림. "
             "Chrono SCM, kphi 5e6, 2 cm 격자 (conformance/probe_scm_rolling.py)", color=INK2, fontsize=8, va="top")
    fig.savefig(path, dpi=130, bbox_inches="tight")


PLOTTERS = {"press": plot_press, "compaction": plot_compaction, "virtual_compaction": plot_virtual, "rolling": plot_rolling}


def plot(record, png=None):
    """기록 파일 -> 그림. 반환: 그림 경로."""
    rec = scm_record.load(record)
    png = png or str(ROOT / DEFAULT_PNG[rec["kind"]])
    PLOTTERS[rec["kind"]](rec, png)
    return png


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("record", help="시험 기록 (.npz)")
    ap.add_argument("-o", "--out", help="그림 (기본: docs/img/<시험 기본 이름>.png)")
    a = ap.parse_args()
    print(f"그림: {plot(a.record, a.out)}")


if __name__ == "__main__":
    main()
