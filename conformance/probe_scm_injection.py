"""Chrono SCM에 바깥에서 높이를 넣을 때 (SCMTerrain.SetModifiedNodes) 접촉과 흙 상태가 어떻게 되는지 재는 시험
(docs/map_server.md 5.5, 9절). pychrono가 필요하다: 다른 환경(mj_ros 등)에서 실행하면 chrono 환경의 python으로 다시 실행한다.

    python conformance/probe_scm_injection.py [--plot docs/img/scm_injection.png] [--record build/scm_probe.npz]

구(반지름 R, 무게 MASS)를 흙 위에 올려 무게로 가라앉힌다. 흙은 scenarios/vehicle_crossing.yaml과 같다. 한 SCM의 네 자리:
  1 새 흙
  2 SetModifiedNodes로 3 cm 낮춘 판 (구 접촉 원보다 넓은 0.4 m 사각형)
  3 실제로 한 번 누른 자리를 다시 누름
  4 3의 첫 누름 뒤 높이만 새 흙에 베껴 넣고 누름
판정: 2는 1보다 정확히 3 cm 낮게 멈춘다 (넣은 높이를 접촉이 따라감). 4는 3보다 훨씬 많이 가라앉는다 (높이만 넣으면 그 노드는
새 흙이 되어 다짐 이력 sigma_yield가 넘어가지 않음). 이 PC (2026-10-24): 1 3.22 cm, 2 3.22 cm(바닥 -6.22), 3 추가 0.02 cm,
4 추가 1.44 cm. 약 4초.
--plot: 위는 누르는 동안 구가 가라앉은 양(시간), 아래는 자리마다 누르기 전·후 흙 단면(구를 지나는 x 방향)과 멈춘 구.
--record: 흙 높이 격자와 구 위치를 약 30 Hz로 기록 (Godot 재생: conformance/replay_scm_probe.py).
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

try:
    import pychrono as chrono
    import pychrono.vehicle as veh
except ModuleNotFoundError:                    # 다른 환경(mj_ros 등)에서 실행: chrono 환경의 python으로 다시 실행
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from sim.chrono_link import find_chrono_python
    py = find_chrono_python({})
    os.execv(py, [py, *sys.argv])

R, MASS, DELTA, STEP = 0.10, 60.0, 0.02, 5e-4
SIZE = (2.4, 1.6)          # SCM 크기 (m), 원점 중심
REC_EVERY = 66             # 기록 간격 (스텝, 약 33 ms)
LIFT = 0.3                 # 기록용: 뗄 때 구를 들어 올리는 높이 (m)
SOIL = dict(kphi=5.0e6, kc=0, n=1.1, coh=0, phi=30, js=0.01, ek=4.0e7, dr=3.0e4)   # vehicle_crossing.yaml과 같은 흙
SPOTS = {1: (-0.7, -0.35), 2: (0.0, -0.35), 3: (-0.7, 0.35), 4: (0.0, 0.35)}
HOLE = 0.03                # 2에서 넣는 판 깊이 (m)
HALF = 10                  # 2와 4에서 넣는 사각형 반폭 (노드)


class Probe:
    def __init__(self):
        self.sys = chrono.ChSystemSMC()
        self.sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # SCM 레이캐스트에 필요
        self.sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
        self.sys.SetNumThreads(1)
        self.ter = veh.SCMTerrain(self.sys)
        self.ter.SetSoilParameters(*SOIL.values())
        self.ter.Initialize(*SIZE, DELTA)
        self.ball = chrono.ChBodyEasySphere(R, 1000, False, True, chrono.ChContactMaterialSMC())
        self.ball.SetMass(MASS)
        self.sys.Add(self.ball)
        self.acc = self.ball.AddAccumulator()
        self.trace = []                            # 누를 때마다 (시각, 구 바닥 - 시작 흙 높이)
        self.frames = None                         # --record: [(흙 높이 격자, 구 중심, 단계 이름, 가라앉은 양, 멈춤 s)]
        self.phase = ""
        self.park()
        for _ in range(10):
            self.sys.DoStepDynamics(STEP)

    def park(self):
        self.ball.SetFixed(True)
        self.ball.SetPos(chrono.ChVector3d(0, 0, 3.0))
        self.ball.SetPosDt(chrono.ChVector3d(0, 0, 0))

    def height(self, xy):
        return self.ter.GetHeight(chrono.ChVector3d(*xy, 0))

    def press(self, xy, seconds=1.5, damping=400.0):
        """구를 xy의 흙 바로 위에 놓고 무게로 가라앉힌다 (감쇠로 천천히 정적 평형). 반환: (시작 흙 높이, 멈춘 구 바닥 높이)."""
        h0 = self.height(xy)
        self.ball.SetPos(chrono.ChVector3d(*xy, h0 + R + 0.001))
        self.ball.SetPosDt(chrono.ChVector3d(0, 0, 0))
        self.ball.SetAngVelLocal(chrono.ChVector3d(0, 0, 0))
        self.ball.SetFixed(False)
        ts, zs = [], []
        self.snap((xy[0], xy[1], h0 + R + LIFT), 0.0, hold=0.6)
        for k in range(int(seconds / STEP)):
            self.ball.EmptyAccumulator(self.acc)
            v = self.ball.GetPosDt()
            self.ball.AccumulateForce(self.acc, chrono.ChVector3d(-damping * v.x, -damping * v.y, -damping * v.z),
                                      self.ball.GetPos(), False)
            self.sys.DoStepDynamics(STEP)
            if k % 10 == 0:
                ts.append((k + 1) * STEP)
                zs.append(self.ball.GetPos().z - R - h0)
            if k % REC_EVERY == 0:
                c = self.ball.GetPos()
                self.snap((c.x, c.y, c.z), h0 - (c.z - R))
        self.trace.append((np.array(ts), np.array(zs)))
        bottom = self.ball.GetPos().z - R
        self.snap((xy[0], xy[1], bottom + R), h0 - bottom, hold=1.2)
        self.park()
        n = int(0.2 / STEP)
        for k in range(n):                         # 떼고 나서 흙이 되튀는 시간 (기록에서는 구를 천천히 들어 올림)
            self.sys.DoStepDynamics(STEP)
            if k % REC_EVERY == 0:
                self.snap((xy[0], xy[1], bottom + R + LIFT * (k + 1) / n), h0 - bottom)
        return h0, bottom

    def inject(self, levels, xy):
        """{(i, j): 높이} -> SetModifiedNodes."""
        lst = veh.ChSCMTerrainNodeLevelList()
        for (i, j), h in levels.items():
            lst.append((chrono.ChVector2i(i, j), h))
        self.snap((xy[0], xy[1], self.height(xy) + R + LIFT), 0.0, hold=0.8, phase_note="before SetModifiedNodes")
        self.ter.SetModifiedNodes(lst)
        self.snap((xy[0], xy[1], self.height(xy) + R + LIFT), 0.0, hold=1.5, phase_note="after SetModifiedNodes")

    def snap(self, center, sink, hold=0.0, phase_note=""):
        if self.frames is None:
            return
        n = (int(round(SIZE[1] / DELTA)) + 1, int(round(SIZE[0] / DELTA)) + 1)
        ys = np.linspace(-SIZE[1] / 2, SIZE[1] / 2, n[0])
        xs = np.linspace(-SIZE[0] / 2, SIZE[0] / 2, n[1])
        grid = np.array([[self.ter.GetHeight(chrono.ChVector3d(x, y, 0)) for x in xs] for y in ys], np.float32)
        self.frames.append((grid, np.array(center, np.float32), self.phase + (f" | {phase_note}" if phase_note else ""),
                            float(sink), float(hold)))

    def save(self, path, results):
        g, c, ph, sk, hd = zip(*self.frames)
        np.savez_compressed(path, heights=np.stack(g), center=np.stack(c), phase=np.array(ph), sink=np.array(sk),
                            hold=np.array(hd), dt=REC_EVERY * STEP, size=np.array(SIZE), delta=DELTA, radius=R,
                            spots=np.array([SPOTS[k] for k in (1, 2, 3, 4)]), half=HALF, results=np.array(results))

    def info(self, xy):
        i = self.ter.GetNodeInfo(chrono.ChVector3d(*xy, 0))
        return (f"h {self.height(xy) * 100:+.2f} cm, sinkage {i.sinkage * 100:.2f}, plastic {i.sinkage_plastic * 100:.2f}, "
                f"sigma_yield {i.sigma_yield:.3g}")


    def profile(self, xy, half=0.25):
        """xy를 지나는 x 방향 흙 높이 (노드 간격)."""
        xs = xy[0] + np.arange(-half, half + 1e-9, DELTA)
        return xs - xy[0], np.array([self.ter.GetHeight(chrono.ChVector3d(x, xy[1], 0)) for x in xs])


def node(xy):
    return int(round(xy[0] / DELTA)), int(round(xy[1] / DELTA))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", metavar="PNG", help="그림으로 저장")
    ap.add_argument("--record", metavar="NPZ", help="Godot 재생용 기록 (conformance/replay_scm_probe.py)")
    a = ap.parse_args()
    p = Probe()
    if a.record:
        p.frames = []
    prof = {}                                       # 자리 -> (누르기 전 단면, 누른 뒤 단면, 멈춘 구 바닥, 처음 흙 기준 높이 변화)
    line = "{:<22} 시작 {:+.2f} cm -> 바닥 {:+.2f} cm (침하 {:.2f} cm) | {}"

    before = p.profile(SPOTS[1])
    p.phase = "1 fresh soil"            # 기록(Godot 화면)은 짧은 영어: Godot 기본 글꼴에 한글이 없어 매번 대체 글꼴을 찾느라 멈춘다
    h0, b1 = p.press(SPOTS[1])
    prof[1] = (before, p.profile(SPOTS[1]), b1, [0.0, b1])
    s1 = h0 - b1
    print(line.format("1 새 흙", h0 * 100, b1 * 100, s1 * 100, p.info(SPOTS[1])))

    c = node(SPOTS[2])
    p.phase = f"2 heights only, -{HOLE * 100:.0f} cm"
    p.inject({(c[0] + di, c[1] + dj): -HOLE for di in range(-HALF, HALF + 1) for dj in range(-HALF, HALF + 1)}, SPOTS[2])
    print(f"{'2 넣은 직후':<22} {p.info(SPOTS[2])}")
    before = p.profile(SPOTS[2])
    h0, b2 = p.press(SPOTS[2])
    prof[2] = (before, p.profile(SPOTS[2]), b2, [0.0, -HOLE, b2])
    print(line.format(f"2 {HOLE * 100:.0f} cm 판 자리", h0 * 100, b2 * 100, (h0 - b2) * 100, p.info(SPOTS[2])))

    p.phase = "3 compact (first press)"
    h0, b = p.press(SPOTS[3])
    c3 = node(SPOTS[3])
    copied = {(n.x - c3[0], n.y - c3[1]): h for n, h in p.ter.GetModifiedNodes(True)
              if abs(n.x - c3[0]) <= HALF and abs(n.y - c3[1]) <= HALF}
    print(f"{'3 첫 누름 뒤':<22} {p.info(SPOTS[3])} (첫 누름 침하 {(h0 - b) * 100:.2f} cm)")
    before = p.profile(SPOTS[3])
    p.phase = "3 press again"
    h0, b3 = p.press(SPOTS[3])
    prof[3] = (before, p.profile(SPOTS[3]), b3, [0.0, b, h0, b3])     # 첫 누름 -> 되튐 -> 다시 누름
    s3 = h0 - b3
    print(line.format("3 다시 누름", h0 * 100, b3 * 100, s3 * 100, p.info(SPOTS[3])))

    c4 = node(SPOTS[4])
    p.phase = "4 heights of 3 copied"
    p.inject({(c4[0] + di, c4[1] + dj): h for (di, dj), h in copied.items()}, SPOTS[4])
    print(f"{'4 베껴 넣은 직후':<22} {p.info(SPOTS[4])} (노드 {len(copied)}개)")
    before = p.profile(SPOTS[4])
    h0, b4 = p.press(SPOTS[4])
    prof[4] = (before, p.profile(SPOTS[4]), b4, [0.0, h0, b4])        # 넣음 -> 누름
    s4 = h0 - b4
    print(line.format("4 높이만 베낀 자리", h0 * 100, b4 * 100, s4 * 100, p.info(SPOTS[4])))

    checks = [("넣은 높이를 접촉이 따라감 (2 바닥 = 1 바닥 - 판 깊이, 2 mm 안)", abs(b2 - (b1 - HOLE)) < 0.002),
              ("실제로 다진 자리는 거의 더 안 가라앉음 (3 추가 침하 < 1 mm)", s3 < 0.001),
              ("높이만 넣은 자리는 다짐 이력이 없음 (4 추가 침하 > 3의 10배)", s4 > 10 * max(s3, 1e-4))]
    for desc, ok in checks:
        print(("OK   " if ok else "FAIL ") + desc)
    if a.record:
        p.save(a.record, [s1, b2 - b1, s3, s4])
        print(f"기록: {a.record} ({len(p.frames)}장)")
    if a.plot:
        # p.trace: 1, 2, 3(첫 누름), 3(다시 누름), 4 -> 그림에는 1, 2, 3 다시 누름, 4
        plot(a.plot, [p.trace[i] for i in (0, 1, 3, 4)], prof)
        print(f"그림: {a.plot}")
    sys.exit(0 if all(ok for _, ok in checks) else 1)


LABELS = {1: "1 새 흙", 2: f"2 높이만 {HOLE * 100:.0f} cm 낮춤", 3: "3 실제로 다진 자리", 4: "4 3의 높이만 베낌"}
COLORS = {1: "#2a78d6", 2: "#eb6834", 3: "#1baf7a", 4: "#eda100"}      # 범주 순서 고정 (dataviz 기본 팔레트 1~4)
INK, INK2, MUTED, SURFACE, GRID = "#0b0b0b", "#52514e", "#8f8d86", "#fcfcfb", "#e4e3df"


def plot(path, traces, prof):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": ["Noto Sans CJK KR", "NanumGothic", "sans-serif"], "font.size": 10,
                         "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "axes.facecolor": SURFACE, "figure.facecolor": SURFACE, "axes.spines.top": False,
                         "axes.spines.right": False})
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


if __name__ == "__main__":
    main()
