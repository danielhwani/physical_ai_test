"""구르는 구로 SCM 다짐 전달 시험 (docs/map_server.md 5.5): 누르기 시험(probe_scm_*.py)을 굴리기로. 흙을 미는 전단(kshear, tau)과
구름 저항이 들어간다.

    python conformance/probe_scm_rolling.py [--masses 60 90] [--rounds 3] [--plot docs/img/scm_rolling.png] [--record build/scm_rolling.npz]
    --record: 첫 하중의 실행을 기록 (흙 높이, 구 위치·회전, 약 15 Hz). Godot 재생: python conformance/replay_scm_probe.py --probe rolling
    (어느 환경이든: pychrono가 없으면 chrono 환경으로 다시 실행)

구(반지름 10 cm)를 흙에 올려 1초 가라앉힌 뒤 수평으로 0.3 m/s로 끈다 (x -0.8 -> 0.6 m). 끄는 힘은 위치·속도 추종 (PD), 위아래와 회전은
자유라 바닥 마찰로 저절로 구른다. 세 줄 (서로 0.5 m 떨어짐, 한 SCM, 한 번 실행마다 새 SCM):
  A 이어 굴린 줄 (단일 SCM 기준)  60 kg으로 한 번 굴려 자국 -> 그 줄 높이를 베껴 두고 -> 같은 자국을 하중 M으로 다시 굴림
  B 높이만 베낀 줄                 A의 첫 자국 높이를 새 흙에 넣고 -> 하중 M으로 굴림
  C 가상 다짐 줄                   A의 높이(+ 보정)를 넣고 -> 60 kg으로 한 번 굴려 다짐 -> 하중 M으로 굴림
가운데 구간(x -0.3 ~ 0.3 m)에서 두 번째 굴림의 추가 침하(굴리기 전 표면 - 구 바닥)와 평균 구름 저항(끄는 힘 x)을 잰다.
보정: C에서 다지는 굴림 동안 더 내려가는 양을 미리 재서 그만큼 올려 넣는다 (--rounds번 되풀이, 매번 새 SCM).
이 PC (2026-10-24, 약 30초): 첫 굴림 60 kg 새 흙 침하 2.80 cm, 저항 95.2 N. 두 번째 굴림 추가 침하 / 저항
  60 kg: A 0.69 cm / 40.5 N, B 2.16 / 98.9, C 0.67 / 49.5.  90 kg: A 1.24 / 108.4, B 2.75 / 166.6, C 1.19 / 113.5.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_scm_injection as P  # noqa: E402  (pychrono가 없으면 여기서 chrono 환경으로 다시 실행된다)

chrono, veh = P.chrono, P.veh
R, DELTA, STEP = P.R, P.DELTA, P.STEP
SIZE = (2.4, 1.8)
LANES = {"A": -0.5, "B": 0.0, "C": 0.5}
X0, X1, SPEED = -0.8, 0.6, 0.3
MID = (-0.3, 0.3)
STRIP = 8                      # 줄 높이를 베낄 때 줄 가운데에서 위아래 노드 수 (16 cm)
COMPACT_MASS = 60.0
KP, KD, KV = 6000.0, 600.0, 300.0     # 끄는 힘 (x, y PD), 위아래 감쇠
REC_EVERY = 132                # 기록 간격 (스텝, 약 66 ms)


class Roller:
    def __init__(self):
        self.sys = chrono.ChSystemSMC()
        self.sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        self.sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
        self.sys.SetNumThreads(1)
        self.ter = veh.SCMTerrain(self.sys)
        self.ter.SetSoilParameters(*P.SOIL.values())
        self.ter.Initialize(*SIZE, DELTA)
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(0.8)
        self.ball = chrono.ChBodyEasySphere(R, 1000, False, True, mat)
        self.sys.Add(self.ball)
        self.acc = self.ball.AddAccumulator()
        self.frames, self.phase = None, ""      # --record: [(흙 높이 격자, 구 중심, 구 자세 xyzw, 단계, 추가 침하, 끄는 힘, 멈춤 s)]
        self.last_xy, self.last_z = (X0, LANES["A"]), R + 0.25   # 기록용: 구를 치워 둔 동안 화면에 둘 자리
        self.park()
        for _ in range(10):
            self.sys.DoStepDynamics(STEP)

    def park(self):
        self.ball.SetFixed(True)
        self.ball.SetPos(chrono.ChVector3d(0, 0, 3.0))
        self.ball.SetPosDt(chrono.ChVector3d(0, 0, 0))
        self.ball.SetAngVelLocal(chrono.ChVector3d(0, 0, 0))

    def height(self, x, y):
        return self.ter.GetHeight(chrono.ChVector3d(x, y, 0))

    def roll(self, y, mass, settle=1.0):
        """줄 y를 굴린다. 반환: (가운데 구간 평균 추가 침하, 평균 구름 저항 N, 가운데 구간 구 바닥 높이 평균).
        self.trace에 끄는 동안의 (x, 구 바닥, 굴리기 전 표면, 끄는 힘)을 20스텝마다 남긴다 (그림용)."""
        xs_ref = np.arange(X0, X1 + 1e-9, DELTA)
        before = {round(x, 4): self.height(x, y) for x in xs_ref}       # 굴리기 전 표면 (줄 가운데)
        self.ball.SetMass(mass)
        self.ball.SetPos(chrono.ChVector3d(X0, y, self.height(X0, y) + R + 0.001))
        self.ball.SetPosDt(chrono.ChVector3d(0, 0, 0))
        self.ball.SetAngVelLocal(chrono.ChVector3d(0, 0, 0))
        self.ball.SetFixed(False)
        t, T = 0.0, settle + (X1 - X0) / SPEED
        sinks, forces, bottoms = [], [], []
        self.trace, k = [], 0
        while t < T:
            xt = X0 + max(0.0, t - settle) * SPEED
            vt = SPEED if t > settle else 0.0
            p, v = self.ball.GetPos(), self.ball.GetPosDt()
            fx = KP * (xt - p.x) + KD * (vt - v.x)
            fy = KP * (y - p.y) - KD * v.y
            self.ball.EmptyAccumulator(self.acc)
            self.ball.AccumulateForce(self.acc, chrono.ChVector3d(fx, fy, -KV * v.z), p, False)
            self.sys.DoStepDynamics(STEP)
            t += STEP
            k += 1
            if k % REC_EVERY == 0:
                self.snap(before.get(round(round(p.x / DELTA) * DELTA, 4), 0.0) - (p.z - R) if t > settle else 0.0, fx)
            if t > settle and k % 20 == 0 and X0 + 0.05 <= p.x <= X1:
                self.trace.append((p.x, p.z - R, before[round(round(p.x / DELTA) * DELTA, 4)], fx))
            if t > settle and MID[0] <= p.x <= MID[1]:
                b = p.z - R
                sinks.append(before[round(round(p.x / DELTA) * DELTA, 4)] - b)
                forces.append(fx)
                bottoms.append(b)
        self.snap(float(np.mean(sinks)), float(np.mean(forces)), hold=1.0, note="mean over mid section")
        self.park()
        for _ in range(int(0.2 / STEP)):
            self.sys.DoStepDynamics(STEP)
        return float(np.mean(sinks)), float(np.mean(forces)), float(np.mean(bottoms))

    def snap(self, sink, force, hold=0.0, note=""):
        if self.frames is None:
            return
        n = (int(round(SIZE[1] / DELTA)) + 1, int(round(SIZE[0] / DELTA)) + 1)
        ys = np.linspace(-SIZE[1] / 2, SIZE[1] / 2, n[0])
        xs = np.linspace(-SIZE[0] / 2, SIZE[0] / 2, n[1])
        grid = np.array([[self.height(x, y) for x in xs] for y in ys], np.float32)
        p, q = self.ball.GetPos(), self.ball.GetRot()
        center = (p.x, p.y, p.z) if not self.ball.IsFixed() else (self.last_xy[0], self.last_xy[1], self.last_z)
        if not self.ball.IsFixed():
            self.last_xy, self.last_z = (p.x, p.y), p.z
        self.frames.append((grid, np.array(center, np.float32), np.array([q.e1, q.e2, q.e3, q.e0], np.float32),
                            self.phase + (f" | {note}" if note else ""), float(sink), float(force), float(hold)))

    def strip(self, y):
        """줄 y 둘레 노드 높이 {(i, dj): h} (dj = 줄 가운데에서 노드 차이)."""
        j0 = int(round(y / DELTA))
        i0, i1 = int(round(-SIZE[0] / 2 / DELTA)), int(round(SIZE[0] / 2 / DELTA))
        return {(i, dj): self.height(i * DELTA, (j0 + dj) * DELTA) for i in range(i0, i1 + 1) for dj in range(-STRIP, STRIP + 1)}

    def inject(self, y, levels, offset=None):
        if self.frames is not None:
            self.last_xy, self.last_z = (X0, y), self.height(X0, y) + R + 0.25
            self.snap(0.0, 0.0, hold=0.8, note="before SetModifiedNodes")
        j0 = int(round(y / DELTA))
        lst = veh.ChSCMTerrainNodeLevelList()
        for (i, dj), h in levels.items():
            if h != 0.0 or (offset and offset.get((i, dj))):
                lst.append((chrono.ChVector2i(i, j0 + dj), h + (offset or {}).get((i, dj), 0.0)))
        self.ter.SetModifiedNodes(lst)
        self.snap(0.0, 0.0, hold=1.5, note="after SetModifiedNodes")


def first_pass(r):
    """A 줄을 60 kg으로 한 번 굴려 자국을 만들고 그 높이를 돌려준다."""
    r.phase = "A: first pass, fresh soil (60 kg)"
    s, f, _ = r.roll(LANES["A"], COMPACT_MASS)
    return r.strip(LANES["A"]), s, f


def calibrate(rounds):
    offset = {}
    for k in range(rounds):
        r = Roller()
        rut, _, _ = first_pass(r)
        r.inject(LANES["C"], rut, offset)
        r.roll(LANES["C"], COMPACT_MASS)
        after = r.strip(LANES["C"])
        err = {key: rut[key] - after[key] for key in rut}
        mid = [v for (i, dj), v in err.items() if dj == 0 and MID[0] <= i * DELTA <= MID[1]]
        offset = {key: offset.get(key, 0.0) + err[key] for key in rut}
        print(f"  보정 {k + 1}회: 다지는 굴림 뒤 베낀 높이와의 차이 (줄 가운데, 가운데 구간) 평균 {np.mean(mid) * 100:+.2f} cm, "
              f"최대 {np.max(np.abs(mid)) * 100:.2f} cm", flush=True)
    return offset


def run(mass, offset, record=False):
    """반환: ((첫 굴림 침하, 저항), A, B, C, 렌더용 기록 또는 None). A/B/C = (추가 침하, 저항, 바닥)."""
    r = Roller()
    if record:
        r.frames = []
    rut, s1, f1 = first_pass(r)
    r.phase = f"A: roll again in the same SCM ({mass:.0f} kg) = reference"
    a = r.roll(LANES["A"], mass)
    ta = r.trace
    r.phase = "B: heights of A copied"
    r.inject(LANES["B"], rut)
    r.phase = f"B: roll on copied heights ({mass:.0f} kg)"
    b = r.roll(LANES["B"], mass)
    tb = r.trace
    r.phase = "C: heights of A (+offset) copied"
    r.inject(LANES["C"], rut, offset)
    r.phase = "C: virtual compaction roll (60 kg)"
    r.roll(LANES["C"], COMPACT_MASS)
    r.phase = f"C: roll after virtual compaction ({mass:.0f} kg)"
    c = r.roll(LANES["C"], mass)
    tc = r.trace
    run.traces[mass] = (ta, tb, tc)
    return (s1, f1), a, b, c, (render(r, mass, a, b, c) if record else None)


run.traces = {}


def render(r, mass, a, b, c):
    """렌더용 기록 (scm_record.py 규약). 화면 글자는 영어와 숫자만 (Godot 기본 글꼴에 한글 없음)."""
    g, cen, q, ph, sk, fo, hd = zip(*r.frames)
    tints = {"A": (0.62, 0.60, 0.56), "B": (0.52, 0.62, 0.78), "C": (0.48, 0.72, 0.60)}      # 그래프와 같은 뜻의 색 (A 기준 회색)
    w = STRIP * DELTA
    regions = [{"name": n, "region": [X0 - 0.1, y - w, X1 + 0.1, y + w], "rgba": [*tints[n], 1.0]} for n, y in LANES.items()]
    lines = [f"second pass {mass:.0f} kg, mid section:  sinkage / rolling resistance",
             f"A {a[0] * 100:.2f} cm / {a[1]:.1f} N   B {b[0] * 100:.2f} cm / {b[1]:.1f} N   C {c[0] * 100:.2f} cm / {c[1]:.1f} N"]
    return dict(heights=np.stack(g), center=np.stack(cen), quat=np.stack(q), phase=np.array(ph), sink=np.array(sk),
                force=np.array(fo), hold=np.array(hd), dt=REC_EVERY * STEP, size=np.array(SIZE), radius=R,
                regions=json.dumps(regions), title="SCM probe: rolling sphere (Chrono SCM -> render stream replay)",
                result_lines=json.dumps(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--masses", type=float, nargs="+", default=[60.0, 90.0], help="두 번째 굴림 하중 (kg)")
    ap.add_argument("--rounds", type=int, default=3, help="C 높이 보정 되풀이 횟수")
    ap.add_argument("--plot", metavar="PNG", help="그림으로 저장 (기록을 저장한 뒤 plot_scm_probe.py로 그림)")
    ap.add_argument("--record", metavar="NPZ", help="시험 기록: 분석 데이터 + 첫 하중 실행의 Godot 재생용 프레임 (scm_record.py)")
    a = ap.parse_args()
    print("보정 (C: 다지는 굴림 동안 내려가는 양 재기):")
    offset = calibrate(a.rounds)
    print()
    results, frames = {}, None
    for mass in a.masses:
        (s1, f1), A, B, C, rend = run(mass, offset, record=bool(a.record) and mass == a.masses[0])
        results[mass] = (s1, f1, A, B, C)
        frames = frames or rend
        print(f"하중 {mass:3.0f} kg (첫 굴림 60 kg: 새 흙 침하 {s1 * 100:.2f} cm, 저항 {f1:.1f} N)")
        for name, (s, f, bot) in (("A 이어 굴린 줄 (단일 SCM 기준)", A), ("B 높이만 베낀 줄", B), ("C 가상 다짐 줄", C)):
            print(f"  {name:<26} 추가 침하 {s * 100:5.2f} cm  구름 저항 {f:6.1f} N  바닥 {bot * 100:+.2f} cm", flush=True)
    if a.record or a.plot:
        import scm_record
        config = {"soil": P.SOIL, "radius": R, "delta": DELTA, "compact_mass": COMPACT_MASS, "mid": MID, "lanes": LANES,
                  "x": [X0, X1], "speed": SPEED, "rounds": a.rounds}
        analysis = {"masses": a.masses, "results": results, "traces": run.traces}
        path = a.record or str(Path(__file__).resolve().parent.parent / "build/scm_rolling_plot.npz")
        scm_record.save(path, "rolling", config, analysis, frames)
        print(f"기록: {path}" + (f" ({len(frames['heights'])}장)" if frames else ""))
        if a.plot:
            from plot_scm_probe import plot
            print(f"그림: {plot(path, a.plot)}")


if __name__ == "__main__":
    main()
