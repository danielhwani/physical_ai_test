"""실제 바퀴(단단한 원통)로 SCM 다짐 전달 시험 (docs/map_server.md 5.5): 구르는 구 시험(probe_scm_rolling.py)을 단일 바퀴 시험대처럼.
바퀴를 목표 각속도로 돌려 미끄럼률을 정하고(구동), 수평으로 일정 속도로 끈다. 견인력(drawbar pull)과 구동 토크, 침하를 잰다.

    python conformance/probe_scm_wheel.py [--masses 60 90] [--slip 0.2] [--passes 1] [--replay-slip S] [--replay-passes N]
    python conformance/probe_scm_wheel.py --record build/scm_wheel.npz [--plot PNG]     # 네 경우를 모두 돌려 기록 (약 2분 20초)
    --record: 네 경우(CASES)의 분석 데이터 + 기본 경우 첫 하중 실행의 Godot 재생용 프레임 (scm_record.py).
    그래프: python conformance/plot_scm_probe.py build/scm_wheel.npz, 재생: python conformance/replay_scm_probe.py build/scm_wheel.npz
    (어느 환경이든: pychrono가 없으면 chrono 환경으로 다시 실행)

바퀴: 반지름 WR, 폭 WW, 축은 옆(y). 수평 속도 SPEED (위치·속도 추종 PD, 그 힘 = 끄는 힘. 음수면 바퀴가 밀고 나간다 = 견인력),
바퀴 회전 = 목표 각속도 ω = v / (WR (1 - 미끄럼률)) 추종 토크 (구동 토크), 위아래는 자유, 축 방향은 토크로 옆을 유지.
세 줄 (한 SCM, 한 번 실행마다 새 SCM):
  A 이어 지나간 줄 (단일 SCM 기준)  하중 COMPACT_MASS, 미끄럼률 s로 한 번 지나가 자국 -> 같은 자국을 하중 M으로 다시 지나감
  B 높이만 베낀 줄                   A의 첫 자국 높이를 새 흙에 넣고 -> 하중 M으로 지나감
  C 가상 다짐 줄                     A의 높이 + A의 노드 sinkage (= 자국 전 높이)를 넣고 -> 같은 바퀴·하중·미끄럼률로 한 번 지나감
                                     -> 하중 M으로 지나감
가운데 구간에서 두 번째 지나감의 추가 침하(지나가기 전 표면 - 바퀴 바닥), 끄는 힘, 구동 토크, 실제 미끄럼률을 잰다.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_scm_rolling as RL  # noqa: E402  (pychrono가 없으면 여기서 chrono 환경으로 다시 실행된다)

chrono, veh, P = RL.chrono, RL.veh, RL.P
DELTA, STEP = RL.DELTA, RL.STEP
LANES, X0, X1, SPEED, MID = RL.LANES, RL.X0, RL.X1, RL.SPEED, RL.MID
WR, WW = 0.20, 0.15                   # 바퀴 반지름, 폭 (m)
COMPACT_MASS = 60.0
KP, KD, KV = RL.KP, RL.KD, RL.KV      # 끄는 힘 (x, y PD), 위아래 감쇠
KW = 2000.0                           # 바퀴 회전 추종 (N m s). 약하면 토크가 걸릴 때 목표보다 느리게 돌아 미끄럼률이 작아진다 (300이면 0.2 -> 0.13)
KR, KRD = 3000.0, 60.0                # 바퀴 축을 옆으로 유지 (N m / rad, N m s)
CASES = {                             # 경우 이름 -> run() 인자. 화면 글자용 짧은 영어 설명
    "base": ({}, "history known: same wheel, load, slip"),
    "slip_unknown": ({"replay_slip": 0.0}, "slip unknown: replay without slip"),
    "two_passes": ({"passes": 2}, "two passes, replay both"),
    "two_passes_one_replay": ({"passes": 2, "replay_passes": 1}, "two passes, replay one (approx.)"),
}


class Wheel(RL.Roller):
    """RL.Roller의 흙판·줄 도구를 그대로 쓰고 구 대신 원통 바퀴."""

    def __init__(self):
        # RL.Roller.__init__과 같지만 처음부터 바퀴로 (구를 만들어 스텝을 돈 뒤 바꿔 넣으면 바퀴가 SCM 접촉에서 빠져 흙을 뚫고 떨어졌다)
        self.sys = chrono.ChSystemSMC()
        self.sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        self.sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
        self.sys.SetNumThreads(1)
        self.ter = veh.SCMTerrain(self.sys)
        self.ter.SetSoilParameters(*P.SOIL.values())
        self.ter.Initialize(*RL.SIZE, DELTA)
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(0.8)
        self.ball = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, WR, WW, 1000, False, True, mat)
        self.sys.Add(self.ball)
        self.acc = self.ball.AddAccumulator()
        self.frames, self.phase = None, ""
        self.last_xy, self.last_z = (X0, LANES["A"]), WR + 0.25
        self.park()
        for _ in range(10):
            self.sys.DoStepDynamics(STEP)

    def set_mass(self, m):
        self.ball.SetMass(m)
        self.ball.SetInertiaXX(chrono.ChVector3d(m * (3 * WR ** 2 + WW ** 2) / 12, 0.5 * m * WR ** 2, m * (3 * WR ** 2 + WW ** 2) / 12))

    def drive(self, y, mass, slip, settle=1.0):
        """줄 y를 바퀴로 지나간다. 반환: dict(sink, force, torque, slip, bottom) 가운데 구간 평균."""
        xs_ref = np.arange(X0, X1 + 1e-9, DELTA)
        before = {round(x, 4): self.height(x, y) for x in xs_ref}
        self.set_mass(mass)
        self.ball.SetPos(chrono.ChVector3d(X0, y, self.height(X0, y) + WR + 0.001))
        self.ball.SetRot(chrono.QUNIT)
        self.ball.SetPosDt(chrono.ChVector3d(0, 0, 0))
        self.ball.SetAngVelLocal(chrono.ChVector3d(0, 0, 0))
        self.ball.SetFixed(False)
        t, T = 0.0, settle + (X1 - X0) / SPEED
        out = {k: [] for k in ("sink", "force", "torque", "slip", "bottom")}
        self.trace = []
        k = 0
        while t < T:
            moving = t > settle
            xt = X0 + max(0.0, t - settle) * SPEED
            vt = SPEED if moving else 0.0
            wt = vt / (WR * (1.0 - slip))
            p, v = self.ball.GetPos(), self.ball.GetPosDt()
            w = self.ball.GetAngVelParent()
            axis = self.ball.GetRot().GetAxisY()
            fx = KP * (xt - p.x) + KD * (vt - v.x)
            fy = KP * (y - p.y) - KD * v.y
            tq = KW * (wt - w.y)                                     # 구동 토크 (옆축)
            # 축을 옆(+y)으로: 회전 오차 = axis × ŷ, 옆축 밖의 각속도는 감쇠
            ex, ez = -axis.z, axis.x                                 # (axis × ŷ)의 x, z 성분
            hold = chrono.ChVector3d(KR * ex - KRD * w.x, tq, KR * ez - KRD * w.z)
            self.ball.EmptyAccumulator(self.acc)
            self.ball.AccumulateForce(self.acc, chrono.ChVector3d(fx, fy, -KV * v.z), p, False)
            self.ball.AccumulateTorque(self.acc, hold, False)
            self.sys.DoStepDynamics(STEP)
            t += STEP
            k += 1
            if k % RL.REC_EVERY == 0:
                self.snap(before.get(round(round(p.x / DELTA) * DELTA, 4), 0.0) - (p.z - WR) if moving else 0.0, fx)
            if moving and k % 20 == 0 and X0 + 0.05 <= p.x <= X1:
                self.trace.append((p.x, p.z - WR, before[round(round(p.x / DELTA) * DELTA, 4)], fx, tq))
            if moving and MID[0] <= p.x <= MID[1]:
                b = p.z - WR
                out["sink"].append(before[round(round(p.x / DELTA) * DELTA, 4)] - b)
                out["force"].append(fx)
                out["torque"].append(tq)
                out["slip"].append(1.0 - v.x / (w.y * WR) if abs(w.y) > 1e-6 else 0.0)
                out["bottom"].append(b)
        res = {k: float(np.mean(v)) for k, v in out.items()}
        self.snap(res["sink"], res["force"], hold=1.0, note="mean over mid section")
        self.park()
        for _ in range(int(0.2 / STEP)):
            self.sys.DoStepDynamics(STEP)
        return res


def run(mass, slip, passes=1, replay_slip=None, replay_passes=None, record=False):
    """반환: (첫 지나감, A, B, C, 렌더용 기록 또는 None). 첫 지나감·A·B·C는 각각 drive() 결과 (dict).
    passes: A를 측정 전에 지나간 횟수 (자국 이력). replay_slip / replay_passes: C의 가상 다짐에 쓴 미끄럼률·횟수
    (None이면 이력과 같게: 이력을 정확히 아는 경우. 다르게 주면 이력을 모르거나 줄여 쓴 경우)."""
    replay_slip = slip if replay_slip is None else replay_slip
    replay_passes = passes if replay_passes is None else replay_passes
    r = Wheel()
    if record:
        r.frames = []
    r.phase = f"A: first pass, fresh soil ({COMPACT_MASS:.0f} kg, slip {slip:.2f})"
    first = r.drive(LANES["A"], COMPACT_MASS, slip)
    for k in range(passes - 1):
        r.phase = f"A: pass {k + 2} ({COMPACT_MASS:.0f} kg, slip {slip:.2f})"
        r.drive(LANES["A"], COMPACT_MASS, slip)
    rut, sink = r.strip(LANES["A"]), r.strip_sinkage(LANES["A"])
    r.phase = f"A: drive again in the same SCM ({mass:.0f} kg) = reference"
    A = r.drive(LANES["A"], mass, slip)
    ta = r.trace
    r.phase = "B: heights of A copied"
    r.inject(LANES["B"], rut)
    r.phase = f"B: drive on copied heights ({mass:.0f} kg)"
    B = r.drive(LANES["B"], mass, slip)
    tb = r.trace
    r.phase = "C: heights of A + sinkage (= pre-rut surface) copied"
    r.inject(LANES["C"], rut, sink)
    for k in range(replay_passes):
        r.phase = f"C: virtual compaction pass {k + 1} ({COMPACT_MASS:.0f} kg, slip {replay_slip:.2f})"
        r.drive(LANES["C"], COMPACT_MASS, replay_slip)             # 가상 다짐: 바퀴·하중은 같게, 미끄럼률·횟수는 주어진 대로
    r.phase = f"C: drive after virtual compaction ({mass:.0f} kg)"
    C = r.drive(LANES["C"], mass, slip)
    run.traces = (ta, tb, r.trace)
    return first, A, B, C, (render(r, mass, A, B, C) if record else None)


run.traces = None


def render(r, mass, a, b, c):
    """렌더용 기록 (scm_record.py 규약). 화면 글자는 영어와 숫자만 (Godot 기본 글꼴에 한글 없음)."""
    g, cen, q, ph, sk, fo, hd = zip(*r.frames)
    tints = {"A": (0.62, 0.60, 0.56), "B": (0.52, 0.62, 0.78), "C": (0.48, 0.72, 0.60)}
    w = RL.STRIP * DELTA
    regions = [{"name": n, "region": [X0 - 0.1, y - w, X1 + 0.1, y + w], "rgba": [*tints[n], 1.0]} for n, y in LANES.items()]
    lines = [f"second pass {mass:.0f} kg, mid section:  sinkage / drawbar pull / torque",
             *(f"{n} {d['sink'] * 100:+.2f} cm / {-d['force']:.0f} N / {d['torque']:.0f} N m" for n, d in (("A", a), ("B", b), ("C", c)))]
    return dict(heights=np.stack(g), center=np.stack(cen), quat=np.stack(q), phase=np.array(ph), sink=np.array(sk),
                force=-np.array(fo), force_label="drawbar pull", hold=np.array(hd), dt=RL.REC_EVERY * STEP, size=np.array(RL.SIZE),
                radius=WR, shape=json.dumps({"kind": "cylinder", "radius": WR, "width": WW, "axis": "y"}),
                regions=json.dumps(regions), title="SCM probe: driven wheel (Chrono SCM -> render stream replay)",
                result_lines=json.dumps(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--masses", type=float, nargs="+", default=[60.0, 90.0], help="두 번째 지나감 하중 (kg)")
    ap.add_argument("--slip", type=float, default=0.2, help="미끄럼률 (0 = 미끄럼 없음, 0.2 = 20 % 헛돎)")
    ap.add_argument("--passes", type=int, default=1, help="측정 전에 A를 지나간 횟수 (자국 이력)")
    ap.add_argument("--replay-slip", type=float, help="C 가상 다짐의 미끄럼률 (기본: --slip과 같게)")
    ap.add_argument("--replay-passes", type=int, help="C 가상 다짐 횟수 (기본: --passes와 같게)")
    ap.add_argument("--record", metavar="NPZ", help="네 경우(CASES)를 모두 돌려 기록 (scm_record.py)")
    ap.add_argument("--plot", metavar="PNG", help="그림 (기록을 저장한 뒤 plot_scm_probe.py로 그림)")
    a = ap.parse_args()
    if a.record or a.plot:
        return record_all(a)
    for mass in a.masses:
        first, A, B, C, _ = run(mass, a.slip, a.passes, a.replay_slip, a.replay_passes)
        print(f"하중 {mass:3.0f} kg, 미끄럼률 {a.slip:.2f} (첫 지나감 60 kg: 침하 {first['sink'] * 100:.2f} cm, 끄는 힘 {first['force']:.1f} N, "
              f"토크 {first['torque']:.1f} N m, 실제 미끄럼률 {first['slip']:.3f})")
        for name, d in (("A 이어 지나간 줄 (단일 SCM 기준)", A), ("B 높이만 베낀 줄", B), ("C 가상 다짐 줄", C)):
            print(f"  {name:<24} 추가 침하 {d['sink'] * 100:5.2f} cm  끄는 힘 {d['force']:7.1f} N  토크 {d['torque']:6.1f} N m  "
                  f"미끄럼률 {d['slip']:.3f}", flush=True)


def record_all(a):
    """네 경우 × 하중을 돌려 기록 하나에. 기본 경우 첫 하중은 렌더용 프레임도."""
    import scm_record
    cases, frames, traces = {}, None, None
    for name, (kw, desc) in CASES.items():
        cases[name] = {"desc": desc, "args": kw, "results": {}}
        for mass in a.masses:
            rec_now = name == "base" and mass == a.masses[0]
            first, A, B, C, rend = run(mass, a.slip, record=rec_now, **kw)
            cases[name]["results"][mass] = {"first": first, "A": A, "B": B, "C": C}
            frames = frames or rend
            if rec_now:
                traces = run.traces
            print(f"{name:<22} {mass:3.0f} kg: A {A['sink'] * 100:+.2f} cm / {-A['force']:.1f} N | B {B['sink'] * 100:+.2f} / "
                  f"{-B['force']:.1f} | C {C['sink'] * 100:+.2f} / {-C['force']:.1f}  (침하 / 견인력)", flush=True)
    config = {"soil": P.SOIL, "wheel": {"radius": WR, "width": WW}, "delta": DELTA, "compact_mass": COMPACT_MASS, "slip": a.slip,
              "mid": MID, "lanes": LANES, "x": [X0, X1], "speed": SPEED}
    analysis = {"masses": a.masses, "cases": cases, "traces": traces, "trace_mass": a.masses[0]}
    path = a.record or str(Path(__file__).resolve().parent.parent / "build/scm_wheel_plot.npz")
    scm_record.save(path, "wheel", config, analysis, frames)
    print(f"기록: {path}" + (f" ({len(frames['heights'])}장)" if frames else ""))
    if a.plot:
        from plot_scm_probe import plot
        print(f"그림: {plot(path, a.plot)}")


if __name__ == "__main__":
    main()
