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
    ap.add_argument("--plot", metavar="PNG", help="세 방법 비교 그림 (기록을 저장한 뒤 plot_scm_probe.py로 그림)")
    ap.add_argument("--record", metavar="NPZ", help="시험 기록 (scm_record.py). 비교용 높이만·단단한 흙도 함께 잰다 (약 20초 더)")
    a = ap.parse_args()
    print("보정 (눌러 둘 때 내려가는 양 재기):")
    offset = calibrate(a.rounds)
    virtual, rows = {}, []
    for name, off in (("보정 없음", {}), ("보정 있음", offset)):
        print(f"\n{name}:")
        for mass in a.masses:
            s3, d3, s4, d4, e = run(mass, off)
            rows.append([bool(off), mass, s3, d3, s4, d4, e])
            if off:
                virtual[mass] = (s3, s4)
            print(f"  하중 {mass:3.0f} kg  이어 누른 자리 {s3 * 100:5.2f} cm (깊이 {d3 * 100:.2f}) | 가상 다짐 자리 {s4 * 100:5.2f} cm "
                  f"(깊이 {d4 * 100:.2f}) | 눌러 둔 뒤 모양 차이 최대 {e * 100:.2f} cm", flush=True)
    if a.record or a.plot:
        import scm_record
        import probe_scm_compaction as C
        print("비교용: 높이만, 단단한 흙", flush=True)
        stiff_mult = 25
        analysis = {"masses": a.masses, "stiff_mult": stiff_mult, "rounds": a.rounds, "rows": rows,
                    "reference": {m: v[0] for m, v in virtual.items()}, "virtual": {m: v[1] for m, v in virtual.items()},
                    "heights_only": {m: C.run(1, m)[1] for m in a.masses}, "stiff": {m: C.run(stiff_mult, m)[1] for m in a.masses}}
        config = {"soil": P.SOIL, "radius": P.R, "delta": P.DELTA, "compact_mass": COMPACT_MASS}
        path = a.record or str(Path(__file__).resolve().parent.parent / "build/scm_virtual_compaction_plot.npz")
        scm_record.save(path, "virtual_compaction", config, analysis)
        print(f"기록: {path}")
        if a.plot:
            from plot_scm_probe import plot
            print(f"그림: {plot(path, a.plot)}")


if __name__ == "__main__":
    main()
