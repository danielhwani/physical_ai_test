"""Chrono SCM 위에 로봇 발 대리 구(반지름 2.2 cm)를 일정한 하중으로 눌러 정적 침하를 잰다 (sim/foot_soil.py 비교 기준).

    ~/miniconda3/envs/chrono/bin/python scripts/scm_foot_static.py [격자 m] [시간 s]   # 기본 0.0025 m, 2 s (약 1분)

결과: conformance/reference/scm_foot_static.json. 발 구는 cosim/chrono_server.py의 발자국 대리 구 그대로 (하중으로 누름, 감쇠).
격자가 발 크기와 비슷하면 값이 크게 달라진다 (2 cm 격자는 단단한 흙 30 N에서 0.25 cm 격자의 1.6배). 0.5 cm와 0.25 cm는 2 % 안.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cosim.chrono_server import ChronoServer  # noqa: E402

SOIL = {"bekker_kc": 0.0, "bekker_n": 1.1, "cohesion": 0.0, "friction_deg": 30.0, "janosi_shear": 0.01, "elastic_k": 4.0e7,
        "damping_r": 3.0e4}
RADIUS = 0.022


def main():
    res = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0025
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rows = []
    for kphi in (2.0e7, 1.0e7, 5.0e6, 1.0e6):
        for fn in (30.0, 50.0, 80.0, 120.0):
            s = ChronoServer(ROOT)
            s.init({"scm": {"center": [0.0, 0.0], "size": [0.2, 0.2], "resolution": res, "soil": {**SOIL, "bekker_kphi": kphi}},
                    "robot_feet": {"radius": RADIUS, "names": ["F"]}})
            for k in range(1, int(round(dur / 0.02)) + 1):
                r = s.advance(k * 0.02, False, feet=[{"pos": [0.0, 0.0, 0.0], "fn": fn}])
            rows.append({"bekker_kphi": kphi, "fn": fn, "sink_m": round(RADIUS - r["feet"][0]["z"], 5)})
            print(rows[-1], flush=True)
    out = ROOT / "conformance/reference/scm_foot_static.json"
    out.write_text(json.dumps({"grid_m": res, "duration_s": dur, "radius_m": RADIUS, "soil": SOIL, "rows": rows}, indent=1))
    print(f"저장: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
