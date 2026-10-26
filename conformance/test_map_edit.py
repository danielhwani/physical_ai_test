"""운용자 지형 편집 (sim/map_edit.py, docs/map_server.md 3단계 3a) 끝에서 끝까지 시험. ROS2 필요 (mj_ros), 약 40초.

    python conformance/test_map_edit.py          (ROS_DOMAIN_ID 79)

  실행 중 편집: flat_trot을 --map-server --realtime으로 돌리는 동안 다른 프로세스(sim.map_edit)가 로봇 앞에 둔덕(add)과 홈(dig)을 넣는다.
  시뮬레이터가 받아 물리에 넣고(서버와 해시 불일치 0), 받은 시각으로 scenario_replay.yaml에 적는다.
  재현: 그 시나리오를 맵 서버 없이 다시 돌리면 시계열이 비트 단위로 같다. 편집이 없었던 실행과는 다르다 (편집이 물리에 들어갔다).
  MOP: 편집이 닿은 칸은 흙 변형(deformed_cells)에서 빠지고 map_received_changes / map_received_cells로 따로 나온다 (실행 = 재현).
  화면용 ROS2 발행(--ros2, --godot과 같은 발행기)과 --map-server를 함께 써도 돈다.
  이 시험이 만든 실행 폴더만 지운다.
"""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ROS_DOMAIN_ID", "79")
WALL = ("wall", "lag", "late", "wake", "compute")


def runs():
    return set(p.name for p in (ROOT / "runs").iterdir())


def same(a, b):
    ta, tb = pq.read_table(a / "timeseries.parquet"), pq.read_table(b / "timeseries.parquet")
    cols = [c for c in ta.column_names if not any(w in c for w in WALL)]
    return ta.num_rows == tb.num_rows and all(np.array_equal(np.asarray(ta[c]), np.asarray(tb[c]), equal_nan=True) for c in cols)


def main():
    from sim.map_edit import main as edit
    before = runs()
    py = [sys.executable, "-m", "sim.runner"]
    try:
        live = subprocess.Popen(py + ["scenarios/flat_trot.yaml", "--map-server", "--realtime"], cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(6.0)                                          # 모델을 만들고 몇 초 걷는다
        assert edit(["bump", "--center", "2.5", "0", "--height", "0.04", "--radius", "0.6"]) == 0
        assert edit(["rut", "--start", "3.2", "-1", "--end", "3.2", "1", "--width", "0.2", "--depth", "0.03", "--box"]) == 0
        out, _ = live.communicate(timeout=120)
        got = [ln for ln in out.splitlines() if "map: " in ln]
        mop = {ln.split()[0]: ln.split()[1] for ln in out.splitlines() if ln.startswith("  map_")}
        assert len(got) == 2 and mop.get("map_hash_mismatch") == "0", (got, mop)
        live_dir = ROOT / "runs" / sorted(runs() - before)[-1]
        scn = live_dir / "scenario_replay.yaml"
        assert scn.exists() and scn.read_text().count("action: map_write") == 2
        times = [float(ln.split("t=")[1].split("]")[0]) for ln in got]
        print(f"test_live_edit: OK (받은 시각 {times} s, 불일치 0, 반영 지연 p95 {mop.get('map_apply_latency_p95_ms')} ms)")

        b2 = runs()
        subprocess.run(py + [str(scn)], cwd=ROOT, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, timeout=300)
        rep_dir = ROOT / "runs" / sorted(runs() - b2)[-1]
        if not same(live_dir, rep_dir):
            ta, tb = pq.read_table(live_dir / "timeseries.parquet"), pq.read_table(rep_dir / "timeseries.parquet")
            t = np.asarray(ta["t"])
            first = min((int(np.argmax(np.asarray(ta[c]) != np.asarray(tb[c]))), c) for c in ta.column_names
                        if not any(w in c for w in WALL) and not np.array_equal(np.asarray(ta[c]), np.asarray(tb[c]), equal_nan=True))
            ev = [ln for ln in out.splitlines() if "event:" in ln or "map:" in ln]
            raise AssertionError(f"재현이 실행과 다르다: 처음 다른 곳 t={t[first[0]]:.3f} ({first[1]}), 실행 이벤트 {ev}")
        b3 = runs()
        subprocess.run(py + ["scenarios/flat_trot.yaml"], cwd=ROOT, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                       timeout=300)
        plain_dir = ROOT / "runs" / sorted(runs() - b3)[-1]
        assert not same(live_dir, plain_dir), "편집이 물리에 들어가지 않았다"
        mops = [json.loads((d / "summary.json").read_text())["mop"] for d in (live_dir, rep_dir)]
        for mo in mops:                                          # 편집은 흙 변형 MOP에 섞이지 않고 따로 (실행 = 재현)
            assert mo["deformed_cells"] == 0 and mo["map_received_changes"] == 2 and mo["map_received_cells"] > 0, mo
        assert mops[0]["map_received_cells"] == mops[1]["map_received_cells"]
        print(f"test_mop_separation: OK (변형 칸 0, 받은 변경 2개 {mops[0]['map_received_cells']}칸: 실행 = 재현)")
        fx = [json.loads((d / "summary.json").read_text())["mop"]["forward_x_m"] for d in (live_dir, rep_dir, plain_dir)]
        print(f"test_replay_identical: OK (실행 = 재현 비트 동일, forward_x 실행 {fx[0]} / 재현 {fx[1]} / 편집 없음 {fx[2]})")

        # 화면용 ROS2 발행(--ros2, --godot, --rviz)과 맵 클라이언트가 한 프로세스에서 같이 ROS2를 쓴다 (전에는 두 번 초기화로 멈춤)
        r = subprocess.run(py + ["scenarios/rough_rut.yaml", "--map-server", "--ros2"], cwd=ROOT, capture_output=True, text=True, timeout=300)
        assert r.returncode == 0 and "map_hash_mismatch    0" in r.stdout, r.stdout[-2000:] + r.stderr[-2000:]
        print("test_with_ros2_stream: OK (--map-server --ros2, 불일치 0)")
    finally:
        for r in runs() - before:                                # 이 시험이 만든 실행 폴더만
            shutil.rmtree(ROOT / "runs" / r)


if __name__ == "__main__":
    main()
