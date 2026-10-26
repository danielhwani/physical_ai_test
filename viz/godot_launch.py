"""Godot 4 렌더러 띄우기: 중계기(viz/godot_bridge.py)와 Godot 프로젝트(godot/). sim.runner / sim.rt_link --godot이 쓰고,
돌고 있는 시뮬레이터에 나중에 렌더러를 붙일 때 혼자 쓴다 (docs/map_server.md 3단계 3b).

    python -m viz.godot_launch [--map-server] [--points] [--port 4243]

  --map-server: 지형을 맵 서버(관찰자)에서 받는다. 실행 중 언제 붙어도 지금 지형 전체를 받는다 (스냅샷 + 그 뒤 변경).
                없으면 /sim/terrain_patch (늦게 붙으면 최근 500개만 다시 받는다: 실행이 길면 앞부분을 놓친다).
  시뮬레이터와 같은 ROS_DOMAIN_ID에서. 시뮬레이터 쪽 --godot과 같은 포트를 쓰면 부딪히므로, 따로 붙일 때는 시뮬레이터를 --godot 없이 띄운다.
물리엔진은 import하지 않는다 (viz 규칙).
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GODOT_BIN = os.environ.get("GODOT_BIN", str(Path.home() / "Applications/godot/Godot_v4.7.2-stable_linux.x86_64"))
GODOT_PORT = 4243


def launch_godot(log_dir, port=GODOT_PORT, points=False, map_server=False):
    """(godot, 중계기) 프로세스를 반환. points: LiDAR 점군과 알고리즘 지형 지도도 그린다 (--godot-lidar).
    map_server: 지형을 맵 서버에서 받는다. GODOT_BIN으로 실행 파일을, GODOT_ARGS로 Godot 쪽 추가 인자(예: 시험용 --shot=경로
    --shot-frames=N)를 바꾼다."""
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    bridge = subprocess.Popen([sys.executable, "-m", "viz.godot_bridge", "--port", str(port), *(["--points"] if points else []),
                               *(["--map-server"] if map_server else [])], cwd=ROOT,
                              stdout=open(log_dir / "godot_bridge.log", "w"), stderr=subprocess.STDOUT)
    if not Path(GODOT_BIN).exists():
        bridge.terminate()
        raise SystemExit(f"Godot 실행 파일이 없다: {GODOT_BIN} (README \"Godot 4 렌더러\", 또는 GODOT_BIN)")
    extra = os.environ.get("GODOT_ARGS", "").split()
    godot = subprocess.Popen([GODOT_BIN, "--path", str(ROOT / "godot"), "--", f"--port={port}", *extra], cwd=ROOT,
                             stdout=open(log_dir / "godot.log", "w"), stderr=subprocess.STDOUT)
    return godot, bridge


def main():
    ap = argparse.ArgumentParser(description="돌고 있는 시뮬레이터에 Godot 렌더러를 붙인다")
    ap.add_argument("--map-server", action="store_true", help="지형을 맵 서버에서 (언제 붙어도 지금 지형 전체)")
    ap.add_argument("--points", action="store_true", help="LiDAR 점군과 알고리즘 지형 지도도")
    ap.add_argument("--port", type=int, default=GODOT_PORT)
    a = ap.parse_args()
    log_dir = ROOT / "build/godot_view"
    godot, bridge = launch_godot(log_dir, a.port, a.points, a.map_server)
    print(f"Godot 렌더러 (ROS_DOMAIN_ID {os.environ.get('ROS_DOMAIN_ID') or '0 (정하지 않음)'}"
          f"{', 지형은 맵 서버에서' if a.map_server else ''}). 창을 닫으면 끝. 로그: {log_dir}")
    try:
        godot.wait()
    except KeyboardInterrupt:
        godot.terminate()
    finally:
        bridge.terminate()


if __name__ == "__main__":
    main()
