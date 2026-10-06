"""ROS 노드 배치 = 같은 프로세스 실행 (연결 지연 흉내) 시험. ROS2가 필요하다 (실시간 실행이라 수십 초).

    python conformance/test_ros_equivalence.py

보행 알고리즘을 별도 ROS2 노드로 돌린 결과(실제 배치 형태, 지형 인지 트롯은 LiDAR 점군도 토픽으로 받음)와, 같은 프로세스에서 같은 연결 지연(link.latency_steps)을
흉내 낸 결과가 같아야 한다. 같으면 빠르고 결정적인 같은 프로세스 실행으로 학습·시험해도 배치 결과를 대표한다.
ROS 전달이 한 주기라도 늦으면(지연이 20 ms가 아닌 순간) 결과가 갈라지므로, 그때는 지연 통계를 함께 보고한다.
"""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PY = sys.executable


def _run(extra, scenario="scenarios/flat_trot.yaml", seconds=6.0, policy=None, sets=()):
    cmd = [PY, "-m", "sim.runner", scenario, "--set", f"duration={seconds}", *extra]
    for kv in sets:
        cmd += ["--set", kv]
    if policy:
        cmd += ["--policy", policy]
    env = {**os.environ, "ROS_DOMAIN_ID": "79"}
    out = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, timeout=180).stdout
    run_dir = ROOT / re.search(r"기록: (\S+)", out).group(1)
    return json.loads((run_dir / "summary.json").read_text())["mop"], run_dir


def _compare(policy=None, **kw):
    a, da = _run([], policy=policy, **kw)                                        # 같은 프로세스 (지연 흉내)
    b, db = _run(["--ros2", "--realtime", "--controller-node"], policy=policy, **kw)   # 별도 ROS2 노드
    for d in (da, db):                                                     # 시험이 만든 기록은 지운다
        subprocess.run(["rm", "-rf", str(d)])
    assert b["cmd_latency_ms_max"] == b["cmd_latency_ms_mean"] == a["cmd_latency_ms_mean"] == 20.0, \
        f"ROS 지연이 일정하지 않음: 평균 {b['cmd_latency_ms_mean']} ms, 최대 {b['cmd_latency_ms_max']} ms"
    for k in ("forward_x_m", "lateral_drift_m"):
        assert abs(a[k] - b[k]) < 1e-3, f"{k}: 같은 프로세스 {a[k]} vs ROS 노드 {b[k]}"
    # 추정 오차는 별도 노드에서도 /control/estimate로 받아 같은 시각의 참값과 비교한다 (첫 주기 한 개만 차이)
    assert abs(a["est_speed_rmse_mps"] - b["est_speed_rmse_mps"]) < 2e-3, (a["est_speed_rmse_mps"], b["est_speed_rmse_mps"])


def test_trot_ros_node_matches_in_process():
    _compare()


def test_policy_ros_node_matches_in_process():
    _compare(policy="policies/go2_trot_bc/card.yaml")


def test_est_policy_ros_node_matches_in_process():
    _compare(policy="policies/go2_trot_bc_est/card.yaml")


def test_perceptive_trot_ros_node_matches_in_process():
    # LiDAR 점군도 ROS 토픽(/sensors/front_lidar/points)으로 받는다. 스캔은 찍힌 다음 주기부터 쓰므로 도착 순서와 무관
    _compare(scenario="scenarios/rough_rut.yaml", sets=["controller.perception.sensor=front_lidar"])


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
