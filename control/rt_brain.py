"""보행 알고리즘 (상위 제어기) 별도 프로세스: 같은 PC의 C++ 실시간 코어(하위 제어기 + 물리)와 공유 메모리로 연결한다.

    python -m control.rt_brain --shm /go2_rt_<번호> [--scenario scenarios/x.yaml] [--policy card.yaml]
    (sim.rt_link --brain shm이 실행한다)

로봇 상태가 새로 올 때마다 한 번 계산해 관절 명령을 쓴다 (control/ros_node.py와 같은 일, 연결만 다르다).
운용자 이동 명령은 공유 메모리의 명령 링으로 받아, 그 시각 이후 첫 로봇 상태부터 적용한다 (/cmd_vel_stamped와 같은 뜻).
지형 인지 보행이면 LiDAR 점군(센서 좌표)을 스캔 공유 메모리(<이름>_scan)로 받는다. 로봇 상태보다 먼저 읽는다.
시뮬레이터(sim/)를 import하지 않는다. 다른 PC에 두뇌를 두려면 ROS2 노드(control/ros_node.py, sim.rt_link --brain ros)를 쓴다.
"""
import argparse
import time
from pathlib import Path

import yaml

from rt.shm import ScanView, ShmView

from .node import ControllerNode

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shm", required=True, help="공유 메모리 이름 (sim.rt_link가 알려줌)")
    ap.add_argument("--scenario", help="트롯 보행기 설정(controller 절)을 읽을 시나리오")
    ap.add_argument("--policy")
    a = ap.parse_args()
    spec = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())
    cfg = yaml.safe_load(Path(a.scenario).read_text()).get("controller") if a.scenario else None
    lidars = yaml.safe_load((ROOT / "specs/go2_sensors.yaml").read_text())["lidars"]
    node = ControllerNode(spec, cfg, a.policy, lidars)
    view = ShmView.open(a.shm)
    scans = ScanView(a.shm) if node.sensor else None
    view.shm.ctl.brain_ready = 1
    pending, last_step = [], -1
    try:
        while not view.shm.ctl.stop:
            pending = sorted(pending + view.new_ops())
            if scans is not None and (sc := scans.read_new()) is not None:
                node.on_scan(node.sensor, *sc)
            ls = view.read_state()
            if ls is None or ls["step"] == last_step:
                if view.done:                  # 코어가 끝났고 남은 상태도 없다 (마지막 상태에 답하기 전에 끝나지 않게: lockstep이 기다린다)
                    break
                time.sleep(0.0002)
                continue
            last_step = ls["step"]
            while pending and pending[0][0] <= ls["t"] + 1e-6:
                _, vx, wz = pending.pop(0)
                node.set_command(vx, wz)
            cmd = node.step(ls)
            view.write_est(ls["t"], node.est, node.clock.cmd_f)                      # 진단용 (기록, 화면 정렬). 명령보다 먼저 써서 lockstep에서 빠지지 않게
            view.write_cmd(cmd)
    finally:
        if scans is not None:
            scans.close()
        view.close()


if __name__ == "__main__":
    main()
