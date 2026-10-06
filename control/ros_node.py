"""보행 알고리즘 ROS2 노드 (별도 프로세스, 실제 탑재 소프트웨어 형태).

    python -m control.ros_node [--policy card.yaml] [--scenario scenarios/x.yaml]   (sim.runner --controller-node가 실행)

  입력  /robot/low_state (std_msgs/String JSON: 로봇 상태), /cmd_vel (geometry_msgs/Twist: 운용자 이동 명령)
  출력  /robot/low_cmd   (std_msgs/String JSON: 관절 명령), /control/estimate (JSON: 상태 추정, 진단용)

로봇 상태가 올 때마다 한 번 계산해 명령을 낸다. 시뮬레이터(sim/)를 모른다.
"""
import argparse
import json
from pathlib import Path

import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .node import ControllerNode

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy")
    ap.add_argument("--scenario", help="트롯 보행기 설정(controller 절)을 읽을 시나리오")
    args = ap.parse_args()
    spec = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())
    cfg = yaml.safe_load(Path(args.scenario).read_text()).get("controller") if args.scenario else None
    ctrl = ControllerNode(spec, cfg, args.policy)

    rclpy.init()
    node = rclpy.create_node("go2_controller")
    pub_cmd = node.create_publisher(String, "/robot/low_cmd", 10)
    pub_est = node.create_publisher(String, "/control/estimate", 10)

    def on_state(msg):
        cmd = ctrl.step(json.loads(msg.data))
        pub_cmd.publish(String(data=json.dumps(cmd)))
        e = ctrl.est
        pub_est.publish(String(data=json.dumps({"t": cmd["t"], "v_body": e.v_body.tolist(), "yaw": e.yaw,
                                                "roll": e.roll, "pitch": e.pitch, "contacts": e.contacts.tolist()})))

    def on_cmd_vel(msg):
        ctrl.set_command(msg.linear.x, msg.angular.z)

    node.create_subscription(String, "/robot/low_state", on_state, 10)
    node.create_subscription(Twist, "/cmd_vel", on_cmd_vel, 10)
    node.get_logger().info(f"controller {ctrl.desc} ready")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):   # Ctrl+C / 종료 신호
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
