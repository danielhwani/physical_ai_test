"""보행 알고리즘 ROS2 노드 (별도 프로세스, 실제 탑재 소프트웨어 형태).

    python -m control.ros_node [--policy card.yaml] [--scenario scenarios/x.yaml]   (sim.runner --controller-node가 실행)

  입력  /robot/low_state (std_msgs/String JSON: 로봇 상태)
        /cmd_vel (geometry_msgs/Twist: 운용자 이동 명령, 받는 즉시 적용 - 원격 조종 도구용)
        /cmd_vel_stamped (geometry_msgs/TwistStamped: 시각을 붙인 이동 명령, 그 시각 이후 첫 로봇 상태부터 적용 - 시나리오 이벤트용.
                          토픽 간 도착 순서와 무관하게 같은 프로세스 실행과 같은 결과가 나온다)
        /sensors/<이름>/points (sensor_msgs/PointCloud2, 센서 좌표: 지형 인지 트롯일 때만. TF의 센서 자세는 쓰지 않는다)
  출력  /robot/low_cmd   (std_msgs/String JSON: 관절 명령), /control/estimate (JSON: 상태 추정, 진단용)

로봇 상태가 올 때마다 한 번 계산해 명령을 낸다. 시뮬레이터(sim/)를 모른다.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .node import ControllerNode

ROOT = Path(__file__).resolve().parent.parent


def msg_stamp(t):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec, s.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1_000_000_000
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy")
    ap.add_argument("--scenario", help="트롯 보행기 설정(controller 절)을 읽을 시나리오")
    ap.add_argument("--viz", action="store_true", help="지형 인지 상태를 RViz용으로 발행 (/control/terrain_map, /control/footholds)")
    args = ap.parse_args()
    spec = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())
    cfg = yaml.safe_load(Path(args.scenario).read_text()).get("controller") if args.scenario else None
    lidars = yaml.safe_load((ROOT / "specs/go2_sensors.yaml").read_text())["lidars"]
    ctrl = ControllerNode(spec, cfg, args.policy, lidars)

    rclpy.init()
    node = rclpy.create_node("go2_controller")
    pub_cmd = node.create_publisher(String, "/robot/low_cmd", 10)
    pub_est = node.create_publisher(String, "/control/estimate", 10)

    viz = args.viz and ctrl.terrain is not None
    if viz:
        from sensor_msgs.msg import PointCloud2 as Cloud
        from visualization_msgs.msg import MarkerArray
        from .ros_viz import perception_markers
        pub_map = node.create_publisher(Cloud, "/control/terrain_map", 2)
        pub_feet = node.create_publisher(MarkerArray, "/control/footholds", 5)
    steps = [0]

    pending_cmds = []          # (적용 시각, vx, yaw_rate)

    def on_state(msg):
        ls = json.loads(msg.data)
        while pending_cmds and pending_cmds[0][0] <= ls["t"] + 1e-6:
            _, vx, wz = pending_cmds.pop(0)
            ctrl.set_command(vx, wz)
        cmd = ctrl.step(ls)
        pub_cmd.publish(String(data=json.dumps(cmd)))
        steps[0] += 1
        if viz and steps[0] % 5 == 0:            # 명령을 보낸 뒤에 (명령 지연에 영향 없게). 디딜 곳 10 Hz, 지도 2 Hz
            snap = ctrl.perception_snapshot()
            if snap:
                cloud, feet = perception_markers(snap, msg_stamp(cmd["t"]))
                pub_feet.publish(feet)
                if steps[0] % 25 == 0:
                    pub_map.publish(cloud)
        e = ctrl.est
        pub_est.publish(String(data=json.dumps({"t": cmd["t"], "v_body": e.v_body.tolist(), "yaw": e.yaw,
                                                "roll": e.roll, "pitch": e.pitch, "contacts": e.contacts.tolist()})))

    def on_cmd_vel(msg):
        ctrl.set_command(msg.linear.x, msg.angular.z)

    node.create_subscription(String, "/robot/low_state", on_state, 10)
    node.create_subscription(Twist, "/cmd_vel", on_cmd_vel, 10)

    def on_cmd_vel_stamped(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pending_cmds.append((t, msg.twist.linear.x, msg.twist.angular.z))
        pending_cmds.sort(key=lambda c: c[0])
    node.create_subscription(TwistStamped, "/cmd_vel_stamped", on_cmd_vel_stamped, 10)
    if ctrl.sensor:
        def on_points(msg):
            pts = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
            xyz = np.ascontiguousarray(pts[:, :12]).view("<f4").reshape(-1, 3)    # x, y, z 가 앞 12바이트 (ros_msgs 규약)
            ctrl.on_scan(ctrl.sensor, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, xyz)
        node.create_subscription(PointCloud2, f"/sensors/{ctrl.sensor}/points", on_points, 5)
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
