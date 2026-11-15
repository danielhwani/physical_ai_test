"""실행 중인 시뮬레이터의 차 위치를 1초마다 보여 준다 (렌더 스트림 /tf에서 이름이 Chassis_body로 끝나는 바디, ROS2).

    python -m viz.watch_vehicles          # Ctrl+C로 끝
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = rclpy.create_node("watch_vehicles")
    last = {}

    def on_tf(m):
        for t in m.transforms:
            if t.child_frame_id.endswith("Chassis_body"):
                p = t.transform.translation
                last[t.child_frame_id] = (round(p.x, 2), round(p.y, 2), round(p.z, 2))

    node.create_subscription(TFMessage, "/tf", on_tf, 10)
    nxt = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() >= nxt:
                nxt += 1.0
                print(time.strftime("%H:%M:%S"), last or "차 위치 없음 (시뮬레이터가 --ros2로 달리는 중인지, ROS_DOMAIN_ID가 같은지)", flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
