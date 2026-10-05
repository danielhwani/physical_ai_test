"""중립 렌더 스트림의 ROS2 전송 (계약: docs/render_interface.md).

시뮬레이터는 그리는 방법을 모른다. 아래 토픽만 내보내고, RViz/UE5 등은 각자의 어댑터가 받아 그린다.

  /tf                  모든 바디의 월드 포즈 (frame: world -> 바디 이름)
  /clock               시뮬레이션 시각 (구독자는 use_sim_time:=true)
  /sim/base_twist      몸통 선속도/각속도 (월드 좌표)
  /joint_states        관절 위치/속도
  /sim/scene_manifest  장면 매니페스트 JSON (transient local)
  /sim/terrain_patch   지형 높이 패치 JSON (처음 전체 + 변경분, transient local)
  /sim/status          시험 상태 JSON
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import mujoco

from .adapters import mj_quat_to_ros
from .stream import TerrainPatchStream, build_manifest

ROOT = Path(__file__).resolve().parent.parent
RVIZ_CONFIG = ROOT / "config/go2.rviz"
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")


def launch_rviz_stack(log_dir):
    """RViz 어댑터(중립 스트림 -> RViz 표현)와 RViz를 띄운다. (rviz, adapter) 프로세스를 반환.
    RViz는 시스템 ROS2를 쓰므로 conda 경로를 뺀 환경에서 실행한다 (Qt 등 라이브러리 충돌 방지)."""
    adapter = subprocess.Popen([sys.executable, "-m", "viz.rviz_adapter"], cwd=ROOT,
                               stdout=open(log_dir / "rviz_adapter.log", "w"), stderr=subprocess.STDOUT)
    env = {k: v for k, v in os.environ.items() if not k.startswith("CONDA") and k != "PYTHONHOME"}
    for key in ("PATH", "LD_LIBRARY_PATH"):
        if key in env:
            env[key] = ":".join(p for p in env[key].split(":") if "conda" not in p)
    cmd = f'source "{ROS_SETUP}" && exec rviz2 -d "{RVIZ_CONFIG}" --ros-args -p use_sim_time:=true'
    rviz = subprocess.Popen(["bash", "-c", cmd], env=env,
                            stdout=open(log_dir / "rviz.log", "w"), stderr=subprocess.STDOUT)
    return rviz, adapter


class Ros2StreamPublisher:
    def __init__(self, model, terrain, frame="world"):
        import rclpy
        from geometry_msgs.msg import TransformStamped, TwistStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self._T, self._Twist, self._Clock, self._JS, self._TF, self._String = (
            TransformStamped, TwistStamped, Clock, JointState, TFMessage, String)
        rclpy.init()
        self.node = rclpy.create_node("mujoco_sim")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_tf = self.node.create_publisher(TFMessage, "/tf", 10)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_twist = self.node.create_publisher(TwistStamped, "/sim/base_twist", 10)
        self.pub_js = self.node.create_publisher(JointState, "/joint_states", 10)
        self.pub_manifest = self.node.create_publisher(String, "/sim/scene_manifest", latched)
        # 처음 전체 + 이후 패치를 모두 보관해야 늦게 붙은 구독자도 전체 지형을 재구성한다
        self.pub_terrain = self.node.create_publisher(
            String, "/sim/terrain_patch", QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_status = self.node.create_publisher(String, "/sim/status", latched)
        self.frame = frame
        self.bodies = [(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)) for i in range(1, model.nbody)]
        self.joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, model.njnt)]

        self.pub_manifest.publish(String(data=json.dumps(build_manifest(model, terrain))))
        self.terrain_stream = TerrainPatchStream(terrain)
        self.pub_terrain.publish(String(data=json.dumps(self.terrain_stream.full())))

    def wait_for_subscriber(self, topic="/tf", timeout=30.0, alive=lambda: True):
        """RViz 같은 구독자가 붙을 때까지 대기 (늦게 켜져 시작 장면을 놓치지 않도록)."""
        t0 = time.time()
        while time.time() - t0 < timeout and alive():
            if self.node.count_subscribers(topic) > 0:
                time.sleep(1.0)          # 구독 후 표시 준비 여유
                return True
            time.sleep(0.2)
        return False

    def publish(self, data):
        stamp = self._stamp(data.time)
        clock = self._Clock(); clock.clock = stamp
        self.pub_clock.publish(clock)
        patch = self.terrain_stream.update()
        if patch:
            self.pub_terrain.publish(self._String(data=json.dumps(patch)))

        msg = self._TF()
        for bid, name in self.bodies:
            t = self._T()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.frame, name
            p, q = data.xpos[bid], mj_quat_to_ros(data.xquat[bid])
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = map(float, q)
            msg.transforms.append(t)
        self.pub_tf.publish(msg)

        tw = self._Twist()
        tw.header.stamp, tw.header.frame_id = stamp, self.frame
        # free joint: qvel[0:3]는 월드 선속도, qvel[3:6]은 몸통 좌표 각속도 -> 둘 다 월드로 통일
        w_world = data.xmat[1].reshape(3, 3) @ data.qvel[3:6]
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = map(float, data.qvel[0:3])
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = map(float, w_world)
        self.pub_twist.publish(tw)

        js = self._JS()
        js.header.stamp = stamp
        js.name = self.joints
        js.position = [float(x) for x in data.qpos[7:]]
        js.velocity = [float(x) for x in data.qvel[6:]]
        self.pub_js.publish(js)

    def publish_status(self, status):
        self.pub_status.publish(self._String(data=json.dumps(status)))

    @staticmethod
    def _stamp(t):
        from builtin_interfaces.msg import Time
        s = Time(); s.sec = int(t); s.nanosec = int((t - int(t)) * 1e9)
        return s

    def close(self):
        import rclpy
        self.node.destroy_node()
        if rclpy.ok():                  # Ctrl+C 때는 rclpy 신호 처리기가 이미 종료함
            rclpy.shutdown()
