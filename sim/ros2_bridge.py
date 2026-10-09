"""중립 렌더 스트림의 ROS2 전송 (계약: docs/render_interface.md).

시뮬레이터는 그리는 방법을 모른다. 아래 토픽만 내보내고, RViz/UE5 등은 각자의 어댑터가 받아 그린다.

  /tf                  모든 바디의 월드 포즈 (frame: world -> 바디 이름)
  /clock               시뮬레이션 시각 (구독자는 use_sim_time:=true)
  /sim/base_twist      몸통 선속도/각속도 (월드 좌표)
  /joint_states        관절 위치/속도
  /sim/scene_manifest  장면 매니페스트 JSON (transient local)
  /sim/terrain_patch   지형 높이 패치 JSON (처음 전체 + 변경분, transient local)
  /sim/status          시험 상태 JSON
  /sensors/<이름>/points  LiDAR 점군 (sensor_msgs/PointCloud2, 센서 좌표계 <이름>; /tf에 world -> <이름>)
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import mujoco

from .adapters import mj_quat_to_ros
from .model_builder import TERRAIN_BODY
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


def launch_sensor_node(log_dir, sensors):
    """별도 프로세스 센서 노드: ROS2 중립 스트림만 구독해 센서 출력을 만든다 (실제 배치 형태)."""
    return subprocess.Popen([sys.executable, "-m", "viz.sensor_node", *sensors], cwd=ROOT,
                            stdout=open(log_dir / "sensor_node.log", "w"), stderr=subprocess.STDOUT)


def launch_controller_node(log_dir, scenario_path=None, policy=None, viz=False):
    """보행 알고리즘 ROS2 노드 (control/ros_node.py): 로봇 상태 토픽만 보고 관절 명령 토픽을 낸다."""
    cmd = [sys.executable, "-m", "control.ros_node"] + (["--viz"] if viz else [])
    if policy:
        cmd += ["--policy", str(policy)]
    if scenario_path:
        cmd += ["--scenario", str(scenario_path)]
    return subprocess.Popen(cmd, cwd=ROOT, stdout=open(log_dir / "controller_node.log", "w"), stderr=subprocess.STDOUT)


def launch_mujoco_viewer(log_dir):
    """MuJoCo 렌더러 어댑터 (중립 스트림 -> 그리기 전용 MuJoCo 모델 -> 뷰어)."""
    return subprocess.Popen([sys.executable, "-m", "viz.mujoco_adapter"], cwd=ROOT,
                            stdout=open(log_dir / "mujoco_adapter.log", "w"), stderr=subprocess.STDOUT)


class Ros2StreamPublisher:
    def __init__(self, model, terrain, cosim=None, frame="world"):
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
        self.bodies = [(i, n) for i in range(1, model.nbody)
                       if (n := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)) != TERRAIN_BODY]
        self.joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, model.njnt)]

        self.cosim = cosim
        self.pub_scans = {}
        self.pub_manifest.publish(String(data=json.dumps(build_manifest(model, terrain, cosim))))
        self.terrain_stream = TerrainPatchStream(terrain)
        self.pub_terrain.publish(String(data=json.dumps(self.terrain_stream.full())))

    def wait_for_subscriber(self, topic="/tf", count=1, timeout=30.0, alive=lambda: True):
        """렌더러 같은 구독자가 count개 붙을 때까지 대기 (늦게 켜져 시작 장면을 놓치지 않도록)."""
        t0 = time.time()
        while time.time() - t0 < timeout and alive():
            if self.node.count_subscribers(topic) >= count:
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
        for name, p, q in (self.cosim.stream_poses() if self.cosim is not None else []):   # 다른 물리엔진 바디
            t = self._T()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.frame, name
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

    def publish_scan(self, name, scan):
        """같은 프로세스 센서 출력 발행: PointCloud2 + world -> 센서 좌표계 TF."""
        from sensor_msgs.msg import PointCloud2
        from viz.ros_msgs import scan_to_msgs
        if name not in self.pub_scans:
            self.pub_scans[name] = self.node.create_publisher(PointCloud2, f"/sensors/{name}/points", 5)
        msg, tf = scan_to_msgs(name, scan, self.frame)
        self.pub_tf.publish(self._TF(transforms=[tf]))
        self.pub_scans[name].publish(msg)

    # ---- 로봇 경계 (보행 알고리즘을 별도 노드로 돌릴 때) ----
    def enable_robot_io(self):
        """로봇 경계 토픽. 형식은 control/robot_msgs.py (go2_rt_msgs 커스텀 메시지, 없으면 JSON. GO2_ROS_MSG / --ros-msg)."""
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Imu, JointState
        from control.robot_msgs import Codec
        self.codec = Codec()
        self._Twist2, self._Imu, self._JS2 = Twist, Imu, JointState
        self.pub_low_state = self.node.create_publisher(self.codec.State, "/robot/low_state", 10)
        self.pub_imu = self.node.create_publisher(Imu, "/robot/imu", 10)
        self.pub_meas_js = self.node.create_publisher(JointState, "/robot/joint_states", 10)
        from geometry_msgs.msg import TwistStamped
        self._TwistS = TwistStamped
        # 시나리오 이벤트의 운용자 명령은 시각을 붙여 보낸다 (노드가 그 시각 상태부터 적용 -> 같은 프로세스와 같은 결과)
        self.pub_cmd_vel = self.node.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.latest_cmd = self.latest_est = None
        self.cmds = []
        self._rx = 0                                  # 받은 메시지 수 (spin_some이 큐가 비었는지 판단)
        self.node.create_subscription(self.codec.Cmd, "/robot/low_cmd", self._on_low_cmd, 10)
        # 알고리즘의 상태 추정 (진단용). 판정자(러너)가 참값과 비교해 추정 오차 MOP를 남긴다
        self.node.create_subscription(self.codec.Est, "/control/estimate", self._on_estimate, 10)

    def _on_low_cmd(self, msg):
        cmd = self.codec.cmd(msg)
        self.cmds.append(cmd)
        del self.cmds[:-10]                            # 최근 명령 몇 개만 (상태 시각으로 고른다)
        self.latest_cmd = cmd; self._rx += 1

    def cmd_until(self, t):
        """상태 시각 t 이하로 계산된 명령 중 가장 최근 것. 연결 지연을 한 주기로 고정해 실행을 결정적으로 만든다
        (노드가 매우 빨라 같은 주기 안에 답이 와도 다음 주기에 실행 = 같은 프로세스의 latency_steps와 같다)."""
        ok = [c for c in self.cmds if c.get("t", -1) <= t + 1e-9]
        return ok[-1] if ok else None

    def _on_estimate(self, msg):
        self.latest_est = self.codec.est(msg); self._rx += 1

    def publish_low_state(self, ls):
        """로봇 상태: 알고리즘용 전체 (go2_rt_msgs/LowState 또는 JSON) + 표준 도구용 Imu, JointState (측정값)."""
        self.pub_low_state.publish(self.codec.state_msg(ls))
        stamp = self._stamp(ls["t"])
        imu = self._Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "imu"
        (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = ls["imu"]["quat"]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = ls["imu"]["gyro"]
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ls["imu"]["accel"]
        self.pub_imu.publish(imu)
        js = self._JS2()
        js.header.stamp, js.name, js.position, js.velocity = stamp, self.joints, ls["q"], ls["dq"]
        self.pub_meas_js.publish(js)

    def publish_cmd_vel(self, vx, yaw_rate, t):
        msg = self._TwistS()
        msg.header.stamp = self._stamp(t)
        msg.twist.linear.x, msg.twist.angular.z = float(vx), float(yaw_rate)
        self.pub_cmd_vel.publish(msg)

    def spin_some(self, max_msgs=50):
        """쌓인 메시지를 모두 처리한다 (spin_once는 한 번에 하나만 처리하므로 명령이 밀리지 않게 비운다)."""
        import rclpy
        for _ in range(max_msgs):
            before = self._rx
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self._rx == before:
                break

    def wait_cmd(self, t_state, timeout):
        """t_state 상태에 대한 명령이 올 때까지 최대 timeout초 기다린다. 시뮬레이터가 실시간보다 늦어져 따라잡는 동안에도
        알고리즘 노드에 한 제어 주기만큼의 계산 시간을 준다 (실시간으로 도는 로봇과 같은 조건). 늦으면 그대로 진행한다."""
        import time
        end = time.perf_counter() + timeout
        while (self.latest_cmd is None or self.latest_cmd.get("t", -1) < t_state - 1e-9) and time.perf_counter() < end:
            self.spin_some()
            time.sleep(0.0005)

    def publish_odom_frame(self, offset, t_sim):
        """world -> odom (알고리즘 주행거리 좌표계, 축은 월드와 같음). 화면 정렬용 평행이동.
        러너가 시작 몸통 위치로 시작해 실제 위치 - 추정 위치를 따라가게 계속 발행한다 (/tf, 10 Hz)."""
        t = self._T()
        t.header.stamp, t.header.frame_id, t.child_frame_id = self._stamp(t_sim), self.frame, "odom"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, offset)
        t.transform.rotation.w = 1.0
        self.pub_tf.publish(self._TF(transforms=[t]))

    def publish_perception(self, snap):
        """같은 프로세스 실행일 때 알고리즘의 지형 인지 상태 (control/ros_viz.py와 같은 토픽)."""
        from control.ros_viz import perception_markers
        if not hasattr(self, "pub_map"):
            from sensor_msgs.msg import PointCloud2
            from visualization_msgs.msg import MarkerArray
            self.pub_map = self.node.create_publisher(PointCloud2, "/control/terrain_map", 2)
            self.pub_feet = self.node.create_publisher(MarkerArray, "/control/footholds", 5)
            self._viz_n = 0
        cloud, feet = perception_markers(snap, self._stamp(snap["t"]))
        self.pub_feet.publish(feet)
        if self._viz_n % 5 == 0:
            self.pub_map.publish(cloud)
        self._viz_n += 1

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
