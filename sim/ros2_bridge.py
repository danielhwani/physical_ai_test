"""ROS2 포즈 스트림 (문서 §6.3: 관절각이 아니라 바디 월드 포즈 + 속도를 보낸다).

나중에 UE5 등 렌더러가 이 토픽을 받아 그리면 된다. 지금은 RViz로 확인 가능:
    rviz2 -d config/go2.rviz --ros-args -p use_sim_time:=true
(타임스탬프가 시뮬레이션 시각이므로 RViz도 /clock을 따라야 한다.)

발행 토픽
  /tf                  모든 바디의 월드 포즈 (frame: world -> 바디 이름)
  /sim/base_twist      몸통 선속도/각속도 (월드 좌표)
  /joint_states        관절 위치/속도
  /clock               시뮬레이션 시각
  /robot_description   URDF (RViz RobotModel 표시용, transient local)
  /sim/terrain         지형 메쉬 (MarkerArray, 0.5 m 타일, 바뀐 타일만 다시 발행, transient local)
"""
import os
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

from .adapters import mj_quat_to_ros

ROOT = Path(__file__).resolve().parent.parent
MESH_DIR = ROOT / "build/rviz_meshes"
RVIZ_CONFIG = ROOT / "config/go2.rviz"
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
TERRAIN_HALF = 6.0      # 지형 마커 범위 (m, 원점 기준 반폭). 저사양 RViz를 위해 제한
TERRAIN_RES = 0.1       # 지형 마커 해상도 (m)
TILE = 5                # 타일 한 변의 셀 수 (0.5 m). 바뀐 타일만 다시 보낸다


def _quat_to_rpy(q_wxyz):
    w, x, y, z = q_wxyz
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def build_urdf(model, mesh_dir=MESH_DIR):
    """MjModel -> URDF 문자열.

    외형은 MuJoCo가 컴파일한 메쉬(geom 좌표계 기준 정점)를 OBJ로 내보내 쓴다. 원본 OBJ를 쓰면
    MuJoCo의 메쉬 재중심화 때문에 위치가 어긋나므로, 시뮬레이터가 실제로 그리는 형상을 그대로 쓴다.
    링크 포즈는 RViz가 /tf에서 직접 읽으므로 관절은 고정 관절로 트리만 구성한다.
    """
    mesh_dir.mkdir(parents=True, exist_ok=True)
    name = lambda t, i: mujoco.mj_id2name(model, t, i)  # noqa: E731
    links, joints = [], []
    for b in range(1, model.nbody):
        visuals = []
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != b or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = model.geom_dataid[g]
            path = mesh_dir / f"{name(mujoco.mjtObj.mjOBJ_MESH, mid)}.obj"
            if not path.exists():
                va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
                fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
                lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in model.mesh_vert[va:va + vn]]
                lines += [f"f {i + 1} {j + 1} {k + 1}" for i, j, k in model.mesh_face[fa:fa + fn]]
                path.write_text("\n".join(lines) + "\n")
            rgba = model.mat_rgba[model.geom_matid[g]] if model.geom_matid[g] >= 0 else model.geom_rgba[g]
            xyz = " ".join(f"{v:.6f}" for v in model.geom_pos[g])
            rpy = " ".join(f"{v:.6f}" for v in _quat_to_rpy(model.geom_quat[g]))
            visuals.append(
                f'<visual><origin xyz="{xyz}" rpy="{rpy}"/>'
                f'<geometry><mesh filename="file://{path}"/></geometry>'
                f'<material name="m{g}"><color rgba="{" ".join(f"{c:.3f}" for c in rgba)}"/></material></visual>')
        bname = name(mujoco.mjtObj.mjOBJ_BODY, b)
        links.append(f'<link name="{bname}">{"".join(visuals)}</link>')
        parent = model.body_parentid[b]
        pname = "world" if parent == 0 else name(mujoco.mjtObj.mjOBJ_BODY, parent)
        xyz = " ".join(f"{v:.6f}" for v in model.body_pos[b])
        rpy = " ".join(f"{v:.6f}" for v in _quat_to_rpy(model.body_quat[b]))
        joints.append(f'<joint name="{bname}_fixed" type="fixed"><parent link="{pname}"/><child link="{bname}"/>'
                      f'<origin xyz="{xyz}" rpy="{rpy}"/></joint>')
    return ('<?xml version="1.0"?><robot name="go2_mujoco"><link name="world"/>'
            + "".join(links) + "".join(joints) + "</robot>")


def launch_rviz(log_path):
    """시스템 ROS2의 RViz를 띄운다. conda 환경의 라이브러리(Qt 등)가 섞이지 않게 conda 경로를 빼고 실행한다."""
    env = {k: v for k, v in os.environ.items() if not k.startswith("CONDA") and k != "PYTHONHOME"}
    for key in ("PATH", "LD_LIBRARY_PATH"):
        if key in env:
            env[key] = ":".join(p for p in env[key].split(":") if "conda" not in p)
    cmd = f'source "{ROS_SETUP}" && exec rviz2 -d "{RVIZ_CONFIG}" --ros-args -p use_sim_time:=true'
    log = open(log_path, "w")
    return subprocess.Popen(["bash", "-c", cmd], env=env, stdout=log, stderr=subprocess.STDOUT)


class Ros2PoseBridge:
    def __init__(self, model, terrain=None, frame="world"):
        import rclpy
        from geometry_msgs.msg import TransformStamped, TwistStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import Marker, MarkerArray

        self._T, self._Twist, self._Clock, self._JS, self._TF, self._Marker = (
            TransformStamped, TwistStamped, Clock, JointState, TFMessage, Marker)
        rclpy.init()
        self.node = rclpy.create_node("mujoco_sim")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_tf = self.node.create_publisher(TFMessage, "/tf", 10)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_twist = self.node.create_publisher(TwistStamped, "/sim/base_twist", 10)
        self.pub_js = self.node.create_publisher(JointState, "/joint_states", 10)
        self.pub_desc = self.node.create_publisher(String, "/robot_description", latched)
        # 처음 전체 + 이후 변경 타일을 모두 보관해야 늦게 켠 RViz도 전체 지형을 받는다
        self.pub_terrain = self.node.create_publisher(
            MarkerArray, "/sim/terrain", QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.frame = frame
        self.model = model
        self.bodies = [(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i))
                       for i in range(1, model.nbody)]
        self.joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                       for j in range(1, model.njnt)]
        self.pub_desc.publish(String(data=build_urdf(model)))
        self.terrain = terrain
        self.terrain_version = None
        self._terrain_sent = None
        if terrain is not None:
            self.publish_terrain(self._stamp(0.0))     # 전체 지형은 실시간 루프 시작 전에 한 번

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
        if self.terrain is not None and self.terrain.applied_version != self.terrain_version:
            self.publish_terrain(stamp)

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
        v_lin = data.qvel[0:3]
        w_world = data.xmat[1].reshape(3, 3) @ data.qvel[3:6]
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = map(float, v_lin)
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = map(float, w_world)
        self.pub_twist.publish(tw)

        js = self._JS()
        js.header.stamp = stamp
        js.name = self.joints
        js.position = [float(x) for x in data.qpos[7:]]
        js.velocity = [float(x) for x in data.qvel[6:]]
        self.pub_js.publish(js)

    def _terrain_grid(self):
        """물리엔진에 반영된 지형(applied)을 마커 해상도로 샘플링 (벡터화)."""
        ter = self.terrain
        xs = np.arange(-TERRAIN_HALF, TERRAIN_HALF + 1e-9, TERRAIN_RES)
        ci = np.clip(np.round((xs + ter.half_x) / (2 * ter.half_x) * (ter.ncol - 1)).astype(int), 0, ter.ncol - 1)
        ri = np.clip(np.round((xs + ter.half_y) / (2 * ter.half_y) * (ter.nrow - 1)).astype(int), 0, ter.nrow - 1)
        return xs, ter.applied[np.ix_(ri, ci)]

    def publish_terrain(self, stamp):
        """지형을 TILE x TILE 셀 타일 마커로 발행. 처음엔 전부, 이후엔 바뀐 타일만 (실시간 루프 지연 방지)."""
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import MarkerArray
        xs, H = self._terrain_grid()
        n = len(xs) - 1
        first = self._terrain_sent is None
        arr = MarkerArray()
        for r0 in range(0, n, TILE):
            for c0 in range(0, n, TILE):
                r1, c1 = min(r0 + TILE, n), min(c0 + TILE, n)
                block = H[r0:r1 + 1, c0:c1 + 1]
                if not first and np.array_equal(block, self._terrain_sent[r0:r1 + 1, c0:c1 + 1]):
                    continue
                X, Y = np.meshgrid(xs[c0:c1 + 1], xs[r0:r1 + 1])
                V = np.stack([X, Y, block], -1)
                a, b, d, e = V[:-1, :-1], V[:-1, 1:], V[1:, :-1], V[1:, 1:]
                tri = np.stack([a, b, e, a, e, d], 2).reshape(-1, 3)
                m = self._Marker()
                m.header.frame_id, m.header.stamp = self.frame, stamp
                m.ns, m.id, m.type, m.action = "terrain", r0 * (n + 1) + c0, self._Marker.TRIANGLE_LIST, self._Marker.ADD
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 1.0
                m.color.r, m.color.g, m.color.b, m.color.a = 0.55, 0.50, 0.42, 1.0
                m.points = [Point(x=x, y=y, z=z) for x, y, z in tri.tolist()]
                arr.markers.append(m)
        if arr.markers:
            self.pub_terrain.publish(arr)
        self._terrain_sent = H
        self.terrain_version = self.terrain.applied_version

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
