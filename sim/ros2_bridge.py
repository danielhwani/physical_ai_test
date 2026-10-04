"""ROS2 포즈 스트림 (문서 §6.3: 관절각이 아니라 바디 월드 포즈 + 속도를 보낸다).

나중에 UE5 등 렌더러가 이 토픽을 받아 그리면 된다. 지금은 RViz로 확인 가능.
사용 전: source /opt/ros/humble/setup.bash
"""
import mujoco

from .adapters import mj_quat_to_ros


class Ros2PoseBridge:
    def __init__(self, model, frame="world"):
        import rclpy
        from geometry_msgs.msg import TransformStamped, TwistStamped
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage

        self._T, self._Twist, self._Clock, self._JS, self._TF = (
            TransformStamped, TwistStamped, Clock, JointState, TFMessage)
        rclpy.init()
        self.node = rclpy.create_node("mujoco_sim")
        self.pub_tf = self.node.create_publisher(TFMessage, "/tf", 10)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_twist = self.node.create_publisher(TwistStamped, "/sim/base_twist", 10)
        self.pub_js = self.node.create_publisher(JointState, "/joint_states", 10)
        self.frame = frame
        self.model = model
        self.bodies = [(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i))
                       for i in range(1, model.nbody)]
        self.joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                       for j in range(1, model.njnt)]

    def publish(self, data):
        stamp = self._stamp(data.time)
        clock = self._Clock(); clock.clock = stamp
        self.pub_clock.publish(clock)

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

    @staticmethod
    def _stamp(t):
        from builtin_interfaces.msg import Time
        s = Time(); s.sec = int(t); s.nanosec = int((t - int(t)) * 1e9)
        return s

    def close(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()
