"""센서 출력 -> ROS2 메시지 변환 (러너와 별도 센서 노드가 함께 쓴다)."""
import mujoco
import numpy as np


def stamp_of(t):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec, s.nanosec = int(t), int((t - int(t)) * 1e9)
    return s


def scan_to_msgs(name, scan, world_frame="world"):
    """LiDAR 스캔 -> (PointCloud2(센서 좌표계 name), TransformStamped(world -> name))."""
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2, PointField
    stamp = stamp_of(scan["t"])
    pts = scan["points"]
    rec = np.zeros(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("ring", "<u2"), ("label", "i1")])
    rec["x"], rec["y"], rec["z"], rec["ring"], rec["label"] = pts[:, 0], pts[:, 1], pts[:, 2], scan["ring"], scan["label"]
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, name
    msg.height, msg.width = 1, len(rec)
    msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name="ring", offset=12, datatype=PointField.UINT16, count=1),
                  PointField(name="label", offset=14, datatype=PointField.INT8, count=1)]
    msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, rec.itemsize, rec.itemsize * len(rec), True
    msg.data = rec.tobytes()
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, world_frame, name
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, scan["p"])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(scan["R"], dtype=float).ravel())
    tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = map(float, q)
    return msg, tf
