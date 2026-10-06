"""지형 인지 상태 -> RViz 마커 (알고리즘 노드와 러너가 함께 쓴다. 표시 전용, 시뮬레이터를 모른다).

  /control/terrain_map  (PointCloud2, 필드 x y z rgb): 알고리즘이 LiDAR로 만든 높이 지도 (칸 중심).
                        몸통 기준 지면보다 낮으면 파랑, 높으면 빨강. RViz에서 Boxes, 칸 크기로 그린다
  /control/footholds    (MarkerArray): 다리별 고른 디딜 곳(공중인 발 주황, 디딘 발 하늘색)과 지형을 모를 때 디딜 곳(회색), 그 사이 선
좌표계는 odom (알고리즘의 주행거리 좌표계). world -> odom은 러너가 시작 위치로 한 번 발행한다 (표류는 화면에 어긋남으로 보인다).
"""
import numpy as np

FRAME = "odom"
SPAN = 0.08          # 색 범위: 몸통 기준 지면 ±8 cm


def _color(c, r, g, b, a=1.0):
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _map_cloud(snap, stamp):
    from sensor_msgs.msg import PointCloud2, PointField
    cells = snap["cells"]
    r = np.clip((cells[:, 2] - snap["h_ref"]) / SPAN, -1, 1)
    low, high = np.minimum(r, 0), np.maximum(r, 0)            # 회색 -> 파랑(낮음) / 빨강(높음)
    rgb = np.stack([0.6 + 0.5 * low + 0.4 * high, 0.6 + 0.3 * low - 0.5 * high, 0.6 - 0.4 * low - 0.5 * high], 1)
    rgb8 = np.clip(rgb * 255, 0, 255).astype(np.uint32)
    packed = (rgb8[:, 0] << 16) | (rgb8[:, 1] << 8) | rgb8[:, 2]
    rec = np.zeros(len(cells), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    rec["x"], rec["y"], rec["z"], rec["rgb"] = cells[:, 0], cells[:, 1], cells[:, 2], packed
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, FRAME
    msg.height, msg.width = 1, len(rec)
    msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1) for k, n in enumerate("xyz")]
    msg.fields.append(PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1))   # RViz 규약: rgb는 float32 자리
    msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 16, 16 * len(rec), True
    msg.data = rec.tobytes()
    return msg


def perception_markers(snap, stamp):
    """반환: (지도 PointCloud2, 디딜 곳 MarkerArray)."""
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    def marker(ns, mid, kind):
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, FRAME
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    terrain = _map_cloud(snap, stamp)

    feet = MarkerArray()
    lines = marker("foothold_shift", 100, Marker.LINE_LIST)
    lines.scale.x = 0.006
    _color(lines.color, 1.0, 1.0, 1.0, 0.9)
    for leg in snap["legs"]:
        i, (cx, cy, cz), (nx, ny, nz) = leg["leg"], leg["chosen"], leg["nominal"]
        chosen = marker("foothold", i, Marker.SPHERE)
        chosen.pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
        chosen.scale.x = chosen.scale.y = chosen.scale.z = 0.05 if leg["swing"] else 0.035
        _color(chosen.color, *((1.0, 0.55, 0.0) if leg["swing"] else (0.3, 0.85, 1.0)), 0.95)
        nominal = marker("foothold_nominal", i, Marker.SPHERE)
        nominal.pose.position = Point(x=float(nx), y=float(ny), z=float(nz))
        nominal.scale.x = nominal.scale.y = nominal.scale.z = 0.025
        _color(nominal.color, 0.7, 0.7, 0.7, 0.8)
        feet.markers += [chosen, nominal]
        lines.points += [Point(x=float(nx), y=float(ny), z=float(nz)), Point(x=float(cx), y=float(cy), z=float(cz))]
    feet.markers.append(lines)
    return terrain, feet
