"""맵 클라이언트 전송 (docs/map_server.md 3단계 3c). 같은 JSON 규약(sim/map_client.py 첫머리)을 ROS2 토픽 또는 TCP 한 줄로 나른다.

  RosTransport  ROS2 (rclpy): /map/write, /map/request -> 서버, /map/updates, /map/reply, /map/info <- 서버
  TcpTransport  TCP 127.0.0.1 (맵 서버의 TCP 입구, --tcp-port): ROS2가 없는 클라이언트용 (예: Chrono 프로세스, Python 3.12)
                한 줄에 {"t": 종류, "d": 메시지 JSON 문자열}. 종류: w(쓰기), r(요청) -> 서버, u(변경), p(응답), i(알림) <- 서버
  둘 다 send_write, send_request, spin_once(timeout), ready(), close()를 갖고, 받으면 on_update / on_reply / on_info(문자열)를 부른다.
  rclpy는 RosTransport 안에서만 가져온다 (TCP만 쓰는 쪽은 ROS2가 없어도 된다).
"""
import json
import select
import socket
import time

KINDS_IN = {"u": "on_update", "p": "on_reply", "i": "on_info"}


class RosTransport:
    def __init__(self, name, on_update, on_reply, on_info):
        import rclpy
        import rclpy.executors
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self._String = String
        self.node = rclpy.create_node(f"map_client_{name}")
        # 자기 실행기: rclpy.spin_once(node)는 전역 실행기라 한 프로세스의 다른 노드(클라이언트 둘, 브리지 등) 콜백도 돌린다
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        rel = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.node.create_publisher(String, "/map/write", rel)
        self.pub_req = self.node.create_publisher(String, "/map/request", rel)
        self.node.create_subscription(String, "/map/updates", lambda m: on_update(m.data), rel)   # 붙는 절차 1: 먼저 구독
        self.node.create_subscription(String, "/map/reply", lambda m: on_reply(m.data), rel)
        self.node.create_subscription(String, "/map/info", lambda m: on_info(m.data), latched)

    def ready(self):
        """서버가 쓰기를 구독하고 변경을 내고 있다 (안 기다리면 init이 사라진다)."""
        return self.pub.get_subscription_count() > 0 and self.node.count_publishers("/map/updates") > 0

    def send_write(self, data):
        self.pub.publish(self._String(data=data))

    def send_request(self, data):
        self.pub_req.publish(self._String(data=data))

    def spin_once(self, timeout):
        self.executor.spin_once(timeout_sec=timeout)

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()


class TcpTransport:
    def __init__(self, addr, on_update, on_reply, on_info, connect_s=30.0):
        host, port = addr.rsplit(":", 1)
        self.cb = {"on_update": on_update, "on_reply": on_reply, "on_info": on_info}
        t0 = time.monotonic()
        while True:                                       # 서버가 막 뜨는 중이면 잠깐 기다린다
            try:
                self.sock = socket.create_connection((host, int(port)), timeout=5.0)
                break
            except OSError:
                if time.monotonic() - t0 > connect_s:
                    raise
                time.sleep(0.1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.setblocking(False)
        self.buf = b""
        self.lines = []                                   # 받았지만 아직 콜백을 부르지 않은 줄 (spin_once 한 번에 하나: ROS2와 같게)
        self.got_info = False

    def ready(self):
        return self.got_info                              # 서버가 붙자마자 지금 알림(/map/info)을 보낸다

    def _send(self, kind, data):
        line = (json.dumps({"t": kind, "d": data}, separators=(",", ":")) + "\n").encode()
        self.sock.setblocking(True)
        try:
            self.sock.sendall(line)
        finally:
            self.sock.setblocking(False)

    def send_write(self, data):
        self._send("w", data)

    def send_request(self, data):
        self._send("r", data)

    def spin_once(self, timeout):
        """받은 줄 하나의 콜백을 부른다 (없으면 timeout까지 기다려 읽는다)."""
        if not self.lines:
            if select.select([self.sock], [], [], timeout)[0]:
                while True:
                    try:
                        chunk = self.sock.recv(1 << 20)
                    except BlockingIOError:
                        break
                    if not chunk:
                        raise ConnectionError("맵 서버 TCP 연결이 끊겼다")
                    self.buf += chunk
                *done, self.buf = self.buf.split(b"\n")
                self.lines += [d for d in done if d]
        if self.lines:
            m = json.loads(self.lines.pop(0))
            if m["t"] == "i":
                self.got_info = True
            self.cb[KINDS_IN[m["t"]]](m["d"])

    def close(self):
        self.sock.close()
