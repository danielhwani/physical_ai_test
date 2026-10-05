"""물리엔진 간 연결 규약 (표준 라이브러리만 사용: mj_ros Python 3.10과 chrono Python 3.12 양쪽에서 import).

메시지 = 4바이트 길이(big-endian) + UTF-8 JSON. 배열은 base64(little-endian)로 실어 보낸다.
지금은 같은 PC에서 socketpair로 연결하지만, 같은 규약을 TCP에 그대로 쓸 수 있다.
"""
import base64
import json
import struct

_LEN = struct.Struct(">I")


def send(sock, obj):
    data = json.dumps(obj).encode("utf-8")
    sock.sendall(_LEN.pack(len(data)) + data)


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("상대 프로세스가 연결을 닫았다")
        buf += chunk
    return bytes(buf)


def recv(sock):
    (n,) = _LEN.unpack(_recv_exact(sock, _LEN.size))
    return json.loads(_recv_exact(sock, n).decode("utf-8"))


def b64(raw_bytes):
    return base64.b64encode(raw_bytes).decode("ascii")


def unb64(text):
    return base64.b64decode(text)
