"""중립 렌더 스트림 디코더 (렌더러 쪽 공용). 계약: docs/render_interface.md

이 패키지(viz/)는 물리엔진(MuJoCo)을 import하지 않는다. 렌더러 어댑터는 스트림만 보고 장면을 재구성한다.
(conformance/test_render_stream.py가 이를 검사한다.)
"""
import base64

import numpy as np

SUPPORTED_MAJOR = "1"


def check_manifest(manifest):
    major = str(manifest.get("stream_version", "")).split(".")[0]
    if major != SUPPORTED_MAJOR:
        raise ValueError(f"지원하지 않는 스트림 버전: {manifest.get('stream_version')}")
    return manifest


def decode_heights(patch):
    assert patch["encoding"] == "base64-float32-le", patch["encoding"]
    a = np.frombuffer(base64.b64decode(patch["heights"]), dtype="<f4")
    return a.reshape(patch["nrow"], patch["ncol"])


class TerrainGrid:
    """매니페스트의 지형 규격 + 높이 패치로 지형을 재구성."""

    def __init__(self, terrain_meta):
        self.half_x, self.half_y = terrain_meta["half_size"]
        self.nrow, self.ncol = terrain_meta["nrow"], terrain_meta["ncol"]
        self.heights = np.zeros((self.nrow, self.ncol), dtype=np.float32)
        self.version = None

    def apply(self, patch):
        r0, c0 = patch["r0"], patch["c0"]
        self.heights[r0:r0 + patch["nrow"], c0:c0 + patch["ncol"]] = decode_heights(patch)
        self.version = patch["version"]
        return r0, c0, patch["nrow"], patch["ncol"]

    def xs(self):
        return np.linspace(-self.half_x, self.half_x, self.ncol)

    def ys(self):
        return np.linspace(-self.half_y, self.half_y, self.nrow)


def quat_to_matrix(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def quat_to_rpy(q_xyzw):
    x, y, z, w = q_xyzw
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def load_obj_vertices(path):
    return np.array([[float(v) for v in line.split()[1:4]] for line in open(path) if line.startswith("v ")])
