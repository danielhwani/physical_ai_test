"""관측 벡터 기준 구현 (문서 §12.2-12.3).

명세(specs/go2_control.yaml 또는 정책 카드)의 observation 절을 따르는 numpy 구현이며,
JAX(MJX 학습)와 C++(실시간 시험) 구현이 맞춰야 하는 기준이다.
MuJoCo 함수를 쓰지 않고 입력 배열만으로 계산한다 (다른 언어로 이식이 쉽도록).

새 관측 항목(예: 관측 이력)은 TERMS에 함수 하나를 등록하고, 필요한 입력을 control/node.py에서 넘기면 된다.
height_scan(지형 높이 격자)의 입력은 "격자점 지면 높이 - 몸통 높이"이며, 어디서 뽑았는지(LiDAR 지도, 시뮬레이터 지형)는 모른다.
"""
import numpy as np


def quat_rotate_inverse(q_wxyz, v):
    """R(q)^T @ v. 쿼터니언 순서 (w, x, y, z) = MuJoCo 규약."""
    w, x, y, z = q_wxyz
    # R(q)를 명시적으로 전개 (JAX/C++로 그대로 옮길 수 있게)
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])
    return R.T @ v


def height_scan_grid(term):
    """height_scan 격자점 (N, 2), 몸통 방향(yaw) 기준 x, y. 순서: x 바깥 루프, y 안쪽 루프 (둘 다 오름차순)."""
    g = term["grid"]
    xs = np.round(np.arange(g["x"][0], g["x"][1] + 1e-9, g["step"]), 6)
    ys = np.round(np.arange(g["y"][0], g["y"][1] + 1e-9, g["step"]), 6)
    return np.array([(x, y) for x in xs for y in ys])


def _height_scan(inp, s):
    """입력 height_scan = 격자점 지면 높이 - 몸통 높이 (m, 모르면 NaN)."""
    t = s.term("height_scan")
    h = np.asarray(inp["height_scan"], dtype=float) + t["base_height_ref"]
    h = np.clip(h, -t["clip_m"], t["clip_m"])
    return np.where(np.isfinite(h), h, t["unknown"])


# 항목 이름 -> 계산 함수(inp, obs_spec). 관절 항목은 정책의 관절 순서로 재배열한다.
TERMS = {
    "base_ang_vel":      lambda inp, s: inp["qvel"][3:6],
    "projected_gravity": lambda inp, s: quat_rotate_inverse(inp["qpos"][3:7], np.array([0.0, 0.0, -1.0])),
    "command":           lambda inp, s: inp["command"],
    "joint_pos_rel":     lambda inp, s: inp["qpos"][7:19][s.joint_idx] - s.default_pose,
    "joint_vel":         lambda inp, s: inp["qvel"][6:18][s.joint_idx],
    "last_action":       lambda inp, s: inp["last_action"],
    "gait_phase":        lambda inp, s: np.array([np.sin(2 * np.pi * inp["gait_phase"]),
                                                  np.cos(2 * np.pi * inp["gait_phase"])]),
    "height_scan":       _height_scan,
}
INPUT_KEYS = ("qpos", "qvel", "command", "last_action", "gait_phase")


class ObservationSpec:
    def __init__(self, obs_cfg, joint_idx, default_pose):
        self.terms = obs_cfg["terms"]
        self.clip = obs_cfg["clip"]
        self.dim = obs_cfg["dim"]
        self.tolerance = obs_cfg.get("tolerance", {"rtol": 1e-5, "atol": 1e-5})
        self.joint_idx = np.asarray(joint_idx)
        self.default_pose = np.asarray(default_pose, dtype=float)
        unknown = [t["name"] for t in self.terms if t["name"] not in TERMS]
        assert not unknown, f"구현되지 않은 관측 항목: {unknown} (control/observation.py TERMS에 추가 필요)"
        assert sum(t["dim"] for t in self.terms) == self.dim, "observation dim 불일치"

    def term(self, name):
        return next((t for t in self.terms if t["name"] == name), None)

    @property
    def needs_height_scan(self):
        return self.term("height_scan") is not None

    def compute(self, qpos, qvel, command, last_action, gait_phase=0.0, height_scan=None):
        inp = {"qpos": np.asarray(qpos, dtype=float), "qvel": np.asarray(qvel, dtype=float),
               "command": np.asarray(command, dtype=float), "last_action": np.asarray(last_action, dtype=float),
               "gait_phase": float(gait_phase), "height_scan": height_scan}
        parts = []
        for t in self.terms:
            v = np.asarray(TERMS[t["name"]](inp, self), dtype=float)
            assert v.shape == (t["dim"],), (t["name"], v.shape)
            parts.append(v * np.asarray(t["scale"], dtype=float))
        return np.clip(np.concatenate(parts), -self.clip, self.clip)

    def slices(self):
        """항목 이름 -> 관측 벡터 내 구간 (디버깅, 항목별 오차 보고용)."""
        out, i = {}, 0
        for t in self.terms:
            out[t["name"]] = slice(i, i + t["dim"])
            i += t["dim"]
        return out
