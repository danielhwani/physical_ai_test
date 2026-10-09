"""로봇 경계 ROS2 메시지 (/robot/low_state, /robot/low_cmd, /control/estimate)의 형식 선택과 변환.

  typed: 커스텀 메시지 go2_rt_msgs/LowState, LowCmd, Estimate (ros2_ws/src/go2_rt_msgs, colcon으로 빌드)
  json:  std_msgs/String 안의 JSON (처음 방식. 메시지 패키지가 없는 PC용)
  auto:  go2_rt_msgs가 빌드돼 있으면 typed, 아니면 json (기본)

두 프로세스(시뮬레이터 쪽, 두뇌 노드)가 같은 형식이어야 한다. 환경변수 GO2_ROS_MSG(= --ros-msg)로 정하며, 시뮬레이터가 띄우는
두뇌 노드는 환경변수를 물려받는다. 다른 PC의 두뇌는 같은 --ros-msg로 띄운다.
setup.bash를 source하지 않아도 저장소의 ros2_ws/install에서 찾아 쓴다 (라이브러리를 미리 불러온다).

    python -m control.robot_msgs --build     # 메시지 패키지 빌드 (colcon, 시스템 ROS2 Humble)
    python -m control.robot_msgs             # 지금 쓰일 형식 확인
dict 형식(로봇 상태, 관절 명령, 상태 추정)은 control/node.py가 쓰는 것과 같다. 실수는 8바이트 그대로라 JSON과 같은 값이다.
"""
import ctypes
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WS = ROOT / "ros2_ws"
PREFIX = WS / "install/go2_rt_msgs"
PKG = "go2_rt_msgs"
_LIBS = ("generator_c", "typesupport_c", "typesupport_fastrtps_c", "typesupport_introspection_c", "typesupport_cpp",
         "typesupport_fastrtps_cpp", "typesupport_introspection_cpp", "generator_py")
_typed = None


def _load_typed():
    """go2_rt_msgs.msg 모듈 (없으면 None). source하지 않은 프로세스면 저장소 install에서 경로를 잡고 라이브러리를 미리 불러온다."""
    global _typed
    if _typed is not None:
        return _typed or None
    try:
        import go2_rt_msgs.msg as m                       # setup.bash를 source했거나 이미 경로가 있음
        _typed = m
        return m
    except ImportError:
        pass
    py = PREFIX / "local/lib/python3.10/dist-packages"
    if not (PREFIX / "lib").exists() or not py.exists():
        _typed = False
        return None
    pre = str(PREFIX)
    for var, val in (("AMENT_PREFIX_PATH", pre), ("LD_LIBRARY_PATH", pre + "/lib"), ("PYTHONPATH", str(py))):
        if val not in os.environ.get(var, "").split(":"):     # 자식 프로세스(두뇌 노드)도 찾게
            os.environ[var] = val + (":" + os.environ[var] if os.environ.get(var) else "")
    for lib in _LIBS:                                         # 이미 시작한 프로세스는 LD_LIBRARY_PATH를 다시 읽지 않는다
        ctypes.CDLL(str(PREFIX / f"lib/lib{PKG}__rosidl_{lib}.so"), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(py))
    import go2_rt_msgs.msg as m
    _typed = m
    return m


def resolve(mode=None):
    """쓸 형식 ("typed" 또는 "json"). mode가 없으면 환경변수 GO2_ROS_MSG, 그것도 없으면 auto."""
    mode = mode or os.environ.get("GO2_ROS_MSG", "auto")
    if mode not in ("auto", "typed", "json"):
        raise ValueError(f"GO2_ROS_MSG / --ros-msg: auto, typed, json 중 하나 ({mode})")
    if mode == "json":
        return "json"
    if _load_typed() is None:
        if mode == "typed":
            raise RuntimeError(f"go2_rt_msgs가 빌드되지 않았다: python -m control.robot_msgs --build ({PREFIX})")
        return "json"
    return "typed"


def set_mode(mode):
    """이 프로세스와 이후 띄우는 자식 프로세스의 형식을 정한다 (--ros-msg)."""
    if mode:
        os.environ["GO2_ROS_MSG"] = mode


def _stamp(t):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec, s.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1_000_000_000
    return s


class Codec:
    """형식 하나에 대한 메시지 타입과 dict <-> 메시지 변환."""

    def __init__(self, mode=None):
        self.mode = resolve(mode)
        if self.mode == "typed":
            m = _load_typed()
            self.State, self.Cmd, self.Est = m.LowState, m.LowCmd, m.Estimate
        else:
            from std_msgs.msg import String
            self.State = self.Cmd = self.Est = String

    # ---- 로봇 상태 ----
    def state_msg(self, ls):
        if self.mode == "json":
            return self.State(data=json.dumps(ls))
        m = self.State()
        m.stamp, m.t, m.step = _stamp(ls["t"]), float(ls["t"]), int(ls.get("step", -1))
        m.q[:], m.dq[:], m.foot_force[:] = ls["q"], ls["dq"], ls["foot_force"]
        imu = ls["imu"]
        m.imu_quat[:], m.imu_gyro[:], m.imu_accel[:] = imu["quat"], imu["gyro"], imu["accel"]
        return m

    def state(self, m):
        if self.mode == "json":
            return json.loads(m.data)
        ls = {"t": m.t, "q": m.q.tolist(), "dq": m.dq.tolist(),
              "imu": {"quat": m.imu_quat.tolist(), "gyro": m.imu_gyro.tolist(), "accel": m.imu_accel.tolist()},
              "foot_force": m.foot_force.tolist()}
        if m.step >= 0:
            ls["step"] = m.step
        return ls

    # ---- 관절 명령 ----
    def cmd_msg(self, cmd):
        if self.mode == "json":
            return self.Cmd(data=json.dumps(cmd))
        m = self.Cmd()
        m.stamp, m.t = _stamp(cmd["t"]), float(cmd["t"])
        m.q_des[:], m.dq_des[:], m.kp[:], m.kd[:], m.tau_ff[:] = cmd["q_des"], cmd["dq_des"], cmd["kp"], cmd["kd"], cmd["tau_ff"]
        return m

    def cmd(self, m):
        if self.mode == "json":
            return json.loads(m.data)
        return {"t": m.t, "q_des": m.q_des.tolist(), "dq_des": m.dq_des.tolist(), "kp": m.kp.tolist(),
                "kd": m.kd.tolist(), "tau_ff": m.tau_ff.tolist()}

    # ---- 상태 추정 ----
    def est_msg(self, est):
        if self.mode == "json":
            return self.Est(data=json.dumps(est))
        m = self.Est()
        m.stamp, m.t = _stamp(est["t"]), float(est["t"])
        m.v_body[:], m.pos[:], m.cmd[:] = est["v_body"], est["pos"], est["cmd"]
        m.yaw, m.roll, m.pitch = float(est["yaw"]), float(est["roll"]), float(est["pitch"])
        m.contacts = [bool(c) for c in est["contacts"]]
        return m

    def est(self, m):
        if self.mode == "json":
            return json.loads(m.data)
        return {"t": m.t, "v_body": m.v_body.tolist(), "yaw": m.yaw, "roll": m.roll, "pitch": m.pitch,
                "contacts": list(m.contacts), "pos": m.pos.tolist(), "cmd": m.cmd.tolist()}


def build():
    """ros2_ws/src/go2_rt_msgs를 colcon으로 빌드한다 (conda 환경을 빼고 시스템 ROS2 Humble로)."""
    env = {"HOME": os.environ["HOME"], "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8"}
    subprocess.run(["bash", "-c", f"source /opt/ros/humble/setup.bash && colcon build --packages-select {PKG}"],
                   cwd=WS, env=env, check=True)
    return PREFIX


if __name__ == "__main__":
    if "--build" in sys.argv:
        print(f"빌드: {build()}")
    print(f"로봇 경계 메시지 형식: {resolve()} (GO2_ROS_MSG={os.environ.get('GO2_ROS_MSG', 'auto')}, 패키지 {PREFIX})")
