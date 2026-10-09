"""실시간 코어 공유 메모리 규약의 Python 정의 (rt/shm_layout.h와 같아야 한다, conformance/test_rt_core.py가 대조).

시뮬레이터(sim/)와 보행 알고리즘(control/)이 함께 쓰므로 어느 쪽도 import하지 않는다 (표준 라이브러리만).
  ShmView.create(...)  관리 프로세스가 만든다 (sim/rt_link.py)
  ShmView.open(이름)   같은 PC의 다른 프로세스가 연다 (두뇌 control/rt_brain.py)
칸 주고받기는 seqlock (쓰는 쪽 하나): 쓰는 쪽이 seq를 홀수로 올리고 쓴 뒤 짝수로 올린다.
"""
import ctypes as C
import mmap
import os
import time
from pathlib import Path

MAGIC, VERSION = 0x47324F52, 9
NJ, NQ, NV, NOISE_DIM, NOISE_RING, TRUTH_RING, OP_RING = 12, 19, 18, 36, 4096, 1024, 32
NBODY_MAX, SCAN_MAX_PTS = 16, 32768
D = C.c_double


def arr(t, n):
    return t * n


class Config(C.Structure):
    _fields_ = [("mode", C.c_int32), ("decim", C.c_int32), ("foot_geom", arr(C.c_int32, 4)), ("terrain_geom", C.c_int32),
                ("accel_adr", C.c_int32), ("base_body", C.c_int32), ("hfield_id", C.c_int32), ("terrain_mocap", C.c_int32),
                ("pad1", C.c_int32), ("cpu", C.c_int32), ("priority", C.c_int32), ("max_steps", C.c_int64),
                ("control_dt", D), ("torque_limit", arr(D, NJ)), ("gyro_bias", arr(D, 3)), ("accel_bias", arr(D, 3)),
                ("yaw_drift", D), ("hold_q", arr(D, NJ)), ("hold_kp", arr(D, NJ)), ("hold_kd", arr(D, NJ)),
                ("qpos0", arr(D, NQ)), ("qvel0", arr(D, NV))]


class Control(C.Structure):
    _fields_ = [("start", C.c_uint64), ("stop", C.c_uint64), ("lockstep_target", C.c_uint64), ("link_mode", C.c_uint64),
                ("torque_scale", D), ("fault_gyro", arr(D, 3)), ("fault_att", arr(D, 3)), ("brain_ready", C.c_uint64), ("freeze", C.c_uint64)]


class LowState(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("step", C.c_uint64), ("t", D), ("q", arr(D, NJ)), ("dq", arr(D, NJ)),
                ("quat_xyzw", arr(D, 4)), ("gyro", arr(D, 3)), ("accel", arr(D, 3)), ("foot_force", arr(D, 4))]


class LowCmd(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("t", D), ("q_des", arr(D, NJ)), ("dq_des", arr(D, NJ)), ("kp", arr(D, NJ)),
                ("kd", arr(D, NJ)), ("tau_ff", arr(D, NJ))]


class Estimate(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("t", D), ("v_body", arr(D, 3)), ("yaw", D), ("contacts", arr(D, 4)), ("pos", arr(D, 3)), ("cmd", arr(D, 2))]


class OpEntry(C.Structure):
    _fields_ = [("t", D), ("vx", D), ("yaw_rate", D)]


class NoiseEntry(C.Structure):
    _fields_ = [("step", C.c_uint64), ("v", arr(D, NOISE_DIM))]


class TruthEntry(C.Structure):
    _fields_ = [("step", C.c_uint64), ("t", D), ("qpos", arr(D, NQ)), ("qvel", arr(D, NV)), ("ctrl", arr(D, NJ)),
                ("q_des", arr(D, NJ)), ("energy", D), ("foot_force", arr(D, 4)), ("cmd_t", D), ("wake_us", D), ("compute_us", D),
                ("base_xmat", arr(D, 9)), ("foot_pos", arr(D, 12)), ("contact_bits", C.c_uint32), ("pad", C.c_uint32),
                ("body_xpos", arr(D, NBODY_MAX * 3)), ("body_xquat", arr(D, NBODY_MAX * 4))]


class Stats(C.Structure):
    _fields_ = [("steps", C.c_uint64), ("overruns", C.c_uint64), ("noise_underruns", C.c_uint64), ("done", C.c_uint64),
                ("locked", C.c_uint64), ("rt_ok", C.c_uint64), ("max_wake_us", D), ("max_compute_us", D)]


class Shm(C.Structure):
    _fields_ = [("magic", C.c_uint32), ("version", C.c_uint32), ("size", C.c_uint64), ("cfg", Config), ("ctl", Control),
                ("stats", Stats), ("state", LowState), ("cmd", LowCmd), ("est", Estimate), ("op_head", C.c_uint64), ("op", arr(OpEntry, OP_RING)),
                ("noise_head", C.c_uint64), ("noise", arr(NoiseEntry, NOISE_RING)), ("truth_head", C.c_uint64),
                ("truth", arr(TruthEntry, TRUTH_RING))]


def layout():
    """rt_core --layout과 같은 형식의 위치·크기."""
    out = {name: [getattr(Shm, name).offset, getattr(Shm, name).size]
           for name in ("cfg", "ctl", "stats", "state", "cmd", "op_head", "op", "noise_head", "noise", "truth_head", "truth")}
    sub = {"cfg.qpos0": (Shm.cfg, Config.qpos0), "cfg.torque_limit": (Shm.cfg, Config.torque_limit),
           "ctl.torque_scale": (Shm.ctl, Control.torque_scale), "state.foot_force": (Shm.state, LowState.foot_force),
           "cmd.tau_ff": (Shm.cmd, LowCmd.tau_ff), "truth[0].wake_us": (Shm.truth, TruthEntry.wake_us)}
    out.update({k: [a.offset + b.offset, b.size] for k, (a, b) in sub.items()})
    out["total"] = [0, C.sizeof(Shm)]
    return out


class TerrainHeader(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("apply_step", C.c_int64), ("nrow", C.c_int32), ("ncol", C.c_int32),
                ("mocap_pos", arr(D, 3))]


class TerrainView:
    """지형 공유 메모리 (<이름>_terrain): 헤더 + heightfield 값 (float32, MuJoCo 정규화 높이). 관리 프로세스가 쓴다."""

    def __init__(self, name, nrow, ncol):
        import numpy as np
        self.path = Path("/dev/shm") / (name.lstrip("/") + "_terrain")
        size = C.sizeof(TerrainHeader) + 4 * nrow * ncol
        with open(self.path, "wb") as f:
            f.truncate(size)
        self.fd = os.open(self.path, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, size)
        self.head = TerrainHeader.from_buffer(self.mm)
        self.head.nrow, self.head.ncol = nrow, ncol
        self.data = np.frombuffer(self.mm, dtype=np.float32, count=nrow * ncol, offset=C.sizeof(TerrainHeader))

    def write(self, hfield, mocap_pos, apply_step):
        h = self.head
        s = h.seq
        h.seq = s + 1
        self.data[:] = hfield
        h.mocap_pos[:] = mocap_pos
        h.apply_step = apply_step
        h.seq = s + 2

    def close(self):
        del self.head, self.data
        try:
            self.mm.close()
        except BufferError:
            pass
        os.close(self.fd)
        self.path.unlink(missing_ok=True)


class ScanHeader(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("t", D), ("n", C.c_int32), ("pad", C.c_int32)]


class ScanView:
    """스캔 공유 메모리 (<이름>_scan): 관리 프로세스가 쓰고 (create=True), 같은 PC의 두뇌가 읽는다."""

    def __init__(self, name, create=False):
        import numpy as np
        self.path = Path("/dev/shm") / (name.lstrip("/") + "_scan")
        size = C.sizeof(ScanHeader) + 4 * 3 * SCAN_MAX_PTS
        if create:
            with open(self.path, "wb") as f:
                f.truncate(size)
        self.fd = os.open(self.path, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, size)
        self.head = ScanHeader.from_buffer(self.mm)
        self.pts = np.frombuffer(self.mm, dtype=np.float32, count=3 * SCAN_MAX_PTS, offset=C.sizeof(ScanHeader))
        self.create, self.seen = create, 0

    def write(self, t, points):
        n = min(len(points), SCAN_MAX_PTS)
        h = self.head
        s = h.seq
        h.seq = s + 1
        self.pts[:3 * n] = points[:n].reshape(-1)
        h.t, h.n = t, n
        h.seq = s + 2

    def read_new(self):
        """새 스캔이 있으면 (t, (n, 3) 점), 없으면 None."""
        h = self.head
        a = h.seq
        if a == self.seen or a & 1:
            return None
        t, n = h.t, h.n
        pts = self.pts[:3 * n].reshape(n, 3).copy()
        if h.seq != a:
            return None
        self.seen = a
        return t, pts

    def close(self):
        del self.head, self.pts
        try:
            self.mm.close()
        except BufferError:
            pass
        os.close(self.fd)
        if self.create:
            self.path.unlink(missing_ok=True)


class ShmView:
    def __init__(self, name, create=False):
        self.name = name
        self.path = Path("/dev/shm") / name.lstrip("/")
        size = C.sizeof(Shm)
        if create:
            with open(self.path, "wb") as f:
                f.truncate(size)
        self.fd = os.open(self.path, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, size)
        self.shm = Shm.from_buffer(self.mm)
        if create:
            self.shm.magic, self.shm.version, self.shm.size = MAGIC, VERSION, size
        elif (self.shm.magic, self.shm.version, self.shm.size) != (MAGIC, VERSION, size):
            raise RuntimeError(f"공유 메모리 {name} 규약이 다르다 (version {self.shm.version}, 기대 {VERSION})")
        self.op_seen = 0

    @classmethod
    def open(cls, name, timeout=10.0):
        end = time.monotonic() + timeout
        while True:
            try:
                v = cls(name)
                return v
            except (FileNotFoundError, RuntimeError, ValueError):
                if time.monotonic() > end:
                    raise
                time.sleep(0.05)

    def close(self, unlink=False):
        del self.shm
        try:
            self.mm.close()
        except BufferError:                       # 남은 ctypes 참조가 있으면 가비지 컬렉션 때 닫힌다
            pass
        os.close(self.fd)
        if unlink:
            self.path.unlink(missing_ok=True)

    # ---- 로봇 경계 (sim/robot_io.py의 로봇 상태 메시지와 같은 모양) ----
    def read_state(self):
        st = self.shm.state
        for _ in range(100):
            a = st.seq
            if a & 1 or a == 0:
                continue
            ls = {"t": st.t, "step": st.step, "q": list(st.q), "dq": list(st.dq),
                  "imu": {"quat": list(st.quat_xyzw), "gyro": list(st.gyro), "accel": list(st.accel)},
                  "foot_force": list(st.foot_force)}
            if st.seq == a:
                return ls
        return None

    def write_cmd(self, cmd):
        c = self.shm.cmd
        s = c.seq
        c.seq = s + 1
        c.t = cmd["t"]
        c.q_des[:], c.dq_des[:], c.kp[:], c.kd[:], c.tau_ff[:] = cmd["q_des"], cmd["dq_des"], cmd["kp"], cmd["kd"], cmd["tau_ff"]
        c.seq = s + 2

    def write_est(self, t, est, cmd):
        e = self.shm.est
        s = e.seq
        e.seq = s + 1
        e.t, e.yaw = t, est.yaw
        e.v_body[:], e.contacts[:], e.pos[:], e.cmd[:] = est.v_body, est.contacts.astype(float), est.pos, cmd
        e.seq = s + 2

    def read_est(self):
        """두뇌의 최신 상태 추정 {t, v_body, yaw, contacts, pos} (없으면 None)."""
        e = self.shm.est
        for _ in range(100):
            a = e.seq
            if a == 0 or a & 1:
                if a == 0:
                    return None
                continue
            out = {"t": e.t, "v_body": list(e.v_body), "yaw": e.yaw, "contacts": list(e.contacts), "pos": list(e.pos),
                   "cmd": list(e.cmd)}
            if e.seq == a:
                return out
        return None

    def cmd_t(self):
        """마지막으로 쓴 명령이 계산된 상태 시각 (없으면 None)."""
        return self.shm.cmd.t if self.shm.cmd.seq else None

    # ---- 운용자 이동 명령 (관리 프로세스 -> 별도 프로세스 두뇌) ----
    def push_op(self, t, vx, yaw_rate):
        h = self.shm.op_head
        e = self.shm.op[h % OP_RING]
        e.t, e.vx, e.yaw_rate = t, vx, yaw_rate
        self.shm.op_head = h + 1

    def new_ops(self):
        """아직 읽지 않은 운용자 명령 [(t, vx, yaw_rate)]."""
        out = []
        while self.op_seen < self.shm.op_head:
            e = self.shm.op[self.op_seen % OP_RING]
            out.append((e.t, e.vx, e.yaw_rate))
            self.op_seen += 1
        return out

    @property
    def done(self):
        return bool(self.shm.stats.done)
