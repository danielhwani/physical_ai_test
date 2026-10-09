"""C++ 실시간 코어(rt/rt_core) 관리 프로세스 (문서 §11).

    python -m sim.rt_link --build                                    # 코어 빌드 (cmake)
    python -m sim.rt_link scenarios/flat_trot.yaml                   # 실시간 (SCHED_FIFO 80, CPU 5)
    python -m sim.rt_link scenarios/flat_trot.yaml --lockstep        # 한 스텝씩 (Python 실행과 비교용)

나눈 일:
  C++ 코어 (실시간)     물리(MuJoCo), 로봇 쪽 경계(센서 모델, PD 모터), 고정 주기 루프
  이 프로세스 (실시간 아님) 모델·시작 상태 준비, 센서 잡음 미리 만들기(Python과 같은 난수), 보행 알고리즘(control/, 지금 그대로),
                          시나리오 이벤트(이동 명령, 고장), 넘어짐 판정, MOP
  둘 사이는 공유 메모리 (rt/shm_layout.h, 아래 ctypes 정의와 같아야 한다)
보행 알고리즘이 늦으면 코어는 마지막 명령을 계속 쓰고, 늦은 만큼이 명령 지연 MOP에 드러난다 (실제 로봇과 같다).
지금 지원: 지형이 바뀌지 않는 시나리오 (patches는 시작 지형에 들어감), 이벤트 set_command / inject_fault / clear_fault.
지형 패치 이벤트, 지형 창, Chrono, LiDAR, ROS2 노드, DIS, 가시화는 아직 이 경로에 없다 (python -m sim.runner를 쓴다).
"""
import argparse
import ctypes as C
import json
import mmap
import os
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from .adapters import quat_to_roll_pitch
from .runner import FALL_HEIGHT, FALL_TILT, Simulation, apply_overrides, load_yaml

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "build/rt/rt_core"
MAGIC, VERSION = 0x47324F52, 1
NJ, NQ, NV, NOISE_DIM, NOISE_RING, TRUTH_RING = 12, 19, 18, 36, 4096, 1024
D = C.c_double


def arr(t, n):
    return t * n


class Config(C.Structure):
    _fields_ = [("mode", C.c_int32), ("decim", C.c_int32), ("foot_geom", arr(C.c_int32, 4)), ("terrain_geom", C.c_int32),
                ("accel_adr", C.c_int32), ("cpu", C.c_int32), ("priority", C.c_int32), ("max_steps", C.c_int64),
                ("control_dt", D), ("torque_limit", arr(D, NJ)), ("gyro_bias", arr(D, 3)), ("accel_bias", arr(D, 3)),
                ("yaw_drift", D), ("hold_q", arr(D, NJ)), ("hold_kp", arr(D, NJ)), ("hold_kd", arr(D, NJ)),
                ("qpos0", arr(D, NQ)), ("qvel0", arr(D, NV))]


class Control(C.Structure):
    _fields_ = [("start", C.c_uint64), ("stop", C.c_uint64), ("lockstep_target", C.c_uint64), ("link_mode", C.c_uint64),
                ("torque_scale", D), ("fault_gyro", arr(D, 3)), ("fault_att", arr(D, 3))]


class LowState(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("step", C.c_uint64), ("t", D), ("q", arr(D, NJ)), ("dq", arr(D, NJ)),
                ("quat_xyzw", arr(D, 4)), ("gyro", arr(D, 3)), ("accel", arr(D, 3)), ("foot_force", arr(D, 4))]


class LowCmd(C.Structure):
    _fields_ = [("seq", C.c_uint64), ("t", D), ("q_des", arr(D, NJ)), ("dq_des", arr(D, NJ)), ("kp", arr(D, NJ)),
                ("kd", arr(D, NJ)), ("tau_ff", arr(D, NJ))]


class NoiseEntry(C.Structure):
    _fields_ = [("step", C.c_uint64), ("v", arr(D, NOISE_DIM))]


class TruthEntry(C.Structure):
    _fields_ = [("step", C.c_uint64), ("t", D), ("qpos", arr(D, NQ)), ("qvel", arr(D, NV)), ("ctrl", arr(D, NJ)),
                ("energy", D), ("foot_force", arr(D, 4)), ("cmd_t", D), ("wake_us", D), ("compute_us", D)]


class Stats(C.Structure):
    _fields_ = [("steps", C.c_uint64), ("overruns", C.c_uint64), ("noise_underruns", C.c_uint64), ("done", C.c_uint64),
                ("locked", C.c_uint64), ("rt_ok", C.c_uint64), ("max_wake_us", D), ("max_compute_us", D)]


class Shm(C.Structure):
    _fields_ = [("magic", C.c_uint32), ("version", C.c_uint32), ("size", C.c_uint64), ("cfg", Config), ("ctl", Control),
                ("stats", Stats), ("state", LowState), ("cmd", LowCmd), ("noise_head", C.c_uint64),
                ("noise", arr(NoiseEntry, NOISE_RING)), ("truth_head", C.c_uint64), ("truth", arr(TruthEntry, TRUTH_RING))]


def layout():
    """rt_core --layout과 같은 형식의 ctypes 위치·크기."""
    out = {}
    for name in ("cfg", "ctl", "stats", "state", "cmd", "noise_head", "noise", "truth_head", "truth"):
        f = getattr(Shm, name)
        out[name] = [f.offset, f.size]
    out["cfg.qpos0"] = [Shm.cfg.offset + Config.qpos0.offset, Config.qpos0.size]
    out["cfg.torque_limit"] = [Shm.cfg.offset + Config.torque_limit.offset, Config.torque_limit.size]
    out["ctl.torque_scale"] = [Shm.ctl.offset + Control.torque_scale.offset, Control.torque_scale.size]
    out["state.foot_force"] = [Shm.state.offset + LowState.foot_force.offset, LowState.foot_force.size]
    out["cmd.tau_ff"] = [Shm.cmd.offset + LowCmd.tau_ff.offset, LowCmd.tau_ff.size]
    out["truth[0].wake_us"] = [Shm.truth.offset + TruthEntry.wake_us.offset, TruthEntry.wake_us.size]
    out["total"] = [0, C.sizeof(Shm)]
    return out


def build():
    mj_dir = os.path.dirname(mujoco.__file__)
    subprocess.run(["cmake", "-S", str(ROOT / "rt"), "-B", str(ROOT / "build/rt"), f"-DMUJOCO_DIR={mj_dir}"], check=True,
                   stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", str(ROOT / "build/rt")], check=True)
    return CORE


class RtLink:
    """Python 시뮬레이터(sim.runner.Simulation)로 모델·시작 상태·보행 알고리즘을 준비하고 C++ 코어를 띄운다."""

    def __init__(self, sim, lockstep=False, cpu=5, priority=80, name=None):
        if not CORE.exists():
            build()
        self.sim, self.lockstep = sim, lockstep
        self.name = name or f"/go2_rt_{os.getpid()}"
        self.model_path = ROOT / f"build/rt/model_{os.getpid()}.mjb"
        mujoco.mj_saveModel(sim.model, str(self.model_path), None)
        size = C.sizeof(Shm)
        self.path = Path("/dev/shm") / self.name.lstrip("/")
        with open(self.path, "wb") as f:
            f.truncate(size)
        self.fd = os.open(self.path, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, size)
        self.shm = Shm.from_buffer(self.mm)
        s, d, m = self.shm, sim.data, sim.model
        s.magic, s.version, s.size = MAGIC, VERSION, size
        cfg = s.cfg
        cfg.mode, cfg.decim, cfg.cpu, cfg.priority = int(lockstep), sim.decim, cpu, priority
        cfg.foot_geom[:] = sim.foot_ids
        cfg.terrain_geom, cfg.accel_adr = sim.terrain_geom, sim.robot.accel_adr
        self.control_dt = sim.decim * m.opt.timestep
        cfg.control_dt = self.control_dt
        cfg.max_steps = int(round(sim.scn["duration"] / self.control_dt))
        cfg.torque_limit[:] = sim.robot.torque_limit
        sens = sim.robot.sensors
        cfg.gyro_bias[:], cfg.accel_bias[:], cfg.yaw_drift = sens.gyro_bias, sens.accel_bias, float(sens.yaw_drift)
        hold = sim.hold_cmd()
        cfg.hold_q[:], cfg.hold_kp[:], cfg.hold_kd[:] = hold["q_des"], hold["kp"], hold["kd"]
        cfg.qpos0[:], cfg.qvel0[:] = d.qpos, d.qvel
        s.ctl.torque_scale = 1.0
        self.noise_n = 0
        self.fill_noise(NOISE_RING // 2)
        self.proc = subprocess.Popen([str(CORE), str(self.model_path), self.name])

    def fill_noise(self, ahead):
        """표본 번호 (지금 코어 스텝 + ahead)까지 잡음을 채운다 (sensors/proprio.py와 같은 난수)."""
        upto = min(self.shm.stats.steps + ahead, self.shm.stats.steps + NOISE_RING - 1)
        sens = self.sim.robot.sensors
        while self.noise_n < upto:
            e = self.shm.noise[self.noise_n % NOISE_RING]
            e.v[:] = sens.draw_noise(self.noise_n)
            e.step = self.noise_n
            self.noise_n += 1
        self.shm.noise_head = self.noise_n

    def start(self):
        self.shm.ctl.start = 1

    def stop(self):
        self.shm.ctl.stop = 1

    def close(self):
        self.stop()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        del self.shm
        try:
            self.mm.close()
        except BufferError:                              # 남은 ctypes 참조가 있으면 가비지 컬렉션 때 닫힌다
            pass
        os.close(self.fd)
        for p in (self.path, self.model_path):
            p.unlink(missing_ok=True)

    # ---- 로봇 경계 (보행 알고리즘 쪽에서 보는 메시지, sim/robot_io.py와 같은 모양) ----
    def read_state(self):
        st = self.shm.state
        for _ in range(100):
            a = st.seq
            if a & 1:
                continue
            ls = {"t": st.t, "step": st.step, "q": list(st.q), "dq": list(st.dq),
                  "imu": {"quat": list(st.quat_xyzw), "gyro": list(st.gyro), "accel": list(st.accel)},
                  "foot_force": list(st.foot_force)}
            if st.seq == a and a > 0:
                return ls
        return None

    def write_cmd(self, cmd):
        c = self.shm.cmd
        s = c.seq
        c.seq = s + 1
        c.t = cmd["t"]
        c.q_des[:], c.dq_des[:], c.kp[:], c.kd[:], c.tau_ff[:] = cmd["q_des"], cmd["dq_des"], cmd["kp"], cmd["kd"], cmd["tau_ff"]
        c.seq = s + 2

    def truth(self, k):
        e = self.shm.truth[k % TRUTH_RING]
        return e if e.step == k else None


def run(scenario, lockstep=False, cpu=5, priority=80, verbose=True):
    """시나리오를 C++ 코어로 실행하고 MOP를 돌려준다."""
    sim = Simulation(scenario)
    unsupported = [e for e in sim.events if e["action"] not in ("set_command", "inject_fault", "clear_fault")]
    if unsupported or sim.cosim is not None or sim.sensor_renderer is not None or sim.terrain.windowed:
        sim.close()
        raise SystemExit("실시간 코어 경로는 아직 지형 패치 이벤트, 지형 창, Chrono, LiDAR를 지원하지 않는다 (python -m sim.runner)")
    link = RtLink(sim, lockstep, cpu, priority)
    node, events = sim.controller, list(sim.events)
    start_xy = sim.data.qpos[:2].copy()
    faults = {}
    energy, fell_at, k_truth, last_step = 0.0, None, 0, -1
    wake, comp, lat = [], [], []
    qpos = None
    try:
        link.start()
        n_steps = link.shm.cfg.max_steps
        while True:
            if lockstep:
                if last_step + 1 >= n_steps:
                    break
                link.shm.ctl.lockstep_target = last_step + 1
                while link.shm.stats.steps < last_step + 2 and not link.shm.stats.done:
                    time.sleep(0.00002)
            else:
                if link.shm.stats.done:
                    break
                time.sleep(0.0005)
            ls = link.read_state()
            if ls is not None and ls["step"] != last_step:
                last_step = ls["step"]
                t = ls["t"]
                while events and events[0]["t"] <= t + 1e-9:          # 시나리오 이벤트 (sim/runner.py와 같은 뜻)
                    e = events.pop(0)
                    if e["action"] == "set_command":
                        node.set_command(e["vx"], e.get("yaw_rate", 0.0))
                    elif e["action"] == "inject_fault":
                        faults[e["fault"]] = (None if e.get("duration") is None else t + e["duration"], e.get("params") or {})
                    elif e["action"] == "clear_fault":
                        faults.pop(e["fault"], None)
                    if verbose:
                        print(f"[t={t:6.2f}] event: {e['action']}")
                for f in [f for f, (until, _) in faults.items() if until is not None and t >= until - 1e-9]:
                    del faults[f]
                ctl = link.shm.ctl
                ctl.torque_scale = float(faults["battery_low"][1].get("torque_scale", 0.6)) if "battery_low" in faults else 1.0
                ip = faults.get("imu_bias", (None, {}))[1]
                ctl.fault_gyro[:] = ip.get("gyro_bias", (0, 0, 0))
                ctl.fault_att[:] = np.radians(ip.get("attitude_offset_deg", (0, 0, 0)))
                if "link_loss" in faults:
                    ctl.link_mode = 2 if faults["link_loss"][1].get("robot_behavior") == "damp" else 1
                else:
                    ctl.link_mode = 0
                    cmd = node.step(ls)                                # 보행 알고리즘 (참값을 모른다)
                    link.write_cmd(cmd)
                link.fill_noise(NOISE_RING // 2)
            while k_truth < link.shm.truth_head:                       # 판정자 쪽 참값
                e = link.truth(k_truth)
                if e is None:                                          # 관리 프로세스가 너무 늦어 링을 놓침
                    k_truth += 1
                    continue
                energy += e.energy
                wake.append(e.wake_us); comp.append(e.compute_us)
                if e.cmd_t >= 0:
                    lat.append(e.t - link.control_dt - e.cmd_t)     # 실행한 스텝의 상태 시각 - 명령이 쓴 상태 시각
                qpos = np.array(e.qpos)
                roll, pitch = quat_to_roll_pitch(qpos[3:7])
                if (qpos[2] - sim.terrain.height_at(*qpos[:2]) < FALL_HEIGHT or max(abs(roll), abs(pitch)) > FALL_TILT) and fell_at is None:
                    fell_at = e.t
                    if verbose:
                        print(f"[t={e.t:6.2f}] FALL detected")
                    link.stop()
                k_truth += 1
            if fell_at is not None and (link.shm.stats.done or lockstep):
                break
        st = link.shm.stats
        dist = float(np.linalg.norm(qpos[:2] - start_xy))
        wake, comp = np.array(wake), np.array(comp)
        mop = {
            "mode": "lockstep" if lockstep else "realtime", "steps": int(st.steps), "sim_time_s": round(int(st.steps) * link.control_dt, 3),
            "fell": fell_at is not None, "fell_at_s": fell_at,
            "forward_x_m": round(float(qpos[0] - start_xy[0]), 3), "lateral_drift_m": round(float(qpos[1] - start_xy[1]), 3),
            "cost_of_transport": round(energy / (sim.total_mass * 9.81 * max(dist, 1e-6)), 3),
            "cmd_latency_ms_mean": round(1000 * float(np.mean(lat)), 1) if lat else None,
            "cmd_latency_ms_max": round(1000 * float(np.max(lat)), 1) if lat else None,
            "noise_underruns": int(st.noise_underruns), "mlockall": bool(st.locked), "sched_fifo": bool(st.rt_ok),
        }
        if not lockstep:
            mop.update({
                "wake_us_mean": round(float(wake.mean()), 1), "wake_us_p99": round(float(np.percentile(wake, 99)), 1),
                "wake_us_max": round(float(wake.max()), 1), "compute_us_mean": round(float(comp.mean()), 1),
                "compute_us_p99": round(float(np.percentile(comp, 99)), 1), "compute_us_max": round(float(comp.max()), 1),
                "overruns": int(st.overruns)})
        return mop, qpos
    finally:
        link.close()
        sim.close()


def main():
    ap = argparse.ArgumentParser(description="C++ 실시간 코어로 시나리오 실행")
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--build", action="store_true", help="코어 빌드 (cmake)")
    ap.add_argument("--lockstep", action="store_true", help="실시간 대신 한 스텝씩 (Python 실행과 비교용)")
    ap.add_argument("--cpu", type=int, default=5, help="코어를 고정할 CPU (-1: 고정 안 함)")
    ap.add_argument("--priority", type=int, default=80, help="SCHED_FIFO 우선순위 (0: 일반)")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE")
    ap.add_argument("--layout", action="store_true", help="공유 메모리 규약 위치·크기 (ctypes)")
    a = ap.parse_args()
    if a.build:
        print(f"빌드: {build()}")
    if a.layout:
        print(json.dumps(layout(), indent=2))
    if a.scenario:
        mop, _ = run(apply_overrides(load_yaml(a.scenario), a.set), a.lockstep, a.cpu, a.priority)
        print("\n== MOP (C++ 실시간 코어) ==")
        for k, v in mop.items():
            print(f"  {k:20s} {v}")


if __name__ == "__main__":
    main()
