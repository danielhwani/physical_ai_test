"""C++ 실시간 코어(rt/rt_core) 관리 프로세스 (문서 §11).

    python -m sim.rt_link --build                                    # 코어 빌드 (cmake)
    python -m sim.rt_link scenarios/flat_trot.yaml                   # 실시간 (SCHED_FIFO 80, CPU 5)
    python -m sim.rt_link scenarios/flat_trot.yaml --lockstep        # 한 스텝씩 (Python 실행과 비교용)
    python -m sim.rt_link scenarios/flat_trot.yaml --brain shm       # 두뇌를 같은 PC의 별도 프로세스로 (공유 메모리)
    python -m sim.rt_link scenarios/flat_trot.yaml --brain ros       # 두뇌를 ROS2 노드로 (같은 PC 또는 다른 PC)
    python -m sim.rt_link scenarios/flat_trot.yaml --rviz            # RViz로 보기 (--mjviz: MuJoCo 렌더러)

상위 제어기(두뇌, 보행 알고리즘)와 하위 제어기(C++ 코어: PD 모터 + 물리 + 센서)를 나눈다. 코어는 항상 공유 메모리만 보고,
두뇌 연결 방식은 고른다:
  --brain inproc  이 프로세스 안 (함수 호출, 기본)
  --brain shm     같은 PC의 별도 프로세스 control/rt_brain.py (공유 메모리를 직접 읽고 씀)
  --brain ros     ROS2 노드 control/ros_node.py (/robot/low_state, /robot/low_cmd, /cmd_vel_stamped). 이 프로세스가
                  공유 메모리와 ROS2 사이를 중계한다. --remote-brain이면 노드를 띄우지 않고 다른 PC의 노드를 기다린다
두뇌 코드(control/node.py ControllerNode)는 세 방식 모두 같다. lockstep에서는 세 방식이 비트 단위로 같은 결과를 낸다.

이 프로세스(실시간 아님)의 일: 모델·시작 상태 준비, 센서 잡음 미리 만들기(Python과 같은 난수), 시나리오 이벤트(이동 명령, 고장),
넘어짐 판정, MOP, 기록(runs/<시나리오>_rt_<시각>/timeseries.parquet: sim.runner와 같은 열 + 실시간 타이밍 열), 가시화(--rviz/--mjviz: 참값으로 자기 쪽 MuJoCo 데이터를 맞춰 sim.runner와 같은 렌더 스트림을 발행.
실시간 루프 밖이라 화면이 느려도 코어 주기는 그대로). 두뇌가 늦으면 코어는 마지막 명령을 계속 쓰고, 늦은 만큼이 명령 지연 MOP에 드러난다 (실제 로봇과 같다).
지형: 이 프로세스가 sim.runner와 같은 규칙(지형 패치 이벤트, 5 Hz 반영, 발 근처 갱신 보류, 지형 창 이동)으로 계산해
지형 공유 메모리로 넘기고, 코어는 다음 제어 주기 시작에 반영한다 (lockstep에서 sim.runner와 같은 스텝).
지금 지원하는 이벤트: set_command, add_patch, inject_fault, clear_fault (+ DIS 콘솔의 freeze, stop).
DIS 콘솔 (--dis-port): sim/dis_server.py를 그대로 쓴다 (제어권, 인증, 통신 두절, 원본 PDU 기록, scenario_replay.yaml).
LiDAR·지형 인지 보행: 이 프로세스가 코어가 넘긴 스텝 직후 바디 위치로 센서를 계산하고(sim.runner와 같은 순서와 값), 스캔을 두뇌로
보낸다 (inproc: 함수, shm: 스캔 공유 메모리, ros: /sensors/<이름>/points).
Chrono (HMMWV, 변형 지면, 발자국): 이 프로세스가 sim.runner와 같은 시각에 Chrono와 교환하고(로봇 위치, 발 하중 -> 차량 포즈,
흙 높이), 바뀐 지형을 지형 공유 메모리로 코어에 보낸다. lockstep은 결과를 기다린다 (sim.runner와 비트 단위로 같음).
실시간은 기다리지 않는다: 결과가 아직 없으면 직전 지형·차량을 쓴다. Chrono가 실시간보다 느려 chrono_max_lag(기본 0.2 s)보다
뒤처지면 몸(코어)을 잠깐 멈춰(일시정지 칸) 따라잡게 한다: 세계 시간이 Chrono 속도로 느려지는 대신 차와 로봇이 같은 시각에 있다
(뒤처짐을 그냥 두면 HMMWV가 몇 초 늦게 와 로봇을 뚫고 지나갔다). 0이면 상한 없음 (뒤처진 시간만 MOP로).
(이 PC: 발자국은 실시간의 약 4.5배 빠르고, HMMWV는 약 0.75배.)
"""
import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import platform

import numpy as np
import yaml

from rt import shm as R

from .adapters import quat_to_roll_pitch
from .adapters import quat_to_yaw
from .recorder import Recorder
from .runner import (FALL_HEIGHT, FALL_TILT, SIM_VERSION, TERRAIN_COMMIT_HZ, Simulation, apply_overrides, load_yaml,
                     terrain_deformation)
from .stream import build_status

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "build/rt/rt_core"
EVENT_LEAD_S = 1.0          # 별도 프로세스 두뇌에 이동 명령을 미리 보내는 시간 (명령에 적용 시각이 붙어 있어 결과는 같다)
VIZ_DT = 0.04               # 가시화 발행 주기 (시뮬레이션 시간 s, 25 Hz)
CHRONO_LAG_WARN = 0.5      # 실시간, 상한 없음: Chrono가 이만큼(s) 뒤처지면 한 번 경고한다
CHRONO_MAX_LAG = 0.2       # 실시간: Chrono가 이만큼(s) 뒤처지면 코어를 멈추고 절반까지 따라잡으면 다시 돈다
SENSOR_WORKERS = 2          # 실시간 LiDAR 작업 프로세스 수 (스캔 한 번 약 98 ms라 10 Hz에 하나로는 모자람, sim/rt_sensor.py)


def layout():
    return R.layout()


def build():
    mj_dir = os.path.dirname(mujoco.__file__)
    subprocess.run(["cmake", "-S", str(ROOT / "rt"), "-B", str(ROOT / "build/rt"), f"-DMUJOCO_DIR={mj_dir}"], check=True,
                   stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", str(ROOT / "build/rt")], check=True)
    return CORE


def steps_for(duration, timestep, decim):
    """sim.runner와 같은 스텝 수: 물리 스텝마다 시각을 더하며 '시각 < duration'인 동안 (부동소수점 누적까지 같게)."""
    t, n = 0.0, 0
    while t < duration:
        for _ in range(decim):
            t += timestep
        n += 1
    return n


class RtLink:
    """Python 시뮬레이터(sim.runner.Simulation)로 모델·시작 상태를 준비하고 C++ 코어를 띄운다."""

    def __init__(self, sim, lockstep=False, cpu=5, priority=80, name=None):
        if not CORE.exists():
            build()
        self.sim, self.lockstep = sim, lockstep
        self.name = name or f"/go2_rt_{os.getpid()}"
        self.model_path = ROOT / f"build/rt/model_{os.getpid()}.mjb"
        mujoco.mj_saveModel(sim.model, str(self.model_path), None)
        self.view = R.ShmView(self.name, create=True)
        self.shm = s = self.view.shm
        d, m = sim.data, sim.model
        cfg = s.cfg
        cfg.mode, cfg.decim, cfg.cpu, cfg.priority = int(lockstep), sim.decim, cpu, priority
        cfg.foot_geom[:] = sim.foot_ids
        cfg.terrain_geom, cfg.accel_adr, cfg.base_body = sim.terrain_geom, sim.robot.accel_adr, sim.base_id
        cfg.hfield_id, cfg.terrain_mocap = sim.hfield_id, int(sim.terrain_mocap)
        self.control_dt = sim.decim * m.opt.timestep
        cfg.control_dt = self.control_dt
        cfg.max_steps = steps_for(sim.scn["duration"], m.opt.timestep, sim.decim)
        cfg.torque_limit[:] = sim.robot.torque_limit
        sens = sim.robot.sensors
        cfg.gyro_bias[:], cfg.accel_bias[:], cfg.yaw_drift = sens.gyro_bias, sens.accel_bias, float(sens.yaw_drift)
        hold = sim.hold_cmd()
        cfg.hold_q[:], cfg.hold_kp[:], cfg.hold_kd[:] = hold["q_des"], hold["kp"], hold["kd"]
        cfg.qpos0[:], cfg.qvel0[:] = d.qpos, d.qvel
        s.ctl.torque_scale = 1.0
        self.noise_n = 0
        self.fill_noise(R.NOISE_RING // 2)
        self.terrain = R.TerrainView(self.name, int(m.hfield_nrow[sim.hfield_id]), int(m.hfield_ncol[sim.hfield_id]))
        adr, n = int(m.hfield_adr[sim.hfield_id]), int(m.hfield_nrow[sim.hfield_id] * m.hfield_ncol[sim.hfield_id])
        self.hfield = slice(adr, adr + n)
        self.scan = R.ScanView(self.name, create=True) if sim.sensor_renderer is not None else None
        self.proc = subprocess.Popen([str(CORE), str(self.model_path), self.name, self.name + "_terrain"])

    def send_terrain(self, apply_step):
        """관리 프로세스 모델의 heightfield와 지형 창 위치를 코어로 (apply_step 스텝 시작부터)."""
        sim = self.sim
        mp = sim.data.mocap_pos[sim.terrain_mocap] if sim.terrain_mocap >= 0 else (0.0, 0.0, 0.0)
        self.terrain.write(sim.model.hfield_data[self.hfield], mp, apply_step)

    def fill_noise(self, ahead):
        """표본 번호 (지금 코어 스텝 + ahead)까지 잡음을 채운다 (sensors/proprio.py와 같은 난수)."""
        upto = min(self.shm.stats.steps + ahead, self.shm.stats.steps + R.NOISE_RING - 1)
        sens = self.sim.robot.sensors
        while self.noise_n < upto:
            e = self.shm.noise[self.noise_n % R.NOISE_RING]
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
        self.view.close(unlink=True)
        self.terrain.close()
        if self.scan is not None:
            self.scan.close()
        self.model_path.unlink(missing_ok=True)

    def read_state(self):
        return self.view.read_state()

    def write_cmd(self, cmd):
        self.view.write_cmd(cmd)

    def truth(self, k):
        e = self.shm.truth[k % R.TRUTH_RING]
        return e if e.step == k else None


# ---- 두뇌(상위 제어기) 연결 방식 ----
class InprocBrain:
    """이 프로세스 안에서 보행 알고리즘을 부른다."""
    name = "inproc"

    def __init__(self, sim, link):
        self.node, self.link = sim.controller, link

    def ready(self):
        return True

    def command(self, t, vx, wz):              # 그 시각 상태보다 먼저 불린다 (sim/runner.py와 같은 순서)
        self.node.set_command(vx, wz)

    def on_state(self, ls):
        self.link.write_cmd(self.node.step(ls))

    def on_scan(self, name, scan):
        if name == self.node.sensor:
            self.node.on_scan(name, scan["t"], scan["points"])

    def poll(self):
        pass

    def cmd_t(self):
        return self.link.view.cmd_t()

    def est_ready(self, t):
        return True

    def close(self):
        pass


class ShmBrain(InprocBrain):
    """같은 PC의 별도 프로세스 (control/rt_brain.py)가 공유 메모리를 직접 읽고 쓴다."""
    name = "shm"
    lead = EVENT_LEAD_S

    def __init__(self, sim, link, scn_path, policy=None, log_dir=None):
        self.link, self.sensor = link, sim.controller.sensor
        cmd = [sys.executable, "-m", "control.rt_brain", "--shm", link.name, "--scenario", str(scn_path)]
        if policy:
            cmd += ["--policy", str(policy)]
        self.proc = subprocess.Popen(cmd, cwd=ROOT, stdout=open((log_dir or ROOT / "build/rt") / "rt_brain.log", "w"),
                                     stderr=subprocess.STDOUT)

    def ready(self):
        if self.proc.poll() is not None:
            raise RuntimeError("두뇌 프로세스가 끝났다 (build/rt/rt_brain.log)")
        return bool(self.link.shm.ctl.brain_ready)

    def command(self, t, vx, wz):              # 적용 시각을 붙여 미리 보낸다
        self.link.view.push_op(t, vx, wz)

    def on_scan(self, name, scan):
        if name == self.sensor:
            self.link.scan.write(scan["t"], scan["points"])

    def on_state(self, ls):
        pass

    def close(self):
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.terminate()


class RosBrain(InprocBrain):
    """ROS2 노드 (control/ros_node.py). 이 프로세스가 공유 메모리 <-> ROS2 토픽을 중계한다 (같은 PC 또는 다른 PC)."""
    name = "ros"
    lead = EVENT_LEAD_S

    def __init__(self, sim, link, scn_path, policy=None, remote=False, log_dir=None, bridge=None, viz=False):
        from .ros2_bridge import Ros2StreamPublisher, launch_controller_node
        self.link, self.remote = link, remote
        self.bridge = bridge or Ros2StreamPublisher(sim.model, sim.terrain, sim.cosim)
        self.bridge.enable_robot_io()
        self.proc = None if remote else launch_controller_node(log_dir or ROOT / "build/rt", scn_path, policy, viz=viz)
        self.forwarded = None

    def ready(self):
        if self.proc is not None and self.proc.poll() is not None:
            raise RuntimeError("두뇌 노드가 끝났다 (build/rt/controller_node.log)")
        return self.bridge.node.count_subscribers("/robot/low_state") > 0 and \
            self.bridge.node.count_subscribers("/cmd_vel_stamped") > 0

    def command(self, t, vx, wz):
        self.bridge.publish_cmd_vel(vx, wz, t)

    def on_scan(self, name, scan):
        self.bridge.publish_scan(name, scan)            # 노드가 /sensors/<이름>/points를 구독한다

    def on_state(self, ls):
        self.bridge.publish_low_state(ls)
        self.poll()

    def poll(self):
        self.bridge.spin_some()
        c = self.bridge.latest_cmd
        if c is not None and (self.forwarded is None or c["t"] > self.forwarded):
            self.link.write_cmd(c)
            self.forwarded = c["t"]

    def est_ready(self, t):                    # lockstep: 상태 추정 메시지도 받은 뒤 기록 (명령과 따로 도착한다)
        e = self.bridge.latest_est
        return e is not None and e["t"] >= t - 1e-9

    def close(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()


def run(scenario, lockstep=False, cpu=5, priority=80, verbose=True, brain="inproc", policy=None, remote=False,
        viz=(), out_dir=None, variant="cpu", overrides=(), dis=None, chrono_max_lag=CHRONO_MAX_LAG):
    """시나리오를 C++ 코어로 실행하고 (MOP, 끝 qpos)를 돌려준다.
    viz: ("rviz", "mjviz") 중 고른 것. out_dir: 기록 폴더 (timeseries.parquet, summary.json, 로그. 없으면 남기지 않음).
    variant: 모델 설정 (cpu / mjx, sim.runner --variant와 같다).
    dis: DIS 콘솔 설정 {port, host, key(bytes), wait, comm_lost} (실시간 전용).
    chrono_max_lag: 실시간에서 Chrono가 이보다(s) 뒤처지면 코어를 멈춰 기다린다 (0: 상한 없음)."""
    sim = Simulation(scenario, policy=policy, variant=variant)
    supported = ("set_command", "add_patch", "inject_fault", "clear_fault", "create_entity", "remove_entity", "set_soil")
    unsupported = [e["action"] for e in sim.events if e["action"] not in supported]
    if unsupported:
        sim.close()
        raise SystemExit(f"실시간 코어 경로가 지원하지 않는 이벤트: {unsupported}")
    link = RtLink(sim, lockstep, cpu, priority)
    scn_path = ROOT / f"build/rt/scenario_{os.getpid()}.yaml"
    scn_path.write_text(yaml.safe_dump(sim.scn, allow_unicode=True, sort_keys=False))
    br, bridge, displays = None, None, []
    events, faults = sim.events, {}           # DIS 서버가 같은 목록에 이벤트를 넣는다
    inflight = []                              # 두뇌에 미리 보낸 이동 명령 (적용 시각이 되면 적용된 것으로 기록)
    stop_after = [None]                        # 종료: 이 스텝까지 돌고 멈춘다 (sim.runner와 같이 그 주기를 마친 뒤)
    shown_cmds = []                                                # 정보판용 (적용 시각, vx, wz)
    sim.remote = True          # 정보판의 명령 표시를 운용자 명령(sim.command)으로 (두뇌가 다른 프로세스일 수 있다)
    sim.faults = {}
    next_viz = 0.0
    rec = None
    if out_dir is not None:                                        # 기록 (sim.runner와 같은 형식, 문서 §4, §13.3)
        rec = Recorder(out_dir, {
            "scenario": sim.scn["name"], "seed": sim.scn.get("seed", 0), "source": "Virtual", "overrides": list(overrides),
            "mode": "lockstep" if lockstep else "realtime", "model_variant": variant,
            "control_link": {"mode": f"rt_core+{brain}", "latency_steps": 1},
            "rt_core": {"cpu": cpu, "priority": priority, "control_dt": link.control_dt},
            "versions": {"sim": SIM_VERSION, "spec": sim.spec["spec_version"], "controller": sim.controller_desc,
                         "mujoco": mujoco.__version__, "model": sim.spec["robot"]["menagerie_commit"]},
            "host": platform.node(), "started": dt.datetime.now().strftime("%Y%m%d_%H%M%S")})
    prev_contact = None
    touchdowns, edge_touchdowns, est_err_v, true_by_t = 0, 0, [], {}
    d0 = sim.data                                                  # 시작 상태의 참값 (첫 상태 추정과 비교, sim.runner와 같게)
    true_by_t[round(d0.time, 6)] = (d0.xmat[sim.base_id].reshape(3, 3).T @ d0.qvel[:3], quat_to_yaw(d0.qpos[3:7]))
    start_xy = sim.data.qpos[:2].copy()
    terrain0 = sim.terrain.applied.copy()                          # 지형 변형 MOP (Chrono 흙, sim.runner와 같은 계산)
    energy, fell_at, k_truth, last_step = 0.0, None, 0, -1
    wake, comp, lat, qpos = [], [], [], None

    def mark_applied(e, t):
        e["applied_t"] = t                     # DIS 서버가 이걸 보고 적용 완료를 응답한다. 다시 실행할 시나리오에도 쓰인다
        sim.event_log.append(e)

    def apply_events(t_next, step_next):
        """다음 스텝(step_next, 상태 시각 t_next)까지의 이벤트 (sim/runner.py와 같은 뜻). 고장은 코어로, 이동 명령은 두뇌로."""
        lead = getattr(br, "lead", 0.0)
        for e in [e for e in list(events) if e["t"] <= t_next + 1e-9 + (lead if e["action"] == "set_command" else 0.0)]:
            events.remove(e)
            if e["action"] == "set_command":
                br.command(e["t"], e["vx"], e.get("yaw_rate", 0.0))
                shown_cmds.append((e["t"], e["vx"], e.get("yaw_rate", 0.0)))
                inflight.append(e)
                continue
            mark_applied(e, t_next)
            if e["action"] == "freeze":
                sim.frozen = True
            elif e["action"] == "stop":
                sim.stop_requested = True
                stop_after[0] = step_next
            elif e["action"] == "add_patch":
                sim.terrain.add_patch(e["patch"])
            elif e["action"] == "inject_fault":
                faults[e["fault"]] = (None if e.get("duration") is None else e["t"] + e["duration"], e.get("params") or {})
            elif e["action"] == "clear_fault":
                faults.pop(e["fault"], None)
            elif e["action"] == "create_entity":                  # 개체 투입 (Chrono 차량 슬롯)
                sim.cosim.spawn_vehicle(e.get("speed"))
                sim.cosim.near_event_sent = False
            elif e["action"] == "remove_entity":
                sim.cosim.remove_vehicle()
            elif e["action"] == "set_soil":
                sim.cosim.set_soil(e["soil"])
            if verbose:
                print(f"[t={t_next:6.2f}] event: {e['action']}" + (f" {e.get('fault', '')}" if "fault" in e else "")
                      + (f"  (DIS {e['request']})" if e.get("source") == "dis" else ""))
            sim.last_event = (e["t"], e["action"] + (f" {e['fault']}" if "fault" in e else ""))
        for e in [e for e in inflight if e["t"] <= t_next + 1e-9]:      # 두뇌가 이 상태부터 적용한다
            inflight.remove(e)
            mark_applied(e, t_next)
            if verbose:
                print(f"[t={t_next:6.2f}] event: set_command vx={e['vx']:.2f} yaw={e.get('yaw_rate', 0.0):.2f}"
                      + (f"  (DIS {e['request']})" if e.get("source") == "dis" else ""))
            sim.last_event = (t_next, f"set_command vx={e['vx']:.2f} yaw={e.get('yaw_rate', 0.0):.2f}")
        for f in [f for f, (until, _) in faults.items() if until is not None and t_next >= until - 1e-9]:
            del faults[f]
        while shown_cmds and shown_cmds[0][0] <= t_next + 1e-9:
            _, vx, wz = shown_cmds.pop(0)
            sim.command = (vx, wz)
        sim.faults = {n: {"until": until, "params": prm} for n, (until, prm) in faults.items()}
        ctl = link.shm.ctl
        ctl.torque_scale = float(faults["battery_low"][1].get("torque_scale", 0.6)) if "battery_low" in faults else 1.0
        ip = faults.get("imu_bias", (None, {}))[1]
        ctl.fault_gyro[:] = ip.get("gyro_bias", (0, 0, 0))
        ctl.fault_att[:] = np.radians(ip.get("attitude_offset_deg", (0, 0, 0)))
        lf = faults.get("link_loss")
        ctl.link_mode = 0 if lf is None else 2 if lf[1].get("robot_behavior") == "damp" else 1

    next_commit = [0.0]

    chrono_lag = [0.0, False]                  # 실시간: Chrono가 로봇 시각보다 뒤처진 최대 시간 (s), 경고 했는지
    chrono_hold = {"on": False, "since": 0.0, "count": 0, "wall": 0.0}   # 실시간: Chrono를 기다리느라 코어를 멈춘 상태·횟수·시간

    def cosim_step(t_now, base_xy):
        """Chrono 교환 (sim.runner Simulation.step_control과 같은 자리: 이벤트 뒤, 지형 갱신 전).
        실시간이면 기다리지 않는다 (결과가 없으면 직전 지형·차량 그대로)."""
        if sim.cosim is None:
            return
        c = sim.cosim
        c.robot_xy = np.array(base_xy[:2], float)                  # 다음 Chrono 요청에 실림 (차량 양보 판단)
        c.sync(t_now, block=lockstep)                              # 결과 반영 (지형 원천 갱신 + 차량 포즈)
        if not lockstep:
            lag = c.lag(t_now)
            chrono_lag[0] = max(chrono_lag[0], lag)
            if chrono_max_lag and lag > chrono_max_lag and not chrono_hold["on"]:   # 코어를 멈추고 Chrono를 기다린다
                chrono_hold.update(on=True, since=time.monotonic(), count=chrono_hold["count"] + 1)
                link.shm.ctl.freeze = 1
                if chrono_hold["count"] == 1:
                    sim.last_event = (t_now, "chrono_slow")
                    if verbose:
                        print(f"[t={t_now:6.2f}] Chrono가 실시간보다 느려 {chrono_max_lag:.1f} s 넘게 뒤처지면 세계를 잠깐 멈춰 기다린다")
            if not chrono_max_lag and lag > CHRONO_LAG_WARN and not chrono_lag[1]:
                chrono_lag[1] = True
                sim.last_event = (t_now, f"chrono_lag {lag:.1f}s")
                if verbose:
                    print(f"[t={t_now:6.2f}] 경고: Chrono가 실시간보다 느려 {lag:.2f} s 뒤처졌다 (차량·흙이 늦게 보인다)")
        dist = c.distance_to(c.robot_xy)
        if dist < c.near_dist and not c.near_event_sent:
            c.near_event_sent = True
            sim.last_event = (t_now, f"vehicle_near {dist:.1f}m")
            if verbose:
                print(f"[t={t_now:6.2f}] event: vehicle_near ({dist:.2f} m)")

    def terrain_step(t_now, base_xy, feet_xy, step):
        """sim.runner Simulation.step_control의 지형 갱신과 같은 규칙. 바뀌었으면 코어로 보낸다."""
        changed = False
        if t_now >= next_commit[0]:
            T = sim.terrain
            if T.recenter(*base_xy, trigger=sim.window_trigger):          # 지형 창 이동
                sim.data.mocap_pos[sim.terrain_mocap] = [*T.window_center, 0.0]
                T.write_all(sim.model, sim.hfield_id)
                changed = True
            changed |= T.commit(sim.model, sim.hfield_id, feet_xy, sim.guard_radius)   # 발 근처 갱신 보류
            next_commit[0] += 1.0 / TERRAIN_COMMIT_HZ
        if changed:
            link.send_terrain(step)

    def mirror(e):
        """참값으로 관리 프로세스의 MuJoCo 데이터를 맞춘다 (위치, 접촉. 물리 진행은 하지 않음)."""
        d = sim.data
        d.qpos[:], d.qvel[:], d.time = e.qpos, e.qvel, e.t
        mujoco.mj_forward(sim.model, d)

    def record(e):
        """한 주기 기록 (sim.runner의 timeseries.parquet와 같은 열 + 실시간 타이밍)."""
        nonlocal prev_contact, touchdowns, edge_touchdowns
        d = sim.data
        contact = np.array([bool(e.contact_bits >> i & 1) for i in range(4)])   # 코어의 스텝 직후 값 (sim.runner와 같음)
        foot_pos = np.array(e.foot_pos).reshape(4, 3)
        if prev_contact is not None:
            for i in np.where(contact & ~prev_contact)[0]:                    # 착지, 모서리 착지 (참 지형)
                fx, fy = foot_pos[i][:2]
                hs = [sim.terrain.height_at(fx + ox, fy + oy) for ox in (-0.04, 0, 0.04) for oy in (-0.04, 0, 0.04)]
                touchdowns += 1
                edge_touchdowns += int(max(hs) - min(hs) > 0.02)
        prev_contact = contact
        v_true = np.array(e.base_xmat).reshape(3, 3).T @ d.qvel[:3]
        true_by_t[round(e.t, 6)] = (v_true, quat_to_yaw(d.qpos[3:7]))
        est = None                                                            # 두뇌의 상태 추정 (같은 상태 시각의 참값과 비교)
        if br.name == "inproc" and br.node.est is not None:
            est = {"t": br.node.last_t, "v_body": br.node.est.v_body, "yaw": br.node.est.yaw, "contacts": br.node.est.contacts}
        elif br.name == "ros":
            est = br.bridge.latest_est
        elif br.name == "shm":
            est = link.view.read_est()
        ref = true_by_t.get(round(est["t"], 6)) if est else None
        if ref is not None:
            est_err_v.append(est["v_body"][0] - ref[0][0])
            rec.log(est_v_body=est["v_body"], est_yaw=est["yaw"], est_contacts=np.asarray(est["contacts"], float), true_v_body=ref[0])
        else:
            rec.log(est_v_body=np.full(3, np.nan), est_yaw=np.nan, est_contacts=np.full(4, np.nan), true_v_body=np.full(3, np.nan))
        while len(true_by_t) > 10:
            true_by_t.pop(next(iter(true_by_t)))
        cmd = (br.node.clock.cmd_f if br.name == "inproc" else            # sim.runner: 가속 제한 뒤 명령 (두뇌가 넘겨준 값)
               np.array(est["cmd"]) if est and "cmd" in est else np.array(sim.command))
        rec.log(t=e.t, base_pos=d.qpos[:3], base_quat_wxyz=d.qpos[3:7], base_linvel=d.qvel[:3], base_angvel_body=d.qvel[3:6],
                q=d.qpos[7:], qd=d.qvel[6:], tau=np.array(e.ctrl), contact=contact.astype(float), cmd=cmd,
                q_des=np.array(e.q_des), terrain_version=sim.terrain.version,
                wake_us=e.wake_us, compute_us=e.compute_us, cmd_state_t=e.cmd_t)
        if sim.cosim is not None and sim.cosim.has_vehicle:
            rec.log(vehicle_pos=sim.cosim.vehicle["pos"], vehicle_speed=sim.cosim.vehicle["speed"])

    scan_log = {}
    odom_offset = [None]
    viz_count = [0]

    sensors = [None]                           # 센서 작업 프로세스 (sim/rt_sensor.py)
    lag_max = [0]                              # 실시간: 관리 프로세스가 코어보다 뒤처진 최대 스텝 수

    def sense(e):
        """센서 (sim.runner Simulation.sense와 같은 시각·값): 스텝 직후 바디 위치를 렌더 스트림으로 작업 프로세스에 보낸다.
        lockstep이면 결과를 기다려 바로 두뇌로 보낸다 (sim.runner와 같은 스텝). 실시간이면 끝나는 대로 deliver_scans가 보낸다."""
        if sensors[0] is None:
            return
        d, nb = sim.data, min(sim.model.nbody, R.NBODY_MAX)
        d.xpos[:nb] = np.array(e.body_xpos[:3 * nb]).reshape(nb, 3)
        d.xquat[:nb] = np.array(e.body_xquat[:4 * nb]).reshape(nb, 4)
        d.time = e.t
        f = faults.get("lidar_blackout")                           # 센서 고장 (보낸 때의 상태로 거른다)
        sensors[0].submit(e.t, None if f is None else ("blackout", f[1].get("sensor")))
        if lockstep:
            deliver_scans(wait=True)

    scan_delay = []                            # 실시간: 스캔이 두뇌에 간 때의 코어 시각 - 스캔 시각 (s)

    def deliver_scans(wait=False):
        for tag, scans in sensors[0].results(wait):
            for name, scan in scans:
                scan_delay.append(int(link.shm.stats.steps) * link.control_dt - scan["t"])
                if tag is not None and tag[1] in (None, name):         # 센서 고장: 점군이 나오지 않는다
                    continue
                if out_dir is not None:
                    scan_log.setdefault(name, []).append(scan)
                br.on_scan(name, scan)
                if bridge is not None and br.name != "ros":
                    bridge.publish_scan(name, scan)

    def publish_viz(e, final_mop=None, mirrored=False):
        """렌더 스트림 발행 (sim.runner와 같은 토픽). 지형 인지면 알고리즘 지형 지도와 디딜 곳도 (관리 안 두뇌)."""
        if not mirrored:
            mirror(e)
        bridge.publish(sim.data)
        viz_count[0] += 1
        if sim.controller.terrain is not None:
            est = (br.node.est.pos if br.name == "inproc" and br.node.est is not None else
                   (br.bridge.latest_est or {}).get("pos") if br.name == "ros" else
                   (link.view.read_est() or {}).get("pos"))
            if est is not None:                                   # world -> odom을 로봇에 맞춘다 (sim.runner와 같은 표시 정렬)
                odom_offset[0] += 0.1 * (sim.data.qpos[:3] - np.asarray(est) - odom_offset[0])
                bridge.publish_odom_frame(odom_offset[0], e.t)
            if br.name == "inproc" and viz_count[0] % 2 == 0 and (snap := br.node.perception_snapshot()):
                bridge.publish_perception(snap)
        bridge.publish_status(build_status(sim, start_xy, energy, int(link.shm.stats.overruns), final_mop=final_mop))

    def drain_truth(max_n=None):
        """참값 링 처리. max_n: 한 번에 처리할 최대 개수 (실시간에서 같은 프로세스의 두뇌가 밀리지 않게)."""
        nonlocal k_truth, energy, fell_at, qpos, next_viz
        d = sim.data
        stop_at = k_truth + max_n if max_n else None
        while k_truth < link.shm.truth_head and (stop_at is None or k_truth < stop_at):   # 판정자 쪽 참값
            e = link.truth(k_truth)
            k_truth += 1
            if e is None:                                          # 이 프로세스가 너무 늦어 링을 놓침
                continue
            energy += e.energy
            d.qpos[:], d.qvel[:], d.time = e.qpos, e.qvel, e.t           # DIS 서버·정보판이 보는 지금 상태 (참값)
            wake.append(e.wake_us); comp.append(e.compute_us)
            if e.cmd_t >= 0:
                lat.append(e.t - link.control_dt - e.cmd_t)        # 실행한 스텝의 상태 시각 - 명령이 쓴 상태 시각
            qpos = np.array(e.qpos)
            roll, pitch = quat_to_roll_pitch(qpos[3:7])
            if fell_at is None and (qpos[2] - sim.terrain.height_at(*qpos[:2]) < FALL_HEIGHT or max(abs(roll), abs(pitch)) > FALL_TILT):
                fell_at = e.t
                if verbose:
                    print(f"[t={e.t:6.2f}] FALL detected")
                link.stop()
            if rec is not None:                                      # 기록은 다음 스텝의 이벤트·지형 전에 (sim.runner와 같은 순서)
                mirror(e)
                record(e)
            if sim.cosim is not None and sim.cosim.feet_enabled:     # 발자국: 스텝 동안 평균한 발 수직력 참값 (sim.runner와 같은 값)
                sim.cosim.observe_feet(np.array(e.foot_pos).reshape(4, 3), np.array(e.foot_force))
            sense(e)                                                 # 센서: 물리 뒤, 다음 스텝 이벤트·지형 전 (sim.runner와 같은 순서)
            if stop_after[0] is not None and e.step >= stop_after[0]:  # 콘솔 종료: 그 주기까지 돌았다
                link.stop()
                continue
            if e.step + 1 < link.shm.cfg.max_steps:                 # 마지막 스텝 뒤에는 다음 스텝 준비를 하지 않는다 (sim.runner와 같게)
                apply_events(e.t, e.step + 1)
                cosim_step(e.t, e.qpos[:2])
                terrain_step(e.t, e.qpos[:2], [np.array(e.foot_pos[3 * i:3 * i + 2]) for i in range(4)], e.step + 1)
            if dis_srv is not None:
                dis_srv.after_events()
            if bridge is not None and (e.t >= next_viz - 1e-9 or fell_at is not None):
                next_viz = e.t + VIZ_DT
                publish_viz(e, mirrored=rec is not None)
            if displays and any(p.poll() is not None for _, p in displays):   # 창을 닫으면 끝
                if verbose:
                    print(f"[t={e.t:6.2f}] 창이 닫혀 끝낸다")
                link.stop()

    helpers, dis_srv = [], None
    try:
        log_dir = out_dir or ROOT / "build/rt"
        if dis:
            from .dis_server import DisScenarioServer
            dis_srv = DisScenarioServer(sim, port=dis["port"], host=dis.get("host", "127.0.0.1"),
                                        log_path=log_dir / "dis_events.jsonl" if out_dir else None,
                                        comm_lost_behavior=dis.get("comm_lost", "STOP"), key=dis.get("key"),
                                        pdu_dir=out_dir)
            if verbose:
                print(f"DIS 시나리오 콘솔 대기: UDP {dis.get('host', '127.0.0.1')}:{dis_srv.port}{' (인증)' if dis.get('key') else ''}")
        if viz or brain == "ros":
            from .ros2_bridge import Ros2StreamPublisher, launch_mujoco_viewer, launch_rviz_stack
            bridge = Ros2StreamPublisher(sim.model, sim.terrain, sim.cosim)
        if "rviz" in viz:
            rviz, adapter = launch_rviz_stack(log_dir)
            displays.append(("RViz", rviz)); helpers.append(adapter)
        if "mjviz" in viz:
            displays.append(("MuJoCo viewer", launch_mujoco_viewer(log_dir)))
        if displays:
            if verbose:
                print(f"{', '.join(n for n, _ in displays)} 시작 대기 중...")
            bridge.wait_for_subscriber(count=len(displays), alive=lambda: all(p.poll() is None for _, p in displays))
        if brain == "inproc":
            br = InprocBrain(sim, link)
        elif brain == "shm":
            br = ShmBrain(sim, link, scn_path, policy, log_dir)
        else:
            br = RosBrain(sim, link, scn_path, policy, remote, log_dir, bridge, viz=bool(viz))
        if sim.sensor_renderer is not None:
            from .rt_sensor import SensorWorker
            sensors[0] = SensorWorker(sim, workers=1 if lockstep else SENSOR_WORKERS)
        if bridge is not None and sim.controller.terrain is not None:     # 알고리즘 지형 지도 표시 정렬: 시작 몸통 위치
            odom_offset[0] = sim.data.qpos[:3].copy()
            bridge.publish_odom_frame(odom_offset[0], 0.0)
        t0 = next_note = time.monotonic()
        while not br.ready():                                      # 두뇌가 붙을 때까지 (다른 PC면 사람이 띄울 때까지)
            if brain == "ros":
                br.bridge.spin_some()
            if remote and time.monotonic() >= next_note:
                next_note += 10
                print(f"두뇌 노드를 기다린다: 다른 PC에서 ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 0)} "
                      f"python -m control.ros_node --scenario <이 시나리오 파일 복사본> (시나리오: {scn_path})")
            if time.monotonic() - t0 > (3600 if remote else 30):
                raise RuntimeError("두뇌가 붙지 않았다")
            time.sleep(0.05)
        if dis_srv is not None and dis.get("wait"):
            if verbose:
                print("콘솔 접속을 기다린다 (Ctrl+C로 중단)...")
            while not dis_srv.consoles:
                dis_srv.poll(); dis_srv.after_events()
                time.sleep(0.05)
            if verbose:
                print("콘솔 접속. 시작")
        if dis_srv is not None:
            dis_srv.poll()
        apply_events(0.0, 0)
        if dis_srv is not None:
            dis_srv.after_events()
        cosim_step(sim.data.time, sim.data.qpos[:2])
        terrain_step(sim.data.time, sim.data.qpos[:2], sim.feet_xy(), 0)
        link.start()
        n_steps = link.shm.cfg.max_steps
        if lockstep:
            wall0 = time.monotonic()
            for k in range(n_steps):
                if displays:                                       # 화면이 있으면 벽시계보다 앞서지 않게 (sim.runner --rviz와 같이 실시간 속도)
                    ahead = k * link.control_dt - (time.monotonic() - wall0)
                    if ahead > 0:
                        time.sleep(ahead)
                state_sent = link.shm.ctl.link_mode == 0           # 링크 두절이면 코어가 상태를 내지 않는다
                link.shm.ctl.lockstep_target = k
                while link.shm.stats.steps < k + 1:
                    time.sleep(0.00002)
                if state_sent:
                    ls = link.read_state()
                    br.on_state(ls)
                    t_end = time.monotonic() + 30
                    while br.cmd_t() is None or br.cmd_t() < ls["t"] - 1e-9 or not br.est_ready(ls["t"]):   # 두뇌가 이 상태로 명령을 낼 때까지
                        br.poll()
                        if time.monotonic() > t_end:
                            raise RuntimeError(f"두뇌가 응답하지 않는다 (t={ls['t']:.2f})")
                        time.sleep(0.00002)
                drain_truth()
                link.fill_noise(R.NOISE_RING // 2)
                if fell_at is not None:
                    break
        else:
            while not link.shm.stats.done:
                if dis_srv is not None:
                    dis_srv.poll()
                    dis_srv.after_events()                        # 일시정지 중에도 재개 요청과 주기 보고
                if chrono_hold["on"]:                              # Chrono 따라잡기 (코어는 멈춰 있다)
                    sim.cosim.sync(sim.data.time, block=False)
                    if sim.cosim.lag(sim.data.time) <= chrono_max_lag / 2:
                        chrono_hold["on"] = False
                        chrono_hold["wall"] += time.monotonic() - chrono_hold["since"]
                link.shm.ctl.freeze = int(sim.frozen or chrono_hold["on"])
                if sensors[0] is not None:
                    deliver_scans()
                ls = link.read_state()
                if ls is not None and ls["step"] != last_step:
                    last_step = ls["step"]
                    br.on_state(ls)
                br.poll()
                lag_max[0] = max(lag_max[0], int(link.shm.truth_head) - k_truth)
                drain_truth(max_n=10)
                link.fill_noise(R.NOISE_RING // 2)
                time.sleep(0.0005)
            drain_truth()
        st = link.shm.stats
        dist = float(np.linalg.norm(qpos[:2] - start_xy))
        wake, comp = np.array(wake), np.array(comp)
        lat_ms = 1000 * np.array(lat)
        t_end = int(st.steps) * link.control_dt
        mop = {
            "mode": "lockstep" if lockstep else "realtime", "brain": br.name, "steps": int(st.steps),
            "sim_time_s": round(t_end, 3), "fell": fell_at is not None, "fell_at_s": fell_at,
            "distance_m": round(dist, 3),
            "forward_x_m": round(float(qpos[0] - start_xy[0]), 3), "lateral_drift_m": round(float(qpos[1] - start_xy[1]), 3),
            "mean_speed_mps": round(dist / max(t_end - 1.0, 1e-6), 3),
            "cost_of_transport": round(energy / (sim.total_mass * 9.81 * max(dist, 1e-6)), 3),
            "cmd_latency_ms_mean": round(float(lat_ms.mean()), 1) if lat else None,
            "cmd_latency_ms_max": round(float(lat_ms.max()), 1) if lat else None,
            "cmd_late_steps": int((lat_ms > 1000 * link.control_dt + 1e-3).sum()),   # 명령이 한 주기보다 늦게 실행된 주기 수
            "noise_underruns": int(st.noise_underruns), "mlockall": bool(st.locked), "sched_fifo": bool(st.rt_ok),
        }
        mop["min_vehicle_distance_m"] = round(sim.cosim.min_dist, 3) if sim.cosim is not None and np.isfinite(sim.cosim.min_dist) else None   # 차가 한 번도 안 나왔으면 None
        mop.update(terrain_deformation(terrain0, sim.terrain.applied))
        if rec is not None:
            mop.update({"touchdowns": touchdowns, "edge_touchdowns": edge_touchdowns,
                        "est_speed_rmse_mps": round(float(np.sqrt(np.mean(np.square(est_err_v[25:])))), 4)
                        if len(est_err_v) > 25 else None})
        if not lockstep:
            mop["supervisor_lag_steps_max"] = lag_max[0]
            if sim.cosim is not None:
                mop["chrono_lag_s_max"] = round(chrono_lag[0], 3)
                mop["chrono_lag_s_end"] = round(sim.cosim.lag(t_end), 3)
                mop["chrono_holds"] = chrono_hold["count"]                  # Chrono를 기다리느라 세계를 멈춘 횟수, 시간 (벽시계 s)
                mop["chrono_hold_s"] = round(chrono_hold["wall"], 2)
            if scan_delay:
                mop["scan_delay_ms_mean"] = round(1000 * float(np.mean(scan_delay)), 1)
                mop["scan_delay_ms_max"] = round(1000 * float(np.max(scan_delay)), 1)
                mop["scan_dropped"] = sensors[0].dropped
            mop.update({
                "wake_us_mean": round(float(wake.mean()), 1), "wake_us_p99": round(float(np.percentile(wake, 99)), 1),
                "wake_us_max": round(float(wake.max()), 1), "compute_us_mean": round(float(comp.mean()), 1),
                "compute_us_p99": round(float(np.percentile(comp, 99)), 1), "compute_us_max": round(float(comp.max()), 1),
                "overruns": int(st.overruns)})
        if dis_srv is not None:
            mop["stopped_by_console"] = sim.stop_requested
        if rec is not None:
            rec.meta["brain"] = br.name
            rec.save(mop)
            for name, scans in scan_log.items():                    # LiDAR 원시 출력 (sim.runner와 같은 형식)
                np.savez_compressed(out_dir / f"{name}.npz", t=np.array([s["t"] for s in scans]),
                                    frame=np.concatenate([np.full(len(s["points"]), i, np.int32) for i, s in enumerate(scans)]),
                                    points=np.concatenate([s["points"] for s in scans]),
                                    ring=np.concatenate([s["ring"] for s in scans]),
                                    label=np.concatenate([s["label"] for s in scans]),
                                    R=np.array([s["R"] for s in scans]), p=np.array([s["p"] for s in scans]))
        if dis_srv is not None:
            dis_srv.announce_end({"reason": "stopped" if sim.stop_requested else "fell" if fell_at is not None else "duration",
                                  "run": str(out_dir.relative_to(ROOT)) if out_dir else "", "sim_time_s": mop["sim_time_s"],
                                  "forward_x_m": mop["forward_x_m"], "fell": mop["fell"]})
            if out_dir is not None:                                    # 콘솔 이벤트를 적용 시각으로 넣은 시나리오
                (out_dir / "scenario_replay.yaml").write_text(yaml.safe_dump(sim.replay_scenario(), allow_unicode=True,
                                                                             sort_keys=False))
        if displays and k_truth > 0:                                # 마지막 상태와 최종 MOP를 화면에 남기고 창이 닫힐 때까지
            publish_viz(link.truth(k_truth - 1) or link.shm.truth[(k_truth - 1) % R.TRUTH_RING], final_mop=mop)
            if any(p.poll() is None for _, p in displays):
                print("창을 닫으면 종료합니다 (Ctrl+C도 가능).")
                try:
                    while all(p.poll() is None for _, p in displays):
                        time.sleep(0.2)
                except KeyboardInterrupt:
                    pass
        return mop, qpos
    finally:
        link.close()
        if sensors[0] is not None:
            sensors[0].close()
        if dis_srv is not None:
            dis_srv.close()
        if br is not None:
            br.close()
        for p in [p for _, p in displays] + helpers:
            if p.poll() is None:
                p.terminate()
        if bridge is not None:
            bridge.close()
        stats = sim.close()
        if stats and verbose:
            print(f"Chrono 서버 계산 시간: {stats['busy_s']:.1f} s")
        scn_path.unlink(missing_ok=True)


def main():
    ap = argparse.ArgumentParser(description="C++ 실시간 코어로 시나리오 실행")
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--build", action="store_true", help="코어 빌드 (cmake)")
    ap.add_argument("--lockstep", action="store_true", help="실시간 대신 한 스텝씩 (Python 실행과 비교용)")
    ap.add_argument("--brain", choices=["inproc", "shm", "ros"], default="inproc",
                    help="두뇌(보행 알고리즘) 연결: inproc 이 프로세스, shm 같은 PC 별도 프로세스, ros ROS2 노드")
    ap.add_argument("--remote-brain", action="store_true", help="--brain ros: 노드를 띄우지 않고 다른 PC의 노드를 기다린다")
    ap.add_argument("--policy", help="정책 카드 (없으면 시나리오의 트롯)")
    ap.add_argument("--variant", choices=["cpu", "mjx"], default="cpu", help="모델 설정 (sim.runner --variant와 같다)")
    ap.add_argument("--dis-port", type=int, metavar="PORT", help="DIS 시나리오 콘솔 요청을 받는다 (UDP, 실시간 전용)")
    ap.add_argument("--dis-wait", action="store_true", help="콘솔이 접속할 때까지 시작하지 않는다")
    ap.add_argument("--dis-key", metavar="FILE", help="DIS 인증 공유 키")
    ap.add_argument("--dis-host", default="127.0.0.1", help="콘솔 요청을 받을 주소 (127.0.0.1 밖은 --dis-key 필요)")
    ap.add_argument("--dis-comm-lost", choices=["STOP", "CONTINUE"], default="STOP")
    ap.add_argument("--chrono-max-lag", type=float, default=CHRONO_MAX_LAG, metavar="S",
                    help=f"실시간 Chrono가 이보다 뒤처지면 세계를 멈춰 기다린다 (기본 {CHRONO_MAX_LAG}, 0: 상한 없음)")
    ap.add_argument("--rviz", action="store_true", help="RViz로 보기 (sim.runner --rviz와 같은 화면)")
    ap.add_argument("--mjviz", action="store_true", help="MuJoCo 렌더러로 보기")
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
        scn = apply_overrides(load_yaml(a.scenario), a.set)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        out = ROOT / "runs" / f"{scn['name']}_rt_{stamp}"          # 기록기가 만든다 (지원하지 않는 시나리오면 만들지 않음)
        viz = tuple(v for v in ("rviz", "mjviz") if getattr(a, v))
        dis = None
        if a.dis_port is not None:
            if a.lockstep:
                raise SystemExit("DIS 콘솔은 실시간 전용이다 (--lockstep을 빼고 실행)")
            if a.dis_host not in ("127.0.0.1", "localhost") and not a.dis_key:
                raise SystemExit("이 PC 밖에서 콘솔 요청을 받으려면 인증 키가 필요하다 (--dis-key)")
            from dis_console.auth import load_key
            dis = {"port": a.dis_port, "host": a.dis_host, "wait": a.dis_wait, "comm_lost": a.dis_comm_lost,
                   "key": load_key(a.dis_key) if a.dis_key else None}
        mop, _ = run(scn, a.lockstep, a.cpu, a.priority, brain=a.brain, policy=a.policy, remote=a.remote_brain,
                     viz=viz, out_dir=out, variant=a.variant, overrides=a.set or (), dis=dis, chrono_max_lag=a.chrono_max_lag)
        print("\n== MOP (C++ 실시간 코어) ==")
        for k, v in mop.items():
            print(f"  {k:20s} {v}")
        print(f"\n기록: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
