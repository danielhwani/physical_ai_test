"""시나리오 실행기.

    python -m sim.runner scenarios/flat_trot.yaml              # lockstep(시뮬레이션 시간), 최대 속도
    python -m sim.runner scenarios/flat_trot.yaml --realtime   # 실시간 페이싱
    python -m sim.runner scenarios/flat_trot.yaml --view       # MuJoCo 뷰어 (저사양: 느릴 수 있음)
    python -m sim.runner scenarios/flat_trot.yaml --ros2       # ROS2 포즈 스트림 발행
    python -m sim.runner scenarios/flat_trot.yaml --rviz       # RViz로 가시화 (ROS2 + 실시간 자동)
    python -m sim.runner scenarios/flat_trot.yaml --mjviz      # MuJoCo 렌더러 어댑터로 가시화 (중립 스트림 사용)
    python -m sim.runner scenarios/flat_trot.yaml --policy policies/go2_trot_bc/card.yaml   # ONNX 정책

운용 모드는 문서 §9.2의 두 가지: lockstep(순수 가상)과 실시간(LVC 대비).
"""
import argparse
import copy
import datetime as dt
import os
import platform
import threading
import time
from collections import deque
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import yaml

from .adapters import quat_to_roll_pitch, quat_to_yaw
from .control_interface import ControlInterface, load_card
from .controllers.common import GaitClock, StepContext
from .controllers.onnx_policy import OnnxPolicyController
from .controllers.trot import TrotController
from .model_builder import ROOT, build_model
from .recorder import Recorder
from .stream import build_status
from .terrain_service import TerrainMapService

SIM_VERSION = "0.3.0"
TERRAIN_COMMIT_HZ = 5.0       # 지형 갱신은 수 Hz로 충분 (문서 §5.3)
FALL_HEIGHT = 0.12            # 몸통 높이(지면 기준)가 이보다 낮으면 넘어짐
FALL_TILT = 1.0               # rad


KEY_SPACE, KEY_RIGHT = 32, 262      # GLFW 키 코드


class PauseControl:
    """뷰어 키 입력: Space = 일시정지/재개, → = 일시정지 중 제어 주기 1회 진행.
    passive 뷰어의 Pause 버튼은 러너가 물리를 직접 돌리므로 동작하지 않아 여기서 처리한다."""

    def __init__(self, paused=False):
        self.paused = paused
        self.step_requests = 0

    def key_callback(self, key):          # 뷰어 스레드에서 호출됨
        if key == KEY_SPACE:
            self.paused = not self.paused
        elif key == KEY_RIGHT and self.paused:
            self.step_requests += 1


STATUS_DT = 0.1      # 상태 스트림 발행 주기 (시뮬레이션 시간 s)


def terrain_deformation(before, after, threshold=0.005):
    """실행 중 지면이 파인 정도 (Chrono 바퀴 자국, 발자국, 시나리오 이벤트). threshold 이상 내려간 셀 기준."""
    dz = after - before
    dug = dz < -threshold
    return {"deformed_cells": int(dug.sum()),
            "deform_mean_m": round(float(dz[dug].mean()), 4) if dug.any() else 0.0,
            "deform_max_m": round(float(dz.min()), 4) if dug.any() else 0.0}


def parse_value(text):
    """--set 값: 숫자(5e6처럼 YAML이 문자열로 읽는 표기 포함) > YAML(true, [1, 2] 등) > 문자열."""
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return yaml.safe_load(text)


def apply_overrides(scn, sets):
    """시나리오 dict에 'a.b.c=값' 목록을 적용한다. 없는 키는 오류 (오타로 조용히 무시되는 것을 막는다)."""
    scn = copy.deepcopy(scn)
    for item in sets or []:
        key, _, text = item.partition("=")
        *path, leaf = key.strip().split(".")
        node = scn
        for k in path:
            if k not in node:
                raise KeyError(f"--set {key}: '{k}' 없음 (있는 키: {list(node)})")
            node = node[k]
        if leaf not in node:
            raise KeyError(f"--set {key}: '{leaf}' 없음 (있는 키: {list(node)})")
        node[leaf] = parse_value(text.strip())
    return scn


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


class Simulation:
    """scenario: YAML 경로 또는 dict. policy: 정책 카드 경로 (없으면 시나리오의 규칙 기반 보행기)."""

    def __init__(self, scenario, spec_path=ROOT / "specs/go2_control.yaml", variant="cpu", seed=None,
                 policy=None):
        self.scn = copy.deepcopy(scenario) if isinstance(scenario, dict) else load_yaml(scenario)
        if seed is not None:
            self.scn["seed"] = seed
        self.spec = load_yaml(spec_path)
        t = self.scn["terrain"]
        self.terrain = TerrainMapService(t["size"], t["resolution"], t["z_range"], self.scn.get("seed", 0))
        # 발 근처 셀 갱신 보류 반경 (m). 로봇이 자기 발자국을 밟게 하려면 보폭보다 작게 (문서 §5.3)
        self.guard_radius = t.get("guard_radius", 0.15)
        for p in t.get("patches", []):
            self.terrain.add_patch(p)
        self.variant = variant
        self.model, self.hfield_id = build_model(self.spec, self.terrain, variant)
        self.data = mujoco.MjData(self.model)

        # 제어 인터페이스: 정책 카드가 있으면 카드 규약, 없으면 로봇 명세 규약
        if policy:
            card = load_card(policy)
            self.iface = ControlInterface(self.spec, card)
            self.ctrl = OnnxPolicyController(card, self.iface)
            self.controller_desc = f"onnx:{card['name']}"
            period = self.iface.gait_period
        else:
            self.iface = ControlInterface(self.spec)
            self.ctrl = TrotController(self.spec, self.scn["controller"], self.iface)
            self.controller_desc = f"trot:{self.scn['controller']}"
            period = self.scn["controller"]["period"]
        self.clock = GaitClock(period or 1.0)
        self.decim = int(round(self.iface.control_dt / self.model.opt.timestep))
        self.delay_steps = self.spec["action"].get("delay_steps", 0)
        self.on_action = None       # (ctx, action) -> action. 모방학습(DAgger)에서 실행 action 교체용

        m = self.model
        self.base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, self.spec["robot"]["base_body"])
        self.foot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f) for f in self.spec["robot"]["feet"]]
        self.terrain_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
        self.total_mass = m.body_subtreemass[self.base_id]
        self.reset()
        self.cosim = None
        if "chrono" in self.scn:                 # 두 번째 물리엔진: 차량 + 변형 지면 (문서 §5)
            from .chrono_link import ChronoLink
            ccfg = self.scn["chrono"]
            if ccfg.get("robot_feet"):           # 발 대리 구 반지름 = 실제 발 geom 반지름 (모델 변형에 따라 자동)
                ccfg["robot_feet"] = dict(ccfg["robot_feet"]) if isinstance(ccfg["robot_feet"], dict) else {}
                ccfg["robot_feet"].setdefault("radius", float(self.model.geom_size[self.foot_ids[0]][0]))
                ccfg["robot_feet"].setdefault("names", list(self.spec["robot"]["feet"]))
            self.cosim = ChronoLink(ccfg, self.terrain)

    def close(self):
        """외부 물리엔진 프로세스 정리. Chrono 통계(서버 계산 시간)를 돌려준다."""
        if self.cosim is not None:
            stats, self.cosim = self.cosim.close(), None
            return stats
        return None

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.data.qpos[2] += self.terrain.height_at(0.0, 0.0)
        mujoco.mj_forward(self.model, self.data)
        self.ctrl.reset()
        self.clock.reset()
        self.q_des = self.iface.targets_from_action(np.zeros(12))
        self.action = np.zeros(12)
        self.pending = deque([np.zeros(12)] * self.delay_steps)
        self.obs = np.zeros(self.iface.obs.dim)
        self.events = sorted(self.scn.get("events", []), key=lambda e: e["t"])
        self.last_event = None      # (시각, 설명) 가시화용
        self.next_terrain_commit = 0.0

    # ---- 이벤트 (나중에 DIS 시나리오 콘솔이 같은 경로로 주입) ----
    def _apply_events(self, verbose=True):
        while self.events and self.events[0]["t"] <= self.data.time + 1e-9:
            e = self.events.pop(0)
            if e["action"] == "set_command":
                self.clock.set_command(e["vx"], e.get("yaw_rate", 0.0))
            elif e["action"] == "add_patch":
                self.terrain.add_patch(e["patch"])
            desc = e["action"]
            if e["action"] == "set_command":
                desc += f" vx={e['vx']:.2f} yaw={e.get('yaw_rate', 0.0):.2f}"
            elif e["action"] == "add_patch":
                desc += f" {e['patch']['kind']}"
            self.last_event = (self.data.time, desc)
            if verbose:
                print(f"[t={self.data.time:6.2f}] event: {e['action']}")

    def feet_xy(self):
        return [self.data.geom_xpos[g][:2].copy() for g in self.foot_ids]

    def foot_normal_forces(self):
        """발마다 지형과의 접촉 수직력 합 (N). Chrono 발자국 계산의 입력."""
        f, out = np.zeros(4), np.zeros(6)
        for k in range(self.data.ncon):
            con = self.data.contact[k]
            for i, g in enumerate(self.foot_ids):
                if g in (con.geom1, con.geom2) and self.terrain_geom in (con.geom1, con.geom2):
                    mujoco.mj_contactForce(self.model, self.data, k, out)
                    f[i] += out[0]               # 접촉 좌표계 첫 성분 = 법선 방향
        return f

    def foot_contacts(self):
        c = np.zeros(4, dtype=bool)
        for k in range(self.data.ncon):
            con = self.data.contact[k]
            for i, g in enumerate(self.foot_ids):
                if g in (con.geom1, con.geom2):
                    c[i] = True
        return c

    def obs_inputs(self):
        """관측 계산 입력. 새 관측 항목이 다른 입력(지형 높이 등)을 쓰면 여기에 추가."""
        d = self.data
        return dict(qpos=d.qpos, qvel=d.qvel, command=self.clock.command_body(),
                    last_action=self.action, gait_phase=self.clock.phase)

    def step_control(self, verbose=True):
        """제어 주기 1회 = 물리 스텝 decim회."""
        d = self.data
        dt = self.decim * self.model.opt.timestep
        self._apply_events(verbose)
        if self.cosim is not None:
            self.cosim.sync(d.time)              # Chrono 결과 반영 (지형 원천 갱신 + 차량 포즈)
            dist = self.cosim.distance_to(d.qpos[:2])
            if dist < self.cosim.near_dist and not self.cosim.near_event_sent:
                self.cosim.near_event_sent = True
                self.last_event = (d.time, f"vehicle_near {dist:.1f}m")
                if verbose:
                    print(f"[t={d.time:6.2f}] event: vehicle_near ({dist:.2f} m)")
        terrain_changed = False
        if d.time >= self.next_terrain_commit:
            terrain_changed = self.terrain.commit(self.model, self.hfield_id, self.feet_xy(), self.guard_radius)
            self.next_terrain_commit += 1.0 / TERRAIN_COMMIT_HZ

        # 1) 명령/위상 갱신  2) 관측  3) 컨트롤러 -> action  4) 행동 처리 -> q_des  5) PD + 물리
        self.clock.update(dt)
        self.obs = self.iface.obs.compute(**self.obs_inputs())
        roll, pitch = quat_to_roll_pitch(d.qpos[3:7])
        R = d.xmat[self.base_id].reshape(3, 3)
        ctx = StepContext(dt=dt, qpos=d.qpos, qvel=d.qvel, roll=roll, pitch=pitch, yaw=quat_to_yaw(d.qpos[3:7]),
                          v_body_x=(R.T @ d.qvel[:3])[0], wz_world=(R @ d.qvel[3:6])[2],
                          clock=self.clock, obs=self.obs)
        action = self.ctrl.act(ctx)
        if self.on_action:
            action = self.on_action(ctx, action)
        self.action = np.clip(action, -self.iface.clip, self.iface.clip)
        if self.delay_steps:
            self.pending.append(self.action)
            applied = self.pending.popleft()
        else:
            applied = self.action
        self.q_des = self.iface.targets_from_action(applied)

        kp, kd, lim = self.iface.kp, self.iface.kd, self.iface.torque_limit
        energy = 0.0
        feet_coupled = self.cosim is not None and self.cosim.feet_enabled
        fn_sum = np.zeros(4)
        for _ in range(self.decim):
            tau = kp * (self.q_des - d.qpos[7:]) - kd * d.qvel[6:]
            d.ctrl[:] = np.clip(tau, -lim, lim)
            mujoco.mj_step(self.model, d)
            energy += np.abs(d.ctrl * d.qvel[6:]).sum() * self.model.opt.timestep
            if feet_coupled:
                fn_sum += self.foot_normal_forces()
        if feet_coupled:
            # 물리 스텝마다 평균한 수직력 (착지 순간의 짧은 접촉력 스파이크는 실제 충격량만큼만 반영된다)
            self.cosim.observe_feet(d.geom_xpos[self.foot_ids], fn_sum / self.decim)
        return terrain_changed, energy, roll, pitch

    def fallen(self, roll, pitch):
        bx, by, bz = self.data.qpos[:3]
        return bz - self.terrain.height_at(bx, by) < FALL_HEIGHT or max(abs(roll), abs(pitch)) > FALL_TILT


def run(args):
    scenario = args.scenario if isinstance(args.scenario, dict) else load_yaml(args.scenario)
    scenario = apply_overrides(scenario, getattr(args, "set", None))
    sim = Simulation(scenario, variant=args.variant, policy=args.policy)
    scn, d, m = sim.scn, sim.data, sim.model
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = ROOT / "runs" / f"{scn['name']}_{stamp}"
    k = 2
    while out.exists():                       # 같은 초에 여러 번 실행해도 (스윕 등) 덮어쓰지 않게
        out, k = ROOT / "runs" / f"{scn['name']}_{stamp}_{k}", k + 1
    meta = {
        "scenario": scn["name"], "seed": scn.get("seed", 0), "source": "Virtual",
        "overrides": list(getattr(args, "set", None) or []),       # --set으로 바꾼 값 (결과 출처 추적)
        "mode": "realtime" if args.realtime else "lockstep",
        "model_variant": args.variant,
        # 버전 조합 기록 (문서 §13.3)
        "versions": {"sim": SIM_VERSION, "spec": sim.spec["spec_version"],
                     "controller": sim.controller_desc, "mujoco": mujoco.__version__,
                     "model": sim.spec["robot"]["menagerie_commit"]},
        "host": platform.node(), "started": stamp,
    }
    rec = Recorder(out, meta)

    viewer = bridge = None
    displays, helpers = [], []   # 창(닫으면 시험 종료) / 보조 프로세스(끝나면 정리)
    pause = PauseControl(paused=args.start_paused)
    if args.view:
        before = set(threading.enumerate())
        viewer = mujoco.viewer.launch_passive(m, d, key_callback=pause.key_callback)
        viewer_threads = set(threading.enumerate()) - before
        viewer.cam.trackbodyid = sim.base_id
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.distance = 2.0
    if args.ros2:
        from .ros2_bridge import Ros2StreamPublisher, launch_mujoco_viewer, launch_rviz_stack
        bridge = Ros2StreamPublisher(m, sim.terrain, sim.cosim)
        if args.rviz:
            rviz, rviz_adapter = launch_rviz_stack(out)
            displays.append(("RViz", rviz))
            helpers.append(rviz_adapter)
        if args.mjviz:
            displays.append(("MuJoCo viewer", launch_mujoco_viewer(out)))
        if displays:
            names = ", ".join(n for n, _ in displays)
            print(f"{names} 시작 대기 중...")
            # 창마다 /tf 구독자 하나 (RViz, MuJoCo 렌더러 어댑터)
            if not bridge.wait_for_subscriber(count=len(displays),
                                              alive=lambda: all(p.poll() is None for _, p in displays)):
                print(f"창이 뜨지 않았다. 로그: {out.relative_to(ROOT)}/*.log")
    if args.rt:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))

    ctrl_dt = sim.decim * m.opt.timestep
    energy_total, fell_at = 0.0, None
    start_xy = d.qpos[:2].copy()
    terrain0 = sim.terrain.applied.copy()     # 지면 변형량 MOP 기준
    wall0 = run_start = time.perf_counter()
    paused_total, pause_start = 0.0, None
    late, max_lag = 0, 0.0
    next_view = 0.0
    next_status = 0.0
    was_paused = False

    interrupted = False
    try:
        while d.time < scn["duration"]:
            if viewer:
                if pause.paused and pause.step_requests == 0:
                    if not was_paused:
                        viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "PAUSED",
                                          f"t = {d.time:.2f} s   [Space] resume  [Right] step"))   # 오버레이 폰트는 ASCII만 지원
                        print(f"[t={d.time:6.2f}] paused")
                        was_paused, pause_start = True, time.perf_counter()
                    viewer.sync()               # 일시정지 중에도 카메라 조작은 가능
                    time.sleep(1 / 30)
                    if not viewer.is_running():
                        break
                    continue
                if was_paused and not pause.paused:
                    viewer.clear_texts()
                    wall0 = time.perf_counter() - d.time    # 실시간 기준점 재설정 (정지 시간은 지연으로 세지 않음)
                    paused_total += time.perf_counter() - pause_start
                    print(f"[t={d.time:6.2f}] resumed")
                    was_paused = False
                if pause.step_requests:
                    pause.step_requests -= 1

            terrain_changed, energy, roll, pitch = sim.step_control()
            energy_total += energy

            rec.log(t=d.time, base_pos=d.qpos[:3], base_quat_wxyz=d.qpos[3:7], base_linvel=d.qvel[:3],
                    base_angvel_body=d.qvel[3:6], q=d.qpos[7:], qd=d.qvel[6:], tau=d.ctrl, q_des=sim.q_des,
                    contact=sim.foot_contacts().astype(float), cmd=sim.clock.cmd_f, gait_phase=sim.clock.phase, obs=sim.obs, action=sim.action,
                    terrain_version=sim.terrain.version)
            if sim.cosim is not None and sim.cosim.has_vehicle:
                rec.log(vehicle_pos=sim.cosim.vehicle["pos"], vehicle_speed=sim.cosim.vehicle["speed"])

            if bridge:
                bridge.publish(d)
                if d.time >= next_status:
                    bridge.publish_status(build_status(sim, start_xy, energy_total, late))
                    next_status = d.time + STATUS_DT
            closed = [n for n, p in displays if p.poll() is not None]
            if closed:                                  # 창을 닫으면 시험 종료
                print(f"[t={d.time:6.2f}] {closed[0]} closed")
                break
            if viewer:
                if not viewer.is_running():
                    break
                if terrain_changed:
                    viewer.update_hfield(sim.hfield_id)
                if was_paused:                   # 1스텝 진행: 바로 그리고 시각 표시 갱신
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "PAUSED",
                                      f"t = {d.time:.2f} s   [Space] resume  [Right] step"))   # 오버레이 폰트는 ASCII만 지원
                    viewer.sync()
                elif d.time >= next_view:        # 저사양: 화면 갱신은 30 Hz로 제한
                    viewer.sync()
                    next_view = d.time + 1 / 30
            if (args.realtime or viewer) and not was_paused:
                lag = (time.perf_counter() - wall0) - d.time
                if lag < 0:
                    time.sleep(-lag)
                elif lag > ctrl_dt:
                    late += 1
                    max_lag = max(max_lag, lag)

            if sim.fallen(roll, pitch):
                fell_at = d.time
                print(f"[t={d.time:6.2f}] FALL detected")
                break

    except KeyboardInterrupt:          # Ctrl+C: 지금까지의 기록과 MOP는 저장하고 정리
        interrupted = True
        print(f"\n[t={d.time:6.2f}] interrupted")

    if was_paused:
        paused_total += time.perf_counter() - pause_start
    wall = time.perf_counter() - run_start - paused_total
    dist = float(np.linalg.norm(d.qpos[:2] - start_xy))
    mop = {
        "sim_time_s": round(d.time, 3),
        "wall_time_s": round(wall, 3),
        "realtime_factor": round(d.time / wall, 2),
        "fell": fell_at is not None, "fell_at_s": fell_at, "interrupted": interrupted,
        "distance_m": round(dist, 3),
        "forward_x_m": round(float(d.qpos[0] - start_xy[0]), 3),
        "lateral_drift_m": round(float(d.qpos[1] - start_xy[1]), 3),
        "mean_speed_mps": round(dist / max(d.time - 1.0, 1e-6), 3),
        "cost_of_transport": round(energy_total / (sim.total_mass * 9.81 * max(dist, 1e-6)), 3),
        "min_vehicle_distance_m": round(sim.cosim.min_dist, 3) if sim.cosim is not None and sim.cosim.has_vehicle else None,
        **terrain_deformation(terrain0, sim.terrain.applied),
        "late_control_steps": late if (args.realtime or viewer) else None,
        "max_lag_s": round(max_lag, 4) if (args.realtime or viewer) else None,
    }
    rec.save(mop)
    if bridge and not interrupted:      # Ctrl+C 때는 rclpy가 이미 종료됨
        bridge.publish_status(build_status(sim, start_xy, energy_total, late, final_mop=mop))
    print("\n== MOP ==")
    for k, v in mop.items():
        print(f"  {k:20s} {v}")
    print(f"\n기록: {out.relative_to(ROOT)}")

    if viewer:
        # close()는 종료 신호만 보낸다. 뷰어 스레드(daemon)가 GL 자원을 다 해제하기 전에
        # 인터프리터가 종료되며 glfw.terminate()가 불리면 segfault가 나므로 끝날 때까지 기다린다.
        viewer.close()
        for t in viewer_threads:
            t.join(timeout=10)
    open_displays = [(n, p) for n, p in displays if p.poll() is None]
    if open_displays and not interrupted:
        print("창을 닫으면 종료합니다 (Ctrl+C도 가능).")
        try:
            while all(p.poll() is None for _, p in open_displays):   # 그동안 마지막 상태가 창에 남아 있다
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
    for p in [p for _, p in displays] + helpers:
        if p.poll() is None:
            p.terminate()
    if bridge:
        bridge.close()
    stats = sim.close()
    if stats:
        print(f"Chrono 서버 계산 시간: {stats['busy_s']:.1f} s")
    return mop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--realtime", action="store_true", help="벽시계에 맞춰 실행")
    ap.add_argument("--view", action="store_true", help="MuJoCo 뷰어 표시")
    ap.add_argument("--start-paused", action="store_true", help="--view와 함께: 일시정지 상태로 시작")
    ap.add_argument("--ros2", action="store_true", help="ROS2 /tf, /joint_states, /clock 발행")
    ap.add_argument("--rviz", action="store_true",
                    help="RViz로 가시화 (--ros2, --realtime 자동 적용. conda 환경에서 바로 실행 가능)")
    ap.add_argument("--mjviz", action="store_true",
                    help="MuJoCo 렌더러 어댑터로 가시화 (중립 스트림만 사용. HMMWV 등 다른 물리엔진 바디도 표시)")
    ap.add_argument("--policy", help="정책 카드 경로 (예: policies/go2_trot_bc/card.yaml). 없으면 트롯 보행기")
    ap.add_argument("--variant", choices=["cpu", "mjx"], default="cpu",
                    help="mjx: specs/go2_mjx_override.yaml 적용 모델 (CPU MuJoCo로 실행)")
    ap.add_argument("--rt", action="store_true", help="SCHED_FIFO 우선순위 50 (RT 커널)")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE",
                    help="시나리오 값 바꾸기 (여러 번 가능). 예: --set chrono.scm.soil.bekker_kphi=2e7")
    args = ap.parse_args()
    if args.rviz or args.mjviz:
        args.ros2 = args.realtime = True
    run(args)


if __name__ == "__main__":
    main()
