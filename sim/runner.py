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
from control.node import ControllerNode
from .model_builder import ROOT, TERRAIN_BODY, build_model
from .recorder import Recorder
from .robot_io import RobotIO
from .stream import build_status
from .terrain_service import TerrainMapService

# 고장 주입 (DIS 콘솔, 문서 §10.2). 모두 로봇 쪽 경계에서 일어난다: 알고리즘은 "안 온다", "다르게 온다"만 안다
FAULTS = ("lidar_blackout", "link_loss", "imu_bias", "battery_low")
CONTROL_ACTIONS = ("freeze", "stop")      # 실행 제어: 물리에 영향 없음. 다시 실행할 시나리오에서는 뺀다 (stop은 duration으로)

SIM_VERSION = "0.3.0"
TERRAIN_COMMIT_HZ = 5.0       # 지형 갱신은 수 Hz로 충분 (문서 §5.3)
FALL_HEIGHT = 0.04            # 몸통 높이(지면 기준)가 이보다 낮으면 넘어짐 (몸통이 땅에 박힘). 넘어짐은 주로 기울기로 판정한다.
                              # 똑바로 엎드린 자세(다리 모양에 따라 0.08~0.14 m)는 넘어짐이 아니다: 링크 두절 감쇠 모드 뒤 일어설 수 있다.
                              # 0.12 -> 0.04로 바꿔도 기존 시나리오 3종 x 시드 3개의 판정과 넘어진 시각이 같았다
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
    """시나리오 dict에 'a.b.c=값' 목록을 적용한다. 없는 키는 오류 (오타로 조용히 무시되는 것을 막는다).
    원래 시나리오에서 값이 null인 절 아래에는 새 키를 만들 수 있다 (켜고 끄는 선택 기능용, 예: controller.perception)."""
    orig, scn = scn, copy.deepcopy(scn)
    for item in sets or []:
        key, _, text = item.partition("=")
        *path, leaf = key.strip().split(".")
        node, src, opened = scn, orig, False
        for k in path:
            if not opened and isinstance(src, dict) and k in src and src[k] is None:
                opened = True
            src = src.get(k) if isinstance(src, dict) else None
            if k not in node or node[k] is None:
                if not opened:
                    raise KeyError(f"--set {key}: '{k}' 없음 (있는 키: {list(node)})")
                node[k] = {}
            node = node[k]
        if leaf not in node and not opened:
            raise KeyError(f"--set {key}: '{leaf}' 없음 (있는 키: {list(node)})")
        node[leaf] = parse_value(text.strip())
    return scn


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


class Simulation:
    """scenario: YAML 경로 또는 dict. policy: 정책 카드 경로 (없으면 시나리오의 규칙 기반 보행기)."""
    dis = None      # DIS 시나리오 콘솔 서버 (sim/dis_server.py). 콘솔 요청을 events에 넣는다

    def __init__(self, scenario, spec_path=ROOT / "specs/go2_control.yaml", variant="cpu", seed=None,
                 policy=None, sensors=None):
        self.scn = copy.deepcopy(scenario) if isinstance(scenario, dict) else load_yaml(scenario)
        if seed is not None:
            self.scn["seed"] = seed
        self.spec = load_yaml(spec_path)
        t = self.scn["terrain"]
        # window: MuJoCo에 올릴 로봇 주변 창의 반폭 (없으면 세계 지도 전체). 창은 window_snap(m) 단위로 옮긴다
        self.terrain = TerrainMapService(t["size"], t["resolution"], t["z_range"], self.scn.get("seed", 0),
                                         window=t.get("window"), snap=t.get("window_snap", 1.0))
        self.window_trigger = t.get("window_trigger", 1.0)   # 로봇이 창 중심에서 이만큼 멀어지면 창을 옮긴다 (m)
        # 발 근처 셀 갱신 보류 반경 (m). 로봇이 자기 발자국을 밟게 하려면 보폭보다 작게 (문서 §5.3)
        self.guard_radius = t.get("guard_radius", 0.15)
        for p in t.get("patches", []):
            self.terrain.add_patch(p)
        self.variant = variant
        self.model, self.hfield_id = build_model(self.spec, self.terrain, variant)
        self.data = mujoco.MjData(self.model)

        # 보행 알고리즘 (control/): 로봇 상태 메시지만 받아 관절 명령을 낸다 (참값을 모른다). 같은 프로세스에서 부른다
        self.sensor_defs = load_yaml(ROOT / "specs/go2_sensors.yaml")
        self.controller = ControllerNode(self.spec, self.scn.get("controller"), policy, self.sensor_defs["lidars"])
        self.controller_desc = self.controller.desc
        self.decim = int(round(self.controller.iface.control_dt / self.model.opt.timestep))

        m = self.model
        self.base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, self.spec["robot"]["base_body"])
        self.foot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f) for f in self.spec["robot"]["feet"]]
        self.terrain_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
        tb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, TERRAIN_BODY)
        self.terrain_mocap = m.body_mocapid[tb] if tb >= 0 else -1
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

        # 센서 (문서 §8): 시나리오 sensors 목록 + 실행 옵션. 센서는 렌더러 쪽(viz/sensor_renderer.py)에서
        # 중립 스트림 메시지만 받아 계산한다. 같은 프로세스에서는 StreamTap이 메시지를 만들어 건넨다 (결정적).
        names = list(dict.fromkeys(list(self.scn.get("sensors", [])) + list(sensors or [])))
        if self.controller.sensor and self.controller.sensor not in names:   # 지형 인지 보행기가 쓰는 센서는 자동으로 켠다
            names.append(self.controller.sensor)
        self.scans, self.sensor_renderer, self.lidars = {}, None, []
        if names:
            from viz.sensor_renderer import SensorRenderer
            from .stream import StreamTap
            defs = self.sensor_defs["lidars"]
            unknown = [n for n in names if n not in defs]
            assert not unknown, f"알 수 없는 센서: {unknown} (specs/go2_sensors.yaml: {list(defs)})"
            self.tap = StreamTap(self)
            self.sensor_renderer = SensorRenderer(self.tap.manifest, {n: defs[n] for n in names},
                                                  terrain_half=min(8.0, max(defs[n]["range_max"] for n in names)))
            self.lidars = self.sensor_renderer.lidars

    def sense(self):
        """센서 주기가 됐으면 스트림 메시지(지형 패치, 바디 포즈)를 센서 렌더러에 건네고 스캔을 받는다.
        반환: 이번 제어 주기에 새로 나온 스캔 이름 목록."""
        r, t = self.sensor_renderer, self.data.time
        if not r.due(t):
            return []
        for patch in self.tap.patches():
            r.apply_patch(patch)
        r.set_poses(self.tap.poses())
        new = []
        for name, scan in r.render(t):
            self.scans[name] = scan
            new.append(name)
        return new

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
        self.controller.reset()
        # 로봇 쪽 경계: 참값 -> 센서 모델(잡음, 편향, 표류) -> 로봇 상태 메시지. 잡음 난수는 시나리오 시드
        self.robot = RobotIO(self, self.sensor_defs.get("proprio"), seed=self.scn.get("seed", 0))
        self.low_state, self.low_cmd = None, None
        self.prev_state_t = None
        self.remote = None            # 보행 알고리즘을 별도 노드로 돌릴 때: 로봇 경계 토픽 전송 (Ros2StreamPublisher)
        # 같은 프로세스 실행의 연결 지연 (ROS 배치와 같게): 명령을 latency_steps 주기 뒤에 실행
        link = {**self.spec.get("link", {}), **self.scn.get("control_link", {})}
        self.link_latency = int(link.get("latency_steps", 0))
        self.link_queue = deque()
        self.command = (0.0, 0.0)     # 운용자 이동 명령 (마지막 값)
        self.events = sorted((dict(e) for e in self.scn.get("events", [])), key=lambda e: e["t"])
        self.event_log = []         # 적용된 이벤트 (applied_t 포함). 콘솔 이벤트까지 넣어 다시 실행할 수 있는 시나리오를 만든다
        self.last_event = None      # (시각, 설명) 가시화용
        self.next_terrain_commit = 0.0
        self.faults = {}            # 고장 이름 -> {"until": 끝 시각 또는 None, "params": {...}}
        self.frozen = False         # 실행 제어: 일시정지 (실시간 루프가 시뮬레이션을 진행하지 않음)
        self.stop_requested = False # 실행 제어: 시험 종료

    # ---- 이벤트 (시나리오 파일과 DIS 시나리오 콘솔이 같은 경로로 넣는다) ----
    def _apply_events(self, verbose=True):
        while self.events and self.events[0]["t"] <= self.data.time + 1e-9:
            e = self.events.pop(0)
            e["applied_t"] = self.data.time
            self.event_log.append(e)
            if e["action"] == "set_command":       # 운용자 명령 -> 보행 알고리즘
                self.command = (e["vx"], e.get("yaw_rate", 0.0))
                if self.remote is not None:
                    self.remote.publish_cmd_vel(*self.command, self.data.time)
                else:
                    self.controller.set_command(*self.command)
            elif e["action"] == "add_patch":
                self.terrain.add_patch(e["patch"])
            elif e["action"] == "inject_fault":
                self._set_fault(e["fault"], e.get("duration"), e.get("params") or {})
            elif e["action"] == "clear_fault":
                self._clear_fault(e["fault"])
            elif e["action"] == "create_entity":    # 개체 투입 (Chrono 차량 슬롯)
                self.cosim.spawn_vehicle(e.get("speed"))
                self.cosim.near_event_sent = False
            elif e["action"] == "remove_entity":
                self.cosim.remove_vehicle()
            elif e["action"] == "set_soil":
                self.cosim.set_soil(e["soil"])
            elif e["action"] == "freeze":
                self.frozen = True
            elif e["action"] == "stop":
                self.stop_requested = True
            desc = e["action"]
            if e["action"] == "set_command":
                desc += f" vx={e['vx']:.2f} yaw={e.get('yaw_rate', 0.0):.2f}"
            elif e["action"] == "add_patch":
                desc += f" {e['patch']['kind']}"
            elif e["action"] in ("create_entity", "remove_entity"):
                desc += f" {e.get('entity_type', 'HMMWV')}"
            elif e["action"] == "set_soil":
                desc += f" {e.get('preset', 'custom')}"
            elif e["action"] in ("inject_fault", "clear_fault"):
                desc += f" {e['fault']}" + (f" {e['duration']:g}s" if e.get("duration") else "")
            self.last_event = (self.data.time, desc)
            if verbose:
                print(f"[t={self.data.time:6.2f}] event: {desc}" + (f"  (DIS {e['request']})" if e.get("source") == "dis" else ""))

    def replay_scenario(self):
        """이번 실행을 그대로 다시 돌리는 시나리오: 이벤트 = 적용된 이벤트(적용 시각으로, 콘솔 이벤트 포함) + 아직 안 된 시나리오 이벤트.
        실행 제어(일시정지, 종료)는 물리에 영향이 없으므로 빼고, 콘솔로 종료했으면 그 시각을 duration으로 한다."""
        scn = copy.deepcopy(self.scn)
        applied = [{**{k: v for k, v in e.items() if k not in ("t", "applied_t")}, "t": round(e["applied_t"], 6)}
                   for e in self.event_log if e["action"] not in CONTROL_ACTIONS]
        pending = [dict(e) for e in self.events if not e.get("source", "").startswith("dis") and e["action"] not in CONTROL_ACTIONS]
        scn["events"] = applied + pending
        if self.stop_requested:            # 끝난 시각보다 반 주기 앞: 루프(while t < duration)가 같은 스텝에서 멈춘다 (부동소수점 비교)
            scn["duration"] = round(self.data.time - 0.5 * self.decim * self.model.opt.timestep, 6)
        return scn

    # ---- 고장 주입 (로봇 쪽 경계) ----
    def _set_fault(self, name, duration, params):
        assert name in FAULTS, name
        self.faults[name] = {"until": None if duration is None else self.data.time + duration, "params": params}
        if name == "imu_bias":
            self.robot.sensors.set_fault(params.get("gyro_bias", (0, 0, 0)), params.get("attitude_offset_deg", (0, 0, 0)))
        elif name == "battery_low":
            self.robot.torque_scale = float(params.get("torque_scale", 0.6))
        elif name == "link_loss":
            self.link_queue.clear()             # 전송 중이던 명령도 잃는다
            self.prev_state_t = None

    def _clear_fault(self, name):
        if self.faults.pop(name, None) is None:
            return
        if name == "imu_bias":
            self.robot.sensors.clear_fault()
        elif name == "battery_low":
            self.robot.torque_scale = 1.0

    def _expire_faults(self, verbose):
        for name, f in list(self.faults.items()):
            if f["until"] is not None and self.data.time >= f["until"] - 1e-9:
                self._clear_fault(name)
                self.last_event = (self.data.time, f"fault_cleared {name}")
                if verbose:
                    print(f"[t={self.data.time:6.2f}] fault cleared: {name}")

    def fallback_cmd(self):
        """명령이 오지 않을 때 로봇 쪽 동작: 마지막으로 받은 명령 유지 (처음이면 기본 자세).
        link_loss의 robot_behavior=damp면 감쇠 모드 (위치 게인 0, 속도 게인만: 천천히 주저앉는다)."""
        f = self.faults.get("link_loss")
        if f and f["params"].get("robot_behavior") == "damp":
            zeros = [0.0] * 12
            return {"q_des": self.data.qpos[7:].tolist(), "dq_des": zeros, "kp": zeros, "kd": [5.0] * 12, "tau_ff": zeros}
        return self.low_cmd if self.low_cmd is not None else self.hold_cmd()

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

    def hold_cmd(self):
        """명령이 아직 없을 때: 기본 자세 유지 (로봇 쪽 안전 기본값)."""
        iface = self.controller.iface
        zeros = [0.0] * 12
        return {"q_des": iface.targets_from_action(np.zeros(12)).tolist(), "dq_des": zeros, "kp": iface.kp.tolist(),
                "kd": iface.kd.tolist(), "tau_ff": zeros}

    def estimate(self):
        """알고리즘의 최신 상태 추정 {t, v_body, yaw, contacts, pos} (t = 그 추정이 쓴 로봇 상태 시각). 별도 노드면 받은 값."""
        if self.remote is not None:
            return self.remote.latest_est
        if getattr(self, "low_state", None) is None:
            return None
        e = self.controller.est
        return {"t": self.low_state["t"], "v_body": e.v_body, "yaw": e.yaw, "contacts": e.contacts, "pos": e.pos}

    @property
    def command_display(self):
        """정보판용 명령: 같은 프로세스면 알고리즘이 쓰는 값(가속 제한 후), 별도 노드면 운용자 명령."""
        return np.array(self.command) if self.remote is not None else self.controller.clock.cmd_f

    # ---- 같은 프로세스 실행 때 알고리즘 상태 보기 (기록, 학습 스크립트, 기준 벡터 생성용) ----
    iface = property(lambda self: self.controller.iface)
    clock = property(lambda self: self.controller.clock)
    ctrl = property(lambda self: self.controller.ctrl)
    obs = property(lambda self: self.controller.obs)
    action = property(lambda self: self.controller.action)
    q_des = property(lambda self: self.controller.q_des)
    on_action = property(lambda self: self.controller.on_action,
                         lambda self, f: setattr(self.controller, "on_action", f))

    def step_control(self, verbose=True):
        """제어 주기 1회 = 물리 스텝 decim회."""
        d = self.data
        dt = self.decim * self.model.opt.timestep
        if self.dis is not None:
            self.dis.poll()                      # 콘솔 요청 -> events (적용 시각이 이 주기 이후면 예약)
        self._expire_faults(verbose)
        self._apply_events(verbose)
        if self.dis is not None:
            self.dis.after_events()              # 적용된 요청에 Complete, 주기 보고
        if self.cosim is not None:
            self.cosim.robot_xy = d.qpos[:2]      # 다음 Chrono 요청에 실림 (차량 양보 판단)
            self.cosim.sync(d.time)              # Chrono 결과 반영 (지형 원천 갱신 + 차량 포즈)
            dist = self.cosim.distance_to(d.qpos[:2])
            if dist < self.cosim.near_dist and not self.cosim.near_event_sent:
                self.cosim.near_event_sent = True
                self.last_event = (d.time, f"vehicle_near {dist:.1f}m")
                if verbose:
                    print(f"[t={d.time:6.2f}] event: vehicle_near ({dist:.2f} m)")
        terrain_changed = False
        if d.time >= self.next_terrain_commit:
            if self.terrain.recenter(*d.qpos[:2], trigger=self.window_trigger):
                # 창 이동: 지형 mocap 바디를 새 창 중심으로 옮기고 창 영역을 다시 채운다 (월드 좌표의 높이는 그대로)
                d.mocap_pos[self.terrain_mocap] = [*self.terrain.window_center, 0.0]
                self.terrain.write_all(self.model, self.hfield_id)
                terrain_changed = True
            terrain_changed |= self.terrain.commit(self.model, self.hfield_id, self.feet_xy(), self.guard_radius)
            self.next_terrain_commit += 1.0 / TERRAIN_COMMIT_HZ

        # 로봇 경계: 센서 측정 -> 보행 알고리즘 -> 관절 명령 -> 모터. 알고리즘은 참값을 모른다
        self.low_state = self.robot.read()
        if "link_loss" in self.faults:         # 제어 링크 두절: 상태가 알고리즘에 가지 않고 명령도 오지 않는다
            self.low_cmd = self.fallback_cmd()
        elif self.remote is None:
            self.link_queue.append(self.controller.step(self.low_state))
            # ROS 배치와 같은 지연: 지연 주기만큼 쌓인 뒤부터 실행. 그 전에는 기본 자세 유지 (첫 명령 전 ROS와 같음),
            # 링크가 복구된 직후에는 마지막 명령 유지
            self.low_cmd = self.link_queue.popleft() if len(self.link_queue) > self.link_latency else self.fallback_cmd()
        else:                                  # 별도 노드: 상태를 보내고, 마지막으로 받은 명령을 실행 (실제 로봇처럼)
            prev = self.prev_state_t
            if prev is not None:               # 직전 상태의 명령을 한 주기까지 기다린다 (실시간보다 늦어진 동안에도 노드에 계산 시간)
                self.remote.wait_cmd(prev, self.controller.iface.control_dt)
            self.prev_state_t = self.low_state["t"]
            self.remote.publish_low_state(self.low_state)
            self.remote.spin_some()
            # 직전 주기까지의 상태로 계산된 명령만 실행 (연결 지연 한 주기 고정). 노드가 늦으면 더 오래된 명령 -> 지연 MOP에 드러난다
            self.low_cmd = (self.remote.cmd_until(prev) if prev is not None else None) or self.fallback_cmd()
        energy = self.robot.apply(self.low_cmd)
        if self.cosim is not None and self.cosim.feet_enabled:
            # 발자국: 물리 스텝마다 평균한 발 수직력 참값 (흙에 걸리는 실제 하중)
            self.cosim.observe_feet(d.geom_xpos[self.foot_ids], self.robot.foot_force)
        roll, pitch = quat_to_roll_pitch(d.qpos[3:7])          # 넘어짐 판정용 참값 (시험 판정자 쪽)
        self.new_scans = self.sense() if self.sensor_renderer is not None else []
        f = self.faults.get("lidar_blackout")
        if f:                                  # 센서 고장: 점군이 나오지 않는다 (지정한 센서만, 없으면 전부)
            self.new_scans = [n for n in self.new_scans if f["params"].get("sensor") not in (None, n)]
        if self.remote is None:                # 같은 프로세스: 점군(센서 좌표)과 시각만 알고리즘에 건넨다. 별도 노드는 ROS 토픽으로 받는다
            for name in self.new_scans:
                self.controller.on_scan(name, self.scans[name]["t"], self.scans[name]["points"])
        return terrain_changed, energy, roll, pitch

    def fallen(self, roll, pitch):
        bx, by, bz = self.data.qpos[:3]
        return bz - self.terrain.height_at(bx, by) < FALL_HEIGHT or max(abs(roll), abs(pitch)) > FALL_TILT


def run(args):
    scenario = args.scenario if isinstance(args.scenario, dict) else load_yaml(args.scenario)
    scenario = apply_overrides(scenario, getattr(args, "set", None))
    sensor_node = getattr(args, "sensor_node", False)
    # --sensor-node: 센서를 별도 프로세스(ROS2 스트림 구독)에서 계산하므로 러너 안에서는 센서를 켜지 않는다
    sim = Simulation(scenario, variant=args.variant, policy=args.policy,
                     sensors=None if sensor_node else getattr(args, "sensor", None))
    if sensor_node and sim.controller.sensor:
        raise SystemExit("지형 인지 보행기는 센서를 러너 안에서 계산한다. --sensor-node 없이 실행한다.")
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
        "control_link": {"mode": "ros_node" if getattr(args, "controller_node", False) else "in_process",
                         "latency_steps": None if getattr(args, "controller_node", False) else sim.link_latency},
        "versions": {"sim": SIM_VERSION, "spec": sim.spec["spec_version"],
                     "controller": sim.controller_desc, "mujoco": mujoco.__version__,
                     "model": sim.spec["robot"]["menagerie_commit"]},
        "host": platform.node(), "started": stamp,
    }
    rec = Recorder(out, meta)
    dis_port = getattr(args, "dis_port", None)
    if dis_port is not None:                 # DIS 시나리오 콘솔 (문서 §10): 콘솔 요청을 시나리오 이벤트로 받는다 (이 PC 안에서만)
        from dis_console.auth import load_key
        from .dis_server import DisScenarioServer
        host, key_path = getattr(args, "dis_host", "127.0.0.1"), getattr(args, "dis_key", None)
        if host not in ("127.0.0.1", "localhost") and not key_path:
            raise SystemExit("이 PC 밖에서 콘솔 요청을 받으려면 인증 키가 필요하다 (--dis-key, python -m dis_console.auth keygen)")
        out.mkdir(parents=True, exist_ok=True)
        sim.dis = DisScenarioServer(sim, port=dis_port, host=host, log_path=out / "dis_events.jsonl",
                                    comm_lost_behavior=getattr(args, "dis_comm_lost", "STOP"),
                                    key=load_key(key_path) if key_path else None, pdu_dir=out)
        meta["dis"] = {"host": host, "port": sim.dis.port, "auth": bool(key_path)}
        if not (args.realtime or args.view):
            args.realtime = True
            meta["mode"] = "realtime"
            print("DIS 콘솔: 사람이 명령을 넣을 수 있게 실시간으로 실행한다 (--realtime)")
        key_opt = f" --key {key_path}" if key_path else ""
        print(f"DIS 시나리오 콘솔 대기: UDP {host}:{sim.dis.port}{' (인증)' if key_path else ''}  "
              f"(python -m dis_console.console --sim 127.0.0.1:{sim.dis.port}{key_opt})")
        if getattr(args, "dis_wait", False):
            print("콘솔 접속을 기다린다 (Ctrl+C로 중단)...")
            while not sim.dis.consoles:
                sim.dis.poll(); sim.dis.after_events()
                time.sleep(0.05)
            print("콘솔 접속. 시작")

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
        from .ros2_bridge import (Ros2StreamPublisher, launch_controller_node, launch_mujoco_viewer, launch_rviz_stack,
                                  launch_sensor_node)
        bridge = Ros2StreamPublisher(m, sim.terrain, sim.cosim)
        odom_offset = None                       # 알고리즘 지형 지도(odom) 표시 정렬 (판정자 쪽 참값, 표시 전용)
        if sim.controller.terrain is not None:   # 시작: odom 원점 = 시작 몸통 위치
            odom_offset = d.qpos[:3].copy()
            bridge.publish_odom_frame(odom_offset, 0.0)
        if args.rviz:
            rviz, rviz_adapter = launch_rviz_stack(out)
            displays.append(("RViz", rviz))
            helpers.append(rviz_adapter)
        if args.mjviz:
            displays.append(("MuJoCo viewer", launch_mujoco_viewer(out)))
        if sensor_node:
            names = list(dict.fromkeys(list(scenario.get("sensors", [])) + list(getattr(args, "sensor", None) or [])))
            helpers.append(launch_sensor_node(out, names or ["front_lidar"]))
        if getattr(args, "controller_node", False):
            # 보행 알고리즘을 별도 노드로: 로봇 상태 토픽만 보고 관절 명령 토픽을 낸다 (실제 탑재 소프트웨어 형태)
            bridge.enable_robot_io()
            sim.remote = bridge
            scn_path = out / "scenario_effective.yaml"         # --set 반영된 시나리오를 노드에도 그대로 넘긴다
            scn_path.write_text(yaml.safe_dump(scenario, allow_unicode=True, sort_keys=False))
            helpers.append(launch_controller_node(out, scn_path, args.policy, viz=True))   # 지형 인지일 때만 실제로 발행
            print("보행 알고리즘 노드 대기 중...")
            if not bridge.wait_for_subscriber(topic="/robot/low_state"):
                print(f"보행 알고리즘 노드가 뜨지 않았다. 로그: {(out / 'controller_node.log').relative_to(ROOT)}")
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
    est_err_v, est_err_yaw = [], []           # 추정 오차 (판정자 쪽에서만 참값과 비교)
    cmd_latency = []                          # 별도 노드: 실행한 명령이 몇 초 전 상태로 계산됐는가
    viz_count = 0
    touchdowns, edge_touchdowns = 0, 0        # 착지 횟수, 그중 지면 모서리(발 주변 ±4 cm 높이 차 > 2 cm)에 디딘 횟수 (참 지형)
    prev_contact = sim.foot_contacts()
    truth_hist = {round(d.time, 6): (d.xmat[sim.base_id].reshape(3, 3).T @ d.qvel[:3], quat_to_yaw(d.qpos[3:7]))}
    scan_log = {}
    wall0 = run_start = time.perf_counter()
    paused_total, pause_start = 0.0, None
    late, max_lag = 0, 0.0
    next_view = 0.0
    next_status = 0.0
    was_paused = False

    interrupted = False
    frozen_since, next_frozen_status = None, 0.0
    try:
        while d.time < scn["duration"]:
            if sim.frozen:                      # 실행 제어 (DIS 콘솔): 일시정지. 콘솔 요청은 계속 받는다 (재개, 예약 등)
                if frozen_since is None:
                    frozen_since = time.perf_counter()
                    print(f"[t={d.time:6.2f}] frozen (DIS)")
                if sim.dis is not None:
                    sim.dis.poll(); sim.dis.after_events()
                if bridge and time.perf_counter() >= next_frozen_status:
                    bridge.publish_status(build_status(sim, start_xy, energy_total, late))
                    next_frozen_status = time.perf_counter() + 0.5
                if any(p.poll() is not None for _, p in displays):
                    break
                time.sleep(0.02)
                continue
            if frozen_since is not None:        # 재개: 정지한 시간은 실시간 지연으로 세지 않는다
                stopped = time.perf_counter() - frozen_since
                wall0 += stopped; paused_total += stopped
                frozen_since = None
                print(f"[t={d.time:6.2f}] resumed (DIS)")
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

            if "t" in sim.low_cmd:                   # 실행한 명령이 몇 초 전 상태로 계산됐는가 (두 실행 방식 공통)
                cmd_latency.append(sim.low_state["t"] - sim.low_cmd["t"])
            contact = sim.foot_contacts()
            for i in np.where(contact & ~prev_contact)[0]:
                fx, fy = d.geom_xpos[sim.foot_ids[i]][:2]
                hs = [sim.terrain.height_at(fx + ox, fy + oy) for ox in (-0.04, 0, 0.04) for oy in (-0.04, 0, 0.04)]
                touchdowns += 1
                edge_touchdowns += int(max(hs) - min(hs) > 0.02)
            prev_contact = contact
            # 추정 오차: 알고리즘의 추정값을 그 추정이 쓴 로봇 상태 시각(t)의 참값과 비교한다.
            # 별도 노드면 추정값은 /control/estimate로 받는다 (한 주기 늦게 도착하므로 시각으로 맞춘다)
            v_true = d.xmat[sim.base_id].reshape(3, 3).T @ d.qvel[:3]
            truth_hist[round(d.time, 6)] = (v_true, quat_to_yaw(d.qpos[3:7]))   # 다음 로봇 상태 시각의 참값
            est = sim.estimate()
            ref = truth_hist.get(round(est["t"], 6)) if est else None
            if ref is not None:
                yaw_err = (est["yaw"] - ref[1] + np.pi) % (2 * np.pi) - np.pi
                est_err_v.append(est["v_body"][0] - ref[0][0]); est_err_yaw.append(yaw_err)
                rec.log(est_v_body=est["v_body"], est_yaw=est["yaw"], est_contacts=np.asarray(est["contacts"], float),
                        true_v_body=ref[0])
            else:
                rec.log(est_v_body=np.full(3, np.nan), est_yaw=np.nan, est_contacts=np.full(4, np.nan),
                        true_v_body=np.full(3, np.nan))
            while len(truth_hist) > 10:
                truth_hist.pop(next(iter(truth_hist)))
            rec.log(t=d.time, base_pos=d.qpos[:3], base_quat_wxyz=d.qpos[3:7], base_linvel=d.qvel[:3],
                    base_angvel_body=d.qvel[3:6], q=d.qpos[7:], qd=d.qvel[6:], tau=d.ctrl,
                    contact=sim.foot_contacts().astype(float), cmd=sim.command_display,
                    q_des=np.asarray(sim.low_cmd["q_des"]),
                    terrain_version=sim.terrain.version)
            if sim.cosim is not None and sim.cosim.has_vehicle:
                rec.log(vehicle_pos=sim.cosim.vehicle["pos"], vehicle_speed=sim.cosim.vehicle["speed"])

            for name in sim.new_scans:              # 센서 출력: 기록 + 발행
                scan_log.setdefault(name, []).append(sim.scans[name])
                if bridge:
                    bridge.publish_scan(name, sim.scans[name])
            if bridge:
                bridge.publish(d)
                viz_count += 1
                if odom_offset is not None and viz_count % 5 == 0 and (est := sim.estimate()) is not None and "pos" in est:
                    # world -> odom을 계속 로봇에 맞춘다: 실제 몸통 위치 - 추정 위치 (1초 저역통과, 평행이동만).
                    # 로봇 주변 지도가 실제 지면에 붙어 보이고, 오래전에 쌓은 먼 칸에는 그동안의 추정 표류가 남아 보인다
                    odom_offset += 0.1 * (d.qpos[:3] - np.asarray(est["pos"]) - odom_offset)
                    bridge.publish_odom_frame(odom_offset, d.time)
                if sim.remote is None and viz_count % 5 == 0 and (snap := sim.controller.perception_snapshot()):
                    bridge.publish_perception(snap)      # 같은 프로세스: 알고리즘 지형 지도와 디딜 곳 (10 Hz, 지도 2 Hz)
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
            if sim.stop_requested:
                print(f"[t={d.time:6.2f}] stopped (DIS)")
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
        "stopped_by_console": sim.stop_requested,
        "distance_m": round(dist, 3),
        "forward_x_m": round(float(d.qpos[0] - start_xy[0]), 3),
        "lateral_drift_m": round(float(d.qpos[1] - start_xy[1]), 3),
        "mean_speed_mps": round(dist / max(d.time - 1.0, 1e-6), 3),
        "cost_of_transport": round(energy_total / (sim.total_mass * 9.81 * max(dist, 1e-6)), 3),
        "min_vehicle_distance_m": round(sim.cosim.min_dist, 3) if sim.cosim is not None and np.isfinite(sim.cosim.min_dist) else None,   # 차가 한 번도 안 나왔으면 None
        **terrain_deformation(terrain0, sim.terrain.applied),
        # 보행 알고리즘의 상태 추정 오차 (알고리즘은 참값을 모른다. 여기서 판정만 한다)
        "est_speed_rmse_mps": round(float(np.sqrt(np.mean(np.square(est_err_v[25:])))), 4) if len(est_err_v) > 25 else None,
        "est_yaw_err_end_deg": round(float(np.degrees(est_err_yaw[-1])), 2) if est_err_yaw else None,
        "cmd_latency_ms_mean": round(1000 * float(np.mean(cmd_latency)), 1) if cmd_latency else None,
        "cmd_latency_ms_max": round(1000 * float(np.max(cmd_latency)), 1) if cmd_latency else None,
        "touchdowns": touchdowns,
        "edge_touchdowns": edge_touchdowns,
        "late_control_steps": late if (args.realtime or viewer) else None,
        "max_lag_s": round(max_lag, 4) if (args.realtime or viewer) else None,
    }
    rec.save(mop)
    if sim.dis is not None:                  # 콘솔 이벤트를 적용 시각으로 넣은 시나리오: python -m sim.runner <이 파일> 로 같은 결과
        reason = ("stopped" if sim.stop_requested else "fell" if fell_at is not None else
                  "interrupted" if interrupted else "duration")
        sim.dis.announce_end({"reason": reason, "run": str(out.relative_to(ROOT)), "sim_time_s": mop["sim_time_s"],
                              "forward_x_m": mop["forward_x_m"], "fell": mop["fell"]})
        sim.dis.close()
        (out / "scenario_replay.yaml").write_text(yaml.safe_dump(sim.replay_scenario(), allow_unicode=True, sort_keys=False))
    for name, scans in scan_log.items():      # LiDAR 원시 출력: 프레임별 센서 좌표 점군 + 센서 월드 자세
        np.savez_compressed(out / f"{name}.npz", t=np.array([s["t"] for s in scans]),
                            frame=np.concatenate([np.full(len(s["points"]), i, np.int32) for i, s in enumerate(scans)]),
                            points=np.concatenate([s["points"] for s in scans]), ring=np.concatenate([s["ring"] for s in scans]),
                            label=np.concatenate([s["label"] for s in scans]),
                            R=np.array([s["R"] for s in scans]), p=np.array([s["p"] for s in scans]))
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
    ap.add_argument("--sensor", action="append", metavar="NAME",
                    help="센서 켜기 (specs/go2_sensors.yaml의 이름, 여러 번 가능). 예: --sensor front_lidar")
    ap.add_argument("--sensor-node", action="store_true",
                    help="센서를 별도 프로세스(ROS2 중립 스트림만 구독)에서 계산 (--ros2, --realtime 자동. 결정성 없음)")
    ap.add_argument("--ros-msg", choices=["auto", "typed", "json"],
                    help="--controller-node 로봇 경계 메시지: typed(go2_rt_msgs), json, auto(빌드돼 있으면 typed, 기본)")
    ap.add_argument("--controller-node", action="store_true",
                    help="보행 알고리즘을 별도 ROS2 노드로 (로봇 상태/관절 명령 토픽으로만 주고받음. --ros2, --realtime 자동)")
    ap.add_argument("--dis-port", type=int, metavar="PORT",
                    help="DIS 시나리오 콘솔 요청을 받는다 (UDP 127.0.0.1, 보통 3000). 실시간으로 실행. 끝나면 scenario_replay.yaml")
    ap.add_argument("--dis-wait", action="store_true", help="--dis-port와 함께: 콘솔이 접속할 때까지 시작하지 않는다")
    ap.add_argument("--dis-key", metavar="FILE", help="DIS 인증 공유 키 (python -m dis_console.auth keygen FILE). 콘솔에도 같은 파일")
    ap.add_argument("--dis-host", default="127.0.0.1",
                    help="콘솔 요청을 받을 주소 (기본: 이 PC 안). 다른 주소는 --dis-key가 있어야 한다")
    ap.add_argument("--dis-comm-lost", choices=["STOP", "CONTINUE"], default="STOP",
                    help="콘솔이 모두 5초 넘게 조용하면(통신 두절): STOP = 로봇 이동 명령 0 (기본), CONTINUE = 그대로")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE",
                    help="시나리오 값 바꾸기 (여러 번 가능). 예: --set chrono.scm.soil.bekker_kphi=2e7")
    args = ap.parse_args()
    from control.robot_msgs import set_mode
    set_mode(getattr(args, "ros_msg", None))
    if args.rviz or args.mjviz or args.sensor_node or args.controller_node:
        args.ros2 = args.realtime = True
    run(args)


if __name__ == "__main__":
    main()
