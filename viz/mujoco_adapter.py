"""MuJoCo 렌더러 어댑터: 중립 렌더 스트림 -> MuJoCo 뷰어.

    python -m viz.mujoco_adapter        (sim.runner --mjviz가 자동 실행)

MuJoCo를 물리엔진이 아니라 "그리기 라이브러리"로만 쓴다. 매니페스트로 그리기 전용 모델을 만든다:
모든 바디는 mocap(포즈만 받는 바디), 외형은 매니페스트의 메쉬, 지형은 높이 패치로 만든 heightfield.
물리 계산은 하지 않으며, 로봇(MuJoCo)과 HMMWV(Chrono)를 구분 없이 같은 방식으로 그린다.
물리 시뮬레이션 코드(sim/)는 import하지 않는다 (conformance/test_render_stream.py가 검사).

  입력  /sim/scene_manifest, /sim/terrain_patch, /tf, /sim/status
  출력  MuJoCo 뷰어 창 (로봇, HMMWV, 지형 변형, 정보 문구, 발 접촉, 명령 화살표, 경로)
"""
import json
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from .stream_decode import TerrainGrid, TerrainWindow, check_manifest
from .terrain_look import add_terrain_look

MAX_PATH = 300          # 경로 표시 최대 선분 수 (뷰어 user_scn 용량 제한)
TERRAIN_BODY = "terrain_window"
RENDER_WINDOW = 4.0     # 세계 지형이 크면 로봇 주변 이 반폭(m)만 그린다


def _wxyz(q_xyzw):
    x, y, z, w = q_xyzw
    return [w, x, y, z]


def build_render_model(manifest):
    """매니페스트 -> (MjModel, 바디이름->mocap 인덱스, hfield id, TerrainWindow). 물리용이 아닌 그리기 전용 모델.
    세계 지형이 크면 로봇 주변 창만 담고, 창은 mocap 바디로 옮긴다 (TerrainWindow 규칙)."""
    root = Path(manifest["asset_root"])
    s = mujoco.MjSpec()
    s.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE   # 질량 계산 불필요
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 720
    t = manifest["terrain"]
    win = TerrainWindow(t, half=RENDER_WINDOW)
    add_terrain_look(s, win.win_half)                            # 흙 질감, 1 m 옅은 격자, 비스듬한 조명
    zmin, zmax = t["z_range"]
    hf = s.add_hfield(name="terrain", nrow=win.wnrow, ncol=win.wncol, size=[*win.win_half, zmax - zmin, 0.1])
    hf.userdata = [0.0] * (win.wnrow * win.wncol)
    holder = s.worldbody.add_body(name=TERRAIN_BODY, mocap=True, pos=[*win.center, 0.0]) if win.windowed else s.worldbody
    holder.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname="terrain",
                    pos=[0, 0, zmin], material="terrain", contype=0, conaffinity=0)

    meshes = {}
    for v in manifest["visuals"]:
        if v["mesh"] not in meshes:
            meshes[v["mesh"]] = f"m{len(meshes)}"
            m = s.add_mesh(name=meshes[v["mesh"]], file=str(root / v["mesh"]))
            m.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL     # 열린 메쉬(타이어 등)도 허용
    bodies = {}
    for b in manifest["bodies"]:
        body = s.worldbody.add_body(name=b["name"], mocap=True, mass=1e-3, inertia=[1e-6] * 3)
        bodies[b["name"]] = body
    for v in manifest["visuals"]:
        bodies[v["body"]].add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=meshes[v["mesh"]],
                                   pos=v["pos"], quat=_wxyz(v["quat"]), rgba=v["rgba"],
                                   contype=0, conaffinity=0, group=1)
    model = s.compile()
    mocap = {b["name"]: model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b["name"])]
             for b in manifest["bodies"]}
    tb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TERRAIN_BODY)
    win.mocap = model.body_mocapid[tb] if tb >= 0 else -1
    return model, mocap, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain"), win


def write_terrain(model, hid, grid, z_range, window=None):
    """지형 높이(창이 있으면 창 영역)를 그리기 모델의 heightfield에 쓴다."""
    zmin, zmax = z_range
    h = window.heights(grid) if window is not None else grid.heights
    adr = model.hfield_adr[hid]
    model.hfield_data[adr:adr + h.size] = ((h - zmin) / (zmax - zmin)).ravel()


def apply_poses(model, data, mocap, poses):
    """poses: {바디 이름: (pos, quat_xyzw)} -> mocap 갱신 후 기구학 계산 (물리 스텝 없음)."""
    for name, (p, q) in poses.items():
        k = mocap.get(name)
        if k is not None and k >= 0:
            data.mocap_pos[k] = p
            data.mocap_quat[k] = _wxyz(q)
    mujoco.mj_kinematics(model, data)


def status_text(s):
    m = s.get("final_mop")
    if m:
        state = "FELL" if m["fell"] else ("INTERRUPTED" if m.get("interrupted") else "FINISHED")
        body = (f"t {m['sim_time_s']:.2f} s\nforward {m['forward_x_m']:.2f} m  lateral {m['lateral_drift_m']:+.2f} m\n"
                f"mean speed {m['mean_speed_mps']:.2f} m/s  CoT {m['cost_of_transport']:.2f}\n"
                f"late steps {m['late_control_steps']}")
        if m.get("min_vehicle_distance_m"):
            body += f"\nmin HMMWV distance {m['min_vehicle_distance_m']:.2f} m"
        return state, body + "\n(close window to exit)"
    cot = "--" if s["cot"] is None else f"{s['cot']:.2f}"
    lines = [f"t {s['t']:.2f} s   [{s['controller']}|{s['variant']}]",
             f"cmd vx {s['command']['vx']:.2f} m/s  yaw {s['command']['yaw_rate']:+.2f} rad/s",
             f"speed {s['speed_avg']:.2f} m/s  dist {s['distance']:.2f} m  CoT {cot}",
             "contact " + " ".join(n if f["contact"] else "--" for n, f in s["feet"].items()),
             f"late steps {s['late_steps']}"]
    for v in s.get("vehicles", []):
        lines.append(f"HMMWV dist {v['distance']:.1f} m  speed {v['speed']:.1f} m/s")
    ev = s.get("event")
    if ev and s["t"] - ev["t"] < 3.0:
        lines.append(f"event @{ev['t']:.1f}s {ev['desc']}")
    return "STATUS", "\n".join(lines)


def draw_overlays(scn, status, path):
    """발 접촉(구), 명령 화살표, 경로(선분)를 뷰어의 그리기 전용 영역(user_scn)에 그린다."""
    scn.ngeom = 0

    def add(gtype, size, pos, mat, rgba):
        if scn.ngeom >= scn.maxgeom:
            return None
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, gtype, np.asarray(size, float), np.asarray(pos, float), np.asarray(mat, float).ravel(),
                            np.asarray(rgba, np.float32))
        scn.ngeom += 1
        return g

    if status:
        for f in status["feet"].values():
            add(mujoco.mjtGeom.mjGEOM_SPHERE, [0.025, 0, 0], f["pos"], np.eye(3),
                [0.2, 0.9, 0.3, 0.9] if f["contact"] else [0.6, 0.6, 0.6, 0.5])
        b, yaw, L = np.array(status["base_pos"]), status["yaw"], max(status["command"]["vx"], 0.02)
        g = add(mujoco.mjtGeom.mjGEOM_ARROW, [0, 0, 0], [0, 0, 0], np.eye(3), [0.25, 0.6, 1.0, 0.9])
        if g is not None:
            p0 = b + [0, 0, 0.15]
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.015, p0, p0 + L * np.array([np.cos(yaw), np.sin(yaw), 0]))
    recent = path[-(MAX_PATH + 1):]
    for a, c in zip(recent[:-1], recent[1:]):                  # 연속한 두 점을 잇는 선분
        g = add(mujoco.mjtGeom.mjGEOM_CAPSULE, [0, 0, 0], [0, 0, 0], np.eye(3), [1.0, 0.85, 0.2, 0.9])
        if g is not None:
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.015, a - [0, 0, 0.22], c - [0, 0, 0.22])


class MujocoViewerAdapter:
    def __init__(self):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("mujoco_viewer_adapter")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        history = QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/sim/scene_manifest", self.on_manifest, latched)
        self.node.create_subscription(String, "/sim/terrain_patch", self.on_patch, history)
        self.node.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.node.create_subscription(String, "/sim/status", self.on_status, 10)
        self.manifest = self.grid = None
        self.patches, self.poses, self.status, self.path = [], {}, None, []
        self.lock = threading.Lock()
        # 메시지는 별도 스레드에서 계속 받아 최신 값만 유지한다 (그리기가 느려도 밀리지 않게)
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        try:
            while self.rclpy.ok():
                self.rclpy.spin_once(self.node, timeout_sec=0.05)
        except Exception:          # 종료 신호로 context가 닫히면 끝낸다
            pass

    def on_manifest(self, msg):
        self.manifest = check_manifest(json.loads(msg.data))

    def on_patch(self, msg):
        with self.lock:
            self.patches.append(json.loads(msg.data))

    def on_tf(self, msg):
        with self.lock:
            for t in msg.transforms:
                tr, r = t.transform.translation, t.transform.rotation
                self.poses[t.child_frame_id] = ([tr.x, tr.y, tr.z], [r.x, r.y, r.z, r.w])

    def on_status(self, msg):
        s = json.loads(msg.data)
        b = np.array(s["base_pos"])
        with self.lock:
            self.status = s
            if not self.path or np.linalg.norm(b[:2] - self.path[-1][:2]) > 0.03:
                self.path.append(b)

    def run(self):
        while self.manifest is None:                  # 매니페스트가 와야 모델을 만들 수 있다
            if not self.rclpy.ok():
                return
            time.sleep(0.1)
        model, mocap, hid, win = build_render_model(self.manifest)
        data = mujoco.MjData(model)
        self.grid = TerrainGrid(self.manifest["terrain"])
        z_range = self.manifest["terrain"]["z_range"]
        self.node.get_logger().info(f"render model: {len(mocap)} bodies, {model.ngeom} geoms")
        before = set(threading.enumerate())
        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            viewer_threads = set(threading.enumerate()) - before
            base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            # 추적 카메라는 질량 중심을 따라가는데 포즈만 받는 모델에선 계산되지 않으므로, 몸통 위치를 직접 따라간다
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 3.5, -20, 135
            while viewer.is_running() and self.rclpy.ok():
                t0 = time.perf_counter()
                with self.lock:
                    patches, self.patches = self.patches, []
                    poses, status, path = dict(self.poses), self.status, list(self.path)
                moved = status is not None and win.recenter(*status["base_pos"][:2])
                if moved:                                 # 창 이동: 지형 mocap 바디를 옮긴다
                    data.mocap_pos[win.mocap] = [*win.center, 0.0]
                if patches:
                    for p in patches:
                        self.grid.apply(p)
                if patches or moved:
                    write_terrain(model, hid, self.grid, z_range, win)
                    viewer.update_hfield(hid)
                with viewer.lock():
                    apply_poses(model, data, mocap, poses)
                    if base >= 0:
                        viewer.cam.lookat[:] = data.xpos[base]
                    draw_overlays(viewer.user_scn, status, path)
                if status:     # set_texts는 내부에서 뷰어 잠금을 잡으므로 viewer.lock() 밖에서 호출 (안에서 부르면 교착)
                    title, text = status_text(status)
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, text))
                viewer.sync()
                time.sleep(max(0.0, 1 / 30 - (time.perf_counter() - t0)))
        # 뷰어 스레드가 GL 자원을 다 해제하기 전에 프로세스가 끝나면 segfault가 나므로 기다린다
        for t in viewer_threads:
            t.join(timeout=10)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main():
    import faulthandler
    import signal
    faulthandler.register(signal.SIGUSR1, all_threads=True)   # 멈춤 진단: kill -USR1 <pid> 로 스레드 위치 출력
    adapter = MujocoViewerAdapter()
    try:
        adapter.run()
    except KeyboardInterrupt:
        pass
    finally:
        adapter.close()


if __name__ == "__main__":
    main()
