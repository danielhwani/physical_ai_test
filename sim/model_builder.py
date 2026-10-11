"""MJCF 원본(Menagerie go2.xml)에 시나리오 지형을 붙여 MjModel을 만든다.

variant="mjx" 이면 specs/go2_mjx_override.yaml 의 adopt 항목을 원본에 적용한다.
(MJX 학습 환경이 쓰는 모델과 같은 물리 설정을 CPU MuJoCo에서 재현 -> 차이의 계층 분리, 문서 §13.2)
"""
from fnmatch import fnmatch
from pathlib import Path

import mujoco
import yaml

from viz.terrain_look import add_terrain_look   # 화면 외형 (viz는 sim을 모르므로 이 방향 의존은 허용)

ROOT = Path(__file__).resolve().parent.parent
MJX_OVERRIDE = ROOT / "specs/go2_mjx_override.yaml"
COLLISION_GROUP = 3
TERRAIN_BODY = "terrain_window"   # 창 모드에서 지형 geom을 붙이는 mocap 바디 (렌더 스트림에서는 제외)
SOIL_PLATE = "soil_plate_"        # 무른 흙: 발마다 흙 받침판 mocap 바디 (렌더 스트림에서는 제외)
PLATE_HALF = (0.06, 0.06, 0.01)   # 받침판 반크기 (m)

_GEOM_TYPES = {"sphere": mujoco.mjtGeom.mjGEOM_SPHERE, "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
               "box": mujoco.mjtGeom.mjGEOM_BOX, "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER}
_CONES = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL, "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}


def robot_spec(spec_cfg, variant="cpu"):
    """로봇만 담은 MjSpec (지형 없음). 적합성 시험에서도 사용."""
    s = mujoco.MjSpec.from_file(str(ROOT / spec_cfg["robot"]["model_source"]))
    s.option.timestep = spec_cfg["physics"]["timestep"]
    if variant == "mjx":
        apply_mjx_override(s, spec_cfg, yaml.safe_load(MJX_OVERRIDE.read_text())["adopt"])
    elif variant != "cpu":
        raise ValueError(f"unknown variant: {variant}")
    return s


def apply_mjx_override(s, spec_cfg, adopt):
    opt = adopt["option"]
    s.option.cone = _CONES[opt["cone"]]
    s.option.iterations = opt["iterations"]
    s.option.ls_iterations = opt["ls_iterations"]
    for flag in opt.get("disable", []):
        s.option.disableflags |= getattr(mujoco.mjtDisableBit, f"mjDSBL_{flag.upper()}")

    for name in spec_cfg["robot"]["joints"]:
        s.joint(name).frictionloss = adopt["joint"]["frictionloss"]

    feet = set(spec_cfg["robot"]["feet"])
    for g in s.geoms:
        if g.name in feet:
            g.size = [adopt["foot"]["size"], 0, 0]
            g.solimp = adopt["foot"]["solimp"] + [0.5, 2.0]

    for body in s.bodies:
        rules = [r for pat, r in adopt["collision_geoms"].items() if fnmatch(body.name, pat)]
        if not rules:
            continue
        geoms = [g for g in body.geoms if g.group == COLLISION_GROUP and g.name not in feet]
        if len(geoms) != len(rules[0]):
            raise ValueError(f"{body.name}: 충돌 geom {len(geoms)}개, 오버라이드 {len(rules[0])}개")
        for g, r in zip(geoms, rules[0]):
            if "type" in r:
                g.type = _GEOM_TYPES[r["type"]]
            if "size" in r:
                g.size = r["size"]
            if "pos" in r:
                g.pos = r["pos"]


def build_model(spec_cfg, terrain, variant="cpu", soft_feet=False):
    """soft_feet: 무른 흙(sim/foot_soil.py)을 쓸 때 흙 마찰계수(tanφ). 발과 정강이(calf 바디)는 지형 heightfield와 닿지 않고,
    발마다 둔 보이지 않는 흙 받침판(mocap 상자)과 닿는다. 받침판 높이는 흙 모델이 정한다. 다른 충돌은 그대로."""
    s = robot_spec(spec_cfg, variant)

    # 사전 할당 heightfield (크기/해상도는 이후 변경 불가)
    # MuJoCo에는 Terrain Map Service의 창만 할당한다 (창 = 세계 지도 전체일 수도 있다)
    hf = s.add_hfield(name="terrain", nrow=terrain.wnrow, ncol=terrain.wncol, size=terrain.hfield_size)
    hf.userdata = [0.0] * (terrain.wnrow * terrain.wncol)   # 실제 높이는 컴파일 후 write_all()로 기록
    add_terrain_look(s, terrain.win_half)                  # 외형만 (흙 질감, 1 m 옅은 격자, 비스듬한 조명)
    if terrain.windowed:
        # 창을 옮길 때 지형 geom 위치를 바꾸면, MuJoCo가 모델 생성 때 계산해 둔 월드 고정 geom의 충돌 경계 상자가
        # 그대로 남아 창 밖(처음 위치 기준)으로 나간 발의 접촉을 놓친다 (확인함). mocap 바디에 붙이면 경계 상자가
        # 바디 좌표계 기준이라 함께 움직인다. 창 중심은 data.mocap_pos로 옮긴다.
        body = s.worldbody.add_body(name=TERRAIN_BODY, mocap=True, pos=[*terrain.window_center, 0.0])
        body.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname="terrain",
                      pos=[0.0, 0.0, terrain.z_min], material="terrain")
    else:
        s.worldbody.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname="terrain",
                             pos=terrain.geom_pos, material="terrain")

    if soft_feet:
        # 충돌 조건: (contype1 & conaffinity2) or (contype2 & conaffinity1).
        # 지형 2, 받침판 4, 발 1|4, 정강이의 나머지 1, 나머지 로봇 1|2 -> 발·정강이는 지형과 안 닿고, 발만 받침판과 닿는다
        feet = set(spec_cfg["robot"]["feet"])
        legs = {b.name for b in s.bodies if b.name.endswith("_calf")}
        s.geom("terrain").contype, s.geom("terrain").conaffinity = 2, 2
        for g in s.geoms:
            if g.name == "terrain" or g.contype == 0 and g.conaffinity == 0:
                continue
            if g.name in feet:
                g.contype, g.conaffinity = 5, 5
            elif g.parent.name in legs:
                g.contype, g.conaffinity = 1, 1
            else:
                g.contype, g.conaffinity = 3, 3
        for f in spec_cfg["robot"]["feet"]:
            body = s.worldbody.add_body(name=SOIL_PLATE + f, mocap=True, pos=[0.0, 0.0, -1.0])
            body.add_geom(name=SOIL_PLATE + f, type=mujoco.mjtGeom.mjGEOM_BOX, size=list(PLATE_HALF), contype=4, conaffinity=4,
                          condim=6, priority=1, friction=[float(soft_feet), 0.02, 0.01], group=4, rgba=[0, 0, 0, 0])

    # IMU 가속도계 참값 (로봇 쪽 센서 모델의 입력). 센서는 동역학에 영향이 없다
    s.add_sensor(name="imu_accel", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER, objtype=mujoco.mjtObj.mjOBJ_SITE,
                 objname="imu")
    model = s.compile()
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
    terrain.applied[:] = terrain.target          # 초기 지형은 보류 없이 전부 반영
    terrain.write_all(model, hid)
    return model, hid
