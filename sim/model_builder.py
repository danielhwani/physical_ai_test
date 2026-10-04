"""MJCF 원본(Menagerie go2.xml)에 시나리오 지형을 붙여 MjModel을 만든다.

variant="mjx" 이면 specs/go2_mjx_override.yaml 의 adopt 항목을 원본에 적용한다.
(MJX 학습 환경이 쓰는 모델과 같은 물리 설정을 CPU MuJoCo에서 재현 -> 차이의 계층 분리, 문서 §13.2)
"""
from fnmatch import fnmatch
from pathlib import Path

import mujoco
import yaml

ROOT = Path(__file__).resolve().parent.parent
MJX_OVERRIDE = ROOT / "specs/go2_mjx_override.yaml"
COLLISION_GROUP = 3

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


def build_model(spec_cfg, terrain, variant="cpu"):
    s = robot_spec(spec_cfg, variant)

    # 사전 할당 heightfield (크기/해상도는 이후 변경 불가)
    hf = s.add_hfield(name="terrain", nrow=terrain.nrow, ncol=terrain.ncol, size=terrain.hfield_size)
    hf.userdata = [0.0] * (terrain.nrow * terrain.ncol)   # 실제 높이는 컴파일 후 write_all()로 기록
    s.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                  builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, width=256, height=256,
                  rgb1=[0.35, 0.33, 0.28], rgb2=[0.28, 0.26, 0.22])
    mat = s.add_material(name="terrain")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    mat.texrepeat = [40, 40]
    s.worldbody.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname="terrain",
                         pos=terrain.geom_pos, material="terrain")
    s.worldbody.add_light(pos=[0, 0, 5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                          castshadow=False)   # 저사양: 그림자 끔

    model = s.compile()
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
    terrain.applied[:] = terrain.target          # 초기 지형은 보류 없이 전부 반영
    terrain.write_all(model, hid)
    return model, hid
