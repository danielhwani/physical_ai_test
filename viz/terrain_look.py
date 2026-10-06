"""MuJoCo 화면의 지형 외형 (MuJoCo 렌더러 어댑터와 디버그 뷰어 공용).

체커 무늬 대신 흙 질감(여러 크기의 잡음) + 1 m 간격의 옅은 격자선을 쓰고, 빛을 비스듬히 비춰
발자국과 바퀴 자국의 굴곡이 음영으로 드러나게 한다. 물리에는 영향이 없다 (외형만).
"""
import mujoco
import numpy as np

SOIL_LIGHT = (0.60, 0.53, 0.44)
SOIL_DARK = (0.47, 0.41, 0.33)
GRID_DARKEN = 0.08          # 1 m 격자선 어둡게 하는 비율 (0이면 격자 없음)
TILE_PX = 256               # 1 m 칸 질감 해상도


def soil_tile(px=TILE_PX, seed=7):
    """1 m 칸 흙 질감 RGB(uint8, px x px x 3). 칸 경계가 이어지도록 주기적인 잡음을 쓴다."""
    rng = np.random.default_rng(seed)
    img = np.zeros((px, px))
    for cells, amp in ((4, 0.45), (16, 0.3), (64, 0.17), (256, 0.08)):
        g = rng.random((cells, cells))
        g = np.pad(g, ((0, 1), (0, 1)), mode="wrap")              # 주기 경계: 칸을 이어 붙여도 이음새가 없다
        x = np.linspace(0, cells, px, endpoint=False)
        i = x.astype(int)
        f = x - i
        f = f * f * (3 - 2 * f)
        top = g[i][:, i] * (1 - f)[None, :] + g[i][:, i + 1] * f[None, :]
        bot = g[i + 1][:, i] * (1 - f)[None, :] + g[i + 1][:, i + 1] * f[None, :]
        img += amp * (top * (1 - f)[:, None] + bot * f[:, None])
    img = (img - img.min()) / (img.max() - img.min())
    rgb = np.array(SOIL_DARK) + (np.array(SOIL_LIGHT) - np.array(SOIL_DARK)) * img[..., None]
    rgb[:2, :] *= 1 - GRID_DARKEN
    rgb[:, :2] *= 1 - GRID_DARKEN
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


def add_terrain_look(spec, half_size, material="terrain"):
    """MjSpec에 흙 재질, 하늘, 조명을 추가한다. 지형 geom에 material=material을 지정해 쓴다."""
    tile = soil_tile()
    tex = spec.add_texture(name=f"{material}_soil", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           width=tile.shape[1], height=tile.shape[0], nchannel=3)
    tex.data = tile.tobytes()
    mat = spec.add_material(name=material)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
    mat.texrepeat = [2 * half_size[0], 2 * half_size[1]]           # 질감 한 장 = 1 m
    mat.specular, mat.shininess, mat.reflectance = 0.05, 0.1, 0.0
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.55, 0.68, 0.82], rgb2=[0.16, 0.2, 0.26], width=256, height=256)
    spec.visual.headlight.ambient = [0.32, 0.32, 0.32]
    spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    spec.visual.headlight.specular = [0.0, 0.0, 0.0]
    # 비스듬한 해: 굴곡(발자국, 자국)이 음영으로 보이게
    spec.worldbody.add_light(pos=[0, 0, 10], dir=[-0.45, -0.3, -1.0], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                             diffuse=[0.75, 0.72, 0.66], specular=[0.05, 0.05, 0.05], castshadow=False)
    return mat
