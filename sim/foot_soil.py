"""무른 흙과 로봇 발의 쌍방향 상호작용 (선택 기능, 시나리오 foot_soil).

발은 단단한 지형 대신 발마다 둔 보이지 않는 "흙 받침판"(mocap 상자, sim/model_builder.py)에 닿는다. 발과 받침판의 접촉과
마찰은 MuJoCo가 푼다 (안정). 받침판 높이를 이 모듈이 흙 식으로 정한다. Chrono 발 대리 구(cosim/chrono_server.py)와 같은 생각이다.

  침하 (Bekker, Chrono SCM과 같은 식): 깊이 z까지 들어간 발 구가 받는 지지력 F_b(z) = 구 아래 각 점의 압력
      (kc/b + kphi)·s^n을 접촉 원 위로 적분 (s = 그 점의 침하). 발이 누르는 힘(MuJoCo 접촉 수직력)이 지금 깊이의 지지력보다 크면
      받침판이 내려간다 (소성 침하, 되돌아오지 않음). 내려가는 속도는 감쇠로 정한다: (F - F_b(z)) / (damping_r · 접촉 면적).
  구멍: 발을 들면 그 자리에 구 모양 구멍(소성 침하)이 남고, 다시 디디면 구멍 바닥부터 받친다 (1 cm 칸 소성 지도).
  마찰: 받침판 마찰계수 = tanφ (흙의 내부 마찰각). 진흙(20°)은 0.36, 다른 흙(30°)은 0.58 (단단한 지형 0.8).
  에너지: 발이 가라앉는 동안 한 일은 흙에 흡수된다 (받침판은 되올라오지 않는다) -> CoT에 드러난다.
  쟁기질: 흙 표면(구멍 포함) 아래로 잠긴 발이 옆이나 위로 움직이면 흙을 밀어낸다. 걸을 때처럼 발이 대각선으로 빠져나오면
      구멍 앞벽을 밀고(수평) 그 위 흙을 들어 올린다(위). 수평과 위를 따로, 각 방향 속도의 반대로 건다:
      크기 = 그 방향에서 본 잠긴 면적 × 그 깊이의 Bekker 압력 × plow_ratio (기본 1. 입상 매질 저항력 이론(RFT)처럼 깊이에 따라
      커지고, 수평 저항이 같은 깊이 수직 저항과 비슷한 크기). 면적: 수평은 잠긴 정면 활꼴, 위는 흙에 덮인 윗면(적도 위 잠긴 띠)을
      위에서 본 면적. 자기 구멍 안에서 곧게 위로 빼면 위에 흙이 없어 0이다. 아래로 가는 성분은 받침판이 맡는다.
      명시적 힘이라 한 스텝에 그 방향 발 끝 속도를 0으로 만드는 힘(유효 질량 × 속도 / dt)을 넘지 않게 자른다 (튕김 없이 안정).
  흙 표면: MuJoCo 지형 heightfield (단단한 접촉이 쓰는 것과 같은 지형). 흙은 지형 전체에 같다.

시나리오:
  foot_soil:
    preset: firm              # 기본 흙 (지역 밖): hard / firm / soft / mud (dis_console/protocol.py SOIL_PRESETS) 또는
    soil: {bekker_kphi: 1.0e7, bekker_kc: 0, bekker_n: 1.1, cohesion: 0, friction_deg: 30, janosi_shear: 0.01,
           elastic_k: 4.0e7, damping_r: 3.0e4}   # preset 값을 덮어쓴다
    plow_ratio: 1.0           # 쟁기질 저항 비율 (0이면 끔)
    regions:                  # 지역마다 다른 흙 (선택). 직사각형 [x0, y0, x1, y1] (m). 겹치면 목록에서 나중 지역이 이긴다
      - {name: mud_strip, region: [2.0, -10, 3.5, 10], preset: mud}          # preset/soil/plow_ratio 중 준 것만 기본 흙을 덮어쓴다
  디딤의 흙은 디딘 순간 발 위치로 정한다 (디디는 동안 그대로). 쟁기질은 발이 지금 지나가는 곳의 흙. 받침판 마찰계수는 그 흙의 tanφ로
  바꾼다. 구멍 지도는 흙과 상관없이 하나다 (구멍은 모양). 흙은 최대 MAX_SOILS종류, 지역은 최대 MAX_REGIONS개 (C++ 코어 규약).
없으면 이전과 같은 단단한 접촉이다 (결과 비트 동일).

빠진 것 (SCM과 다른 점): 탄성 되튐(elastic_k), Janosi 전단 변위에 따른 미끄럼(마찰은 쿨롱), 점착력(cohesion), 경사·홈 벽의 옆 반력,
흙에서 발을 뽑을 때의 흡착, 밀린 흙이 쌓이는 것.
"""
import bisect
import math

import mujoco
import numpy as np

from dis_console.protocol import SOIL_PRESETS

from .model_builder import PLATE_HALF, SOIL_PLATE

RINGS = 24                 # 지지력 적분 고리 수
MAX_SOILS, MAX_REGIONS = 4, 8   # 흙 종류(기본 포함), 지역 수 (rt/shm_layout.h와 같다)
PLASTIC_CELL = 0.01        # 소성 지도 칸 (m)
LIFT_GAP = 0.002           # 받침판에서 이만큼 떨어지면 발을 든 것으로 본다 (m)


def soil_params(cfg):
    soil = dict(SOIL_PRESETS[cfg["preset"]]) if cfg.get("preset") else {}
    soil.update(cfg.get("soil") or {})
    missing = [k for k in ("bekker_kphi", "bekker_kc", "bekker_n", "cohesion", "friction_deg", "janosi_shear", "elastic_k",
                           "damping_r") if k not in soil]
    if missing:
        raise ValueError(f"foot_soil: 흙 값이 없다 {missing} (preset 또는 soil)")
    return {k: float(v) for k, v in soil.items()}


def friction(cfg):
    return math.tan(math.radians(soil_params(cfg)["friction_deg"]))


def soil_list(cfg):
    """(흙 목록, 지역 목록). 흙 = (이름, 흙 값, plow_ratio), 0번이 기본 흙. 지역 = (x0, y0, x1, y1, 흙 번호).
    지역의 흙은 기본 흙에서 시작해 preset, soil, plow_ratio로 덮어쓴다. 같은 흙은 하나로 합친다."""
    base = soil_params(cfg)
    ratio = float(cfg.get("plow_ratio", 1.0))
    soils = [(cfg.get("preset") or "base", base, ratio)]
    regions = []
    for k, rg in enumerate(cfg.get("regions") or []):
        x0, y0, x1, y1 = (float(v) for v in rg["region"])
        if not (x0 < x1 and y0 < y1):
            raise ValueError(f"foot_soil.regions[{k}].region: [x0, y0, x1, y1]이고 x0 < x1, y0 < y1 ({rg['region']})")
        sp = dict(base)
        if rg.get("preset"):
            sp.update(SOIL_PRESETS[rg["preset"]])
        sp.update(rg.get("soil") or {})
        sp = {key: float(v) for key, v in sp.items()}
        r = float(rg.get("plow_ratio", ratio))
        idx = next((i for i, (_, s0, r0) in enumerate(soils) if s0 == sp and r0 == r), None)
        if idx is None:
            name = rg.get("name") or rg.get("preset") or f"region{k}"
            if name in [n for n, _, _ in soils]:
                name = f"{name}_{k}"
            soils.append((name, sp, r))
            idx = len(soils) - 1
        regions.append((x0, y0, x1, y1, idx))
    if len(soils) > MAX_SOILS or len(regions) > MAX_REGIONS:
        raise ValueError(f"foot_soil: 흙은 {MAX_SOILS}종류, 지역은 {MAX_REGIONS}개까지 (흙 {len(soils)}, 지역 {len(regions)})")
    return soils, regions


def bearing(z, r, soil):
    """깊이 z(구 바닥 기준)까지 들어간 반지름 r 구의 Bekker 지지력 (N)과 접촉 면적 (m²)."""
    if z <= 0:
        return 0.0, 0.0
    a = math.sqrt(max(2 * r * z - z * z, 0.0)) if z < r else r
    rho = (np.arange(RINGS) + 0.5) / RINGS * a
    s = np.maximum(z - (r - np.sqrt(r * r - rho * rho)), 0.0)
    dA = 2 * math.pi * rho * (a / RINGS)
    p = (soil["bekker_kc"] / (2 * a) + soil["bekker_kphi"]) * np.power(s, soil["bekker_n"])
    return float(np.sum(p * dA)), float(np.sum(dA))


def front_area(h, r):
    """흙 표면 아래로 h만큼 잠긴 반지름 r 구의 정면(수평 방향) 투영 면적 (원의 활꼴)."""
    h = min(max(h, 0.0), 2 * r)
    return r * r * math.acos((r - h) / r) - (r - h) * math.sqrt(max(2 * r * h - h * h, 0.0))


def top_area(h, r):
    """h만큼 잠긴 구에서 흙에 덮인 윗면(적도 위쪽 잠긴 띠)을 위에서 본 면적. 위로 움직일 때 밀어 올리는 흙."""
    t = min(max(h - r, 0.0), r)
    return math.pi * t * t


def interp(x, xs, ys):
    """선형 보간 (np.interp와 같은 뜻: 범위 밖은 끝값). C++ 코어(rt/rt_core.cpp soil_interp)와 같은 연산 순서."""
    n = len(xs)
    if x <= xs[0]:
        return ys[0]
    if x >= xs[n - 1]:
        return ys[n - 1]
    j = bisect.bisect_right(xs, x) - 1                      # xs[j] <= x < xs[j+1]
    return ys[j] + (ys[j + 1] - ys[j]) / (xs[j + 1] - xs[j]) * (x - xs[j])


class FootSoil:
    """흙 상태와 물리 스텝마다의 갱신. C++ 실시간 코어(rt/rt_core.cpp FootSoil)와 같은 알고리즘·연산 순서라 lockstep에서 비트 단위로 같다
    (그래서 numpy 행렬 곱 대신 순서를 정한 반복문, hypot 대신 sqrt, np.interp 대신 interp를 쓴다)."""

    def __init__(self, sim, cfg):
        self.sim = sim
        soils, self.regions = soil_list(cfg)
        self.region_names = [rg.get("name") or rg.get("preset") or f"region{k}" for k, rg in enumerate(cfg.get("regions") or [])]
        m = sim.model
        self.feet = list(sim.foot_ids)
        self.bodies = [int(m.geom_bodyid[g]) for g in self.feet]
        names = sim.spec["robot"]["feet"]
        self.plates = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, SOIL_PLATE + n) for n in names]
        self.mocap = [int(m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, SOIL_PLATE + n)]) for n in names]
        self.r = float(m.geom_size[self.feet[0]][0])
        self.dt = float(m.opt.timestep)
        # 흙 표면 = MuJoCo 지형 heightfield (코어가 가진 것과 같은 float32 값)
        self.hid, self.tgeom, self.tmocap = sim.hfield_id, sim.terrain_geom, int(sim.terrain_mocap)
        self.hnrow, self.hncol = int(m.hfield_nrow[self.hid]), int(m.hfield_ncol[self.hid])
        self.hadr = int(m.hfield_adr[self.hid])
        self.hsize = [float(v) for v in m.hfield_size[self.hid][:3]]
        self.gpos = [float(v) for v in m.geom_pos[self.tgeom]]
        # 흙마다: 지지력 표 (깊이 -> 지지력, 면적. 단조 증가라 힘 -> 깊이도 보간으로), 마찰, 감쇠, 쟁기질 계수. 코어에는 그대로 넘긴다
        z = [float(v) for v in np.concatenate([np.linspace(0, 0.03, 301)[1:], np.linspace(0.03, 0.3, 271)[1:]])]
        self.soils = []
        for name, sp, ratio in soils:
            tab = [bearing(v, self.r, sp) for v in z]
            self.soils.append({"name": name, "params": sp, "z_tab": z, "f_tab": [t[0] for t in tab], "a_tab": [t[1] for t in tab],
                               "friction": math.tan(math.radians(sp["friction_deg"])), "damping_r": sp["damping_r"],
                               "n": sp["bekker_n"], "plow_ratio": ratio,
                               "p_coef": ratio * (sp["bekker_kc"] / (2 * self.r) + sp["bekker_kphi"])})   # 쟁기질 압력 = p_coef·h^n
        self.soil = self.soils[0]["params"]                     # 기본 흙 (예전 이름)
        self.plow = any(sd["p_coef"] > 0 for sd in self.soils)
        self.area_min = math.pi * (0.25 * self.r) ** 2
        self.plastic = {}                     # (ix, iy) -> 원래 표면 아래 소성 침하 (m)
        n = len(self.feet)
        self.on = [False] * n                 # 디딤 중
        self.zp = [0.0] * n                   # 지금 디딘 자리의 침하 (받침판 윗면 = 표면 - zp)
        self.xy = [[0.0, 0.0] for _ in range(n)]   # 디딘 자리
        self.top = [0.0] * n                  # 받침판 윗면 높이
        self.fn = np.zeros(n)                 # 직전 물리 스텝의 발 수직력
        self.stance_n, self.sink_sum, self.sink_max, self.slip_sum = 0, 0.0, 0.0, 0.0   # 끝난 디딤 통계
        self.sidx = [0] * n                   # 지금 디딤의 흙 번호
        self.n_by = [0] * len(self.soils)     # 흙별 끝난 디딤 수, 침하 합
        self.sink_by = [0.0] * len(self.soils)
        self.plow_work = 0.0                  # 쟁기질로 흙에 준 일 (J)
        self.plow_work_up = 0.0               # 그중 위로 움직인 성분 (발을 빼며 흙을 들어 올린 일)
        self.stamps = []                      # 남긴 구멍 [(x, y, zp)] (화면 발자국)
        self._f = np.zeros(6)
        self._jac = np.zeros((3, m.nv))
        self._ju = np.zeros((1, m.nv))
        self._minv = np.zeros((1, m.nv))
        T = sim.terrain
        self.display = np.zeros((T.nrow, T.ncol), np.float32)   # 화면 전용: 지형 격자 점마다 발자국 구멍 깊이 (물리·센서는 쓰지 않음)
        for i, g in enumerate(self.feet):    # 시작: 받침판을 발 아래 흙 표면에
            x, y, zc = (float(v) for v in sim.data.geom_xpos[g])
            self._place(i, x, y, zc - self.r + 0.001)

    def region_name_at(self, x, y):
        """(x, y)의 지역 이름 (화면 정보판용), 지역 밖이면 기본 흙 이름."""
        name = self.soils[0]["name"]
        for (x0, y0, x1, y1, _), n in zip(self.regions, self.region_names):
            if x0 <= x <= x1 and y0 <= y <= y1:
                name = n
        return name

    def display_regions(self):
        """화면용 흙 지역 (매니페스트 soil_regions): 이름, 직사각형, 흙 이름, 색 (무를수록·미끄러울수록 짙은 갈색)."""
        out = []
        for (x0, y0, x1, y1, k), n in zip(self.regions, self.region_names):
            sp = self.soils[k]["params"]
            soft = min(max((math.log10(2.0e7) - math.log10(sp["bekker_kphi"])) / (math.log10(2.0e7) - math.log10(5.0e5)), 0.0), 1.0)
            wet = min(max((30.0 - sp["friction_deg"]) / 10.0, 0.0), 1.0)
            t = min(soft + 0.3 * wet, 1.0)
            rgb = [round(a + (b - a) * t, 3) for a, b in zip((0.66, 0.60, 0.50), (0.28, 0.20, 0.13))]
            out.append({"name": n, "region": [x0, y0, x1, y1], "soil": self.soils[k]["name"], "rgba": rgb + [1.0]})
        return out

    def soil_at(self, x, y):
        """(x, y)의 흙 번호: 지역 목록에서 나중에 맞는 지역, 없으면 기본 흙 0 (C++ 코어 soil_at과 같다)."""
        idx = 0
        for x0, y0, x1, y1, k in self.regions:
            if x0 <= x <= x1 and y0 <= y <= y1:
                idx = k
        return idx

    # ---- 흙 표면 ----
    def surface(self, x, y):
        """원래 흙 표면 높이: MuJoCo 지형 heightfield의 쌍선형 보간 (지형 창이면 창 위치 기준)."""
        d = self.sim.data
        hx, hy, hz = self.hsize
        cx, cy, cz = self.gpos
        if self.tmocap >= 0:
            mp = d.mocap_pos[self.tmocap]
            cx, cy, cz = float(mp[0]) + cx, float(mp[1]) + cy, float(mp[2]) + cz
        nr, nc = self.hnrow, self.hncol
        col = (x - cx + hx) / (2.0 * hx) * (nc - 1)
        row = (y - cy + hy) / (2.0 * hy) * (nr - 1)
        c0 = min(max(int(math.floor(col)), 0), nc - 2)
        r0 = min(max(int(math.floor(row)), 0), nr - 2)
        fx = min(max(col - c0, 0.0), 1.0)
        fy = min(max(row - r0, 0.0), 1.0)
        hf, a = self.sim.model.hfield_data, self.hadr + r0 * nc + c0
        h00, h01, h10, h11 = float(hf[a]), float(hf[a + 1]), float(hf[a + nc]), float(hf[a + nc + 1])
        h = (h00 * (1.0 - fx) + h01 * fx) * (1.0 - fy) + (h10 * (1.0 - fx) + h11 * fx) * fy
        return cz + h * hz

    def _cell(self, x, y):
        return round(x / PLASTIC_CELL), round(y / PLASTIC_CELL)     # 짝수 반올림 (C++ nearbyint와 같음)

    def _hole(self, x, y):
        return self.plastic.get(self._cell(x, y), 0.0)

    def _stamp_display(self, x, y, zp, out):
        """화면용 구멍: 지형 격자 점마다 그 칸(반 칸 둘레) 안에서 가장 깊은 구멍 깊이. 격자(예: 5 cm)가 발(지름 4.4 cm)보다
        거칠어도 구멍이 보이게 칸 단위로 넓혀 그린다."""
        if zp <= 0:
            return
        T, r = self.sim.terrain, self.r
        a = math.sqrt(max(2 * r * zp - zp * zp, 0.0)) if zp < r else r
        reach, half = a + 0.5 * T.res, 0.5 * T.res
        c0 = max(int(math.floor((x - reach + T.half_x) / T.res)), 0)
        c1 = min(int(math.ceil((x + reach + T.half_x) / T.res)), T.ncol - 1)
        r0 = max(int(math.floor((y - reach + T.half_y) / T.res)), 0)
        r1 = min(int(math.ceil((y + reach + T.half_y) / T.res)), T.nrow - 1)
        for rr in range(r0, r1 + 1):
            for cc in range(c0, c1 + 1):
                rho = max(math.hypot(T.xs[cc] - x, T.ys[rr] - y) - half, 0.0)
                if rho < a:
                    depth = zp - (r - math.sqrt(r * r - rho * rho))
                    if depth > out[rr, cc]:
                        out[rr, cc] = depth

    def display_depth(self):
        """화면 전용 발자국 깊이 격자 (남은 구멍 + 지금 디딘 발의 구멍). 물리와 센서(LiDAR)는 쓰지 않는다."""
        out = self.display.copy()
        for i in range(len(self.feet)):
            if self.on[i]:
                self._stamp_display(self.xy[i][0], self.xy[i][1], self.zp[i], out)
        return out

    def add_display_stamp(self, x, y, zp):
        self.stamps.append((x, y, zp))
        self._stamp_display(x, y, zp, self.display)

    def _stamp(self, x, y, zp):
        """발을 든 자리에 구 모양 구멍을 남긴다."""
        if zp <= 0:
            return
        self.add_display_stamp(x, y, zp)
        r = self.r
        a = math.sqrt(max(2 * r * zp - zp * zp, 0.0)) if zp < r else r
        k = int(math.ceil(a / PLASTIC_CELL))
        ix0, iy0 = self._cell(x, y)
        for ix in range(ix0 - k, ix0 + k + 1):
            for iy in range(iy0 - k, iy0 + k + 1):
                dx, dy = ix * PLASTIC_CELL - x, iy * PLASTIC_CELL - y
                rho2 = dx * dx + dy * dy
                if rho2 <= a * a:
                    depth = zp - (r - math.sqrt(r * r - rho2))
                    if depth > self.plastic.get((ix, iy), 0.0):
                        self.plastic[(ix, iy)] = depth

    def _foot_force(self, i):
        """발 i와 받침판 사이 접촉 수직력 (N)."""
        d, f = self.sim.data, 0.0
        g, p = self.feet[i], self.plates[i]
        for k in range(d.ncon):
            c = d.contact[k]
            if (c.geom1 == g and c.geom2 == p) or (c.geom1 == p and c.geom2 == g):
                mujoco.mj_contactForce(self.sim.model, d, k, self._f)
                f += float(self._f[0])
        return f

    def _place(self, i, x, y, bottom):
        """받침판: 디딤 중이면 디딘 자리 높이, 아니면 발 아래 지금 자리의 흙(구멍 포함) 높이. 수평으로는 늘 발 아래.
        공중의 발이 아직 흙 표면 아래(깊은 구멍에서 나오는 중)면 발 바로 아래에 둔다 (발을 안쪽에서 치지 않게)."""
        if self.on[i]:
            top = self.surface(self.xy[i][0], self.xy[i][1]) - self.zp[i]
            k = self.sidx[i]
        else:
            top = min(self.surface(x, y) - self._hole(x, y), bottom - 0.001)
            k = self.soil_at(x, y)
        self.top[i] = top
        self.sim.model.geom_friction[self.plates[i], 0] = self.soils[k]["friction"]   # 그 흙의 마찰
        self.sim.data.mocap_pos[self.mocap[i]] = (x, y, top - PLATE_HALF[2])

    # ---- 물리 스텝마다 ----
    def update(self):
        """mj_step 뒤에 부른다: 이번 스텝의 접촉력(이번 스텝 접촉과 일치)으로 침하를 갱신하고, 다음 스텝의 받침판 높이를 정한다."""
        d = self.sim.data
        for i, g in enumerate(self.feet):
            x, y, zc = (float(v) for v in d.geom_xpos[g])
            fn = self._foot_force(i)
            self.fn[i] = fn
            if self.on[i]:
                top = self.surface(self.xy[i][0], self.xy[i][1]) - self.zp[i]
                if fn <= 0 and zc - self.r > top + LIFT_GAP:          # 발을 들었다: 구멍을 남긴다
                    self._stamp(self.xy[i][0], self.xy[i][1], self.zp[i])
                    dx, dy = x - self.xy[i][0], y - self.xy[i][1]
                    self.stance_n += 1
                    self.sink_sum += self.zp[i]
                    self.sink_max = max(self.sink_max, self.zp[i])
                    self.slip_sum += math.sqrt(dx * dx + dy * dy)
                    self.n_by[self.sidx[i]] += 1
                    self.sink_by[self.sidx[i]] += self.zp[i]
                    self.on[i] = False
            if not self.on[i] and fn > 0:                            # 디딤 시작: 받침판이 있던 깊이부터 (구멍 바닥 또는 발이 들어가 있던 깊이)
                self.on[i] = True
                self.xy[i] = [x, y]
                self.sidx[i] = self.soil_at(x, y)
                self.zp[i] = max(self.surface(x, y) - self.top[i], 0.0)
            if self.on[i] and fn > 0:                                # 소성 침하: 지지력보다 세게 누르면 받침판이 내려간다
                zp, sd = self.zp[i], self.soils[self.sidx[i]]
                zt, ft, at = sd["z_tab"], sd["f_tab"], sd["a_tab"]
                fb = interp(zp, zt, ft) if zp > zt[0] else 0.0
                if fn > fb:
                    z_eq = interp(fn, ft, zt)
                    area = max(interp(zp, zt, at) if zp > zt[0] else 0.0, self.area_min)
                    rate = (fn - fb) / (sd["damping_r"] * area)
                    self.zp[i] = min(zp + rate * self.dt, z_eq)
            self._place(i, x, y, zc - self.r)
        if self.plow:
            self._plow()

    def _plow(self):
        """흙 속을 옆이나 위로 지나가는 발의 저항. 다음 물리 스텝에 걸 힘을 qfrc_applied에 쓴다."""
        m, d = self.sim.model, self.sim.data
        nv = m.nv
        qf, qv = d.qfrc_applied, d.qvel
        qf[:] = 0.0
        jac, ju, minv = self._jac, self._ju, self._minv
        for i, g in enumerate(self.feet):
            x, y, zc = (float(v) for v in d.geom_xpos[g])
            bottom = zc - self.r
            h = self.surface(x, y) - self._hole(x, y) - bottom      # 흙 표면(구멍 포함) 아래 잠긴 깊이
            sd = self.soils[self.soil_at(x, y)]                     # 지금 지나가는 곳의 흙
            if h <= 0 or sd["p_coef"] <= 0:
                continue
            point = np.array([x, y, bottom + 0.5 * min(h, 2 * self.r)])
            mujoco.mj_jac(m, d, jac, None, point, self.bodies[i])
            v = [0.0, 0.0, 0.0]
            for k in range(3):                                       # 발 끝 속도 (순서를 정한 덧셈)
                s = 0.0
                for j in range(nv):
                    s += float(jac[k, j]) * float(qv[j])
                v[k] = s
            p = sd["p_coef"] * h ** sd["n"]
            vh = math.sqrt(v[0] * v[0] + v[1] * v[1])
            # 수평 (정면 활꼴)과 위 (흙에 덮인 윗면)를 따로: 각 방향 속도의 반대로, 각자의 면적으로
            for up in (False, True):
                if up:
                    speed, u0, u1, u2, area = v[2], 0.0, 0.0, 1.0, top_area(h, self.r)
                else:
                    speed, area = vh, front_area(h, self.r)
                    if speed >= 1e-6:
                        u0, u1, u2 = v[0] / speed, v[1] / speed, 0.0
                if speed < 1e-6 or area <= 0:
                    continue
                for j in range(nv):                                  # 그 방향의 발 끝 자코비안
                    ju[0, j] = u0 * float(jac[0, j]) + u1 * float(jac[1, j]) + u2 * float(jac[2, j])
                mujoco.mj_solveM(m, d, minv, ju)
                s = 0.0
                for j in range(nv):
                    s += float(ju[0, j]) * float(minv[0, j])
                m_eff = 1.0 / max(s, 1e-9)                           # 그 방향 유효 질량
                f = min(p * area, m_eff * speed / self.dt)          # 한 스텝에 멈추게 하는 힘을 넘지 않게
                for j in range(nv):
                    qf[j] = float(qf[j]) - f * float(ju[0, j])
                self.plow_work += f * speed * self.dt
                if up:
                    self.plow_work_up += f * speed * self.dt

    def contacts(self):
        return self.fn > 0

    def mop(self):
        """디딤 통계 (끝난 디딤 기준): 침하(발자국 깊이)와 디딘 자리에서 미끄러진 거리. 합계로 세어 C++ 코어와 같다."""
        plow = {"plow_work_j": round(self.plow_work, 2), "plow_work_up_j": round(self.plow_work_up, 2)}
        if len(self.soils) > 1:                  # 흙별 디딤 수와 평균 침하
            plow["foot_sink_by_soil"] = {sd["name"]: {"stances": self.n_by[k],
                                                      "sink_mean_m": round(self.sink_by[k] / self.n_by[k], 4) if self.n_by[k] else None}
                                         for k, sd in enumerate(self.soils)}
        if not self.stance_n:
            return {"foot_sink_mean_m": None, "foot_sink_max_m": None, "foot_slip_mean_m": None, **plow}
        return {"foot_sink_mean_m": round(self.sink_sum / self.stance_n, 4), "foot_sink_max_m": round(self.sink_max, 4),
                "foot_slip_mean_m": round(self.slip_sum / self.stance_n, 4), **plow}
