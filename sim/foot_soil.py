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
  흙 표면: 지형 원천(Terrain Map Service의 반영된 지형). 흙은 지형 전체에 같다.

시나리오:
  foot_soil:
    preset: firm              # hard / firm / soft / mud (dis_console/protocol.py SOIL_PRESETS) 또는
    soil: {bekker_kphi: 1.0e7, bekker_kc: 0, bekker_n: 1.1, cohesion: 0, friction_deg: 30, janosi_shear: 0.01,
           elastic_k: 4.0e7, damping_r: 3.0e4}   # preset 값을 덮어쓴다
    plow_ratio: 1.0           # 쟁기질 저항 비율 (0이면 끔)
없으면 이전과 같은 단단한 접촉이다 (결과 비트 동일).

빠진 것 (SCM과 다른 점): 탄성 되튐(elastic_k), Janosi 전단 변위에 따른 미끄럼(마찰은 쿨롱), 점착력(cohesion), 경사·홈 벽의 옆 반력,
흙에서 발을 뽑을 때의 흡착, 밀린 흙이 쌓이는 것.
"""
import math

import mujoco
import numpy as np

from dis_console.protocol import SOIL_PRESETS

from .model_builder import PLATE_HALF, SOIL_PLATE

RINGS = 24                 # 지지력 적분 고리 수
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


class FootSoil:
    def __init__(self, sim, cfg):
        self.sim, self.soil = sim, soil_params(cfg)
        self.plow_ratio = float(cfg.get("plow_ratio", 1.0))
        m = sim.model
        self.feet = list(sim.foot_ids)
        names = sim.spec["robot"]["feet"]
        self.plates = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, SOIL_PLATE + n) for n in names]
        self.mocap = [int(m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, SOIL_PLATE + n)]) for n in names]
        self.r = float(m.geom_size[self.feet[0]][0])
        self.dt = float(m.opt.timestep)
        # 지지력 표 (깊이 -> 지지력, 면적): 단조 증가라 힘 -> 깊이도 보간으로
        self.z_tab = np.concatenate([np.linspace(0, 0.03, 301)[1:], np.linspace(0.03, 0.3, 271)[1:]])
        tab = [bearing(z, self.r, self.soil) for z in self.z_tab]
        self.f_tab, self.a_tab = np.array([t[0] for t in tab]), np.array([t[1] for t in tab])
        self.plastic = {}                     # (ix, iy) -> 원래 표면 아래 소성 침하 (m)
        n = len(self.feet)
        self.on = np.zeros(n, bool)           # 디딤 중
        self.zp = np.zeros(n)                 # 지금 디딘 자리의 침하 (받침판 윗면 = 표면 - zp)
        self.xy = np.zeros((n, 2))            # 디딘 자리
        self.fn = np.zeros(n)                 # 직전 물리 스텝의 발 수직력
        self.stances = []                     # 끝난 디딤 [(침하, 디딘 자리에서 미끄러진 거리)]
        self._f = np.zeros(6)
        self._jac = np.zeros((3, m.nv))
        self._minv = np.zeros((1, m.nv))
        self.plow_work = 0.0                  # 쟁기질로 흙에 준 일 (J)
        self.plow_work_up = 0.0               # 그중 위로 움직인 성분 (발을 빼며 흙을 들어 올린 일)
        self.top = np.zeros(n)                # 받침판 윗면 높이
        for i, g in enumerate(self.feet):    # 시작: 받침판을 발 아래 흙 표면에
            x, y, zc = sim.data.geom_xpos[g]
            self._place(i, x, y, zc - self.r + 0.001)

    # ---- 흙 표면 ----
    def surface(self, x, y):
        """원래 흙 표면 높이 (반영된 지형 원천의 쌍선형 보간)."""
        T = self.sim.terrain
        cx, cy = (x + T.half_x) / T.res, (y + T.half_y) / T.res
        c0 = min(max(int(math.floor(cx)), 0), T.ncol - 2)
        r0 = min(max(int(math.floor(cy)), 0), T.nrow - 2)
        fx, fy = min(max(cx - c0, 0.0), 1.0), min(max(cy - r0, 0.0), 1.0)
        a = T.applied
        return float((a[r0, c0] * (1 - fx) + a[r0, c0 + 1] * fx) * (1 - fy) + (a[r0 + 1, c0] * (1 - fx) + a[r0 + 1, c0 + 1] * fx) * fy)

    def _cell(self, x, y):
        return int(round(x / PLASTIC_CELL)), int(round(y / PLASTIC_CELL))

    def _stamp(self, x, y, zp):
        """발을 든 자리에 구 모양 구멍을 남긴다."""
        if zp <= 0:
            return
        a = math.sqrt(max(2 * self.r * zp - zp * zp, 0.0)) if zp < self.r else self.r
        k = int(math.ceil(a / PLASTIC_CELL))
        ix0, iy0 = self._cell(x, y)
        for ix in range(ix0 - k, ix0 + k + 1):
            for iy in range(iy0 - k, iy0 + k + 1):
                rho2 = (ix * PLASTIC_CELL - x) ** 2 + (iy * PLASTIC_CELL - y) ** 2
                if rho2 <= a * a:
                    depth = zp - (self.r - math.sqrt(self.r * self.r - rho2))
                    if depth > self.plastic.get((ix, iy), 0.0):
                        self.plastic[(ix, iy)] = depth

    def _foot_force(self, i):
        """발 i와 받침판 사이 접촉 수직력 (N)."""
        d, f = self.sim.data, 0.0
        g, p = self.feet[i], self.plates[i]
        for k in range(d.ncon):
            c = d.contact[k]
            if (c.geom1, c.geom2) in ((g, p), (p, g)):
                mujoco.mj_contactForce(self.sim.model, d, k, self._f)
                f += self._f[0]
        return f

    def _place(self, i, x, y, bottom):
        """받침판: 디딤 중이면 디딘 자리 높이, 아니면 발 아래 지금 자리의 흙(구멍 포함) 높이. 수평으로는 늘 발 아래.
        공중의 발이 아직 흙 표면 아래(깊은 구멍에서 나오는 중)면 발 바로 아래에 둔다 (발을 안쪽에서 치지 않게)."""
        if self.on[i]:
            top = self.surface(*self.xy[i]) - self.zp[i]
        else:
            top = min(self.surface(x, y) - self.plastic.get(self._cell(x, y), 0.0), bottom - 0.001)
        self.top[i] = top
        self.sim.data.mocap_pos[self.mocap[i]] = (x, y, top - PLATE_HALF[2])

    # ---- 물리 스텝마다 ----
    def update(self):
        """mj_step 뒤에 부른다: 이번 스텝의 접촉력(이번 스텝 접촉과 일치)으로 침하를 갱신하고, 다음 스텝의 받침판 높이를 정한다."""
        d = self.sim.data
        S = self.soil
        for i, g in enumerate(self.feet):
            x, y, zc = d.geom_xpos[g]
            fn = self._foot_force(i)
            self.fn[i] = fn
            if self.on[i]:
                top = self.surface(*self.xy[i]) - self.zp[i]
                if fn <= 0 and zc - self.r > top + LIFT_GAP:          # 발을 들었다: 구멍을 남긴다
                    self._stamp(*self.xy[i], self.zp[i])
                    self.stances.append((self.zp[i], float(np.hypot(x - self.xy[i][0], y - self.xy[i][1]))))
                    self.on[i] = False
            if not self.on[i] and fn > 0:                            # 디딤 시작: 받침판이 있던 깊이부터 (구멍 바닥 또는 발이 들어가 있던 깊이)
                self.on[i] = True
                self.xy[i] = (x, y)
                self.zp[i] = max(self.surface(x, y) - self.top[i], 0.0)
            if self.on[i] and fn > 0:                                # 소성 침하: 지지력보다 세게 누르면 받침판이 내려간다
                fb = float(np.interp(self.zp[i], self.z_tab, self.f_tab, left=0.0))
                if fn > fb:
                    z_eq = float(np.interp(fn, self.f_tab, self.z_tab))
                    area = max(float(np.interp(self.zp[i], self.z_tab, self.a_tab, left=0.0)), math.pi * (0.25 * self.r) ** 2)
                    rate = (fn - fb) / (S["damping_r"] * area)
                    self.zp[i] = min(self.zp[i] + rate * self.dt, z_eq)
            self._place(i, x, y, zc - self.r)
        if self.plow_ratio > 0:
            self._plow()

    def _plow(self):
        """흙 속을 옆이나 위로 지나가는 발의 저항. 다음 물리 스텝에 걸 힘을 qfrc_applied에 쓴다."""
        m, d, S = self.sim.model, self.sim.data, self.soil
        d.qfrc_applied[:] = 0.0
        for i, g in enumerate(self.feet):
            x, y, zc = d.geom_xpos[g]
            bottom = zc - self.r
            h = self.surface(x, y) - self.plastic.get(self._cell(x, y), 0.0) - bottom     # 흙 표면(구멍 포함) 아래 잠긴 깊이
            if h <= 0:
                continue
            point = np.array([x, y, bottom + 0.5 * min(h, 2 * self.r)])
            mujoco.mj_jac(m, d, self._jac, None, point, int(m.geom_bodyid[g]))
            v = self._jac @ d.qvel
            p = self.plow_ratio * (S["bekker_kc"] / (2 * self.r) + S["bekker_kphi"]) * h ** S["bekker_n"]
            vh = math.hypot(v[0], v[1])
            # 수평 (정면 활꼴)과 위 (흙에 덮인 윗면)를 따로: 각 방향 속도의 반대로, 각자의 면적으로
            for speed, u, area, up in ((vh, (v[0], v[1], 0.0), front_area(h, self.r), False),
                                       (v[2], (0.0, 0.0, 1.0), top_area(h, self.r), True)):
                if speed < 1e-6 or area <= 0:
                    continue
                u = np.array(u) / (speed if not up else 1.0)
                ju = u @ self._jac                                   # 그 방향의 발 끝 자코비안
                mujoco.mj_solveM(m, d, self._minv, ju[None, :])
                m_eff = 1.0 / max(float(ju @ self._minv[0]), 1e-9)   # 그 방향 유효 질량
                f = min(p * area, m_eff * speed / self.dt)          # 한 스텝에 멈추게 하는 힘을 넘지 않게
                d.qfrc_applied[:] -= f * ju
                self.plow_work += f * speed * self.dt
                if up:
                    self.plow_work_up += f * speed * self.dt

    def contacts(self):
        return self.fn > 0

    def mop(self):
        """디딤 통계 (끝난 디딤 기준): 침하(발자국 깊이)와 디딘 자리에서 미끄러진 거리."""
        plow = {"plow_work_j": round(self.plow_work, 2), "plow_work_up_j": round(self.plow_work_up, 2)}
        if not self.stances:
            return {"foot_sink_mean_m": None, "foot_sink_max_m": None, "foot_slip_mean_m": None, **plow}
        a = np.array(self.stances)
        return {"foot_sink_mean_m": round(float(a[:, 0].mean()), 4), "foot_sink_max_m": round(float(a[:, 0].max()), 4),
                "foot_slip_mean_m": round(float(a[:, 1].mean()), 4), **plow}
