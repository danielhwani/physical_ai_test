// Go2 실시간 코어 (문서 §11): 물리(MuJoCo) + 로봇 쪽 경계(센서 모델, PD 모터)를 고정 주기로 돈다.
//
//   rt_core <model.mjb> <공유 메모리 이름> [<지형 공유 메모리 이름>]   실행 (관리 프로세스 sim/rt_link.py가 띄운다)
//   rt_core --layout                           공유 메모리 규약의 크기와 위치 (Python 정의와 대조)
//
// 실시간 루프 규칙: 시작 전에 모든 메모리를 잡고(mlockall), 루프 안에서는 할당·입출력·잠금을 하지 않는다.
// 제어 주기마다
//   0. 지형이 바뀌었으면 반영 (heightfield 값, 지형 창 위치. 관리 프로세스가 sim.runner와 같은 규칙으로 계산)
//   1. 로봇 상태 측정 (참값 + 센서 모델 + 관리 프로세스가 미리 만든 잡음)
//   2. 실행할 관절 명령 고르기: 직전 주기까지의 상태로 계산된 최신 명령 (연결 지연 한 주기, Python 실행과 같다).
//      없으면 마지막 명령 유지 (처음이면 기본 자세). 링크 두절이면 마지막 명령 유지 또는 감쇠.
//      그다음에 1의 상태를 공유 메모리 LowState로 내보낸다 (먼저 내보내면 두뇌가 명령 슬롯을 덮어쓸 수 있다)
//   3. 물리 decim 스텝: 스텝마다 PD 토크 -> 토크 한계 -> mj_step (sim/robot_io.py와 같은 식, 같은 순서)
//   4. 참값·타이밍 기록 -> 공유 메모리 참값 링
// 같은 libmujoco를 쓰므로 같은 명령을 넣으면 Python 시뮬레이터와 물리가 비트 단위로 같다 (conformance/test_rt_core.py).
#include <mujoco/mujoco.h>

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <climits>
#include <vector>

#include "shm_layout.h"

using namespace rtshm;

namespace {

inline void barrier() { asm volatile("" ::: "memory"); }
inline double now_ns() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1e9 + t.tv_nsec;
}

// seqlock 쓰기/읽기
template <class T>
void write_slot(volatile T* dst, const T& src) {
  uint64_t s = dst->seq;
  dst->seq = s + 1;
  barrier();
  std::memcpy((void*)((char*)dst + sizeof(uint64_t)), (const char*)&src + sizeof(uint64_t), sizeof(T) - sizeof(uint64_t));
  barrier();
  dst->seq = s + 2;
}
template <class T>
bool read_slot(const volatile T* src, T* out) {
  for (int tries = 0; tries < 100; tries++) {
    uint64_t a = src->seq;
    barrier();
    if (a & 1) continue;
    std::memcpy((void*)out, (const void*)src, sizeof(T));
    barrier();
    if (src->seq == a) return true;
  }
  return false;
}

// sensors/proprio.py와 같은 식, 같은 순서
void quat_from_rpy(double r, double p, double y, double q[4]) {
  double cr = std::cos(r / 2), sr = std::sin(r / 2), cp = std::cos(p / 2), sp = std::sin(p / 2),
         cy = std::cos(y / 2), sy = std::sin(y / 2);
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}
void quat_mul(const double a[4], const double b[4], double o[4]) {
  double w1 = a[0], x1 = a[1], y1 = a[2], z1 = a[3], w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];
  o[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  o[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  o[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  o[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

// ---- 무른 흙 발 상호작용 (sim/foot_soil.py FootSoil과 같은 알고리즘·같은 연산 순서 -> lockstep 비트 동일) ----
// 소성 지도: 1 cm 칸 -> 원래 표면 아래 소성 침하. 실시간 루프에서 할당하지 않도록 시작 때 크기를 정한 해시 표 (선형 탐사).
struct PlasticMap {
  static constexpr size_t CAP = 1u << 18;                // 26만 칸 (구멍 하나 최대 49칸, 디딤 5천 번 넘게)
  std::vector<int64_t> keys;
  std::vector<double> vals;
  size_t used = 0;
  uint64_t dropped = 0;                                   // 가득 차서 버린 칸 (0이어야 Python과 같다)
  PlasticMap() : keys(CAP, INT64_MIN), vals(CAP, 0.0) {}
  static int64_t key(int64_t ix, int64_t iy) { return (int64_t)(((uint64_t)ix << 32) ^ (uint32_t)iy); }
  size_t slot(int64_t k) const { return (size_t)((uint64_t)k * 0x9E3779B97F4A7C15ULL >> 46) & (CAP - 1); }
  double get(int64_t ix, int64_t iy) const {
    int64_t k = key(ix, iy);
    for (size_t s = slot(k);; s = (s + 1) & (CAP - 1)) {
      if (keys[s] == k) return vals[s];
      if (keys[s] == INT64_MIN) return 0.0;
    }
  }
  void set_max(int64_t ix, int64_t iy, double v) {        // Python: if depth > plastic.get(...): plastic[...] = depth
    int64_t k = key(ix, iy);
    for (size_t s = slot(k);; s = (s + 1) & (CAP - 1)) {
      if (keys[s] == k) { if (v > vals[s]) vals[s] = v; return; }
      if (keys[s] == INT64_MIN) {
        if (!(v > 0.0)) return;
        if (used + 1 >= CAP * 3 / 4) { dropped++; return; }
        keys[s] = k; vals[s] = v; used++;
        return;
      }
    }
  }
};

double soil_interp(double x, const double* xs, const double* ys, int n) {   // sim/foot_soil.py interp
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  int j = (int)(std::upper_bound(xs, xs + n, x) - xs) - 1;
  return ys[j] + (ys[j + 1] - ys[j]) / (xs[j + 1] - xs[j]) * (x - xs[j]);
}

double front_area(double h, double r) {
  h = std::min(std::max(h, 0.0), 2 * r);
  return r * r * std::acos((r - h) / r) - (r - h) * std::sqrt(std::max(2 * r * h - h * h, 0.0));
}

double top_area(double h, double r) {
  double t = std::min(std::max(h - r, 0.0), r);
  return M_PI * t * t;
}

struct FootSoil {
  const SoilConfig& c;
  const Config& cfg;
  mjModel* m;                                             // 받침판 마찰계수를 흙에 맞춰 바꾼다
  mjData* d;
  volatile Shm* shm;
  bool on[4] = {false, false, false, false};
  double zp[4] = {0, 0, 0, 0}, xy[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, top[4] = {0, 0, 0, 0}, fn[4] = {0, 0, 0, 0};
  double plow_work = 0.0, plow_work_up = 0.0, sink_sum = 0.0, sink_max = 0.0, slip_sum = 0.0;
  uint64_t stance_n = 0;
  int sidx[4] = {0, 0, 0, 0};                             // 지금 디딤의 흙 번호
  double sink_by[MAX_SOILS] = {0, 0, 0, 0};
  uint64_t n_by[MAX_SOILS] = {0, 0, 0, 0};
  bool any_plow = false;
  PlasticMap plastic;
  std::vector<mjtNum> jac, ju, minv;
  double out6[6];

  FootSoil(const Config& cfg_, mjModel* m_, mjData* d_, volatile Shm* shm_)
      : c(cfg_.soil), cfg(cfg_), m(m_), d(d_), shm(shm_), jac(3 * m_->nv), ju(m_->nv), minv(m_->nv) {
    for (int k = 0; k < c.n_soil; k++) any_plow = any_plow || c.p_coef[k] > 0;
    for (int i = 0; i < 4; i++) {                         // 시작: 받침판을 발 아래 흙 표면에
      const double* p = d->geom_xpos + 3 * cfg.foot_geom[i];
      place(i, p[0], p[1], p[2] - c.r + 0.001);
    }
  }

  int soil_at(double x, double y) const {                 // 지역 목록에서 나중에 맞는 지역의 흙, 없으면 기본 흙 0
    int idx = 0;
    for (int k = 0; k < c.n_region; k++)
      if (c.region[k][0] <= x && x <= c.region[k][2] && c.region[k][1] <= y && y <= c.region[k][3]) idx = c.region_soil[k];
    return idx;
  }

  double surface(double x, double y) const {              // MuJoCo 지형 heightfield의 쌍선형 보간
    const int hid = cfg.hfield_id, tg = cfg.terrain_geom, tm = cfg.terrain_mocap;
    const double hx = m->hfield_size[4 * hid], hy = m->hfield_size[4 * hid + 1], hz = m->hfield_size[4 * hid + 2];
    double cx = m->geom_pos[3 * tg], cy = m->geom_pos[3 * tg + 1], cz = m->geom_pos[3 * tg + 2];
    if (tm >= 0) { cx = d->mocap_pos[3 * tm] + cx; cy = d->mocap_pos[3 * tm + 1] + cy; cz = d->mocap_pos[3 * tm + 2] + cz; }
    const int nr = m->hfield_nrow[hid], nc = m->hfield_ncol[hid];
    double col = (x - cx + hx) / (2.0 * hx) * (double)(nc - 1);
    double row = (y - cy + hy) / (2.0 * hy) * (double)(nr - 1);
    int c0 = std::min(std::max((int)std::floor(col), 0), nc - 2);
    int r0 = std::min(std::max((int)std::floor(row), 0), nr - 2);
    double fx = std::min(std::max(col - c0, 0.0), 1.0);
    double fy = std::min(std::max(row - r0, 0.0), 1.0);
    const float* hf = m->hfield_data + m->hfield_adr[hid] + r0 * nc + c0;
    double h00 = hf[0], h01 = hf[1], h10 = hf[nc], h11 = hf[nc + 1];
    double h = (h00 * (1.0 - fx) + h01 * fx) * (1.0 - fy) + (h10 * (1.0 - fx) + h11 * fx) * fy;
    return cz + h * hz;
  }

  int64_t cell(double v) const { return (int64_t)std::nearbyint(v / c.plastic_cell); }   // Python round (짝수 반올림)
  double hole(double x, double y) const { return plastic.get(cell(x), cell(y)); }

  void stamp(double x, double y, double z) {
    if (z <= 0) return;
    uint64_t h = shm->stamp_head;                          // 화면 발자국 (관리 프로세스가 읽는다)
    shm->stamp[h % STAMP_RING].x = x; shm->stamp[h % STAMP_RING].y = y; shm->stamp[h % STAMP_RING].zp = z;
    barrier();
    shm->stamp_head = h + 1;
    const double r = c.r;
    double a = z < r ? std::sqrt(std::max(2 * r * z - z * z, 0.0)) : r;
    int64_t k = (int64_t)std::ceil(a / c.plastic_cell);
    int64_t ix0 = cell(x), iy0 = cell(y);
    for (int64_t ix = ix0 - k; ix <= ix0 + k; ix++)
      for (int64_t iy = iy0 - k; iy <= iy0 + k; iy++) {
        double dx = (double)ix * c.plastic_cell - x, dy = (double)iy * c.plastic_cell - y;
        double rho2 = dx * dx + dy * dy;
        if (rho2 <= a * a) plastic.set_max(ix, iy, z - (r - std::sqrt(r * r - rho2)));
      }
  }

  double foot_force(int i) {
    double f = 0.0;
    const int g = cfg.foot_geom[i], p = c.plate_geom[i];
    for (int k = 0; k < d->ncon; k++) {
      const mjContact& con = d->contact[k];
      if ((con.geom1 == g && con.geom2 == p) || (con.geom1 == p && con.geom2 == g)) {
        mj_contactForce(m, d, k, out6);
        f += out6[0];
      }
    }
    return f;
  }

  void place(int i, double x, double y, double bottom) {
    double t;
    int k;
    if (on[i]) { t = surface(xy[i][0], xy[i][1]) - zp[i]; k = sidx[i]; }
    else { t = std::min(surface(x, y) - hole(x, y), bottom - 0.001); k = soil_at(x, y); }
    top[i] = t;
    m->geom_friction[3 * c.plate_geom[i]] = c.friction[k];   // 그 흙의 마찰
    double* mp = d->mocap_pos + 3 * c.plate_mocap[i];
    mp[0] = x; mp[1] = y; mp[2] = t - c.plate_half_z;
  }

  void update() {                                         // mj_step 뒤 (sim/foot_soil.py FootSoil.update)
    const double dt = m->opt.timestep;
    for (int i = 0; i < 4; i++) {
      const double* p = d->geom_xpos + 3 * cfg.foot_geom[i];
      const double x = p[0], y = p[1], zc = p[2];
      const double f = foot_force(i);
      fn[i] = f;
      if (on[i]) {
        double t = surface(xy[i][0], xy[i][1]) - zp[i];
        if (f <= 0 && zc - c.r > t + c.lift_gap) {        // 발을 들었다: 구멍을 남긴다
          stamp(xy[i][0], xy[i][1], zp[i]);
          double dx = x - xy[i][0], dy = y - xy[i][1];
          stance_n++;
          sink_sum += zp[i];
          sink_max = std::max(sink_max, zp[i]);
          slip_sum += std::sqrt(dx * dx + dy * dy);
          n_by[sidx[i]]++;
          sink_by[sidx[i]] += zp[i];
          on[i] = false;
        }
      }
      if (!on[i] && f > 0) {                               // 디딤 시작
        on[i] = true;
        xy[i][0] = x; xy[i][1] = y;
        sidx[i] = soil_at(x, y);
        zp[i] = std::max(surface(x, y) - top[i], 0.0);
      }
      if (on[i] && f > 0) {                                // 소성 침하
        const double z = zp[i];
        const int k = sidx[i];
        const double *zt = c.z_tab[k], *ft = c.f_tab[k], *at = c.a_tab[k];
        double fb = z > zt[0] ? soil_interp(z, zt, ft, c.n_tab) : 0.0;
        if (f > fb) {
          double z_eq = soil_interp(f, ft, zt, c.n_tab);
          double area = std::max(z > zt[0] ? soil_interp(z, zt, at, c.n_tab) : 0.0, c.area_min);
          double rate = (f - fb) / (c.damping_r[k] * area);
          zp[i] = std::min(z + rate * dt, z_eq);
        }
      }
      place(i, x, y, zc - c.r);
    }
    if (any_plow) plow();
  }

  void plow() {                                           // 흙 속을 옆이나 위로 지나가는 발의 저항 (다음 스텝 qfrc_applied)
    const int nv = m->nv;
    const double dt = m->opt.timestep;
    for (int j = 0; j < nv; j++) d->qfrc_applied[j] = 0.0;
    for (int i = 0; i < 4; i++) {
      const double* p = d->geom_xpos + 3 * cfg.foot_geom[i];
      const double x = p[0], y = p[1], zc = p[2];
      const double bottom = zc - c.r;
      const double h = surface(x, y) - hole(x, y) - bottom;
      const int k = soil_at(x, y);                        // 지금 지나가는 곳의 흙
      if (h <= 0 || c.p_coef[k] <= 0) continue;
      mjtNum point[3] = {x, y, bottom + 0.5 * std::min(h, 2 * c.r)};
      mj_jac(m, d, jac.data(), nullptr, point, c.foot_body[i]);
      double v[3];
      for (int k = 0; k < 3; k++) {
        double s = 0.0;
        for (int j = 0; j < nv; j++) s += jac[k * nv + j] * d->qvel[j];
        v[k] = s;
      }
      const double pr = c.p_coef[k] * std::pow(h, c.bekker_n[k]);
      const double vh = std::sqrt(v[0] * v[0] + v[1] * v[1]);
      for (int up = 0; up < 2; up++) {
        double speed, u0 = 0.0, u1 = 0.0, u2 = 0.0, area;
        if (up) { speed = v[2]; u2 = 1.0; area = top_area(h, c.r); }
        else {
          speed = vh; area = front_area(h, c.r);
          if (speed >= 1e-6) { u0 = v[0] / speed; u1 = v[1] / speed; }
        }
        if (speed < 1e-6 || area <= 0) continue;
        for (int j = 0; j < nv; j++) ju[j] = u0 * jac[j] + u1 * jac[nv + j] + u2 * jac[2 * nv + j];
        mj_solveM(m, d, minv.data(), ju.data(), 1);
        double s = 0.0;
        for (int j = 0; j < nv; j++) s += ju[j] * minv[j];
        const double m_eff = 1.0 / std::max(s, 1e-9);
        const double f = std::min(pr * area, m_eff * speed / dt);
        for (int j = 0; j < nv; j++) d->qfrc_applied[j] = d->qfrc_applied[j] - f * ju[j];
        plow_work += f * speed * dt;
        if (up) plow_work_up += f * speed * dt;
      }
    }
  }
};

void print_layout() {
#define F(field) std::printf("  \"%s\": [%zu, %zu],\n", #field, offsetof(Shm, field), sizeof(((Shm*)0)->field))
  std::printf("{\n");
  F(cfg); F(ctl); F(stats); F(state); F(cmd); F(op_head); F(op); F(noise_head); F(noise); F(truth_head); F(truth);
  F(stamp_head); F(stamp);
  F(cfg.qpos0); F(cfg.torque_limit); F(ctl.torque_scale); F(state.foot_force); F(cmd.tau_ff); F(truth[0].wake_us);
  F(ctl.fault_seq); F(truth[0].terrain_seq); F(cfg.soil); F(truth[0].stance_n);
  std::printf("  \"total\": [0, %zu]\n}\n", sizeof(Shm));
#undef F
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--layout") == 0) {
    print_layout();
    return 0;
  }
  if (argc < 3) {
    std::fprintf(stderr, "사용: rt_core <model.mjb> <공유 메모리 이름>\n");
    return 2;
  }
  int fd = shm_open(argv[2], O_RDWR, 0);
  if (fd < 0) { std::perror("shm_open"); return 1; }
  auto* shm = (volatile Shm*)mmap(nullptr, sizeof(Shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (shm == MAP_FAILED) { std::perror("mmap"); return 1; }
  if (shm->magic != MAGIC || shm->version != VERSION || shm->size != sizeof(Shm)) {
    std::fprintf(stderr, "공유 메모리 규약이 다르다 (magic %x version %u size %lu, 기대 %zu)\n", shm->magic, shm->version,
                 (unsigned long)shm->size, sizeof(Shm));
    return 1;
  }
  Config cfg;
  std::memcpy(&cfg, (const void*)&shm->cfg, sizeof(cfg));
  volatile TerrainHeader* terr = nullptr;                 // 지형 공유 메모리 (선택)
  if (argc >= 4) {
    int tfd = shm_open(argv[3], O_RDWR, 0);
    if (tfd < 0) { std::perror("shm_open terrain"); return 1; }
    TerrainHeader th;
    if (pread(tfd, &th, sizeof(th), 0) != (ssize_t)sizeof(th)) { std::perror("read terrain"); return 1; }
    size_t tsize = sizeof(TerrainHeader) + sizeof(float) * (size_t)th.nrow * th.ncol;
    terr = (volatile TerrainHeader*)mmap(nullptr, tsize, PROT_READ | PROT_WRITE, MAP_SHARED, tfd, 0);
    close(tfd);
    if (terr == MAP_FAILED) { std::perror("mmap terrain"); return 1; }
  }

  char err[1000] = "";
  mjModel* m = mj_loadModel(argv[1], nullptr);
  if (!m) { std::fprintf(stderr, "모델 읽기 실패: %s\n", argv[1]); return 1; }
  if (m->nq != NQ || m->nv != NV || m->nu != NJ) { std::fprintf(stderr, "모델 크기가 규약과 다르다\n"); return 1; }
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);
  for (int i = 0; i < NQ; i++) d->qpos[i] = cfg.qpos0[i];
  for (int i = 0; i < NV; i++) d->qvel[i] = cfg.qvel0[i];
  mj_forward(m, d);
  (void)err;
  if (terr && (terr->nrow != m->hfield_nrow[cfg.hfield_id] || terr->ncol != m->hfield_ncol[cfg.hfield_id])) {
    std::fprintf(stderr, "지형 공유 메모리 크기가 모델 heightfield와 다르다\n");
    return 1;
  }
  // 무른 흙 (선택): 소성 지도 등은 여기서 미리 잡는다 (실시간 루프에서 할당하지 않는다)
  FootSoil* soil = cfg.soil.on ? new FootSoil(cfg, m, d, shm) : nullptr;
  uint64_t terrain_seq = 0;
  const int hf_n = m->hfield_nrow[cfg.hfield_id] * m->hfield_ncol[cfg.hfield_id];
  float* hf_dst = m->hfield_data + m->hfield_adr[cfg.hfield_id];
  const float* hf_src = terr ? (const float*)((const char*)terr + sizeof(TerrainHeader)) : nullptr;

  // 실시간 준비: 메모리 잠금, CPU 고정, 우선순위 (실패해도 돌되 기록한다)
  shm->stats.locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
  if (cfg.cpu >= 0) {
    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(cfg.cpu, &cs);
    sched_setaffinity(0, sizeof(cs), &cs);
  }
  if (cfg.priority > 0) {
    sched_param sp{cfg.priority};
    shm->stats.rt_ok = sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
  }
  while (!shm->ctl.start && !shm->ctl.stop) usleep(1000);

  const double ts = m->opt.timestep;
  const int base_body = cfg.base_body;
  const long period_ns = std::lround(cfg.control_dt * 1e9);
  LowCmd hold{}, applied{}, incoming{};
  for (int j = 0; j < NJ; j++) { hold.q_des[j] = cfg.hold_q[j]; hold.kp[j] = cfg.hold_kp[j]; hold.kd[j] = cfg.hold_kd[j]; }
  hold.t = -1.0;
  applied = hold;
  bool have_cmd = false;
  double prev_state_t = -1.0, foot_force[4] = {0, 0, 0, 0};
  double min_cmd_t = -1e300;                              // 링크 두절 뒤에는 그 뒤 상태로 계산된 명령만 받는다
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  double next_ns = now_ns();

  for (int64_t k = 0; k < cfg.max_steps && !shm->ctl.stop; k++) {
    double wake_us = 0.0;
    if (cfg.mode == 0 && shm->ctl.freeze) {               // 일시정지 (DIS 콘솔): 풀릴 때까지 기다렸다가 지금부터 다시 주기를 센다
      while (shm->ctl.freeze && !shm->ctl.stop) usleep(1000);
      clock_gettime(CLOCK_MONOTONIC, &next);
      next_ns = now_ns();
    }
    if (shm->ctl.stop) break;
    if (cfg.mode == 0) {                                  // 실시간: 절대 시각까지 잔다
      next.tv_nsec += period_ns;
      while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
      next_ns += period_ns;
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
      wake_us = (now_ns() - next_ns) / 1e3;
    } else {                                              // lockstep: 관리 프로세스가 허락할 때까지
      while (shm->ctl.lockstep_target < (uint64_t)k && !shm->ctl.stop) {
        timespec w{0, 20000};
        nanosleep(&w, nullptr);
      }
      if (shm->ctl.stop) break;
    }
    double t0 = now_ns();
    const double t = d->time;
    const uint64_t fault_seq = shm->ctl.fault_seq;        // 고장 값보다 먼저 읽는다 (관리 프로세스는 값을 쓴 뒤 올린다): 새 판이면 값도 새 것
    barrier();

    // 0. 지형 반영 (seqlock: 복사 중 바뀌면 다음 주기에 다시)
    if (terr) {
      uint64_t s1 = terr->seq;
      if (!(s1 & 1) && s1 != terrain_seq && terr->apply_step <= k) {
        barrier();
        std::memcpy(hf_dst, hf_src, sizeof(float) * hf_n);
        double mp[3] = {terr->mocap_pos[0], terr->mocap_pos[1], terr->mocap_pos[2]};
        barrier();
        if (terr->seq == s1) {
          terrain_seq = s1;
          if (cfg.terrain_mocap >= 0)
            for (int a = 0; a < 3; a++) d->mocap_pos[3 * cfg.terrain_mocap + a] = mp[a];
        }
      }
    }

    // 1. 로봇 상태 측정
    const volatile NoiseEntry& ne = shm->noise[k % NOISE_RING];
    double z[NOISE_DIM] = {0};
    if (ne.step == (uint64_t)k) {
      for (int i = 0; i < NOISE_DIM; i++) z[i] = ne.v[i];
    } else {
      shm->stats.noise_underruns = shm->stats.noise_underruns + 1;
    }
    double fault_att[3], fault_gyro[3];
    for (int i = 0; i < 3; i++) { fault_att[i] = shm->ctl.fault_att[i]; fault_gyro[i] = shm->ctl.fault_gyro[i]; }
    double err_q[4], qm[4], qw[4] = {d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]};
    quat_from_rpy(z[0] + fault_att[0], z[1] + fault_att[1], cfg.yaw_drift * t + fault_att[2], err_q);
    quat_mul(err_q, qw, qm);
    double nrm = std::sqrt(qm[0] * qm[0] + qm[1] * qm[1] + qm[2] * qm[2] + qm[3] * qm[3]);
    for (double& v : qm) v /= nrm;
    LowState ls{};
    ls.step = k;
    ls.t = t;
    for (int j = 0; j < NJ; j++) { ls.q[j] = d->qpos[7 + j] + z[2 + j]; ls.dq[j] = d->qvel[6 + j] + z[14 + j]; }
    ls.quat_xyzw[0] = qm[1]; ls.quat_xyzw[1] = qm[2]; ls.quat_xyzw[2] = qm[3]; ls.quat_xyzw[3] = qm[0];
    for (int i = 0; i < 3; i++) {
      ls.gyro[i] = d->qvel[3 + i] + cfg.gyro_bias[i] + z[26 + i] + fault_gyro[i];
      ls.accel[i] = d->sensordata[cfg.accel_adr + i] + cfg.accel_bias[i] + z[29 + i];
    }
    for (int i = 0; i < 4; i++) ls.foot_force[i] = std::max(foot_force[i] + z[32 + i], 0.0);
    const uint64_t link = shm->ctl.link_mode;

    // 2. 실행할 명령 (상태를 내보내기 전에 고른다: 별도 프로세스 두뇌가 이번 상태로 곧바로 새 명령을 써서, 실행할 직전 상태의 명령을
    //    한 칸짜리 명령 슬롯에서 덮어쓰기 전에. 부하가 클 때 lockstep shm 두뇌가 한 스텝 어긋난 적이 있다)
    LowCmd cmd;
    if (link == 2) {                                      // 링크 두절 + 감쇠: 위치 게인 0, 속도 게인만
      cmd = LowCmd{};
      for (int j = 0; j < NJ; j++) { cmd.q_des[j] = d->qpos[7 + j]; cmd.kd[j] = 5.0; }
      cmd.t = -2.0;
      applied = cmd;                                      // 복구 직후 새 명령이 오기 전에는 직전에 실행한 것을 유지 (sim/runner.py와 같다)
      min_cmd_t = t;
    } else {
      if (link != 0) min_cmd_t = t;                       // 두절 중: 두절 전에 계산된 명령이 복구 뒤 실행되지 않게
      if (link == 0 && read_slot(&shm->cmd, &incoming) && incoming.seq > 0 && incoming.t <= prev_state_t + 1e-9 &&
          incoming.t > min_cmd_t && (!have_cmd || incoming.t > applied.t)) {
        applied = incoming;
        have_cmd = true;
      }
      cmd = applied;                                      // 새 명령이 없으면 마지막 명령 유지 (처음이면 기본 자세)
    }
    prev_state_t = t;
    if (link == 0) write_slot(&shm->state, ls);           // 1의 로봇 상태를 내보낸다 (링크 두절이면 보행 알고리즘에 가지 않는다)

    // 3. 물리 (sim/robot_io.py RobotIO.apply와 같은 식)
    double energy = 0.0, fn[4] = {0, 0, 0, 0}, out6[6];
    const double scale = shm->ctl.torque_scale;
    for (int s = 0; s < cfg.decim; s++) {
      for (int j = 0; j < NJ; j++) {
        double tau = cmd.kp[j] * (cmd.q_des[j] - d->qpos[7 + j]) + cmd.kd[j] * (cmd.dq_des[j] - d->qvel[6 + j]) + cmd.tau_ff[j];
        double lim = cfg.torque_limit[j] * scale;
        d->ctrl[j] = std::min(std::max(tau, -lim), lim);
      }
      mj_step(m, d);
      if (soil) soil->update();                           // 이번 스텝 접촉력으로 침하 갱신, 다음 스텝 받침판·쟁기질 (Python RobotIO와 같은 자리)
      double e = 0.0;
      for (int j = 0; j < NJ; j++) e += std::fabs(d->ctrl[j] * d->qvel[6 + j]);
      energy += e * ts;
      double f[4] = {0, 0, 0, 0};        // 스텝마다 발별 합을 먼저 내고 누적 (Python과 같은 덧셈 순서: 발 하나에 접촉 여러 개일 때)
      for (int c = 0; c < d->ncon; c++) {
        const mjContact& con = d->contact[c];
        for (int i = 0; i < 4; i++) {
          int g = cfg.foot_geom[i];
          if ((con.geom1 == g || con.geom2 == g) && (con.geom1 == cfg.terrain_geom || con.geom2 == cfg.terrain_geom)) {
            mj_contactForce(m, d, c, out6);
            f[i] += out6[0];
          }
        }
      }
      if (soil)
        for (int i = 0; i < 4; i++) f[i] += soil->fn[i];  // 무른 흙: 받침판 접촉력 (sim.runner foot_normal_forces와 같은 덧셈)
      for (int i = 0; i < 4; i++) fn[i] += f[i];
    }
    for (int i = 0; i < 4; i++) foot_force[i] = fn[i] / cfg.decim;

    // 4. 참값·타이밍 기록
    double compute_us = (now_ns() - t0) / 1e3;
    volatile TruthEntry& te = shm->truth[k % TRUTH_RING];
    te.step = k;
    te.t = d->time;
    for (int i = 0; i < NQ; i++) te.qpos[i] = d->qpos[i];
    for (int i = 0; i < NV; i++) te.qvel[i] = d->qvel[i];
    for (int j = 0; j < NJ; j++) { te.ctrl[j] = d->ctrl[j]; te.q_des[j] = cmd.q_des[j]; }
    te.energy = energy;
    for (int i = 0; i < 4; i++) te.foot_force[i] = foot_force[i];
    te.cmd_t = cmd.t;
    te.wake_us = wake_us;
    for (int i = 0; i < 9; i++) te.base_xmat[i] = d->xmat[9 * base_body + i];
    uint32_t bits = 0;
    for (int i = 0; i < 4; i++) {
      for (int a = 0; a < 3; a++) te.foot_pos[3 * i + a] = d->geom_xpos[3 * cfg.foot_geom[i] + a];
      for (int c = 0; c < d->ncon; c++)
        if (d->contact[c].geom1 == cfg.foot_geom[i] || d->contact[c].geom2 == cfg.foot_geom[i]) { bits |= 1u << i; break; }
    }
    te.contact_bits = bits;
    te.fault_seq = fault_seq;
    te.terrain_seq = terrain_seq;
    if (soil) {
      uint32_t sb = 0;
      for (int i = 0; i < 4; i++) {
        if (soil->on[i]) sb |= 1u << i;
        te.soil_zp[i] = soil->zp[i];
        te.soil_xy[2 * i] = soil->xy[i][0];
        te.soil_xy[2 * i + 1] = soil->xy[i][1];
      }
      te.soil_on = sb;
      te.plow_work = soil->plow_work; te.plow_work_up = soil->plow_work_up;
      te.sink_sum = soil->sink_sum; te.sink_max = soil->sink_max; te.slip_sum = soil->slip_sum;
      te.stance_n = soil->stance_n;
      for (int k = 0; k < MAX_SOILS; k++) { te.sink_by[k] = soil->sink_by[k]; te.n_by[k] = soil->n_by[k]; }
    }
    for (int b = 0; b < std::min((int)m->nbody, NBODY_MAX); b++) {
      for (int a = 0; a < 3; a++) te.body_xpos[3 * b + a] = d->xpos[3 * b + a];
      for (int a = 0; a < 4; a++) te.body_xquat[4 * b + a] = d->xquat[4 * b + a];
    }
    te.compute_us = compute_us;
    barrier();
    shm->truth_head = k + 1;
    if (cfg.mode == 0) {
      if (wake_us + compute_us > period_ns / 1e3) shm->stats.overruns = shm->stats.overruns + 1;
      shm->stats.max_wake_us = std::max((double)shm->stats.max_wake_us, wake_us);
      shm->stats.max_compute_us = std::max((double)shm->stats.max_compute_us, compute_us);
    }
    barrier();
    shm->stats.steps = k + 1;
  }
  if (soil && soil->plastic.dropped)
    std::fprintf(stderr, "무른 흙 소성 지도가 가득 차 %lu칸을 버렸다\n", (unsigned long)soil->plastic.dropped);
  delete soil;
  shm->stats.done = 1;
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
