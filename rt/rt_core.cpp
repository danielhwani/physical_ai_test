// Go2 실시간 코어 (문서 §11): 물리(MuJoCo) + 로봇 쪽 경계(센서 모델, PD 모터)를 고정 주기로 돈다.
//
//   rt_core <model.mjb> <공유 메모리 이름>      실행 (관리 프로세스 sim/rt_link.py가 띄운다)
//   rt_core --layout                           공유 메모리 규약의 크기와 위치 (Python 정의와 대조)
//
// 실시간 루프 규칙: 시작 전에 모든 메모리를 잡고(mlockall), 루프 안에서는 할당·입출력·잠금을 하지 않는다.
// 제어 주기마다
//   1. 로봇 상태 측정 (참값 + 센서 모델 + 관리 프로세스가 미리 만든 잡음) -> 공유 메모리 LowState
//   2. 실행할 관절 명령 고르기: 직전 주기까지의 상태로 계산된 최신 명령 (연결 지연 한 주기, Python 실행과 같다).
//      없으면 마지막 명령 유지 (처음이면 기본 자세). 링크 두절이면 마지막 명령 유지 또는 감쇠
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

void print_layout() {
#define F(field) std::printf("  \"%s\": [%zu, %zu],\n", #field, offsetof(Shm, field), sizeof(((Shm*)0)->field))
  std::printf("{\n");
  F(cfg); F(ctl); F(stats); F(state); F(cmd); F(noise_head); F(noise); F(truth_head); F(truth);
  F(cfg.qpos0); F(cfg.torque_limit); F(ctl.torque_scale); F(state.foot_force); F(cmd.tau_ff); F(truth[0].wake_us);
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
  const long period_ns = std::lround(cfg.control_dt * 1e9);
  LowCmd hold{}, applied{}, incoming{};
  for (int j = 0; j < NJ; j++) { hold.q_des[j] = cfg.hold_q[j]; hold.kp[j] = cfg.hold_kp[j]; hold.kd[j] = cfg.hold_kd[j]; }
  hold.t = -1.0;
  applied = hold;
  bool have_cmd = false;
  double prev_state_t = -1.0, foot_force[4] = {0, 0, 0, 0};
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  double next_ns = now_ns();

  for (int64_t k = 0; k < cfg.max_steps && !shm->ctl.stop; k++) {
    double wake_us = 0.0;
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
    write_slot(&shm->state, ls);

    // 2. 실행할 명령
    const uint64_t link = shm->ctl.link_mode;
    LowCmd cmd;
    if (link == 2) {                                      // 링크 두절 + 감쇠: 위치 게인 0, 속도 게인만
      cmd = LowCmd{};
      for (int j = 0; j < NJ; j++) { cmd.q_des[j] = d->qpos[7 + j]; cmd.kd[j] = 5.0; }
      cmd.t = -2.0;
    } else {
      if (link == 0 && read_slot(&shm->cmd, &incoming) && incoming.seq > 0 && incoming.t <= prev_state_t + 1e-9 &&
          (!have_cmd || incoming.t > applied.t)) {
        applied = incoming;
        have_cmd = true;
      }
      cmd = applied;                                      // 새 명령이 없으면 마지막 명령 유지 (처음이면 기본 자세)
    }
    prev_state_t = t;

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
      double e = 0.0;
      for (int j = 0; j < NJ; j++) e += std::fabs(d->ctrl[j] * d->qvel[6 + j]);
      energy += e * ts;
      for (int c = 0; c < d->ncon; c++) {
        const mjContact& con = d->contact[c];
        for (int i = 0; i < 4; i++) {
          int g = cfg.foot_geom[i];
          if ((con.geom1 == g || con.geom2 == g) && (con.geom1 == cfg.terrain_geom || con.geom2 == cfg.terrain_geom)) {
            mj_contactForce(m, d, c, out6);
            fn[i] += out6[0];
          }
        }
      }
    }
    for (int i = 0; i < 4; i++) foot_force[i] = fn[i] / cfg.decim;

    // 4. 참값·타이밍 기록
    double compute_us = (now_ns() - t0) / 1e3;
    volatile TruthEntry& te = shm->truth[k % TRUTH_RING];
    te.step = k;
    te.t = d->time;
    for (int i = 0; i < NQ; i++) te.qpos[i] = d->qpos[i];
    for (int i = 0; i < NV; i++) te.qvel[i] = d->qvel[i];
    for (int j = 0; j < NJ; j++) te.ctrl[j] = d->ctrl[j];
    te.energy = energy;
    for (int i = 0; i < 4; i++) te.foot_force[i] = foot_force[i];
    te.cmd_t = cmd.t;
    te.wake_us = wake_us;
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
  shm->stats.done = 1;
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
