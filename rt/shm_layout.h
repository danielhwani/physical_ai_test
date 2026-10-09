// 실시간 코어 <-> Python 관리 프로세스 공유 메모리 규약 (/dev/shm/<이름>).
// Python 쪽 정의는 sim/rt_link.py (ctypes). 두 정의가 같은지는 rt_core --layout 출력으로 시험한다 (conformance/test_rt_core.py).
//
// 쓰는 쪽이 하나인 칸(slot)은 seqlock으로 읽는다: 쓰는 쪽이 seq를 홀수로 올리고 쓴 뒤 짝수로 올린다.
// 읽는 쪽은 seq가 짝수이고 읽기 전후가 같을 때만 그 값을 쓴다. x86-64 메모리 순서(TSO) + 컴파일러 장벽으로 충분하다.
#pragma once
#include <cstdint>

namespace rtshm {

constexpr uint32_t MAGIC = 0x47324F52;   // "RO2G"
constexpr uint32_t VERSION = 1;
constexpr int NJ = 12;
constexpr int NQ = 19, NV = 18;           // Go2 (자유 관절 + 12)
constexpr int NOISE_DIM = 36;             // sensors/proprio.py ProprioSensors.NOISE_DIM
constexpr int NOISE_RING = 4096;          // 잡음 미리 만들기 (약 80 s)
constexpr int TRUTH_RING = 1024;          // 참값 기록 (약 20 s)

struct Config {                           // 관리 프로세스가 코어 시작 전에 쓴다
  int32_t mode;                           // 0 실시간 (시계), 1 lockstep (관리 프로세스가 한 스텝씩)
  int32_t decim;                          // 제어 주기당 물리 스텝
  int32_t foot_geom[4];
  int32_t terrain_geom;
  int32_t accel_adr;                      // imu_accel 센서 주소
  int32_t cpu;                            // 고정할 CPU (-1이면 안 함)
  int32_t priority;                       // SCHED_FIFO 우선순위 (0이면 일반)
  int64_t max_steps;                      // 이만큼 돌고 끝 (시나리오 duration)
  double control_dt;
  double torque_limit[NJ];
  double gyro_bias[3], accel_bias[3], yaw_drift;   // 센서 모델 (실행마다 정해진 값, Python이 계산)
  double hold_q[NJ], hold_kp[NJ], hold_kd[NJ];     // 명령이 아직 없을 때 (기본 자세)
  double qpos0[NQ], qvel0[NV];                     // 시작 상태 (지형 높이 맞춘 뒤)
};

struct Control {                          // 관리 프로세스 -> 코어 (실행 중 바꿀 수 있는 값)
  uint64_t start;                         // 1이면 시작
  uint64_t stop;                          // 1이면 끝
  uint64_t lockstep_target;               // lockstep: 이 스텝 번호까지 진행
  uint64_t link_mode;                     // 0 정상, 1 링크 두절 + 마지막 명령 유지, 2 링크 두절 + 감쇠
  double torque_scale;                    // 배터리 저하 (1.0 정상)
  double fault_gyro[3], fault_att[3];     // IMU 고장 (rad/s, rad)
};

struct LowState {                         // 코어 -> 보행 알고리즘 (최신 하나)
  uint64_t seq;
  uint64_t step;
  double t;
  double q[NJ], dq[NJ];
  double quat_xyzw[4], gyro[3], accel[3];
  double foot_force[4];
};

struct LowCmd {                           // 보행 알고리즘 -> 코어 (최신 하나)
  uint64_t seq;
  double t;                               // 이 명령을 계산한 로봇 상태 시각
  double q_des[NJ], dq_des[NJ], kp[NJ], kd[NJ], tau_ff[NJ];
};

struct NoiseEntry {                       // 표본 번호 step의 센서 잡음 (관리 프로세스가 미리 채움)
  uint64_t step;
  double v[NOISE_DIM];
};

struct TruthEntry {                       // 코어 -> 관리 프로세스 (판정자 쪽 참값, 기록, 타이밍)
  uint64_t step;
  double t;                               // 물리 진행 뒤 시각
  double qpos[NQ], qvel[NV];
  double ctrl[NJ];
  double energy;                          // 이 주기 소비 에너지 (J)
  double foot_force[4];                   // 이 주기 평균 발 수직력 (참값)
  double cmd_t;                           // 실행한 명령이 계산된 상태 시각 (-1: 기본 자세, -2: 감쇠)
  double wake_us, compute_us;             // 실시간: 깨어남 지연, 한 주기 계산 시간
};

struct Stats {
  uint64_t steps;                         // 끝낸 스텝 수
  uint64_t overruns;                      // 계산이 주기를 넘김
  uint64_t noise_underruns;               // 잡음이 아직 없어 0으로 대신함
  uint64_t done;                          // 코어 끝남
  uint64_t locked;                        // mlockall 성공
  uint64_t rt_ok;                         // SCHED_FIFO 성공
  double max_wake_us, max_compute_us;
};

struct Shm {
  uint32_t magic, version;
  uint64_t size;
  Config cfg;
  Control ctl;
  Stats stats;
  LowState state;
  LowCmd cmd;
  uint64_t noise_head;                    // 관리 프로세스가 채운 마지막 표본 번호 + 1
  NoiseEntry noise[NOISE_RING];
  uint64_t truth_head;                    // 코어가 쓴 참값 개수
  TruthEntry truth[TRUTH_RING];
};

}  // namespace rtshm
