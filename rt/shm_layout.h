// 실시간 코어 <-> Python 관리 프로세스 공유 메모리 규약 (/dev/shm/<이름>).
// Python 쪽 정의는 rt/shm.py (ctypes, 보행 알고리즘 쪽 control/rt_brain.py와 관리 프로세스 sim/rt_link.py가 함께 쓴다). 두 정의가 같은지는 rt_core --layout 출력으로 시험한다 (conformance/test_rt_core.py).
//
// 쓰는 쪽이 하나인 칸(slot)은 seqlock으로 읽는다: 쓰는 쪽이 seq를 홀수로 올리고 쓴 뒤 짝수로 올린다.
// 읽는 쪽은 seq가 짝수이고 읽기 전후가 같을 때만 그 값을 쓴다. x86-64 메모리 순서(TSO) + 컴파일러 장벽으로 충분하다.
#pragma once
#include <cstdint>

namespace rtshm {

constexpr uint32_t MAGIC = 0x47324F52;   // "RO2G"
constexpr uint32_t VERSION = 9;
constexpr int NJ = 12;
constexpr int NQ = 19, NV = 18;           // Go2 (자유 관절 + 12)
constexpr int NOISE_DIM = 36;             // sensors/proprio.py ProprioSensors.NOISE_DIM
constexpr int NOISE_RING = 4096;          // 잡음 미리 만들기 (약 80 s)
constexpr int TRUTH_RING = 1024;          // 참값 기록 (약 20 s)
constexpr int NBODY_MAX = 16;            // 바디 위치·자세를 넘길 수 있는 바디 수 (Go2 모델 14)
constexpr int SCAN_MAX_PTS = 32768;       // 스캔 공유 메모리의 최대 점 수
constexpr int OP_RING = 32;               // 운용자 이동 명령 (관리 프로세스 -> 별도 프로세스 두뇌)

struct Config {                           // 관리 프로세스가 코어 시작 전에 쓴다
  int32_t mode;                           // 0 실시간 (시계), 1 lockstep (관리 프로세스가 한 스텝씩)
  int32_t decim;                          // 제어 주기당 물리 스텝
  int32_t foot_geom[4];
  int32_t terrain_geom;
  int32_t accel_adr;                      // imu_accel 센서 주소
  int32_t base_body;                      // 몸통 바디 번호
  int32_t hfield_id;                      // 지형 heightfield 번호
  int32_t terrain_mocap;                  // 지형 창 mocap 번호 (-1: 창 없음)
  int32_t pad1;
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
  uint64_t brain_ready;                   // 별도 프로세스 두뇌가 공유 메모리를 열었다 (control/rt_brain.py)
  uint64_t freeze;                        // 1이면 일시정지 (실시간: 다음 주기 전에 멈추고, 풀리면 시계를 다시 맞춘다)
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

struct Estimate {                         // 별도 프로세스 두뇌 -> 관리 프로세스: 상태 추정 (기록과 화면 정렬용, 진단)
  uint64_t seq;
  double t;                               // 추정에 쓴 로봇 상태 시각
  double v_body[3], yaw, contacts[4], pos[3];
  double cmd[2];                          // 두뇌가 쓰는 명령 (가속 제한 뒤 vx, yaw_rate)
};

struct OpEntry {                          // 운용자 이동 명령: 상태 시각 t 이후 첫 로봇 상태부터 적용 (/cmd_vel_stamped와 같은 뜻)
  double t, vx, yaw_rate;
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
  double q_des[NJ];                       // 이 주기에 실행한 목표 관절 각도
  double energy;                          // 이 주기 소비 에너지 (J)
  double foot_force[4];                   // 이 주기 평균 발 수직력 (참값)
  double cmd_t;                           // 실행한 명령이 계산된 상태 시각 (-1: 기본 자세, -2: 감쇠)
  double wake_us, compute_us;             // 실시간: 깨어남 지연, 한 주기 계산 시간
  // 물리 스텝 직후 mjData에 남은 값 (sim.runner가 기록하는 값과 같게: 접촉과 위치는 마지막 스텝 시작 때 계산된 것)
  double base_xmat[9];
  double foot_pos[12];
  uint32_t contact_bits;                  // 발 i가 무엇이든 닿아 있으면 비트 i
  uint32_t pad;
  double body_xpos[NBODY_MAX * 3];        // 바디 위치·자세 (스텝 직후 값, 센서 렌더러가 쓴다)
  double body_xquat[NBODY_MAX * 4];       // w, x, y, z
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
  Estimate est;
  uint64_t op_head;                       // 관리 프로세스가 쓴 운용자 명령 개수
  OpEntry op[OP_RING];
  uint64_t noise_head;                    // 관리 프로세스가 채운 마지막 표본 번호 + 1
  NoiseEntry noise[NOISE_RING];
  uint64_t truth_head;                    // 코어가 쓴 참값 개수
  TruthEntry truth[TRUTH_RING];
};

// 지형 공유 메모리 (<이름>_terrain): 관리 프로세스가 지형 높이를 바꾸면 쓴다 (자국 이벤트, 5 Hz 반영, 지형 창 이동).
// 코어는 제어 주기 시작에 seq가 바뀌었으면 heightfield 값(MuJoCo 정규화 높이)과 지형 창 위치를 반영한다.
struct TerrainHeader {
  uint64_t seq;                           // seqlock
  int64_t apply_step;                     // 이 스텝부터 (lockstep에서는 정확히 이 스텝 시작에 반영된다)
  int32_t nrow, ncol;                     // heightfield 크기 (모델과 같아야 함)
  double mocap_pos[3];                    // 지형 창 위치
  // 뒤에 float data[nrow * ncol]
};

// 스캔 공유 메모리 (<이름>_scan): 관리 프로세스가 센서를 계산해 쓰고, 별도 프로세스 두뇌(control/rt_brain.py)가 읽는다.
struct ScanHeader {
  uint64_t seq;                           // seqlock
  double t;                               // 스캔 시각
  int32_t n;                              // 점 수
  int32_t pad;
  // 뒤에 float pts[SCAN_MAX_PTS * 3] (센서 좌표)
};

}  // namespace rtshm
