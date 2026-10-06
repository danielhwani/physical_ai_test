# 4족 보행로봇 가상 필드시험 환경 (최소 구성)

`PhysicalAI_가상필드시험_논의정리.docx`의 구조를 **CPU 전용, 저사양 PC**에 맞춰 축소해 구현한 출발점.
i5-9400 / 8 GB RAM / GPU 드라이버 없음(llvmpipe) 기준으로, 시뮬레이션 시간 모드에서 실시간보다 약 9배 빠르게 동작한다.

## 설치

```bash
conda activate mj_ros                      # Python 3.10, ROS2 Humble rclpy는 시스템 패키지 사용
pip install -r requirements.txt
./scripts/fetch_models.sh                  # Unitree Go2 모델 (MuJoCo Menagerie, 고정 커밋)
```

## 실행

```bash
conda activate mj_ros                      # 또는 ~/miniconda3/envs/mj_ros/bin/python
python -m sim.runner scenarios/flat_trot.yaml            # lockstep, 최대 속도, 화면 없음
python -m sim.runner scenarios/rough_rut.yaml --view     # MuJoCo 뷰어 (소프트웨어 렌더링이라 느릴 수 있음)
python -m sim.runner scenarios/flat_trot.yaml --realtime --rt   # 실시간 + SCHED_FIFO
python -m sim.runner scenarios/rough_rut.yaml --rviz     # RViz로 가시화 (MuJoCo는 물리엔진으로만 사용)
python -m sim.runner scenarios/hmmwv_follow.yaml --mjviz # MuJoCo 렌더러 어댑터로 가시화 (HMMWV 등 다른 엔진 바디도 실제 메쉬로)
#   RViz를 자동으로 띄우고 준비되면 시작한다. 끝난 뒤 RViz 창을 닫으면 종료, Ctrl+C로 중단해도 기록은 저장된다.
#   로봇 위 정보판(시각, 명령, 속도, 거리, CoT, 발 접촉, 이벤트, 지연 -> 종료 시 최종 MOP),
#   발 접촉(초록=접지), 명령 화살표(파랑), 경로(노랑). 같은 정보가 /sim/status(JSON)로도 나온다.
#   수동으로 띄우려면: --realtime --ros2 로 실행하고, python -m viz.rviz_adapter 를 따로 실행한 뒤,
#   conda를 끈 터미널에서 rviz2 -d config/go2.rviz --ros-args -p use_sim_time:=true
python -m sim.runner scenarios/rough_rut.yaml --policy policies/go2_trot_bc_est/card.yaml --view   # ONNX 정책
python train/imitate_trot.py                             # 모방학습 정책 재생성 (CPU 작업자 3개, 약 5분)
MUJOCO_GL=egl python -m sim.snapshot scenarios/rough_rut.yaml --t 5   # 오프스크린 스냅샷
python -m sim.runner scenarios/rough_rut.yaml --variant mjx   # MJX용 물리 설정 모델을 CPU에서 실행
python conformance/test_conformance.py                   # 결정성, IK-모델 일치
python conformance/test_specs.py                         # 관측 기준 벡터, 카드 관절 재배열, MJX 오버라이드 검증
python conformance/test_policy.py [card.yaml]            # 정책 카드/ONNX 입출력/결정성/추론 지연 (기본 go2_trot_bc)
python conformance/test_control.py                       # 로봇 경계: 알고리즘이 참값을 모름, 기구학, 다리 주행거리계, 재현
python conformance/test_lidar.py                         # LiDAR: 스트림만으로 재현, 지형 표면, 자기 몸 제외
python conformance/test_ros_equivalence.py               # ROS 노드 = 같은 프로세스(20 ms 지연 흉내) (ROS2 필요)
python conformance/test_render_stream.py                 # 렌더 스트림 계약 (스트림만으로 장면 재구성 = 시뮬레이터)
python conformance/compare_variants.py [--policy card.yaml]   # cpu vs mjx 설정의 MOP 분포 비교
python conformance/make_obs_reference.py                 # (관측 명세를 바꾼 경우) 기준 벡터 재생성
python conformance/make_policy_reference.py <card.yaml>  # (ONNX를 바꾼 경우) 정책 입출력 기준 재생성
```

시나리오 값은 파일을 고치지 않고 `--set 키=값`으로 바꿀 수 있다 (여러 번 가능, 없는 키는 오류, 바꾼 값은 기록 메타데이터에 남음).
`python -m sim.sweep <시나리오> --param <키> --values ...`는 값 하나를 바꿔 가며 MOP 비교표(`runs/sweep_*.csv`)를 만든다.
Chrono 시나리오의 MOP에는 지면 변형량(`deformed_cells`, `deform_mean_m`, `deform_max_m`)이 들어간다.

결과는 `runs/<시나리오>_<시각>/`에 `timeseries.parquet`(시계열, 메타데이터에 버전 조합)과 `summary.json`(MOP)으로 남는다.

## 구성과 문서 대응

| 경로 | 역할 | 문서 |
|---|---|---|
| `specs/go2_control.yaml` | 관절 순서, 기본 자세, 행동 처리 경로(scale/clip/PD/토크 한계), 관측 정의·스케일·허용 오차 | §12.2 |
| `specs/go2_mjx_override.yaml` | MJX용 모델 차이 목록. 받아들인 항목(adopt)과 거부한 항목(reject)을 이유와 함께 기록 | §12.2 |
| `control/observation.py` | 관측 기준 구현 (numpy, qpos/qvel 배치만 사용). JAX·C++ 구현이 맞춰야 할 기준 | §12.2 |
| `scenarios/*.yaml` | 지형 패치, 시간 이벤트(명령 변경, 지형 변형) | §10 (DIS 콘솔 이전 단계) |
| `sim/terrain_service.py` | Terrain Map Service: 절대 높이 원천, heightfield 사전 할당, 발 근처 갱신 보류, 5 Hz 반영 | §5.1, §5.3 |
| `sim/model_builder.py` | Menagerie Go2 MJCF + heightfield 결합 (MjSpec) | §4 |
| `control/` | **보행 알고리즘 (시뮬레이터를 모름)**. `node.py` 로봇 상태 -> 관절 명령, `estimator.py` 상태 추정(IMU + 다리 주행거리계), `kinematics.py` 다리 기구학, `trot.py` 규칙 기반 트롯, `onnx_policy.py` ONNX 정책, `interface.py` 명세 + 정책 카드, `clock.py` 명령 가속 제한·보행 위상, `ros_node.py` ROS2 노드 | §14 |
| `sim/robot_io.py` | 로봇 쪽 경계: 물리 참값 -> 센서 모델 -> 로봇 상태 메시지, 관절 명령 -> 모터(PD, 토크 한계) | §14 |
| `policies/go2_trot_bc_est/`, `policies/go2_trot_bc/` | 트롯 모방 정책 (추정값+지연 학습 / 참값 학습): `policy.onnx`, `card.yaml`(규약+출처), `train_log.json`, `io_reference.npz` | – |
| `train/imitate_trot.py` | DAgger 모방학습 (numpy MLP, ONNX 그래프 직접 구성, PyTorch 불필요) | §12 |
| `sim/runner.py` | 시나리오 실행, lockstep/실시간 두 모드, 넘어짐 판정, MOP 계산 | §9.2, §13 |
| `sim/adapters.py` | 좌표계 변환 단일 지점 (MuJoCo / ROS2 REP-103 / UE5) | §6.3, §8 |
| `sim/stream.py` | 중립 렌더 스트림 인코더: 장면 매니페스트, 지형 패치, 상태. 그리는 방법은 모름 | §8 |
| `sim/ros2_bridge.py` | 중립 스트림의 ROS2 전송: `/tf`, `/clock`, `/joint_states`, `/sim/base_twist`, `/sim/scene_manifest`, `/sim/terrain_patch`, `/sim/status` | §6.3 |
| `viz/` | 렌더러 쪽 (물리 시뮬레이션 `sim/` import 금지). `stream_decode.py` 공용 디코더, `rviz_adapter.py` 중립 스트림 -> RViz, `mujoco_adapter.py` 중립 스트림 -> 그리기 전용 MuJoCo 모델 -> 뷰어 | §6, §8 |
| `config/go2.rviz` | RViz 설정 (로봇, 지형, 정보, 카메라가 몸통 추적). 시뮬레이션 시각을 쓰므로 `use_sim_time:=true` 필요 | – |
| `docs/render_interface.md` | 렌더 인터페이스 계약 (메시지, 규약, 어댑터 추가 방법, 계약 시험) | §8 |
| `sim/chrono_link.py` | Chrono 연동 (러너 쪽): 프로세스 실행, 격자 정합 검사, 파이프라인 동기, 지형/차량 반영, 근접 판정 | §5 |
| `sensors/` | 엔진 무관 센서 모델 (`lidar.py`). 명세는 `specs/go2_sensors.yaml` | §8 |
| `viz/sensor_renderer.py`, `viz/sensor_node.py` | 센서 렌더러 (중립 스트림 -> 장면 -> 레이캐스트 -> 점군)와 ROS2 별도 프로세스 노드 | §8 |
| `cosim/` | 엔진 간 연결 규약 `wire.py`(표준 라이브러리), Chrono 서버 `chrono_server.py`(chrono 환경) | §5 |
| `sim/recorder.py` | Parquet 기록, Virtual 출처와 버전 조합 기록 | §4, §9.2, §13.3 |
| `conformance/` | 결정성, IK-모델 일치, 관측 기준 벡터(`reference/obs_reference.npz`), 오버라이드-상류 일치, 변형 간 MOP 비교 | §12.3, §13.2 |

## MJX 준비 사항

- **모델 차이**: Menagerie `go2_mjx.xml`은 원본과 솔버 설정(pyramidal cone, iterations 1), 관절 마찰, 발 반경(22→17.5 mm), **발 외 충돌 형상(전부 구로 단순화)**이 다르다. 이 항목은 `adopt`로 받아들였다.
  상류의 **무릎 토크 24 N·m(실기 45.43)**, 뒷다리 관절 범위 축소, 위치 서보 액추에이터는 실기 성능을 바꾸므로 `reject`했다.
  `test_specs.py`가 오버라이드 결과를 상류 파일과 대조하므로, Menagerie를 갱신했을 때 차이가 생기면 시험이 실패한다.
- **관측**: 실제 MJX 학습 환경을 만들 때 JAX 관측 함수는 `obs_reference.npz`의 입력(qpos, qvel, command, last_action)으로
  계산한 값이 `obs`와 `tolerance`(1e-5) 안에서 같아야 한다. float32 계산도 이 허용 오차를 통과하는 것을 확인했다.
- **차이의 계층**: `compare_variants.py` 결과(rough_rut, 시드 8개), 설정 단순화로 생긴 MOP 차이는 시드 간 편차보다 작다
  (전진 거리 +0.09 m, 표준편차 0.25~0.50 m). 단, 규칙 기반 보행기 기준이며 학습 정책은 단순화된 충돌 형상을 이용할 수 있으니 정책이 생기면 다시 비교한다.

## 로봇 경계: 보행 알고리즘은 참값을 모른다 (문서 §14)

```bash
python -m sim.runner scenarios/flat_trot.yaml                                 # 알고리즘을 같은 프로세스에서 (결정적)
python -m sim.runner scenarios/flat_trot.yaml --controller-node --rviz         # 알고리즘을 별도 ROS2 노드로
python conformance/test_control.py
```

```
[로봇 쪽 sim/]                                              [알고리즘 쪽 control/ (sim, MuJoCo를 모름)]
 MuJoCo 참값 -> 센서 모델(sensors/proprio.py) -> 로봇 상태 ──▶ 상태 추정(IMU 자세 + 다리 주행거리계)
   관절 엔코더 q, dq / IMU 자세(AHRS), 각속도, 가속도 / 발 힘        -> 트롯 보행기 또는 ONNX 정책
 관절 명령(목표각 + PD 게인 + 앞먹임 토크) -> 모터(토크 한계) ◀── 관절 명령
                                                           ◀── 이동 명령 (운용자: 지금은 시나리오 이벤트)
```

- **로봇 상태** (Unitree LowState를 본뜸): `t, q[12], dq[12], imu{quat(x,y,z,w), gyro, accel}, foot_force[4]`. 몸통 위치·속도 참값은 없다.
  센서 잡음·편향·방향 표류는 `specs/go2_sensors.yaml`의 `proprio` (난수 시드 = 시나리오 seed).
- **상태 추정**: 자세는 IMU 출력, 몸통 속도는 다리 주행거리계 (디딘 발의 **접촉점**이 움직이지 않는다고 보고 관절 속도로 역산).
  Go2 발은 구라서 딛는 동안 굴러, 발 중심 고정 가정은 약 10% 과소추정했다 (평지 편향 -0.038 -> -0.003 m/s로 보정).
  요철·바퀴 자국에서는 발이 미끄러져 오차가 커진다 (편향 +0.03~0.05 m/s). 실제 로봇의 다리 주행거리계와 같은 한계다.
- **관절 명령** (Unitree LowCmd를 본뜸): `q_des, dq_des, kp, kd, tau_ff`. 정책 카드의 PD 게인은 알고리즘이 명령에 실어 보내고,
  토크 한계는 로봇 쪽이 지킨다.
- **별도 노드** (`--controller-node`): `/robot/low_state`(JSON) + `/robot/imu`, `/robot/joint_states`(측정값) -> `control/ros_node.py`
  -> `/robot/low_cmd`(JSON). 운용자 명령은 `/cmd_vel`. 로봇 쪽은 마지막으로 받은 명령을 실행하고, 명령 지연을 MOP
  (`cmd_latency_ms_*`)로 기록한다. 이 PC에서 지연은 항상 20 ms(1주기)였고, 같은 프로세스에서 1주기 지연을 넣은 결과와 같다.
  (처음에는 수신 메시지를 한 주기에 하나씩만 처리해 명령이 밀려 넘어졌다 -> 쌓인 메시지를 모두 비우도록 수정)
- **MOP**: 추정 오차 `est_speed_rmse_mps`(전진 속도, 순간값 대비), `est_yaw_err_end_deg`. 판정자 쪽(러너)만 참값과 비교한다.

**참값(완벽한 인지) -> 추정값 -> 추정값 + 연결 지연 비교** (전진 거리 m / 측방 m)

| 시나리오 | 참값, 지연 없음 | 추정값, 지연 없음 | **추정값 + 20 ms 지연 (= ROS 배치, 현재 기본)** | 속도 추정 RMS 오차 |
|---|---|---|---|---|
| flat_trot | 3.90 / +0.75 | 3.92 / +0.81 | 3.90 / +0.78 | 0.05 m/s |
| rough_rut | 4.48 / -0.64 | 4.25 / +0.19 | 4.10 / -0.35 | 0.10 m/s |
| vehicle_crossing | 2.18 (자국에 걸림) | 2.25 (자국에 걸림) | 2.01 (**자국에서 넘어짐**) | 0.12 m/s |
| hmmwv_follow | 4.10 / +0.20 | 2.42 / +1.45 | 2.82 / +0.40 | 0.18 m/s |
| footprints | 4.77 / +0.04 | 4.78 / +0.05 | 4.73 / +0.06 | 0.05 m/s |
| 정책 go2_trot_bc, flat_trot | 4.11 | 4.16 / -0.70 | **2.33 / -4.35** | 0.05 m/s |
| 정책 go2_trot_bc, rough_rut | 4.49 / -1.24 | 2.72 / -2.79 | **0.55 / -3.64** | 0.10 m/s |
| 정책 go2_trot_bc_est, flat_trot | – | – | **4.24 / -1.27** | 0.04 m/s |
| 정책 go2_trot_bc_est, rough_rut | – | – | **3.86 / +0.26** | 0.11 m/s |

(속도 추정 오차는 추정값을 그 추정이 쓴 로봇 상태 시각의 참값과 비교한다. 별도 노드에서도 `/control/estimate`로 받아 같은 값이 나온다)

바퀴 자국 안을 걷는 `hmmwv_follow`처럼 발이 미끄러지는 곳에서 속도 추정이 틀어져 성능이 떨어진다.
트롯 보행기는 20 ms 지연에도 거의 그대로지만, 참값·지연 없이 모방학습한 정책 `go2_trot_bc`는 지연이 들어가자 크게 무너진다.

### 연결 지연: 학습·시험은 같은 프로세스로 빠르게, 결과는 ROS 배치와 같게

```bash
python -m sim.runner scenarios/flat_trot.yaml --policy policies/go2_trot_bc_est/card.yaml                       # 같은 프로세스 (지연 흉내, 결정적)
python -m sim.runner scenarios/flat_trot.yaml --policy policies/go2_trot_bc_est/card.yaml --controller-node --rviz   # ROS 노드 (실제 배치)
python conformance/test_ros_equivalence.py     # 두 실행이 같은 MOP를 내는지 (ROS2 필요, 약 1분)
```

- 별도 ROS 노드로 돌리면 명령은 항상 한 주기(20 ms) 늦게 실행된다 (상태 발행 -> 노드 계산 -> 다음 주기에 수신).
  같은 프로세스 실행도 `specs/go2_control.yaml`의 `link.latency_steps: 1`만큼 명령을 늦춰 실행한다.
  시나리오에서 `control_link: {latency_steps: N}`으로 바꿀 수 있다 (0 = 이전 동작).
- 이 지연을 넣으면 같은 프로세스 실행과 ROS 노드 실행의 MOP가 소수 셋째 자리까지 같다 (트롯, 두 정책 모두 확인).
  그래서 학습과 대량 시험은 빠르고 결정적인 같은 프로세스로 하고, 최종 판정 MOP는 `--controller-node`로 남긴다.
- 실기에서는 지연이 일정하지 않으므로, 학습 때는 지연을 0~2주기에서 무작위로 바꾼다 (`train/imitate_trot.py`).

## ONNX 정책 탑재

정책 = `policy.onnx` + `card.yaml`. 실행: `--policy <card.yaml>`. 카드에 없는 항목은 로봇 명세를 따른다.

| 외부 정책의 상황 | 필요한 작업 |
|---|---|
| 우리 명세대로 학습 | ONNX 교체 (카드는 복사) |
| 관측 순서/스케일, 관절 순서, PD, 기본 자세, 제어 주기가 다름 | 카드만 작성 (코드 수정 없음, `test_card_joint_reordering`으로 검증) |
| 없는 관측 항목 (높이맵, 관측 이력, 순환 신경망 상태 등) | `control/observation.py`의 `TERMS`에 함수 등록 + `control/node.py`에서 입력 추가 (로봇 상태 메시지나 센서 토픽에서 얻을 수 있는 값만) |

어느 경우든 탑재 후 `test_policy.py`(카드/ONNX 정합)와 `compare_variants.py --policy`(MOP)로 확인한다.

**정책 `go2_trot_bc_est` (현재 권장)**: 트롯 보행기를 DAgger로 모방 (11만 샘플, CPU 3 프로세스로 약 5분).
배치 구조와 같은 조건에서 학습했다: 관측은 상태 추정값(잡음 있는 센서 -> 추정기), 연결 지연은 에피소드마다 0~2주기 무작위.
교사(트롯 보행기)도 같은 추정값만 본다. 카드의 `provenance`에 관측 출처와 지연 범위가 기록된다.
- 20 ms 지연(ROS 배치)에서 평지 4.24 m, 요철 3.86 m로 넘어지지 않는다 (같은 조건의 `go2_trot_bc`는 2.33 m, 0.55 m).
- ROS 노드 실행 결과가 같은 프로세스 실행과 같다 (`test_ros_equivalence.py`).
- 여전히 관측에 방위 정보가 없어 측방 표류가 있다 (평지 -1.27 m).
- 추론 지연 p99 약 20 us (제어 주기 20 ms).

**이전 정책 `go2_trot_bc`**: 참값 관측, 지연 없이 학습. 탑재 경로 시험용으로 남겨 둔다.
- 지연 없이 참값으로 돌리면 트롯과 비슷하지만(4.11 / 4.49 m), 실제 배치 조건(추정값 + 20 ms)에서는 크게 무너진다 (위 표).
- MJX 설정 모델에서는 트롯(전진 +0.09 m)보다 영향이 크다: 전진 -0.74 m, CoT +30% (시드 8개, 약 1.8σ, 참값·지연 없음 조건).
  학습한 모델 설정에 맞춰진 정책일수록 설정 차이에 민감하다는 것이 §13.2 계층 비교를 정책마다 다시 해야 하는 이유다.

학습 시간 참고: 이 PC(메모리 7.7 GB, 스왑 없음)에서 작업자 5개로 돌리면 다른 프로그램과 함께 메모리가 모자라 시스템이 멈췄다.
작업자 하나가 약 0.55 GB를 쓰므로 `--workers 3`(기본값)을 권장한다.

## 저사양을 고려해 의도적으로 뺀 것

- **UE5 / 센서 시뮬레이션**: GPU 드라이버가 없어 불가. 렌더러는 포즈 스트림을 받는 얇은 클라이언트로 나중에 붙인다. 연동 계획과 추천 PC 제원은 [`docs/ue5_integration.md`](docs/ue5_integration.md).
- **MJX / 강화학습**: GPU 없이는 학습 처리량이 나오지 않음. 학습은 다른 장비에서 하고 ONNX 정책만 가져오는 구조로 간다.
- 그림자 끔, 뷰어 갱신 30 Hz 제한, 지형 20 m × 20 m @ 5 cm.

## 알려진 한계

- 트롯 보행기는 규칙 기반이라 요철에서 방위가 최대 약 18° 흔들리고 측방 이동이 남는다 (측방 스텝 없음).
- 실시간 루프가 Python이므로 지터 보장은 없다. 문서 §11대로 RT 루프는 추후 C++로 옮긴다.
- 지형 갱신은 발 근처 셀을 보류하므로 로봇이 홈 위에 서 있으면 반영이 늦어진다 (의도된 동작).
