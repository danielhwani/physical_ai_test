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
python conformance/test_perception.py                    # 지형 인지: 높이 지도, 디딜 곳, LiDAR 지도 정확도, 점군만으로 재현, height_scan 규약
python conformance/test_ros_equivalence.py               # ROS 노드 = 같은 프로세스(20 ms 지연 흉내) (ROS2 필요)
python conformance/test_dis_console.py                   # DIS 콘솔: 핸드셰이크, 고장 주입, 실행 제어, 링크 감시, 제어권·인증·원본 기록, HMMWV 투입·흙(Chrono), 기록으로 비트 단위 재실행
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
| `scenarios/*.yaml` | 지형 패치, 시간 이벤트(명령 변경, 지형 변형). DIS 콘솔 이벤트도 같은 경로로 들어간다 | §10 |
| `dis_console/`, `sim/dis_server.py` | DIS 시나리오 콘솔: 봉투(`envelope.py`, DIS_test에서 가져옴), 규약(`protocol.py`), 콘솔(`console.py`), 시뮬레이터 쪽 서버 | §10 |
| `sim/terrain_service.py` | Terrain Map Service: 절대 높이 원천, heightfield 사전 할당, 발 근처 갱신 보류, 5 Hz 반영 | §5.1, §5.3 |
| `sim/model_builder.py` | Menagerie Go2 MJCF + heightfield 결합 (MjSpec) | §4 |
| `control/` | **보행 알고리즘 (시뮬레이터를 모름)**. `node.py` 로봇 상태 -> 관절 명령, `estimator.py` 상태 추정(IMU + 다리 주행거리계), `kinematics.py` 다리 기구학, `trot.py` 규칙 기반 트롯, `terrain_map.py` LiDAR 높이 지도, `foothold.py` 디딜 곳 평가, `onnx_policy.py` ONNX 정책, `interface.py` 명세 + 정책 카드, `clock.py` 명령 가속 제한·보행 위상, `ros_node.py` ROS2 노드 | §14 |
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
| `sensors/` | 엔진 무관 센서 모델 (`lidar.py` LiDAR, `proprio.py` 관절·IMU·발 힘 잡음). 명세는 `specs/go2_sensors.yaml` | §8 |
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
  -> `/robot/low_cmd`(JSON). 운용자 명령은 `/cmd_vel`(원격 조종 도구, 즉시 적용)과 `/cmd_vel_stamped`(시나리오 이벤트, 시각을 붙여 그 시각 상태부터 적용). 로봇 쪽은 마지막으로 받은 명령을 실행하고, 명령 지연을 MOP
  (`cmd_latency_ms_*`)로 기록한다. 이 PC에서 지연은 항상 20 ms(1주기)였고, 같은 프로세스에서 1주기 지연을 넣은 결과와 같다.
  (처음에는 수신 메시지를 한 주기에 하나씩만 처리해 명령이 밀려 넘어졌다 -> 쌓인 메시지를 모두 비우도록 수정)
- **MOP**: 추정 오차 `est_speed_rmse_mps`(전진 속도, 순간값 대비), `est_yaw_err_end_deg`. 판정자 쪽(러너)만 참값과 비교한다.

**참값(완벽한 인지) -> 추정값 -> 추정값 + 연결 지연 비교** (전진 거리 m / 측방 m)

| 시나리오 | 참값, 지연 없음 | 추정값, 지연 없음 | **추정값 + 20 ms 지연 (= ROS 배치, 현재 기본)** | 속도 추정 RMS 오차 |
|---|---|---|---|---|
| flat_trot | 3.90 / +0.75 | 3.92 / +0.81 | 3.92 / +0.74 | 0.05 m/s |
| rough_rut | 4.48 / -0.64 | 4.25 / +0.19 | 4.03 / -0.12 | 0.10 m/s |
| vehicle_crossing | 2.18 (자국에 걸림) | 2.25 (자국에 걸림) | 2.10 (자국에 걸림. 이전 2.01 넘어짐) | 0.12 m/s |
| hmmwv_follow | 4.10 / +0.20 | 2.42 / +1.45 | 1.93, **12.3초에 넘어짐** (이전 2.82 / +0.40) | 0.18 m/s |
| footprints | 4.77 / +0.04 | 4.78 / +0.05 | 4.76 / +0.14 | 0.05 m/s |
| 정책 go2_trot_bc, flat_trot | 4.11 | 4.16 / -0.70 | **2.33 / -4.35** | 0.05 m/s |
| 정책 go2_trot_bc, rough_rut | 4.49 / -1.24 | 2.72 / -2.79 | **0.55 / -3.64** | 0.10 m/s |
| 정책 go2_trot_bc_est, flat_trot | – | – | **4.24 / -1.27** | 0.04 m/s |
| 정책 go2_trot_bc_est, rough_rut | – | – | **3.86 / +0.26** | 0.11 m/s |

트롯 줄의 마지막 열은 2026-10-08 보행기 수정(속도 보정 와인드업 방지, 다리 처짐 시 일어서기) 뒤 다시 잰 값이다 (시나리오 기본 시드 한 번).
앞의 두 열과 속도 추정 오차는 그 전 보행기로 잰 비교용 값이다. 정책 줄은 수정 뒤에도 같았다 (보행기를 쓰지 않음).

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

## 센서: LiDAR (문서 §8)

```bash
python -m sim.runner scenarios/rough_rut.yaml --sensor front_lidar --rviz        # 점군을 RViz로 (/sensors/front_lidar/points)
python -m sim.runner scenarios/vehicle_crossing.yaml --sensor front_lidar --rviz # HMMWV도 점군에 잡힘
python conformance/test_lidar.py
```

- **구조**: 센서 모델(`sensors/lidar.py`: 스캔 패턴, 장착 위치, 거리 잡음, 누락)은 엔진·ROS와 무관하고, 광선 거리만
  `raycast` 함수로 받는다. 레이캐스트는 `viz/sensor_renderer.py`가 **중립 스트림 메시지만으로** 재구성한 장면(지형 창 + 로봇 +
  다른 엔진 물체)에 MuJoCo를 기하 계산기로만 써서 한다. 시뮬레이터 상태를 직접 읽지 않으며, 같은 프로세스(StreamTap, 결정적)나
  별도 ROS 노드(`--sensor-node`)로 돈다. MuJoCo 물리 모델에 없는 Chrono HMMWV도 보인다.
- **명세**: `specs/go2_sensors.yaml`의 `front_lidar` (Go2 L1 참고: 10 Hz, 앞쪽 180도 x 24채널 = 2904 광선, 거리 잡음 1 cm).
  시나리오 `sensors: [front_lidar]` 또는 `--sensor front_lidar`로 켠다. 자기 몸은 레이캐스트에서 뺀다.
- **출력**: ROS2 `/sensors/front_lidar/points` (PointCloud2, 센서 좌표계, 필드 x y z ring label(0 지형, 1 다른 물체)),
  `/tf`에 world -> front_lidar. 기록 폴더에 `front_lidar.npz` (프레임별 점군과 센서 자세).
- **결정성**: 시뮬레이션 시간에 맞춰 같은 프로세스에서 스캔하고, 잡음은 (seed, 프레임 번호) 난수라 같은 실행은 같은 점군.
- **정확도 (시험)**: 잡음 없는 점이 heightfield 삼각형 면에서 1 mm 안 (요철 + 바퀴 자국). 수직 광선이 요철 heightfield를
  약 2% 빠져나가는 MuJoCo `mj_ray` 특성은 LiDAR의 비스듬한 광선에서는 나타나지 않았다 (17,280개 중 0개).
- **성능 (이 PC)**: 스캔 1회 약 90 ms (거의 수평인 광선이 지형 창을 가로지르는 비용). LiDAR를 켜면 시뮬레이션이 실시간 수준으로 느려진다.

## 지형 인지 보행: LiDAR 높이 지도로 디딜 곳 고르기

```bash
python -m sim.runner scenarios/vehicle_crossing.yaml --set controller.perception.sensor=front_lidar                   # 같은 프로세스
python -m sim.runner scenarios/vehicle_crossing.yaml --set controller.perception.sensor=front_lidar --controller-node --rviz  # ROS 노드
python -m sim.sweep scenarios/vehicle_crossing.yaml --param seed --values 1 2 3 4 5 --set controller.perception.sensor=front_lidar
python conformance/test_perception.py
```

규칙 기반 트롯에 지형 인지를 붙였다 (`control/terrain_map.py`, `control/foothold.py`, `control/trot.py`). 모두 알고리즘 쪽이며 참값을 모른다.
켜기: 시나리오 `controller.perception: {sensor: front_lidar}` 또는 `--set controller.perception.sensor=front_lidar` (센서는 자동으로 켜진다).
세부 값(`perception.foothold.*`, `perception.map.*`)도 `--set`으로 바꿀 수 있다. 끄면 이전 트롯과 비트 단위로 같다.

### 작동 구조

20 ms마다 한 번 도는 반복 루프다. 이 안에서 쓰는 값은 모두 로봇이 스스로 아는 값(측정값과 추정값)뿐이다.

```
[입력: 로봇 쪽]                [보행 알고리즘 control/: 20 ms마다]                        [출력]
로봇 상태(관절, IMU, 발 힘) ──▶ ① 상태 추정 ──▶ ② 지도 관리 ──▶ ③ 보행 시계
LiDAR 점군(센서 좌표, 10 Hz) ─────────────────▶   (높이 지도)         │
운용자 명령(속도, 회전) ──────────────────────────────────────────────▶│
                                                                      ▼
                           ④ 디딜 곳 고르기 (공중인 발) ──▶ ⑤ 발 궤적 ──▶ ⑥ 관절 각도 ──▶ 관절 명령
```

**① 상태 추정** (`estimator.py`)
- 자세는 IMU 출력, 속도는 다리 주행거리계(디딘 발은 미끄러지지 않는다고 보고 관절 속도로 몸통 속도를 역산)로 구한다.
- **주행거리 위치(odom)**는 출발점이 원점이고 시간이 지나며 조금씩 틀어진다. 높이 지도는 이 좌표계에 쌓는다.
  - 수평(x, y)은 그 속도를 적분한다. IMU 방향 표류(0.05°/s)만큼 옆으로 틀어진다 (평지 60초, 21 m 보행에 옆 0.6 m).
  - 높이(z)는 **디딘 발에 묶는다**: 발이 닿은 순간 접촉점 높이를 기억하고, 몸통 높이 = 기억한 접촉점 높이 - 다리 기구학 높이.
    처음에는 속도를 적분했는데 작은 치우침이 쌓여 같은 보행에 +0.28 m 표류했다 (RViz에서 지도가 떠올라 로봇이 지도 상자에 파묻혀 보였다).
    발에 묶은 뒤 -0.11 m이고 40초 이후로는 더 늘지 않았다. rut_crossing 8개 시드: 이전 6개, 바꾼 뒤 7개가 두 자국을 깨끗이 통과
    (각각 다른 시드 1~2개가 첫 자국 근처에서 막힘. 넘어짐 없음), rough_rut 5개 시드 차이 없음.

**② 높이 지도** (`terrain_map.py`, elevation map)
- LiDAR 점(센서 좌표)을 스캔 시점의 추정 자세로 odom 좌표에 옮기고, 4 cm 칸마다 떨어진 점들의 평균 높이를 저장한다.
- 새 값이 이전 값과 비슷하면 평균을 내고, 4 cm 넘게 다르면 지형이 바뀐 것(새 바퀴 자국, 차량 통과)으로 보고 새 값으로 바꾼다.
- 지도는 로봇 주변 4 m × 4 m이고 로봇을 따라 움직인다.
- 앞쪽 LiDAR는 발밑을 못 본다. 앞에서 미리 본 지면을 지도가 **기억**했다가, 로봇이 그 위를 걸어갈 때 쓴다.
- 스캔은 찍힌 시각의 추정 자세로 넣고 찍힌 **다음** 제어 주기부터 쓴다. 그래서 ROS로 받을 때 상태와 점군 도착 순서가 바뀌어도
  같은 프로세스 실행과 결과가 같다 (`test_ros_equivalence.py`). 별도 노드는 `/sensors/front_lidar/points`를 구독하며 TF의 센서 참 자세는 쓰지 않는다.

**③ 보행 시계** (`clock.py`, 기존 트롯)
- 대각선 다리 쌍(왼앞+오른뒤, 오른앞+왼뒤)이 0.36초 주기로 번갈아 움직인다. 한 주기 안에서 각 다리는 앞 절반은 딛고(지지), 뒤 절반은 공중(스윙)이다.
- 보폭은 명령 속도로 정하고, 실제 속도와의 차이로 보정한다.

**④ 디딜 곳 고르기** (`foothold.py`, 발이 공중에 있는 동안만)
1. 착지 위치 예측: 지금 몸통 위치 + 남은 스윙 시간 동안 몸통이 갈 거리 + 반 보폭 = 원래 디딜 위치.
2. 후보: 그 위치에서 앞뒤 ±6 cm 안의 7개 지점.
3. 후보마다 비용:
   - **모서리**: 1~2칸 옆과의 높이 차. **1.5 cm 이하는 무시**한다 (거리 잡음, 발자국).
   - **옮긴 거리**: 원래 위치에서 멀수록 조금 벌점.
   - **모르는 칸**: 벌점.
   - **깊이 자체는 벌점이 아니다**: 바퀴 자국 바닥은 좋은 자리, 벽 근처는 나쁜 자리다.
4. 비용이 가장 낮은 후보를 고른다. 스윙의 70%까지는 계속 다시 고르고 그 뒤로 고정한다. 직전 선택을 유지하면 비용을 조금 깎아
   선택이 오락가락하지 않게 한다.

**⑤ 발 궤적** (`trot.py`)
- 앞뒤: 발을 든 자리에서 고른 자리까지 부드럽게 옮긴다.
- 높이:
  - 각 발은 **고른 자리의 지면 높이**까지 내린다.
  - 몸통 높이는 **네 발 지면 높이의 평균**을 천천히 따라간다. 한 발이 자국 안에 있으면 그 다리를 더 뻗고 몸통은 거의 수평을 유지한다.
  - 올라갈 때는 스윙 앞 절반에 목표 높이까지 올리고, 내려갈 때는 뒤 절반에 내린다 (발끝이 벽에 걸리지 않게).
  - 두 끝 사이에 **양쪽 끝보다 2 cm 넘게 높은 턱**이 있으면 그만큼 발을 더 든다.
- 몸통 기울기: 다리 길이를 조절해 몸통을 수평으로 맞추는 기존 보정은 그대로다.

**⑥ 관절 각도**
- 다리 역기구학으로 발 목표 위치를 고관절·허벅지·무릎 각도로 바꾼다.
- 목표 각도와 PD 게인을 관절 명령(`/robot/low_cmd`)으로 보낸다. 로봇 쪽은 한 주기(20 ms) 뒤에 실행한다.

**설계 원칙**
- **참값을 모름**: 알고리즘은 실제 지형이나 실제 위치를 보지 않는다. 추정이 틀어지면 지도도 같이 틀어진다 (실제 로봇과 같은 조건).
- **끄면 원래대로**: 인지 기능을 끄면 모든 값이 원래 트롯과 비트 단위로 같다.
- **어느 쪽이든 같은 결과**: 같은 프로세스로 돌리든 ROS 노드로 돌리든 결과가 소수 셋째 자리까지 같다.

### RViz에서 보기 (`--rviz`)

- **Algorithm terrain map** (`/control/terrain_map`): 알고리즘이 LiDAR로 만든 **elevation map**. 디딜 곳은 이 지도로만 계산한다.
  - **타일 하나** = 지도의 4 cm × 4 cm 칸. LiDAR 점이 떨어진 칸만 그려지고, 아직 못 본 칸은 비어 있다.
  - **타일 높이** = 그 칸에 저장된 지면 높이 (점들의 평균).
  - **타일 색** = 절대 높이가 아니라 **몸통 기준 지면(네 발 지면 높이 평균)과의 차이**. 같으면 회색, 낮을수록 파랑, 높을수록 빨강
    (±8 cm에서 가장 진함). 그래서 바퀴 자국 바닥은 파랗게 보인다.
- **Footholds** (`/control/footholds`):
  - **주황 공**: 공중에 있는 발이 고른 착지 자리.
  - **하늘색 공**: 디딘 발의 자리.
  - **회색 공**: 지형을 몰랐다면 디뎠을 자리 (원래 디딜 위치).
  - **흰 선**: 회색 공에서 고른 자리까지. 벽을 피해 얼마나 옮겼는지를 보여 준다.
- 디딜 곳 후보를 평가할 때 쓰는 값이 바로 타일들의 높이다. 후보 칸과 1~2칸 옆 칸들의 높이 차로 모서리를 판단하고,
  고른 칸의 높이로 발을 얼마나 내릴지 정한다. 공이 파란 줄(자국) 경계를 피해 바닥 가운데나 자국 밖 평지에 놓이는 것이 그 결과다.
- **지면이 두 개 보인다**: **Terrain**은 시뮬레이터의 실제 지형(렌더 스트림으로 받은 참값, 알고리즘은 모름)이고,
  **Algorithm terrain map**은 알고리즘의 elevation map이다. 둘이 어긋나 보이면 그만큼이 위치 추정 표류다.
  알고리즘 표시는 알고리즘 좌표계 `odom`에 있다. 러너가 `world -> odom`을 시작 위치로 맞춘 뒤, **계속 로봇에 맞춰 다시 정렬한다**
  (실제 몸통 위치 - 추정 위치를 1초 저역통과, 평행이동만, 10 Hz, 표시 전용). 그래서 로봇 주변 지도는 실제 지면에 붙어 보이고,
  오래전에 쌓은 먼 칸에만 그동안의 추정 표류가 남아 보인다. 한 번만 맞추던 때는 오래 걸으면 지도 전체가 떠오르거나 옆으로 밀렸다.
- **갱신 속도**: 화면의 지도는 0.5초마다, 디딜 곳은 0.1초마다 갱신한다. 알고리즘 안의 지도는 스캔마다(0.1초) 바뀌므로 화면 지도가 조금 늦게 보일 수 있다.
- 같은 프로세스 실행은 러너가, `--controller-node`는 알고리즘 노드가 발행한다 (명령을 보낸 뒤 발행하므로 명령 지연에 영향 없음).

**MOP 추가**: `edge_touchdowns` = 착지 중 참 지형 모서리(발 주변 ±4 cm 높이 차 > 2 cm)에 디딘 횟수 (판정자 쪽 참값으로 계산).

**결과** (시드별 센서 잡음을 바꿔 여러 번. 한 번 실행은 성공/실패가 우연히 갈려 판단에 쓰지 않았다)

| 시나리오 (시드 수) | 지형 모름 | 지형 인지 |
|---|---|---|
| vehicle_crossing (5) | **3번 넘어짐**, 2번 자국에 걸림 (전진 1.9~2.3 m), 모서리 착지 8~21 | **0번**, 4번 두 자국 통과 (약 5.05 m, 모서리 착지 0~2), 1번 첫 자국에 걸림 (2.31 m) |
| rut_crossing, Chrono 없이 만든 상자형 자국 5 cm (5) | 0번 넘어짐, 모두 첫 자국에서 막힘 (1.9~2.1 m) | **0번**, 4번 통과 (4.86~5.09 m), 1번 두 번째 자국 앞 (3.34 m) |
| vehicle_crossing_soft, 자국 15 cm (3) | 3번 넘어짐 | 3번 넘어짐 |
| hmmwv_follow, 자국 안을 따라 걷기 (5) | 2번 넘어짐 | 3번 넘어짐 |
| flat_trot (3) | 전진 3.88~3.94, CoT 2.40~2.44 | 3.88~3.90, CoT 2.49~2.50 (차이 없음) |
| rough_rut (3) | 전진 4.01~4.09, CoT 3.56~3.98 | 3.91~4.41, CoT 3.31~4.28 (차이 없음) |
| footprints (3) | 전진 4.78~4.81 | 4.72~4.75 (차이 없음) |

2026-10-08 보행기 수정(속도 보정 와인드업 방지, 다리 처짐 시 일어서기) 뒤 다시 잰 값이다. 이전 값과 비교하면 지형 모름 트롯은 넘어짐이 줄었다
(vehicle_crossing 4 -> 3번, rut_crossing 2 -> 0번, hmmwv_follow 4 -> 2번: 자국에 걸려 있는 동안 적분이 쌓였다가 한꺼번에 밀어붙이던 것이 없어져
넘어지는 대신 걸린 채 멈춘다). 지형 인지는 대부분 같고, vehicle_crossing 시드 3이 첫 자국에 걸렸으며(이전 5개 모두 통과), 15 cm 자국 넘어짐이 2 -> 3번이다.
시드 3이 약한 것은 10-07 추정기 높이 수정 때 rut_crossing에서도 보였다. 시드 3~5개라 1번 차이는 우연일 수 있다.

조정하며 확인한 것 (모두 여러 시드로 비교):
- **1.5 cm 이하 높이 차는 모서리로 보지 않는다**: 없으면 거리 잡음(1 cm)과 발자국에 반응해 vehicle_crossing에서도 5번 중 4번 넘어졌다.
- **턱 넘기는 두 끝보다 2 cm 넘게 높을 때만**: 없으면 요철 지도 잡음 때문에 매번 발을 더 들어 rough_rut CoT가 15~40% 늘었다.
- **옆 옮김(±4 cm)은 기본 끔** (`perception.foothold.max_side=0.04`로 켬): 나란한 자국(hmmwv_follow)에서는 조금 나았지만(끝까지 간 시드 0 -> 2개),
  가로 자국(vehicle_crossing)에서 5번 중 1~2번 넘어졌다. 지도의 자국 벽 칸이 들쭉날쭉해 다리를 쓸데없이 옆으로 흔드는 것으로 보인다.
- **디딘 발 아래 지도 높이로 몸통 높이 고정**: 시험했으나 발이 지면에 몇 mm 파고드는 만큼이 스캔마다 누적되어 표류가 오히려 늘었다 -> 뺐다.
- **발마다 높이 맞추기를 작은 높이 차에서 줄이기**: 몸통 기준 높이와 어긋나 vehicle_crossing 넘어짐이 늘었다 -> 기본 끔.

**한계**: 자국 안을 따라 걷기(hmmwv_follow)와 15 cm 자국은 아직 자주 넘어진다. 트롯은 걸음 주기가 고정이고 옆 방향 위치 제어가 없어
자국 벽 쪽으로 흘러가는 것을 막지 못한다. 이 경우는 걸음 조절(멈춤, 짧은 걸음)이나 학습 정책이 필요할 것으로 보인다.
LiDAR를 켜면 같은 프로세스 실행이 약 5배 느려진다 (스캔 계산).

### 지형 인지 정책의 관측 규약 (`observation_perceptive`, 명세 0.5.0)

학습 정책(모방이든 강화학습이든)이 지형을 입력으로 받을 때의 규약이다. `specs/go2_control.yaml`에 있다.
- 256차원 = 기존 47개(`observation`과 같은 순서, 같은 값) + `height_scan` 209개.
- `height_scan`: 몸통 기준 격자(앞뒤 -0.30~+0.60 m × 옆 ±0.25 m, 5 cm 간격, x 바깥 반복·y 안쪽 반복, 오름차순)를
  몸통 방향(yaw)으로 돌린 점들의 지면 높이 - 몸통 높이 + 0.26 m. ±0.30 m에서 자르고 5배. 모르는 칸은 0.
  평지에 서 있으면 모두 약 0이다 (0.26 m는 트롯이 평지에서 실제로 서는 몸통 높이).
- **학습할 때**(시뮬레이터)는 참 지형에서, **배치할 때**(알고리즘 노드)는 LiDAR 높이 지도에서 같은 격자를 뽑는다.
  두 값이 같은 뜻인지 `test_height_scan_input_matches_true_terrain`이 확인한다 (걷는 동안 오차 폭 4 cm 이내, 격자점 70% 이상 앎).
- 배치: 정책 카드의 `observation`에 `height_scan`이 있으면 알고리즘 노드가 카드의 `perception: {sensor: front_lidar}`로 지도를 만든다.
  코드 수정 없이 카드만으로 된다. 같은 프로세스와 ROS 노드 실행 결과가 같다 (rut_crossing 전진 1.74 / 1.74 m, 지연 20 ms).

**시나리오 `rut_crossing`**: vehicle_crossing의 바퀴 자국(평평한 바닥, 가파른 벽)을 Chrono 없이 MuJoCo 지형 패치(`rut`, `profile: box`)로
재현했다. 빠르게 돌릴 수 있어 학습과 반복 시험에 쓴다. 지형을 모르는 트롯은 첫 자국에서 막히고 지형 인지 트롯은 통과한다.

## 넓은 지형: 로봇을 따라다니는 지형 창

```bash
python -m sim.runner scenarios/footprints_free.yaml --rviz   # 20 x 20 m 세계 지도, 좌회전하며 자유 보행 + 발자국
python conformance/test_terrain_window.py                     # 창 시험
```

- MuJoCo heightfield는 크기가 모델 생성 때 고정된다. 그래서 세계 지도(`terrain.size`)는 Terrain Map Service가 들고 있고,
  MuJoCo에는 로봇 주변 창(`terrain.window`, 반폭)만 올린다. 로봇이 창 중심에서 `window_trigger`(기본 1 m)보다 멀어지면
  창을 `window_snap`(기본 1 m) 단위로 옮겨 다시 채운다. 격자 칸의 정수배로만 옮기므로 발밑 높이는 그대로다.
- 지형 geom은 mocap 바디에 붙여 옮긴다. 월드에 붙인 geom의 위치만 바꾸면, MuJoCo가 모델 생성 때 계산한 충돌 경계
  상자가 남아 처음 창 밖으로 나간 발의 접촉을 놓친다 (확인 후 수정, `test_window_matches_full_world_on_flat`).
- 평지에서 창 모드와 세계 지도 전체 모드의 궤적이 부동소수점 수준(1e-13 m)으로 같다. 요철 지형은 보행이 혼돈적이라
  (초기 위치 1e-9 m 차이로도 끝 위치가 0.5 m 달라짐) 창 좌표의 미세한 차이로 결과가 조금 다를 수 있다.
- 렌더러도 세계 지도가 크면 로봇 주변만 그린다 (MuJoCo 렌더러 반폭 4 m, RViz는 원래 해상도로 그릴 수 있는 크기).
  스트림 계약(월드 좌표의 매니페스트와 지형 패치)은 그대로다.
- `terrain.window`가 없으면 창 = 세계 지도 전체 (기존 시나리오 그대로).

## Chrono 연동 (두 번째 물리엔진, 문서 §5)

```bash
python -m sim.runner scenarios/vehicle_crossing.yaml --rviz        # 단단한 흙: 자국 약 5.5 cm
python -m sim.runner scenarios/vehicle_crossing_soft.yaml --rviz   # 무른 흙: 자국 약 15 cm
python -m sim.runner scenarios/hmmwv_follow.yaml --rviz            # HMMWV 뒤를 따라 왼발이 바퀴 자국 안을 걷기
python -m sim.runner scenarios/footprints.yaml --rviz              # 로봇 발자국 (HMMWV 없이 Chrono를 변형 지면 계산기로만)
python conformance/test_cosim.py                                   # 연동 시험 (약 50초)
python -m sim.runner scenarios/footprints.yaml --set chrono.scm.soil.bekker_kphi=2e7   # 값 바꿔 실행
python -m sim.sweep scenarios/footprints.yaml --param chrono.scm.soil.bekker_kphi --values 2e6 1e7 5e7   # 스윕 비교표
```

- **역할 분담**: Chrono = SCM 변형 지면 (+ 선택적으로 HMMWV, 로봇 발자국), MuJoCo = Go2. 로봇과 HMMWV 사이 물리 작용은 없고,
  근접은 논리 이벤트(`vehicle_near`, MOP `min_vehicle_distance_m`)로만 판정한다 (문서 §5.4).
- **프로세스**: Chrono는 `chrono` conda 환경(Python 3.12)에서 별도 프로세스로 돈다 (`cosim/chrono_server.py`).
  러너가 자동으로 띄우며, socketpair + 길이 접두 JSON(`cosim/wire.py`)으로 연결한다. python 경로는 `CHRONO_PYTHON`으로 바꿀 수 있다.
- **동기**: 0.04 s마다 차량 포즈, 0.2 s(5 Hz)마다 지형 변경분을 교환. 파이프라인 방식이라 두 엔진이 같은 구간을 동시에 계산하며 결과는 결정적이다.
- **지형**: SCM 격자 = MuJoCo 지형 격자 (해상도 같고 SCM 중심이 격자점). 바퀴 자국은 Terrain Map Service를 거쳐 MuJoCo로 들어가며,
  발 근처 셀 보류도 그대로 적용된다. SCM 영역의 기본 지형은 평평해야 한다 (시작 시 검사).
- **가시화**: HMMWV 바디는 중립 스트림에 `physics_source: Chrono`로 들어가므로 `--rviz`, `--mjviz` 어댑터 모두 수정 없이 함께 그린다.
  디버그용 `--view`(물리 모델을 직접 보여주는 MuJoCo 뷰어)에는 HMMWV가 보이지 않는다.
- **이 PC 기준 성능**: 실시간의 약 0.8배 (Chrono가 상한). `--rviz`에서는 화면이 실제 시간보다 느리게 흐르고 `late_control_steps`가 크게 나온다. 결과(결정성)에는 영향 없음.
- **결과 예 (`vehicle_crossing`)**: 규칙 기반 트롯은 5.5 cm 자국에서 뒷다리가 걸려 x≈2.2 m에서 멈춘다 (넘어지지는 않음).
  무른 흙(15 cm)에서는 첫 자국에서 넘어진다. 지형을 보지 못하는 보행기의 한계를 보여주는 시험 결과다.
- **결과 예 (`hmmwv_follow`)**: 같은 평지에서 HMMWV 유무만 바꿔 비교 (20초). 자국 때문에 전진 거리 6.20 -> 4.09 m,
  CoT 2.54 -> 6.08, 평균 몸통 롤 0.9° -> 4.7°. 넘어지지는 않음.
- **로봇 발자국 (`robot_feet`)**: MuJoCo가 네 발 위치와 지면 수직 하중(물리 스텝 평균)을 Chrono에 보내고, 발과 같은 반지름의
  대리 구가 그 힘으로 SCM 흙을 누른다. 깊이는 하중과 흙 강도로 정해진다 (kphi 1e7: 50 N -> 약 1.8 cm, 80 N -> 약 2.8 cm).
  패인 모양은 지형으로 돌아와 이후 걸음이 밟는다. 디딘 발이 그 자리에서 가라앉지는 않는다 (흙 반력을 MuJoCo로 되돌리지 않음).
  - 격자는 발 반지름(2.2 cm)보다 촘촘해야 한다 (`footprints.yaml`은 2 cm). 5 cm면 발이 격자점 사이로 빠진다.
  - `terrain.guard_radius`(발 근처 갱신 보류 반경, 기본 0.15 m)를 보폭보다 작게(0.06 m) 해야 로봇이 자기 발자국을 밟는다.
  - Chrono 시간 간격은 1 ms (`chrono.step`). 2 ms면 작은 발 접촉에서 대리 구가 떨다가 순간적으로 박혀 -15 cm까지 판다 (수렴 확인: 1 ms = 0.5 ms).
  - 결과 (`footprints`, 15초, 흙 강도 스윕): kphi 2e6 / 1e7 / 5e7 -> 발자국 평균 2.5 / 1.2 / 0.6 cm, 최대 5.4 / 3.4 / 0.8 cm.
    전진 거리는 모두 약 4.78 m로 보행에는 영향이 거의 없다 (디딘 발이 그 자리에서 가라앉지 않으므로).
  - 진단: `CHRONO_DEBUG_FEET=1`로 실행하면 대리 구의 높이, 속도, 흙 표면 높이가 `build/chrono_server.log`에 스텝마다 기록된다.
- **새 시나리오에 HMMWV 넣기**: 코드 변경 없이 시나리오에 `chrono:` 절만 추가한다. 조건(시작 시 검사): SCM 해상도 = 지형 해상도,
  SCM 중심이 지형 격자점, SCM 영역의 기본 지형이 평평, HMMWV 경로가 지형 범위 안.

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

**정책 `go2_trot_bc_terrain` (실패, 탑재 경로 시험용)**: 지형 인지 트롯을 DAgger로 모방하려 했으나 **지형을 쓰지 못한다**.
관측은 `observation_perceptive`, 학습 지형은 무작위 상자형 자국. 평지 4.23 m는 걷지만 rut_crossing에서는 지형을 모르는 트롯처럼
첫 자국에서 막힌다 (1.95 m, 넘어짐). 오프라인으로 나눠 본 원인:
- 처음에는 자국을 건너는 샘플이 2~5%뿐이었다. 자국을 촘촘히 배치해 61%로 늘려도 결과가 같았다.
- 같은 데이터에서 높이 입력을 준 학생이 주지 않은 학생보다 교사 행동을 더 못 맞혔다 (자국 근처 시험 오차 0.019 vs 0.012).
  높이 입력 압축(주성분 4~16개), 직전 행동 입력 빼기(교사의 결정을 그대로 따라 하는 효과 제거)도 나아지지 않았다.
- 해석: 교사의 판단은 4 cm 칸의 미세한 높이 차, 문턱값(1.5 cm, 2 cm), 내부 기억(고른 자리, 발 든 높이)에 달려 있어
  이 규모(10만 샘플)로는 학생이 규칙을 일반화하지 못하고 과적합한다.
- 그래서 지형 사용은 보상으로 직접 배우는 강화학습(GPU)에 맡긴다 (아래 "강화학습 준비"). 관측 규약과 배치 경로는 그대로 쓴다.
- 이 정책은 규약 확정 전(기준 몸통 높이 0.30 m)에 학습해 카드의 `base_height_ref`가 명세(0.26 m)와 다르다. 카드가 우선하므로 동작은 맞다.

학습 시간 참고: 이 PC(메모리 7.7 GB, 스왑 없음)에서 작업자 5개로 돌리면 다른 프로그램과 함께 메모리가 모자라 시스템이 멈췄다.
작업자 하나가 약 0.55 GB를 쓰므로 `--workers 3`(기본값)을 권장한다.
에피소드마다 메모리가 약 100 MB씩 쌓이던 누수는 에피소드마다 `gc.collect()`, 작업자를 10 에피소드마다 새로 띄우기(`maxtasksperchild`)로 막았다.

## DIS 시나리오 콘솔 (문서 §10)

실행 중인 시뮬레이터에 운용자가 이벤트(로봇 이동 명령, 지형 변경, 고장 주입, 개체 투입, 흙 상태, 실행 제어)를 넣는 콘솔이다.
봉투와 핸드셰이크는 [DIS_test](https://github.com/danielhwani/DIS_test)(SIMAN-R 프로토타입)의 것을 그대로 쓴다.

```bash
# 터미널 1: 시뮬레이터 (--dis-port를 주면 실시간으로 돈다. --dis-wait: 콘솔이 접속할 때까지 시작하지 않음)
python -m sim.runner scenarios/dis_console.yaml --dis-port 3000 --dis-wait --rviz
# 터미널 2: 콘솔 (대화형)
python -m dis_console.console --sim 127.0.0.1:3000
dis> cmd 0.35            # 전진 0.35 m/s
dis> rut 3.0 @12         # 시뮬레이션 시각 12 s에 x = 3.0 m를 가로지르는 바퀴 자국 (폭 0.25, 깊이 0.05, 평평한 바닥)
dis> ridge 5.0           # x = 5.0 m를 가로지르는 턱 (과속방지턱 모양, 폭 0.4, 높이 0.05), 바로 적용
dis> bump 2.0 0.5        # 한 점 둔덕 (중심 (2.0, 0.5), 높이 0.05, 지름 0.4)
dis> ahead rut 1.5       # 로봇 정면 1.5 m에 진행 방향을 가로지르는 자국 (ahead bump / ridge도 같음)
dis> cmd 0.3 0.5         # 전진하며 왼쪽으로 회전
dis> status              # 마지막 주기 보고 (시각, 로봇 위치·방향, 명령, 마지막 이벤트). watch: 1초마다 계속 표시
dis> stop
dis> fault lidar 5       # 5초 동안 LiDAR 점군이 알고리즘에 오지 않음 (link / imu / battery, clear로 해제)
dis> freeze              # 일시정지 (resume로 재개), end: 시험 종료
# Chrono 시나리오: 숨겨 둔 HMMWV 투입, 흙 상태
python -m sim.runner scenarios/dis_chrono.yaml --dis-port 3000 --dis-wait --rviz
dis> cmd 0.35
dis> spawn ahead         # 차가 로봇 앞 2.5 m 지점을 먼저 지나가도록 시뮬레이터가 출발 시각을 맞춤 (spawn 4: 바로 4 m/s로 출발)
dis> soil soft           # 흙을 무르게 (hard / firm / soft / mud)
# 명령을 차례로 보내고 끝내기 (각각 완료까지 기다림)
python -m dis_console.console --send "cmd 0.35" --send "rut 3.0 @12"
```

**구조**
```
콘솔 (2/1/1)                         시뮬레이터 (1/10/0, sim/dis_server.py)
 │ Action Request-R  Request_Connection ───▶ │ 시나리오 이름, 시각, 지원 메시지
 │ Action Request-R  Event_AddTerrainPatch ─▶ │ sim.events에 예약 (시나리오 파일 events와 같은 목록)
 │◀── Action Response-R  Pending (예정 시각)   │
 │                                          │ 제어 주기에서 그 시각이 되면 적용
 │◀── Action Response-R  Complete (적용 시각)  │
 │◀── Data PDU  Report_SimStatus (0.2 s) ────  │
```
- **봉투**: DIS 7 Action Request-R(56) / Action Response-R(57) / Data PDU(20)에 JSON 페이로드. 페이로드 언어는 시나리오 관리(4, 예시값).
- **핸드셰이크**: 1초 안에 응답이 없으면 같은 Request ID로 3번까지 재전송. Pending 뒤 3초 동안 조용하면 같은 Request ID로 상태 재질의.
  시뮬레이터는 (콘솔, Request ID)를 기억해 중복이면 다시 넣지 않고 마지막 응답만 보낸다.
- **메시지**

| 메시지 | body | 동작 |
|---|---|---|
| `Request_Connection` | `role`(control / observe) | 접속. 주기 보고를 받기 시작. control은 한 콘솔만 (다른 콘솔이 갖고 있으면 `CONTROL_BUSY`) |
| `Request_ReleaseControl` | – | 제어권 내놓기 (관찰 콘솔이 됨) |
| `Event_SetCommand` | `vx`, `yaw_rate`, (`t_apply`) | 로봇 운용자 이동 명령 (가상 모드 전용. 범위 \|vx\| ≤ 1, \|yaw_rate\| ≤ 1.5) |
| `Event_AddTerrainPatch` | `patch`, (`t_apply`) | 지형 변경: bump / rough / ramp / rut (시나리오 `patches`와 같은 형식). Complete에 `robot_rel` |
| `Event_InjectFault` | `fault`, `duration`(없으면 해제까지), `params`, (`t_apply`) | 고장 주입 (아래 표) |
| `Event_ClearFault` | `fault`, (`t_apply`) | 고장 해제. 걸려 있지 않으면 거부 `FAULT_NOT_ACTIVE` |
| `Event_CreateEntity` | `entity_type`(HMMWV), `speed`, `cross_ahead`, (`t_apply`) | 개체 투입 (Chrono 시나리오, 아래 설명). `cross_ahead`: 로봇 앞을 지나가도록 출발 시각 자동 |
| `Event_RemoveEntity` | `entity_type`, (`t_apply`) | 개체 제거: 제동하고 숨김 |
| `Event_SetSoil` | `preset`(hard/firm/soft/mud) 또는 `soil`(SCM 값 8개), (`t_apply`) | 흙 상태 (Chrono SCM, 이후 생기는 변형에 적용) |
| `Event_Freeze` / `Event_Resume` | (`t_apply`) / – | 일시정지 / 재개. 재개는 바로 (정지 중에는 시뮬레이션 시각이 멈춤) |
| `Event_Stop` | (`t_apply`) | 시험 종료: 기록과 MOP 저장 (`stopped_by_console`) |
| `Report_SimStatus` (Data PDU, 0.2 s) | – | 시뮬레이션 시각, 로봇 위치·방향(시험 판정자 쪽 참값), 명령, 마지막 이벤트, 고장(남은 시간), 일시정지, 차량, 링크 상태 |
| `Report_ConsoleHeartbeat` (Data PDU, 1 s) | `seq`, `link_state` | 콘솔 -> 시뮬레이터 생존 신호 (DIS_test와 같음) |

**고장 주입** (문서 §10.2 "고장 주입", 모두 로봇 쪽 경계에서 일어난다. 알고리즘은 실제 로봇처럼 "안 온다", "다르게 온다"만 안다)

| `fault` | 콘솔 | 로봇 쪽에서 일어나는 일 |
|---|---|---|
| `lidar_blackout` | `fault lidar <초> [센서]` | LiDAR 점군이 알고리즘에 오지 않는다 (기록과 ROS 토픽에서도 빠짐). 지형 인지 보행은 이미 쌓은 지도로만 걷는다 |
| `link_loss` | `fault link <초> [damp]` | 제어 링크 두절: 로봇 상태가 알고리즘에 가지 않고 관절 명령도 오지 않는다. 로봇은 마지막 명령을 계속 실행(`hold_last`, 기본) 또는 감쇠 모드(`damp`: 위치 게인 0, 속도 게인 5로 천천히 주저앉음). 전송 중이던 명령도 잃는다 |
| `imu_bias` | `fault imu <초> [gyro=gx,gy,gz] [att=롤,피치,방향]` | IMU 각속도 편향(rad/s, 기본 방향 축 0.05)과 자세 출력 오프셋(도) |
| `battery_low` | `fault battery <초> [비율]` | 모터 토크 한계를 비율만큼 (기본 0.6, 전압 저하) |

**고장 주입 확인 순서 (일반 터미널)**

```bash
# 터미널 1: 시뮬레이터 (LiDAR 고장을 보려면 지형 인지를 켠다)
python -m sim.runner scenarios/dis_console.yaml --dis-port 3000 --dis-wait --rviz --set controller.perception.sensor=front_lidar
# 터미널 2: 콘솔
python -m dis_console.console --sim 127.0.0.1:3000
dis> cmd 0.3
dis> watch          # 1초마다 상태 표시 (고장 이름과 남은 시간)
```

고장이 걸려 있는 동안 RViz 정보판에 `FAULT:이름(남은 초)`가 빨간 글씨로 나온다.

| 순서 | 콘솔 명령 | RViz에서 볼 것 |
|---|---|---|
| 1 | `ahead rut 2` 후 바로 `fault lidar 6` | 알고리즘 지형 지도(타일)가 6초 동안 갱신되지 않는다. 이미 본 자국은 지도에 남아 피해 딛고, 새로 만든 자국은 지도에 나타나지 않는다 (비교: `fault lidar 6` 직후 `ahead rut 1.5`). 끝나면 다시 쌓인다 |
| 2 | `fault link 1` | 1초 동안 마지막 명령 자세로 굳는다. 풀린 뒤 잠깐 서 있다가 다시 걷는다 |
| 3 | `fault link 1 damp` | 다리 힘이 빠져 주저앉는다 (몸통 약 0.14 m). 풀린 뒤 약 1초에 걸쳐 일어서고 다시 걷는다 |
| 4 | `fault link 3 damp` | 배를 대고 엎드린다 (0.08~0.14 m). 넘어짐 판정 없이 일어서고, 지형 인지 보행도 다시 잘 걷는다 |
| 5 | `fault imu 5 gyro=0,0,0.3` | 알고리즘이 왼쪽으로 돌고 있다고 믿고 반대로 돌려, 실제로는 오른쪽으로 휜다. 측정: 5초 동안 방향 -10°. 해제 뒤 원래 방향(+1°)으로 돌아온다 (방향 유지는 각속도가 아니라 IMU 자세 출력 기준) |
| 6 | `fault imu 5 att=0,5,0` | 피치 5° 오프셋. 트롯은 IMU 기울기만큼 앞뒤 다리 길이를 조절해 몸통을 수평으로 맞추므로, 들렸다고 잘못 믿고 앞을 낮춰 실제로는 숙인 채 걷는다. 측정: 실제 피치 -2.9°, 알고리즘이 믿는 피치 +2.2° (수평 보정이 기울기를 일부만 따라감), 해제 후 -0.1°로 복귀. 크게 보려면 `att=0,15,0`(앞뒤), `att=10,0,0`(옆) |
| 7 | `fault battery - 0.2` → `clear battery` | 토크 한계 20%. 다리가 몸무게에 눌려 처지면 알고리즘이 일어서기로 들어가 낮은 자세로 버틴다 (거의 멈춤, 몸통 약 0.22 m). 해제하면 일어서서 다시 걷는다 (측정: 해제 뒤 6초 전진 1.2 m 이상, 방향 틀어짐 6° 이하). 0.4에서는 평지 보행에 영향이 없다. 고치기 전에는 해제 순간 다리가 튕겨 몸이 떠서 -40°~-110° 돌았다 (아래 설명) |
| 8 | 콘솔 터미널에서 Ctrl+Z | 통신 두절: 5초 뒤 터미널 1에 `DIS 통신 두절 … -> 단절 시 동작 STOP`, 로봇이 선다. `fg`로 되살리면 `DIS 링크 복구`. 명령은 자동으로 돌아오지 않으므로 `cmd 0.3`을 다시 넣는다 |

`end`로 끝내면 `runs/dis_console_<시각>/scenario_replay.yaml`로 같은 고장 세션을 그대로 다시 돌릴 수 있다
(`python -m sim.runner runs/dis_console_<시각>/scenario_replay.yaml --rviz`, 처음 실행의 `--set`은 이 파일에 이미 들어 있다).
지형 인지를 켜면 실시간의 약 0.85배로 돈다. 표의 5~7번 설명은 시뮬레이션으로 측정한 값이다 (화면 확인은 아직).

**일어서기** (알고리즘 쪽, `control/node.py`, `control/estimator.py`): 로봇 상태가 0.1 s 넘게 오지 않다가 다시 오면(링크 두절), 또는 디딘 다리가
명령보다 2 cm 넘게 눌린 상태가 쌓여 0.5초가 되면(토크 부족, 처진 시간은 더하고 괜찮은 시간은 뺀다)
- 보행을 멈추고, 지금 관절 각도에서 서 있는 자세까지 천천히(관절 최대 1 rad/s) 옮긴 뒤 0.5 s 서 있다가 보행을 처음 위상부터 다시 시작한다.
  처음에는 주저앉은 자세에서 곧바로 트롯을 이어 큰 PD 오차가 한꺼번에 걸려 고꾸라졌다 (감쇠 1초 뒤 피치 -65°, 뒤집힘).
- 상태 추정기는 공백 동안 제자리에 있었다고 보고 속도 0에서 다시 시작한다. 처음에는 끊기기 전 속도로 공백을 적분해
  3초 두절에 추정 위치가 0.7 m 튀었다 (지형 모름 보행은 위치를 안 써서 괜찮았지만 지형 인지 보행은 지도·디딜 곳이 어긋나 잘 걷지 못함).
- 다시 디딘 발은 공백 전 지면 높이에 놓는다. 엎드리면 발 하중이 사라져 기억한 접촉점을 잃는데, 지금 추정 높이로 다시 잡으면
  주저앉은 만큼(11 cm) 지면이 높다고 믿어 지형 인지 보행이 발을 뻗다 튀어 넘어졌다.
- 결과 (복구 뒤 8초 전진, 일어서는 약 1.5초 포함): 지형 인지 감쇠 3초 1.84 m (고치기 전 0.09 m 뒤 넘어짐), 감쇠 1초 1.92 m,
  마지막 명령 유지 1초 2.00 m, 지형 모름 감쇠 3초 1.86 m. 추정 오차는 모두 5 cm 이내 (`test_stand_up_after_link_loss`).
- 서 있는 자세로 옮기는 것은 관절이 따라오는 만큼만 진행한다 (명령과 측정 각도 차이 0.15 rad 넘으면 멈춤). 토크가 모자란 동안 명령이
  실제 자세에서 멀어지지 않게 해, 토크가 돌아오는 순간 튕기지 않는다.
- 배터리 저하에서 찾은 결함과 수정 (`test_battery_low_recovery`):
  - 토크 20%로 멈춰 있는 동안 트롯의 속도 보정 적분기가 오차를 상한까지 쌓았다가 해제 순간 한꺼번에 내보냈다 (와인드업).
    -> 출력이 이미 한계에 걸린 방향으로는 적분하지 않는다 (`control/trot.py`, 조건부 적분).
  - 그래도 처진 자세에서 서서 걷는 명령을 계속 내고 있다가 토크가 돌아오면 다리가 튕겨 몸이 떠서 -73° 돌았다 -> 다리 처짐을 보고 일어서기.
  - 30%에서는 처짐이 띄엄띄엄이라 "한 번이라도 괜찮으면 0"으로는 감지하지 못해 -110° 돌았다 -> 처진 시간을 쌓고 빼는 방식.
  - 정상 보행의 0.5초 지속 처짐은 최대 0.9 cm (평지, 요철, 자국, 지형 인지 포함 5종 측정). 9개 시나리오 × 시드 3개에서 일어서기는
    자국에 걸린 vehicle_crossing 한 번(2회)만 들어갔다.
- 넘어짐 판정: 똑바로 엎드린 자세는 다리 모양에 따라 몸통이 0.08~0.14 m여서 높이로는 넘어짐과 구분되지 않는다. 그래서 넘어짐은
  기울기(1 rad)로 판정하고, 높이 기준은 0.12 m에서 0.04 m(몸통이 땅에 박힘)로 낮췄다. 뒤집히면 기울기로 걸린다.
  기존 시나리오 3종 × 시드 3개(rut_crossing, vehicle_crossing_soft, hmmwv_follow)의 판정과 넘어진 시각은 그대로였다.

`<초>` 자리에 `-`를 쓰면 `clear`까지 유지한다. 시간이 다 되면 시뮬레이션 시각 기준으로 저절로 풀리므로 기록으로 다시 돌려도 같은 스텝에서 풀린다.
RViz 정보판에 `FAULT:이름(남은 초)`가 빨간 글씨로 나온다. 센서를 별도 노드로 돌리는 `--sensor-node`에서는 `lidar_blackout`이 적용되지 않는다.

**실행 제어**: `freeze`(일시정지)와 `end`(종료)는 예약할 수 있고, 적용된 제어 주기를 마친 뒤 멈춘다. 일시정지 동안에도 콘솔 요청과
주기 보고, heartbeat는 계속된다. 일시정지 중에 접수된 명령은 재개해야 적용되므로 콘솔이 `※ 일시정지 중: resume 하면 적용`으로 알린다
(재개는 제어권을 가진 콘솔만 할 수 있다). 일시정지·종료는 물리에 영향이 없으므로 `scenario_replay.yaml`에서 빠지고, 콘솔로 종료했으면 그 시각이 `duration`이 된다.

**링크 감시** (DIS_test와 같은 양방향 heartbeat)
- 콘솔 -> 시뮬레이터 `Report_ConsoleHeartbeat` 1 s, 시뮬레이터 -> 콘솔 주기 보고 0.2 s. 상대에게서 온 어떤 PDU든 생존 신호로 본다.
- 시뮬레이터: 접속한 콘솔 모두에게서 5 s 동안 아무것도 오지 않으면 통신 두절. 단절 시 동작 `--dis-comm-lost STOP`(기본)이면
  로봇 이동 명령 0을 이벤트로 넣는다 (`source: dis_comm_lost`, 기록되므로 다시 실행해도 같다). `CONTINUE`면 그대로.
  다시 받으면 OK로 돌아가지만 명령은 자동으로 되돌리지 않는다.
- 콘솔: 시뮬레이터 보고가 3 s 끊기면 "정보 갱신 안 됨", 5 s면 "통신 두절"로 표시한다.

**개체 투입과 흙 상태** (Chrono 시나리오, 예: `scenarios/dis_chrono.yaml`)
- 시나리오의 `chrono.vehicle`에 `t_start: null`을 두면 HMMWV를 시작 때 만들어 출발점에 제동해 두고 **숨긴다**
  (땅속 깊이 둔 포즈로 내보내 화면과 LiDAR에 보이지 않고 근접 판정도 하지 않음). `spawn`이면 나타나 시나리오에 선언한 경로를 따라 출발한다
  (`dis_chrono`: (5, -8) -> (5, 8), 로봇 진행선을 x = 5 m에서 가로지름. 변형 지면은 x 3.5~6.5 m).
  속도만 콘솔에서 정할 수 있다. `despawn`이면 제동하고 다시 숨긴다.
- 실행 중에 새로 만들지 않는 이유: Chrono에서 시뮬레이션을 진행한 뒤 HMMWV를 만들면 SCM 흙에 비정상적으로 깊이 빠졌고
  (차체 높이 0.57 -> 0.36 m), SCM 계산 영역(active domain)을 붙이면 프로세스가 죽었다 (`SCMLoader::UpdateActiveDomain`). 그래서 미리 만든 슬롯을 투입하는 방식이다.
  숨겨 둔 동안에도 차량 무게로 출발점의 흙이 조금 눌린다.
- **로봇 앞으로 지나가게 하기** (`spawn ahead [간격] [속도]`, `cross_ahead`): 차량과 로봇 사이에는 물리 작용이 없어(문서 §5.4)
  시점을 잘못 맞추면 차가 로봇을 뚫고 지나갔다. 시뮬레이터가 로봇의 위치·방향·명령 속도와 차량 경로로 교차점을 구하고,
  차 꼬리가 로봇 진행선의 "로봇 앞 (간격) m 지점"을 로봇보다 0.3초 먼저 지나가는 순간에 출발시킨다
  (차량 이동 시간은 이 PC 측정값: 목표 속도의 95%, 출발 지연 0.15 + 0.08 × 속도 초). 접수 응답에 예상 출발 시각이 오고,
  로봇이 너무 가까우면 `TOO_LATE`, 진행선이 경로와 만나지 않으면 `NO_CROSSING`. 기다리는 동안 로봇이 방향을 바꿔도 다시 판단한다.
  확인: 로봇이 출발 직후 요청 -> 3.0초에 출발, 차 꼬리가 지날 때 로봇은 선보다 2.7 m 앞, 최소 거리 2.96 m.
- **양보** (`vehicle.yield_to_robot: true`, `dis_chrono`에서 켬): 차 앞 진로(앞 9 m, 옆 ±1.8 m, 차체 중심 기준)에 로봇이 있으면 HMMWV가 제동하고
  기다린다. 시점이 어긋난 `spawn`이면 차가 로봇 앞에서 서 있다가 로봇이 건너간 뒤 그 뒤로 지나간다 (확인: 약 10초 대기, 최소 거리 2.89 m).
  로봇 위치는 Chrono 교환 때마다 보내므로(0.04 s) 결정적이다. `vehicle_crossing` 등 기존 시나리오는 꺼져 있어 결과가 같다.
- **로봇 발자국** (`scenarios/dis_footprints.yaml`): `dis_chrono`는 HMMWV 자국만 계산하고 로봇 발자국은 계산하지 않는다
  (5 cm 격자에서는 발이 격자점 사이로 빠지고, 발자국 연결도 꺼져 있음). 발자국에는 2 cm 격자와 Chrono 1 ms 스텝이 필요한데,
  HMMWV까지 같은 조건으로 계산하면 이 PC에서 실시간의 0.06배로 느려졌다. 그래서 HMMWV 없이 발자국만 계산하는 콘솔 시나리오를 따로 두었다
  (실시간의 약 2.6배). 변형 지면은 x 1~9 m. 12초 걷기, 흙별 발자국 깊이 평균/최대:
  hard 0.7 / 1.2 cm, firm 1.9 / 4.9 cm, soft 3.0 / 8.4 cm (같은 시간에 전진 4.1 / 3.6 / 2.8 m: 무를수록 발이 빠져 느려짐).
  mud는 변형 지면에 들어서자마자(x 약 1.25 m) 발이 계속 빠지며 갇힌다 (구덩이 15 cm = 시나리오 지형 높이 하한, 몸통 0.18 m까지 내려앉음, 넘어지지는 않음).
- 흙: `soil hard|firm|soft|mud`. HMMWV 자국 깊이 약 1.8 / 5.5 / 15 cm, mud는 더 무르고 미끄럽다. 이후 생기는 변형부터 적용된다.
  확인: `soil soft` 뒤 투입한 HMMWV가 14.7 cm 자국을 남겼다.
- 투입·흙 명령은 다음 Chrono 교환 구간(0.04 s)부터 적용된다. 교환 시각이 시뮬레이션 시각으로 정해져 있어 기록으로 다시 돌리면
  로봇과 차량 위치가 비트 단위로 같다 (`test_entity_and_soil_with_chrono`). Chrono 없는 시나리오에서는 거부(`NO_CHRONO`).
- 이 PC에서 HMMWV가 달리는 동안은 실시간의 약 0.87배로 돈다.

- **좌표**: 시뮬레이터 세계 좌표 (원점 = 로봇 출발점, +x = 처음 바라본 방향, +y = 왼쪽, m). `dis_console` 시나리오의 지형은 ±30 m이고,
  MuJoCo에는 로봇을 따라다니는 ±3 m 창만 올린다.
- **로봇 기준으로 넣기 (`ahead`)**: 콘솔이 마지막 주기 보고(0.2 s 간격)와 명령 속도로 적용 시점의 로봇 위치를 예측해 세계 좌표로 바꿔 보낸다.
  메시지와 기록은 절대 좌표 그대로라 재현성은 같다. 실제 속도가 명령보다 느리면 그만큼 조금 더 앞에 생긴다.
- **적용 위치 확인**: 지형 패치의 Complete 응답에 적용 순간 로봇 기준 위치 `robot_rel` [앞, 왼쪽]이 들어간다.
  콘솔은 `적용 완료 t=… (로봇 기준 앞 1.48 m, 왼쪽 0.02 m)`처럼 보여 주고, 로봇 뒤 0.3 m 넘게 떨어진 곳이면 "로봇이 이미 지나간 곳"이라고 알린다.
- **턱 (`ridge`)**: 따로 메시지를 만들지 않고 깊이가 음수인 완만한(cosine) `rut` 패치로 보낸다. 시드 2개 확인에서 5 cm 턱은
  지형 모름 트롯이 한 번은 넘고 한 번은 넘어졌고, 지형 인지 트롯은 둘 다 넘었다.
- **시각 T 적용** (문서 §10.1): `t_apply`가 있으면 그 시뮬레이션 시각에, 없으면 받은 제어 주기에 적용. 이미 지난 시각은 거부(`T_APPLY_PASSED`).
- **거부**: 접속 전 요청 `NOT_CONNECTED`, 범위 밖·값 누락 `INVALID_VALUE`, 모르는 메시지 `UNSUPPORTED`, 정지 중이 아닌데 재개 `NOT_FROZEN`,
  개체 `ENTITY_ACTIVE` / `ENTITY_NOT_ACTIVE` / `NO_ENTITY_SLOT`. 다른 Exercise ID와 다른 수신자 앞 PDU는 응답 없이 버린다.
- **재현성** (문서 §10.1 "초기 구성과 이벤트 로그가 재실행 가능한 시나리오 스크립트"):
  - 실행 폴더에 `dis_events.jsonl`(받은 요청과 보낸 응답, 벽시계와 시뮬레이션 시각)과
    `scenario_replay.yaml`(적용된 이벤트를 적용 시각으로 넣은 시나리오)이 남는다.
  - `python -m sim.runner runs/<실행>/scenario_replay.yaml`로 콘솔 없이 같은 결과가 나온다. 실시간 실행을 lockstep으로 다시 돌려도
    로봇 궤적이 비트 단위로 같다 (`test_console_events_replay_bit_identical`, 고장 주입 포함). 실제 실행에서도
    링크 두절·일시정지·예약 종료가 들어간 세션의 종료 시각, 전진 거리, 옆 표류가 같았다.
  - 이벤트는 제어 주기 시작에서 적용한다. 같은 프로세스 실행이 결정적이므로, 언제 도착했든 적용 시각만 같으면 결과가 같다.
**제어권** (DIS_test와 같은 방식)
- 명령할 수 있는 콘솔은 하나다. 콘솔은 기본으로 제어권을 요청하고(`role: control`), 이미 다른 콘솔에 있으면 `CONTROL_BUSY`로 거부된다.
  `--observe`로 접속하면 관찰 콘솔이 되어 주기 보고만 받고, 명령하면 `NOT_IN_CONTROL`로 거부된다.
- 콘솔 명령 `release`(내놓기), `take`(받기). 콘솔을 `quit`으로 끝내면 제어권을 내놓는다.
- 제어 콘솔이 통신 두절되면 단절 시 동작을 실행하고 제어권을 푼다. 그 콘솔이 돌아와도 링크만 OK가 되고, 제어권은 `take`로 다시 받아야 한다
  (그 사이 다른 콘솔이 받았을 수 있음). 콘솔은 주기 보고의 `control_owner`로 제어권이 풀린 것을 알린다.

**메시지 인증** (`dis_console/auth.py`, HMAC-SHA256 공유 키)
```bash
python -m dis_console.auth keygen ~/.config/physics_ai_test/dis.key       # 키 만들기 (권한 600, 저장소 밖에 둔다)
python -m sim.runner scenarios/dis_console.yaml --dis-port 3000 --dis-wait --rviz --dis-key ~/.config/physics_ai_test/dis.key
python -m dis_console.console --sim 127.0.0.1:3000 --key ~/.config/physics_ai_test/dis.key
```
- 키가 있으면 모든 PDU 끝에 Variable Datum 하나(카운터 8 B + HMAC 32 B)를 붙이고, 받은 PDU는 확인해서 맞지 않으면 **응답 없이 버린다**
  (공격자에게 단서를 주지 않음). 시뮬레이터 터미널에 `DIS 인증 실패로 버림`이 나오고 원본 기록에 `"dropped": "auth"`로 남는다.
- MAC은 헤더(Exercise ID, PDU 종류, 타임스탬프)와 페이로드를 모두 덮는다. 한 바이트라도 바뀌면 거부된다 (예: DIS_test 재생기가 Exercise ID를 99로 바꾼 명령).
- 재전송 공격: 카운터 = 보낸 시각(마이크로초). 30초 넘게 어긋나면(오래된 캡처) 거부하고, 보낸 쪽별로 카운터가 줄면 거부한다.
  응답이 없어 같은 요청을 그대로 다시 보내는 핸드셰이크 재전송은 최근 받은 패킷과 바이트까지 같으면 받는다 (다시 실행하지 않고 응답만).
  다른 장비끼리 쓰려면 시계를 맞춰야 한다 (NTP/PTP).
- 인증 datum을 모르는 체계(DIS_test, Wireshark)도 Variable Datum이 하나 더 있는 것으로 읽는다.
- 키 없는 콘솔로 접속하면 응답이 없어 콘솔이 "인증을 쓰는 중일 수 있다 (`--key`)"고 알린다.

**받는 주소**: 기본은 이 PC 안(127.0.0.1)에서만 받는다. `--dis-host <주소>`로 다른 장비의 콘솔을 받을 수 있지만, **`--dis-key`가 없으면
실행을 거부한다** (인증 없이 열면 같은 네트워크의 누구나 로봇에 명령할 수 있다). 다른 장비의 콘솔은 `--sim <시뮬레이터 주소>:3000 --key <같은 키>`.
이 기능은 이 PC 안에서만 시험했다.

**원본 PDU 기록** (`dis_pdus.jsonl`, `dis_pdus.pcap`, 문서 §11 원본 층)
- 시뮬레이터가 받고 보낸 PDU를 그대로 남긴다. 형식은 DIS_test 기록 중계기와 같아 그쪽 도구로 바로 읽힌다:
  ```bash
  cd ~/DIS_test
  python3 -m siman_r.player ~/physics_ai_test/runs/<실행>/dis_pdus.jsonl --print --speed 0      # 타임라인
  python3 -m siman_r.player ~/physics_ai_test/runs/<실행>/dis_pdus.pcap --print --vehicle-port 3000
  wireshark ~/physics_ai_test/runs/<실행>/dis_pdus.pcap                                          # UDP 3000 = DIS
  ```
- `dis_events.jsonl`(해석된 요청·결과)과 `scenario_replay.yaml`(적용된 이벤트)은 그대로 남는다. 원본 기록은 무엇이 오갔는지(인증 실패 포함),
  replay 시나리오는 시뮬레이션에 무엇이 적용됐는지를 보여 준다.

**아직 없는 것** (문서 §10.2의 나머지)
- 임의 위치·여러 대 개체 투입 (지금은 시나리오에 선언한 HMMWV 한 대), 표준 Entity State PDU, 임무 명령 (C-BML Order),
  기상(날씨, 시정 -> 센서 영향), 교전 효과, 장애물·지뢰, 전자전 (GNSS 재밍, 통신 열화. 링크 두절은 고장 주입으로 가능)
- 초기 구성 MSDL (지금은 시나리오 YAML)
- 암호화 (인증만 있고 내용은 보인다), 키 교체·여러 키, 다른 장비 사이 시험
- Live 로봇 권한 분리: 실제 로봇에는 이동 명령이 아니라 임무만 보내야 한다 (§10.1). 지금의 `Event_SetCommand`는 가상 모드 전용이다.

## 강화학습 준비 (GPU 장비에서)

이 PC(GPU 없음)에서는 처리량이 나오지 않아 학습은 다른 장비에서 한다. 이 저장소는 **학습 결과(ONNX + 카드)를 받아 시험하는 쪽**이다.
학습 쪽이 맞춰야 할 것과 이미 준비된 것:

| 항목 | 상태 |
|---|---|
| 모델 | `go2_mjx.xml` + 명세 오버라이드 (위 "MJX 준비 사항", `test_specs.py`) |
| 관측 | 지형 모름: `observation` 47개 / 지형 인지: `observation_perceptive` 256개. JAX 관측 함수는 `obs_reference.npz`와 1e-5 안에서 같아야 한다 |
| 행동, PD, 제어 주기 | `action` 절 (기본 자세 + 0.25 × 행동, kp/kd, 20 ms) |
| 배치 조건 | 관측은 추정값(센서 잡음 + 다리 주행거리계), 연결 지연 1주기. 학습 때 잡음과 지연 0~2주기를 무작위로 주기를 권장 (`go2_trot_bc`와 `go2_trot_bc_est`가 이 차이로 크게 갈렸다) |
| 지형 | 무작위 상자형 자국(`train/imitate_trot.py`의 `random_ruts`: 3~6개, 깊이 3~6 cm, 폭 18~35 cm) + 요철. 시험은 rut_crossing, vehicle_crossing |
| 탑재 | 카드 작성 -> `test_policy.py`, `compare_variants.py --policy`, `sim.sweep --param seed`, `test_ros_equivalence.py` |

지형 인지 정책은 학습 때 `height_scan`을 참 지형에서 뽑되, 배치 때 LiDAR 지도의 오차(위치 표류로 생기는 수 cm의 일정한 차이, 앞쪽만 보임)를
흉내 내도록 높이 잡음, 일정한 치우침, 일부 칸 모름(0)을 넣는 것이 좋다.

**학습 방침: 배치용 관측으로 바로 강화학습, critic에만 특권 정보 (asymmetric actor-critic)**
- 행동을 내는 정책(actor)은 처음부터 배치 때와 같은 입력만 받는다: `observation_perceptive`(추정값 + 잡음 섞은 `height_scan`).
  학습 결과를 그대로 ONNX + 카드로 가져와 쓴다. 강화학습 교사를 따로 만들어 모방학습으로 옮기는 두 단계는 거치지 않는다.
- 점수를 매기는 쪽(critic)은 학습 중에만 쓰므로 특권 정보(참 지형 높이, 참 속도, 발 접촉, 지면 마찰)를 받아도 된다. 학습이 쉬워진다.
- 두 단계(교사 -> 학생 증류)가 필요한 경우: 잡음 많은 입력만으로는 학습이 안 될 때, 큰 정책을 작게 줄일 때, 입력 센서를 바꿀 때
  (예: LiDAR -> 카메라). 그때는 `train/imitate_trot.py`의 DAgger를 신경망 교사로 바꿔 쓴다.
- 룰 기반 교사 모방(`go2_trot_bc_*`)은 지형 판단 없는 기본 보행을 CPU로 빠르게 만들어 배치 경로를 시험하는 용도로 남긴다
  (룰 기반 교사를 넘어설 수 없고, 미세한 문턱값 판단은 따라 하기도 어려웠다: 위 `go2_trot_bc_terrain`).

## 저사양을 고려해 의도적으로 뺀 것

- **UE5 / 카메라 센서**: GPU 드라이버가 없어 불가 (LiDAR는 CPU 레이캐스트로 구현). 렌더러는 포즈 스트림을 받는 얇은 클라이언트로 나중에 붙인다. 연동 계획과 추천 PC 제원은 [`docs/ue5_integration.md`](docs/ue5_integration.md).
- **MJX / 강화학습**: GPU 없이는 학습 처리량이 나오지 않음. 학습은 다른 장비에서 하고 ONNX 정책만 가져오는 구조로 간다.
- 그림자 끔, 뷰어 갱신 30 Hz 제한, 지형 20 m × 20 m @ 5 cm.

## 알려진 한계

- 트롯 보행기는 규칙 기반이라 요철에서 방위가 최대 약 18° 흔들리고 측방 이동이 남는다 (측방 위치 제어 없음).
- 지형 인지 트롯도 자국 안을 따라 걷기(hmmwv_follow)와 15 cm 자국에서는 자주 넘어진다 (위 지형 인지 보행 절).
- 실시간 루프가 Python이므로 지터 보장은 없다. 문서 §11대로 RT 루프는 추후 C++로 옮긴다.
- 지형 갱신은 발 근처 셀을 보류하므로 로봇이 홈 위에 서 있으면 반영이 늦어진다 (의도된 동작).
