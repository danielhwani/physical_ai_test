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
python conformance/test_perception.py                    # 지형 인지: 높이 지도, 디딜 곳, LiDAR 지도 정확도, 점군만으로 재현
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
- 그 속도를 적분한 **주행거리 위치(odom)**는 출발점이 원점이고 시간이 지나며 조금씩 틀어진다. 높이 지도는 이 좌표계에 쌓는다.

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
  알고리즘 표시는 알고리즘 좌표계 `odom`에 있고, 러너가 `world -> odom`을 시작 위치로 한 번 발행해 맞춘다.
- **갱신 속도**: 화면의 지도는 0.5초마다, 디딜 곳은 0.1초마다 갱신한다. 알고리즘 안의 지도는 스캔마다(0.1초) 바뀌므로 화면 지도가 조금 늦게 보일 수 있다.
- 같은 프로세스 실행은 러너가, `--controller-node`는 알고리즘 노드가 발행한다 (명령을 보낸 뒤 발행하므로 명령 지연에 영향 없음).

**MOP 추가**: `edge_touchdowns` = 착지 중 참 지형 모서리(발 주변 ±4 cm 높이 차 > 2 cm)에 디딘 횟수 (판정자 쪽 참값으로 계산).

**결과** (시드별 센서 잡음을 바꿔 여러 번. 한 번 실행은 성공/실패가 우연히 갈려 판단에 쓰지 않았다)

| 시나리오 (시드 수) | 지형 모름 | 지형 인지 |
|---|---|---|
| vehicle_crossing (5) | **4번 넘어짐**, 전진 약 2.1 m, 모서리 착지 10~24 | **0번**, 전진 약 5.0 m (두 자국 통과), 모서리 착지 0~3 |
| vehicle_crossing_soft, 자국 15 cm (3) | 3번 넘어짐 | 2번 넘어짐 |
| hmmwv_follow, 자국 안을 따라 걷기 (5) | 4번 넘어짐 | 3번 넘어짐 |
| flat_trot (3) | 전진 3.86~3.93, CoT 2.40~2.46 | 3.85~3.93, CoT 2.44~2.51 (차이 없음) |
| rough_rut (3) | 전진 4.10~4.25, CoT 3.46~3.87 | 4.03~4.50, CoT 3.32~3.86 (차이 없음) |
| footprints (3) | 전진 4.73~4.79 | 4.73~4.79 (차이 없음) |

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

학습 시간 참고: 이 PC(메모리 7.7 GB, 스왑 없음)에서 작업자 5개로 돌리면 다른 프로그램과 함께 메모리가 모자라 시스템이 멈췄다.
작업자 하나가 약 0.55 GB를 쓰므로 `--workers 3`(기본값)을 권장한다.

## 저사양을 고려해 의도적으로 뺀 것

- **UE5 / 카메라 센서**: GPU 드라이버가 없어 불가 (LiDAR는 CPU 레이캐스트로 구현). 렌더러는 포즈 스트림을 받는 얇은 클라이언트로 나중에 붙인다. 연동 계획과 추천 PC 제원은 [`docs/ue5_integration.md`](docs/ue5_integration.md).
- **MJX / 강화학습**: GPU 없이는 학습 처리량이 나오지 않음. 학습은 다른 장비에서 하고 ONNX 정책만 가져오는 구조로 간다.
- 그림자 끔, 뷰어 갱신 30 Hz 제한, 지형 20 m × 20 m @ 5 cm.

## 알려진 한계

- 트롯 보행기는 규칙 기반이라 요철에서 방위가 최대 약 18° 흔들리고 측방 이동이 남는다 (측방 위치 제어 없음).
- 지형 인지 트롯도 자국 안을 따라 걷기(hmmwv_follow)와 15 cm 자국에서는 자주 넘어진다 (위 지형 인지 보행 절).
- 실시간 루프가 Python이므로 지터 보장은 없다. 문서 §11대로 RT 루프는 추후 C++로 옮긴다.
- 지형 갱신은 발 근처 셀을 보류하므로 로봇이 홈 위에 서 있으면 반영이 늦어진다 (의도된 동작).
