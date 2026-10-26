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
python conformance/test_rt_core.py                       # C++ 실시간 코어: 규약 일치, 물리 비트 동일, 닫힌 고리 동일, 실시간 실행
python conformance/test_dis_console.py                   # DIS 콘솔: 핸드셰이크, 고장 주입, 실행 제어, 링크 감시, 제어권·인증·원본 기록, HMMWV 투입·흙(Chrono), 기록으로 비트 단위 재실행
python conformance/test_link_jitter.py                   # lockstep 통계적 명령 지연 (시드 결정성, 분포, 로봇 쪽 규칙)
python conformance/compare_link_timing.py <시나리오> --seeds 30   # 실시간 지연 vs lockstep 통계적 지연 비교 (실측 분포도 잰다)
python conformance/test_foot_soil.py                     # 무른 흙 발 상호작용 (SCM 기준 침하, 흙별 걷기)
python conformance/test_render_stream.py                 # 렌더 스트림 계약 (스트림만으로 장면 재구성 = 시뮬레이터)
python conformance/test_map_server.py                    # 맵 서버 1단계: 합치는 규칙 (min, NaN, 순서), 소유 영역, 서버 없이 / 있게 비트 동일
python conformance/test_map_remote.py                    # 맵 서버 2단계: 별도 프로세스 서버 + 클라이언트 (주고받기, 다른 바탕 거부, 감사, 늦게 붙기, 지연, 바탕 파일, 빈칸 복구, 이어 시작)
python conformance/test_map_base.py                      # 맵 서버 2c: 바탕 파일 (쓰고 읽기, 고친 타일 찾기, 시나리오 바탕 = 시작 지형)
python conformance/test_map_edit.py                      # 맵 서버 3a: 실행 중 운용자 편집이 물리에 들어가고, 재현 시나리오가 비트 동일 (약 40초)
python conformance/test_map_render.py                    # 맵 서버 3b: 변경 700개 뒤에 붙은 렌더러 (패치 토픽은 틀리고, 맵 서버는 같음)
python conformance/test_map_chrono.py                    # 맵 서버 3c: TCP 입구, ROS2 없는 클라이언트, Chrono 소유자 비트 동일 (약 1분)
python conformance/compare_variants.py [--policy card.yaml]   # cpu vs mjx 설정의 MOP 분포 비교
python conformance/probe_scm_injection.py [--record build/scm_press.npz] [--plot PNG]   # SCM에 바깥 높이 넣기: 접촉은 따라가고 다짐 이력은 안 넘어감 (docs/map_server.md 5.5). 어느 환경이든: chrono 환경으로 다시 실행
python conformance/probe_scm_compaction.py [--record build/scm_compaction.npz]   # 다짐 -> 단단한 흙 콜백: kphi ×25로 다짐 흉내 (약 1분)
python conformance/probe_scm_virtual_compaction.py [--record build/scm_virtual_compaction.npz] [--offset sinkage|iterate]   # 가상 다짐: 자국 전 높이를 넣고 한 번 다시 누르면 단일 SCM과 같음 (세 방법 비교, 약 35초)
python conformance/probe_scm_rolling.py [--record build/scm_rolling.npz]   # 구르는 구: 높이만 / 굴려서 가상 다짐 vs 단일 SCM (침하, 구름 저항, 약 30초)
python conformance/probe_scm_offset_compare.py [--record build/scm_offset_compare.npz]   # 가상 다짐 두 보정 비교: 되풀이 vs sinkage (약 1분 20초)
python conformance/probe_scm_wheel.py [--record build/scm_wheel.npz]   # 구동 바퀴: 미끄럼률, 여러 번 지나간 자리 (네 경우, 약 2분 30초. 기록 없이는 한 경우)
python conformance/plot_scm_probe.py build/scm_<시험>.npz [-o PNG]   # 기록에서 분석 그래프 (시뮬레이션 없이, 기본 docs/img/scm_*.png)
python conformance/replay_scm_probe.py [--probe press|rolling|wheel] [--speed 0.5] [--loop]   # 같은 기록을 Godot로 재생 (기록 없으면 시험부터. mj_ros 환경)
python conformance/make_obs_reference.py                 # (관측 명세를 바꾼 경우) 기준 벡터 재생성
python conformance/make_policy_reference.py <card.yaml>  # (ONNX를 바꾼 경우) 정책 입출력 기준 재생성
```

**전체 시험** (`conformance/test_*.py` 14개). pytest 없이 파일마다 `python`으로 돌리며, 한 파일 안에서는 첫 실패에서 멈춘다.
이 PC(메모리 7.7 GB)에서는 동시에 돌리지 말고 차례로 돌린다.
```bash
for f in specs conformance control link_jitter foot_soil policy render_stream terrain_window lidar perception ros_equivalence cosim dis_console rt_core; do
  python conformance/test_$f.py || echo "실패: test_$f"
done
```
마지막 전체 실행 (2026-10-17, DIS 콘솔 `soil`로 로봇 발 흙 바꾸기 추가 후, 로봇 경계 메시지 auto = `go2_rt_msgs` typed): **91개 모두 통과, 약 9분.**

| 시험 파일 | 개수 | 시간 | 다루는 것 |
|---|---|---|---|
| test_specs | 6 | 8 s | 관측 기준 벡터, 카드 관절 재배열, MJX 오버라이드 |
| test_conformance | 2 | 3 s | 결정성, IK-모델 일치 |
| test_control | 6 | 5 s | 로봇 경계, 기구학, 다리 주행거리계, 재현 |
| test_link_jitter | 5 | 5 s | 통계적 명령 지연: 시드 결정성, 분포, 로봇 쪽 규칙, C++ 경로 거부 |
| test_foot_soil | 9 | 30 s | 무른 흙: 지지력 식 = SCM 기준, 서 있는 침하, 구멍, 흙별 걷기, 쟁기질(수평·위), 화면 전용 발자국, 지역마다 다른 흙(진흙 띠), 옵션 끄면 모델 그대로 |
| test_policy | 4 | 3 s | 정책 카드, ONNX 입출력, 결정성, 추론 지연 |
| test_render_stream | 10 | 8 s | 스트림만으로 장면 재구성, 흙 지역 화면 표시, Godot 중계기 메시지·점군·디딜 곳 |
| test_terrain_window | 4 | 13 s | 움직이는 지형 창, 바뀐 셀만 보는 지형 반영 = 전체 비교 규칙 |
| test_lidar | 5 | 56 s | LiDAR 재현, 지형 표면, 자기 몸 제외 |
| test_perception | 6 | 20 s | 높이 지도, 디딜 곳, height_scan 규약 |
| test_ros_equivalence | 4 | 61 s | ROS 노드 = 같은 프로세스 (트롯, 정책 2종, 지형 인지) |
| test_cosim | 7 | 50 s | Chrono 연동, 바퀴 자국, 로봇 발자국 |
| test_dis_console | 13 | 111 s | DIS 콘솔, 고장, 제어권·인증, HMMWV·흙, 무른 흙 시나리오의 흙 바꾸기, 비트 단위 재실행 |
| test_rt_core | 13 | 196 s | C++ 코어: 비트 동일, 두뇌 연결 방식, 실시간, 콘솔 요청 재실행(예약·즉시), LiDAR, Chrono, 무른 흙 |

- 2026-10-11 오전 재실행에서 `test_dis_console_on_rt_core`가 한 번 실패했다 (실시간 0.976 m, Python 재현 0.978 m, 늦은 명령 0).
  원인을 찾아 고쳤다 (실시간 코어 절의 "콘솔 요청을 기록대로 다시 실행하기"). 그 전 표의 74개는 72개를 잘못 센 것이다.

시나리오 값은 파일을 고치지 않고 `--set 키=값`으로 바꿀 수 있다 (여러 번 가능, 없는 키는 오류, 바꾼 값은 기록 메타데이터에 남음).
`python -m sim.sweep <시나리오> --param <키> --values ...`는 값 하나를 바꿔 가며 MOP 비교표(`runs/sweep_*.csv`)를 만든다.
Chrono 시나리오의 MOP에는 지면 변형량(`deformed_cells`, `deform_mean_m`, `deform_max_m`)이 들어간다.

결과는 `runs/<시나리오>_<시각>/`에 `timeseries.parquet`(시계열, 메타데이터에 버전 조합)과 `summary.json`(MOP), `map_updates.json`(맵 서버 변경 기록 요약, `docs/map_server.md`)으로 남는다.

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
| `docs/map_server.md` | 맵 서버 설계 초안 (타일, 바탕 배포 + 변경 동기화, 여러 물리엔진·렌더러가 같은 지형. 아직 구현 안 함) | §5.1 |
| `sim/chrono_link.py` | Chrono 연동 (러너 쪽): 프로세스 실행, 격자 정합 검사, 파이프라인 동기, 지형/차량 반영, 근접 판정 | §5 |
| `sensors/` | 엔진 무관 센서 모델 (`lidar.py` LiDAR, `proprio.py` 관절·IMU·발 힘 잡음). 명세는 `specs/go2_sensors.yaml` | §8 |
| `viz/sensor_renderer.py`, `viz/sensor_node.py` | 센서 렌더러 (중립 스트림 -> 장면 -> 레이캐스트 -> 점군)와 ROS2 별도 프로세스 노드 | §8 |
| `cosim/` | 엔진 간 연결 규약 `wire.py`(표준 라이브러리), Chrono 서버 `chrono_server.py`(chrono 환경) | §5 |
| `sim/recorder.py` | Parquet 기록, Virtual 출처와 버전 조합 기록 | §4, §9.2, §13.3 |
| `rt/`, `sim/rt_link.py` | C++ 실시간 코어 (물리, 로봇 쪽 경계, 고정 주기 루프)와 Python 관리 프로세스, 공유 메모리 규약 | §11 |
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
- **별도 노드** (`--controller-node`): `/robot/low_state`(`go2_rt_msgs/LowState`) + `/robot/imu`, `/robot/joint_states`(측정값) -> `control/ros_node.py`
  -> `/robot/low_cmd`(`go2_rt_msgs/LowCmd`). 메시지 형식은 아래 "로봇 경계 ROS2 메시지". 운용자 명령은 `/cmd_vel`(원격 조종 도구, 즉시 적용)과 `/cmd_vel_stamped`(시나리오 이벤트, 시각을 붙여 그 시각 상태부터 적용). 로봇 쪽은 마지막으로 받은 명령을 실행하고, 명령 지연을 MOP
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

### 통계적 명령 지연: lockstep에서 실시간과 비슷한 지연 패턴 (2026-10-11)

고정 지연만 쓰면 lockstep 시험의 명령은 언제나 정확히 한 주기 늦다. 실시간 실행과 실제 링크에서는 가끔 더 늦거나 사라진다.
시나리오 `control_link`에 분포를 주면 명령마다 추가 지연을 뽑아, 그 "가끔 늦는" 상황을 결정적으로(시드로) 재현한다 (`sim/link_jitter.py`).

```bash
python -m sim.runner scenarios/flat_trot_jitter.yaml                            # 이 PC 실측 분포 (ROS2 두뇌, 아래 표)
python -m sim.runner scenarios/rough_rut.yaml --set 'control_link.extra_steps_probs=[0.05,0.02]' --set control_link.drop_probability=0.01   # 거친 링크
python -m sim.sweep scenarios/rough_rut.yaml --param seed --values 0 1 2 3 4 --set 'control_link.extra_steps_probs=[0.004]'           # 시드 여러 개로 통계 비교
python conformance/test_link_jitter.py                                          # 시험 (약 5초)
```
```yaml
control_link:
  latency_steps: 1               # 기본 지연 (지금과 같음)
  extra_steps_probs: [0.004]     # 명령마다 +1주기, +2주기, ... 늦을 확률 (실측 히스토그램을 그대로)
  # late_probability: 0.02       # 또는 약식: 늦을 확률, 늦으면 1..max_extra_steps 주기 균등 (extra_steps_probs와 함께 쓰지 않음)
  # max_extra_steps: 2
  drop_probability: 0.0          # 명령이 사라질 확률 (무선 손실)
```
**켜고 끄기**: 분포를 주지 않으면 이전과 똑같이(비트 단위로) 돈다. 이때도 고정 한 주기 지연(20 ms, `link.latency_steps: 1`)은 있다.
빠지는 것은 랜덤 추가 지연과 손실뿐이다.

| 실행 | 지연 |
|---|---|
| `flat_trot.yaml`, `rough_rut.yaml` 등 기존 시나리오 (옵션 없음) | 고정 20 ms (이전과 비트 동일) |
| 기존 시나리오 + `--set control_link...` | 고정 20 ms + 랜덤 추가 지연·손실 |
| `flat_trot_jitter.yaml` (파일 안에 `extra_steps_probs: [0.004]`) | 고정 20 ms + 랜덤 (실측 0.4%). 끄려면 `--set 'control_link.extra_steps_probs=[]'` |

- **로봇 쪽 규칙**은 C++ 코어와 같다: 도착한 명령 중 가장 새 것(계산에 쓴 상태 시각이 가장 늦은 것)을 실행하고, 지금 실행 중인 것보다
  오래된 명령은 버린다(뒤늦게 와서 추월당한 명령). 새 명령이 없으면 마지막 명령을 유지한다. 링크 두절 전에 보낸 명령은 복구 뒤 버린다.
  그래서 +2주기로 뽑힌 명령도 다음 명령에 추월당하면 실행되지 않고, 실행한 명령의 나이는 대개 20 ms, 가끔 40 ms다.
- **결정성**: 난수는 시나리오 시드로 정해진다 (같은 시드 = 같은 지연 순서). 센서 잡음 난수와 따로 뽑으므로 지연을 켜도 잡음 순서는 같다.
  분포를 주지 않으면 이전 고정 지연 경로 그대로다 (flat_trot 시계열 비트 동일 확인).
- **`--set control_link...`**: `control_link`가 없는 시나리오에도 만들 수 있다 (`OPTIONAL_SECTIONS`). 목록 값은 따옴표로 감싼다.
- **MOP**: `cmd_late_steps`(실행한 명령이 기본 지연보다 오래된 주기 수, `sim.rt_link` 실시간 MOP와 같은 뜻, 모든 실행에 나옴),
  지연을 켰을 때 `link_cmds_sent / late / dropped / stale`, `link_extra_steps_hist`. 기록 메타데이터 `control_link.jitter`에 분포가 남는다.
- **적용 범위**: `sim.runner`의 같은 프로세스 실행 전용. `--controller-node`(별도 ROS2 노드)는 실제 지연을 쓰므로 무시하고 알린다.
  `sim.rt_link`(C++ 코어)는 이 설정이 있으면 실행하지 않는다 (실시간은 실제 지연, lockstep 비트 동일 비교가 깨지므로).

**이 PC 실측 분포** (C++ 실시간 코어 `sim.rt_link`, flat_trot, 시스템 튜닝 없음, 시작 0.5초 제외. 기록의 `cmd_state_t`로 셈):

| 두뇌 | 실행 | 제어 주기 | +1주기 | +2주기 이상 |
|---|---|---|---|---|
| inproc | 12초 | 600 | 0 | 0 |
| shm | 12초, 지형 인지 60초 | 약 3600 | 0 | 0 |
| ros (같은 PC) | 12초, 60초 ×2 | 6548 | 27 (0.41%) | 0 |
| ros, 지형 인지 | 60초 | 2976 | 0 | 0 |

`flat_trot_jitter.yaml`의 `[0.004]`는 ros 두뇌의 값이다. 다른 PC의 두뇌나 무선 링크는 그 장비에서 같은 방법으로 재서 넣는다.

**예 (rough_rut, 시드 0~4, 시드는 지형과 지연 순서를 함께 바꿈)**:

| 링크 | 넘어짐 | 전진 평균 (m) | 측방 표류 절댓값 평균 (m) | CoT 평균 |
|---|---|---|---|---|
| 고정 한 주기 | 0/5 | 3.96 | 0.16 | 3.87 |
| 실측 `[0.004]` | 0/5 | 3.94 | 0.35 | 3.80 |
| 거친 링크 `[0.05, 0.02]` | 0/5 | 3.65 | 0.39 | 4.35 |

- 0.4%만 늦어도 개별 실행 결과는 바뀐다 (걸음이 조금씩 달라지며 퍼짐). 그래서 한 번의 실행으로 비교하지 말고 시드 여러 개로 본다.
- 이 규칙 기반 트롯은 5~7% 늦는 링크에서도 넘어지지 않지만 전진이 줄고 표류와 CoT가 는다.

**검증: 실제 실시간 실행과 비교** (2026-10-11, rough_rut, 시드 0~29, 이 PC, 시스템 튜닝 없음)

```bash
python conformance/compare_link_timing.py scenarios/rough_rut.yaml --seeds 30    # 약 13분. --brain shm|inproc|ros, --keep
```
시드마다 A: lockstep 고정 한 주기, C: C++ 실시간 코어 + ROS2 두뇌(실제 지연), B: lockstep + C에서 잰 분포, B0: lockstep + `[0.004]`(flat_trot 실측).

| | A 고정 | C 실시간 (실제) | B 잰 분포 (`[0.0045]`, 시드 10~29는 `[0.0043]`) | B0 `[0.004]` |
|---|---|---|---|---|
| 넘어짐 | 0/30 | 0/30 | 0/30 | 0/30 |
| 전진 평균 ± 표준편차 (m) | 3.73 ± 0.73 | 3.69 ± 0.70 | 3.87 ± 0.51 | 3.85 ± 0.45 |
| 측방 표류 절댓값 평균 (m) | 0.39 | 0.43 | 0.42 | 0.39 |
| CoT 평균 ± 표준편차 | 4.32 ± 1.65 | 4.40 ± 1.61 | 3.97 ± 0.73 | 3.98 ± 0.61 |
| 같은 시드 A와 전진 차이 평균 (m) | 0 | 0.44 | 0.55 | 0.41 |
| 실행별 늦은 명령 수 (평균, 분산) | 0 | 3.17, 4.70 | 3.10, 2.64 | |

- **실제 분포**: 실시간 21780주기 중 95주기(0.44%)가 한 주기 더 늦었고, 두 주기 이상 늦거나 사라진 명령은 없었다. flat_trot에서 잰 0.4%와 같다.
  늦게 걸린 고장·지형 변경(`late_applies`)은 0.
- **실시간과 lockstep의 차이는 명령 시각뿐이다**: 늦은 명령이 하나도 없던 실시간 실행 2개(시드 9, 13)는 lockstep과 결과가 같았다.
- **한 실행에 주는 영향의 크기는 맞다**: 같은 시드에서 고정 지연 결과와의 차이가 실제(C) 0.44 m, 모델(B) 0.55 m, B0 0.41 m로 비슷하다.
  0.4%의 늦은 명령은 결과를 한쪽으로 미는 것이 아니라 걸음을 조금씩 바꿔 퍼뜨린다 (시드마다 늘기도 줄기도 함).
- **시드 30개의 MOP 분포로는 넷을 구분하지 못한다**: 평균 차이와 표준편차 차이의 부트스트랩 95% 구간이 모두 0을 포함한다
  (C-A 전진 [-0.39, 0.34] m, C-B 전진 [-0.50, 0.12] m, C-B CoT [-0.11, 1.12]). 이 PC의 이 정도 지연에서는 고정 지연 lockstep도
  실시간과 통계적으로 구분되지 않으므로, 통계적 지연이 MOP 분포를 실시간에 "더 가깝게" 만든다는 근거도 아직 없다.
  다만 B·B0의 표준편차가 A·C보다 작은 경향(전진 0.51 vs 0.70, CoT 0.73 vs 1.61)은 구간이 0에 가까워, 시드를 늘리면 차이가 드러날 수 있다.
- **실제 지연은 조금 몰려 온다**: 실행별 늦은 명령 수의 분산이 평균보다 크다 (실제 4.70 / 3.17, 모델 2.64 / 3.10). 시작 직후, 지형 패치
  이벤트(4초) 부근에 몰리는 실행이 있었다 (한 실행에서 4.04~4.72 s 사이 4번). 명령마다 독립으로 뽑는 지금 모델은 이 몰림을 만들지 않는다.
- **정리**: 통계적 지연의 쓸모는 (1) 실측보다 거친 링크(무선, 다른 PC, 부하)를 결정적으로 시험하는 것, (2) "가끔 늦는" 상황을 시드로 재현하는 것이다.
  이 PC의 같은 PC 링크(0.4%)에서는 고정 지연 lockstep만으로도 실시간과 같은 MOP 분포를 낸다. 다른 장비에서는 이 스크립트로 분포를 다시 잰다.
  몰림을 넣는 모델(예: 두 상태 마르코프)은 몰림이 MOP에 영향을 주는 경우가 확인되면 넣는다.

**구조와 `--rviz`: 실시간처럼 보여도 lockstep이다.**

```
           시나리오 yaml (+ control_link 지연 분포)
                │
                ▼
  제어기 ◄──[링크 지연 모델]──► runner ◄──► MuJoCo (물리 10스텝 = 제어 1주기 20 ms)
 (트롯 두뇌)                     │   └─ 벽시계 페이싱: 계산이 앞서면 sleep (--realtime, --rviz면 자동)
  같은 프로세스                  │
                    ┌────────────┼────────────┐
                    ▼            ▼            ▼
              RViz (ROS2 토픽)  MOP 계산    runs/ 기록
              (보여주기만)
```
- `--rviz`를 주면 `--realtime`이 자동으로 켜진다. runner는 한 제어 주기를 계산한 뒤, 시뮬레이션 시각이 벽시계보다 앞서 있으면
  그만큼 잠든다. 이 PC에서 한 주기 계산은 20 ms보다 훨씬 짧아 화면이 실제 시간 속도로 흐른다. 속도를 맞추는 것은 RViz가 아니라 runner다.
- 계산 순서는 그대로이고 기다리기만 넣으므로 결과는 `--rviz` 없이 돌린 것과 같다 (두뇌가 같은 프로세스, 지연은 시드로 결정).
  통계적 지연을 켜도 마찬가지다. 한 주기 계산이 20 ms를 넘으면(Chrono 시나리오 등) 화면만 느려지고 결과는 같다.

| | `sim.runner --rviz` | `sim.rt_link` 실시간 |
|---|---|---|
| 시간 기준 | 시뮬레이션 시각 (벽시계에 맞춰 늦출 뿐) | 벽시계 (몸은 기다리지 않음) |
| 두뇌가 늦으면 | 다 계산할 때까지 기다림 | 지난 명령으로 진행 (실제로 늦음) |
| 계산이 20 ms를 넘으면 | 화면이 느려짐 (`late_control_steps`), 결과는 같음 | 주기 초과, 결과가 달라짐 |
| 지연 | 고정 한 주기, 또는 시드로 뽑은 통계적 지연 | 실제 지연 |

즉 `sim.runner --rviz`는 lockstep을 사람이 볼 수 있는 속도로 재생하는 것이고, 통계적 지연은 그 lockstep 안에서 `sim.rt_link` 실시간의
지연 패턴을 흉내 낸다.

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

## Godot 4 렌더러 (`--godot`, 2026-10-18)

RViz, MuJoCo 화면과 같은 중립 렌더 스트림을 받는 세 번째 렌더러다. 시뮬레이터는 바꾸지 않았다.

```bash
python -m sim.runner scenarios/flat_trot.yaml --godot                       # Python
python -m sim.rt_link scenarios/soil_strip.yaml --brain shm --godot         # C++ 코어 (실시간)
python -m sim.runner scenarios/vehicle_crossing.yaml --godot                # HMMWV (Chrono)
python -m sim.runner scenarios/vehicle_crossing.yaml --map-server --godot   # 지형을 맵 서버에서 (3b)
python -m sim.runner scenarios/vehicle_crossing.yaml --map-server --ros2 --realtime   # 렌더러를 나중에 붙일 시뮬레이터 (--ros2: 바디·상태 스트림)
python -m viz.godot_launch                                                  # 다른 터미널에서 나중에 붙이기 (맵 서버가 있으면 거기서: 늦게 열어도 이미 생긴 자국이 보인다)
python -m sim.rt_link scenarios/rut_crossing.yaml --brain shm --godot-lidar  # + LiDAR 점군
python conformance/test_render_stream.py                                    # 중계기 메시지 시험 포함
```

**Godot을 띄우는 방법** (2026-10-26). 같은 시뮬레이션을 보되, 렌더러를 누가 언제 띄우고 지형을 어디서 받느냐가 다르다.

| | 1. `--godot` | 2. `--map-server --godot` | 3. `--map-server --ros2` + `viz.godot_launch` |
|---|---|---|---|
| 터미널 | 하나 | 하나 | 둘 (시뮬레이터, 렌더러) |
| Godot 띄우는 쪽 | 시뮬레이터 (시작할 때) | 시뮬레이터 (시작할 때) | 사용자 (실행 중 아무 때나) |
| 지형 출처 | 시뮬레이터 스트림 (`/sim/terrain_patch`) | 맵 서버 | 맵 서버 (있으면 스스로 고름) |
| 늦게 붙기 | 해당 없음 (처음부터 같이) | 해당 없음 | 됨: 그 전에 생긴 자국까지 보인다 |
| 창을 닫으면 | 시험도 끝남 | 시험도 끝남 | 시험은 계속됨 (다시 열어도 됨) |
| 쓰는 때 | 그냥 보기 (가장 간단) | 맵 서버 시험을 보면서 | 렌더러를 나중에 붙이거나 3b 확인 |

```bash
python -m sim.runner scenarios/vehicle_crossing_lidar.yaml --godot                          # 1. 로봇이 자국을 넘는 모습 보기
python -m sim.runner scenarios/vehicle_crossing_lidar.yaml --map-server --godot             # 2
python -m sim.runner scenarios/vehicle_crossing_lidar.yaml --map-server --ros2 --realtime   # 3. 터미널 1
python -m viz.godot_launch                                                                  # 3. 터미널 2 (같은 ROS_DOMAIN_ID)
```
- 3에서 시뮬레이터에 `--godot`을 같이 주면 렌더러와 같은 포트(4243)를 써서 부딪힌다. `--ros2`만 준다 (바디·위치·상태·장면 매니페스트는
  이 스트림에서 온다. 없으면 Godot 정보판에 `WAITING FOR SIMULATOR`).
- `viz.godot_launch`는 맵 서버를 띄우지 않는다. 떠 있는 맵 서버가 있으면 거기서 받고, 없으면 시뮬레이터 스트림에서 받는다 (`--terrain
  map|stream`으로 정할 수 있다). 어느 쪽으로 정했는지는 `build/godot_view/godot_bridge.log`의 `지형:` 줄.
- 늦게 붙으면 시뮬레이터 스트림은 최근 패치 500개만 다시 준다: 실행이 길면 앞부분을 놓친다. 맵 서버는 스냅샷으로 지금 지형 전체를 준다
  (`conformance/test_map_render.py`: 변경 700개 뒤에 붙으면 스트림은 약 18000칸이 틀리고 맵 서버는 같다).

```
[시뮬레이터] ── ROS2 중립 스트림 ──▶ viz/godot_bridge.py ── TCP 127.0.0.1:4243 (한 줄에 JSON) ──▶ godot/ (Godot 4 프로젝트)
 (그대로)      매니페스트, 지형 패치,    (중계기, mj_ros)                                      GDScript main.gd: 메시, 지형,
               화면 발자국, TF, 상태                                                         발자국, 흙 지역, 정보판, 카메라
```
- **설치**: Godot 4.7.2 공식 바이너리를 `~/Applications/godot`에 둔다 (공식 SHA512 확인). 다른 위치면 `GODOT_BIN`.
  `--godot`이 중계기와 Godot를 함께 띄우고, 창을 닫으면 끝난다 (RViz와 같다).
- **왜 중계기**: GDScript는 ROS2를 직접 받지 못한다. 중계기가 받아 Godot가 바로 쓸 형태로 TCP로 넘긴다 (Godot 기본 TCP만 써서 추가 패키지 없음).
  지형 창(로봇 주변 6 m, 2 m 벗어나면 옮김), 화면 발자국(지형 - 깊이), 흙 지역 색, 높이 명암은 RViz 어댑터와 같은 규칙으로 중계기에서 계산하고,
  Godot는 받은 격자로 메시만 만든다 (약 4 Hz, 바뀌었을 때만). 포즈는 약 30 Hz.
- **좌표**: 스트림은 ROS(REP-103: x 앞, y 왼쪽, z 위). Godot 장면 맨 위 노드 하나를 X축 -90° 돌려(ROS (x, y, z) -> Godot (x, z, -y))
  그 아래에서는 ROS 좌표를 그대로 쓴다. 두 좌표계 모두 오른손 좌표계라 회전만 하면 되고, 변환하는 곳이 이 한 군데다.
  OBJ는 Godot가 직접 읽는다 (같은 PC). 법선은 원래 감김(바깥이 반시계)으로 구하고, 삼각형 순서는 Godot 앞면(시계)으로 뒤집는다.
- **그리는 것**: 로봇·HMMWV 메시, 지형(패치·지형 창·화면 발자국), 흙 지역 색과 이름, 발 접촉 구, 경로, 명령 화살표, 정보판(끝나면 최종 MOP),
  몸통을 따라가는 카메라.
- **카메라 조작** (바라보는 점 둘레를 도는 궤도 카메라. 화면 아래에 안내 문구):
  왼쪽 끌기 = 돌리기, 휠 = 줌 인/아웃(0.5~60 m), 오른쪽·가운데 끌기 = 옮기기, F = 몸통 따라가기 켜기/끄기, R = 처음 시점.
  끄는 중인지는 마우스가 움직일 때 눌린 버튼으로 판단한다 (창 밖에서 버튼을 떼도 계속 돌아가지 않는다).
  처음 시점은 `GODOT_ARGS="--cam=방위,내려다보는각,거리"`(도, 도, m)로 바꿀 수 있다. 기본은 `-45,22.7,3.37`(왼쪽 뒤 위).
- **점군** (`--godot-lidar`, 선택): `--godot`에 더해 LiDAR 점군(`/sensors/<이름>/points`, 높이 색: 낮음 파랑 -> 높음 빨강)과
  알고리즘 지형 지도(`/control/terrain_map`, 지도 자기 색, 큰 점)를 그린다. 센서가 없으면 `front_lidar`를 켠다. L = 점군 보이기/숨기기.
  중계기(`--points`)가 점군 토픽을 찾아 구독하고, 센서 좌표계 -> 월드 변환(/tf)을 해서 약 10 Hz로 넘긴다.
  지형 지도는 알고리즘이 같은 프로세스일 때만 나온다 (`sim.runner`, `sim.rt_link --brain inproc`. RViz와 같다).
  `sim.runner`는 LiDAR를 같은 프로세스에서 계산해 실시간보다 느리다 (아래 "rut_crossing + LiDAR" 표). 실시간으로 보려면 `sim.rt_link --brain shm`.
  확인: rough_rut + 지형 인지(`sim.runner`, 점군과 지도), rut_crossing(`sim.rt_link --brain shm`, 점군, 늦은 스텝 0).
  `test_godot_bridge_points`: 센서 좌표 -> 월드 변환, 높이 색, TF 전에는 보내지 않음.
- **디딜 곳** (2026-10-24, 지형 인지 보행기일 때 저절로): 두뇌의 `/control/footholds`(RViz와 같은 MarkerArray)를 중계기가 찾아 구독하고
  odom -> 월드로 옮겨 약 10 Hz로 넘긴다. 다리마다 고른 곳(공중 다리 주황 큰 구, 디딘 다리 하늘색), 원래 자리(회색 작은 구), 옮긴 만큼의 흰 선분.
  선분은 다리에 가려도 보이게 그린다. L로 점군과 함께 끄고 켠다. 지형 지도처럼 알고리즘이 같은 프로세스이거나(`sim.runner`,
  `sim.rt_link --brain inproc`) 다른 프로세스 두뇌가 발행할 때(`--brain ros`, 렌더러가 켜져 있으면 저절로) 나온다. `--brain shm`에서는 안 나온다.
  확인: rut_crossing + 지형 인지(`sim.runner`, `sim.rt_link --brain inproc`, `--brain ros`)에서 구와 선분이 다리 아래에 나왔다
  (ros 실시간: 넘어짐 없음, 늦은 명령 3, 디딜 곳 메시지 67개). 디딘 다리의 구는 발 접촉 표시(초록 구)와 거의 같은 자리라 겹쳐 보인다.
  `test_godot_bridge_footholds`: odom -> 월드, 구 크기·선분 끝점, TF 전에는 보내지 않음.
  (`sim.rt_link --brain inproc` 실시간 지형 인지는 늦은 명령 45개에 넘어졌다. 실시간 지형 인지는 shm/ros 두뇌를 쓴다)
  **`--brain shm`에서 안 나오는 이유**: 지형 지도와 디딜 곳은 두뇌 안에만 있는 상태다. inproc은 `rt_link`가 같은 프로세스의 두뇌 상태
  (`perception_snapshot()`)를 읽어 발행하고, ros 두뇌(`control/ros_node.py`)는 ROS2 노드라 직접 발행한다. shm 두뇌(`control/rt_brain.py`)는
  ROS2 없이 공유 메모리만 쓰고, 공유 메모리 규약에는 로봇 상태·관절 명령·이동 명령·LiDAR 스캔 칸만 있어 내보낼 길이 없다
  (같은 PC 최소 지연용으로 연결을 최소로 뒀다). 필요하면: (1) shm 두뇌가 렌더러가 켜졌을 때만 ROS2로 직접 발행 (단순, 실시간 두뇌에 ROS2가 들어감),
  (2) 공유 메모리에 지형 인지 칸을 더해 `rt_link`가 발행 (규약 v14, C++ 배치도 맞춤). 아직 하지 않았다. 지금은 화면이 필요하면 inproc이나 ros 두뇌를 쓴다.
  ```bash
  python -m sim.runner scenarios/rut_crossing.yaml --godot --set controller.perception.sensor=front_lidar                      # Python
  python -m sim.rt_link scenarios/rut_crossing.yaml --brain ros --godot --set controller.perception.sensor=front_lidar          # 실시간
  python -m sim.rt_link scenarios/rut_crossing.yaml --brain ros --godot-lidar --set controller.perception.sensor=front_lidar    # + 점군·지형 지도
  python -m sim.rt_link scenarios/rut_crossing.yaml --brain shm --godot --set controller.perception.sensor=front_lidar          # 비교: 안 나옴
  ```
  휠로 줌 인하고 낮은 각도로 돌려 보면 잘 보인다. 바퀴 자국 가장자리 근처에서 흰 선이 길어진다 (가장자리를 피해 옮김).
  안 나오면 실행 폴더 `godot_bridge.log`에 `디딜 곳 구독: /control/footholds`가 있는지 본다.
- **렌더러**: 호환 렌더러(OpenGL 3.3). 일반 커널에서는 GTX 1650으로 돈다. RT 커널에서는 NVIDIA 드라이버가 없어 소프트웨어 렌더링이 되며,
  느릴 것으로 보지만 아직 확인하지 않았다.
- **확인** (일반 커널, 시험용 스크린샷 `GODOT_ARGS="--shot=경로 --shot-frames=N"`: N프레임 뒤 PNG로 저장하고 끝냄):
  flat_trot(`sim.runner`, `sim.rt_link`), soil_strip(진흙 띠 색과 이름, 발자국 두 줄, 정보판 `soil mud_strip`),
  vehicle_crossing(HMMWV 메시, 바퀴 자국, 차와 거리). 메시 방향·면·조명이 맞게 나왔다.
  `test_godot_bridge_messages`: 지형 메시지의 높이 = 스트림 지형 - 발자국 깊이, 진흙 띠 색이 짙음, 정보판 문구.
  `test_viz_does_not_depend_on_physics`: 중계기는 MuJoCo도 sim도 불러오지 않는다.
- **화면 글자는 영어와 숫자만**: Godot 기본 글꼴에 한글이 없어, 정보판이나 이름표에 한글을 쓰면 바뀔 때마다 시스템 대체 글꼴을 찾느라
  화면이 거의 멈춘다 (2026-10-24 확인: 같은 상태 메시지가 영어면 정상, 한글이면 150프레임을 20초 안에 못 그림). 한글이 필요하면
  한글 글꼴을 프로젝트에 넣어 기본 글꼴로 지정해야 한다 (하지 않음).
- **로봇이 아닌 출처도 그린다**: 상태에 `info_lines`가 있으면 그 문구를 정보판에 쓰고, `command`가 없으면 경로와 명령 화살표를 생략한다.
  `base` 바디가 없으면 카메라는 장면의 첫 바디를 따라간다. 예: `conformance/replay_scm_probe.py` (SCM 시험 기록 재생:
  Chrono 기록 -> 중립 스트림 -> Godot. `--probe press`: 누르기, 자리 번호 1~4 (뜻은 `docs/img/scm_injection.png` 범례).
  `--probe rolling`: 굴리기, 줄 A / B / C (뜻은 `docs/img/scm_rolling.png` 범례). 구 표면의 빨강·검정 표시점으로 회전이 보인다).
- **아직 없는 것**: 같은 PC 밖의 Godot (중계기는 127.0.0.1만).

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
python -m sim.runner scenarios/vehicle_crossing.yaml --rviz        # 단단한 흙: 자국 약 5.5 cm (지형을 보지 않는 보행기: 첫 자국에 걸림)
python -m sim.runner scenarios/vehicle_crossing_lidar.yaml --godot # 같은 시나리오, 지형 인지 보행기 (시연용: 두 자국을 넘음, 5.04 m)
python -m sim.runner scenarios/vehicle_crossing_soft.yaml --rviz   # 무른 흙: 자국 약 15 cm
python -m sim.runner scenarios/hmmwv_follow.yaml --rviz            # HMMWV 뒤를 따라 왼발이 바퀴 자국 안을 걷기
python -m sim.runner scenarios/footprints.yaml --rviz              # 로봇 발자국 (HMMWV 없이 Chrono를 변형 지면 계산기로만)
python conformance/test_cosim.py                                   # 연동 시험 (약 50초)
python -m sim.runner scenarios/footprints.yaml --set chrono.scm.soil.bekker_kphi=2e7   # 값 바꿔 실행
python -m sim.sweep scenarios/footprints.yaml --param chrono.scm.soil.bekker_kphi --values 2e6 1e7 5e7   # 스윕 비교표
```

**맵 서버와 Chrono: 지형을 누가 쓰나** (2026-10-26, 맵 서버 3단계 3c. 자세한 것은 [`docs/map_server.md`](docs/map_server.md) 8절 "3c 결과")

`--map-server`로 돌리면 **지형을 쓰는 권한이 시뮬레이터에서 Chrono로 옮겨 간다.** 물리 결과는 예전과 비트 단위로 같고, 바뀐 것은 지형이
가는 길이다. 이 구조가 4단계(물리 프로세스 여럿)의 바탕이다.

```
3c 전:  Chrono ──(소켓: 바뀐 흙 노드)──▶ 시뮬레이터 ──(쓰기, 쓴 쪽 "sim")──▶ 맵 서버 ──▶ 다른 클라이언트
3c 후:  Chrono ──(TCP 입구: SCM 영역의 소유자로 직접 쓰기)──────────────▶ 맵 서버 ──▶ 시뮬레이터, 렌더러, 관찰자
        Chrono ──(소켓: 시각 맞추기, 차량 포즈, "어디까지 썼다")──▶ 시뮬레이터
```
- Chrono는 원래도 별도 프로세스(`chrono` 환경, Python 3.12)였지만 지형은 시뮬레이터를 거쳐 갔다. 이제는 Chrono가 SCM 영역의 소유 선언과
  쓰기를 직접 하고(맵 서버 기록에 `chrono/own`, `chrono/set`), 시뮬레이터는 운용자 편집·렌더러처럼 서버에서 **받는 쪽**이다.
- Chrono 환경은 ROS2(Humble, Python 3.10)를 쓸 수 없어서 맵 서버에 **TCP 입구**(127.0.0.1 전용, 같은 JSON 규약)를 두었다. ROS2 없는 다른
  엔진(게임 엔진, 다른 PC의 도구)도 같은 길로 붙을 수 있다.
- 그대로인 것: Chrono는 여전히 시뮬레이터가 띄우고, 시각 맞추기(0.04 s)와 차량 포즈는 소켓으로 간다. 시뮬레이터는 Chrono의 지형 변경을
  붙잡아 두었다가 예전과 같은 교환 시각에 받아 넣는다 (그래서 물리가 보는 시각과 값이 같다. 기다린 시간은 MOP `map_partner_wait_max_ms`,
  수 ms). Chrono 변경은 재현 시나리오와 받은 변경 MOP에 넣지 않는다 (물리를 다시 돌리면 다시 나온다).
- **의미**: 예를 들어 HMMWV 두 대를 서로 다른 Chrono 프로세스로, 로봇을 또 다른 프로세스로 돌릴 때, 예전 구조면 모든 지형이 시뮬레이터
  하나를 거친다 (병목이자 단일 고장점). 이제는 물리 엔진마다 자기 영역을 지도 하나에 직접 쓰고, 다른 프로세스는 그 지도에서 모두의 자국을
  받는다 ("물리 엔진마다 자기 땅의 주인, 지도는 하나").
- **아직 반쪽**: Chrono는 지도에 쓰기만 하고, 다른 쪽 변경(운용자 편집, 다른 차량 자국)을 자기 흙(SCM)에 넣지는 못한다 (사본은 받지만 SCM은
  모른다: 설계 9절). 그래서 SCM 영역에 편집을 넣으면 다음 Chrono 쓰기가 덮는다. 두 Chrono가 서로의 자국을 밟으려면 이것이 필요하다
  (높이 넣기는 확인함, 남은 것은 다짐 상태 전달: 아래 "맵 서버와 SCM 다짐 전달").
- 확인: `python -m sim.runner scenarios/vehicle_crossing.yaml --map-server` (MOP `map_partner_wait_max_ms`, `map_received_changes 0`,
  실행 폴더 `map_updates.json`의 `counts`가 `chrono/own`, `chrono/set`), `python conformance/test_map_chrono.py`.

**로봇 발자국은 두 가지** (2026-10-26 정리). 시나리오에 따라 계산하는 곳과 지도에 들어가는지가 다르다.

| | Chrono 발자국 | 차량 바퀴 자국 | MuJoCo 안 무른 흙 |
|---|---|---|---|
| 시나리오 | `footprints`, `footprints_free`, `dis_footprints` (`chrono.robot_feet`) | `vehicle_crossing` 등 (`chrono.vehicle`) | `soil_strip` (`foot_soil`) |
| 계산 프로세스 | Chrono (별도, 차량과 같은 SCM) | Chrono (별도) | 시뮬레이터 안 (`sim/foot_soil.py`) |
| 입력 | 시뮬레이터가 0.04 s마다 보낸 발 위치·수직 하중 | Chrono 안의 차량 | 시뮬레이터 안 |
| 로봇에 돌아오는 것 | 패인 모양 (한 방향: 흙의 힘은 안 옴) | 패인 모양 | 힘 (양방향: 침하, 마찰, 쟁기질, CoT에 드러남) |
| 패인 모양이 가는 곳 | 물리 지형 -> 맵 서버 (3c부터 Chrono가 직접) | 같음 | 자기 1 cm 침하 지도 + 화면 표시 깊이(`/sim/terrain_overlay`)만 |
| 맵 서버에 있나 | 있음 | 있음 | 없음 (물리 지형, LiDAR, 지도에 안 들어감) |

- Chrono 발자국은 흙 계산과 지형 쓰기가 Chrono 프로세스이고, 발 하중만 시뮬레이터가 보내는 입력이다 (차량 자국은 입력까지 Chrono 쪽).
- MuJoCo 안 무른 흙은 흙이 로봇을 받치는 힘을 물리 스텝마다(1 ~ 2 ms) 돌려줘야 하는 양방향이라 별도 프로세스로 떼어 내기 어렵다 (매 스텝
  주고받으면 느리고 실시간을 지키기 어렵다: 아래 "무른 흙과 로봇 발의 쌍방향 상호작용"에서 A를 고른 이유). Chrono 발자국은 한 방향이라
  0.04 s 교환으로 된다. 무른 흙 자체는 "무른 흙과 로봇 발 (`foot_soil`)" 절.
- 그래서 무른 흙 발자국은 맵 서버 밖이다. 다른 PC의 렌더러나 다른 물리 프로세스는 지도로 볼 수 없고 (Godot에 보이는 것은 시뮬레이터
  화면 스트림이라서), 지도에서 나누려면 흙 모듈이 자기 층으로 써야 한다 (설계 5단계의 `soil_holes` 층).

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
- **로봇 몸 물리 (MuJoCo)**: 로봇도 차량처럼 다물체 동역학으로 계산한다 (Menagerie `go2.xml`).

  | 항목 | 내용 |
  |---|---|
  | 몸 구성 | 몸통 + 다리 4개의 hip·thigh·calf = 강체 13개, 질량 합 15.2 kg |
  | 관절 | 몸통 자유관절(6) + 회전관절 12개 = 18 자유도. 12개 모두 각도 한계 |
  | 운동방정식 | 질량행렬, 코리올리·원심력, 중력 (다리를 휘두르면 몸통이 반작용을 받음) |
  | 관절 손실 | 감쇠 2.0, 회전자 관성(armature) 0.01, 마찰 손실 0.2 |
  | 발 접촉 | 미끄럼·비틀림·구름 마찰(condim 6), 마찰계수 0.8, 타원 마찰 원뿔, 약간 무른 접촉(solref/solimp) |
  | 모터 | 토크 구동. 2 ms 물리 스텝마다 PD 토크를 계산해 ±23.7/45.4 Nm로 자름. 배터리 고장은 토크 상한 축소(`torque_scale`) |
  | 적분 | 2 ms, 오일러, Newton 접촉 풀이 |

  없는 것: 모터 전기 동특성(전류, 역기전력에 따른 토크-속도 곡선), 기어 백래시, 링크 탄성, 발 고무의 실제 변형(무른 접촉으로 근사).
- **흙과의 연결: 차량은 힘으로, 로봇은 모양으로만.** 차량 바퀴는 Chrono 안에서 SCM 흙과 매 스텝 힘을 주고받아 가라앉고 저항을 받는다.
  로봇은 MuJoCo의 단단한 높이지도 위를 걷고, 흙과는 아래처럼 한쪽 방향에 가깝게 연결된다.

  | 방향 | 반영 | 내용 |
  |---|---|---|
  | 로봇 발 -> 흙 | 반영 | 발 하중이 SCM으로 가서 발자국(침하)이 생김 |
  | 흙 -> 로봇 발 (힘) | 없음 | 밟는 순간 빠짐, 무르게 받쳐 줌, 흙 종류에 따른 저항이 없음 |
  | 흙 -> 로봇 발 (모양) | 늦게 반영 | 바뀐 높이가 5 Hz로 높이지도에 들어감 (발 근처는 발이 떠날 때까지 보류). 바퀴 자국이나 앞선 발자국을 나중에 밟을 때 느낌 |

  - 의미 있는 시험: 패인 길, 바퀴 자국 같은 기하 변화 위의 보행.
  - 나타나지 않는 것: 무른 흙에서의 미끄러짐, 빠짐, 에너지 소모(CoT) 증가. 위 `footprints` 스윕에서 흙 강도와 상관없이 전진 거리가 같은 이유.
  - 선택 기능으로 넣었다: 아래 "무른 흙과 로봇 발 (`foot_soil`)". 켜면 흙의 힘(침하, 마찰, 에너지)이 로봇 발에 돌아온다.
- **새 시나리오에 HMMWV 넣기**: 코드 변경 없이 시나리오에 `chrono:` 절만 추가한다. 조건(시작 시 검사): SCM 해상도 = 지형 해상도,
  SCM 중심이 지형 격자점, SCM 영역의 기본 지형이 평평, HMMWV 경로가 지형 범위 안.

## 무른 흙과 로봇 발 (선택 기능 `foot_soil`, 2026-10-11)

흙의 힘이 로봇 발에 돌아오게 하는 선택 기능이다. 시나리오에 `foot_soil` 절이 없으면 이전과 같은 단단한 접촉이다(결과 비트 동일).
흙 값과 식은 Chrono SCM(HMMWV 흙)과 같은 이름·같은 식을 쓴다.

**땅이 파이는 세 가지 경로** (헷갈리기 쉬워서 정리):

| 땅이 파이는 원인 | 계산하는 곳 | 어떤 시나리오 |
|---|---|---|
| `rut` 명령, 시나리오 `add_patch` (홈 패치) | 지형 지도 서비스(`sim/terrain_service.py`)가 정해진 모양(폭, 깊이)을 지형에 그림. 물리 계산이 아니라 지형 편집. Chrono 없음 | 모든 시나리오 (예: `dis_console.yaml`에서 콘솔 `rut`) |
| HMMWV 바퀴 자국 | Chrono SCM이 차 무게와 흙 값으로 계산해 지형에 넣음 | `chrono.vehicle`이 있는 시나리오 (`dis_chrono`, `vehicle_crossing`) |
| 로봇 발자국 | Chrono SCM 대리 구(`chrono.robot_feet`) 또는 무른 흙 모듈(`foot_soil`) (아래 표) | `footprints`, `dis_footprints` / `foot_soil`을 켠 시나리오 |

**로봇 발자국의 두 방식**:

| | Chrono 발자국 (`chrono.robot_feet`) | 무른 흙 (`foot_soil`) |
|---|---|---|
| 만든 때 | 2026-10-06 | 2026-10-11 ~ |
| 흙 계산 | Chrono SCM (별도 프로세스). MuJoCo가 발 위치·하중을 보내면 대리 구가 흙을 누름 | 이 프로젝트의 흙 모듈 (`sim/foot_soil.py`, C++ 코어 `FootSoil`). MuJoCo와 같은 프로세스에서 물리 스텝마다 계산. MuJoCo 내장 기능이 아니다 (발과 받침판의 접촉·마찰만 MuJoCo가 푼다) |
| 흙 식 | SCM | SCM과 같은 Bekker 식을 직접 적분 (SCM 기준값과 2.5 mm 안) |
| 흙 -> 로봇 발 | 모양만 늦게 (파인 지형이 MuJoCo 지형에 5 Hz로). 발은 빠지지 않음 | 힘으로: 발이 빠지고, 흙별 마찰, 쟁기질 저항 |
| 구멍이 남는 곳 | MuJoCo 지형(heightfield) 자체 -> LiDAR도 봄 | 흙 모듈의 구멍 지도 -> 물리에 쓰고 화면에는 화면 전용 토픽으로만 (LiDAR는 안 봄) |
| 무게 | 무거움 (2 cm 격자, 1 ms 스텝. HMMWV와 함께면 이 PC에서 실시간의 0.06배라 `dis_chrono`와 `dis_footprints`를 나눔) | 가벼움 (실시간 계산 시간이 오히려 줄었음) |
| 시나리오 | `footprints.yaml`, `dis_footprints.yaml` | `--set foot_soil.preset=...`를 붙인 아무 시나리오, `soil_strip.yaml` |

둘은 같은 흙을 두 번 계산하게 되어 같이 켤 수 없다. HMMWV 바퀴 자국은 Chrono로, 로봇 발이 빠지는 것은 `foot_soil`로 하는 조합은 된다
(예: `python -m sim.rt_link scenarios/dis_chrono.yaml --dis-port 3000 --dis-wait --brain shm --rviz --set foot_soil.preset=firm`).

**무엇을 라이브러리가 하고 무엇을 직접 만들었나** (`foot_soil`): MuJoCo는 단단한 물체끼리의 접촉·마찰을 푸는 엔진이라 흙이 눌려 파이고
모양이 남는 변형 지면은 계산하지 않는다. Chrono SCM은 흙을 아래로 쏜 광선으로만 봐서 흙 속을 옆·위로 지나가는 저항이 없다.
그래서 흙 쪽은 직접 만들고, 접촉·마찰만 MuJoCo에 맡겼다.

| 하는 일 | 누가 |
|---|---|
| 발과 받침판 사이 접촉력, 마찰 (안정적으로 풀어야 하는 부분) | MuJoCo |
| 받침판을 얼마나 내릴지 (발이 누르는 힘 vs 흙 지지력. Bekker 식 적분, 소성 침하) | 직접 코딩한 흙 모듈 |
| 발을 든 자리에 구멍 남기기, 다시 디디면 구멍 바닥부터 받치기 | 흙 모듈 (구멍 지도) |
| 흙 속을 지나가는 발의 저항 (쟁기질, 수평·위) | 흙 모듈 (MuJoCo 외력 칸에 넣음) |
| 흙별 마찰, 지역별 흙, 실행 중 흙 바꾸기 | 흙 모듈 (받침판 마찰계수를 바꿈) |
| 화면에 구멍 보이기 | 흙 모듈이 화면 전용 토픽으로 보냄 -> RViz·MuJoCo 화면이 그림 (MuJoCo 지형 자체는 파이지 않아 LiDAR는 안 봄) |

흙 식의 모양과 흙 값 이름은 SCM을 따랐고, 맞는지는 Chrono SCM으로 잰 기준값으로 확인했다 (2.5 mm 안).
처음에는 흙 힘을 발에 직접 거는 방식으로 만들었는데 2 ms 스텝에서 불안정해(단단한 흙에서도 걷지 못함), 어려운 접촉·마찰은 MuJoCo에
맡기고 흙 모듈은 받침판 높이만 조절하는 지금 방식으로 바꿨다 (Chrono 발 대리 구와 같은 생각).

쟁기질 (수평·위, 직접 코딩):

| | 내용 |
|---|---|
| 언제 | 발이 흙 표면(구멍 포함) 아래로 잠긴 채 움직일 때 |
| 수평 | 잠긴 부분의 정면 면적(원의 활꼴) × 그 깊이의 Bekker 압력 × `plow_ratio`, 수평 속도의 반대 방향 |
| 위쪽 | 흙에 덮인 윗면을 위에서 본 면적 × 같은 압력 × `plow_ratio`, 올라가는 속도의 반대 방향. 자기 구멍 안에서 곧게 빼면 0 |
| 아래쪽 | 쟁기질이 아니라 받침판(MuJoCo 접촉)이 맡음 |
| 안정화 | 한 스텝에 그 방향 발 끝 속도를 0으로 만드는 힘(유효 질량 × 속도 / dt)을 넘지 않게 자름. 유효 질량은 MuJoCo 자코비안(`mj_jac`)과 질량행렬 풀이(`mj_solveM`)로 계산 |
| 거는 방법 | 계산한 힘을 MuJoCo 외력 칸(`qfrc_applied`)에 넣어 다음 스텝에 반영 |

바탕은 입상 매질 다리 운동의 저항력 이론(RFT)에서 "깊이에 따라 커지고 방향에 따라 다른 저항"이라는 꼴만 빌려 왔다. 크기를 정하는
`plow_ratio`(기본 1)는 실측이나 다른 기준으로 맞추지 않았으므로 무른 흙 결과는 비율 0.25~1 범위로 보고한다 (아래).
MuJoCo는 여기서도 계산 도구(자코비안, 질량행렬, 외력 칸)만 제공한다.

```bash
python -m sim.runner scenarios/flat_trot.yaml --set foot_soil.preset=firm          # hard / firm / soft / mud (DIS 흙 프리셋과 같음)
python -m sim.runner scenarios/rough_rut.yaml --set foot_soil.preset=soft --rviz
python -m sim.sweep scenarios/flat_trot.yaml --param foot_soil.soil.bekker_kphi --values 2e7 5e6 2e6 1e6 --set foot_soil.preset=firm
python conformance/test_foot_soil.py                                                # 시험 (약 20초)
~/miniconda3/envs/chrono/bin/python scripts/scm_foot_static.py                     # SCM 기준값 다시 만들기 (약 1분)
```
```yaml
foot_soil:
  preset: firm            # 프리셋 (dis_console/protocol.py SOIL_PRESETS)
  soil: {bekker_kphi: 2.0e6}   # 프리셋 값 덮어쓰기 (bekker_kc, bekker_n, cohesion, friction_deg, janosi_shear, elastic_k, damping_r)
  plow_ratio: 1.0         # 쟁기질 저항 비율 (0이면 끔)
```

**프리셋이 로봇에게 어느 정도인가** (DIS 흙 프리셋과 같은 값. 원래 HMMWV 바퀴 자국 깊이로 정한 것이라, 작은 로봇 발에는 이름보다 무르다)

| 프리셋 | kphi, 마찰각 | HMMWV 자국 | 발 정지 침하 (37 N / 75 N) | 걸을 때 디딤 평균 | 대략 |
|---|---|---|---|---|---|
| `hard` | 2e7, 30° | 1.8 cm | 0.7 / 1.0 cm | 1.0 cm | 다져진 흙길 |
| `firm` | 5e6, 30° | 5.5 cm | 1.5 / 2.2 cm | 1.8 cm | 보통 흙 (로봇 시험의 "무른 흙"은 여기서 시작) |
| `soft` | 1e6, 30° | 15 cm | 4.1 / 7.2 cm | 6.0 cm | 갓 쌓인 눈, 아주 무른 모래 |
| `mud` | 5e5, 20° | 더 깊음 | 7.1 / 12.9 cm | 10.3 cm | 젖은 진흙 (미끄러움) |

- 정지 침하는 지지력 식(SCM과 같은 식)으로 발 하중 37 N(네 발로 섬)과 75 N(트롯 디딤, 두 발)에서 계산, 걸을 때는 flat_trot 시드 5개 평균.
- `soft`에서 흙을 보지 못하는 트롯은 발이 고르지 않게(2~16 cm) 빠지며 몸이 틀어지고 거의 멈춘다. 몸통은 약 10 cm 가라앉는다
  (0.265 -> 0.16~0.20 m). 중간 정도의 무른 흙을 보려면 `--set foot_soil.soil.bekker_kphi=3e6`처럼 값을 직접 준다.
- `flat_trot`은 8초에 왼쪽으로 도는 명령이 있어 단단한 지형에서도 8초 뒤에는 돈다 (흙 때문이 아니다).

**작동 방식** (`sim/foot_soil.py`): 발마다 보이지 않는 "흙 받침판"(mocap 상자)을 두고, 발은 단단한 지형 대신 이 판에 닿는다
(발·정강이와 지형의 충돌은 끈다. 다른 충돌은 그대로). 발과 판의 접촉·마찰은 MuJoCo가 풀고, 판의 높이를 흙 식이 정한다.
Chrono의 발 대리 구와 같은 생각이다.

| 효과 | 식 |
|---|---|
| 침하 | 깊이 z까지 들어간 발 구의 지지력 F_b(z) = 구 아래 각 점의 Bekker 압력 (kc/b + kphi)·s^n을 접촉 원 위로 적분 (SCM과 같은 식). 발이 누르는 힘(MuJoCo 접촉력)이 지금 깊이의 지지력보다 크면 판이 내려간다 (소성, 되돌아오지 않음). 속도는 감쇠 (F - F_b)/(damping_r·면적) |
| 구멍 | 발을 든 자리에 구 모양 구멍이 남고(1 cm 칸 소성 지도), 다시 디디면 구멍 바닥부터 받친다 |
| 마찰 | 판 마찰계수 = tanφ (흙 내부 마찰각): 30° 흙 0.58, 진흙 20° 0.36 (단단한 지형 0.8) |
| 에너지 | 발이 가라앉는 동안의 일은 흙에 흡수된다 -> CoT |
| 쟁기질 | 흙 표면(구멍 포함) 아래로 잠긴 발이 옆이나 위로 움직이면 흙을 밀어낸다. 수평: 잠긴 정면 활꼴 면적, 위: 흙에 덮인 윗면(적도 위 잠긴 띠)을 위에서 본 면적. 각각 그 방향 속도의 반대로, 크기 = 면적 × 그 깊이의 Bekker 압력 × `plow_ratio`(기본 1). 깊이에 따라 커진다 (입상 매질 저항력 이론 RFT와 같은 꼴). 걸을 때처럼 발이 대각선으로 빠져나오면 구멍 앞벽을 밀고(수평) 그 위 흙을 들어 올린다(위). 자기 구멍 안에서 곧게 위로 빼면 위에 흙이 없어 0. 아래로 가는 성분은 받침판이 맡는다. 명시적 힘이라 한 스텝에 그 방향 발 끝 속도를 0으로 만드는 힘(유효 질량 × 속도 / dt)을 넘지 않게 자른다 (튕김 없이 안정) |
| 발 힘 센서, 접촉 | 판과의 접촉력 (로봇 상태 메시지의 foot_force, 기록의 contact) |
| MOP | `foot_sink_mean_m`, `foot_sink_max_m` (디딤 침하), `foot_slip_mean_m` (디딘 자리에서 미끄러진 거리), `plow_work_j` (쟁기질로 흙에 준 일), `plow_work_up_j` (그중 발을 빼며 위로 든 일) |

**지역마다 다른 흙** (`foot_soil.regions`, 2026-10-17): 직사각형 지역마다 흙을 다르게 둔다 (예: 2 m 앞부터 진흙).

```bash
python -m sim.runner scenarios/soil_strip.yaml --rviz        # 보통 흙 위를 걷다 x 2.0~3.5 m 진흙 띠를 만남
python -m sim.rt_link scenarios/soil_strip.yaml --rviz       # 같은 시나리오, C++ 코어 (실시간)
```
```yaml
foot_soil:
  preset: firm                       # 기본 흙 (지역 밖)
  regions:                           # [x0, y0, x1, y1] (m). 겹치면 목록에서 나중 지역이 이긴다. 경계 포함
    - {name: mud_strip, region: [2.0, -10, 3.5, 10], preset: mud}
    - {region: [5, -1, 6, 1], soil: {bekker_kphi: 2.0e6}, plow_ratio: 0.5}   # 준 값만 기본 흙을 덮어쓴다
```
- 디딤의 흙은 디딘 순간 발 위치로 정하고 디디는 동안 그대로다. 쟁기질은 발이 지금 지나가는 곳의 흙을 쓴다.
  받침판 마찰계수는 그 흙의 tanφ로 바꾼다 (진흙 0.36, 다른 흙 0.58). 구멍 지도는 흙과 상관없이 하나다 (구멍은 모양).
- 흙은 기본 포함 4종류, 지역은 8개까지 (C++ 코어 규약). 같은 흙 값의 지역은 하나의 흙으로 합친다.
- MOP `foot_sink_by_soil`: 흙별 디딤 수와 평균 침하.
- `soil_strip` 결과 (16초, 시드 0): 보통 흙 디딤 181번 평균 1.9 cm, 진흙 띠 127번 10.2 cm. 진흙 띠에 들어가 멈춘다 (전진 2.60 m, CoT 7.5).
  `plow_ratio` 0.25면 3.26 m (띠 끝 3.5 m 앞)까지 가고 CoT 6.4. 흙을 보지 못하는 트롯이 진흙 띠를 건너지 못하는 시험이 된다.
- C++ 코어도 같은 규칙이고, 진흙 띠 시나리오도 lockstep에서 `sim.runner`와 기록·MOP(흙별 침하 포함)가 비트 단위로 같다
  (`test_foot_soil_matches_runner`). 공유 메모리 규약 12판 (흙별 지지력 표, 지역 목록, 흙별 디딤 통계).
- **화면 표시** (화면 전용, 물리 영향 없음): RViz는 지형 메쉬를 흙 지역마다 그 흙 색으로 칠하고(무를수록·미끄러울수록 짙은 갈색,
  진흙 띠는 기본 지형색보다 짙다) 지역 이름을 띄운다. MuJoCo 화면(`--mjviz`)은 지역 테두리를 그린다 (로봇 근처 ±3 m).
  두 화면의 정보판에 몸통 아래 흙 이름이 나온다 (`soil:mud_strip`). 지역은 매니페스트 `soil_regions`, 몸통 아래 흙은 상태 `soil`로 보낸다.
- **일반 터미널에서 확인**:
  ```bash
  python -m sim.runner scenarios/soil_strip.yaml --rviz      # RViz: 진흙 띠가 짙은 갈색, 이름 "mud_strip"
  python -m sim.runner scenarios/soil_strip.yaml --mjviz     # MuJoCo 화면: 진흙 띠 테두리 선 (로봇 근처 ±3 m)
  python -m sim.rt_link scenarios/soil_strip.yaml --rviz     # C++ 코어 실시간에서도 같은 표시
  python -m sim.runner scenarios/soil_strip.yaml --rviz --set 'foot_soil.regions=[{name: soft_strip, region: [2, -10, 3, 10], preset: soft}]'
  ```
  볼 것: x 2.0~3.5 m 띠가 기본 흙색보다 짙고 위에 이름이 뜬다. 정보판이 `soil:firm`에서 띠에 들어서면 `soil:mud_strip`으로 바뀐다
  (MuJoCo 화면은 왼쪽 위 글 `soil mud_strip`). 띠에서 발자국이 깊어지고 몸통이 내려가며 거의 멈춘다. 마지막 명령은 띠 이름이 `soft_strip`,
  색은 진흙보다 조금 옅다 (색은 흙이 무를수록·미끄러울수록 짙다). 목록 값은 작은따옴표로 감싼다.
  C++ 코어와 Python이 같은지, 시험:
  ```bash
  python -m sim.runner scenarios/soil_strip.yaml                 # 둘의 MOP(forward_x_m 약 2.6, foot_sink_by_soil firm 약 181번,
  python -m sim.rt_link scenarios/soil_strip.yaml --lockstep     #   mud_strip 약 127번)가 똑같아야 한다
  python conformance/test_foot_soil.py                           # 9개 (지역 규칙, 진흙 띠)
  python conformance/test_render_stream.py                       # 7개 (흙 지역 화면 표시)
  python conformance/test_rt_core.py                             # 13개, 약 3분 (진흙 띠 비트 동일, shm 두뇌 경쟁 조건 수정)
  ```
  공유 메모리 규약이 12판이라 예전에 띄운 `sim.rt_link`는 끄고 돌린다. `test_rt_core`의 shm 두뇌 시험은 다른 터미널에서 실시간 코어를
  돌리는 중에도 통과해야 한다 (아래 "lockstep shm 두뇌가 부하에서 가끔 한 스텝 어긋나던 것").

**화면 발자국** (`--rviz`, `--mjviz`): 발이 판 구멍(남은 구멍 + 지금 디딘 발)을 화면 전용 토픽 `/sim/terrain_overlay`로 보내고
렌더러가 `지형 - 구멍 깊이`로 그린다 (약 10 Hz). 물리 지형, 센서(LiDAR)가 받는 지형 패치, 지형 인지 보행은 그대로다 (시험으로 확인).
지형 격자(예: 5 cm)가 발(지름 4.4 cm)보다 거칠어 구멍은 격자 한 칸 둘레로 넓혀 그린다 (모양은 대략적, 깊이는 맞음).
예전에는 무른 흙에서 정강이가 평평한 땅을 뚫고 들어간 것처럼 보이던 것을 고쳤다.

**1단계 기준: Chrono SCM 정적 침하** (`scripts/scm_foot_static.py`, 발 대리 구 반지름 2.2 cm를 일정 하중으로 누름, 2초).
SCM 값은 격자에 민감하다: 2 cm 격자는 단단한 흙 30 N에서 0.62 cm, 1 cm 격자 0.33 cm. 0.5 cm와 0.25 cm 격자는 2 % 안이라 0.25 cm 값을 기준으로 삼았다
(지금 `footprints.yaml`의 2 cm 격자 발자국은 단단한 흙에서 실제보다 깊게 나온다).

| kphi | 30 N | 50 N | 80 N | 120 N |
|---|---|---|---|---|
| 2e7 (hard) | 0.40 / 0.65 | 0.62 / 0.84 | 0.89 / 1.06 | 1.15 / 1.32 |
| 1e7 | 0.71 / 0.92 | 1.03 / 1.20 | 1.36 / 1.54 | 1.75 / 1.94 |
| 5e6 (firm) | 1.15 / 1.32 | 1.56 / 1.75 | 2.12 / 2.32 | 2.84 / 3.03 |
| 1e6 (soft) | 3.36 / 3.55 | 5.04 / 5.22 | 7.45 / 7.61 | 10.55 / 10.67 |

(cm, SCM / 이 모델의 식). 같은 식을 적분하는데 이 모델이 0.1~0.25 cm 더 깊다 (가장 큰 차이 단단한 흙 30 N에서 2.5 mm).
Chrono 충돌 여유 두께로 맞춰 보았지만 설명되지 않아 보정 계수는 넣지 않았다.

**3단계 확인**
- 서 있는 로봇: 소성 침하가 일어서는 동안 받은 최대 하중에서의 SCM 침하와 맞는다
  (단단한 흙 0.9~1.1 cm vs SCM 0.7~0.9, 보통 1.9~2.2 vs 1.7~2.1, 무른 흙 5.1~5.8 vs 5.1~5.9).
- flat_trot 12초, 시드 0~4 평균 (쟁기질 수평·위 포함):

| 흙 | 넘어짐 | 전진 (m) | CoT | 디딤 침하 평균 / 최대 (cm) | 미끄럼 (cm) | 쟁기질 일 (그중 위) (J) |
|---|---|---|---|---|---|---|
| 단단한 지형 (옵션 없음) | 0/5 | 3.92 | 2.42 | - | - | - |
| hard (kphi 2e7) | 0/5 | 4.10 | 2.53 | 1.0 / 1.6 | 0.9 | 51 (0) |
| firm (5e6) | 0/5 | 4.32 | 2.96 | 1.8 / 4.5 | 1.0 | 110 (18) |
| kphi 2e6 | 0/5 | 0.66 | 14.48 | 3.4 / 7.5 | 1.2 | 316 (79) |
| soft (1e6) | 0/5 | 0.90 | 17.68 | 5.9 / 16.0 | 1.6 | 342 (68) |
| mud (5e5, 마찰 20°) | 0/5 | 0.75 | 20.58 | 10.3 / 17.5 | 1.8 | 319 (61) |

- 흙이 무를수록 침하와 CoT가 커진다. 흙을 보지 못하는 이 트롯은 kphi 5e6(보통 흙)까지는 걷고, 2e6 이하에서는 발이 3 cm 넘게 빠지며
  거의 나아가지 못한다 (넘어지지는 않음). 예전에는 흙 강도와 상관없이 같았다.
  단단한·보통 흙의 전진은 단단한 지형보다 조금 길다 (4.1~4.3 vs 3.9 m, 걸음 모양이 조금 달라진 결과로 보이며 원인은 따로 보지 않았다).
- 단단한 흙은 발이 반지름(2.2 cm)보다 얕게 빠져 발 위에 덮인 흙이 없으므로 위쪽 저항이 0이다.
- **쟁기질이 왜 필요했나**: 흙이 무르면 몸이 가라앉아(kphi 1e6에서 몸통 0.266 -> 0.210 m) 공중의 발이 흙 표면 아래를 지나간다
  (공중 시간 중 5e7: 5 %, 5e6: 20 %, 1e6: 86 %). 쟁기질이 없으면 발이 흙을 그냥 지나가 무른 흙에서 오히려 더 멀리 갔다
  (soft 5.30 m, CoT 3.20). 처음에는 수평 저항만 넣었고(soft 1.35 m, CoT 9.26), 발이 대각선으로 빠져나올 때 위 흙을 들어 올리는
  저항을 더했다.
- **결과를 `plow_ratio` 범위로 보고하기**: 쟁기질 비율은 아직 맞추지 않았으므로, 무른 흙 결과는 범위의 양 끝(0.25와 1)으로 돌려 함께 낸다.
  ```bash
  python -m sim.sweep scenarios/flat_trot.yaml --param foot_soil.plow_ratio --values 0.25 1 --set foot_soil.preset=soft
  # 시드 여러 개 (비율마다 따로)
  python -m sim.sweep scenarios/flat_trot.yaml --param seed --values 0 1 2 3 4 --set foot_soil.preset=soft --set foot_soil.plow_ratio=0.25
  python -m sim.sweep scenarios/flat_trot.yaml --param seed --values 0 1 2 3 4 --set foot_soil.preset=soft --set foot_soil.plow_ratio=1
  ```
  첫 명령의 결과 예 (시나리오 기본 시드 0 한 번. 2026-10-11 일반 터미널에서 잰 뒤 C++ 코어와 맞춘 계산 순서로 다시 잼):

  | plow_ratio | 전진 (m) | CoT | 측방 표류 (m) |
  |---|---|---|---|
  | 0.25 | 3.84 | 4.73 | 0.10 |
  | 1 | 0.36 | 26.90 | 0.45 |

  같은 soft 흙에서 비율만 바꿔도 "거의 정상으로 걷는다"(단단한 지형 3.92 m의 98 %)와 "거의 멈춘다"로 갈린다. 두 값 모두에서 성립하는 것은
  CoT가 단단한 지형(2.42)보다 높다는 것과 디딤 침하(평균 4~6 cm)뿐이다. 1에서 측방 표류가 큰 것은 거의 멈춘 채 몸이 틀어졌기 때문이다.
  sweep은 시드 하나만 돌므로 아래 표(시드 5개 평균)와 값이 다르다.
  flat_trot 12초, 시드 5개 평균, `plow_ratio` 0.25 ~ 1 (넘어짐은 모두 0):

  | 흙 | 전진 (m) | CoT | 디딤 침하 평균 (cm) |
  |---|---|---|---|
  | hard | 4.14 ~ 4.10 | 2.48 ~ 2.53 | 1.0 |
  | firm | 4.37 ~ 4.32 | 2.67 ~ 2.96 | 1.8 |
  | kphi 2e6 | 4.59 ~ 0.66 | 3.63 ~ 14.48 | 2.3 ~ 3.4 |
  | soft | 3.25 ~ 0.90 | 5.61 ~ 17.68 | 4.2 ~ 5.9 |
  | mud | 1.85 ~ 0.75 | 9.28 ~ 20.58 | 7.9 ~ 10.3 |

  단단한·보통 흙은 비율과 거의 상관없다 (발이 얕게 빠져 흙 속을 지나가는 일이 적다). kphi 2e6보다 무른 흙에서는 "걸을 수 있다 ~ 거의 멈춘다"
  사이로 갈리므로, 범위 양 끝에서 모두 성립하는 결론(예: soft·mud에서 CoT가 단단한 지형의 2배 이상)만 믿는다.
- **`plow_ratio`에 민감하다** (soft, 시드 5개): 0 -> 5.30 m (CoT 3.20), 0.25 -> 3.25 m (5.61), 0.5 -> 1.02 m (14.55), 1 -> 0.90 m (17.68),
  2 -> 0.86 m (17.77). 0.5 이상에서는 거의 멈춰 비슷하고, 그 아래에서 크게 달라진다. 무른 흙 결과의 절댓값은 이 값에 달려 있으므로,
  맞추기 전에는 흙끼리의 비교(순서, 경향)로 쓴다.

**시험** (`python conformance/test_foot_soil.py`, 약 25초. 2026-10-11 일반 터미널에서 모두 통과, 제 실행과 숫자까지 같음 = 결정적.
C++ 코어 지원 뒤 거부 시험은 빼고 `test_rt_core.py`의 `test_foot_soil_matches_runner`로 옮김)

| 시험 | 확인하는 것 | 그날 값 |
|---|---|---|
| `test_bearing_matches_scm_reference` | 지지력 식이 Chrono SCM 기준과 맞음 | 16개 조건에서 최대 차이 2.5 mm |
| `test_off_keeps_model` | `foot_soil`이 없으면 모델 그대로 | 받침판 없음, 발 충돌 설정 그대로 |
| `test_standing_sinkage_matches_scm` | 서 있는 로봇의 소성 침하 = 일어설 때 받은 최대 하중에서의 SCM 침하 | 예: soft 5.8 cm vs SCM 5.9 cm, firm 2.2 vs 2.05 |
| `test_holes_remain` | 발을 든 자리에 구멍이 남음 | 가장 깊은 구멍 ≈ 가장 깊은 디딤 |
| `test_walks_on_hard_and_firm_and_softer_sinks_more` | hard·firm은 걷고, 무를수록 더 빠짐 | 10초 전진: 단단한 지형 3.48, hard 3.54, firm 3.58, kphi 2e6 0.56 m |
| `test_plowing_slows_soft_soil` | 쟁기질이 무른 흙에서 전진을 줄이고, 단단한 흙에서는 위쪽 저항이 거의 0 | soft 8초 3.50 -> 0.50 m (흙에 준 일 200 J, 그중 위로 40 J), hard 위로 0.0 J |
| `test_footprints_shown_on_screen_only` | 구멍은 화면 토픽에만, 물리·센서 지형은 그대로 | 격자 점 45개, 가장 깊은 곳 9.8 cm |

시험은 8~10초만 돌아서 전진 거리가 위 표(12초)와 다르다.

**C++ 실시간 코어** (4단계, 2026-10-11): `sim.rt_link`도 `foot_soil`을 받는다. 구조는 그대로이고, 물리 스텝마다의 흙 계산
(받침판 높이, 소성 침하, 구멍 지도, 쟁기질)만 코어(`rt/rt_core.cpp` `FootSoil`) 안에서 한다.

```bash
python -m sim.rt_link scenarios/flat_trot.yaml --set foot_soil.preset=firm --rviz                  # 실시간 (C++ 코어)
python -m sim.rt_link scenarios/flat_trot.yaml --set foot_soil.preset=soft --lockstep               # lockstep (sim.runner와 비트 동일)
```

| | `sim.runner` (Python) | `sim.rt_link` (C++ 코어) |
|---|---|---|
| 받침판·소성 침하·구멍 지도·쟁기질 | `sim/foot_soil.py` `FootSoil.update` | `rt/rt_core.cpp` `FootSoil::update` (같은 알고리즘, 같은 연산 순서) |
| 지지력 표 (깊이 -> 힘, 면적) | Python이 계산 | Python이 계산해 시작 때 넘김 (공유 메모리 `cfg.soil`) |
| 흙 표면 | MuJoCo 지형 heightfield (float32) | 같은 heightfield (지형 공유 메모리로 받은 값) |
| 구멍 지도 | 사전 (1 cm 칸) | 시작 때 잡은 고정 크기 해시 표 (26만 칸, 루프 안 할당 없음. 차면 경고) |
| MOP, 화면 발자국 | 같은 프로세스 | 코어가 참값 기록(디딤 상태, 소성 침하, 디딤 통계, 쟁기질 일)과 구멍 링에 남기고 관리 프로세스가 받음 |

- **비트 동일 확인** (`test_rt_core.py` `test_foot_soil_matches_runner`): 지형 패치 이벤트가 있는 rough_rut(firm, inproc·shm·ros 세 두뇌)와
  지형 창 + 실행 중 자국 추가(soft)가 기록 전체와 흙 MOP까지 `sim.runner`와 같다. 처음 실행부터 같았다.
- 같게 하려고 Python 쪽 계산을 C와 같은 순서로 바꿨다: 흙 표면을 Terrain Map Service 원천(float64) 대신 MuJoCo heightfield(float32)에서
  읽고, 보간을 직접 쓴 이분 탐색으로, 행렬·벡터 곱을 순서를 정한 반복문으로, hypot을 sqrt로 (파이썬 math.hypot은 C와 마지막 자리가 다를 수 있다).
  결과가 조금 바뀌었다 (예: soft 시드 0 전진 0.368 -> 0.362 m). 위 표는 바꾼 뒤 다시 잰 값이다.
- **실시간 계산 시간** (flat_trot, shm 두뇌, 이 PC 기본 설정): 단단한 지형 평균 7.1 ms(최대 15.1) -> firm 3.1 ms(4.7), soft 3.6 ms(7.0).
  오히려 줄었다: 발·정강이가 지형 heightfield와 충돌 계산을 하지 않고 작은 상자 받침판과만 닿기 때문으로 보인다. 주기 초과 0, 늦은 명령 0.
  실시간 결과는 lockstep과 같았다 (firm 4.374 m, soft 0.362 m).
- 공유 메모리 규약 11판 (흙 설정, 참값의 흙 상태, 구멍 링). 지역마다 다른 흙을 넣으며 12판.

**한계와 다음 단계**
- 쟁기질 비율(`plow_ratio`)은 실측으로 맞추지 않았다 (위). 밀린 흙이 쌓이는 것, 진흙에서 발을 뽑을 때의 흡착(빨아들임)은 없다 (위로 빼는 저항은 덮인 흙을 들어 올리는 몫만).
- **쟁기질 비율을 맞추는 방법** (2026-10-11 검토, 진행하지 않음). Chrono SCM은 흙을 수직으로만 보는 모델이라(아래로 쏜 광선으로 침하 계산)
  흙 속을 옆으로 지나가는 저항의 기준이 될 수 없다.

  | 방법 | 내용 | 이 PC에서 | 정확도 |
  |---|---|---|---|
  | 문헌 범위 | 입상 매질 저항력 이론(RFT, Li·Zhang·Goldman 2013 등)에서 같은 깊이의 수평/수직 저항 비 | 바로 가능 | 범위 수준. 마른 모래류에서 수평이 수직보다 작은 쪽으로 기억하지만 값은 논문 확인 필요 |
  | 입자 시뮬레이션 (DEM) | Chrono CPU 입자 모델로 작은 모래 상자(입자 수만 개)를 만들고, 발 크기 구를 ① 아래로 눌러 Bekker 값(kphi, n)을 맞추고 ② 같은 깊이에서 옆으로 끌어 수평 저항을 잼 -> 같은 매질 안에서 비율 | 가능하지만 무거움 (한 번, 1~2시간 예상) | 마른 입상 흙에 한해 자기 일관적. 점착력 있는 진흙은 대표하지 못함 |
  | 실측 | 모래 상자에서 발 크기 구를 일정 깊이로 끌며 힘 측정, 또는 실제 Go2를 알려진 흙에서 걸려 침하·속도·전력으로 맞춤 | 장비 필요 | 가장 정확 |

  맞추기 전에는 무른 흙 결과를 흙끼리의 비교(순서, 경향)로 쓰고, 필요하면 `plow_ratio` 범위의 양 끝으로 돌려 범위로 보고한다.
- 탄성 되튐(elastic_k), Janosi 전단 변위에 따른 미끄럼(마찰은 쿨롱), 점착력, 경사·홈 벽의 옆 반력이 없다. 지역은 직사각형만 된다.
- 발자국은 화면에만 보인다 (아래 "화면 발자국"). LiDAR와 물리 지형에는 없다. Chrono 발자국(`chrono.robot_feet`)과 함께 쓸 수 없다.
- C++ 실시간 코어에서도 된다 (아래 "C++ 실시간 코어").
- 처음 구현(발에 흙 반력을 외력으로 직접 걸기)은 버렸다: 미끄럼 거리를 스프링처럼 다뤄 착지 때 앞으로 미끄러진 발이 뒤로 밀 때 힘 없이 돌아갔고,
  마찰을 명시적 힘으로 세게 주면 불안정했다 (단단한 흙에서도 걷지 못함).

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
| `Event_SetSoil` | `preset`(hard/firm/soft/mud) 또는 `soil`(SCM 값 8개), (`t_apply`) | 흙 상태 (Chrono SCM, 이후 생기는 변형에 적용. 무른 흙 `foot_soil`이면 로봇 발의 기본 흙도) |
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
(재개는 제어권을 가진 콘솔만 할 수 있다).
시험이 끝나면(`end`, 넘어짐, 시나리오 시간 끝, 시뮬레이터 Ctrl+C) 시뮬레이터가 접속한 콘솔에 알리고, 콘솔은 `시험이 끝났다 (이유, 기록 폴더)`를 띄운 뒤
링크 감시를 멈춘다 (전에는 끝난 시뮬레이터를 통신 두절로 표시했다). 이후 명령은 보내지 않는다. 일시정지·종료는 물리에 영향이 없으므로 `scenario_replay.yaml`에서 빠지고, 콘솔로 종료했으면 그 시각이 `duration`이 된다.

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
  로봇과 차량 위치가 비트 단위로 같다 (`test_entity_and_soil_with_chrono`). Chrono 없는 시나리오에서 투입은 거부(`NO_CHRONO`),
  흙은 무른 흙(`foot_soil`)도 없을 때만 거부(`NO_SOIL`).
- **무른 흙 시나리오에서 `soil`** (2026-10-17): 로봇 발의 기본 흙(지역 밖)을 바꾼다. 직접 정한 지역 흙(`regions`)은 그대로이고,
  Chrono가 있으면 Chrono 흙도 함께 바뀐다. 다른 이벤트처럼 적용 스텝부터 (디딤 중인 발도 그 스텝부터 새 흙).
  ```bash
  python -m sim.rt_link scenarios/soil_strip.yaml --dis-port 3000 --dis-wait --brain shm --rviz --set 'events=[]'   # 터미널 1 (sim.runner도 된다)
  python -m dis_console.console --sim 127.0.0.1:3000                                            # 터미널 2: cmd 0.4, soil soft, end @20
  ```
  `soil_strip.yaml`에는 1초에 걷기 시작하는 시나리오 이벤트가 있어 그대로 띄우면 콘솔 명령 없이도 걷는다 (`--dis-wait`는 접속할 때까지
  시작만 미룬다). 콘솔로만 조종하려면 `--set 'events=[]'`로 이벤트를 비운다 (콘솔용 `dis_console.yaml`, `dis_chrono.yaml`은 처음부터 비어 있다).
  C++ 코어에는 관리 프로세스가 새 흙의 지지력 표를 계산해 공유 메모리(`soil_update`, 규약 13판)로 그 스텝에 보낸다. 표 계산을 배열로 바꿔
  9.8 -> 0.6 ms로 줄였다 (처음에는 실시간에서 1스텝 늦게 걸려 재실행과 달랐다: 1.141 vs 1.165 m). 지금은 실시간 콘솔로 바꿔도 늦게 걸린 변경 0,
  재실행과 같다 (3번 확인). lockstep에서는 `sim.runner`와 비트 단위로 같다 (`test_foot_soil_matches_runner`, 3초에 진흙으로).
  흙별 MOP(`foot_sink_by_soil`)의 기본 흙 항목은 바뀌기 전후를 합쳐 세고 이름은 마지막 흙이다 (흙 바꾼 시각은 기록의 이벤트에 남는다).
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

**DIS_test 기록 중계기·재생기 연결** (도구는 `~/DIS_test`에 있고 이 저장소에 복사하지 않았다)

```bash
# 터미널 1: 시뮬레이터 (3000)
python -m sim.runner scenarios/dis_console.yaml --dis-port 3000 --dis-wait --rviz --dis-key ~/.config/physics_ai_test/dis.key
# 터미널 2: 기록 중계기 (콘솔 -> 3001 -> 시뮬레이터 3000). Enter로 링크 차단/복구, Ctrl+C로 저장하고 끝
cd ~/DIS_test && python3 -m siman_r.recorder --listen 3001 --vehicle 127.0.0.1:3000 --out recordings/sim_session
# 터미널 3: 콘솔은 중계기로 보낸다
cd ~/physics_ai_test && python -m dis_console.console --sim 127.0.0.1:3001 --key ~/.config/physics_ai_test/dis.key
# 재생 (출력만 / 수신기로 / 시뮬레이터로)
cd ~/DIS_test
python3 -m siman_r.player recordings/sim_session.jsonl --print --speed 0
python3 -m siman_r.monitor --port 4000 &  python3 -m siman_r.player recordings/sim_session.jsonl --target 127.0.0.1:4000
python3 -m siman_r.player recordings/sim_session.jsonl --target 127.0.0.1:3000 --from console
```
- 중계기는 바이트를 바꾸지 않고 전달하므로 인증 서명이 그대로 통과한다. 중계기 기록은 양 끝 사이(콘솔이 실제로 보낸 것)이고,
  시뮬레이터의 `dis_pdus.*`는 시뮬레이터가 실제로 받은 것이라, 둘을 비교하면 망에서 잃은 패킷을 알 수 있다 (문서 §11.2).
- 확인 (2026-10-09): 콘솔 -> 중계기 -> 시뮬레이터로 명령, 중계기에서 링크 8초 차단 -> 시뮬레이터 5초 뒤 통신 두절(로봇 정지, 제어권 해제)
  -> 복구 후 콘솔이 "제어권이 풀렸다" -> `take` -> 명령 -> `end`. 패킷 125개 기록.
- 재생기로 시뮬레이터에 다시 보낸 결과:

| 대상 | 재생 방법 | 결과 |
|---|---|---|
| 인증 쓰는 시뮬레이터 | 기본 (Exercise ID를 99로 바꿈) | 모두 버림 (MAC 불일치. 인증 없이도 Exercise ID가 달라 버려진다) |
| 인증 쓰는 시뮬레이터 | `--keep-exercise` (바이트 그대로) | 모두 버림 (30초 지난 캡처) |
| 인증 없는 시뮬레이터 | `--keep-exercise` | 명령이 다시 실행된다 (접속, 이동 명령, 종료) |

- 원본 PDU 재생은 벽시계 기준으로 보내므로 원래와 다른 시뮬레이션 시각에 적용된다. 똑같이 재현하려면 `scenario_replay.yaml`을 쓴다
  (적용 시각으로 기록되어 비트 단위로 같다). 원본 재생은 연동 시험(다른 체계가 같은 PDU를 받는지, 부하)용이다.
- 한계: 인증을 써도 새로 띄운 시뮬레이터는 30초 이내의 캡처를 처음 보는 카운터로 받아들인다 (같은 시뮬레이터 안에서는 카운터로 막힌다).
  시뮬레이터 `dis_pdus.jsonl`의 방향은 `console->sim` / `sim->console`이라 재생기의 `--from console`은 되고 `--from vehicle`은 맞지 않는다.

**아직 없는 것** (문서 §10.2의 나머지)
- 임의 위치·여러 대 개체 투입 (지금은 시나리오에 선언한 HMMWV 한 대), 표준 Entity State PDU, 임무 명령 (C-BML Order),
  기상(날씨, 시정 -> 센서 영향), 교전 효과, 장애물·지뢰, 전자전 (GNSS 재밍, 통신 열화. 링크 두절은 고장 주입으로 가능)
- 초기 구성 MSDL (지금은 시나리오 YAML)
- 암호화 (인증만 있고 내용은 보인다), 키 교체·여러 키, 다른 장비 사이 시험
- Live 로봇 권한 분리: 실제 로봇에는 이동 명령이 아니라 임무만 보내야 한다 (§10.1). 지금의 `Event_SetCommand`는 가상 모드 전용이다.

## 강화학습 준비 (GPU 장비에서)

**이 PC에서 MJX 학습 실험** (2026-10-17, 실험적): GTX 1650(4 GB)이 있다. NVIDIA 드라이버는 RT 커널에서 올라가지 않으므로(PREEMPT_RT 미지원,
모듈이 일반 커널용으로만 빌드됨) 학습은 일반 커널(`6.8.0-138-generic`, 부팅 메뉴 Advanced options)로 부팅해서 한다.
conda 환경 `mjx` (Python 3.11, JAX 0.8.2 CUDA 12, MuJoCo 3.5.0 + MJX, Brax 0.14.2. JAX 0.10은 Brax 0.14와 맞지 않음).

```bash
JAX_PLATFORMS=cpu ~/miniconda3/envs/mjx/bin/python -m train.train_mjx --smoke        # RT 커널(GPU 없음): 전체 경로 확인 (약 3분)
~/miniconda3/envs/mjx/bin/python -m train.train_mjx --envs 1024 --steps 20000000     # 일반 커널 + GPU
```
- `train/mjx_env.py`: 평지 걷기. actor = 배치용 관측 47개 + 잡음 + 연결 지연 0~2주기 (JAX 관측 함수는 `obs_reference.npz`와 최대 5e-7),
  critic = 참 몸통 속도·높이·발 접촉 추가 (비대칭). 행동은 카드 규칙(기본 자세 + 0.25 × action), 정책 출력 −1~1 × 2. PD kp 40, kd 1.
- `train/train_mjx.py`: Brax PPO -> ONNX(관측 정규화 포함, 저장 전에 Brax 추론과 비교) -> 정책 카드. 그다음 mj_ros에서
  `make_policy_reference.py`, `test_policy.py`, `sim.runner --policy`.
- CPU 확인 (RT 커널): 환경 16개, 2560스텝 학습 180초, ONNX = Brax 최대 차이 0, 시뮬레이터에 탑재되고 `test_policy` 4개 통과
  (학습량이 아주 적어 걷지는 못함). 학습 안 된 정책이 일어서기 복구를 자주 불러 입출력 기준이 어긋나던 것을 고쳤다
  (`make_policy_reference.py`가 복구 중인 스텝을 건너뜀).
- **GPU 확인** (일반 커널, GTX 1650): JAX가 `CudaDevice`를 잡는다. 환경 수별 처리량 (제어 스텝/초, GPU 메모리 최대):
  128: 2,988 (76 MiB), 512: 3,463 (258 MiB), 1,024: 4,145 (514 MiB), 2,048: 4,353 (857 MiB), 4,096: 4,246 (1,674 MiB).
  1천~2천 개에서 약 4,200으로 포화 (CPU 369의 약 11배). 병목은 메모리가 아니라 연산. 첫 스텝 JIT 컴파일 약 25초.
- **첫 학습** (환경 1,024개, 1천만 스텝, 54분, 학습 중 실효 약 2,800 스텝/초): 전체 경로(학습 -> ONNX(Brax와 차이 0) -> 카드 -> 시뮬레이터
  탑재)가 GPU에서도 된다. 하지만 정책은 넘어짐 기준(0.15 m) 바로 위로 웅크리고 서 있기만 배웠다 (몸통 0.155 m, 전진 0.24 m에서 멈춤).
  평가 보상은 113만 스텝에서 12.7로 가장 높았다가 끝에 0.8로 떨어졌다. 학습 환경(MJX)과 Python 시뮬레이터에서의 동작이 거의 같아
  (0.155 / 0.165 m, 0.24 / 0.31 m) 학습한 것이 배치 쪽으로 그대로 옮겨진다는 확인은 됐다. 정책은 저장소에 넣지 않았다 (`policies/go2_mjx_flat`).
  - 원인: 서 있기만 해도 속도 추종 보상을 꽤 받고(명령 0.4 m/s에서 1스텝 약 0.5), 걷다 넘어지면 벌점, 몸통 높이 벌점 없음 -> 흔한 국소 최적.
  - 또 하나: 제어 노드의 다리 처짐 복구(`control/node.py` SAG_M)가 낮은 게인(kp 40)의 학습 정책을 처짐으로 오인해 600스텝 중 557스텝을
    덮어썼다 (규칙 기반 보행기의 높은 게인에 맞춘 기준). 정책이 제대로 서도 생길 문제라 따로 고쳐야 한다.
- **학습 정책을 시뮬레이터에서 보기** (MJX 정책 = 카드만 바꾸면 두 실행기 모두에 탑재):
  ```bash
  python -m sim.runner scenarios/flat_trot.yaml --policy policies/go2_mjx_flat/card.yaml --rviz                 # Python
  python -m sim.rt_link scenarios/flat_trot.yaml --policy policies/go2_mjx_flat/card.yaml --brain shm --rviz    # C++ 코어 (실시간)
  python -m sim.rt_link scenarios/flat_trot.yaml --policy policies/go2_trot_bc_est/card.yaml --brain shm --rviz # 비교: 모방학습 정책 (걸음)
  ```
  `--mjviz`(MuJoCo 화면), `--lockstep`(sim.runner와 같은 결과), `--brain inproc|ros`, `scenarios/rough_rut.yaml`로 바꿀 수 있다.
  MJX 정책은 낮게 웅크리고(몸통 약 0.16 m) 30 cm쯤 움찔한 뒤 서 있고, 대부분의 시간 동안 일어서기 복구가 정책을 덮어쓴다.
  모방학습 정책은 0.4 m/s로 걷다 8초에 왼쪽으로 돈다 (12초에 약 4 m).

  | 실행 (MJX 정책, flat_trot 12초, 일반 커널) | 넘어짐 | 전진 | 비고 |
  |---|---|---|---|
  | `sim.runner` | 아니요 | 0.279 m | CoT 1.366 |
  | `sim.rt_link --lockstep --brain inproc` / `shm` | 아니요 | 0.279 m | CoT 1.366 (sim.runner와 같음) |
  | `sim.rt_link --brain shm` (실시간) | 아니요 | 0.279 m | 늦은 명령 0, 주기 초과 0, SCHED_FIFO 잡힘 (일반 커널이라 실시간 보장은 없음) |
- **다음 판 제안 (하지 않음)**: 몸통 높이 벌점(목표 0.27 m), 넘어짐 기준 0.20 m, 보행 위상에 맞춘 대각 다리 디딤 보상, 더 날카로운 속도 추종,
  3천만 스텝(이 GPU에서 약 2.5~3시간). 제어 노드는 학습 정책 카드면 처짐 복구 기준을 게인에 맞추거나 카드에서 끌 수 있게.


**정리: MJX 강화학습은 무엇을 했고, 모방학습과 무엇이 다른가** (2026-10-17)

알고리즘: **PPO** (Brax 0.14 `ppo.train`), **비대칭 actor-critic**.

| 항목 | 설정 |
|---|---|
| actor (정책) | 입력 = 배치용 관측 47개(잡음 포함). MLP 47-128-128-128-12, silu. 출력 tanh -> × 2 = 카드 단위 action (ONNX에 같은 곱) |
| critic (가치) | 입력 55개 = 잡음 없는 관측 47 + 참 몸통 속도(몸통 좌표) 3 + 몸통 높이 1 + 발 접촉 4. MLP 256-256-256 |
| PPO | 환경 1,024개, unroll 20, 배치 256 × 미니배치 4, 배치당 갱신 4회, 학습률 3e-4, 엔트로피 1e-2, 할인 0.97, clip 0.2, 기울기 노름 1, 관측 정규화 |
| 환경 | 평지, 에피소드 20 s, 명령 10 s마다: vx U[-0.2, 0.8], 회전 U[-0.6, 0.6] rad/s, 10 %는 정지. 명령 가속 제한 0.5, 보행 위상(주기 0.36 s), 관측 잡음, 연결 지연 0~2주기, 시작 관절 ±0.1 rad |
| 행동·PD | 카드 규칙: 기본 자세 + 0.25 × action, PD kp 40, kd 1 (물리 스텝마다), 토크 한계는 명세 |

보상 (제어 스텝마다, 합에 dt=0.02를 곱함):

| 항 | 식 | 가중치 |
|---|---|---|
| 전진·옆 속도 추종 | exp(-‖v_cmd,xy - v_xy‖² / 0.25) (몸통 좌표) | +1.0 |
| 회전 속도 추종 | exp(-(ω_cmd - ω_z)² / 0.25) | +0.5 |
| 위아래 속도 | v_z² | -2.0 |
| 몸통 흔들림 | ‖ω_xy‖² | -0.05 |
| 자세 (기울기) | ‖중력 벡터의 xy 성분‖² | -2.0 |
| 토크 | ‖τ‖² | -2e-4 |
| 행동 변화 | ‖a_t - a_t-1‖² | -0.01 |
| 발 공중 시간 | Σ (공중 시간 - 0.12 s) × 이번에 닿은 발 (걷기 명령일 때만) | +0.5 |
| 정지 자세 | ‖q - 기본 자세‖² (정지 명령일 때만) | -0.5 |
| 넘어짐 | 몸통 < 0.15 m 또는 기울기 60° 초과면 그 스텝 보상 = -2, 에피소드 끝 | |

결과와 원인: 1천만 스텝(54분)에서 정책은 넘어짐 기준 바로 위(0.155 m)로 웅크리고 서 있기만 배웠다.
- 서 있기만 해도 속도 추종 보상을 꽤 받는다 (명령 0.4 m/s에서 exp(-0.16/0.25) = 0.53, 회전 0.5와 합치면 1스텝 약 1.0).
- 걸으려다 넘어지면 큰 벌점을 받고, 몸통 높이 벌점은 없다. 그래서 낮게 웅크리면 넘어질 위험이 줄어든다 -> 흔한 국소 최적.
- 발 공중 시간 보상은 발을 들어야만 받는데, 처음 탐색에서 발을 들면 넘어지기 쉬워 이 보상을 거의 보지 못했다.
- 학습량도 적다. 이런 정책은 보통 수천만~수억 스텝을 학습한다 (큰 GPU에서 수십 분, 이 GPU에서 수 시간~하루).

개선 방향 (아직 하지 않음, 효과가 클 것으로 보는 순서):

| 개선 | 내용 | 기대 |
|---|---|---|
| 모방학습 정책으로 시작 | actor를 `go2_trot_bc_est`(관측이 같다)로 초기화하거나, 처음 얼마 동안 모방 손실(규칙 기반 트롯의 행동과의 차이)을 더한 뒤 PPO로 다듬는다 | 이미 걷는 상태에서 출발해 "웅크리고 서기"로 빠지지 않는다. 신경망 모양(지금 ELU 128-128, 출력 tanh 없음)을 맞춰야 한다 |
| 몸통 높이 벌점 | (z - 0.27)², 가중치 수십 | 웅크리기를 막는다 |
| 넘어짐 기준 | 0.15 -> 0.20 m | 웅크린 자세를 넘어짐으로 |
| 보행 위상 보상 | 정책이 받는 보행 위상에 맞춰 대각 다리(FL+RR / FR+RL)가 번갈아 디디면 보상, 발 들어 올리기(발 높이) 보상 | 발을 떼는 탐색을 유도 |
| 속도 추종을 날카롭게 | 오차 폭 0.25 -> 0.1 정도, 또는 정지 명령 비율을 줄임 | 서 있기로 얻는 점수를 줄인다 |
| 교육과정 | 처음엔 느린 명령, 잘 따라가면 빠르게. 평지 -> 요철 -> 홈 | 학습이 안정 |
| 무작위화 | 마찰, 질량, 밀기(외란), PD 게인 | 시뮬레이터 차이·실기에 강해짐 |
| 보상 항별 기록 | 항마다 평균을 로그로 | 어느 항이 학습을 끌고 가는지 진단 |
| 학습량 | 3천만~1억 스텝 (이 GPU에서 약 3~10시간) | |
| 제어 노드 | 다리 처짐 복구 기준을 카드의 PD 게인에 맞추거나 카드에서 끔 | 낮은 게인 정책을 오인해 덮어쓰지 않게 |

모방학습(`go2_trot_bc_est`, `train/imitate_trot.py`)은 어떻게 했나:
- **전문가**: 규칙 기반 트롯 보행기 (`control/trot.py`). **학생**: MLP 47-128-128-12 ELU, 관측 정규화를 ONNX에 넣음. numpy로 직접 학습(Adam, 평균제곱오차), PyTorch 없이 CPU.
- **DAgger 6회**: 처음에는 전문가가 로봇을 움직이고(β = 1), 회차마다 학생이 직접 움직이는 비율을 늘린다 (β = 1, 0.5, 0.25, 0.1, 0, 0).
  학생이 움직인 상태에서도 전문가의 정답 행동을 받아 데이터에 더한다. 전문가 상태만 배우면 작은 오차가 쌓여 못 본 상태로 흘러가 넘어지기 때문이다.
- **무작위 시나리오**: 요철(60 %), 범프, 상자형 홈, 2~4초마다 바뀌는 명령(정지 포함), 연결 지연 0~2주기. 관측은 로봇 경계의 추정값
  (센서 잡음 + 상태 추정기) 그대로. 5개 프로세스로 모아 113,895개 표본, CPU 323초.
- 결과: flat_trot 4.24 m, rough_rut 4.04 m, 넘어짐 없음.

모방학습의 여러 방식과 DAgger를 고른 이유:

| 방식 | 내용 | 특징 |
|---|---|---|
| 행동 복제 (Behavior Cloning, BC) | 전문가 시연(상태 -> 행동)을 모아 지도 학습 | 가장 기본. 학생이 작은 실수로 전문가가 안 가 본 상태에 들어가면 오차가 쌓여 무너진다 (공변량 이동) |
| DAgger (Ross 2011) | BC + 학생이 직접 움직인 상태에서 전문가에게 정답을 다시 물어 데이터에 더함 | 공변량 이동을 해결. 어떤 상태에서도 정답을 물어볼 수 있는 전문가가 필요하다 |
| 역강화학습 (IRL) | 시연에서 보상 함수를 추정한 뒤 강화학습 | 전문가의 의도까지 배운다. 계산이 무겁다 |
| 적대적 모방 (GAIL, AMP) | 판별기가 "전문가 같은 움직임인지"를 보상으로 줌 | 4족 보행에서는 AMP(Adversarial Motion Priors)로 걸음 모양을 동물처럼 만드는 데 많이 쓴다 |
| 교사-학생 증류 | 특권 정보를 아는 교사 정책(보통 강화학습)을 센서만 보는 학생이 따라 배움 | 4족 보행 연구에서 아주 흔하다. 학생 학습에 DAgger를 많이 쓴다 |

- 여기서는 전문가가 규칙 기반 트롯 보행기(코드)라서 학생이 어디로 가든 그 자리에서 정답을 바로 계산해 줄 수 있다. 그래서 DAgger를 썼다.
  사람 시연이나 녹화 데이터라면 이렇게 물어볼 수 없어 BC나 적대적 방식을 쓴다.
- 위 "학습 방침"의 "필요하면 강화학습 교사 -> 학생 증류"가 교사-학생 방식이다 (`imitate_trot.py`의 DAgger에서 전문가를 신경망 교사로 바꾸면 된다).

| | 모방학습 `go2_trot_bc_est` | MJX 강화학습 `go2_mjx_flat` |
|---|---|---|
| 방법 | DAgger (전문가 행동을 지도 학습) | PPO (보상 최대화) |
| 전문가 필요 | 예 (규칙 기반 트롯) | 아니요 (보상만) |
| 학습 신호 | 매 스텝 "정답 행동" (촘촘하고 직접적) | 보상 (간접적, 탐색 필요) |
| 데이터·시간 | 11만 표본, CPU 5분 | 1천만 스텝, GPU 54분 |
| 관측 | 배치와 같은 추정값 + 지연 | 참값 + 잡음 + 지연 (critic은 특권 정보) |
| 신경망 | 47-128-128-12 ELU | 47-128-128-128-12 silu, tanh × 2 |
| PD | kp 100/150, kd 4 (명세) | kp 40, kd 1 |
| 결과 (flat_trot 12초) | 4.24 m 걸음 | 0.28 m, 웅크리고 섬 |
| 한계 | 전문가를 넘어설 수 없다. 지형 인지 버전은 높이 입력을 무시해 실패 (`go2_trot_bc_terrain`) | 원리상 전문가를 넘어설 수 있지만 보상 설계와 계산량이 필요하다 |

모방학습이 쉽게 된 이유는 매 스텝 정답이 주어지기 때문이다. 강화학습은 보상을 보고 스스로 찾아야 해서 국소 최적에 빠지기 쉽다.
그래서 개선 방향의 첫째로 "모방학습 정책으로 시작"을 꼽았다. 이미 걷는 정책에서 출발해, 강화학습으로 전문가가 못 하는 것(예: 지형 판단)을 더하는 순서다.
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

## 실시간 코어 (C++, 문서 §11)

```bash
python -m sim.rt_link --build                                  # 코어 빌드 (cmake, mj_ros의 libmujoco 사용)
python -m sim.rt_link scenarios/flat_trot.yaml                 # 실시간: SCHED_FIFO 80, CPU 5 고정, 메모리 잠금
python -m sim.rt_link scenarios/flat_trot.yaml --lockstep      # 한 스텝씩 (Python 실행과 비교)
python -m sim.rt_link scenarios/flat_trot.yaml --brain shm     # 두뇌를 같은 PC의 별도 프로세스로 (공유 메모리)
python -m sim.rt_link scenarios/flat_trot.yaml --brain ros     # 두뇌를 ROS2 노드로 (같은 PC 또는 다른 PC)
python -m sim.rt_link scenarios/flat_trot.yaml --rviz          # RViz로 보기 (--mjviz: MuJoCo 렌더러). 두뇌 옵션과 함께 쓸 수 있다
python -m sim.rt_link scenarios/rough_rut.yaml --rviz          # 지형이 바뀌는 시나리오 (자국 이벤트, 지형 창)
python -m sim.rt_link scenarios/flat_trot.yaml --variant mjx   # 모델 설정 (sim.runner --variant와 같다)
python -m sim.rt_link scenarios/dis_console.yaml --dis-port 3000 --dis-wait --brain shm --rviz   # DIS 콘솔 (콘솔은 그대로)
python -m sim.rt_link scenarios/rut_crossing.yaml --brain shm --rviz --set controller.perception.sensor=front_lidar   # LiDAR 지형 인지 보행
python -m sim.rt_link scenarios/footprints.yaml --rviz         # Chrono (HMMWV, 변형 지면, 발자국). --lockstep이면 sim.runner와 같은 결과
python -m sim.rt_link scenarios/dis_chrono.yaml --dis-port 3000 --dis-wait --brain shm --rviz   # DIS 콘솔 + Chrono (spawn, soil)
python conformance/test_rt_core.py                             # 규약 일치, 물리 비트 동일, 닫힌 고리 동일, 두뇌 연결 방식·고장, 실시간
```

**실행기 고르기: `sim.runner`, `sim.rt_link`, `sim.rt_link --lockstep`**. 시나리오와 `--set` 옵션은 둘 다 같다.

| | `python -m sim.runner` | `python -m sim.rt_link` (기본) | `python -m sim.rt_link --lockstep` |
|---|---|---|---|
| 물리·흙 계산 | Python 프로세스 안 | C++ 코어 (별도 실행 파일) | C++ 코어 |
| 시간 | 비실시간 (lockstep) | **실시간**: 벽시계로 20 ms마다, 누가 늦어도 기다리지 않음 | 비실시간: 코어가 매 스텝 관리 프로세스와 두뇌를 기다림 |
| 두뇌가 늦으면 | 기다림 | 지난 명령으로 진행 (실제로 늦음) | 기다림 |
| 12초 시나리오 | 화면 없으면 몇 초 | 12초 | 화면 없으면 몇 초 |
| 결과 | 기준, 항상 같음 | 타이밍에 따라 조금 다를 수 있음 | 항상 같음, `sim.runner`와 비트 단위로 같음 |
| 통계적 지연 (`control_link`) | 됨 | 안 됨 (실제 지연) | 안 됨 |
| 무른 흙 (`foot_soil`), Chrono, LiDAR, DIS | 됨 | 됨 | 됨 (DIS 콘솔은 실시간 전용) |

- `--rviz`(또는 `--mjviz`)를 붙이면 lockstep도 벽시계보다 앞서지 않게 기다리며 돌아 실시간처럼 보인다. 계산은 그대로 한 스텝씩이라
  결과는 화면 없이 돌린 것과 같다 (`sim.runner --rviz`도 같다). 계산이 실시간보다 느리면(Chrono 등) 화면이 느리게 흐른다.
- C++ 코어에 lockstep이 있는 이유: 타이밍 차이를 없앤 상태에서 Python과 같은 물리를 계산하는지 확인하려고. lockstep 기록이 비트 단위로
  같으면 계산은 맞고, 실시간에서 달라지는 것은 순전히 타이밍 탓이라고 말할 수 있다 (무른 흙을 C++로 옮긴 뒤에도 이렇게 확인).
- 쓰는 곳: 대량 시험·결과 비교·재현은 `sim.runner`(또는 `sim.rt_link --lockstep`), 실제 시간 동작·DIS 콘솔 시험은 `sim.rt_link`.

**`sim.rt_link`의 역할: C++ 코어 경로의 관리자(오케스트레이터)**. 물리엔진(C++ 코어, Chrono), 센서, 두뇌를 띄우고 시각과 데이터를 맞춘다.
로봇 물리는 직접 계산하지 않는다. 렌더러와는 중립 스트림으로 느슨하게 이어진다.

```
                         sim.rt_link (관리 프로세스, Python)
        ┌──────────────┬──────────────┼──────────────┬───────────────┐
        ▼              ▼              ▼              ▼               ▼
   C++ 코어       Chrono 서버     LiDAR 작업      두뇌            DIS 서버
 (MuJoCo 물리,   (HMMWV, SCM 흙,  프로세스 2개   (inproc / shm /  (콘솔 요청 접수)
  PD, 센서 모델,  별도 프로세스)                  ros 노드)
  무른 흙)
        │ 공유 메모리      │ 소켓            │ 파이프        │ 공유 메모리·ROS2
        └──────────── 참값 ──────────────▶ 관리 프로세스 ──▶ ROS2 중립 스트림 ──▶ RViz / MuJoCo 화면 어댑터
```

| 역할 | 누가 하나 | `rt_link`가 하는 일 |
|---|---|---|
| 로봇 물리 (MuJoCo), PD 모터, 센서 모델, 무른 흙 | C++ 코어 | 띄우고, 모델·설정·잡음·지형을 공유 메모리로 넘김 |
| 두 번째 물리엔진 (Chrono: HMMWV, 흙 자국) | Chrono 서버 프로세스 | 띄우고, 시각을 맞춰 결과를 주고받음. 실시간에서 많이 뒤처지면 코어를 잠깐 멈춤 |
| LiDAR 계산 | 센서 작업 프로세스 | 스캔 차례를 정하고 결과를 두뇌에 전달 |
| 두뇌 (상위 제어기) | inproc이면 rt_link 안, shm·ros면 별도 프로세스 | 띄우고 연결 (ros면 공유 메모리 <-> ROS2 중계) |
| 시간 진행 | 실시간: 코어가 벽시계로 / lockstep: rt_link가 한 스텝씩 허락 | lockstep에서만 시계를 쥠 |
| 이벤트, 지형 변경, 고장, DIS 콘솔 | rt_link | 직접 함 (Terrain Map Service, 고장 값, 이벤트 적용 시각 기록) |
| 판정·기록 (넘어짐, MOP, timeseries) | rt_link | 직접 함 (코어의 참값으로) |
| 화면 | RViz / MuJoCo 어댑터 (별도 프로세스) | 중립 스트림(ROS2 토픽)을 발행할 뿐, 렌더러를 직접 조종하지 않음. 화면이 몇 개든, 다른 PC든 상관없음 |

`sim.runner`도 같은 관리 역할을 하지만 로봇 물리까지 자기 프로세스 안에서 직접 계산한다.

**관리(오케스트레이션) 때문에 실시간성이 깨지지 않나**: lockstep과 실시간은 실행할 때 고르는 다른 모드라 섞이지 않는다
(`--lockstep`이면 처음부터 끝까지 비실시간, 없으면 끝까지 실시간). 실시간 모드에서 코어는 관리 프로세스를 기다리지 않는다.

| 장치 | 내용 |
|---|---|
| 코어 혼자 시계를 쥠 | 20 ms마다 절대 시각까지 잠들었다 깨어난다 (`clock_nanosleep`). 관리 프로세스·두뇌의 응답을 기다리는 코드가 실시간 경로에 없다 |
| 실시간 우선순위·격리 | SCHED_FIFO 80, CPU 5 고정, 메모리 잠금(`mlockall`), 루프 안 할당 없음 (무른 흙 구멍 지도도 미리 잡음) |
| 공유 메모리로만 주고받음 | 코어는 마지막으로 다 쓴 명령·지형·고장 값을 읽기만 한다. 쓰는 중이면 기다리지 않고 지난 값을 쓴다 |
| 참값·잡음 링 | 코어는 참값을 링에 쓰고 지나간다. 관리 프로세스가 늦으면 따라잡을 뿐 코어를 막지 않는다 |

늦음은 결과에 드러나고 잰다:

| 늦는 쪽 | 코어의 동작 | 보이는 곳 |
|---|---|---|
| 두뇌 | 지난 명령으로 진행 (실제 로봇과 같음) | `cmd_late_steps`, `cmd_latency_ms_*` |
| 관리 프로세스 (이벤트, 지형, 고장) | 다음 스텝 이후에 반영 | `late_applies` (경고도 출력) |
| 관리 프로세스 (센서 잡음 미리 채우기) | 잡음 0으로 대신 | `noise_underruns` |
| 코어 자신의 계산 | 주기를 넘김 | `overruns`, `compute_us_*` |

예외로 실시간 코어를 일부러 멈추는 경우가 둘 있다: (1) Chrono가 0.2 s 넘게 뒤처질 때(`--chrono-max-lag`, 차가 로봇을 뚫고 지나가지 않게.
`chrono_holds`, `chrono_hold_s`. 이 PC에서는 Chrono가 실시간보다 느려 자주 일어난다), (2) DIS 콘솔의 일시정지. 둘 다 정해 둔 규칙대로
멈추고, 그 시간은 실시간 지연으로 세지 않는다 (코어가 다시 시계를 맞춘다). `--rviz`일 때 벽시계에 맞춰 기다리는 것은 lockstep 모드에만
해당하고, 실시간 모드에서는 화면이 코어 속도에 영향을 주지 않는다.

**상위 제어기(두뇌)와 하위 제어기(몸)의 연결** (2026-10-09 결정: 같은 PC와 다른 PC 모두 지원)
```
[하위 제어기 PC]                                                         [상위 제어기 PC]
 C++ 코어 (PD 모터 + 물리 + 센서) ─공유 메모리─┬─ inproc: sim.rt_link 안의 두뇌 (함수 호출)
                                                ├─ shm:    같은 PC의 별도 프로세스 control/rt_brain.py
                                                └─ ros:    sim.rt_link가 중계 ──ROS2──▶ control/ros_node.py (같은 PC 또는 다른 PC)
```
- 코어는 어느 방식이든 공유 메모리만 본다. 네트워크 입출력은 실시간 루프 밖(관리 프로세스)에서 하므로 네트워크가 늦거나 끊겨도
  코어의 20 ms 주기는 흔들리지 않고, "마지막 명령 유지 + 명령 지연 MOP"로 드러난다.
- 두뇌 코드(`control/node.py`의 `ControllerNode`)는 세 방식이 같다. 운용자 이동 명령은 적용 시각을 붙여 미리 보낸다
  (shm: 공유 메모리 명령 링, ros: `/cmd_vel_stamped`). 고장(배터리, IMU, 링크 두절)은 코어가 처리한다.
- 명령에 "어느 시각의 상태로 계산했는지"(시뮬레이션 시각)가 실려 있어, 두 PC의 시계를 맞추지 않아도 지연을 잰다.
- 다른 PC: `python -m sim.rt_link <시나리오> --brain ros --remote-brain`으로 띄우면 노드를 기다린다. 두뇌 PC에서 같은
  `ROS_DOMAIN_ID`로 `python -m control.ros_node` (어느 쪽을 먼저 띄워도 된다). (이 PC 안에서만 시험했다)
  시나리오 파일은 보내지 않아도 된다: 시뮬레이터가 `--set`까지 반영한 controller 절, 정책 카드(저장소 기준 경로), 로봇 메시지 형식을
  `/robot/brain_config`(std_msgs/String JSON, 늦게 붙어도 받는 토픽)로 내고, `--scenario` 없는 노드는 이걸 받아 같은 설정으로 뜬다.
  화면에 렌더러(`--rviz`, `--mjviz`, `--godot`)가 켜져 있으면 그것도 알려, 두뇌가 지형 인지 상태(`/control/terrain_map`,
  `/control/footholds`, 화면용이고 로봇 동작과 무관)를 알아서 발행한다. 두뇌 쪽에서 정하려면 `--publish-perception` / `--no-publish-perception`
  (예전 이름 `--viz`도 받는다).
  정책 카드와 ONNX는 두뇌 PC의 저장소에도 있어야 한다. `--scenario`/`--policy`/`--ros-msg`를 주면 그 값이 우선한다.
  확인 (같은 PC, 두뇌를 먼저 띄움): rough_rut + 지형 인지(`--set controller.perception.sensor=front_lidar`)가 노드에 그대로 전달됐고,
  flat_trot + `--policy policies/go2_trot_bc/card.yaml`이 ONNX 정책으로 떴다. 렌더러를 켜면 두뇌가 "렌더러 켬"을 받아 지형 지도를 발행하고
  (Godot 중계기가 `/control/terrain_map`을 구독), 끄면 그 토픽이 생기지 않았다. `test_ros_equivalence` 4개 통과.
  같은 PC에서 두 PC처럼 띄우기 (두 터미널 모두 `export ROS_DOMAIN_ID=77`처럼 같은 번호):
  ```bash
  python -m control.ros_node                                                    # 터미널 2 (두뇌), 먼저 띄워도 됨
  python -m sim.rt_link scenarios/rough_rut.yaml --brain ros --remote-brain --godot-lidar --set controller.perception.sensor=front_lidar   # 터미널 1
  ```
  두뇌 노드는 실행 한 번에 설정을 한 번 받는다 (시뮬레이션을 다시 돌리면 두뇌도 다시 띄운다).
  **실시간 결과가 실행마다 크게 다르다** (rough_rut + 지형 인지, ROS 두뇌 실시간 15초): 같은 조건에서 전진 2.2, 3.5, 4.4 m
  (다른 실행 4.05, 4.21, 4.15, 1.91 m). 두뇌의 화면용 발행 여부와는 관계없었다 (켬 2.2, 4.4 m / 끔 3.5 m).
  같은 프로세스 실행은 1.9~2.3 m였다 (`sim.runner` 1.888 m, `sim.rt_link --brain shm` 2.293 m, 각 한 번).
  lockstep에서는 ROS 두뇌도 같은 프로세스와 비트 단위로 같다 (`test_perceptive_trot_ros_node_matches_in_process`).
  원인은 아직 조사하지 않았다 (LiDAR 스캔과 명령이 네트워크로 오가는 시점이 매번 달라서일 수 있다. 늦은 주기 2~34개, 스캔 버림 4~6개).
- lockstep: 세 방식이 Python 시뮬레이터와 비트 단위로 같다. 링크 두절(감쇠·유지), 배터리 저하가 섞여도 같다 (복구 뒤 일어서기 포함).
  링크 두절이 풀린 직후에는 두절 전에 계산된 명령을 실행하지 않고 직전에 실행한 명령을 유지한다 (Python과 같게 맞춤).
- 실시간 15초 (750주기): 명령 지연 평균/최대, 한 주기보다 늦게 실행된 주기 수

| 두뇌 | 지연 평균 / 최대 | 늦은 주기 |
|---|---|---|
| inproc (관리 작업과 한 프로세스) | 20.0 / 40 ms | 1 |
| shm (별도 프로세스) | 20.0 / 20 ms | 0 |
| ros (ROS2, 이 측정 때는 JSON 메시지) | 20.1 / 60 ms | 2 |

- 로봇 경계 메시지는 커스텀 ROS2 메시지 `go2_rt_msgs`다 (2026-10-10, 아래 "로봇 경계 ROS2 메시지"). JSON도 선택지로 남겼다.

**나눈 일**
```
C++ 코어 rt/rt_core (실시간 루프)                     Python 관리 프로세스 sim/rt_link.py (실시간 아님)
  20 ms마다 (절대 시각 clock_nanosleep)                  모델(.mjb)·시작 상태·센서 편향 준비, 코어 띄우기
   1. 로봇 상태 측정 (센서 모델) ──LowState──▶  공유 메모리  ──▶ 보행 알고리즘 control/ (지금 코드 그대로)
   2. 최신 관절 명령 고르기      ◀──LowCmd──   /dev/shm    ◀──  관절 명령
   3. 물리 10스텝 (PD 모터 -> mj_step)            ◀── 센서 잡음 (미리, Python과 같은 난수)
   4. 참값·타이밍 기록          ──참값 링──▶               ──▶ 넘어짐 판정, MOP, 이벤트(이동 명령, 고장)
```
- 코어는 시작 전에 메모리를 잠그고(`mlockall`), CPU를 고정하고, SCHED_FIFO로 돈다. 루프 안에서는 할당·입출력·잠금을 하지 않는다.
  공유 메모리 칸은 seqlock(쓰는 쪽 하나)으로 주고받는다. 규약은 `rt/shm_layout.h`와 `sim/rt_link.py`(ctypes)에 같은 것을 두고 시험으로 대조한다.
- 명령은 직전 주기까지의 상태로 계산된 최신 것을 쓴다 (연결 지연 한 주기, `sim.runner`와 같다). 보행 알고리즘이 늦으면 마지막 명령을
  계속 쓰고 늦은 만큼이 명령 지연 MOP에 드러난다. 링크 두절(마지막 명령 유지 / 감쇠), 배터리 저하, IMU 고장도 코어가 처리한다.
- 센서 잡음은 Python이 `(seed, 표본 번호)` 난수로 미리 만들어 링 버퍼로 넘긴다 (`ProprioSensors.draw_noise`). 그래서 같은 실행은 같은 잡음이다.

**같은 결과** (`test_rt_core.py`)
- Python 시뮬레이터가 실행한 관절 명령을 그대로 넣으면 200스텝 매번 몸통·관절 상태가 비트 단위로 같다 (같은 libmujoco 3.5,
  같은 PD 식과 계산 순서, `-ffp-contract=off`). 로봇 상태 출력은 삼각함수·노름의 마지막 자리 차이로 최대 1.7e-13.
- 보행 알고리즘까지 붙인 lockstep 실행도 Python 실행과 끝 상태가 비트 단위로 같다 (flat_trot: 전진 3.918 m, CoT 2.404).

**실시간 측정** (flat_trot 30초 = 1500주기, 이 PC 기본 설정: 절전 `powersave`, 깊은 대기 상태 C10까지, 코어 격리 없음, 데스크톱 사용 중)

| | Python 루프 (`sim.runner --realtime`과 같은 방식, SCHED_FIFO) | C++ 코어 |
|---|---|---|
| 주기 초과 | 25번 (1.7%) | **0번** |
| 깨어남 지연 평균 / p99 / 최대 | 0.13 / 1.2 / 5.4 ms | **0.07 / 0.26 / 0.74 ms** |
| 한 주기 계산 평균 / p99 / 최대 | 8.7 / 21 / 24 ms | 7.3 / 14 / 19 ms |
| 명령 지연 | – | 항상 20 ms (보행 알고리즘이 매 주기 제때 응답) |

- 계산 시간이 아직 크고 들쭉날쭉한 것은 CPU 절전 때문이다: 같은 물리 10스텝이 CPU가 쉬지 않으면 0.8 ms, 주기마다 쉬면 3.4 ms이고
  가끔 24 ms까지 튄다 (C++ 측정). 잠들지 않고 기다리면(spin) 빨라지지만 커널 RT 스로틀링(1초에 0.95초까지)에 걸려 매초 멈췄다.
  그래서 시간 보장의 다음 단계는 시스템 설정이다 (루트 권한): CPU 주파수 `performance`, `/dev/cpu_dma_latency`로 깊은 대기 상태 금지,
  커널 인자 `isolcpus=4,5 nohz_full=4,5 rcu_nocbs=4,5 irqaffinity=0-3`로 실시간 코어 격리.

**RT 커널과 시스템 설정** (2026-10-09 확인, 이 PC: i5-9400 6코어)

RT 커널은 부팅된 순간부터 항상 켜져 있다 (`PREEMPT_RT`는 컴파일 단계 기능이라 실행 중 켜고 끄는 스위치가 없다). 따로 "활성화"할 것은 없고,
좋아질 여지는 커널 위의 시스템 설정(절전)에 있다.

| 확인 항목 | 이 PC 값 | 뜻 |
|---|---|---|
| `uname -v` | `PREEMPT_RT` | 완전 선점형 실시간 커널로 부팅됨 |
| `/boot/config-$(uname -r)` | `CONFIG_PREEMPT_RT=y` | 실시간 커널 설정 |
| `cat /sys/kernel/realtime` | 1 | 실시간 커널로 동작 중 |
| `ulimit -r` | 99 | 일반 사용자도 SCHED_FIFO 사용 가능 |
| `ulimit -l` | 약 1 GB | 코어의 `mlockall` 가능 |
| CPU 주파수 정책 | `powersave` (800 MHz~4.1 GHz) | 절전: 낮은 클럭 유지 |
| 깊은 대기 상태 | C10까지 (깨어남 최대 890 µs) | 절전: 깨어날 때 늦음 |
| 코어 격리 | 없음 | CPU 5에 다른 작업·인터럽트도 옴 |

부팅할 때마다 확인: `cat /sys/kernel/realtime` (1이면 RT 커널), `uname -v` (`PREEMPT_RT`). 일반 커널(generic, 예: GPU 시험용)로
부팅하면 RT 기능이 없다 (일반 커널의 부팅 옵션 `preempt=full` 등은 선점 정도만 바꾸며 PREEMPT_RT와 다르다).

RT 커널이어도 자동이 아닌 것:
1. **프로그램이 직접 요청해야 실시간으로 돈다.** 보통 프로그램은 RT 커널에서도 일반 스케줄링이다. C++ 코어는 스스로 SCHED_FIFO 80,
   메모리 잠금, CPU 고정을 요청하고 결과를 MOP(`sched_fifo`, `mlockall`)에 남긴다 (지금까지 모두 True). `sim.runner`는 `--rt`일 때만
   (우선순위 50). 두뇌, 관리 프로세스, Chrono, LiDAR 작업 프로세스는 일반 스케줄링이다.
2. **실시간 작업에도 시간 제한이 있다.** 1초 중 0.95초까지만 (`/proc/sys/kernel/sched_rt_runtime_us` 950000). 코어는 주기마다 잠들어
   해당하지 않는다 (잠들지 않고 기다리는 방식을 시험했을 때 여기에 걸려 매초 멈췄다).
3. **절전 설정은 RT 커널과 별개다.** 주파수 정책과 깊은 대기 상태는 그대로라, 남은 계산 시간 튐(최대 19~24 ms)과 가끔의 주기 초과 원인이다.

시스템 설정 (sudo, 직접 실행. 앞의 두 가지는 재부팅하면 원래대로):

| 설정 | 명령 | 되돌리기 | 기대 효과 |
|---|---|---|---|
| 주파수 고정 | `echo performance \| sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor` | `powersave`로 같은 명령 | 계산 시간 튐 감소 (물리 10스텝: 쉬지 않으면 0.8 ms, 쉬다 깨면 3.4 ms). Chrono·LiDAR도 빨라질 가능성 |
| 깊은 대기 상태 끄기 (C3~C10) | `for s in /sys/devices/system/cpu/cpu*/cpuidle/state[3-8]/disable; do echo 1 \| sudo tee $s; done` | `echo 0`으로 같은 명령 | 깨어남 지연·계산 최대값 감소 → 주기 초과 감소 |
| 코어 격리 | GRUB `GRUB_CMDLINE_LINUX`에 `isolcpus=4,5 nohz_full=4,5 rcu_nocbs=4,5 irqaffinity=0-3` 추가, `sudo update-grub`, 재부팅 | 지우고 다시 `update-grub` | CPU 5(코어)에 다른 작업·인터럽트가 오지 않음. 가장 확실하지만 부팅 설정 변경 |

- 좋아지지 않는 것: 두뇌·LiDAR·Chrono의 계산량 자체 (최고 클럭 유지로 빨라질 수는 있다. HMMWV 멈춤이 얼마나 줄지는 재 봐야 안다).
  shm 두뇌의 명령 지연은 이미 항상 20 ms.
- 대가: 소비 전력과 발열 증가. 시험할 때만 켜고 끝나면 되돌리거나 재부팅한다.
- 권하는 순서: 재부팅 없는 두 가지를 먼저 켜고 같은 측정(flat_trot 30초의 주기 초과·깨어남·계산 시간, vehicle_crossing 실시간의 Chrono
  멈춤)으로 전후를 비교한 뒤, 부족하면 코어 격리를 검토한다.

**측정 결과** (2026-10-10, shm 두뇌, 화면 없음. 1단계 = 주파수 `performance` + C3~C10 끔, 코어 격리 없음)

| 측정 | 설정 전 (`powersave`, C10까지) | 1단계 후 |
|---|---|---|
| flat_trot 30초: 주기 초과 | 0 | 0 |
| 깨어남 지연 평균 / p99 / 최대 | 0.055 / 0.18 / 0.28 ms | **0.006 / 0.012 / 0.025 ms** (약 1/10) |
| 한 주기 계산 평균 / p99 / 최대 | 6.6 / 13.1 / 16.2 ms | **1.8 / 3.4 / 5.0 ms** (약 1/3) |
| vehicle_crossing 16초 x2: Chrono 멈춤 | 44번 7.2 s / 44번 7.2 s | 44번 7.2 s / 43번 7.1 s |
| Chrono 뒤처짐 최대 | 0.22~0.24 s | 0.22 s |
| 늦은 명령 | 0 | 0 |

- C++ 몸: 20 ms 주기에서 계산 최대가 16.2 -> 5.0 ms로 여유가 크게 생기고, 깨어나는 시각도 거의 정확해졌다. 들쭉날쭉했던 원인은 절전 설정이었다.
- HMMWV 멈춤은 그대로: Chrono는 쉬지 않고 계산하는 프로세스라 절전 설정에서도 이미 최고 클럭으로 돌고 있었다. Chrono는 CPU 계산량이
  한계라 이 설정으로는 빨라지지 않는다. 줄이려면 더 빠른 CPU이거나 Chrono 설정(계산 간격, SCM 영역)을 낮춰야 한다.
- 코어 격리(2단계)는 지금 권하지 않는다: 몸은 이미 20 ms 중 최대 5 ms만 쓰고 주기 초과도 0이다. `isolcpus=4,5`로 두 코어를 떼면
  Chrono, LiDAR 작업 프로세스, 두뇌, 관리 프로세스, RViz가 남은 4코어를 나눠 써 HMMWV 멈춤이 오히려 늘 수 있다. 한다면 코어가 쓰는
  CPU 5 하나만 (`isolcpus=5 nohz_full=5 rcu_nocbs=5 irqaffinity=0-4`, 다른 무거운 작업을 같이 돌릴 때 대비).
- 정리: 실시간 시험 전에 1단계를 켜고, 끝나면 되돌리거나 재부팅한다.

**가시화** (`--rviz`, `--mjviz`): 관리 프로세스가 코어의 참값으로 자기 쪽 MuJoCo 데이터를 맞추고(`mj_forward`) `sim.runner`와 같은
렌더 스트림(`/tf`, `/clock`, 지형, 정보판)을 25 Hz로 발행한다. RViz 설정과 어댑터는 그대로다. 실시간 루프 밖이라 화면이 느려도 코어 주기에는
영향이 없다 (확인: 10초 발행 중 주기 초과 0, 별도 프로세스 두뇌의 늦은 주기 0). 창을 닫으면 끝나고, 끝나면 최종 MOP를 정보판에 남긴다.
실행 폴더 `runs/<시나리오>_rt_<시각>/`에 로그와 `summary.json`(MOP, 코어·두뇌 종류)이 남는다.

**기록** (`timeseries.parquet`, `sim.runner`와 같은 형식): 관리 프로세스가 코어의 참값 링을 읽어 같은 열을 남기고, 실시간 열
(`wake_us`, `compute_us`, `cmd_state_t`)을 더한다. 발 접촉, 몸통 회전, 발 위치는 코어가 물리 스텝 직후 mjData 값을 그대로 넘긴다
(`sim.runner`가 기록하는 값과 같게: 위치를 다시 계산하면 접촉과 몸통 속도가 조금 달랐다). 스텝 수도 `sim.runner`와 같이
"시각 < duration"인 동안으로 센다 (20초 시나리오는 부동소수점 누적으로 1001스텝).

**지형 변경**: 관리 프로세스가 `sim.runner`와 같은 규칙(지형 패치 이벤트, 5 Hz 반영, 발 근처 갱신 보류, 지형 창 이동)으로 지형을 계산하고,
바뀌면 지형 공유 메모리(`<이름>_terrain`: heightfield 값 + 지형 창 위치)로 보낸다. 코어는 다음 제어 주기 시작에 반영한다.
RViz 지형 표시는 관리 프로세스의 지형을 그대로 쓴다.

**lockstep 같은 결과** (`test_recording_matches_runner`): `timeseries.parquet`의 모든 공통 열(80개)과 MOP가 `sim.runner`와 비트 단위로 같다.
flat_trot (cpu, mjx 모델), rough_rut (자국 이벤트), 지형 창 + 실행 중 자국 추가 (dis_console 지형, 10초 이동).

**DIS 콘솔** (`--dis-port`, `--dis-wait`, `--dis-key`, `--dis-host`, `--dis-comm-lost`, 실시간 전용): `sim/dis_server.py`를 그대로 쓴다
(제어권, 인증, 통신 두절, 원본 PDU 기록, 시험 종료 알림). 관리 프로세스가 서버가 넣은 이벤트를 같은 이벤트 목록에서 꺼내 적용하고 적용 시각을
기록하므로 접수 -> 적용 완료 응답과 `scenario_replay.yaml`이 그대로 나온다. 일시정지는 코어의 멈춤 칸으로 처리하고(풀리면 코어가 시계를
다시 맞춤), 종료는 `sim.runner`처럼 그 주기를 마친 뒤 멈춘다. 별도 프로세스 두뇌에 미리 보낸 이동 명령은 실제로 적용되는 시각에 완료로 표시한다.
확인 (`test_dis_console_on_rt_core`, 별도 프로세스 두뇌): 콘솔로 이동·자국·감쇠 링크 두절·예약 종료 -> 남은 `scenario_replay.yaml`을
Python 시뮬레이터(`sim.runner`)로 다시 돌리면 같은 결과 (실시간 실행에서 늦은 명령이 없을 때. 인증·일시정지·배터리까지 넣은 세션도 같았다).

**콘솔 요청을 기록대로 다시 실행하기** (2026-10-11 수정): 실시간 실행과 그 `scenario_replay.yaml` 재실행이 가끔 달랐다
(`test_dis_console_on_rt_core` 한 번 실패, 늦은 명령 0). 실행 도중 시각 없이(`@` 없이) 보낸 요청을 재현해 원인 셋을 찾았다.

| 원인 | 증상 (재현) | 수정 |
|---|---|---|
| 즉시 고장의 끝 시각을 요청 시각부터 셈 | `fault link 0.5 damp`가 한 주기 일찍 끝남. 실시간 0.862 m, 재실행 0.842 m (4번 모두) | 끝 시각 = 적용 시각 + 지속 시간 (`sim.runner`와 같게) |
| 지난 시각이 붙은 즉시 이동 명령 | 두뇌는 아직 처리하지 않은 첫 상태부터 적용해, 어느 상태인지가 타이밍에 따라 갈림 (기록 시각과 어긋날 수 있음) | 다음 스텝 시각으로 정해 보내고 기록. 같은 프로세스 두뇌도 시각 순서 대기열로 (세 두뇌 같은 규칙) |
| 지형 반영이 코어보다 늦음 | 관리 프로세스의 지형 반영이 세계 지도(1201x1201) 전체를 매번 계산해 9~16 ms, 홈 패치는 약 100 ms. 코어가 다음 스텝을 이미 시작해 1~5스텝 늦게 걸림 | 바뀐 셀만 반영 (약 30 -> 0.1 ms), 홈 패치는 그 사각형만 계산·수정 (59 -> 0.2 ms), 시각이 정해진 패치는 1초 미리 계산 |

- 지형 계산 변경은 결과가 이전과 같다: 무작위 패치 150개(홈, 범프, 거친 지형, 경사)로 이전 코드와 같은 지형, 바뀐 셀만 보는 반영 = 전체 비교 규칙
  (`test_commit_on_changed_cells_matches_full_rule`), lockstep 시험 전부 비트 동일 그대로.
- **늦게 걸린 변경 감시** (공유 메모리 규약 10판): 관리 프로세스가 고장 값을 바꿀 때마다 판 번호(`ctl.fault_seq`)를 올리고, 코어는 스텝마다
  자기가 쓴 고장 판과 지형 판을 참값 기록(`TruthEntry.fault_seq`, `terrain_seq`)에 남긴다. 관리 프로세스는 변경이 정한 스텝보다 늦게 걸리면
  세고 경고한다 (MOP `late_applies`). 0이 아니면 그 실행은 재실행과 다를 수 있다 (시험은 늦은 명령·늦게 걸린 변경이 없을 때만 비교).
- 남은 것: 시각 없이 보낸 즉시 패치는 미리 계산할 수 없어, 범프·경사·거친 지형처럼 지도 전체를 바꾸는 패치는 실시간에서 늦게 걸릴 수 있다
  (`late_applies`로 보임). 관리 프로세스가 크게 밀리면(다른 무거운 작업) 고장도 늦을 수 있다. 같은 PC ROS 두뇌는 실제 ROS 지연(가끔 한 주기)이 있어 비교 대상이 아니다.
- 확인 (`test_dis_immediate_events_replay_on_rt_core`): 실행 1초 뒤 시각 없이 이동·홈·감쇠 링크 두절 -> 감쇠 26주기(0.5 s + 복구 주기),
  재실행과 같은 결과. 재현 스크립트로 shm 두뇌 9번 중 늦은 명령이 없던 8번, inproc 2번이 모두 같았다
  (나머지 1번은 시작 직후 늦은 명령 1개로 비교 대상 아님).

**lockstep shm 두뇌가 부하에서 가끔 한 스텝 어긋나던 것** (2026-10-17 수정): 전체 시험 중 `test_brain_transports_and_faults_match_python`이
shm 두뇌에서 한 번 실패했다 (관절 각도 차이 0.00044). 그때 일반 터미널에서 실시간 C++ 코어가 같은 CPU 5번을 쓰고 있었다.
같은 부하(실시간 코어 하나를 띄워 둠)를 만들어 재현하니 8~24번에 1번꼴로 어긋났다.

| 원인 | 수정 |
|---|---|
| 관리 프로세스가 "두뇌가 이 상태로 명령을 냈다"를 명령 칸의 시각(`t`)만 보고 판단했다. 두뇌는 시각을 먼저 쓰고 관절 값 배열을 쓴 뒤 seq를 올리므로, 배열을 쓰는 도중에 코어를 다음 스텝으로 보냈고, 코어는 쓰는 중인 칸을 읽지 못해 직전 명령으로 한 스텝을 돌았다 | `ShmView.cmd_t()`를 seqlock으로 읽는다 (다 쓴 명령만) |
| 코어가 스텝 시작에 상태를 먼저 내보내고 명령을 골랐다. 두뇌가 그 사이 새 상태로 곧바로 새 명령을 쓰면 한 칸짜리 명령 칸에서 실행할 직전 명령이 덮어써질 수 있다 (추가 방어) | 명령을 고른 뒤 상태를 내보낸다 (`rt/rt_core.cpp`, 결과 규칙은 그대로) |

- 확인: 같은 부하에서 30번씩, 고친 코어와 예전 순서의 코어 + 고친 `cmd_t()` 모두 어긋남 0 (`cmd_t()`가 주된 원인).
  inproc·ros 두뇌는 관리 프로세스가 명령을 직접 쓰고 나서 넘어가므로 해당 없다.

**LiDAR·지형 인지 보행** (`controller.perception.sensor`가 있는 시나리오):
```
C++ 코어 ──참값 링(스텝 직후 바디 위치)──▶ 관리 프로세스: 스캔 차례 판단 (10 Hz) ──렌더 스트림(지형 패치, 바디 포즈)──▶ LiDAR 작업 프로세스
                                                                                                    (sim/rt_sensor.py, CPU 레이캐스트)
두뇌 ◀── 스캔 (inproc: 함수, shm: 스캔 공유 메모리 <이름>_scan, ros: /sensors/<이름>/points) ◀── 관리 프로세스 ◀── 스캔
```
- 스캔 한 번이 이 PC에서 약 98 ms(최대 107 ms)다. 관리 프로세스 안에서 계산하면 10 Hz만으로 실시간을 넘쳐 같은 프로세스 두뇌가 밀렸고
  (12초에 0 m), 작업 프로세스 하나로는 스캔이 쌓였다 (두뇌 도착까지 평균 0.4~0.56초). 그래서 실시간에서는 작업 프로세스 2개가 번갈아
  계산한다. 둘 다 밀려 있으면 그 스캔은 건너뛴다 (실제 LiDAR 드라이버의 프레임 버림과 같다, MOP `scan_dropped`).
  잡음은 스캔 번호로 정해지므로 어느 작업 프로세스가 계산해도 같은 점군이다.
- 실시간에서 스캔은 계산 시간만큼 늦게 두뇌에 도착한다 (MOP `scan_delay_ms_mean/max`). 두뇌는 스캔 시각의 자기 추정 위치로 지도에
  넣으므로 늦게 와도 위치는 맞다 (lockstep에서 스캔을 0/100/200 ms 늦게 넣어도 전진 3.71/3.64/3.68 m, 넘어짐 없음).
- LiDAR 끊김 고장(`lidar_blackout`)은 관리 프로세스가 스캔을 보낼 때 거른다. 원시 점군은 `sim.runner`처럼 `<센서>.npz`로 남는다.
- 상태 추정 기록: 별도 프로세스 두뇌(shm)는 추정값과 가속 제한 뒤 명령을 공유 메모리 추정 칸으로, ROS2 노드는 `/controller/estimate`로
  넘긴다. lockstep에서는 이것까지 받은 뒤 기록한다.
- 화면: 알고리즘 지형 지도와 디딜 곳 표시는 inproc(관리 프로세스가 발행)과 ros(노드가 발행)에서 나온다. shm은 로봇·지형·점군·odom 정렬만 나온다.
- lockstep 같은 결과 (`test_perception_matches_runner`): rut_crossing + 4~5초 LiDAR 끊김, 세 두뇌 모두 `timeseries.parquet` 모든 열과
  스캔 점이 `sim.runner`와 비트 단위로 같다.
- 실시간 12초 (rut_crossing, lockstep 전진 3.71 m)

| 두뇌 | 전진 | 늦은 주기 | 명령 지연 최대 | 스캔 지연 평균 / 최대 | 건너뛴 스캔 |
|---|---|---|---|---|---|
| shm (3번) | 3.70 / 3.74 / 3.74 m | 0 / 600 | 20 ms | 93~96 / ~460 ms | 4~5 / 120 |
| inproc | 3.65 m | 28 / 600 | 580 ms | 99 / 660 ms | 5 / 120 |

  inproc은 두뇌와 관리 작업(기록, 스캔 전달, 화면)이 한 프로세스라 가끔 밀린다. 지형 인지 보행은 shm 또는 ros를 쓴다.

**Chrono** (HMMWV, SCM 변형 지면, 발자국):
- 관리 프로세스가 `sim.runner`와 같은 자리(다음 스텝의 이벤트 뒤, 지형 갱신 전)에서 Chrono와 교환한다: 로봇 위치(차량 양보 판단)와
  스텝마다 코어가 넘긴 발 위치·평균 수직력(발자국)을 보내고, 차량 포즈와 흙 높이를 받는다. 흙 높이는 지형 원천에 들어가 지형 공유 메모리로
  코어에 간다. 개체 투입·제거(`create_entity`, `remove_entity`), 흙 바꾸기(`set_soil`) 이벤트, 차량 근접 이벤트, 차량 최소 거리와
  지형 변형 MOP, 기록의 차량 열, 화면·LiDAR의 차량도 같다.
- 화면(`--rviz`, `--mjviz`)을 띄우면 lockstep도 벽시계보다 앞서지 않게 맞춘다 (`sim.runner --rviz`와 같은 속도. 없으면 최대 속도로
  돌아 빨리 감기처럼 보였다). 계산 결과는 같다.
- lockstep: 교환마다 Chrono 결과를 기다린다. `sim.runner`와 비트 단위로 같다.
- 실시간 (2026-10-09 결정: 기다리지 않는 교환 + 뒤처짐 상한): 몸은 20 ms마다 돌고, Chrono 결과가 아직 없으면 직전 지형·차량 포즈를 쓴다.
  Chrono가 0.2 s(`--chrono-max-lag`) 넘게 뒤처지면 몸(코어)을 일시정지 칸으로 잠깐 멈추고, 절반까지 따라잡으면 다시 돈다. 그동안 두뇌도
  새 상태가 없어 같이 멈춘다. 세계 시간이 Chrono 속도로 느려지는 대신 차와 로봇이 0.2 s 안의 같은 시각에 있다. 멈춘 횟수·시간은 MOP
  `chrono_holds`, `chrono_hold_s`, 뒤처짐은 `chrono_lag_s_max`, `chrono_lag_s_end`. 결과는 실행마다 조금 다르다 (결과 도착 시각에 따라).
- 상한이 없으면(`--chrono-max-lag 0`, 처음 구현) 이 PC에서 HMMWV가 10초에 3~5초 뒤처져 늦게 왔고, 로봇이 이미 차 진로에 들어선 뒤라
  차가 로봇을 뚫고 지나갔다 (차와 로봇에는 물리 작용이 없다. vehicle_crossing 16초에 로봇이 차체 범위 안에 있던 주기 490).
  HMMWV 시나리오(vehicle_crossing, vehicle_crossing_soft, hmmwv_follow)에는 양보(`yield_to_robot`)도 켰다 (lockstep 결과는 그대로).

| 이 PC 실시간 (shm 두뇌) | Chrono 뒤처짐 최대 | 멈춤 | 차체와 겹친 주기 | 차량 최소 거리 (lockstep) |
|---|---|---|---|---|
| footprints 10초 | 0 s (Chrono가 실시간의 약 4.5배 빠름) | 0 | – | – |
| vehicle_crossing 16초 | 0.22 s | 43번, 7.1 s | 0 | 1.94 m (2.02 m) |
| vehicle_crossing, 상한 없음 | 5.3 s | – | **490** | 1.52 m |
| vehicle_crossing_soft 16초 | 0.22 s | 25번, 4.2 s | 0 | 1.88 m (1.97 m) |

  멈춤은 한 번에 약 0.17 s라 화면이 1초에 2~3번 살짝 멈칫한다. 성능 좋은 PC나 시스템 실시간 설정에서는 멈춤이 줄어든다.
- DIS 콘솔 + Chrono (dis_chrono, dis_footprints)도 이 경로에서 실시간으로 돈다 (`test_dis_chrono_realtime_on_rt_core`: 콘솔로 흙 soft,
  차량 투입, 예약 종료, 뒤처짐 0.3 s 미만). 실시간 Chrono라 `scenario_replay.yaml`을 `sim.runner`로 다시 돌린 결과와는 조금 다르다.
- 맞추다 찾은 것: 발 하나에 접촉점이 여럿이면(패인 발자국) 코어의 발 수직력 덧셈 순서가 Python과 달라 마지막 자리가 달랐다
  (2.6초부터 갈라짐) -> 스텝마다 발별 합을 먼저 내고 누적하게 고쳤다. 마지막 스텝 뒤에 다음 스텝 준비(이벤트, Chrono 교환, 지형)를
  하던 것도 `sim.runner`처럼 하지 않게 했다 (지형 변형 MOP가 달랐다).
- lockstep 같은 결과 (`test_chrono_matches_runner`): footprints, dis_chrono + 흙 soft·차량 투입·제거·LiDAR 지형 인지 보행
  (inproc, shm)에서 `timeseries.parquet` 모든 열, 차량·변형 MOP, 스캔 점이 `sim.runner`와 비트 단위로 같다.

**Python만 실행(`sim.runner`)과 비교** (2026-10-09)

| | Python만 (`sim.runner`) | C++ 실시간 코어 (`sim.rt_link`) |
|---|---|---|
| 몸 (물리, PD 모터, 센서 모델) | Python 루프 안 | C++ 별도 프로세스 (SCHED_FIFO, CPU 고정, 메모리 잠금) |
| 두뇌 (보행 알고리즘) | 같은 루프 안, 또는 ROS2 노드 (`--controller-node`) | inproc / shm 별도 프로세스 / ros 노드 (다른 PC 가능) |
| 관리 (이벤트, 지형, 판정, 기록, 화면, DIS) | 같은 루프 안 | Python 관리 프로세스 |
| LiDAR | 같은 루프 안에서 계산 (또는 `--sensor-node`) | 작업 프로세스 2개 |
| Chrono | 별도 프로세스, 매번 기다림 | 별도 프로세스. lockstep은 기다림, 실시간은 기다리지 않음 |

| | Python `--realtime` | C++ lockstep | C++ 실시간 |
|---|---|---|---|
| 시간 | 벽시계에 맞추려 하지만 늦으면 세계 전체가 같이 늦어짐 | 시뮬레이션 시간 (화면이 있으면 벽시계보다 앞서지 않게만) | 몸은 20 ms마다 반드시 돎 |
| 두뇌가 늦으면 | 몸도 기다림 | 몸도 기다림 | 몸은 마지막 명령 유지, 늦은 만큼 명령 지연 MOP |
| 결과 | 결정적 (lockstep과 같음) | `sim.runner`와 비트 단위로 같음 | 실행마다 조금 다름 (실제 로봇과 같은 성격) |

Python의 "실시간"은 화면 속도만 맞춘 lockstep에 가깝다: 결과는 같지만 로봇이 실제 시간에 쫓기는 상황은 재현하지 못한다.

| 이 PC 실시간 | Python `--realtime` | C++ 실시간 (shm 두뇌) |
|---|---|---|
| flat_trot | 실시간 속도, 늦음 0 (30초 루프 측정: 주기 초과 25번, 깨어남 지연 최대 5.4 ms) | 주기 초과 0, 깨어남 지연 최대 0.74 ms, 명령 지연 항상 20 ms |
| rut_crossing + LiDAR 12초 | 0.8배, 600주기 모두 늦음, 끝에 3.0 s 뒤처짐, 전진 3.708 m (lockstep과 같음) | 실시간 유지, 늦은 명령 0, 스캔 약 95 ms 늦게 도착·120개 중 4~5개 건너뜀, 전진 3.70~3.74 m |
| vehicle_crossing 16초 | 0.76배, 끝에 5.2 s 뒤처짐 (벽시계 약 21 s), 차량 최소 2.02 m | Chrono 뒤처짐 0.22 s 안, 대신 43번 합계 7.1 s 멈춤 (벽시계 약 23 s), 늦은 명령 0, 차량 최소 1.94 m |

HMMWV는 이 PC에서 Chrono가 실시간보다 느려 어느 쪽도 진짜 실시간은 아니다. Python은 계속 조금씩 늦어지고, C++는 도는 동안 몸·두뇌가
20 ms 주기를 지키는 대신 가끔 통째로 멈춘다.

| 기능 | 어디에 |
|---|---|
| `--view` (MuJoCo 뷰어 일시정지·한 스텝), `--start-paused`, `--sensor-node` (센서 ROS2 노드), `--sensor` (C++는 `--set sensors=...`) | Python에만 |
| shm 두뇌, 다른 PC 두뇌 (`--remote-brain`), 실시간 타이밍 MOP (깨어남·계산 시간, 주기 초과, 명령·스캔 지연, Chrono 뒤처짐), LiDAR 프레임 버림, Chrono 뒤처짐 상한 | C++에만 |
| 시나리오 파일, 기록 형식 (`timeseries.parquet`, MOP), RViz 화면, DIS 콘솔, 고장, 지형 변경, `--variant` | 같음 |

| 쓰임 | 권장 |
|---|---|
| 알고리즘 비교, 회귀 시험, 학습 데이터, 재현 | `sim.runner` 또는 `sim.rt_link --lockstep` (같은 결과. 뷰어 일시정지가 필요하면 `sim.runner`) |
| 실제 로봇처럼 시간에 쫓기는 시험 (두뇌·센서·통신 지연, 다른 PC 두뇌, DIS 콘솔 개입) | C++ 실시간 |
| 이 PC에서 HMMWV 결과 비교 | `--lockstep` |

**내부 구조 비교**
```
Python만 (sim.runner)                       C++ 실시간 코어 (sim.rt_link)
┌────────────────────────────┐              ┌──────────────┐  /dev/shm   ┌──────────────────────────────┐
│ 한 프로세스, MjData 1개      │              │ rt/rt_core   │◀──────────▶│ 관리 프로세스 (Python)          │
│  물리 + PD 모터 + 센서 모델  │              │  MjData (진짜)│            │  MjData (거울: 참값을 복사)     │
│  보행 알고리즘              │              │  물리, PD,    │            │  이벤트, 지형, 판정, 기록, 화면, │
│  이벤트, 지형, 판정, 기록    │              │  센서 모델    │            │  DIS, Chrono 교환              │
│  화면, DIS, LiDAR           │              └──────────────┘            └──────────────────────────────┘
└────────────────────────────┘                                              ▲ 함수 / shm / ROS2
                                   두뇌 (inproc: 관리 안, shm: control/rt_brain.py, ros: control/ros_node.py)
                                   LiDAR 작업 프로세스 2개 (sim/rt_sensor.py), Chrono 서버
```
- MuJoCo 데이터: Python은 하나를 모두가 직접 읽고 쓴다. C++는 진짜 물리가 코어에만 있고, 관리 프로세스의 것은 참값을 복사한 거울이다
  (화면, DIS, LiDAR용. 물리 진행 안 함). 기록·판정은 거울에서 다시 계산하지 않고 코어가 스텝 직후 넘긴 값(접촉, 몸통 회전, 발 위치,
  바디 포즈)을 쓴다 (다시 계산하면 접촉과 속도가 조금 달랐다).
- 한 제어 주기(20 ms)의 순서:

| 순서 | Python `Simulation.step_control` | C++ 코어 | C++ 관리 프로세스 (`drain_truth`, 참값 한 개마다) |
|---|---|---|---|
| 1 | DIS 요청, 이벤트 적용 | 멈춤 칸 확인, 시각까지 대기 | 기록 |
| 2 | Chrono 교환 | 지형 공유 메모리 반영 | 발 하중을 Chrono로 |
| 3 | 지형 갱신 (5 Hz) | 센서 측정 → LowState 쓰기 | LiDAR 차례면 작업 프로세스로 |
| 4 | 센서 측정 → 두뇌 → 관절 명령 | 명령 고르기 (조건에 맞는 최신 것) | 다음 스텝 이벤트 적용 |
| 5 | 물리 10스텝 | PD + 물리 10스텝 | Chrono 교환 |
| 6 | 발 하중을 Chrono로 | 참값 링에 쓰기 | 지형 갱신 → 지형 공유 메모리 |
| 7 | LiDAR | | DIS 보고, 화면 |
| 8 | 기록, 판정 | | |

  관리 프로세스는 "스텝 k 결과 처리"와 "k+1 준비"를 함께 하며, Python과 같은 순서라 lockstep이 같다. 두뇌는 몸 루프 밖에서 계산하고,
  실시간에서는 몸이 기다리지 않는다.

| 정보 | Python | C++ |
|---|---|---|
| 로봇 상태 → 두뇌 / 관절 명령 → 몸 | 함수 반환값 / 인자 | shm `state` / `cmd` 칸 (seqlock) |
| 운용자 이동 명령 | `controller.set_command()` | inproc 함수, shm 명령 링(적용 시각), ros `/cmd_vel_stamped` |
| 고장 | `faults` 딕셔너리를 `robot_io`가 읽음 | shm `ctl` 칸 (`torque_scale`, `fault_gyro`/`fault_att`, `link_mode`) |
| 센서 잡음 | 루프 안 난수 | 미리 만든 잡음 링 (`draw_noise`, 같은 값) |
| 지형 | heightfield에 직접 씀 | 지형 shm, 코어가 다음 주기 시작에 복사 |
| 참값 (판정·기록) | 자기 MjData | 참값 링 1024칸 |
| LiDAR 점군 → 두뇌 | 함수 | inproc 함수, shm 스캔 shm, ros 토픽 |
| 상태 추정 → 기록 | 객체 속성 | shm `est` 칸, ros `/controller/estimate` |
| 일시정지 | 루프가 멈춤 | shm `freeze` 칸 (풀리면 코어가 시계를 다시 맞춤) |

- 같은 뜻, 다른 구현:

| 항목 | Python | C++ |
|---|---|---|
| 명령 지연 한 주기 | 명령 큐 (`link_queue`) | 코어가 "직전 상태 시각 이하로 계산된 명령"만 받고, 없으면 마지막 명령 유지 |
| 링크 두절 뒤 | 큐를 비움 | 두절 이후 계산된 명령만 받음 (`min_cmd_t`) |
| 시간 맞추기 | 루프 끝 `time.sleep` | `clock_nanosleep` 절대 시각 + 메모리 잠금·CPU 고정·SCHED_FIFO, 루프 안 할당·입출력·잠금 없음 |
| LiDAR | 루프 안 `sense()` (한 번 98 ms 막음) | 작업 프로세스 (lockstep은 기다리고, 실시간은 도착하는 대로) |
| Chrono | 매 교환 기다림 | lockstep은 기다림, 실시간은 기다리지 않고 0.2 s 넘게 뒤처지면 코어를 멈춤 |
| 발 수직력 합 | 스텝마다 발별 합 → 누적 | 같은 순서로 맞춤 |
- 함께 쓰는 코드: C++ 경로도 `Simulation`을 만들지만 물리는 돌리지 않는다 (모델·지형·설정·기록기용). 두뇌(`ControllerNode`), `Terrain`,
  `Recorder`, DIS 서버, `ChronoLink`, `SensorRenderer`, ROS2 발행, 시나리오 형식이 같은 코드다. C++로 새로 쓴 것은 `rt/rt_core.cpp`의
  몸(물리, PD 모터, 센서 모델, 고장 적용)뿐이다.
- 대가: 상태가 두 곳(진짜와 거울)에 있고, 규약을 C++·Python 양쪽에 맞춰 두어야 하며, 같은 결과를 내려면 순서·계산 방식을 하나씩 맞춰야
  한다 (그렇게 찾은 차이: 발 수직력 덧셈 순서, 마지막 스텝 뒤 처리, 두뇌 종료 경쟁 상태).

**빌드** (mj_ros 환경)
```bash
python -m sim.rt_link --build          # = 아래 두 줄 (실행 파일이 없으면 sim.rt_link가 처음 실행 때 알아서 빌드)
cmake -S rt -B build/rt -DMUJOCO_DIR=$(python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")
cmake --build build/rt                 # rt_core.cpp, shm_layout.h를 고친 뒤에는 이것만
./build/rt/rt_core --layout            # C++ 쪽 공유 메모리 규약. python -m sim.rt_link --layout과 같아야 함
```
- MuJoCo는 mj_ros pip 패키지의 `libmujoco.so`와 헤더에 링크한다 (Python 시뮬레이터와 같은 라이브러리 = 같은 물리).
  `-O2 -ffp-contract=off`: FMA 축약을 막아 numpy와 같은 부동소수점 계산 순서. 필요: cmake 3.16 이상, C++17 컴파일러.
- `rt/shm_layout.h`를 고치면 `rt/shm.py`도 같이 고친다 (`test_layout_matches_cpp`가 대조).

**코드 구성**

| 파일 | 역할 | 줄 수 |
|---|---|---|
| `rt/rt_core.cpp` | 몸 (C++, 유일한 C++ 코드): 20 ms 주기 보장, 물리, PD 모터, 센서 모델, 고장 적용 | 328 |
| `rt/shm_layout.h` | 공유 메모리 규약 (C++ 쪽) | 149 |
| `rt/shm.py` | 같은 규약의 Python 쪽 (ctypes 구조체) | 289 |
| `sim/rt_link.py` | 관리 프로세스: 코어 실행·설정, 이벤트, 지형, 판정, 기록, 화면, DIS, Chrono, 두뇌 연결 | 826 |
| `sim/rt_sensor.py` | LiDAR 작업 프로세스 | 96 |
| `control/rt_brain.py` | shm 두뇌 프로세스 | 63 |

`rt_core.cpp` 안: 시작(모델·공유 메모리 열기, 메모리 잠금, CPU 고정, SCHED_FIFO) → 루프마다 [멈춤 확인·대기 → 0. 지형 반영 →
1. 로봇 상태 측정 (잡음, 편향, IMU 고장) → 2. 실행할 명령 (지연 한 주기, 마지막 명령 유지, 링크 두절) → 3. 물리 (PD, 배터리 저하,
`mj_step` 10번, 에너지, 발 수직력) → 4. 참값·타이밍 기록]. 몸 계산은 Python 몸(`sim/robot_io.py`, `sensors/proprio.py`)을 옮긴 것이라
한쪽을 고치면 다른 쪽도 같이 고쳐야 lockstep 결과가 같다 (어긋나면 `test_rt_core.py` 비트 동일 시험에서 드러남).

**Python과 C++ 코어의 연결**: Python이 C++를 라이브러리로 부르지 않는다 (함수 호출, pybind11 없음). `rt_core`는 독립 실행 파일이고,
`sim/rt_link.py`가 별도 프로세스로 띄운 뒤 공유 메모리(`/dev/shm`)로만 주고받는다. ctypes는 C++ 함수 호출이 아니라 메모리 모양을 맞추는 데만 쓴다.
```
sim/rt_link.py (Python)                                   build/rt/rt_core (C++)
  ① 모델 저장 build/rt/model_<pid>.mjb ───────────────────▶ mj_loadModel
  ② 공유 메모리 생성 (/dev/shm/go2_rt_<pid>, _terrain, _scan),
     설정 칸 채움 (발 geom, 제어 주기, 스텝 수, 토크 한계, 센서 편향, 시작 자세, CPU, 우선순위), 잡음 링 채움
  ③ subprocess.Popen([rt_core, 모델, 이름, 이름_terrain]) ─▶ shm_open + mmap (magic·version·크기가 다르면 종료)
  ④ ctl.start = 1 ─────────────────────────────────────────▶ 루프 시작
     주기마다: state 읽기 ◀── / cmd 쓰기 ──▶ / ctl 쓰기 (고장, 일시정지, lockstep 허락) ──▶ /
               지형 shm 쓰기 ──▶ (다음 주기 반영) / 참값 링 읽기 ◀── (판정, 기록, 화면)
  ⑤ ctl.stop = 1 ──────────────────────────────────────────▶ 루프 끝, stats.done = 1, 종료
     공유 메모리 삭제, 모델 파일 정리
```
- 프로세스를 나눈 이유: Python(GIL, 가비지 컬렉션)의 멈춤이 C++ 루프로 번지지 않게, 코어만 SCHED_FIFO·메모리 잠금·CPU 고정.
  코어는 공유 메모리만 보므로 두뇌가 어디 있든 (inproc / shm / ros) 코어 쪽은 바뀌지 않는다.

**두뇌 연결: 로봇 상태와 관절 명령의 길**
```
inproc:  C++ 코어 ──shm──▶ rt_link (안에서 두뇌 함수 호출) ──shm──▶ C++ 코어
shm:     C++ 코어 ──shm──▶ rt_brain.py ──shm──▶ C++ 코어          (rt_link는 이 길에 없다)
ros:     C++ 코어 ──shm──▶ rt_link ──ROS2──▶ ros_node.py ──ROS2──▶ rt_link ──shm──▶ C++ 코어   (rt_link가 중계 = proxy)
```
`rt_link`는 코어를 띄우고 설정·감독하는 관리자다. 제어 고리의 중계자(proxy)는 ros 방식에서만이고, inproc은 두뇌를 품고, shm은 옆에서 거든다.

| 단계 (shm 방식) | `rt_link` (`ShmBrain`) | `control/rt_brain.py` |
|---|---|---|
| 시작 | `python -m control.rt_brain --shm <이름> --scenario <복사본>` 실행 | 같은 공유 메모리를 엶, 시나리오 controller 절로 `ControllerNode` 생성 |
| 준비 | `ctl.brain_ready`를 기다린 뒤 코어 시작 | 준비되면 `ctl.brain_ready = 1` |
| 주기마다 | 하지 않음 (`on_state`가 비어 있음) | `state` 읽기 → 두뇌 계산 → `cmd` 쓰기 |
| 이동 명령 | 적용 시각을 붙여 명령 링에 1초 앞서 넣음 | 링에서 꺼내 두었다가 그 시각 상태부터 적용 |
| LiDAR | 스캔을 스캔 shm에 씀 | 상태보다 먼저 읽어 두뇌에 넣음 |
| 기록용 | `est` 칸을 읽음 (상태 추정, 가속 제한 뒤 명령) | 명령을 쓰기 직전 `est` 칸에 씀 |
| lockstep | 코어를 한 스텝 진행, `cmd` 시각이 그 상태 시각이 될 때까지 기다림 | 똑같이 동작 (lockstep인지 모름) |
| 끝 | 두뇌 종료를 기다림 (3초 뒤 강제 종료), 로그 `rt_brain.log` | `stats.done`이고 남은 상태가 없으면 끝 |

| 공유 메모리 칸 | 쓰는 쪽 | 읽는 쪽 |
|---|---|---|
| `state` (로봇 상태) | 코어 | 두뇌 |
| `cmd` (관절 명령) | 두뇌 | 코어, `rt_link` (lockstep 대기) |
| `est` (상태 추정) | 두뇌 | `rt_link` |
| 명령 링 (이동 명령) | `rt_link` | 두뇌 |
| 스캔 shm | `rt_link` | 두뇌 |
| `ctl` (고장, 일시정지, lockstep 허락) | `rt_link` | 코어 |
| 참값 링 | 코어 | `rt_link` |

칸마다 쓰는 쪽이 하나라 잠금 없이 seqlock만 쓴다.

**왜 shm 두뇌는 `rt_link`를 거치지 않게 했나**
- `rt_link`는 기록, 화면, DIS, 지형, Chrono 같은 무거운 일을 하는 Python 프로세스라 가끔 밀린다. 상태와 명령이 그 사이를 지나가면
  `rt_link`가 밀릴 때 명령도 같이 늦어진다. inproc에서 실제로 그랬다: LiDAR 실시간 12초 실행에서 늦은 주기가 inproc 28/600, shm 0/600.
  shm 두뇌는 코어와 직접 이어져 있어 `rt_link`가 밀려도 제어 고리에 영향이 없다.
- `rt_brain.py`는 `sim/`를 import하지 않는다. 시뮬레이터 코드 없이 공유 메모리 규약(`rt/shm.py`)과 두뇌 코드(`control/`)만 쓰므로,
  실제 로봇의 상위 제어기와 같은 자리에 있다.

**inproc 방식에도 `rt_brain.py`와 같은 일을 하는 코드가 있나**: 있다. 다만 `rt_brain.py`를 복사한 것이 아니고, 두 쪽 모두 같은 두뇌 객체
`ControllerNode`(`control/node.py`)를 부른다. 다른 것은 그 객체를 부르는 "껍데기"뿐이고, inproc에서는 그 껍데기가 `rt_link`의
`InprocBrain` 클래스와 주 루프에 나뉘어 들어 있다.
- 이것이 가능한 이유: `run()`은 두뇌 방식마다 같은 이름의 메서드(`ready`, `command`, `on_state`, `on_scan`, `poll`, `cmd_t`)를 가진
  클래스(`InprocBrain`, `ShmBrain`, `RosBrain`)를 쓴다. 어느 방식이든 같은 메서드를 부르고, inproc은 두뇌를 직접 부르고, shm은 공유
  메모리에 넣기만 하고 계산은 `rt_brain.py`에 맡기며, ros는 ROS2로 중계한다.

대응 관계:

| 하는 일 | `rt_brain.py` (shm) | `rt_link.py` (inproc) |
|---|---|---|
| 두뇌 만들기 | `ControllerNode(spec, cfg, policy, lidars)` | `Simulation`이 만든 `sim.controller` |
| 새 상태 확인 | 자기 루프에서 `read_state()`, 스텝 번호 비교 | 주 루프에서 같게, 바뀌면 `br.on_state(ls)` |
| 계산 → 명령 | `node.step(ls)` → `view.write_cmd()` | `InprocBrain.on_state`: `node.step(ls)` → `link.write_cmd()` |
| 이동 명령 | 명령 링 → 그 시각 상태에서 `node.set_command()` | 이벤트 적용(`apply_events`) 때 바로 `node.set_command()` (그 시각 상태보다 먼저, `sim.runner`와 같은 순서) |
| LiDAR | 스캔 shm → `node.on_scan()` | 스캔 받는 곳(`deliver_scans`)에서 바로 `node.on_scan()` |
| 상태 추정 | `est` 칸에 씀 | 필요 없음 (기록 때 `node.est`를 직접 읽음) |
| 준비 신호 | `brain_ready = 1` | 필요 없음 |
| lockstep 대기 | 두뇌는 모름 | 같은 프로세스라 `on_state`가 끝나면 명령이 이미 있음 |

`rt_brain.py`는 두뇌만 도는 전용 루프이고, inproc은 그 루프를 `rt_link` 주 루프(기록, LiDAR 전달, 화면, DIS 사이사이)에 끼워 넣은 형태다.
그래서 inproc은 관리 작업이 밀리면 명령도 늦어진다. 결과는 lockstep 세 방식이 비트 단위로 같다.

## 로봇 경계 ROS2 메시지 (`go2_rt_msgs`, 2026-10-10)

두뇌(상위 제어기)를 ROS2 노드로 돌릴 때(`sim.runner --controller-node`, `sim.rt_link --brain ros`) 주고받는 세 토픽의 형식이다.

| 토픽 | 방향 | 커스텀 메시지 (typed) | JSON (처음 방식) |
|---|---|---|---|
| `/robot/low_state` | 몸 -> 두뇌 | `go2_rt_msgs/LowState` | `std_msgs/String` |
| `/robot/low_cmd` | 두뇌 -> 몸 | `go2_rt_msgs/LowCmd` | `std_msgs/String` |
| `/control/estimate` | 두뇌 -> 판정자 (진단) | `go2_rt_msgs/Estimate` | `std_msgs/String` |

나머지(`/cmd_vel_stamped` TwistStamped, `/sensors/<이름>/points` PointCloud2, `/robot/imu` Imu, `/robot/joint_states` JointState)는 처음부터 ROS2 표준 메시지다.

```bash
python -m control.robot_msgs --build      # 메시지 패키지 빌드 (ros2_ws/src/go2_rt_msgs, colcon + 시스템 ROS2 Humble). 한 번만
python -m control.robot_msgs              # 지금 쓰일 형식 확인
python -m sim.rt_link scenarios/flat_trot.yaml --brain ros                 # 기본 auto: 빌드돼 있으면 typed, 아니면 json
python -m sim.rt_link scenarios/flat_trot.yaml --brain ros --ros-msg json  # 처음 방식
python -m control.ros_node --ros-msg typed --scenario <시나리오>           # 다른 PC의 두뇌: 시뮬레이터와 같은 형식으로
source ros2_ws/install/setup.bash && ros2 topic echo /robot/low_cmd       # ROS2 도구로 필드별 보기 (도구 쪽만 source 필요)
```

**왜 바꿨나** (속도가 아니라 형식): 이 PC에서 JSON 변환은 로봇 상태 899바이트·만들기 23 µs·읽기 20 µs, 관절 명령 389바이트·10/9 µs로
20 ms 주기에 비해 작고, Python JSON은 실수를 손실 없이 주고받아 lockstep도 비트 단위로 같았다. 문제는 JSON 안의 필드 구성이 코드 속
암묵적 약속이라 ROS2에는 "문자열 하나"로만 보인다는 점이다 (사실상 비공식 커스텀 메시지). 정식 메시지로 바꾸면:
- 형식 검사: 필드 이름·배열 길이(`float64[12]`)가 틀리면 보낼 때 오류 (JSON은 받는 쪽에서 실행 중 오류).
- 도구: `ros2 topic echo`, `ros2 bag`이 필드별로 다룬다.
- 다른 언어 두뇌: C++/Python 코드가 자동 생성된다 (JSON 해석 코드 불필요).
- ROS2 표준 메시지만으로는 부족하다: 관절 각도·속도(JointState), IMU(Imu)는 있지만 저수준 모터 명령(kp, kd, 앞먹임 토크)과
  "어느 상태 시각으로 계산한 명령인가"(`t`, 지연 측정과 lockstep 판정의 기준)를 담는 표준이 없다.

**커스텀 메시지 vs Unitree 메시지** (결정: 안쪽은 커스텀, Unitree는 경계 어댑터로)

| 기준 | 커스텀 (`go2_rt_msgs`) | Unitree (`unitree_go/LowState`, `LowCmd`) |
|---|---|---|
| 벤더 중립 목표 | 맞음 | Go2 전용 (다른 로봇이면 변환 필요) |
| 지연 측정용 상태 시각 `t` | 있음 | 없음 |
| 시뮬레이터에 없는 정보 | 필요한 것만 | 모터 온도, 배터리, 리모컨, CRC, 모터 20칸 등을 채워야 함 |
| 실제 Go2 소프트웨어 연결 | 어댑터 필요 | 그대로 붙음 (Unitree SDK2 제어기를 시뮬레이터에) |
| 형식 변경 주도권 | 우리 | Unitree |
```
우리 두뇌 (control/) ──go2_rt_msgs──▶ 시뮬레이터 (sim.rt_link / sim.runner)
                                            ▲
Go2용 외부 제어기 (Unitree SDK2) ──Unitree 메시지──▶ Unitree 어댑터 노드 (나중, 필요할 때) ──go2_rt_msgs──┘
```
Unitree 어댑터는 실제 Go2용 외부 제어기를 시험할 일이 생기면 만든다 (그때 Unitree 메시지 정의를 직접 확인한다).

**구성**
- `ros2_ws/src/go2_rt_msgs/msg/{LowState,LowCmd,Estimate}.msg`: 정의. 필드는 JSON 때와 같고 실수는 8바이트 그대로라 값이 같다.
  `LowState.step`은 실시간 코어 경로의 주기 번호(없으면 -1). 필드는 추가만 하고 지우거나 뜻을 바꾸지 않는다.
- `control/robot_msgs.py`: 형식 선택(`GO2_ROS_MSG` 환경변수 = `--ros-msg`)과 dict <-> 메시지 변환. 시뮬레이터 쪽(`sim/ros2_bridge.py`)과
  두뇌 노드(`control/ros_node.py`)가 같은 코드를 쓴다. setup.bash를 source하지 않아도 저장소의 `ros2_ws/install`에서 찾아 쓰고
  (라이브러리를 미리 불러옴), 띄우는 두뇌 노드는 환경변수를 물려받는다.
- 두 프로세스의 형식이 다르면 (예: 다른 PC 두뇌를 `--ros-msg json`으로 띄움) `sim.rt_link`가 기다리지 않고 바로 알려 준다.
- 빌드 결과(`ros2_ws/build`, `install`, `log`)는 git에 넣지 않는다.

**배포 (다른 PC의 두뇌)**: 양쪽이 같은 메시지 정의를 가져야 한다. 지금은 저장소에 포함해 두뇌 PC에서 `git clone` 후
`python -m control.robot_msgs --build` (같은 ROS2 배포판 Humble 권장). 다른 팀이 두뇌만 만들게 되면 `go2_rt_msgs`만 별도 저장소로,
여러 PC 설치 관리가 필요해지면 deb 패키지(`bloom`)로.

**일반 터미널에서 확인** (모든 터미널에서 같은 `ROS_DOMAIN_ID`, mj_ros 환경, `~/physics_ai_test`에서)
```bash
# 1. 형식: "로봇 경계 메시지 형식: typed"면 커스텀 메시지가 빌드돼 있음
python -m control.robot_msgs

# 2. ros 두뇌로 실행 (터미널 1). 두뇌 노드 로그 runs/<실행>/controller_node.log 끝에 "robot messages: typed"
export ROS_DOMAIN_ID=77
python -m sim.rt_link scenarios/flat_trot.yaml --brain ros --rviz

# 3. ROS2 도구로 보기 (터미널 2, 2가 도는 동안)
export ROS_DOMAIN_ID=77
source /opt/ros/humble/setup.bash
source ~/physics_ai_test/ros2_ws/install/setup.bash        # 도구가 go2_rt_msgs를 알게 (시뮬레이터·두뇌 노드는 source 불필요)
ros2 topic list -t | grep -E "low_state|low_cmd|estimate"  # go2_rt_msgs/msg/... (json이면 std_msgs/msg/String)
ros2 topic hz /robot/low_state                             # 약 50 Hz
ros2 topic echo /robot/low_cmd --once                      # 필드별로 보임 (json이면 JSON 문자열 한 줄)
ros2 interface show go2_rt_msgs/msg/LowState

# 4. JSON 방식과 비교: 로봇 움직임은 같고, 3의 토픽 타입이 std_msgs/msg/String으로 바뀜
python -m sim.rt_link scenarios/flat_trot.yaml --brain ros --ros-msg json --rviz

# 5. 두뇌를 따로 띄우기 (다른 PC 흉내). 터미널 1: 두뇌 노드, 터미널 2: 시뮬레이터 (노드를 기다림)
python -m control.ros_node                          # 시나리오 설정·메시지 형식은 시뮬레이터에서 받는다 (/robot/brain_config)
python -m sim.rt_link scenarios/flat_trot.yaml --brain ros --remote-brain --ros-msg typed --rviz
#    파일로 주려면: python -m control.ros_node --ros-msg typed --scenario scenarios/flat_trot.yaml
#    두뇌 노드를 --ros-msg json으로 띄우면 시뮬레이터가 바로 "두뇌 노드의 메시지 형식이 다르다 ... --ros-msg typed로 띄울 것"
#    (이때 빈 실행 폴더 runs/flat_trot_rt_<시각>가 생길 수 있다). 노드는 Ctrl+C로 끝낸다
```
- 옵션은 `--scenario`처럼 `--`를 붙인다 (`scenario`만 쓰면 usage 오류).
- 3에서 `ros2_ws/install/setup.bash`를 source하지 않으면 도구가 타입을 몰라 `echo`가 실패한다.

**5의 두 터미널 = 상위 제어기와 로봇 쪽.** 터미널 1은 상위 제어기만이고, 터미널 2는 하위 제어기에 로봇 몸과 시험장까지 들어 있다.

| 터미널 | 실행 | 역할 | 실제 로봇이라면 |
|---|---|---|---|
| 1 | `control.ros_node` | **상위 제어기**(두뇌): 로봇 상태를 받아 보행 알고리즘으로 관절 명령(목표 각도, kp, kd, 앞먹임 토크)을 계산 | 로봇 위나 옆 제어 컴퓨터의 보행 소프트웨어 |
| 2 | `sim.rt_link` -> C++ 코어 `rt_core` | **하위 제어기**: 20 ms마다 명령을 받아 PD 모터 제어, 링크 두절·배터리 같은 안전 동작 | 로봇 안의 저수준 제어 보드와 모터 드라이버 |
| | (같은 C++ 코어 안) | 로봇 몸: 물리, 센서 모델(잡음, 편향, 고장) | 로봇 몸체, 모터, IMU, 발 센서 |
| | `sim.rt_link` 관리 프로세스 | 시험 환경: 이벤트, 지형, Chrono, LiDAR 계산, 넘어짐 판정, 기록, RViz, DIS 콘솔, ROS2 중계 | 시험장과 계측 장비 |

- 두 터미널 사이에는 실제 로봇과 같은 경계의 정보만 오간다. 2 -> 1: 로봇 상태 `/robot/low_state`, LiDAR 점군, 이동 명령.
  1 -> 2: 관절 명령 `/robot/low_cmd`. 터미널 1의 코드는 시뮬레이터(`sim/`)를 모른다.
- 그래서 터미널 1을 다른 PC로 옮기면 상위 제어기를 별도 컴퓨터에 둔 실제 로봇 구성과 같은 형태가 된다.
- 다른 두뇌 방식: `--brain shm`은 상위 제어기가 같은 PC의 별도 프로세스(`control/rt_brain.py`)이고 시뮬레이터가 자동으로 띄운다.
  `--brain inproc`은 상위 제어기가 관리 프로세스 안에 들어가 터미널 하나로 돈다.

**확인**: `test_ros_equivalence.py` 4개(트롯, 정책, 추정 정책, 지형 인지 트롯)가 typed와 json 모두 통과. 실시간 코어 경로의 ros 두뇌
(lockstep 비트 동일, 고장, LiDAR 지형 인지)도 typed로 통과.

## 앞으로 할 일

**무른 흙과 로봇 발의 쌍방향 상호작용** (2026-10-10 정함, 2026-10-11 1~3단계 구현: 위 "무른 흙과 로봇 발 (`foot_soil`)")
- 방법은 A(MuJoCo 안에서 흙 식)로 했다. B(Chrono 발 대리 구 쌍방향)는 두 엔진이 1~2 ms마다 주고받아 이 PC에서 너무 느리고,
  C(로봇 전체를 Chrono로)는 시험 환경 대부분을 다시 맞춰야 한다. SCM은 1단계 기준값을 만드는 데만 썼다.
- 쟁기질(흙 속을 옆이나 위로 지나가는 발의 저항)도 넣었다 (2026-10-11).
- 남은 것: 쟁기질 비율 맞추기(실측 또는 다른 기준), 탄성 되튐·Janosi 미끄럼.
  발자국 화면 표시, C++ 실시간 코어(4단계), 지역마다 다른 흙, DIS 콘솔 `soil`로 로봇 발 흙 바꾸기(2026-10-17)는 했다.

**맵 서버와 SCM 다짐 전달** (2026-10-24 정리. 설계와 시험 결과는 [`docs/map_server.md`](docs/map_server.md) 5.5, 9절)
- 지금까지: 맵 서버 설계 v0.4.7 (타일, 공유 영역 파임은 min, 운용자 편집은 순서, 오르내리는 칸은 한 엔진이 소유).
  SCM 시험 다섯 가지 (단일 SCM과 비교): 높이 넣기, 위치마다 흙 성질 콜백, 단단한 흙 콜백, 가상 다짐(누르기), 구르는 구.
  결론: 높이만 넘기면 다짐 이력이 없어 새 흙처럼 가라앉는다. 넘겨받은 쪽이 같은 하중으로 한 번 눌러(굴려) 두는 **가상 다짐**이
  기본 후보다. 시험 기록 하나에서 분석 그래프(`conformance/plot_scm_probe.py`)와 Godot 재생(`conformance/replay_scm_probe.py`)이 나온다.
- 다음 (권하는 순서):
  1. ~~가상 다짐의 높이 보정 계산식~~ (2026-10-25 풀림): 앞쪽 SCM의 노드 `sinkage`만큼 올려 넣으면(= 자국 전 높이) 되풀이 없이
     단일 SCM과 같아진다 (누르기·굴리기 모두, `docs/map_server.md` 5.5 "높이 보정 = 앞쪽 SCM의 sinkage"). 넘길 것: 지금 높이, sinkage,
     자국을 만든 하중·물체·경로.
  2. ~~실제 바퀴로 가상 다짐~~ (2026-10-25, 단단한 원통 구동 바퀴로 함): 이력을 알면 바퀴로도 단일 SCM과 같다. 넘길 이력에
     **미끄럼률**이 있어야 견인력이 맞는다 (없으면 33 ~ 35 % 작음). 여러 번 지나간 자리는 순서대로 되풀이하면 같다
     (`docs/map_server.md` 5.5 "구동 바퀴 시험"). 남은 것: 타이어 변형·트레드, HMMWV 바퀴 자국을 실제로 넘겨받기.
  3. ~~맵 서버 1단계 구현~~ (2026-10-25, `sim/map_server.py`): 같은 프로세스 안 모드. 시나리오 13개 모두 이전과 비트 단위로 같다
     (설계 문서 8절 "1단계 결과"). 찾은 것: Chrono는 시나리오의 SCM 패치 밖도 변형한다 (소유 영역을 잡는 규칙이 필요).
     2a (2026-10-25): 별도 프로세스 서버 `python -m sim.map_server_node`, `sim.runner`/`sim.rt_link --map-server`. 시나리오 13개 비트 동일,
     서버와 사본 해시 불일치 0.
     2b (2026-10-25): 늦게 붙기 (스냅샷 + 그 뒤 변경), 관찰자 `python -m sim.map_client --observe` (다른 터미널에서 실행 중에 붙음),
     MOP `map_hash_mismatch`, `map_recv_latency_p95_ms`, `map_apply_latency_p95_ms`.
     2c (2026-10-25): 바탕 파일 `python -m sim.map_base build|verify` (타일 + 매니페스트 해시), 서버 `--base`, init은 타일 해시만
     (1.7 MB -> 874 바이트), `--map-base <폴더>` (미리 만든 바탕, 시작 지형과 다르면 시작하지 않음).
     2d (2026-10-25): `seq` 빈칸 다시 보내기·다시 받기 (10% 일부러 버려도 같아짐), 스냅샷으로 다시 맞추기, 서버 `--save-dir`·`--resume`,
     시뮬레이터 `--map-save` (끝난 지도를 실행 폴더 `map_state/`에), MOP `map_seq_gaps`, `map_gap_delay_max_ms`, `map_lost_changes`. 다시 받을 수 없는 빈칸은 서버가 5 s 뒤 건너뛴다 (늦게 온 파임은 적용,
     소유 영역은 통째로 다시 받음). 2단계 끝.
     3a (2026-10-26): 운용자 편집 `python -m sim.map_edit bump|ramp|rough|rut|set ...` (실행 중인 `--map-server` 시뮬레이터에 다른
     터미널에서). 받은 편집은 `scenario_replay.yaml`에 받은 시각으로 적혀 다시 돌리면 비트 동일. 엇갈린 편집은 서버 번호로 바꿔 맞춘다
     (rebase: 버리지 않음).
     3b (2026-10-26): Godot 렌더러가 지형을 맵 서버에서 받는다 (`--map-server --godot`, 또는 시뮬레이터를 `--map-server --ros2`로
     띄우고 나중에 `python -m viz.godot_launch`: 맵 서버가 있으면 스스로 거기서 받는다). 실행 중 언제 붙어도 같은 지형. 시뮬레이터 스트림이 없으면 Godot 정보판에
     WAITING FOR SIMULATOR.
     3c (2026-10-26): 맵 서버 TCP 입구 (127.0.0.1, ROS2 없는 클라이언트용). `--map-server`면 Chrono 프로세스(Python 3.12)가 SCM 영역의
     소유자로 맵 서버에 직접 쓰고, 시뮬레이터는 교환 시각에 받아 넣는다 (2단계와 비트 동일). 3단계 끝.
     **다음에 할 것** (2026-10-26 기준, 권하는 순서):
     1. **다른 쪽 변경을 Chrono의 SCM에 넣기** (설계 9절 "소유 영역에 넘어온 변경"): 지금 Chrono 사본은 운용자 편집·다른 차량 자국을 받지만
        SCM은 모른다. 그래서 SCM 영역에 넣은 편집을 다음 Chrono 쓰기가 덮는다. 높이 넣기(`SetModifiedNodes`)는 확인함 (설계 5.5 시험 2).
        남은 것은 다짐 상태를 함께 넘기기: 가상 다짐 (앞쪽 SCM 노드 `sinkage`만큼 올려 넣고 같은 하중·물체·경로·미끄럼률로 다시 누름,
        설계 5.5)과 그에 필요한 자국 이력 저장 (sinkage, 물체, 하중, 미끄럼률).
     2. **4단계: 물리 프로세스 여럿** (설계 8절 4단계): 예 HMMWV 두 대를 서로 다른 Chrono 프로세스로, 뒤 차량이 앞 차량 자국을 밟는다
        (MOP: 자국 위 침하, 자세, 단일 SCM 대비 침하 차이). 소유권 이양. 렌더러의 시뮬레이터별 이름공간과 장면 합치기 (설계 8절 3b 끝:
        지금 `/sim/scene_manifest`, `/tf`, `/sim/status`는 시뮬레이터 하나 전제).
     3. 아이디어로 남긴 것:
        - 위치 조건으로 재현하는 운용자 편집: 기록할 때 그 순간 로봇 위치를 같이 적고, 재현할 때 "로봇이 그 지점에서 몇 m 안에 오면
          넣는다"로 (지금은 받은 시각에 넣어 비트 동일, 대신 로봇 쪽을 바꾸면 편집이 의도와 다른 자리에 생길 수 있다).
        - 지도가 커지면: 스냅샷을 바뀐 타일만, 관심 영역 타일만 받기, 큰 응답을 나눠 보내기, 서버의 바탕 높이를 타일로 (설계 9절 첫 항목,
          지금 401 × 401에서 스냅샷 2.6 MB, 2001 × 2001에서 43.6 MB).
        - 서버 상태 주기 저장 (지금은 끝날 때와 요청할 때만), RViz 어댑터도 맵 클라이언트로, MuJoCo 안 무른 흙 발자국을 지도에
          (`soil_holes` 층, 5단계).
  4. **타당성 확인**: 지금 결과는 모두 단일 SCM과의 일관성이다. 실제 흙과 비교하려면 판 재하 시험이나 반복 주행 시험의 실측 자료와
     그에 맞춘 흙 매개변수가 필요하다. 자료를 구하는 것부터 정한다.
- 급하지 않은 것: Godot에 한글 글꼴 넣기 (지금 화면 글자는 영어와 숫자만), 다른 PC 두뇌 + 지형 인지 실시간 실행의 결과 흔들림
  원인 조사 (위 실시간 코어 절), shm 두뇌에서도 지형 지도와 디딜 곳을 화면에 내보내기 (위 Godot 절).

## 검토 중인 아이디어 (결정 전)

(지금 없음. lockstep의 통계적 명령 지연은 2026-10-11 구현: 위 "통계적 명령 지연")

## 저사양을 고려해 의도적으로 뺀 것

- **UE5 / 카메라 센서**: GPU 드라이버가 없어 불가 (LiDAR는 CPU 레이캐스트로 구현). 렌더러는 포즈 스트림을 받는 얇은 클라이언트로 나중에 붙인다. 연동 계획과 추천 PC 제원은 [`docs/ue5_integration.md`](docs/ue5_integration.md).
- **MJX / 강화학습**: GPU 없이는 학습 처리량이 나오지 않음. 학습은 다른 장비에서 하고 ONNX 정책만 가져오는 구조로 간다.
- 그림자 끔, 뷰어 갱신 30 Hz 제한, 지형 20 m × 20 m @ 5 cm.

## 알려진 한계

- 트롯 보행기는 규칙 기반이라 요철에서 방위가 최대 약 18° 흔들리고 측방 이동이 남는다 (측방 위치 제어 없음).
- 지형 인지 트롯도 자국 안을 따라 걷기(hmmwv_follow)와 15 cm 자국에서는 자주 넘어진다 (위 지형 인지 보행 절).
- `sim.runner`의 실시간 루프는 Python이라 지터 보장이 없다. C++ 실시간 코어(`sim.rt_link`)도 시간 보장에는 시스템 설정이 더 필요하고, 이 PC에서는 실시간 HMMWV(Chrono)가 느려 세계를 자주 잠깐 멈춘다 (위 실시간 코어 절).
- 기본(단단한 지형)에서는 흙의 힘이 로봇 발에 돌아오지 않는다. 선택 기능 `foot_soil`을 켜면 침하·마찰·에너지·쟁기질 저항이 돌아온다. 무른 흙 결과의 절댓값은 쟁기질 비율에 민감하다 (위 `foot_soil` 절).
- 지형 갱신은 발 근처 셀을 보류하므로 로봇이 홈 위에 서 있으면 반영이 늦어진다 (의도된 동작).
- SCM 다짐 전달 시험(`conformance/probe_scm_*.py`)의 기준은 단일 SCM 결과다. 실측과 비교하지 않았고, 흙 매개변수(kphi 5e6 등)도
  특정 흙에 맞춘 값이 아니다 (`docs/map_server.md` 5.5 "비교 기준은 실측값이 아니다").
- Godot 화면 글자는 영어와 숫자만 쓴다. 한글을 쓰면 대체 글꼴을 찾느라 화면이 거의 멈춘다 (위 Godot 절).
