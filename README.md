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
#   RViz를 자동으로 띄우고 준비되면 시작한다. 끝난 뒤 RViz 창을 닫으면 종료, Ctrl+C로 중단해도 기록은 저장된다.
#   로봇 위 정보판(시각, 명령, 속도, 거리, CoT, 발 접촉, 이벤트, 지연 -> 종료 시 최종 MOP),
#   발 접촉(초록=접지), 명령 화살표(파랑), 경로(노랑). 같은 정보가 /sim/status(JSON)로도 나온다.
#   수동으로 띄우려면: --realtime --ros2 로 실행하고, python -m viz.rviz_adapter 를 따로 실행한 뒤,
#   conda를 끈 터미널에서 rviz2 -d config/go2.rviz --ros-args -p use_sim_time:=true
python -m sim.runner scenarios/rough_rut.yaml --policy policies/go2_trot_bc/card.yaml --view   # ONNX 정책
python train/imitate_trot.py                             # 모방학습 정책 재생성 (CPU, 약 3분)
MUJOCO_GL=egl python -m sim.snapshot scenarios/rough_rut.yaml --t 5   # 오프스크린 스냅샷
python -m sim.runner scenarios/rough_rut.yaml --variant mjx   # MJX용 물리 설정 모델을 CPU에서 실행
python conformance/test_conformance.py                   # 결정성, IK-모델 일치
python conformance/test_specs.py                         # 관측 기준 벡터, 카드 관절 재배열, MJX 오버라이드 검증
python conformance/test_policy.py                        # 정책 카드/ONNX 입출력/결정성/추론 지연
python conformance/test_render_stream.py                 # 렌더 스트림 계약 (스트림만으로 장면 재구성 = 시뮬레이터)
python conformance/compare_variants.py [--policy card.yaml]   # cpu vs mjx 설정의 MOP 분포 비교
python conformance/make_obs_reference.py                 # (관측 명세를 바꾼 경우) 기준 벡터 재생성
python conformance/make_policy_reference.py <card.yaml>  # (ONNX를 바꾼 경우) 정책 입출력 기준 재생성
```

결과는 `runs/<시나리오>_<시각>/`에 `timeseries.parquet`(시계열, 메타데이터에 버전 조합)과 `summary.json`(MOP)으로 남는다.

## 구성과 문서 대응

| 경로 | 역할 | 문서 |
|---|---|---|
| `specs/go2_control.yaml` | 관절 순서, 기본 자세, 행동 처리 경로(scale/clip/PD/토크 한계), 관측 정의·스케일·허용 오차 | §12.2 |
| `specs/go2_mjx_override.yaml` | MJX용 모델 차이 목록. 받아들인 항목(adopt)과 거부한 항목(reject)을 이유와 함께 기록 | §12.2 |
| `sim/observation.py` | 관측 기준 구현 (numpy, qpos/qvel만 사용). JAX·C++ 구현이 맞춰야 할 기준 | §12.2 |
| `scenarios/*.yaml` | 지형 패치, 시간 이벤트(명령 변경, 지형 변형) | §10 (DIS 콘솔 이전 단계) |
| `sim/terrain_service.py` | Terrain Map Service: 절대 높이 원천, heightfield 사전 할당, 발 근처 갱신 보류, 5 Hz 반영 | §5.1, §5.3 |
| `sim/model_builder.py` | Menagerie Go2 MJCF + heightfield 결합 (MjSpec) | §4 |
| `sim/controllers/trot.py` | 규칙 기반 트롯 (IK + 속도 PI + 방위 유지 + 자세 보정). 학습 정책 전 기준 컨트롤러 | – |
| `sim/controllers/onnx_policy.py` | ONNX 정책 실행기 (onnxruntime CPU, 단일 스레드) | – |
| `sim/control_interface.py` | 제어 인터페이스 = 로봇 명세 + 정책 카드. 관절 순서·기본 자세·스케일·PD·제어 주기·관측 규약 차이를 카드로 흡수 | §8 |
| `sim/controllers/common.py` | 명령 가속 제한과 보행 위상 시계 (모든 컨트롤러 공용) | – |
| `policies/go2_trot_bc/` | 트롯 모방 정책: `policy.onnx`, `card.yaml`(규약+출처), `train_log.json`, `io_reference.npz` | – |
| `train/imitate_trot.py` | DAgger 모방학습 (numpy MLP, ONNX 그래프 직접 구성, PyTorch 불필요) | §12 |
| `sim/runner.py` | 시나리오 실행, lockstep/실시간 두 모드, 넘어짐 판정, MOP 계산 | §9.2, §13 |
| `sim/adapters.py` | 좌표계 변환 단일 지점 (MuJoCo / ROS2 REP-103 / UE5) | §6.3, §8 |
| `sim/stream.py` | 중립 렌더 스트림 인코더: 장면 매니페스트, 지형 패치, 상태. 그리는 방법은 모름 | §8 |
| `sim/ros2_bridge.py` | 중립 스트림의 ROS2 전송: `/tf`, `/clock`, `/joint_states`, `/sim/base_twist`, `/sim/scene_manifest`, `/sim/terrain_patch`, `/sim/status` | §6.3 |
| `viz/` | 렌더러 쪽 (물리엔진 import 금지). `stream_decode.py` 공용 디코더, `rviz_adapter.py` 중립 스트림 -> RViz 표현 | §6, §8 |
| `config/go2.rviz` | RViz 설정 (로봇, 지형, 정보, 카메라가 몸통 추적). 시뮬레이션 시각을 쓰므로 `use_sim_time:=true` 필요 | – |
| `docs/render_interface.md` | 렌더 인터페이스 계약 (메시지, 규약, 어댑터 추가 방법, 계약 시험) | §8 |
| `sim/chrono_link.py` | Chrono 연동 (러너 쪽): 프로세스 실행, 격자 정합 검사, 파이프라인 동기, 지형/차량 반영, 근접 판정 | §5 |
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

## Chrono 연동 (두 번째 물리엔진, 문서 §5)

```bash
python -m sim.runner scenarios/vehicle_crossing.yaml --rviz        # 단단한 흙: 자국 약 5.5 cm
python -m sim.runner scenarios/vehicle_crossing_soft.yaml --rviz   # 무른 흙: 자국 약 15 cm
python -m sim.runner scenarios/hmmwv_follow.yaml --rviz            # HMMWV 뒤를 따라 왼발이 바퀴 자국 안을 걷기
python conformance/test_cosim.py                                   # 연동 시험 (약 40초)
```

- **역할 분담**: Chrono = HMMWV 차량 + SCM 변형 지면, MuJoCo = Go2. **단방향 연결**이라 차량과 로봇 사이 물리 작용은 없고,
  근접은 논리 이벤트(`vehicle_near`, MOP `min_vehicle_distance_m`)로만 판정한다 (문서 §5.4).
- **프로세스**: Chrono는 `chrono` conda 환경(Python 3.12)에서 별도 프로세스로 돈다 (`cosim/chrono_server.py`).
  러너가 자동으로 띄우며, socketpair + 길이 접두 JSON(`cosim/wire.py`)으로 연결한다. python 경로는 `CHRONO_PYTHON`으로 바꿀 수 있다.
- **동기**: 0.04 s마다 차량 포즈, 0.2 s(5 Hz)마다 지형 변경분을 교환. 파이프라인 방식이라 두 엔진이 같은 구간을 동시에 계산하며 결과는 결정적이다.
- **지형**: SCM 격자 = MuJoCo 지형 격자 (해상도 같고 SCM 중심이 격자점). 바퀴 자국은 Terrain Map Service를 거쳐 MuJoCo로 들어가며,
  발 근처 셀 보류도 그대로 적용된다. SCM 영역의 기본 지형은 평평해야 한다 (시작 시 검사).
- **가시화**: 차량 바디는 중립 스트림에 `physics_source: Chrono`로 들어가므로 RViz 어댑터 수정 없이 함께 그려진다. MuJoCo 뷰어(`--view`)에는 차량이 보이지 않는다.
- **이 PC 기준 성능**: 실시간의 약 0.8배 (Chrono가 상한). `--rviz`에서는 화면이 실제 시간보다 느리게 흐르고 `late_control_steps`가 크게 나온다. 결과(결정성)에는 영향 없음.
- **결과 예 (`vehicle_crossing`)**: 규칙 기반 트롯은 5.5 cm 자국에서 뒷다리가 걸려 x≈2.2 m에서 멈춘다 (넘어지지는 않음).
  무른 흙(15 cm)에서는 첫 자국에서 넘어진다. 지형을 보지 못하는 보행기의 한계를 보여주는 시험 결과다.
- **결과 예 (`hmmwv_follow`)**: 같은 평지에서 HMMWV 유무만 바꿔 비교 (20초). 자국 때문에 전진 거리 6.20 -> 4.09 m,
  CoT 2.54 -> 6.08, 평균 몸통 롤 0.9° -> 4.7°. 넘어지지는 않음.
- **새 시나리오에 HMMWV 넣기**: 코드 변경 없이 시나리오에 `chrono:` 절만 추가한다. 조건(시작 시 검사): SCM 해상도 = 지형 해상도,
  SCM 중심이 지형 격자점, SCM 영역의 기본 지형이 평평, HMMWV 경로가 지형 범위 안.

## ONNX 정책 탑재

정책 = `policy.onnx` + `card.yaml`. 실행: `--policy <card.yaml>`. 카드에 없는 항목은 로봇 명세를 따른다.

| 외부 정책의 상황 | 필요한 작업 |
|---|---|
| 우리 명세대로 학습 | ONNX 교체 (카드는 복사) |
| 관측 순서/스케일, 관절 순서, PD, 기본 자세, 제어 주기가 다름 | 카드만 작성 (코드 수정 없음, `test_card_joint_reordering`으로 검증) |
| 없는 관측 항목 (높이맵, 관측 이력, 순환 신경망 상태 등) | `sim/observation.py`의 `TERMS`에 함수 등록 + `Simulation.obs_inputs()`에 입력 추가 |

어느 경우든 탑재 후 `test_policy.py`(카드/ONNX 정합)와 `compare_variants.py --policy`(MOP)로 확인한다.

**현재 정책 `go2_trot_bc`**: 트롯 보행기를 DAgger로 모방 (11만 샘플, CPU 3분). 성능 향상이 아니라 탑재 경로 시험용이다.
- 평지·요철 모두 넘어지지 않고 전진 거리는 트롯과 비슷하다 (4.11 / 4.49 m vs 3.90 / 4.48 m).
- 관측에 방위 정보가 없어 트롯의 방위 유지를 흉내 내지 못한다: 직진 중 6초에 약 23° 틀어지고 yaw rate는 명령의 절반 정도만 따른다.
- 추론 지연 p99 약 18 us (제어 주기 20 ms).
- MJX 설정 모델에서는 트롯(전진 +0.09 m)보다 영향이 크다: 전진 -0.74 m, CoT +30% (시드 8개, 약 1.8σ).
  학습한 모델 설정에 맞춰진 정책일수록 설정 차이에 민감하다는 것이 §13.2 계층 비교를 정책마다 다시 해야 하는 이유다.

## 저사양을 고려해 의도적으로 뺀 것

- **UE5 / 센서 시뮬레이션**: GPU 드라이버가 없어 불가. 렌더러는 포즈 스트림을 받는 얇은 클라이언트로 나중에 붙인다. 연동 계획과 추천 PC 제원은 [`docs/ue5_integration.md`](docs/ue5_integration.md).
- **MJX / 강화학습**: GPU 없이는 학습 처리량이 나오지 않음. 학습은 다른 장비에서 하고 ONNX 정책만 가져오는 구조로 간다.
- 그림자 끔, 뷰어 갱신 30 Hz 제한, 지형 20 m × 20 m @ 5 cm.

## 알려진 한계

- 트롯 보행기는 규칙 기반이라 요철에서 방위가 최대 약 18° 흔들리고 측방 이동이 남는다 (측방 스텝 없음).
- 실시간 루프가 Python이므로 지터 보장은 없다. 문서 §11대로 RT 루프는 추후 C++로 옮긴다.
- 지형 갱신은 발 근처 셀을 보류하므로 로봇이 홈 위에 서 있으면 반영이 늦어진다 (의도된 동작).
