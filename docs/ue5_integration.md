# UE5 가시화 연동 계획 (보류)

> 상태: **계획만 기록**. 현재 PC(i5-9400, RAM 7.7 GB, GTX 1650 드라이버 없음)로는 UE5 실행이 어려워,
> 고성능 PC를 확보한 뒤 진행한다. 설계 근거는 `PhysicalAI_가상필드시험_논의정리.docx` §6, §7, §8.

## 1. 원칙

- **물리 계산은 MuJoCo(이후 Chrono 포함)가 하고, UE5는 받은 포즈로 그리기와 센서 시뮬레이션만 한다.**
- UE5 안의 로봇과 지형 액터는 물리를 끄고(키네마틱) 외부 포즈만 적용한다.
- 러너, 제어, 기록, 적합성 시험은 바뀌지 않는다. MuJoCo 뷰어 자리를 UE5가 대신할 뿐이다.
- 렌더러는 교체 가능해야 한다 (UE5 → O3DE/Unity 등). 그래서 송신 형식은 엔진 중립(REP-103 좌표, 미터)으로 하고,
  엔진별 변환은 엔진 쪽 어댑터 한 곳에서만 한다.

```
[시뮬레이션 PC] sim.runner ──포즈/지형 스트림(LAN)──▶ [UE5 PC] 수신 → 액터 갱신 → 렌더링/센서
                (물리·제어·기록)                         (물리 끔)
```

시뮬레이션 PC와 UE5 PC가 달라도 구조는 같다. 지금 PC는 시뮬레이션용으로 그대로 쓰고 UE5만 새 PC에서 돌릴 수 있다.

## 2. 이미 있는 것

**중립 렌더 스트림과 계약이 준비되어 있다: [`docs/render_interface.md`](render_interface.md).**
RViz 어댑터(`viz/rviz_adapter.py`)가 이 스트림만으로 로봇, 지형 변형, 시험 정보를 그리고 있으므로,
UE5 어댑터는 같은 입력을 받아 UE5 표현으로 바꾸는 "두 번째 어댑터"가 된다.

| 항목 | 위치 | 비고 |
|---|---|---|
| 바디 포즈 | `/tf` | 모든 바디 월드 포즈, 50 Hz |
| 장면 매니페스트 | `/sim/scene_manifest` | 바디, 외형 메쉬(외형 좌표계 OBJ), 지형 규격. 아래 3.2의 매니페스트에 해당 |
| 지형 패치 | `/sim/terrain_patch` | 높이 격자 변경분 (base64 float32). 아래 3.1의 지형 패치에 해당 |
| 상태 | `/sim/status` | 명령, 속도, 접촉, 이벤트, 최종 MOP |
| 공용 디코더 | `viz/stream_decode.py` | UE5 쪽 C++로 옮길 때 참고 |
| 좌표 변환식 | `sim/adapters.py` `mj_pose_to_ue()` | 위치 Y 반전 ×100(cm), 쿼터니언 (−x, y, −z, w). UE5 어댑터 안에서만 사용 |

## 3. 시뮬레이션 쪽에서 추가할 것

내용(매니페스트, 지형 패치, 상태)은 이미 계약으로 정해져 있다. 남은 것은 ROS2 없이 받을 수 있는 **UDP 전송**뿐이다.

### 3.1 렌더 스트림 송신 (`--render-udp host:port`)
ROS2 플러그인 없이 UE5 내장 소켓으로 받을 수 있게 UDP를 기본으로 한다. ROS2(`/tf`)는 대안으로 유지.

패킷(안, little-endian):

| 필드 | 형식 | 설명 |
|---|---|---|
| magic, version | u32, u16 | 형식 식별 |
| seq | u32 | 순번 (유실 감지) |
| sim_time | f64 | 시뮬레이션 시각 (s) |
| terrain_version | u32 | 이 값이 바뀌면 지형 패치 메시지를 확인 |
| n_bodies | u16 | |
| 바디 × n | u16 id, f32×3 pos, f32×4 quat(x,y,z,w), f32×3 linvel, f32×3 angvel | 월드 좌표, REP-103, m |

- 바디 13개 기준 패킷 약 0.6 KB, 60 Hz 송신이면 약 36 KB/s.
- 속도를 같이 보내는 이유: 텔레포트식 갱신은 속도를 잃어 모션 블러, 레이더 도플러 등에 필요 (문서 §6.3).
- 접촉점(발자국, 먼지 효과용)은 2단계에서 추가.

지형 패치 메시지(별도 타입): `terrain_version, r0, c0, nrow, ncol, heights(f32, m)`.
변경된 직사각형 영역만 보낸다. 큰 패치는 여러 패킷으로 나눈다.

### 3.2 장면 매니페스트 (`scene_manifest.json`, 문서 §8)
UE5가 처음 한 번 읽어 액터를 구성한다.

- 바디 목록: id, 이름(`base`, `FL_thigh` …), PhysicsSource(`MuJoCo`/`Chrono`/`Live`)
- 바디별 외형: 메쉬 파일, 바디 기준 위치/자세, 재질 색 (MJCF의 visual geom에서 추출)
- 지형: 격자 크기, 해상도, 원점, 초기 높이 파일(.npy 또는 .r16)
- 좌표 규약: REP-103, m, 쿼터니언 (x, y, z, w)

### 3.3 시험용 수신기
UE5 없이 패킷을 검증하는 작은 파이썬 수신기. 받은 포즈로 MuJoCo 오프스크린 렌더를 다시 그려 원본과 비교.

## 4. UE5 쪽에서 만들 것 (C++, 문서 §6.2)

| 구성 | 역할 |
|---|---|
| `UExternalPhysicsSubsystem` (WorldSubsystem) | UDP 수신 스레드, 프레임 단위로 최신 포즈 정리, 시뮬레이션 시각 보간 |
| `UExternalPhysicsBinding` (ActorComponent) | 액터마다 부착. `PhysicsSource`(Chaos/MuJoCo/Chrono/Live), 외부 바디 id 속성. 매 프레임 포즈 적용 |
| 좌표 어댑터 | REP-103 → UE (Y 반전, m→cm). 이 한 곳에서만 변환 |
| 매니페스트 로더 | JSON을 읽어 바디 액터와 메쉬 컴포넌트 생성 |
| 지형 | `ProceduralMeshComponent` 또는 런타임 메쉬로 heightfield 생성, 패치 메시지로 부분 갱신 |

유의사항 (문서 §6.3):
- 물리 시뮬레이션은 끄되 **충돌 형상은 유지**해야 LiDAR 레이캐스트 등 센서가 객체를 감지한다.
- 포즈 적용 시 텔레포트 플래그 사용, 속도는 메시지 값으로 별도 보관.
- Chaos 물리는 시험 결과와 무관한 연출(풀, 잔해)에만 쓴다.
- 정책 추론을 UE5(NNE)에 두지 않는다. 물리와 같은 주기로 맞물려야 하므로 러너 쪽에 둔다.

## 5. 단계

1. **포즈만**: 매니페스트 + UDP 송신 → UE5에서 Go2가 걷는 모습. 평지.
2. **지형**: 초기 heightfield + 패치 갱신 (rough_rut의 바퀴 자국이 UE5에 나타나는지).
3. **정합 확인**: 같은 시각의 MuJoCo 오프스크린 렌더와 UE5 화면을 겹쳐 위치 오차 확인, 송수신 지연 측정.
4. **센서**: 카메라/LiDAR를 UE5에서 렌더링해 ROS2 메시지로 내보냄 (시험 대상 소프트웨어 입력).
5. **다른 개체**: 차량(Chrono), 시나리오 콘솔의 DIS Entity State 개체 표시.

## 6. 미결 사항

- UE5 비게임 산업용 라이선스의 기관 적용 조건 확인 (문서 §16).
- UE5 PC OS: Windows(에디터 사용 편의) vs Linux(ROS2 일원화). 얇은 클라이언트 구조라 Windows여도 무방.
- ROS2 연동이 필요해지면 UE5용 ROS2 플러그인(예: Rapyuta Robotics의 rclUE) 검토.

## 7. 추천 PC 제원 (2026년 기준, 구매 시점에 세대 확인)

UE5 가시화 + 이후 MJX/Isaac Lab 강화학습까지 한 대로 하는 것을 전제로 한다.
학습 도구(JAX GPU, Isaac Lab, MuJoCo Warp)는 사실상 NVIDIA CUDA가 필요하므로 GPU는 NVIDIA를 권한다.
(시험평가 체계 자체는 CPU MuJoCo라 벤더 비종속 원칙과 충돌하지 않는다.)

| 항목 | 실용 구성 | 권장 구성 | 비고 |
|---|---|---|---|
| GPU | RTX 5060 Ti 16 GB급 | RTX 5070 Ti / 5080 16 GB급 (여유 있으면 5090 32 GB) | **VRAM 16 GB 이상**. UE5 대형 야외 지형 + 센서 렌더, RL 병렬 환경 수가 VRAM에 좌우됨 |
| CPU | 8코어 (Ryzen 7 / Core Ultra 7급) | 12~16코어 (Ryzen 9 / Core Ultra 9급) | UE5 셰이더 컴파일, CPU MuJoCo 병렬 시험(시드 여러 개) |
| RAM | 32 GB | 64 GB | UE5 에디터 권장 32 GB 이상 |
| 저장장치 | NVMe SSD 1 TB | NVMe SSD 2 TB | UE5 엔진+프로젝트+캐시 100 GB 이상 |
| 전원 | 750 W | 850~1000 W | GPU 등급에 맞춤 |
| 네트워크 | 유선 1 GbE | 유선 2.5 GbE | 시뮬레이션 PC와 같은 스위치. 무선은 지연 변동으로 비권장 |
| OS | Ubuntu 22.04/24.04 또는 Windows 11 | 듀얼 부팅 | ROS2 Humble은 Ubuntu 22.04 |

참고: 현재 PC의 GTX 1650도 NVIDIA 드라이버만 설치하면 MuJoCo 뷰어가 하드웨어 가속되어 훨씬 부드러워진다 (UE5용으로는 부족).
