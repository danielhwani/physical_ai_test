# 렌더 인터페이스 계약 (중립 스트림 v1.0)

설계 문서 §8 "인터페이스 계약을 먼저 고정"에 해당한다. 시뮬레이터(물리엔진)와 렌더러는 이 계약으로만 연결된다.

```
[시뮬레이터]                       중립 스트림                       [렌더러 어댑터]           [렌더러]
 sim/stream.py (인코더)  ──▶  TF, 매니페스트, 지형 패치, 상태  ──▶  viz/rviz_adapter.py  ──▶  RViz
 sim/ros2_bridge.py (전송)                                         (UE5 어댑터, 계획)  ──▶  UE5
```

- 시뮬레이터는 **무엇이 어디에 있는가**만 보낸다. 그리는 방법(URDF, 마커, 액터, 색, 문구)은 어댑터가 정한다.
- 어댑터(`viz/`)는 물리엔진을 import하지 않는다. `conformance/test_render_stream.py`가 검사한다.
- 내용은 전송 수단과 무관하다. 지금은 ROS2로 보내고, UE5용 UDP 전송도 같은 JSON을 실어 나르면 된다.

## 공통 규약

| 항목 | 값 |
|---|---|
| 좌표계 | REP-103: x 전방, y 왼쪽, z 위, 오른손 좌표계 |
| 단위 | m, rad, s |
| 쿼터니언 | x, y, z, w |
| 기준 프레임 | `world` (매니페스트 `frame`) |
| 시각 | 시뮬레이션 시각. ROS2 구독자는 `use_sim_time:=true` |
| 버전 | 매니페스트 `stream_version` = `"1.0"`. 주 버전(1)이 다르면 어댑터가 거부 |

엔진별 좌표 변환(예: UE5 왼손 좌표계, cm)은 그 렌더러의 어댑터 한 곳에서만 한다.

## 메시지

### 1. 바디 포즈: `/tf` (tf2_msgs/TFMessage)
제어 주기(50 Hz)마다 모든 바디의 월드 포즈. `header.frame_id = world`, `child_frame_id = 바디 이름`.
보조: `/clock`, `/joint_states`, `/sim/base_twist`(몸통 선속도·각속도, 월드 좌표).

### 2. 장면 매니페스트: `/sim/scene_manifest` (std_msgs/String, JSON, transient local)
시작 시 한 번. 늦게 붙은 구독자도 받는다.

```json
{
  "stream_version": "1.0", "frame": "world",
  "conventions": {"coordinates": "REP-103 ...", "units": "m", "quaternion": "x,y,z,w"},
  "asset_root": "/절대/경로/physics_ai_test",
  "bodies":  [{"id": 1, "name": "base", "parent": "world", "physics_source": "MuJoCo"}, ...],
  "visuals": [{"body": "base", "mesh": "build/scene_meshes/base_0.obj",
               "pos": [x, y, z], "quat": [x, y, z, w], "rgba": [r, g, b, a]}, ...],
  "terrain": {"half_size": [hx, hy], "nrow": 401, "ncol": 401, "z_range": [-0.2, 0.4],
              "layout": "row-major; row i -> y = -half_y + i*dy, col j -> x = -half_x + j*dx"}
}
```

- 외형 메쉬(`mesh`)는 `asset_root` 기준 상대 경로의 OBJ이며, 정점은 **외형 좌표계**다.
  월드 정점 = 바디 포즈(TF) ∘ (`pos`, `quat`) ∘ 정점.
  (MuJoCo가 컴파일한 메쉬를 내보낸 것이라 원본 Menagerie OBJ와 좌표계가 다르다.)
- `physics_source`: 그 바디의 움직임을 누가 결정하는가 (`MuJoCo`, `Chrono`, 이후 `Live`). 문서 §6.2, §9.
  다른 물리엔진의 바디는 id 1000부터 쓰고(MuJoCo 바디 id와 겹치지 않게), 포즈는 같은 `/tf`로 보낸다.
  예: Chrono HMMWV는 차체와 바퀴 축 4개(`hmmwv_*`)가 들어가며, 메쉬는 `build/scene_meshes/chrono/`.
- 다른 PC의 렌더러는 `build/scene_meshes/`를 복사해 가거나, 이후 자산 전송 방식을 정한다 (미결).

### 3. 지형 패치: `/sim/terrain_patch` (std_msgs/String, JSON, transient local, depth 500)
시작 시 전체 격자 1개, 이후 물리엔진에 반영된 지형이 바뀔 때마다 바뀐 셀을 감싸는 최소 직사각형 1개.

```json
{"version": 3, "r0": 0, "c0": 260, "nrow": 401, "ncol": 21,
 "encoding": "base64-float32-le", "heights": "..."}
```

- `heights`: `nrow × ncol` 절대 높이(m), 행 우선, float32 little-endian을 base64로.
- 순서대로 적용하면 물리엔진이 쓰는 지형과 같아진다 (오차 < 1e-6 m, 계약 시험).
- "물리엔진에 반영된" 지형이다. Terrain Map Service가 로봇 발 근처 셀을 보류하는 동안은 그 셀이 바뀌지 않는다.

### 4. 상태: `/sim/status` (std_msgs/String, JSON, 10 Hz)

```json
{"t": 6.2, "controller": "trot", "variant": "cpu", "vehicles": [{"name": "hmmwv", "pos": [x, y, z], "speed": 2.8, "distance": 2.5}],
 "command": {"vx": 0.35, "yaw_rate": 0.0}, "speed_avg": 0.33, "distance": 2.1, "cot": 3.8,
 "base_pos": [x, y, z], "yaw": 0.01,
 "feet": {"FL": {"pos": [x, y, z], "contact": true}, ...},
 "late_steps": 0, "event": {"t": 4.0, "desc": "add_patch rut"},
 "final_mop": null}
```

- `final_mop`: 시험이 끝나면 최종 MOP(`runs/*/summary.json`과 같은 내용)가 들어간 마지막 상태를 한 번 더 보낸다.
- `speed_avg`: 보행 주기 출렁임을 없앤 약 0.5 s 평균 전진 속도 (표시용).

## 어댑터 추가 방법

1. 위 입력을 구독한다 (ROS2 또는 이후 UDP).
2. 매니페스트로 바디/외형을 만들고, TF로 매 프레임 바디 포즈를 갱신한다 (물리 끔, 키네마틱).
3. 지형 패치로 높이 격자를 유지하고 바뀐 영역만 다시 만든다.
4. 상태는 렌더러에 맞게 표시한다 (문구, 색, 위젯).

공용 디코더: `viz/stream_decode.py` (`TerrainGrid`, `decode_heights`, `quat_to_matrix` 등).

| 어댑터 | 실행 | 표현 |
|---|---|---|
| `viz/rviz_adapter.py` | `--rviz` | URDF(RobotModel), 지형 타일 마커, 정보판 마커. MuJoCo도 import하지 않음 |
| `viz/mujoco_adapter.py` | `--mjviz` | 매니페스트로 만든 **그리기 전용** MuJoCo 모델 (모든 바디 mocap, 물리 계산 없음) + 뷰어. MuJoCo를 그리기 라이브러리로만 사용 |
| UE5 어댑터 | (계획) | `docs/ue5_integration.md` |

`--rviz`와 `--mjviz`는 함께 켤 수 있다. 렌더러는 `sim/`(물리 시뮬레이션)을 import하지 않는다.

## 계약 시험 (`conformance/test_render_stream.py`)

| 시험 | 확인 내용 |
|---|---|
| `test_viz_does_not_depend_on_physics` | `viz/`가 `sim`을 불러오지 않음. RViz 경로는 `mujoco`도 불러오지 않음 |
| `test_visuals_reconstruct_simulator_geometry` | 스트림으로 놓은 메쉬 정점 = MuJoCo 월드 geom 정점 (< 1e-5 m) |
| `test_terrain_patches_reconstruct_physics_terrain` | 패치 누적 = 물리엔진 지형, 실행 중 변형 포함 (< 1e-6 m) |
| `test_status_and_urdf` | 상태/매니페스트 JSON 왕복, 매니페스트로 만든 URDF의 링크와 메쉬 파일 |
| `test_mujoco_render_model_matches_simulator` | 그리기 전용 MuJoCo 모델 + 포즈 = 시뮬레이터 외형 (< 1e-5 m), 지형 일치 |
| `test_mujoco_overlay_path_segments` | 경로 선분이 연속한 점을 잇는지 (길이 0 선분 없음) |
