"""DIS 시나리오 콘솔 규약 (문서 §10): 콘솔과 시뮬레이터가 함께 쓰는 값.

봉투와 핸드셰이크는 DIS_test(SIMAN-R)와 같다: Action Request-R(56)에 JSON 페이로드, 같은 Request ID로
Action Response-R(57) Pending -> Complete, 응답이 없으면 같은 Request ID로 재전송, 받는 쪽은 중복이면 다시 실행하지 않고
마지막 응답만 다시 보낸다. 주기 보고는 확인 없는 Data PDU(20).
페이로드 언어는 시나리오 관리(PAYLOAD_LANG 4). 아래 값은 모두 설명용 예시값이며 인터페이스 규약에서 확정한다.

메시지 (body의 t_apply: 적용할 시뮬레이션 시각. 없으면 받은 다음 제어 주기에 적용)
  Request_Connection                      -> 시나리오 이름, 시뮬레이션 시각, 지원 메시지
  Event_SetCommand    {vx, yaw_rate}      -> 로봇(Go2) 운용자 이동 명령 (가상 모드 전용)
  Event_AddTerrainPatch {patch}           -> 지형 변경 (bump / rough / ramp / rut, 시나리오 patches와 같은 형식)
  Event_InjectFault {fault, duration, params} -> 고장 주입 (로봇 쪽 경계). duration 없으면 Event_ClearFault까지
      lidar_blackout {sensor}            LiDAR 점군이 알고리즘에 오지 않음 (sensor 없으면 전부)
      link_loss {robot_behavior}         제어 링크 두절: 상태가 알고리즘에 안 가고 명령도 안 옴. 로봇은 hold_last(마지막 명령 유지)
                                         또는 damp(감쇠: 위치 게인 0)
      imu_bias {gyro_bias [3] rad/s, attitude_offset_deg [3]}   IMU 각속도 편향, 자세 출력 오프셋 (롤, 피치, 방향)
      battery_low {torque_scale}         모터 토크 한계 비율 (전압 저하)
  Event_ClearFault {fault}                -> 고장 해제
  Event_CreateEntity {entity_type, speed, cross_ahead} -> 개체 투입. cross_ahead(m)가 있으면 차가 로봇 진행선의 로봇 앞
                                             그 거리 지점을 먼저 지나가도록 시뮬레이터가 출발 시각을 정한다 (늦었으면 TOO_LATE). 지금은 HMMWV (Chrono). 시나리오에 선언해 둔 차량(vehicle.t_start: null)을
                                             숨겨 둔 출발점에서 나타나게 하고 선언한 경로로 출발시킨다 (실행 중 생성은 Chrono에서 불안정)
  Event_RemoveEntity {entity_type}        -> 개체 제거 (제동하고 숨김)
  Event_SetSoil {preset | soil}           -> 흙 상태 (Chrono SCM 값). preset: hard / firm / soft / mud
  Event_Freeze                            -> 일시정지 (시뮬레이션 시각이 멈춤. t_apply 가능)
  Event_Resume                            -> 재개 (바로)
  Event_Stop                              -> 시험 종료 (기록과 MOP 저장, t_apply 가능)
  Report_SimStatus (Data PDU, 0.2 s)      <- 시뮬레이션 시각, 로봇 위치·방향(판정자 참값), 명령, 마지막 이벤트, 고장, 일시정지,
                                             링크 상태
  Report_ConsoleHeartbeat (Data PDU, 1 s) -> 콘솔 생존 신호 (DIS_test와 같음)
링크 감시: 접속한 콘솔 모두에게서 COMM_LOST_S 동안 아무것도 오지 않으면 통신 두절. 시뮬레이터는 단절 시 동작
(STOP: 로봇 이동 명령 0을 이벤트로 넣음, CONTINUE: 그대로)을 실행하고 기록한다. 콘솔은 STALE 3 s, LOST 5 s로 표시.
좌표는 시뮬레이터 세계 좌표 (원점 = 로봇 출발점, +x = 처음 바라본 방향, +y = 왼쪽). 지형 패치의 Complete 응답에는
적용 순간의 로봇 기준 위치 robot_rel [앞, 왼쪽]이 들어간다 (로봇이 이미 지나간 곳인지 알 수 있게)
"""
from . import envelope as E

EXERCISE_ID = 1
SIM_ENTITY = E.EntityId(1, 10, 0)        # 시뮬레이터 (개체가 아니라 응용이므로 entity 0)
CONSOLE_ENTITY = E.EntityId(2, 1, 1)
DEFAULT_PORT = 3000

CONNECT = "Request_Connection"
SET_COMMAND = "Event_SetCommand"
ADD_PATCH = "Event_AddTerrainPatch"
INJECT_FAULT = "Event_InjectFault"
CLEAR_FAULT = "Event_ClearFault"
FREEZE, RESUME, STOP = "Event_Freeze", "Event_Resume", "Event_Stop"
CREATE_ENTITY, REMOVE_ENTITY, SET_SOIL = "Event_CreateEntity", "Event_RemoveEntity", "Event_SetSoil"
REPORT = "Report_SimStatus"
HEARTBEAT = "Report_ConsoleHeartbeat"
SUPPORTED = [CONNECT, SET_COMMAND, ADD_PATCH, INJECT_FAULT, CLEAR_FAULT, CREATE_ENTITY, REMOVE_ENTITY, SET_SOIL,
             FREEZE, RESUME, STOP]
ENTITY_TYPES = ("HMMWV",)
# 흙 (Bekker-Wong + Janosi, Chrono SCM). 자국 깊이는 HMMWV 기준 이 PC 측정값 (scenarios/vehicle_crossing*.yaml)
_SOIL = {"bekker_kc": 0.0, "bekker_n": 1.1, "cohesion": 0.0, "friction_deg": 30.0, "janosi_shear": 0.01,
         "elastic_k": 4.0e7, "damping_r": 3.0e4}
SOIL_PRESETS = {
    "hard": {**_SOIL, "bekker_kphi": 2.0e7},                    # 자국 약 1.8 cm
    "firm": {**_SOIL, "bekker_kphi": 5.0e6},                    # 약 5.5 cm (vehicle_crossing 기본)
    "soft": {**_SOIL, "bekker_kphi": 1.0e6},                    # 약 15 cm
    "mud": {**_SOIL, "bekker_kphi": 5.0e5, "friction_deg": 20.0, "janosi_shear": 0.03},   # 젖은 흙: 더 깊고 미끄러움
}
SOIL_KEYS = tuple(_SOIL) + ("bekker_kphi",)

FAULTS = ("lidar_blackout", "link_loss", "imu_bias", "battery_low")
MAX_FAULT_S = 3600.0
HEARTBEAT_S = 1.0                         # 콘솔 -> 시뮬레이터
CONSOLE_STALE_S, CONSOLE_LOST_S = 3.0, 5.0   # 콘솔 쪽 판정 (시뮬레이터 보고가 끊긴 시간)
COMM_LOST_S = 5.0                         # 시뮬레이터 쪽 판정 (콘솔에서 아무것도 안 온 시간)

# 운용자 명령 범위 (로봇 보호용 한계)
MAX_VX, MAX_YAW_RATE = 1.0, 1.5
PATCH_KEYS = {"bump": ("center", "height", "radius"), "rough": ("region", "amplitude"),
              "ramp": ("region", "rise"), "rut": ("start", "end", "width", "depth")}


def payload(type_, body=None):
    return E.Payload(type_, body or {}, lang=E.LANG_SCENARIO)
