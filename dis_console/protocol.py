"""DIS 시나리오 콘솔 규약 (문서 §10): 콘솔과 시뮬레이터가 함께 쓰는 값.

봉투와 핸드셰이크는 DIS_test(SIMAN-R)와 같다: Action Request-R(56)에 JSON 페이로드, 같은 Request ID로
Action Response-R(57) Pending -> Complete, 응답이 없으면 같은 Request ID로 재전송, 받는 쪽은 중복이면 다시 실행하지 않고
마지막 응답만 다시 보낸다. 주기 보고는 확인 없는 Data PDU(20).
페이로드 언어는 시나리오 관리(PAYLOAD_LANG 4). 아래 값은 모두 설명용 예시값이며 인터페이스 규약에서 확정한다.

메시지 (body의 t_apply: 적용할 시뮬레이션 시각. 없으면 받은 다음 제어 주기에 적용)
  Request_Connection                      -> 시나리오 이름, 시뮬레이션 시각, 지원 메시지
  Event_SetCommand    {vx, yaw_rate}      -> 로봇(Go2) 운용자 이동 명령 (가상 모드 전용)
  Event_AddTerrainPatch {patch}           -> 지형 변경 (bump / rough / ramp / rut, 시나리오 patches와 같은 형식)
  Report_SimStatus (Data PDU, 0.2 s)      <- 시뮬레이션 시각, 로봇 위치·방향(판정자 참값), 명령, 마지막 이벤트
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
REPORT = "Report_SimStatus"
SUPPORTED = [CONNECT, SET_COMMAND, ADD_PATCH]

# 운용자 명령 범위 (로봇 보호용 한계)
MAX_VX, MAX_YAW_RATE = 1.0, 1.5
PATCH_KEYS = {"bump": ("center", "height", "radius"), "rough": ("region", "amplitude"),
              "ramp": ("region", "rise"), "rut": ("start", "end", "width", "depth")}


def payload(type_, body=None):
    return E.Payload(type_, body or {}, lang=E.LANG_SCENARIO)
