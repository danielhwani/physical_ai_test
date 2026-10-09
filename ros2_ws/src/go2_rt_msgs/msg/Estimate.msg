# 두뇌의 상태 추정 (진단용, /control/estimate). 판정자가 같은 시각의 참값과 비교해 추정 오차 MOP를 남긴다.
builtin_interfaces/Time stamp   # = t
float64 t                       # 추정에 쓴 로봇 상태 시각 (s)
float64[3] v_body               # 몸통 속도 (m/s, 몸통 좌표)
float64 yaw
float64 roll
float64 pitch
bool[4] contacts                # 발 접촉 추정
float64[3] pos                  # 위치 (m, odom)
float64[2] cmd                  # 가속 제한 뒤 이동 명령 (vx, yaw_rate)
