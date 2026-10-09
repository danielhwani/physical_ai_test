# 로봇 상태 (몸 -> 두뇌, /robot/low_state). 센서 측정값 (잡음·편향·고장 포함), 참값이 아니다.
# 필드 추가만 하고 지우거나 뜻을 바꾸지 않는다 (두 PC의 정의가 어긋날 때 사고를 줄이기 위해).
builtin_interfaces/Time stamp   # = t (표준 도구용)
float64 t                       # 시뮬레이션 시각 (s). 명령 지연 측정과 lockstep 판정의 기준
int64 step                      # 제어 주기 번호 (실시간 코어 경로만, 없으면 -1)
float64[12] q                   # 관절 각도 (rad), specs/go2_control.yaml의 관절 순서
float64[12] dq                  # 관절 각속도 (rad/s)
float64[4] imu_quat             # IMU 자세 (x, y, z, w)
float64[3] imu_gyro             # 각속도 (rad/s, 몸통 좌표)
float64[3] imu_accel            # 가속도 (m/s^2, 몸통 좌표)
float64[4] foot_force           # 발 접촉 힘 (N), 앞오른쪽·앞왼쪽·뒤오른쪽·뒤왼쪽
