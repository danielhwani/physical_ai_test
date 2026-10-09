# 관절 명령 (두뇌 -> 몸, /robot/low_cmd). 몸은 tau = kp (q_des - q) + kd (dq_des - dq) + tau_ff 를 토크 한계 안에서 실행한다.
builtin_interfaces/Time stamp   # = t
float64 t                       # 이 명령을 계산하는 데 쓴 로봇 상태의 시각 (s). 몸은 직전 상태 시각 이하의 명령만 실행한다
float64[12] q_des
float64[12] dq_des
float64[12] kp
float64[12] kd
float64[12] tau_ff
