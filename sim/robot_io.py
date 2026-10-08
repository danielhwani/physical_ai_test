"""로봇 쪽 입출력 (실제 로봇의 하드웨어 경계): 물리엔진 참값 -> 센서 모델 -> 로봇 상태 메시지, 관절 명령 -> 모터.

보행 알고리즘(control/)은 이 경계 너머의 메시지만 본다. 참값은 여기서만 쓰인다 (센서 모델의 입력).
  상태: 관절 엔코더(q, dq), IMU(자세 AHRS, 각속도, 가속도), 발 힘 (sensors/proprio.py의 잡음/편향/표류 적용)
  명령: 목표 관절각 + PD 게인 + 앞먹임 토크를 물리 스텝마다 토크로 바꿔 적용 (모터 드라이버). 토크 한계는 로봇 명세.
"""
import mujoco
import numpy as np

from sensors.proprio import ProprioSensors, ProprioSpec


class RobotIO:
    def __init__(self, sim, proprio_cfg=None, seed=0):
        self.sim = sim
        self.sensors = ProprioSensors(ProprioSpec.from_dict({**(proprio_cfg or {}), "seed": seed}))
        m = sim.model
        self.accel_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")]
        self.torque_limit = np.array(sim.spec["action"]["torque_limit"], dtype=float)
        self.foot_force = np.zeros(4)
        self.torque_scale = 1.0          # 고장 주입 battery_low: 토크 한계 비율

    def read(self):
        """로봇 상태 메시지 (지금 시각의 측정값)."""
        d = self.sim.data
        return self.sensors.measure(d.time, d.qpos[3:7], d.qvel[3:6], d.sensordata[self.accel_adr:self.accel_adr + 3],
                                    d.qpos[7:], d.qvel[6:], self.foot_force)

    def apply(self, cmd, on_substep=None):
        """관절 명령을 제어 주기 동안 실행 (물리 스텝 decim회). 반환: 소비 에너지(J)."""
        sim, d = self.sim, self.sim.data
        q_des, dq_des = np.asarray(cmd["q_des"]), np.asarray(cmd["dq_des"])
        kp, kd, tau_ff = np.asarray(cmd["kp"]), np.asarray(cmd["kd"]), np.asarray(cmd["tau_ff"])
        energy, fn = 0.0, np.zeros(4)
        for _ in range(sim.decim):
            tau = kp * (q_des - d.qpos[7:]) + kd * (dq_des - d.qvel[6:]) + tau_ff
            limit = self.torque_limit * self.torque_scale
            d.ctrl[:] = np.clip(tau, -limit, limit)
            mujoco.mj_step(sim.model, d)
            energy += np.abs(d.ctrl * d.qvel[6:]).sum() * sim.model.opt.timestep
            fn += sim.foot_normal_forces()
        # 발 힘 센서: 제어 주기 동안 물리 스텝 평균 (착지 순간 스파이크는 충격량만큼만)
        self.foot_force = fn / sim.decim
        return energy
