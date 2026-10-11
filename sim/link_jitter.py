"""lockstep(같은 프로세스) 실행에서 명령 지연을 통계적으로 흉내 낸다: 가끔 더 늦게 도착하는 명령, 사라지는 명령.

고정 지연(control_link.latency_steps)만 쓰면 lockstep 시험의 명령은 언제나 정확히 한 주기 늦다. 실시간 실행과 실제 링크에서는
가끔 더 늦거나(계산 시간 튐, 스케줄링, 네트워크) 사라진다. 시나리오 control_link에 분포를 주면 명령마다 추가 지연을 뽑는다.

  control_link:
    latency_steps: 1             # 기본 지연 (지금과 같음)
    late_probability: 0.02       # 명령마다 더 늦을 확률. 늦으면 1..max_extra_steps 주기 균등
    max_extra_steps: 2
    extra_steps_probs: [0.02, 0.005]   # 또는 직접: +1주기 확률, +2주기 확률, ... (실측 히스토그램을 그대로. late_probability와 함께 쓰지 않음)
    drop_probability: 0.0        # 명령이 사라질 확률 (무선 손실)

로봇 쪽 규칙은 C++ 코어(rt/rt_core.cpp)와 같다: 도착한 명령 중 가장 새 것(계산에 쓴 상태 시각 t가 가장 늦은 것)을 실행하고,
지금 실행 중인 것보다 오래된 명령은 버린다 (뒤늦게 도착해 추월당한 명령). 새 명령이 없으면 마지막 명령을 유지한다.
난수는 시나리오 시드로 정해진다 (같은 시드 = 같은 지연 순서). 센서 잡음 난수와는 따로 뽑아 지연을 켜도 잡음 순서는 같다.
"""
import numpy as np

KEYS = ("late_probability", "max_extra_steps", "extra_steps_probs", "drop_probability")


def make_link_jitter(link, seed):
    """control_link 설정에 분포가 있으면 LinkJitter, 없으면 None (고정 지연, 이전과 비트 단위로 같은 경로)."""
    if not any(link.get(k) for k in ("late_probability", "extra_steps_probs", "drop_probability")):
        return None
    return LinkJitter(link, seed)


class LinkJitter:
    def __init__(self, link, seed):
        probs = link.get("extra_steps_probs")
        if probs and link.get("late_probability"):
            raise ValueError("control_link: extra_steps_probs와 late_probability 중 하나만")
        if not probs:
            p, m = float(link.get("late_probability", 0.0)), int(link.get("max_extra_steps", 1))
            if m < 1:
                raise ValueError("control_link.max_extra_steps >= 1")
            probs = [p / m] * m if p else []
        self.probs = np.array([float(x) for x in probs])
        self.drop = float(link.get("drop_probability", 0.0))
        if (self.probs < 0).any() or self.drop < 0 or self.probs.sum() + self.drop > 1.0 + 1e-12:
            raise ValueError(f"control_link: 확률이 잘못됐다 (추가 지연 합 {self.probs.sum()} + 손실 {self.drop} <= 1)")
        self.cum = np.cumsum(self.probs)
        self.rng = np.random.default_rng([int(seed), 0x4C494E4B])   # 시드 + 'LINK': 센서 잡음 난수와 독립
        self.inflight = []          # [(실행 가능한 제어 스텝, 명령)]
        self.last_t = None          # 지금 실행 중인 명령의 상태 시각
        self.sent = self.late = self.dropped = self.stale = 0
        self.hist = np.zeros(len(self.probs) + 1, int)     # 추가 지연 주기 수별 명령 수

    def config(self):
        return {"extra_steps_probs": self.probs.tolist(), "drop_probability": self.drop}

    def deliver(self, k, cmd, latency):
        """제어 스텝 k에 두뇌가 낸 명령을 보내고, 이번 스텝에 로봇이 새로 실행할 명령을 돌려준다 (없으면 None: 마지막 명령 유지)."""
        self.sent += 1
        u, v = self.rng.random(2)                          # 명령마다 두 개씩: 설정을 바꿔도 순서가 밀리지 않게
        if u < self.drop:
            self.dropped += 1
        else:
            extra = int(np.searchsorted(self.cum, v, side="right")) + 1 if len(self.cum) and v < self.cum[-1] else 0
            self.hist[extra] += 1
            self.late += extra > 0
            self.inflight.append((k + latency + extra, cmd))
        ready = [c for a, c in self.inflight if a <= k]
        if not ready:
            return None
        self.inflight = [(a, c) for a, c in self.inflight if a > k]
        new = max(ready, key=lambda c: c["t"])
        self.stale += len(ready) - 1
        if self.last_t is not None and new["t"] <= self.last_t:
            self.stale += 1                                 # 추월당한 명령: 이미 더 새 명령을 실행 중
            return None
        self.last_t = new["t"]
        return new

    def mop(self):
        return {"link_cmds_sent": self.sent, "link_cmds_late": self.late, "link_cmds_dropped": self.dropped,
                "link_cmds_stale": self.stale, "link_extra_steps_hist": self.hist.tolist()}
