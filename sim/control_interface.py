"""제어 인터페이스 = 관측 규약 + 행동 처리 규약.

기본값은 로봇 명세(specs/go2_control.yaml)이고, 정책 카드(policies/*/card.yaml)가 있으면
카드에 적힌 항목이 덮어쓴다. 외부에서 학습한 정책은 관측 순서, 스케일, 관절 순서, PD 게인,
기본 자세, 제어 주기가 저마다 다르므로 그 차이를 코드가 아니라 카드로 흡수한다.

물리적 한계(토크 한계)는 정책이 아니라 로봇의 성질이므로 항상 로봇 명세를 따른다.
"""
from pathlib import Path

import numpy as np
import yaml

from .observation import ObservationSpec


class ControlInterface:
    def __init__(self, spec, card=None):
        card = card or {}
        robot, act = spec["robot"], spec["action"]
        card_act = card.get("action", {})

        self.model_joints = robot["joints"]
        self.joints = card.get("joints", self.model_joints)          # 정책의 관절 순서
        missing = set(self.joints) - set(self.model_joints)
        assert not missing and len(self.joints) == len(self.model_joints), f"관절 이름 불일치: {missing}"
        self.idx = np.array([self.model_joints.index(j) for j in self.joints])   # 정책 i -> 모델 idx[i]

        self.default_pose = np.array(card.get("default_pose", robot["default_pose"]), dtype=float)
        self.scale = card_act.get("scale", act["scale"])
        self.clip = card_act.get("clip", act["clip"])
        self.control_dt = card_act.get("control_dt", act["control_dt"])
        pd = card_act.get("pd", act["pd"])
        # PD 게인은 정책 순서로 적혀 있으므로 모델 순서로 바꿔 둔다
        self.kp = self._to_model(np.array(pd["kp"], dtype=float))
        self.kd = self._to_model(np.array(pd["kd"], dtype=float))
        self.torque_limit = np.array(act["torque_limit"], dtype=float)  # 항상 로봇 명세

        self.obs = ObservationSpec(card.get("observation", spec["observation"]), self.idx, self.default_pose)
        self.gait_period = card.get("gait_clock", {}).get("period")

    def _to_model(self, v_policy):
        out = np.empty_like(v_policy)
        out[self.idx] = v_policy
        return out

    def targets_from_action(self, action):
        """action(정책 순서) -> 관절 목표각(모델 순서). 명세의 행동 처리 경로."""
        a = np.clip(action, -self.clip, self.clip)
        return self._to_model(self.default_pose + self.scale * a)

    def action_from_targets(self, q_model):
        """관절 목표각(모델 순서) -> action(정책 순서). 규칙 기반 보행기를 같은 경로로 통과시킬 때 사용."""
        return np.clip((q_model[self.idx] - self.default_pose) / self.scale, -self.clip, self.clip)


def load_card(path):
    path = Path(path)
    card = yaml.safe_load(path.read_text())
    card["_dir"] = path.parent
    return card
