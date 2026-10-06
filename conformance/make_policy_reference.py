"""정책 입출력 기준 벡터 생성 (어떤 정책 카드에도 사용 가능).

    python conformance/make_policy_reference.py policies/go2_trot_bc/card.yaml

정책 자신이 걸어간 궤적에서 관측을 모아 ONNX 출력과 함께 <카드 폴더>/io_reference.npz 로 저장한다.
나중에 같은 ONNX를 C++ 런타임이나 탑재 컴퓨터에서 돌릴 때, 같은 관측을 넣어 같은 출력이 나오는지 확인하는 기준이다.
ONNX 파일을 바꾸면 onnx_sha256이 달라져 시험이 재생성을 요구한다.
"""
import contextlib
import hashlib
import io
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from control.interface import load_card  # noqa: E402
from sim.runner import Simulation  # noqa: E402


def onnx_sha256(card):
    return hashlib.sha256((card["_dir"] / card["onnx"]).read_bytes()).hexdigest()[:16]


def main(card_path):
    card = load_card(card_path)
    obs, act = [], []
    for scn in ("flat_trot", "rough_rut"):
        sim = Simulation(ROOT / f"scenarios/{scn}.yaml", policy=card_path)
        with contextlib.redirect_stdout(io.StringIO()):
            k = 0
            while sim.data.time < 10.0:
                sim.step_control()
                if k % 10 == 0:
                    obs.append(sim.obs.copy()); act.append(sim.ctrl.last_raw.copy())
                k += 1
    out = card["_dir"] / "io_reference.npz"
    np.savez_compressed(out, obs=np.array(obs, dtype=np.float32), actions=np.array(act, dtype=np.float32),
                        onnx_sha256=np.array(onnx_sha256(card)))
    print(f"{Path(out).resolve().relative_to(ROOT)}: {len(obs)} samples")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ROOT / "policies/go2_trot_bc/card.yaml")
