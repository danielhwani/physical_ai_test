"""정책 카드 + ONNX 적합성 시험.

    python conformance/test_policy.py [policies/<이름>/card.yaml]

1. 카드 정합성: 카드 규약으로 관측을 만들 수 있고 ONNX 입출력 차원과 맞는지
2. 입출력 기준 벡터: 같은 관측 -> 같은 action (ONNX 파일이 바뀌면 재생성 요구)
3. 결정성: 정책으로 같은 시나리오를 두 번 돌리면 상태가 비트 단위로 같은지
4. 추론 지연: 제어 주기 대비 충분히 짧은지 (실시간/LVC 운용 조건)
"""
import contextlib
import io
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from conformance.make_policy_reference import onnx_sha256  # noqa: E402
from control.interface import ControlInterface, load_card  # noqa: E402
from control.onnx_policy import OnnxPolicyController  # noqa: E402
from sim.runner import Simulation  # noqa: E402

CARD_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 and __name__ == "__main__" else ROOT / "policies/go2_trot_bc/card.yaml"
SPEC = yaml.safe_load((ROOT / "specs/go2_control.yaml").read_text())


def _controller():
    card = load_card(CARD_PATH)
    iface = ControlInterface(SPEC, card)
    return card, iface, OnnxPolicyController(card, iface)


def test_card_consistency():
    card, iface, ctrl = _controller()          # 생성자에서 관측 항목/관절 이름/입력 차원을 검사
    out = ctrl.session.get_outputs()[0]
    assert out.shape[-1] == 12, out.shape
    assert iface.control_dt > 0 and (iface.control_dt / SPEC["physics"]["timestep"]).is_integer()


def test_io_reference():
    card, _, ctrl = _controller()
    ref = np.load(card["_dir"] / "io_reference.npz")
    assert str(ref["onnx_sha256"]) == onnx_sha256(card), \
        "ONNX 파일이 바뀌었다. conformance/make_policy_reference.py 로 기준 벡터를 다시 만들 것"
    for o, a in zip(ref["obs"], ref["actions"]):
        assert np.allclose(ctrl.infer(o), a, rtol=1e-5, atol=1e-5)


def _run(seconds):
    sim = Simulation(ROOT / "scenarios/rough_rut.yaml", policy=CARD_PATH)
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < seconds:
            sim.step_control()
    return sim.data.qpos.copy(), sim.data.qvel.copy()


def test_policy_determinism():
    (q1, v1), (q2, v2) = _run(5.0), _run(5.0)
    assert np.array_equal(q1, q2) and np.array_equal(v1, v2)


def test_inference_latency():
    _, iface, ctrl = _controller()
    obs = np.zeros(iface.obs.dim, dtype=np.float32)
    for _ in range(50):
        ctrl.infer(obs)
    lat = []
    for _ in range(2000):
        t0 = time.perf_counter(); ctrl.infer(obs); lat.append(time.perf_counter() - t0)
    p50, p99, worst = np.percentile(lat, 50) * 1e6, np.percentile(lat, 99) * 1e6, max(lat) * 1e6
    print(f"    추론 지연: p50 {p50:.0f} us, p99 {p99:.0f} us, 최대 {worst:.0f} us (제어 주기 {iface.control_dt * 1e3:.0f} ms)")
    assert np.percentile(lat, 99) < 0.05 * iface.control_dt     # 제어 주기의 5% 이내


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name}: OK")
