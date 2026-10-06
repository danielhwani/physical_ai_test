"""ONNX 정책 실행기.

정책 카드(card.yaml)가 가리키는 ONNX 파일을 불러 관측 -> action 추론만 한다.
관측 계산과 행동 처리(clip, scale, PD)는 runner가 같은 ControlInterface로 수행한다.
학습 프레임워크(Isaac Lab, MJX, 모방학습 등)와 무관하게 ONNX 파일 + 카드만 있으면 탑재된다.
"""
import numpy as np
import onnxruntime as ort


class OnnxPolicyController:
    name = "onnx"

    def __init__(self, card, interface):
        self.iface = interface
        io = card.get("io", {})
        opts = ort.SessionOptions()
        # 단일 스레드: 결정성(같은 입력 -> 같은 출력)과 실시간 지터 감소
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(card["_dir"] / card["onnx"]), opts,
                                            providers=["CPUExecutionProvider"])
        inp = self.session.get_inputs()[0]
        self.input_name = io.get("input", inp.name)
        self.output_name = io.get("output", self.session.get_outputs()[0].name)
        dim = inp.shape[-1]
        assert dim == interface.obs.dim, f"ONNX 입력 차원 {dim} != 카드 관측 차원 {interface.obs.dim}"

    def reset(self):
        self.last_raw = np.zeros(12, dtype=np.float32)

    def infer(self, obs):
        x = np.asarray(obs, dtype=np.float32)[None, :]
        return self.session.run([self.output_name], {self.input_name: x})[0][0]

    def act(self, ctx):
        self.last_raw = self.infer(ctx.obs)      # clip 전 원출력 (입출력 기준 벡터용)
        return self.last_raw.astype(float)
