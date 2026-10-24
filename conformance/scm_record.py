"""SCM 시험 기록(.npz) 규약: 시험 한 번의 결과를 담아, 분석 그래프(plot_scm_probe.py)와 Godot 재생(replay_scm_probe.py)이 같은 기록을
읽는다 (시뮬레이션은 한 번). pychrono 없이 읽고 쓴다.

  kind      시험 종류: "press" (probe_scm_injection), "compaction" (probe_scm_compaction),
            "virtual_compaction" (probe_scm_virtual_compaction), "rolling" (probe_scm_rolling), "offset_compare" (probe_scm_offset_compare),
            "wheel" (probe_scm_wheel)
  format    이 규약의 판 (FORMAT)
  config    시험 설정 (JSON): 흙, 구 반지름, 격자, 하중 등
  analysis  분석 데이터 (JSON): 표의 숫자와 그래프용 곡선
  렌더용 (누르기·굴리기를 --record로 돌렸을 때만): heights (프레임, 행, 열), center (프레임, 3), [quat (프레임, 4) xyzw], phase,
            sink, [force, force_label], hold, dt, size, radius, [shape (JSON: kind sphere/cylinder, radius, width)], regions (JSON),
            title, result_lines (JSON). 화면 글자는 영어와 숫자만.
"""
import json

import numpy as np

FORMAT = 1


def _plain(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.generic):
        return o.item()
    raise TypeError(f"JSON으로 바꿀 수 없다: {type(o)}")


def save(path, kind, config, analysis, render=None):
    np.savez_compressed(path, kind=kind, format=FORMAT, config=json.dumps(config, default=_plain),
                        analysis=json.dumps(analysis, default=_plain), **(render or {}))


def load(path):
    d = dict(np.load(path))
    if "kind" not in d:
        raise ValueError(f"{path}: 시험 기록 규약(kind, format)이 없는 옛 기록. 시험을 --record로 다시 돌릴 것")
    if int(d["format"]) != FORMAT:
        raise ValueError(f"{path}: 기록 규약 판 {int(d['format'])} (이 코드는 {FORMAT})")
    d["kind"] = str(d["kind"])
    d["config"] = json.loads(str(d["config"]))
    d["analysis"] = json.loads(str(d["analysis"]))
    return d


def has_render(d):
    return "heights" in d
