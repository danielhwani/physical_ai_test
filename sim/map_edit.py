"""운용자 지형 편집 (docs/map_server.md 5.6, 3단계 3a). 실행 중인 맵 서버에 별도 프로세스로 붙어 편집 하나를 넣고 끝난다.
시뮬레이터는 다음 지형 반영(5 Hz)에 받아 물리에 넣고, 재현 시나리오(scenario_replay.yaml)에 받은 시각으로 적는다.

    python -m sim.map_edit bump  --center 2 0 --height 0.05 --radius 0.5
    python -m sim.map_edit ramp  --region 1 -1 2 1 --rise 0.04
    python -m sim.map_edit rough --region 1 -1 3 1 --amplitude 0.01 [--seed 0]
    python -m sim.map_edit rut   --start 1 -1 --end 1 1 --width 0.2 --depth 0.04 [--box]
    python -m sim.map_edit set   --region 1 -1 2 1 --height 0.0          (높이 정하기: 그 칸의 자국도 지운다)

  종류: bump, ramp, rough -> add (변화량, 1 µm보다 작은 곳은 자른 사각형), rut -> dig (지금 높이 기준 절대 높이), set -> set.
  격자(칸 간격, 크기)는 서버가 쓰는 바탕 파일의 매니페스트에서 얻는다 (서버를 --base로 띄운 경우: sim.runner --map-server는 늘 그렇다).
  소유 영역(Chrono SCM)과 겹치면 알린다: 넣기는 하지만 SCM은 그 편집을 모르므로 다음 소유자 쓰기가 덮는다 (9절 "소유 영역에 넘어온 변경").
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .map_client import RemoteMap, domain_id
from .terrain_service import TerrainMapService

CROP = 1e-6           # bump처럼 끝이 0이 되지 않는 편집은 이보다 작은 변화량을 0으로 자른다 (m)


def grid_of(info):
    """서버의 바탕 매니페스트 -> TerrainMapService (칸 좌표 계산용)."""
    path = info.get("base_path")
    if not path:
        raise SystemExit("서버가 바탕 파일 없이 떠 있어 격자 크기를 모른다 (sim.map_server_node --base ...)")
    mf = json.loads((Path(path) / "manifest.json").read_text())
    return TerrainMapService(tuple(mf["size"]), mf["res"], tuple(mf["z_range"]))


def edit_message(t, patch, seed=0):
    """편집 -> (op, r0, c0, block). t: 지금 서버 지도를 target으로 가진 TerrainMapService (홈은 지금 높이 기준)."""
    t.rng = np.random.default_rng(seed)
    if patch["kind"] == "set":
        x0, y0, x1, y1 = patch["region"]
        rows, cols = np.nonzero(t._in(x0, y0, x1, y1))
        r0, c0 = rows.min(), cols.min()
        block = np.full((rows.max() - r0 + 1, cols.max() - c0 + 1), float(np.clip(patch["height"], t.z_min, t.z_max)))
        return "set", int(r0), int(c0), block
    d = t.patch_delta(patch)
    if isinstance(d, tuple):                                     # 홈: 파임 (바뀐 뒤 절대 높이, 안 바뀐 칸은 NaN)
        (rs, cs), block = d
        new = np.clip(t.target[rs, cs] + block, t.z_min, t.z_max)
        new[block == 0] = np.nan
        return "dig", rs.start, cs.start, new
    d = np.where(np.abs(d) < CROP, 0.0, d)
    rows, cols = np.nonzero(d)
    if rows.size == 0:
        raise SystemExit("편집이 격자 안에서 아무 칸도 바꾸지 않는다")
    r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
    return "add", int(r0), int(c0), d[r0:r1, c0:c1]


def main(argv=None):
    ap = argparse.ArgumentParser(description="실행 중인 맵 서버에 지형 편집 하나 넣기")
    common = argparse.ArgumentParser(add_help=False)             # 편집 종류 뒤에 써도 되게 (bump ... --wait 3)
    common.add_argument("--name", default="operator", help="클라이언트 이름 (서버 안에서 하나뿐이어야 함)")
    common.add_argument("--wait", type=float, default=10.0, help="서버를 기다리는 시간 (s)")
    sub = ap.add_subparsers(dest="kind", required=True)
    b = sub.add_parser("bump", parents=[common])
    b.add_argument("--center", type=float, nargs=2, required=True)
    b.add_argument("--height", type=float, required=True)
    b.add_argument("--radius", type=float, required=True)
    for k in ("ramp", "rough", "set"):
        p = sub.add_parser(k, parents=[common])
        p.add_argument("--region", type=float, nargs=4, required=True, metavar=("X0", "Y0", "X1", "Y1"))
        if k == "ramp":
            p.add_argument("--rise", type=float, required=True)
        elif k == "rough":
            p.add_argument("--amplitude", type=float, required=True)
            p.add_argument("--seed", type=int, default=0)
        else:
            p.add_argument("--height", type=float, required=True)
    r = sub.add_parser("rut", parents=[common])
    r.add_argument("--start", type=float, nargs=2, required=True)
    r.add_argument("--end", type=float, nargs=2, required=True)
    r.add_argument("--width", type=float, required=True)
    r.add_argument("--depth", type=float, required=True)
    r.add_argument("--box", action="store_true", help="평평한 바닥 + 가파른 벽 (기본: 완만한 cosine)")
    a = ap.parse_args(argv)

    patch = {k: v for k, v in vars(a).items() if k not in ("name", "seed", "box", "wait") and v is not None}
    if a.kind == "rut" and a.box:
        patch["profile"] = "box"
    try:
        m = RemoteMap(None, name=a.name, wait_s=a.wait)          # 붙는 절차: 지금 지도를 받는다
    except RuntimeError as e:
        print(f"편집하지 못함: {e}", file=sys.stderr)
        return 2
    t = grid_of(m.info)
    assert (t.nrow, t.ncol) == (m.local.nrow, m.local.ncol), "매니페스트 격자와 서버 지도가 다르다"
    t.target = m.local.compose(0, t.nrow, 0, t.ncol)
    op, r0, c0, block = edit_message(t, patch, getattr(a, "seed", 0))
    r1, c1 = r0 + block.shape[0], c0 + block.shape[1]
    owned = m.local.owned[r0:r1, c0:c1] & ~np.isnan(np.where(block == 0, np.nan, block) if op == "add" else block)
    m.write(op, a.name, r0, c0, block)
    au = m.audit(wait_s=10.0)
    ok = au.get("hash_mismatch") == 0
    x0, y0, x1, y1 = t.xs[c0], t.ys[r0], t.xs[c1 - 1], t.ys[r1 - 1]
    print(f"편집 {a.kind} -> {op}: 칸 [{r0}:{r1}, {c0}:{c1}] (x {x0:.2f} ~ {x1:.2f}, y {y0:.2f} ~ {y1:.2f} m), 서버 판 {au.get('server_version')}"
          f" (ROS_DOMAIN_ID {domain_id()}, 바탕 {m.info.get('map_name')})"
          f"{'' if ok else ', 감사 실패: ' + str(au)}")
    if owned.any():
        print(f"주의: 소유 영역(Chrono SCM)과 {int(owned.sum())}칸 겹친다. SCM은 이 편집을 모른다 (다음 소유자 쓰기가 덮음)")
    m.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
