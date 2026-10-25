"""맵 서버 바탕 파일 (docs/map_server.md 5.3, 2단계 2c). 안 바뀌는 바탕은 미리 만들어 두고(배포), 서버와 클라이언트는 해시로 같은지 확인한다.
네트워크로는 바탕 대신 타일 해시만 가고, 그 뒤 변경만 간다.

    python -m sim.map_base build scenarios/vehicle_crossing.yaml [--out build/maps/vehicle_crossing]   시나리오 시작 지형으로 만들기
    python -m sim.map_base verify build/maps/vehicle_crossing                                           파일과 해시가 맞는지

  폴더 <out>/
    manifest.json        {"format", "name", "grid": [nrow, ncol], "res", "size", "z_range", "tile", "layer": "height",
                          "base_sha256": 격자 전체 해시, "tiles": {"tr,tc": {"file", "shape", "sha256"}}, "source": {...}}
    height/<tr>_<tc>.npy 타일 높이 (float64 little-endian, 절대 높이 m). 타일 = TILE × TILE 칸 (끝 타일은 작다), 행 = y, 열 = x
  해시: float64 little-endian C 순서 바이트의 SHA-256 (sim/map_client.py sha와 같다). 타일 해시는 tile_shas로 바탕 격자에서 바로 낸다.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from .map_client import sha
from .map_server import FORMAT, TILE

ROOT = Path(__file__).resolve().parent.parent


def tile_ranges(nrow, ncol, tile=TILE):
    """(키 "tr,tc", r0, r1, c0, c1) 목록 (MapServer의 타일과 같은 나눔)."""
    out = []
    for tr in range((nrow + tile - 1) // tile):
        for tc in range((ncol + tile - 1) // tile):
            out.append((f"{tr},{tc}", tr * tile, min((tr + 1) * tile, nrow), tc * tile, min((tc + 1) * tile, ncol)))
    return out


def tile_shas(h, tile=TILE):
    return {k: sha(h[r0:r1, c0:c1]) for k, r0, r1, c0, c1 in tile_ranges(*h.shape, tile)}


def write_base(h, out, z_range, res=None, size=None, name=None, source=None, tile=TILE):
    """바탕 격자 h를 타일 파일과 매니페스트로 쓴다. 이미 같은 바탕(base_sha256)이 있으면 그대로 둔다. 반환: 매니페스트."""
    out = Path(out)
    h = np.ascontiguousarray(h, dtype="<f8")
    whole = sha(h)
    mf_path = out / "manifest.json"
    if mf_path.exists():
        try:
            old = json.loads(mf_path.read_text())
            if old.get("base_sha256") == whole and old.get("tile") == tile:
                load_base(out)                                   # 타일 파일도 맞는지 (누가 고쳤으면 다시 쓴다)
                return old
        except (ValueError, OSError, RuntimeError):
            pass
    (out / "height").mkdir(parents=True, exist_ok=True)
    tiles = {}
    for k, r0, r1, c0, c1 in tile_ranges(*h.shape, tile):
        f = f"height/{k.replace(',', '_')}.npy"
        np.save(out / f, h[r0:r1, c0:c1])
        tiles[k] = {"file": f, "shape": [r1 - r0, c1 - c0], "sha256": sha(h[r0:r1, c0:c1])}
    mf = {"format": FORMAT, "name": name or out.name, "grid": list(h.shape), "res": res, "size": size,
          "z_range": list(z_range), "tile": tile, "layer": "height", "base_sha256": whole, "tiles": tiles, "source": source or {}}
    mf_path.write_text(json.dumps(mf, indent=1))
    return mf


def load_base(path, verify=True):
    """바탕 폴더 -> (h, 매니페스트). verify: 타일마다 해시를 확인하고, 어긋나면 그 타일들을 적어 RuntimeError."""
    path = Path(path)
    mf = json.loads((path / "manifest.json").read_text())
    h = np.empty(tuple(mf["grid"]), dtype="<f8")
    bad = []
    for k, r0, r1, c0, c1 in tile_ranges(*mf["grid"], mf["tile"]):
        t = mf["tiles"][k]
        a = np.load(path / t["file"]).astype("<f8", copy=False)
        if list(a.shape) != [r1 - r0, c1 - c0]:
            bad.append(f"{k} (크기 {list(a.shape)})")
            continue
        if verify and sha(a) != t["sha256"]:
            bad.append(k)
        h[r0:r1, c0:c1] = a
    if bad:
        raise RuntimeError(f"바탕 파일이 매니페스트 해시와 다르다 ({path}): 타일 {bad}")
    if verify and sha(h) != mf["base_sha256"]:
        raise RuntimeError(f"바탕 전체 해시가 다르다 ({path})")
    return h, mf


def scenario_base(scn):
    """시나리오(dict)의 시작 지형 -> (h, z_range, res, size). 시뮬레이터 시작 지형과 같은 함수 (TerrainMapService.from_scenario)."""
    from .terrain_service import TerrainMapService
    t = TerrainMapService.from_scenario(scn)
    return t.target.copy(), (t.z_min, t.z_max), t.res, [t.half_x, t.half_y]


def build_for_scenario(scn, out):
    h, z_range, res, size = scenario_base(scn)
    src = {"scenario": scn.get("name"), "seed": scn.get("seed", 0), "patches": len(scn["terrain"].get("patches", []))}
    return write_base(h, out, z_range, res=res, size=size, name=scn.get("name"), source=src)


def prepare_base(scn, map_base=None):
    """시뮬레이터 --map-server: 바탕 폴더. map_base(미리 배포한 바탕)를 주면 그대로 (서버가 해시 확인, 시작 지형과 다르면 클라이언트가
    거부됨), 없으면 시나리오 시작 지형으로 build/maps/<이름>에 만든다 (같은 바탕이 이미 있으면 다시 쓰지 않음)."""
    if map_base:
        return Path(map_base)
    out = ROOT / "build/maps" / scn.get("name", "scenario")
    build_for_scenario(scn, out)
    return out


def main():
    import yaml
    ap = argparse.ArgumentParser(description="맵 서버 바탕 파일")
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="시나리오 시작 지형으로 바탕 파일 만들기")
    b.add_argument("scenario")
    b.add_argument("--out", help="폴더 (기본 build/maps/<시나리오 이름>)")
    v = sub.add_parser("verify", help="파일과 해시 확인")
    v.add_argument("path")
    a = ap.parse_args()
    if a.cmd == "build":
        scn = yaml.safe_load(open(a.scenario))
        out = Path(a.out) if a.out else ROOT / "build/maps" / scn["name"]
        mf = build_for_scenario(scn, out)
        print(f"바탕: {out}  격자 {mf['grid']}, 타일 {len(mf['tiles'])}개, 전체 해시 {mf['base_sha256'][:16]}")
    else:
        h, mf = load_base(a.path)
        print(f"맞음: {a.path}  격자 {mf['grid']}, 타일 {len(mf['tiles'])}개, 전체 해시 {mf['base_sha256'][:16]}, "
              f"높이 {h.min():.3f} ~ {h.max():.3f} m")


if __name__ == "__main__":
    main()
