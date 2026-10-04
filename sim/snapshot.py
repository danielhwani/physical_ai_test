"""오프스크린 스냅샷 (뷰어 없이 장면 확인용, 저해상도).

    python -m sim.snapshot scenarios/rough_rut.yaml --t 6 --out runs/snap.png
"""
import argparse
import contextlib
import io

import mujoco
import numpy as np

from .runner import Simulation


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--t", type=float, default=5.0, help="스냅샷 시각 (s)")
    ap.add_argument("--out", default="runs/snapshot.png")
    ap.add_argument("--size", type=int, nargs=2, default=[640, 400])
    args = ap.parse_args()

    sim = Simulation(args.scenario)
    with contextlib.redirect_stdout(io.StringIO()):
        while sim.data.time < args.t:
            sim.step_control()
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = args.size
    r = mujoco.Renderer(sim.model, height=args.size[1], width=args.size[0])
    cam = mujoco.MjvCamera()
    cam.type, cam.trackbodyid = mujoco.mjtCamera.mjCAMERA_TRACKING, sim.base_id
    cam.distance, cam.azimuth, cam.elevation = 1.8, 135, -20
    cam.lookat[:] = sim.data.xpos[sim.base_id]
    r.update_scene(sim.data, cam)
    from PIL import Image
    Image.fromarray(np.asarray(r.render())).save(args.out)
    print(args.out)


if __name__ == "__main__":
    main()
