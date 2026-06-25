#!/usr/bin/env python3
"""calibrate_guided.py -- guided C920 -> robot-base homography calibration.

Replicates, automatically, the socket-proxy procedure used to produce
``config/c920_homography.npz`` (see MANUAL_CALIBRATION.md). You move the arm with
**Franka native guiding** (grip handles); this script only DETECTS the socket and
READS the end-effector pose. It issues no robot motion.

Per point (do >= 4, spread across the workspace):
  1. Place the SOCKET where the overhead C920 sees it, arm parked clear -> Enter.
     The script grabs a frame and detects the white-cube CENTROID
     (robo67_insertion.lib.hole_detect.detect_white_cubes) -- the same feature
     the homography is calibrated on. Robust to overexposure; the bore sits
     ~centred on the cube so the centroid is a stable proxy.
  2. Guide the peg tip into THAT socket's bore (don't move the socket) -> Enter.
     The script reads the EE pose from FrankaState = that socket's base XY.
Then it fits + saves the homography and prints the RMS reprojection error.

WHY this (not bore detection / autonomous motion): the real subscriber
controller is too stiff to soft-float and its gravity comp is off (arm sags),
so the arm is hand-guided; and the white-on-white bore is too faint to detect
reliably when the C920 is even slightly overexposed -- the cube square is not.

RUN (inside multipanda-container; normally via ./start_calibration.sh):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 calibration/calibrate_guided.py

Verify detection OFFLINE on a saved still (no robot/ROS needed):
    python3 calibration/calibrate_guided.py --test-detect captures/pt5b_annot.jpg
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# Resolve the robo67_insertion package (sibling of this calibration/ folder).
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PKG = os.path.join(_REPO, "robo67_insertion")
for p in (_PKG, _REPO):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

from robo67_insertion.lib.hole_detect import WhiteCubeParams, detect_white_cubes  # noqa: E402
from robo67_insertion.lib import geometry  # noqa: E402
from robo67_insertion.nodes.calibration_node import fit_and_save  # noqa: E402

DEFAULT_OUT = os.path.join(_PKG, "config", "c920_homography.npz")
DEFAULT_CONFIG = os.path.join(_PKG, "config", "robo67.yaml")
DEFAULT_CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")
ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"


def annotate(img, hole, path):
    import cv2
    r = int(hole.radius_px)
    cv2.rectangle(img, (int(hole.u - r), int(hole.v - r)),
                  (int(hole.u + r), int(hole.v + r)), (0, 0, 255), 2)
    cv2.circle(img, (int(hole.u), int(hole.v)), 4, (0, 255, 0), -1)
    cv2.imwrite(path, img)


def detect_cube(frames, params):
    """Median cube centroid over frames that detected a cube; (Hole-like, n) or (None,0)."""
    us, vs, rs = [], [], []
    for img in frames:
        d = detect_white_cubes(img, params)
        if d:
            us.append(d[0].u); vs.append(d[0].v); rs.append(d[0].radius_px)
    if not us:
        return None, 0
    from robo67_insertion.lib.hole_detect import Hole
    return Hole(u=float(np.median(us)), v=float(np.median(vs)),
                radius_px=float(np.median(rs)), score=1.0), len(us)


def test_detect(args):
    import cv2
    img = cv2.imread(args.test_detect)
    if img is None:
        print(f"could not read {args.test_detect}", file=sys.stderr); return 1
    d = detect_white_cubes(img, WhiteCubeParams())
    print("cubes:", [(round(h.u), round(h.v), round(h.radius_px), round(h.score, 2)) for h in d])
    if d:
        out = os.path.splitext(args.test_detect)[0] + "_cube.jpg"
        annotate(img, d[0], out)
        print("annotated:", out)
    return 0 if d else 1


# ---------------------------------------------------------------------------
# Live guided capture.
# ---------------------------------------------------------------------------

def run(args):
    import rclpy
    from rclpy.node import Node
    from franka_msgs.msg import FrankaState
    from robo67_insertion.nodes.socket_detector_node import grab_frame_gst

    if args.c920_device is None:
        from robo67_insertion.config_schema import load_config
        cam = load_config(args.config).camera
        args.c920_device = cam.c920_device
        if args.exposure is None:
            args.exposure = cam.c920_exposure
    params = WhiteCubeParams()
    os.makedirs(args.capture_dir, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("calibrate_guided")
    state = {}

    def on_state(m):
        state["xyz"], _ = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        state["mode"] = int(m.robot_mode)
        state["t"] = time.time()
    node.create_subscription(FrankaState, ROBOT_STATE, on_state, 10)

    def fresh_ee(timeout=6.0):
        state.pop("xyz", None)
        t0 = time.time()
        while "xyz" not in state and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        return state.get("xyz"), state.get("mode")

    print("=" * 60)
    print(" Robo67 guided C920->base calibration (socket-proxy, cube)")
    print("=" * 60)
    print(" Move the arm ONLY with Franka native guiding (grip handles).")
    print(" Keep ONLY the socket cube in the camera view (hole-up).")
    print(f" device={args.c920_device}  exposure={args.exposure}  out={args.out}")

    if fresh_ee(8.0)[0] is None:
        print("\nERROR: no FrankaState. The bringup is down (a guide/e-stop can crash "
              "it). Relaunch it (see MANUAL_CALIBRATION.md) and re-run.", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    pixels, base_xy, zs = [], [], []
    i = 0
    while True:
        print(f"\n--- point #{i}  ({len(pixels)} captured) ---")
        try:
            tok = input("  1) Place socket (arm CLEAR), then Enter to detect "
                        "[q=finish, r=retry]: ").strip().lower()
        except EOFError:
            break
        if tok == "q":
            break
        if tok == "r":
            continue
        frames = []
        for _ in range(max(1, args.frames)):
            f = grab_frame_gst(args.c920_device, exposure=args.exposure)
            if f is not None:
                frames.append(f)
        hole, n = detect_cube(frames, params)
        if hole is None:
            print(f"     NO cube detected in {len(frames)} frame(s). Ensure the white "
                  "socket is in view, arm clear; then retry."); continue
        annotate(frames[-1].copy(), hole, os.path.join(args.capture_dir, f"point_{i:02d}.jpg"))
        print(f"     cube pixel = ({hole.u:.1f}, {hole.v:.1f})  [{n}/{len(frames)} frames]"
              f"  -> {args.capture_dir}/point_{i:02d}.jpg")
        if len(detect_white_cubes(frames[-1], params)) > 1:
            print("     WARNING: more than one white cube in view -- remove the blank cube!")

        input("  2) Guide the peg tip into THAT bore (don't move socket), then Enter: ")
        xyz, mode = fresh_ee()
        if xyz is None:
            print("     no robot state (bringup down?). Relaunch it, then press Enter "
                  "to retry this point.", file=sys.stderr)
            input("     Enter to retry: "); 
            xyz, mode = fresh_ee()
            if xyz is None:
                print("     still no state -- skipping this point."); continue
        print(f"     base XY = ({xyz[0]:.4f}, {xyz[1]:.4f})  z={xyz[2]:.4f}  mode={mode}")
        pixels.append([hole.u, hole.v]); base_xy.append([float(xyz[0]), float(xyz[1])])
        zs.append(float(xyz[2])); i += 1

    node.destroy_node(); rclpy.shutdown()
    return fit_report(np.array(pixels, float), np.array(base_xy, float),
                      np.array(zs, float), args)


def fit_report(pixels, base_xy, zs, args):
    n = 0 if pixels.size == 0 else len(pixels)
    print(f"\n=== fit === ({n} points)")
    if n < args.min_points:
        print(f"ERROR: need >= {args.min_points} points, have {n} -- nothing saved.",
              file=sys.stderr)
        return 1
    H, rms = fit_and_save(pixels, base_xy, args.out)
    reproj = geometry.pixel_to_base(H, pixels)
    err = np.linalg.norm(reproj - base_xy, axis=1) * 1000
    corr = os.path.join(args.capture_dir, "..", "c920_corr.csv")
    np.savetxt(os.path.abspath(corr), np.hstack([pixels, base_xy]),
               delimiter=",", header="u,v,base_x,base_y")
    print(f"homography saved : {args.out}")
    print(f"correspondences  : {os.path.abspath(corr)}")
    print(f"RMS reprojection : {rms*1000:.2f} mm   per-point: {[round(e,1) for e in err]}")
    print(f"socket-top Z mean: {zs.mean():.4f} m  (range {zs.min():.3f}-{zs.max():.3f})")
    print("\nVerify (no motion):")
    print(f"  python3 robo67_insertion/scripts/hw_peg_in_hole_vision.py "
          f"--socket-top-z {zs.mean()+0.015:.3f} --dry-run")
    if rms * 1000 > args.max_rms_mm:
        print(f"WARNING: RMS {rms*1000:.1f} mm > {args.max_rms_mm} mm -- spread points "
              "wider / centre the peg more carefully; consider recapturing.")
        return 1
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Guided C920->base homography calibration.")
    ap.add_argument("--test-detect", default="", help="offline: run cube detection on an image and exit")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR)
    ap.add_argument("--c920-device", type=str, default=None, help="default: from config")
    ap.add_argument("--exposure", type=int, default=None, help="default: from config")
    ap.add_argument("--frames", type=int, default=4, help="frames fused per detection")
    ap.add_argument("--min-points", type=int, default=4)
    ap.add_argument("--max-rms-mm", type=float, default=8.0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if args.test_detect:
        return test_detect(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
