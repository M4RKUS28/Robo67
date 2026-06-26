#!/usr/bin/env python3
"""hw_teach_port_offset.py -- TEACH the target port as a box-frame offset.

Why this exists
---------------
A *single* overhead C920 + homography can locate the box but cannot resolve
where on the box face the target port is to sub-cm precision: the homography is
metric only on its calibration plane, and the port panel rides a few cm above
it, so off-nadir parallax adds a ~1-2 cm systematic offset. Measuring the port
with a ruler inherits that error.

So instead we TEACH the port once, through the ROBOT (parallax-free): with the
box in view, hand-guide the EE tip onto the real target port and read its
base-frame XY straight from ``FrankaState``. We store that as an OFFSET in the
BOX'S OWN frame (built from the ORB-matched template corners). At runtime
``hw_cable_insertion_vision.py`` re-detects the box -- which may have been moved
AND rotated -- and re-applies the offset, so the port target follows the box
rigidly (:mod:`robo67_insertion.lib.port_offset`). Any residual is absorbed by
the insertion spiral search.

This tool mirrors ``calibration/calibrate_guided.py``: you move the arm ONLY
with Franka native guiding (grip handles); this script issues NO robot motion --
it only detects the box and reads the EE pose.

USAGE (inside multipanda-container; see CLAUDE.md runbook)
----------------------------------------------------------
Offline self-test (NO ROS, camera, or robot) -- synthetic box + teach/save/load:
    python3 scripts/hw_teach_port_offset.py --selftest

Live teach (default ORB box detector + overhead camera_publisher topic):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_teach_port_offset.py

It writes ``config/port_offset.npz`` (``--out`` to change). After teaching:
    python3 scripts/hw_cable_insertion_vision.py --box-top-z <z> --dry-run
prints the resolved PORT target (it auto-loads ``config/port_offset.npz``).

PREREQUISITE: the same calibrated C920->base homography the box detector uses
(``config/c920_homography.npz``); the box template (``config/box_template.jpg``)
for ORB. Refuses to run without the homography.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_SCRIPTS_DIR)
for _p in (_SCRIPTS_DIR, _PKG_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402

from robo67_insertion.lib.pixel_mapping import HomographyMappingAdapter  # noqa: E402
from robo67_insertion.lib.port_offset import (  # noqa: E402
    box_frame_base,
    port_base_from_box,
    teach_port_offset,
)
# Host-safe (rclpy/cv2 lazy): box detector + corner-aware perception, frame grab.
from hw_cable_insertion_vision import (  # noqa: E402
    DEFAULT_CONFIG,
    DEFAULT_HOMOGRAPHY,
    DEFAULT_PORT_OFFSET,
    DEFAULT_TEMPLATE,
    build_box_detector,
    perceive_box_pose,
)
from hw_peg_in_hole_vision import _grab_frames, load_mapper  # noqa: E402
from robo67_insertion.lib.box_detect import BoxOrbParams, BoxParams  # noqa: E402

ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"


def save_offset(path, offset, *, center, R, corners_base, port_xy, box_top_z):
    """Persist the taught box-frame offset + teach-time diagnostics to ``path``."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez(
        path,
        offset_box=np.asarray(offset, float),
        teach_center=np.asarray(center, float),
        teach_R=np.asarray(R, float),
        teach_corners_base=np.asarray(corners_base, float),
        teach_port_xy=np.asarray(port_xy, float),
        box_top_z=float(box_top_z if box_top_z is not None else np.nan),
    )


# ---------------------------------------------------------------------------
# Live guided teach.
# ---------------------------------------------------------------------------

def run(args):
    if not os.path.exists(args.homography):
        print(f"ERROR: homography not found: {args.homography}", file=sys.stderr)
        print("       Run the C920->base calibration first (see calibration/).",
              file=sys.stderr)
        return 2

    if args.c920_device is None and args.source == "device":
        from robo67_insertion.config_schema import load_config
        args.c920_device = load_config(args.config).camera.c920_device

    mapper = load_mapper(args.homography)
    detect_fn = build_box_detector(args)

    import rclpy
    from franka_msgs.msg import FrankaState
    from robo67_insertion.lib import geometry

    rclpy.init()
    node = rclpy.create_node("hw_teach_port_offset")
    state = {}

    def on_state(m):
        state["xyz"], _ = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        state["mode"] = int(m.robot_mode)
    node.create_subscription(FrankaState, ROBOT_STATE, on_state, 10)

    def fresh_ee(timeout=6.0):
        state.pop("xyz", None)
        t0 = time.time()
        while "xyz" not in state and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        return state.get("xyz"), state.get("mode")

    print("=" * 64)
    print(" Robo67 TEACH port offset (box-frame, jogged truth)")
    print("=" * 64)
    print(" Move the arm ONLY with Franka native guiding (grip handles).")
    print(f" detector={args.method}  out={args.out}")
    z_provided = args.box_top_z is not None  # else taught from the jogged tip Z
    if fresh_ee(8.0)[0] is None:
        print("\nERROR: no FrankaState. The bringup is down (a guide/e-stop can crash "
              "it). Relaunch it, then re-run.", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    rc = 1
    while True:
        try:
            tok = input("\n 1) Place the box in view (arm CLEAR), Enter to detect "
                        "[q=quit, r=retry]: ").strip().lower()
        except EOFError:
            break
        if tok == "q":
            break
        if tok == "r":
            continue

        frames, err = _grab_frames(args)
        if err:
            print(f"     vision: {err}"); continue
        base_xy, corners_base, dets = perceive_box_pose(frames, mapper, detect_fn)
        if base_xy is None:
            print(f"     NO box detected in {len(frames)} frame(s) -- retry."); continue
        center, R = box_frame_base(corners_base)
        yaw_deg = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        best = max(dets, key=lambda d: d[4])
        print(f"     box centre base XY = ({center[0]:.4f}, {center[1]:.4f}) m  "
              f"[{len(dets)}/{len(frames)} frames, score={best[4]:.0f}]")
        print(f"     box yaw (base)     = {yaw_deg:+.1f} deg  (from ORB corners)")
        print(f"     box corners base   = {np.round(corners_base, 4).tolist()}")

        try:
            input(" 2) Hand-guide the EE TIP onto the TARGET PORT (tip touching it), "
                  "then Enter to read the pose: ")
        except EOFError:
            break
        port_xyz, mode = fresh_ee()
        if port_xyz is None:
            print("     no robot state (bringup down?). Relaunch it and retry.",
                  file=sys.stderr); continue
        port_xy = np.array([float(port_xyz[0]), float(port_xyz[1])], float)
        # box-top Z: the overhead camera cannot measure it, so use the jogged tip
        # Z (the tip is touching the port plane) unless --box-top-z was given.
        box_top_z = float(args.box_top_z) if z_provided else float(port_xyz[2])
        z_src = "--box-top-z" if z_provided else "jogged tip Z"

        offset = teach_port_offset(corners_base, port_xy)
        # sanity: re-applying the offset to the SAME corners must return the port.
        back = port_base_from_box(corners_base, offset)
        resid_mm = float(np.linalg.norm(back - port_xy) * 1000.0)
        print(f"     port base XY = ({port_xy[0]:.4f}, {port_xy[1]:.4f}) m  z={port_xyz[2]:.4f}  mode={mode}")
        print(f"     box-top Z    = {box_top_z:.4f} m  (from {z_src})")
        print(f"     box-frame offset = dx={offset[0]*100:+.2f} dy={offset[1]*100:+.2f} cm  "
              f"(round-trip residual {resid_mm:.3f} mm)")

        save_offset(args.out, offset, center=center, R=R, corners_base=corners_base,
                    port_xy=port_xy, box_top_z=box_top_z)
        print(f"     saved -> {args.out}  (offset_box + box_top_z={box_top_z:.4f} + teach diagnostics)")
        print("     Verify (no motion):")
        print(f"       python3 scripts/hw_cable_insertion_vision.py "
              f"--box-top-z {box_top_z:.3f} --dry-run")
        rc = 0
        try:
            again = input(" Teach another (overwrites)? [y/N]: ").strip().lower()
        except EOFError:
            again = "n"
        if again != "y":
            break

    node.destroy_node(); rclpy.shutdown()
    return rc


# ---------------------------------------------------------------------------
# Offline self-test: NO ROS/camera/robot. Synthetic teach -> save -> load -> apply.
# ---------------------------------------------------------------------------

def selftest(args):
    import tempfile

    print("=== teach_port_offset self-test ===")
    # synthetic axis-aligned box (ORB order TL,TR,BR,BL) + a port on its face
    cx, cy, w, h = 0.45, -0.20, 0.14, 0.09
    corners = np.array([[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
                        [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]], float)
    port = np.array([cx + 0.03, cy - 0.02], float)

    offset = teach_port_offset(corners, port)
    ok = np.allclose(offset, [0.03, -0.02], atol=1e-9)  # identity frame -> metric

    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "port_offset.npz")
        save_offset(path, offset, center=corners.mean(0),
                    R=np.eye(2), corners_base=corners, port_xy=port, box_top_z=0.10)
        loaded = np.load(path)["offset_box"]
        ok = ok and np.allclose(loaded, offset)
        # apply the LOADED offset to a MOVED+ROTATED box -> port follows rigidly
        theta = np.deg2rad(30.0)
        Rm = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        t = np.array([0.05, -0.03])
        moved = (Rm @ (corners - corners.mean(0)).T).T + corners.mean(0) + t
        got = port_base_from_box(moved, loaded)
        expected = corners.mean(0) + t + Rm @ (port - corners.mean(0))
        ok = ok and np.allclose(got, expected, atol=1e-9)

    print(f"taught offset      : dx={offset[0]*100:+.2f} dy={offset[1]*100:+.2f} cm")
    print(f"save/load + follow : {'OK' if ok else 'BAD'}")
    print("RESULT             :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def build_parser():
    ap = argparse.ArgumentParser(
        description="Teach the target port as a box-frame offset (jogged truth + ORB box).")
    ap.add_argument("--selftest", action="store_true",
                    help="offline teach/save/load/follow test (no ROS, camera, or robot)")
    ap.add_argument("--out", default=DEFAULT_PORT_OFFSET, help="output port_offset.npz")
    ap.add_argument("--box-top-z", type=float, default=None,
                    help="taught box-top Z (m); default = the jogged tip Z at the port")

    # vision (shared with hw_cable_insertion_vision)
    ap.add_argument("--homography", default=DEFAULT_HOMOGRAPHY)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--image", default="", help="detect on a still image instead of live")
    ap.add_argument("--source", choices=["topic", "device"], default="topic")
    ap.add_argument("--camera-topic",
                    default="/robo67/camera/overhead/image_raw/compressed")
    ap.add_argument("--camera-timeout", type=float, default=6.0)
    ap.add_argument("--c920-device", type=str, default=None)
    ap.add_argument("--exposure", type=int, default=100)
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--method", choices=["orb", "texture"], default="orb",
                    help="orb (default, identity-preserving corners) is recommended for "
                         "the box-frame offset; texture corners are not rotation-stable")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE)
    ap.add_argument("--orb-min-inliers", type=int, default=BoxOrbParams.min_inliers)
    ap.add_argument("--fallback-texture", action=argparse.BooleanOptionalAction, default=False,
                    help="off by default: a texture fallback scrambles the box frame")
    ap.add_argument("--min-texture-std", type=float, default=BoxParams.min_texture_std)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        return selftest(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
