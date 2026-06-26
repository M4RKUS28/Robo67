#!/usr/bin/env python3
"""hw_cable_insertion_vision.py -- cable-into-port insertion on the real arm.

Cable-task twin of ``hw_peg_in_hole_vision.py``. It owns ONLY the vision
front-end + CLI and reuses the proven motion/insertion primitives:

WORKFLOW (see docs/superpowers/plans/2026-06-26-cable-insertion.md)
------------------------------------------------------------------
1. PERCEIVE BOX (overhead C920) -- grab overhead frame(s), detect the dark
   port-covered I/O box (:func:`~robo67_insertion.lib.box_detect.detect_gray_box`,
   a local-texture-energy detector), and map the best detection's centroid pixel
   -> robot-base XY through the calibrated homography
   (:class:`~robo67_insertion.lib.pixel_mapping.HomographyMappingAdapter`). The
   per-axis MEDIAN base XY over several frames is used for robustness. The box
   top Z cannot come from a single overhead camera, so it is taught via
   ``--box-top-z``.
2. MOVE ABOVE (this script's job) -- command the arm to a tool-down pose
   ``--approach-height`` (default 10 cm) ABOVE the perceived box center, reusing
   the gentle two-phase RAMP+SETTLE move primitive
   (:class:`scripts.hw_move_to.Mover`). It then HOLDS there.

Later phases (NOT done here) hand off to the wrist D405 to localize the exact
port and run the force-compliant seat -- see the plan. This script stops once it
is hovering above the box, the clean hand-off point for the wrist stage.

USAGE (run INSIDE multipanda-container; see CLAUDE.md runbook)
-------------------------------------------------------------
Offline self-test (NO ROS, camera, or robot) -- synthetic box + demo homography:
    python3 scripts/hw_cable_insertion_vision.py --selftest

Dry run on the real arm (perceives the box, computes the above-box target,
publishes NOTHING -- always do this first):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_cable_insertion_vision.py --box-top-z 0.10 --dry-run

Live move-above (gentle, human at the e-stop):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_cable_insertion_vision.py --box-top-z 0.10 --confirm

PREREQUISITE: a calibrated C920->base homography ``c920_homography.npz`` (same
one the peg-in-hole socket uses). If it is missing this script refuses to run.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

# Make sibling scripts (hw_move_to, hw_peg_in_hole_vision, hw_cmd_iface) and the
# ``robo67_insertion`` package importable without PYTHONPATH set.
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_SCRIPTS_DIR)
for _p in (_SCRIPTS_DIR, _PKG_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402

# Pure (numpy/cv2/stdlib -- NO rclpy at import). rclpy/the Mover are imported
# lazily inside run_live so --selftest stays ROS-free.
from robo67_insertion.lib.box_detect import (  # noqa: E402
    BoxOrbParams,
    BoxParams,
    OrbBoxMatcher,
    detect_gray_box,
)
from robo67_insertion.lib.pixel_mapping import (  # noqa: E402
    HomographyMappingAdapter,
    MappingContext,
    PixelObservation,
)
from robo67_insertion.lib.port_offset import (  # noqa: E402
    box_frame_base,
    map_corners_to_base,
    port_base_from_box,
)
# Reuse the host-safe frame-grab + homography loader from the peg runner (their
# rclpy/cv2 imports are lazy, so importing this module pulls in no ROS), and the
# proven real-arm insertion loop for the optional SEAT hand-off (its rclpy is
# lazy inside run_ros, so a bare import stays ROS-free).
from hw_peg_in_hole_vision import _grab_frames, load_mapper  # noqa: E402
from robo67_insertion.nodes import hardware_insertion_node as hin  # noqa: E402


DEFAULT_HOMOGRAPHY = os.path.join(_PKG_ROOT, "config", "c920_homography.npz")
DEFAULT_CONFIG = os.path.join(_PKG_ROOT, "config", "robo67.yaml")
DEFAULT_TEMPLATE = os.path.join(_PKG_ROOT, "config", "box_template.jpg")
DEFAULT_PORT_OFFSET = os.path.join(_PKG_ROOT, "config", "port_offset.npz")


def load_port_offset(path):
    """Load a taught box-frame port offset ``(dx, dy)`` (m) from ``path``.

    Saved by ``scripts/hw_teach_port_offset.py``. Returns the ``(2,)`` offset, or
    ``None`` if ``path`` is falsy / missing (caller then targets the box centre).
    """
    if not path or not os.path.exists(path):
        return None
    data = np.load(path)
    return np.asarray(data["offset_box"], float).reshape(2)


# ---------------------------------------------------------------------------
# Vision front-end (pure: no ROS). Detect the box and map its centroid to base.
# ---------------------------------------------------------------------------

def build_box_detector(args):
    """Return a callable ``img -> list[Box]`` for the chosen box detector.

    ``orb`` (default) matches the stored reference TEMPLATE of this specific I/O
    box (robust to clutter/position/rotation; rejects distractors). ``texture``
    is the busiest-blob heuristic. With ORB, fall back to texture only if
    ``--fallback-texture`` and ORB finds nothing.
    """
    tparams = BoxParams(min_texture_std=args.min_texture_std)
    if args.method == "orb":
        import cv2
        tmpl = cv2.imread(args.template)
        if tmpl is None:
            print(f"WARNING: box template not found: {args.template} -- using texture detector",
                  file=sys.stderr)
            return lambda img: detect_gray_box(img, tparams)
        matcher = OrbBoxMatcher(tmpl, BoxOrbParams(min_inliers=args.orb_min_inliers))

        def _detect(img):
            boxes = matcher.detect(img)
            if boxes or not args.fallback_texture:
                return boxes
            return detect_gray_box(img, tparams)
        return _detect
    return lambda img: detect_gray_box(img, tparams)


def perceive_box_pose(frames, mapper, detect_fn):
    """Detect the box per frame and map its centroid AND corner quad to base.

    Returns ``(base_xy (2,), corners_base (4, 2), detections)`` where the centroid
    and the four ORB-quad corners are each per-element MEDIAN-fused over the
    frames that yielded a detection, or ``(None, None, [])`` if none did.
    ``detections`` lists ``(u, v, w_px, h_px, score, base_x, base_y)`` per frame.

    ``corners_base`` is in the detector's corner order; with the default ORB
    matcher this uses ``Box.template_corners`` -- the projected template quad
    ``[TL, TR, BR, BL]`` (identity-preserving across rotations) which the
    box-frame port offset relies on -- NOT the tight ``Box.corners`` (a
    ``minAreaRect`` quad whose corner identity is not stable under rotation).
    """
    detections = []
    corner_sets = []

    def _map(u, v):
        return mapper.map_xy(PixelObservation(u, v), MappingContext())

    for img in frames:
        boxes = detect_fn(img)
        if not boxes:
            continue
        b = boxes[0]  # already sorted best (densest * largest) first
        bx, by = _map(b.u, b.v)
        detections.append((b.u, b.v, b.width_px, b.height_px, b.score,
                           float(bx), float(by)))
        # identity-preserving template quad for the box frame; fall back to the
        # tight quad only if absent (e.g. texture detector).
        frame_quad = b.template_corners if b.template_corners is not None else b.corners
        corner_sets.append(map_corners_to_base(frame_quad, _map))
    if not detections:
        return None, None, []
    base_xy = np.array([
        statistics.median(d[5] for d in detections),
        statistics.median(d[6] for d in detections),
    ], float)
    corners_base = np.median(np.stack(corner_sets, axis=0), axis=0)
    return base_xy, corners_base, detections


def perceive_box_xy(frames, mapper, detect_fn):
    """Back-compat shim: ``(base_xy, detections)`` (drops the corner quad)."""
    base_xy, _corners, dets = perceive_box_pose(frames, mapper, detect_fn)
    return base_xy, dets


def above_box_target(base_xy, box_top_z, approach_height):
    """The tool-down target ``(x, y, z)`` ``approach_height`` ABOVE the box top."""
    return np.array([float(base_xy[0]), float(base_xy[1]),
                     float(box_top_z) + float(approach_height)], float)


def _base_to_pixel(H, base_xy):
    """Inverse of the C920 homography: base XY -> image pixel (u, v)."""
    Hinv = np.linalg.inv(np.asarray(H, float))
    p = Hinv @ np.array([float(base_xy[0]), float(base_xy[1]), 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def save_port_overlay(path, frame, boxes, mapper, port_xy, base_xy, *,
                      used_offset, box_yaw_deg):
    """Annotate ``frame`` with the detected box + the RESOLVED PORT and save it.

    Draws the box overlay (oriented quad + centroid + base XY), then maps the
    resolved port base XY back to a pixel (inverse homography) and marks it with
    a red crosshair + circle + label. Returns the saved path (or None on error).
    """
    import cv2
    from robo67_insertion.lib.image_overlay import draw_box_overlay

    out = draw_box_overlay(frame, boxes, base_xy=(float(base_xy[0]), float(base_xy[1])))
    try:
        pu, pv = _base_to_pixel(mapper.H, port_xy)
    except Exception as e:  # pragma: no cover - degenerate homography
        print(f"WARNING: could not map port to pixel: {e}", file=sys.stderr)
        pu = pv = None
    if pu is not None:
        red = (0, 0, 255)
        u, v = int(round(pu)), int(round(pv))
        cv2.drawMarker(out, (u, v), red, cv2.MARKER_CROSS, 26, 2, cv2.LINE_AA)
        cv2.circle(out, (u, v), 10, red, 2, cv2.LINE_AA)
        tag = "PORT" if used_offset else "BOX-CENTRE"
        cv2.putText(out, f"{tag} ({port_xy[0]:+.3f},{port_xy[1]:+.3f}) yaw{box_yaw_deg:+.0f}",
                    (u + 14, v + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, red, 2, cv2.LINE_AA)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    ok = cv2.imwrite(path, out)
    return path if ok else None


def resolve_port_xy(base_xy, corners_base, offset):
    """Return the target XY: the taught port (if an ``offset`` is given) else the
    box centre. With an ``offset`` the port follows the box's measured pose
    (translation + rotation) via :func:`port_base_from_box`.
    """
    if offset is None:
        return np.asarray(base_xy, float), False
    return np.asarray(port_base_from_box(corners_base, offset), float), True


# ---------------------------------------------------------------------------
# SEAT hand-off: feed the port XYZ to the proven real-arm insertion loop, kept
# GRIPPED (no --release-on-insert). The insertion node owns approach/descend/
# spiral/push-seat + the full safety envelope; this script only supplies the
# target and the seat-tuned, bounded force knobs.
# ---------------------------------------------------------------------------

def build_insertion_args(args, port_xyz):
    """Build the ``hardware_insertion_node`` arg namespace for a SEAT-while-gripped
    run at ``port_xyz``. Mirrors the peg runner's hand-off but with NO gripper
    release: the cable connector stays clamped and is pushed to seat."""
    ns = hin.build_parser().parse_args([])
    ns.selftest = False
    ns.nudge = None
    ns.socket_from_current = False
    ns.socket_xyz = [float(port_xyz[0]), float(port_xyz[1]), float(port_xyz[2])]

    ns.dry_run = args.dry_run
    ns.dry_run_seconds = args.dry_run_seconds
    ns.confirm = args.confirm
    ns.countdown = args.countdown

    ns.topic = args.topic
    ns.state_topic = args.state_topic
    ns.recovery_srv = args.recovery_srv

    ns.rate = args.rate
    ns.v_max = args.v_max
    ns.standoff = args.standoff
    ns.pos_stiff = args.pos_stiff
    ns.approach_tol = args.approach_tol
    ns.contact_fz = args.contact_fz
    ns.press_force = args.press_force
    ns.insert_press = args.insert_press
    ns.max_press_depth = args.max_press_depth
    ns.insert_depth = args.insert_depth
    ns.spiral_max_radius = args.spiral_max_radius

    ns.f_abort = args.f_abort
    ns.torque_abort = args.torque_abort
    ns.watchdog_s = args.watchdog_s
    ns.workspace_aabb = list(args.workspace_aabb)

    # SEAT while gripped: explicitly DO NOT release (peg task opens the gripper
    # on the z-drop; the cable connector must stay clamped and be pushed home).
    ns.release_on_insert = False
    return ns


# ---------------------------------------------------------------------------
# Live: perceive the box, then move the arm above it (reusing hw_move_to.Mover).
# ---------------------------------------------------------------------------

def _move_above(target, args):
    """Drive the real arm to ``target`` (tool-down) and hold. Returns rc int."""
    import rclpy
    from hw_move_to import Mover, tool_down_quat  # rclpy-importing; lazy on purpose

    rclpy.init()
    n = Mover(cmd_mode=args.cmd_mode)
    try:
        if not n.wait_state():
            print("ERROR: no robot_state -- refusing to move.", file=sys.stderr)
            return 1
        n.cmd.detect(timeout=3.0)
        hold_quat = tool_down_quat(n.ee_quat)  # vertical, preserve current yaw
        print(f"[cable] start ee={[round(v,4) for v in n.ee_xyz]} "
              f"cmd_mode={n.cmd.mode} -> above-box {[round(v,4) for v in target]}")
        n.set_stiffness(args.trans, args.rot)
        ok = n.move_to(np.asarray(target, float), hold_quat, speed=args.speed,
                       tol=args.tol)
        # Hold the LAST commanded equilibrium (includes the overshoot that keeps
        # the EE on target) so the soft arm does not relax back when we exit.
        hold_xyz = n.last_cmd_xyz.copy() if n.last_cmd_xyz is not None else n.ee_xyz.copy()
        t0 = time.time()
        while time.time() - t0 < args.hold_after_s:
            rclpy.spin_once(n, timeout_sec=0.02)
            n.publish(hold_xyz, hold_quat)
        print(f"[cable] above-box reached={ok} final ee={[round(v,4) for v in n.ee_xyz]} "
              f"mode={n.mode}")
        return 0 if ok else 1
    finally:
        n.destroy_node()
        rclpy.shutdown()


def run_live(args):
    if not os.path.exists(args.homography):
        print(f"ERROR: homography not found: {args.homography}", file=sys.stderr)
        print("       Run the C920->base calibration first (see calibration/).",
              file=sys.stderr)
        return 2

    if args.c920_device is None:
        from robo67_insertion.config_schema import load_config
        args.c920_device = load_config(args.config).camera.c920_device

    mapper = load_mapper(args.homography)
    aabb = np.array(args.workspace_aabb, float).reshape(3, 2)
    offset = load_port_offset(args.port_offset)

    # The box-frame offset relies on ORB's identity-ordered template corners to
    # recover the box ROTATION. The texture detector's corners are NOT
    # rotation-stable, so a silent ORB->texture fallback would scramble the frame
    # and send the arm to the wrong place. With an offset loaded, force ORB-only.
    if offset is not None:
        if args.method != "orb":
            print("NOTE: port offset loaded -> forcing --method orb (the box-frame "
                  "rotation needs the ORB template corners).", file=sys.stderr)
            args.method = "orb"
        if not os.path.exists(args.template):
            print(f"ERROR: port offset needs the ORB box template (identity-preserving "
                  f"corners); not found: {args.template} -- refusing (texture corners "
                  "would scramble the box rotation frame).", file=sys.stderr)
            return 2
        args.fallback_texture = False

    detect_fn = build_box_detector(args)
    frames, err = _grab_frames(args)
    if err:
        print(f"ERROR: vision: {err} -- refusing to move.", file=sys.stderr)
        return 1
    base_xy, corners_base, dets = perceive_box_pose(frames, mapper, detect_fn)
    if base_xy is None:
        print(f"ERROR: no I/O box detected in {len(frames)} frame(s) "
              "-- refusing to move.", file=sys.stderr)
        return 1

    port_xy, used_offset = resolve_port_xy(base_xy, corners_base, offset)
    port_xyz = np.array([port_xy[0], port_xy[1], args.box_top_z], float)
    target = above_box_target(port_xy, args.box_top_z, args.approach_height)

    print("=== perceived box ===")
    print(f"frames used        : {len(dets)}/{len(frames)} with a detection")
    best = max(dets, key=lambda d: d[4])
    print(f"best pixel (u,v)    : ({best[0]:.1f}, {best[1]:.1f})  "
          f"size={best[2]:.0f}x{best[3]:.0f}px score={best[4]:.0f}")
    print(f"box XY (median)     : x={base_xy[0]:.4f} y={base_xy[1]:.4f} m")
    _, R_box = box_frame_base(corners_base)
    box_yaw_deg = float(np.degrees(np.arctan2(R_box[1, 0], R_box[0, 0])))
    print(f"box yaw (base)      : {box_yaw_deg:+.1f} deg (from ORB corners; rotates the offset)")
    if used_offset:
        print(f"port offset (box fr): dx={offset[0]*100:+.2f} dy={offset[1]*100:+.2f} cm "
              f"(taught: {args.port_offset})")
        print(f"port XY (target)    : x={port_xy[0]:.4f} y={port_xy[1]:.4f} m")
    else:
        print("port offset         : NONE -- targeting the box CENTRE "
              "(run hw_teach_port_offset.py to aim at a port)")
    print(f"box top Z (taught)  : z={args.box_top_z:.4f} m")
    print(f"above-box target    : x={target[0]:.4f} y={target[1]:.4f} z={target[2]:.4f} m "
          f"(+{args.approach_height:.3f} m)")

    if args.save_overlay:
        boxes = detect_fn(frames[-1])
        saved = save_port_overlay(args.save_overlay, frames[-1], boxes, mapper,
                                  port_xy, base_xy, used_offset=used_offset,
                                  box_yaw_deg=box_yaw_deg)
        print(f"overlay saved       : {saved if saved else 'FAILED to write ' + args.save_overlay}")

    if not np.all((target >= aabb[:, 0]) & (target <= aabb[:, 1])):
        print(f"ERROR: above-box target {target.tolist()} outside workspace AABB "
              f"{aabb.tolist()} -- refusing to move.", file=sys.stderr)
        return 1
    if not np.all((port_xyz >= aabb[:, 0]) & (port_xyz <= aabb[:, 1])):
        print(f"ERROR: port target {port_xyz.tolist()} outside workspace AABB "
              f"{aabb.tolist()} -- refusing to move.", file=sys.stderr)
        return 1

    # SEAT: hand the port XYZ to the insertion loop (it owns approach/descend/
    # spiral/push-seat + confirm + dry-run). Kept gripped (no release).
    if args.insert:
        ins_args = build_insertion_args(args, port_xyz)
        print(f"handing PORT to the insertion loop for SEAT-while-gripped "
              f"({'DRY-RUN' if args.dry_run else 'LIVE'}) ...")
        return hin.run_ros(ins_args)

    # Otherwise: just hover above the target (perception/positioning check).
    if args.dry_run:
        print("DRY-RUN: would move above the box (publishing NOTHING).")
        return 0

    if args.confirm:
        try:
            if input("Area clear, e-stop in hand? Type YES to move above the box: ").strip() != "YES":
                print("not confirmed -- exiting without moving.")
                return 0
        except EOFError:
            return 1

    return _move_above(target, args)


# ---------------------------------------------------------------------------
# Offline self-test: NO ROS, camera, or robot. Synthetic box + demo homography.
# ---------------------------------------------------------------------------

def _synthetic_box_image(cx=700, cy=560, bw=240, bh=150, seed=7):
    """Uniform mid-gray 'carpet' with a HIGH-texture rectangle (the I/O box face)."""
    rng = np.random.default_rng(seed)
    img = np.clip(np.full((720, 1280), 116.0) + rng.normal(0.0, 4.0, (720, 1280)),
                  0, 255).astype(np.uint8)
    patch = rng.integers(0, 256, size=(bh, bw), dtype=np.uint8)
    img[cy - bh // 2:cy + bh // 2, cx - bw // 2:cx + bw // 2] = patch
    return np.dstack([img, img, img])  # BGR (gray detector ignores colour)


def _demo_homography(cx=700, cy=560):
    """Scale+offset homography: pixel (cx, cy) -> base (0.45, 0.0)."""
    sx = sy = 5.0e-4
    return np.array([[sx, 0.0, 0.45 - cx * sx],
                     [0.0, sy, 0.00 - cy * sy],
                     [0.0, 0.0, 1.0]], float)


def selftest(args):
    print("=== cable_insertion_vision self-test ===")
    cx, cy = 700, 560
    mapper = HomographyMappingAdapter(_demo_homography(cx, cy))
    img = _synthetic_box_image(cx, cy)
    # The synthetic image is a generic texture blob, not the real box, so the
    # offline pipeline check uses the texture detector regardless of --method.
    args.method = "texture"
    base_xy, corners_base, dets = perceive_box_pose([img] * 3, mapper, build_box_detector(args))

    ok = base_xy is not None and len(dets) == 3 and corners_base is not None
    if ok:
        u, v = dets[0][0], dets[0][1]
        ok = abs(u - cx) < 25 and abs(v - cy) < 25
    if base_xy is not None:
        aabb = np.array(args.workspace_aabb, float).reshape(3, 2)

        # box-CENTRE target (no offset)
        target = above_box_target(base_xy, args.box_top_z, args.approach_height)
        inside = bool(np.all((target >= aabb[:, 0]) & (target <= aabb[:, 1])))
        ok = ok and inside and abs(target[2] - (args.box_top_z + args.approach_height)) < 1e-9
        print(f"box detected       : pixel=({dets[0][0]:.1f},{dets[0][1]:.1f}) "
              f"-> base XY=({base_xy[0]:.4f},{base_xy[1]:.4f}) m")
        print(f"above-box target   : ({target[0]:.4f},{target[1]:.4f},{target[2]:.4f}) "
              f"inside_aabb={inside}")

        # PORT target via a synthetic box-frame offset (+3 cm box-x, -2 cm box-y):
        # it must shift the target off-centre and still wire cleanly into the
        # SEAT-while-gripped insertion namespace (socket = port, no release).
        synth_offset = np.array([0.03, -0.02], float)
        port_xy, used = resolve_port_xy(base_xy, corners_base, synth_offset)
        port_xyz = np.array([port_xy[0], port_xy[1], args.box_top_z], float)
        moved = bool(np.linalg.norm(port_xy - base_xy) > 1e-6)
        ins = build_insertion_args(args, port_xyz)
        wired_ok = (used and moved
                    and ins.socket_xyz[:2] == [float(port_xyz[0]), float(port_xyz[1])]
                    and ins.release_on_insert is False
                    and ins.nudge is None and not ins.socket_from_current)
        # round-trip: with NO offset, resolve_port_xy returns the box centre.
        ctr_xy, used_none = resolve_port_xy(base_xy, corners_base, None)
        wired_ok = wired_ok and (not used_none) and np.allclose(ctr_xy, base_xy)
        ok = ok and wired_ok
        print(f"port via offset    : base XY=({port_xy[0]:.4f},{port_xy[1]:.4f}) m "
              f"moved_off_centre={moved}")
        print(f"seat wiring        : socket={[round(v,4) for v in ins.socket_xyz]} "
              f"release_on_insert={ins.release_on_insert} -> {'OK' if wired_ok else 'BAD'}")
    else:
        print("box detected       : NONE")
        ok = False
    print("RESULT             :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    ap = argparse.ArgumentParser(
        description="Perceive the I/O box (overhead C920) and move the real arm above it.")
    ap.add_argument("--selftest", action="store_true",
                    help="offline vision + target test (no ROS, camera, or robot)")

    # vision
    ap.add_argument("--homography", default=DEFAULT_HOMOGRAPHY,
                    help="path to c920_homography.npz from calibration")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="robo67 config (used for the C920 device in --source device)")
    ap.add_argument("--image", default="",
                    help="detect on this still image instead of the live C920")
    ap.add_argument("--source", choices=["topic", "device"], default="topic",
                    help="topic (default) = subscribe to the camera_publisher feed; "
                         "device = open the C920 directly")
    ap.add_argument("--camera-topic",
                    default="/robo67/camera/overhead/image_raw/compressed",
                    help="overhead camera CompressedImage topic (--source topic)")
    ap.add_argument("--camera-timeout", type=float, default=6.0,
                    help="seconds to wait for frames on --camera-topic before giving up")
    ap.add_argument("--c920-device", type=str, default=None,
                    help="overhead C920 device for --source device (default: from config)")
    ap.add_argument("--exposure", type=int, default=100,
                    help="lock C920 manual exposure for --source device")
    ap.add_argument("--frames", type=int, default=5,
                    help="number of frames to fuse (per-axis median)")
    ap.add_argument("--method", choices=["orb", "texture"], default="orb",
                    help="orb (default) = match the stored box template (robust to clutter); "
                         "texture = busiest-blob heuristic")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE,
                    help="reference box template for --method orb")
    ap.add_argument("--orb-min-inliers", type=int, default=BoxOrbParams.min_inliers,
                    help="min RANSAC inliers to accept an ORB match (else box absent)")
    ap.add_argument("--fallback-texture", action=argparse.BooleanOptionalAction, default=False,
                    help="off by default: the object-agnostic texture detector will pick the "
                         "busiest distractor as the box when the real box is absent (a wrong "
                         "detection is worse than none for insertion). Opt in only if needed.")
    ap.add_argument("--min-texture-std", type=float, default=BoxParams.min_texture_std,
                    help="texture detector local-std threshold (busy port face vs carpet)")
    ap.add_argument("--box-top-z", type=float, default=None,
                    help="taught box-top Z in base frame (m); REQUIRED for a live/dry run")
    ap.add_argument("--approach-height", type=float, default=0.10,
                    help="height above the box top to hover at (m)")
    ap.add_argument("--save-overlay", default="",
                    help="write an annotated frame (detected box + RESOLVED PORT marked) "
                         "to this path for inspection; e.g. captures/port_overlay.jpg")

    # port targeting (box-frame taught offset; see hw_teach_port_offset.py)
    ap.add_argument("--port-offset", default=DEFAULT_PORT_OFFSET,
                    help="taught box-frame port offset npz; if present the target is the "
                         "PORT (follows the box pose), else the box CENTRE. Pass '' to "
                         "force box-centre targeting.")

    # motion (forwarded to hw_move_to.Mover)
    ap.add_argument("--speed", type=float, default=0.015, help="command speed (m/s)")
    ap.add_argument("--tol", type=float, default=0.010, help="reach tolerance (m)")
    ap.add_argument("--trans", type=float, default=500.0,
                    help="translational stiffness (mmc/sim only; subscriber path is fixed)")
    ap.add_argument("--rot", type=float, default=30.0, help="rotational stiffness (mmc/sim)")
    ap.add_argument("--hold-after-s", type=float, default=2.0,
                    help="seconds to hold the above-box equilibrium before exiting")
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")

    # SEAT hand-off: when --insert, the perceived PORT XYZ is handed to
    # hardware_insertion_node.run_ros (approach/descend/spiral/push-seat), kept
    # GRIPPED (no release). Defaults mirror the verified peg set but with a small
    # bounded seat push -- TUNE on the real arm (Phase 5).
    ap.add_argument("--insert", action="store_true",
                    help="after perceiving the port, run the SEAT-while-gripped insertion "
                         "loop (else just hover above the target)")
    ap.add_argument("--dry-run-seconds", type=float, default=20.0)
    ap.add_argument("--countdown", type=int, default=3)
    ap.add_argument("--topic", default="/cartesian_impedance/pose_desired")
    ap.add_argument("--state-topic", default="/franka_robot_state_broadcaster/robot_state")
    ap.add_argument("--recovery-srv",
                    default="/panda_error_recovery_service_server/error_recovery")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--v-max", type=float, default=0.03, help="insertion command speed cap (m/s)")
    ap.add_argument("--standoff", type=float, default=0.05,
                    help="height above the port the insertion approaches to (m)")
    ap.add_argument("--pos-stiff", type=float, default=2000.0,
                    help="MUST match the running controller's translational stiffness")
    ap.add_argument("--approach-tol", type=float, default=0.015)
    ap.add_argument("--contact-fz", type=float, default=4.0)
    ap.add_argument("--press-force", type=float, default=18.0)
    ap.add_argument("--insert-press", type=float, default=10.0,
                    help="bounded seat push force (N); keep modest -- a sustained hard "
                         "push trips the firmware reflex on the soft controller")
    ap.add_argument("--max-press-depth", type=float, default=0.02)
    ap.add_argument("--insert-depth", type=float, default=0.008,
                    help="small commanded seat depth (m) for the gripped connector")
    ap.add_argument("--spiral-max-radius", type=float, default=0.02)
    ap.add_argument("--torque-abort", type=float, default=10.0,
                    help="hard external-moment cap (Nm) -- the seat-while-gripped guard")
    ap.add_argument("--f-abort", type=float, default=20.0)
    ap.add_argument("--watchdog-s", type=float, default=0.25)

    # run / safety
    ap.add_argument("--dry-run", action="store_true",
                    help="perceive box + compute target but publish NOTHING")
    ap.add_argument("--confirm", action="store_true", help="prompt YES before motion")
    ap.add_argument("--workspace-aabb", type=float, nargs=6,
                    default=[0.20, 0.65, -0.45, 0.45, 0.06, 0.55],
                    help="xmin xmax ymin ymax zmin zmax (m); pre-move sanity check")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        if args.box_top_z is None:
            args.box_top_z = 0.10  # representative value for the offline check
        return selftest(args)
    if args.box_top_z is None:
        print("ERROR: --box-top-z is required for a live/dry run (an overhead "
              "camera cannot measure box-top Z).", file=sys.stderr)
        return 2
    return run_live(args)


if __name__ == "__main__":
    raise SystemExit(main())
