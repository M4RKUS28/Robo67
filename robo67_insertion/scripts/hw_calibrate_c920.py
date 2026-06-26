#!/usr/bin/env python3
"""hw_calibrate_c920.py -- overhead C920 -> robot-base homography calibration.

Produces ``config/c920_homography.npz`` (the file
``hw_peg_in_hole_vision.py`` / ``socket_detector_node`` need to turn a detected
socket-hole pixel into a robot-base XY). Robot-as-groundtruth (plan Task 2.2 /
Phase 4): drive the EE to N known base-XY points AT THE SOCKET-TOP PLANE, grab a
C920 frame at each, find the marker pixel, then fit + save the homography and
report the reprojection error.

  >>> A homography is PLANE-SPECIFIC. <<<
Every correspondence MUST be taken at the same height the socket hole will sit at
(its TOP face). Pass that height as ``--z`` (base-frame Z, metres). Calibrating
at the wrong Z makes the mapping silently wrong at the socket.

Marker (where the known base point shows up in the image):
  * ``--marker manual`` (default, always works): the tool saves each frame to
    ``--capture-dir`` and you type the pixel ``u v`` you see (open the jpg).
  * ``--marker auto``: auto-detect the marker via the SAME detector used for the
    socket (:func:`~robo67_insertion.lib.hole_detect.detect_white_cubes`).
    Use this if a white cube marker reads cleanly from overhead. Verify with
    ``--image`` first.

USAGE (run INSIDE multipanda-container; see CLAUDE.md runbook)
-------------------------------------------------------------
Verify the fit math offline (NO robot, NO camera, NO ROS):
    python3 scripts/hw_calibrate_c920.py --selftest

Just (re)fit from a correspondences file you already have (CSV rows
``u,v,base_x,base_y`` or .npz with ``pixels``/``base_xy``):
    python3 scripts/hw_calibrate_c920.py --from-file corr.csv --out config/c920_homography.npz

Plan the grid + test detection on a saved still WITHOUT moving the arm:
    python3 scripts/hw_calibrate_c920.py --z 0.10 --dry-run \
        --marker hole --image robo67_insertion/captures/c920_socket_newfloor.jpg

Live capture (gentle, marker on the tool, human at the e-stop):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_calibrate_c920.py --z 0.10 --confirm
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import numpy as np  # noqa: E402

# PURE imports (no rclpy at load): fit math, hole detector, geometry round-trip.
from robo67_insertion.lib import geometry  # noqa: E402
from robo67_insertion.lib.hole_detect import (  # noqa: E402
    WhiteCubeParams,
    detect_white_cubes,
)
from robo67_insertion.nodes.calibration_node import fit_and_save, load_correspondences  # noqa: E402

DEFAULT_OUT = os.path.join(_PKG_ROOT, "config", "c920_homography.npz")
DEFAULT_CONFIG = os.path.join(_PKG_ROOT, "config", "robo67.yaml")
DEFAULT_CAPTURE_DIR = os.path.join(_PKG_ROOT, "captures", "calib")


# ---------------------------------------------------------------------------
# Grid of known base-XY stations to drive the EE to (all at --z).
# ---------------------------------------------------------------------------

def make_grid(box, nx, ny):
    """Return a list of (x, y) stations spanning ``box`` = [xmin,xmax,ymin,ymax]."""
    xmin, xmax, ymin, ymax = box
    xs = np.linspace(xmin, xmax, nx)
    ys = np.linspace(ymin, ymax, ny)
    pts = []
    for j, y in enumerate(ys):
        row = list(xs) if j % 2 == 0 else list(xs[::-1])  # serpentine: short moves
        for x in row:
            pts.append((float(x), float(y)))
    return pts


def build_detector(args):
    """Return ``img -> list[Hole]`` for the marker detector.

    Finds the white-on-dark socket cube centroid (:func:`detect_white_cubes`).
    """
    return lambda img: detect_white_cubes(img, WhiteCubeParams())


# ---------------------------------------------------------------------------
# Marker pixel acquisition for a single station.
# ---------------------------------------------------------------------------

def pixel_from_image(img, detect_fn):
    """Auto: best detected marker pixel in a BGR frame, or None."""
    holes = detect_fn(img)
    if not holes:
        return None
    h = holes[0]
    return (float(h.u), float(h.v))


def pixel_manual(prompt_label):
    """Ask the operator for the marker pixel; '' / 'skip' skips this station."""
    try:
        raw = input(f"  [{prompt_label}] enter marker pixel 'u v' "
                    "(or blank to skip): ").strip()
    except EOFError:
        return None
    if not raw or raw.lower() == "skip":
        return None
    parts = raw.replace(",", " ").split()
    if len(parts) != 2:
        print("    need two numbers 'u v'"); return None
    try:
        return (float(parts[0]), float(parts[1]))
    except ValueError:
        print("    could not parse"); return None


# ---------------------------------------------------------------------------
# Live capture (lazy ROS imports; only reached on a real run).
# ---------------------------------------------------------------------------

def run_capture(args):
    if args.z is None:
        print("ERROR: --z (socket-top plane height, m) is required for capture.",
              file=sys.stderr)
        return 2

    box = list(args.box)
    grid = make_grid(box, args.nx, args.ny)
    print("=== C920 calibration: plan ===")
    print(f"plane Z (socket top): {args.z:.4f} m   <-- correspondences taken here")
    print(f"box (xy)            : x[{box[0]:.3f},{box[1]:.3f}] y[{box[2]:.3f},{box[3]:.3f}]")
    print(f"stations            : {len(grid)} ({args.nx}x{args.ny}), marker={args.marker}")
    for i, (x, y) in enumerate(grid):
        print(f"  #{i:02d}  ({x:.3f}, {y:.3f}, {args.z:.3f})")

    detect_fn = build_detector(args)
    if args.dry_run:
        print("\n[dry-run] not moving the arm.")
        if args.image:
            import cv2
            img = cv2.imread(args.image)
            px = pixel_from_image(img, detect_fn) if img is not None else None
            print(f"[dry-run] cube-detect on {args.image}: pixel={px}")
        return 0

    # ---- real run: lazy-import ROS + the safe mover + camera grab ----
    import rclpy
    from hw_move_to import Mover
    from robo67_insertion.nodes.socket_detector_node import grab_frame_gst

    if args.c920_device is None:
        from robo67_insertion.config_schema import load_config
        # by-id symlink path or bare int index; grab_frame_gst resolves either.
        args.c920_device = load_config(args.config).camera.c920_device

    if args.confirm:
        try:
            if input("Area clear, marker on tool, e-stop in hand? Type YES: ").strip() != "YES":
                print("not confirmed -- exiting without moving."); return 0
        except EOFError:
            return 1

    os.makedirs(args.capture_dir, exist_ok=True)
    rclpy.init()
    mover = Mover(cmd_mode=args.cmd_mode)
    if not mover.wait_state():
        print("ERROR: no robot_state -- aborting.", file=sys.stderr)
        rclpy.shutdown(); return 1
    mover.cmd.detect(timeout=3.0)
    hold_quat = mover.ee_quat
    mover.set_stiffness(args.trans, args.rot)

    pixels, base_xy = [], []
    try:
        for i, (x, y) in enumerate(grid):
            target = np.array([x, y, args.z], float)
            print(f"\n-> station #{i:02d} {target.tolist()}")
            if not mover.move_to(target, hold_quat, speed=args.speed):
                print("   move failed -- skipping station."); continue
            # settle + hold while grabbing
            t0 = time.time()
            while time.time() - t0 < args.settle_s:
                rclpy.spin_once(mover, timeout_sec=0.02)
                mover.publish(target, hold_quat)
            truth = mover.ee_xyz.copy()  # ground-truth base XY = measured EE
            img = grab_frame_gst(args.c920_device, exposure=args.exposure)
            if img is None:
                print("   frame grab failed -- skipping station."); continue
            fpath = os.path.join(args.capture_dir, f"calib_{i:02d}.jpg")
            try:
                import cv2
                cv2.imwrite(fpath, img)
            except Exception:
                fpath = "(unsaved)"

            if args.marker == "auto":
                px = pixel_from_image(img, detect_fn)
                if px is None:
                    print(f"   no marker detected (saved {fpath}); falling back to manual.")
                    px = pixel_manual(f"#{i:02d}")
            else:
                print(f"   frame saved: {fpath}")
                px = pixel_manual(f"#{i:02d}")
            if px is None:
                print("   skipped."); continue
            pixels.append([px[0], px[1]])
            base_xy.append([float(truth[0]), float(truth[1])])
            print(f"   recorded pixel=({px[0]:.1f},{px[1]:.1f}) <- base=({truth[0]:.4f},{truth[1]:.4f})")
    except KeyboardInterrupt:
        print("\ninterrupted -- proceeding to fit with what was collected.")
    finally:
        mover.destroy_node()
        rclpy.shutdown()

    return _fit_save_report(np.array(pixels, float), np.array(base_xy, float),
                            args.out, args.max_rms_mm, args.capture_dir)


# ---------------------------------------------------------------------------
# Fit + save + report (shared by capture and --from-file).
# ---------------------------------------------------------------------------

def _fit_save_report(pixels, base_xy, out_path, max_rms_mm, capture_dir=None):
    n = 0 if pixels.size == 0 else len(pixels)
    print(f"\n=== fit === ({n} correspondences)")
    if n < 4:
        print(f"ERROR: need >= 4 correspondences, have {n} -- nothing saved.",
              file=sys.stderr)
        return 1
    if capture_dir:
        corr_csv = os.path.join(capture_dir, "c920_corr.csv")
        os.makedirs(capture_dir, exist_ok=True)
        # header is '#'-commented so load_correspondences (comments="#") + a
        # subsequent --from-file re-fit can read this file back unchanged.
        np.savetxt(corr_csv, np.hstack([pixels, base_xy]), delimiter=",",
                   header="u,v,base_x,base_y")
        print(f"correspondences saved: {corr_csv}")
    H, rms = fit_and_save(pixels, base_xy, out_path)
    print(f"RMS reprojection error: {rms * 1000:.2f} mm")
    print(f"homography saved       : {out_path}")
    if rms * 1000 > max_rms_mm:
        print(f"WARNING: RMS {rms*1000:.2f} mm > --max-rms-mm {max_rms_mm:.1f}. "
              "Spread stations wider / re-check the marker; consider recapturing.")
        return 1
    return 0


def run_from_file(args):
    # The caller already has the correspondences file, so don't re-save it
    # (capture_dir=None); just fit + report.
    pixels, base_xy = load_correspondences(args.from_file)
    return _fit_save_report(np.asarray(pixels, float), np.asarray(base_xy, float),
                            args.out, args.max_rms_mm, capture_dir=None)


# ---------------------------------------------------------------------------
# Offline self-test: synthesize a known homography, generate correspondences,
# fit, and assert the recovered mapping round-trips. NO ROS / camera / robot.
# ---------------------------------------------------------------------------

def _base_to_pixel(H, base_xy):
    """Inverse of geometry.pixel_to_base: base XY -> pixel (uses H^-1)."""
    Hinv = np.linalg.inv(np.asarray(H, float))
    bxy = np.atleast_2d(base_xy)
    homog = np.concatenate([bxy, np.ones((len(bxy), 1))], axis=1)
    proj = homog @ Hinv.T
    uv = proj[:, :2] / proj[:, 2:3]
    return uv


def selftest(args):
    import tempfile

    print("=== c920 calibration self-test ===")
    # A realistic-ish overhead homography: ~2000 px/m, image-centred.
    H_true = np.array([[1.0 / 2000.0, 0.0, 0.30],
                       [0.0, 1.0 / 2000.0, -0.20],
                       [0.0, 0.0, 1.0]], float)
    grid = make_grid([0.38, 0.58, -0.12, 0.12], 3, 3)
    base_xy = np.array(grid, float)
    pixels = _base_to_pixel(H_true, base_xy)
    rng = np.random.default_rng(0)
    pixels_noisy = pixels + rng.normal(0.0, 0.4, pixels.shape)  # 0.4 px detector noise

    tmp = tempfile.NamedTemporaryFile(suffix=".npz", delete=False)
    tmp.close()
    try:
        H, rms = fit_and_save(pixels_noisy, base_xy, tmp.name)
        reproj = geometry.pixel_to_base(H, pixels_noisy)
        max_err_mm = float(np.max(np.linalg.norm(reproj - base_xy, axis=1))) * 1000.0
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass

    # serpentine ordering keeps consecutive stations close (small, safe moves)
    consecutive = np.max(np.abs(np.diff(np.array(grid), axis=0)), axis=0)
    serpentine_ok = bool(consecutive[1] <= (0.24 / 2) + 1e-9)  # y only steps one row

    print(f"stations           : {len(grid)} (3x3 serpentine)")
    print(f"fit RMS            : {rms*1000:.3f} mm   (0.4 px synthetic noise)")
    print(f"max reproj error   : {max_err_mm:.3f} mm")
    ok = rms * 1000 < 1.0 and max_err_mm < 2.0 and serpentine_ok
    print("RESULT             :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    ap = argparse.ArgumentParser(
        description="Calibrate the overhead C920 -> robot-base homography.")
    ap.add_argument("--selftest", action="store_true",
                    help="offline fit-math test (no ROS, camera, or robot)")
    ap.add_argument("--from-file", default="",
                    help="fit from an existing correspondences CSV/.npz (skip capture)")

    ap.add_argument("--out", default=DEFAULT_OUT, help="output homography .npz")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="robo67 config (for the C920 device number)")
    ap.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR,
                    help="where per-station frames + corr.csv are saved")

    # capture geometry
    ap.add_argument("--z", type=float, default=None,
                    help="socket-TOP plane height in base frame (m); REQUIRED for capture")
    ap.add_argument("--box", type=float, nargs=4, default=[0.38, 0.58, -0.12, 0.12],
                    help="grid region: xmin xmax ymin ymax (m)")
    ap.add_argument("--nx", type=int, default=3, help="grid columns (x)")
    ap.add_argument("--ny", type=int, default=3, help="grid rows (y)")

    # marker / detection
    ap.add_argument("--marker", choices=["manual", "auto"], default="manual",
                    help="manual = operator types the pixel per station (robust); "
                         "auto = detect the white socket cube centroid")
    ap.add_argument("--exposure", type=int, default=100,
                    help="lock C920 manual exposure (~40-120); passed to each grab")
    ap.add_argument("--image", default="",
                    help="(dry-run) test the detector on this still image")
    ap.add_argument("--c920-device", type=str, default=None,
                    help="C920 device: a by-id symlink/path or a bare /dev/video "
                         "index (default: from config)")

    # motion / safety
    ap.add_argument("--dry-run", action="store_true",
                    help="print the planned grid (and optionally test --image); do NOT move")
    ap.add_argument("--confirm", action="store_true", help="prompt YES before motion")
    ap.add_argument("--speed", type=float, default=0.03, help="move speed (m/s)")
    ap.add_argument("--settle-s", type=float, default=1.0, help="hold/settle before grab")
    ap.add_argument("--trans", type=float, default=500.0, help="translational stiffness")
    ap.add_argument("--rot", type=float, default=30.0, help="rotational stiffness")
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")

    ap.add_argument("--max-rms-mm", type=float, default=5.0,
                    help="warn/non-zero exit if reprojection RMS exceeds this")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        return selftest(args)
    if args.from_file:
        return run_from_file(args)
    return run_capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
