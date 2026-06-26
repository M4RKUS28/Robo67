#!/usr/bin/env python3
"""hw_calibrate_socket_proxy.py -- C920->base homography via the SOCKET as the
robot-as-groundtruth marker, HAND-GUIDED (you move the compliant arm).

How it works
------------
The overhead detector (:func:`~robo67_insertion.lib.hole_detect.detect_white_cubes`)
finds the WHITE socket, not the tool, so we use the socket itself as the
ground-truth marker. Per station you seat the peg tip in the socket bore (the
hole self-centres it -> EE XY == socket XY), record, then move the arm clear so
the camera sees the socket and we detect its pixel. >= 4 socket positions -> fit.

COMPLIANCE: the real subscriber controller has a fixed firm stiffness that can't
be lowered, so we float the arm the hw_handguide way -- continuously stream the
equilibrium at the *current measured* EE at 50 Hz. The spring force then keeps
resetting and the arm moves freely by hand. (This needs the robot in MOVE mode;
if it's wedged in Idle after a reflex, relaunch the bringup first.) Collision
thresholds are loosened while guiding and restored (firm hold) at the end, so a
hand push won't trip a reflex.

USAGE (INSIDE multipanda-container; controller ACTIVE) -- normally via
``./start_calibration``:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_calibrate_socket_proxy.py

Per station the tool prompts:
    1. "Seat the peg tip in the socket bore, then Enter"   -> records EE
    2. "Move the arm CLEAR of the socket, then Enter"      -> detects the socket
Type 'q' then Enter to finish and fit; 'r' to skip a station.

Offline fit/detection self-test (no ROS, camera, or robot):
    python3 scripts/hw_calibrate_socket_proxy.py --selftest
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import numpy as np  # noqa: E402

from robo67_insertion.lib import geometry  # noqa: E402
from robo67_insertion.lib.hole_detect import WhiteCubeParams, detect_white_cubes  # noqa: E402
from robo67_insertion.nodes.calibration_node import fit_and_save  # noqa: E402

DEFAULT_OUT = os.path.join(_PKG_ROOT, "config", "c920_homography.npz")
DEFAULT_CONFIG = os.path.join(_PKG_ROOT, "config", "robo67.yaml")
DEFAULT_CAPTURE_DIR = os.path.join(_PKG_ROOT, "captures", "calib")


# ---------------------------------------------------------------------------
# Vision helpers (pure).
# ---------------------------------------------------------------------------

def detect_socket_pixel(frames, params):
    """Best socket pixel (median u,v) over frames that yielded a detection."""
    us, vs = [], []
    for img in frames:
        holes = detect_white_cubes(img, params)
        if holes:
            us.append(holes[0].u)
            vs.append(holes[0].v)
    if not us:
        return None, 0
    return (float(np.median(us)), float(np.median(vs))), len(us)


# ---------------------------------------------------------------------------
# Offline self-test (no ROS / camera / robot).
# ---------------------------------------------------------------------------

def _base_to_pixel(H, base_xy):
    Hinv = np.linalg.inv(np.asarray(H, float))
    bxy = np.atleast_2d(base_xy)
    homog = np.concatenate([bxy, np.ones((len(bxy), 1))], axis=1)
    proj = homog @ Hinv.T
    return proj[:, :2] / proj[:, 2:3]


def selftest(_args):
    import tempfile

    import cv2

    print("=== socket-proxy calibration self-test ===")

    def cube(bore):
        img = np.full((480, 640, 3), 35, np.uint8)
        cv2.rectangle(img, (290, 210), (350, 270), (252, 252, 252), -1)
        if bore:
            cv2.circle(img, (320, 240), 20, (150, 150, 150), 2)
            cv2.circle(img, (320, 240), 18, (205, 205, 205), -1)
        return img

    px, _ = detect_socket_pixel([cube(True)] * 3, WhiteCubeParams())
    det_ok = px is not None and abs(px[0] - 320) < 6 and abs(px[1] - 240) < 6
    # detect_white_cubes keys on the white cube body (not the bore), so an empty
    # dark scene must yield no detection.
    empty = np.full((480, 640, 3), 35, np.uint8)
    det_ok = det_ok and detect_socket_pixel([empty] * 3, WhiteCubeParams())[0] is None

    base = np.array([(0.40, -0.10), (0.50, -0.10), (0.45, 0.0),
                     (0.40, 0.10), (0.50, 0.10), (0.55, 0.0)], float)
    H_true = np.array([[1 / 2000.0, 0.0, 0.30],
                       [0.0, 1 / 2000.0, -0.20],
                       [0.0, 0.0, 1.0]], float)
    pix = _base_to_pixel(H_true, base) + np.random.default_rng(0).normal(0, 0.4, base.shape)
    tmp = tempfile.NamedTemporaryFile(suffix=".npz", delete=False); tmp.close()
    try:
        H, rms = fit_and_save(pix, base, tmp.name)
        reproj = geometry.pixel_to_base(H, pix)
        max_err_mm = float(np.max(np.linalg.norm(reproj - base, axis=1))) * 1000.0
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass

    print(f"detector check     : {'PASS' if det_ok else 'FAIL'}")
    print(f"fit RMS            : {rms*1000:.3f} mm   max reproj {max_err_mm:.3f} mm")
    ok = det_ok and rms * 1000 < 1.0 and max_err_mm < 2.0
    print("RESULT             :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# Live hand-guided capture (lazy ROS imports).
# ---------------------------------------------------------------------------

def _key():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.readline().strip().lower() if r else None


def wait_key_reading(node, prompt):
    """Wait for Enter while SPINNING (so ee_xyz stays fresh) but WITHOUT streaming
    any command -- so the operator can use Franka's native guiding (grip handles)
    without the controller fighting it. Returns the typed token ('' / 'q' / 'r')."""
    import rclpy

    print(prompt, flush=True)
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        tok = _key()
        if tok is not None:
            return tok


def recover_if_needed(node, rclpy, recovery_srv):
    """Ensure the robot is controllable (mode 1 Idle / 2 Move). Clears a reflex
    (mode 4) via error_recovery; a user-stop (mode 5) needs the e-stop released."""
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.mode in (1, 2):
        return True
    if node.mode == 5:
        print("Robot is USER-STOPPED (mode 5). Release the e-stop, then re-run.",
              file=sys.stderr)
        return False
    from franka_msgs.srv import ErrorRecovery
    cli = node.create_client(ErrorRecovery, recovery_srv)
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f"error_recovery unavailable ({recovery_srv}); robot_mode={node.mode}.",
              file=sys.stderr)
        return False
    print(f"robot_mode={node.mode} -> calling error_recovery ...")
    fut = cli.call_async(ErrorRecovery.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f"robot_mode after recovery = {node.mode}")
    return node.mode in (1, 2)


def run_capture(args):
    import rclpy
    from hw_handguide import Guide
    from robo67_insertion.nodes.socket_detector_node import grab_frame_gst

    if args.c920_device is None:
        from robo67_insertion.config_schema import load_config
        cam = load_config(args.config).camera
        args.c920_device = cam.c920_device
        if args.exposure is None:
            args.exposure = cam.c920_exposure
    params = WhiteCubeParams()

    rclpy.init()
    node = Guide(cmd_mode=args.cmd_mode)
    if not node.wait_state():
        print("ERROR: no robot_state -- aborting. (bringup up? ROS_LOCALHOST_ONLY "
              "matching it?)", file=sys.stderr)
        rclpy.shutdown(); return 1
    node.cmd.detect(timeout=3.0)
    if not recover_if_needed(node, rclpy, args.recovery_srv):
        print("ERROR: robot not controllable. If it's wedged in Idle after a reflex, "
              "relaunch the bringup, then re-run.", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1
    os.makedirs(args.capture_dir, exist_ok=True)

    print("\n=== socket-proxy calibration (Franka native guiding) ===")
    print("Move the arm with the GRIP HANDLES (press the guiding/stop button to enable).")
    print("This tool only SAMPLES the EE on Enter -- it streams no command, so it won't")
    print("fight the guiding. The robot holds where you leave it.")

    pixels, base_xy, zs = [], [], []
    station = 0
    try:
        while True:
            print(f"\n--- station #{station} ({len(pixels)} captured) ---")
            tok = wait_key_reading(
                node, "  1) Guide the peg tip into the socket bore (let it self-centre),\n"
                      "     release the guiding button, then press Enter  (q=finish, r=skip): ")
            if tok == "q":
                break
            if tok == "r":
                continue
            truth = node.ee_xyz.copy()
            print(f"     recorded base XY = ({truth[0]:.4f}, {truth[1]:.4f})")
            wait_key_reading(
                node, "  2) Guide the arm CLEAR of the socket (camera must see it),\n"
                      "     release the button, then press Enter to detect: ")
            frames = []
            for _ in range(max(1, args.frames)):
                f = grab_frame_gst(args.c920_device, exposure=args.exposure)
                if f is not None:
                    frames.append(f)
            px, nseen = detect_socket_pixel(frames, params)
            if px is None:
                print(f"     NO socket detected in {len(frames)} frame(s) -- skipped. "
                      "(arm still occluding? socket out of view? re-do this spot.)")
                continue
            try:
                import cv2
                if frames:
                    cv2.imwrite(os.path.join(args.capture_dir, f"proxy_{station:02d}.jpg"), frames[-1])
            except Exception:
                pass
            print(f"     socket pixel = ({px[0]:.1f}, {px[1]:.1f})  [{nseen}/{len(frames)} frames]")
            pixels.append([px[0], px[1]])
            base_xy.append([float(truth[0]), float(truth[1])])
            zs.append(float(truth[2]))
            station += 1
    except KeyboardInterrupt:
        print("\ninterrupted -- proceeding to fit with what was collected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return _fit_and_report(np.array(pixels, float), np.array(base_xy, float),
                           np.array(zs, float), args)


def _fit_and_report(pixels, base_xy, zs, args):
    n = 0 if pixels.size == 0 else len(pixels)
    print(f"\n=== fit === ({n} socket positions)")
    if n < 4:
        print(f"ERROR: need >= 4 positions, have {n} -- nothing saved.", file=sys.stderr)
        return 1
    corr = os.path.join(args.capture_dir, "c920_corr.csv")
    np.savetxt(corr, np.hstack([pixels, base_xy]), delimiter=",", header="u,v,base_x,base_y")
    H, rms = fit_and_save(pixels, base_xy, args.out)
    print(f"correspondences  : {corr}")
    print(f"RMS reprojection : {rms*1000:.2f} mm")
    print(f"socket-top Z (mean): {zs.mean():.4f} m  (spread {zs.max()-zs.min():.4f} m)")
    print(f"homography saved : {args.out}")
    print(f"\nNext: python3 scripts/hw_peg_in_hole_vision.py --socket-top-z {zs.mean():.4f} --dry-run")
    if rms * 1000 > args.max_rms_mm:
        print(f"WARNING: RMS {rms*1000:.2f} mm > {args.max_rms_mm} mm -- spread the "
              "positions wider / centre more carefully; consider recapturing.")
        return 1
    return 0


def build_parser():
    ap = argparse.ArgumentParser(
        description="Hand-guided socket-proxy C920->base homography calibration.")
    ap.add_argument("--selftest", action="store_true",
                    help="offline detector + fit test (no ROS/camera/robot)")
    ap.add_argument("--out", default=DEFAULT_OUT, help="output homography .npz")
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR)
    ap.add_argument("--c920-device", type=str, default=None,
                    help="C920 device (by-id symlink/path or index; default from config)")
    ap.add_argument("--exposure", type=int, default=None,
                    help="C920 exposure (default from config)")
    ap.add_argument("--frames", type=int, default=4, help="frames to fuse per detection")
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    ap.add_argument("--recovery-srv",
                    default="/panda_error_recovery_service_server/error_recovery",
                    help="error_recovery service (clears a reflex / mode 4 at startup)")
    ap.add_argument("--lock-trans", type=float, default=600.0)
    ap.add_argument("--lock-rot", type=float, default=30.0)
    ap.add_argument("--max-rms-mm", type=float, default=5.0)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        return selftest(args)
    return run_capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
