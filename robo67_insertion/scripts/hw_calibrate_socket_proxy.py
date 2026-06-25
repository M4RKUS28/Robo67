#!/usr/bin/env python3
"""hw_calibrate_socket_proxy.py -- C920->base homography via the SOCKET as the
robot-as-groundtruth marker, ROBOT-DRIVEN (the arm moves; you move the socket).

Why robot-driven (not hand-guided)
----------------------------------
The real arm runs the *subscriber* Cartesian impedance controller whose
stiffness is fixed firm at activation (pos_stiff=500) and is NOT live-settable,
so the arm cannot be floated for hand-guiding. Instead we COMMAND the arm
(gently, clamped -- the validated ``hw_move_to`` carrot path) to known base-XY
positions, and you align the socket to it. A stiff arm is ideal for precise
commanded holds.

Per station (do >= 4, spread out):
  1. The arm moves to a known (x, y) at a safe hover Z and HOLDS.
  2. You bring the SOCKET so its bore sits centred around the hovering peg tip,
     press Enter (records EE base XY = socket XY), then set the socket straight
     down on the table.
  3. The arm moves to a PARK pose (clear of the camera view); the socket pixel
     is auto-detected (detect_sockets).
Repeat; type 'q' to finish -> it fits + saves config/c920_homography.npz and
prints the RMS reprojection error. The socket-top Z is irrelevant here (the
homography is XY-only; insertion force-probes the true contact Z).

SAFETY: motion is gentle and clamped (workspace AABB, speed cap, force abort,
reflex auto-recovery). The arm moves only in XY at a fixed hover Z plus to the
park pose -- it never descends toward the table. Keep the e-stop in hand.

USAGE (INSIDE multipanda-container; cartesian impedance controller ACTIVE) --
normally launched via ``./start_calibration``:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_calibrate_socket_proxy.py --confirm

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
from robo67_insertion.lib.hole_detect import WhiteSocketParams, detect_sockets  # noqa: E402
from robo67_insertion.nodes.calibration_node import fit_and_save  # noqa: E402

DEFAULT_OUT = os.path.join(_PKG_ROOT, "config", "c920_homography.npz")
DEFAULT_CONFIG = os.path.join(_PKG_ROOT, "config", "robo67.yaml")
DEFAULT_CAPTURE_DIR = os.path.join(_PKG_ROOT, "captures", "calib")


# ---------------------------------------------------------------------------
# Geometry + vision helpers (pure).
# ---------------------------------------------------------------------------

def make_grid(box, nx, ny):
    """Serpentine list of (x, y) stations spanning ``box`` = [xmin,xmax,ymin,ymax]."""
    xmin, xmax, ymin, ymax = box
    xs = np.linspace(xmin, xmax, nx)
    ys = np.linspace(ymin, ymax, ny)
    pts = []
    for j, y in enumerate(ys):
        row = list(xs) if j % 2 == 0 else list(xs[::-1])  # short moves between stations
        pts += [(float(x), float(y)) for x in row]
    return pts


def detect_socket_pixel(frames, params):
    """Best socket pixel (median u,v) over frames that yielded a detection."""
    us, vs = [], []
    for img in frames:
        holes = detect_sockets(img, params)
        if holes:
            us.append(holes[0].u)
            vs.append(holes[0].v)
    if not us:
        return None, 0
    return (float(np.median(us)), float(np.median(vs))), len(us)


# ---------------------------------------------------------------------------
# Offline self-test: synthesize a known homography + correspondences, fit, and
# assert round-trip; also check detect_sockets on a synthetic white socket.
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

    px, _ = detect_socket_pixel([cube(True)] * 3, WhiteSocketParams())
    det_ok = px is not None and abs(px[0] - 320) < 6 and abs(px[1] - 240) < 6
    det_ok = det_ok and detect_socket_pixel([cube(False)] * 3, WhiteSocketParams())[0] is None

    grid = make_grid([0.40, 0.55, -0.12, 0.12], 3, 3)
    base = np.array(grid, float)
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
    print(f"grid stations      : {len(grid)} (serpentine)")
    print(f"fit RMS            : {rms*1000:.3f} mm   max reproj {max_err_mm:.3f} mm")
    ok = det_ok and rms * 1000 < 1.0 and max_err_mm < 2.0
    print("RESULT             :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# Live, robot-driven capture (lazy ROS imports).
# ---------------------------------------------------------------------------

def _key():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.readline().strip().lower() if r else None


def hold_until_key(node, target, quat, prompt):
    """Hold the arm at ``target`` (stream the fixed equilibrium) until the
    operator presses Enter; return the typed token ('' for Enter, 'q'/'r')."""
    import rclpy

    print(prompt, flush=True)
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        node.publish(target, quat)
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
    from hw_move_to import Mover
    from robo67_insertion.nodes.socket_detector_node import grab_frame_gst

    if args.c920_device is None:
        from robo67_insertion.config_schema import load_config
        cam = load_config(args.config).camera
        args.c920_device = cam.c920_device
        if args.exposure is None:
            args.exposure = cam.c920_exposure
    params = WhiteSocketParams()
    grid = make_grid(args.box, args.nx, args.ny)

    rclpy.init()
    mover = Mover(cmd_mode=args.cmd_mode)
    if not mover.wait_state():
        print("ERROR: no robot_state -- aborting. (bringup up? ROS_LOCALHOST_ONLY "
              "matching it?)", file=sys.stderr)
        rclpy.shutdown(); return 1
    mover.cmd.detect(timeout=3.0)
    if not recover_if_needed(mover, rclpy, args.recovery_srv):
        print("ERROR: robot not controllable (need Idle/Move) -- aborting.", file=sys.stderr)
        mover.destroy_node(); rclpy.shutdown(); return 1
    os.makedirs(args.capture_dir, exist_ok=True)

    hold_quat = mover.ee_quat
    z_hover = args.z_hover if args.z_hover is not None else float(mover.ee_xyz[2])
    park = (args.park if args.park is not None
            else [0.30, -0.35, max(z_hover + 0.05, 0.40)])
    print("\n=== socket-proxy calibration (robot-driven) ===")
    print(f"hover Z = {z_hover:.3f} m  | stations = {len(grid)} "
          f"({args.nx}x{args.ny}) in x[{args.box[0]:.2f},{args.box[1]:.2f}] "
          f"y[{args.box[2]:.2f},{args.box[3]:.2f}]  | park = {[round(v,2) for v in park]}")
    print("I MOVE the arm; you align the SOCKET to the hovering peg tip.")

    if args.confirm:
        try:
            if input("Area clear, e-stop in hand? Type YES to let the arm move: ").strip() != "YES":
                print("not confirmed -- exiting without moving.")
                mover.destroy_node(); rclpy.shutdown(); return 0
        except EOFError:
            mover.destroy_node(); rclpy.shutdown(); return 1
    for c in range(args.countdown, 0, -1):
        print(f"moving in {c} ..."); time.sleep(1.0)

    pixels, base_xy = [], []
    park_np = np.array(park, float)
    try:
        for i, (x, y) in enumerate(grid):
            target = np.array([x, y, z_hover], float)
            print(f"\n--- station #{i} -> hover ({x:.3f}, {y:.3f}, {z_hover:.3f}) "
                  f"[{len(pixels)} captured] ---")
            if not mover.move_to(target, hold_quat, speed=args.speed):
                print("   move failed -- skipping station."); continue
            tok = hold_until_key(
                mover, target, hold_quat,
                "   Center the SOCKET bore around the hovering peg tip, then press\n"
                "   Enter (record). Set the socket straight down after. ('q'=finish, 'r'=skip): ")
            if tok == "q":
                break
            if tok == "r":
                continue
            truth = mover.ee_xyz.copy()
            print(f"   recorded base XY = ({truth[0]:.4f}, {truth[1]:.4f})  -- "
                  "now set the socket down; moving arm clear to detect ...")
            mover.move_to(park_np, hold_quat, speed=args.speed)
            frames = []
            for _ in range(max(1, args.frames)):
                f = grab_frame_gst(args.c920_device, exposure=args.exposure)
                if f is not None:
                    frames.append(f)
            px, nseen = detect_socket_pixel(frames, params)
            if px is None:
                print(f"   NO socket detected in {len(frames)} frame(s) -- skipped. "
                      "(arm still occluding? socket out of view? re-do this spot.)")
                continue
            try:
                import cv2
                if frames:
                    cv2.imwrite(os.path.join(args.capture_dir, f"proxy_{i:02d}.jpg"), frames[-1])
            except Exception:
                pass
            print(f"   socket pixel = ({px[0]:.1f}, {px[1]:.1f})  [{nseen}/{len(frames)} frames]")
            pixels.append([px[0], px[1]])
            base_xy.append([float(truth[0]), float(truth[1])])
    except KeyboardInterrupt:
        print("\ninterrupted -- proceeding to fit with what was collected.")
    finally:
        # leave the arm parked + held
        try:
            for _ in range(40):
                rclpy.spin_once(mover, timeout_sec=0.02)
                mover.publish(park_np, hold_quat)
        except Exception:
            pass
        mover.destroy_node()
        rclpy.shutdown()

    return _fit_and_report(np.array(pixels, float), np.array(base_xy, float), args)


def _fit_and_report(pixels, base_xy, args):
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
    print(f"homography saved : {args.out}")
    print("\nNext: python3 scripts/hw_peg_in_hole_vision.py --socket-top-z <measured> --dry-run")
    if rms * 1000 > args.max_rms_mm:
        print(f"WARNING: RMS {rms*1000:.2f} mm > {args.max_rms_mm} mm -- spread the "
              "positions wider / align more carefully; consider recapturing.")
        return 1
    return 0


def build_parser():
    ap = argparse.ArgumentParser(
        description="Robot-driven socket-proxy C920->base homography calibration.")
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

    # grid / motion
    ap.add_argument("--box", type=float, nargs=4, default=[0.40, 0.55, -0.12, 0.12],
                    help="station grid region: xmin xmax ymin ymax (m)")
    ap.add_argument("--nx", type=int, default=3)
    ap.add_argument("--ny", type=int, default=3)
    ap.add_argument("--z-hover", type=float, default=None,
                    help="fixed hover Z for all stations (m); default = current EE z")
    ap.add_argument("--park", type=float, nargs=3, default=None,
                    help="x y z the arm moves to so the camera sees the socket")
    ap.add_argument("--speed", type=float, default=0.03, help="move speed cap (m/s)")
    ap.add_argument("--confirm", action="store_true", help="prompt YES before any motion")
    ap.add_argument("--countdown", type=int, default=3)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    ap.add_argument("--recovery-srv",
                    default="/panda_error_recovery_service_server/error_recovery")
    ap.add_argument("--max-rms-mm", type=float, default=5.0)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        return selftest(args)
    return run_capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
