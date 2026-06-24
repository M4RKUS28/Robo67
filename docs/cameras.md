# Cameras

Three cameras:

- Two **webcams** on articulated desk arms — **static, external** viewpoints.
- One **Intel RealSense D405** depth camera **mounted on the arm via a 3D-printed
  flange bracket — eye-in-hand.** It moves with the robot and gives a close-up
  depth view that tracks the end-effector. The wrist carries both the peg and the
  D405.

## Hardware

| Stable id (`/dev/v4l/by-id/`) | Current node | Model | Notes |
|--------|------|-------|-------|
| `…Sonix…USB_2.0_Camera…SN0001` | `/dev/video0` | Microdia USB 2.0 Camera (Sonix chipset) | Overhead view, slightly dark |
| `…Intel…RealSense…405…215123071253` | `/dev/video2` depth, `/dev/video6` color | **Intel RealSense D405** — short-range stereo depth | Eye-in-hand RGB-D, see below |
| `…046d_HD_Pro_Webcam_C920…` | `/dev/video8` | Logitech HD Pro Webcam C920 | Overhead view, overexposed by default — lock exposure |

> ⚠️ **Bare `/dev/videoN` numbers are not stable across replug/reboot.** The C920
> is on `/dev/video8` and the RealSense occupies `/dev/video2`–`/dev/video7`.
> Prefer the `/dev/v4l/by-id/…` symlinks (`ls -l /dev/v4l/by-id/`).

Each device exposes several `/dev/video*` nodes; the **even-numbered** ones are
the capture nodes, the odd ones are metadata.

## Intel RealSense D405 (depth)

- Short-range **stereo depth + RGB** camera, USB-C 3.x, serial `215123071253`.
- **Eye-in-hand (arm-mounted).** Its pose moves with the robot — recover the
  camera pose from forward kinematics. Needs a **hand-eye calibration**
  (camera → flange/EE frame), *not* a fixed camera→base extrinsic.
- Ideal working range **~7–50 cm** (min-Z ~7 cm). Built for close-up
  manipulation: the robot can drive it to a known standoff above the socket for a
  high-detail depth view, then close the loop on insertion. Depth quality falls
  off past ~0.5–1 m.
- Passive stereo, no IR projector.
- UVC stream → node mapping (current):
  - `/dev/video2` → **depth** (Z16, raw 16-bit — not directly JPEG-encodable)
  - `/dev/video6` → **color** (YUY2, up to 1280×720)
  - `/dev/video4` → second/left imager (magenta/IR tint)
- **Not the same as the "Intel acceleration" bonus.** That bonus is compute
  (NPU/OpenVINO), which we're skipping. Using the D405 as a depth *sensor* is
  unrelated and fine.

### See what it sees

No RealSense SDK is installed yet, so use GStreamer (already present):

```bash
# Live color preview window
gst-launch-1.0 v4l2src device=/dev/video6 ! videoconvert ! autovideosink

# Grab one color still
gst-launch-1.0 v4l2src device=/dev/video6 num-buffers=1 ! jpegenc ! filesink location=rs_color.jpg
```

For depth + 3D point cloud, install the SDK and use the official viewer (run in
a real terminal — needs sudo):

```bash
sudo apt install librealsense2-utils   # provides realsense-viewer + rs-* tools
realsense-viewer
```

## Exposure fix (C920)

```bash
v4l2-ctl -d /dev/video8 --set-ctrl=auto_exposure=1,exposure_time_absolute=150
```

(`v4l2-ctl` ships in `v4l-utils` — not currently installed.)

## Usage notes

- The two webcams are fixed external viewpoints; the **D405 is eye-in-hand** and
  moves with the arm.
- Calibrate before use: webcam extrinsics (camera → robot base frame) **and** the
  D405 hand-eye transform (camera → flange/EE frame).
- Viewpoints available: two static overhead webcams (Microdia / C920) + the
  moving eye-in-hand D405 the robot can position over the socket for close-range
  depth.
- **Webcams are autofocus — verify focus is sharp on the workspace surface**
  (where peg and socket sit), not on the robot arm above it.
- Capture webcam frames with GStreamer (OpenCV / ffmpeg not installed):

```bash
gst-launch-1.0 v4l2src device=/dev/video0 num-buffers=1 ! jpegenc ! filesink location=cam0.jpg   # Microdia
gst-launch-1.0 v4l2src device=/dev/video8 num-buffers=1 ! jpegenc ! filesink location=cam2.jpg   # C920
```

## Photos

See `assets/depthCam/` for the D405 and its bracket.
