# Cameras

Two **webcams** on articulated desk arms. Can be freely repositioned to any static viewpoint — **cannot be mounted on the robot arm**. The wrist bracket holds only the peg.

## Hardware

| Device | Model | Notes |
|--------|-------|-------|
| `/dev/video0` | Microdia USB 2.0 Camera (Sonix chipset) | Overhead view, slightly dark |
| `/dev/video2` | Logitech HD Pro Webcam C920 | Overhead view, overexposed by default — lock exposure |

Each physical camera exposes 2 `/dev/video*` nodes (capture + metadata). Use `video0` and `video2` for capture.

## Exposure fix (C920)

```bash
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1,exposure_time_absolute=150
```

## Usage notes

- All vision must use fixed external viewpoints — no wrist cam.
- Camera extrinsics (camera → robot base frame) must be calibrated before use.
- Plan for up to 2 viewpoints (e.g. overhead + front/side). User can reposition between runs.
- **Both are autofocus webcams — verify focus is sharp on the floor/workspace surface** (where the peg and socket sit), not on the robot arm above it. Reposition or manually lock focus if the important area is blurry.
- Capture frames with GStreamer (OpenCV not installed, ffmpeg not installed):

```bash
gst-launch-1.0 v4l2src device=/dev/video0 num-buffers=1 ! jpegenc ! filesink location=cam0.jpg
gst-launch-1.0 v4l2src device=/dev/video2 num-buffers=1 ! jpegenc ! filesink location=cam2.jpg
```
