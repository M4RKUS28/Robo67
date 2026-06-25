# calibration/

Everything for the overhead-C920 → robot-base homography used by the peg-in-hole
vision (`config/c920_homography.npz`). Produced 2026-06-25 (socket-proxy method,
~5 mm RMS).

## Files

| File | What |
|------|------|
| `start_calibration.sh` | One command to run the guided calibration (host → docker-execs into `multipanda-container`, or run inside it). |
| `calibrate_guided.py` | The guided tool. Detects the white-cube centroid (`detect_white_cubes`) + reads the EE pose; you move the arm with Franka native guiding. Fits + saves the homography. |
| `MANUAL_CALIBRATION.md` | Step-by-step manual fallback (raw commands) if the guided tool fails — includes the bringup-relaunch recovery and all the gotchas. |
| `c920_homography.npz` | The calibration captured this session (copy of `config/c920_homography.npz`). |
| `c920_corr.csv` | The 5 correspondences (`u,v,base_x,base_y`) it was fit from. |
| `captures/` | Annotated detection frames from the session (`pt*_annot.jpg`) + new `point_NN.jpg` written by a guided run. |

## Run it

```bash
./calibration/start_calibration.sh
```

Per point (do ≥4, spread out): place the **socket** in view with the **arm
clear** → Enter (it detects the cube); guide the **peg into the bore** with the
grip handles → Enter (it reads the pose). Type `q` to fit + save. It writes
`robo67_insertion/config/c920_homography.npz` and prints the RMS.

Test detection offline (no robot):
```bash
python3 calibration/calibrate_guided.py --test-detect calibration/captures/pt5b_annot.jpg
```

## How it works (and why)

- **Feature = white-cube centroid**, not the bore: the white-on-white bore washes
  out when the C920 is slightly overexposed; the bright square doesn't. The bore
  is ~centred on the cube, so the centroid is a stable proxy. The insertion uses
  the same detector (`hw_peg_in_hole_vision.py --detector cube`, the default).
- **Arm moved by Franka native guiding** (grip handles): the real controller is
  too stiff to soft-float and its gravity comp is off, so software guiding
  doesn't work. Guiding can crash the bringup — that's expected; relaunch it
  (see `MANUAL_CALIBRATION.md`). `FrankaState` still reports the pose meanwhile.
- A homography is **plane-specific**: every point is taken at socket-top height
  (the peg is seated in the bore each time). Z is force-probed by the insertion.

See `MANUAL_CALIBRATION.md` for the full rationale, the exact captured data, and
recovery commands.
