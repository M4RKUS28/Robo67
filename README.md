# Robo67

**EE26 Hackathon, Munich, June 2026.**
45 hours to teach a robot arm to insert a peg into a hole.
Yes, we know. The jokes write themselves. We've heard them all.

We are Team 67. We have a Franka Panda, two webcams zip-tied to desk lamps, and a dangerous amount of caffeine.
The goal: classical vision + force control. No neural networks. No suffering. (Some suffering.)

---

## The Challenge

**Challenge 1 — Peg-in-Hole Insertion**
Detect a socket with a camera, align the arm above it, push a peg in with compliant force control.
If it works: glory. If it doesn't: spiral search. If that doesn't work: check the Eigen version.

The robot is a Franka Emika Panda (`192.168.1.67/desk/`).
The controller stack is [`multipanda_ros2`](https://github.com/tenfoldpaper/multipanda_ros2).
The branch is `jearningers`. Main is for people with time.

---

## Docs

```
docs/
  cameras.md                   # two overhead webcams and why the C920 needs a manual exposure fix
  franka/
    specs.md                   # joint limits, Cartesian limits, don't exceed them
    fci_overview.md            # 1 kHz FCI architecture, exclusive Desk/FCI rule
    bringup_api.md             # ros2 launch incantations and service names
  hackathon/
    hacker_handbook.md         # schedule, locations, WiFi, food, sleep
    intel_challenge.md         # full challenge brief, credentials, software stack, bonus points
```

---

## Quick Reference

| Thing | Value |
|-------|-------|
| Franka Desk | `https://192.168.1.67/desk/` — user `franka` / pass `frankaRSI` |
| Black workstation | password `ee26` |
| Intel workstation | password `H@ckathon2026` |
| Controller topic | `/cartesian_impedance/pose_desired` — Float64MultiArray [px,py,pz, R00..R22] |
| Error recovery | `ros2 service call ~/service_server/error_recovery std_srvs/srv/Trigger {}` |
| Camera capture | `gst-launch-1.0 v4l2src device=/dev/video2 num-buffers=1 ! jpegenc ! filesink location=frame.jpg` |
| C920 exposure fix | `v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1,exposure_time_absolute=150` |

---

## Rules We Learned the Hard Way

- **Never use the joint-position controller.** Bad motor behavior. You will regret it.
- **Eigen 3.3.9 only.** 3.4.0 breaks compilation. Don't ask.
- **FCI and Desk cannot run at the same time.** One commander. Like a good kitchen.
- **After a ControlException:** call `error_recovery`, don't reload the controller.
- **Prototype in MuJoCo first.** The sim uses the same controller. Touch hardware last.

---

## Stack

- ROS 2 Humble, Ubuntu 22.04
- `multipanda_ros2` (branch `humble`) — Panda driver + identical MuJoCo sim
- `libfranka` 0.9.2, MuJoCo 3.2.0, Eigen **3.3.9**
- Cartesian impedance controller for compliant contact
- Two overhead webcams (`/dev/video0`, `/dev/video2`) — external only, no wrist mount

---

*Named after the robot's IP. We are not creative. We are engineers.*
