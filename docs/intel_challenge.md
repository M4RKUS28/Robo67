# Intel — Industrial Robotics Arm Challenge

> Source: EE26 Hacker Handbook Notion page (scraped 2026-06-25)

---

## Overview

Build a manipulation policy for the **Franka Emika Panda** arm. Two challenges:

1. **Contact-rich insertion** — peg-in-hole with 3D-printed shapes
2. **Dynamic ball-balancing** — keep a table-tennis ball on a plate while the arm moves

For each challenge, choose your own approach: classical vision + force control, a learned VLA/imitation policy, or a hybrid. Running inference through **OpenVINO** on the Intel Pantherlake machine earns **bonus points**.

**Cameras:** wrist/gripper cameras and external/overhead cameras — may use either or both.

---

## Challenge 1 — Insertion (peg-in-hole)

### Approaches

**Vision + force (classical)**
Detect socket pose (external cam, e.g. AprilTag or known CAD), align peg above it, then do compliant Cartesian-impedance insertion with a small search/spiral on contact while watching external wrench `O_F_ext`. Deterministic, no training, plays to the RT impedance controller's strengths.

**VLA / imitation**
Collect teleop demos, train ACT / SmolVLA / Pi0 in `physical-ai-studio`, deploy via OpenVINO. More general and variation-tolerant, earns the Intel bonus — but costs demo-collection and tuning time.

**Hybrid**
Learned coarse alignment + classical compliant insertion for the last centimetres.

### Practical Tips

- Prototype in **MuJoCo first** with the same controller — sim/real parity is the whole point of this stack.
- Use **Cartesian impedance** and soften stiffness near contact so you don't fight the hole.
- Set collision thresholds loose enough that reflexes don't trip during insertion, and rehearse `error_recovery`.
- Calibrate camera→base extrinsics carefully; insertion is unforgiving of pose error.
- Chamfered hole entries make a big difference if you control the print.

---

## Challenge 2 — Balance Table-Tennis Ball

**Base task:** balance a table-tennis ball on a plate mounted to the TCP.
**Bonus:** keep it balanced while moving through **4 different TCP poses**.

### What Kind of Problem This Is

A **dynamic-stabilization** ("ball on plate") problem. You need feedback on the ball's position on the plate (overhead/external cam), and you command **plate tilt** (Cartesian orientation) to keep the ball centred while the TCP translates between poses.

### Approaches

**Classical (recommended for the time budget)**
Track the ball (colour-blob or Hough-circle) → PD/LQR on (ball position, ball velocity) → commanded plate tilt via Cartesian impedance/pose controller. Feed-forward the known trajectory between the 4 poses; let feedback reject ball drift.

**Learned**
RL / imitation in sim then transfer — harder to get working inside the hackathon window.

### Practical Tips

- Keep accelerations low and trajectories smooth between the 4 poses.
- **Orientation authority** matters more than position authority here.
- Mind the **camera latency budget** — control loop is only as fast as your perception. (Running the ball tracker through OpenVINO earns the Intel bonus.)

---

## Hardware & Credentials

**Arm:** Franka Emika Panda with Franka Hand gripper. The driver (`multipanda_ros2`) is built specifically for the Panda (FR3 support is still WIP).

| Machine | Login / Password | Notes |
|---------|-----------------|-------|
| Black workstations | `ee26` | RT kernel preinstalled — use these to drive the real arm |
| Intel (Pantherlake) workstations | `H@ckathon2026` | Target for inference / OpenVINO bonus |
| Franka Desk (web UI) | user `franka` / pass `frankaRSI` | Unlock joints, activate FCI, manage brakes |

**Network:** Arm reachable at its FCI IP. FCI must be activated in Desk. Only one client can command the robot at a time (Desk *or* external control, not both).

---

## Before Touching Hardware (Read First)

1. Watch the FCI video: https://www.youtube.com/watch?v=91wFDNHVXI4
2. Read the Panda handbook (safety, Desk workflow, operating modes): https://www.generationrobots.com/media/franka-emika-robot-handbook.pdf
3. Developer docs (most important): https://frankarobotics.github.io/docs/
4. Resource hub: https://franka.world/resources

**Answer these before powering the arm:**
- Where is the E-stop, and what is the enabling device / guiding-mode button?
- How do you unlock joints and activate FCI in Desk? How do brakes behave?
- What is a `ControlException` (a collision/reflex trip) and how do you recover?
- What are sensible collision thresholds so reflexes don't fire mid-task?

---

## Software Stack — `multipanda_ros2`

**Repo:** https://github.com/tenfoldpaper/multipanda_ros2 (branch `humble`)

A `ros2_control`-based driver for the Panda on **ROS 2 Humble / Ubuntu 22.04**. Ships an identical-interface **MuJoCo simulation** so the same controller runs in sim and on hardware. Gives **1 kHz** access to robot state and model; exposes all `libfranka` control modes: torque, joint position, joint velocity, Cartesian position/velocity (Cartesian is **real-robot only**, not in sim).

**Pinned versions:**
- `libfranka` 0.9.2
- Panda firmware 4.2.1 / 4.2.2
- MuJoCo 3.2.0
- **Eigen 3.3.9** — NOT 3.4.0 (3.4.0 breaks compilation)

### Install (Docker — recommended)

```bash
git clone --recursive https://github.com/tenfoldpaper/multipanda_ros2.git
cd multipanda_ros2
./tools/setup_env   # builds the docker image (takes a while)
./run               # opens bash shell in the container
colcon build
source ~/multipanda_ws/install/setup.bash
ros2 launch franka_bringup franka_sim.launch.py   # sanity check: opens MuJoCo with one Panda
```

Extra terminal into the same container:
```bash
docker exec -it --user developer multipanda-container bash
```

Verify RT kernel + robot connection (FCI must be active):
```bash
~/Libraries/libfranka/bin/communication_test <robot-ip>
```

If `colcon build` complains about missing packages:
```bash
rosdep update && rosdep install --from-paths src --ignore-src -y -r
```

### Bring Up the Real Arm

```bash
# Single arm — arm_id is fixed to "panda"
ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>

# Multi-mode variant (fast switching between controllers in one control mode)
ros2 launch franka_bringup multimode_franka.launch.py robot_ip:=<fci-ip>
```

Useful launch args: `hand` (gripper on/off), `use_rviz`.

### Control Modes & Warning

Swap controllers live with `rqt_controller_manager`.

**Known issue:** the **joint-position controller can produce bad motor behavior** — prefer torque or velocity, or Cartesian on the real arm. For both challenges, the **subscriber Cartesian impedance controller** is the natural starting point (compliant contact).

### Error Recovery

After a `ControlException`, call:
```bash
~/service_server/error_recovery
```
On recovery it re-runs the previous control loop — no need to reload the controller.

### Where the Data Lives

- `franka_robot_state_broadcaster` publishes robot model + state as ROS 2 topics (lower rate).
- Inside a `ros2_control` controller, get full **1 kHz** state/model via `franka_semantic_components`: pose `O_T_EE`, external wrench `O_F_ext_hat`, joint `q`/`dq`/`tau`, Jacobians, mass, gravity, coriolis.
- Gripper: action-server interface (`franka_gripper`: homing / move / grasp / gripper_action), identical in sim.

### Inspect Running System

```bash
ros2 topic list
ros2 service list
ros2 action list
ros2 control list_controllers
```

**Key topics:**
- `/joint_states`
- `/tf`, `/tf_static`
- `/franka_robot_state_broadcaster/...` → `FrankaRobotState` msg (EE pose, external wrench, joint state, etc.)
- Active controller's command/goal topic (e.g. equilibrium/target-pose topic for Cartesian impedance controller)
- `/franka_gripper/joint_states` + gripper action topics
- Camera streams: `.../image_raw`, `.../camera_info`, depth if RGB-D

**Key services:**
- `/controller_manager/{list,load,configure,switch,unload}_controller`
- `~/service_server/error_recovery`
- Parameter setters under `~/service_server/`: collision behavior, joint impedance, Cartesian impedance, load, EE frame, K frame, force/torque collision behavior

---

## Intel Acceleration Bonus

Run inference on the Pantherlake Intel machine via **OpenVINO**.

### physical-ai-studio
**Repo:** https://github.com/open-edge-platform/physical-ai-studio

End-to-end imitation-learning / VLA framework: record demos → train → export → deploy.

Policies: **ACT, Pi0, SmolVLA, GR00T, Pi0.5**, plus full LeRobot policy zoo.

Exports to: OpenVINO / ONNX / Torch / ExecuTorch.

```bash
# GUI (Docker)
docker compose --profile xpu up   # for Intel → app at localhost:7860

# Library
pip install physicalai-train
physicalai fit --config ...
physicalai benchmark ...
policy.export("./policy", backend="openvino")
# Rollout:
InferenceModel.load(...)
```

### openvinotoolkit/physicalai
**Repo:** https://github.com/openvinotoolkit/physicalai

The OpenVINO-toolkit home of the `physicalai` library that Studio builds on. Check its README for current entry points before relying on a specific API.
