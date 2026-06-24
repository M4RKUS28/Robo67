# 🦾 Intel — Industrial Robotics Arm Challenge

> Build a manipulation policy for the Franka Emika Panda arm and run inference on the Intel Pantherlake workstations.
> Two challenges: a contact-rich insertion task (peg-in-hole) and a dynamic ball-balancing task.
> Running inference through OpenVINO on the Intel machine earns bonus points.

---

## Challenge 1 — Insertion (peg-in-hole)

### Approaches

- **Vision + force (classical):** Detect the socket pose (wrist or external cam, e.g. AprilTag or known CAD), align the peg above it, then do a compliant Cartesian-impedance insertion with a small search/spiral on contact while watching the external wrench `O_F_ext`. Deterministic, no training, plays to the RT impedance controller's strengths.
- **VLA / imitation:** Collect teleop demos, train ACT / SmolVLA / Pi0 in `physical-ai-studio`, deploy via OpenVINO. More general and variation-tolerant, and it lands the Intel bonus — but it costs demo-collection and tuning time.
- **Hybrid:** Learned coarse alignment + classical compliant insertion for the last centimetres.

### Practical Tips

- Prototype in MuJoCo first with the same controller — sim/real parity is the whole point of this stack.
- Use Cartesian impedance and soften stiffness near contact so you don't fight the hole.
- Set collision thresholds loose enough that reflexes don't trip during insertion, and rehearse `error_recovery`.
- Calibrate camera→base extrinsics carefully; insertion is unforgiving of pose error.
- Chamfered hole entries make a big difference if you control the print.

---

## Challenge 2 — Balance Table-Tennis Ball

> **Base task:** balance a table-tennis ball on a plate mounted to the TCP.
> **Bonus:** keep it balanced while moving through 4 different TCP poses.

### What Kind of Problem This Is

A dynamic-stabilization ("ball on plate") problem: a ball rolling on a tilting surface. You need feedback on the ball's position on the plate (overhead/external cam or custom 3D-printed wrist-cam mount), and you command plate tilt (Cartesian orientation) to keep the ball centred while the TCP translates between poses.

### Approaches

- **Classical (recommended for the time budget):** Track the ball (colour-blob or Hough-circle) → PD/LQR on (ball position, ball velocity) → commanded plate tilt via the Cartesian impedance/pose controller. Feed-forward the known trajectory between the 4 poses; let feedback reject ball drift.
- **Learned:** RL / imitation in sim then transfer — harder to get working inside the hackathon window.

### Practical Tips

- Keep accelerations low and trajectories smooth between the 4 poses — don't snap between them.
- Orientation authority matters more than position authority here.
- Mind the camera latency budget; the control loop is only as fast as your perception. (Run the ball tracker through OpenVINO and you again pick up the Intel bonus.)

---

## 2. Hardware, Machines & Credentials

**Arm:** Franka Emika Panda with Franka Hand gripper. The driver (`multipanda_ros2`) is built specifically for the Panda (FR3 support is still WIP).

**Workstations (Ubuntu 22.04):**

| Machine | Login / Password | Notes |
|---------|-----------------|-------|
| Black workstations | `ee26` | RT kernel preinstalled — use these to drive the real arm |
| Intel (Pantherlake) workstations | `H@ckathon2026` | Target for inference / OpenVINO bonus |
| Franka Desk (web UI) | user `franka` / pass `frankaRSI` | Unlock joints, activate FCI, manage brakes |

**Network:** The arm is reachable at its FCI IP. FCI must be activated in Desk, and only one client can command the robot at a time (Desk or external control, not both).

---

## 3. First Read Before Touching Hardware

> You only need to touch the arm early if you already have Panda experience.

- Watch the Franka Control Interface video: https://www.youtube.com/watch?v=91wFDNHVXI4
- **IMPORTANT:** Read the handbook (safety, Desk workflow, operating modes): https://www.generationrobots.com/media/franka-emika-robot-handbook.pdf
- Developer docs (the important one): https://frankarobotics.github.io/docs/
- Resource hub: https://franka.world/resources

**Make sure you can answer these before powering the arm:**
- Where is the E-stop, and what is the enabling device / guiding-mode button?
- How do you unlock the joints and activate FCI in Desk, and how do brakes behave?
- What is a `ControlException` (a collision/reflex trip), and how do you recover from one? (See the recovery service in §4.)
- What are sensible collision thresholds so the reflexes don't fire mid-task?

---

## 4. Software Stack — `multipanda_ros2`

**Repo:** https://github.com/tenfoldpaper/multipanda_ros2 (branch `humble`)

### What It Is

A `ros2_control`-based driver for the Panda on ROS 2 Humble / Ubuntu 22.04. Community continuation that re-adds Panda support after the official `franka_ros2` dropped it. Ships an identical-interface MuJoCo simulation so the same controller runs in sim and on hardware. Gives 1 kHz access to robot state and model, and exposes all `libfranka` control modes: torque, joint position, joint velocity, and Cartesian position/velocity (Cartesian is real-robot only, not in sim).

**Pinned versions:** `libfranka` 0.9.2, Panda firmware 4.2.1/4.2.2, MuJoCo 3.2.0, Eigen 3.3.9 (not 3.4.0 — 3.4.0 breaks compilation).

### Install (Docker — Recommended)

```bash
git clone --recursive https://github.com/tenfoldpaper/multipanda_ros2.git
cd multipanda_ros2
./tools/setup_env   # builds the docker image (takes a while)
./run               # opens a bash shell in the container, ROS 2 ws at ~/multipanda_ws
colcon build
source ~/multipanda_ws/install/setup.bash
ros2 launch franka_bringup franka_sim.launch.py   # sanity check: opens MuJoCo with one Panda
```

Extra terminal into the same container:
```bash
docker exec -it --user developer multipanda-container bash
```

Verify the RT kernel + robot connection (FCI must be active):
```bash
~/Libraries/libfranka/bin/communication_test <robot-ip>
```

If `colcon build` complains about missing packages:
```bash
rosdep update && rosdep install --from-paths src --ignore-src -y -r
```

### Bring Up the Real Arm

```bash
# single arm — arm_id is fixed to "panda"
ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>

# multi-mode variant (fast switching between controllers in one control mode)
ros2 launch franka_bringup multimode_franka.launch.py robot_ip:=<fci-ip>
```

Useful launch args: `hand` (gripper on/off), `use_rviz`. (`use_fake_hardware`/`fake_sensor_commands` exist but are legacy — just use the sim.)

### Control Modes & Warning

Swap controllers live with `rqt_controller_manager`. **Known issue:** the joint-position controller can produce bad motor behavior — prefer torque or velocity (and Cartesian on the real arm). For both challenges, the **subscriber Cartesian impedance controller** is the natural starting point because it gives you compliant contact.

### Error Recovery

After a `ControlException`, call the `~/service_server/error_recovery` service. On recovery it re-runs the previous control loop — no need to reload the controller.

### Data Available

- `franka_robot_state_broadcaster` publishes the robot model + state as ROS 2 topics (lower rate).
- Inside a `ros2_control` controller you get the full 1 kHz state/model via `franka_semantic_components`: pose `O_T_EE`, external wrench `O_F_ext_hat`, joint `q`/`dq`/`tau`, Jacobians, mass, gravity, coriolis.
- The gripper is an action-server interface (`franka_gripper`: homing / move / grasp / gripper_action), identical in sim.

### Inspect the Running System

```bash
ros2 topic list
ros2 service list
ros2 action list
ros2 control list_controllers
```

**Topics you'll likely see:**
- `/joint_states`
- `/tf`, `/tf_static`
- `/franka_robot_state_broadcaster/...` → the `FrankaRobotState` message (EE pose, external wrench, joint state, etc.)
- Active controller's command/goal topic (e.g. equilibrium/target-pose for the Cartesian-impedance controller)
- `/franka_gripper/joint_states` plus gripper action topics
- Camera streams from wrist and external cams (`.../image_raw`, `.../camera_info`, and depth if RGB-D)

**Services you'll likely see:**
- `/controller_manager/{list_controllers, load_controller, configure_controller, switch_controller, unload_controller}`
- `~/service_server/error_recovery`
- Parameter setters (`franka_msgs` set, under `~/service_server/`): collision behavior, joint impedance, Cartesian impedance, load, EE frame, K frame, force/torque collision behavior

---

## 5. Intel Acceleration — Bonus Points

Run inference on the Pantherlake Intel machine via OpenVINO.

### `physical-ai-studio`

https://github.com/open-edge-platform/physical-ai-studio

End-to-end imitation-learning / VLA framework: record demos → train → export → deploy. Native policies: ACT, Pi0, SmolVLA, GR00T, Pi0.5, plus the full LeRobot policy zoo. Exports to OpenVINO / ONNX / Torch / ExecuTorch.

```bash
# GUI (Docker)
docker compose --profile xpu up   # Intel → app at localhost:7860

# Library
pip install physicalai-train
physicalai fit --config ...
physicalai benchmark ...
policy.export("./policy", backend="openvino")
# Roll out with:
InferenceModel.load(...)
```

### `openvinotoolkit/physicalai`

https://github.com/openvinotoolkit/physicalai

The OpenVINO-toolkit home of the `physicalai` library that Studio builds on. Check its README for current entry points before relying on a specific API.
