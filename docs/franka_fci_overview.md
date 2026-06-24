# Franka FCI Architecture Overview

Source: https://frankarobotics.github.io/docs/overview.html

## Architecture

- FCI = direct 1 kHz bidirectional connection to Arm and Hand over Ethernet
- External workstation connects to robot via dedicated Ethernet link
- **Exclusive control**: Desk/Apps cannot operate simultaneously with FCI

## 1 kHz Feedback Available

- Joint position, velocity, link-side torque sensor signals
- Estimated externally applied torques and forces (`O_F_ext_hat_k`)
- Collision and contact detection

## Robot Model (via libfranka)

- Forward kinematics (all joints)
- Jacobian matrices (per joint)
- Inertia matrix
- Coriolis/centrifugal vectors
- Gravity vectors

## Network Requirement

- Requires stable, low-latency network connection
- RT kernel required on the control workstation

## Software Bindings

- C++ (libfranka native)
- Python (pylibfranka)
- ROS2 (via ros2_control, what we use — multipanda_ros2)
- MATLAB/Simulink
