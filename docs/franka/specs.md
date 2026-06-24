# Franka Robot Specifications & Limits

Source: https://frankarobotics.github.io/docs/robot_specifications.html

**We have the FER (Franka Emika Robot = Panda), NOT the FR3.**

## FER (Panda) Cartesian Limits

| Quantity | Limit |
|----------|-------|
| Translation velocity | 1.7 m/s max |
| Rotation velocity | 2.5 rad/s max |
| Translation acceleration | 13.0 m/s² max |
| Rotation acceleration | 25.0 rad/s² max |
| Translation jerk | 6500.0 m/s³ max |
| Rotation jerk | 12500.0 rad/s³ max |

## FER (Panda) Joint Limits

| Joint | Position Range | Velocity | Torque |
|-------|---------------|----------|--------|
| 1 | ±2.8973 rad | 2.175 rad/s | 87 Nm |
| 2 | -1.7628 to 1.7628 rad | 2.175 rad/s | 87 Nm |
| 3 | ±2.8973 rad | 2.175 rad/s | 87 Nm |
| 4 | -3.0718 to -0.0698 rad | 2.175 rad/s | 87 Nm |
| 5 | ±2.8973 rad | 2.61 rad/s | 12 Nm |
| 6 | -0.0175 to 3.7525 rad | 2.61 rad/s | 12 Nm |
| 7 | ±2.8973 rad | 2.61 rad/s | 12 Nm |

Acceleration: 7.5–15 rad/s² (varies by joint)
Jerk: 3750–10000 rad/s³ (varies by joint)

## Control Requirements

- Initial and final trajectory states must have zero velocity and acceleration.
- "Necessary conditions" violation → immediate motion abort.
- "Recommended conditions" → optimal performance.

## FCI Control Modes (1 kHz)

1. Gravity + friction compensated joint torque commands
2. Joint position or velocity commands
3. Cartesian pose or velocity commands

**Important:** Desk and FCI cannot operate simultaneously — only one client at a time.
