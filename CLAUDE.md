# Robo67 — CLAUDE.md

EE26 Hackathon. Franka Emika Panda arm. **Challenge 1 (peg-in-hole insertion) only.**
Full challenge doc, hardware credentials, and stack reference: [`docs/CHALLENGE.md`](docs/CHALLENGE.md)

**Branch:** `jearningers` — all commits go here, never `main`. If you use a worktree or branch branch from `jearningers` and back onto `jearningers`

## Rules

- Classical vision + force first. Only pivot to VLA/imitation if it's working.
- Prototype in MuJoCo sim before touching real hardware.
- Cartesian impedance controller is the starting point — gives compliant contact.
- Never use the joint-position controller — known bad motor behavior.
- Eigen 3.3.9 only — 3.4.0 breaks compilation.
- After a `ControlException`, call `~/service_server/error_recovery` — no need to reload the controller.
