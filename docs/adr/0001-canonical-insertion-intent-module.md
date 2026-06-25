# ADR-0001: Canonical insertion intent module across command paths

- Status: Accepted — Implemented 2026-06-25
- Date: 2026-06-25

## Context

The codebase currently carries two load-bearing insertion state models:

- A pure insertion state model in sim-oriented logic
- A second insertion sequence in hardware-oriented logic

Both represent the same insertion intent but differ in command-path behavior. This causes semantic drift risk: phase changes, retry logic, and confirmation criteria can diverge across command paths.

## Decision

Adopt one canonical insertion intent module as the external seam for insertion semantics.

Use command path adapters to express controller-specific command behavior:

- MMC command path adapter
- Impedance command path adapter

Controller quirks are adapter concerns and must not alter canonical insertion phase semantics.

## Consequences

### Positive

- Higher depth: one interface for insertion intent, more implementation hidden behind seam.
- Better locality: insertion behavior changes land in one module.
- Better leverage: one behavior model serves both command paths.
- Better testability: one test surface plus adapter conformance tests.

### Trade-offs

- Requires extraction and migration effort across two existing nodes.
- Requires explicit adapter contract tests to prevent semantic regressions.

## Alternatives considered

1. Keep separate insertion models per command path.
   - Rejected: maintains ongoing semantic drift risk.
2. Share only helper functions while keeping separate state models.
   - Rejected: shallow refactor, low deletion-test value.

## Guardrails

- Canonical insertion intent must remain controller-agnostic.
- Adapter behavior must not redefine insertion phase transitions.
- New insertion tests should target the canonical seam first.

## Implementation notes (2026-06-25)

- Canonical module: `robo67_insertion/lib/insertion_intent.py`
  (`InsertionIntentModule`). It emits a controller-agnostic absolute
  `target_xyz` plus `contact_z`/`hole_xy`; all phase transitions (contact, drop,
  spiral retry/exhaustion, confirm, retract) live here and nowhere else.
- Adapters: `robo67_insertion/lib/command_path_adapters.py`.
  - `MMCCommandPathAdapter` — sim path; holds the tool-down quaternion, passes the
    canonical target through (the orchestrator's carrot lead-clamp produces the
    step), light downward bias during SEARCH_SPIRAL.
  - `ImpedanceCommandPathAdapter` — real-arm subscriber path; holds the row-major
    R, translates targets to below-surface equilibrium gaps
    (`press_gap = press_force/pos_stiff`, `insert_gap = insert_press/pos_stiff`),
    and keeps `px`/`R22` non-zero. Controller quirks live ONLY here.
  - Both delegate to `InsertionIntentModule`; neither carries transition logic.
- `InsertionFSM` (`lib/insertion_fsm.py`) was reduced to a thin shim that
  delegates to the canonical module, so the pre-existing `test_insertion_fsm.py`
  remains green as the MMC-path parity harness (proves the single model preserves
  the original behavior).
- Conformance test `test/test_command_path_adapters.py` feeds one sensor stream to
  both adapters and asserts an identical phase sequence — the guardrail that the
  two command paths cannot diverge.
- Validated on the real Franka (subscriber command path
  `/cartesian_impedance/pose_desired`): dry-run gap math against live state, a
  +2 cm nudge, and a guarded force-contact probe.
