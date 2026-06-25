# HANDOFF - Robo67 Peg-in-Hole (Updated)

Date: 2026-06-25
Branch: jearningers
Scope: Challenge 1 only (peg-in-hole)
Status: Baseline software path implemented; architecture deepening decisions locked; implementation of deepening pending.

## Mission For Next Agent

Do not re-litigate architecture. Execute the plan.

1. Use docs/superpowers/plans/2026-06-25-peg-in-hole-insertion.md as the execution contract.
2. Implement Phase 8 (architecture deepening) in the locked order.
3. Keep hardware runs serialized with a lockfile; parallelize pure logic and node refactors with worktrees.

## What Was Completed In This Session

1. Architecture review run completed and narrowed to candidates 1 to 4 (candidate 5 explicitly skipped).
2. Domain glossary was created and updated:
   - CONTEXT.md
3. Deepening roadmap created with module/seam/test strategy:
   - docs/architecture/deepening-roadmap-2026-06-25.md
4. ADR recorded for the highest leverage irreversible decision:
   - docs/adr/0001-canonical-insertion-intent-module.md
5. Detailed TDD plan updated before this handoff rewrite:
   - docs/superpowers/plans/2026-06-25-peg-in-hole-insertion.md
   - Added Architecture Deepening Lock and Phase 8 implementation tasks.

## Locked Architecture Decisions (Authoritative)

Selection outcome:
- Execute candidates 1, 2, 3, and 4.
- Skip candidate 5.

Candidate 1 (Strong): canonical insertion intent seam
- One canonical insertion state model for both command paths.
- Controller quirks belong only in command path adapters.
- Two adapters define a real seam:
  - MMC command path adapter
  - Impedance command path adapter
- Primary test surface shifts to insertion intent seam + adapter conformance tests.

Candidate 2 (Worth exploring): pixel-to-base mapping seam
- One shared mapping interface for camera flows.
- Camera-model differences isolated in adapters:
  - Homography adapter
  - Pinhole adapter
- Mapping seam owns sign conventions and parameter validation.
- Control gain remains outside mapping seam.

Candidate 3 (Worth exploring): contact lifecycle seam
- Baseline update/freeze and contact threshold behavior moved into one module.
- Mode/phase is explicit input to the seam.
- Orchestrator no longer owns baseline lifecycle choreography.

Candidate 4 (Worth exploring): safety envelope seam
- Clamp ordering and force abort checks moved behind one module interface.
- Command-path anchor behavior implemented by safety profiles:
  - MMC profile (anchor to measured EE)
  - Impedance profile (anchor to previous command)

## Current Build Reality

Baseline status remains:
- robo67_insertion package exists and is implemented for baseline software path.
- Pure libraries and node stack are present.
- Sim plumbing/path is validated (see robo67_insertion/PHASE0_VERIFIED.md).

Not completed yet:
- Phase 8 deepening implementation.
- Hardware milestone closure still depends on arm availability and camera/container constraints.

## Source-Of-Truth Files

Read these first in order:
1. CLAUDE.md
2. CONTEXT.md
3. docs/adr/0001-canonical-insertion-intent-module.md
4. docs/architecture/deepening-roadmap-2026-06-25.md
5. docs/superpowers/plans/2026-06-25-peg-in-hole-insertion.md
6. robo67_insertion/PHASE0_VERIFIED.md

## Execution Order (Do Not Change)

Use this sequence from the plan:
1. Candidate 1 (canonical insertion intent seam)
2. Candidate 3 (contact lifecycle seam)
3. Candidate 4 (safety envelope seam)
4. Candidate 2 (pixel mapping seam)
5. Full test pass + cleanup + docs updates

Why this order:
- Candidate 1 gives the biggest leverage/locality gain and eliminates semantic drift risk.
- Candidate 3 and 4 stabilize insertion and safety behavior semantics before perception seam unification.
- Candidate 2 is high value but lower immediate risk to insertion correctness.

## Required Test Surfaces

Add and keep these seam-level tests as implementation proceeds:
- robo67_insertion/test/test_insertion_intent.py
- robo67_insertion/test/test_command_path_adapters.py
- robo67_insertion/test/test_contact_lifecycle.py
- robo67_insertion/test/test_safety_envelope.py
- robo67_insertion/test/test_pixel_mapping.py

Run:
- python3 -m pytest robo67_insertion/test -q

Rule:
- Delete superseded shallow tests only after equivalent seam-level coverage exists.

## Operational Constraints (Still Active)

1. Single-arm mutex for hardware control is mandatory.
   - Use flock /tmp/robo67_arm.lock for any real-arm run.
2. Do not restart multipanda-container if a live arm session is active.
3. Never use joint-position controller.
4. Keep Eigen pinned to 3.3.9.
5. On ControlException/reflex states, use error recovery service and relaunch as needed.

## Open Risks And Blockers

1. Hardware contention: real arm may be occupied by another active session.
2. Container camera access and cv2 ABI compatibility may still require host-side frame bridge or container reconfiguration.
3. Hardware phases (calibration tuning and milestone closure) remain to be completed under mutex conditions.

## Definition Of Done For This Deepening Wave

Done means all are true:
- Both command paths consume one canonical insertion intent module.
- Contact lifecycle is no longer orchestrator-owned glue logic.
- Safety envelope is applied through one seam with explicit profiles.
- Pixel mapping callers use one mapping interface.
- Seam-level tests above are green and are the primary behavior tests.
- Roadmap and ADR updated with implementation notes.

## Notes

This handoff supersedes earlier handoff revisions that only captured pre-deepening planning.
If conflicts are found between older notes and the files listed under Source-Of-Truth Files, trust the source-of-truth files.
