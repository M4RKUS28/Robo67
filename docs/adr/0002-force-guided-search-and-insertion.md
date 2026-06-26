# ADR-0002: Force-guided (admittance) search + force-drop insertion detection

- Status: Proposed — 2026-06-26
- Date: 2026-06-26
- Supersedes: none (extends ADR-0001 — the canonical intent module stays controller-agnostic and position-based)

## Context

On the real arm the search/seat phase shows three coupled failures (observed
2026-06-26):

1. **Bounce at contact.** When `DESCEND_TO_CONTACT` detects contact it hands off
   to `SEARCH_SPIRAL`, which snaps the commanded equilibrium from the deep
   descend target (`socket_top − max_press_depth`, ≈ −5 cm) to a *fixed*
   `contact_z − press_gap`. Combined with the soft controller's contact
   transient, the arm rebounds slightly.
2. **Does not go deeper at the correct XY.** During `SEARCH_SPIRAL` the
   `ImpedanceCommandPathAdapter` commands a **fixed** below-surface equilibrium
   `z_cmd = contact_z − press_gap`. On a pure-stiffness Cartesian impedance
   controller this is a *constant press force* (`F ≈ pos_stiff·gap = press_force`)
   **only while the peg is blocked by the surface**. The instant the bore opens,
   the arm sinks toward that fixed equilibrium and the press force **decays to
   zero** over `press_gap`; the arm never *chases* the peg down. Chamfer
   friction / the controller's ~cm stiction deadband then stalls entry.
3. **Does not detect insertion.** Detection is purely positional
   (`z_ee < contact_z − drop`, 3–4 mm) and therefore hostage to an accurate
   `contact_z` and to several real millimetres of physical sink — both of which
   the bounce and stiction corrupt.

The proposal (from the operator) is sound and the failures localize to one
missing capability: **regulate a constant gentle axial press and let it keep
pushing as resistance slackens, then detect insertion from the force-slacken
itself.** This is classic force-guided peg-in-hole.

Key fact this ADR records: the current scheme is *already constant-force while
blocked* — the gap is that the **fixed** equilibrium lets the force **decay**
during entry instead of being **regulated** to a setpoint as the peg moves.

## Decision

Introduce an **axial force-regulation (admittance) command mode** for the
real-arm impedance path, plus a **force-drop ("slacken") insertion-event
detector**, as new PURE seams composed by the node — without changing the
canonical phase semantics in `lib/insertion_intent.py`.

- The canonical `InsertionIntentModule` keeps emitting controller-agnostic
  position targets (the Archimedean spiral XY, the phase machine). **ADR-0001
  invariant preserved.**
- A new pure seam `lib/force_regulator.py` (`AxialForceRegulator`) converts a
  *target press force* into a per-tick commanded-equilibrium **z** by an
  admittance law, ratcheting from the previous command with anti-windup and a
  velocity cap.
- A new pure seam `lib/insertion_event.py` (`InsertionEventDetector`) filters
  the noisy `o_f_ext_hat_k` Fz, emits a `slacken` event (press force fell by
  ≥ `slacken_frac` of the held setpoint) and an `inserted` event (slacken +
  confirmed continued descent), replacing/augmenting the bare position drop.
- A force-mode in the impedance command path
  (`ImpedanceCommandPathAdapter(force_mode=True)`): for `SEARCH_SPIRAL` /
  `PUSH_INSERT` it takes the canonical **XY** target but uses the regulator's
  **z** (instead of the fixed `contact_z − press_gap`). Controller quirk →
  adapter concern, exactly like the gaps it replaces.
- Contact handoff is made continuous: initialize the regulator at the
  just-measured contact force and `z_cmd = z_ee − Fz/pos_stiff` so there is **no
  equilibrium jump** → kills the bounce.
- Everything stays gated behind config/CLI (`force_mode` default OFF) so the
  current verified behavior is the default and the change is reversible.

## Consequences

### Positive

- Active seating: the arm *chases* the peg down at constant gentle force, so it
  enters and seats instead of relaxing at the bore mouth.
- Robust detection: the force-slacken signal does not depend on a precise
  `contact_z`; confirmed-descent coupling rejects noise dips.
- No contact bounce: continuous force handoff removes the equilibrium
  discontinuity.
- Architecture stays deep: new behavior lands in two small host-tested seams +
  one adapter flag; the canonical intent and safety envelope are untouched.

### Trade-offs / risks

- A constant-force chase can still jam and build force → the firmware reflex
  (the original reason for `--release-on-insert`) remains a hazard; the force
  setpoint stays gentle and ALL safety caps (force/torque abort, z-floor, step
  cap, release-on-insert) remain mandatory.
- `o_f_ext_hat_k` is an estimate → the detector MUST filter (EMA/median) and use
  hysteresis or it will false-trigger.
- One more control parameter set to tune (`adm_gain`, `search_press_n`,
  `slacken_frac`, `confirm_drop_m`).

## Alternatives considered

1. **Just increase `press_gap` (deeper fixed equilibrium).** Rejected: more
   force when blocked (closer to the reflex) AND the force still *decays* during
   entry — it does not chase. Strictly dominated by admittance.
2. **Pure force-drop detection, keep the fixed equilibrium.** Rejected on its
   own: fixes detection but not seating ("does not go deeper"); the peg still is
   not actively pushed in.
3. **Direct model re-reference `z_cmd = z_ee − F*/pos_stiff` (no integral).**
   Kept as the regulator's degenerate/feed-forward term but not alone:
   sensitive to `pos_stiff` mismatch and Fz bias; the admittance integral
   compensates model error.
4. **Move force control into `insertion_intent.py`.** Rejected: violates
   ADR-0001 (intent must stay controller-agnostic; force/equilibrium is a soft-
   impedance quirk → adapter/node concern).

## Guardrails

- `lib/insertion_intent.py` stays controller-agnostic and position-based.
- New seams (`force_regulator.py`, `insertion_event.py`) import no rclpy/cv2 and
  are TDD'd on the host.
- Force mode is opt-in; the adapter conformance test still proves an identical
  canonical phase sequence across command paths.
- Prototype + validate in MuJoCo sim and the offline plant self-test before any
  real-arm run (per `CLAUDE.md`).

## Linked documents

- Design: [`docs/architecture/force-guided-insertion-2026-06-26.md`](../architecture/force-guided-insertion-2026-06-26.md)
- Plan: [`docs/superpowers/plans/2026-06-26-force-guided-insertion.md`](../superpowers/plans/2026-06-26-force-guided-insertion.md)
- Builds on: [`ADR-0001`](0001-canonical-insertion-intent-module.md)
