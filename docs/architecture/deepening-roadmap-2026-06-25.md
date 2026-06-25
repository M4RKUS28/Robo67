# Deepening Roadmap (Candidates 1-4)

Date: 2026-06-25
Scope: Peg-in-hole insertion architecture in robo67_insertion.
Status: Approved decisions captured, implementation pending.

## Goal

Increase module depth, leverage, and locality for the insertion stack by deepening four high-value seams:

1. Canonical insertion intent shared across sim and hardware command paths
2. Shared pixel-to-base mapping seam for camera models
3. Encapsulated contact lifecycle module
4. Composed safety envelope with command-path-specific safety profiles

## Dependency Category Classification

- Candidate 1: Ports & adapters
  - Rationale: two command paths with different controller constraints need separate adapters behind one seam.
- Candidate 2: In-process
  - Rationale: pure math and calibration logic, no remote transport dependency.
- Candidate 3: In-process
  - Rationale: pure force lifecycle logic with deterministic state transitions.
- Candidate 4: In-process with local-substitutable behavior
  - Rationale: safety math is pure; command-path policy differences are local adapter behavior.

## Candidate 1: Canonical insertion intent module

### Decision

Use one canonical insertion state model and keep controller quirks in command path adapters.

### Deep module name

InsertionIntentModule

### External seam

A small interface that consumes normalized sensor snapshots and outputs controller-agnostic insertion intent.

### Adapter set

- MMCCommandPathAdapter (sim command path)
- ImpedanceCommandPathAdapter (hardware command path)

### Interface sketch

```python
from dataclasses import dataclass
from typing import Literal

Phase = Literal[
    "IDLE",
    "MOVE_ABOVE",
    "DESCEND_TO_CONTACT",
    "SEARCH_SPIRAL",
    "PUSH_INSERT",
    "CONFIRM",
    "RETRACT",
    "DONE",
    "ERROR",
]

@dataclass(frozen=True)
class IntentSensors:
    ee_xyz: tuple[float, float, float]
    fz: float
    fz_baseline: float
    t: float

@dataclass(frozen=True)
class InsertionIntent:
    phase: Phase
    target_xyz: tuple[float, float, float]
    done: bool
    error: str | None

class InsertionIntentModule:
    def step(self, phase: Phase, sensors: IntentSensors) -> InsertionIntent:
        ...
```

### What sits behind the seam

- Contact and drop event handling
- Spiral retry and exhaustion logic
- Confirmation and retract criteria
- Canonical transition rules

### Adapter responsibilities

- Translate intent target to controller command representation
- Enforce controller-specific acceptance windows and rate behavior
- Preserve canonical phase semantics

### File plan

- Extract canonical logic from:
  - robo67_insertion/robo67_insertion/lib/insertion_fsm.py
  - robo67_insertion/robo67_insertion/nodes/hardware_insertion_node.py (InsertionSequence)
- Introduce new module:
  - robo67_insertion/robo67_insertion/lib/insertion_intent.py
- Introduce adapters:
  - robo67_insertion/robo67_insertion/lib/command_path_adapters.py
- Wire nodes:
  - robo67_insertion/robo67_insertion/nodes/insertion_orchestrator_node.py
  - robo67_insertion/robo67_insertion/nodes/hardware_insertion_node.py

### Test surface

- New intent tests at one interface:
  - robo67_insertion/test/test_insertion_intent.py
- Adapter conformance tests:
  - robo67_insertion/test/test_command_path_adapters.py
- Keep only interface-level behavior tests; remove redundant shallow tests once parity is proven.

---

## Candidate 2: Pixel-to-base mapping seam

### Decision

Use one mapping module interface shared by overhead and eye-in-hand flows. Keep camera model differences in adapters. Keep gain outside the mapping seam.

### Deep module name

PixelToBaseMappingModule

### External seam

A single interface for mapping image evidence into base-frame correction/position terms.

### Adapter set

- HomographyMappingAdapter
- PinholeMappingAdapter

### Interface sketch

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class PixelObservation:
    u: float
    v: float

@dataclass(frozen=True)
class MappingContext:
    depth_m: float | None
    fx: float | None
    fy: float | None

class PixelToBaseMappingModule:
    def map_xy(self, obs: PixelObservation, ctx: MappingContext) -> tuple[float, float]:
        ...
```

### What sits behind the seam

- Sign convention normalization
- Parameter checks (required depth and intrinsics for pinhole path)
- Coordinate conversion semantics

### File plan

- New mapping seam module:
  - robo67_insertion/robo67_insertion/lib/pixel_mapping.py
- Adapter extraction from:
  - robo67_insertion/robo67_insertion/lib/geometry.py
  - robo67_insertion/robo67_insertion/lib/servoing.py
- Node integration:
  - robo67_insertion/robo67_insertion/nodes/socket_detector_node.py
  - robo67_insertion/robo67_insertion/nodes/d405_servo_node.py

### Test surface

- New mapping seam tests:
  - robo67_insertion/test/test_pixel_mapping.py
- Cross-adapter consistency scenarios for synthetic correspondences
- Keep existing pure math tests but shift behavioral assertions to the seam.

---

## Candidate 3: Contact lifecycle module

### Decision

Move baseline tracking and contact detection lifecycle behind one module. Make phase explicit input so mode switching is testable.

### Deep module name

ContactLifecycleModule

### External seam

One interface receives phase + Fz sample and returns structured contact status.

### Interface sketch

```python
from dataclasses import dataclass
from typing import Literal

ContactMode = Literal["free_space", "contact_search", "insert", "confirm"]

@dataclass(frozen=True)
class ContactState:
    baseline_fz: float
    is_frozen: bool

@dataclass(frozen=True)
class ContactOutcome:
    baseline_fz: float
    contact_detected: bool

class ContactLifecycleModule:
    def observe(self, mode: ContactMode, fz: float) -> ContactOutcome:
        ...
```

### What sits behind the seam

- Baseline update/freeze policy
- Threshold comparison policy
- Deterministic lifecycle transitions

### File plan

- New module:
  - robo67_insertion/robo67_insertion/lib/contact_lifecycle.py
- Integrate in node:
  - robo67_insertion/robo67_insertion/nodes/insertion_orchestrator_node.py
- Keep wrench primitives as internal implementation details:
  - robo67_insertion/robo67_insertion/lib/wrench.py

### Test surface

- Lifecycle tests:
  - robo67_insertion/test/test_contact_lifecycle.py
- Assert update-in-free-space and freeze-in-contact behavior directly through module interface.

---

## Candidate 4: Safety envelope composition module

### Decision

Put clamp ordering and force abort checks inside one seam. Keep command-path anchor policy in safety profile adapters.

### Deep module name

SafetyEnvelopeModule

### External seam

Single interface to apply safety envelope and evaluate abort conditions.

### Adapter set

- MMCSafetyProfile
- ImpedanceSafetyProfile

### Interface sketch

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class SafetyInput:
    desired_xyz: tuple[float, float, float]
    ee_xyz: tuple[float, float, float]
    prev_cmd_xyz: tuple[float, float, float]
    wrench6: tuple[float, float, float, float, float, float]

@dataclass(frozen=True)
class SafetyOutput:
    safe_xyz: tuple[float, float, float]
    abort: bool

class SafetyEnvelopeModule:
    def apply(self, data: SafetyInput) -> SafetyOutput:
        ...
```

### What sits behind the seam

- Clamp ordering (workspace then step)
- Anchor policy selection per safety profile
- Force cap checks

### File plan

- New module:
  - robo67_insertion/robo67_insertion/lib/safety_envelope.py
- Profile adapters and integration:
  - robo67_insertion/robo67_insertion/nodes/insertion_orchestrator_node.py
  - robo67_insertion/robo67_insertion/nodes/hardware_insertion_node.py
- Existing primitives remain implementation building blocks:
  - robo67_insertion/robo67_insertion/lib/safety.py

### Test surface

- Envelope behavior tests:
  - robo67_insertion/test/test_safety_envelope.py
- Profile behavior tests for anchor selection and clamp outcomes.

---

## Execution Order (autonomous sequence)

1. Candidate 1
   - Build canonical insertion intent seam and adapters.
   - Add interface tests and adapter conformance tests.
2. Candidate 3
   - Encapsulate contact lifecycle and remove node-level lifecycle leakage.
3. Candidate 4
   - Introduce safety envelope seam and safety profiles.
4. Candidate 2
   - Unify mapping seam and migrate camera nodes.

Reason for order:

- Candidate 1 provides the highest leverage and largest deletion-test win.
- Candidate 3 and 4 stabilize insertion behavior semantics and safety semantics before perception refactor.
- Candidate 2 is valuable but lower immediate risk to insertion correctness.

## Done Criteria

The architecture iteration is done when all of the following are true:

- One canonical insertion intent module drives both command paths.
- Insertion semantics do not diverge across sim and hardware nodes.
- Baseline/contact lifecycle logic is no longer orchestrator-call-site choreography.
- Safety envelope behavior is applied through one module interface in both command paths.
- Pixel-to-base conversion semantics are invoked through one mapping seam.
- Tests primarily target deep module interfaces, not shallow helper fragments.
