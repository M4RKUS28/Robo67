# Robo67 Insertion Context

This context defines the domain language for peg-in-hole insertion with the Franka arm. The goal is to keep planning, implementation, and tests aligned on one shared meaning per term.

## Language

**Peg**:
The insertion part mounted at the tool center point and guided into the socket opening.
_Avoid_: pin, tip

**Socket**:
The receiving part that contains the opening and defines the target pose for insertion.
_Avoid_: hole block, target cube

**Socket top**:
The top surface height of the socket, used as the contact reference plane.
_Avoid_: table height, contact plane guess

**Contact event**:
A sustained force deviation from free-space baseline indicating peg-to-socket-top contact.
_Avoid_: touch guess, bump

**Search spiral**:
A planar search motion under light contact used to find the socket opening.
_Avoid_: random search, wiggle

**Drop event**:
A downward motion after contact that indicates the peg has entered the socket opening.
_Avoid_: slip, bounce

**Insertion cycle**:
One complete attempt from approach, through contact and seating, to retract or failure.
_Avoid_: run, episode

**Insertion intent**:
The controller-agnostic meaning of insertion behavior, including phase transitions and success criteria.
_Avoid_: command path logic, controller policy

**Canonical insertion state model**:
The single shared phase model for insertion intent across all command paths.
_Avoid_: sim-only state model, hardware-only state model

**Command path**:
A controller-specific translation of insertion intent into executable robot commands.
_Avoid_: insertion semantics

**Command path adapter**:
The module that implements one command path translation while preserving canonical insertion intent.
_Avoid_: alternate insertion logic, controller-specific state model

**Mapping model**:
The camera-specific method for converting image evidence into robot-base corrections.
_Avoid_: ad-hoc pixel math, node-local sign convention

**Contact phase**:
The insertion phase where force baseline handling switches from free-space tracking to contact validation.
_Avoid_: implicit baseline mode, hidden force mode

**Safety envelope**:
The set of workspace, step, and force constraints that every commanded motion must satisfy.
_Avoid_: optional clamp set, caller-only safety logic

**Safety profile**:
The command-path-specific rule set for applying the safety envelope.
_Avoid_: one-size-fits-all clamp behavior
