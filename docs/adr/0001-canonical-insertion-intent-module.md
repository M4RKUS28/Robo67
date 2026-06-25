# ADR-0001: Canonical insertion intent module across command paths

- Status: Accepted
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
