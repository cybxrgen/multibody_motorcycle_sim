# Phase Plan

**Authoritative for:** the one current development phase, the one current milestone, the remaining work in this phase, and the next-phase objective.
**Belongs elsewhere:** what exists today and verified results → `PROJECT_STATE.md`; the last assignment's report → `HANDOFF.md`;
design → `ARCHITECTURE.md`. Completed phases and past milestones live only in Git history; do not archive them here.
**Update when:** a milestone's status changes. When a phase completes, **replace** this document's content with the next phase's plan.
A milestone being listed here is **not** authorization to start it: work begins only on an explicit user assignment (`AGENTS.md`).

## Current phase: M1 — Joints and contacts (in progress)
**Objective:** extend the generic constraint solver (rows, projected Gauss-Seidel, warm start, Baumgarte) with the joint and unilateral
constraints a vehicle/rider simulation needs, with no motorcycle- or rider-specific logic in the core.

**Phase acceptance criteria** (M1 is complete only when all hold):
1. M1.1, M1.2 and M1.3 are each implemented, verified by headless tests with physically justified tolerances, documented in their owning documents, delivered, and accepted by the user.
2. Core and solver remain generic (`AGENTS.md` §4); no built-in Godot physics is used as authority.
3. Remaining project-wide limitations are recorded in `PROJECT_STATE.md`.
Per-milestone acceptance criteria are defined by the assignment that authorizes the milestone.

**Current milestone: none authorized.** M1.1.1 is complete (awaiting user review and publication); M1.2 and M1.3 start only on an explicit user assignment.

### Milestones
| Milestone | Status |
|---|---|
| M1.1 Hinge joint with angular limits and velocity motor | Complete (implemented and committed; test results in `PROJECT_STATE.md`) |
| M1.1.1 Constraint/world body-membership validation | Complete (implemented and committed; verification in `PROJECT_STATE.md`) |
| M1.2 Contact (unilateral) constraints with friction | Pending — not started |
| M1.3 Split-impulse stabilization | Pending — not started |

## Next phase: M2 — Motorcycle
**Objective:** build the motorcycle as independent rigid bodies coupled only through M1 joints and contacts: chassis, steering/fork, swingarm, wheels,
suspension, tire model and drivetrain. Requires M1 complete.

## Later phases (outline only; not planned in detail)
- M3 — Rider: articulated rider bodies, joint limits, muscle actuators, grips/seat/pegs. Needs M1 and M2 interfaces.
- M4 — Control: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK. Needs M3.
