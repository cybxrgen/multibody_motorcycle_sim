# Phase Plan

**Authoritative for:** the one current development phase, the one current milestone, the remaining work in this phase, and the next-phase objective.
**Belongs elsewhere:** what exists today and verified results → `PROJECT_STATE.md`; the last assignment's report → `HANDOFF.md`;
design → `ARCHITECTURE.md`. Completed phases and past milestones live only in Git history; do not archive them here.
**Update when:** a milestone's status changes. When a phase completes, **replace** this document's content with the next phase's plan.
A milestone being listed here is **not** authorization to start it: work begins only on an explicit user assignment (`AGENTS.md`).

## Current phase: M2 — Motorcycle (in progress)
**Objective:** build the motorcycle as independent rigid bodies coupled only through M1 joints and contacts: chassis, steering/fork, swingarm, wheels,
suspension, tire model and drivetrain. M1 (generic joints, contacts, split-impulse stabilization) is complete and accepted; its history is in Git.

**Current milestone: M2.3 — Rear suspension (implemented; awaiting user review and acceptance).** A direct physical spring-damper between the chassis and the swingarm
(`Simulation/Motorcycle/rear_suspension.gd`), acting along the instantaneous line between two attachment points as an equal-and-opposite force pair through the generic
force-provider hook; it adds no body or joint. What exists and how it was verified: `PROJECT_STATE.md`. This milestone is **not** M2 and does not complete it.
M2.1 (rigid assembly) is complete and accepted (PR #10 merged); M2.2 (front telescopic suspension) is merged into `main` (PR #11).

### Milestones
| Milestone | Status |
|---|---|
| M2.1 Rigid motorcycle assembly (bodies, free hinges, builder, illustrative prototype parameters, demonstration scene) | Complete and accepted |
| M2.2 Front suspension (6 bodies, 5 joints; telescopic fork: prismatic joint with travel limits, spring-damper, generic force-provider hook) | Merged (PR #11) |
| M2.3 Rear suspension (direct spring-damper between chassis and swingarm, 6 bodies, 5 joints unchanged) | Implemented; awaiting user review |
| Ground contact and tire model (contact patch, slip, traction, rolling resistance) | Not started — not authorized |
| Drivetrain (engine, clutch, gearbox, final drive) | Not started — not authorized |

Remaining M2 work is the two "Not started" rows; their order and scope are the user's decision. A milestone listed here is **not** authorization to start it:
work begins only on an explicit user assignment (`AGENTS.md`). M2 stays in progress and is complete only when the user accepts it against the objective above.

## Later phases (outline only; not planned in detail)
- M3 — Rider: articulated rider bodies, joint limits, muscle actuators, grips/seat/pegs. Needs M1 and M2 interfaces.
- M4 — Control: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK. Needs M3.
