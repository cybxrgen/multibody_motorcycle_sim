# Phase Plan

**Authoritative for:** the one current development phase, the one current milestone, the remaining work in this phase, and the next-phase objective.
**Belongs elsewhere:** what exists today and verified results → `PROJECT_STATE.md`; the last assignment's report → `HANDOFF.md`;
design → `ARCHITECTURE.md`. Completed phases and past milestones live only in Git history; do not archive them here.
**Update when:** a milestone's status changes. When a phase completes, **replace** this document's content with the next phase's plan.
A milestone being listed here is **not** authorization to start it: work begins only on an explicit user assignment (`AGENTS.md`).

## Current phase: M2 — Motorcycle (in progress)
**Objective:** build the motorcycle as independent rigid bodies coupled only through M1 joints and contacts: chassis, steering/fork, swingarm, wheels,
suspension, tire model and drivetrain. M1 (generic joints, contacts, split-impulse stabilization) is complete and accepted; its history is in Git.

**Current milestone: M2.1 — Rigid motorcycle assembly (implemented; awaiting user review and acceptance).** Five independent rigid bodies (chassis,
steering/fork assembly, swingarm, front wheel, rear wheel) connected by four generic hinges, built by a reusable scene-tree-independent builder
(`Simulation/Motorcycle/`). What exists and how it was verified: `PROJECT_STATE.md`. This milestone is **not** M2 and does not complete it.

### Milestones
| Milestone | Status |
|---|---|
| M2.1 Rigid motorcycle assembly (5 bodies, 4 free hinges, builder, illustrative prototype parameters, demonstration scene) | Implemented; awaiting user review |
| Front suspension (telescopic fork travel, spring/damper) | Not started — not authorized |
| Rear suspension (shock, spring/damper on the swingarm) | Not started — not authorized |
| Ground contact and tire model (contact patch, slip, traction, rolling resistance) | Not started — not authorized |
| Drivetrain (engine, clutch, gearbox, final drive) | Not started — not authorized |

Remaining M2 work is the four "Not started" rows; their order and scope are the user's decision. A milestone listed here is **not** authorization to start it:
work begins only on an explicit user assignment (`AGENTS.md`). M2 is complete only when the user accepts it against the objective above.

## Later phases (outline only; not planned in detail)
- M3 — Rider: articulated rider bodies, joint limits, muscle actuators, grips/seat/pegs. Needs M1 and M2 interfaces.
- M4 — Control: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK. Needs M3.
