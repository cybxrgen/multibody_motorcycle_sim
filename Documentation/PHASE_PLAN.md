# Phase Plan

**Authoritative for:** the one current development phase, the one current milestone, the remaining work in this phase, and the next-phase objective.
**Belongs elsewhere:** what exists today and verified results → `PROJECT_STATE.md`; the last assignment's report → `HANDOFF.md`;
design → `ARCHITECTURE.md`. Completed phases and past milestones live only in Git history; do not archive them here.
**Update when:** a milestone's status changes. When a phase completes, **replace** this document's content with the next phase's plan.
A milestone being listed here is **not** authorization to start it: work begins only on an explicit user assignment (`AGENTS.md`).

## Current phase: M2 — Motorcycle (in progress)
**Objective:** build the motorcycle as independent rigid bodies coupled only through M1 joints and contacts: chassis, steering/fork, swingarm, wheels,
suspension, tire model and drivetrain. M1 (generic joints, contacts, split-impulse stabilization) is complete and accepted; its history is in Git.

**Current milestone: M2.4 — Tire–ground contact foundation (M2.4.1 wheel–ground contact implemented; awaiting user review).** M2.4.1 adds a flat static ground and one point contact per wheel
(`Simulation/Motorcycle/wheel_ground_contact.gd`) that re-derives its geometry at every fixed step and is solved by the generic `ContactConstraint`; it adds no tire force model, no body and no joint.
What exists and how it was verified: `PROJECT_STATE.md`. M2.4.1 is **not** all of M2.4 beyond what the user decides, and it is not M2 and does not complete it.
M2.1 (rigid assembly) is complete and accepted (PR #10); M2.2 (front telescopic suspension, PR #11) and M2.3 (rear suspension, PR #12) are merged into `main`.

### Milestones
| Milestone | Status |
|---|---|
| M2.1 Rigid motorcycle assembly (bodies, free hinges, builder, illustrative prototype parameters, demonstration scene) | Complete and accepted |
| M2.2 Front suspension (6 bodies, 5 joints; telescopic fork: prismatic joint with travel limits, spring-damper, generic force-provider hook) | Merged (PR #11) |
| M2.3 Rear suspension (direct spring-damper between chassis and swingarm, 6 bodies, 5 joints unchanged) | Merged (PR #12) |
| M2.4 Tire–ground contact foundation — M2.4.1 wheel–ground contact (flat static ground, one point contact per wheel, per-step geometry refresh, generic contact solving) | M2.4.1 implemented; awaiting user review |
| M2.5 Tire forces and rolling interaction (slip, longitudinal/lateral force, rolling resistance, contact patch) | Not started — not authorized |
| M2.6 Powertrain interaction (engine, clutch, gearbox, final drive) | Not started — not authorized |
| Integrated M2 validation | Not started — not authorized |

Remaining M2 work is the rows after M2.4 in the order shown; their scope is the user's decision, and what else M2.4 still contains beyond M2.4.1 is the user's decision too. A milestone listed here is **not** authorization to start it:
work begins only on an explicit user assignment (`AGENTS.md`). M2 stays in progress and is complete only when the user accepts it against the objective above.

## Later phases (outline only; not planned in detail)
- M3 — Rider: articulated rider bodies, joint limits, muscle actuators, grips/seat/pegs. Needs M1 and M2 interfaces.
- M4 — Control: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK. Needs M3.
