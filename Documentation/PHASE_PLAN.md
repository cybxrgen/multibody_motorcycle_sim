# Phase Plan

**Authoritative for:** the one current development phase, the one current milestone, the remaining work in this phase, and the next-phase objective.
**Belongs elsewhere:** what exists today and verified results → `PROJECT_STATE.md`; the last assignment's report → `HANDOFF.md`;
design → `ARCHITECTURE.md`. Completed phases and past milestones live only in Git history; do not archive them here.
**Update when:** a milestone's status changes. When a phase completes, **replace** this document's content with the next phase's plan.
A milestone being listed here is **not** authorization to start it: work begins only on an explicit user assignment (`AGENTS.md`).

## Current phase: M2 — Motorcycle (in progress)
**Objective:** build the motorcycle as independent rigid bodies coupled only through M1 joints and contacts: chassis, steering/fork, swingarm, wheels,
suspension, tire model and drivetrain. M1 (generic joints, contacts, split-impulse stabilization) is complete and accepted; its history is in Git.

**Current milestone: M2.4 — Tire–ground contact foundation (M2.4.1 wheel–ground contact merged; M2.4.2 tire behavior and the M2.4.2.1 speculative-contact correction implemented, accepted and merged).** M2.4.1 added a flat static ground and one point contact per wheel
(`Simulation/Motorcycle/wheel_ground_contact.gd`) that re-derives its geometry at every fixed step and is solved by the generic `ContactConstraint`. M2.4.2 added a simple tyre model on those contacts (longitudinal and lateral slip, friction-ellipse combined slip, rolling resistance) with the tyre as the only owner of the tangential response; M2.4.2.1 corrected it so that the tyre does not act across the air gap of a speculative contact. Neither adds a body, a joint, an engine or a drivetrain.
What exists and how it was verified: `PROJECT_STATE.md`. M2.4.2 and M2.4.2.1 are **not** all of M2.4 beyond what the user decides, and they are not M2 and do not complete it: M2 remains in progress.
M2.5.1 (finite-width tyre geometry: a motorcycle-specific tyre cross-section and its lowest point against a plane; geometry only, contact and tyre forces unchanged) and its M2.5.1.1 validation hardening are implemented, accepted and merged (PR #17). The explicitly authorized presentation-only geometry wireframe (M2.5.1.2, with the M2.5.1.2.1 box-refresh correction) is implemented and awaits the user's review (PR #18). M2.5.2 and later are not started.
M2.1 (rigid assembly) is complete and accepted (PR #10); M2.2 (front telescopic suspension, PR #11), M2.3 (rear suspension, PR #12), M2.4.1 (PR #13) and M2.4.2 with M2.4.2.1 (PR #15) are merged into `main`.

### Milestones
| Milestone | Status |
|---|---|
| M2.1 Rigid motorcycle assembly (bodies, free hinges, builder, illustrative prototype parameters, demonstration scene) | Complete and accepted |
| M2.2 Front suspension (6 bodies, 5 joints; telescopic fork: prismatic joint with travel limits, spring-damper, generic force-provider hook) | Merged (PR #11) |
| M2.3 Rear suspension (direct spring-damper between chassis and swingarm, 6 bodies, 5 joints unchanged) | Merged (PR #12) |
| M2.4 Tire–ground contact foundation — M2.4.1 wheel–ground contact (flat static ground, one point contact per wheel, per-step geometry refresh, generic contact solving) | M2.4.1 merged (PR #13) |
| M2.4.2 Tire behavior (slip, longitudinal/lateral force, combined-slip friction ellipse, rolling resistance on the wheel contacts; simple prototype model) | Implemented, reviewed, accepted and merged (PR #15) |
| M2.4.2.1 Tire speculative-contact correction (no tyre action across the air gap of a speculative contact) | Implemented, reviewed, accepted and merged (PR #15) |
| M2.5.1 Finite-width tire geometry (motorcycle-specific tyre cross-section, lowest point against a plane with camber, per-wheel configuration; geometry only), with M2.5.1.1 validation hardening | Implemented, accepted and merged (PR #17) |
| M2.5.1.2 Toggleable geometry wireframe (presentation only: oriented body boxes and finite-width tyre surfaces, key G), with M2.5.1.2.1 runtime box-dimension refresh | Implemented, awaiting user review (PR #18); explicitly authorized increment, not a physics milestone |
| M2.5.2 Discrete contact-patch approximation | Not started — not authorized |
| M2.5.3 Tire-force integration | Not started — not authorized |
| M2.5.4 Further tire/rolling interaction (scope to be defined by the user) | Not started — not authorized |
| M2.6 Powertrain interaction (engine, clutch, gearbox, final drive) | Not started — not authorized |
| Integrated M2 validation | Not started — not authorized |

Remaining M2 work is the rows after M2.4 in the order shown; their scope is the user's decision, and what else M2.4 still contains beyond M2.4.1, M2.4.2 and M2.4.2.1 is the user's decision too. A milestone listed here is **not** authorization to start it:
work begins only on an explicit user assignment (`AGENTS.md`). M2 stays in progress and is complete only when the user accepts it against the objective above.

## Later phases (outline only; not planned in detail)
- M3 — Rider: articulated rider bodies, joint limits, muscle actuators, grips/seat/pegs. Needs M1 and M2 interfaces.
- M4 — Control: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK. Needs M3.
