# Phase Plan

**Authoritative for:** the one current development phase, the one current milestone, the remaining work in this phase, and the next-phase objective.
**Belongs elsewhere:** what exists today and verified results → `PROJECT_STATE.md`; the last assignment's report → `HANDOFF.md`;
design → `ARCHITECTURE.md`. Completed phases and past milestones live only in Git history; do not archive them here.
**Update when:** a milestone's status changes. When a phase completes, **replace** this document's content with the next phase's plan.
A milestone being listed here is **not** authorization to start it: work begins only on an explicit user assignment (`AGENTS.md`).

## Current phase: M2 — Motorcycle (in progress)
**Objective:** build the motorcycle as independent rigid bodies coupled only through M1 joints and contacts: chassis, steering/fork, swingarm, wheels,
suspension, tire model and drivetrain. M1 (generic joints, contacts, split-impulse stabilization) is complete and accepted; its history is in Git.

**Current milestone: M2.5.3 — Distributed finite-width tire contact patch. Current stage: Stage C — Tire-force integration.**
- **Completed:** M2.5.2 (merged, PR #19). M2.5.3 Stage A, deterministic finite-width tyre surface sampling (merged, PR #20). M2.5.3 Stage B, area-weighted compliant normal support (merged, PR #21).
- **Current stage:** Stage C, tyre-force integration (the M2.4.2 tyre model driven by the aggregate state of the patch), is implemented; the corrected candidate is under review in PR #24 (it contains the PR #23 commit plus the review corrections; PR #23 is still open and is superseded by PR #24); it is the authorized stage and is not accepted or merged.
- **Remaining in M2.5.3:** Stage D motorcycle builder integration with transactional registration, Stage E regression, documentation and final reconciliation. Neither is started nor authorized.
- **Next authorized work boundary:** the outcome of the Stage C review. Stages D and E start only on an explicit user assignment.

What exists and how it was verified: `PROJECT_STATE.md`.

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
| M2.5.1.2 Toggleable geometry wireframe (presentation only: oriented body boxes and finite-width tyre surfaces, key G), with M2.5.1.2.1 runtime box-dimension refresh | Implemented, accepted and merged (PR #18); explicitly authorized increment, not a physics milestone; its tyre-surface and visual-size box views were replaced in M2.5.2 |
| M2.5.2 Motorcycle collision geometry and basic controls (oriented-box ground contacts for chassis, steering assembly, fork slider and swingarm through the generic contact; wheels keep tyre contact only; estimated per-part masses; direct throttle / brake / steering inputs replacing the test torque toggles; wireframe of the physical boxes) | Implemented, accepted and merged (PR #19) |
| M2.5.3 Distributed finite-width tire contact patch (per-wheel patch constraint with compliant sample normal support, aggregate contact state for the existing tyre model, front/rear independence, debug view) | Stage A (deterministic tyre surface sampling) merged; Stage B (area-weighted compliant normal support, `TirePatchNormalSupport`) merged; Stage C (tyre-force integration on the patch, not registered in the assembly) implemented; the corrected candidate is under review in PR #24 (supersedes the still-open PR #23), not accepted or merged; Stages D–E not started |
| Further tire/rolling interaction (listed earlier as M2.5.4; scope to be defined by the user) | Not started — not authorized |
| M2.6 Powertrain interaction (engine, clutch, gearbox, final drive) | Not started — not authorized |
| Integrated M2 validation | Not started — not authorized |

Remaining work in M2.5.3, in the established order:
- Stage A — deterministic finite-width tyre surface sampling: merged (PR #20).
- Stage B — area-weighted compliant normal support: merged (PR #21).
- Stage C — tyre-force integration: implemented; the corrected candidate is under review in PR #24 (supersedes the still-open PR #23).
- Stage D — motorcycle builder integration and transactional registration: not started.
- Stage E — regression, documentation and final reconciliation: not started.

After M2.5.3 the remaining M2 work is the rows after it in the table; their scope is the user's decision. A milestone or stage listed here is **not** authorization to start it:
work begins only on an explicit user assignment (`AGENTS.md`). M2 stays in progress and is complete only when the user accepts it against the objective above.

## Later phases (outline only; not planned in detail)
- M3 — Rider: articulated rider bodies, joint limits, muscle actuators, grips/seat/pegs. Needs M1 and M2 interfaces.
- M4 — Control: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK. Needs M3.
