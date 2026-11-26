# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.3 Stage B second follow-up on the PR #21 branch: separate the velocity-side Jacobian (compression rate and effective mass, no axle spin) from the impulse-side Jacobian (the full physical wrench `r x n`) of the tyre patch rows, and remove the frozen spin-bias compensation. Stage C and every later milestone were not started; M2.5.3 is not complete.

## 2. Git observation at delivery
- VERIFIED: branch `m2/tire-patch-normal-support`, PR #21 head `9e40558` on base `main` `8a246f9`; the working tree was clean before the work. One new local commit on top of `9e40558`; not pushed, not merged, no PR, no review. The commit SHA and artifact names are in the delivery report.

## 3. What changed
- `Simulation/Solver/velocity_jacobian_row.gd` (new, with `.uid`): `ConstraintRow` subclass with `velocity_angular_a/b`; `compute_velocity`, `compute_k`, `compute_split_velocity` and `apply_split_impulse` use the velocity side; warm start, `apply_impulse` and `solve` apply the inherited full wrench. `ConstraintRow` and all existing constraints are unchanged.
- `Simulation/Motorcycle/tire_patch_normal_support.gd`: rows are `VelocityJacobianRow`; impulse `J_ang = r x n`, velocity `J_ang = (r x n) - u (u . (r x n))`; `spin_bias` removed (also from the non-finite check; bottomed rows have bias 0); header rewritten.
- `Tests/test_tire_patch_normal_support.gd`: five new tests (`test_n_*`: effective mass, compression velocity and support independent of spin, sequential spin change, distinct Jacobians, split correction), fixture option `inertia`; one existing bound changed (see section 5).
- Documentation: `PHYSICS_CONVENTIONS.md`, `ARCHITECTURE.md`, `PROJECT_STATE.md`, `TESTING.md`, `HANDOFF.md`. Not changed: the solver, `ConstraintRow`, `ContactConstraint`, `SimWorld`, the sampler, the tyre model, the builder, scenes.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: baseline on the PR head: 585 passed. After the change: `godot --headless --path . --script res://Tests/run_tests.gd` → **590 passed, 0 failed, 0 skipped (16657 checks)**, exit 0, no `SCRIPT ERROR` in the log.
- VERIFIED mutation checks (source restored byte-identical afterwards, patch test file only): previous implementation (frozen bias, full Jacobian) with the new tests → the 5 `test_n_*` tests fail (20 "fed back" failures in the sequential test); axle-projected impulse Jacobian → `test_e_row_angular_impulse...`, `test_e_normal_response...`, `test_n_row_holds_...` and `test_n_axle_spin_change...` fail.
- VERIFIED: fresh import (`.godot/` deleted, `godot --headless --path . --import`) exit 0, no error lines; headless smoke runs of `simulation_test`, `main`, `motorcycle_test` (`--quit-after 240`) exit 0, no error lines. Not run: visual inspection, long scene runs.

## 5. Unresolved / limitations
`PROJECT_STATE.md` ("Tyre patch normal support"). Review points: (1) the axle-impulse bound of `test_e_normal_response...` changed from `2e-6 N R dt` to 6 ULP of the stored spin times the axial inertia, with an added load-based axle-moment check; the old value held only because float32 rounding of the stored 40 rad/s spin happened to be small (measured 3 ULP now; the moment of the loads is 4e-8 N R). (2) The used effective mass `J_v M^-1 J_v^T` equals the exact response `J_v M^-1 J_i^T` only because the axle is a principal axis of the wheel inertia. (3) The test runner exits 0 even when a test hits a `SCRIPT ERROR` (the test is aborted, not counted as failed): grep the log.

## 6. Next action
Await the user's review. The agent does not push or merge.
