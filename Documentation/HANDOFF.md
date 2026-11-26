# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.3 Stage B follow-up on the PR #21 branch: (P1) preserve the physical moment of normal impulses at the distributed sample positions; (P2) reject, rather than skip, compressed samples with non-finite derived coefficients. Stage C and every later milestone were not started; M2.5.3 is not complete.

## 2. Git observation at delivery
- VERIFIED: branch `m2/tire-patch-normal-support`, PR #21 head `f7b499e` on base `main` `8a246f9`; the working tree was clean before the work. One new local commit on top of `f7b499e`; not pushed, not merged, no PR, no review. The commit SHA and artifact names are in the delivery report.

## 3. What changed
- `Simulation/Motorcycle/tire_patch_normal_support.gd`: the row's wheel-side angular Jacobian is the full `r x n`; `applied_wheel_moment()` sums the installed rows' `angular_b` times the load; the spin is removed from the compression rate by a frozen row-bias term (`spin_bias`); a compressed sample with a non-finite `gamma`, `bias`, `split_bias` or `spin_bias` clears the whole patch with a specific `last_reason`.
- `Tests/test_tire_patch_normal_support.gd`: three new tests (row angular impulse against an independent asymmetric moment; solver angular impulse against the moment of the reported loads; overflow of a derived coefficient) and two changed (`test_e_normal_response...` lever arm without projection; `test_e_spin...` now runs c_A = 0 and 1e5, and its exact-zero axle-moment assertion became a rounding-level bound because the zero came only from cutting the component out of the wrench).
- Documentation: `PHYSICS_CONVENTIONS.md`, `ARCHITECTURE.md`, `PROJECT_STATE.md`, `TESTING.md`, `HANDOFF.md`. Not changed: the solver, `ConstraintRow`, `ContactConstraint`, `SimWorld`, the sampler, the tyre model, the builder, scenes.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: baseline on the PR head: 582 passed. After the change: `godot --headless --path . --script res://Tests/run_tests.gd` → **585 passed, 0 failed, 0 skipped (16052 checks)**.
- VERIFIED mutation checks (source restored afterwards, byte-identical): axle-projected angular row → 3 tests fail (including the new row test); skipping the sample instead of invalidating → the overflow test fails.
- Design decision for review: with the full Jacobian alone the spin returns to `vn` (probe: +27 % to +412 % load, axle moment 13 % to 20 % of `N R`). The compensation is in the bias, not in the wrench. An exact velocity map separate from the wrench map would need a generic solver change, which was not made.
- VERIFIED: fresh import (`.godot/` deleted, `godot --headless --path . --import`) exit 0, no error lines.
- Not run: scene smoke runs and visual inspection (no scene or solver code changed; reasoned, not run).

## 5. Unresolved / limitations
`PROJECT_STATE.md` ("Tyre patch normal support"). The spin compensation is frozen over the step. The `.uid` files of the patch script and its test are not in the PR (Godot generates them on import); they were left out of this commit.

## 6. Next action
Await the user's review. The agent does not push or merge.
