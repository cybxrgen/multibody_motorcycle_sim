# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
PR #21 P2 review finding (M2.5.3 Stage B): the `VelocityJacobianRow` effective mass is exact only when the wheel axle is a principal axis of the wheel inertia, but `SimMassProperties` accepts full tensors and `TirePatchNormalSupport` did not enforce it. Make the principal-axis condition an explicit, validated precondition. Narrow correction: the solver, `VelocityJacobianRow`, `ConstraintRow`, `SimMassProperties` and the wheel inertia are unchanged; Stage C and every later milestone were not started; M2.5.3 is not complete.

## 2. Git observation at delivery
- VERIFIED: branch `m2/tire-patch-normal-support`, PR #21 head `91f8446` (the base of the new commit); the working tree was clean before the work. One new local commit on top of `91f8446`; not pushed, not merged, no PR created, no review. The commit SHA and artifact names are in the delivery report.

## 3. What changed
- `Simulation/Motorcycle/tire_patch_normal_support.gd`: `validate()` calls the new static `axle_principal_axis_error(local_inertia)` for a dynamic wheel. The check is made in the BODY frame (axle exactly body-local +X): the symmetrised products of inertia `sqrt(J_xy^2 + J_xz^2)` must not exceed `AXLE_PRINCIPAL_REL_TOL = 1e-5` times the largest tensor element. The error message states the requirement, the measured `I_xy`, `I_xz`, the coupling and the tolerance, and what to change. An invalid (zeroed) wheel body keeps reporting only the dynamic-body rule. Header comment updated. Nothing is projected, diagonalised, repaired or skipped.
- `Tests/test_tire_patch_normal_support.gd`: seven tests, group O (`test_o_*`), reading only `validate()`, `create()` and the world, with the tolerance as a literal.
- Documentation: `PHYSICS_CONVENTIONS.md` (precondition, frame, tolerance, rejected approach), `ARCHITECTURE.md`, `TESTING.md`, `PROJECT_STATE.md`, `HANDOFF.md`. Not changed: the solver, `ConstraintRow`, `VelocityJacobianRow`, `SimMassProperties`, `SimWorld`, `MotorcycleConfig`, scenes.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: baseline on `91f8446`: `godot --headless --path . --script res://Tests/run_tests.gd` → 590 passed, 0 failed (16657 checks). With the change: **597 passed, 0 failed, 0 skipped (16967 checks)**, exit 0, no `SCRIPT ERROR` in the log. Fresh import (`.godot/` deleted, `--import`) exit 0, no error lines; headless smoke runs of `simulation_test`, `main`, `motorcycle_test` (`--quit-after 240`) exit 0, no error lines. `git diff --check` clean.
- VERIFIED: the group-O tests against the previous implementation of the source (same test file): 1 passes (the diagonal wheel), 6 fail by assertion; with the fix all 7 pass. The 7 tests use no new symbol, so the previous run fails by assertion and not by a parse error.
- VERIFIED mutation checks (source restored byte-identical afterwards): world-frame check, tolerance 1e-3, tolerances 1e-8 and 1e-10, `I_xy` only → each fails at least one group-O test.
- VERIFIED: tensors assembled through general rotations carry float32 roundoff in `I_xy`/`I_xz` of at most 2.2e-7 of the largest element (75 cases), 45 times below the tolerance.
- Not run: visual inspection, long scene runs, the full suite per mutant. A one-file runner (kept outside the repository) was used for the focused runs; the repository has no per-file option.

## 5. Unresolved / limitations
`PROJECT_STATE.md` ("Tyre patch normal support"). Review points: (1) the patch tolerance (1e-5 of the largest element) is stricter than the `MotorcycleConfig` axle residual tolerance (1e-4 relative), so a configuration wheel with a coupling between the two is accepted by the configuration and refused by the patch; aligning the two was out of scope. (2) A wheel that violates the precondition is refused, not supported by a cross-Jacobian effective mass. (3) The test runner exits 0 even when a test hits a `SCRIPT ERROR` (the test is aborted, not counted as failed): grep the log.

## 6. Next action
Await the user's review. The agent does not push or merge.
