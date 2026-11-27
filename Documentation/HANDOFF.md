# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.3 Stage C, tyre-force integration: connect the merged Stage B `TirePatchNormalSupport` to the M2.4.2 tyre model so that one coherent tangential response and one rolling-resistance response per wheel are driven by the patch's aggregate physical normal load and contact kinematics, preserving the M2.4.2 behaviour and parameters. Stage D (builder integration) and Stage E (regression and reconciliation) were not authorized and were not started; M2.5.3 is not complete.

## 2. Git observation at delivery
- VERIFIED: base `main` HEAD `71717a1` (merge of PR #22), clean working tree at the start; Stage B (PR #21 merge `d078590`) present. Branch `m2/tire-patch-force-integration` created from it, one new local commit. Not pushed, not merged, no PR, no review. The commit SHA and artifact names are in the delivery report.

## 3. What changed
- `Simulation/Motorcycle/tire_patch_normal_support.gd`: new `create_with_tire()` (copies the tyre spec, takes the surface friction), `tire` / `surface_friction` / `tire_forces_enabled`, validation of both, and the Stage C solve: `prepare()` reads the start-of-solve tyre frame before the normal warm start and applies the tangential warm start after it; `solve_iteration()` solves the Stage B normal rows (unchanged) and then makes one tyre update from the CURRENT aggregate (`Lambda_n`, impulse-weighted effective point, rigid-body velocity there): implicit coupled 2x2 response, one ellipse of the aggregate load, resultant applied as linear impulse plus angular impulse about the effective point (re-seated each iteration), rolling resistance as one axle moment. New queries (aggregate impulse/load/point/velocity, axes, slip, forces, moments, carried impulse, response count). Lifecycle clearing in `_clear_state()` / `reset_warm_start()` / `finish()`. `create()` and a patch with tyre forces disabled are bit-identical to Stage B.
- `Simulation/Motorcycle/tire_model.gd`: shared pure helpers `into_ellipse()` and `implicit_tangential_impulse()` (the M2.4.2 arithmetic moved verbatim). `tire_ground_contact.gd` now calls them (same results; all M2.4.2 tests unchanged and passing).
- `Tests/test_tire_patch_force_integration.gd` (+ `.uid`): 29 tests (aggregate state, axes and slip, tangential solver, rolling resistance, sample density, lifecycle, configuration, determinism, driven wheel on a hub fixture).
- Documentation: `PHYSICS_CONVENTIONS.md`, `ARCHITECTURE.md`, `TESTING.md`, `PROJECT_STATE.md`, `PHASE_PLAN.md` (current stage updated in place), this file. Not changed: `AGENTS.md`, the solver, `ConstraintRow`, `ContactConstraint`, `SimWorld`, `MotorcycleBuilder`, scenes, masses, controls.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: baseline on a pristine export of `71717a1`: `godot --headless --path . --script res://Tests/run_tests.gd` -> 599 passed, 0 failed, 0 skipped (17026 checks). With the change: **628 passed, 0 failed, 0 skipped (17383 checks)**; no `SCRIPT ERROR` or parse error in the log. The suite was started detached, so its exit code was not captured. Single-file runs of the new file (a runner kept outside the repository): 29 passed.
- VERIFIED: fresh import (`.godot/` deleted) exit 0, 0 error or warning lines; `simulation_test`, `main`, `motorcycle_test` with `--quit-after 240`: exit 0, 0 error or warning lines.
- VERIFIED (not committed): five mutants of the patch (tangential impulse at the wheel centre, stale load, rolling resistance per sample, effective point = first sample, no coupling matrix) each fail at least one new test; an earlier attempt at the last mutant was a parse error and was redone.
- Not run: the 9000-frame scene run, anything graphical, any assembled-machine scenario with the patch tyre (the patch is not in the assembly).

## 5. Unresolved / limitations
`PROJECT_STATE.md` ("Tyre patch force integration (M2.5.3 Stage C)"). Review points: (1) one resultant force at the load-weighted point: no spin slip, no moment about the contact normal; (2) the shared helpers were extracted from `TireGroundContact` (verified by the unchanged M2.4.2 tests, not by a separate old-versus-new bit comparison); (3) a small slip is sensitive to the effective rolling height, so cross-resolution small-slip forces differ by a few percent; (4) the first two iterations of a saturated wheel are not yet on the ellipse; (5) the test runner exits 0 even when a test hits a `SCRIPT ERROR`: grep the log.

## 6. Next action
Await the user's review. The agent does not push or merge, and does not start Stage D.
