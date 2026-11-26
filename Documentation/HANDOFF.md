# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.3 — Distributed finite-width tire contact patch, **Stage B only** (area-weighted compliant normal support over the Stage A samples, aggregate patch constraint, lifecycle, tests, documentation). Stages C (tyre-force integration), D (motorcycle integration, debug view) and E (regression and documentation of the complete increment) were not started and M2.5.3 is not complete.

## 2. Git observation at delivery
- VERIFIED: base `main` = `8a246f9` (merge of PR #20, Stage A). Work is on local branch `m2/tire-patch-normal-support` created from that base; one commit; not pushed, not merged, no PR, no review or approval. The working tree was clean on `main` before the work. The commit SHA and artifact names are in the delivery report.

## 3. What changed
- New: `Simulation/Motorcycle/tire_patch_normal_support.gd` (`TirePatchNormalSupport`); `Tests/test_tire_patch_normal_support.gd` (20 test methods).
- Changed: `Documentation/PROJECT_STATE.md`, `PHASE_PLAN.md`, `ARCHITECTURE.md`, `PHYSICS_CONVENTIONS.md`, `TESTING.md`, `HANDOFF.md`.
- Not changed: the solver, `ContactConstraint`, `ConstraintRow`, `SimWorld`, `TireGeometry`, `TirePatchSampler`, `WheelGroundContact`, `TireGroundContact`, the tyre model, the builder and assembly, the debug views and the scenes. Nothing registers the new constraint.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: full suite `godot --headless --path . --script res://Tests/run_tests.gd` → **582 passed, 0 failed, 0 skipped (15952 checks)**, exit 0 (baseline 562 passed on `8a246f9`). Mutation checks (no area weighting, tensile support, speculative activation, spin in the rows, no hard stop, stale rows kept) each made at least one new test fail; the source was restored afterwards.
- Design finding during development, recorded for review: a first version let the rows see the wheel spin. A probe showed +28 % to +416 % load and a moment about the axle of 13 % to 20 % of `N R` at 30 to 60 rad/s. The rows now drop the axle component of the angular Jacobian (see `PHYSICS_CONVENTIONS.md`). Also, a free wheel on a plane topples in roll, so the long tests use a test-only vertical fixture.
- Not run: scene smoke runs and visual inspection (no scene, solver or simulator code changed; a reasoned statement, not a run). Nothing graphical was examined.

## 5. Unresolved / limitations
See `PROJECT_STATE.md` ("Tyre patch normal support"). Open design points for review: the hard stop is a discontinuous load; no speculative row (one step of overshoot bound); numerical damping `dt k_A` adds to `c_A`; default sample resolution and parameters are not chosen.

## 6. Next action
Await the user's review of Stage B and the explicit go-ahead for Stage C. The agent does not push or merge.
