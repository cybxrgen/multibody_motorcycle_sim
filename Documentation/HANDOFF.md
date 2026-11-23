# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.4.1 — Wheel–Ground Contact: a flat static ground plane and one point contact per wheel for the six-body motorcycle, geometry refreshed at every fixed step, solved by the existing generic `ContactConstraint`; no tire force model, no roll stabilisation.

## 2. Git observation at delivery
- Start (VERIFIED): `main` at `1029b8b` (merge of pull request #12, M2.3), clean tree, shallow read-only clone of `https://github.com/cybxrgen/multibody_motorcycle_sim`. Work branch `m2/wheel-ground-contact` created from it. One local commit on that branch; not pushed, not merged, no pull request. The commit SHA is in the delivery report.
- Milestones (separate from Git): M2 in progress; M2.3 merged; M2.4.1 implemented, awaiting the user's review.

## 3. What changed
- **No change in `Simulation/Core` or `Simulation/Solver`.** New in `Simulation/Motorcycle/`: `wheel_ground_contact.gd` (`WheelGroundContact`, a `ContactConstraint` subclass whose `prepare()` re-derives the geometry every fixed step, re-seats the anchors/normal and deactivates itself, discarding carried impulses, when degenerate or clearly separated) and `motorcycle_ground_spec.gd`. `MotorcycleConfig.ground` (default null), builder step 3c with rollback, assembly references `ground`, `front_wheel_contact`, `rear_wheel_contact`, `set_ground_contacts_enabled()`, `remove_ground()`, `MotorcyclePrototype.config_with_ground()`.
- **Scene:** `motorcycle_test` now runs under gravity on the ground, with contact markers and a per-wheel gap / normal force / state HUD.
- **Tests:** `test_wheel_ground_geometry.gd` (14), `test_wheel_ground_contact.gd` (18), `test_motorcycle_ground.gd` (10), helper `ground_rig.gd` (contains the TEST-ONLY vertical-slide roll fixture). No existing test was edited.
- **Docs updated:** `PROJECT_STATE.md`, `PHASE_PLAN.md`, `ARCHITECTURE.md`, `PHYSICS_CONVENTIONS.md`, `TESTING.md`, this file. `AGENTS.md` untouched.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display; official binary from the godotengine release, downloaded with the user's authorization)
- VERIFIED baseline on unchanged `main`: 348 passed, 0 failed (7748 checks).
- VERIFIED final: 390 passed, 0 failed, 0 skipped (8111 checks), exit 0, no `SCRIPT ERROR`, after a fresh import (`.godot/` deleted; exit 0, 0 error/warning lines). Numbers are re-confirmed in the delivery report for the committed tree.
- VERIFIED headless smoke (`--quit-after 240`): `motorcycle_test`, `simulation_test`, `main` all exit 0, 0 error/warning lines; `motorcycle_test` with `--fixed-fps 60 --quit-after 9000` exit 0.
- VERIFIED sustained runs (scripted, not committed): 60 s unconstrained undisturbed (standing, gaps 0, 940 N front / 806 N rear, no fault); 60 s with the test-only fixture (support/weight 0.99999996); roll disturbance 0.01 rad/s topples past 30° at 1.7 s. Details: `PROJECT_STATE.md`.
- VERIFIED mutation checks (three implementation defects injected and reverted) — `PROJECT_STATE.md`.
- UNVERIFIED: graphical inspection of the scene (never performed).

## 5. Findings worth keeping
- `prepare()` is already a per-fixed-step hook; no generic extension was needed.
- The undisturbed assembly stays upright only through exact symmetry (knife-edge), not stability; the scene text and tests say so.
- After a topple only the wheels collide with the ground; the chassis passes through the plane (limitation, `PROJECT_STATE.md`).
- A first draft of the contact class set its fields after the generic constructor validated; the constructor now takes them (a construction-order defect found by the smoke test).

## 6. Unresolved / limitations
See `PROJECT_STATE.md` ("Ground contact (M2.4.1)"): no roll stability, wheel-only collision, thin-circle geometry, graphical inspection not done.

## 7. Next action
Await the user's review of M2.4.1 and an explicit assignment for further work (M2.5 is not authorized). The agent does not push or merge.
