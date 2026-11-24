# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.4.2 — Tire Behavior: longitudinal and lateral slip, longitudinal and lateral tyre forces, combined-slip friction limit and rolling resistance on the existing front and rear wheel-ground contacts, with exactly one owner of the tangential response; no engine, clutch, gearbox, final drive, rider or automatic control (M2.5/M2.6 not started).

## 2. Git observation at delivery
- Start (VERIFIED): `main` at `c31940d` (merge of pull request #13, M2.4.1), clean tree, clone of `https://github.com/cybxrgen/multibody_motorcycle_sim`. Work branch `m2/tire-behavior` created from it. One local commit on that branch; not pushed, not merged, no pull request. The commit SHA is in the delivery report.
- Milestones (separate from Git): M2 in progress; M2.4.1 merged; M2.4.2 implemented, awaiting the user's review. M2.4 and M2 are not complete or accepted.

## 3. What changed
- **No change in `Simulation/Core` or `Simulation/Solver`.** New in `Simulation/Motorcycle/`: `motorcycle_tire_spec.gd` (validated per-wheel tyre data), `tire_model.gd` (pure slip, force-law and rolling-resistance functions), `tire_ground_contact.gd` (`WheelGroundContact` subclass: normal response + the tyre as the only owner of the tangential response, implicit 2x2 solve per iteration with the same-iteration normal load, rolling-resistance axle moment). One-method hook `_prepare_active()` added to `wheel_ground_contact.gd` (default = previous behaviour). `MotorcycleConfig.front_tire`/`rear_tire` (default null, validated before any mutation), builder creates `TireGroundContact` for a wheel with a tyre (rollback unchanged), assembly `front_tire`/`rear_tire`, `tires()`, `set_tire_forces_enabled()`, `MotorcyclePrototype.tire_spec()`/`config_with_tires()`.
- **Friction ownership:** the generic Coulomb block is bypassed for tyre wheels; wheels without a tyre keep the M2.4.1 behaviour. Details: `ARCHITECTURE.md` ("Tyre model"), `PHYSICS_CONVENTIONS.md` ("Tyre behaviour").
- **Scene:** `motorcycle_test` uses `config_with_tires()`, shows slips, forces, load, utilisation and rolling-resistance moment per tyre, and has two TEST-ONLY external axle-torque toggles (`T` drive, `B` brake) applied by a provider defined in the scene script.
- **Tests:** `test_tire_slip.gd` (14), `test_tire_forces.gd` (23), `test_tire_rolling_lifecycle.gd` (13), `test_tire_regression.gd` (12, permanent mutants), `test_motorcycle_tires.gd` (12), fixture `tire_rig.gd` (contains the TEST-ONLY planar carriage fixture). No existing test was edited.
- **Docs updated:** `PROJECT_STATE.md`, `PHASE_PLAN.md`, `ARCHITECTURE.md`, `PHYSICS_CONVENTIONS.md`, `TESTING.md`, this file. `AGENTS.md` untouched.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display)
- VERIFIED baseline on unchanged `main`: 390 passed, 0 failed (8111 checks), exit 0, 147 s.
- VERIFIED final: 464 passed, 0 failed, 0 skipped (8931 checks), exit 0, no `SCRIPT ERROR`, 166 s; fresh import (`.godot/` deleted) exit 0, 0 error/warning lines.
- VERIFIED headless smoke (`--quit-after 240`): `motorcycle_test`, `simulation_test`, `main` exit 0, 0 error/warning lines; `motorcycle_test` with `--fixed-fps 60 --quit-after 9000` exit 0.
- VERIFIED scripted sustained runs (not committed) and the scene's test-input toggles exercised through the scene script headlessly: `PROJECT_STATE.md`.
- UNVERIFIED: graphical inspection (never performed); behaviour beyond the scripted runs.

## 5. Findings worth keeping
- Rolling resistance applied as a contact force is cancelled by the slip force of a free wheel (no deceleration); it is an axle moment now.
- An explicit tyre force is unstable for the wheel's axle inertia; the tangential response is solved implicitly inside the solver iteration, which also gives the same-iteration normal load.
- With `k_x != k_y` the direction of a saturated force is stiffness-weighted, not the sliding direction (documented limitation).
- A first independent-clamp mutant was not detected because the common-factor law already stays inside the ellipse; it was rewritten as a genuine per-axis law.
- A constant negative axle torque is not a brake (it reverses the machine after standstill); the scene's brake toggle opposes the spin instead.

## 6. Unresolved / limitations
See `PROJECT_STATE.md` ("Tyre model (M2.4.2)"): illustrative parameters, no relaxation/patch/camber thrust, no static friction at rest, cornering and combined scenarios only on a single wheel and the test-only fixture, graphics not inspected.

## 7. Next action
Await the user's review of M2.4.2 and an explicit assignment for further work (M2.5 and M2.6 are not authorized). The agent does not push or merge.
