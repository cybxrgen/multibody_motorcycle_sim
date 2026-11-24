# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.4.2.1 — Prevent tire forces during speculative contact: the M2.4.2 tyre treated any positive normal impulse as physical load, so it applied longitudinal and lateral force, rolling resistance and a tangential warm start while the wheel was still separated from the ground inside the speculative activation distance. Smallest targeted correction in the tyre contact, regression tests for the missed boundary, no change to the generic speculative policy, the tyre model, its parameters or the suspension/drivetrain/rider (M2.5/M2.6 not started).

## 2. Git observation at delivery
- Start (VERIFIED): head of pull request #14 fetched as `refs/pull/14/head`, commit `78ccd46` (M2.4.2, on top of `c31940d`, the merge of PR #13), clean tree, clone of `https://github.com/cybxrgen/multibody_motorcycle_sim`. Work branch `m2/tire-speculative-contact` created from it. One local commit on that branch; not pushed, not merged, no pull request. The commit SHA is in the delivery report.
- Milestones (separate from Git): M2 in progress; M2.4.1 merged; M2.4.2 and this correction implemented, awaiting the user's review. M2.4 and M2 are not complete or accepted.

## 3. What changed
- **`Simulation/Motorcycle/tire_ground_contact.gd`:** `_touching = gap <= TOUCH_TOLERANCE` (start-of-step gap, the existing 0.1 mm constant of `WheelGroundContact`, penetration included) gates every tyre effect: no tangential warm start (the carried tyre impulse is cleared), zero tangential target, zero rolling-resistance target. `normal_load()`, the force limits and `friction_utilization()` report physical load only while touching. New queries `is_touching()` and `speculative_normal_impulse()`. `reset_warm_start()` clears the flag.
- **`wheel_ground_contact.gd`:** comment of `TOUCH_TOLERANCE` only. **No change in `Simulation/Core`, `Simulation/Solver`, the generic contact rows, the activation distance, tyre parameters, the friction envelope, the builder or the assembly.**
- **Tests:** new `Tests/test_tire_speculative_contact.gd` (11 tests: speculative approach for front and rear wheel with the positive normal impulse and the positive gap asserted, fast and slow approach, no warm start across separation and over six speculative steps, actual contact, penetration and the touch boundary, re-contact, natural landing, in-suite old-behaviour mutant). One existing test adapted: `test_motorcycle_tires.gd::test_g10` encoded the defect (a hand-applied steering input leaves a 0.15 mm gap and the test measured a tyre force across it); it now steps until the wheel touches, expectation and tolerances unchanged (`TESTING.md`).
- **Docs updated:** `PROJECT_STATE.md`, `PHYSICS_CONVENTIONS.md`, `ARCHITECTURE.md`, `TESTING.md`, `PHASE_PLAN.md`, this file. `AGENTS.md` untouched.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display)
- VERIFIED baseline on the unchanged PR #14 head: 464 passed, 0 failed, 0 skipped (8931 checks), exit 0.
- VERIFIED defect on the unfixed code (rear wheel 2 mm above the plane, 2 m/s downward): normal impulse 27.6 N s, longitudinal force -2077 N, lateral force -230 N, rolling moment 15.9 N m, `normal_load()` 3315 N; with the correction the last four are exactly zero.
- VERIFIED final: 475 passed, 0 failed, 0 skipped (9053 checks), exit 0, no `SCRIPT ERROR`; fresh import (`.godot/` deleted) exit 0, 0 error/warning lines.
- VERIFIED headless smoke: `motorcycle_test`, `simulation_test`, `main` with `--quit-after 240` exit 0, 0 error/warning lines; `motorcycle_test` with `--fixed-fps 60 --quit-after 9000` exit 0, 0 error/warning lines.
- UNVERIFIED: graphical inspection (never performed).

## 5. Findings worth keeping
- The new tests are only meaningful with an ACTIVE contact, `0 < gap <= activation distance` and a positive speculative impulse; the old suite used heights of 0.15 m and above, where the contact is inactive and discards everything.
- The generic normal row legitimately warm starts from the speculative impulse, so the first touching step after a speculative one carries about 1 % different load than a fresh contact; the tyre state itself starts cold.
- A hand-made kinematic change (the steering helper of the tests) can open a sub-millimetre gap; any test that sets state by hand and then expects tyre force on the next step must first make sure the wheel touches.

## 6. Unresolved / limitations
See `PROJECT_STATE.md` ("Tyre model (M2.4.2)"), in particular the new bullet: no tyre action on the landing step, judged from the start-of-step gap; the 0.1 mm tolerance is an illustrative fixture value. Everything else is unchanged.

## 7. Next action
Await the user's review and an explicit assignment for further work (M2.5 and M2.6 are not authorized). The agent does not push or merge.
