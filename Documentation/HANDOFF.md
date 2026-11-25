# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.2 — Motorcycle collision geometry and basic controls: physical oriented-box ground collision for the four structural bodies through the generic contact (wheels keep the tyre contact only), estimated independent per-part masses with consistent inertia, continuous throttle / brake / steering controls replacing the `T` / `B` test torques, and a wireframe of the physical boxes only. M2.6 and later were not started.

## 2. Git observation at delivery
- VERIFIED: base `main` = `0af3d20` (merge of PR #18; PR #17 and PR #18 with follow-ups present), working tree clean at start. Work is on local branch `m2/motorcycle-collision-and-basic-controls`; one commit; not pushed, not merged. The commit SHA and artifact names are in the delivery report.

## 3. What changed
- New: `motorcycle_collision_box.gd`, `box_ground_contact.gd`, `motorcycle_controls.gd`, `motorcycle_control_spec.gd` (`Simulation/Motorcycle/`); config fields `chassis_box` … `swingarm_box`, `controls`; builder step 3e/3f (boxes, box contacts, controls); assembly members `collision_boxes`, `box_contacts`, `controls` and helpers; `MotorcyclePrototype.config_with_collision()`, mass constants (chassis 120 -> 128 kg, total 186 kg).
- Changed: `Debug/geometry_wireframe.gd` (physical boxes only; tyre surface and `visual_size` boxes removed), `Scenes/motorcycle_test.gd` (keys, HUD, `inputs_from_keys`; `TestInput` and `T`/`B` removed; the scene no longer sets tyre geometry).
- Tests: three new files, `test_geometry_wireframe.gd` rewritten, two earlier tests adapted (`TESTING.md`). Docs: all six technical documents.
- Generic solver, `SimWorld`, `ContactConstraint`, tyre model: not modified.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: baseline 512 passed; final **548 passed, 0 failed, 0 skipped (11089 checks)**. Fresh import, `simulation_test`, `main`, `motorcycle_test` (240 frames) and a 3000-frame `--fixed-fps 60` run of `motorcycle_test`: 0 error or warning lines. `git diff --check` clean.
- UNVERIFIED: any graphical inspection (wireframe, falling, driving) — not performed; a real display or GPU; the keyboard path through a real window.

## 5. Unresolved / limitations
- Box contacts keep all corners within the activation distance (no face/edge reduction); no box-box or self-collision.
- Zero steering input keeps the steering motor on with target rate 0 (rate lock), so the steering is not free.
- Brake cap uses half the one-step stopping torque (measured overshoot with the full cap was ~1 %).
- Masses are estimates; only the chassis changed from the M2.1 values.
- Earlier future-milestone numbering (contact patch etc.) was shifted in `PHASE_PLAN.md`; the user should confirm it.

## 6. Next action
Await the user's review and an explicit assignment. The agent does not push or merge.
