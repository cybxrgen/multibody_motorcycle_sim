# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.1.1 — Tire geometry validation hardening: fix two reported defects of M2.5.1 (PR #17) in `Simulation/Motorcycle/tire_geometry.gd` without changing the profile model, camber behaviour, contacts or solver: (1) `profile_radius()` could return a negative radius; (2) an invalid result could expose a non-finite plane normal. M2.5.2 and later were not started.

## 2. Git observation at delivery
- Start (VERIFIED): clone of `https://github.com/cybxrgen/multibody_motorcycle_sim`; `refs/pull/17/head` = `86f8ede28dc070fc4db2e1b0ad670a039a5f2da8` (tree identical to the earlier local delivery commit `8db6a9d`, which is only a different SHA of the same patch). Clean tree. The local branch `m2/finite-width-tire-geometry` was set to that head and one follow-up commit added; not pushed, not merged. The follow-up SHA is in the delivery report.

## 3. What changed
- Root cause 1: `profile_radius()` validated the tyre without the wheel radius, so the rule "shoulder drop <= R/2" (which keeps `r(s) >= R/2 > 0`) never ran; e.g. width 0.5, crown 0.25, R 0.1 gave `r(w/2) = -0.15`. Correction: it now validates against `wheel_radius` (`is_valid(wheel_radius)`) and returns NaN unless the result is finite and > 0. `_evaluate` already validated against the wheel radius; it gained the same finite-and-positive guard as defence in depth.
- Root cause 2: `_fail()` echoed the caller's normal, so a NaN/INF normal came back in an invalid result. Correction: the normal is echoed only when finite, otherwise it is `Vector3.ZERO`. Valid results and finite rejected normals are unchanged.
- Tests: two regression tests in `Tests/test_tire_geometry.gd`. Documentation: this file and the verification numbers in `PROJECT_STATE.md` only (the technical documents already state the shoulder-drop rule and were accurate).

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display)
- VERIFIED reproduction before the fix: `profile_radius(0.1, 0.25)` = -0.15 for the 0.5 m / 0.25 m tread; invalid result carried normal `(0, nan, 0)`.
- VERIFIED baseline 491 passed; final 493 passed, 0 failed, 0 skipped (10194 checks); the two new tests fail on the uncorrected file (491 passed, 2 failed) and pass on the corrected one. Fresh import and the four headless scene runs: exit 0, 0 error/warning lines.
- UNVERIFIED: graphical inspection (never performed).

## 5. Unresolved / limitations
Unchanged from M2.5.1 (`PROJECT_STATE.md`, "Finite-width tyre geometry (M2.5.1)").

## 6. Next action
Await the user's review and an explicit assignment. The agent does not push or merge.
