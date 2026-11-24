# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.1 — Finite-width tire geometry: replace the zero-width wheel-centreline geometry assumption with an independently testable, motorcycle-specific tyre cross-section (finite configurable width, circular crown profile, per-wheel configuration, lowest point against a plane that responds to camber). Geometry foundation for M2.5.2 only: no contact patch, no solver, contact or tyre-force change. M2.5.2 and later were not started.

## 2. Git observation at delivery
- Start (VERIFIED): clone of `https://github.com/cybxrgen/multibody_motorcycle_sim`, branch `main` at `c9c7a1ad5e7f87d2daac95b30547a9a98638ab92` (merge of PR #16, the documentation reconciliation, so it is applied), clean working tree, `origin/main` identical. Work branch `m2/finite-width-tire-geometry` created from it; one local commit; not pushed, not merged, no pull request. The agent commit SHA and artifact names are in the delivery report.
- Milestones (separate from Git): M2 in progress; M2.4.x merged; M2.5.1 implemented and awaiting the user's review. M2.5.2, M2.5.3, M2.5.4, M2.6 and the integrated validation are not started and not authorized.

## 3. What changed
- New `Simulation/Motorcycle/tire_geometry.gd` (`TireGeometry`, with `.uid`): width and crown radius, `validate(wheel_radius)` (single home of the rules), pure static `lowest_point` and `point_at_offset`, `profile_radius`, `shoulder_drop`, `duplicate_spec`. Profile `r(s) = R - rho + sqrt(rho^2 - s^2)`; lowest point at `s = clamp(-rho (n.u), -w/2, w/2)`. Equations: `PHYSICS_CONVENTIONS.md`.
- `MotorcycleConfig`: optional `front_tire_geometry` / `rear_tire_geometry` (default null) and `_check_tire_geometry` (delegates to `TireGeometry.validate`, rejects a shared object). `MotorcycleBuilder`: stores copies in the new `MotorcycleAssembly.front_tire_geometry` / `rear_tire_geometry` after the existing steps (validation already happens before any registration). Contacts, tyre model, solver, Core and scenes are untouched.
- New `Tests/test_tire_geometry.gd` (16 tests, with `.uid`). Documentation: `PROJECT_STATE.md`, `ARCHITECTURE.md`, `PHYSICS_CONVENTIONS.md`, `TESTING.md`, `PHASE_PLAN.md`, this file. `AGENTS.md` unchanged.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display)
- VERIFIED baseline on unchanged `main`: 475 passed, 0 failed, 0 skipped (9053 checks), exit 0.
- VERIFIED final: 491 passed, 0 failed, 0 skipped (9422 checks); no existing test, expectation or tolerance changed. Fresh import (`.godot/` deleted) exit 0, 0 error/warning lines. Headless smoke (`--quit-after 240`) of `simulation_test`, `main`, `motorcycle_test` exit 0, 0 error/warning lines; `motorcycle_test` with `--fixed-fps 60 --quit-after 9000` exit 0, 0 error/warning lines.
- VERIFIED: the one failure seen during development (a 1e-9 tolerance on a float32 axle component) was a test tolerance error and was corrected to the documented 5e-6; no implementation change was needed.
- UNVERIFIED: graphical inspection (never performed); nothing draws the tyre profile.

## 5. Findings worth keeping
- The optimum `s* = -rho (n.u)` is exact and independent of the wheel radius and of the in-plane factor; only the shoulder clamp makes the result depend on width. Camber moves the contact toward the lower end of the axle.
- The tests use a brute-force search over the whole tread surface as an independent oracle; it resolves the minimum to about 1e-5 m.

## 6. Unresolved / limitations
See `PROJECT_STATE.md` ("Finite-width tyre geometry (M2.5.1)"): geometry only (the contact still uses the zero-width point), simple symmetric circular crown, bounds are model limits, no prototype sets a geometry, nothing viewed graphically.

## 7. Next action
Await the user's review and publication of the branch, and an explicit assignment for further work. The agent does not push or merge.
