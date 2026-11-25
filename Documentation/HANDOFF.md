# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.1.2.1 — Runtime bounding-box dimension refresh: follow-up correction to PR #18 (M2.5.1.2 geometry wireframe). Boxes were built in `setup()` and went stale when a source dimension changed later (a tyre-width change updated the tyre surface but not the wheel box). Add a per-box dimension signature, rebuild only the affected box, keep transforms independent, add a runtime regression test. M2.5.2 and later were not started.

## 2. Git observation at delivery
- Start (VERIFIED): `origin/pr-18` (`refs/pull/18/head`) = `3d02e43ecf92339c603b0260ed8e18632f5ccb6b`, parent `1181458` (main, the PR #17 merge); PR #18 is not merged (`main` still at `1181458`). My earlier local commit `fcad6ac` has the identical tree (`3c3789b`) as that head; the local branch `m2/geometry-wireframe-visualization` was moved to `3d02e43` (no work lost) and one follow-up commit added; not pushed, not merged. The follow-up SHA is in the delivery report.

## 3. What changed
- `Simulation/Debug/geometry_wireframe.gd`: every body slot (all six, also when skipped) holds `{name, body, instance, signature}`. A box signature is `[half x, y, z, centre x, y, z]` (body frame), a tyre signature `[width, crown radius, wheel radius]`. `update_wireframe()` calls `_refresh_box()` / `_refresh_tire()` before `_place()`: the current source dimensions (re-read each frame from the assembly's configuration and tyre geometry objects via the new `bound_entry()`; `bounds_entries()` is now a loop over it with unchanged results and messages) are compared with `same_signature()` (each component within `DIMENSION_TOLERANCE` = 1e-7 m, above float32 rounding, NaN never equal) and only a changed item is rebuilt. World pose is not in any signature: transforms are set every frame regardless. Invalid or unavailable sources skip the item with the existing diagnostics and bring it back when valid again. New read-only accessors `box_signature()`, `tire_signature()`; `box_count()` now counts boxes that have a mesh. The tyre comparison changed from exact equality to the same tolerance (no other change to tyre generation).
- Tests: two tests added to `Tests/test_geometry_wireframe.gd` (19 total): the front-width runtime mutation (front tyre surface and box rebuilt with test-written expected extents, profile and signatures; rear tyre and box, other four boxes: same mesh objects, same vertices, same signatures; transforms independent both ways; repeated updates and a 1e-9 m change rebuild nothing) and a body-size change (one box rebuilt, invalid size skipped with a diagnostic, restored).
- Docs: `PHASE_PLAN.md` (M2.5.1 / M2.5.1.1 now accepted and merged per PR #17 and the user's assignment text; M2.5.1.2 with this correction listed as awaiting review in PR #18; M2.5.2 untouched), `PROJECT_STATE.md`, `ARCHITECTURE.md`, `TESTING.md`, this file.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64)
- VERIFIED baseline on `3d02e43`: 510 passed, 0 failed (10340 checks).
- VERIFIED final: **512 passed, 0 failed, 0 skipped (10426 checks)**, exit 0, no leaked-object lines. The new tests FAIL when only the box refresh is removed from the update path (the PR #18 behaviour): front box stays 0.12 m wide, chassis box stale; the tyre-surface and rear assertions still pass there.
- VERIFIED: fresh import (`.godot/` deleted), `simulation_test`, `main`, `motorcycle_test` smoke runs (`--quit-after 240`) and the 9000-frame `motorcycle_test` run: 0 error or warning lines; `git diff --check` clean. Presentation-only claim: the existing test that compares body states and contact results with the wireframe off, on and toggled stays green.
- VERIFIED graphical, software rendering only (Xvfb, no GPU): screenshots before and after setting the front tyre width 0.12 -> 0.14 m at runtime were viewed: front surface and box wider, rear unchanged, red visual wheel mesh unchanged. The difference is small at the camera used; the numbers are confirmed by the signatures.
- UNVERIFIED: a real display or GPU; interactive use.

## 5. Unresolved / limitations
- A wheel box takes its width from the tyre geometry object whenever one exists, even if that geometry is invalid for the profile (e.g. width 0.20 with crown radius 0.07 is rejected by `TireGeometry`, so the tyre surface is skipped, but the box still follows the finite width). This is the unchanged M2.5.1.2 source-dimension semantics; not changed here.
- The four box-shaped bodies still use `MotorcycleBodySpec.visual_size` (no physical bounding volume exists).
- Per frame, each box builds a small dictionary to read its current dimensions; no mesh or node is created unless a dimension changed.

## 6. Next action
Await the user's review and an explicit assignment. The agent does not push or merge.
