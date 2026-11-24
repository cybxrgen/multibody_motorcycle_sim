# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M2.5.1.2 — Toggleable geometry wireframe visualization: one runtime key in the motorcycle demonstration scene that shows oriented bounding boxes of the six dynamic bodies and the finite-width 3D tyre surfaces (from the M2.5.1 `TireGeometry`), presentation only. M2.5.2 and later were not started.

## 2. Git observation at delivery
- Start (VERIFIED): fresh clone of `https://github.com/cybxrgen/multibody_motorcycle_sim`, clean tree, `main` at `1181458` (merge of PR #17, containing M2.5.1 `86f8ede` and M2.5.1.1 `2efded8`). The work is one local commit on the new branch `m2/geometry-wireframe-visualization`, not pushed, not merged. Its SHA and the artifact names are in the delivery report.

## 3. What changed
- `Simulation/Debug/geometry_wireframe.gd` (new): one `Node3D`, off by default. Oriented boxes in the body frame: wheels use half extents (tyre width / 2, R, R) (tyre width from `TireGeometry`, else the config wheel width; R the contact-geometry wheel radius); the four box bodies use `MotorcycleBodySpec.visual_size` (the only per-body dimension in the configuration; for the prototype it is the same Vector3 that produced the inertia tensor, which a test checks). Tyre surface: line mesh in the wheel frame from `TireGeometry.profile_radius` (48 angles, 12 profile segments, 16 meridians, 816 segments per tyre), built once, rebuilt only when that tyre's width, crown radius or wheel radius changes; per frame only transforms are set from the interpolated published state. Missing bodies, missing or invalid geometry are skipped with entries in `diagnostics` (and one log warning each).
- `Scenes/motorcycle_test.gd`: key **G** toggles it; the HUD shows `[G] geometry wireframe: off/ON`; the scene assigns illustrative tyre geometry to the config (front 0.12 m / crown radius 0.07 m, rear 0.18 m / 0.10 m) because the prototype configuration has none and there would be nothing to draw. The assembly only stores the geometry, so the physics is unchanged. Existing keys (Space, N, R, T, B, L, C, J) are unchanged and the R reset rebuilds the wireframe and keeps its state.
- `TireGeometry`, `SimWorld`, `SimRigidBody`, solver and contacts are untouched (no new sampling API was needed).
- Tests: `Tests/test_geometry_wireframe.gd` (17 tests). Documentation: `PROJECT_STATE.md`, `ARCHITECTURE.md`, `TESTING.md`, this file. `PHASE_PLAN.md` was not edited.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64)
- VERIFIED automated, headless: baseline on `main` 493 passed; final **510 passed, 0 failed, 0 skipped (10340 checks)**, exit 0, no leaked-object lines (after fixing leaks in the new tests' own scene-free construction). Body states, step count, time and contact results are bit-identical with tyre geometry absent, wireframe on and updated every step, and wireframe toggled during a 360-step run. Fresh import (`.godot/` deleted), the three scene smoke runs and the 9000-frame `motorcycle_test` run: 0 error or warning lines. A separate 300-frame headless run with the wireframe on: no diagnostics.
- VERIFIED script-level: the scene's G key, status text and the unchanged L / Space handling are tested by instantiating the scene script (`_ready` called directly, `_unhandled_input` fed synthetic key events).
- VERIFIED graphical, software rendering only (Xvfb, OpenGL driver, no GPU): screenshots of the paused scene with the wireframe off, on (default camera) and on (side camera) were viewed. Off: no wireframe and the HUD says off. On: white boxes on all six bodies, green front and violet rear tyre surfaces following the wheels with visible width and crown, HUD says ON.
- UNVERIFIED: a real display/GPU, interactive key presses and mouse use, a motorcycle that is steered, cambered or moving fast in the scene (the camber and spin pose is tested numerically, not looked at).

## 5. Unresolved / limitations
- Box sizes of the four box-shaped bodies are the configuration's `visual_size`; no physical bounding volume exists, so they are only as authoritative as that field (checked against the inertia tensors for the prototype).
- A wheel's box is built at `setup()`; a tyre dimension changed afterwards updates the tyre surface but not the wheel box until the next `setup()` (nothing changes geometry at runtime today).
- Lines are 1 px with no depth test (readable through the bodies); `PHASE_PLAN.md` still lists M2.5.1 as awaiting review although PR #17 is merged (not touched, outside this assignment).

## 6. Next action
Await the user's review and an explicit assignment. The agent does not push or merge.
