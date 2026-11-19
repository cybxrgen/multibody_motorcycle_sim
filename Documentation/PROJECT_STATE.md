# Project State

**Authoritative for:** what currently exists in the repository, the latest verified test/runtime results, and project-wide known
limitations and technical debt.
**Belongs elsewhere:** roadmap and milestone status → `PHASE_PLAN.md`; last assignment's report → `HANDOFF.md`; module design →
`ARCHITECTURE.md`; equations and conventions → `PHYSICS_CONVENTIONS.md`; how to test → `TESTING.md`.
**Update when:** a capability is added or removed, a persistent limitation is found or resolved, or verified results change.
Keep it a snapshot of the present: no roadmap, no diary, no resolved issues.

Evidence labels used here: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed),
**UNVERIFIED** (not checked).

## Purpose and maturity
A custom rigid-body and constraint physics foundation (Godot 4.7, GDScript) intended to become a motorcycle + rider multibody simulator.
Current maturity: a generic physics foundation with ball and hinge joints and a minimal inspection scene. There is **no** contact,
friction, collision, motorcycle, tire, drivetrain, rider or controller code. See `PHASE_PLAN.md` for what is planned.

## Repository state
- VERIFIED: simulator source and tests were last changed by the M1.1.1 commit ("M1.1.1: validate constraint world membership"), on top of `0fe49f1` (documentation-only workflow commits after M1.1 `cbc8c83`). The hash of the M1.1.1 commit is in the delivery manifest and `HANDOFF.md`.
- `git log --oneline --decorate` is the source of truth for the current commit; this file does not track documentation-only commits.

## Capabilities

| Area | Status | Where / notes |
|---|---|---|
| Godot 4.7 project scaffold, entry scene | Implemented | `project.godot` (main scene `Scenes/main.tscn`), `Scenes/main.gd` |
| Rigid-body core: `SimConfig`, `SimMassProperties`, `SimBodyState`, `SimRigidBody`, `SimIntegrator`, `SimWorld` | Implemented | `Simulation/Core/` |
| Forces, torques, point forces, impulses, angular impulses; static bodies (zero inverse mass/inertia) | Implemented | `SimRigidBody`, `SimMassProperties` |
| Semi-implicit Euler, explicit gyroscopic term, quaternion exponential orientation update | Implemented | `SimIntegrator` |
| Fixed-timestep accumulator (frame clamp, step cap, tolerance `step_tolerance`, interpolation alpha, published/previous state) | Implemented | `SimWorld` |
| Diagnostics: total linear/angular momentum, kinetic and potential energy | Implemented | `SimWorld` |
| Input validation: inertia, `SimConfig.validate()`, `SimConstraint.validate()`, constraint/world body membership (`add_constraint()`, `preflight()`, `remove_body()` also removes the body's constraints), `SimWorld.preflight()` (invalid ⇒ no step, no state change) | Implemented | `SimMassProperties`, `SimConfig`, `SimConstraint`, `SimWorld` |
| Constraint framework: `ConstraintRow`, `SimConstraint`, `ConstraintSolver` (projected Gauss-Seidel), warm starting, Baumgarte stabilization | Implemented | `Simulation/Solver/` |
| Ball joint (3 anchor rows); static-body participation | Implemented | `BallJointConstraint` |
| Hinge joint: 5 constrained DOF, free axis rotation, continuous multi-turn angle, optional lower/upper limits, velocity motor with torque cap | Implemented | `HingeJointConstraint` (8 rows) |
| Minimal visualization: camera, light, primitive proxies with interpolation, axes, COM markers, anchors, HUD, pause/step/reset | Implemented, **visually UNVERIFIED** | `Simulation/Presentation`, `Simulation/Debug`, `Scenes/simulation_test.*` |
| Headless test suite and runner | Implemented | `Tests/` (see `TESTING.md`) |
| Verified delivery tooling | Implemented | `Tools/make_delivery.sh` |
| Contacts / friction / collision, split-impulse stabilization | Planned (not started) | `PHASE_PLAN.md` |
| Motorcycle, suspension, tires, drivetrain, rider, controllers | Planned (not started) | `PHASE_PLAN.md` |

Capabilities above were checked against the source tree (module files and symbols exist). Behavioural correctness rests on the test results below.

## Latest verification results
Environment of the runs: Godot 4.7.stable.official.5b4e0cb0f, linux x86_64, headless, no display (official `Godot_v4.7-stable_linux.x86_64`).
- VERIFIED (M1.1.1 agent): `godot --headless --path . --script res://Tests/run_tests.gd` → **130 passed, 0 failed, 0 skipped (3967 checks)**, exit 0, no `SCRIPT ERROR`. The same suite on the unmodified base `0fe49f1` gave 114 passed (3529 checks). The new `Tests/test_constraint_membership.gd` has 16 tests; run against the original `sim_world.gd` (with only a `has_body` shim) 14 of them fail and a null body causes a script error, with the fix all pass.
- VERIFIED: fresh import (`.godot/` deleted) `godot --headless --path . --import` exit 0, no error/warning lines; `Scenes/simulation_test.tscn` and `Scenes/main.tscn` each ran 240 frames headless, exit 0, no errors (smoke only).
- Test-function counts per file are in the suite itself (`grep -c "^func test_" Tests/test_*.gd`); hinge joint 44 of the 130.
- UNVERIFIED / not done: visual inspection on a graphical display. Camera, colors, debug geometry, HUD and interaction have never been looked at.
  No automated test covers `Scenes/`, `Simulation/Presentation` or `Simulation/Debug`.
- The suite intentionally prints many `ERROR:` lines from invalid-input tests; failures are only the `[FAIL]` lines and the exit code (see `TESTING.md`).

## Known limitations and technical debt (project-wide)
Measured values come from earlier agents' experiments and are REPORTED unless re-run.

### Solver and joints
- **Frozen Jacobian per step (fast rotation).** Row Jacobians are linearized once per step. Joint error grows with spin: ~`r·sin(φ)·(|ω|dt)²/2`
  (17 mm at 50 rad/s, 0.42 rad/step), independent of iteration count. A hinge free-spinning loses rate ~(ω dt)²: 0.5 % over 5 s at 3 rad/s, ~4 % at 6 rad/s
  (independent of iterations; halving dt cut the kinetic-energy loss from 7.3 % to 1.2 %). Mechanism: Suspected; behaviour: Confirmed.
- **Baumgarte drift and energy.** Steady-state anchor error `v²dt²/(2Lβ)` (3.3 mm for a 1 m, 4.4 m/s pendulum). Baumgarte injects a small amount
  of energy proportional to the constraint error.
- **Slow scalar Gauss-Seidel convergence.** Large mass/inertia ratios converge slowly (0.1 m sphere on a 1 m rod: cold-start anchor error 35 mm at 20 iterations,
  11 mm at 50, 4.7 mm at 100, 3.3 mm at 200; warm starting reaches 3.3 mm at 50). The 3×3 anchor block is not solved simultaneously.
- **Hinge limit-stop convergence.** After a 3 rad/s impact on a limit (static body A) the residual separation speed is 10 % of impact speed at the default 20 iterations,
  1.5 % at 40, 0 at ≥ 80; the angle never penetrates the limit. Only the iteration count changes this (row order and β do not).
- **Hinge input domain.** Axes anti-parallel (b = −a) also satisfy the axis rows, so only joints that start satisfied are meaningful. The angle tracker needs
  < π relative rotation between samples (|rate|·dt < π; every step samples). Limits: both enabled by one flag, finite bounds only (no one-sided limit).
- **Explicit gyroscopic term.** O(dt) energy / angular-momentum error for fast asymmetric spin.
- Joint-row accumulated impulses are only meaningful for the last step (warm-start state).
- Interpretation note: after an inelastic stop a hinge can drift off the limit because the free pair of bodies legitimately tumbles (anchor forces exert axial torques). This is physics, not drift.
- Mitigations deferred (not decided): block (3×3) solve, sub-stepping, re-linearization, split impulses (planned as a milestone), implicit gyroscopic solve.

### Numerics and validation
- Godot `Vector3`/`Quaternion`/`Basis` are float32 (≈1e-7 relative error; ~8e-6 m resolution at 100 m). Double-precision Godot build is the deferred migration path (`PHYSICS_CONVENTIONS.md`).
- Inertia validation does not enforce the triangle inequality of principal moments and rejects tensors with reciprocal condition number < 1e-5.
- Invalid configuration, mass properties or constraints are refused (no stepping), not repaired; the caller must check `step()`'s return value or `SimWorld.last_error`.

### Verification gaps
- No visual inspection ever performed; presentation/debug code and scenes have no automated tests.
- A GDScript runtime error aborts the whole test run (engine limitation); treat as failure.

## Important unresolved architectural issues
- Solver strategy for stiff, fast or high-mass-ratio systems (block solve vs sub-stepping vs re-linearization) is undecided; it matters for fast-spinning bodies such as wheels (planned), where the frozen-Jacobian error above grows.
- Float32 vs double-precision build is undecided.
- Contact/friction design is not defined in the repository (the earlier plan only suggests reusing the hinge's unilateral-row and speculative-bias pattern if appropriate); see `PHASE_PLAN.md`.
