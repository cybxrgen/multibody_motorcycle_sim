# Project State

## Implemented systems (at this commit)
- Godot 4.7 project scaffold: `project.godot` (main scene `res://Scenes/main.tscn`), `Scenes/main.tscn` + `main.gd`, `Scenes/simulation_test.tscn` + `simulation_test.gd`.
- Custom rigid-body core: `SimConfig`, `SimMassProperties`, `SimBodyState`, `SimRigidBody`, `SimIntegrator`, `SimWorld`.
- Forces, torques, point forces, impulses, angular impulses; static bodies via zero inverse mass/inertia.
- Semi-implicit Euler with gyroscopic term and quaternion exponential orientation update.
- Fixed-timestep accumulator with frame clamp + step cap, interpolation alpha, published/previous state.
- Diagnostics: total linear/angular momentum, kinetic and potential energy.
- Constraint framework: `ConstraintRow` (Jacobian, bias, softness, limits, warm start), `SimConstraint`, `ConstraintSolver` (PGS), `BallJointConstraint`.
- Baumgarte velocity-level stabilization; static-body participation in joints.
- Minimal visualization: camera, light, primitive proxies with interpolation, world axes, COM markers, local axes, anchors, HUD, pause/resume/step/reset.
- Headless test runner and tests (see TESTING.md).
- M0.1 hardening: inertia validation (`SimMassProperties.validate_inertia`, `valid`/`error_message`), `SimConfig.validate()`, `SimWorld.preflight()` (invalid config or body mass properties => no step, no state change), verified delivery script, permanent user delivery gate in `AGENTS.md`.

## Planned systems
Nothing remains in M0 scope; see `PHASE_PLAN.md` for M1+.

## Verification status (what was actually done, by level)
Environment: Godot 4.7.stable.official.5b4e0cb0f, linux x86_64, headless, no display.
1. **Project import / parse: done.** `godot --headless --path . --import` from a fresh checkout (`.godot/` deleted first): exit 0, no error lines.
2. **Headless automated tests: done.** 70 passed, 0 failed, 0 skipped (1249 checks). See TESTING.md.
3. **Headless runtime of the scenes: done (smoke only).** `Scenes/simulation_test.tscn` and `Scenes/main.tscn` each ran 240 frames
   (`--quit-after 240`): exit 0, no error output. This shows the scripts load and run; it says nothing about how anything looks.
4. **Visual inspection on a graphical display: NOT done (unverified).** Camera placement, colors, debug geometry, HUD layout and
   interaction have never been looked at. No automated test covers `Scenes/`, `Simulation/Presentation` or `Simulation/Debug`.

## Known limitations / problems
- **Joint drift limits (measured):** (1) Baumgarte steady state error is `v^2 dt^2/(2 L beta)`: 3.3 mm for the 1 m / 4.4 m/s pendulum.
  (2) Scalar-row Gauss-Seidel converges slowly when a body has tiny inertia relative to m*r^2 (0.1 m sphere on a 1 m rod: cold-start error
  35 mm @20 iterations, 11 mm @50, 4.7 mm @100, 3.3 mm @200); warm starting reaches 3.3 mm at 50. (3) Fast spin degrades the joint: the
  Jacobian is frozen per step, error ~ r*sin(phi)*(|w|dt)^2/2; 17 mm at 50 rad/s (0.42 rad/step), independent of iteration count.
  Mitigations deferred: block (3x3) solve, sub-stepping, or re-linearization.
- Godot vectors are float32 (see PHYSICS_CONVENTIONS.md, "Numerical precision").
- Inertia validation does not enforce the rigid-body triangle inequality of principal moments, and rejects tensors with reciprocal condition number < 1e-5 (documented in PHYSICS_CONVENTIONS.md).
- Invalid configuration is refused (no stepping) rather than repaired; the caller must check `step()`'s return value or `last_error`.
- Explicit gyroscopic term: O(dt) energy / angular-momentum error for fast asymmetric spin.
- Only the ball joint exists; no contact/friction, hinge, limits or motors.
- Baumgarte stabilization injects a small amount of energy proportional to constraint error; scalar-row Gauss-Seidel converges slowly for large mass ratios.
- Joint-row accumulated impulses are only meaningful for the last step (warm start state).
