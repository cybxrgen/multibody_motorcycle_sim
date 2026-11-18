# Architecture

Status legend: **Implemented** = exists in the repo at this commit. **Planned** = design intent only.

## Principle
One authoritative simulation (`SimWorld`) owns all physical state. Motorcycle parts and rider segments will be
independent rigid bodies coupled only through constraints. Godot nodes only visualize published state.

## Modules
| Module | Responsibility | Status |
|---|---|---|
| `Tests/` | Headless runner, assertion base class, Godot-convention checks | Implemented |
| `Tools/make_delivery.sh` | Verified ZIP + manifest + SHA-256 (three separate files) from an exact commit; `sha256sum -c`, commit stamp, file-list and manifest checks; optional verified copy | Implemented (M0.1) |
| `Simulation/Core/sim_config.gd` | All numerical parameters (`Resource`) + `validate()` / `is_valid()` (single home of parameter rules) | Implemented (validation M0.1) |
| `Simulation/Core/sim_mass_properties.gd` | Mass, inverse mass, full 3x3 local inertia + inverse, world transforms, box/sphere helpers; `validate_inertia()`, `valid`/`error_message` | Implemented (validation M0.1) |
| `Simulation/Core/sim_body_state.gd` | Kinematic snapshot + interpolation (render contract) | Implemented |
| `Simulation/Core/sim_rigid_body.gd` | Body state (COM pose, velocities, accumulators), force/torque/impulse API; `set_mass_properties` rejects invalid input (returns bool) | Implemented |
| `Simulation/Core/sim_integrator.gd` | Semi-implicit Euler, gyroscopic term, quaternion exponential update | Implemented |
| `Simulation/Core/sim_world.gd` | Body registry, accumulator-driven fixed stepping, pipeline, diagnostics | Implemented (owns constraints, runs solver; `preflight()` refuses invalid config/bodies, M0.1) |
| `Simulation/Solver/constraint_row.gd` | Scalar row: Jacobian, bias, softness, effective mass, accumulated impulse, [min,max] limits, warm start, PGS update | Implemented |
| `Simulation/Solver/sim_constraint.gd` | Base constraint between two bodies owning rows; `_update_rows` hook; `validate()`/`is_valid()` (default valid, M1.1) | Implemented |
| `Simulation/Solver/ball_joint_constraint.gd` | Ball joint: 3 rows, Baumgarte bias, anchor queries | Implemented |
| `Simulation/Solver/hinge_joint_constraint.gd` | Hinge: 8 rows (3 anchor, 2 axis alignment, motor, lower limit, upper limit), continuous angle, torque->impulse cap; extends `SimConstraint`, uses `ConstraintRow` unchanged, solved by the unchanged `ConstraintSolver` | Implemented (M1.1) |
| `Simulation/Solver/constraint_solver.gd` | prepare -> iterate rows -> finish; generic (no body-type logic) | Implemented |
| `Simulation/Debug/debug_draw_3d.gd` | Line batcher (ImmediateMesh) | Implemented |
| `Simulation/Debug/simulation_debug.gd` | World axes, COM markers, local axes, joint anchors, text report | Implemented |
| `Simulation/Presentation/body_visual_proxy.gd`, `simulation_renderer.gd` | Interpolated primitive-mesh proxies, read-only | Implemented |
| `Scenes/main.tscn`, `Scenes/simulation_test.tscn` (+ `.gd`) | Entry scene; inspection scenario (double pendulum, free spinner), camera, HUD, key controls | Implemented |

## Dependency rule
`Presentation/Debug → Core/Solver`, never the reverse. Core and Solver are `RefCounted` and scene-tree free.
Scripts reference each other through `preload` constants (no `class_name`), so headless runs need no import step.

## Decisions
- D1: custom engine only; no Godot physics objects (task requirement).
- D2: COM is the body reference point; mesh offsets are presentation-only.
- D3: inertia stored as full `Basis` to permit non-diagonal tensors later.
- D4: no `class_name` to avoid dependence on the editor's global class cache in headless mode.

## State ownership
`SimWorld.bodies` owns the list; each `SimRigidBody` owns its pose, velocities and accumulators. Only `SimWorld.step()`
and the integrator mutate them during a step. External code may apply forces/impulses between steps. Render code reads
`previous_state` / `published_state` only.

## Validation and failure behavior (M0.1, implemented)
- Rules live in two places only: `SimConfig.validate()` (parameters) and `SimMassProperties.validate_inertia()` plus the constructor
  (mass/inertia). Rules: `PHYSICS_CONVENTIONS.md` (inertia), `sim_config.gd` (config ranges). Nothing is clamped or repaired.
- `SimWorld.preflight()` runs at the start of **every** `advance()` and `step()` (SimConfig is a mutable Resource and can change after
  construction). If the config (or a null config) or any body's mass properties are invalid: `advance()` returns 0, `step()` returns
  false, **no state is touched** (accumulator, bodies, force accumulators, step count, time), `rejected_step_count` increments,
  `last_error` holds the reason, and `push_error` fires once per distinct message (not per call). Fixing the config resumes simulation.
- `SimWorld.add_body()` returns null for invalid mass properties. `interpolation_alpha()` returns 0 for an invalid timestep.
- API changes in M0.1: `SimWorld.step()` now returns bool (was void), `SimRigidBody.set_mass_properties()` returns bool (was void),
  `SimWorld.add_body()` may return null. Existing valid callers are unaffected.

## Constraint validation (M1.1, implemented)
`SimConstraint.validate()` (empty = valid) is the only generic extension M1.1 needed. `SimWorld.add_constraint()` returns null for a null or invalid constraint, and `preflight()` re-checks every constraint each step (limits, motor settings and frame vectors are mutable): an invalid one refuses the step with no state change, like invalid config/bodies. `BallJointConstraint` inherits the always-valid default; its behavior is unchanged.

## Hinge joint (M1.1)
Relation to the solver: rows are ordinary `ConstraintRow`s; inactive features (limits/motor off) keep bounds [0,0], so `ConstraintRow.warm_start` clamps any carried impulse to 0 and `solve` applies nothing. No solver or row change. Row layout, Jacobians, biases, angle tracking, limit and motor rules: header of `hinge_joint_constraint.gd` and PHYSICS_CONVENTIONS.md ("Hinge joint"). Rejected: extending `BallJoint` (kept isolated; the 3 anchor rows are duplicated on purpose); an arbitrary perpendicular basis for the axis rows (uses the joint's own reference direction instead, never singular); re-ordering rows (anchor rows last) — measured identical.

## Execution pipeline (implemented in `SimWorld.step`)
preflight (M0.1; refuses without side effects) -> capture previous state -> add gravity force (m*g) -> integrate velocities (F/m, gyroscopic alpha) -> constraint
prepare + PGS solve (velocity level) -> integrate positions (x += v dt, q = exp(w dt/2) q) -> publish state -> clear
accumulators. Step 7 ("stabilization") is not a separate pass; it is the Baumgarte bias inside the solver rows.

## Integration method
Semi-implicit Euler (velocities first, positions from new velocities). Rotation uses the exact exponential map of the
new world-frame omega applied as a pre-multiplied increment, then normalizes. Gyroscopic term is explicit
(`alpha = I_w^-1 (tau - w x I_w w)`) exactly as specified; this is only first-order accurate and does not conserve
energy/momentum exactly for fast asymmetric spin. An implicit gyroscopic solve is a deferred option.

## Deviations from the suggested layout
No `force_accumulator.gd`, `impulse_solver.gd`, `rigid_body_dynamics.gd`: force/impulse logic is ~30 lines and lives
on `SimRigidBody` (accumulators are two vectors; the "impulse solver" is the constraint solver in commit 3).

## Solver notes
- Rows are scalar and solved sequentially (Gauss-Seidel); a ball joint is three rows along world axes, so its 3x3 block is
  not solved simultaneously. Convergence is slower than a block solve for stiff mass ratios; `solver_iterations` is configurable.
- Warm starting is implemented: each constraint keeps its rows between steps and re-applies the previous accumulated impulse
  (scaled by `warm_start_factor`, clamped to the row limits). Ball-joint rows use fixed world axes so impulses stay meaningful.
- Row impulses obey `lambda` along `J^T`; ball-joint rows have `Jv_a = -Jv_b`, so linear momentum is conserved exactly and
  angular momentum up to `lambda * (C x e)`, i.e. the (small) constraint error.
- Failed approaches / rejected options: none yet (nothing has been run). Rejected up front: 3x3 block solve (spec asks for rows),
  split impulses (deferred to M1).

## Visualization contract
`Scenes/simulation_test.gd` is the only runner: `world.advance(frame_delta)` (fixed-step accumulator) then
`renderer.render(alpha)` / `debug_overlay.update_overlay(alpha)`, where alpha = `world.interpolation_alpha()`. Proxies and overlay
read `previous_state`/`published_state` and never write bodies. Mesh origin = body COM. Scenes build their node trees in code;
the `.tscn` files only attach scripts. Controls: Space pause, N step, R reset, L/C/J toggles, mouse orbit/zoom.
