# Architecture

**Authoritative for:** the module inventory and responsibilities, dependencies, data flow and state ownership, architectural boundaries,
and design decisions with their rationale.
**Belongs elsewhere:** what is implemented/verified and known limitations → `PROJECT_STATE.md`; milestone status → `PHASE_PLAN.md`;
equations and sign conventions → `PHYSICS_CONVENTIONS.md`; test methodology → `TESTING.md`.
**Update when:** a module is added/removed/re-scoped, a dependency or ownership rule changes, or a design decision is made or reversed.
Describes only modules that exist; planned systems are not listed here until they are built.

## Principle
One authoritative simulation (`SimWorld`) owns all physical state. Motorcycle parts and rider segments will be independent rigid bodies coupled
only through constraints. Godot nodes only visualize published state. The core is generic: no vehicle or rider special cases.

## Modules
| Module | Responsibility |
|---|---|
| `Simulation/Core/sim_config.gd` | All numerical parameters (`Resource`) + `validate()` / `is_valid()` — the single home of parameter rules |
| `Simulation/Core/sim_mass_properties.gd` | Mass, inverse mass, full 3×3 local inertia + inverse, world transforms, box/sphere helpers; `validate_inertia()`, `valid`/`error_message` |
| `Simulation/Core/sim_body_state.gd` | Kinematic snapshot + interpolation (render contract) |
| `Simulation/Core/sim_rigid_body.gd` | Body state (COM pose, physical velocities, solver-scratch split velocities, accumulators), force/torque/impulse API; `set_mass_properties` rejects invalid input (returns bool) |
| `Simulation/Core/sim_integrator.gd` | Semi-implicit Euler, gyroscopic term, quaternion exponential update; positions advance with physical + split velocity |
| `Simulation/Core/sim_world.gd` | Body and constraint registry, accumulator-driven fixed stepping, step pipeline (zeroes the split velocities around the solve), diagnostics, `preflight()` |
| `Simulation/Solver/constraint_row.gd` | Scalar row: Jacobian, bias, softness, effective mass, accumulated impulse, [min,max] limits, warm start, PGS update; split channel (split bias, split bounds, split accumulated impulse, `solve_split()`) |
| `Simulation/Solver/sim_constraint.gd` | Base constraint between two bodies owning rows; `_update_rows` hook; `solve_iteration()` and `solve_split_iteration()` hooks (default: every row in order); `validate()`/`is_valid()` (default valid) |
| `Simulation/Solver/ball_joint_constraint.gd` | Ball joint: 3 rows (physical `Cdot = 0`, split bias for the anchor error), anchor queries |
| `Simulation/Solver/hinge_joint_constraint.gd` | Hinge: 8 rows (3 anchor, 2 axis alignment, motor, lower limit, upper limit), continuous angle, torque→impulse cap; extends `SimConstraint`, uses `ConstraintRow` unchanged, solved by the unchanged `ConstraintSolver` |
| `Simulation/Solver/contact_constraint.gd` | Generic one-point contact: unilateral normal row (speculative physical bias; penetration removed through its split channel) + isotropic Coulomb friction (two tangent rows solved as one 2×2 block and projected onto a disk); extends `SimConstraint`, reuses `ConstraintRow`, validates its own data |
| `Simulation/Solver/constraint_solver.gd` | prepare → iterate `solve_iteration()` of each active constraint (physical pass) → zero the split velocities of the bodies involved → iterate `solve_split_iteration()` (split pass) → finish; generic (no body-type or contact logic) |
| `Simulation/Debug/debug_draw_3d.gd`, `simulation_debug.gd` | Line batcher (ImmediateMesh); world axes, COM markers, local axes, joint anchors, text report |
| `Simulation/Presentation/body_visual_proxy.gd`, `simulation_renderer.gd` | Interpolated primitive-mesh proxies, read-only |
| `Scenes/main.tscn`, `Scenes/simulation_test.tscn` (+ `.gd`) | Entry scene; inspection scenario (double pendulum, free spinner), camera, HUD, key controls |
| `Tests/` | Headless runner, assertion base class, test files (see `TESTING.md`) |
| `Tools/make_delivery.sh` | Verified ZIP + manifest + SHA-256 (three separate files) from an exact commit; see `AGENTS.md` §7 |

## Dependency rule
`Presentation/Debug → Core/Solver`, never the reverse. Core and Solver are scene-tree free (`RefCounted`; `SimConfig` is a `Resource`).
Scripts reference each other through `preload` constants (no `class_name`), so headless runs need no import step.

## State ownership and data flow
`SimWorld.bodies` owns the list; each `SimRigidBody` owns its pose, velocities and accumulators. `SimWorld.constraints` owns the constraints; each
constraint owns its rows (and their warm-start impulses). Only `SimWorld.step()` and the integrator mutate bodies during a step; the solver changes
bodies only through their velocities. External code may apply forces/impulses between steps. Render code reads `previous_state` / `published_state` only.

### Execution pipeline (`SimWorld._step_unchecked`, after `preflight`)
capture previous state → add gravity force (m·g) → integrate velocities (F/m, gyroscopic α) → zero split velocities → constraint prepare → **physical PGS pass** → **split-impulse PGS pass** →
integrate positions (x += (v + v_s) dt, q = exp((ω + ω_s) dt/2) q) → zero split velocities → publish state → clear accumulators.
Stabilization is the split pass: positional error is corrected through temporary split velocities that move the pose and are never stored as physical velocity (decisions D13–D15).
The physical pass never reads split state and the split pass never reads or writes physical velocities or impulses (it reads only poses and inertia through the shared Jacobians), so the ordering only fixes determinism. Friction, motor and speculative biases live entirely in the physical pass; `SimWorld` and `ConstraintSolver` contain no joint- or contact-specific stabilization logic.

### Integration method
Semi-implicit Euler (velocities first, positions from the new physical velocities plus the split velocities of the step). Rotation uses the exact exponential map of the new world-frame ω applied as a
pre-multiplied increment, then normalizes. The gyroscopic term is explicit (`α = I_w⁻¹ (τ − ω × I_w ω)`): first-order accurate, does not conserve
energy/momentum exactly for fast asymmetric spin. An implicit gyroscopic solve is a deferred option.

## Validation and failure behavior
- Rules live in two places only: `SimConfig.validate()` (parameters) and `SimMassProperties.validate_inertia()` plus the constructor (mass/inertia);
  constraints validate their own data in `SimConstraint.validate()` (empty = valid). Rules: `PHYSICS_CONVENTIONS.md` (inertia), `sim_config.gd` (config ranges). Nothing is clamped or repaired.
- `SimWorld.preflight()` runs at the start of **every** `advance()` and `step()` (`SimConfig` and constraint data are mutable after construction). If the
  config (or a null config), any body's mass properties, any constraint's data, or any constraint's world membership is invalid: `advance()` returns 0, `step()` returns false, **no state is touched**
  (accumulator, bodies, force accumulators, step count, time), `rejected_step_count` increments, `last_error` holds the reason, and `push_error` fires once per
  distinct message (not per call). Fixing the cause resumes simulation.
- `SimWorld.add_body()` returns null for invalid mass properties; `SimWorld.add_constraint()` returns null for a null constraint, a constraint with a null body or a body that is not a registered member of this world, or invalid constraint data.
  `interpolation_alpha()` returns 0 for an invalid timestep. `step()` returns bool; `SimRigidBody.set_mass_properties()` returns bool.
- `BallJointConstraint` inherits the always-valid default. `HingeJointConstraint` and `ContactConstraint` validate their own data in `validate()` (nothing is duplicated in `SimWorld`), so invalid contact data is refused both by `add_constraint()` and by `preflight()` after later mutation.

## Solver design
- Rows are scalar and solved sequentially (Gauss-Seidel); a ball joint is three rows along world axes, so its 3×3 block is not solved simultaneously.
  `solver_iterations` is configurable. Each iteration calls `SimConstraint.solve_iteration()` on every active constraint: the default solves its rows in order; a constraint whose rows are coupled overrides it.
- Warm starting: each constraint keeps its rows between steps and re-applies the previous accumulated impulse (scaled by `warm_start_factor`, clamped to the row limits).
  Ball-joint rows use fixed world axes so impulses stay meaningful.
- Split channel: each `ConstraintRow` carries `split_enabled`, `split_bias`, `split_min_impulse`, `split_max_impulse` and `split_accumulated_impulse` next to its physical fields. It reuses the row's Jacobian and effective mass and is solved against `SimRigidBody.split_linear_velocity` / `split_angular_velocity`. A row with `split_enabled = false` (the default, so hand-built rows are unaffected) is skipped. The split accumulated impulse is reset in every `prepare` (no warm start). Constraints fill the split fields in `_update_rows`: joint anchor and axis rows are always enabled (`split_bias = (β/dt) C`, bilateral); a hinge limit row and the contact normal row are enabled only while violated/penetrating (bounds `[0, +inf)`). Biases and equations: `PHYSICS_CONVENTIONS.md` ("Stabilization: split impulses").
- Row impulses obey `λ` along `Jᵀ`; ball-joint rows have `Jv_a = −Jv_b`, so linear momentum is conserved exactly and angular momentum up to `λ·(C × e)`, i.e. the (small) constraint error.
- Inactive hinge features (limits/motor off) keep impulse bounds [0,0]: `ConstraintRow.warm_start` clamps any carried impulse to 0 and `solve` applies nothing — no solver or row change was needed.
- Row layout, Jacobians, biases, angle tracking, limit and motor rules: header of `hinge_joint_constraint.gd` and `PHYSICS_CONVENTIONS.md` ("Hinge joint").
- Contact: 3 rows (normal, two tangent). The normal row is an ordinary scalar `ConstraintRow` with bounds `[0, +INF)`. The tangent rows only hold Jacobians and accumulated impulses; their solve is coupled (a disk whose radius is `mu` times the normal row's current impulse), so `ContactConstraint` overrides `solve_iteration()` (normal row first, then the friction block; the friction disk reads the physical normal impulse only) and `prepare()` (friction warm start projected into the current disk). Equations: `PHYSICS_CONVENTIONS.md` ("Contact"). The caller owns contact geometry (creation, anchor refresh as the patch moves, removal); the solver and `SimWorld` contain no contact logic.

## Decisions and rationale
- D1: custom engine only; no Godot physics objects as authority (task requirement).
- D2: COM is the body reference point; mesh offsets are presentation-only.
- D3: inertia stored as a full `Basis` to permit non-diagonal tensors.
- D4: no `class_name`, to avoid dependence on the editor's global class cache in headless mode.
- D5: no `force_accumulator.gd`, `impulse_solver.gd`, `rigid_body_dynamics.gd` (suggested layout): force/impulse logic is ~30 lines and lives on `SimRigidBody`; the "impulse solver" is the constraint solver.
- D6: invalid input is refused, never repaired or clamped (visible failure over hidden correction).
- D7: the only generic extension needed for the hinge was `SimConstraint.validate()`; the hinge extends `SimConstraint` directly and does not extend `BallJointConstraint` (kept isolated; the 3 anchor rows are duplicated on purpose).
- D8: hinge frame is defined by an explicit per-body reference direction (never singular, any orientation); `from_world()` (static factory in `hinge_joint_constraint.gd`) makes the creation pose angle 0.
- D9: hinge both limits enabled by one flag, finite bounds only; the motor target is clamped into the rate interval the limit rows admit so motor and limits never oppose.
- D10: world-membership invariant — every registered constraint's `body_a` and `body_b` are members of the same `SimWorld.bodies` (object identity, never body ids). Enforced generically for every `SimConstraint` subclass at `add_constraint()` and again in `preflight()` (the collections are public, so it is re-checked every step). `remove_body()` also removes every constraint that references the body (chosen over refusing the removal: `remove_body()`/`remove_constraint()` return nothing, and a constraint cannot outlive its body); unrelated bodies and constraints keep their order.

- D11: generic solver extension for coupled rows is one hook, `SimConstraint.solve_iteration()` (default loops the rows exactly as before; `ConstraintSolver` calls it instead of looping rows itself). `ConstraintRow`, `SimWorld` and the joints are unchanged. Chosen over a contact-specific branch in the solver and over a new row type: the friction disk couples two rows with a third, which a per-row `[min, max]` clamp cannot express.
- D12: contact friction is a circular Coulomb limit solved as a 2×2 block with K-metric projection (basis independent, maximum dissipation at convergence), not two clamped scalar rows. Friction is solved after the normal row in every iteration so the limit uses the current normal impulse. The shared world contact point (midpoint of the two anchors) is used for all lever arms so the impulse pair is collinear.

- D13: split impulses use the rows' existing Jacobians and effective masses through extra fields on `ConstraintRow` and one extra hook, `SimConstraint.solve_split_iteration()` (default loops the rows). Chosen over a second row class or a parallel constraint list (duplicated Jacobian construction) and over contact- or joint-specific branches in `ConstraintSolver`/`SimWorld`. `ConstraintSolver` only runs two passes; `SimWorld` only zeroes the split velocities around the solve and integrates them.
- D14: split velocities are scratch fields on `SimRigidBody` (so rows reach them like physical velocity), but are never part of physical state: temporary scratch quantities, zeroed before the solver and after the position integration, never published. *Direct isolation:* they are not included in the physical velocity, in the momentum or energy diagnostics, or in the motor, friction and contact-velocity calculations, which read the physical `v, w` only. They are not warm started (the accumulated split impulse is reset in `prepare`), as required for a correction that must not persist. *Indirect pose effects:* they are integrated into the pose, so a split angular correction can change a body's orientation. For an anisotropic body the new orientation changes the world-space inertia, so diagnostics calculated from the corrected pose and world inertia, such as spin angular momentum and rotational kinetic energy, can change indirectly although the physical angular velocity is unchanged. This is not a use of split velocity by the diagnostics; the physics is in `PHYSICS_CONVENTIONS.md` ("Stabilization: split impulses", orientation-dependent quantities).
- D15: the only stabilization parameter stays `SimConfig.baumgarte_factor` (beta); no new parameter, no hidden damping. Positional error uses `split_bias = (beta/dt) C`, the physical bias of the same row is 0 (joints) or the speculative `C/dt` (non-violated contact gap and hinge limit). The split pass is solved after the whole physical pass because the two do not interact.


### Rejected / deferred options (with reasons)
- 3×3 block solve: rejected for now (specification asks for scalar rows); remains an option for the known convergence limits (`PROJECT_STATE.md`).
- Split impulses via a separate position-level (nonlinear Gauss-Seidel) pass, a pose teleport, or a warm-started split impulse: rejected for M1.3. The first changes the row/Jacobian model and risks hidden corrections; a teleport is forbidden (`AGENTS.md` §4); a warm-started split impulse would carry a correction velocity across steps. Interleaving the split row with the physical row (solving both in one iteration) was not needed because the passes do not interact.
- Contact friction as two independently clamped scalar rows (square/pyramid region): rejected, anisotropic (allows `√2·mu·lambda_n` on diagonals) and dependent on the basis; and as radial Euclidean scaling of the block update: rejected, loses maximum dissipation for anisotropic `K`. Contact normal frame in world space only: rejected, a normal stored in body A's frame follows A's orientation (anchors still need caller refresh).
- Hinge: wrapped `atan2` as angle state (jumps at ±π; the continuous accumulated angle is used); an arbitrary perpendicular basis for the axis rows (singular cases); re-ordering rows (anchor rows last: measured identical result); extending `BallJointConstraint`; damping or more iterations as a fix for limit behaviour.

## Visualization contract
`Scenes/simulation_test.gd` is the only runner: `world.advance(frame_delta)` (fixed-step accumulator) then `renderer.render(alpha)` /
`debug_overlay.update_overlay(alpha)`, where alpha = `world.interpolation_alpha()`. Proxies and overlay read `previous_state`/`published_state` and never write bodies.
Mesh origin = body COM. Scenes build their node trees in code; the `.tscn` files only attach scripts. Controls: Space pause, N step, R reset, L/C/J toggles, mouse orbit/zoom.
