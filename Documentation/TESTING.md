# Testing

## Run
```
godot --headless --path . --script res://Tests/run_tests.gd
godot --headless --path . --script res://Tests/run_tests.gd -- --report=/tmp/report.txt
```
Exit code: 0 all passed, 1 failures, 3 no tests found. Report lists passed / failed / skipped tests.
Tests are `Tests/test_*.gd` files extending `Tests/sim_test_case.gd`; every `test_*` method is executed.
Tolerances: exact discrete solutions of semi-implicit Euler are compared at 1e-9; comparisons against continuous
or conserved-quantity references use O(dt) tolerances (documented inline in each test).

## Coverage at this commit
- `test_conventions.gd`: handedness, rotation direction, quaternion/Basis product order, Basis helpers.
- `test_rigid_body.gd`: static body, free motion, gravity, constant force, force consumption, torque, fixed timestep,
  backlog cap, render interpolation.
- `test_mass_properties.gd`: m*m^-1, static body, principal inertias, rotated world inertia, full tensors,
  force-at-offset torque, impulse vs force semantics. M0.1 additions (inertia validation): valid diagonal and non-diagonal
  tensors, negative principal moments, singular, nonsymmetric, positive-determinant-but-invalid (premise asserted in the test),
  small (1 g body, mm edges; ~1e-30) and large (~1e12, ~1e20) tensors, scale invariance over 1e-30..1e12, conditioning boundary
  (rcond 1e-4/2e-5 accepted, 5e-6/1e-7 rejected), non-finite tensors, non-finite and unrepresentable mass, preserved static
  convention, refusal by `SimWorld.add_body`, `set_mass_properties` and `step()`.
- `test_accumulator.gd` (M0.1.1): `advance()` accumulator edge cases for dt 1/120 .. 1e-15 (zero elapsed time, partial steps,
  exact/multiple steps, split increments with no early or lost steps, default tolerance == 1e-12, max-steps cap policy). Tolerances:
  1e-9 * dt for accumulator/time (the tolerance's own bound), 1e-6 relative for float32 velocities.
- `test_config_validation.gd` (M0.1): default config valid; 26 invalid values (each reported exactly once, naming its parameter);
  valid boundary values; no clamping; `advance()` and direct `step()` with every invalid case leave the world untouched
  (accumulator, step count, time, pose, velocities, force/torque accumulators, interpolation states); mutation of the config after
  world creation, recovery, null config; refusals counted and reported once.
- `test_rotation.gd`: normalization, analytic spin, world-frame omega, principal-axis spin, gyroscopic sanity.
- `test_constraints.gd`: anchor coincidence, static-dynamic coupling (joint force = m g), pendulum period vs theory,
  5 s swing stability with and without warm start, two-link chain, recovery from axial/lateral/off-center impulses,
  dynamic-dynamic joint, unilateral/clamped rows.
- `test_momentum.gd`: equal/opposite impulses, constraint impulse symmetry, linear and angular momentum conservation.

- `test_hinge_joint.gd` (M1.1, 44 tests): A basic hinge (static/dynamic, arbitrary orientations, gravity, linear/off-center impulses, axis alignment, free axial rotation via tau/I_pivot, perpendicular rotation constrained, static body unchanged); B angle (zero convention, signs, +-PI continuity, multi-revolution, misalignment second-order, world-orientation independence); C limits (block, inside, away, never pulls outward, dynamic pair, stable under gravity with holding impulse = gravity torque * dt, motor into/away from limits); D motor (acceleration, target under load, cap, cap scaling with dt, disabled, zero torque, signs, static reaction, equal-and-opposite, motor+limits stability); E solver (warm start, limit/motor toggling, linear and angular momentum with the analytic C x lambda term); F input validation (invalid construction, limit/motor settings, world refuses to step, initial misalignment recovery).
  Tolerances: stop 5e-4 rad (measured arrival error ~1e-4 with >= 100 iterations); anchor drift bounds derived from the Baumgarte steady state `r (w dt)^2/(2 beta)` (doubled) with the peak measured w; spin tests keep w dt <= 0.025 rad (loss ~(w dt)^2); angular momentum compared to the analytic `sum C x lambda` at 2e-4 (float32 over 480 steps). Limit/motor tests use 100 iterations (convergence above). Pre-existing tests and tolerances untouched.

## Not covered
- `Scenes/` and `Simulation/Presentation|Debug` have no automated tests (only the headless smoke run); visual inspection via `godot --path .` on a display has not been done.

## Latest verified results (M1.1)
Godot 4.7.stable.official.5b4e0cb0f, headless: baseline 70 passed (1249 checks); final **114 passed, 0 failed, 0 skipped (3529 checks)**, exit 0, 0 `SCRIPT ERROR`. Fresh import exit 0; both scenes 240 frames headless exit 0, no errors. Seven deliberate hinge mutants were each detected by the new tests. Visual inspection: not done. The suite prints expected `ERROR:` lines from deliberate invalid-input tests.

## M0.1.1 results (kept as history)
- M0.1.1: **70 passed, 0 failed, 0 skipped (1249 checks)**, exit 0, 0 `SCRIPT ERROR`; scenes `simulation_test` and `main` 240 frames headless, exit 0, no errors.
  Starting point was 62/62 (1151 checks). New tests failed before the fix (3 of 8) and pass after.

### M0.1 results
Godot 4.7.stable.official.5b4e0cb0f (linux x86_64, headless), executed in the M0.1 session:
- Baseline before any change (commit `f691adf`): **40 passed, 0 failed, 0 skipped (139 checks)**.
- M0.1 final: **62 passed, 0 failed, 0 skipped (1151 checks)**, exit code 0, 0 `SCRIPT ERROR` lines.
- The run prints ~79 `ERROR:` lines (+ backtraces). They are expected: the validation tests deliberately build invalid mass properties
  / invalid worlds and `push_error` reports them. Failures are reported only by the `[FAIL]` lines and the exit code.
- Import (`--import`, `.godot/` deleted first): exit 0, no error lines.
- Scene smoke test (not part of the suite): `godot --headless --path . res://Scenes/simulation_test.tscn --quit-after 240` and the same for
  `res://Scenes/main.tscn`: exit 0, no error output.
- **No tolerance was changed or loosened** in existing tests. New tolerances: I*I^-1 vs identity 1e-5 (float32 eps 1.2e-7 times the
  condition numbers used); symmetry 1e-5 and rcond 1e-5 are implementation thresholds justified in PHYSICS_CONVENTIONS.md.

## Levels of verification (do not conflate)
1. Automated headless tests (above) - done. 2. Headless scene runtime smoke test - done. 3. Visual inspection on a graphical display -
**not done / unverified**; the headless runs prove nothing about camera, colors, debug geometry or HUD layout.

## History: first-ever M0 run (before M0 fixes), kept for the record
First run had 24 failures. Root causes, all Confirmed by measurement:
- 21 failures: tolerances of 1e-9..1e-12 were unreachable because Godot vectors/bases/quaternions are float32. Tolerances were raised to
  float32-appropriate values (>= 1e-6 relative), not to hide errors; values are documented inline.
- 1 failure: test bug. `advance(0.1)` is capped at 10 steps (frame clamp allows 12). Test now uses 0.05 s and lifts the clamp for the cap test.
- 1 failure: cold-start large-swing test used too few iterations for the ill-conditioned 0.1 m sphere (see PROJECT_STATE.md). Now 200 iterations.
- 1 failure: off-center-impulse test spun the sphere at 50 rad/s, hitting the documented fast-spin joint limit. Now ~5 rad/s.
Also found by running the scene (not by tests): `%e` is unsupported in GDScript string formatting; fixed in `simulation_debug.gd`.

## Known limitations
- A GDScript runtime error aborts the whole run (engine limitation); treat as failure.
