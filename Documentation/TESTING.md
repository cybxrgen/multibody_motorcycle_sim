# Testing

**Authoritative for:** test-suite organization, test-writing conventions, commands for tests/import/scene runs, the three verification levels
(import/parse, headless runtime, visual), and how to interpret results.
**Belongs elsewhere:** the latest test/runtime results → `PROJECT_STATE.md` (and the current `HANDOFF.md`); what the tests protect physically →
`PHYSICS_CONVENTIONS.md`; commit/delivery rules → `AGENTS.md`.
**Update when:** test organization, conventions, commands or the verification methodology change. Do not paste run reports here.

## Environment
Godot 4.7 stable. Binary used for verification so far: `Godot_v4.7-stable_linux.x86_64` from godotengine/godot-builds release `4.7-stable`.
Run `--import` once after a fresh checkout (`.godot/` is git-ignored). If no Godot binary is available, say so explicitly and never claim a run (`AGENTS.md` §5).

## Commands (run from the repository root)
| Purpose | Command |
|---|---|
| Import / parse check (delete `.godot/` first for a fresh-checkout check) | `godot --headless --path . --import` |
| Full automated suite | `godot --headless --path . --script res://Tests/run_tests.gd` |
| Suite with a report file | `godot --headless --path . --script res://Tests/run_tests.gd -- --report=/tmp/report.txt` |
| Headless scene smoke run | `godot --headless --path . res://Scenes/simulation_test.tscn --quit-after 240` and the same for `res://Scenes/main.tscn` |
| Visual inspection | `godot --path .` on a machine with a graphical display |

Suite exit code: 0 all passed, 1 failures, 3 no tests found. The report lists passed / failed / skipped tests.

## Suite organization and conventions
- Tests are `Tests/test_*.gd` files extending `Tests/sim_test_case.gd`; the runner (`Tests/run_tests.gd`) discovers them and executes every `test_*` method.
- Tests must validate physical outcomes with justified tolerances, not merely that code runs. Each tolerance is documented inline in the test.
- Never loosen a tolerance just to pass; if one must change, record why.
- **Tolerance policy.** Godot `Vector3`/`Quaternion`/`Basis` are float32: use ≥ 1e-6 (relative to magnitude) for vector/basis quantities and never 1e-9 on vector results
  (`PHYSICS_CONVENTIONS.md`, "Numerical precision"). Tighter tolerances (e.g. 1e-9) are legitimate only for float64 scalars or exact discrete solutions of semi-implicit Euler
  (accumulator, simulation time). Comparisons against continuous or conserved-quantity references use O(dt) tolerances.
- Derive bounds from the mechanism where possible (e.g. hinge anchor drift from the Baumgarte steady state `r (ω dt)²/(2β)` with the measured peak ω).
- Conservation and decay tests state the exact mechanism: angular momentum about the origin changes by the anchor `C x lambda` term plus the split displacement term; a free hinge spin follows the frozen-Jacobian loss law. Do not assert `w = w0` or strict `L` conservation with a tolerance tuned to the old Baumgarte velocity, which masked both.
- Keep a new test honest: it should fail against the defect it targets (mutation or pre-fix check) and pass after the fix.
- A GDScript runtime error aborts the whole run (engine limitation); treat it as a failure.

### Coverage map (what each file protects)
- `test_conventions.gd`: handedness, rotation direction, quaternion/Basis product order, Basis helpers.
- `test_rigid_body.gd`: static body, free motion, gravity, constant force, force consumption, torque, fixed timestep, backlog cap, render interpolation.
- `test_mass_properties.gd`: m·m⁻¹, static body, principal inertias, rotated world inertia, full tensors, force-at-offset torque, impulse vs force semantics;
  inertia validation (valid diagonal/non-diagonal tensors, negative principal moments, singular, nonsymmetric, positive-determinant-but-invalid, small ~1e-30 and large ~1e12–1e20 tensors,
  scale invariance, conditioning boundary, non-finite tensors and mass, preserved static convention, refusal by `SimWorld.add_body`, `set_mass_properties` and `step()`).
- `test_accumulator.gd`: `advance()` accumulator edge cases for dt from 1/120 down to 1e-15 (zero elapsed time, partial steps, exact/multiple steps, split increments with no early or lost steps,
  default tolerance, max-steps cap policy). Tolerances: 1e-9·dt for accumulator/time, 1e-6 relative for float32 velocities.
- `test_config_validation.gd`: default config valid; each invalid value reported exactly once and naming its parameter; valid boundary values; no clamping; `advance()` and direct `step()` with every
  invalid case leave the world untouched; mutation of the config after world creation, recovery, null config; refusals counted and reported once.
- `test_rotation.gd`: normalization, analytic spin, world-frame omega, principal-axis spin, gyroscopic sanity.
- `test_constraints.gd`: anchor coincidence, static-dynamic coupling (joint force = m g), pendulum period vs theory, 5 s swing stability with and without warm start, two-link chain,
  recovery from axial/lateral/off-center impulses, dynamic-dynamic joint, unilateral/clamped rows.
- `test_momentum.gd`: equal/opposite impulses, constraint impulse symmetry, linear and angular momentum conservation; angular momentum about the origin is checked against the decomposition `sum C x lambda` (anchor impulses) + `sum m dx_split x v` (split pose correction), see the methodology notes above.
- `test_constraint_membership.gd`: world-membership invariant for ball, hinge and bare `SimConstraint`: acceptance (dynamic-dynamic, static-dynamic), rejection (A absent, B absent, neither, null constraint/bodies, bodies of another world with colliding ids), `preflight()` with a body erased or nulled after registration (also for a disabled constraint), `step()`/`advance()` refusal with exact state preservation and recovery, `remove_body()` policy (constraints on either side removed, others kept in order, non-member removal is a no-op), unrelated chain identical to a reference world, no stale constraint in the solver (exact free fall). State is compared for exact equality because nothing is integrated.
- `test_hinge_joint.gd`: A basic hinge (static/dynamic, arbitrary orientations, gravity, impulses, axis alignment, free axial rotation via τ/I_pivot, perpendicular rotation constrained, static body unchanged);
  B angle (zero convention, signs, ±π continuity, multi-revolution, second-order misalignment, world-orientation independence); C limits (block, inside, away, never pulls outward, dynamic pair,
  stable under gravity with holding impulse = gravity torque·dt, motor into/away from limits); D motor (acceleration, target under load, cap, cap scaling with dt, disabled, zero torque, signs,
  static reaction, equal-and-opposite, motor+limits stability); E solver (warm start, limit/motor toggling, linear and angular momentum with the analytic `C × λ` term); F input validation (invalid construction,
  limit/motor settings, world refuses to step, initial misalignment recovery).
  Tolerances: stop 5e-4 rad (measured arrival error ~1e-4 with ≥ 100 iterations); anchor drift bounds from the Baumgarte steady state (doubled) with the measured peak ω; spin tests keep ω dt ≤ 0.025 rad
  (loss ~(ω dt)²); angular momentum vs the analytic `Σ C × λ` at 2e-4 (float32 over 480 steps). Limit/motor tests use 100 iterations because limit convergence needs ~80 (`PROJECT_STATE.md`). Free-spin tests (a8, b5) compare with the frozen-Jacobian loss law `w^-2 = w0^-2 + f dt t` (`spin_loss_factor` in the file; `PHYSICS_CONVENTIONS.md`, "Stabilization: split impulses"), not with `w0`; violated-limit tests (c5) check the split behaviour (physical rate 0, pose moves by beta*violation); e5 adds the split-displacement term to the analytic `C x lambda`; e4 uses a rounding-justified 5e-5.
- `test_contact.gd`: `ContactConstraint`. Normal: ground stop and impulse = m·v, no attractive impulse (gap 0, gap > 0, penetrating but separating), impulse never negative through approach/separation/rest, bounds `[0, +INF)`, dynamic-dynamic inelastic contact, oblique normal with two rotated boxes against an independently derived `K`, off-center contact angular response, resting weight impulse (warm and cold), speculative bias (no tunnelling at 20 m/s, no early braking), penetration removed by the split channel (pose moves by beta*|g|, no velocity added; displacement scales with beta) and bounded penetration. Friction: μ = 0, resistance, static hold (impulse, slip, sustained tangential load), sliding at the Coulomb bound (exact impulse, velocity and spin), sliding-to-rolling (5/7 law), isotropy (every direction and tangent rotation), no friction without normal impulse, no normal component, basis independence for sliding and sticking with an anisotropic block, maximum dissipation (impulse antiparallel to residual slip), friction bound uses the current normal impulse in the same iteration, warm start clipped to the current disk (μ and load changes), two independent contacts equal their solo runs, four-corner support. Conservation/regression: linear momentum, angular momentum about the origin (also with a gap), static body exactly unchanged, disabled contact, coexistence with ball and hinge joints, validation at `add_constraint()` and at `preflight()` after mutation, membership invariant.
  Tolerances: single-contact solves are exact after one iteration, so float32 expectations use 1e-5; the four-corner sum uses 2 % (Gauss-Seidel on a statically indeterminate support); rolling speed 2e-3 (discrete gravity impulses); angular momentum 1e-4 absolute on values of order 5 (float32); sustained-load impulse 1 % (residual rotation of the I = 100 body). Tests that need exactness on a fixed pose call `solve_in_place()` (prepare, iterate, finish) instead of `World.step()`, because the step first applies the explicit gyroscopic term and moves the bodies.
  Regression sensitivity (record of the method, not results): defects injected into `ContactConstraint` one at a time — pulling normal row, no speculative bias, square friction clamp, Euclidean instead of K-metric projection, friction before normal, no warm-start clip, no positional correction, friction limit not tied to the normal impulse, lever arms at the anchors instead of the shared point, a tangent basis ignoring its rotation — must each fail at least one test.
- `test_split_impulse.gd`: split-impulse stabilization (M1.3). A: a penetrating contact, a ball joint, hinge anchor + axis error and a violated hinge limit are corrected in the pose with the physical velocity **exactly** unchanged (`==` comparison: the physical solve adds literally zero impulse at rest), no kinetic energy from the correction (zero gravity and resting under gravity), beta independence of the physical result, gravity/normal impulse/friction/motor still physical, and the split pass leaves physical velocities, physical accumulated impulses and all diagnostics (kinetic energy, point velocity, momentum) untouched. B: geometric gap decay `(1 - beta)^n g0`, speculative contact brakes with a physical impulse and has an inactive split row, no attractive impulse for a separating penetrating contact (including a speed below the old push-out speed), friction saturates at `mu` times the PHYSICAL normal impulse (different from the split impulse) and is identical to a beta = 0 run, frictionless contact, static bodies exactly unchanged also mid-step. C: monotonic decrease of ball, hinge-anchor and hinge-axis errors, violated lower/upper limits move in the correct direction without overshoot, speculative hinge limit stays physical with an inactive split row, motor rate (and no split displacement from the motor row) and free rotation, axial torque `alpha = tau/I_pivot`. D: split velocities start at zero and never leak (stale values discarded, zero after every step, no residual motion after a correction, split impulse rebuilt each step), three coupled constraints at rest inject exactly zero kinetic energy, dynamic-dynamic pairs keep their centre of mass, disabled and zero-error constraints produce no split impulse, invalid constraints still refused (registration, preflight, membership), bit-identical determinism, the angular-momentum change equals `dx_split x p`, beta as the only stabilization parameter (beta = 1 removes the error in one step, beta = 0 none, no extra `SimConfig` property).
  Tolerances: states that must be unchanged are compared with `==`; one-step corrections `(1 - beta) * error` use 1e-4 (second-order rotation term `r theta^2 / 2`) or 1e-6 where the contact is purely translational; analytic float32 expectations 1e-5. Joint tests use 100 iterations (limit stops need ~80, and the split pass is cold started).
  Regression sensitivity (record of the method, not results): defects injected one at a time — old velocity-level Baumgarte bias on the contact normal row, on the hinge limit rows or on the ball joint rows, split impulse applied to the physical velocity, split velocity not integrated, split velocity not cleared after the step or not zeroed before the solve, split impulse warm started, split pass ignoring the `enabled` flag, spurious split bias on the motor row, speculative contact moved into the split channel, friction limit including the split impulse — must each fail at least one test (the last one is an equivalent mutant while the physical pass runs completely before the split pass).
- Not covered by automated tests: `Scenes/`, `Simulation/Presentation`, `Simulation/Debug` (only the headless smoke run).

## Verification levels (never conflate)
1. **Import / parse** — `--import` exits 0 with no error lines: scripts load.
2. **Headless runtime** — automated suite and scene smoke runs: logic and no-crash behaviour only. Says nothing about how anything looks.
3. **Visual inspection** — run on a graphical display and look at camera placement, colors, debug geometry, HUD layout and interaction (Space/N/R/L/C/J, mouse orbit/zoom).
   Only this level may be reported as "visually verified", and the report must say what was actually looked at.
Report the highest level actually reached for each claim; mark the rest UNVERIFIED.

## Interpreting results
- The suite deliberately prints many `ERROR:` lines (with backtraces): validation tests build invalid mass properties/configs/constraints and `push_error` reports them. These are expected.
  A failure is a `[FAIL]` line, a non-zero exit code, or a `SCRIPT ERROR` line.
- A skipped test is not a pass: investigate why (`[SKIP]` carries a reason).
- Before trusting a new failing tolerance, find the root cause by measurement. Past lessons: float32 makes 1e-9..1e-12 vector tolerances unreachable (raise to float32-appropriate values and document, do not hide errors);
  `advance(0.1)` is capped at 10 steps although the frame clamp allows 12 (use a smaller frame or lift the clamp in cap tests); ill-conditioned bodies (0.1 m sphere on a 1 m rod) need many iterations from a cold start;
  fast spin (~50 rad/s) hits the documented frozen-Jacobian limit — keep test spin rates modest; `%e` is not supported in GDScript string formatting.
- Record latest results in `PROJECT_STATE.md` and the current `HANDOFF.md`, with the environment (Godot version string, headless or display) and which level was reached.
