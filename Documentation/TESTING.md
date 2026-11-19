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
- `test_momentum.gd`: equal/opposite impulses, constraint impulse symmetry, linear and angular momentum conservation.
- `test_constraint_membership.gd`: world-membership invariant for ball, hinge and bare `SimConstraint`: acceptance (dynamic-dynamic, static-dynamic), rejection (A absent, B absent, neither, null constraint/bodies, bodies of another world with colliding ids), `preflight()` with a body erased or nulled after registration (also for a disabled constraint), `step()`/`advance()` refusal with exact state preservation and recovery, `remove_body()` policy (constraints on either side removed, others kept in order, non-member removal is a no-op), unrelated chain identical to a reference world, no stale constraint in the solver (exact free fall). State is compared for exact equality because nothing is integrated.
- `test_hinge_joint.gd`: A basic hinge (static/dynamic, arbitrary orientations, gravity, impulses, axis alignment, free axial rotation via τ/I_pivot, perpendicular rotation constrained, static body unchanged);
  B angle (zero convention, signs, ±π continuity, multi-revolution, second-order misalignment, world-orientation independence); C limits (block, inside, away, never pulls outward, dynamic pair,
  stable under gravity with holding impulse = gravity torque·dt, motor into/away from limits); D motor (acceleration, target under load, cap, cap scaling with dt, disabled, zero torque, signs,
  static reaction, equal-and-opposite, motor+limits stability); E solver (warm start, limit/motor toggling, linear and angular momentum with the analytic `C × λ` term); F input validation (invalid construction,
  limit/motor settings, world refuses to step, initial misalignment recovery).
  Tolerances: stop 5e-4 rad (measured arrival error ~1e-4 with ≥ 100 iterations); anchor drift bounds from the Baumgarte steady state (doubled) with the measured peak ω; spin tests keep ω dt ≤ 0.025 rad
  (loss ~(ω dt)²); angular momentum vs the analytic `Σ C × λ` at 2e-4 (float32 over 480 steps). Limit/motor tests use 100 iterations because limit convergence needs ~80 (`PROJECT_STATE.md`).
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
