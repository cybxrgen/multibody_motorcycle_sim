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
Current maturity: a generic physics foundation with ball and hinge joints, a generic one-point contact constraint with Coulomb friction, split-impulse positional stabilization, and a minimal inspection scene.
There is **no** collision detection, contact manifold, motorcycle, tire, drivetrain, rider or controller code. See `PHASE_PLAN.md` for what is planned.

## Repository state
- VERIFIED: the M1.3 work (split impulses, M1.3.1, M1.3.2) is merged into `main` (merge commit of pull request #7; `main` and `origin/m1/split-impulse-stabilization` have identical trees). Milestone M1.3 has also been formally accepted by the user (`PHASE_PLAN.md`); merging and acceptance are separate facts.
- Rotational-stability correction (pull request #8, branch `m1/rotational-integration-stability`): the implicit-midpoint gyroscopic update and numerical-fault containment, plus the follow-up that fixes the step-abort return value, `advance()` after a fault and the rollback of constraint warm-start state, are published on that remote branch (commit hashes in `HANDOFF.md`). Not merged into `main`; the PR awaits its own review and merge, which are the user's decision. This is independent of the accepted milestone M1.3.
- `git log --oneline --decorate` is the source of truth for the current commit; this file does not track documentation-only commits.

## Capabilities

| Area | Status | Where / notes |
|---|---|---|
| Godot 4.7 project scaffold, entry scene | Implemented | `project.godot` (main scene `Scenes/main.tscn`), `Scenes/main.gd` |
| Rigid-body core: `SimConfig`, `SimMassProperties`, `SimBodyState`, `SimRigidBody`, `SimIntegrator`, `SimWorld` | Implemented | `Simulation/Core/` |
| Forces, torques, point forces, impulses, angular impulses; static bodies (zero inverse mass/inertia) | Implemented | `SimRigidBody`, `SimMassProperties` |
| Semi-implicit Euler, implicit-midpoint gyroscopic update (conserves angular-momentum magnitude and rotational kinetic energy of a torque-free body), quaternion exponential orientation update | Implemented | `SimIntegrator`, `PHYSICS_CONVENTIONS.md` ("Rotational dynamics") |
| Numerical failure containment: non-finite / invalid state detected before the solver and before publishing, step aborted and rolled back (`step()` returns false, `advance()` stops and keeps the unprocessed time, carried constraint impulses invalidated), stepping refused until `clear_numerical_fault()` | Implemented | `SimWorld`, `ARCHITECTURE.md` (D17) |
| Fixed-timestep accumulator (frame clamp, step cap, tolerance `step_tolerance`, interpolation alpha, published/previous state) | Implemented | `SimWorld` |
| Diagnostics: total linear/angular momentum, kinetic and potential energy | Implemented | `SimWorld` |
| Input validation: inertia, `SimConfig.validate()`, `SimConstraint.validate()`, constraint/world body membership (`add_constraint()`, `preflight()`, `remove_body()` also removes the body's constraints), `SimWorld.preflight()` (invalid ⇒ no step, no state change) | Implemented | `SimMassProperties`, `SimConfig`, `SimConstraint`, `SimWorld` |
| Constraint framework: `ConstraintRow`, `SimConstraint` (with the `solve_iteration()` hook for coupled rows), `ConstraintSolver` (projected Gauss-Seidel), warm starting, split-impulse stabilization (`solve_split_iteration()` hook, split channel on `ConstraintRow`) | Implemented | `Simulation/Solver/` |
| Ball joint (3 anchor rows); static-body participation | Implemented | `BallJointConstraint` |
| Hinge joint: 5 constrained DOF, free axis rotation, continuous multi-turn angle, optional lower/upper limits, velocity motor with torque cap | Implemented | `HingeJointConstraint` (8 rows) |
| Minimal visualization: camera, light, primitive proxies with interpolation, axes, COM markers, anchors, HUD, pause/step/reset | Implemented, **visually UNVERIFIED** | `Simulation/Presentation`, `Simulation/Debug`, `Scenes/simulation_test.*` |
| Headless test suite and runner | Implemented | `Tests/` (see `TESTING.md`) |
| Verified delivery tooling | Implemented | `Tools/make_delivery.sh` |
| Generic one-point contact: unilateral normal row (speculative physical bias; penetration removed by the split channel), isotropic Coulomb friction as a coupled 2×2 block with K-metric disk projection, friction warm start projected into the current disk, static-body participation, self-validation | Implemented (M1.2, accepted and merged) | `ContactConstraint` (3 rows), `PHYSICS_CONVENTIONS.md` ("Contact") |
| Split-impulse stabilization: separate split velocities and split solve for ball/hinge anchor error, hinge axis alignment, violated hinge limits and contact penetration; physical velocities never directly modified (the pose change can still alter orientation-dependent quantities of anisotropic bodies, `PHYSICS_CONVENTIONS.md`); pose integrated with physical + split velocity; not warm started; reuses `SimConfig.baumgarte_factor` | Implemented (M1.3, merged into `main` and formally accepted by the user, `PHASE_PLAN.md`) | `ConstraintRow`, `SimConstraint`, `ConstraintSolver`, `SimIntegrator`, `SimWorld`, `PHYSICS_CONVENTIONS.md` ("Stabilization: split impulses") |
| Collision detection, contact manifolds/persistence, restitution | Not implemented | — |
| Motorcycle, suspension, tires, drivetrain, rider, controllers | Planned (not started) | `PHASE_PLAN.md` |

Capabilities above were checked against the source tree (module files and symbols exist). Behavioural correctness rests on the test results below.

## Latest verification results
Environment of the runs: Godot 4.7.stable.official.5b4e0cb0f, linux x86_64, headless, no display (official `Godot_v4.7-stable_linux.x86_64`).
- VERIFIED (PR #8 hardening follow-up), automated tests: `godot --headless --path . --script res://Tests/run_tests.gd` → **214 passed, 0 failed, 0 skipped (5169 checks)**, exit 0, no `SCRIPT ERROR`. Baseline on the unchanged PR #8 head (export of the implicit-midpoint commit): 204 passed, 0 failed (5054 checks). New: `Tests/test_fault_handling.gd` (10 tests, `TESTING.md`). One existing assertion was corrected because it encoded the defect (`test_rotational_stability.gd::test_c1`: `step()` now returns false for an aborted step); no tolerance changed.
- VERIFIED, regression sensitivity of the hardening: three defects injected one at a time into copies of the tree outside the repository — `step()` ignoring the abort (4 tests fail: `test_a2`, `test_c1`, `test_c2` of the new file and `test_c1` of the rotational suite), `advance()` continuing after an abort (3 fail: `test_b2`, `test_b3`, `test_b4`), no solver reset after an abort (3 fail: `test_b4`, `test_c1`, `test_c2`). A first attempt at the last mutant was invalid (parse error from an emptied block) and was redone.
- VERIFIED, long-duration rotational stability still holds after the hardening: `test_a1`/`test_a2`/`test_a3` of `test_rotational_stability.gd` (150 s asymmetric spinner, discrimination against the former explicit scheme, dt scaling) pass in the 214-test run, and, in this follow-up, a fresh import (`.godot/` deleted) exits 0 with no error or warning lines, `Scenes/simulation_test.tscn` and `Scenes/main.tscn` run 240 headless frames with exit 0 and no error or warning lines, and `Scenes/simulation_test.tscn --fixed-fps 60 --quit-after 9000` (150 simulated seconds) exits 0 with no error or warning lines; `git diff --check` is clean. Earlier measurements of the same regression (numbers below) were not re-measured individually.
- Earlier verification of the rotational-stability implementation (previous assignment, REPORTED unless re-run above): 204 passed (5054 checks). Environment of those runs as above.
- REPORTED (earlier assignment; not re-measured, only the pass/fail aspects are re-run above), long-duration rotational stability (spinner of `Scenes/simulation_test.gd`: box 0.6×0.3×0.15 m, 1 kg, ω₀ = (0.2, 6, 0.2) rad/s, 120 Hz, ball joint at the COM, float32 Godot): **old explicit gyroscopic update** — `T/T0` = 1.06 (10 s), 1.88 (40 s), 16.0 (80 s), `|L|/L0` = 4.0 (80 s), `|ω|` 24 rad/s at 80 s, 144 at 85.45 s, 881 at 85.60 s, 4e18 at 85.65 s, `inf` at step 10279 (t = 85.658 s) → `Quaternion` axis error, then NaN; orientation stayed normalized (`||q|−1| ≤ 1.2e-7`) throughout; identical failure without the joint; first failure 160.3 s at 240 Hz, none within 60 s at 480 Hz (`T/T0` = 1.19 at 50 s): O(dt). **New implicit-midpoint update** over 150 s: no failure, `max |ΔL|/L0` = 2.5e-6, `max |ΔT|/T0` = 4.6e-6, `|ω|` in [6.005, 7.003] rad/s (analytic range from the invariants 6.0049–7.0029), `||q|−1| ≤ 1.2e-7`, `inertia_world` equals `R I Rᵀ` exactly, 25 sign flips of body-frame `ω_y` in 120 s (tumbling), `L` direction error max 0.0157 rad (120 Hz; 0.0317 at 60 Hz and 0.0078 at 240 Hz in a float64 Python prototype, not committed). Full-scene configuration (double pendulum + spinner) also 150 s without fault.
- REPORTED, cost: single free asymmetric body in an otherwise empty world, headless, one core, 9600 steps: 6.2–7.8 µs/step (old) vs 14.3–16.1 µs/step (new), i.e. about +8 µs per asymmetric dynamic body per step (includes the state checks).
- REPORTED (earlier assignment): fresh import (`.godot/` deleted) exit 0, no error/warning lines; `Scenes/simulation_test.tscn` and `Scenes/main.tscn` 240 frames headless exit 0 without error/warning lines; `Scenes/simulation_test.tscn` with `--fixed-fps 60 --quit-after 9000` (150 simulated seconds through the real scene script) exit 0, no error lines — on the previous code the same command prints the reported `must be normalized` error once and 3860 `instance_set_transform: Condition "!v.is_finite()"` errors.
- VERIFIED, regression sensitivity: twelve defects injected into the split implementation one at a time (old Baumgarte bias on the contact, hinge limit and ball joint rows, split impulse applied to physical velocity, split velocity not integrated / not cleared after the step / not zeroed before the solve, split warm start, split pass ignoring `enabled`, split bias on the motor row, speculative contact moved to the split channel, friction limit including the split impulse). Eleven are caught by `test_split_impulse.gd`; one (friction limit including the split impulse) is an equivalent mutant while the physical pass runs completely before the split pass. A first run of the set missed the spurious motor split bias; a pose check was added and that defect is now caught. The set was not re-run in full after the final M1.3 test edits (only the motor defect was re-run). M1.3.1 added five integrator/body mutants run against the new D10 test (split `ω` or split `v` added to physical velocity, split `ω` not integrated into the pose, `inertia_world` not refreshed after an orientation change, forbidden `L_spin` compensation): D10 fails for each (VERIFIED).
- Test-function counts per file are in the suite itself (`grep -c "^func test_" Tests/test_*.gd`); hinge joint 44, contact 33, split impulse 29, rotational stability 12, fault handling 10 of the 214.
- UNVERIFIED / not done: visual inspection on a graphical display. Camera, colors, debug geometry, HUD and interaction have never been looked at. The effect of split correction on the visualization (poses are now corrected without velocity) was only smoke-run.
  No automated test covers `Scenes/`, `Simulation/Presentation` or `Simulation/Debug`; contacts have no debug visualization.
- The suite intentionally prints many `ERROR:` lines from invalid-input tests; failures are only the `[FAIL]` lines and the exit code (see `TESTING.md`).


## Known limitations and technical debt (project-wide)
Measured values come from earlier agents' experiments and are REPORTED unless re-run.

### Solver and joints
- **Frozen Jacobian per step (fast rotation).** Row Jacobians are linearized once per step. Joint error grows with spin: ~`r·sin(φ)·(|ω|dt)²/2`
  (17 mm at 50 rad/s, 0.42 rad/step), independent of iteration count. A hinge free-spinning loses rate by the law `ω(t)⁻² = ω₀⁻² + f dt t` (`f = m d⊥²/I_pivot`, `PHYSICS_CONVENTIONS.md`),
  O(dt): 3.1 % of the kinetic energy in 1 s at 2 rad/s and dt = 1/120 (1.6 % at 1/240, 0.8 % at 1/480); 3 rad/s decays to 2.42 rad/s in 8 s. The loss is **unmasked since M1.3**: the former velocity-level Baumgarte term
  injected energy that offset most of it (0.15 % over the same 1 s). Mechanism: Confirmed by the derivation above and by runs with beta = 0 (kinetic-energy ratio 0.96865 with split impulses, 0.96861 with beta = 0).
- **Positional drift and its correction.** Steady-state anchor error `v²dt²/(2Lβ)`. Since M1.3 the correction is a split impulse: it never directly modifies physical linear or angular velocity, but the pose is moved without momentum,
  so total angular momentum about the origin changes by `Σ m Δx_split × v` per step (checked in the tests). In addition, the split angular correction rotates anisotropic bodies, which changes their world inertia `I_world`; with the physical `ω` unchanged, `L_spin = I_world ω` and `T_rot = ½ ωᵀ I_world ω` can therefore change (M1.3.1, tested; no compensation is applied, by design). `total_angular_momentum()` reports physical state and shows both effects.
- **Split impulses are cold started.** They are rebuilt from zero every step (required, so the correction cannot persist), so high mass-ratio configurations converge more slowly than the warm-started physical rows: a 0.1 m sphere on a 1.2 m pendulum arm peaks at 16 mm anchor error at 20 iterations and 4.0 mm at 100 (4.3 mm and 3.3 mm with the former warm-started Baumgarte bias). More iterations are the only remedy tried.
- **Slow scalar Gauss-Seidel convergence.** Large mass/inertia ratios converge slowly (0.1 m sphere on a 1 m rod: cold-start anchor error 35 mm at 20 iterations, The split pass shows the same behaviour without the warm-start benefit (previous bullet).
  11 mm at 50, 4.7 mm at 100, 3.3 mm at 200; warm starting reaches 3.3 mm at 50). The 3×3 anchor block is not solved simultaneously.
- **Hinge limit-stop convergence.** After a 3 rad/s impact on a limit (static body A) the residual separation speed is 10 % of impact speed at the default 20 iterations,
  1.5 % at 40, 0 at ≥ 80; the angle never penetrates the limit. Only the iteration count changes this (row order and β do not).
- **Hinge input domain.** Axes anti-parallel (b = −a) also satisfy the axis rows, so only joints that start satisfied are meaningful. The angle tracker needs
  < π relative rotation between samples (|rate|·dt < π; every step samples). Limits: both enabled by one flag, finite bounds only (no one-sided limit).
- **Gyroscopic update accuracy.** The implicit midpoint step conserves `|L|` and `T` of a torque-free body, but the world-frame direction of `L` has an O(dt) bounded error (0.0157 rad max at 120 Hz for the spinner), and the phase of an unstable (intermediate-axis) rotation is not reproducible against a fine-dt reference (exponential sensitivity). With applied torque the invariants no longer apply (first-order torque kick). The Newton solve fails (step aborted, fault reported) only for `|ω| dt` far outside the tested range (largest tested: 0.5 in the solver unit test, 0.05 in the scene). Float32 storage gives a ~1e-6 round-off random walk in `|L|`, `T`.
- **Numerical fault policy.** A detected fault aborts the step, restores the bodies to the pre-step state, invalidates the carried constraint impulses (cold start after recovery) and freezes the world until `clear_numerical_fault()`; `advance()` stops at the first aborted step and keeps the unprocessed time (`ARCHITECTURE.md`, D17). Detection and rollback only, no automatic recovery. The fault-detection tests inject faults inside the step; cases where a fault arises organically from the solver with finite inputs were not constructed.
- Joint-row accumulated impulses are only meaningful for the last step (warm-start state); after an aborted step they are zero.
- Interpretation note: after an inelastic stop a hinge can drift off the limit because the free pair of bodies legitimately tumbles (anchor forces exert axial torques). This is physics, not drift.
- Mitigations deferred (not decided): block (3×3) solve, sub-stepping, re-linearization, moving the free-rotor step after the constraint solve for exact `L` direction.

### Contacts
- **Caller-owned geometry.** Anchors are fixed to the bodies; the caller must re-seat them as the contact patch moves (sliding, rolling) and must create/remove contacts itself. There is no collision detection, manifold or persistence; a multi-point contact is several independent constraints.
- **Inelastic only.** No restitution; static and kinetic friction share one coefficient.
- **Speculative bias is linear.** Closing is judged with the current linear contact-point velocity; rotation of the bodies inside one step is second order (frozen Jacobian, as above).
- **Gauss-Seidel on coupled contacts.** Several contacts on one body converge iteratively (a box on four corners carries its weight to within 2 % at 20 iterations with warm starting); statically indeterminate loads are not solved exactly.
- **Friction projection cost.** The disk projection uses 60 bisection steps, only when friction is saturated (sliding).
- Contact accumulated impulses are only meaningful for the last step (warm-start state).

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
