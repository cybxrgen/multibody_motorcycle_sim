# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M1.3 — generic split-impulse stabilization: correct positional constraint error (ball anchor, hinge anchor, hinge axis alignment, violated hinge limit, contact penetration) through a separate split-velocity channel so the correction never changes physical velocity, momentum or kinetic energy.
Explicitly authorized by the user. M2 and any other work were **not** started.

## 2. Git state
- Base commit: `9af295c2230fafe57511f8e8e59325911344c8da` (`main` at the time of work). It contains the merged M1.2 (PR #5) and M1.2.1 (PR #6). VERIFIED with `git log` and `git branch -r --contains`; the user's statement that both were accepted is consistent with that history.
- Work branch: `m1/split-impulse-stabilization`, one commit "M1.3: implement generic split-impulse stabilization" (hash in the delivery manifest and the final report; a file cannot contain its own hash). The branch is local only: not pushed, not merged, not published. The working tree was clean at the start. VERIFIED.
- M1.3 is implemented and awaiting user review and acceptance. It is **not** accepted and not merged.

## 3. What was done
- `ConstraintRow`: split fields (`split_enabled`, `split_bias`, split bounds, `split_accumulated_impulse`), `compute_split_velocity()`, `apply_split_impulse()`, `solve_split()`; the split impulse is reset in `prepare()` (no warm start).
- `SimConstraint.solve_split_iteration()` (default: every row's split solve); `ConstraintSolver`: physical pass, then zero the split velocities of the involved bodies, then split pass. This is the whole solver extension (`ARCHITECTURE.md` D13–D15).
- `SimRigidBody`: scratch `split_linear_velocity` / `split_angular_velocity` and `clear_split_velocity()`. `SimIntegrator.integrate_positions` uses `v + v_s` and `ω + ω_s`; `SimWorld` zeroes the split velocities before the solve and right after the position integration. Diagnostics and `velocity_at_point` still use physical velocity only.
- `BallJointConstraint`, `HingeJointConstraint` (anchor, axis-alignment rows, violated limits), `ContactConstraint` (penetrating normal row): physical bias 0 (or the speculative `C/dt` while not violated), positional error through the split bias `(β/dt) C`. Motor, friction, speculative contact/limit biases and the contact normal impulse stay physical; friction reads the physical normal impulse only. `SimConfig.baumgarte_factor` is reused; no new parameter.
- Tests: `Tests/test_split_impulse.gd` (28 tests, with `.uid`); ten existing tests updated (below).
- Docs: `PHYSICS_CONVENTIONS.md` (split equations, channel table, consequences), `ARCHITECTURE.md` (pipeline, hooks, D13–D15, rejected options), `TESTING.md`, `PROJECT_STATE.md`, `PHASE_PLAN.md`, this file. `AGENTS.md` unchanged.

### Existing tests that changed (all verified by measurement against the old tree; none by guesswork)
Cause 1 — the test asserted the old Baumgarte velocity: `test_contact::test_penetration_correction…` (rewritten: pose moves by β|g|, zero velocity, displacement ∝ β); `test_hinge_joint::c5` (a violated limit now removes outward rate and moves the angle by β·violation instead of producing a positive rate).
Cause 2 — the old Baumgarte velocity masked the frozen-Jacobian kinetic-energy loss of a spinning hinge: `a8`, `b5` now compare with the derived law `ω⁻² = ω₀⁻² + f dt t` (prediction 1.97065 vs measured 1.97091; angle 21.426 vs 21.410, 0.08 %), instead of `ω₀` within 1–2 %. β = 0 runs in the old and new tree give the identical unstabilized loss (0.96861), the old β = 0.2 run gave 0.9985.
Cause 3 — the split displacement moves `x` without changing `p`, changing `Σ x×p`: `test_momentum`, `e5` and the contact angular-momentum test with a 1 cm gap (the last one runs with the split channel off, the other two use the explicit decomposition `ΣC×λ + Σ m Δx_split×v`). A β = 0 attempt for `test_momentum` was rejected because the drift grew (0.017) without Baumgarte-limited error.
Cause 4 — rounding: `e4` tolerance 1e-5 → 5e-5, justified by a √N random walk over ~3.8e5 impulse applications; the old code with β = 0 already gives 2.05e-5, the old 1e-5 held only through the old Baumgarte trajectory (6e-6). Split impulses add exactly zero physical momentum.
Cause 5 — measurement phase / convergence: `test_constraints::two_dynamic_bodies` now measures anchor velocity right after the solve (the post-step value is a swing-phase-dependent frozen-Jacobian lag, 0.017 m/s peak in both trees); `test_contact::mixed_with_ball_joint_pendulum` uses 100 iterations because the cold-started split pass converges slower (16 mm vs 4.3 mm peak anchor error at 20 iterations, 4.0 vs 3.3 mm at 100). The latter two keep their original tolerances.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, linux x86_64, headless)
The binary came from the user's upload `Godot_v4_7-stable_linux_x86_64.zip` (extracted binary `Godot_v4.7-stable_linux.x86_64`); its checksum was not compared with a published one (provenance UNVERIFIED, version string as printed by the binary).
1. Automated tests. VERIFIED: baseline on the base commit 163 passed, 0 failed (4528 checks). With the change: **191 passed, 0 failed, 0 skipped (4940 checks)**, exit 0, no `SCRIPT ERROR`.
2. Import/parse. VERIFIED: fresh import (`.godot/` deleted) exit 0, no error/warning lines.
3. Headless scene smoke. VERIFIED: `simulation_test.tscn` and `main.tscn`, 240 frames each, exit 0, no error/warning lines. Neither scene uses contacts or reports the split channel.
4. Visual inspection. **Not done.** UNVERIFIED.
- Regression sensitivity. VERIFIED: twelve injected defects; eleven caught by `test_split_impulse.gd`, one equivalent mutant (friction limit including the split impulse, benign while the physical pass completes before the split pass). The spurious motor split bias survived the first version of the tests and was caught after a pose check was added; only that one was re-run after the final test edits.
- Old-implementation counterfactual. VERIFIED: with the old Baumgarte bias on the contact normal row nine tests of the new file fail (e.g. 0.48 m/s and 0.115 J left in a body by a 2 cm penetration).

## 5. Unresolved / notes
- **Frozen-Jacobian dissipation is now visible** (3.1 % kinetic energy per second at 2 rad/s, dt = 1/120; O(dt)). Previously masked by Baumgarte energy injection. The mitigations (sub-stepping, re-linearization, block solve) remain undecided: `PROJECT_STATE.md`. This matters for the planned fast-spinning wheels.
- **Split impulses are cold started** (required), so high mass-ratio setups need more iterations (limitation in `PROJECT_STATE.md`).
- `SimWorld.total_angular_momentum()` includes the split displacement term; it is documented and tested.
- Contacts have no debug visualization and no scene uses them; visual behaviour of the corrected poses UNVERIFIED.
- Tooling: `Tools/make_delivery.sh` is not executable in the repository; it was run with `bash`. The commit signing state is stated in the final report.
- Two background mutation runs were interrupted by tool limits and left a mutated source file twice; both were detected and restored from a verified backup, and the final tree was re-verified (identical to the backup, 191/0 suite, clean import) before the commit. VERIFIED.

## 6. Next action
Stop and wait for the user's review and acceptance of M1.3. Do not describe M1.3 as accepted or merged until the user says so. M2 is not authorized and must not be started; it requires a separate explicit assignment.
