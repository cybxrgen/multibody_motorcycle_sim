# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
Investigate and fix the long-duration rotational instability of the inspection scene: the torque-free asymmetric spinner (box 0.6×0.3×0.15 m, ω₀ = (0.2, 6, 0.2) rad/s) gains angular velocity until, after ~85–100 s, `sim_integrator.gd` reports `The axis Vector3 (0.0, 0.0, 0.0) must be normalized` followed by `instance_set_transform: Condition "!v.is_finite()"` and NaN diagnostics. Scope: rotational integration stability, directly necessary numerical safeguards and regression tests only. M2 not started, not merged, not pushed.

## 2. Git state
- Base commit: `aabaee136d5d2fa6b49147c06f56e96fcb200f25` = `main` (VERIFIED). **Deviation from the assignment text:** PR #7 is already merged into `main` (merge commit `aabaee1`), and `main` has the same tree as `origin/m1/split-impulse-stabilization` (`d64d654`, M1.3.2); `git diff d64d654 aabaee1` is empty. The M1.3/M1.3.1/M1.3.2 work and its history are therefore present unchanged; the new branch is based on `main`. `PHASE_PLAN.md` still says M1.3 awaits acceptance (not changed here, the milestone decision is the user's) — **please reconcile**. The working tree was clean at the start; PR #7's web page was not inspected (no web access to the PR UI), only the Git history.
- Work branch `m1/rotational-integration-stability`, one commit on top of the base (hash in the delivery manifest and the final report). Not pushed, not merged, `main` untouched.

## 3. Diagnosis (VERIFIED by measurement, numbers in `PROJECT_STATE.md`)
- Reproduced on the unmodified code: first non-finite state at step 10279 (t = 85.658 s, 120 Hz), also through the real scene with `--fixed-fps 60` (1 axis error + 3860 `instance_set_transform` errors). Failure time scales ~1/dt (160.3 s at 240 Hz): first-order error.
- Root cause: the explicit gyroscopic update `ω += dt I_w⁻¹(−ω × I_w ω)` has a one-signed second-order error, `ΔT = +½ dt² c·I⁻¹c` (`c = ω × I_w ω`), and `|L|²` also grows every step. Kinetic energy rose from 1.06× (10 s) to 16× (80 s), `|ω|` from 6 to 24 rad/s; once `|ω| dt` reached ~3 the iteration became super-exponential (881 → 1.9e3 → 9.7e3 → … 4e18 rad/s in 0.05 s), overflowed float32 to `inf`, and `w / speed` became NaN, which is where the `Quaternion` constructor error appeared.
- Not causes (measured): the quaternion constructor (only where the failure became visible), loss of orientation normalization (`||q|−1| ≤ 1.2e-7` up to the overflow), world inertia consistency, the ball joint (the failure is identical without it: the anchor is at the COM, no torque), the pendulum (independent bodies). The documented known limitation (explicit gyroscopic term) **is** the explanation; no second defect was found.

## 4. What was done
- `SimIntegrator.integrate_velocities`: torque kick unchanged; gyroscopic part replaced by an implicit midpoint step of Euler's equations in the body frame (Newton in float64, `solve_free_rotation`), conserving `|L|` and rotational `T` exactly, hence bounding `|ω|` for any dt. Only the increment is transformed to the world frame (bodies with unchanged `L_b` keep ω bit for bit). Returns false on non-finite input / non-convergence. `integrate_positions`: a non-finite ω no longer reaches the `Quaternion` constructor (not a repair, the world reports it).
- `SimWorld`: state checks after velocity integration, after the solve and after position integration; a fault aborts the step, restores all bodies to the pre-step state, publishes nothing, records `numerical_fault` (body id and name, quantity, stage, step), reports once, refuses further steps until `clear_numerical_fault()`.
- `Tests/test_rotational_stability.gd`: 12 tests (A long-duration spinner, discrimination vs the old scheme, dt scaling; B zero/tiny/symmetric/asymmetric/torque cases and solver invariants; C containment). Documentation: `PHYSICS_CONVENTIONS.md`, `ARCHITECTURE.md` (D16, D17), `TESTING.md`, `PROJECT_STATE.md`, this file. `PHASE_PLAN.md` and `AGENTS.md` unchanged. No constraint solver, joint, split-impulse, contact, scene or existing test change.

## 5. Verification (environment: Godot 4.7.stable.official.5b4e0cb0f, Linux 6.18 x86_64, headless, 1 core)
- VERIFIED: baseline suite before changes 192 passed, 0 failed (exit 0); after: `godot --headless --path . --script res://Tests/run_tests.gd` → 204 passed, 0 failed, 5054 checks, exit 0.
- VERIFIED: regression sensitivity: replacing the midpoint increment by the explicit one fails 5 new tests (a1, a3, b3, b4, b6). Containment tests were not mutation-tested.
- VERIFIED: old vs new, identical conditions (`PROJECT_STATE.md`): old fails at 85.658 s with `|L|` ×4.0 and `T` ×16 at 80 s; new, 150 s: `|ΔL|/L0` 2.5e-6, `|ΔT|/T0` 4.6e-6, `|ω|` ∈ [6.005, 7.003], 25 tumbling flips/120 s, `L`-direction error ≤ 0.0157 rad; cost ≈ +8 µs per asymmetric body per step.
- VERIFIED: fresh import exit 0 (no error/warning lines); both scenes 240 frames headless exit 0; inspection scene 150 simulated seconds (`--fixed-fps 60 --quit-after 9000`) exit 0 with no error lines (old code: errors reproduced).
- **Not done / UNVERIFIED:** graphical inspection (no display here): that the spinner visibly tumbles and that no debugger errors appear in an interactive run is inferred from the invariants and the headless run only; headless runs are not visual verification. PR #7's web state. Mutation tests of the containment code. Performance on other machines.

## 6. Unresolved / notes
- The `L` direction is first-order accurate (bounded 0.0157 rad at 120 Hz); an exact scheme needs the free-rotor step in the position phase after the constraint solve (deferred, `ARCHITECTURE.md`). Constraint-coupled fast rotation still has the frozen-Jacobian limitation (`PROJECT_STATE.md`).
- Stale wording left in untouched tests: comments in `Tests/test_contact.gd` and `Tests/test_hinge_joint.gd` still say "explicit gyroscopic term" (their assertions are unaffected).
- Prototype scripts (float64 Python, diagnostic harness) are not committed.

## 7. Next action
Stop and wait for the user's review. Do not begin M2 or merge. To apply: see `AGENTS.md` §7.
