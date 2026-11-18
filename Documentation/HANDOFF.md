# Handoff

## Current objective
M1.1 — hinge joint with angular limits and velocity motor (first subsystem of M1). **M1.2 (contacts/friction) and M1.3 (split impulse) have NOT begun** and need explicit user authorization.

## Branch / commits
- Branch `m1/hinge-joint`, based on `7bc4dba` (M0.1.1, verified: `SimWorld.step_tolerance` present, 70/70 baseline tests). Nothing pushed or merged; `main` untouched.
- The uploaded repository has `main`/`origin/main` and no `foundation/simulator-core` branch, and hashes differ from older notes (M0.1.1 was `75eb62b`, now `7bc4dba`): the history was rebuilt with `git init` earlier, content verified by the baseline run. `AGENTS.md` still names `foundation/simulator-core` as the current branch; left untouched (out of scope).
- M1.1 is a single commit on top of `7bc4dba`; its hash is in the delivery manifest and the final report (a file cannot contain its own hash).

## M1.1 implemented
- `Simulation/Solver/hinge_joint_constraint.gd`: 8 scalar rows (3 anchor, 2 axis alignment, motor, lower limit, upper limit). Details and sign conventions: file header, PHYSICS_CONVENTIONS.md.
- Continuous angle: accumulated wrapped differences, no jump at +-PI, multi-revolution. Limits: unilateral rows, speculative bias inside the range, Baumgarte when violated. Motor: velocity row, bounds `+-max_torque*dt`, target clamped into the limit-admitted rate interval (prevents fighting).
- Generic extension (only one): `SimConstraint.validate()`; `SimWorld.add_constraint()` rejects invalid constraints; `preflight()` refuses steps while one is invalid. Ball joint unchanged.
- No change to solver, rows, integrator, `SimConfig`, defaults, or any existing test.

## Decisions / rejected
- Frame defined by an explicit reference direction per body; `from_world()` makes the creation pose angle 0.
- Rejected: wrapped atan2 as state; arbitrary perpendicular basis; row re-ordering (measured no effect); extending BallJoint; damping/more iterations as a fix.
- Both limits enabled by one flag, finite bounds only.

## Tests actually executed (Godot 4.7.stable.official.5b4e0cb0f, headless)
- Baseline before changes: 70 passed (1249 checks). Final: **114 passed, 0 failed, 0 skipped (3529 checks)**; 44 new in `Tests/test_hinge_joint.gd`. Seven deliberate hinge mutations (axis rows off, motor sign, bilateral limit, uncapped-by-dt motor, no unwrapping, no speculative bias, no motor clamp) were each caught.
- Fresh import (`.godot` deleted): exit 0, no errors. `simulation_test.tscn` and `main.tscn`, 240 frames headless: exit 0, no errors. **No visual inspection on a display was done.**

## Known limitations (measured; details in PROJECT_STATE.md)
- Limit stop convergence needs ~80 solver iterations (residual separation speed after a 3 rad/s impact: 10% at the default 20, 1.5% at 40, 0 at >= 80); the stop never penetrates.
- Free spin loses rate ~(w dt)^2 (the M0 frozen-Jacobian limit): 0.5% over 5 s at 3 rad/s, ~4% at 6 rad/s.
- Anti-parallel axes (b = -a) also satisfy the axis rows; angle tracker needs < PI relative rotation per sample.

## Exact next step (only after user authorization)
User review of this checkpoint; then M1.2 (contact constraints with friction), reusing the unilateral-row and speculative-bias pattern if appropriate.

## Delivery
Three separate files from `Tools/make_delivery.sh <commit>`: `motorcycle_multibody_<short>.zip`, `_manifest.txt`, `.sha256`. Work stops after delivery (see `AGENTS.md`).
