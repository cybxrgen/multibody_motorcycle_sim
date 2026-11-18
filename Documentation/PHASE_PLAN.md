# Phase Plan

## Current milestone: M0 — Simulator foundation (branch `foundation/simulator-core`)
- [x] C1 Scaffold, rules, docs, test infrastructure, delivery tooling
- [x] C2 Rigid body, mass properties, integrator, fixed-step world
- [x] C3 Constraint rows, PGS solver, ball joint
- [x] C4 Minimal visualization

**M0 complete and verified headless on Godot 4.7.stable (40/40 tests at M0; 62/62 after M0.1). Visual inspection on a display has not been done (see PROJECT_STATE.md).**

## Checkpoint: M0.1 — Foundation hardening (not M1)
Maintenance only; no solver, integrator or physics-default changes. Baseline: uploaded M0 snapshot (reported commit `0907991`);
the upload had no `.git`, so a baseline import commit was created first (see HANDOFF.md).
- [x] Fix A: inertia tensor validation (finite, symmetric, positive definite via Cholesky, condition number; scale-aware; invalid state observable)
- [x] Fix B: `SimConfig.validate()` and `SimWorld.preflight()` (no partial steps; `advance()` and `step()` protected)
- [x] Fix C: documentation made consistent (verification levels separated)
- [x] Fix D: `make_delivery.sh` verifies ZIP/checksum/manifest; sidecars kept outside the ZIP
- [x] AGENTS.md: permanent user delivery gate (one commit, one delivery, wait for the user)
M1 is **not started**; it requires explicit user authorization.

## Current milestone: M1 — Joints and contacts (IN PROGRESS; branch `m1/hinge-joint`)
- [x] M1.1 Hinge joint with angular limits and velocity motor (generic constraint, 5 DOF constrained, Baumgarte, warm start)
- [ ] M1.2 Contact (unilateral) constraints with friction — **pending, not started**
- [ ] M1.3 Split-impulse stabilization — **pending, not started**
M1 is NOT complete. M1.2 needs explicit user authorization.

## Deferred (not part of M0 / M0.1 / M1.1)
- M1 remainder: contact (unilateral) constraints with friction; split-impulse stabilization.
- M2: motorcycle bodies (chassis, steering, fork, swingarm, wheels), suspension, tire model, drivetrain.
- M3: rider articulated bodies, joint limits, muscle actuators, grips/seat/pegs.
- M4: controller layers (intent → rider controller → PD/PID joints), procedural animation, IK.

Dependencies: M1 needs M0 solver; M2 needs M1 joints+contacts; M3 needs M1 and M2 interfaces; M4 needs M3.
