# Handoff

## Current objective
M0.1.1 — narrow corrective fix of the fixed-timestep accumulator, on top of M0.1 (foundation hardening). Neither is M1.
M1 has NOT been started.

## M0.1.1: accumulator edge-case fix
- **Bug:** `SimWorld.advance()` compared `accumulator >= dt - 1.0e-12`. For `fixed_timestep <= ~1e-12` (still an accepted config) the
  slack is >= dt, so `advance(0.0)` on an empty accumulator ran physics steps (reproduced: dt 1e-13, 1e-12, 1e-15 all stepped).
- **Root cause:** an absolute tolerance independent of the timestep.
- **Fix:** new `SimWorld.step_tolerance(dt) = min(1e-12, 1e-9 * dt)`, used at the three places that had the literal. For dt >= 1e-3
  (including the 1/120 default) it is exactly the old 1e-12, so default behavior is unchanged; for smaller dt it is at most a billionth of
  a step (double rounding is ~1e-16 relative per addition). No minimum timestep was introduced; `SimConfig` rules are unchanged.
- **Tests:** `Tests/test_accumulator.gd` (8 tests, A-H: zero elapsed, tiny dt, partial step, exactly one step, multiple steps,
  split-increment accumulation, default behavior incl. tolerance == 1e-12, max-steps cap policy). Before the fix the tiny-dt tests
  failed (3 tests); after it all pass. No existing test or tolerance was modified.
- Not changed: solver, integrators, constraints, inertia validation, defaults, architecture, phase plan.
- Known limitation: for extremely small dt the cap/phase logic relies on double precision of `accumulator`; below ~1e-15 s timesteps are
  numerically meaningless anyway (tested down to 1e-15 only for the zero-elapsed case).

## Delivery status
The latest delivery (M0.1.1: ZIP, manifest, SHA-256) has been given to the user in the chat (M0.1 was delivered earlier as `78e4aec`). **Further work requires explicit user
authorization.** Do not begin M1 or any other implementation on your own; read `AGENTS.md` ("User delivery gate").

## Branch / commits
- Branch: `foundation/simulator-core`. Nothing was pushed or merged; `main` untouched.
- The upload for M0.1 contained **no `.git`**. Baseline commit `f691adf` ("Baseline import of uploaded snapshot") was created from the
  uploaded files; its content corresponds to the reported M0 commit `0907991` (the original history was not supplied, so equality
  with `0907991` is unverified; the original manifest's own checksum did not match its ZIP).
- M0.1 is commit `78e4aec` on top of `f691adf`; M0.1.1 is the single commit on top of `78e4aec`. A file cannot contain its own hash: its exact hash is in
  `deliveries/motorcycle_multibody_<short>_manifest.txt` and the final report (`git log -1`).

## Completed in M0.1
- Inertia validation: finite, symmetric, positive definite (Cholesky), condition number >= 1e-5, scale-normalized float64; invalid
  state observable (`valid`, `error_message`); world/body refuse it. Non-finite mass rejected; finite mass <= 0 still static.
- Inverse inertia computed in float64 (Basis.inverse() overflows float32 above ~1e12).
- `SimConfig.validate()` / `is_valid()`; `SimWorld.preflight()` guards `advance()` and `step()` (no partial step, no flood of errors).
- API changes: `SimWorld.step()` -> bool, `SimRigidBody.set_mass_properties()` -> bool, `SimWorld.add_body()` may return null.
- `Tools/make_delivery.sh` rewritten to verify archive, commit stamp, file list, `sha256sum -c`, manifest; sidecars kept outside the ZIP.
- Documentation made consistent; `AGENTS.md` now holds the permanent delivery gate.
- 22 new tests (62 total).

## Not completed / not done
- Visual inspection on a display (never done). M1 and everything after. Joint-drift/convergence improvements (deliberately out of scope).
- No physics, solver, integrator or default-parameter change was made.

## Tests actually executed (Godot 4.7.stable.official.5b4e0cb0f, headless)
- M0 baseline (`f691adf`): 40 passed (139 checks). After M0.1 (`78e4aec`, also the M0.1.1 starting point): 62 passed (1151 checks).
- M0.1.1 final: **70 passed, 0 failed, 0 skipped (1249 checks)**; with the new tests but the old tolerance, 3 of the 8 new tests failed (bug reproduced).
  Both scenes ran 240 frames headless, exit 0, no errors. Not viewed on a display. See TESTING.md.

## Known physics limitations
Unchanged from M0: Baumgarte drift floor, slow scalar-row GS for tiny-inertia bodies, fast-spin joint error, explicit gyroscopic term,
float32 vectors. Details in PROJECT_STATE.md.

## Updated behavior to know about
- Invalid config or invalid body mass properties => `advance()` returns 0 / `step()` returns false, no state change, `last_error` set.
- Inertia rules: PHYSICS_CONVENTIONS.md. Config rules: `SimConfig.validate()`.

## Delivery artifacts
`motorcycle_multibody_<short>.zip`, `_manifest.txt`, `.sha256` — three separate files, checksum verified with `sha256sum -c`.

## Recommended next milestone (only after the user authorizes it)
Look at the scene on a real display; then M1 (hinge joint, unilateral contacts), possibly a 3x3 block ball-joint solve first given the
measured convergence limits.
