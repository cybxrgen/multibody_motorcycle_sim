# Handoff

## Current objective
M0.1 — foundation hardening (inertia validation, config validation, documentation consistency, delivery integrity, delivery gate).
M0.1 is a checkpoint, **not** M1. M1 has NOT been started.

## Delivery status
The latest M0.1 delivery (ZIP, manifest, SHA-256) has been given to the user in the chat. **Further work requires explicit user
authorization.** Do not begin M1 or any other implementation on your own; read `AGENTS.md` ("User delivery gate").

## Branch / commits
- Branch: `foundation/simulator-core`. Nothing was pushed or merged; `main` untouched.
- The upload for M0.1 contained **no `.git`**. Baseline commit `f691adf` ("Baseline import of uploaded snapshot") was created from the
  uploaded files; its content corresponds to the reported M0 commit `0907991` (the original history was not supplied, so equality
  with `0907991` is unverified; the original manifest's own checksum did not match its ZIP).
- The M0.1 work is the single commit on top of `f691adf`. A file cannot contain its own hash: its exact hash is in
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
- Baseline (before changes, on `f691adf`): 40 passed, 0 failed, 0 skipped (139 checks).
- Final: 62 passed, 0 failed, 0 skipped (1151 checks). Import ok. Both scenes ran 240 frames headless without errors. See TESTING.md.

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
