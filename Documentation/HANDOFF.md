# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M1.3.1 — clarify split-angular-correction accounting (review finding on M1.3). The split impulses never directly modify physical velocity, but the split **angular** pose correction rotates anisotropic bodies, which changes their world inertia `I_world`; with the physical `ω` unchanged, `L_spin = I_world ω` and `T_rot = ½ ωᵀ I_world ω` can change. Documentation and regression coverage did not account for that. Narrow correction: one regression test and documentation; **no solver change**. Explicitly authorized by the user. M1.4, M2 and any other work were **not** started; M2 remains unauthorized.

## 2. Git state
- Base commit: `9af295c2230fafe57511f8e8e59325911344c8da` (`main`); M1.3 implementation commit `27d0594` (`M1.3: implement generic split-impulse stabilization`) is preserved unchanged as the parent of the follow-up. VERIFIED with `git log` / `git merge-base` (merge base of the branch and `origin/main` is `9af295c`).
- Branch: `m1/split-impulse-stabilization`. The working tree was clean at the start and matched the expected M1.3 state (branch at `27d0594`, equal to `origin/m1/split-impulse-stabilization`). VERIFIED.
- One focused follow-up commit on top of `27d0594` (hash in the delivery manifest and the final report; a file cannot contain its own hash). Not pushed, not merged, no PR state touched, `main` untouched. The delivered patch applies to `27d0594` (the exact documented parent).
- M1.3 is **not** accepted and not merged; it still awaits user review. The M1.3.1 correction awaits review too.

## 3. What was done
- `Tests/test_split_impulse.gd`: builder `tilted_hinge_spinner()` and `test_d10_split_angular_correction_changes_orientation_dependent_quantities_but_not_physical_velocity` (33 checks). Anisotropic box (principal moments 0.1217 / 0.1133 / 0.0217), `ω` = 3 rad/s along its hinge axis (a principal axis), axis tilted 0.4 rad plus a 1 cm anchor error, driven through the real `World.step()` (split pass + position integration; no hand-edited pose). A `β = 0` twin is the control, an isotropic body the counter-check. Measured: split `ω_s` ≈ 10.1 rad/s after the solve; physical `ω`, `v` unchanged (equal to the twin within 1e-5); the pose is rotated by `β tan(tilt)` = 0.0846 rad; `inertia_world` equals `R I Rᵀ` recomputed from the new orientation; `L_spin` changes by 0.023 (|L| = 0.065) and `T_rot` by +3.0 % at fixed `ω`; the twin and the isotropic body keep their values.
- Docs: `PHYSICS_CONVENTIONS.md` (replaced the broad "never changes physical velocity, momentum or kinetic energy" claims by the direct-isolation statement; added the orientation-dependent consequence with the closed form `½|ω|²(I₂ − I₁) sin²φ`; the translational term `Σ m Δx_split × v` is kept), `PROJECT_STATE.md` (limitation, capability wording, verification figures), `TESTING.md` (methodology note and D10 description), `PHASE_PLAN.md` (M1.3 row only; M2 not authorized), this file. `ARCHITECTURE.md` and `AGENTS.md` unchanged (nothing misleading found; D13–D15 already say split velocities are scratch, not that diagnostics are invariant).
- No solver, integrator or body source changed; no compensation of momentum or energy was added.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, linux x86_64, headless; binary from the user's upload, provenance UNVERIFIED as in M1.3)
All VERIFIED, run from the repository root:
1. Baseline before changes: `godot --headless --path . --script res://Tests/run_tests.gd` → exit 0, **191 passed, 0 failed, 0 skipped (4940 checks)**.
2. Fresh import (`.godot/` deleted): `godot --headless --path . --import` → exit 0, no error/warning lines.
3. After changes, full suite, same command → exit 0, **192 passed, 0 failed, 0 skipped (4973 checks)**, no `SCRIPT ERROR`.
4. New test alone: the runner has no per-test filter (and was not changed), so a scratch script outside the repository instantiated `test_split_impulse.gd` and called only the new method: `godot --headless --path . --script <scratch>.gd` → exit 0, 33 checks, 0 failures.
5. `godot --headless --path . res://Scenes/simulation_test.tscn --quit-after 240` → exit 0, no error/warning lines.
6. `godot --headless --path . res://Scenes/main.tscn --quit-after 240` → exit 0, no error/warning lines.
- Regression sensitivity of the new test: five injected defects, each restored through `git checkout` afterwards, each failing the new test (and others): split `ω` added to physical `ω`; split `v` added to physical `v`; split `ω` not integrated into the pose; `inertia_world` not refreshed after an orientation change; the forbidden `L_spin` compensation (`ω` rescaled to keep `I_w ω`).
- Visual inspection: **not done.** Smoke runs only show that the scenes start and run 240 frames.

## 5. Unresolved / notes
- The first version of the new test contained two weak checks (a float inequality of two angles; a muddled orientation condition in the isotropic counter-check); both were replaced by hinge-axis-direction checks before the final runs.
- The assignment asked for the new commit hash in this file; `AGENTS.md` forbids a file containing its own commit's hash, so it is only in the manifest and the final report.
- Frozen-Jacobian dissipation, cold-started split impulses and the other limitations from M1.3 are unchanged (`PROJECT_STATE.md`).
- Tooling: `Tools/make_delivery.sh` is not executable in the repository; run it with `bash`.

## 6. Next action
Stop and wait for the user's review of M1.3 and M1.3.1. Do not describe M1.3 as accepted or merged until the user says so. M1.4 and M2 are not authorized.
