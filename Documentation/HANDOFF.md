# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
Follow-up to the review of PR #23 (commit `3423569a`, M2.5.3 Stage C): (1) the patch's rolling-resistance response did not validate its derived values (P2); (2) a failed tyre response could resume in a later solver iteration of the same solve and `finish()` could clear the bookkeeping of an impulse still applied to the wheel (P2); (3) the M2.5.2 row of `PHASE_PLAN.md` still said "awaiting user review". Only these corrections were authorized. Stage D (builder integration), Stage E, the one-point `TireGroundContact`, the tyre force law and any solver redesign were not touched; M2.5.3 is not complete.

## 2. Git observation at delivery
- VERIFIED: base = PR #23 head `3423569a3261ef20a6059f6917bad5e8f7efa853` (fetched as `refs/pull/23/head`, clean tree at the start). Branch `m2/tire-patch-force-integration-review-fixes` created from it, one new local commit. Not pushed, not merged, no PR created, no review submitted. The commit SHA and artifact names are in the delivery report.
- The working changes were carried over from an earlier session's snapshot (applied to the base with `git apply`; the six files were byte-identical to the snapshot); every result below was re-run in this session.

## 3. What changed
- `Simulation/Motorcycle/tire_patch_normal_support.gd`: `_solve_rolling_resistance()` validates the target, the change to the applied impulse, the angular-velocity change and the resulting angular velocity before touching the wheel; an invalid value is a tyre failure (never clamped to zero; a valid zero response stays valid). One failure path `_fail_tire_response()` sets `_tire_failed` and `_withdraw_tire_response()` removes the applied tangential wrench and rolling moment using the existing bookkeeping. `_tire_failed` is terminal for the solve: guards in `_solve_tire_iteration()`, `_apply_tangential_wrench()` and `_solve_rolling_resistance()`; a non-finite aggregate also fails through the single path; both clearing branches of `finish()` withdraw before clearing. `prepare()` still resets the flag, so a new solve or re-contact recovers.
- `Tests/test_tire_patch_force_integration.gd`: three tests (`test_i_a_*` non-finite rolling resistance from finite inputs, `test_i_b_*` failure stays terminal for later iterations and recovers in the next solve, `test_i_c_*` `finish()` never clears a record the wheel still carries). They check linear and angular velocity, not only flags.
- `Documentation/PHASE_PLAN.md`: M2.5.2 row corrected in place (accepted and merged, PR #19); Stage C wording updated in place to "under review in PR #23, subject to the review corrections" (current-stage line, M2.5.3 row, remaining-work list); Stages D-E still not started.
- `PHYSICS_CONVENTIONS.md` (failure contract, rejected approach), `TESTING.md` (coverage entry), `PROJECT_STATE.md` (latest verification, two limitations), this file.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: fresh import (`.godot/` deleted) `godot --headless --path . --import` exit 0, 0 error or warning lines.
- VERIFIED: `godot --headless --path . --script res://Tests/run_tests.gd` with the corrections: **631 passed, 0 failed, 0 skipped (17544 checks)**, exit 0, no `SCRIPT ERROR` or parse error. Baseline on the unmodified PR #23 head (earlier session): 628 passed, 17383 checks, exit 0.
- VERIFIED: the same command with the ORIGINAL `tire_patch_normal_support.gd` and the new tests: exit 1, 628 passed, 3 failed (`test_i_a_*`, `test_i_b_*`, `test_i_c_*`); with the corrected file they pass.
- VERIFIED: per-file results read from the full-suite log: Stage C file 32, Stage B normal support 37, sampler 13, and the unchanged M2.4.2 files (rolling lifecycle 13, forces 23, slip 14, regression 12, speculative contact 11, motorcycle tires 12) all pass, none failed. The runner has no per-file option.
- VERIFIED: scenes `simulation_test`, `main`, `motorcycle_test` with `--quit-after 240`: exit 0, 0 error or warning lines. `git diff --check` clean.
- REPORTED (earlier session, not re-run): seven uncommitted mutants of the corrected implementation are each killed by at least one new test.
- Not run: the 9000-frame scene run; anything graphical; any assembled-machine scenario with the patch tyre (the patch is not in the assembly).

## 5. Unresolved / limitations
`PROJECT_STATE.md`. New: the active one-point `TireGroundContact._solve_rolling_resistance` has the same unvalidated update (by inspection only; out of scope and unchanged); `TireModel.rolling_resistance_force` returns exactly 0.0 when the squared quotient overflows and NaN when the quotient overflows (unchanged). The test runner exits 0 even when a test hits a `SCRIPT ERROR`: grep the log.

## 6. Next action
Await the user's review. The agent does not push or merge, and does not start Stage D.
