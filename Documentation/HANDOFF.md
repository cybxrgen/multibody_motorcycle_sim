# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
Follow-up to the review of PR #24 (the corrected M2.5.3 Stage C candidate): (1) `_apply_tangential_wrench()` updated both wheel velocities and the record without validating the derived values, so finite impulses could still overflow float32 and leave a half-applied wrench; (2) the live roadmap still named PR #23 as the Stage C candidate. Only these were authorized. Stage D, Stage E, the one-point `TireGroundContact`, `MotorcycleBuilder`, the tyre law and any solver redesign were not touched; M2.5.3 is not complete and Stage C is not accepted or merged.

## 2. Git observation at delivery
- VERIFIED: base = PR #24 head `fa2cd5ccac8330bf2307e58b95e09d691621dd72` (fetched as `refs/pull/24/head`; parent `3423569a3261ef20a6059f6917bad5e8f7efa853` = PR #23 head; its tree equals the tree of the previously delivered commit `a77d927`). Clean tree at the start. Branch `m2/tire-patch-atomic-tangential-wrench` created from it, one new local commit. Not pushed, not merged, no PR created, no review submitted. `main` was `71717a1` (`git ls-remote`). The commit SHA and artifact names are in the delivery report.
- VERIFIED (GitHub web pages, the API was not accessible from this session): PR #23 and PR #24 were both OPEN, both targeting `main`, no review, no comment linking them; PR #24 lists two commits (the PR #23 commit and the corrections), so it contains and supersedes PR #23. UNVERIFIED: anything on GitHub after that observation.

## 3. What changed
- `Simulation/Motorcycle/tire_patch_normal_support.gd`: `_apply_tangential_wrench()` is validate-then-apply: scalar impulses and point, candidate linear and angular impulse, their differences against the applied wrench, the velocity changes and both resulting velocities are checked; only then are both velocities and the record (`_applied_linear`, `_applied_angular`, `_tire_lx`, `_tire_ly`) written. An invalid candidate calls the existing `_fail_tire_response()` (earlier tangential and rolling response withdrawn, terminal for the solve) and changes nothing else. `_solve_tire_iteration()` returns right after the re-seat call if that call failed. No second failure mechanism; no clamping.
- `Tests/test_tire_patch_force_integration.gd`: `test_i_d_*` (+ helper `tangential_candidates`): four cases (derived linear impulse overflows; angular impulse overflows through a finite lever arm; finite impulses whose velocity update overflows; non-finite scalar) x two modes (first wrench: wheel bit-identical; after a real response: withdrawn and equal to the tyre-less twin), then later iterations, `finish()`, and recovery equal to a cold solve. The test recomputes the candidates to prove the fixture overflows in a DERIVED quantity.
- `Documentation/PHASE_PLAN.md`: Stage C wording in the current-stage line, the M2.5.3 row and the remaining-work list now names PR #24 as the corrected candidate under review and PR #23 as still open and superseded by it (in place). `PHYSICS_CONVENTIONS.md` (atomic application, rejected approach 8), `TESTING.md` (group I), `PROJECT_STATE.md`, this file.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: fresh import exit 0, 0 error or warning lines.
- VERIFIED: `godot --headless --path . --script res://Tests/run_tests.gd`: **632 passed, 0 failed, 0 skipped (17936 checks)**, exit 0 captured, no `SCRIPT ERROR`. The 631 earlier tests have identical names and check counts; baseline of the same tree (previous delivery): 631 passed, 17544 checks.
- VERIFIED: `test_i_d_*` fails on the PR #24 implementation (single-file run through a runner kept outside the repository) and on two mutants; passes on the corrected one.
- VERIFIED: per-file results from the full-suite log: Stage C 33, Stage B normal support 37, sampler 13, unchanged M2.4.2 files (rolling lifecycle 13, forces 23, slip 14, regression 12, speculative contact 11, motorcycle tires 12), none failed.
- VERIFIED: scenes `simulation_test`, `main`, `motorcycle_test` `--quit-after 240`: exit 0, 0 error or warning lines. `git diff --check` clean.
- Not run: the 9000-frame scene run; anything graphical; any assembled-machine scenario with the patch tyre.

## 5. Unresolved / limitations
`PROJECT_STATE.md`. The overflow regression calls the tangential application directly with finite arguments after real iterations; no natural solver trajectory with a valid tyre was found that overflows there. The one-point `TireGroundContact` rolling-resistance update remains unvalidated (by inspection, out of scope). The withdrawal assumes the recorded values were applied and finite, which the atomic application now guarantees; a pathological near-float32-limit velocity could in principle still overflow while withdrawing (not exercised).

## 6. Next action
Await the user's review. The agent does not push or merge, and does not start Stage D.
