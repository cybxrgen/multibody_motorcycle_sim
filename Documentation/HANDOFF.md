# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
PR #8 hardening checkpoint: (A) `step()` must return false for an aborted step, (B) `advance()` must stop at an aborted step without counting it or discarding time, (C) constraint solver state must be rollback-safe, (D) resolve the M1.3 status inconsistency in the documentation. Preserve the implicit-midpoint rotational integrator unchanged. No M2, no push, no merge.

## 2. Git state (VERIFIED)
- Branch `m1/rotational-integration-stability`; base = original PR #8 head `c200c5c5a2d8c8b5b6359afd6349e1060294cdfc` (the implicit-midpoint commit, present and unchanged, matches the remote branch head). Its parent is `main` (`aabaee1`, the PR #7 merge). The working tree was clean at the start.
- One new follow-up commit on top (hash in the delivery manifest and the final report). Nothing pushed or merged, `main` and PR #7 history untouched. PR #8's web page was not inspected (the session had read-only Git access, no GitHub API); the branch head was confirmed with `git ls-remote`.

## 3. What PR #8 contains (original commit, REPORTED by the earlier handoff, behaviour re-run below)
Implicit-midpoint gyroscopic update in `SimIntegrator` (conserves `|L|` and rotational `T` of a torque-free body), state validation and abort/rollback in `SimWorld`, `Tests/test_rotational_stability.gd` (12 tests). Unchanged by this follow-up.

## 4. What was done in this follow-up
- **A.** `_step_unchecked()` returns bool; `step()` returns it. False for a preflight refusal (counted in `rejected_step_count`, as before) and for an aborted step (not counted as a rejection). Successful-step behaviour unchanged.
- **B.** `advance()` returns at the first aborted step. Policy: the aborted step is not counted and its `dt` is not subtracted; the accumulator keeps the aborted step's time plus all unattempted time (including that call's frame time); the backlog guard does not run, so nothing is dropped; later calls are refused by preflight (refused frame time not accumulated), so failed steps are never retried; after `clear_numerical_fault()` the preserved time is consumed (still capped by `max_steps_per_frame`). No deviation from the preferred policy. The step cap and normal accumulator behaviour are unchanged.
- **C.** Investigation: before a fault can be detected after the solve, `prepare`/`solve`/`finish` have rewritten every row's `accumulated_impulse` and `ContactConstraint._friction_impulse` (possibly non-finite); `split_accumulated_impulse` is rebuilt every step; the hinge angle tracker only samples the unchanged start-of-step pose, so it needs nothing. Chosen: **invalidate**, not snapshot — `_abort_step()` calls the new `SimConstraint.reset_warm_start()` on every constraint (`ConstraintRow.reset_impulses()`; `ContactConstraint` also clears the friction impulse). No copying or allocation in normal steps; solver mathematics untouched. Consequence: the first step after recovery is cold started.
- **Tests:** new `Tests/test_fault_handling.gd` (10 tests); `test_rotational_stability.gd::test_c1` had asserted that an aborted `step()` returns true (the defect) and now asserts false — the only existing assertion changed.
- **Documentation:** `PROJECT_STATE.md`, `PHASE_PLAN.md`, `HANDOFF.md` as assigned; `ARCHITECTURE.md` (D17 section, constraint table) and `TESTING.md` (coverage map) because the implementation changed. `PHYSICS_CONVENTIONS.md` and `AGENTS.md` unchanged.
- **M1.3 status:** merged into `main` (Git) is recorded separately from milestone acceptance, which stays pending until the user decides; M2 is not authorized.

## 5. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display)
Commands (from the repository root; exit codes in parentheses):
- Baseline on an export of the unchanged PR #8 head: `godot --headless --path . --import` (0); `--script res://Tests/run_tests.gd` → 204 passed, 0 failed (0). VERIFIED.
- After the change, `.godot/` deleted first: `--import` (0, no error/warning lines); `--script res://Tests/run_tests.gd` → **214 passed, 0 failed, 0 skipped, 5169 checks** (0), including the 150 s spinner regression (`test_rotational_stability.gd` a1–a3) and all 12 rotational tests. VERIFIED.
- `res://Scenes/simulation_test.tscn` and `res://Scenes/main.tscn` with `--quit-after 240` (0, 0, no error/warning lines); `simulation_test.tscn --fixed-fps 60 --quit-after 9000` = 150 simulated seconds through the real scene (0, no error/warning lines); `git diff --check` (0). VERIFIED.
- Mutation checks in copies outside the repository: `step()` ignoring the abort, `advance()` continuing, and no solver reset are each caught (details in `PROJECT_STATE.md`). VERIFIED. One earlier mutant was invalid (parse error) and redone.
- Not done / UNVERIFIED: graphical inspection (headless only; no visual verification claimed); the exact 150 s numbers (`|ΔL|/L0`, `|ω|` range, flips) were not re-measured, only the test's pass/fail bounds; the Python-side measurements of the earlier handoff stay REPORTED; PR #8 web state; `Tools/make_delivery.sh` results are in the delivery report, not here.

## 6. Unresolved / limitations
- Faults in the new tests are injected inside the step by a probe constraint; a fault arising organically from the solver with finite inputs was not constructed.
- After a fault the world stays frozen until the caller fixes the cause and calls `clear_numerical_fault()`; no automatic recovery. Warm starting restarts cold afterwards.
- Earlier notes remain: `L` direction first-order accurate, frozen-Jacobian limit for constraint-coupled fast rotation, stale "explicit gyroscopic term" comments in `test_contact.gd` and `test_hinge_joint.gd` (`PROJECT_STATE.md`).

## 7. Next action
Stop and wait for the user's review. Do not merge PR #8 and do not begin M2. The user applies the delivered patch and publishes the branch (`AGENTS.md` §7).
