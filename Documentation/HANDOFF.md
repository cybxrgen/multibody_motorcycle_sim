# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignments
This handoff covers two assignments on PR #8. (1) PR #8 hardening checkpoint: (A) `step()` must return false for an aborted step, (B) `advance()` must stop at an aborted step without counting it or discarding time, (C) constraint solver state must be rollback-safe, (D) resolve the M1.3 status inconsistency in the documentation. Preserve the implicit-midpoint rotational integrator unchanged. No M2, no push, no merge (the agent's rules). (2) Documentation-only correction of this handoff: the earlier text claimed nothing was pushed, but the user had published the follow-up; also record that the user formally accepted M1.3. No code, test or configuration change.

## 2. Git state (VERIFIED with `git ls-remote`, `git fetch` and tree comparison on the day of this correction)
**Local activity.** The hardening follow-up commit was created locally on `m1/rotational-integration-stability` as `2407464ad258bb21084dc3fdeece58d31d4c4a2a`, directly on the original PR #8 commit `c200c5c5a2d8c8b5b6359afd6349e1060294cdfc` (its parent is `main`, `aabaee1`). The agent did not push (its rules forbid it; the session had read-only Git access).

**Remote publication.** The follow-up was nevertheless published to the remote PR #8 branch by the user (applied from the delivered patch, as `AGENTS.md` §7 describes). Remote branch: `m1/rotational-integration-stability`; remote head: `430b1d35d991a410722eab778947e581390f3b03`, parent `c200c5c`. It is a different commit object than the local `2407464` (different SHA), with the same tree (`e839c9db18d4e5b88b395ca41dc6ae115fb9cca3`), the same parent and the same message, so the published change is exactly the delivered one. Per `AGENTS.md` §7 the remote SHA is the definitive identifier of the reviewed version. The documentation-only correction commit of this assignment is based on `430b1d3` and is not published by the agent.

**Merge status.** PR #8 is not merged: `main` is still at `aabaee136d5d2fa6b49147c06f56e96fcb200f25` (PR #7's merge commit) and does not contain the PR #8 commits. The state of the PR page itself (open, draft, reviewed) is UNVERIFIED: the session has no GitHub API access. PR #7 and its follow-ups (M1.3, M1.3.1, M1.3.2) remain merged into `main`; their history was not touched.

**Milestone status (separate from Git status).** M1.3 is formally accepted by the user (stated by the user in this assignment). M2 is not authorized. PR #8's Git integration status does not affect either.

## 3. What PR #8 contains (original commit, REPORTED by the earlier handoff, behaviour re-run below)
Implicit-midpoint gyroscopic update in `SimIntegrator` (conserves `|L|` and rotational `T` of a torque-free body), state validation and abort/rollback in `SimWorld`, `Tests/test_rotational_stability.gd` (12 tests). Unchanged by this follow-up.

## 4. What was done in this follow-up
- **A.** `_step_unchecked()` returns bool; `step()` returns it. False for a preflight refusal (counted in `rejected_step_count`, as before) and for an aborted step (not counted as a rejection). Successful-step behaviour unchanged.
- **B.** `advance()` returns at the first aborted step. Policy: the aborted step is not counted and its `dt` is not subtracted; the accumulator keeps the aborted step's time plus all unattempted time (including that call's frame time); the backlog guard does not run, so nothing is dropped; later calls are refused by preflight (refused frame time not accumulated), so failed steps are never retried; after `clear_numerical_fault()` the preserved time is consumed (still capped by `max_steps_per_frame`). No deviation from the preferred policy. The step cap and normal accumulator behaviour are unchanged.
- **C.** Investigation: before a fault can be detected after the solve, `prepare`/`solve`/`finish` have rewritten every row's `accumulated_impulse` and `ContactConstraint._friction_impulse` (possibly non-finite); `split_accumulated_impulse` is rebuilt every step; the hinge angle tracker only samples the unchanged start-of-step pose, so it needs nothing. Chosen: **invalidate**, not snapshot — `_abort_step()` calls the new `SimConstraint.reset_warm_start()` on every constraint (`ConstraintRow.reset_impulses()`; `ContactConstraint` also clears the friction impulse). No copying or allocation in normal steps; solver mathematics untouched. Consequence: the first step after recovery is cold started.
- **Tests:** new `Tests/test_fault_handling.gd` (10 tests); `test_rotational_stability.gd::test_c1` had asserted that an aborted `step()` returns true (the defect) and now asserts false — the only existing assertion changed.
- **Documentation:** `PROJECT_STATE.md`, `PHASE_PLAN.md`, `HANDOFF.md` as assigned; `ARCHITECTURE.md` (D17 section, constraint table) and `TESTING.md` (coverage map) because the implementation changed. `PHYSICS_CONVENTIONS.md` and `AGENTS.md` unchanged.
- **M1.3 status:** merged into `main` (Git) and formally accepted by the user, recorded as separate facts; M2 is not authorized.

## 5. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display)
Commands (from the repository root; exit codes in parentheses):
- Baseline on an export of the unchanged PR #8 head: `godot --headless --path . --import` (0); `--script res://Tests/run_tests.gd` → 204 passed, 0 failed (0). VERIFIED.
- After the change, `.godot/` deleted first: `--import` (0, no error/warning lines); `--script res://Tests/run_tests.gd` → **214 passed, 0 failed, 0 skipped, 5169 checks** (0), including the 150 s spinner regression (`test_rotational_stability.gd` a1–a3) and all 12 rotational tests. VERIFIED.
- `res://Scenes/simulation_test.tscn` and `res://Scenes/main.tscn` with `--quit-after 240` (0, 0, no error/warning lines); `simulation_test.tscn --fixed-fps 60 --quit-after 9000` = 150 simulated seconds through the real scene (0, no error/warning lines); `git diff --check` (0). VERIFIED.
- Mutation checks in copies outside the repository: `step()` ignoring the abort, `advance()` continuing, and no solver reset are each caught (details in `PROJECT_STATE.md`). VERIFIED. One earlier mutant was invalid (parse error) and redone.
- The documentation-only correction commit changed no source, tests or configuration; the simulator test suite was not re-run for it and no new test results are claimed (`AGENTS.md` §5).
- Not done / UNVERIFIED: graphical inspection (headless only; no visual verification claimed); the exact 150 s numbers (`|ΔL|/L0`, `|ω|` range, flips) were not re-measured, only the test's pass/fail bounds; the Python-side measurements of the earlier handoff stay REPORTED; PR #8 web state; `Tools/make_delivery.sh` results are in the delivery report, not here.

## 6. Unresolved / limitations
- Faults in the new tests are injected inside the step by a probe constraint; a fault arising organically from the solver with finite inputs was not constructed.
- After a fault the world stays frozen until the caller fixes the cause and calls `clear_numerical_fault()`; no automatic recovery. Warm starting restarts cold afterwards.
- Earlier notes remain: `L` direction first-order accurate, frozen-Jacobian limit for constraint-coupled fast rotation, stale "explicit gyroscopic term" comments in `test_contact.gd` and `test_hinge_joint.gd` (`PROJECT_STATE.md`).

## 7. Next action
Stop and wait for the user's review. Do not merge PR #8 and do not begin M2. Publishing this documentation-only correction is the user's step (`AGENTS.md` §7).
