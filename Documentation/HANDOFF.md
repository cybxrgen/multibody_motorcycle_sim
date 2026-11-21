# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked). They describe the evidence available when this handoff was created.

## 1. Assignment
Final M1 / PR #8 refinement after the merge of PR #8: (1) a constraint-backed regression proving that an invalid static-body pose is detected before the constraint solver consumes it, with the minimum implementation correction; (2) document the recovery behaviour when a static pose was already invalid before the step began; (3) bring `PROJECT_STATE.md`, `ARCHITECTURE.md`, `TESTING.md` and this handoff up to date. No M2, no `PHASE_PLAN.md` or `AGENTS.md` change, no push, no merge.

## 2. Git observation at delivery (VERIFIED, 2026-11-21, after `git fetch`)
- At the start of the assignment PR #8 was already **merged**: `origin/main` was `5cf38e5458ca11dbe42e670bdc053d0f4d4e673c`, the merge commit of pull request #8 (parents `aabaee1…`, the pull request #7 merge, and `dc007335ff76b7be15ff50d9f90b8b15e058919d`, the former branch head). Its tree equals the tree of the earlier delivered static-pose fix (`5098018e…`).
- Work branch: `m1/static-pose-prestep-validation`, created from that `main`. The refinement is **one local commit on that branch, not pushed, not merged, not part of `main`**. The agent never pushes; a file cannot contain its own commit's hash, so the commit SHA is in the delivery report.
- Remote publication and review of this refinement: none at delivery (it exists only as the delivered artifacts). This says nothing about what the user does afterwards.
- Milestones (separate from Git): M1.3 is formally accepted by the user and merged into `main`. PR #8 is merged. M2 is not authorized.

## 3. What changed
- `Simulation/Core/sim_world.gd`: the first state check (after velocity integration, before constraint prepare/solve) now runs with `check_pose = true`. Position and orientation (finite, unit length) of **every** body, static bodies included, are therefore validated before the solver can read them; velocity and world-inertia checks stay dynamic-only. The later checks are unchanged (velocities after the solve; velocities and poses after position integration). One line of code. Consequence also for dynamic bodies: an invalid dynamic pose present before the step is now reported before the solve instead of after it (same fault mechanism, same quantity names).
- `Tests/test_fault_handling.gd`: `test_d1` (static position NaN / inf) and `test_d2` (static orientation NaN / zero quaternion) with a real `ProbeBall` ball joint between a static anchor and a dynamic bob. They check: fault names the static body and quantity; stage "before the constraint solve"; the joint's `prepare()` was not entered (`prepare_calls` unchanged, also while refused); bob restored exactly and finite; nothing published; step not counted, time unchanged; carried impulses reset; further steps refused; rollback restores the already-corrupted pose; `clear_numerical_fault()` alone leaves the next step faulting; stepping resumes after the pose is restored and the fault cleared. The earlier static-body tests (`test_rotational_stability.gd` c3–c6) are retained and unchanged.
- Documentation: `PROJECT_STATE.md` (PR #8 merged with its merge commit, latest results, capability row, recovery limitation, historical baselines labelled), `ARCHITECTURE.md` (pipeline, D17 check stages, pre-existing-invalid-pose rollback), `TESTING.md` (coverage map; no run counts), this file. `PHASE_PLAN.md`, `AGENTS.md`, `PHYSICS_CONVENTIONS.md` untouched.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless, no display; VERIFIED unless stated)
- Fresh import: `.godot/` deleted, `godot --headless --path . --import` → exit 0, 0 error/warning lines.
- Full suite: `godot --headless --path . --script res://Tests/run_tests.gd` → **220 passed, 0 failed, 0 skipped, 5285 checks**, exit 0.
- Smoke runs: `res://Scenes/simulation_test.tscn` and `res://Scenes/main.tscn` with `--quit-after 240` (exit 0, 0 error/warning lines each); `simulation_test.tscn --fixed-fps 60 --quit-after 9000` = 150 simulated seconds (exit 0, 0 error/warning lines). `git diff --check` clean.
- Pre-fix comparison (temporary, reverted): old `sim_world.gd` with the new tests → 218 passed, 2 failed (`test_d1`, `test_d2`), exit 1. Pre-fix the fault surfaced only after the constraint solve, against the dynamic body (non-finite split velocity), and `prepare()` had run with the invalid pose. The unchanged tree before the change: 218 passed, 5205 checks (same session).
- Not verified: graphical inspection (headless only, no visual claim); CI of any commit (UNVERIFIED, not queried); `Tools/make_delivery.sh` results are in the delivery report.

## 5. Unresolved / limitations
- Intentional rollback behaviour, documented in `ARCHITECTURE.md` (D17) and `PROJECT_STATE.md`: a pose corrupted **before** `step()` is already in the start-of-step snapshot, so the abort restores that invalid pose; `clear_numerical_fault()` does not repair it. The caller must restore a valid pose (or recreate the body), then clear the fault.
- Faults in the tests are injected by the caller or by a probe constraint; a fault arising organically from the solver with finite inputs has not been constructed.
- Earlier notes remain in `PROJECT_STATE.md` (frozen-Jacobian limit, `L` direction accuracy, stale "explicit gyroscopic term" comments in `test_contact.gd` and `test_hinge_joint.gd`).

## 6. Next action
Await the user's review of this local refinement and explicit direction. The agent does not push or merge; M2 is not authorized.
