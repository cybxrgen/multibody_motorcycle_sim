# Handoff

**Authoritative for:** the last assignment's delivery-time snapshot — what the last agent was assigned, did, verified, left open, and what the next agent should do, **as observed at delivery**. It is not a live status page: current branch, commit and publication facts come from Git (`AGENTS.md`).
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** a new assignment is completed or interrupted. Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
Follow-up to the verification of PR #25 (the M2.5.3 Stage C candidate): (1) add focused tests of the distributed patch's low-speed behaviour (the reference-speed crossover and at-rest creep); (2) reconcile `HANDOFF.md`, `PROJECT_STATE.md` and `TESTING.md` with the candidate. Production code, the other four documents, Stage D, Stage E, `MotorcycleBuilder`, the one-point `TireGroundContact` and the tyre law were not touched. Stage C is not accepted or merged; Stage D and E are not started and not authorized.

## 2. Git observation at delivery
- VERIFIED: base = PR #25 head `84c40e24bc5e299117da04e292460e8d07376653` = tip of `m2/tire-patch-atomic-tangential-wrench` (fetched as `refs/pull/25/head`), the expected candidate; clean tree at the start; work stays on that branch, one new local commit. Not pushed, not merged, no PR created, no review submitted. The commit SHA and artifact names are in the delivery report.
- VERIFIED (GitHub web page; the API was not accessible from this session): PR #25 open, not draft, not merged, base `main`, 6 commits, no review, no comment. UNVERIFIED: anything on GitHub after that observation.

## 3. What changed
- `Tests/test_tire_patch_force_integration.gd`: new group J, five tests on the real patch path (`create_with_tire()`, `prepare()` / `solve_iteration()` / `finish()`, `SimWorld.step()`), expectations written out in the file from the definitions (see `TESTING.md`): `test_j_a_*` crossover of `v_hub` through `v_ref` against the hand brush law with `D = sqrt(v_hub^2 + v_ref^2)`, mirror symmetry, a derived no-jump bound; `test_j_b_*` the reference speed uses only `v . e_x` of the wheel centre and is frozen for the whole solve; `test_j_c_*` a loaded wheel at rest keeps no tangential force, rolling moment, creep or carried impulse; `test_j_d_*` creep at zero hub speed below `3 mu v_ref / k` (both axes), lateral saturation at the lateral bound and 1.5 times it (longitudinal saturation is not asserted), lateral force per load for 9x48 against 17x96 samples; `test_j_e_*` repeated iterations apply the response once, stay inside the friction limit and converge. No production code changed; no existing test or tolerance changed.
- `Documentation/TESTING.md`: the Stage C entry's group I text rewritten to match `test_i_a_*` to `test_i_d_c_*` (failure after an applied response, atomic withdrawal with the record held, terminal numerical fault, reverse-order angular rollback through the real tangential-wrench and rolling-resistance paths) and group J added; the stale mutant list replaced by what was actually run.
- `Documentation/PROJECT_STATE.md`: Stage C inventory row (withdrawal behaviour, not accepted, not registered), a new latest verification entry, one new limitation entry.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, Linux x86_64, headless)
- VERIFIED: baseline on the unmodified head: 634 passed, 0 failed, 0 skipped (18116 checks).
- VERIFIED: `godot --headless --path . --script res://Tests/run_tests.gd`: **639 passed, 0 failed, 0 skipped (18598 checks)**, exit 0 captured; the difference is exactly the five group J tests (482 checks).
- VERIFIED: fresh import exit 0, 0 error or warning lines; scenes `simulation_test`, `main`, `motorcycle_test` `--quit-after 240`: exit 0, 0 error or warning lines; `git diff --check` clean.
- VERIFIED: eleven production mutants run in a scratch copy each fail at least one group J test (listed in `PROJECT_STATE.md`).
- Not run: the 9000-frame scene run; anything graphical; any assembled-machine scenario with the patch tyre.

## 5. Unresolved / limitations
`PROJECT_STATE.md`. The rest tests start at ten solver iterations because the normal rows have not converged after one (a tyre-less twin shows the same motion), and the real-step rest test is limited to 120 steps because a free upright wheel on a patch is not roll-stable. The combined-slip creep bound is not separately asserted. The overflow regression of the tangential application calls it directly with finite arguments (no natural solver trajectory overflows).

## 6. Next action
Await the user's review. The agent does not push or merge, does not declare Stage C accepted, and does not start Stage D.
