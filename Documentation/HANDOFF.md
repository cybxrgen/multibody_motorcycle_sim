# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M1.2 — generic contact constraints with friction (unilateral normal contact and isotropic Coulomb friction between two rigid bodies), integrated with `SimConstraint`, `ConstraintRow`, `ConstraintSolver` and `SimWorld`.
Explicitly authorized by the user. M1.3 and later work were **not** started.

## 2. Git state
- Base commit: `b877c008047fe7bcce06b332153c6856c120a00a` (`origin/main` at the time of work, contains the merged M1.1.1, PRs #3 and #4). VERIFIED with `git log`.
- Work branch: `m1/contact-friction`, one commit "M1.2: implement generic contact constraints with friction" (`2975a6b25fd222005e8419e73198a8eceb1eb2e2`). The agent delivered it locally as a patch and ZIP and did not push or merge. The user subsequently published the branch and opened PR #5 (https://github.com/cybxrgen/multibody_motorcycle_sim/pull/5). PR #5 is awaiting review and user acceptance; it is not accepted and not merged, and `main` does not contain M1.2. VERIFIED against the remote branch and `main`; the PR's open/review state is REPORTED by the user.
- The previous handoff's statement that M1.1.1 was awaiting review was stale; Git history shows it merged. Reconciled in `PHASE_PLAN.md` and `PROJECT_STATE.md`. VERIFIED.

## 3. What was done
- `Simulation/Solver/contact_constraint.gd` (new): `ContactConstraint`, 3 rows (normal, two tangent). Convention, Jacobians, biases, friction, tangent basis and warm start are in its header and `PHYSICS_CONVENTIONS.md` ("Contact"). Normal row: bounds `[0, +INF)`, speculative bias `g/dt` for `g >= 0`, Baumgarte `(beta/dt) g` for `g < 0` with `SimConfig.baumgarte_factor`. Friction: circular Coulomb limit, 2×2 block solve with K-metric projection, solved after the normal row each iteration; friction warm start stored as a world vector, projected to the current tangent plane and clipped to the current disk. Shared contact point (midpoint of the anchors) for all lever arms. Self-validating; `from_world()` factory.
- Minimal generic solver extension: `SimConstraint.solve_iteration()` (default loops the rows as before); `ConstraintSolver` calls it per constraint. Reason: the friction disk couples two rows with the normal row's impulse, which per-row `[min, max]` clamps cannot express (`ARCHITECTURE.md` D11, D12). `ConstraintRow`, `SimWorld` and the joints are unchanged.
- `Tests/test_contact.gd` (33 tests) plus generated `.uid` files.
- Docs: `PHYSICS_CONVENTIONS.md`, `ARCHITECTURE.md`, `TESTING.md`, `PROJECT_STATE.md`, `PHASE_PLAN.md`, this file.

## 4. Verification (Godot 4.7.stable.official.5b4e0cb0f, linux x86_64, headless)
Godot was not preinstalled; the official release zip was downloaded and its SHA-512 matched the published `SHA512-SUMS.txt`.
1. Automated tests. VERIFIED: baseline on the base commit 130 passed, 0 failed (3967 checks). With the change: **163 passed, 0 failed, 0 skipped (4528 checks)**, exit 0, no `SCRIPT ERROR`.
2. Import/parse. VERIFIED: fresh import (`.godot/` deleted) exit 0, no error/warning lines.
3. Headless scene smoke. VERIFIED: `simulation_test.tscn` and `main.tscn`, 240 frames each, exit 0, no error/warning lines. Neither scene uses contacts.
4. Visual inspection. **Not done.** UNVERIFIED.
- Regression sensitivity. VERIFIED: ten injected defects; eight were caught by the first tests, two survived (Euclidean projection instead of K-metric, friction before normal). Two tests were added and both are now caught. After that addition only those two and the Baumgarte defect were re-run, not the full set of ten.
- Test-writing lessons: exactness checks use `solve_in_place()` because `World.step()` adds the explicit gyroscopic term and moves bodies; three first-draft expectations were test errors (not simulator errors) and were corrected after analysis.

## 5. Unresolved / notes
- Contacts have no debug visualization and no scene uses them. UNVERIFIED visually.
- Anchors are fixed to the bodies; the caller must re-seat them when the contact patch moves. Limitations list: `PROJECT_STATE.md` ("Contacts").
- Duplicate registration of one constraint and `add_body()` of an already registered body remain unrejected (pre-existing).
- Commit signing was configured in the environment; see the final report for whether the commit was signed.

## 6. Next action
Stop and wait for the user's review and acceptance of PR #5. Do not describe M1.2 as accepted or merged until the user says so. M1.3 is not authorized and must not be started; it requires a separate explicit assignment.
