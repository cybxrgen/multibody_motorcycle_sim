# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
M1.3.2 — documentation-only wording corrections found in the review of PR #7, before M1.3 is accepted: (1) a malformed sentence about `L_spin` in `PHYSICS_CONVENTIONS.md`; (2) decision D14 in `ARCHITECTURE.md` read as if split velocities were never involved in diagnostics, which looked contradictory to the M1.3.1 regression test. Explicitly authorized by the user. No source, test or other document was changed. M1.4 and M2 were **not** started; M2 remains unauthorized.

## 2. Git state
- Base commit: `29fff9433b5e7064288bc8bf993ee53a6e374a88`, the published M1.3.1 head of `m1/split-impulse-stabilization` (`origin/m1/split-impulse-stabilization`). Its tree is identical to the tree of the agent's original M1.3.1 commit `1e9da1e`; the SHAs differ because the user applied the patch with `git am` (recorded in both SHAs, not a defect). The M1.3 commit `27d0594` and the M1.3.1 commit are preserved unchanged. VERIFIED with `git log`, `git rev-parse` and `git merge-base --is-ancestor`.
- Work branch `m1/split-impulse-stabilization`, one documentation-only follow-up commit on top of the base (hash in the delivery manifest and the final report; a file cannot contain its own hash). The working tree was clean at the start. Not pushed, not merged, PR state untouched, `main` untouched.
- M1.3 is **not** accepted and not merged; it awaits user review and acceptance.

## 3. What was done
- `PHYSICS_CONVENTIONS.md`, orientation-dependent quantities bullet: replaced "and `L_spin` turns off the direction of `w`" by a statement that rotating the body changes `I_w`, so `L_spin = I_w w` need not stay parallel to `w` (in the tested example parallel before the correction because `w` is on a principal axis, in general not after), hence the direction of `L_spin` changes although `w` does not. Equations, example and the rest of the bullet unchanged.
- `ARCHITECTURE.md`, D14: separated direct isolation (split velocities are not part of physical velocity, momentum or energy diagnostics, not published, not warm started, not read by motor, friction or contact velocity) from indirect pose effects (integrated into the pose; split angular correction changes orientation and, for anisotropic bodies, the world inertia, so spin angular momentum and rotational kinetic energy can change although the physical `w` does not; not a use of split velocity by diagnostics). Refers to `PHYSICS_CONVENTIONS.md` instead of repeating the derivation.
- This handoff replaced as `AGENTS.md` §8 requires; no other document touched.

## 4. Verification
- VERIFIED: `git diff --check` clean; the diff touches only `Documentation/ARCHITECTURE.md`, `Documentation/PHYSICS_CONVENTIONS.md` and `Documentation/HANDOFF.md`; no source or test file changed.
- Not run (documentation-only, `AGENTS.md` §5): the Godot suite, import and scene smokes. The last executed results (192 passed, 0 failed, 4973 checks; M1.3.1 run) are recorded in `PROJECT_STATE.md` and were not re-run here. Visual inspection: not done.

## 5. Unresolved / notes
- Frozen-Jacobian dissipation, cold-started split impulses and the other limitations are unchanged (`PROJECT_STATE.md`).

## 6. Next action
Stop and wait for the user's review of M1.3.2 and acceptance of M1.3. M1.4 and M2 are not authorized.
