# Handoff

**Authoritative for:** the current assignment transfer — what the last agent was assigned, did, verified, left open, and what the next agent should do.
**Belongs elsewhere:** full project state and limitations → `PROJECT_STATE.md`; roadmap → `PHASE_PLAN.md`. Do not duplicate them; reference them.
**Update when:** every assignment ends (completed or interrupted). Replace this file; never append history (Git is the history).
Labels: **VERIFIED** (confirmed by inspection/execution), **REPORTED** (stated by an earlier agent, not re-confirmed), **UNVERIFIED** (not checked).

## 1. Assignment
Documentation refactoring checkpoint (documentation only): establish the seven-document structure with clear ownership, remove contradictions, and add a fresh-agent
handoff procedure. M1.1.1, M1.2 and M1.3 were **not** started; no simulator code, tests, scenes or configuration were changed.

## 2. Git state
- Base commit: `cbc8c83` ("M1.1: implement hinge joint limits and motor"), which was `main` = `origin/main`. VERIFIED.
- Work branch: `documentation/authoritative-structure`, one commit titled "Docs: establish authoritative documentation structure" on top of the base; its hash is in the delivery manifest and the final report (a file cannot contain its own hash). Nothing pushed or merged; `main` untouched. VERIFIED.
- Working tree after the commit: clean. VERIFIED.

## 3. What was done
- `AGENTS.md`: rewritten as permanent rules only: documentation map with ownership matrix and change→document table, mandatory update procedure, startup routine, scope, architecture/physics rules, testing, Git, delivery gate and format, handoff, handling of unverified information. Removed branch/status statements.
- `PROJECT_STATE.md`: one snapshot — capability table (implemented / planned / visually unverified), repository state, verification results with labels, consolidated and de-duplicated project-wide limitations, unresolved architectural issues.
- `PHASE_PLAN.md`: one current phase (M1), one current milestone (M1.1.1, pending, not authorized), milestone table, next-phase objective (M2). Removed the contradictory "M0 current / M1 not started / M1 in progress" blocks and M0/M0.1 archive.
- `ARCHITECTURE.md`: status columns and milestone tags removed; decisions D1–D9 with rejected options; stale "failed approaches: none yet" removed.
- `PHYSICS_CONVENTIONS.md`: purpose header, milestone tags removed, tolerance-policy cross-reference; equations and conventions otherwise unchanged.
- `TESTING.md`: methodology only (commands, conventions, coverage map, verification levels, interpretation, past lessons); per-milestone result reports and stale hashes removed (results now live in `PROJECT_STATE.md`).

## 4. Verification performed
- VERIFIED: Git state (`git status`, branch, `git log --all`); test-function count in `Tests/` is 114, matching the reported total; documented symbols and files exist in source (`SimConfig.validate`, `SimWorld.preflight/add_constraint`, `SimConstraint.validate`, `HingeJointConstraint` row constants, `step_tolerance`, scene controls, no `class_name` or Godot physics nodes).
- VERIFIED: only the seven documentation files differ from the base commit (`git diff --name-only`).
- VERIFIED: manual cross-read of all seven documents for contradictory phase/milestone/limitation/branch statements, and a fresh-agent review using only the repository documents.
- NOT done: any Godot run. The environment had no Godot binary, so the suite, import and scene runs were not repeated; test results in `PROJECT_STATE.md` remain REPORTED.

## 5. Discrepancies found (resolved in the documents unless noted)
- The previous `HANDOFF.md` named branch `m1/hinge-joint` and `AGENTS.md` named `foundation/simulator-core`; neither exists. VERIFIED: only `main`/`origin/main` existed, and M1.1 and earlier commits are on `main`.
- Old notes cited commits (`75eb62b`, `0907991`, `f691adf`) that are not in the current history (history was rebuilt earlier, REPORTED). Removed as stale.
- `README.md` contains only a one-line title. Not one of the seven documents and left untouched; the user may want to decide its role.
- Scope of M1.1.1 is not defined anywhere in the repository (UNVERIFIED what the user intends); the plan records it as pending and unauthorized rather than inventing a scope.

## 6. Known limitations specific to this assignment
- Test and runtime results are REPORTED, not re-run (no Godot available here).
- Project-wide limitations are in `PROJECT_STATE.md`.

## 7. Next action
Stop and wait. The user reviews this documentation checkpoint and, if satisfied, defines and authorizes the separate M1.1.1 hardening checkpoint. Do not start M1.1.1, M1.2 or M1.3 without that explicit assignment. In a Godot-equipped environment the next agent should first re-run the suite and import (`TESTING.md`) to turn the REPORTED results into VERIFIED ones.
