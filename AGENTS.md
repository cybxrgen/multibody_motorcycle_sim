# AGENTS.md — Permanent operating rules for every agent

**Authoritative for:** how any agent must work in this repository (startup, scope, Git, testing, delivery, handoff,
documentation upkeep) and which document owns which information.
**Belongs elsewhere:** current status, milestones, test results, findings → `PROJECT_STATE.md`, `PHASE_PLAN.md`, `HANDOFF.md`.
**Update when:** a permanent agent workflow rule changes — and only then. This file never carries temporary status.

Project: custom rigid-body / constraint physics foundation (Godot 4.7, GDScript) for a motorcycle + rider multibody
simulator. The custom simulator is the **only** physical authority.

## 1. Documentation map and ownership

All project documentation is the seven files below (this file included). Do not create further documentation files,
archives or decision logs unless a concrete need is justified to the user first.

| Document | Purpose | Owns | Does not own |
|---|---|---|---|
| `AGENTS.md` | Permanent agent rules | Workflow, scope, Git, testing, delivery, doc rules, ownership matrix | Current project status |
| `Documentation/PROJECT_STATE.md` | Canonical current state | What exists, latest verified results, project-wide limitations and debt | Future roadmap, assignment diary |
| `Documentation/PHASE_PLAN.md` | Current roadmap | Current phase and milestone, remaining work, next-phase objective | Historical milestone archive (Git is the archive) |
| `Documentation/HANDOFF.md` | Current assignment transfer | What the last assignment did, how it was verified, next action | Permanent project state, full roadmap |
| `Documentation/ARCHITECTURE.md` | Stable system architecture | Modules, dependencies, data flow, boundaries, design decisions and rationale | Temporary task status |
| `Documentation/PHYSICS_CONVENTIONS.md` | Physics and math conventions | Units, frames, quaternions, inertia, Jacobians, solver and numerical assumptions | Implementation progress, test results |
| `Documentation/TESTING.md` | Verification methodology | Suite layout, test-writing rules, commands, verification levels, how to read results | Duplicated test reports |

### Which documents a change must update

| Change | Required documentation |
|---|---|
| New implemented capability | `PROJECT_STATE.md` + the relevant technical docs (`ARCHITECTURE.md`, `PHYSICS_CONVENTIONS.md`, `TESTING.md`) |
| Milestone status change | `PHASE_PLAN.md` |
| Assignment completed or interrupted | `HANDOFF.md` (replace it; do not append) |
| Persistent project limitation discovered or resolved | `PROJECT_STATE.md` |
| Physics convention changed | `PHYSICS_CONVENTIONS.md` |
| Architectural decision changed | `ARCHITECTURE.md` |
| Test methodology changed | `TESTING.md` |
| Permanent agent workflow changed | `AGENTS.md` |

Do not touch an unrelated document just to look comprehensive. Record the latest verified test numbers only in
`PROJECT_STATE.md` and `HANDOFF.md`, never in the technical documents.

### Documentation update procedure (mandatory)
1. Read the existing documentation before implementing.
2. Decide which documents the assignment can legitimately affect (table above).
3. Keep implementation and documentation synchronized.
4. Edit documentation only after the implementation and verification results are known.
5. Remove outdated statements; never append a contradictory correction below an old statement.
6. Never claim a test, commit, runtime check or visual inspection that was not performed.
7. Cross-reference instead of duplicating (link the owning document).
8. Make the final `HANDOFF.md` agree with the actual Git state (`git branch --show-current`, `git log`, `git status`).
9. Before delivery, read all seven documents together and search for contradictions (phase/milestone declarations, limitations, counts, branch and commit statements).
10. Documentation must never describe functionality that does not exist. Avoid dates, commit hashes, test counts and branch names outside
    the documents that own them (`PROJECT_STATE.md`, `HANDOFF.md`); a file cannot contain the hash of its own commit.

Every document starts with a short statement of what it is authoritative for, what belongs elsewhere and when to update it. Keep it.
Record failed or rejected approaches with reasons in the owning technical document so they are not retried blindly.

## 2. Mandatory start-of-work routine
1. Read, in order: this file, `Documentation/HANDOFF.md`, `Documentation/PROJECT_STATE.md`, `Documentation/PHASE_PLAN.md`,
   `Documentation/ARCHITECTURE.md`, `Documentation/PHYSICS_CONVENTIONS.md`, `Documentation/TESTING.md`.
2. Inspect the real repository: `git status`, `git branch --show-current`, `git log --oneline --decorate -20`.
3. **If the working tree is dirty, stop and report it. Never modify or discard another party's uncommitted work.**
4. Inspect the code before changing it. Do not trust documentation or a previous agent's claims blindly: `HANDOFF.md` labels
   claims VERIFIED / REPORTED / UNVERIFIED; re-verify what your work depends on (re-run the test suite before changing simulator code, if Godot is available).
5. State the exact current state, the authorized assignment and open issues before starting.
6. Do only what the current assignment authorizes (next section).

## 3. Scope discipline
- The user's current assignment defines scope. Being listed in `PHASE_PLAN.md` is **not** authorization: pending milestones start only on explicit user instruction.
- Do not expand scope, refactor unrelated code, or add placeholder systems for future subsystems.
- A documentation-only assignment changes only documentation: no simulator source, solver behaviour, tests or expectations, scenes, assets, project configuration or dependencies. If tooling changes seem necessary, explain why and ask first.
- Do not change Git history, merge, or push (publication is the user's step, §7).

## 4. Architectural and physics rules (do not violate)
Details and rationale: `ARCHITECTURE.md`, `PHYSICS_CONVENTIONS.md`.
- Physical state is owned by `SimWorld` / `SimRigidBody`. Godot nodes are visual/debug/UI only.
- No `RigidBody3D`, `Generic6DOFJoint3D`, `PhysicalBone3D`, or any built-in physics as authority.
- The physics core (`Simulation/Core`, `Simulation/Solver`) never touches the scene tree (`RefCounted`-based; `SimConfig` is a `Resource`).
- Solver and bodies are generic. No motorcycle/rider special cases in the core.
- Never merge a rider's mass into a motorcycle body while also simulating the rider separately.
- SI units, right-handed, +Y up, local −Z forward, +X right. Quaternions for orientation.
- No arbitrary damping, hidden corrective forces, or position teleportation to hide solver problems. Every stabilization term needs a documented mathematical purpose.
- Numerical parameters live in `SimConfig` (`Simulation/Core/sim_config.gd`); their validity rules live in `SimConfig.validate()` only. `SimWorld` refuses to step while config, any body's mass properties, or any constraint is invalid.
- No `class_name` globals: scripts reference each other with `preload("res://...")` constants so the project works headless without an editor import step.
- Commit the `*.uid` files Godot generates next to scripts. Never commit `.godot/`.

## 5. Testing and verification
Methodology, commands and verification levels: `TESTING.md`. Binding rules:
- Run the full headless suite before every commit that changes simulator code or tests.
- Tests validate physical outcomes with justified tolerances, not merely that code runs. Never loosen a tolerance without recording why in the test and in `TESTING.md` if the policy changes.
- Report failures honestly. If Godot is unavailable, say so everywhere (commit notes, manifest, handoff) and never claim a run that did not happen.
- Keep import/parse, headless runtime and visual inspection distinct; claim only the level actually reached.
- A documentation-only commit does not require a simulator test rerun; do not claim new test results for it.

## 6. Git rules
- Work on a dedicated local branch (name given by the assignment; otherwise a descriptive one). Never commit to `main`.
- Never push, never merge, never rewrite history, never discard anyone's uncommitted work.
- Each assignment ends in **one** coherent commit with its documentation updated, unless the assignment explicitly allows several. No intermediate commits for individual fixes.
- Inspect `git diff` and `git status` before committing; no accidental files. Commit messages state what changed, not what was merely intended.
- The agent's local commit and its delivery are **intermediate steps**. Applying the patch and publishing the branch to GitHub is the user's responsibility (§7, "Remote review workflow"); the agent never pushes, merges or publishes.

## 7. User delivery gate (permanent)
- **A commit is not permission to begin the next commit.** A completed commit is a user review checkpoint.
- After the commit, in this order: generate the Git patch (below) and the ZIP, manifest and SHA-256 with `Tools/make_delivery.sh <commit> ...` from that exact commit;
  verify them; **share all four files (patch, ZIP, manifest, SHA-256) with the user through a working user-accessible mechanism** (attachment or download);
  write the final report; **stop**. A local ZIP or patch is not a substitute for remote publication.
- Wait for explicit user instruction. A new assignment is required before any further work.
- Agents must never: create several commits consecutively without delivering intermediate snapshots; announce a ZIP and proceed to the next
  commit; keep implementing after the delivery was generated; treat a path inside an isolated sandbox as delivered; claim a file was shared when only an internal path was printed. If the environment cannot publish artifacts, say so and stop.
- Never modify the project or a ZIP after its checksum was computed. Any change means a new commit (or, for a defect in the delivery itself, regenerating all four files) and full re-verification.

### Delivery format (every delivery)
- `Tools/make_delivery.sh <commit> ["<milestone>" "<tests executed>" "<test results>" "<known limitations>" "<godot status>"]` writes three
  **separate** files to `deliveries/` (git-ignored): `motorcycle_multibody_<short>.zip`, `motorcycle_multibody_<short>_manifest.txt`,
  `motorcycle_multibody_<short>.sha256`. Manifest and checksum are never inside the ZIP.
- The ZIP is built by `git archive` from that exact commit: single root `motorcycle_multibody/`, exactly the committed files, no `.git`/`.godot`.
- The script verifies CRC, commit stamp, file list, SHA-256 (`sha256sum -c`) and manifest consistency. `DELIVERY_COPY_DIR=<dir>` copies the files to a user-visible location and re-verifies them there.
- Fill the optional manifest arguments honestly (what was and was not executed); the defaults state that Godot was not verified.

### Patch delivery (mandatory, every assignment)
Every assignment delivers **four separate artifacts**: the Git patch (`*.patch`), the ZIP built from the exact agent commit, the manifest (`*_manifest.txt`) and the SHA-256 checksum (`*.sha256`).
- Generate the patch with `git format-patch`, never a plain diff, so commit metadata is preserved and the user can apply it with `git am`. Example (follows the delivery naming convention, into the git-ignored `deliveries/`):
  `git format-patch -1 --binary --full-index --output=deliveries/motorcycle_multibody_<short>.patch <commit>`
- Before delivery: (1) apply the patch with `git am` in a clean checkout of the exact stated base commit and confirm it applies cleanly; (2) confirm the resulting tree equals the agent commit's tree
  (the intended changeset, nothing more); (3) record the exact base commit SHA, the original agent commit SHA, the feature branch name and the patch filename.
- The existing ZIP, manifest and checksum generation and verification requirements are unchanged and still apply.

### Remote review workflow
1. GitHub is the authoritative source for implementation review. ZIP files and local agent workspaces are delivery artifacts, not the primary review source.
2. The agent implements and commits locally on a dedicated feature branch, delivers the four artifacts, and stops. It never pushes, merges or publishes, and never claims work is remotely available before the user has published it.
3. The user applies the patch to a clean checkout of the stated base commit on the intended feature branch, verifies the resulting local commit, and pushes the branch to GitHub. The user may open a PR or otherwise make the branch available for review.
4. Review (including ChatGPT review) is performed against the actual remote branch or PR. A local agent workspace or an unpublished branch cannot be inspected by reviewers.
5. The published remote commit SHA is the definitive identifier of the reviewed version. If applying the patch yields a different SHA from the agent's original commit, record both SHAs; that difference alone is not a defect.
6. If the published changes differ from the delivered patch, identify and resolve the discrepancy before review.

### User application instructions
Run in a clean checkout of the stated base commit (placeholders, not real names or hashes):
```bash
git checkout -b <feature-branch> <base-commit>
git am <patch-file>
git status
git log -1 --oneline
git push -u origin <feature-branch>
```
The final command is the user's; the agent must never execute a push or merge.

### Final report and publication status
Every final report includes:
```text
Base commit:       <base SHA>
Agent commit:      <agent SHA>
Feature branch:    <branch name>
Patch:             <patch filename>
ZIP:               <archive filename>
Manifest:          <manifest filename>
SHA256:            <checksum filename>

Remote publication status: Awaiting user publication
Remote review status:       Not yet available
```
and states explicitly that: the patch was generated and verified locally; the ZIP, manifest and checksum were generated and verified; the agent did not push or merge;
the user must publish the resulting branch before remote review can take place; the agent has stopped and is awaiting instructions.

## 8. Handoff requirements
At the end of every assignment (completed or interrupted) replace `Documentation/HANDOFF.md` with one current handoff that answers:
what was assigned, what was actually accomplished, what was verified and how, what remains unresolved or blocked, and what the next agent should do.
Use the labels VERIFIED (confirmed by inspection/execution), REPORTED (stated by a previous agent, not re-confirmed), UNVERIFIED (not checked).
Format and content rules are in `HANDOFF.md`'s own header.

## 9. Missing, contradictory or unverified information
- Establish the truth from source code, tests, Git history or actual execution before editing documentation.
- If it cannot be established, record the uncertainty explicitly (UNVERIFIED / REPORTED) instead of inventing an answer, and tell the user.
- Never silently "fix" a discrepancy by deleting one side of it without evidence; resolve it, then remove the outdated statement.
- If the assignment contradicts these rules, follow this file and explain the discrepancy to the user.
