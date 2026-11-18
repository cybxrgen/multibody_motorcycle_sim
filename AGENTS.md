# AGENTS.md — Permanent rules for every agent working on this repository

Project: custom rigid-body / constraint physics foundation (Godot 4.7, GDScript) for a
motorcycle + rider multibody simulator. The custom simulator is the **only** physical authority.

## Mandatory start-of-work routine
1. Read this file, `Documentation/HANDOFF.md`, `Documentation/PROJECT_STATE.md`, `Documentation/ARCHITECTURE.md`
   (and `PHYSICS_CONVENTIONS.md` before touching any math).
2. `git branch --show-current`, `git log --oneline -20`, `git status`.
3. Inspect the code before changing it. **Never trust a previous agent's claims** — re-run the tests and verify.
4. State the exact current state and open issues before starting.

## Architectural principles
- Physical state is owned by `SimWorld` / `SimRigidBody`. Godot nodes are visual/debug/UI only.
- No `RigidBody3D`, `Generic6DOFJoint3D`, `PhysicalBone3D`, or any built-in physics as authority.
- The physics core (`Simulation/Core`, `Simulation/Solver`) extends `RefCounted`, never touches the scene tree.
- Solver and bodies are generic. No motorcycle/rider special cases anywhere in the core.
- Never merge a rider's mass into a motorcycle body while also simulating the rider separately.
- SI units, right-handed, +Y up, local -Z forward, +X right. Quaternions for orientation.
- No arbitrary damping, hidden corrective forces, or position teleportation to hide solver problems.
  Every stabilization term needs a documented mathematical purpose.
- Numerical parameters live in `SimConfig` (`Simulation/Core/sim_config.gd`), not scattered in scripts. Their validity rules live in
  `SimConfig.validate()` only; `SimWorld` refuses to step while the config or any body's mass properties are invalid.
- No `class_name` globals: scripts reference each other with `preload("res://...")` constants so the project
  works headless without an editor import step (no `.godot` cache needed).
- Do not expand scope or add empty placeholder systems for future subsystems.

## Testing requirements
- Run before every commit:
  `godot --headless --path . --script res://Tests/run_tests.gd`
- Tests must validate physical outcomes with justified tolerances, not merely that code runs.
- Report failures honestly; never loosen a tolerance just to pass without recording why.
- If Godot is unavailable, say so explicitly everywhere (commit notes, manifests, handoff). Never claim a run that did not happen.

## Repository workflow
- Work on a feature branch (current: `foundation/simulator-core`). Never commit to `main`.
- Never push, never merge into `main`, never discard other people's uncommitted work.
- Each assignment ends in a coherent, testable commit with its documentation updated. Do not create intermediate commits
  for individual fixes unless the assignment explicitly allows several commits.
- Inspect `git diff` and `git status` before committing; no accidental files.

## User delivery gate (permanent, applies to every agent and every assignment)
- **A commit is not permission to begin the next commit.** A completed implementation commit is a user review checkpoint.
- After the commit, in this order: generate the ZIP, manifest and SHA-256 with `Tools/make_delivery.sh` from that exact
  commit; verify them; **share all three files with the user through a working user-accessible mechanism** (attachment or
  download); write the final report; **stop**.
- Then wait for the user's explicit instruction. The user decides when the next milestone or commit begins; a new
  assignment or explicit instruction is required before any further implementation.
- The user must be able to inspect, archive, download and independently preserve each completed project state.
- Agents must never:
  - create several commits consecutively without delivering the intermediate snapshots;
  - announce that a ZIP is complete and immediately proceed into the next commit;
  - keep implementing another milestone after the latest delivery has been generated;
  - treat an artifact inside an isolated sandbox as delivered to the user (printing a sandbox path is not delivery
    unless that path is exposed through a working download mechanism);
  - claim that a file was shared when only an internal path was printed. If the environment cannot publish artifacts,
    report that limitation honestly and stop.
- Never modify the project or a ZIP after its checksum was computed. Any change means a new commit (or, for a defect in
  the delivery itself, regenerating all three files) and full re-verification.

## Delivery requirements (every delivery)
- Run `Tools/make_delivery.sh <commit> ...` to produce, from that exact commit via `git archive`, three **separate** files in
  `deliveries/` (git-ignored): `motorcycle_multibody_<short>.zip`, `motorcycle_multibody_<short>_manifest.txt`,
  `motorcycle_multibody_<short>.sha256`. The manifest and checksum are never packed inside the ZIP.
- The ZIP has the single root `motorcycle_multibody/` and contains exactly the committed files (no `.git`, `.godot`).
- The script verifies CRC, the commit stamp, the file list, the SHA-256 (`sha256sum -c`) and manifest consistency.
  `DELIVERY_COPY_DIR=<dir>` copies the files to a user-visible location and re-verifies them there.

## Documentation upkeep
Update `HANDOFF.md`, `PROJECT_STATE.md`, `ARCHITECTURE.md`, `PHASE_PLAN.md`, `TESTING.md` (and `PHYSICS_CONVENTIONS.md` for math/precision rules) whenever behaviour changes.
All documents must describe the same actual state. Distinguish import/parse, headless runtime and visual inspection; claim only what was actually done; mark unknowns as unverified.
Documentation must never describe functionality that does not exist. Record failed approaches and decisions with reasons.
- Commit the `*.uid` files Godot generates next to scripts (stable resource references). Never commit `.godot/`.
- Godot 4.7 binary used for verification: `Godot_v4.7-stable_linux.x86_64` from godotengine/godot-builds release `4.7-stable`. Run `--import` once after a fresh checkout. Vector/Basis/Quaternion are float32: use tolerances >= 1e-6.
