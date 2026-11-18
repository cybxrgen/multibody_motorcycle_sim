#!/usr/bin/env bash
# Create a verified delivery (ZIP + manifest + sha256) from an exact Git commit.
#
# Usage:
#   Tools/make_delivery.sh <commit-ish> "<milestone>" "<tests executed>" "<test results>" "<known limitations>" "<godot opened/tested?>"
#
# Optional environment:
#   DELIVERY_COPY_DIR=<dir>   also copy the three files to <dir> (e.g. a user-download folder)
#                             and re-verify them there (sha256sum -c + byte comparison).
#
# Output goes to ./deliveries/ (git-ignored). Three SEPARATE files are produced; none is ever
# placed inside the ZIP:
#   motorcycle_multibody_<short>.zip  motorcycle_multibody_<short>_manifest.txt  motorcycle_multibody_<short>.sha256
# The ZIP comes from `git archive` of the commit, never from the working directory. The SHA-256 is
# computed only after the ZIP is final. Any change means rerunning this script: all three files are
# regenerated and re-verified together; a ZIP is never modified after its checksum exists.
set -euo pipefail

COMMIT_ISH="${1:?commit-ish required}"
MILESTONE="${2:-unspecified}"
TESTS_EXECUTED="${3:-none}"
TEST_RESULTS="${4:-not run}"
LIMITATIONS="${5:-see Documentation/PROJECT_STATE.md}"
GODOT_STATUS="${6:-NOT verified in Godot 4.7 (binary unavailable)}"

PROJECT="motorcycle_multibody"
ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

FULL="$(git rev-parse --verify "${COMMIT_ISH}^{commit}")"
SHORT="$(git rev-parse --short=7 "$FULL")"
BRANCH="$(git branch --show-current)"
SUBJECT="$(git log -1 --format=%s "$FULL")"
if [ -n "$(git status --porcelain)" ]; then TREE_STATE="dirty (ignored: archive is built from the commit only)"; else TREE_STATE="clean"; fi

OUT="deliveries"
mkdir -p "$OUT"
ZIP="${PROJECT}_${SHORT}.zip"
MANIFEST="${PROJECT}_${SHORT}_manifest.txt"
SHA="${PROJECT}_${SHORT}.sha256"
rm -f "${OUT}/${ZIP}" "${OUT}/${MANIFEST}" "${OUT}/${SHA}"   # never reuse stale artifacts

# 1. ZIP from the exact commit, single root directory.
git archive --format=zip --prefix="${PROJECT}/" "$FULL" -o "${OUT}/${ZIP}"

# 2. Integrity + content checks (before any checksum exists).
python3 - "$OUT/$ZIP" "$FULL" "$PROJECT" <<'PY'
import subprocess, sys, zipfile
zip_path, commit, project = sys.argv[1:4]
with zipfile.ZipFile(zip_path) as z:
    bad = z.testzip()
    if bad is not None:
        sys.exit(f"CRC failure in {bad}")
    comment = z.comment.decode("ascii", "replace")
    if comment != commit:
        sys.exit(f"ZIP commit stamp {comment!r} != requested commit {commit}")
    all_names = z.namelist()
names = {n for n in all_names if not n.endswith("/")}
tracked = [p for p in subprocess.check_output(["git", "ls-tree", "-r", "--name-only", "-z", commit]).decode().split("\0") if p]
expected = {f"{project}/{p}" for p in tracked}
if names != expected:
    sys.exit(f"Archive/commit mismatch: missing={sorted(expected-names)} extra={sorted(names-expected)}")
for n in all_names:
    parts = n.split("/")
    if parts[0] != project:
        sys.exit(f"Entry outside single root '{project}/': {n}")
    if any(p in (".git", ".godot", "deliveries") for p in parts[1:]):
        sys.exit(f"Forbidden path in archive: {n}")
    if n.endswith("_manifest.txt") or n.endswith(".sha256"):
        sys.exit(f"Delivery sidecar inside archive: {n}")
for required in ("project.godot", "AGENTS.md"):
    if f"{project}/{required}" not in names:
        sys.exit(f"{required} missing from archive")
print(f"archive verified: {len(names)} files, CRC ok, root '{project}/', commit stamp matches, matches committed file list")
PY
if command -v unzip >/dev/null 2>&1; then unzip -tq "${OUT}/${ZIP}" >/dev/null && echo "unzip -t: ok"; fi
FILE_COUNT="$(python3 -c "import sys,zipfile; print(sum(1 for n in zipfile.ZipFile(sys.argv[1]).namelist() if not n.endswith('/')))" "${OUT}/${ZIP}")"

# 3. Checksum only after the ZIP is final; verify it with sha256sum -c.
( cd "$OUT" && sha256sum "$ZIP" > "$SHA" && sha256sum -c "$SHA" )
ZIP_SHA="$(cut -d' ' -f1 "${OUT}/${SHA}")"

# 4. Manifest for the same commit.
cat > "${OUT}/${MANIFEST}" <<MANIFEST_EOF
Project:            Motorcycle Multibody Simulator
Godot target:       4.7 stable
Branch:             ${BRANCH}
Full commit:        ${FULL}
Short commit:       ${SHORT}
Commit message:     ${SUBJECT}
ZIP filename:       ${ZIP}
SHA-256:            ${ZIP_SHA}
ZIP size (bytes):   $(stat -c %s "${OUT}/${ZIP}")
Files in ZIP:       ${FILE_COUNT} (root directory ${PROJECT}/; no .git, .godot, manifest or checksum inside)
Working tree:       ${TREE_STATE}
Included milestone: ${MILESTONE}
Tests executed:     ${TESTS_EXECUTED}
Test results:       ${TEST_RESULTS}
Known limitations:  ${LIMITATIONS}
Opened/tested in Godot 4.7: ${GODOT_STATUS}
Generated (UTC):    $(date -u +%Y-%m-%dT%H:%M:%SZ)
MANIFEST_EOF

# 5. Verify manifest consistency against the real artifacts.
ACTUAL_SHA="$(sha256sum "${OUT}/${ZIP}" | cut -d' ' -f1)"
[ "$ACTUAL_SHA" = "$ZIP_SHA" ] || { echo "FATAL: checksum file does not match ZIP" >&2; exit 1; }
grep -qx "SHA-256:            ${ACTUAL_SHA}" "${OUT}/${MANIFEST}" || { echo "FATAL: manifest SHA-256 mismatch" >&2; exit 1; }
grep -qx "Full commit:        ${FULL}" "${OUT}/${MANIFEST}" || { echo "FATAL: manifest commit mismatch" >&2; exit 1; }
grep -qx "ZIP filename:       ${ZIP}" "${OUT}/${MANIFEST}" || { echo "FATAL: manifest ZIP name mismatch" >&2; exit 1; }
( cd "$OUT" && sha256sum -c "$SHA" )

# 6. Optional copy to a user-visible location, re-verified there.
if [ -n "${DELIVERY_COPY_DIR:-}" ]; then
  mkdir -p "$DELIVERY_COPY_DIR"
  for f in "$ZIP" "$MANIFEST" "$SHA"; do
    cp "${OUT}/${f}" "${DELIVERY_COPY_DIR}/${f}"
    cmp "${OUT}/${f}" "${DELIVERY_COPY_DIR}/${f}"
  done
  ( cd "$DELIVERY_COPY_DIR" && sha256sum -c "$SHA" )
  echo "Copied and re-verified in: ${DELIVERY_COPY_DIR}"
fi

echo "DELIVERY VERIFIED: ${SHORT} sha256=${ACTUAL_SHA}"
echo "Delivery: ${OUT}/${ZIP}"
echo "Manifest: ${OUT}/${MANIFEST}"
echo "Checksum: ${OUT}/${SHA}"
