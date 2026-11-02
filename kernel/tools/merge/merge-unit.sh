#!/usr/bin/env bash
# Merges a unit branch into the current branch of the main checkout.
# Append-only docs (decisions, issues, changelog) are resolved by keeping both sides; any other conflict stops the
# merge for manual resolution. The merge is committed only if the whole solution builds and every test passes.
# Usage: merge-unit.sh [--no-gate] <branch> "<title>"
# --no-gate commits without running the tests: for the first of two units merged together, whose gate is the second's.
set -u
S="$(cd "$(dirname "$0")" && pwd)"
cd "$(git -C "$S" rev-parse --show-toplevel)" || exit 1
gate=1; if [ "${1:-}" = "--no-gate" ]; then gate=0; shift; fi
branch="$1"; title="$2"
if [ -e .git/MERGE_HEAD ] || [ -n "$(git status --porcelain)" ]; then echo "Stopping: a merge is in progress or the tree is dirty or has untracked files."; exit 4; fi
git merge --no-ff --no-commit "$branch" >/dev/null 2>&1
conflicts=$(git diff --name-only --diff-filter=U)
for f in $conflicts; do
  case "$f" in
    kernel/docs/decisions.md|kernel/docs/occt-issues.md|kernel/CHANGELOG.md|kernel/docs/port-status.md|modeling/CHANGELOG.md)
      sed -i '/^<<<<<<< /d; /^=======$/d; /^>>>>>>> /d' "$f"; git add "$f" ;;
    *.csproj)
      # Units append their native recordings to the test projects at the same place: keep both sides, but only when
      # every conflicting line is an EmbeddedResource or None item, or a comment (on one line or several)
      if awk '/^<<<<<<< /{c=1;next} /^=======$/{next} /^>>>>>>> /{c=0;next} !c{next} incom{ if (/-->/) incom=0; next } /^[ \t]*<!--/ && !/-->/ {incom=1; next} !/^[ \t]*(<!--.*-->|<(EmbeddedResource|None) [^>]*\/>)[ \t\r]*$/ {bad=1} END{exit bad}' "$f"; then
        sed -i '/^<<<<<<< /d; /^=======$/d; /^>>>>>>> /d' "$f"; git add "$f"; echo "kept both resource lists in $f"
      else
        echo "CODE CONFLICT: $f"
      fi ;;
    kernel/tests/OcctNet.DrawLite/DrawBuiltins.cs|kernel/tests/OcctNet.DrawLite/DrawSourceFiles.cs)
      # Units each add a line registering their Draw-lite commands at the same place: keep both sides, but only when
      # every conflicting line is a registration call or a comment
      if awk '/^<<<<<<< /{c=1;next} /^=======$/{next} /^>>>>>>> /{c=0;next} c && !/^[ \t]*([A-Za-z0-9_]+\.Register\(theInterpreter\);|\/\/.*)?[ \t\r]*$/ {bad=1} END{exit bad}' "$f"; then
        sed -i '/^<<<<<<< /d; /^=======$/d; /^>>>>>>> /d' "$f"; git add "$f"; echo "kept both registrations in $f"
      else
        echo "CODE CONFLICT: $f"
      fi ;;
    *) echo "CODE CONFLICT: $f" ;;
  esac
done
if git diff --name-only --diff-filter=U | grep -q .; then echo "Stopping: resolve the code conflicts by hand."; exit 2; fi
perl "$S/renumber.pl" kernel
note="Decisions renumbered into sequence."
if [ $gate = 1 ]; then
out=$(dotnet test Kerf.slnx -c Release -nologo --logger "console;verbosity=normal" 2>&1)
# the oracle tests, outside Kerf.slnx while native OCCT is in use (phase 10, unit 10R): against the native build in
# BUILD123D_NATIVE_ROOT (build123d.NET's native/out and .deps/install), compiled out without one
out="$out
$(dotnet test kernel/tests/OcctNet.Oracle.Tests -c Release -nologo --logger "console;verbosity=normal" ${BUILD123D_NATIVE_ROOT:+"-p:Build123dNativeRoot=$BUILD123D_NATIVE_ROOT"} 2>&1)"
# the whole output, with each test's duration and the failures' messages (inside .git, so never committed)
echo "$out" > .git/merge-unit-test.log
echo "$out" | grep -E ": error [A-Z]+[0-9]+|Passed!|Failed!|Test Run (Successful|Failed|Aborted)|Total tests|\[FAIL\]" | sed 's/\[C:.*//' | sort | uniq -c
# the detailed logger prints "Test Run Failed." (not "Failed!") for a project with failures
if echo "$out" | grep -qE ": error [A-Z]+[0-9]+|Failed!|Test Run Failed|Test Run Aborted|\[FAIL\]"; then echo "Stopping: build or tests failed; merge left uncommitted."; exit 3; fi
else
dotnet build Kerf.slnx -c Release -nologo -v q > .git/merge-unit-test.log 2>&1 || { echo "Stopping: the build failed; merge left uncommitted."; exit 3; }
note="$note Built, not tested: the next merge's gate tests it."
fi
git add -A
git -c core.safecrlf=false commit -q -m "Merge $branch: $title

$note

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1
