#!/usr/bin/env bash
# Merges a unit branch into the current branch of the main checkout.
# Append-only docs (decisions, issues, changelog) are resolved by keeping both sides; any other conflict stops the
# merge for manual resolution. The merge is committed only if the whole solution builds and every test passes.
# Usage: merge-unit.sh <branch> "<title>"
set -u
S="$(cd "$(dirname "$0")" && pwd)"
cd "$(git -C "$S" rev-parse --show-toplevel)" || exit 1
branch="$1"; title="$2"
if [ -e .git/MERGE_HEAD ] || [ -n "$(git status --porcelain)" ]; then echo "Stopping: a merge is in progress or the tree is dirty or has untracked files."; exit 4; fi
git merge --no-ff --no-commit "$branch" >/dev/null 2>&1
conflicts=$(git diff --name-only --diff-filter=U)
for f in $conflicts; do
  case "$f" in
    docs/decisions.md|docs/occt-issues.md|CHANGELOG.md|docs/port-status.md)
      sed -i '/^<<<<<<< /d; /^=======$/d; /^>>>>>>> /d' "$f"; git add "$f" ;;
    *.csproj)
      # Units append their native recordings to the test projects at the same place: keep both sides, but only when
      # every conflicting line is an EmbeddedResource item or a comment
      if awk '/^<<<<<<< /{c=1;next} /^=======$/{next} /^>>>>>>> /{c=0;next} c && !/^[ \t]*(<!--.*-->|<EmbeddedResource [^>]*\/>)[ \t\r]*$/ {bad=1} END{exit bad}' "$f"; then
        sed -i '/^<<<<<<< /d; /^=======$/d; /^>>>>>>> /d' "$f"; git add "$f"; echo "kept both resource lists in $f"
      else
        echo "CODE CONFLICT: $f"
      fi ;;
    tests/OcctNet.DrawLite/DrawBuiltins.cs|tests/OcctNet.DrawLite/DrawSourceFiles.cs)
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
perl "$S/renumber.pl" .
out=$(dotnet test OcctNet.slnx -nologo 2>&1)
echo "$out" | grep -E " error |Passed!|Failed!|\[FAIL\]" | sed 's/\[C:.*//' | sort -u
if echo "$out" | grep -qE " error |Failed!"; then echo "Stopping: build or tests failed; merge left uncommitted."; exit 3; fi
git add -A
git -c core.safecrlf=false commit -q -m "Merge $branch: $title

Decisions renumbered into sequence.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1
