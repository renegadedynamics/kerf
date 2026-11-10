#!/usr/bin/env bash
# Merges a unit branch into the current branch of the main checkout.
# Append-only docs (decisions, issues, changelog) are resolved by keeping both sides; any other conflict stops the
# merge for manual resolution. The merge is committed only if the whole solution builds and every test passes.
# Usage: merge-unit.sh [--no-gate | --all] <branch> "<title>"
# --no-gate commits without running the tests: for the first of two units merged together, whose gate is the second's.
# --all runs every suite; by default the gate runs those the merge's change can affect (choose-suites.pl, README.md).
set -u
S="$(cd "$(dirname "$0")" && pwd)"
cd "$(git -C "$S" rev-parse --show-toplevel)" || exit 1
gate=1; all=0
case "${1:-}" in --no-gate) gate=0; shift ;; --all) all=1; shift ;; esac
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
    kernel/tests/Kerf.Kernel.DrawLite/DrawBuiltins.cs|kernel/tests/Kerf.Kernel.DrawLite/DrawSourceFiles.cs)
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
# Runs the suites the merge's change can affect (choose-suites.pl: from the paths it changes and the project graph), or
# all of them with --all or when the choice says so; prints the choice, dotnet's output and the suites it ran
run_suites() {
  local logger=(-c Release -nologo --logger "console;verbosity=normal")
  local choice ran sln=kernel/obj/merge-unit-gate.slnx projects
  if [ $all = 1 ]; then choice="ALL	--all"
  elif ! choice=$(perl "$S/choose-suites.pl" 2>&1); then echo "$choice"; echo "RULE	choose-suites.pl failed"; return; fi
  echo "$choice"
  if echo "$choice" | grep -q '^ALL'; then
    dotnet test Kerf.slnx "${logger[@]}" 2>&1
    echo "Suites run: all (Kerf.slnx)"; return
  fi
  # the chosen projects, whole, in one solution so they run side by side as Kerf.slnx's do (in kernel/obj, so ignored)
  projects=$(echo "$choice" | awk -F'\t' '($1 == "TEST" && $3 == "-") || $1 == "BUILD" { print $2 }')
  if [ -n "$projects" ]; then
    mkdir -p kernel/obj
    { echo '<Solution>'; for p in $projects; do echo "  <Project Path=\"../../$p\" />"; done; echo '</Solution>'; } > $sln
    dotnet test $sln "${logger[@]}" 2>&1
    rm -f $sln
  fi
  # a suite's own check alone, where only its record or a project file changed
  while IFS=$'\t' read -r kind p filter why; do
    dotnet test "$p" "${logger[@]}" --filter "$filter" 2>&1
  done < <(echo "$choice" | awk -F'\t' '$1 == "TEST" && $3 != "-"')
  ran=$(echo "$choice" | awk -F'\t' '$1 == "TEST" { n = $2; sub(/.*\//, "", n); sub(/\.csproj$/, "", n); if ($3 != "-") n = n " (" $3 ")"; s = s (s ? ", " : "") n }
    $1 == "BUILD" { n = $2; sub(/.*\//, "", n); sub(/\.csproj$/, "", n); b = b (b ? ", " : "") n }
    END { print (s ? s : "none") (b ? "; built: " b : "") }')
  echo "Suites run: $ran"
}
if [ $gate = 1 ]; then
# the oracle tests are in Kerf.slnx since unit 10A: they replay native OCCT's recorded results, needing no native build
out=$(run_suites)
# the whole output, with each test's duration and the failures' messages (inside .git, so never committed)
echo "$out" > .git/merge-unit-test.log
echo "$out" | grep -E ": error [A-Z]+[0-9]+|Passed!|Failed!|Test Run (Successful|Failed|Aborted)|Total tests|\[FAIL\]" | sed 's/\[C:.*//' | sort | uniq -c
# the detailed logger prints "Test Run Failed." (not "Failed!") for a project with failures
echo "$out" | grep -E '^(Suites run|RULE|ALL|TEST|BUILD|NONE)' | cut -c1-200
if echo "$out" | grep -qE ": error [A-Z]+[0-9]+|Failed!|Test Run Failed|Test Run Aborted|\[FAIL\]|^RULE"; then echo "Stopping: build or tests failed; merge left uncommitted."; exit 3; fi
else
dotnet build Kerf.slnx -c Release -nologo -v q > .git/merge-unit-test.log 2>&1 || { echo "Stopping: the build failed; merge left uncommitted."; exit 3; }
note="$note Built, not tested: the next merge's gate tests it."
fi
git add -A
git -c core.safecrlf=false commit -q -m "Merge $branch: $title

$note

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1
