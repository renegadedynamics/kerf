# Merge tooling

Used to merge each unit's branch into an integration branch (today `phase5/construction`) with a test gate. Run from
Git Bash on Windows; needs `perl` and the .NET SDK.

## `merge-unit.sh <branch> "<title>"`

Merges `<branch>` into the branch checked out in the repository that holds this script, then:

1. **Refuses to start** if a merge is in progress, the tree is dirty, or there are untracked files (exit 4). Helpers
   have twice left stray files in the main checkout; this stops them being committed.
2. **Resolves conflicts that are always "keep both sides":**
   - appended sections of `docs/decisions.md`, `docs/occt-issues.md`, `CHANGELOG.md` and `docs/port-status.md`;
   - `*.csproj` conflicts where every conflicting line is an `<EmbeddedResource>` item or a comment (units append
     their native recordings at the same place);
   - `tests/OcctNet.DrawLite/DrawBuiltins.cs` conflicts where every conflicting line is an `X.Register(theInterpreter);`
     call or a comment.

   Any other conflict stops the merge (exit 2) and leaves it for you to resolve by hand.
3. **Runs `renumber.pl`** to turn the unit's placeholder decision numbers into real ones.
4. **Runs `dotnet test OcctNet.slnx`** and commits only if everything passes (exit 3 and the merge left uncommitted
   otherwise). The failing tests' names are printed.

A full gate takes about 25 minutes on an idle machine and up to an hour when several helpers share it. Merge units one
at a time, each as its own command, and check the exit code directly: a pipe through `grep` hides it.

If a gate fails for a reason outside the unit, fix it in the working tree (the merge is still staged), rerun the
failing project's tests, and commit the merge with a message that says what was fixed.

## `renumber.pl <repo> [floor]`

Helpers number their decisions with placeholders of 1000 or more (one range per unit, e.g. 7201…). This script gives
every placeholder heading in `docs/decisions.md` the next real number in file order, and rewrites every reference to
them ("decision 7203", "decisions 7201, 7202 and 7205–7207") in `docs`, `src`, `tests`, `tools`, `samples` and
`benchmarks` (`.md .cs .csproj .props .targets .ps1 .py .cpp .sh .sed`).

## `decision-map.txt`

Every mapping applied so far (`placeholder real`, latest last). A reference to a placeholder with no heading in the
merge being renumbered (a unit citing a decision renumbered before it merged) takes its latest mapping, so stale
citations are corrected automatically. Placeholders are reused across units, so a heading in the current merge always
wins over this history. Commit the updated file after each merge.
