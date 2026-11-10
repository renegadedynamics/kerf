# Merge tooling

Used to merge each unit's branch into an integration branch (today `phase5/construction`) with a test gate. Run from
Git Bash on Windows; needs `perl` and the .NET SDK.

## `merge-unit.sh [--no-gate | --all] <branch> "<title>"`

Merges `<branch>` into the branch checked out in the repository that holds this script, then:

1. **Refuses to start** if a merge is in progress, the tree is dirty, or there are untracked files (exit 4). Helpers
   have twice left stray files in the main checkout; this stops them being committed.
2. **Resolves conflicts that are always "keep both sides":**
   - appended sections of `docs/decisions.md`, `docs/occt-issues.md`, `CHANGELOG.md` and `docs/port-status.md`;
   - `*.csproj` conflicts where every conflicting line is an `<EmbeddedResource>` or `<None>` item or a comment (units append
     their native recordings at the same place);
   - `tests/Kerf.Kernel.DrawLite/DrawBuiltins.cs` conflicts where every conflicting line is an `X.Register(theInterpreter);`
     call or a comment.

   Any other conflict stops the merge (exit 2) and leaves it for you to resolve by hand.
3. **Runs `renumber.pl`** to turn the unit's placeholder decision numbers into real ones.
4. **Runs the test suites the change can affect** (`choose-suites.pl`, below), in Release, and commits only if
   everything passes (exit 3 and the merge left uncommitted otherwise). The change is everything since the last merge
   this gate tested, so a merge committed with `--no-gate`, or a commit made on the branch directly, counts too. It
   prints the choice and ends with `Suites run:`, naming them. With `--all`, or when the choice says so, it runs
   `dotnet test Kerf.slnx -c Release` (the kernel and the modeling layer), as it did before unit 15J. The failing
   tests' names are printed. The whole test output is kept in `.git/merge-unit-test.log`, with the failures' messages.

**A release always runs every suite:** the release branch's merge (decision 2086's process, `release/x.y.z`) runs them
all, whatever the change since the last merge: the script takes `--all` for a branch named `release/…` by itself.

Every suite together takes about six minutes on an idle machine, plus the build (decision 2085: 345 to 368 s, from 1,246 s before
unit 14H), longer when helpers share the machine. The longest project bounds it: Kerf.Modeling.Tests (about 6 minutes),
then DrawLite (5.5), DataExchange and ModelingAlgorithms (4 to 5). DrawLite's corpus cases are rows of shard classes over
one shared scan and one run of each case (decision 2082), so a corpus test that grows wants more shards (its
`CorpusPlan`'s count and a line each in `CorpusShards.cs`, about 30 s of case time a shard), not a class of its own.
Merge units one
at a time, each as its own command, and check the exit code directly: a pipe through `grep` hides it.

If a gate fails for a reason outside the unit, fix it in the working tree (the merge is still staged), rerun the
failing project's tests, and commit the merge with a message that says what was fixed.

**Two units at once.** When both units of a step are ready, merge the first with `--no-gate` (it is resolved,
renumbered and built in Release, then committed untested) and the second without: its gate tests both. If that gate
fails, the failure may be either unit's; the log says which tests.

## `choose-suites.pl [--rev <commit>] [--base <commit>] [--explain]`

Chooses the suites a change can affect (unit 15J, decision 2118), and prints each changed path with why it counts,
then the choice: `ALL`, `TEST <project> <filter or ->`, `BUILD <project>` (in the solution, not a test, and built by
no chosen test), `DEP <project>` (what those build on, listed so it's built in Release too: MSBuild builds a reference
outside the solution being built in Debug) or `NONE`. With no `--rev` the change is the working tree and index against
`--base` (`HEAD` by default), as in the gate; with `--rev` it is a commit against its first parent, to replay a merge;
`--explain` adds, for each test suite not chosen, what it builds on, checked to hold none of the changed projects.

The project graph is read from the `.csproj` files and every `Directory.Build.props` above them, at both ends of the
change: each project's references (with `$(KerfModelingRoot)`, `$(KerfKernelRoot)` and `$(MSBuildThisFileDirectory)`
resolved; an unknown property runs everything) and the files it takes from outside its folder (`Compile`,
`EmbeddedResource`, and `None` or `Content` copied to the output, such as `kernel/tests/Shared`'s and the PaveFiller
recordings the oracle shares). A changed path is then, in this order:

- **everything** if nothing can place it: a `Directory.Build.props` or `.targets`, a solution, `.editorconfig`,
  `.gitattributes`, `nuget.config`, `global.json`, `occt-version.json`, `kernel/.deps`, or the analyzers
  (`kernel/src/Kerf.Analyzers`, which every kernel project runs), or any path no rule below takes;
- **the public API's record** (`kernel/docs/api/public-api.txt`, `modeling/docs/public-api.txt`): its suite's
  `PublicApiBaselineTests` alone, unless the suite runs whole anyway;
- **an input of the projects** whose folder holds it (the deepest) or that name it as an item; a changed `.csproj` also
  runs `ProjectGraphTests`, the rule below;
- **a document** (`*.md`, any `docs/` folder, the licences and notices, `.gitignore`) that no project names: nothing;
- **a tool or benchmark** no project builds (`kernel/tools`, `kernel/benchmarks`, `modeling/tests/reference`): the
  benchmarks' build. A tool or benchmark a test project builds on is an input like any other: `stepgen` is
  DataExchange's tests', `Kerf.UserCases` Kerf.Modeling.Tests'.

Every test project that builds on a changed project, directly or through others, runs whole; the rest of the solution
that does builds. A project outside `Kerf.slnx` (`kerf-rename`, `Kerf.Modeling.Packaging.Tests`) is never run, as
before. Choosing takes a few seconds.

**The rule:** no project under `kernel/src` or `kernel/tests` builds on Kerf.Modeling, directly or through another
project, so a change to the modelling layer alone runs none of the kernel's suites. `ProjectGraphTests` in
Kerf.Kernel.PublicApi.Tests checks it, and the script stops (exit 5, the gate exit 3) if a change breaks it.

## `renumber.pl <repo> [floor]`

Helpers number their decisions with placeholders of 5000 or more (one range per unit, e.g. 8201…). Real numbers passed 1000 in phase 7, so the threshold was raised from 1000 to 5000. This script gives
every placeholder heading in `docs/decisions.md` the next real number in file order, and rewrites every reference to
them ("decision 926", "decisions 924, 925 and 928–930") in `docs`, `src`, `tests`, `tools`, `samples` and
`benchmarks` (`.md .cs .csproj .props .targets .ps1 .py .cpp .sh .sed`).

## `decision-map.txt`

Every mapping applied so far (`placeholder real`, latest last). A reference to a placeholder with no heading in the
merge being renumbered (a unit citing a decision renumbered before it merged) takes its latest mapping, so stale
citations are corrected automatically. Placeholders are reused across units, so a heading in the current merge always
wins over this history. Commit the updated file after each merge.
