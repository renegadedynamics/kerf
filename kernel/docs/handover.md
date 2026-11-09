# Handover: finishing phases 5 and 6

> **Done, 31 October 2026.** Every unit below is merged, and `phase5/construction` is merged into `main`. The rest of this
> document is kept as the record of how phases 5 and 6 were finished. Phase 7 is not started, at the owner's request.

For the agent taking over OCCT.NET from 2026-10-30. Read this, then `docs/phase5-6-plan.md`, `docs/translation-rules.md`
and the decisions it points to.

## The owner's instructions

- **Current goal:** finish phases 5 and 6, merge `phase5/construction` into `main`, then **pause** and report. Don't
  start phase 7.
- **Standing rules:**
  - Work on branches; merge them yourself once tested.
  - Port OCCT class for class and reproduce its bugs. Mark each `// OCCT`, list it in `docs/occt-issues.md`, and fix
    it only later, once behaviour matches native. The exception is a bug that would hang or corrupt the caller's
    frozen input; fix those minimally and log the fix.
  - Record decisions as you go in `docs/decisions.md`, so the owner can review them.
  - Owner questions with provisional answers live in `docs/open-questions.md`. Follow the provisional answers; don't
    block on them.
- **Repositories:** since unit 10R this is the `kernel/` folder of Kerf (github.com/renegadedynamics/kerf), with
  build123d.NET in `../modeling/`. Never write to `../../bento` except on a branch.

## Environment

| Needed | Where | How |
|---|---|---|
| OCCT V8_0_1 source | `.deps/occt-src` | `tools/get-occt.ps1` |
| OCCT test dataset (99 MB) | `.deps/occt-testdata` | `tools/get-occt-testdata.ps1` |
| Native oracle | build123d.NET's native build (`native/out/win-x64/bin/b3docct.dll`, `.deps/install`): in `../modeling/`, or elsewhere with `-p:Build123dNativeRoot=` | `modeling/native/build-deps.ps1` and `build-shim.ps1` |
| Native recorders (`tools/*rec`) | MSVC 14.44 against build123d.NET's native OCCT | each tool's `build.ps1` |

Build and test with `dotnet test OcctNet.slnx`. In a worktree outside the repo, pass
`-p:OcctSourceDir=… -p:OcctTestDataDir=… -p:Build123dNetRoot=…` (absolute paths to the folders above).

## State

- `main`: phases 0–11, with phase 12 under way ([phase12-plan.md](phase12-plan.md)) and Kerf 0.4.0 prepared on 9 November 2026. Phase 11 closed on 7 November 2026 at Kerf 0.3.0, and what carries on is in its banner:
  [phase11-plan.md](phase11-plan.md); phase 10 on 5 November 2026 at Kerf 0.2.0: [phase10-plan.md](phase10-plan.md); phase 7 merged on 1 November 2026 from `phase7/finishing`, phases 8 and 9 on 2 November 2026,
  phase 9 from `phase9/switch`; phase 8 from
  `phase8/exchange`; see each plan's banner).
- `phase5/construction`: the integration branch for phases 5 and 6. Merged so far:
  - **Intersections:** 5A, 5B, 5C, 5D, 5E, 5F, 5G.
  - **Construction:** 5H, 5K1, 5K2, 5L1, 5L2, 5L3, 5M, 5O1, 5P, 5Q, 5R, 5S1, 5S2, 5T, 5O2, 5W.
  - **Booleans:** 6A, 6B, 6C, 6D, 6E, 6F, 6G, 6P, 6L, 6M.
  - **Healing and sewing:** 6H1, 6H2, 6H3, 6S, 6U.
- **Tests:** the full suite passes (10,008 tests, 9 skipped).
- **Native comparison:** everything the oracle reaches is bit-identical or attributed. Booleans are byte-identical
  with native on all 1,853 cases.
- **Decisions:** the last real number is 971.

## Releasing

As 0.4.0 was prepared (decision 2002), with the release check since unit 13C (decision 25301). From the repository's root:

1. **Everything for the release merged**, and the full corpus run and compared with the last release's, each difference
   attributed in a decision.
2. **The version:** `VersionPrefix` in `kernel/Directory.Build.props` and `modeling/Directory.Build.props`, and the two
   places `kernel/tools/pack-nuget.ps1` names it; each CHANGELOG's `Unreleased` becomes the version.
3. **The rig, on a quiet machine:** `kernel/tools/bench.ps1 -Compare` the run named last in decisions.md; its run goes in
   `kernel/docs/benchmarks/`, and a decision makes it the one to compare with.
4. **The packages:** `kernel/tools/pack-nuget.ps1`, which builds the committed HEAD and runs its smoke test on them.
5. **The release check:** `kernel/tools/release-check.ps1 -CandidatePackages <pack-nuget.ps1's folder> -Out
   kernel/docs/benchmarks/<date>-release-check.txt`. It runs users' cases on the last release's packages from nuget.org
   and on the new ones, and exits non-zero naming any case more than 20% slower or giving a different result
   ([performance.md](performance.md#the-release-check-unit-13c)). The release waits until it passes, or until each case it
   names is explained in a decision and given to it with `-Explained`.
6. **The owner** tests the packages and publishes them.

## Remaining units

Sizes are the plan's, in thousands of OCCT lines. Each row in `docs/phase5-6-plan.md` gives the unit's full scope and
tests.

| Unit | k lines | State | Needs | Placeholder decisions |
|---|---:|---|---|---|
| **5S2** Pipe shells | ≈11.6 | Not started | merged | 7201 |
| **5T** Lofts and projection | 3.8 | Not started | 5S2 for pipe-related parts; ThruSections needs only 5L3, 5S1, 6P | 7301 |
| **5O2** 2D wire offset | ≈10.7 | About half, on `phase5/u5o2` (6cafb9a) | merged | its own 5995–5999 and 59951–59955 |
| **6L** Draw-lite: Booleans and healing | ≈3.5 C# | Not started | merged | 7401 |
| **6M** build123d.NET's Booleans | C# tests | Not started | 6L | 7501 |
| **5W** Draw-lite: construction | ≈2 C# | Not started | 5S2, 5T, 5O2 | 7601 |

### 5S2: pipe shells (`BRepFill_Sweep`, `TrimShellCorner`, `PipeShell`; `BRepOffsetAPI_MakePipeShell`, `MiddlePath`)

- **The placeholder:** 5S1 left `src/OcctNet.ModelingAlgorithms/BRepFill/BRepFill_Sweep.cs` as an internal
  placeholder whose constructor throws (decision 909). Replace it with the port. Keep the constructor (section law,
  location law, bool) and the members `BRepFill_Pipe` calls, with their signatures: `SetBounds`, `SetForceApproxC1`,
  `Build` (8 arguments), `IsDone`, `Shape`, `ErrorOnSurface`, `SubShape`, `InterFaces`, `Sections`, `Tape`.
- **What depends on it:** `BRepOffsetAPI_MakePipe`, Draw-lite's `pipe` (58 corpus cases now fail on the placeholder)
  and Evolved's ElementaryPerform. Turn `DrawPipeMakerCommandsTests` and `BRepFill_PipeTests` into passing runs.
- **Accessors:** use the internal, non-freezing ones: `InternalPerformVertex`, `Section.InternalWire`, CompatibleWires'
  `InternalShape`/`InternalGenerated`, `GeomFill_Sweep.InternalSurface`/`InternalRestriction`/`InternalTrace`.
- **Frozen shapes:** `SectionLaw.D0` and `Vertex` hand out frozen shapes, which for a constant ShapeLaw are the
  caller's own wire. Copy before writing into anything built from them.
- **Laws:** Sweep calls `SetInterval`/`SetTolerance` on the laws it is given, so keep BRepFill's per-edge copies of
  the laws.
- **Native comparison:** the oracle is `MakePipeShell`, which is bound natively: every mode and transition, solids
  and shells, `Simulate`, `Generated`.
- **Performance:** CorrectedFrenet on offset free-form spines is very slow natively and slower in the port (unit 5L1;
  the Release figure is still to measure).

### 5T: lofts and projection (`BRepFill_Generator`, `BRepOffsetAPI_ThruSections`, `BRepProj_Projection`)

- **Compatible wires:** use CompatibleWires' internal accessors. Its result wires share the caller's frozen edges
  where it didn't cut them, so copy before writing pcurves or tolerances into them.
- **Native variation:** compare lofts whose first section is also the last canonically, because native varies from
  run to run there.
- **Sections and surfaces:**
  - `BRepFill_NSections` reads `InternalBSplineSurface`.
  - The smooth lofts use GeomFill's SectionGenerator, Line and AppSurf (5L3's notes, decision 852).
- **Oracle:** ThruSections and BRepProj_Projection are bound natively.
- **Unskip:** the two `BOPAlgo_PaveFillerTests` loft GTests, skipped until ThruSections existed.

### 5O2: finish the 2D wire offset

- **Branch:** `phase5/u5o2` (6cafb9a), with the merged fork `phase5/u5o2-evolved` (b690c7c). Merge
  `phase5/construction` into it first.
- **Done:**
  - MultiLine, ComputeCLine, ApproxSeewing, TrimSurfaceTool.
  - BRepFill_Evolved and MakeEvolved without the Boolean modes; Draw-lite `evolved`.
  - OffsetWire, TrimEdgeTool, OffsetAncestors: ported, but **never run**.
  - `BRepFill.Axe`: it is also in `BRepFill.cs`, which 5S1 merged. Resolve the conflict by keeping 5S1's file.
- **Still to do:**
  1. **`BRepOffsetAPI_MakeOffset`, with `ConvertFace`:**
     - in `BuildDomains`: `FR.Init(face, true)`, `Perform()`, `InternalCurrent()`;
     - the approximation option uses `BRepAlgo.InternalConvertWire`;
     - copy `myWires` with `Assign`.
  2. **Tests:** unit and frozen-input tests for OffsetWire and MakeOffset.
  3. **Oracle:** MakeOffset is bound natively. Cover open and closed wires, both join types, both signs, and
     build123d.NET's `Offset2D` and slot objects.
  4. **Draw-lite:** `mkoffset` and `openoffset`; teach DrawCorpus the offset group's `command` branch. About 552 corpus
     cases are expected.
  5. **Skipped tests:** fix and unskip two:
     - `BRepFill_EvolvedTests.Vertical_profile_off_the_spine_follows_a_parallel`, whose expected values were estimates;
     - `DrawEvolvedCommandsTests.Evolved_builds_walls_and_solids`, which fails on a Tcl option error.
  6. **Evolved:** its MakePipe path works once 5S2 lands. The volume mode waits for phase 7.
  7. **Docs:** decisions, OCCT issues and the CHANGELOG. Issues to log:
     - OffsetWire's `GeneratedShapes` iterates in address order natively;
     - `myCallGen` is uninitialised in OCCT;
     - TrimSurfaceTool lengthens the live bisector in place;
     - Evolved's `myMap` order.
- **From 5M:** maps keyed by MAT objects compare by reference. Copy `GeomBis(...).Value()` before trimming it.

### 6L: Draw-lite Booleans and healing (the phase 6 corpus milestone)

- **Scope:** BOPTest's commands (`bop*`, `bfuse`, `bcut`, `bcommon`, `bsection`, `bbuild`, `bbop`, `bsplit`,
  `edgestofaces`, the options, the DS debug commands), BRepTest's history commands, `sewing`, `fastsewing`, and
  SWDRAW's ShapeFix, ShapeAnalysis and ShapeTool commands (`fixshape`, `fixwgaps`, `fixsmall`, `fbclose`, …), plus
  `removeloc` and `reshape`.
- **Already in place from 6G:**
  - `BOPTestObjects`, the Boolean session per interpreter, with `baddobjects`, `baddtools`, `bfillds`, the options and
    the history commands;
  - `bopcheck`, `bopargcheck`, `bopapicheck`;
  - `mkvolume`, `bc*`, the periodicity commands, `makeconnected` and `removefeatures`.

  Extend `BOPTestObjects` with partial files.
- **Destructive mode:** Draw's default is the destructive mode, so on frozen shapes the port's results can differ from
  DRAWEXE's. The port is always non-destructive (decision 577 in the plan). Compare against native's non-destructive
  result.
- **Expected size:** this unlocks the most corpus cases of any unit. The scan expects about 4,000 cases with public
  data.

### 6M: build123d.NET's Booleans (the phase 6 milestone)

- **What it is:** a harness that replays build123d.NET's own Boolean and `Clean()` calls. `Clean` is
  `ShapeUpgrade_UnifySameDomain(shape, true, true, true)` with `AllowInternalEdges(false)`.
- **How to compare:** against native's non-destructive result, bytes first, then canonically. Compare histories after
  `Clean` as sets.
- **Without changing build123d.NET:** open question 20 proposes a test-only capture hook there, and that change is the
  owner's to make. Until they agree, rebuild the operands from build123d.NET's public API.
- **Benchmarks:** add the Booleans benchmark suite (plan 2.9).

### 5W: Draw-lite construction (the phase 5 corpus milestone)

- **Commands:** `thrusections`, the sweep commands (`mksweep`, `addsweep`, `buildsweep`, `simulsweep`…), `gener`,
  `middlepath`, `geompipe`, `prj`, `ruled`, and continuity.
- **Not yet in Draw-lite:** any of the filling, Fillet2D, constraint, `sweep`/`tuyau`, `appsurf`/`fillcurves`, fair
  curve, `pipe`, `evolved` and `bisec` commands that aren't already there.
- **Last:** this unit runs last, because it needs 5S2, 5T and 5O2.

## How the work was done

- **Units:** one branch and one git worktree per unit, `phase5/u<unit>`, cut from the latest `phase5/construction`.
  - Each helper reads the plan row, the decisions it lists and the previous units' notes.
  - Each ports, tests against native, logs bugs and decisions, and commits.
  - Big units fork into sub-worktrees and merge them.
- **Brief each helper with:**
  - **The worktree path, and what it must not touch:** never write into the main checkout, never create junctions.
    Helpers have written into the main checkout by mistake several times.
  - **The build properties.**
  - **The decisions to read**, and earlier units' notes copied into the brief.
  - **Placeholder decision numbers:** one range of 5000 or more per unit (1000 or more until phase 7, when real numbers passed 1000).
  - **The oracle method:** use the native binding where it exists; otherwise a native recorder in `tools/<name>rec`
    built with MSVC, its recording embedded in the tests.
  - **The final report:** what you need back.
- **Merging:** one unit at a time with `tools/merge/merge-unit.sh` (see `tools/merge/README.md`), then remove the
  unit's worktree and branch. Delete long paths with PowerShell `Remove-Item -LiteralPath "\\?\<path>" -Recurse
  -Force`, first checking that the variable holding the folder name isn't empty.
- **Port status:** regenerate `docs/port-status.md` with `dotnet run tools/port-status.cs` before merging to `main`.

## Conventions that bite

- **Frozen shapes and geometry:**
  - Public results are frozen. Ported code reads the `Internal*` accessors, which don't freeze.
  - Algorithms that edit their input copy at the entry, using `BRepTools_CopyAtEntry` or `ExactCopyForWrite` plus
    `KeepUnchanged`, and give back the caller's own sub-shapes where nothing changed.
  - A write into a frozen shape throws `TopoDS_LockedShape`.
- **The Free flag (open question 1):** results that share the caller's frozen sub-shapes keep `Free = 1` where native
  writes 0. The oracle masks that column, and counts the masked cases.
- **Determinism:** parallel must equal serial bit for bit. Run loops through `OSD_Parallel` and `BOPTools_Parallel`,
  with one context per worker.
- **Iteration order:**
  - Integer maps that OCCT iterates use the OCCT-ordered `NCollection_IntMap` and `IntDataMap` (decision 784).
  - Maps OCCT keys by address iterate in insertion order in the port. Log each site where that order is observable.
- **Argument checks:** native OCCT keeps them (decision 9, as corrected), so match native's exception types.
- **Process-wide state:** Draw-lite tests that change process-wide state, such as the mesher's defaults, share the
  non-parallel `MeshDefaultsCollection`. DrawCorpus resets those defaults after each case.

## When everything is merged

1. Regenerate `docs/port-status.md`.
2. Update `docs/design.md`'s phase table and `CHANGELOG.md`.
3. Merge `phase5/construction` into `main` with `--no-ff`, and check that `main`'s tree equals the tested branch's.
4. Report to the owner and pause.
