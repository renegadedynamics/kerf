# Phase 11 plan: after 0.2.0

> **Agreed, 5 November 2026** (decision 1644). Started. The owner's order: "Let's get these phase 10 units done, then we
> can go ahead and optimise from there." So optimisation comes first, and the points the owner set aside until after 0.2.0
> follow it. The next release is 0.3.0, with no fixed scope and no rush: possibly Saturday 7 November (decision 1681).

## The goal

Phases 0 to 10 made Kerf correct and made it a .NET library. Phase 11 makes it fast, measured on what Bento does, and then
closes what phase 10 left, so that a 1.0 can be considered.

## 1. Where the time stands

**Unit 11A's baseline, 5 November (0.2.0's code, ReadyToRun; [performance.md](performance.md)):** recalculation 1,211 ms
(1.77× native), opening 1,764 ms (1.92×), allocation 450 MB. The rest of this section is as written before it.

The last full review is [performance.md](performance.md) (unit 9E, 2 November), against native OCCT on Bento's eleven guide
examples:

| | Native | Kerf at 9E, ReadyToRun |
|---|---:|---:|
| Opening a workbook (first pass), ms | 921 | 2,034 |
| Recalculation (median), ms | 685 | 1,162 |
| Allocated per recalculation, MB | 3 | 1,025 |

Unit 10J since cut "extrude until" from 802 MB to 142 MB a recalculation. Phase 10 changed most of the code above the
engine, and 10Q2's before-and-after only shows it did no harm (936 ms against 949 ms, summing medians). So these figures
are stale, and the first job is to measure again.

Three things to keep in mind:
- **Memory is the widest gap.** Allocation also drives the parallel cost: under load, the garbage collector was the
  largest single loss at 9E.
- **Opening is mostly the JIT.** ReadyToRun took about a third off; what's left needs measuring separately.
- **Native is a yardstick, not the goal.** Its figures are frozen with the recordings, and can't be measured again.

## 2. The optimisation units

**The targets (the owner's):** recalculation within 1.2× of native's frozen figure, so **822 ms** from 1,211, and allocation
cut by half, from 11A's baseline, so **225 MB** per recalculation from 450.

**How the work runs (the owner's):**
- **Pragmatic:** longer-running and more commonly used operations come first. 11A ranks them.
- **A time box for each operation:** one helper's run, about two hours. An operation that doesn't reach its target by then
  gets a note in `performance.md`, with what was tried and what's left. The owner hears about it, and the work moves on to
  the next operation.
- **Cheap wins are welcome:** if something beats the target at little cost, take it.

In order. Each takes its figures from 11A, and states before and after. Results stay identical unless a unit says
otherwise and records why: the corpus, the recordings and the Python-reference values are the check.

| Unit | What | Done when |
|---|---|---|
| **11A** Benchmark rig | One command that runs a fixed set on a quiet machine and writes a dated table: Bento's examples (first pass, recalculation, allocation), Booleans, fillets, meshing, STEP read and write, parallel throughput. Fails if the machine is busy. Bring `Kerf.Kernel.Bento.Benchmarks` and `Kerf.Kernel.Benchmarks` into a build the gate checks, since 10Q broke one unnoticed. | A baseline table for 0.2.0 is in `performance.md`; a regression check exists. **Done** (decisions 1646 to 1651): `tools/bench.ps1`, with `-Quick` and `-Compare`; both benchmarks in the gate's build; the baseline and the ranking are in `performance.md`. "Extrude until" and the bottle are 93% of the excess over target; meshing and its tolerance box lead the ranking |
| **11B** Allocation | Profile by allocation. Likely first: small vectors and matrices on the stack (performance.md's proposal 5), then whatever else the profile names. Take the remaining collections from [collections-plan.md](collections-plan.md) only where they show up: that plan is about 25,000 references, and converting it all isn't the aim. | **Done: 450 MB to 217 MB** (target 225), with recalculation 1,211 ms to 1,095 ms. [performance.md](performance.md) |
| **11C** Opening | Separate JIT, type loading and first-call cost. Try composite ReadyToRun, tiered settings, and avoiding work at load (static tables, large initialisers). | **Done: opening 745 ms to 509, the first pass 710 ms to 473, recalculation 364 ms to 269,** no result changed, for packages 19.5 MB to 23.4. Nearly all of it is ReadyToRun inlining across modules and holding generics over other assemblies' types. Static tables were no cost; no new tiering setting helps. A version bubble (478 ms) and a host's composite (419 ms) are left as choices, with the figures. [performance.md](performance.md), "Unit 11C" |
| **11D** Parallel | Throughput across cores, with Bento's recalculation as the case. Contention from the 23 locks ([state.md](state.md)), shared caches, and the collector's settings. This is where the locks get looked at, so plan 2.7's "only one-time locks" is settled by evidence. | **Done:** the curve is in [performance.md](performance.md), "Unit 11D". A round fell from 2,763 ms to 620 under the settings a host should use, nearly all from settings (TieredPGO off, call counting from the start, the server collector); no lock costs anything, so all 23 stay (decisions 1683 to 1688) |
| **11E** Algorithms | With the profile from 11A, the hot paths: Boolean setup, "extrude until", healing after Booleans. Phase 10 allowed divergence from OCCT, so an algorithm may change, with its results checked as intended differences. | **Done:** the mesh's tolerance box and "extrude until", no result changed (decisions 1671 to 1675). Back to back on a busy machine, recalculation fell by about two thirds (1,500 ms to 500 ms); the quiet run comes after merging. [performance.md](performance.md) |
| **11F** Caching in Bento | **The owner is doing this separately; [modeling/docs/bento-caching.md](../../modeling/docs/bento-caching.md) has use cases and an outline.** It finds that Bento knows only which cells were edited, not which really changed. As first planned: Bento knows which cells really changed. Cache meshes and shapes across recalculations by the shapes' identity. Immutability makes this safe, and it may help users more than any engine work. On a Bento branch, as usual. | Recalculating an edit to one cell, measured |

**A separate question: fillets.** A rolling-ball rewrite (beyond-compatibility.md item 5) only makes sense with a list of
real failures in hand. **11G** sorts the fillet corpus's failures into reconstruction gaps, port bugs and OCCT's own, then
reports. Any rewrite is a decision after that, not part of this plan. **Done, 6 November (decisions 1719 to 1724):** [corpus-failures.md](corpus-failures.md). Three small reconstruction bugs fixed, 13 corpus cases with them. Of the 37 fillet cases still FAILED or BAD: 3 reconstruction gaps, 34 OCCT's own, no known port bug. Of 1,074 steps run against native, one is invalid only on the port. **Recommended: no rewrite now; the owner decides.**

**Known defects, done 6 November (decisions 1735 to 1741):** what 11G and 11I left. A fillet or chamfer that would silently lose a face of an open shell now fails, and four false IMPROVEMENTs are gone. J5 passes. G8 and bug27711_3 stay, as only their edge numbers differ. The projection on a very uneven spline, OCCT's own, is improved. Copied faces' history and radius 11 are confirmed as they are. Of 1,044 steps run against native, none is invalid only on the port. [corpus-failures.md](corpus-failures.md).

## 3. What phase 10 left

The owner set these aside until after 0.2.0 (decision 1642). They don't depend on the optimisation, and can run alongside it.

| Unit | What | Note |
|---|---|---|
| **11H** Re-measure coverage | About four hours with `-WithOracle -FullCorpus`. | **Done, 5 November:** 81.6% overall, unchanged; 81 packages under 80%, 40 of them outside `DataExchange`. 64 of `Kerf.Modeling`'s public members now have no test calling them, which goes first in 11J |
| **11I** Coverage floor (question 49) | 81.6% overall at the last measure; 81 of 272 packages under 80%, 41 of them STEP's. The corpus adds at most 0.2 points, so all of them are real gaps. | **The owner's decision:** hand-written engine packages first. Generated STEP entities are decided later. **First** (the owner's, decision 1669): `tools/coverage.ps1` scales the corpus's time limits. **First slice, 5 November:** the scale is 3, measured (decision 1679). Raised over the floor by real tests, measured by package: Bnd 62 to 85, PLib 74 to 99, MathOpt 59 to 82, MathUtils 57 to 81, MathInteg 61 to 85, MathLin 75 to 87, MathPoly 74 to 82, GC 48 to 100, gce 62 to 88, BndLib 66 to 85, Adaptor2d 72 to 96, BRepFilletAPI 64 to 83, ShapeAlgo 39 to 87. Tests found and fixed three of OCCT's bugs (decisions 1676 to 1678), and listed three more in `occt-issues.md`. Still under, by exposure: ShapeFix, ShapeConstruct, BRepBlend, Blend, FilletSurf, IntCurveSurface, Plate, Geom2dHatch, Geom2dInt, BRepApprox, LocOpe, BRepFeat, Draft, BiTgte, HLRBRep, Geom2dGcc, AppParCurves, Hermit, MathRoot, MathSys, NCollection, BVH, and the legacy Booleans. 11J's first slice is done too: the 64 uncalled members of `Kerf.Modeling` have direct tests. The figures in `coverage.md` wait for the next whole run. **Second slice, 6 November:** healing and blends over the floor, by 232 new test cases that check results by laws (native has gone, so no new oracle recordings): ShapeFix 74.9 to 84.7, ShapeConstruct 79.1 to 82.7, BRepBlend 80.2 to 83.7, Blend 65.1 to 96.5. Measured with `coverage.ps1 -Only Kerf.Kernel.ModelingAlgorithms.Tests`, merged with 11H's other suites carried through the source diff, so the other suites' part is inferred until the next whole run. One bug fixed (decision 1713), two more listed in `occt-issues.md`. Still under, by exposure: FilletSurf, IntCurveSurface, Plate, Geom2dHatch, Geom2dInt, BRepApprox, LocOpe, BRepFeat, Draft, BiTgte, HLRBRep, Geom2dGcc, AppParCurves, Hermit, MathRoot, MathSys, NCollection, BVH, and the legacy Booleans **Third slice, 6 November:** the whole run at main's head (`-WithOracle -FullCorpus`, 3 h 43 on a loaded machine) gave 83.3% and 64 packages under the floor, and it confirms the second slice's inferred figures exactly: ShapeFix 84.7, ShapeConstruct 82.7, BRepBlend 83.7, Blend 96.5. Then 522 new test cases by laws (decision 1753), measured by adding their runs and a fresh Foundation run to that whole run, since only two files' lines moved: 84.0% and 57 packages. BRepFeat 75.7 to 81.5, LocOpe 66.0 to 82.8, Plate 79.0 to 85.1, Geom2dGcc 68.1 to 92.8, BVH 72.4 to 91.9, MathRoot 77.0 to 82.3, MathSys 77.7 to 80.9; still under, higher: Draft 73.7 to 75.4, HLRBRep 75.4 to 76.7, IntCurveSurface 66.4 to 69.4, Geom2dHatch 72.8 to 77.9, FilletSurf 72.2 to 78.6. Four of OCCT's bugs fixed (decisions 1749 to 1752), more listed in `occt-issues.md`. IntCurveSurface can't reach the floor honestly: about 400 lines are dead in OCCT too. Still under, by exposure, apart from STEP: HLRBRep, Draft, Geom2dHatch, FilletSurf, BiTgte, IntCurveSurface, AppParCurves, Hermit, GeomBndLib, BRepApprox, Geom2dInt, TopBas and the legacy Booleans |
| **11J** Tests of their own (plan 3.2) | 0 of 4,417 public members are uncalled, but "called" includes indirect calls. Tighten to a direct call and a checked result, area by area, starting from what users meet first. | Large; do it in slices **Second slice, 6 November:** 15 tests call directly what no test named: tangent angles, `TrimInfinite`, `FindIntersections`, parabola and hyperbola arcs, connected edges and faces, front and back planes, `ToFrame3D`, `LocationBetween`, rotation orders, location lists, projections, curve positions, perpendicular lines and `ModelHistory.IsLast`. **Third slice, 6 November:** `tools/direct-members.cs` reads the test assemblies' IL and lists the members no test names (`direct-members.md`, counts in `coverage.md`): 993 of 4,463 before, 250 after. New tests, by laws and worked values, found eight bugs (decisions 1768 to 1774): the largest is that OCCT's lengths were 1.5% out on uneven curves. "Checked" is only a filter, that the naming test reaches an assertion. Left: `Kerf.Exchange`, `Kerf.Drawing`, the rest of `Kerf.Geometry` and the kernel's result records; coverage's remainder (`Draft`, `HLRBRep`, `Geom2dHatch`, `FilletSurf`) not started |
| **11K** The remaining exceptions | Operators, constructors and the shapes' own methods still throw (decisions 1589, 1592). | **Closed: the owner keeps them** (decision 1644) |
| **11L** Sharp edges in the API | From `one-set-of-shapes.md`'s "Left": `shape.BoundBox()` beside `shape.BoundingBox()`, with different defaults; clashes with `Kerf.Meshing`, `Kerf.Exchange` and `Kerf.Offsets`; odd internal names. Also error messages that still name OCCT's types: `Plane`'s "Expected gp_Pln…", which Bento shows in a cell for a zero `zDir`, and `TopoUtil.TypeName`'s `TopoDS_Face` and so on. And STEP's header, which still says organisation `Open CASCADE`, an Open CASCADE processor and the product name `Open CASCADE STEP translator`: change them to Kerf, and update STEP's fixtures once. | The API's points are best done before anyone relies on them. **The messages and STEP's header are the owner's for the next release** (decision 1643) |
| **11O** Deflection without a flag (decision 1680) | OCCT's mesher doesn't check surface deflection on elementary surfaces, so long fillets miss it: 3.7× on `data/occ/bottle.brep`, 1.7× on `Ball.brep`. `ControlAllSurfaces` narrows that, with more triangles, and is off by default. The owner feels we can do better here. Find a mesher that meets the deflection well enough, at a sensible triangle count, for the option to go altogether. | **The owner's note, for later.** Not for 0.3.0 |
| **11P** Flaky tests (decision 1681) | The ones known: `bug24157`'s time checks, and the memory trends of `bug7287` and `bug24863`, which flip between runs because they measure heap growth under a garbage collector, and `bug31075`. Find out why each flips, and redesign it if it can't be made reliable as it is. Every corpus comparison has had to explain these. | **Done, 6 November (decisions 1699 to 1704):** cases that read their process's figures run in a process of their own, and a heap reading holds only what the case keeps. No real leak; `bug31120`'s own check can't fail, so a test asks its question. `bug24639`'s parallel timing is left |
| **11Q** Slow use cases (decision 1681) | **Ongoing, before and after 0.3.0 (decision 1734):** keep finding, reviewing and fixing wasteful cases, time first. Once Kerf is close to native in most or all cases, stop comparing with native and simply improve what's slow. Build the list from what takes longest: the corpus's slowest cases and the rig's slowest suites, as well as Bento's examples. "Extrude until" is the model: most of its time went on a sort that barely needed to run. | The owner would happily spend more time on this, before or after 0.3.0. **First slice done:** what allocation is made of, the ranked list and nine changes (performance.md, "What allocation is made of", "Slow use cases"; decisions 1705 to 1711). **Second slice done:** what limits the Booleans' parallel option (the largest item of each loop, not serial phases), `Iterator` and `Explorer` as structs, `CurveOnPlane`'s table, three work sets (performance.md, "The second slice"; decisions 1714 to 1718). **Third slice done, going after time:** STEP export about a third less CPU and 48% of its bytes, import about 10% to 15% less, fillets on every edge 14% fewer bytes, each of the list's other items checked but max fillet and hidden lines (performance.md, "The third slice"; decisions 1727 to 1732). **Fourth slice done:** the list re-ranked from a new corpus pass; large meshes about a tenth quicker (a flat array of cells, no restarts in the insertion loop, storage kept between faces: the same triangles); max fillet about 20% less CPU and hard otherwise, hidden lines hard; 2D offsets share polygons (performance.md, "The fourth slice"; decisions 1754 to 1760). **Fifth slice done:** what threads keep, measured: 72 MB on 51 threads to 20 MB on 53 threads, nearly all the mesher's lists, now listed in state.md with limits that add up to about 760 kB a thread; `GaussLeastSquare`, polygon interference and `QuasiAngularConvertor` wasteful in bytes; Booleans, contexts and edge against free-form spread out or hard (performance.md, "The fifth slice"; decisions 1761 to 1767). More of the list remains |
| **11R** No success for an invalid result (decision 1742) | A failure may carry the invalid shape. Survey which operations can return one, measure what checking costs, then build it. | **The owner's decision.** For 0.3.0 if the figures allow |
| **11S** OCCT's defects (decision 1743) | Triage OCCT's open issues (224), its commits since 8.0.1, rows kept only for compatibility and the corpus's BAD cases, then fix from the top: silent wrong answers first. | **The owner's direction.** Ongoing, before and after 0.3.0. **First pass, 6 November (decisions 1775 to 1788):** triage complete in [occt-defects.md](occt-defects.md); ten silent wrong answers or invalid results fixed, among them Booleans of nearly parallel cylinders and boxes smaller than their shapes. 44 corpus scripts hold figures from one of those bugs and are pinned as intended differences. What's left is ranked there |
| **11M** Engine tidying | About 3,000 IDE1006 exemptions (3,002 measured; `Kerf.Modeling` has none now); the `Transient` base; 572 "touch" rows in the bug triage, fixed when their code is next changed. | Opportunistic: no unit of its own unless something needs it. The clean-up below took what was cheap |
| **Clean-up** (decision 1681) | The code, public and private: [clean-up.md](clean-up.md) is the survey, with counts, what was done and six questions for the owner. | **Done** (decisions 1695 to 1698, and 1744 to 1748 for the second pass); what's left is listed there |
| **11N** StepTidy (decision 1645) | Port OCCT's StepTidy (2,655 lines, with its GTests): after writing, it merges STEP entities that are equal, such as points, directions, lines and planes, and removes the duplicates. `CleanDuplicates` is already there for it, off by default as in OCCT. Then measure file size and writing time on the dataset's STEP files, and check that the files read back the same. | With STEP's header in 11L, so that STEP's fixtures change once. Whether it's on by default goes to the owner with the measurements. For 0.3.0 |

## 4. The owner's answers (decision 1644)

1. **The order:** as above, optimisation first, with the coverage work alongside it.
2. **The target:** as in section 2, pragmatically, with a time box for each operation.
3. **The coverage floor:** hand-written engine packages first. STEP is decided later.
4. **The remaining exceptions:** they stay.
5. **The next release:** 0.3.0, with 11L's fixes and the first optimisation results.
6. **The unported rest of OCCT:** StepTidy goes in (11N, decision 1645), and the rest is left for now. [unported.md](unported.md)
   says what's left and what it might be useful for.
7. **1.0:** needs a stable API, and more testing by the owner and other potential users. A Kerf GUI on Avalonia might help
   with that, but is still to be decided.

## 5. Order on the machine

Timing needs a quiet machine and the coverage run takes it over for hours, so they can't overlap. Allocation counts don't
depend on load, though, so 11B's profiling can run beside it.

1. 11A, with its baseline.
2. 11H, alongside the allocation work in 11B.
3. The rest, one timed unit at a time.
