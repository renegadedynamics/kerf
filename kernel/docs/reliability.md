# Reliability: no hangs, no crashes, no "sometimes it works"

*Unit 11T, 6 November 2026. The owner: "Some operations can sometimes hang the application for a few seconds, or even crash it - whereas other times they succeed. That's a particularly bad user experience, which I'd like to minimise with Kerf."*

This is about the worst case, not the typical one. **Everything here that could kill a process ran in a child process,** on a thread with a 1 MB stack, a heap limit of 2 GB and a time limit. Times are from a machine in use, with other work beside them and two processors for each run, so read them for their size. Decisions 1853 to 1858. The tests are `Kerf.Kernel.Reliability.Tests`, which run each dangerous case in a process of their own, and `Kerf.Modeling.Tests`' `ReliabilityTests`. The scripts were scratch: what they ran is listed here so it can be run again.

## The short answer

| The owner's words | What was found | Now |
|---|---|---|
| "crash it" | Stack overflows, which no `catch` can stop: a STEP file of nested assemblies or curves; about 3,000 moves of a shape; compounds or assemblies 300 levels deep. Running out of memory on a mesh at a tiny deflection, or a pattern of a billion | All of them are a failure with a message, at limits no real model meets. **None is left that we know of** |
| "hang … for a few seconds" | **The main cause is that a cell couldn't be stopped.** Nothing in `Kerf.Modeling` took a token, and fillets ignored theirs: a fillet went to its end, 12 s where it was going to fail, and its neighbours take 50 to 160 ms | `Kerf.Cancellation`, and checks in the long loops: 3 ms or so typically. **Bento needs one line** ([11t-bento.md](../../modeling/docs/11t-bento.md)). The 12 s themselves are still there, if nobody cancels |
| "other times they succeed" | **Not chance:** the same call gave the same bytes every time, serially, in parallel and in new processes (0 of 900 compared). It's nearly equal inputs: failure islands, ranked below | The list is the result. **Few are fixed**: they're in the geometry, inherited from OCCT |

## 1. What can take the process down

Measured on Kerf at `6b66dd4e7`, the depth at which each died, on a 1 MB stack, in steps of 300, 1,000, 3,000, 10,000:

| Input | Operation | Died at | Now |
|---|---|---|---|
| Compound nested n deep | `Transformed` with a copy, `Mirrored`, `Scaled` | 300 (5 kB a level) | Refused past 64 levels, at `Builder.Add` (decision 1854) |
| | `Fix` | 1,000 | |
| | `Validate`, `Fuse`, `Cut`, `Unify`, `ShowTopology` | 3,000 | |
| | `Transformed`, STEP and glTF writers | 10,000 | |
| | `AllFaces`, volume, bounding box, mesh, `Clean` | never: these walks are loops | |
| `ModelNode` or `Assembly` tree n deep | STEP, glTF and 3MF writers; `new Assembly` | 120 to 1,000 (8 kB a level) | Refused past 64 |
| **A file:** STEP assemblies 990 deep (1.6 MB) | `Step.Read` | died | `exchange.limit-exceeded`. An `InsufficientExecutionStackException`, at 450, had come straight out of `Read` |
| **A file:** STEP composite curves 3,000 and 30,000 deep | `Step.Read` | died | `exchange.limit-exceeded` (decision 1859) |
| A file: STEP trimmed curves 30,000; offset surfaces 30,000 | `Step.Read` | **read as a success with the entity missing** | `exchange.limit-exceeded` (decision 1859) |
| A file: STEP mapped items 30,000, parameter lists 200,000 | `Step.Read` | stopped cleanly already | |
| A file: BREP 990, glTF 300 and 3,000 | their readers | stopped cleanly already at 1,000 and 256; each now fails past 64 | |
| A chain of placements | `s = s.Moved(step)`, 3,000 times | 3,000 | A loop; folds past 1,024 items (decision 1853). 1,000,000 moves in 39 s |
| Curves or surfaces on curves or surfaces, 100,000 deep (`OffsetCurve3D` on itself…) | evaluation | never: they flatten, or stop | |
| `c = Compound.Create([c, c])` 40 times | `AllFaces`; mesh | out of 2 GB in 6 s; never returned | Refused past 100 million leaves (decision 1857) |

**Why a limit on depth and not a guard in each walk:** there are hundreds of recursions over a shape, in code ported from OCCT, and a new one gets written without one. Depth is made in one place. The guard `EnsureSufficientExecutionStack` stays where it was, and is the second line.

**Out of memory is an ordinary exception here,** which Bento shows in a cell, but it's a poor way to find out: other threads run out too, and the machine is thrashing by then.

| Request | Before | Now |
|---|---|---|
| Mesh of a sphere of radius 10, deflection 1e-3 | 301,560 triangles, 3 s, 180 MB | the same |
| 1e-4 | 3,018,318 triangles, 144 s, 3.1 GB | the same (cancellable) |
| 1e-5, 1e-6, 1e-7 | out of 2 GB in 1 s | `meshing.too-large` in 125 to 233 ms |
| 1e-9 and finer | already a failure (`engine.numeric-error`) | the same |
| Angular deflection on a sphere | 1e-3 out of 2 GB in 16 s; 1e-5 no result in 25 s | 1e-4 and 1e-5: `meshing.too-large` in 1 to 5 s |
| `PolarLocations` of 10⁸, 2 × 10⁹ | out of 2 GB in 7 s | `ArgumentException` in 13 ms |
| `GridLocations` or `HexLocations`, 50,000 × 50,000 | out of 2 GB in 1 to 5 s; the count overflowed an `int` | `ArgumentException` |
| `Box` or `Sphere` of NaN, infinity, 1e300, 1e-300 | a failure already | the same |
| A helix of a million turns in 1,000 mm | 0.9 s: it's one edge | the same |

**What else was read for this.** No finaliser, no `FailFast`, no `async void`. Each `stackalloc` has a fixed size, or one a B-spline's degree limits, or a check beside it. The one `new Thread` in `src` is `BVH.BuildThread`, where an exception would end the process, but it's only made when a BVH builder is given more than one thread and no caller in `src` does that. `unsafe` code and `Unsafe.*` are in about 27 files, for speed, with their bounds checked by the callers, which wasn't audited.

## 2. What can stall

**From the corpus's times** (`KERF_CORPUS_TIMES`, 17,872 cases, 8,435 s), slowest against the median of its kind:

| Group | Median | Slowest | Ratio | What it is |
|---|---|---|---:|---|
| bugs/modalg_5 | 433 ms | 59 s `bug25232_9` | 136 | a general fuse of 63 planes and a sphere: much work, not a stall |
| bugs/mesh | 155 ms | 16 s `bug29715` | 105 | a large STEP, at a fine deflection |
| bugs/modalg_7 | 437 ms | 45 s `bug28119_1` | 103 | a fuse of two tori and a fillet |
| bugs/modalg_1 | 250 ms | 25 s `buc60463` | 100 | a section, `FunctionSetRoot` over and over |
| bugs/modalg_8 | 226 ms | 21 s `bug32944` | 91 | a Boolean of **one box**: a particle swarm for each conic and surface |
| boolean/bsection | 264 ms | 19 s `P4` | 73 | section of complex shapes |

Those are large inputs. **What the owner describes is a small change of input making the time jump,** which the sweeps of section 4 timed step by step:

| Sweep | Median a step | Slowest | Ratio | Where |
|---|---:|---:|---:|---|
| Fillet on a pipe (a spline swept) | 46 ms | 12.5 s | 273 | radii 0.66 to 0.72, **and it fails** |
| Fillet on a loft (square to circle) | 164 ms | 13.9 s | 85 | radii 3.94 to 3.96, **and it fails** |
| Fillet on a spline's extrusion | 41 ms | 2.8 s | 68 | near the largest radius, **and it fails** |
| Fillet on a box | 6.6 ms | 3.4 s, 0.7 s | up to 520 | just above the largest radius (5.0026), **and it fails** |
| Fillet on a slot | 30 ms | 0.9 s | 31 | |
| Booleans of analytic shapes | 3 to 30 ms | under 0.4 s | under 40 | nothing like it |

**So the fillet that fails is the hang:** it tries harder than the one that works. In the loft's and the pipe's, all of the time is `ApproxAFunction` dividing the surface again and again, solving a system at each point (`AppFuncRoot.SearchPoint`, `FunctionSetRoot`), the code being OCCT's. It can't be bounded without changing what some real fillet gets, so it's a question for the owner. It can be stopped now, which is what matters in a spreadsheet.

**Other slow requests, all cancellable.** A polygon of 10,000 sides takes 4.4 s and of 100,000 over 45 s: building its wire is quadratic. A cylinder meshed at an angular deflection of 1e-5 took over 170 s without finishing: by the arithmetic, 630,000 points on each end circle, so long thin triangles. A fuse of 2,000 boxes, 21 s; 200,000 boxes in a flat compound, 22 s for its volume. **Loops without a bound:** OCCT's #1495 has one now (decision 1858). There are 112 `for (;;)` or `while (true)` in `src`; each was left as it is, unread but for that one. None showed up spinning in any run here.

## 3. Cancellation

**Who takes a token (before this unit).** The kernel's public operations do. **`Kerf.Modeling`'s don't,** except its imports and exports, and `Bento.Cad`'s `ModelFunctions` passes nothing. So a modelling cell could never stop: `reader.Cancellation` is checked between steps of a formula, and not during one. **This is the largest finding here.**

Time from cancelling to returning, cancelled at 2, 10, 30, 50, 70 and 90% of a run, with the worst of those. From a loaded machine, two processors. "Before" is at `6b66dd4e7`, through the kernel's own token, the only route there was:

| Operation | Length | Before, worst | Now, worst | Typical now |
|---|---:|---:|---:|---:|
| **Fillet,** every edge of a plate of 1,600 holes | 19 to 26 s | **23.9 s** | 39 ms | 2 ms |
| **A fillet that fails** (a loft at 3.94; `Fil` by `Cancellation.Use`) | 12.8 s | all of it | 2 ms | 1 ms |
| **Boolean, a swarm** (`bug32944`) | 4.2 to 6 s | **3.5 s** | 3 ms | 0 ms |
| Boolean, section (`buc60463`) | 2.6 to 5 s | 274 ms | 217 ms | 100 ms |
| Boolean, 1,620 faces (`bug26701`, `FuseAll`) | 2.5 to 3.4 s | 818 ms | 364 ms | 100 ms |
| Boolean, a plate of 625 holes | 1.9 s | 28 ms | not rerun | |
| `a - b` in `Kerf.Modeling` | 0.6 s | **all of it** | 5 ms | 3 ms |
| Offset, a plate of 900 holes | 1.3 to 2 s | 90 ms | 396 ms | 50 ms |
| Mesh, sphere at 1e-3 | 2.4 to 3.4 s | 68 ms | 131 ms | 15 ms |
| Mesh, `bug29715` at 0.002 | 8.4 s | not run | 699 ms | 90 ms |
| Mesh, cylinder at an angular deflection of 1e-3 | 12 s | not run | 8 ms | 5 ms |
| STEP read, `bug32239` (15 MB) | 3.9 to 4.3 s | 440 ms | 673 ms | 60 ms |
| STEP read, `tr9_r1001` (7 MB) | 1.1 s | 6 ms | not rerun | |
| STEP write, one solid of 2,500 to 6,400 holes | 5.3 to 20 s | **25 s** | 54 ms (2,500 holes) | 1 ms |
| STEP write, `bug24867_pump` | 0.7 to 0.8 s | 107 ms | 122 ms | 60 ms |
| BREP read, 9.6 MB | 0.2 s | 3 ms | not rerun | |
| Healing (`Fix`) | 0.7 to 0.9 s | 167 ms | 61 ms | 10 ms |
| Sewing | 0.3 s | not run | 6 ms | 3 ms |
| Validate | 0.2 s | not run | 12 ms | 7 ms |
| Hidden lines, exact | 1.6 s | not run | 4 ms | 1 ms |
| Hidden lines, polygonal | 0.9 s | not run | 141 ms | 2 ms |

**A result that's finished when the cancel arrives is returned,** which the tests expect.

## 4. "Sometimes it works": failure islands

**The same call always gives the same answer.** Ten models (cuts, fuses, fillets, chamfers, an offset, a loft, a sweep, 81 holes): volume, area, counts, every mesh vertex and triangle, and the STEP text, by SHA-256. Six times serially and twelve times on four threads, in each of five new processes: **one digest for each model, 0 flips in 900.** Island points gave the same in new processes with 1, 2 and 8 processors (379 steps, 0 differences). Nothing here depends on timing, or the hash codes drawn for shapes.

**It's nearly equal inputs.** 82 sweeps, 93,954 calls of `Kerf.Modeling`, stepping a radius by 0.1 to 1%, or a tool across a shape, or both by powers of ten down to 1e-13. A clean sweep has one flip, where the radius gets too large. Ordinary shapes mostly do: boxes, cylinders, cones, plates, an L, a boss, a tee of unequal cylinders, all by fillet and chamfer; offsets of those; cuts sliding across them, turning through seams and poles. **Ranked by what a user would meet:**

| # | Island | Rate | Result | Why, so far as known |
|---|---|---:|---|---|
| 1 | **Two cylinders of equal radius crossing,** offset by 1e-13 to 1e-5, as rounding or an imported file does. **The worst is a tool whose axis lies in the other's end plane,** a round notch in the end | **In that position: Cut 20, Common 20 of 51 steps within 1e-7 wrong, Fuse 16 failed and 2 wrong, so 38%;** none from 1e-7 to 5e-3 in the same sweep. A finer Cut sweep to 7e-3: 14 of 67. **Elsewhere (axis crossing at z = 0, 1, 2.5, 4 of a cylinder of 10):** 7 of 268 Cuts fail, between 1e-7 and 7e-6, none silently wrong beyond one of 0.1% | **Silently wrong in the notch:** at 1e-12 a Cut removes nothing and succeeds (785.4 for 452.1), Common is empty for 333.3, Fuse 2356.2 for 2022.9; at 1e-9 to 5e-8 a Fuse is invalid, which unit 11R reports. Elsewhere a failure | The section is right in every case, so it's the splitting and classifying. In the notch, the tool's sides also touch the end circle at two points, at the same offset (inferred from the geometry). `QuadQuadGeo` calls axes within 1e-7 crossing and gives ellipses, whose double points the faces then split at a seam. Same code as OCCT (by reading; not run on native) |
| 2 | **An equal tee:** a branch of the same radius through a cylinder, filleted where they meet | **514 of 2,508 fail, 285 flips**: nearly all below 0.1, then scattered up to 2.6 | Failure | Tangency at the crossing points of the saddle. A branch of another radius has none (0 of 2,508) |
| 3 | **Chamfer on a loft** (square to circle), all edges | **946 of 2,194, 228 flips** | Failure | With fillets below: a loft's B-spline faces |
| 4 | **Fillet on the same loft** | 39 of 2,194, 26 flips, around radii 0.063 to 0.11 | Failure: "Bad orientation of subshape" (30), walking failure (9) | |
| 5 | **Offset of the same loft** | 550 of 555, 10 flips | Failure: "Bad orientation of subshape" | Mostly fails, so it's closer to not working than an island; worth asking if the check or the loft is at fault |
| 6 | **Crossing cylinders of different radii, all edges filleted** | 401 of 401 | Failure | Not an island. Only the edges where they meet work, 441 of 441 (`fillet-tee-branch`) |
| 7 | A cylinder's seam, a sphere's pole: turned in 0.5° steps, 0.01° near the seam | 0 of 3,165 | | Clean |
| 8 | Near-tangent, near-coplanar, near-coaxial, by powers of ten to 1e-13: box on box, sphere in cylinder, torus on plane | 0 failures of 1,196; volumes follow | | Clean |

**Also found: the kernel's default volume is wrong on these shapes.** `IntegrateVolume()` with no options gives 139.90 for 118.73 on a Cut of those crossing cylinders 1.5e-5 off, 18% out, and the same at many other offsets (a walked intersection's faces), where `MassOptions { Tolerance = 1e-6 }` gives the right figure each time. This is decision 1802's finding at a larger size. `Kerf.Modeling` uses the precise path, so only the kernel's direct callers see it. **It looks like an island and isn't one in the shape,** so anyone measuring by the default will think the Boolean is at fault, as this unit first did. A question below.

**Also found, a smaller thing:** `Kerf.Modeling`'s `Volume` of a compound counts solids directly in it, and in compounds directly in it, as build123d's does, so deeper ones count nothing: a box in three nested compounds has a volume of 0. Not changed.

## 5. What's left

- **For unit 11Q:** writing a large solid to STEP spends about 85% of its time in shape processing, nearly all of it `SplitCommonVertexFixer` and the check. That fixer goes over all pairs of a face's wires and makes an `ExtendWireData` for each pair, so a face of 3,600 holes costs seconds, to find vertices shared between wires, which it could do with one map. From one trace of `write-plate 60`, 6.3 s.
- **Bento's line** in `ModelFunctions.Run`. Nothing above about cells helps until then.
- **Island 1** is the worst thing here: a wrong answer, reported as a success. A fix needs the Boolean's handling of a double point worked out, and it's not small.
- **Fillets that fail slowly,** if anyone wants them to fail quickly. A question below.
- **`unsafe` and `Unsafe.*`,** and the 112 unbounded loops, weren't audited.
- **An unknown stack overflow** in code not walked here can still end the process. The tests cover what could be found by building deep input, not every recursion.

**Tested on:** Reliability 35, Foundation 2,533, ModelingData 3,338, PublicApi 400, DataExchange 348, ModelingAlgorithms 5,712 (9 skipped), DrawLite 737, Kerf.Modeling 2,097, all passing; `KERF_CORPUS_FULL=1` on the fillet, Boolean, STEP, STEP-CAF, offset and hidden-line corpus classes, 23 tests passing in 18 minutes. The full corpus wasn't run.

## 6. Questions for the owner

1. **Is the ambient `Kerf.Cancellation` the right shape?** It breaks the rule against ambient state, as `OperationContext` and the overlay already do under decision 1386, because operators can't take a token and hundreds of overloads would be the alternative. **Recommended: keep.** It's public API and easily renamed before 0.3.0.
2. **Should a fillet give up after a time?** A failing fillet takes up to 14 s. A limit on the approximation's work, say ten times a usual fillet of that many edges, would end those early. It might fail some fillet that would have worked after 10 s, and which fillet would depend on the limit and not the machine if it's counted in work. **Recommended: not yet.** Cancellation answers the case in a spreadsheet. Ask again if it's still felt after Bento's line goes in.
3. **The limits:** 64 levels of nesting, 100 million leaves, 10 million triangles, 10 million locations. **Recommended: as they are.** The last two are the only ones that a real use could conceivably meet, and `MaxTriangles` is an option.
4. **Should the kernel's `IntegrateVolume()` default to the precise path too,** as `Kerf.Modeling` does (decision 1802)? It's silently 18% out in the case above. **Recommended: yes,** with a relative tolerance by default; about the same cost by that decision's figures. Not done here, as it changes results.
5. **Island 1 and 2 next?** They're the ones a user would meet, and they aren't small. **Recommended: yes, 1 first, after 0.3.0,** starting with native OCCT on the same inputs to see if it's inherited.
