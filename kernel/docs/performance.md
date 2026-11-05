# Performance: where OCCT.NET's time goes

*Unit 9E, 2 November 2026: the performance review of phase9-plan.md section 2.4 and item 2 of
beyond-compatibility.md. Only changes that keep native's results were made; the rest are proposals. The summary, and
everything after the next section, are unit 9E's figures. The current ones are [the 0.2.0 baseline](#the-020-baseline-unit-11a)
and the dated files in [benchmarks/](benchmarks/).*

## Summary (unit 9E's)

- **The "4.4× native" was mostly three things, none of them OCCT's algorithms:**
  - one accidental quadratic in the port (a tangent zone's insertions at the front of a .NET list), which alone
    made `MaxFillet` take 28 s against native's 0.8 s;
  - the garbage collector under parallel load: the suite allocates 17 to 20 GB of managed memory on OCCT.NET (native's
    wrappers allocate 0.5 GB; native OCCT's own heap is outside .NET's), and xUnit's 24 threads spend about a fifth
    of the run stopped for collections;
  - the JIT: a cold run compiles some 50,000 to 70,000 methods.
- **Fixed** (results unchanged): the quadratic, an eager 256-element allocation in every `NCollection_DynamicArray`,
  a queue drained from the front of a list, an array clone in the cell filter, and a temporary vector in Newton's
  minimisation.
- **After the fixes**, run one test at a time, build123d.NET's suite takes **1.9× native's time** (3.7× before), and
  half of what is left is JIT. Run in parallel as `dotnet test` does, it takes about 5× native's time with the
  workstation garbage collector and 2.5× with the server collector.
- **Bento's workload** (the guide's examples, built, meshed and measured): recalculation **1.5× native**, opening a
  workbook 3.5× native (the JIT).

## The 0.2.0 baseline (unit 11A)

*5 November 2026, the head of phase 11's rig, a14650837: the code of `v0.2.0` with only the benchmarks changed. Every table
is in [benchmarks/2026-11-05-a14650837.md](benchmarks/2026-11-05-a14650837.md).*

| | Native (frozen) | Kerf | Kerf / native | Target |
|---|---:|---:|---:|---:|
| Recalculation (median of passes 2 to 9, summed), ms | 685 | 1,211 | 1.77× | 822 (1.2×) |
| Opening (first pass, summed), ms | 921 | 1,764 | 1.92× | |
| Allocated per recalculation, MB | 3 | 450 | | 225 (half) |

Every example gives native's volume, face count and triangle count.

### Bento's examples

Native's per-example figures are unit 9E's run of the benchmark's native project (9 passes), kept in its working files and
never committed until now; `tools/bench.ps1` holds them. Native can't be measured again.

| Example | First pass, ms | Native | Recalculation, ms | Native | Ratio | Over 1.2× native, ms | Settled, ms | MB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 plate with hole | 141 | 103 | 8.2 | 5.4 | 1.52× | 1.7 | 3.4 | 5.6 |
| 3 prismatic solid | 31 | 16 | 5.5 | 4.7 | 1.16× | | 2.7 | 4.6 |
| 5 working point | 12 | 8.1 | 9.1 | 7.8 | 1.16× | | 4.7 | 8.1 |
| 6 point lists | 17 | 13 | 13.4 | 7.6 | 1.76× | 4.3 | 7.2 | 13.2 |
| 9 chamfer, fillet | 76 | 52 | 14.4 | 12.9 | 1.12× | | 9.5 | 17.0 |
| 10 select last, hole | 58 | 18 | 44.5 | 17.4 | 2.56× | 23.6 | 22.2 | 17.6 |
| 29 bottle | 296 | 143 | 273 | 106 | 2.58× | 146 | 156 | 139 |
| 31 nesting locations | 44 | 46 | 36.1 | 34.2 | 1.06× | | 21.8 | 38.1 |
| 32 for-loop | 21 | 21 | 19.0 | 17.5 | 1.08× | | 10.5 | 21.2 |
| 33 function, for-loop | 16 | 15 | 14.8 | 15.2 | 0.98× | | 8.0 | 16.4 |
| 36 extrude until | 1,054 | 486 | 774 | 457 | 1.69× | 226 | 725 | 169 |
| **Total** | **1,764** | **921** | **1,211** | **685** | **1.77×** | | **969** | **450** |

- **Two examples are 93% of the excess.** The sum over 1.2× is 402 ms, and "extrude until" and the bottle are 372 of it. Eight of the eleven are within 1.2×, or within a few milliseconds of it.
- **"Settled"** is the median of passes 11 to 20. Recalculation by native's definition (passes 2 to 9) still catches
  the runtime tiering up, so the same work takes 969 ms once it has finished: 1.41× native. Native's figure is the
  one the target is set against, so the target stays on passes 2 to 9.
- **ReadyToRun costs recalculation what it gives opening.** Without it, in the same rig (`-NoReadyToRun`): opening 2,619 ms
  (1,764 with) with 1,254 ms in the JIT (430 with) and 14,251 methods compiled (4,738 with), but recalculation **1,041 ms**
  (1,211 with) and settled 912 (969 with). Precompiled code is slower than tier 1 until the runtime replaces it. Unit 11C's
  to weigh, with tiered settings: 170 ms may be the cheapest on the table.
- **Allocation is 450 MB, not the 311 MB the old counter would show.** The benchmark read the calling thread's bytes, so
  it missed what the parallel mesher allocates on other threads. It now reads all threads'. Unit 9E's 1,025 MB and
  unit 10J's 142 MB for "extrude until" are the old counter's (169 MB on the new).

### The rest of the set

| Suite | Time | Allocated |
|---|---:|---:|
| Booleans, 407 simple cases, serial (sum of medians) | 2,168 ms: pave filler 1,403, builder 789 | 2,811 MB |
| Booleans, 193 complex cases, serial | 2,251 ms: pave filler 1,474, builder 429 | 2,077 MB |
| Booleans' speed-up with their own parallel option | 1.32× simple, 1.47× complex | |
| The clean after each Boolean, simple / complex | 184 ms / 80 ms | 362 / 124 MB |
| Offset, arc join / intersection, 16 parts | 118 / 76 ms | 90 / 98 MB |
| Thick solid | 117 ms | 104 MB |
| Fillet / chamfer every edge | 98 / 86 ms | 181 / 171 MB |
| Draft the sides / boss | 5.9 / 39 ms | 7 / 39 MB |
| STEP parser, 340 files, 125 MB | 857 ms (146 MB/s) | |
| STEP import with healing, 289 files up to 400 kB | 6,529 ms | 6,215 MB |
| ShapeFixer / UnifySameDomain on those | 3,936 / 801 ms | 5,159 / 1,273 MB |
| STEP export of those | 2,234 ms | 3,039 MB |
| Meshing seven `data/occ` shapes, serial | 39 to 272 ms each | 28 to 257 MB each |
| Construction | 4 µs (a wire) to 2.5 ms (a projection) | 7 KB to 2.3 MB |
| Images: SVG / PNG, per example | 1 to 48 ms / 42 to 174 ms | 0.4 to 14 / 22 to 50 MB |
| **Parallel:** 24 copies of the examples on 24 logical processors | a round 5.2 s, **5.5× speed-up** | 10.8 GB a round, 10 s of the collector's pauses |

### The ranking, for the optimisation units

"Longer-running" comes from a sampling profile of the recalculation (`tools/profile-report.cs`, passes 2 to 20, 12,040 samples
on any thread, so shares of thread time, not wall time; each sample goes to the outermost operation on its stack).
"More commonly used" has **no telemetry behind it**. The proxy is how often the guide
(`docs/cad/build123d-in-bento.md` in Bento) mentions each family of `CAD_` functions, which measures what the guide chooses to
show, not what users do. Mesh and volume run on every recalculation whatever the guide says.

| Rank | Operation | Thread time | Without "until" | Allocation | In the guide | Why here |
|---:|---|---:|---:|---:|---:|---|
| 1 | **Meshing and the mesh's tolerance** | 15.0% + 12.7% | 30.8% + 23.3% | 29.2% + 8.3% | every recalculation | Runs on every shape. The second figure is `MeshTolerances.For` asking for an *optimal* bounding box to set the deflection: extrema on every face, for one number. It's the likeliest cheap win; a cheaper box has to give the same deflection, or the meshes change |
| 2 | **Curve extrema in "extrude until"** (`DistanceShapeShape`, `GlobOptMin`, `NewtonMinimum`) | 48.6% | | 17.6% | `CAD_PT_EXTRUDE` 11, few with `until` | The longest by far, and 226 ms over target alone. It's one example, and slow on native too (457 ms). 11E may change the algorithm |
| 3 | **Booleans**: pave filler, builder, clean | 8.1% + 2.2% + 1.2% | 14.0% + 4.5% + 2.4% | 16.9% + 2.3% + 4.8% | `CAD_CUT`, `CAD_FUSE`, `CAD_INTERSECT` 21 | The most used modelling operation. Each is quick, and there are many |
| – | **The collector** (threads stopped, across all of the above) | 16.5% | 31.3% | | | Not an operation: 11B's measure. Doubles, `NCollection_Array1<double>`, `VectorBase` and `NCollection_Array2` are 29% of the bytes, which supports proposal 5 as the first step |
| 4 | **Construction**: extrude, revolve, faces | 1.5% | 3.0% | 6.8% | `CAD_PT_EXTRUDE` and others 14 | Cheap in time, worth a look for allocation |
| 5 | **Fillets and chamfers** | 0.6% | 1.3% | 4.1% | 10 | Low in the examples. On every edge of a part they're the features' largest allocators |
| 6 | **Offsets, thick solids** | 1.3% | 2.8% | 3.8% | 8 | The bottle only |
| 7 | **Healing** inside other operations; **volume** | 1.6%; 1.3% | 3.3%; 2.7% | 3.4%; 1.5% | every recalculation (volume) | Small |
| 8 | **STEP import and export** | not in a recalculation | | 6.2 GB and 3.0 GB on the set | 9 | Long, and paid once per file on opening or saving. Worth 11C's attention, not 11B's first |
| 9 | **Images** | not in a recalculation | | | not counted | 42 to 174 ms each |

The bottle has about 4% of the samples in its build, against 22% of the recalculation's time. That suggests its excess is
mostly meshing its 9,556 triangles and finding its box, not its fillets and offset, though the two shares aren't quite
like for like (thread time over all passes, wall time over passes 2 to 9).

The order for the units: 11B's allocation work starts with `NewtonMinimum`'s arrays and the mesher's cell filter, the two
largest named sites; 11E starts with the mesh's tolerance box and then "extrude until".

### Noise, and how to run it

Run `kernel/tools/bench.ps1`, on Windows PowerShell 5.1 or later. It takes about 15 minutes (`-Quick`: 5), and writes
`docs/benchmarks/<date>-<commit>.md` and `.tsv`. For a unit's before-and-after, on each commit:

```powershell
kernel/tools/bench.ps1 -Quick -Compare kernel/docs/benchmarks/2026-11-05-a14650837.tsv
```

It exits with 1 on a regression and 2 on a busy machine; `-Result <tsv>` compares without running. Each row is the median
of its suite's fresh processes (Bento 5, most others 3, the long ones 2, the lower of two).

Four full runs and a quick one, as two pairs, the second after the noisiest suites were given more runs. The busy threshold is 10% total CPU. This machine idled at 2%, and at 6 to 7% through all of these, because of a stray
script spinning on one thread; one busy thread is about 4%, a build far more. The rig refused twice while an earlier run of
its own was still going.

| Kind of row | Largest change between runs | Threshold |
|---|---:|---:|
| Allocation | 1.3% (0.0% median) | 2%, and 0.1 of its unit |
| Bento's total recalculation / first pass | 2.4% / 0.7% | 5% |
| Bento's settled total | 8.8% | 15% |
| Other totals: Booleans, STEP, the parallel round, the clean | 10.2% | 15% |
| Parallel speed-up | 4.9% | 15% |
| One operation's time (median 2.3%, 90th percentile 11%) | 60% (5 passes of PNG; 51.5% after) | 60%, and 10 ms or 20 µs |
| A result: a volume, a face count, failed runs | none | any change |

So allocation and the totals are a real check, and one short operation's time only catches something large: single rows
are bimodal, depending on when the runtime tiers up. Don't read a 20% move on one row as a finding without running it again.

## Unit 11B: allocation

*5 November 2026, on a busy machine: a four-hour coverage run was going alongside, so this unit reads only allocation and
results, and claims nothing about time. The timed run on a quiet machine comes after merging.*

**Allocation per recalculation of Bento's examples: 450 MB to 217 MB, against the target of 225 MB** (`tools/bench.ps1 -Quick
-Force`, compared with 11A's baseline). Every volume, face count and triangle count is unchanged, and every test suite passes.
The rig's run isn't in `docs/benchmarks`, because it was taken on a busy machine. No site needed a change to the public API.

### Each site

Each row is one commit, so that any one can be dropped. The figures are the benchmark's total at each commit, in order, so a
row's saving depends a little on the rows above it. These builds weren't ReadyToRun; the last agrees with the rig's within 1 MB.

| Site | What changed | Total after, MB | Saved, MB |
|---|---|---:|---:|
| `math_GlobOptMin`, `math_NewtonMinimum` | One `NewtonMinimum` for each `GlobOptMin`, reset for each local search (it was new each time); its iterates and `GlobOptMin`'s small vectors kept | 371.9 | 78.1 |
| `NCollection_CellFilter` | A cell's list is in one array, linked by position; keys need no array up to three dimensions; no `Cell` objects for a range | 342.1 | 29.8 |
| `BRep_Tool::CurveOnPlane` | A line in the plane is worked out directly, with the projection's arithmetic | 310.1 | 32.0 |
| `math_VectorBase` | Holds its values itself, not through an `NCollection_Array1` object | 296.0 | 14.1 |
| Optimal box, `MathOpt::PSO` | One swarm for each surface's box, reset for each optimisation | 274.7 | 21.3 |
| `LU_Decompose` | No `ProgressScope` when nothing is watching | 275.6 | none |
| Delaunay mesher | The map of free edges kept between vertices, with its nodes; iterators and the frontier search kept; `CreateTriangles`' lists made only when needed | 258.3 | 17.3 |
| The same map | Also keeps the buckets it grew through | 247.0 | 11.3 |
| `Approx_SameParameter` | Borrows its four 8 kB arrays from the pool | 241.6 | 5.4 |
| `TopoDS_Iterator` | Holds a list and a position, not a list iterator | 237.1 | 4.5 |
| `math_FunctionSetRoot`, `math_SVD` | Its Gauss, its SVD and their work vectors kept between iterations | 217.6 | 19.5 |

The first two are the sites 11A's ranking named first. `CurveOnPlane` wasn't on its list: the profile found it, at 12% of the
bytes, under `Tool.CurveOnSurface` for every edge on a plane without a stored pcurve.

### Bento's examples, and the rest

| Example | MB, baseline | MB, now | |
|---|---:|---:|---:|
| 2 plate with hole | 5.6 | 4.3 | −24% |
| 3 prismatic solid | 4.6 | 3.2 | −31% |
| 5 working point | 8.1 | 5.6 | −31% |
| 6 point lists | 13.2 | 9.0 | −32% |
| 9 chamfer, fillet | 17.0 | 12.9 | −24% |
| 10 select last, hole | 17.6 | 9.0 | −49% |
| 29 bottle | 139 | 83.8 | −40% |
| 31 nesting locations | 38.1 | 23.1 | −39% |
| 32 for-loop | 21.2 | 12.4 | −41% |
| 33 function, for-loop | 16.4 | 9.5 | −42% |
| 36 extrude until | 169 | 44.0 | −74% |
| **Total** | **450** | **217** | **−52%** |

Elsewhere in the rig's quick set: a parallel round 10.8 GB to 5.2 GB; meshing the seven `data/occ` shapes about half (Motor-c 230 to
116 MB, Axis_of_bearing 257 to 109 MB); fillet every edge 181 to 102 MB, chamfer 171 to 107 MB; offsets 90 and 98 to 62 and 58 MB;
Booleans, 407 simple cases, 2,811 to 2,554 MB, with the clean 362 to 296 MB. The quick set leaves out the complex Booleans and
STEP's import and export. No allocation row went up.

In ten passes of the benchmark, collections went from 284/83/29 (generations 0/1/2) to 152/70/26. Whether that gives the time the
collector took back is for the timed run.

### Reading the rig's run

It exits with 1 on this run: 78 regressions, all of them times, which are meaningless on a busy machine. One row needed
checking: **methods the JIT compiled at opening went from 4,738 to 8,079**, which isn't a result but could have been this unit's
doing. It isn't: four alternating runs under the same load gave 8,080 to 9,451 for `main` and 7,999 to 9,528 for this unit. A
loaded machine lets more methods tier up during the first pass. The quiet run should show the baseline's figure again.

### What didn't work, and what's left

- **`LU_Decompose`'s `ProgressScope` saved nothing measurable,** though the profile gave it 46 MB over nine passes. Why isn't
  known; the commit is harmless, and can be dropped.
- **Proposal 5 (small vectors on the stack) wasn't taken.** Keeping the work objects removed most of what it was for, without the
  API change across `math`. Vectors are now 1.7% of the bytes (`VectorBase`) and doubles 7.2%, from 3.9% and 13.7%.
- **The adaptors' boxed elementary curves and surfaces** (`Line3D`, `Plane3D` and so on in `_curveData`, about 1%). Reading them
  from the curve instead would see a curve changed after loading, which the copy doesn't, so they stay.
- **`Matrix` has three objects over its array** (`DoubleTab`, `NCollection_Array2`). After the above that's little; not done.
- **Left, each under about 2% of what remains, for a later look:**
  - `CurveOnPlane` for circles, which still takes the whole projection;
  - `LinkedListOfInteger`'s nodes in the mesher, about 8 MB a recalculation. A removed node has to keep its link, so they can't
    simply be reused;
  - the growth of the mesher's `Triangle[]`, `MeshCircle[]` and cell list arrays;
  - `Explorer`'s iterators and `Placement.Inverted`, from `ReShapeBuilder`;
  - `MeshTolerances.For`'s optimal box, which is 11E's: finding it less often is an algorithm change.

To profile again, take a trace as `tools/profile-report.cs` says and read it with `tools/alloc-report.cs`, which sorts the bytes
by type and by the first frame that isn't a collection.

## How unit 9E measured it

- **Machine:** 24 logical cores, Windows 11, .NET 10, Release builds. Other helpers were running tests on the same
  machine, so timings vary by up to 30% from run to run; the figures below are the quieter runs, and each comparison
  was run back to back.
- **build123d.NET's suite** (1,851 tests on native, 1,876 on OCCT.NET), on both backends from the same commit:
  - through `dotnet test`, as before;
  - and in process, through xUnit's runner (a scratch program), so that it could be traced and its garbage
    collector, JIT and allocation counters read: in parallel as `dotnet test` runs it, and serially, one test at a
    time, which takes contention out.
- **Profiles:** EventPipe traces (CPU sampling every millisecond, exceptions, allocation ticks every 100 KB, the
  JIT's and the collector's events), read with TraceEvent.
- **Bento's workload:** `benchmarks/OcctNet.Bento.Benchmarks` (new). Bento's 3D models extension can't be run here,
  so the benchmark takes eleven examples from Bento's guide to its CAD functions (`docs/cad/build123d-in-bento.md`,
  section 9: examples 2, 3, 5, 6, 9, 10, 29, 31, 32, 33 and 36, written there as `CAD_` formulas) through build123d.NET's
  algebra API, as the `CAD_` functions call it. Each is built, triangulated with build123d's tolerances as Bento's viewer
  does, and its volume and face count read. The first pass is a workbook being opened; the median of the others is a
  recalculation. `Native/` runs the same program on native OCCT.

## Unit 9E's measurements

### build123d.NET's suite

| Run | Native | OCCT.NET before | OCCT.NET after |
|---|---:|---:|---:|
| `dotnet test`, wall | 2 s | 31–37 s | – |
| In process, parallel, wall | 3.4–3.9 s | 43–46 s | 18–21 s |
| In process, parallel, server GC | – | 56 s (one test, `MaxFillet`, sets the wall) | 8.5–10.7 s |
| In process, serial | 16.0 s | 60.0 s (3.7×) | 32.4 s (1.9×) |
| Allocated | 0.55 GB | 20.5 GB | 17.5 GB |
| GC pause, parallel | 0.2 s | 8–10 s | 8–11 s |
| JIT time (thread time, serial) | 3.4 s | 17.5 s | 15–21 s |

- The tests both backends run, serially: native 15.6 s, OCCT.NET 56.9 s before and 29.7 s after.
- `MaxFillet`: native 0.8 s, before 28.3 s, after 2.8 s.
- The slowest tests after the fixes, serially (native, OCCT.NET): `TheEstimateGrowsWithDetail` 1.6 s, 4.1 s;
  `ABigMeshComesOutQuickly` 1.6 s, 3.7 s; `MutatedBRepReadsOrFailsCleanly` 2.0 s, 2.9 s; `MaxFillet` 0.8 s, 2.8 s;
  `OperationsOnSharedInputsRunConcurrently` 0.6 s, 2.0 s. Meshing and BRep reading lead.
- One test fails on OCCT.NET before and after, the same way: `Fillet2DTests.FilletRecord` (the 2D fillet's history
  counts a vertex as modified twice). It is in build123d.NET's history mapping or the port's ChFi2d and was left alone.

### Bento's workload

| | Native | OCCT.NET before | OCCT.NET after | After, ReadyToRun |
|---|---:|---:|---:|---:|
| Opening (first pass), ms | 921 | 3,498 | 3,224 | 2,034 |
| Recalculation (median), ms | 685 | 1,170 | 1,051 | 1,162 |
| of which example 36 (extrude until) | 457 | 821 | 693 | 757 |
| Allocated per recalculation, MB | 3 | 1,180 | 1,026 | 1,025 |

Every example gives native's volume, face count and triangle count, before and after.

- Example 36 is slow on native too: build123d's "extrude until" runs a global curve-to-curve extremum
  (`math_GlobOptMin` with `math_NewtonMinimum`), which allocates 800 MB on OCCT.NET.
- Without it the other ten examples recalculate in 358 ms against native's 228 ms (1.6×).

## Where the time goes

CPU samples of build123d.NET's suite in parallel, before and after the fixes (exclusive time, the share of all samples
in tests' code; a sample is a thread running managed code):

| Hot spot | Package | Kind | Before | After |
|---|---|---|---:|---:|
| `Intf_TangentZone.Append`/`InsertBefore` copying | Intf (BRepCheck's wire self-intersection) | accidental O(n²): insertions at the front of a `List<T>` | 16.7% | 0 |
| Threads stopped for a collection (`PollGC`) | – | allocation, under parallel load | 12.2% | 14.4% |
| `NCollection_List.Remove(Iterator)` in `createTrianglesOnNewVertices` | BRepMesh | O(n) removal from the middle of a `List<T>` (O(1) in OCCT's linked list) | 2.6% | 7.6% |
| `NCollection_CellFilter.Cell` cloning its index array | NCollection (BRepMesh) | `Array.Clone` | 4.7% | <0.5% |
| `BRepExtrema_CheckPair[]` and other 256-element arrays | NCollection_DynamicArray | eager allocation | 2.1% (17.5% of bytes) | 0 |
| `RemoveFirst` on the Delaunay's free links | BRepMesh | O(n) dequeue | 1.7% | 0 |
| `BSplCLib.Bohm`, `Eval`, B-spline evaluation | BSplCLib | the algorithm | 2.4% | 3.7% |
| `BRepMesh_Delaun`, `CSLib_Class2d`, cell filter | BRepMesh | the algorithm | ~8% | ~10% |
| Identity hash codes for shape maps | TopoDS | `RuntimeHelpers.GetHashCode` | 1.9% | 2.1% |

Percentages after are of a smaller total, so a hot spot left alone grows. In Bento's workload, recalculating:

| Hot spot | Kind | Share |
|---|---|---:|
| Threads stopped for a collection | allocation | 11.5% |
| `math_NewtonMinimum.Perform`: `math_Jacobi`, `math_Gauss`, matrices and vectors per iteration (60% of the bytes) | allocation; OCCT allocates the same on its heap | 5.2% and the collector's share |
| B-spline evaluation (`Bohm`, `Eval`, `GeomAdaptor_Curve.EvalD0`) | the algorithm | 16% |
| `NCollection_List.Remove(Iterator)` in the Delaunay mesher | O(n) removal | 7% |
| `math_PSO`, `Extrema_GlobOptFuncCC`, `math_Recipes` | the algorithm | 10% |

**Not significant:**
- **Exceptions.** The whole suite throws about 2,400 (most of them build123d.NET's own `InvalidOperationException` in
  `Measures`, `Storage_StreamFormatError` from the malformed-BRep tests, and ChFi3d's `Standard_Failure` while
  searching for a fillet's maximum radius). At tens of microseconds each, that is well under a second. Bento's workload
  throws none.
  Unit 10F counted them again (decision 1368) and removed the one frequent pair it found, ShapeProcess catching a
  missing parameter's exception (32,869 of the ModelingAlgorithms tests' 38,602).
- **Copy at entry, `Handle` wrappers, locks:** none showed in the samples above 0.5%.

**The JIT** doesn't show in the samples (it runs in native code): a serial run spends 15 to 21 s of thread time
compiling, against native's 3.4 s for its wrappers. ReadyToRun (compiled ahead of time) cut the opening of Bento's
workbook by 37% (3,224 to 2,034 ms).

## What was fixed

Each keeps native's results: the same values, iteration order and tolerances. Decision 1313.

1. **`Intf_TangentZone`'s points** are kept in a deque (two lists, the front one reversed) in place of a
   `NCollection_Sequence` over a .NET list. `PolygonInsert` puts points at the front, which OCCT's linked sequence does
   for nothing and a .NET list does by moving every point; merging the zones of two long coincident polygons, as
   BRepCheck's wire self-intersection does on a filleted solid, was quadratic. The points' order is OCCT's.
2. **`NCollection_DynamicArray`** takes its first capacity (OCCT's increment, at most 256) when the first element is
   added, as OCCT allocates its first block, rather than when it is constructed. Most arrays stay empty (one per thread
   band in `BRepExtrema_DistShapeShape`, one per intersection in `IntCurveSurface_Inter`, `BOPDS_Iterator`'s, ...).
   Elements still move only when the array grows past that capacity, so a `ref` taken from `Append` stays valid as
   long as it did before.
3. **`BRepMesh_DataStructureOfDelaun`'s deleted links** are a `Queue<int>`: OCCT uses its list only as a first-in,
   first-out queue, and `RemoveFirst` on a .NET list moved the rest.
4. **`NCollection_CellFilter.Cell`** copies its indexes with `ToArray` rather than `Array.Clone`.
5. **`math_NewtonMinimum.Perform`** computes `suivant = precedent - TheStep` in place (`Subtract`) rather than through a
   temporary vector: the same subtraction, element by element.

**Checked:** OcctNet.Foundation.Tests (2,206), the ModelingAlgorithms tests of BRepMesh, Intf, BRepCheck, IntCurve,
BRepExtrema, MeshExtraction, Geom2dAPI and the hatchers (400), the ModelingData tests of Extrema, math, GeomLib and
Approx (611), the oracle tests of BRepMesh, BRepCheck, Extrema, MeshExtraction, Geom2dAPI, IntTools, BRepAlgoAPI,
BOPAlgo and build123d.NET's Booleans and phase 7 operations (67, among them the captures of thousands of calls), the
corpus classes DrawCheapGaps, DrawFillet2d, DrawIntersect and DrawMeshingCommands (13), and build123d.NET's suite on
OCCT.NET (1,875 of 1,876, `FilletRecord` failing as before). All green. DrawCheapGaps failed once while run beside the
others under heavy machine load (its cases have time and memory limits) and passed alone.

## Proposals

Ranked by expected gain against cost. None changes results unless it says so.

| # | Proposal | Expected gain | Cost | Risk |
|---|---|---|---|---|
| 1 | **Server GC (or a larger gen-0 budget) in hosts that compute in parallel.** A setting in Bento's and build123d.NET's test projects (`<ServerGarbageCollector>true</ServerGarbageCollector>`, or `<ConcurrentGarbageCollection>` with `GCgen0size`). Measured: the suite in parallel from 21 s to 10 s; a 64 MB gen-0 budget alone gave 16 s. | 2× on parallel workloads | A line per host | Memory: server GC keeps larger heaps; check Bento's working set |
| 2 | **ReadyToRun for the four OcctNet packages** (crossgen when packing, or `PublishReadyToRun` in Bento's publish). Measured: opening Bento's workbook 37% faster. A composite or "input bubble" build across the four assemblies would precompile the generic collections each uses from Foundation, which plain ReadyToRun leaves to the JIT. | 30–50% off the first evaluation of each function | Small: build settings; packages grow about 3× | None to results; R2R code is rejitted when hot |
| 3 | **`NCollection_List` as a linked list**, as OCCT's is: O(1) `Remove(Iterator)`, `RemoveFirst`, `Prepend` and `InsertBefore`. Removes the mesher's hot spot (7% of the suite, 7% of Bento's recalculation) and the other `RemoveFirst` queues. Its enumerator type is public (`List<T>.Enumerator`), so it changes the API. | 5–10% | Moderate: one class, but used everywhere; run the whole oracle and corpus | Low for results (the order is the same), moderate for API users |
| 4 | **Reuse `math_NewtonMinimum`'s work arrays** (`math_Jacobi`, `math_Gauss`, the Hessian's copies) across its iterations and calls, with re-entrant `Perform` methods on those classes. 60% of the bytes Bento's recalculation allocates, so most of its collections. | 10–20% on extrema-heavy operations (extrude until, distances) | Moderate: changes to ported classes' shape, kept beside OCCT's API | Low; the arithmetic is unchanged |
| 5 | **Small vectors and matrices on the stack** (`math_Vector` of up to 32 elements, as OCCT's inline buffer): the many `math_VectorBase`/`NCollection_Array1<double>` allocations in `math_Recipes`, `LU_Decompose`, `AdvApprox`, `Approx_SameParameter`. | 5–10% of allocation | Large: an API change across math | Low |
| 6 | **`NCollection_Sequence` with a gap at the front** (a deque), for the remaining `Prepend`s (`IntTools_FClass2d.Init`, 0.4%). | Small | Small | Low |
| 7 | **A cheaper identity hash for `TopoDS_TShape`** (an id assigned at creation in place of `RuntimeHelpers.GetHashCode`). It must not change iteration order: OCCT.NET's maps already iterate by insertion where OCCT's would by address, so this needs checking against decision points on canonical order first. | 2% | Small | Must keep map iteration order; check |

Anything beyond these (iteration order, different algorithms) waits for beyond-compatibility.md's decision point B.

**Unit 10J** (decisions 1400 to 1409) took proposals 3, 4 and 7, and two hot spots of the Booleans' profile:
- **3:** the mesher's circle list no longer moves on removal (a marker in a `List<int>`, not a linked list).
- **4:** `math_NewtonMinimum` keeps its `math_Jacobi` and `math_Gauss`: "extrude until" allocates 142 MB a
  recalculation, not 802 MB, and a parallel round of Bento's examples 11.8 GB, not 27.7 GB.
- **7:** a TShape's hash is drawn once at creation (4.5% of the Booleans' samples were `RuntimeHelpers.GetHashCode`'s
  slow path).
- `Extrema_ExtElC`'s rectangular arrays (1.7%) are inline arrays.
- Proposals 5 and 6, and the rest of the collections, are in collections-plan.md.

## Running the measurements again

- Everything Bento-shaped: `kernel/tools/bench.ps1`, above. Native's project went in unit 10B.
- The modelling layer's suite: `dotnet test modeling/tests/Kerf.Modeling.Tests`; `--logger trx` gives each test's time.
- A trace: run any of these with `DOTNET_EnableEventPipe=1`, `DOTNET_EventPipeOutputPath=<file>.nettrace` and
  `DOTNET_EventPipeConfig=Microsoft-DotNETCore-SampleProfiler:0:5,Microsoft-Windows-DotNETRuntime:4c14fccbd:5`
  (in a process that exits normally: vstest kills its test host before the trace is written), and read it with
  `tools/profile-report.cs`, PerfView or TraceEvent.
