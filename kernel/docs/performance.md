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
kernel/tools/bench.ps1 -Quick -Compare kernel/docs/benchmarks/2026-11-05-fdaa7a7f6.tsv
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

## Where it stands (the timed run after 11E)

*[benchmarks/2026-11-05-fdaa7a7f6.md](benchmarks/2026-11-05-fdaa7a7f6.md), on a quiet machine, 5 November. Compare with this run from here on.*

| | Native (frozen) | 0.2.0 | Now | Now / native |
|---|---:|---:|---:|---:|
| Recalculation, ms | 685 | 1,211 | **363** | 0.53× |
| Settled recalculation, ms | | 969 | 354 | |
| Opening, ms | 921 | 1,764 | **709** | 0.77× |
| Allocated per recalculation, MB | 3 | 450 | 208 | |
| A parallel round, 24 copies on 24 processors, ms | | 5,238 | 2,763 | |

- **The targets are met:** recalculation was to be within 1.2× of native, and it's nearly twice as fast. Eight of the eleven examples are ahead of or within 1.2× of native. "Point lists" (1.82×, 6 ms over) and "select last" (1.49×, 5 ms over) are small in absolute terms.
- **Native stops being the yardstick from here** (decision 1681): the work moves to what's slow, unit 11Q.
- **Since this table:** unit 11C took opening to 509 ms and recalculation to 269 ms; [its run](benchmarks/2026-11-06-37d3afa6b.md) is the one to compare with now. See below.
- **The parallel speed-up fell from 8.33× to 3.11×, and that's the next thing to look at.** The round itself got quicker, so nothing regressed: the serial work fell by two-thirds while allocation barely moved (4.9 GB a round), which leaves the collector dominant, at 4.9 s of pauses. The rig flags this and the pauses as regressions against the run after 11B. That's unit 11D's, which found the figure mostly misleading: see below.

## Unit 11B: allocation

*5 November 2026, on a busy machine: a four-hour coverage run was going alongside, so this unit reads only allocation and
results, and claims nothing about time. The timed run on a quiet machine comes after merging.*

**Allocation per recalculation of Bento's examples: 450 MB to 217 MB, against the target of 225 MB** (`tools/bench.ps1 -Quick
-Force`, compared with 11A's baseline). Every volume, face count and triangle count is unchanged, and every test suite passes.
The rig's run isn't in `docs/benchmarks`, because it was taken on a busy machine. No site needed a change to the public API.

### The timed run

*[benchmarks/2026-11-05-0198121ae.md](benchmarks/2026-11-05-0198121ae.md), on a quiet machine, after 11B, the healing fix, 11L and
11N were merged. The rig's only regression is the healing suite's failed runs going from 12 to 0, which is the fix. Every other result
matches, and the full corpus, against `v0.2.0`, differs only in four memory-trend cases.*

| | 11A's baseline | Now | Change | Target |
|---|---:|---:|---:|---:|
| Recalculation, ms (native 685) | 1,211 | 1,095 (1.60×) | −9.6% | 822 |
| Settled recalculation, ms | 969 | 804 | −17.0% | |
| Opening, ms (native 921) | 1,764 | 1,738 (1.89×) | −1.5% | |
| Allocated per recalculation, MB | 450 | 217 | −51.8% | 225 |
| Booleans, 407 simple cases, serial, ms | 2,168 | 1,746 | −19.5% | |
| Booleans, 193 complex cases, serial, ms | 2,251 | 2,049 | −9.0% | |
| STEP import with healing, 289 files, ms | 6,529 | 5,854 | −10.3% | |

- **What's left is two examples:** "extrude until" is 736 ms against its 1.2× of 548, and the bottle 229 ms against 127. Together that's 290 of the
  total's excess, and the other nine are within 22 ms of theirs in all.
- **Settled recalculation is now under the target's figure:** 804 ms against 822. So once the runtime has finished tiering up, Kerf is at 1.17× native,
  and part of what's left is how quickly it gets there. That makes the tiering settings in 11C worth trying early.

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

- **`LU_Decompose`'s `ProgressScope` saved nothing measurable, so it was reverted,** though the profile gave it 46 MB over nine passes. Why isn't
  known; the commit is harmless, and can be dropped.
- **Proposal 5 (small vectors on the stack) wasn't taken.** Keeping the work objects removed most of what it was for, without the
  API change across `math`. Vectors are now 1.7% of the bytes (`VectorBase`) and doubles 7.2%, from 3.9% and 13.7%.
- **The adaptors' boxed elementary curves and surfaces** (`Line3D`, `Plane3D` and so on in `_curveData`, about 1%). Reading them
  from the curve instead would see a curve changed after loading, which the copy doesn't, so they stay.
- **`Matrix` has three objects over its array** (`DoubleTab`, `NCollection_Array2`). After the above that's little; not done.
- **Left, each about 4% of what remains or less, for a later look:**
  - `CurveOnPlane` for circles, which still takes the whole projection;
  - `LinkedListOfInteger`'s nodes in the mesher, about 8 MB a recalculation. A removed node has to keep its link, so they can't
    simply be reused;
  - the growth of the mesher's `Triangle[]`, `MeshCircle[]` and cell list arrays;
  - `Explorer`'s iterators and `Placement.Inverted`, from `ReShapeBuilder`;
  - `MeshTolerances.For`'s optimal box, which is 11E's: finding it less often is an algorithm change.

To profile again, take a trace as `tools/profile-report.cs` says and read it with `tools/alloc-report.cs`, which sorts the bytes
by type and by the first frame that isn't a collection.

## Unit 11E: the hot paths

*5 November 2026, on a busy machine: the rig saw 84% total CPU before it started, so this unit reads profiles, counts of calls and runs back to back, and claims no timed figure. The
quiet run comes after merging.*

**Recalculation fell by about two thirds, past the target of 822 ms (1.2× native), with "extrude until" under native's own time.** No volume, face count or Bento triangle count
changed. Expected on a quiet machine: **1,095 ms to about 400 to 500 ms** (0.6× to 0.7× native), and I'm confident of beating 822. "Extrude until" should go from 736 ms to about 150 to 200 (target 548; native 457). The bottle should go
from 229 to about 110 to 130 (target 127): that's close, and the one figure that could still miss.

### Before and after, back to back

Three pairs in fresh processes, 12 passes, ReadyToRun, with other work on the machine throughout, so only the comparison means anything. The rig's own `-Quick -Force` run agrees: 857 ms under 84% CPU, with a settled time of 495 ms.

| | Before | After | |
|---|---:|---:|---:|
| Recalculation, ms (pairs 1, 2, 3) | 1,669, 1,512, 1,538 | 629, 468, 524 | about −66% |
| Opening (first pass), ms | 1,938, 1,939, 1,936 | 1,236, 788, 763 | about −55% in the two pairs without a spike |
| 36 extrude until, ms | 1,061, 991, 973 | 268, 183, 235 | about −77% |
| 29 bottle, ms | 374, 334, 358 | 158, 129, 123 | about −60% |
| 10 select last, hole, ms | 67, 56, 61 | 46, 26, 32 | about −40% |
| Allocated, MB (from the rig) | 217.1 | 207.6 | −4.4% |

The optimal box alone, warm: the bottle 121 to 13.5 ms, "extrude until" 25 to 2.0 ms, example 10 11.7 to 2.1 ms. In the profile, distance went from 47.9% of thread time to 0.1%, and the optimal box from 12.0% to 7.0% of a much smaller total.

### The mesh's tolerance box

**Why it's optimal:** `MeshTolerances.For` is build123d's: 0.1% of the box's diagonal. It has to keep the optimal box, or the meshes change: the loose box is 2.5% longer on the bottle and 8.2% on "extrude until". The mesher is
sensitive: a change of 3.4e-10 in the deflection moved `data/occ/bottle.brep` by 10 triangles. So the box got cheaper, and stayed the same box. Most of the time was in faces that could never affect the answer.

| Commit | What changed |
|---|---|
| Torus patches | Worked out instead of searched for with PSO: about 10 ms a patch (decision 1671) |
| `AddOptimal` | Faces that take their box from their edges come first, and a curved face already inside the box so far is skipped (decision 1672). Tori and spheres were all of the bottle's cost, and all of them lie inside its sides |
| `GenExtSS` | Its 160,000 pairs of samples read from a flat array, as ahead-of-time code can't inline accessors from another assembly. The same pairs in the same order. This search is what's left of the box: 4.5% of thread time before this commit |

**The check, over the 1,499 BRep files up to 500 kB in OCCT's data and the dataset:** 1,490 boxes are bit for bit the same, and 9 differ by at most 1e-7 absolute, 3.4e-10 of the diagonal at most on its length. Of those 9, one changes its
triangle count at the default tolerances (decision 1674):

| Shape | Triangles before | After | Deflection, relative change |
|---|---:|---:|---:|
| `data/occ/bottle.brep` | 12,862 | 12,872 | 3.4e-10 |
| All eleven of Bento's examples | unchanged | unchanged | 0 |

**Something to look at, separately:** on that file the worst triangle centroid is 0.350 from its face, before and after, against a stated deflection of 0.094. It's the same with or without this unit, and may be the mesher's own behaviour on a
particular face; it wasn't looked into here.

### "Extrude until"

**Why so many distance searches:** the operation sorts the target's surfaces by exact distance from the profile, to pick the next one. Each surface there is bounded by intersection curves, so every profile edge against every bounding curve went through `GlobOptMin`
and `NewtonMinimum`: 300 ms to choose between two surfaces 14 units apart. Now each surface has an interval, from its box to its nearest vertex, and the exact distances are taken only if the intervals don't settle it (decision 1673). Where they do settle it the answer must be the sort's, so the
result can't change. A test with a thin hollow sphere checks the case they can't settle.

### Booleans

Not changed (decision 1675). The target's beaten, and nothing is cheap: the pave filler is now 21% of thread time, and 35% of its samples are threads stopped for a collection, with 11% in the parallel loop's spin wait, which is 11D's. By stage, as shares of the filler: edge/face 18%,
face/face 17%, `Init` 11%, `DataIterator.Intersect` 7%, `MakeBlocks` 7%, `FillShrunkData` 6%. "Extrude until" (37%) and the bottle's offset (29%) are most of its samples. Meshing is now the largest operation, at 33% of thread time.

### What's left

- **`GenExtSS` in `IsModifySize`:** a face touching the shape's box still runs up to six of these searches, because the box's tolerance (1e-7) puts a touching face just outside. A closed form for spheres and tori would remove it, but
  picking the same extremum where a whole circle touches is the difficulty.
- **BSpline edges and other searched surfaces:** only B-spline faces have a cheap test first. Bezier, offset and swept faces still search, and a trimmed Bezier's cheap box is sampled so can't be trusted as a limit. Bento's examples have none of these, but imported STEP models will.
- **The bottle:** its build is offset and fillet, with no hot site of its own.

## Unit 11D: parallel throughput

*5 November 2026, overnight on a quiet machine. [benchmarks/2026-11-05-ad0d0c424.md](benchmarks/2026-11-05-ad0d0c424.md); its header says "with uncommitted changes", which were these documents. Curves from `tools/parallel-curve.ps1`.*

**A round of 24 copies on 24 logical processors fell from 2,763 ms to 618 ms, and nearly all of that is two runtime settings and the collector, which belong to the host.** No result changed: 0 differing in every round of every run below, and the examples' volumes, faces and triangles are the same under the new settings as under the old. The rig has no regressions against the run after 11E. No lock costs anything, so all 23 stay.

| 24 copies on 24 processors | Round, ms | Pauses in it, ms | Recalculation, ms | Opening, ms | Peak, MB |
|---|---:|---:|---:|---:|---:|
| The runtime's defaults (what the rig has always measured) | 2,802 | 1,611 | 364 | 710 | 235 |
| Server collector (what Bento uses today) | 1,270 | 197 | 370 | 730 | 390 |
| ... and `TieredPGO` off | 670 | 165 | 260 | 660 | 420 |
| ... and `CallCountingDelayMs` 0 (the recommendation) | 618 | 169 | 182 | 615 | 355 |

### What limits the speed-up

**The rig's speed-up figure (3.11×) misleads, twice over.** Its "serial work" is 24 times a recalculation, but that recalculation isn't serial: the mesher's and others' inner loops use every core, and it burns about 1,000 ms of processor time in 363 ms of wall time. Nor has its code settled: the first nine passes run while the runtime is still tiering up. Against one processor, with the whole process limited by an affinity mask, the default's speed-up is 4.5×, and the recommendation's 9.6×. The rig keeps the row so runs stay comparable; don't read it as a speed-up.

**The physical limit is about 13×.** The processor has 12 cores, with 24 logical processors. Separate processes pinned to one logical processor each, without PGO, share nothing but the machine:

| Processes | 1 | 2 | 4 | 8 | 12 | 16 | 24 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Settled recalculation, each, ms | 235 | 238 | 246 | 253 | 264 | 375 | 427 |
| Throughput against one | 1.0 | 2.0 | 3.8 | 7.4 | 10.7 | 10.0 | 13.2 |

Memory bandwidth and the shared cache cost at most 12% across all 12 cores. The other halves of the cores add only 23% more. Where those start being used (16), throughput dips, because some threads share a core and a round waits for its slowest.

**So, from 2,763 ms:**

| Cause | Evidence | Size |
|---|---|---|
| Workstation collector's pauses | One collector thread, every other thread stopped: 1,611 ms of a round, against 197 with the server collector. It doesn't scale, and gets slower past 8 processors | 1,500 ms |
| PGO's instrumented code | With `TieredPGO` off a round halves. Serial work is only 1.4× slower with it, and a round 1.9×, which fits every thread writing the same shared counters; that cause is inferred, not measured | 600 ms |
| Waiting to tier up | The default 100 ms delay keeps restarting while new methods are being called | 50 ms in a round; a recalculation 260 ms to 182 |
| Pauses still there | 169 of 618 ms, 27%, though with allocation at 4.8 GB a round | see "Allocation" |
| Everything else: shared state, false sharing, inner loops, the JIT | Leaving out the pauses, the recommendation gets 11.9× against the physical 13.2×, so all of these share about 10% | small |
| Lock contention | 0.35 ms in 46 s of processor time | none |

### The curve

A round's time in ms, with the whole process on that many logical processors, cores first. The last columns are the speed-up against the same settings on one processor, and the peak working set at 24.

| Settings | 1 | 2 | 4 | 8 | 12 | 16 | 24 | Speed-up | Peak, MB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Workstation, concurrent (the default) | 11,994 | 5,725 | 2,925 | 2,776 | 2,240 | 2,625 | 2,665 | 4.5× | 196 |
| Workstation, not concurrent | 12,568 | 6,138 | 3,391 | 2,570 | 2,117 | 1,985 | 2,099 | 6.0× | 320 |
| Server, concurrent, DATAS (Bento today) | 13,148 | 5,052 | 2,816 | 1,861 | 1,415 | 1,260 | 1,272 | 10.3× | 391 |
| Server, not concurrent, DATAS | 12,660 | 6,021 | 3,030 | 1,856 | 1,344 | 1,391 | 1,182 | 10.7× | 298 |
| Server, concurrent, fixed heaps | 13,534 | 5,852 | 2,995 | 1,779 | 1,329 | 1,249 | 1,053 | 12.9× | 735 |
| Server, not concurrent, fixed heaps | 12,980 | 5,954 | 2,850 | 1,886 | 1,412 | 1,472 | 1,158 | 11.2× | 704 |
| **The recommendation** | 5,928 | 3,344 | 1,794 | 971 | 702 | 696 | 617 | 9.6× | 415 |
| The recommendation's tiering, workstation collector | 5,940 | 3,266 | 2,146 | 1,788 | 1,803 | 2,061 | 2,174 | 2.7× | 263 |

A speed-up isn't comparable between rows with different tiering: the recommendation's lower figure is from its far better single processor. Compare the times. Each cell is one process, with rounds varying by about 10%, so a difference of a few per cent between server rows means little.

### What a host should use

The recommendation is in [the kernel's README](../README.md) and [bento-integration.md](../../modeling/docs/bento-integration.md), and Bento's list in [11d-bento.md](../../modeling/docs/11d-bento.md). Each setting was checked in `runtimeconfig.json`, as a host would set it, and the rig's new `parallel-host` suite (groups starting "host ") keeps measuring them.

- **Memory:** under full load, about 355 to 415 MB against 196 to 235 on the workstation collector. Calculating serially, 141 MB against 119, because DATAS keeps one heap until the load needs more. Turning DATAS off costs 668 MB serially and 735 MB in a round, for at most 5 to 10% of the round, so leave it on.
- **What PGO off costs:** code that has fully settled is about 6% slower (the last half of 120 passes, 164 ms against 154; a round 585 ms against 520), and a minute of Booleans 11% (1,992 ms against 1,771). Nine passes take 2.7 s against 3.8 s, and 80 passes 15.4 s against 21.0 s, so at those rates PGO would pay for its start after about two minutes of continuous recalculation. A spreadsheet should turn it off; a long batch job might not.
- **`CallCountingDelayMs` 0** is the one to check in a real application: it starts counting calls in every assembly at once, and the benchmark has no UI to start.
- **No help:** `GCConserveMemory` 5 (404 to 426 MB, within the noise; an early reading of about 340 didn't repeat). `GCgen0size` under the server collector: a little quicker at 1.6 GB, and it has no effect from `runtimeconfig.json` as `System.GC.Gen0Size`. `GCgen0size` under the workstation collector: pauses stay at about 1 s, because generation 1 and 2 collections replace generation 0's. `GCHeapCount` with fixed heaps: 12 heaps give 670 ms at 730 MB and 4 give 830 ms at 640 to 880 MB, worse than DATAS on both counts.
- **The benchmarks' projects had the property's name wrong** (`ServerGarbageCollector`, which does nothing). They asked for the default, so nothing changed; they now name it correctly. Proposal 1 below had the same slip.

### Allocation

**The bytes are spread thinly, and the largest site is 5%.** A round is still 4.8 GB. Now that pauses are 27% of a round, 1% of the bytes is worth about 0.25% of a round, so I took one site and left the rest.

| Site | Share of a round's bytes | What happened |
|---|---:|---|
| `Placement.Inverted`, mostly from `ReShapeBuilder` through `Builder.Add` | 2.3% | **Done:** the chain's head keeps its inverse. Recalculation 207.8 to 203.5 MB, a round 4,940 to 4,856 MB (the rig's 4,853) |
| `CircleTool.BindCore`'s `MeshCircle[]` and cell-filter nodes | 5.0% | Left: arrays growing, and reserving them saves about half at best, so 2.5% of the bytes |
| `LinkedListOfInteger.Append` | 3.3% | Left, for 11B's reason: a removed node has to keep its link |
| `Triangle[]` in `AddElement` | 2.8% | Left, as `CircleTool` |
| `Explorer`'s iterators | 6.7% over several sites | Left: already reused within an explorer, and what's left is one explorer for each exploration. Pooling needs a dispose at each of a great many sites |
| `CurveOnPlane` for circles | about 4.5% | Left: not cheap, since the projection's arithmetic has to be matched exactly |

To repeat: take a trace as `tools/profile-report.cs` says, and read it with `tools/alloc-report.cs`.

### The locks: plan 2.7 settled

A trace of three rounds under the server collector with PGO off (`tools/contention-report.cs`) found **12 contentions and 0.35 ms of waiting, all in the runtime's own `Parallel.For`, and none on a lock of Kerf's.** The rig's new row counts 3 to 5 in a round, of any lock in the process. So all 23 stay (decision 1686, [state.md](state.md)): removing one would swap a lock nobody waits for for something harder to check.

### Inner loops and the Booleans' own loop

- **Inner loops when the pool is already full:** every recalculation's own `Parallel.For` queues work no thread is free to take. Forcing them all serial gives 625 ms against 660 under the recommendation's collector and PGO setting. A switch that goes serial when the pool is busy (`ThreadPool.GetAvailableThreads`, tried at 4, 12 and 24 busy threads) gave about 655 against 675, inside the noise, and nothing with PGO on. Not taken (decision 1687): it would make the choice depend on timing, for perhaps 3%. A single recalculation keeps the loops, which give 22% (175 ms settled, against 225 serial).
- **The Booleans' 1.26× and 1.38×** aren't down to the collector, PGO or the loops' overhead. The same under the server collector, with PGO on or off. Going serial below 4 items gains nothing (1.22×, 1.38×), and below 16 loses most of the gain (1.05×, 1.22×), so the big loops are what earns it and small loops aren't costing it. That reading says most of a Boolean is outside the parallel loops: 1.26× on many cores means about 80% is serial. I didn't get to which phases those are, nor whether a wait seen in 11E's profile is more than idle time. For 11Q, if it matters: **independent workbooks are the case that scales, and they don't use this option.**

### BRepGraph

I didn't read its design closely. Its aim is fast traversal and parallel work on one shape's topology, and this unit found neither shared topology nor locks limiting anything, so it wouldn't help here. Whether it would help serial time and allocation is a question for later.

### What's left

- **The last pauses,** 27% of a round: only fewer bytes helps, and there's no large site left.
- **Bento's own start-up** under `CallCountingDelayMs` 0, which only Bento can measure.
- **The cause of PGO's cost on many threads** is inferred. `DOTNET_TieredPGO_InstrumentOnlyHotCode` and similar settings weren't tried.
- **Workbooks in Bento:** this all ran on the benchmark, with no UI thread, and every copy the same size.

## Unit 11C: opening

*6 November 2026, on a quiet machine. [benchmarks/2026-11-06-37d3afa6b.md](benchmarks/2026-11-06-37d3afa6b.md), compared with 11D's run. Traces from `tools/opening-report.cs`. Decisions 1689 to 1694.*

**Opening fell from 745 ms to 509, and the first pass from 710 ms to 473, with every result unchanged.** Nearly all of it is one change to how the packages are compiled. The packages grow from 19.5 MB to 23.4 MB.

| | Before | After |
|---:|---:|---:|
| Opening, process start to the end of the first pass, ms | 745 | **509** |
| First pass, ms | 710 | **473** |
| First pass under a host's settings, ms | 615 | **469** |
| Opening under a host's settings, ms | 653 | **509** |
| The JIT's time in opening, all threads, ms | 285 | 141 |
| Methods the JIT compiled in opening | 4,496 | 1,759 |
| Recalculation, ms | 364 | 269 |
| Recalculation under a host's settings, ms | 182 | 181 |
| Settled recalculation, ms | 347 | 312 |
| A round under a host's settings, ms | 618 | 614 |
| The five assemblies on disk, MB | 39.7 | 48.1 |
| The five packages, MB | 19.5 | 23.4 |

The rig reports one regression and it isn't a real one: the parallel speed-up under the runtime's defaults falls from 3.19× to 2.47×, because the serial work it's divided by fell by 26% while the round fell by 5% (2,802 ms to 2,675). Under a host's settings the speed-up is unchanged at 7.1×. All 45 result rows are the same, and neither parallel suite differs from its serial run. Anything comparing against this run with `-Compare` starts from the new figure.

### Where opening's time goes

Measured, in fresh processes, unless it says otherwise. Turning call counting off (`DOTNET_TC_CallCounting=0`) stops code being recompiled, so a later pass is the work alone, in the code the first pass ended up with, and the difference from the first pass is the cost of getting ready.

| ms | Before | After | How it's known |
|---|---:|---:|---|
| Start of the process to the first example | 36 | 36 | Measured. The host and the runtime starting, and `Main`; Kerf isn't involved |
| The first pass | 710 | 473 | Measured |
| of which the work (a later pass, without tiering) | 349 | 242 | Measured. It falls too, because generics that were quick-jitted, unoptimised, are now precompiled and optimised |
| of which getting ready | 360 | 231 | The difference |
| JIT on the working thread | about 155 | about 60 | Inferred: its share of a traced pass (200 of 913 ms, and 80 of 632), since tracing slows everything |
| Type loads outside the JIT | about 40 | about 45 | Inferred the same way (53 of 913 ms, and 62 of 632): 8,029 and 7,605 loads |
| Static constructors | under 10 | under 10 | Inferred from samples: 2 to 5 of about 255 on the working thread, none over 1 |
| Assembly loading | a few | a few | From the trace: seven assemblies load during the pass, each in under about a millisecond |
| First calls: entry-point lookups, fixups and stubs | about 150 | about 115 | Inferred: what's left. The trace counts the lookups (22,406 and 28,768 in two passes) but can't time them |

The JIT also works on the background thread (131 ms traced before, 76 after), which doesn't directly hold up the working thread.

**Why the JIT ran despite ReadyToRun.** Almost nothing was rejected: 34 non-generic methods out of 3,282 on the working thread, costing 2.5 ms. The rest were generic instantiations crossing an assembly, which plain ReadyToRun can't precompile, because each assembly is compiled with only its own code: traced, before, 1,601 methods of the framework's generic types over Kerf's (`List<Shape>`, `Dictionary<Shape, int>`), 77 ms; 874 of Kerf's generics over types from another Kerf assembly or as generic methods, 64 ms; 370 of the framework's generic methods (mostly LINQ), 33 ms. `Shape` being a struct is what makes each instantiation need its own code. After the change, 744 of the first group remain, 214 of the second and 144 of the third, 68 ms traced in all. Many of those are small: `List<T>`'s static constructor, for one, for every `T`.

By example, first pass, from the rig, and the shares of the final build's trace:

| Example | First pass before, ms | After, ms | Traced JIT, ms | Traced type loads, ms | Traced rest, ms |
|---|---:|---:|---:|---:|---:|
| 2 plate with hole (the first, so it bears the most) | 141 | 90 | 34.7 | 38.4 | 99.4 |
| 3 prismatic solid | 29 | 20 | 5.5 | 5.0 | 22.7 |
| 5 working point | 10 | 7 | 0 | 0 | 7.1 |
| 6 point lists | 17 | 12 | 0.3 | 0.5 | 9.9 |
| 9 chamfer, fillet | 74 | 48 | 18.1 | 8.5 | 51.0 |
| 10 select last, hole | 34 | 20 | 1.0 | 1.1 | 22.4 |
| 29 bottle | 141 | 97 | 11.6 | 5.5 | 96.7 |
| 31 nesting locations | 36 | 23 | 3.0 | 0.4 | 21.3 |
| 32 for-loop | 17 | 12 | 0.7 | 0 | 12.3 |
| 33 function, for-loop | 13 | 9 | 0.5 | 0 | 9.0 |
| 36 extrude until | 196 | 133 | 4.1 | 2.3 | 139.2 |

The traced columns add up to the traced pass, which is slower than the untraced one. The first example pays for the runtime getting ready for everything after it; "extrude until" is nearly all work.

### ReadyToRun: what the packages ship, and the choices

Each row is the five assemblies compiled with crossgen2 and the same benchmark, medians of 7 to 11 fresh processes run in turn, with spreads of about 5 ms.

| | Opening, ms | JIT'd methods | Assemblies, MB | Whose choice |
|---|---:|---:|---:|---|
| Plain ReadyToRun (what 0.2.0 ships) | 743 | 4,497 | 39.7 | |
| `--Ot` alone | 736 | 4,499 | 40.6 | |
| `--non-local-generics-module:*`, `--Ot` | 739 | 4,499 | 40.6 | No use alone |
| `--opt-cross-module:*`, `--Ot` | 600 | 2,888 | 45.6 | |
| Both, without `--Ot` | 530 | 1,731 | 46.1 | |
| **Both, with `--Ot`: taken** | **512** | **1,789** | **48.1** | **The library's** |
| Both, with `--instruction-set:x86-64-v3` | 522 | 1,716 | 46.1 | Not taken: 8 ms, for code that assumes AVX2 |
| Both, with `--type-validation:SkipTypeValidation` | 531 | 1,731 | 46.1 | No use |
| Kerf's five as one version bubble (`--inputbubble`), alone | 683 | 3,818 | 42.6 | The owner's, below |
| A version bubble with both and `--Ot` | 478 | 1,473 | 51.8 | The owner's, below |
| Kerf's five as one composite image | 521 | 2,184 | 46.2 | Not possible in a package |
| The benchmark self-contained, composite with the framework | 419 | 449 | 169 for the whole application | The host's |

- **Taken** (decision 1689): `PublishReadyToRunCrossgen2ExtraArgs` in both `Directory.Build.props`, used only when publishing ReadyToRun. It costs a library user 3.9 MB of download and 8.4 MB on disk, and nothing else: no setting, no change of API. `--opt-cross-module` lets framework code inline into Kerf's and `--non-local-generics-module` lets an assembly hold instantiations over other assemblies' types; the first is what lets the second help.
- **The framework can change under these.** Code inlined across the version boundary carries a check of the IL it inlined, and is dropped for the JIT if that doesn't match, which a forced mismatch confirmed (decision 1689): a library swapped under a small application compiled this way, and its inlined and generic code was rejected and compiled afresh, returning the new library's values. What was measured: assemblies compiled against framework 10.0.0 and run on 10.0.11 open in 512 ms against 514, with the same 1,789 methods and the same results, so a patch of the framework is no trouble, and the JIT is the worst case. .NET 9's framework couldn't be used to force mismatches, because Kerf's IL doesn't compile against it. Five suites pass with these assemblies in place of the IL (below). These two options are little documented; crossgen2's help doesn't call them unsupported, as it does `--inputbubbleref`.
- **The version bubble is left to the owner** (decision 1690). It gives another 34 ms for another 3.7 MB. But it ties the five assemblies together: code from one is compiled into another with no check, so a mix of versions would misbehave or stop at load (inferred from how the runtime treats bubbles; not tried). The packages would have to depend on each other at exactly the same version, where NuGet now allows a newer one. A bubble is also what `--inputbubbleref` makes, the option the help calls unsupported.
- **Composite is the host's** (decision 1691). It needs every assembly in one image, so a library can't ship it, and the SDK only offers it self-contained. For an application that ships its own runtime, 419 ms against 509, at 169 MB. That was built from the projects, so from IL. Publishing a composite from the packages, whose assemblies are already ReadyToRun, wasn't checked: the attempt failed copying a file during the publish, and the cause wasn't looked into.
- **A profile (`--mibc`) wasn't tried.** It might root the instantiations still being compiled and lay code out better, but `dotnet-pgo`, which makes one, isn't on nuget.org.

### Work at load

There's little to take (decision 1693). Static constructors are 2 to 5 of about 255 samples on the working thread, in dozens of small types, the largest a sample or two: `Default<T>` for each `T`, `BOPAlgo_msg`'s message table (one `Lazy`, on the first Boolean), `Primitives`. Nothing large is built whether or not it's used, so nothing was made lazy, and no order or timing could change.

Two things ran needlessly on a first pass, and were cheap to stop (decision 1692). Together they gave about 4 to 6 ms of opening, which is a small part of the total:

- **`DiagnosticCode` checked each well-known code with a regular expression,** which loaded the assembly and built the expression on every opening. A plain loop now accepts what's plainly well formed, and anything else still goes to the expression, so what's accepted hasn't changed, including its final newline.
- **Grouping by a double boxed its keys and compared them through the non-generic `Comparer`,** which starts the current culture's collation: `System.Globalization.Invariant` used to save 7 ms of opening, and now saves nothing. Doubles now compare directly, which orders them the same, NaN included, and a test says so.

### Tiering settings

11D's settings stand. None of these helps opening, so there's no new setting for a host (decision 1694). Before the compilation change, in opening's milliseconds, against 748 by default:

| Setting | Opening, ms |
|---|---:|
| `TieredCompilation` off | 996 |
| `TC_QuickJit` off | 1,003 |
| `TC_QuickJitForLoops` off | 864 |
| `OSR_HitLimit` 0 | 774 |
| `TC_BackgroundWorkerTimeoutMs` 0 | 743 |

After it, against 512 by default: `TC_QuickJitForLoops` off 597, `TC_BackgroundWorkerTimeoutMs` 0 512, `TC_CallCountingDelayMs` 1000 512, `TC_DelaySingleProcMultiplier` 1 512. `TC_CallCountingDelayMs` 0 alone, with PGO on, costs 607, and `TieredPGO` off alone gives 497. Under 11D's full recommendation it's 509, so counting calls from the start and the server collector together now cost opening about 12 ms, for recalculation falling from about 260 ms to 180. That's still the right trade.

Other things that didn't help: `gcConcurrent` off (522), a generation 0 budget of 64 MB (517) or 256 MB (533), and fewer processors (1: 526; 4: 514; 12: 514). So the working thread's waits on the inner parallel loops, about a fifth of its samples, aren't lost time: going serial is slower.

### Suites

In Release, each run twice: as built, and with the five assemblies as the packages ship them in place of the IL. Foundation 2,474, ModelingData 3,319, ModelingAlgorithms 4,923 and 9 skipped, DataExchange 342 and 1 skipped, Kerf.Modeling 1,950, with none failing either way. `pack-nuget.ps1`'s consumer smoke test passes on the new packages.

### What's left

- About 1,750 methods still go through the JIT, 141 ms across all threads. Some of Kerf's own generics over its own types remain, and it isn't clear why crossgen2 leaves them; the bubble takes about 300 more.
- Bento's own assemblies aren't Kerf's to compile. If Bento publishes ReadyToRun, the same property applies to its own generics over Kerf's types. Nothing here was measured in Bento.
- About 115 ms of first calls, inferred. A native profiler would time them directly; this machine's account can't take a kernel trace without elevation.

## StepTidy (unit 11N)

*5 November 2026, `Kerf.Kernel.Benchmarks -- step-tidy` over the 289 files `-- healing` exports, written by the plain STEP
writer with `CleanDuplicates` off and on. Whether it's on by default is the owner's decision (decision 1667).*

| | Off | On | Change |
|---|---:|---:|---:|
| Total file size | 49,067,067 bytes | 40,272,582 bytes | 17.9% less |
| Allocation, all files | 3,023 MB | 3,347 MB | 10.7% more |

- **Per file:** the median reduction is 18.9%, and the largest 55.6% (`pr1db.stp`, 177,965 to 79,010 bytes). Some small files
  have nothing to merge: 0.0%.
- **Read back:** all 289 files, both ways. Volumes (to a part in 10^9) and face counts match on every file, and none failed.
- **Writing time isn't measured yet:** the machine was running a coverage run. Taken there, the total was 3,068 ms off and
  3,983 ms on, which only says the order of it. It belongs in a quiet run of the same command.
- **Merging within OCCT's 1e-12 isn't worth it:** 20.7% where exact merging gives 17.9%, and one file read back differently
  (decision 1665).

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
| 1 | **Server GC (or a larger gen-0 budget) in hosts that compute in parallel.** A setting in Bento's and build123d.NET's test projects (`<ServerGarbageCollection>true</ServerGarbageCollection>`, or `<ConcurrentGarbageCollection>` with `GCgen0size`). Measured: the suite in parallel from 21 s to 10 s; a 64 MB gen-0 budget alone gave 16 s. | 2× on parallel workloads | A line per host | Memory: server GC keeps larger heaps; check Bento's working set |
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
