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
kernel/tools/bench.ps1 -Quick -Compare kernel/docs/benchmarks/2026-11-07-f5a7bd7a2.tsv
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
- **Since this table:** [the run after 11C, 11D, 11P, 11Q's first slice and the clean-up](benchmarks/2026-11-06-bd7045018.md), on a quiet machine on 6 November, is the one to compare with now. Recalculation 279 ms (0.41× native), opening 470 ms (0.51×), 198 MB allocated per recalculation. Under a host's settings (unit 11D): recalculation 175 ms, a round 610 ms.
- **Phase 12's section 1, 7 November:** [its run](benchmarks/2026-11-07-f5a7bd7a2.md), quiet, is the one to compare with now. Against 0.3.0 as packed: recalculation 265 to 251 ms, 126 to 123 MB, and opening 518 to 522 ms, which is noise; nesting 14.5 to 12.9 MB and for-loop 9.04 to 8.67 MB; under a host's settings 166 to 163 ms and a round of 530 to 512 ms. **A first fillet, cut and STEP export, five times each:** 2,096 ms loaded as Bento does today, 203 ms from files outside a collectible context (decisions 1896 and 1897). Its flags: a fillet's limit check, 6.74 to 6.98 MB on example 9 (decision 1898, which saves far more on a refusal); and `Pump_Nut.brep`'s parallel mesh, 5.86 to 6.81 MB, in code this section didn't change, where its time and the serial figure don't move.
- **0.3.0 as packed, 7 November:** [the release's run](benchmarks/2026-11-07-d18bc4ede.md), quiet, is the one to compare with now. Recalculation 265 ms (0.39× native), opening 518 ms (0.56×), 126 MB allocated per recalculation, settled 302 ms; the default parallel round 2,062 ms and a speed-up of 3.08×; under a host's settings 166 ms, a round of 530 ms and a speed-up of 7.50×. Against the run below it has only lost non-local generics (decision 1890): opening 446 to 518 ms, and the JIT's time in that 141 to 206 ms. Later recalculation, parallel rounds and bytes don't move beyond their noise, but for a few bytes of the JIT's own on the first small examples.
- **What none of these show: Bento's own loading gets no ReadyToRun.** An extension's `AssemblyLoadContext` is collectible, and it loads from bytes, and the runtime doesn't use precompiled code in either case. So all of Kerf is compiled by the JIT there. **Measured, a fillet, a cut and a STEP export, first thing in a new process, on Kerf's packages:** 170 ms loaded normally, 2,200 ms collectible from bytes (twice each, on a quiet machine). The rig's opening figures, and the effect on code that's still warming up, don't apply to Bento's extension as it stands. The rig needs a row that loads as Bento does, and it's Bento's choice how Kerf is loaded; `bento-integration.md` has the options. **Since then (12D):** the rig has those rows, `first-bento-today` and `first-bento-files`, 2,444 and 225 ms on a loaded machine, and Bento's branch `kerf/load-from-files` loads Kerf from its files: 2,205 to 330 ms through its real loader, on the same terms ([12d-bento.md](../../modeling/docs/12d-bento.md)). What this paragraph says holds until that's merged.
- **Before non-local generics went, 7 November:** [that run](benchmarks/2026-11-07-bcf6d5826.md), quiet. Recalculation 251 ms (0.37× native), opening 446 ms (0.48×), 127 MB allocated per recalculation, settled 290 ms; the default parallel round 2,068 ms and a speed-up of 2.97×; under a host's settings 166 ms, a round of 511 ms and a speed-up of 7.87×. The default round moves by about 12% within a run, so 1,903 and 2,068 ms aren't a difference; its bytes are, 3,495 to 3,189 MB. It holds the fix for the bottle's rise below (decision 1887). All nine suites pass, and the full corpus doesn't differ, in all 17,872 cases, by status.
- **The morning of 7 November:** [the run after the modelling layer and the reliability second pass](benchmarks/2026-11-07-b54ad69ed.md), quiet, is the one to compare with now. Recalculation 256 ms (0.37× native), opening 454 ms (0.49×), 138 MB allocated per recalculation, settled 302 ms; the default parallel round 1,903 ms (from 2,378), with a speed-up of 3.25× and collector's pauses 27% shorter; under a host's settings 174 ms, a round of 531 ms and a speed-up of 7.97×. The longest single pause is 2.6 ms serially, 21.1 ms in a default round and 20.0 ms under a host's settings. Its flags: the bottle's allocation 46.6 to 49.2 MB, which hid a larger rise from decision 1881 that the modelling layer's cuts covered, 36.7 to 49.2 MB, since fixed (decision 1887); and a thick solid's 79.1 to 80.8 MB, not yet traced.

- **The night of 6 to 7 November:** [the run with the checks in](benchmarks/2026-11-07-3e9d11be2.md), quiet, is the one to compare with now. Recalculation 275 ms (0.40× native), opening 468 ms (0.51×), 152 MB allocated per recalculation; under a host's settings 174 ms and a round of 571 ms. So this holds what each operation now checks (units 11R and the Boolean volumes) and the precise area, volume and centre, with the rest of the work paying for them, except on examples of many small solids: "for-loop" 10.6 to 17.8 ms, and 9.0 to 10.6 MB. Settled recalculation 298 to 328 ms.
- **What a user would feel from the collector isn't a hitch:** the longest single pause is 5.2 ms serially, 25.8 ms in a default parallel round and 16.8 ms under a host's settings. **Peak working set under a host's settings is noisy:** 355, 409, 444, 333 and 459 MB over the last five runs. The slowest one model in any round, 532 ms under a host's settings, includes the first, cold round.

- **Since then:** [the run after 11Q's second slice and 11G](benchmarks/2026-11-06-b21606ad7.md), quiet, 6 November, is the one to compare with now. Recalculation 275 ms (0.40× native), opening 457 ms (0.50×), 160 MB allocated per recalculation; under a host's settings 168 ms and a round of 575 ms. **Allocation fell 19% and recalculation only 1.6%:** on one thread those bytes weren't costing much, as decision 1705 suggests. It's a host's parallel round (down 5.8%), healing (ShapeFixer down 7%) and memory that gain. A default parallel round didn't move (2,564 ms), and the serial passes' collector pauses read 16% higher on less allocation, which the collector's own budgeting may explain (not investigated).
- **That run's three result flags are 11G's:** on the three examples with fillets, SVG grew by 8, 4 and 26 characters, with the same edges and the same PNG. The images suite at the commit before 11G matches the old figures. A fillet's edges are now coded tangent (decision 1719), so hidden-line removal classes them as smooth rather than sharp (`DrawingOperations`, `reg1`); that the difference is only this class is inferred from the code.

- **Its four flags were noise.** A rerun of those suites, against that run, gave 0 regressions: PNG on two examples 49 and 46 ms, then 21 and 18 (the others had fallen by about 27%); a host's processor time 609 then 453 ms; and "fillet every edge" 106 MB then 102, so that row's allocation moves by about 4% between runs, more than the rig's 2%. The full corpus against the run after 11I: 7 of 17,872 cases differ, six memory checks now passing (unit 11P) and `bug83`, which now always fails its known time check (decision 1702).
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
| **`--opt-cross-module:*`, `--Ot`: taken since 0.3.0** (then; 518 at the release's run) | **600** | **2,888** | **45.6** | **The library's** |
| Both, without `--Ot` | 530 | 1,731 | 46.1 | |
| Both, with `--Ot`, as at first | 512 | 1,789 | 48.1 | Breaks a second copy (decision 1890) |
| Both, with `--instruction-set:x86-64-v3` | 522 | 1,716 | 46.1 | Not taken: 8 ms, for code that assumes AVX2 |
| Both, with `--type-validation:SkipTypeValidation` | 531 | 1,731 | 46.1 | No use |
| Kerf's five as one version bubble (`--inputbubble`), alone | 683 | 3,818 | 42.6 | The owner's, below |
| A version bubble with both and `--Ot` | 478 | 1,473 | 51.8 | The owner's, below |
| Kerf's five as one composite image | 521 | 2,184 | 46.2 | Not possible in a package |
| The benchmark self-contained, composite with the framework | 419 | 449 | 169 for the whole application | The host's |

- **Replaced** (decision 1890): non-local generics are out, as a second copy of Kerf in a process compiled by the JIT then fails. The rest of this list is as decision 1689 had it, with both.
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

## What allocation is made of

*Unit 11Q, 6 November 2026, on a shared machine; bytes don't depend on the load. One trace of Bento's eleven examples, passes 2 to 20, ReadyToRun: 3,737 MB in 35,688 allocation ticks, about one for each 100 kB, so a group of 1% is good to about ±0.05 of a point. A recalculation is 204.6 MB. To repeat it: `tools/alloc-report.cs ... ticks`, then `tools/alloc-groups.py`.*

**Is the high allocation caused by making most types immutable? No. Immutability's own costs are 3.5% of the bytes, and only 0.9% has no counterpart in OCCT. The most a sceptic could lay at its door is about 12%.** About 93% is temporary working objects from the port: arrays, maps, iterators and adaptors that the algorithms make and drop. What a caller ends up keeping is 0.64%.

| Group | Share of the bytes | MB per recalculation |
|---|---:|---:|
| **Immutability's own costs** | **3.5%** | 7.2 |
| `ReShapeBuilder`, all it calls (OCCT has `ShapeBuild_ReShape`, doing the same copies) | 2.24% | |
| Working overlay: `WorkingOverlay` and the working copies it makes | 0.73% | |
| `Placement` (OCCT's `TopLoc_Location` also allocates these nodes) | 0.39% | |
| Freezing a result | 0.12% | |
| `with` copies, `ImmutableArray` | 0.01% | |
| **Temporaries from the port** | **92.9%** | 190 |
| Collections and arrays, mesher: triangles, circles, links, cell filter | 26.2% | |
| Collections and arrays, everything else: `double[]` 6.7%, shape lists and maps, points | 30.9% | |
| Adaptors and projectors | 10.3% | |
| Iterators and explorers: `Iterator` alone 4.0%, `Explorer` 1.4% | 8.7% | |
| Helper objects: `SLProps` 1.4%, extrema's point-on-curve objects, `SurfaceContinuity` | 7.4% | |
| Elementary geometry values boxed in adaptors (`Plane3D`, `Circle3D`, `Line3D`) | 3.4% | |
| Curves and surfaces made to be read once, as by `CurveOnPlane`'s projection | 3.0% | |
| Math objects: `VectorBase`, `Matrix`, solvers | 2.5% | |
| Closures and delegates | 0.5% | |
| **Results** | **2.7%** | 5.5 |
| Meshes: triangulations, and reading them into `ShapeMesh` | 1.84% | |
| Topology and geometry built | 0.84% | |
| **Other:** tasks, threads, strings | **0.9%** | 1.8 |

**How the lines were drawn.** A tick takes the first group that matches, and immutability goes first. So anything with `WorkingOverlay`, `ReShapeBuilder`, `Placement`, freezing or an immutable collection *anywhere on its stack* counts as immutability, including whatever those call. That leans towards immutability. Results come next, by type and where they were made. A curve made under an adaptor, projection or box is a temporary; a curve made anywhere else is a result. The rest is temporaries, divided by type, unless the stack holds no Kerf code or the type is the runtime's. `tools/alloc-groups.py` has the exact rules.

**Results are checked a second way.** `Kerf.Kernel.Bento.Benchmarks -- retained` holds eight copies of each example's shape and mesh, and reads what the collector can't free: **1.3 MB of 204.8 MB allocated, or 0.64%** (0.28% to 1.61% by example). So the 2.7% above is, if anything, generous. The difference is mostly each step's result, which the next step drops.

**The sceptic's bound.** Grant every doubtful case to immutability, even those where the C++ does the same:

| Added to the group above | Share |
|---|---:|
| Immutability's own costs | 3.5% |
| All boxed elementary geometry values, though the box comes from standing in for a C++ variant | 3.41% |
| Reading meshes into `ShapeMesh`, and the triangulations they're read from | 1.84% |
| Any copy, reversal or transform within six frames of the allocation (OCCT's `BRep_Tool` and `GeomProjLib` copy too) | 1.52% |
| Everything the public layers allocate directly, `Kerf.Modeling` included | 1.07% |
| Every other shape and curve built | 0.61% |
| **Upper bound** | **12.0%** |

Even that leaves 88% with no link to immutability. What remains ambiguous is whether an algorithm builds a list because it can't edit in place, which a stack can't show. None of the largest sites does: they're the mesher's own structures, `MapShapes`, the numerics' work arrays, and adaptors. Immutable *structs* such as `Point3D` cost nothing here unless they're boxed. They're values, and a `with` is a copy on the stack.

**OCCT for the same work is an estimate, not a measurement.** Native's "3 MB" only ever counted the managed wrappers. OCCT's own heap was outside .NET's count, and native can't be measured again. So the comparison with 204 MB was never like for like. Reading what the C++ does group by group:

| Group | Share here | Share OCCT would also put on its heap (estimate) | Why |
|---|---:|---:|---|
| Mesher's collections | 26.2% | a half to four fifths | Bump allocators and segmented vectors, the same data, but no copies from growing a list |
| Other collections, arrays | 30.9% | 70% to all | `NCollection_Array1`, maps and lists are on the heap there too |
| Adaptors, projectors | 10.3% | 30% to 50% | Often on the stack, though with handles inside |
| Iterators, explorers | 8.7% | up to 10% | `TopoDS_Iterator` is on the stack, and `TopExp_Explorer` holds its stack inline |
| Helpers | 7.4% | 10% to 30% | `BRepLProp_SLProps` and extrema objects are mostly on the stack |
| Boxed geometry values, closures | 3.9% | none | |
| Math objects | 2.5% | 20% to 50% | `math_Vector` keeps up to 32 values inline |
| Geometry read once, results, `ReShape`, `Placement` | 8.3% | all | The same objects |
| Overlay, freezing, other | 1.8% | none | |

That comes to **about 50% to 70% of Kerf's bytes, or 100 to 145 MB for a recalculation, so Kerf allocates perhaps 1.4 to 2 times what the C++ would.** The excess is the port's stack objects that became heap objects in .NET, and growing lists. It's a real target, and it isn't immutability. C++ also pays for its heap differently, with `malloc` on each object and no collector.

**What this says to do.** Don't touch the immutable design for allocation's sake. If more allocation is to go, the largest single groups are the mesher's collections, then iterators and explorers (8.7%, stack objects in OCCT) and adaptors with their boxed values (13.7%). Unit 11D's point stands that allocation is what limits parallel rounds.

## Slow use cases (unit 11Q)

*6 November 2026, on a machine shared with up to three other helpers, so every time here comes from a loaded machine. Allocation is exact, and shares and counts are good. The clean timed run comes after merging. Decision 1681: no more comparing with native, just improving what's slow.*

### The list

Re-ranked by the fourth slice from a new pass of the corpus; the figures are that pass's, or the rig's. Ranked by what a user would plausibly meet, not by raw time: a common operation that takes long goes above an obscure one that takes longer. "Hard" means the time goes on the geometry itself, and nothing was found that barely needed to run.

| # | What | How long (loaded) | Why | State |
|---:|---|---:|---|---|
| 1 | **Large and fine meshes** (every shape is meshed) | `bug29715` 16 s, `bug25503_1` 13 s, `bug27453` 13 s; Axis_of_bearing about 200 ms | Half is finding each node's circles: long cell lists, as in OCCT | **Better** (fourth slice); what's left needs a question |
| 2 | **STEP import with healing** | 5.3 s for 289 files up to 400 kB; `de/step_4/E4` 12 s, `de/step_2/S9` 10 s | Half is `ShapeFixer`: projecting curves point by point | Better, not done |
| 3 | **Booleans of many tools, general fuses** | `bug25232_9` 59 s, `boolean/simplify/A3` 23 s, `boolean/bsection/P4` 19 s, `boolean/periodicity/A4` 17 s, `bug25614_genfuse` 14 s | Many small intersections | Better, not done |
| 4 | **Edges against free-form faces** | `buc60463` 25 s, `bug32944` 21 s, `bug29182_1` 16 s, `buc60555_3` 16 s | Exact intersections, a particle swarm for conics. Polyhedra and grids taken out | **Hard** now |
| 5 | **Face-face on free-form surfaces** | `bug28119_1` 45 s, `bug30760` 17 s, `bug24905` 12 s | Fitting curves | Hard, better |
| 6 | **STEP export** | 1.3 to 1.9 s for the same files | Strings for every parameter | Better (third slice) |
| 7 | **Validity checks on big shapes** | a quarter of `bug25232_9`'s active time; half of `bug26701`'s bytes | Real 2D work, and `Placement` | Spread out |
| 8 | **Boolean with many small faces, in parallel** | `bug26701` 29 s | A new context for each thread in each loop rebuilds classifiers and bounds | Decision 1718's limit: larger change |
| 9 | **Pipe shells with corners** | `bug32444` 40 s | Booleans in each corner | Hard |
| 10 | **Max fillet** | about 400 ms for two searches on a box (was 500) | A search over real fillets | **Hard by design, better** (fourth slice) |
| 11 | **Fillets, offsets on every edge** | 56 to 186 ms for 16 parts; `offset/.../B7` 7 s | Booleans inside offsets | Hard |
| 12 | **2D offsets** | `bug6768` 9 s | Bisector and polygon intersection on B-splines | **Better** (fourth slice) |
| 13 | **Hidden lines** | the bottle's SVG 29 ms | Contours and hiding | **Hard** (fourth slice) |
| – | **PNG pictures, area and volume to a precision** | | | Fixed (first slice) |

**Not Kerf:** the corpus's slowest cases are the test harness. `lowalgos/extcs/circ_sph_*` (1,212 s, 14% of the corpus) and the `chamfer/*_sequence` cases spend about 99% of their allocation (106 GB in one case) and nearly all their time in Draw-lite's `lappend`, which copies a growing string each time. Only 0.75% of a chamfer case's samples are chamfering. A fix needs Tcl values that aren't strings, so it wasn't done, but it would take perhaps a fifth off a full corpus run.

Bento's examples aren't on the list. At 269 ms for all eleven they're no longer slow, and what's left there is meshing and small Booleans.

### What changed

Every result is unchanged, and each commit can be dropped alone.

| Change | Evidence |
|---|---|
| **A polyhedron for each face, not each edge** (decision 1707) | `buc60555_3` **5,192 MB to 1,143 MB**, about half the time alone. It was 71% of the case's active samples and 88% of its allocation. Booleans on planes and quadrics don't take this path |
| **Precise area and volume reuse their work vectors,** and scan only what's been written (decision 1708) | `bug25232_9` **37.3 GB to 13.3 GB**, generation 2 collections 224 to 18, 59.5 s to 29.2 s run alone back to back |
| **Fitting curves reuses its vectors and matrix** (decision 1708) | `bug24905` **2,508 MB to 1,000 MB** |
| **Reshape copies a shape only if a sub-shape changes,** and skips the walk with no requests (decision 1709) | On the 289 STEP files: ShapeFixer **3,605 to 2,455 MB**, UnifySameDomain 922 to 649, import 5,371 to 4,564 |
| **Stick vertices from an index,** not a scan of all interferences for each pair of faces | `bug25614_fuse`: 5.6 share points of its Boolean's 45.8 to about 0.15. No effect on small Booleans |
| **STEP's reals written in place** | Export 2,954 to 2,319 MB. A test compares 1.2 million conversions with the general path |
| **A polygon's self-intersection skips far pairs from their extents** | `Interference` 5.5% of healing's samples to 0.7% |
| **PNG's filter stops a filter once it can't win** | `Png.Filter` about 1,440 samples to about 295 on the same work; the known-bytes test and sizes unchanged |
| **The rasteriser visits only each row's span** | `Triangle`'s own samples about 1,080 to 740. PNG hashes of all eleven of Bento's match with and without |

Bento's allocation went from 204.6 to 198.7 MB a recalculation, with the same volumes, faces and triangles.

### The profiler's blind spot

**The sampling profile overstates anything that polls, and collector pauses most of all.** EventPipe's sampler has to suspend each thread, which stops at its next safe point. Ahead-of-time and tiered code in a tight loop often has none, so the whole stretch is credited to the next call that polls: `Array.Copy`, `BulkMoveWithWriteBarrier`, a `Dictionary` resize. That's inferred from what follows, not measured directly. Measured from the collector's own events on the same trace:

| Trace | `PollGC` samples | Time really in collections |
|---|---:|---:|
| Bento's recalculation | 24.7% | 2.5% |
| Healing suite | 33.5% | 14.2% |

So 11A's "the collector, 16.5% of thread time" and 11E's "35% of its samples are threads stopped for a collection" are mostly this, not pauses. `BSplineCurveGrid.ExtractCurveData` had 24% of one case's samples, 99.6% of them in `PollGC` under a struct copy. `Png.Filter`'s `Memmove`, at 25%, was really the filter's loops. `RuntimeHelpers.GetHashCodeWorker` at 13% to 15% looks misleading too, as it shows under methods with no hashing in them; that's unexplained. Trust inclusive shares of high-level operations, allocation and call counts, and don't trust leaves. Earlier sections keep their words, as they were written.

### What didn't help, and what's left

- **The Booleans suites hardly moved** (56.8 to 56.5 GB over the profile's rounds). Their cost is spread over many small intersections, and the polyhedra aren't on their path.
- **The large object heap:** `GenExtPS`'s grids are 1.3 of healing's 2.2 GB of large objects, and each can provoke a generation 2 collection. An extrema object has no end of life to pool them at. A host setting does the job under the workstation collector: `DOTNET_GCLOHThreshold=0x400000` took healing's generation 2 collections from 656 to 163 and time in collections from 18.5% to 12.1%, and Bento's serial passes from 44 to 8. Under the server collector, which Bento uses, a parallel run's count didn't change (36 and 35). So there's no recommendation yet.
- **The write session's graph and check** after a transfer are about 13% of a plain `Writer`'s export. The public path, `CafWriter`, uses the graph afterwards, so it stays.
- **`Iterator` and `Explorer`** are 8.7% of Bento's bytes and 6.5% of healing's. Each explorer has an iterator object for each level, where OCCT's are inline. Making them one array of structs is the next allocation step. It wasn't done, as `Iterator` is used everywhere. The second slice did it, below.
- **`CurveOnPlane` for circles, `FClass2d`, `WireExplorer`:** not looked at further.
- **The full corpus at this head, on a loaded machine:** every status is as before but for time and memory checks. `bug24639` (parallel must beat serial by 20%) and `bug83` (a section under 0.1 s) now fail their time limits, and `bug24639` passes alone or 12 at once; the memory trends `bug31075` and `bug7287_*` flipped to OK. They need the clean run to confirm, and they're for 11P.
- **Corpus times are noisy under load:** the runs differ by up to a factor of two on single cases, either way, so only the large changes above are claims.

### The second slice: what limits the Booleans' parallel option, and the next cuts

*6 November 2026, the same shared machine: bytes and counts are exact, every time is from a loaded machine, and the clean run comes after merging. `-- booleans many` is new: a plate with 100 holes, and `bug25614_fuse`'s 121 boxes and 100 cylinders.*

**The Booleans' parallel option isn't held back by serial phases, as 11D inferred from its 1.26×. It's held back by the largest item of each loop.** A temporary timer around every loop of `BooleanParallel` and every phase, serially:

| Serial, ms a pass (loaded) | Whole | In the loops | Outside them | Largest item of each loop, added up | Best possible with endless processors |
|---|---:|---:|---:|---:|---:|
| Simple, 407 cases | 2,674 | 2,104 (79%) | 570 | 1,245 (face/face 763) | **1.47×** |
| Complex, 193 cases | 3,038 | 2,644 (87%) | 394 | 1,549 (face/face 1,060) | **1.56×** |
| Plate with 100 holes | 997 | 803 (81%) | 194 | 239 (solid 79, face 70) | 2.3× |
| 121 boxes and 100 cylinders | 5,420 | 3,336 (62%) | 2,084, with 1,222 of the collector's pauses | 341 | 2.2× |

So the rig's 1.23× and 1.47× are near the limit for these inputs. A typical Boolean in those suites has about ten face pairs, with one taking most of the time, and a loop can't finish before its largest item. Outside the loops there's no large phase: 13% to 21%, spread over the filler's bookkeeping, `FillImagesSolids`' maps and building the result. Only a Boolean with many similar pairs gains much, and there the collector's pauses come first. Splitting a single face/face intersection would be the way past this, which is a large change, and a question for the owner (decision 1718).

**The two cheap things that looked promising don't show.** Each loop gives every other thread a new `IntersectionContext`, as OCCT does, and edge/face items took about three times as long in a parallel pass as in a serial one (22 to 69 µs). Each `Parallel.For` also cost about 0.2 to 0.3 ms, even for loops of items under 1 µs. Keeping a filler's worker contexts across its loops saved under 1% of the bytes, and running cheap loops serially, by a cost for each solver type, about 1%. A pass stayed within the noise, alternating on the loaded machine: 2,491 and 3,968 ms for the base, 2,521 and 2,574 with the contexts, 2,615 and 3,191 with the cheap loops. So neither went in. That the contexts cause the items' slowness is an inference, and not all of it: face/face items, which have no context, took 1.7 times as long too.

| Change | Evidence |
|---|---|
| **`Iterator` and `Explorer` are structs, as in OCCT** (decision 1714). An explorer holds six levels inline, and an array beyond that. A level is four fields, since only the top needs its current shape | Bento **198.4 to 185.5 MB**; simple Booleans 2,461 to 2,206; complex 1,534 to 1,438; STEP import 4,556 to 4,281. An exploration allocates nothing, which a test checks |
| **`Tool.CurveOnPlane` keeps each thread's latest answers** for frozen curves and planes, and returns a copy (decision 1715). Most edges from STEP have no curve on their plane, and each question projected it again | Bento **185.1 to 165.3 MB**; ShapeFixer 2,245 to 1,562; import 4,278 to about 3,700; UnifySameDomain 542 to 455 |
| **`GenExtPS`'s three grids of distances** come from one set for each thread, as each search writes them again (decision 1716); **`TotCross2D`** samples a wire into one sequence | Import about 3,700 to 3,362: mostly `ShapeAnalysis_Surface`'s `ValueOfUV`, which makes a new extrema for each analysis |
| **A wire's points on their way to a 2D classifier,** one sequence for each thread (decision 1716) | Import 3,362 to 3,314 |
| **`FunctionSetRoot`'s search vectors** come from one set for each thread (decision 1716) | Import 3,314 to 3,224. Its 101-element history is most of those bytes |
| **`ProgressScope` is a struct,** with an object only when a range is active (decision 1717). Almost none is, and `DistanceShapeShape` made several for each pair | 1.2% of the many-tool case's bytes; Bento 185.5 to 185.1 |

**Altogether, in bytes:** Bento's recalculation 198.4 to about 165 MB (−17%). Import with healing 4,556 to 3,224 MB (−29%). ShapeFixer 2,448 to 1,502 (−39%). UnifySameDomain 647 to 455 (−30%). Export 2,316 to 2,195 (−5%). The many-tool pair 2,465 to 1,933 (−22%; its first figure is from the build with the temporary timers). Simple Booleans 2,461 to 1,970 (−20%), complex 1,534 to 1,280 (−17%), and the clean after them 204 to 134 and 69 to 46. Every shape's volume, faces and triangles in Bento's run are unchanged.

**What the copy is for.** The first version of `CurveOnPlane`'s table returned the kept curve itself. Ten tests failed, oracle comparisons with native OCCT among them: a caller stored the same object on two edges, and a shape's writers number curves by identity, so the written shape differed. Geometry was the same, but "results stay identical" includes that. A copy costs far less than a projection.

**Where a validity check goes** (a temporary benchmark: the 1,586 faces of the fuse above, 381 ms and 120 MB a check, loaded; only 135 samples, so rough). `FaceCheck.OrientationOfWires` is about a third of its inclusive samples and `FClass2d`, which builds a classifier for each wire, a fifth, and 17% of the bytes. `Analyzer.Put` is about a seventh. The rest is spread over `WireCheck`. Nothing else looked wasteful enough to take, and the sequence above has already helped `FClass2d`.

**`GenExtPS`'s last grid** stays: its `_points` holds positions evaluated once for each object, so it would have to be kept by surface and range, which needs a cache of extrema objects with a hit rate I didn't measure. That's the first thing to try next for healing: about 1.3% of its bytes, and more of its time, as it would save evaluating each grid.

**What's left:** `CurveAdaptor.LoadCore`'s boxed elementary values (3.4% of Bento's bytes; a struct or a variant in the adaptor is a large change), `EdgeCurveAdaptor`, the mesher's collections, and STEP's writer (6% of healing's bytes by `StepWriter.Print` and `SendEntity`). Face/face on free-form surfaces wasn't looked at again.

### The third slice: time, not bytes

*6 November 2026, on a machine at up to 98% total CPU. Bytes and output are exact. Every time here is from a loaded machine: CPU time of one process, alternating base and new builds three times (not ReadyToRun), so a range is what it is worth. The rig's run under that load, with `-Force`, isn't comparable in time, and isn't committed. The clean run comes after merging.*

| Change | Evidence |
|---|---|
| **`TCollection_HAsciiString` holds a string, not a builder** (decision 1727), and **STEP's writer** stops going through `Sprintf` for identifiers and integers, looking up each name in the model, rebuilding text with nothing to clean, and making a byte array for each line; the line buffer is an array (decision 1728). `EntityGraph.Evaluate` keeps one iterator, and `FreezeGraph` skips its walk when the children are frozen (decision 1729) | **Export 2,212 to 1,148 MB** (the rig; −48%). CPU a round 3.0 to 3.5 s before, 1.85 to 2.4 s after: **about a third less**. Each of the 289 files' output is the same, by digest with the header's time removed |
| **`GenExtPS`'s search reads its grids by reference** (decision 1730). Its elements are 64 bytes, copied several times for each cell; OCCT's are references | **Import with healing 6.4 to 7.6 s to 5.3 to 7.0 s of CPU: about 10% to 15% less.** `BuildGrid`'s inclusive share 18.9% to 9.8%. 3,220 to 3,127 MB (the rig). The commit message's 4.75 s was one lucky pair of runs; this table is the figure |
| **`BrepTopolTool` loads its restrictions when first asked** (decision 1733) | **Fillet every edge 85.5 to 73.4 MB, chamfer 86.9 to 75.0** (the features suite; −14%). Times too noisy to read here (fillet 170 to 95 and 152 ms) |
| **`Bfgs`'s matrix update over spans** (decision 1731): the same arithmetic in the same order | `bug24905` 8.7 to 8.2 s of CPU in the whole case, including its other changes and start-up, so small |

**The export row's 5% rise in the last run** (1,801 to 1,894 ms): noise, most likely. It's within the 10% this kind of row moves between runs, the row's own two runs differed by 7%, and that run's bytes fell. That's inferred; it wasn't timed again on a quiet machine.

**Why a name was slow, as far as I can tell.** `RWCartesianPoint.WriteStep` showed 21% of export's samples, 77% of them in `GetHashCodeWorker`, where the code asks the model for the number of an entity's name. Each name is a new object, so each lookup gives it its first hash code, which leaves managed code. Decision 1706 says not to trust leaves, and this looks like the "unexplained" hash code samples of the earlier slices, so it isn't proof. But the case's share fell to 4.7% once the lookup went, so this one was at least partly real. Other work asking for first hash codes may be paying the same.

**What the rest of the list came to:**

| Item | What's there | Verdict |
|---|---|---|
| **Import, what's left** | Of import's samples, 67% is shape processing, 24% adding pcurves (`FixAddPCurve`), of which `ValueOfUV` and `NextValueOfUV` are most; `FixSelfIntersection` 12%; parsing and graph 19% | Hard in the main: points projected one by one |
| **Face/face, free-form** (`bug24905`) | Of `FaceFace.Perform`: fitting 73% (`Bfgs`, then least squares, each iteration a new solve), `ComputeTolReached3d` 15% (a particle swarm along each curve), the surfaces' intersection 11% | **Hard: the geometry.** The swarm's evaluations are the work |
| **Edge against free-form face** (`buc60555_3`) | `BeanFaceIntersector` 10% of samples, face/face 12%. **47% of the bytes are `GenExtPS`'s work grids,** 151 sets, the largest 41 by 1,641 and about 13 MB, as anything over 4,096 cells isn't kept (decision 1716) | Bytes only; time not measured. A larger or weakly held spare could take most of it |
| **`GenExtPS._points`** | Evaluating the grid is now about 2% of import's samples, and the array 5.6% of its bytes | **Not worth a cache of extrema objects** (decision 1732), though the second slice put it first |
| **Boxed adaptor values** | 3.4% of Bento's bytes, unchanged | Left: allocation only, and a large change |
| **Validity checks** (`Analyzer`, 289 shapes, 2.1 s of CPU, 1,174 MB) | `WireCheck.SelfIntersect` 26%, `FClass2d` 19%, each doing real 2D work. Curve caches rebuilt 7% | **Spread out; nothing to take.** Unchanged by this slice |
| **9, fine meshes** (`bug29715`) | Nearly all in `OptimizeMesh`, the deflection loop: Delaunay insertions and splits | Hard, confirmed |
| **12, pipes with corners** (`bug32444`) | All in the corners' pave filler, as listed; 46 s on the list, 25 s here. `FunctionSetRoot.SearchDirection` is 17% of its bytes | Hard, confirmed; a small allocation cut is there |
| **13, 2D offsets** (`bug6768`) | **Not entirely hard:** 54% of the bytes are `Polygon2dGen`, a polygon of each curve for each pair in `TrimEdgeTool.IntersectWith`, and evaluating curves is 13.5% of samples. 5.7 s here, 44 s on the list | The same shape as decision 1707's polyhedra. A polygon per curve might save a good part. Not tried: `IntCurveCurve`'s interface would have to change |
| **11, every edge** (the features suite) | **Not entirely hard:** `ChFi3d.IsTangentFaces` was 10% of the suite's samples and 21% of its bytes, four fifths of that in `BrepTopolTool`'s constructor, which built an adaptor for each edge of the face when only a number of samples was wanted. Offsets: `ConnexIntByArc`, 29%, is intersections, and `InitSphere` 12%, with no single waste | Fixed for the fillets (below). Offsets hard |
| **10, 14: max fillet, hidden lines** | Not traced in this slice's time box | Still marked hard on the earlier evidence only |

**For the owner.** Nothing here needed a large restructuring. The one that might pay is below; none of the items above is a question that blocks anything.

- **A curve's polygon kept across `IntCurveCurve` calls,** for 2D offsets (item 13). Expected gain: most of half the bytes of those cases, and some of their time; only 2D offsets would notice. A moderate change, not a large one.

### The fourth slice: re-ranked, and the mesher's own time

*6 November 2026, on the same shared machine. Bytes, collection counts and call counts are exact. Every time here is from a loaded machine: processor time of one process, alternating base and new builds, so a range is what it is worth. The clean run comes after merging.*

**The list above is re-ranked from one new pass of the corpus** (`KERF_CORPUS_TIMES`, 17,872 cases, 8,435 s of case time, six at a time, at the head before this slice). The detailed log of the last full corpus has statuses but no case times, so it couldn't rank anything. The harness is still the largest thing in it: `lowalgos/extcs/circ_sph_*` are 918 s, the chamfer sequences 183 s, and `bug24157_5` (51 s) spends 0.3 s of processor time on its one thread that shows, nearly all in Draw-lite (decision 1710). The `hlr/exact_hlr` and `pipe/specific` grids mostly skip for missing data, so their times say nothing. **Bytes per case in that file are of no use:** a case runs on another thread than the one that reads the count.

| Change | Verdict | Evidence |
|---|---|---|
| **The mesher's circles keep their cells in a flat array** (decision 1756). Binding a circle looks up each cell it covers, and a circle can cover thousands | **Wasteful** | `IterateAdd` was 53% of `CreateTrianglesOnNewVertices`' samples on `bug27453`. Time: the row below |
| **The insertion loop goes back only as far as a deleted triangle's neighbours,** not to the first circle, and `CircleInspector` is a struct (decision 1757) | **Wasteful, and in OCCT too** | `bug27453`: 23,217 nodes had 3.8 million circles between them, 164 each, and the restarts made 126 million steps, half of them over circles already removed. Those scans are gone. **With the row above, first and last builds alternating: `bug27453` 7.7, 8.9 s to 7.1, 8.2 s, about 8% of the whole case including its import; meshing 284 STEP files 77 to 98 s to 77 to 84 s, too noisy for more than "about a tenth"; `bug25503_1` and Bento's examples no visible change.** Runs of single builds looked better (15% and 10% for `bug27453`), but the machine's load moved between them, so those aren't claims |
| **A thread's lists of triangles and circles, kept between faces** (decision 1759: the time-boxed experiment) | **Worth having** | Bento 155.8 to 143.7 MB (−8%), generation 2 collections over a 30-pass run **60 to 14**, CPU no different (30.4, 27.7, 28.7 s against 29.8, 27.9, 28.6). The rig's meshing rows about −20%: Motor-c 114.6 to 91.4 MB, bottle 32.8 to 26.2. The triangles of 284 shapes are the same, by digest. A very large face gains a little: `bug25503_1` 295 to 303 MB, as its list of triangles starts small |
| **`ApproxAFunction`'s coefficients and `BSplCLib`'s basis matrix, one for each thread** (decision 1758) | **Wasteful** | Max fillet on a box: **577 to 419 MB, generation 2 collections 30 to 7, about 20% less CPU** (1.6 to 1.9 s, 1.3 to 1.4 s), the same radii. Each same-parameter made 96 kB on the large object heap: 1,000 segments × 12 coefficients, of which few are used. No effect on the rig's fillets, which don't reach it often |
| **`GenExtPS` holds a larger set of grids weakly** (decision 1754) | **Wasteful in bytes; little time** | `buc60555_3` 1,546 to 952 MB (−38%), generation 2 collections 81 to 18, CPU 7.7 to 8.3 s against 7.4 to 8.1 s. Its work grids took about 2% of samples, so the time was never there |
| **A 2D offset shares each curve's first polygon** (decision 1755) | **Wasteful in part** | `bug6768`: 64,512 polygons were 14,616 different ones, and 4.76 million points 1.74 million. 649 to 487 MB (−25%), collections 31 to 6, CPU 5.3 to 5.8 s against 5.1 to 5.3 s: about 8% less. Bisector and intersection work is the rest |

**Max fillet and hidden lines, traced at last.**

| Case | What's there | Verdict |
|---|---|---|
| **Max fillet** | A binary search of about ten real fillets and validity checks, each costing what a fillet costs: `ThreeCorner`'s plate surfaces 35% of samples, `SameParameter` 22% (47% of the bytes), validity checks 20%. The search can't start narrower without changing its answers | **Hard by design,** with the wasteful part above taken out |
| **Hidden lines** (`images`, 30 passes) | Of `ProjectLines`: contours 32%, hiding 24%, the rest spread. Nothing above 10% that barely needed to run. `MapShapesAndAncestors` 3.5% | **Hard,** spread out |

**Other cases traced from the new list.** `bug26701` (29 s): 21% of samples in `ComputeVF`, of which `FClass2d` 14 points and `UVBounds` 9, because a parallel loop's new context for each thread rebuilds them, as decision 1718 found on other cases; and 53% of its bytes in validity checks, mostly `Placement`. `bug32944` (21 s): a particle swarm for each conic against each surface (`GlobMinConicS`, 83% of the Boolean). `buc60463` (25 s) and `bug32444`: `FunctionSetRoot` over and over; a third of `buc60463`'s bytes are `GaussLeastSquare` and its transposes, in OCCT too. `offset/shape_type_i_c_multi/B7`: Booleans inside the offset. All hard, or need a larger change.

**What was tried and left out.**

- **The mesher's link map as chunks of structs** (decision 1760). Bento 160 to 156 MB, the mesh harness 7,259 to 7,122 MB, but CPU in three alternating pairs was as likely worse (29.8, 28.2, 26.7 s against 31.1, 28.9, 29.6). Not plainly no slower, so it isn't in. The same digests.
- **`FunctionSetRoot`'s solvers for each thread.** `buc60463` 1,539 to 1,532 MB and nothing in time: the bytes are the least-squares branch, which makes its own.

**The mesher's cost that's left** is in the cell lists: on `bug29715`, 142,518 nodes inspected 174 million circles, 1,222 a node, for 4.8 hits each. OCCT has the same grid. A finer grid, or a walk from a neighbouring triangle, would change the order circles are found in, and with it the order of triangles and perhaps the mesh, so that's a question below and wasn't tried.

**For the owner.**

- **Find a node's circles by another route, or on a finer grid, in the mesher.** Expected gain: most of what's left in `CreateTrianglesOnNewVertices`, which was 48% of the samples of meshing 284 STEP files (about 85 s of CPU, base); perhaps a quarter off large meshes, little on small ones (inferred from those samples, which decision 1706 says to read with care). Meshes would stay valid but wouldn't be identical to today's or OCCT's: triangle order and possibly some triangles change. A large change, and the only thing on this list that gives up "identical".
- **A polygon of each curve for 2D offsets** is now done, so last slice's question has gone.

### The fifth slice: what threads keep, and down the list

*6 November 2026, on the same shared machine. Bytes, collections and retained memory are exact. Every time here is from a loaded machine: processor time of one process, alternating builds, so a range is what it is worth. The clean run comes after merging.*

#### What the per-thread sets hold

`Kerf.Kernel.Bento.Benchmarks -- thread-sets` runs three parallel rounds of Bento's eleven examples, 24 copies of each on 24 processors, then has every thread set each `[ThreadStatic]` reference to null, one field at a time, with a full collection after each, and reads what that lets go. So this is what a build without the sets would hold, measured in one process. The machine's load makes the pool grow past 24 threads, 37 to 52 here, so a figure a thread is the one to carry to another machine.

| Set | Limit before | Held after the rounds, before | Limit now | Held now |
|---|---:|---:|---:|---:|
| The mesher's lists | 65,536 values, about 2.9 MB (the code's 4 MB was too high) | **1,280 kB a thread** | 8,192, about 360 kB; larger arrays weakly | **352 kB a thread** |
| `ApproxAFunction`'s coefficients | 1 MB | 187 kB, on threads that used it | 64 kB; larger weakly | 0 (all are larger, and weakly held) |
| `CurveOnPlane`'s 64 answers | 64 | 18 to 23 kB | 64 | the same |
| `Class2dPoints` | 16,384 points, 260 kB | 8 kB | 4,096, 65 kB | 8 kB |
| `BSplCLib`'s matrices | one for each shape | 6.5 kB | the same | the same |
| `FunctionSetRoot`, `Bfgs` | 16 unknowns; 512 unknowns, about 2 MB | 1.8 kB, 1 kB | the same; 64, about 36 kB, larger weakly | the same |
| `GenExtPS`'s grids | 4,096 cells, 800 kB | none here | 1,024 cells, 200 kB; larger weakly | none here |
| `MeasurementGauss` | about 500 kB | none here | only weakly | none |
| `QuasiAngularConvertor` (new, below) | | | about 1 kB | none here |
| **All, measured** | | **72.4 MB of 76.7 MB live (51 threads); 54.2 MB with 37; 58.6 MB under the server collector with 46** | | ****20.0 MB of 24.5 MB live (53 threads); 14.0 MB with 37; 17.8 MB under the server collector with 47**** |

- **So the owner's estimate was about right, and nearly all of it was the mesher,** which bytes alone had hidden: decision 1759's limit was reached by doubling on one large face, and then stayed for the thread's life. Nothing else passed 7 MB.
- **What a thread can hold at most is now about 760 kB** by adding the limits (360 + 200 + 65 + 64 + 36 + 8 + 7 + 1 kB, and `CurveOnPlane`'s 64 answers): 18 MB on 24 threads, against about 7 MB a thread before. That's inferred from the limits; Bento's examples don't reach most of them. **`CurveOnPlane` is the exception to a fixed limit:** its answers keep their curves and planes alive, so a large B-spline there counts in full. That's 64 at most, and they aren't shapes.
- **The pool's own threads let go:** 40 s after the rounds the pool had **no threads, and only 0.37 MB was left in sets,** as it lets an idle thread go after about 20 s, its sets with it. Bento's calculation thread and its what-if workers are its own (`CalcEngine`, `VariantPlan`), and a calculation thread lives as long as the workbook: for those the figure a thread above stands.
- **Why a weak reference to the array, not to the list.** `WeakReference<MeshWorkStorage>`, as `GenExtPS` holds its grids, took Bento's generation 2 collections over 30 passes from about 22 to about 48: the young list object dies in the next generation 0 collection, while its arrays wait on the large object heap, so most large faces found nothing and each grew its own. An array on the large object heap only goes in a generation 2 collection, so a weak reference to the array itself holds for as long as it can, which needed the small `MeshWorkList` in place of `List<T>`. A limit of 4,096 was too low (35 to 41 collections, +2.4% in bytes); 8,192 costs about 1%.
- **What the lower limits cost, each against what the set was added for:** Bento 143.8 to 144.8 MB before and 144.3 to 147.0 after, about 1%, with 17 to 24 generation 2 collections before and 23 to 29 after; meshing 284 STEP files 6,703 to 6,699 MB, 12 to 11, **the same triangles by digest**; `bug25232_9` 10,269 to 10,230 MB; `buc60555_3` 953 MB; `bug24905` 952 MB; max fillet 420 MB and the same radii; import with healing 3,110 to 3,117 MB. Processor time showed no difference in alternating runs.
- **Left as it is:** `GenExtPS`'s larger grids and `Bfgs`'s larger set are weak references to small young objects, so they die sooner than the arrays would. They still do their job, as above; holding the arrays instead is only worth it if a case shows it.

#### Down the list

| Case | What's there | Verdict | Before and after |
|---|---|---|---|
| **`GaussLeastSquare`'s bytes** (`buc60463`) | `FunctionSetRoot`'s over-determined branch built a new solver, and a transposed copy to copy from, for each direction, where its `Gauss` and `Svd` branches reuse theirs (decision 1764) | **Wasteful in bytes** | **1,540 to 1,154 MB (−25%)**, generation 0 collections 93 to 67, processor time 10.3 s either way |
| **`bug26701`'s contexts** | Worker contexts kept for a filler's life, across its loops, tried on the case decision 1718 didn't: 2,387 to about 2,320 MB (−3%), and processor time no better in alternating runs (19.8, 19.1 s against 21.7, 19.1 s). So the 21% in `ComputeVF` is mostly classifiers that one context needs once anyway (`FClass2d`), and the rest of its bytes are validity checks | **Not the contexts; spread out.** Not in (decision 1767) | |
| **STEP import** (`de/step_2/S9`, `de/step_4/E4`) | S9: `WireAnalysis.CheckIntersectingEdges` was 40% of its active samples and 36% of its bytes, polygon against polygon. A new sequence of section points for each pair of segments with close boxes, which nearly always stays empty, and a box for every pair however far apart, which one polygon's search stopped doing in the first slice (decision 1765). E4: projecting points (`EdgeProjAuxFixer` 14%, `CurveAnalysis.Project` 15% of active samples), reading 23% | **S9 wasteful in part; E4 hard** | S9 **913 to 812 MB (−11%)**, `Interference` 4.2 to 2.3 share points; 6.1 to 6.8 s to 6.0 to 6.6 s of processor time, so no more than a few per cent |
| **Many tools** (`bug25614_genfuse`, `boolean/simplify/A3`, `bug25232_9`) | No site over 3% to 5% of the bytes in the first two. By active samples: edge/edge, edge/face, face/face, the builder, each 15% to 20%; `FClass2d`'s init about 10% of A3, walking real wires. In `bug25232_9`, **26% of its wall time is the collector's** (7 of 26.6 s, a large live heap), and a tenth of the Boolean is `IsExistingPaveBlock`, 90% of that building a tree of pave blocks for each pair of faces, as in OCCT | **Spread out.** The tree needs a larger change: below | |
| **Edge against free-form** (`bug32944`, `bug29182_1`) | `bug32944`: `GlobMinConicS` is 83% of the Boolean, the swarm's evaluations. 52% of its bytes are a new `PsoParticlesPool` for each conic, but with 0.03 s of pauses that's no time. `GetWorstParticle`, a scan for each accepted grid point, is about 7% of the case. `bug29182_1`: spread over edge/face and edge/edge | **Hard.** A pool for each thread would be one more set for no time, so it isn't in | |
| **Free-form face-face** (`bug30760`, `bug28119_1`) | `bug28119_1`, in fact a fillet: **31% of its bytes were `new QuasiAngularConvertor()`** for each circle of each section, a matrix and six vectors, when the class is made to be initialised once. Its time is in fitting (decision 1766). `bug30760`: face/face and edge/face over real surfaces, and `CheckerSI` | **The convertor wasteful in bytes; the rest hard** | `bug28119_1` **3,632 to 2,420 MB (−33%)**, generation 0 collections 153 to 78, processor time 9.9 to 10.5 s against 10.0 to 10.1 s |

**Large meshes and `Analyzer` were left alone,** as the brief asked. **Nothing here moved processor time past the noise,** and the bytes are what changed: the time on these cases is real work, or spread thinly. Decision 1734's measure is time first, so that's the finding: the next slices should expect less.

**The rig, quick, `-Force`, against [the last clean run](benchmarks/2026-11-06-b21606ad7.md), so including the fourth slice; times aren't comparable.** Every result is the same. Bento 160.1 to 145.3 MB, a parallel round 3,825 to 3,500 MB, simple Booleans 1,967 to 1,951 MB. Peak working set in the parallel rounds 212 to 251 MB, taken under load with more threads, so it needs the clean run to read.

**For the owner.**

- **A tree of pave blocks for each face rather than each pair of faces.** Expected gain: most of a tenth of `bug25232_9`'s Boolean, inferred from samples, and less on most others. Hits would come out of the tree in another order, so which of two equally close blocks is found could differ, unless hits are sorted, which is OCCT's order only by accident. A moderate change that gives up "identical", so I wouldn't, unless identical goes anyway for the mesher's question above.

### The sixth slice: what stays in memory, and the peak

*6 November 2026, on the same shared machine. Bytes kept are exact: they're each object's size from its method table, walked from the result, and the collector's own count agrees (below). Peaks and every time here are from a loaded machine, so a range is what it is worth. To repeat it: `Kerf.Kernel.Bento.Benchmarks -- kept`, `kept-types [name]`, `chain [cells]`, `step files... [same]`, `peak file [mesh]`, `statics file` and `digest files...`, and `Kerf.Kernel.Benchmarks -- step-phases file`.*

Earlier slices counted bytes allocated in passing. A workbook is different: it keeps a shape and perhaps a mesh in every cell, for hours, so what limits it is what stays alive and the peak.

#### What a kept result costs

| Kept | Faces | Before: shape, KB | Now | Per face now | Mesh, KB | Per triangle |
|---|---:|---:|---:|---:|---:|---:|
| 3 prismatic solid | 7 | 16.2 | **11.4** | 1,663 B | 13.3 | 51 B |
| 2 plate with hole | 7 | 28.6 | **22.3** | 3,265 B | 13.4 | 51 B |
| 9 chamfer, fillet | 14 | 91.0 | **75.8** | 5,547 B | 19.4 | 57 B |
| 29 bottle | 56 | 146.3 | **117.1** | 2,141 B | 282.5 | 30 B |
| 31 nesting locations | 164 | 343.3 | **154.5** | 965 B | 58.3 | 210 B |
| 32 for-loop | 44 | 93.6 | **52.1** | 1,214 B | 16.9 | 187 B |
| 33 function, for-loop | 36 | 81.3 | **43.3** | 1,231 B | 13.9 | 197 B |
| STEP `bug30790` (21 MB file, 2,086 B-spline surfaces) | 2,337 | 19,333 | **12,281** | 5,381 B | | |
| STEP `bug32239` (14 MB) | 2,004 | 15,532 | **12,943** | 6,614 B | | |
| STEP `tr9_r1001-ec` (7 MB) | 918 | 5,008 | **3,649** | 4,071 B | | |
| STEP `bug33261` (4 MB) | 4,050 | 8,487 | **6,315** | 1,597 B | | |

**What it's made of now,** `bug33261` and the bottle: topology 32% and 39% (`TShape`s, their child lists, `Shape`s), representations 24% and 37% (curves on surfaces, each with its 2D curve), geometry 41% and 23%, and placements, the rest, under 3% unless the shape is patterned (31% of "31 nesting locations"). In `bug30790` geometry is 71%, mostly poles: real data. A mesh is 26 to 57 bytes a triangle, tight floats and indices with nothing spare; with few triangles the small `FaceMesh` and `EdgeMesh` objects dominate, hence 210 B.

**The books close.** The walk follows instance fields only, so if something else held on to a result, the collector's count would show more than the walk. `bug30790`: 12,281 KB walked, 13,027 KB by the collector on the first import and within 0.3% on later ones. Bento's examples agree within what the thread's sets grow by, and the gap shrinks with more copies (the bottle: 605 KB a copy with 16 copies, 479 KB with 64, against 429 KB walked).

**Looked for, and not found (measured):**

- **What only the operation needed:** no overlay, map, adaptor, context or cache of any kind is in what a result holds. The types kept are topology, representations, geometry, placements, the mesh's own, and the record.
- **The validity check's results:** nothing keeps them. No type of unit 11R's is reachable from a kept result, and on success its report is dropped.
- **Mesh data held twice:** it isn't. The mesher works in an overlay, so a kept shape holds no triangulation of its own, only (until now) an empty list for one.
- **A STEP import's entity model:** gone after the transfer. Import and drop `bug30790` in a method of its own: **0.74 MB left** for good, tables made once. A later import leaves 2 to 55 KB. An early reading of 26 MB was this harness's own: an unoptimised method's stack slot holding the last result.
- **A record holding a chain:** it doesn't. A record names the inputs of its own step, not their records: the last of 30 cells holds 146.7 KB with its record and 131.0 KB without, against 473.6 KB for the whole chain.

**Chains of cells:** a chain of 30 features where each cell keeps its step.

| Chain | Shapes held, before | Now | Without the records | Each alone would be | A cell adds | Faces shared with earlier cells | Meshes, if every cell's is kept |
|---|---:|---:|---:|---:|---:|---:|---:|
| A hole in a plate | 0.64 MB | **0.46** | 0.29 | 2.16 | 19.8 KB, 5.7 of it the record | 71% | 4.96 MB |
| A boss on a plate | 1.10 MB | **0.88** | 0.55 | 6.06 | 34.3 KB, 11.3 the record | 87% | 1.09 MB |
| Fillet each new hole | 0.81 MB | **0.58** | 0.37 | 3.16 | 26.8 KB, 9.8 the record | 83% | 27.8 MB |

So **unchanged faces are already shared: a chain holds a fifth to a seventh of what its cells would alone.** A record is about a third of what a cell adds, and callers can ask for it (`Select.New`, `Select.Last`), so it stays. **What a chain doesn't share is its meshes:** they're 55% to 98% of everything held once every cell has been shown, though most of their faces are the same faces.

#### What was wasteful, and what changed

| Case | Verdict | Before and after |
|---|---|---|
| **Identical knots and multiplicities on every B-spline** (decision 1841). `bug30790` held 24,873 arrays of knots with 459 different contents, and 11,080 of multiplicities with 11: 38% of all it kept | **Wasteful** | Frozen splines take a shared array from a fixed table. **`bug30790` 18,357 to 12,483 KB (−32%),** `bug33261` 7,398 to 6,603 (−11%), on top of the changes below; with them, `bug30790` −36%, `tr9_r1001-ec` −27%, `bug33261` −26%, `bug32239` −17% |
| **A list inside a list** (decision 1839). Each `NCollection_List` was two objects and an array, and each TShape and representation has one or more: 9% of `bug33261` | **Wasteful** | An array and a count. `bug33261` 8,487 to 7,914 KB, and in passing: every list in the kernel is one object fewer |
| **Room for growing, and empty lists, on frozen shapes** (decisions 1839, 1840). A list starts with room for four; a face has one wire and an edge two vertices. A vertex has an empty child list, and a face an empty list of meshes | **Wasteful** | Trimmed as a TShape freezes, and an empty one is one shared list. `bug33261` 7,914 to 7,398 KB for the trimming, with the placements; about 4% more on it for the empty lists. No more allocated: Bento 155.1 to 154.8 MB |
| **The same placement chain built over and over** (decision 1842). Placing a sub-shape multiplies two chains, a node of about 170 bytes each time, so siblings each had their own: 1,158 nodes for 137 distinct chains in "31 nesting locations", 47% of what it held | **Wasteful** | A thread remembers its last four products. **"31 nesting locations" 343 to 154 KB (−55%), the for-loops −44% and −47%** |
| **A STEP parse's parameters** (decision 1843): an object for each of 1.3 million, in a list grown by doubling though the count was known, each with its own string, while the parser's records stayed until the end | **Wasteful** | Values in one array of the right size; a record goes as soon as it's been read; a short token seen recently shares its string. **Largest live data in an import 178 to 110 MB;** peak below |

**Results are identical, measured, twice:** one pass of the corpus on each build, all 17,872 cases: every status is the same (OK 10,478, BAD 502, FAILED 59, IMPROVEMENT 8, skipped 6,825). And the BREP text and every byte of the mesh, hashed, for all 336 STEP files of the dataset and Bento's eleven examples, before and after (`digest`): no differences, and the same 4 files stop with the same exception in both. **No history or lazily built data was dropped:** nothing was removed from any result. What changed is whether equal, immutable data is one object or several, and each of those was compared bit for bit or is a pure function of what it's built from. So a caller can ask for everything it could before, and `History` is untouched.

**What's shared between threads, and how that's known to be safe.** The table of arrays (`SharedArrays`) holds only arrays of objects being frozen, which are never written again. The guard on writing is the one every frozen object has: `CheckMutable` is the first line of every mutator, and a test has a copy of a frozen spline changed with the original's knots unmoved. A view of another array's storage can take a shared array but isn't offered, unless it's the unit weights, which nobody writes. A slot is one reference, read and written whole, so a race loses an entry and no more. The shared empty list refuses to grow, by a check on its one path to growing, and has a test. `ProductMemo` is per thread.

**What they can keep alive:** the table, 1,024 arrays of up to 32 values for each of two types, about 300 kB of doubles and 180 kB of integers at most. `ProductMemo`, four chains a thread, a few kB unless a chain is very long. Both are in [state.md](state.md).

#### The peak

Three runs of each, alternating, on a loaded machine.

| | Allocated | Largest live | Peak, workstation collector | Peak, server collector as Bento has it |
|---|---:|---:|---:|---:|
| Import `bug30790` | 2,350 to 2,213 MB | 178 to 110 MB | **415 to 291 MB** | **401 to 320 to 376 MB** |
| Import `bug32239` | 1,476 to 1,379 MB | 95 to 96 MB | 260 to 214 MB (one run each) | 342 to 319 to 367 MB: no change to read |
| Parallel round of Bento's examples, 24 copies | 3,684 to 3,669 MB | | **275 to 244 MB** (two runs each) | 420, 452 to 372, 427 MB (two runs each) |
| Import and mesh `bug30790`, about 6.7 million triangles | 9,743 MB | 816 MB | | 1,637 MB, before: not touched, unit 11O's |

The last parallel round's 212 to 251 MB was the load, not a change: 251 sits among these "before" and "after" runs, whose difference is real because they alternate under one load. The clean run will say what the figure is now. After a round, 23 MB is live, 18.7 MB of it threads' sets, and a round keeps nothing, so the rest of its peak is what the collector lets build up (inferred from those two figures).

**What holds an import's peak up, walked at each stage of `bug30790`, a 21 MB file:**

| Stage | Live | What it is |
|---|---:|---|
| The parser's records | 109 MB | 1.3 million `Argument`s 49 MB, strings 37 MB, 420,000 `Record`s 22 MB |
| The reader's parameters | 57 MB | Strings 29 MB, one array of values 20 MB, tables 8 MB |
| The entities, with the parameters still held | 111 MB, adding the two walks | Entities 54 MB: the graph's lists 15 MB, points' coordinates 7 MB and the points 7 MB, names 6 MB |
| After the transfer, the reader still held | 87 MB | Entities and binders, and the shape |
| After | 12 MB | The shape |

So **the live peak is the file's text, parsed twice over, then its entities: five times the file.** Above that, the collector: the workstation collector's peak is about 2.6 times the largest live data, and the server collector's about 3 to 3.5 times, set by its budgets rather than by live data, which is why its peak moved so much less and so unevenly. **The large object heap is 34 to 64 MB at the peak, a fifth,** and it's garbage: `POnSurfParams[]` (`GenExtPS`'s grids) 256 to 287 MB over the import, of up to 0.47 MB each. **The largest single allocations:** the parameters' one array (19.7 MB, from two arrays of 8 and 16 MB of references), the entity table's `Entry` arrays (7.4 MB), and the meshed case's `Int32[]`, 80.6 MB, the whole mesh's triangles.

**The host's settings stay as they are.** Nothing here says otherwise: the server collector's 320 to 376 MB against the workstation's 291 is the price already agreed in `bento-integration.md`, for a one-off import.

#### Tried, and not in

- **`GenExtPS`'s larger grids held by their arrays, of any size large enough** (the fifth slice's "only if a case shows it"; this looked like the case): 287 to 256 MB of them and no fewer collections. Most of those arrays are `_points`, the search's own grid, which lives as long as its object, and an extrema object has no end of life to give it back at. Not in.

#### Back to speed: the list again

From one more pass of the corpus on this build (`KERF_CORPUS_TIMES`, six at a time, 5,144 s of case time, loaded; the base's pass took 7,906 s under a heavier load, so the two say nothing about each other). The list above still stands. Its order moves little, and neither the validity check (21% of `A6`, the most seen here) nor the precise path puts a new case at the top by itself: inferred for the precise path, which I didn't profile apart. **New names at the top, each profiled:**

| Case | Time in the pass | What's there | Verdict |
|---|---:|---|---|
| `bugs/modalg_5/bug24157_5` | 48 s | **Half this harness:** 42% of its active time is Tcl making a list of 8,100 edges, one `lappend` at a time, and the Boolean 44%. 20 s alone | Not Kerf |
| `boolean/splitter/A6`, `B6` | 26 s, 23 s | Spread: of active time, the filler 33%, the check 21%, solids 18%, and threads stopped for collections about a third of it overall. About 6 s alone | Spread out |
| `boolean/volumemaker/H1`, `C6` | 21 s, 17 s | `CheckerSI`, the corpus's own self-intersection check, 55%: real face-face work | Hard |
| `bugs/modalg_4/bug77_1`, `_2` | 19 s, 17 s | `CheckerSI` again: walking between B-spline surfaces | Hard |
| `bugs/modalg_5/bug25614_section` | 19 s | Booleans of many tools, item 3: the filler 81% of active time, among face-face 27%, blocks 25%, edge-edge 16% | Spread out |

**Nothing here is wasteful:** as the fifth slice said, the next slices should expect less. `A6` takes about a quarter of its time here when it runs alone, and `bug24157_5` about 40%, so load and the collector are much of what a corpus pass ranks. The slowest kernel cases otherwise are as listed: `bug32444` 35 s, `bug26701` 34 s, `boolean/simplify/A3` 29 s, `bug28119_1` 29 s, `bug25232_9` 24 s.

#### For the owner

- **Share meshes between cells, by the face.** Expected gain: most of a chain's meshes, which are 55% to 98% of what it holds, 71% to 87% of whose faces are unchanged, and the time to mesh them: in a chain, only new faces would be meshed. It's a larger change: a mesh would have to be the same whichever shape a face is in, and today a face's mesh depends on the whole shape's tolerance and on edges shared with its neighbours. Bento can already get much of the memory by not keeping models off screen ([11q-bento.md](../../modeling/docs/11q-bento.md)), and unit 11O may change the mesh's storage anyway, so I'd take this up there.
- **One pass from the parser into the reader's data.** The parser's records are copied into the reader's parameters, and the first stage above is the import's largest. A moderate rewrite of `StepFileReader` and `ReadData`, where the sizes the reader needs are the only reason for two passes. Expected gain: the first stage goes, but the third stays, so live data falls only if the entities shrink too (the graph's lists, a point's coordinates, names): together perhaps 110 to 80 MB, inferred, and the peak in proportion on the workstation collector. Only for large imports, so I wouldn't yet.

### The seventh slice: the modelling layer

*7 November 2026, on a machine shared with another helper: every time here is from a loaded machine, alternating fresh processes, ReadyToRun, and the clean timed run follows. Bytes don't depend on the load, though a whole-process figure moves by about 2% between runs. `KERF_BENTO_ONLY` runs some examples alone.*

**The layer itself costs almost nothing, and that's the finding.** Measured by inclusive shares (decision 1706), attributing each sample or tick to the nearest Kerf frame:

| | Layer's own bytes | Layer's own samples |
|---|---:|---:|
| Bento's eleven examples, all threads | 0.57% | 0.68% |
| The same, the main thread only | 0.94% | 1.85% |
| "For-loop" alone | 1.02% | 1.45% |
| A harness of the modelling tests' common operations: selectors, ordering, `Center`, `Bounds`, `Area`, moves, sketches, extrusions | 2.79% | 2.94% |

The wrappers, conversions, selectors and `Explore` are all small. No site of its own passes 0.42% in the harness, or 0.08% in Bento's examples. A pass of the test suite couldn't be traced, as the test host is stopped before its trace closes. What the layer *asks for* is what costs, so that's where this slice looked: of the main thread's bytes and samples before this slice, the Boolean 21.8% and 18.6%, the mesher 19.4% and 32.5%, validity checks 5.6% and 3.3%, cleans 6.6% and 3.1%, measuring 5.3% and 9.7%, optimal boxes 3.6% and 7.3%.

**Work done twice or for nothing:**

| Found | Verdict against the bar | Before, after |
|---|---|---|
| **A volume of planes needs no integration** (decision 1871). A planar face's part is its height over the reference point, times its area in UV, over 3, and the area is closed-form for lines and circles. Curved faces are still integrated | Time: **clears it on small solids** | Measuring 9.7% to about 3% of the main thread's samples. Most of "for-loop"'s time below, with the next row |
| **An extrusion integrated each face for a plane's origin, and read only its normal** (decision 1872) | Part of the above | 1% of "for-loop"'s main thread |
| **A spatial tree's search made a 64-slot stack each time, 536 bytes, and the mesher's check searches once for each segment** (decision 1874), and **each filler made and seeded a Mersenne Twister** of 2.5 kB | Bytes, with the rows below | 3.2% of the bytes gone |
| **The mesher's lists of links** were an object for each link at each node (decision 1875) | Bytes, with the rows below | 4.4% of the bytes gone |
| **The cells of the circles** grew a new array for each face (decision 1875) | Bytes, with the rows below | About 3% of the bytes |
| **The map of links** was an object for each link (decision 1876, which reverses 1760) | Bytes | 127.2 to 123.1 MB, 263 to 259 ms; 250 to 245 ms on one processor |
| **A set and two lists grown from nothing where the size was known** (decisions 1874 and 1875) | Bytes | About 1% |
| **Boxed adaptor values, 3.4%** (decision 1873): the geometry object already holds them | **Kept**, in about 10 minutes of the 45: plainly no slower | 149.5 to 144.9 MB (3.1%), 263.5 to 258.4 ms, processor time within noise, the same digests |

**All together, on the owner's bar: a success by both of its tests.**

| Loaded machine | Before | After | |
|---|---:|---:|---|
| **Recalculation, 8 alternating pairs** | 369.6 ms | 295.0 ms | **-20%**; whole-process CPU 36.1 to 33.9 s, -6%, with the pool threads' spinning in it |
| Allocated per recalculation, the same pairs | 151.3 MB | 124.7 MB | **-18%** |
| "For-loop", 6 alternating pairs | 19.4 ms, 10.5 MB | 14.6 ms, 9.2 MB | **-25%**, -12%; the night's clean run was 17.8 ms and 10.6 MB |
| "Function, for-loop", the same | 15.6 ms, 7.8 MB | 12.0 ms, 7.1 MB | -23%, -9% |
| "Point lists", bytes, one run each | 6.3 MB | 3.9 MB | -38% |
| The bottle, bytes, one run each | 47.1 MB | 36.8 MB | -22% |
| **A parallel round, 24 copies, three alternating pairs** | 3,656, 3,656, 4,431 ms | 2,767, 2,954, 3,178 ms | **about -20%**, as the owner expected |
| Its collector pauses | about 2,400 ms | about 1,800 ms | about -25% |
| Its bytes | 3,820 MB | 3,120 MB | -18% |

Results: the same BREP text and meshes by digest on all eleven examples. Six volumes change in their last digit or two, and the figures the rig reads don't. The closed form against the integration, run together inside `IntegrateVolume` and `IntegrateArea` on the modelling suite and the Boolean corpus: 5,286 comparisons, all under 1e-13 of the value but one, 4.8e-11 of a volume of 4e-5 that's nearly all cancellation. That count is nearly all the modelling suite: the corpus's own measuring commands use the default integration, which is unchanged.

**Tested on:** Foundation 2,541, ModelingData 3,343, PublicApi 432, DataExchange 349, Reliability 38, ModelingAlgorithms 5,743 (9 skipped), DrawLite 656, Kerf.Modeling 2,125, all passing; and `KERF_CORPUS_FULL=1` on all 53 of the corpus's classes, passing against their expected results. That shows no case worsened against those lists; it isn't a per-case comparison of two builds.

**What a thread keeps grew:** `MeshWorkStorage` from 360 to about 610 kB, so the limits add up to about 1 MB a thread. Measured after three rounds, `-- thread-sets 24`: 18.0 to 23.6 MB. That's under the owner's rule of about 50 MB, and bytes fell by far more per round.

**Looked at and left:**

- **Remembering a shape's volume or validity on its `TShape`** isn't needed: on planes a volume now costs almost nothing, and decisions 1798 and 1864 stand. A result's volume is still worked out twice where it's curved, by the Boolean's check and by whoever reads it (2% to 3% of those examples at most).
- **Checks repeated over the same faces:** an extrusion checks its profile, after `FixFace` or a fuse checked those faces. That's 1.0% of the bytes, and only the memo of decisions 1798 to 1801 removes it.
- **A clean after each extrusion:** 3.6% of the main thread's bytes and 1.9% of its samples, 5.4% and 4.2% on "for-loop". An extrusion with no two neighbouring edges of its profile on one line or circle has nothing to clean, but proving that the result is the same isn't plainly safe tonight. See the question below.
- **`Center` and `Plane(face)`'s centre of mass** could be closed-form on planes too, and take a fifth of the harness's samples, but a different last digit there moves the geometry built from it, so it isn't the same result. Exact centres would be a small change with a visible effect, for the owner.
- **Helpers:** `SLProps`, `SurfaceContinuity` and `IsTangentFaces` are 0.4% and 3.4% inclusive, spread over fillets. `FunctionSetRoot`'s vectors are 3.9%, nearly all under intersections, which another helper had open. Boxes in "extrude until" are three different boxes, not one three times.

#### For the owner

- **Skip the clean after an extrusion when the profile has nothing to merge?** About 2% of Bento's bytes, and 4% to 5% of examples of many small solids. It needs a check that the result really is the same, over more than these examples.
- **Exact centres of planar faces?** See above: quicker `Center`, with different last digits in what's built from one.

## Exact planar centres and no needless clean (units 12A and 12B)

*7 November 2026, on a machine shared with the owner and other helpers, with a corpus run alongside: bytes are good to about 2%, a whole-process figure a little worse, and the times here are from a loaded machine. The clean timed run follows. Each row is one call, after 200 to warm up, counted on its thread.*

| Bytes for one call | Rectangle | Plate with a hole | Hexagon | Circle |
|---:|---:|---:|---:|---:|
| Centre, integrated | 4,109 | 4,518 | 5,261 | 3,285 |
| `Center`, now | 904 | 888 | 1,184 | 888 |
| Extrusion with its clean | 161,002 | 235,372 | 240,972 | 80,241 |
| Extrusion now | 93,800 | 147,808 | 123,743 | 58,312 |
| of which the test for the skip | 2,704 | 2,968 | 2,568 | 1,416 |

So `Center` falls by about three quarters in bytes, and from 0.05 to 0.24 ms to about 0.001 ms; an extrusion falls by 27% to 49% in bytes. Bento's examples, four alternating runs, seven passes, medians, in MB:

| | Before | After | |
|---|---:|---:|---|
| Prismatic solid | 1.6 | 1.5 | -9% |
| Nesting locations | 14.9 | 13.1 | -12% |
| For-loop | 9.4 | 8.9 | -5% |
| Function, for-loop | 7.0 | 6.6 | -6% |
| All eleven | 128.4 | 124.2 | -3.3% |

The other seven move within the noise of a run, which reaches 2 MB on the largest. Against the bar of decision 1829: an extrusion clears 20% in bytes wherever it's the work, and examples of many small solids clear 5%, while a whole recalculation of Bento's mixed examples doesn't. Results: decisions 1894 and 1895. The corpus's boolean, blend, lowalgos and heal groups, 7,615 cases, are the same case by case as the run after unit 11E; neither change touches what they run.

## StepTidy (unit 11N)

*5 November 2026, `Kerf.Kernel.Benchmarks -- step-tidy` over the 289 files `-- healing` exports, written by the plain STEP
writer with `CleanDuplicates` off and on. It stays off by default (decision 1790).*

| | Off | On | Change |
|---|---:|---:|---:|
| Total file size | 49,067,067 bytes | 40,272,582 bytes | 17.9% less |
| Allocation, all files | 3,023 MB | 3,347 MB | 10.7% more |

- **Per file:** the median reduction is 18.9%, and the largest 55.6% (`pr1db.stp`, 177,965 to 79,010 bytes). Some small files
  have nothing to merge: 0.0%.
- **Read back:** all 289 files, both ways. Volumes (to a part in 10^9) and face counts match on every file, and none failed.
- **Writing time, on a quiet machine (the night of 5 November):** 1,708 ms off and 2,263 ms on, 32% slower. The same quiet run read 49.05 to 40.26 MB and 2,939 to 3,263 MB allocated, which is 17.9% and 11%, as above. So it stays off by default (decision 1790).
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

## Unit 11R: what checking every result costs

*6 November 2026, on a shared machine: every timing here is from a loaded machine, and bytes don't depend on the load. Medians of alternating fresh processes, 40 passes each, the last 20 being "settled". Parallelism is off for the check in the first table.*

**Over the 10% the brief set, so the owner decides (decision 1797).** The change is built on `phase11/11r-invalid`, with its tests, and isn't merged.

**Ways of applying it, to Bento's eleven examples** (a throwaway patch, 5 alternating runs each; the base recalculation is 271 ms and 157 MB):

| What is checked | Checks a pass | Check's own time | Share of 271 ms | Bytes |
|---|---|---|---|---|
| Every kernel operation that returned a shape | 205 | 81 ms | 30% | +42% |
| Every kernel operation that can fail this way | 88 | 56 ms | 21% | +30% |
| Each modelling-level Boolean, offset, revolve and extrude-until, once (fillets and chamfers were already checked) | 15 | 17 ms | 6% | +9% |
| The same and each plain extrusion | 34 | 46 ms | 17% | +20% |

Nothing was invalid in any of those checks. A check costs far more than the small operations it follows: 3 times a clean's time, 4 times an extrusion's, 79% of a face's fix, against 15% of a Boolean, 33% of a fillet, 24% of a chamfer and 19% of a thick solid. Run in the kernel on every step, it would be paid many times for one cell. Topology alone (`Geometry = false`) only saves about a quarter (89 ms against 118, 50 MB against 67).

**As built** (`Kerf.Modeling` checks once per operation, an extrusion its profile; 8 alternating pairs):

| | Before | After | |
|---|---|---|---|
| Settled recalculation, wall | 193.2 ms | 218.3 ms | **+13.0%** |
| Thread cycles a pass | 680 M | 790 M | +16% |
| Bytes a pass | 157.5 MB | 172.9 MB | **+9.8%** |
| Results | | | Same in all eleven |

Most of it is three examples of many small solids: "for-loop" +60%, "function, for-loop" +56%, "nesting locations" +37%, together 13 of the 25 ms. "Extrude until" +0.4%, "chamfer, fillet" +7% and the bottle +12%: the fillets' check was already there, and the bottle's `.Fix()` no longer repeats one. Measured from the same kind of run; rows differ by several percent between runs under load.

**The Boolean suites,** the median of each operation in one process, 407 simple and 193 complex cases:

| Suite | Boolean and clean | One check after the clean | Share | Bytes | Median, 90th percentile of a case |
|---|---|---|---|---|---|
| Simple | 3,136 ms | 221 ms | 7.1% | 10.2% | 9.5%, 24.9% |
| Complex | 3,153 ms | 182 ms | 5.8% | 8.1% | 6.2%, 23.9% |

A check after the Boolean as well as the clean would be 17.1% and 12.3%. Nine of the 600 cleaned results were invalid (occt-issues.md).

**Could it be cheaper when the operation knows what it touched?** Not measured, only read: a Boolean's untouched faces are the same `TShape`s as its inputs', so a check limited to new faces and the shells would help most where these examples spend it, fusing solids that barely meet. It would miss a bad face passed through from an input that wasn't itself checked, such as an imported one, and it means a new path through `Analyzer`, which is mostly OCCT's order and caches. 11Q found the check itself spread out, with little to cut.

**Options for the owner:**

1. **Keep it as built.** About 25 ms on 193 under load, and a recalculation stays well ahead of native's frozen 685 ms. Hosts can't turn it off in `Kerf.Modeling`. This is the simplest, and what the branch does.
2. **As built, with a way to turn it off in `Kerf.Modeling`:** a scope like `SkipClean`, or a setting Bento reads. Cost when on is the same; when off, a shape nothing can use is back.
3. **A cheaper check by default, the full one on request:** new faces only (above), or leaving 2D Booleans unchecked. Needs more measuring, and a fault can go unseen for a few steps.
4. **Check fillets, chamfers, offsets, sweeps and lofts only, not Booleans.** Inferred at about 2 to 4%. The Booleans' 9 of 600 go back to reporting success, so it doesn't meet the decision as worded.

## What a check can remember

*6 November 2026, on a shared machine: every timing here is from a loaded machine, and bytes don't depend on the load. Decisions 1798 to 1801. The code is on `phase11/valid-cache` (8a05c7c1f) and isn't merged.*

**The answer is no: it's safe, as far as reading and comparison can show, but it gains too little.** It removes about a fifth of the check's cost on Bento's examples and nothing on the Boolean suites, against an aim of most of it. Most of the faces a check sees are being seen for the first time.

**The idea:** a frozen face found valid stays valid, so remember that on its `TShape` and skip it next time. Reading each sub-check shows what that depends on:

| Sub-check | Depends on | Can it be remembered? |
|---|---|---|
| Edge's own flags, range and polygons; vertex on an edge | The edge's `TShape`, and the placement it's reached at | Yes, under its face |
| Edge against a face's surface; vertex on a face; wire in a face; wires' intersection, nesting and orientation | Both shapes, all reached from the face, the face's placement, and its orientation (`WireCheck` reads it for closure in 2D) | Yes, under its face, with that placement and orientation |
| Shell closed and oriented; solid's shells | Every face of the shell: shell and solid checks read the shapes directly, not the faces' results | No: nearly always a new `TShape`, so it wasn't tried |
| The same at another placement | Every distance is measured after moving the geometry, so rounding differs, and at large coordinates reaches the tolerances | No: not proved |
| `Geometry = false`, or `Exact` | A different check | No: only the default geometric check by sampling is remembered and read. Parallel doesn't change it: sampling doesn't use the flag, and which thread checks doesn't change what's found |

**What makes the skip safe:**

- A face is remembered only after a check in which *every* status of *every* sub-shape, in every context, is no fault. `IsValid` reads only some of them (`ValidSub` stops at a seam's second occurrence), so this is stronger, and whichever of them a later check would read, none is a fault.
- Only a frozen face, whose whole sub-graph is frozen with its geometry. Never with a working overlay open on the thread, whose working copies stand in for frozen `TShape`s. The key is the `TFace`, the placement by the identity of its datums (`IsEqual`) and the orientation.
- A check that isn't clean is repeated in full by its caller, so reports don't change.
- **Threads:** the one field holds an immutable record, replaced whole by `Volatile.Write`, after the check has finished. Two threads that check one face write true things, and the last wins. It holds a placement and no shape.

**The comparison mode** (`KERF_CHECK_MEMORY=compare=file`) does each check in full as well, and records any difference in the verdict, or a clean remembered result where the full check isn't clean. No differences:

| Run | Checks compared | Invalid | Faces skipped | Differences |
|---|---:|---:|---:|---:|
| Corpus, cases using the Boolean commands (4,654 run of 7,100) | 5,753 | 51 | 6,635 | 0 |
| Corpus, fillets and chamfers (800 run of 884) | 832 | 5 | 72 | 0 |
| Corpus, healing (1,326 run of 2,107) | 1,210 | 43 | 3,991 | 0 |
| `Kerf.Modeling.Tests` (2,074 tests) | 5,573 | 208 | 2,042 | 0 |
| `Kerf.Kernel.PublicApi.Tests` (383 tests) | 264 | 73 | 191 | 0 |
| Bento's eleven examples, 5 passes | 500 | 0 | 360 | 0 |

The corpus's groups overlap, and each case compares only the checks its script asks for. Volumes, face and triangle counts of the eleven are the same in every run.

**What it gains on Bento's eleven examples** (the same build with the check left out, in full and with the memory; 6 alternating rounds of 30 passes, medians of the settled recalculation):

| | No check | Full check | With the memory |
|---|---:|---:|---:|
| Recalculation | 222 ms | 248 ms (+11.7%) | 243 ms (+9.5%) |
| Bytes | 140.5 MB | 160.6 MB (+14.3%) | 157.9 MB (+12.4%) |
| "for-loop" | 9.75 ms | 14.7 ms (+51%) | 12.7 ms (+30%) |
| "for-loop" bytes | 9.0 MB | 12.9 MB | 12.4 MB |

Processor time was no use here: under this load it put no check above the full check. Three runs of bytes give 13% to 19% of the check's bytes saved, and the times about 19%; "for-loop" saves more time than bytes, but only 13% of bytes, and is 2 ms.

**Why so little:** counted over three passes, 17% of the faces a check meets were remembered, 48% were new, 31% were known at another placement and 4% the other way round. An example like "for-loop" extrudes new solids and fuses them once, so its one check sees only new faces. Even allowing any placement, which isn't proved, removes only 44% of the check's bytes.

**The Boolean suites** (one check after the clean, serial, 407 simple and 193 complex cases):

| Suite | Faces | A full check | With the memory, as `Kerf.Modeling` checks | If each input had been checked first |
|---|---:|---:|---:|---:|
| Simple | 3,566 | 199.6 MB, 258 ms | **0 skipped**, 200.5 MB, 242 ms | 767 skipped (22%), 172.6 MB, 223 ms |
| Complex | 1,465 | 103.3 MB, 148 ms | **0 skipped**, 103.6 MB, 151 ms | 411 skipped (28%), 80.6 MB, 133 ms |

Nothing checks a Boolean's inputs, so nothing is known. Even if each input came from a checked operation, a little over a quarter of the faces at most go unchecked, as these cases cut and fuse what they touch.

**What would help more is a different question:** trusting a face without having checked it, such as a primitive's or an extrusion's, which is what decision 1793 infers for extrusions. That isn't proved, so it wasn't tried. The owner's options in "Unit 11R" stand.

## The check on small shapes

*6 November 2026, on a shared machine with its owner using it: every time here is rough, and bytes and counts don't depend on the load. Decisions 1823 to 1828.*

**The answer is that about half of a small shape's check was bookkeeping, and it's gone, with the verdict unchanged.** Bytes of one check, from `GC.GetTotalAllocatedBytes`, and the smallest time for a batch of checks over 3 alternating rounds of 1,000:

| Shape | Sub-shapes | Bytes before | After | | Time before | After | |
|---|---:|---:|---:|---:|---:|---:|---:|
| Box | 55 | 321,856 | 169,008 | **-47%** | 214 µs | 168 µs | -22% |
| One square face | 15 | 52,464 | 29,400 | -44% | 30 µs | 24 µs | -22% |
| One hexagonal face | 21 | 86,936 | 44,552 | -49% | 52 µs | 39 µs | -26% |
| Hexagonal prism | 79 | 561,704 | 265,647 | **-53%** | 241 µs | 153 µs | -36% |
| Plate with a hole | 69 | 431,584 | 258,136 | -40% | 234 µs | 168 µs | -28% |
| "for-loop"'s result | 403 | 2,878,392 | 1,357,368 | **-53%** | 1,563 µs | 945 µs | -40% |
| "function, for-loop"'s result | 325 | 2,332,488 | 1,092,312 | -53% | 1,305 µs | 758 µs | -42% |
| "nesting locations"' result | 1,411 | 9,944,047 | 4,962,928 | -50% | 3,210 µs | 2,040 µs | -36% |

**Bento's eleven examples,** 8 alternating rounds of 30 passes, bytes per recalculation; results the same in all eleven:

| | Before | After | |
|---|---:|---:|---:|
| Recalculation | 165.2 MB | 152.5 MB | **-7.7%** |
| "for-loop" | 13.5 MB | 10.9 MB | -19% |
| "function, for-loop" | 9.9 MB | 8.1 MB | -18% |
| "nesting locations" | 20.4 MB | 17.9 MB | -12% |

Against "What a check can remember", this is more than remembering gave (about a fifth of the check's bytes) and it applies to new faces too. The bytes also include what `IndexedSet` and `WireExplorer` save outside the check, which wasn't separated out.

**The recalculation's times here are no use.** The same build differed by up to 23% on one row from one run to the next, and the medians went both ways: 553 ms against 564, then 497 against 525 by the smallest round, with "for-loop" at 23.6 against 27.4 in one run and 22.2 in the next for the same code. So the gain in a recalculation is inferred from the check's own time and bytes, and needs the quiet timed run.

**Where a small shape's check went** (allocation ticks' inclusive shares on a box and "for-loop", decision 1706: the leaves of a CPU profile aren't read, and here `PollGC` is 95% of them). No site was above 5.4%:

| What | Before | What was done |
|---|---:|---|
| A dictionary, a lock and a snapshot for each sub-shape's contexts | About 15% | `ContextMap`, a small array with an index past 16; no snapshot; no lock unless on the pool |
| Sets and maps of a few shapes, in the wire's checks and `WireExplorer` | About 25%, with a dictionary and growing arrays for four edges | `IndexedSet` compares in turn up to 8 items, and the check uses it everywhere; `Propagate` is its own queue; `Closed` counts instead of keeping lists |
| The wire's map built twice | 4% | With everything oriented, `Minimum`'s map is the one `Closed` would build, and its connection is the one it would find |
| Measuring a line on a plane, and projecting it when it has no pcurve | 7% on a box, 17% on "for-loop" | Decision 1825 |
| Boxes and bounds to compare the wires of a face with one wire | 5% | Still measured, as a curve that can't be measured is a failure; no longer kept |
| Common vertices of two edges that only meet at their ends | 2% | Found when an intersection first needs them |
| Searching a context for a shape found by exploring it | No bytes; quadratic in a face's edges | Decision 1826 |
| The face's adaptor for each step of a wire; `FClass2d`'s adaptors for each edge; a copy of an unplaced curve | 5% | One each, loaded again |

**What wasn't shortcut, and why.** A wire's self-intersection is done by `GInter`, whose line against itself is already nothing, and two straight edges need the same intersection the check is for. The 2D classifier for one wire is the orientation's verdict. Both still run in full.

**How it's known that nothing changed.** `tests/Shared/FullCheck` is the check as it was, generated from the commit before this work: its classes and `FClass2d`, `WireExplorer`, `EdgeCurveAdaptor`, `IndexedSet` and `IndexedMap`, with only the namespace changed. With `KERF_CHECK_COMPARE=file`, each check any test makes is made both ways, comparing the verdict and every status of every sub-shape in every context, and their order unless on the pool. Its counts:

| Run | Cases or tests | Checks compared | Invalid | Sub-shapes | Differences |
|---|---:|---:|---:|---:|---:|
| Corpus, `boolean` | 4,091 of 4,093 | 3,499 | 15 | 378,299 | 0 |
| Corpus, `blend` | 475 of 475 | 436 | 2 | 40,257 | 0 |
| Corpus, `chamfer` | 279 of 279 | 261 | 3 | 29,127 | 0 |
| Corpus, `heal` | 2,739 of 2,741 | 665 | 34 | 59,986 | 0 |
| `Kerf.Kernel.ModelingAlgorithms.Tests`, with the check's own invalid shapes and those of `BRepCheckComparisonTests` (3,000 random small faces and each shortcut's limits, on either side) | 5,709 | 15,491 | 6,747 | 350,960 | 0 |
| `Kerf.Modeling.Tests` | 2,093 | 5,570 | 123 | 135,553 | 0 |
| **All** | | **25,922** | **6,924** | **994,182** | **0** |

Each group ran on its own, four cases at a time. Each case compares only the checks its script asks for, and the cases weren't judged: they are there to make shapes, and their limits were stretched fourfold. `Kerf.Kernel.PublicApi.Tests` passes but doesn't hold the comparison.

Then, deliberately wrong shortcuts, to see the comparison fail: an end allowed 100 tolerances fails 5 and 4 of its tests, and the slope limit left out fails 1 (`A_slope_whose_ends_are_within_tolerance_but_whose_parameters_are_large`: a false "valid" without the limit). **One guard wasn't shown to matter:** leaving out the shortest range fails nothing, including in a test of stubs made for it. It stays, as nothing is lost by measuring a stub.

**What's left:** an `NCollection_List` is three objects (itself, a `List<T>` and an array), so each status list is about 100 bytes: dropping the `List<T>` would help everywhere in the kernel, but is a change of its own. `FClass2d` is now the largest single part of a small shape's check, about a quarter, and what's in it is its answer. Beyond that the rest is spread: the top site is 3%.

## Booleans check their result's volume

*7 November 2026, on a shared machine: every time here is from a loaded machine, and bytes don't depend on the load. Decisions 1860 to 1868.*

**Under the 3% the brief set, measured by the check's own time and bytes inside a recalculation,** as whole passes on this machine differ by more than the check costs. Bento's eleven examples, 30 passes, the last 15:

| | Calls a pass | Time a pass | Share of a loaded recalculation (490 ms) | Bytes a pass | Share of 152 MB |
|---|---:|---:|---:|---:|---:|
| The check | 13, of which 7 are on solids | 6.2 ms | **1.3%** | 1.55 MB | **1.0%** |

Six alternating pairs of whole processes (median of 30 passes, 490 ms against 632 ms for the base, 152.3 MB against 152.5 MB) can't see it: that's the load. Sketches and faces measure nothing, as their kinds are looked at first.

**The Boolean suites,** each case's Boolean with `Simplify` and with no validity check, in one process, the check called after it:

| Suite | Boolean | Check | Share | Bytes | A case's share: median, 90th percentile |
|---|---:|---:|---:|---:|---:|
| Simple, 407 cases | 6,625 ms | 454 ms | 6.8% | 5.4% | 9.5%, 17.5% |
| Complex, 193 cases | 4,415 ms | 476 ms | 10.8% | 11.7% | 7.2%, 16.0% |

Over the corpus's 4,215 Booleans the check's median is 0.43 ms, its 90th percentile 4.7 ms, its 99th 47 ms and its longest 1.29 s (`bugs/modalg_7/bug27760`).

**How often it fires.** None of the corpus's 4,215 Booleans (3,425 cases), the suites' 600 or Bento's examples. Of the first, 3,268 had a rule to meet; 902 had something that isn't a solid, 41 an input of no positive volume, 4 an empty input. Six Booleans in 4 cases break a rule on inputs that aren't valid, and pass for that reason (decision 1862). Near-misses with valid inputs, past a rule as a share of the larger input; the margin starts at 1e-4:

| Past a rule by | Booleans | |
|---|---:|---|
| Over 1e-4 | 0 | |
| 1e-5 to 1e-4 | 2 | bug227: tolerance 0.6, and a mesh agrees |
| 1e-6 to 1e-5 | 4 | buc60703 (tolerances up to 0.25), bug25722 (fuzzy value 1e-4) |
| 1e-7 to 1e-6 | 2 | buc60703_3, bug25477_1 (fuzzy value 2e-5) |
| 1e-12 to 1e-7 | 52 | Measurement's noise |

Each of the eight above that is a fuse of solids that touch, whose result comes out a little over their sum as a large tolerance or a fuzzy value bridges them; a mesh of each result agrees with the integration to within the mesh's own error.

**What it finds.** The thread of #1629 over a grid of 227 depths and sizes: 4 fire, all with a solid inside out, and no other. The issues, with the precise path's figures:

| Issue | Cut | Common | Fuse |
|---|---|---|---|
| #1629 | Fails: solid of −29.97 | Fails: empty | Fails: 8023.19, over 7993.21 |
| #1643, body and lip | Fails: removed nothing | Fails: empty | Passes, and is short by a little |
| #1643, lip and body | Passes, right | Passes, right | Passes, right |
| #1659 | Fails: removed nothing | Fails: empty | Passes, though it's the two inputs apart |
| #245 | Fails: removed nothing | Fails: empty | Passes |
| #1039, a shell | | No rule | |

The table's rules alone find only #1629: decision 1865 says why, and 1866 what was added.

## Three rises explained (unit 12N)

Counted, not timed: the machine was shared with the owner and another helper, and the rig wasn't run. Decision 1940 has the reasoning.

| Rise | Cause | Counted | Done |
|---|---|---|---|
| 36, extrude until, +3.6 MB (31.7 to 35.3, decision 1887) | Decision 1881's check that a fuse as large as both its inputs didn't lose an overlap. The inputs touch along a curved face, so their boxes overlap deeply and the grid's points are all classified | 371 classifications, 3.56 MB a recalculation on the thread; switched off, 36.3 to 33.0 MB on the last pass | Left: the price of catching a silent wrong fuse, about 10% of one example |
| A thick solid, +2.2% (79.1 to 80.8 MB) | Decision 1885's walked curves for equal cylinders whose axes missed by 7e-15, rounding, on one part's pipes | That part 11.4 to 16.8 MB; bisected to 989e1a577 | **Fixed** (decision 1941): the row 80.0 to 74 MB, the part back to 11.4 |
| `Pump_Nut.brep`'s parallel mesh, 5.86 to 6.81 MB | Each thread keeps its own `MeshWorkStorage`, so the bytes depend on which threads mesh which faces; the rig keeps the last run's | 31 runs in one process: 5.66 to 9.64 MB parallel, 5.55 to 5.61 serial; 24 sets made | Left: larger shapes' parallel meshes are within 1% and 9% of serial |

## Unit 12P: the slowest list again

*8 November 2026, on a machine shared with the owner and another helper: times are from a loaded machine, and the figures that count are samples and bytes. Decisions 1950 and 1951.*

**Alone, the corpus's slowest cases take a fraction of their time in a pass.** One process, one case after another (`ZzP12pProbe`, a throwaway test; processor time and bytes for the whole process):

| Case | Wall | Processor | Bytes | Where it goes | Verdict |
|---|---:|---:|---:|---|---|
| `bugs/modalg_5/bug25232_9` | 11.6 s | 12.8 s | 7,557 MB | `mkvolume` of 63 planes and a sphere into 9,832 solids: `MakerVolume` 34% of samples, 69% of bytes; `checkshape` 26% of bytes. `BSplineCurve2dBounds.Box` copies a whole pcurve and segments it to bound an arc, 896 MB (12%), as OCCT does | Large by nature; the box a candidate (below) |
| `bugs/modalg_7/bug32444` | 7.8 s | 8.9 s | 2,583 MB | A sweep's corner trimming: its Booleans' edge-face 26% and face-face 15% of samples | Hard, spread out |
| `boolean/simplify/A3` | 5.0 s | 7.4 s | 1,759 MB | The Boolean's filler 23%, building 13%, `UnifySameDomain` 10% | Spread out |
| `bugs/modalg_1/buc60462_2` | 4.1 s | 6.0 s | 534 MB | The fuse; Draw-lite runs no check of volume | Below |
| `bugs/modalg_6/bug26701` | 2.2 s | 2.6 s | 1,761 MB | The filler 21% of samples; `checkshape` 64% of bytes, of which composing placements in `VertexCheck.InContext` 924 MB (52%) | A candidate (below) |
| `bugs/modalg_7/bug28119_1` | 1.3 s | 1.7 s | 225 MB | BAD, as before | Nothing to do |

**`buc60462_2`'s check of volume** (decision 1937's 10 s), through `BooleanVolumes.Check` alone: 8.5 and 7.8 s, 133 MB a check. 99.5% of its samples are decision 1881's grid, 343 points classified against each input; the classifier's ray asks each edge it passes near for its nearest point (`ExtCC`, `GlobOptMin`, Newton). **A quarter of those samples were a B-spline evaluated at the very end of its range**, again and again, as Newton clamps to it. Kept now (decision 1950): **5,958 samples to 3,858 (-35%)**, the same bytes. What's left: Newton 68%, rebuilding the one-span cache 15.5%.

#### Candidates, not done

- **A cache of a few spans** for a B-spline adaptor: 15.5% of what's left of that check. The span must still be the one `LocateParameter` gives, so the same coefficients are used; a new cache allocates where a rebuild doesn't, so bytes go up a little where an adaptor walks along a curve.
- **The placements in a validity check** (`bug26701`): `(eloc * loc).Predivided(vloc)` for each vertex, each of its edges and each representation, half of that case's bytes. A shape's identity reads its chain, so a shortcut has to give the very chain the product does. **Done** by composing only where read, and taking a product's transformation without its chain: -48% (decision 1971, "The perf search").
- **The box of a pcurve's arc** (`bug25232_9`): the segmented poles without the copy, if they come out the same. Measured since: the copy is 3% of the case and the segmentation 9% ("The perf search").

## The perf search (8 November 2026)

*On a machine shared with the owner and another helper: no time here is worth more than a sign, and none is used for a verdict. Bytes are `GC.GetTotalAllocatedBytes` over a case, or the benchmarks' own figures; shares are allocation ticks, which sample about every 100 kB. Decisions 1971 to 1973.*

**What Bento's examples spend now, against the rig's baseline (f5a7bd7a2):** the same bytes, 122.2 MB a recalculation against 122.1 at f5a7bd7a2 built the same way, every example within a megabyte of it, which is the noise between runs, and with the same results. Nothing new reaches the top ten sites by bytes; the order moves within sampling. The Boolean's filler is 29.6% of the bytes, the mesher 25.6%, healing 9.4%, offsets 8.3%, construction 7.8%, and the checks together about 12%: the validity check 8.1%, `BooleanVolumes.Check` 3.5% (of which decision 1881's grid 2.7%), `OffsetVolumes.Check` 0.3%. **One thing is new in samples, not bytes:** extrude until's half torus now meshes through the deflection guard of decision 1929 (a torus at an absolute deflection), and the mesher's share of that example's samples rose from 17% to 28%, with the same 4,156 triangles. On this machine that can't be told from the load: for the quiet run. The sweep of dead circles (decision 1955) isn't it: switched off, no difference.

**The validity check of a placed, meshed shape composed placements it never read,** half of `bug26701`'s bytes (decision 1971). Two changes, each the same result by construction:

| `bugs/modalg_6/bug26701`, the whole case | Bytes | |
|---|---:|---:|
| Before | 1,815 and 1,799 MB | |
| A vertex's placement on a representation composed only where it's read | 1,136 and 1,119 MB | -37% |
| An edge's polygons placed by the product's transformation, without its chain | 958 and 923 MB | **-48% in all** |

What's left of the placements there is 99 MB, 10%: `Tool.CurveOnSurface` and `Tool.CurveOnPlane` compose a placement to compare a representation's with, which a comparison against the product's items, as `ProductTransformation` folds them, could do without the chain. Not done: 7% of one case.

**Shape healing made what it never used** (decision 1973). Two changes, each under the bar alone and over it together for ShapeFix, an operation over many different files:

| The healing suite's 289 files, bytes | ShapeFix | STEP import with healing |
|---|---:|---:|
| Before | 1,439 MB | 2,895 MB |
| A wire's 2D intersection makes only the sub-intersector it uses | 1,389 MB (-3.5%) | 2,845 MB (-1.7%) |
| A one-shot orientation of a shell builds no tree of its edges (reverted, then reinstated by the owner: decision 1974) | **1,357 MB (-5.7%)** | **2,812 MB (-2.9%)** |

`IntCurveCurveGen` (OCCT's `Geom2dInt_GInter`) made its three sub-intersectors, each with sequences of its own, for every pair of edges a wire's check intersects, and a pair uses one. A classifier made to orient a shell and dropped filled the map and box tree of its edges and vertices, which the point at infinity never reads. The Booleans suites, MB per Boolean: 6.30 to 6.23 complex, 4.34 to 4.25 simple. Allocation ticks had put the tree at 8.6% of ShapeFix, as the JIT inlines the faces' intersectors into `InitShape` and the ticks fall to it: bytes measured whole are what decide.

**Looked at, measured, and under the bar** (decision 1972):

| Where | What it is | Measured | Verdict |
|---|---|---|---|
| `bug25232_9`, 7.6 GB | The box of a pcurve's arc segments a copy of the curve: 912 MB, 12% of the case. 72,314 segmentations of only 6,864 different curves and ranges | The copy is 245 MB of that (3%); the segmentation itself 647 MB | A cache per curve would take most of it, but 4,576 of the 4,621 curves are a Boolean's working copies, which can change: under the bar, and not plainly safe |
| STEP import with healing | `GenExtPS`'s grids for projecting points, 12% of the import | 3,384 grids for 3,282 surfaces | One a surface, as OCCT: nothing repeated |
| Every Boolean and fix | `Tool.IsClosed` composes a placement before looking for a pair of pcurves, which most edges haven't got | The healing suites' bytes the same (2,895.2 and 2,896.0 MB) | Not in |
| `bug26701` | A vertex's check on a face composes the face's placement with no points to place | No change | Not in |
| Bento | `MapShapesAndAncestors`, 2.4% to 2.6% of the bytes everywhere; its lists start at four | A capacity of two would save perhaps a fifth of it | Under the bar |
| Bento | Decision 1881's grid in extrude until, 3.2 MB a pass, 9% of the example | Its classifications are spread: no site over 5% of them. Against the extrusion first, the example takes 8 MB more | Under the bar; the order is right as it is |

**Ideas not tried, by likely gain and then risk:**

1. **A box of a pcurve's arc kept for a Boolean's working curves** (`bug25232_9` 12%, through `AddUVBounds` in `BuilderSolid`, the checks and the classifiers): only frozen curves can keep it themselves, and in a Boolean most aren't, so the Boolean would keep the boxes of its working copies, and drop one when it changes a pcurve. Large on large Booleans; medium risk.
2. **Comparing a representation's placement with a product without building it** (`Tool.CurveOnSurface`, `Tool.CurveOnPlane`): 7% of `bug26701`, and every placed shape a little; low risk with a bit-for-bit test as `ProductTransformation` has.
3. **A cache of a few B-spline spans** (12P's): 15.5% of `buc60462_2`'s check; medium risk, bytes up a little.
4. **`SClassifier.Perform`'s edge-face map kept by the classifier,** which builds it for every point: 5% of decision 1881's classification bytes, and every classifier asked many points (the Booleans' `BuilderSolid`); low risk.
5. **`MapShapesAndAncestors`' lists made for two:** perhaps half a percent everywhere; low risk, and more allocations where a list grows past two.

**Tested on:** Foundation 2,544, ModelingData 3,346, ModelingAlgorithms 5,793 (9 skipped), DataExchange 366, PublicApi 497, Reliability 41 and Kerf.Modeling 2,251, all passing in Release; and the whole corpus, 17,872 cases four at a time, against `corpus-11e`, after the check's changes and again at the head. Both differ only in `bugs/mesh/bug31461` and `bugs/modalg_7/bug27928`, which already differ on main; the head's run also has `bugs/modalg_5/bug24639` failing its check that parallel beats serial, a time on a loaded machine, and alone it's OK, as in 12P's run.

## The perf search, second round (8 November 2026)

*On a machine shared with the owner: no time here is used for a verdict. Bytes are `GC.GetTotalAllocatedBytes` over each whole case, one case after another in one process; shares are allocation ticks, which sample about every 100 kB and credit a frame the JIT inlines to its caller. Decisions 1975 and 1976.*

**The first round's five ideas, counted over 561 corpus cases** (every twelfth runnable case of the Boolean, blend, offset, pipe, sweep, sewing, healing and STEP groups, with the five slowest; 26.7 GB of ticks):

| Idea | Over the sample | The most in a case or group | Verdict |
|---|---:|---|---|
| 1. Boxes of B-spline pcurves' arcs kept | 4.3% of the bytes | `boolean/volumemaker` 17%, `bug25232_9` 11% | **Done** (below) |
| 2. A representation's placement compared without the product | 0.85% | 11% of `bug26701` and of `boolean/periodicity/A4` | Under the bar |
| 3. A cache of a few B-spline spans | 1.5% of the samples, 0.6% of the bytes | 3% of a group | Under the bar |
| 4. `SClassifier`'s edge-face map kept | 0.13% | 0.5% of a case | Under the bar |
| 5. `MapShapesAndAncestors`' lists made for two | 1.3% for the whole call | 6.1% of `boolean/simplify/A3`, of which a part | Under the bar |

**The arcs bounded again are on frozen curves,** not a Boolean's working copies as the first round took them to be. A cache of every curve's arcs, for the measurement only, and the same cache of frozen curves alone give the same bytes:

| `boolean/volumemaker` (74 cases) and `bug25232_9`, MB | All |
|---|---:|
| Before | 17,360 |
| Every curve's arcs cached (unsafe: measurement only) | 14,630 |
| Frozen curves' arcs cached, no limit | 14,619 |
| Kept with the curve, up to 4 arcs | 15,232 |
| Kept with the curve, up to 64 arcs (decision 1975) | **14,697** |

So a frozen 2D B-spline keeps the boxes of up to 64 arcs, which it can't change: `volumemaker` 9,981 to 8,064 MB (-19%), the 20 cases over 150 MB each 4% to 37% less and 15 of them past 20%, and `bug25232_9` 7,379 to 6,633 MB (-10%). Two small cases rise by a megabyte (`D8`, `E2`), the kept boxes.

Over the 561 cases the same statuses, 26,383 to 25,405 MB (-3.7%), and -0.4% without `volumemaker` and `bug25232_9`: `bug24359` 410 to 335 MB, `boolean/cells_test/B1` 21 to 16; `boolean/simplify/A3` 1,704 to 1,715 MB, a Boolean in parallel whose bytes move by that much between runs.

**Tested on:** Foundation 2,544, ModelingData 3,376, ModelingAlgorithms 5,871 (9 skipped), DataExchange 444 (2 skipped), PublicApi 541, Reliability 41 and Kerf.Modeling 2,251, all passing in Release; and the whole corpus, 17,872 cases four at a time, against `corpus-11e`: only main's two (`bugs/mesh/bug31461`, `bugs/modalg_7/bug27928`) and `bugs/modalg_5/bug24639`'s check that parallel beats serial, a time on a loaded machine, which passes alone.

## Booleans on ordinary parts: the gap a user sees (unit 12Y)

*8 and 9 November 2026, on a machine shared with the owner and two other helpers: every time here is from a loaded machine, each process pinned to four logical processors, and the verdicts rest on bytes and on runs alternated back to back. Decisions 2000 and 2001.*

**The report** (a user's agent, on `Kerf.Modeling` 0.3.0): ordinary extrude-and-cut parts' Booleans take 1.3 to 1.4 times build123d's, and its test suite about twice as long. **Both reproduce, and neither is the kernel's Booleans.** Without `Kerf.Modeling`'s checks, Kerf's Booleans and clean take about 0.5 to 0.7 times native OCCT's. The warm gap was the two checks Kerf adds on purpose (decisions 1742 and 1860), about as much again as the Boolean; two changes take a third of the checks off with the same results, and the parts now take 0.94 times native's at the quietest (0.85 to 1.05 under more load). The rest of "twice as long" is a fresh process: the JIT and tiering, two to three times native's for the first few hundred Booleans, ReadyToRun or not.

### The rig

Eight parts, 145 Booleans in all, each a plate or block extruded from a sketch of lines and arcs with 8 to 47 cuts and fuses one at a time, as `p -= Pos(...) * hole` does: a bracket with holes, a slot, two bosses and a pocket; a block with a pocket and holes in three faces; a flange with a hub, a keyway and counterbored bolt holes; a plate of 45 vent holes and two ribs; an angle from lines and an arc, with holes, slots and gussets; a lid with a pocket, bosses, holes and ten vent slots; a wheel with slots, a hub and twelve notches; a manifold of crossing bores and a tab. Kerf made the inputs and wrote them as BREP, and each side reads the same text, so both start from the same shapes. Native is OCCT 8.0.1 (MSVC) through build123d.NET's binding, run as build123d runs it: `BRepAlgoAPI_Cut` or `_Fuse` in parallel, non-destructive as Kerf's, then `ShapeUpgrade_UnifySameDomain(shape, true, true, true)` without internal edges; asking for both histories made no difference. Python's OCP couldn't be used: its native libraries are missing from the build123d checkout's environment. Both sides give the same faces and edges, and the same volumes to 1e-9 but the manifold's, 1.5e-6 apart (each measured by its own side's default integration). The rig is in the unit's scratch folder, not the repository.

### Warm: where the time goes

Medians of nine passes after three, alternating fresh processes; ms a pass of the 145 Booleans. The last three columns are from 01:00 on 9 November, when the machine was quietest: three pairs of before and after, and three runs of native and the kernel just before.

| | Earlier runs, under more load, before | Quietest, before | Quietest, after both | Of native's, after |
|---|---:|---:|---:|---:|
| Native: Boolean and clean | 1,115 to 1,375 | 1,012 to 1,022 | (the same) | 1 |
| Kerf, the kernel: Boolean and clean (not changed) | 584 to 791 | 563 to 570 | (the same) | 0.56 |
| `Kerf.Modeling`'s operators | 1,354 to 2,163 (1.0 to 1.6 times native) | 1,132 to 1,147 (1.12 times) | 950 to 953 (-16%) | 0.94 |
| `Kerf.Modeling`, MB a pass | 1,113 | 1,113 to 1,116 | 1,078 to 1,079 (-3.2%) | |

By phase, the kernel's Boolean with each of `Kerf.Modeling`'s checks timed apart (the parts' own runs; native's rig times only its Boolean and its clean):

| Share of a pass | Kerf before | Kerf after both | Native |
|---|---:|---:|---:|
| The Boolean (pave filler and builder) | 47% to 57% | 55% to 57% | 83% |
| The clean (`UnifySameDomain`) | 4.5% to 5% | 5% | 17% |
| The volume check (`BooleanVolumes.Check`) | 33% to 37% (388 to 390 ms at the quietest) | 22% (211 to 214 ms) | |
| The validity check | 15% to 19% (171 to 177 ms) | 19% (179 to 184 ms) | |
| `Kerf.Modeling`'s own work around the kernel (history, compounds, names) | under 1% of the samples | the same | |

In samples (EventPipe, `Kerf.Modeling`'s run): the pave filler 30% and the builder 15%, the volume check 21% (19.4% of it integrating volumes), the validity check 11%, the clean 5%. A sample lands at a safe point, so 43% of them sit in `PollGC`; the collector's own pauses were measured at 50 to 75 ms a pass, 5%, and neither the server collector (pauses halved) nor a 64 MB first generation changes the wall time. Bytes: the kernel 785 MB a pass, `Kerf.Modeling` 1,113 before, 1,078 after both changes; the pave filler is half of them.

**What was avoidable, and changed** (each the same result by construction, the suites and the corpus's Boolean and `modalg` groups as before):

1. **The volume check measured each result twice** (decision 2000): once as the check's result, again as the next Boolean's object. A frozen shape's volume is now kept with its TShape. The check's own time 388 to 212 ms a pass at the quietest (490 to 230 under load); the parts in `Kerf.Modeling` 16% quicker in the quietest pairs, 12% to 30% in the loaded ones.
2. **A fuse of a second boss or rib mapped the part's edges for every point of decision 1881's grid** (decision 2001). The first boss on a face only touches it and the check skips the slab (decision 1887); from the second on the part's box takes in the first boss's height and all 343 points are classified in the part, each mapping the faces of every edge again: a 40 MB check after a 7 MB Boolean. The map is kept for a frozen shape. That fuse 30% to 36% fewer bytes (the vents plate's 53.7 to 34.6 MB, the bracket's 20.9 to 14.5), the parts 2.4%.

**What's left, for the owner:** the two checks are still about 40% of `Kerf.Modeling`'s Boolean time on these parts (the volume check 211 to 214 ms a pass at the quietest, the validity check 179 to 184), against a kernel Boolean that is already ahead of native's. With them, `Kerf.Modeling` is 0.94 times native; without them it would be 0.56. What would take more:

- **The result's volume** is most of what's left of the volume check: every face integrated, each Boolean, so a plate of 45 holes integrates its 45 cylinders 47 times. Keeping each face's share would make it the new faces only, but the integration is about the shape's rough barycentre, which moves with every Boolean, so the last digits would change: a deliberate change of the check's figures, not done.
- **The grid of decision 1881** still classifies 343 points in the part for each further boss (the rest of that check: rays meeting the part's faces, 2D classification of the hits). A tighter region than the boxes' overlap, or the tool classified first when it's the cheaper, are the ideas; neither changes a verdict, neither was measured to the bar.
- **The validity check** is OCCT's `BRepCheck_Analyzer` on the whole result each time; 11R's "new faces only" (option 3) is the large saving and changes what is checked.

### Cold: a fresh process

The rig's 145 Booleans in fresh processes, one at a time, five of each (medians, ms; "JIT" is the runtime's own count of methods compiled and their time, all threads):

| | First Boolean | First pass | Second pass | Methods the JIT compiled | Their time |
|---|---:|---:|---:|---:|---:|
| Native | 9.3 | 1,024 | 990 | | |
| Kerf, the kernel, IL (a project reference) | 326 | 2,922 | 1,657 | 15,500 | 2,750 |
| Kerf, the kernel, ReadyToRun | 102 | 1,702 | 1,519 | 4,700 | 800 |
| `Kerf.Modeling`, IL | 383 | 4,415 | 2,251 | 18,200 | 4,300 |
| `Kerf.Modeling`, ReadyToRun (as the packages ship) | 124 | 3,059 | 2,730 | 6,700 | 1,660 |

Before the two changes, against a settled pass of about 1,300 ms. The process's start to `Main` was 70 to 140 ms on every side, and reading the cases about 100 ms. So in a fresh process `Kerf.Modeling` pays about 1.7 s more than settled for its first 145 Booleans, ReadyToRun or not, and still about 1.4 s more for the next 145: ReadyToRun code runs until tiering replaces it, and the second pass is still about twice the settled one. Native pays nothing. The ReadyToRun builds were published from the rig, compiling Kerf's assemblies with the application's settings, not the packages' exact ones (decision 1689); the counts are within a few percent of 11C's.

### How much of the agent's figures each explains

- **"1.3 to 1.4 times build123d's" Booleans, warm:** all of it is the checks. `Kerf.Modeling` was 1.0 to 1.6 times native on this machine, 1.12 at its quietest; the kernel's Boolean and clean alone 0.5 to 0.7. After the two changes, 0.94 at the quietest (0.85 to 1.05 under load). build123d itself adds Python's calls over native's, which this rig can't measure (its OCP is broken), so against build123d Kerf is if anything a little better than these ratios.
- **"About twice as long" for a test suite:** mostly the fresh process. A test assembly is one process: `dotnet test` of an empty test took 0.9 s (five runs, 0.90 to 0.91 after the first), and a test making the rig's bracket (construction and its 10 Booleans, against the ReadyToRun build) 370 ms inside it, against 60 to 100 ms for the same Booleans settled and about 80 ms native. On the rig's figures a fresh process pays `Kerf.Modeling` about 3 s more than settled over its first 300 Booleans; with the checks' 1.3 times on top, a suite of up to about 500 Booleans takes twice native's time or more, and a larger one tends to the settled ratio, now about 1. xUnit also runs a suite's tests in parallel, where 9E found a fifth of the run stopped for collections. Python's start (importing build123d and OCP) wasn't measurable here; it isn't free either, so the figure against build123d would be a little better than against native. Without the agent's suite, how the two divide its figure can't be said more closely.
