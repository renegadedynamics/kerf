# Performance: where OCCT.NET's time goes

*Unit 9E, 2 November 2026: the performance review of phase9-plan.md section 2.4 and item 2 of
beyond-compatibility.md. Only changes that keep native's results were made; the rest are proposals.*

## Summary

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

## How it was measured

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

## The measurements

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
- **Copy at entry, `Handle` wrappers, locks:** none showed in the samples above 0.5%.

**The JIT** doesn't show in the samples (it runs in native code): a serial run spends 15 to 21 s of thread time
compiling, against native's 3.4 s for its wrappers. ReadyToRun (compiled ahead of time) cut the opening of Bento's
workbook by 37% (3,224 to 2,034 ms).

## What was fixed

Each keeps native's results: the same values, iteration order and tolerances. Decision 14101.

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

## Running the measurements again

- Bento's workload: `dotnet run -c Release --project benchmarks/OcctNet.Bento.Benchmarks -p:Build123dNetRoot=<build123d.NET>/`,
  and `--project benchmarks/OcctNet.Bento.Benchmarks/Native` for native OCCT (with build123d.NET's native build).
- The suite: `dotnet test` in build123d.NET with `-p:Build123dBackend=OcctNet -p:OcctNetRoot=<this checkout>/`, and with
  the default backend; `--logger trx` gives each test's time.
- A trace: run any of these with `DOTNET_EnableEventPipe=1`, `DOTNET_EventPipeOutputPath=<file>.nettrace` and
  `DOTNET_EventPipeConfig=Microsoft-DotNETCore-SampleProfiler:0:5,Microsoft-Windows-DotNETRuntime:0x8019:5`
  (in a process that exits normally: vstest kills its test host before the trace is written), and read it with
  PerfView or TraceEvent.
