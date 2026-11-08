# State: the kernel's and the modelling layer's statics

*Unit 10I, 3 November 2026 (phase10-plan.md, section 2.7). Counted at Kerf `main` 296531cd and after the unit with
`tools/state-survey.py` (run `python -I tools/state-survey.py <repository> <out.tsv>` from `kernel/` to recount).
Decisions 1385 to 1392. Unit 10P3, 4 November 2026, moved the BlendFunc and Contap scratch and the rest of what was cheap
and added [every field left, one by one](#every-field-left).*

## What this is

OCCT keeps configuration, caches and scratch in process-wide statics: the message table, ShapeProcess's operator
registry, the cached resource file, debug flags, parallel defaults, counters. In a library that runs many operations
at once (Bento recalculates cells in parallel), each is either a race or a way for one operation to change another's
result. Plan 2.7 asks for none to remain but constant tables, one-time initialisation and per-operation state.

This file lists every static that holds mutable state, every `[ThreadStatic]` and every lock in `kernel/src` and
`modeling/src`, with its class and what this unit did with it.

## Totals

The survey counts declarations (a line `static int a, b;` is one). The plan's estimate (about 215 statics, 25 thread
statics, 71 locks) counted more widely, `static readonly` constants included.

| | Before | After |
|---|---:|---:|
| Mutable static fields (not `readonly`, not `const`) | 46 | 14 |
| `[ThreadStatic]` fields | 21 (41 variables) | 19 (39 variables) |
| `static readonly` fields | 241 | 247 (the settings' keys added) |
| of them holding state that changes (caches, registries, ambient scopes) | 18 | 14 |
| `lock` statements | 55, on 31 objects | 23, on 13 objects |

After 10P3, by the same count, at Kerf `main` 17995a01 plus the unit: **9 mutable static fields** (from 14),
**5 `[ThreadStatic]` fields** (from 19; 5 variables, from 39), 23 `lock` statements, none of them on process-wide state
that an operation reads or writes other than the two registries filled at initialisation.

By class, as unit 10I left them (fields; a declaration of several variables counts once), before → after:

| Class | Mutable statics | `[ThreadStatic]` | `static readonly` holding state | Decision |
|---|---|---|---|---|
| Constant table | 0 | 0 | 0 (about 220 constants) | Stay. Already `readonly`; the NCollection-typed tables become frozen collections with 10J (decision 1349, 58) |
| Configuration | 17 → 0 | 0 | 4 → 0 | Settings of the operation context: immutable defaults, overrides per flow |
| One-time initialisation | 13 → 7 | 0 | 2 → 0 | `Lazy<T>`, static initialisers, or registries written only from them, published whole |
| Cache | 3 → 3 | 1 → 1 | 7 → 8 | Stay where thread-safe and invisible in results; published without a lock |
| Scratch | 9 → 0 | 16 → 14 | 0 | Per thread where that is the right scope; per instance where it crossed operations; unread counters removed |
| Ambient scopes (per flow already) | 1 → 1 | 1 → 1 | 5 → 6 | Stay: 10H1's overlay, the test scopes, the modelling layer's scopes, and the operation context |
| Test and debug hooks | 3 → 3 | 3 → 3 | 0 | Stay: off in a library's use |

## The operation context

`Kerf.Engine.Foundation.OperationContext` (Foundation, `Context/OperationContext.cs`; decision 1385):

- **A setting** is a `ContextKey<T>`: a name and an immutable default, declared as a `static readonly` field by the
  class it configures (`Options.ParallelModeSetting`, `MsgFile.AdditionsSetting`, `ProcessContext.ResourceOverridesSetting`).
  Foundation knows none of the others' settings.
- **A context** is immutable: the few settings it overrides, as key and value pairs. `With(key, value)` gives a new one.
  `OperationContext.Default` overrides nothing.
- **The current context** is an `AsyncLocal<OperationContext?>`. It flows into the tasks and `Parallel.For` bodies an
  operation starts and into every operation nested in it; independent operations elsewhere see their own. Reading a
  setting where nothing was set costs one `AsyncLocal` read.
- **Setting:** `using (OperationContext.Begin(key, value)) { … }` for a scope, restored on disposal (or `Begin(context)`
  for several); `OperationContext.Set(key, value)`, behind OCCT's process-wide setters (`Quantity_Color::SetEpsilon`,
  `BOPAlgo_Options::SetParallelMode`, `GeomTools::SetUndefinedTypeHandler`…), for the rest of the current flow. A flow's
  setting ends with it: an `async` method's caller, a finished task, another test, another Bento cell never see it.
- **Ambient, not parameters** (decision 1386): the settings are read deep in OCCT's call graph (the message table by
  every algorithm that reports, BRepLib's plane by every 2D maker, the mesher's default by every factory call), so
  threading a parameter would touch hundreds of signatures; an explicit parameter stays where the chain is short (a
  `ProcessContext` already carries its resource manager; an algorithm its own `SetRunParallel`).
- **The working overlay** (10H1's `WorkingOverlay`, ModelingData) is the topology part of an operation's state. It flows
  the same way, through its own `AsyncLocal`, mirrored in a thread static its accessors read on every TShape access
  (decision 1359). It stays separate: one more indirection on its hot path costs, and 10H2 is changing it.
- **No scratch pools in the context** (decision 1387): the scratch that exists is per thread by nature (an
  operation's parallel bodies run on several threads, so one pool per operation would race), and 10J's buffers come
  from `ArrayPool` or the stack.

## Configuration: into the context

| Where | OCCT | Setting | Setter |
|---|---|---|---|
| Foundation/Message/Message.cs | the default messenger (`s_defaultMessenger`, and the flow's `FlowMessenger`) | `Message.MessengerSetting` | `SetDefaultMessenger`, `SetFlowMessenger` (internal) |
| Foundation/Message/Message.cs | the default report, made on first use (`s_defaultReport`, under a lock) | `Message.ReportSetting` | `SetDefaultReport` (internal); `DefaultReport(true)` makes one for the flow |
| Foundation/Message/MsgFile.cs | the message table (a dictionary under a lock) | defaults + `MsgFile.AdditionsSetting` | `AddMsg`, `LoadFromString` (internal) to the flow; `AddDefaults` for the compiled-in texts |
| Foundation/Quantity/Color.cs | `Quantity_Color::Epsilon` | `Color.EpsilonSetting` | `SetEpsilon` |
| ModelingData/GeomTools/GeomTools.cs | the undefined-type handler | `GeomTools.HandlerSetting` | `SetUndefinedTypeHandler` |
| ModelingAlgorithms/AppBlend/AppBlendDebug.cs | two approximation switches | `AppBlend.ContextSplineApprox`, `…ApproxWithNoTgt` | internal setters |
| ModelingAlgorithms/Blend/BlendDebug.cs | two trace flags | `Blend.traceDRAWSECT`, `Blend.contextNOTESTDEFL` | the `Blend_Set…` functions |
| ModelingAlgorithms/BOPAlgo/Options.cs | the global parallel mode | `Options.ParallelModeSetting` | `SetParallelMode` (internal) |
| ModelingAlgorithms/BRepLib/BrepLibrary.cs | BRepLib's precision and plane | `PrecisionSetting`, `PlaneSetting` | `Precision(p)`, `Plane(p)` (internal) |
| ModelingAlgorithms/BRepMesh/IncrementalMesh.cs | the factories' parallel default | `ParallelDefaultSetting` | `SetParallelDefault` (internal) |
| ModelingAlgorithms/BRepMesh/DiscretFactory.cs | the meshing algorithm's name | `DiscretFactory.DefaultNameSetting` | `SetDefaultName` (internal) |
| ModelingAlgorithms/ShapeAlgo/ShapeAlgo.cs | the AlgoContainer | a `Lazy` default + `ShapeAlgo.AlgoContainer` | `SetAlgoContainer` |
| ModelingAlgorithms/ShapeProcess/ShapeProcess.cs | the operator registry (a dictionary under a lock) | defaults + `ShapeProcess.OperatorsSetting` | `RegisterOperator` to the flow |
| ModelingAlgorithms/ShapeProcess/ProcessContext.cs | the cached resource file (five statics under a mutex) | `ProcessContext.ResourceOverridesSetting` (new) | a scope |

The three tables the plan names (decision 1388):

- **The message table:** immutable defaults, the texts compiled into the kernel (the Booleans', Shape Healing's, data
  exchange's), each set added once by its owner's `Lazy` initialisation (`MsgFile.AddDefaults`), the table replaced
  whole by compare-and-swap so readers take no lock; over them, the flow's additions. `Msg` of an unknown keyword gives
  OCCT's surrogate text without storing it (OCCT stores it, so that `HasMsg` answers true for it afterwards in the whole
  process: a read that writes; `AlertDiagnostics` asks `HasMsg` before `Msg`, so an earlier operation's unknown key
  changed a later operation's message).
- **ShapeProcess's registry:** Shape Healing's 18 operators, published once by `OperLibrary.Init` as a frozen
  dictionary; operators a caller registers go to the flow. A name can't be registered twice, as in OCCT.
- **`Resource_Manager`'s settings:** OCCT reloads the last resource file into statics when its name or modification
  time changes, behind a mutex. Each compiled-in file is now parsed once (a cache keyed by name, invisible: a context
  gets a copy); files from the caller's directories are read each time; and `ResourceOverridesSetting` overrides
  resources (by full name) in every context loaded in the flow, the STEP and IGES readers' own included: an operation
  can run with its own shape processing parameters beside others on the defaults (`ConcurrencyTests`).
  `ProcessContext.ClearResourceCache`, for Draw-lite's corpus, goes with the cache.

## One-time initialisation

| Where | Before | After |
|---|---|---|
| ModelingAlgorithms/BOPAlgo/BOPAlgo_msg.cs | a flag and a lock | `Lazy<bool>` adding the texts to the defaults |
| ModelingAlgorithms/ShapeExtend/ShapeExtend.cs | a flag and a lock | `Lazy<bool>` |
| DataExchange/Interface/StaticSet.cs (`LoadMessages`) | a lock | `Lazy<bool>` |
| ModelingAlgorithms/ShapeProcess/OperLibrary.cs | a flag and a lock | `Lazy<bool>` publishing the operators |
| ModelingAlgorithms/ShapeAlgo/ShapeAlgo.cs | a flag, a container and a lock | `Lazy<AlgoContainer>` |
| DataExchange/STEPControl/StepController.cs, STEPCAFControl/CafController.cs | a recorded controller under a lock | `Lazy<…>` |
| Foundation/PLib/HermitJacobi.cs | three matrices made on first use (`LazyInitializer`) | `Lazy<Matrix>` (10P3) |
| ModelingData/AdvApp2Var/MathBase.cs (`mmprcsn_`) | the precision block, written once in the static constructor by `mmwprcs_` | constants (10P3) |
| ModelingAlgorithms/BRepMesh/DiscretAlgoFactory.cs, IncrementalMeshFactory.cs, XBRepMesh/Factory.cs | a list under a lock, filled by self-registering singletons | an array built in the static initialiser |
| ModelingAlgorithms/ShapeBuild/ShapeBuildBuilder.cs | the XOY plane made on first use | `static readonly` |
| DataExchange/XSControl/Controller.cs | the recorded controllers under a lock | an immutable dictionary replaced by compare-and-swap |
| DataExchange/LibCtl/Library.cs (`s_theGlobal`), StepData/StepData.cs (`s_theheader`) | registries filled by the protocols' one-time initialisations, under a writer lock | unchanged: written only from `Lazy` initialisers |
| the protocols, Unicode tables, `TypedValue`'s library (12 `Lazy<T>`) | | unchanged |

## Caches

| Where | Decision |
|---|---|
| Foundation/Poly/Triangulation.cs, the cached range | per instance, shared by every operation holding the frozen mesh: was a lock; now each box is published whole (`Volatile`), never changed once published |
| ModelingData/FEmTool/LinearFlexion.cs, LinearJerk.cs, LinearTension.cs | per order, under a lock: now published by compare-and-swap (computed twice at worst; the values are the same) |
| DataExchange/XdeDoc/DocumentProduct.cs, an assembly's compound | under a lock: now published by compare-and-swap (racing threads made equal compounds already) |
| Foundation/TopLoc/SListNodeOfItemLocation.cs (`_inverse`) | per chain head, shared by every shape holding that placement (unit 11D): the inverse, published whole (`Volatile`). Threads racing make equal chains, and either may stay. `IsEqual` and the hash compare datums and powers, so which one stays can't show |
| ModelingAlgorithms/ShapeProcess/ProcessContext.cs | see above: the compiled-in files, parsed once |
| Foundation/Results/FailureDiagnostics.cs (`Codes`) | a `ConcurrentDictionary` of codes by exception type: stays |
| Foundation/Standard/StreamUtilities.cs, ModelingData/BinTools/BinTools.cs | `ConditionalWeakTable`s of a stream's state: per stream, thread-safe, stay |
| ModelingData/AdvApp2Var/MathBase.cs (`t_mmcmher`) | per thread (decision 113): stays |

## Scratch

| Where | Decision |
|---|---|
| ModelingAlgorithms/BlendFunc/ConstRad.cs, EvolRad.cs (14 variables each) | **per instance** (10P3). OCCT's are `thread_local`, read across calls when an instance's guard skips a positioning (decision 1037). A new instance always positions first, so the scratch belongs with the guards that decide when it is read, which are the instance's. Two instances interleaved on one thread no longer read each other's guide point; no recording or fillet corpus case depended on that |
| ModelingAlgorithms/Contap/HContTool.cs (4 bounds) | **gone** (10P3): `SamplePoint` works the bounds out from its surface, the one `NbSamplePoints` was just given by its only two callers |
| ModelingAlgorithms/TopOpeBRepTool/SC.cs | **gone** (10P3): its one caller makes its own classifier, and it no longer holds on to a thread's last shapes |
| ModelingData/GeomTools/GeomTools.cs (`s_nesting`) | stays: a reader's recursion depth, on its own thread |
| ModelingAlgorithms/BRepAlgo/ShapeImage.cs (`s_noImage`), AsDes.cs (`s_empty`) | **per instance** (decision 1389). OCCT's static list of `Image` grows with every shape without images and is returned to the caller, and AsDes's shared "empty" list is writable through `ChangeDescendant`: per thread, what one operation appended reached the next operation on that thread |
| ModelingAlgorithms/HLRBRep/HiddenLineData.cs (7 counters) | **removed**: written without synchronisation on every intersection, read by nothing (OCCT's `OCCT_DEBUG` traces) |
| ModelingData/AdvApp2Var/SysBase.cs (`mblank__`) | **removed**: written by `macinit_`, read by nothing |
| DataExchange/STEPConstruct/ConstructAssembly.cs (`s_processNauoId`) | **removed** with the overload that used it (decision 1389): it numbered assembly occurrences from a process-wide counter, so an output depended on what the process wrote before; the writer already passed its own (decision 1266) |
| ModelingAlgorithms/BRepGProp/MeasurementGauss.cs (`GaussWork.t_spare`), Foundation/math/Bfgs.cs (`Work.t_spare`) | **new** (11Q, decision 1708): a thread's spare work vectors, out of the static while in use and zeroed on the way back in. Nothing crosses threads |
| ModelingData/BRep/Tool.cs (`PlaneCurves.t_entries`) | **new** (11Q, decision 1715): a thread's latest curves on planes, at most 64, for frozen geometry only, and a hit hands out a copy. Nothing crosses threads |
| ModelingData/Extrema/GenExtPS.cs (`WorkGrids.t_spare`), Foundation/math/FunctionSetRoot.cs (`Work.t_spare`), Foundation/CSLib/Class2d.cs (`Class2dPoints.t_spare`) | **new** (11Q, decision 1716): a thread's spare work, out of the static while in use. Each is written before it's read. Nothing crosses threads |
| ModelingData/Extrema/GenExtPS.cs (`WorkGrids.t_largeSpare`) | **new** (11Q, decision 1754): a thread's larger grids, held weakly. Nothing crosses threads |
| ModelingAlgorithms/IntCurve/SampledPolygons.cs (`t_current`) | **new** (11Q, decision 1755): the table of a 2D offset's trimming, set only around `IntersectWith`. Nothing crosses threads |
| ModelingData/AdvApprox/ApproxAFunction.cs (`CoefficientsWork.t_spare`), Foundation/BSplCLib/BSplCLib.cs (`t_localMatrices`) | **new** (11Q, decision 1758): a thread's work, zeroed before use. Nothing crosses threads |
| ModelingAlgorithms/BRepMesh/MeshWorkStorage.cs (`t_spare`) | **new** (11Q, decision 1759): a thread's lists for meshing a face, out of the static while in use. Nothing crosses threads |
| Foundation/TopLoc/Placement.cs (`ProductMemo.s_entries`, `s_next`) | **new** (11Q, decision 1842): a thread's last four products of placements, chains that never change. Nothing crosses threads |
| ModelingAlgorithms/GeomFill/QuasiAngularConvertor.cs (`t_ofThread`) | **new** (11Q, decision 1766): a thread's convertor, initialised, for one `Section` at a time, which writes every vector it reads. Nothing crosses threads |
| Foundation/math/Bfgs.cs (`Work.t_largeSpare`), ModelingData/AdvApprox/ApproxAFunction.cs (`CoefficientsWork.t_largeSpare`) | **new** (11Q, decision 1763): a thread's larger work, held weakly. `MeasurementGauss`'s `t_spare` is weak now too, and the mesher's larger arrays (decision 1762). Nothing crosses threads |
| the shared empty values (`BOPDS_DS`'s pave blocks and face information, `TopoDS_Iterator`'s, BRepFill's, ChFi3d's, TopOpeBRepDS's lists) | constants: read-only for callers, as OCCT's; 10J returns read-only types |

**What each thread keeps, all together** (decision 1761; measured in performance.md, "The fifth slice"). These hold memory for as long as their thread lives; the thread pool lets its idle threads go after about 20 s, a host's own threads don't. Limits are strong references; anything larger is weak or not kept.

| Set | Most a thread keeps | Typical, Bento's examples |
|---|---:|---:|
| `MeshWorkStorage` | 8,192 triangles and circles, link nodes and cell nodes; 4,096 links and buckets (decisions 1875, 1876): about 610 kB | not measured apart; 23.6 MB over all the sets on about 50 threads |
| `GenExtPS.WorkGrids` | 1,024 cells, 200 kB | none |
| `Class2dPoints` | 4,096 points, 65 kB | 8 kB |
| `ApproxAFunction.CoefficientsWork` | 8,192 values, 64 kB | none |
| `Bfgs.Work` | 64 unknowns, 36 kB | 1 kB |
| `Tool.PlaneCurves` | 64 answers, with their curves and planes | 20 kB |
| `FunctionSetRoot.Work` | 16 unknowns, 1,024 iterations, about 9 kB | 2 kB |
| `BSplCLib.t_localMatrices` | one small matrix for each shape | 6.5 kB |
| `QuasiAngularConvertor` | about 1 kB | none |
| `MeasurementGauss.GaussWork` | nothing strongly; about 500 kB weakly | none |
| `MathBase.t_mmcmher` | one small record, fillet approximations only | none |
| `GeomTools.s_nesting`, `WorkingOverlay.t_current`, `SampledPolygons.t_current`, the walkings' step traces | a number, or null outside an operation | none |

The limits add up to about 1 MB a thread: 24 MB on 24 threads.

`ProductMemo` adds four placement chains a thread, a few kB unless a chain is very long. **Two statics are for the process:** `SharedArrays<T>.s_slots` (decision 1841), arrays of frozen splines, at most 1,024 of up to 32 values for each of `double` and `int`, about 300 kB and 180 kB; a slot is one reference, so a race can only lose an entry. And `NCollection_List<T>.s_sharedEmpty` (decision 1840), one empty list for each `T`, which refuses to grow.

**Unit 11C looked at what one-time initialisation costs on opening a workbook:** 2 to 5 of about 255 samples on the first pass's working thread, spread over dozens of small types. So nothing was made lazy, and this list is unchanged (decision 1693).

**Unit 11G adds no static, thread static or lock:** `NewSurfaceModification` and the reconstruction's `_onNewSurface` belong to one fillet's builder (decision 1721).

**The known defects unit adds no static, thread static or lock:** `Reconstruction.LostFaces` belongs to one fillet's builder, and `FollowsCurve` asks the projector's own `SurfaceAnalysis`, as the rest of that class does (decisions 1735, 1737).

**Unit 11P's `DrawCaseProcess.Own` is Draw-lite's, not the library's:** set once in that process's `Main`, before anything else runs, and only read from then on. In the tests' process it stays `None`.

## Test and debug hooks: stay

| Where | Why |
|---|---|
| ModelingAlgorithms/IntWalk/IWalking.cs, PWalking.cs (`s_threadStepTrace`, `[ThreadStatic]`) | the walking's step trace for its recorded comparisons (decision 687); null in a library's use |
| ModelingAlgorithms/IntPatch/Polygo.cs (`s_dumpNum`) | numbers `Dump`'s output (Draw's), `Interlocked` |
| Foundation/OSD/OsdParallel.cs (`ForceSerial`, `MaxDegree`) | `AsyncLocal` test scopes (decision 619): per flow already |
| ModelingData/BRepTools/WorkingOverlay.cs (`s_openCount`, `s_current`, `t_current`) | 10H1's overlay: per operation already (`Interlocked` counter, `AsyncLocal`) |
| modeling/…/Topology/ShapeLog.cs | the differential run's shape log, on only with `KERF_SHAPE_LOG`; its lock orders writes to one file |
| modeling/…/Topology/Shape.Booleans.cs (`Depth`), Warnings.cs (`Current`) | the modelling layer's `AsyncLocal` scopes: per flow already |

## Locks

Every lock left (decision 1391):

| Where | What it guards | Why it stays |
|---|---|---|
| DataExchange/LibCtl/Library.cs | the global list of a library's modules | written only by the protocols' one-time initialisations; readers take none |
| DataExchange/StepData/StepData.cs | the header protocol | written only by RWHeaderSection's one-time initialisation |
| DataExchange/XSControl/ExchangeWorkSession.cs (2) | a controller's actors, while a session makes a model or transfers a shape | was one process-wide mutex (OCCT's); now the controller's: sessions on a shared recorded controller take turns, and every STEP reader and writer has its own (decision 1264), so none waits for another |
| ModelingAlgorithms/BOPAlgo/BOPAlgo_PaveFiller_7.cs (`_verticesLock`) | a vertex's tolerance, raised by the filler's parallel p-curve makers (decision 789) | the filler's own since 10P3; another operation writes to its own working copies |
| ModelingAlgorithms/BRepExtrema/DistanceShapeShape.cs | the inner-solid search's result, from its parallel bodies | one operation's own, per call |
| Foundation/BVH/BuildQueue.cs, QueueBuilder.cs (3) | the parallel BVH builder's queue | one builder's own |
| Foundation/Message/Report.cs (6), Message_Progress.cs (3) | an algorithm's report and progress, written by its parallel bodies (OCCT's mutexes) | one operation's own objects |
| ModelingData/BRepTools/WorkingOverlay.cs | making a working copy, from an operation's parallel bodies (decision 1359) | one operation's own overlay |
| DataExchange/StepSelect/StepType.cs | a signature's library lookup (OCCT's mutex) | one object's own |
| modeling/…/Topology/Warnings.cs (2) | a warning scope's messages, from the operation's tasks | one scope's own |
| modeling/…/Topology/ShapeLog.cs | the shape log's file | test tooling, off by default |

None guards process-wide state that an operation reads or writes, apart from the two registries filled at
initialisation. The `Lazy<T>`s lock only while they initialise. Plan 2.7 asks for no locks but one-time initialisation's;
the ones that stay guard one object against one operation's own parallel bodies, where the alternative is a data race,
and only `ExchangeWorkSession`'s can make one session wait for another, on a controller they chose to share.

**Unit 11D settled plan 2.7 by measurement: all 23 stay, because they cost nothing.** A trace of three rounds of Bento's
examples, 24 copies on 24 logical processors under the server collector with PGO off (46 s of processor time), found 12 contentions and 0.35 ms of
waiting in all. Every one of them was in the runtime's own `Parallel.For` (`ManualResetEventSlim`, in `Task.SpinThenBlockingWait` and
`RunContinuations`), and none on any lock in this table. `tools/contention-report.cs` reads such a trace, and the rig's
"contended locks in a round" row keeps the count. Removing a lock here would swap a lock nobody waits for
for a race or a compare-and-swap (decision 1686). The trace covers what Bento's examples reach, and the STEP session locks and the
BVH builder's queue aren't among that. Those are per object too, so they can't make independent workbooks wait for one another.

## Every field left

Every row `tools/state-survey.py` reports as mutable or `[ThreadStatic]` after 10P3, and why it isn't context or
instance state (plan 2.7, the audit's question). None changes a result.

| Field | Kind | What it is | Why it stays |
|---|---|---|---|
| Foundation/Message/MsgFile.cs `s_defaults` | one-time | the compiled-in message texts | an immutable table, swapped whole by compare-and-swap when an owner's `Lazy` adds its set; only ever gains the same texts |
| ModelingAlgorithms/ShapeProcess/ShapeProcess.cs `s_defaultOperators` | one-time | Shape Healing's 18 operators | a frozen dictionary published once by `OperLibrary`'s `Lazy` |
| DataExchange/LibCtl/Library.cs `s_theGlobal` | one-time | a library's module list | written only by the protocols' `Lazy` initialisers, under the writer lock; readers take none |
| DataExchange/StepData/StepData.cs `s_theheader` | one-time | the header protocol | written only by RWHeaderSection's one-time initialisation |
| DataExchange/XSControl/Controller.cs `s_listad` | one-time | the recorded controllers, by name | an immutable dictionary swapped by compare-and-swap; each controller records itself from its `Lazy` |
| ModelingData/BRepTools/WorkingOverlay.cs `s_openCount` | per operation | how many overlays are open | `Interlocked`; zero is the accessors' fast path. Counts operations but no result reads it |
| ModelingData/BRepTools/WorkingOverlay.cs `t_current` (`[ThreadStatic]`) | per operation | the flow's overlay, mirrored from its `AsyncLocal` | a field read on every TShape access, where the `AsyncLocal` would cost a lookup (decision 1359) |
| ModelingData/GeomTools/GeomTools.cs `s_nesting` (`[ThreadStatic]`) | scratch | a reader's recursion depth, against hostile input | entered and left in a `using` on one thread; threading a depth through every reader's recursion would touch all the geometry readers for nothing |
| ModelingData/AdvApp2Var/MathBase.cs `t_mmcmher` (`[ThreadStatic]`) | cache | `mmherm0_`'s Hermite coefficients, read by `mmherm1_` | f2c-translated static functions with no instance to hold it; per thread it is safe (decision 113), and each computation refills what it reads |
| ModelingAlgorithms/IntWalk/IWalking.cs, PWalking.cs `s_threadStepTrace` (`[ThreadStatic]`, 2) | test hook | the walking's step trace for recorded comparisons (decision 687) | null in a library's use; a test sets it on its own thread |
| ModelingAlgorithms/IntPatch/Polygo.cs `s_dumpNum` | debug | numbers the polygons `Dump` prints | `Interlocked`, read only by Draw's `Dump`; OCCT's printed text |
| modeling/…/Topology/ShapeLog.cs `s_test`, `s_seq` | test tooling | the differential run's current test and sequence number | only with `KERF_SHAPE_LOG`, under its lock; off in a library's use |

## Measurements

Decision 1392. Release, 24 logical processors, other units running beside (the machine's noise is in the figures):

| | Before | After |
|---|---:|---:|
| Bento's examples, serial: sum of the medians | 1,545 ms | 1,469 ms |
| allocated by the largest example (extrude until) | 802 MB | 802 MB |
| Bento's examples, 24 copies of each at once: a round | 11.6 s | 10.7 s |
| allocated in a round | 27.7 GB | 27.7 GB |
| results differing from the serial run | 0 | 0 |
| Booleans (407 simple cases), serial | 2,651 ms | 2,693 ms |
| Booleans, each in parallel | 3,030 ms (a noisy run) | 1,993 ms |
| allocated per Boolean | 8.26 MB | 8.25 MB |

No cost found. The parallel rounds are dominated by workstation GC (23 to 25 s of pauses in each run), not by
contention: performance.md's proposal 1 (server GC in parallel hosts) is what would speed them.

10P3's moves were checked the same way (ConcurrencyTests, and Bento's parallel rounds), with no result differing.

## Draw-lite

Each `DrawInterpreter.Eval` runs on a thread of its own (for Tcl's stack), so a setting a command makes would end with
the call. The interpreter keeps an operation context (`Context`) from one `Eval` to the next, starting from its
creator's: a session's settings last as DRAWEXE's do, and parallel interpreters (the corpus runs cases in parallel)
don't see each other's.

## The modelling layer's `HistoryLog`

`Kerf.Modeling` keeps one piece of ambient state: `HistoryLog`, an internal `AsyncLocal` scope. An operation of a `Part`, `Sketch` or `Curve` opens one, the shared operations note
their record in it, and the object keeps what it finds on its result. It is per flow of execution, so parallel workbooks don't see each other's.

Unit 10Q2 looked again now that the layer is on structs, and kept it (decision 1638). There are 35 notes in 14 files. A struct's public operation must return the bare shape (`solid.Fillet(…)` is a `Solid`), so
threading `(shape, record)` needs a second, internal form of each operation, about thirty signatures doubled, to remove one private field that no caller can see. A later unit that
reworks those operations anyway could thread it then.

## The clean-up

The clean-up after decision 1681 deleted fields that were written and never read (78), none of them shared state that anything read, and changed no ambient state. The second pass (decisions 1744 to 1748) changed names, docs and private signatures only. See [clean-up.md](clean-up.md).

## Unit 11S

The fixes for OCCT's defects (decisions 1775 to 1788) add no shared state: all are local to a call. `SamplingHelpers`' two new methods and the offset's concentric case work on their arguments alone. `StepGenTests` writes to a temporary folder of its own.

**The third pass (decisions 1830 to 1838)** adds one piece of state: `BrepTopolTool`'s classifiers per tolerance, an immutable list whose head is replaced whole (decision 1835). A tool shared between threads may build one twice and can't read one for another tolerance. The rest is local to a call.

## A check's memory

Not merged (decision 1798), so nothing changes here. On `phase11/valid-cache`: one field on `BrepTFace`, an immutable record replaced whole by `Volatile.Write`, and in `ValidMemory` a setting read once from the environment and counters that only the comparison mode touches.
## The owner's answers

Decisions 1802 to 1810 add one piece of shared state: `TopoUtil.Measured` and `TopoUtil.MeasuredMoments`, immutable `MassOptions` held in statics and only read. The retry in `Sweeps.Sweep` and the slice in `Solid.Torus` work on their arguments alone. `SweepMode` is immutable. `Failures.TryWarned` uses `Warnings.Capture`, whose scope is an `AsyncLocal`, so it belongs to the call that opened it.
The second pass (decisions 1811 to 1822) adds no static, thread static or lock, and removes one: `GuideTrihedronAC.ReplayNative`, an `AsyncLocal`. `UnifySameDomain`'s saved pcurves are local to one merge.
## The check on small shapes

Decisions 1823 to 1828 add one static, `Analyzer.Finished`: a delegate for tests, set once at a test process's start before any check when `KERF_CHECK_COMPARE` is set, and otherwise always null. A result's `ContextMap` is under the same lock as the dictionary it replaces when on the pool, and that lock is made before the pool starts; the `IndexedSet` for sub-shapes in the analyzer is one for each chunk, so each thread has its own. `_line` on an edge's check is written while the map is built, on one thread, and only read after. A wire's map kept from `Minimum` is no longer written by `Closed`, so there's less to race than before. The comparison's counters in `tests/Shared/FullCheck` are `Interlocked`, and in tests only.

## Fillet limits

Decisions 1847 to 1852 add no static, thread static or lock. `FilletLimits` works on its arguments, its `Neighbours` belong to one call, and its Boolean runs in the calling thread, on the caller's frozen shape, as any Boolean does.
Unit 11T (decisions 1853 to 1858) adds one static: `Kerf.Cancellation`'s `AsyncLocal<CancellationToken>`, per flow, as the other `AsyncLocal` scopes are, and flowing into parallel loops the same way. A scope that is set is cleared by the `using` that set it. `MeshBudget` is one object for each `MeshOptions.ToEngine()`, shared by the faces of one mesh: its total is `Interlocked`, and it decides only whether to stop, so it can't change a mesh that finishes. `TShape._compoundNesting` and `TCompound._walkLeaves` are written while a shape is being built, before it's frozen or shared, as its child list is. A field in `TShape` takes two bytes that were padding (`TCompound` gains eight of its own).
## Booleans check their result's volume

Decisions 1860 to 1868 add no shared state: `BooleanVolumes` has one static, `Measured`, an immutable `MassOptions` only read, as `TopoUtil.Measured` is, and everything else is local to a call. A volume isn't remembered on a shape (decision 1864). Each call makes its own classifiers.

Unit 11T's second pass (decisions 1879 to 1884) adds no shared state: `MassOptions.Default` and `MeasureOperations.Moments` are immutable and only read, and `AppFuncRoot._searches` is an instance's, with one instance for each approximation, on one thread.

Units 12A and 12B (decisions 1894 and 1895) add no shared state: `PlanarFaceMass.TryCentre` and `TopoUtil.NothingToCleanAfterExtruding` work only on locals and read frozen shapes.
Unit 12D (decisions 1896 and 1897) adds no shared state to Kerf. The rig's `FirstLoad` runs once, before `Main`, on one thread. On Bento's branch, `SharedLibraryContext`'s table of contexts is read and written only under its lock, and each context's own table never changes once made.
Units 12K to 12M (decisions 1904 to 1911) add no shared state: `FilletLimits`'s tools, `Neighbours` and `Mitre` are local to the call, its Gauss points are constant arrays read only, and its Booleans and `UnifySameDomain` run on the caller's thread, on copies (`SetNonDestructive`, `SetSafeInputMode`).

Unit 12C (decisions 1898 to 1901) adds no shared state: `FilletLimits.Judge` and its `Neighbours` are local to the call, on frozen shapes, `WellApart` is a constant, and `MaxFillet` holds its limit in a local.

Units 12H and 12I (decisions 1913 to 1921) add no shared state: `StepLimit` is static with only a constant, and each count is a local of its loop; `OffsetVolumes`' options are immutable and only read; `WithDirectives` is a pure function; a compound's count is set as it's built, before it's frozen, as it was.
Unit 12O (decisions 1953 to 1955) adds no shared state: `SphereDeflectionMeshAlgo` and `SphereRangeSplitter` are one face's, made by the factory for it; a `CellFilter`'s sweep, its `Dead` test and its counts are fields of one face's `CircleTool`, on one thread.
Unit 12H2 (decisions 1942 to 1946) shares one thing: `MeshBudget`'s count of refused faces, which faces meshed in parallel add to with `Interlocked` and the mesh reads once they've all finished, as it already did the triangles. The circles' count and the polygon walks' work are fields of a `CircleTool` and a `Delaun`, each one face's, on one thread; the limits are constants.
Units 12E, 12F and 12G (decisions 1922 to 1925) add no shared state: `HidesCorner`, `TouchSpan` and `ProvedByPoints` work on locals and read frozen shapes, each `SolidClassifier` is local to one call, and the new fields are constants.
Unit 12P (decisions 1947 to 1951) adds no shared state: a curve adaptor's kept ends are its own, as its cache is, and an adaptor is used on one thread; `OffsetOperations.Checked` and `OffsetVolumes.FillsItsCavity` work on locals and frozen shapes, each classifier local to one call; `Kerf.Modeling`'s `WarningScope` keeps a code beside each message in the one list it had, under the same lock.
Unit 12N (decisions 1938 to 1941) adds no shared state: the retried offset's options are a new immutable record, and the rest are locals. Units 12H and 12I (decisions 1913 to 1921) add no shared state: `StepLimit` is static with only a constant, and each count is a local of its loop; `OffsetVolumes`' options are immutable and only read; `WithDirectives` is a pure function; a compound's count is set as it's built, before it's frozen, as it was.
