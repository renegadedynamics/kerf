# Changelog

All notable changes to the Kerf.Kernel packages (until 0.2.0 the OCCT.NET packages). Versions follow [SemVer](https://semver.org); before 1.0 a minor
version may break the API.

## Unreleased

- **A Cut, Common or Fuse no longer goes wrong where a section curve stops just short of another:** on #1659's loft and cylinder the Cut removed nothing, which had become a failure, and now it's right (decision 22610).
- **A closed section curve between surfaces that cross at a shallow angle is no longer lost as a tangency,** which gave a Common with an edge missing and the wrong area (decision 22608).
- **A chamfer that fails says which edges failed and why,** and a 2D chamfer given the same corner twice says so, where it was an internal fault (decision 22609).
- **A Common or Cut of faces with a solid fails, carrying the shape, if it kept the wrong side of the solid at points it can prove it:** a lost section edge used to pass with 24% too much area (decision 22606).
- **`GeneralTransform3D.ToTransform` accepts a rotation and a uniform scale given as a matrix,** which it refused (decision 22607).
- **A cone from a STEP file whose seam leans by a rounding error now meshes:** it had no triangles (decision 22605).
- **A thin torus's mesh keeps to the deflection asked:** it could be three times out, and say nothing (decision 22604).
- **An extrusion along its own plane, a loft of sections in one plane and a sweep along the section's plane now fail,** carrying the empty solid, where they reported success (decision 22602).
- **A minimum distance between parts that intersect is zero:** where their faces cross inside both, with no edge or vertex involved, it used to report the nearest vertex (decision 22601).
- **A parallel operation's progress never runs backwards,** and a progress callback gets its reports one at a time (decision 1903).
- **A cube rounded all over is a sphere, a bar is a capsule, and a plate is a pillow, at the radius that just fits:** fillets where edges meet at a limit now work, with corners on exact spheres, as do slanted ends, a chain round a top, an L's top with its re-entrant corner, and convex and concave edges that touch. `MaxFillet` gives that radius itself. What the fillet algorithm makes invalid is now also tried by this path before it's reported.

- **A disc's rim rounded to its full height, full rounds, hemispheres and a sphere from a short cylinder work,** as does a hole's rim or a boss's foot rounded to its wall's height: circular edges between a plane, cylinder or cone, with the result checked by Pappus, on exact tori and spheres.

- **A fillet too large for a circular edge between a plane, cylinder or cone now says what fits:** `FilletLimit` gives the radius, and `Fillet` refuses above it at once, with a radius within tolerance of it taken as it.

- **`CentreOfArea()`** gives the centre of a shape's faces' area without its moments, exact for a planar face bounded by lines and circles.
- **A fillet that can't fit fails at once and says what does,** where it could take seconds; a radius within the shape's tolerance of the limit is the limit, the same in every figure, on either side (decisions 1898, 1899). All of a cube's edges past half its side used to fail with `result.invalid`, carrying what it made; now with `fillet.too-large`. A fillet just below a limit, by under the tolerance, no longer leaves faces narrower than that.
- **`FilletLimit` no longer claims a limit where the next face is within 0.1 radians,** as the fillet rolls on to that face, rightly.

## 0.3.0

- **Bento's examples recalculate about 4.6 times as fast as at 0.2.0, with 72% fewer bytes:** 265 ms from 1,211 ms, where native OCCT 8.0.1 takes 685 ms, and 126 MB from 450 MB, measured on a quiet machine with ReadyToRun (`docs/performance.md`). Opening them, cold, takes 518 ms from 1,764 ms (native 921 ms). A host's settings, as in `docs/performance.md`, give a recalculation of 166 ms and 24 copies in parallel in 530 ms, a speed-up of 7.5×. A host that loads Kerf collectibly or from bytes gets none of the ReadyToRun figures: the runtime compiles it all, and the first operation takes much longer (`docs/performance.md`, "Where it stands").
- **Recalculation and a parallel round are about a fifth quicker, with about a fifth fewer bytes,** on Bento's examples, with the same shapes and meshes (a loaded machine). `IntegrateVolume` and `IntegrateArea` with a tolerance give planar faces exactly, so a result can differ from before in its last digits.
- **Equal cylinders that nearly cross are right:** a round notch, offset by rounding or an imported file, could remove nothing or fail; now Cut, Common and Fuse are right at the offsets measured, for radii of 0.5 and 5 (`docs/reliability.md`). A radius of 500 can still fail.
- **`IntegrateVolume()` and `IntegrateArea()` are precise by default,** to 1e-6: they could be 18% out on faces bounded by a walked intersection. `MassOptions { Tolerance = 0 }` asks for the fixed points.
- **A fillet that can't be made fails sooner:** 0.2 to 0.6 s where it took 4 to 7 s. **`Face.Classify` with a NaN or infinite parameter** answers `Out`; it never returned.
- **A fuse as large as both its inputs, with points inside both, fails.**
- **Kept shapes take less memory, and importing large STEP files peaks lower:** a patterned shape up to 55% less and a STEP import with many B-splines 35% less, as equal data is now shared, and about 20% less otherwise; `bug30790`'s largest live data during its import fell from 178 to 110 MB. Every shape, mesh and record reads the same.
- **Fillets and chamfers that just fit work:** a radius equal to a face's width on a straight edge between planes, two fillets that meet on a face (a plate's end fully rounded), four edges that make a cylinder, and a chamfer of a face's whole width are built exactly, where they used to fail although a hair less worked. Only what used to fail changes. Corners and circular edges at a limit still fail. A size that doesn't fit fails with `FilletOperations.TooLarge`, saying what fits, and `FilletLimit` gives it (decisions 1847 to 1849).
- **Nothing a user or a file controls should end the process now** (`docs/reliability.md`). Compounds and `ModelNode` trees nest at most 64 levels, and past that `Compound.Create` and `ModelNode.Assembly` throw `ArgumentException`, and a STEP, glTF or BREP file fails: a STEP file of assemblies 990 levels deep used to end it with a stack overflow, as did composite curves 3,000 deep, which now fail like it (deep trimmed curves and offset surfaces, which used to go missing silently, fail too), as did about 3,000 moves of a shape, or `Transformed` on 300 levels. A compound whose walk would visit more than 100 million shapes is refused too (decision 1857).
- **`Kerf.Cancellation`:** `using (Cancellation.Use(token))` stops every Kerf operation inside it, which take no token or can't, such as the operators. Fillets and chamfers, which ignored their token until the end (24 s on a large one), hidden lines, and the long searches inside Booleans now stop within milliseconds to a few hundred.
- **A mesh too large to be meant fails at once:** `MeshOptions.MaxTriangles`, 10 million by default, `meshing.too-large`, where a tiny deflection used to take all the memory. Results below it are unchanged.
- **A Boolean whose result is proved wrong fails, carrying the shape:** a common larger than the smaller input, a cut outside A − B to A, a fuse outside the larger input to A + B, a solid inside out in the result, and an empty common or a cut that removed nothing where points lie well inside both inputs. `result.invalid`, leaving it out with `BooleanOptions.CheckResult = false`; `BooleanVolumes.Check` is the check itself. For solids only (decisions 1860 to 1868; OCCT #1629, #1643, #1659, #245).
- **The validity check is cheaper on small shapes:** about half the bytes and a third less time on a box, a prism or many small solids (from a loaded machine), and about 8% fewer bytes over a recalculation of Bento's examples. No verdict or status changes: compared with the old check, on every sub-shape, over the corpus's Boolean, blend, chamfer and healing groups.
- A fuse with a compound of reversed solids no longer drops them (#1613), and a cut by a helical tube no longer removes nothing depending on where it leaves (#1543).
- A fillet or chamfer whose tolerance passes its size fails with `result.invalid`, carrying the shape, rather than succeeding with a wrong volume (#1639).
- `BooleanOptions.ApproximateSection` is true by default: without it a section can lose most of a curve (#1124).
- `UnifySameDomain` no longer makes a box cut by a sphere invalid, or a closed pipe unmeshable (#1541, #1340, #1310).
- A STEP offset surface whose basis is only C0 keeps its face on reading (#994).
- A reused solid classifier's answer no longer depends on the points it classified before (#1692).

- **A partial torus is a slice of the tube (a bug fix; results change):** `Solid.Torus` with `MinorStart` and `MinorEnd` closes the arc of the section with a straight line across its ends, not OCCT's flat discs to the axis: 53.5 and not 210.6 for the outer half with R = 5, r = 1. The engine's `MakeTorus` is unchanged.
- **A default sweep that fails tries another frame:** `Sweeps.Sweep` with the corrected Frenet frame tries the Frenet and then the discrete frame, takes the first valid result, and says so in a warning (`Sweeps.SweptWithFrenet`, `Sweeps.SweptWithDiscrete`). `SweepOptions.TryOtherFrames = false` keeps the failure. OCCT #1630.
- **Threads keep less memory between operations:** after Bento's examples in parallel, 72 MB on 51 threads became 20 MB on 53 threads, by lower limits on each thread's work sets and weak references for anything larger (decisions 1761 to 1763). Fillet sections, over-determined root searches and intersecting edges in healing allocate up to a third less on the cases that use them (decisions 1764 to 1766). Results are unchanged.
- **Public lengths are true (a bug fix; results change):** `EdgeCurve`, `WireCurve` and `Curve3D`'s `Length`, `LengthBetween` and `ParameterAtLength`, `IntegrateLength` and `MassProperties(Dimension.Length)` were up to 1.5% out on uneven curves, as OCCT's are. The engine's own callers are unchanged (decision 1768). `Solid.Torus` refuses a minor circle reaching its axis (decision 1773). `LengthBetween`'s documents said a reversed range was negative: it never was, and they now say so.
- **Large meshes are quicker, and meshing makes less garbage,** with the same triangles: about a tenth less processor time on large meshes, and 8% fewer bytes on Bento's examples with three quarters fewer full collections (decisions 1756, 1757 and 1759). `MaxFillet` takes about 20% less, and 2D offsets of B-splines about 8% (decisions 1755 and 1758).
- **Fixes for OCCT's own defects (`docs/occt-defects.md`):** Booleans of cylinders about a degree from parallel no longer come out
  empty. An optimal bounding box always holds its shape. Revolving an arc that ends on the axis gives a valid solid. A sweep
  with a parallel guide stays exact. Areas and volumes asked for with a precision are right on periodic and swept faces, so a
  few differ from before. A ring of concentric circles can be offset inward. Mirroring through a point now inverts correctly.
  NaN is refused as a radius or an angle.
- **More of OCCT's defects fixed (a bug fix; results change where it was wrong):** `Unify` and a simplified Boolean no longer spoil a good result, where OCCT's clean gives an
  invalid shape up to 17% out in volume. A sphere touching a torus along a circle fuses, cuts and commons correctly, where OCCT's fuse was empty; a fuse that comes out empty from
  shapes with something in them fails. An offset surface on a mirrored plane is on the right side. `ApproximateBSplines` keeps a cylinder and a sphere valid. A direction refuses NaN and
  infinity. A torus within tolerance of horn is written to STEP as a torus.
- **No operation reports success for an invalid result (breaking).** Booleans, fillets, chamfers, offsets, sweeps, lofts, features and healing now
  check what they made and fail with `result.invalid`, saying what's wrong and naming the sub-shapes. `Result<T>.TryGetInvalid` gives the shape to a
  caller who wants to look at it; `Value` still throws. `CheckResult = false` on the options, or `checkResult: false`, leaves the check out. `Fix`'s
  warning `healing.not-fixed` is gone: it's this failure. `Validate` runs serially on small shapes. See `docs/api/README.md`, section 2.2 (decisions
  1742, 1791 to 1796).
- **STEP export uses about a third less CPU and half the memory, and import with healing about 10% to 15% less CPU,** with the same output (measured on a loaded machine; decisions 1727 to 1731).
- **Every public member of the kernel has XML docs,** including `Kerf.Geometry`'s value types, saying units, handedness and what
  throws; a new one without docs fails the build (CS1591). The docs of `Kerf.Geometry` name the exception you actually catch.
- **Tangent circles and root finders, fixes that change results:** a circle tangent to a curve through a point, with its
  centre on a curve, is now solved rather than failing by chance, and one tangent to a circle and a curve with its centre
  on a circle no longer throws. Bisection takes an end of its bracket that is already a root, where it returned the other
  end. A distance field built in parallel gives the serial result (decisions 1749 to 1752).
- **Fillets and chamfers, three fixes that change results:** the edges between a fillet and its faces are coded tangent,
  as in OCCT, so a draft can follow a fillet, and saved BRep files gain those lines. A fillet that runs past a Bezier or
  B-spline face puts the face on the surface it extended, where the result used to be invalid. A closed edge filleted
  with no radius at its one vertex no longer leaves the old face behind as well (`docs/corpus-failures.md`; decisions
  1719 to 1721).
- **A fillet or chamfer that would silently lose a face of an open shell now fails:** a chamfer whose curves don't close on a face used to
  drop that face and the one beside it and look valid. OCCT gives an invalid shape there. Two other results change, to OCCT's: a
  fillet's boundary running along an edge no longer leaves that edge in the face beside it, and a stub of an edge is no longer kept
  (decisions 1735, 1736).
- **A projected pcurve is checked on a spline with very uneven speed:** where OCCT's standard projector misses a fast stretch, by 10
  on a cylinder of radius 5 with the status "done", the projection uses points spread by knot span, or fails (decision 1737).

- **Names, cleaned up (no results change):** the geometry's `With…` methods that kept OCCT's abbreviations now say what they set
  (`WithCirc`, `WithCirc2d` → `WithCircle`; `WithElips` → `WithEllipse`; `WithHypr` → `WithHyperbola`; `WithLin` → `WithLine`; `WithParab` →
  `WithParabola`; `WithPln` → `WithPlane`; `WithTrsf` → `WithTransform`). `EulerSequence`'s members lose their underscore (`ExtrinsicXYZ`), and
  `TransformForm`'s are `PointMirror`, `AxisMirror`, `PlaneMirror` and `Composite`. `Direction2D.D`, `Direction3D.D` and the constructors that
  take them are gone from the API: use `UnitX` and the like. About 750 parameters of public members have full names (`a1` → `axis`, `ang` →
  `angle`, `p` → `point`, `raiseExc` → `throwOnInvalid`), which matters only to a call that names its arguments.
- **The corpus's memory and time checks give one answer:** cases that read their process's own figures run in a process of their own, and a heap reading holds only what the case keeps, so they no longer pass or fail by what else is running. Nothing in the library changed.
- **Much less memory churned:** exploring a shape allocates nothing, and a curve on a plane is no longer projected again for each question. A
  recalculation of Bento's examples allocates 17% less, STEP import with healing 29%, and `ShapeFix` 39%. Results are unchanged.
- **Some long operations are far lighter:** edges against free-form faces in a Boolean, area and volume to a precision, curve fitting in face-face intersections, and healing allocate half or less on the cases measured (one Boolean 5.2 GB to 1.1 GB; a volume maker with its checks 37 GB to 13 GB; ShapeFixer on 289 STEP files by a third). STEP export allocates a fifth less, and PNG's filter takes a fifth of the time. Every result is the same.
- **Quicker to open:** the first use of Kerf in a process, as a workbook opening, takes about a third less time (745 ms to 509 on Bento's examples), and early recalculations about a quarter less. The packages are compiled with more of the framework's code and generics in them, and are about 20% larger (19.5 MB to 23.4 MB). Results don't change, and there's nothing to set.
- **Runtime settings for a host, measured:** the README lists them. A parallel round of Bento's examples takes 620 ms against 2,700 ms under the runtime's defaults, and a recalculation 180 ms against 360. `Placement.Inverted` keeps its result, for 2% less allocation. All of the kernel's locks stay, because a trace found no time lost on any of them.
- **STEP's header names Kerf:** organisation `Kerf`, processor `Kerf STEP processor 0.3.0`, product `Kerf STEP translator N`, and
  a description saying Kerf derives from Open CASCADE Technology. Files naming Open CASCADE read as before. STL's default
  header is `STL Exported by Kerf`.
- **Messages in Kerf's terms:** exceptions and diagnostics no longer name OCCT's types (`gp_Dir`, `TopoDS_Face`). Codes don't change.
- **`MeshOptions.ControlAllSurfaces`,** off by default: also checks surface deflection on elementary surfaces, for long fillets that otherwise stray from it.
- **StepTidy:** `StepWriteOptions.CleanDuplicates` merges exactly equal entities when writing STEP, 17.9% smaller over the
  dataset's files. Off by default.
- **Joining edges on part of a Bezier curve:** the joined curve now follows that part. OCCT trimmed it twice, so it came back shorter and on the wrong parameters.
- **Faster optimal boxes:** a torus patch's is worked out rather than searched for, and `AddOptimal` skips faces that can't enlarge the box.
  The results differ by 1e-7 at most.
- **Unifying (and so a simplified Boolean) no longer fails on two kinds of valid shape:** edges meeting where one has a
  zero-length tangent, as a B-spline with its end pole repeated, are left unmerged, and infinite lines, which have no
  vertices, pass through. In OCCT both raise (`docs/occt-issues.md`; decisions 1652 and 1653).

## 0.2.0

The first release as Kerf.Kernel, replacing `OcctNet.Foundation`, `OcctNet.ModelingData`, `OcctNet.ModelingAlgorithms`
and `OcctNet.DataExchange`. Every name and most signatures changed from the `OcctNet.*` and `Build123d` packages, and there's no compatibility
layer. Before 1.0 a minor version may break the API again.

- **A public API for .NET** (`docs/api`): namespaces by area (`Kerf.Geometry`, `Kerf.Topology`, `Kerf.Booleans`,
  `Kerf.Exchange`…), .NET names and 0-based indexing. The engine is internal, in `Kerf.Engine.*`.
- **Immutable, with no copy at entry:** public values never change, and algorithms never write into their inputs, so
  results share the faces they don't change.
- **Results:** expected failures come back as `Result` and `Result<T>` with diagnostics. No engine exception reaches a
  public caller; misuse throws .NET's own exceptions.
- **OCCT's bugs:** triaged, and the ones on the fix list fixed; each is recorded in `docs/occt-issues.md`.
- **Images:** hidden-line drawings as SVG, and shaded pictures as PNG, in managed code.
- **No dependencies beyond .NET:** Schematiq.Collections and native OCCT are gone. Native OCCT's results stay as frozen
  recordings for the tests.
- **Files say who wrote them:** glTF, 3MF and OBJ files name Kerf as their generator. STEP output is unchanged, so its
  header still says Open CASCADE.
- **Not done yet:** the coverage floor, and the rest of `docs/phase11-plan.md`.

The units, oldest first:

Phase 10 (part): one repository, Kerf (unit 10R).
- OCCT.NET and build123d.NET move, with their histories, into one repository: `kernel/` and `modeling/`. The packages
  keep their names until 0.2.0, when they become Kerf.Kernel and Kerf.Modeling.

Phase 10 (part): conventions and tooling (unit 10C).
- `tools/kerf-rename`: the renamer that unit 10E uses to move the kernel and the modelling layer to .NET names, from a
  reviewed name map (types and namespaces, members, methods that become properties, and the systematic rules for
  parameters, fields, locals and methods), reporting every rename it drops rather than guessing. It drafts the
  systematic part of a map from the code, and applies a map to another repository (Bento) as well.
- `docs/design-rules.md`: the rules for new and redesigned code (names, values and references, immutability,
  operations, results and errors, collections, spans, nullability, modern C#, state); translation-rules.md stays as the
  rules the port followed.
- A root `.editorconfig` with .NET's naming rules, inert until 10E turns them on folder by folder.
- No change to the libraries: the pilot rename of gp and math stays on an unmerged branch.

Phase 10 (part): OCCT's bugs triaged (unit 10K's first half).
- Every bug in docs/occt-issues.md that the port reproduced to match native OCCT now has a verdict: 134 to fix, 572 to
  fix when their code is next rewritten, 313 to leave, 135 already settled by the port. The ones to fix are those a
  user would see: wrong Boolean or bounding-box results, hangs and failures in healing, sweeps, lofts, 2D offsets,
  drafts and fillets, wrong 2D fillets and chamfers, the 2D tangent-circle solvers, wrong properties and continuity,
  and STEP and STL files read or written wrong. docs/occt-issues-triage.md lists them in eight batches by impact,
  with each bug's site and the tests that will change. No change to the libraries yet.
Phase 10 (part): the public API design (unit 10D).
- `docs/api`: the design of Kerf.Kernel's public API, for review: an overview of results and diagnostics, options,
  cancellation, values, shapes and collections, and an API review for each area (geometry, topology, construction,
  Booleans, fillets, offsets and features, healing, measurement, meshing, exchange), each mapped from today's types.
- Every public type classified as public API, engine or removed (`docs/api/classification.tsv`, from rules by package
  with `tools/api-classify.cs`), and the draft name map for the rename (`docs/api/kerf.map`).
- No change to the libraries.
Phase 10 (part): test independence (unit 10A).
- The oracle tests no longer need native OCCT: native's results are recorded (`tests/OcctNet.Oracle.Tests/Recordings`,
  109 MB) and replayed through a generated facade, the port compared with them exactly as before; they can be recorded
  again while native OCCT exists. They join Kerf.slnx. The largest corpora and random runs are sampled in the
  recordings (run in full live).
- The differential run compares build123d.NET's suite on OCCT.NET with native's log as recorded (digests, properties,
  and the texts that differed).
- `tools/coverage.ps1` measures line coverage per OCCT package (`docs/coverage.md`).
Phase 10 (part): build123d.NET on OCCT.NET alone (unit 10B).
- Native OCCT is gone from the repository: build123d.NET's native backend (its bindings, shim, generator and native
  build), and with it the oracle tests' recording and live modes. The oracle replays native's results as recorded in
  unit 10A, frozen; `tools/oracle-facade.cs` and `b3d-differential.ps1 -Record` went. The benchmarks time the port
  alone, and build without native OCCT again.
- Public for the modelling layer, which reached them with `UnsafeAccessor`: `TopoDS_Builder`'s constructor, `Make*`,
  `Add` and `Remove`; `BRep_Builder` and its `MakeFace` of a triangulation; `TopoDS_TShape.FreezeGraph`. Engine API,
  internal again once 10L's public API replaces them.
- `tools/pack-nuget.ps1` packs `Build123d` from the repository, without `Build123d.OcctNet`, which is gone.
Phase 10 (part): the rename (unit 10E). **Breaking: every name changes.**
- The packages are `Kerf.Kernel.Foundation`, `Kerf.Kernel.ModelingData`, `Kerf.Kernel.ModelingAlgorithms` and
  `Kerf.Kernel.DataExchange`, at 0.2.0 (the `OcctNet.*` packages end at 0.1.0).
- Types are in namespaces by area without OCCT's package prefixes: the public values and curves in `Kerf.Geometry`
  (`gp_Pnt` is `Point3D`, `gp_Cylinder` is `Cylinder`, `Geom_BSplineCurve` is `BSplineCurve3D`), the shapes in
  `Kerf.Topology` (`TopoDS_Edge` is `Edge`, `TopLoc_Location` is `Placement`), everything else in `Kerf.Engine.<Area>`
  (`BOPAlgo_PaveFiller` is `Kerf.Engine.Booleans.PaveFiller`). Members, parameters and fields follow .NET's naming
  (`theX` is `x`, `myX` is `_x`; `NbPoles()` is the property `PoleCount`). `docs/api/kerf.map` says what each OCCT name
  became; the types 10J replaces (NCollection, TCollection) keep their names in `Occt`.
- Exceptions end in `Exception` (`Standard_DomainError` is `DomainErrorException`); their `OcctType`, messages, STEP's
  signatures and every other text OCCT prints keep OCCT's names (`OcctTypeNames`).
- The analyzers are `Kerf.Analyzers`, with ids `KERF0001` and `KERF0002`; .NET naming (IDE1006) is an error in the
  build where the code obeys it.
- No behaviour changes: every test passes as before.
Phase 10 (part): the working overlay (unit 10H1).
- An operation can now run OCCT's in-place code on a caller's frozen shape without copying it: its writes go to working
  copies of the TShapes it changes, which it reads back, and its result shares every sub-shape it didn't change, with a
  direct history (`WorkingOverlay`, internal; docs/overlay-survey.md lists the 110 copy-at-entry sites 10H2 to 10H6
  convert). Without an open overlay nothing changes.
- ShapeFix_Edge's and ShapeFix_ShapeTolerance's twins (`AddPCurveFixed`, `VertexToleranceFixed`, `ToleranceLimited`, …)
  run on it instead of a copy of their input: the same results, faster.

Phase 10 (part): errors (unit 10F).
- `Kerf.Result<T>`, `Result`, `Diagnostic` (a severity, a stable string code, a plain message and, from ModelingData,
  the sub-shapes concerned), `DiagnosticCode`, `Severity` and `KerfException`, in Foundation; `Built<T>` in
  ModelingData. Nothing public returns them yet: unit 10L's operations will.
- Bridges from OCCT's error models to diagnostics: `FailureDiagnostics` (exceptions), `AlertDiagnostics` (a report's
  alerts), `StatusDiagnostics` and `OperationDiagnostics` (the status enums; a failed operation), `ExchangeDiagnostics`
  (`IFSelect_ReturnStatus`). docs/api/diagnostics.md lists the codes.
- `Resource_Manager.TryGetInteger`; ShapeProcess reads its parameters without catching an exception for each one that
  isn't set (results unchanged).
Phase 10 (part): immutability (unit 10G). **Breaking for code that changed values in place.**
- gp's values are `readonly struct`s: `Point3D`, `Vector3D`, `Direction3D`, `Axis3D`, `Frame3D`, `CoordinateSystem`,
  `Transform3D`, `GeneralTransform3D`, `Matrix3x3`, the conics and quadrics and their 2D twins, and `Placement`. Their
  in-place mutators (internal since phase 5) are gone; `Transformed`, `WithX`, `Normalized` and the other returning forms
  give changed values. New: `Vector3D.LinearForm`, `Vector2D.LinearForm`, `Matrix3x3.PreMultiplied`.
- The B-spline and Bézier curves' and surfaces' `Poles()`, `Knots()`, `Weights()` and the like return
  `IReadOnlyList<T>` in place of `PackedList<T>`.
- KERF0003, a build error: a public type of Kerf's public namespaces can't gain a setter, a mutable field or an in-place
  method.
- No behaviour changes: every result is the same to the bit, and every test passes as before.
Phase 10 (part): no copy at entry in the Booleans (unit 10H2).
- The Booleans no longer copy their inputs anywhere: the self-check (`BOPAlgo_CheckerSI`, `BRepAlgoAPI_Check`,
  `bopargcheck`), the wire tools (`BOPAlgo_Tools::WiresToFaces`, `BRepAlgo::ConcatenateWireC0`,
  `BRepAlgo_FaceRestrictor`, `TopOpeBRepBuild_WireToFace`), the cells builder, defeaturing's removal of internal wires,
  and the periodicity's trim and repetitions (`BOPAlgo_MakePeriodic`, `BOPAlgo_MakeConnected`) run OCCT's in-place code
  in a working overlay. Results are as before; the periodic and connected shapes' histories now record a sub-shape as
  modified only when it changed.
- The general fuse and `BRepAlgoAPI_*` keep OCCT's non-destructive mode on frozen arguments; a Boolean run inside
  another operation's overlay is destructive, as OCCT's on an unlocked shape.

Phase 10 (part): state (unit 10I).
- No process-wide settings: what OCCT kept in statics (the message table's additions, ShapeProcess's registered
  operators, the parallel defaults of the Booleans and the mesher, the meshing algorithm's name, BRepLib's precision and
  plane, `Quantity_Color`'s epsilon, the default messenger and report, GeomTools' handler, ShapeAlgo's container, debug
  flags) are settings of `OperationContext` (Foundation): immutable defaults, overridden for a scope
  (`OperationContext.Begin`) or a flow of execution, which flow into the tasks an operation starts and never reach
  another operation, thread or test. OCCT's setters now change the current flow only.
- `ProcessContext.ResourceOverridesSetting`: an operation can run shape processing (the STEP and IGES readers' too)
  with its own resources beside others on the defaults. The resource files compiled in are parsed once; OCCT's cache of
  the last file goes, and `ClearResourceCache` with it.
- `MsgFile.Msg` of an unknown keyword no longer stores the surrogate text, so `HasMsg` stays false for it.
- Removed: `STEPConstruct_Assembly.MakeRelationship()`, which numbered occurrences from a process-wide counter (the form
  taking the caller's counter stays).
- 32 locks go: the registries are built once and published whole, the caches publish without locks, and
  `XSControl_WorkSession`'s process-wide mutex is its controller's, so STEP readers and writers no longer wait for each
  other. docs/state.md lists every static, thread static and lock that remains, and why.
- Concurrency tests run Booleans, fillets, offsets, healing, shape processing, STEP reading and writing, meshing and
  the modelling layer's parts on many threads, each giving its serial BRep text.

Phase 10 (part): no copy at entry in healing (unit 10H4).
- ShapeFix (every fixer and the statics' twins), ShapeBuild's and ShapeConstruct's twins, ShapeUpgrade's dividers and
  `UnifySameDomain`, ShapeProcess's operators and sewing no longer copy a frozen input: they run OCCT's in-place code on
  the caller's shape in a working overlay, and a tool called by another (ShapeFix_Wire by ShapeFix_Face, ShapeFix by
  ShapeProcess) writes into the outer one's, as in OCCT. Results are as before and share more of the input; the input is
  never written, and a twin no longer writes into a caller's unfrozen compound either.
- Tools fixed over several calls (ShapeFix_Face, _Wire, _WireVertex, _Wireframe, _FixSmallFace, _ComposeShell) keep
  their overlay between calls without leaving it open: the shapes they hold show the fixes only in their result.
- The overlay maps its input before assembling the result, which corrects the history of an operation on an unfrozen
  input whose sub-shapes it changed (a Boolean's simplified result). BRepLib's SameParameter and the frozen-shape checks
  write where OCCT does inside an overlay.
- `HealingInputsUnchangedTests` and a STEP import healing test check the inputs untouched; the benchmarks gain
  `healing` (STEP import, ShapeFix_Shape and UnifySameDomain on the dataset's STEP files).


Phase 10 (part): heap and collections (unit 10J).
- `Grid<T>` (`Kerf.Geometry`): a read-only 2D grid, 0-based. `BSplineSurface` and `BezierSurface` return their poles
  and weights as grids (`Poles()`, `Weights()`, `WeightsArray()`); the copy-into overloads `Poles(array)` and
  `Weights(array)` are internal.
- `NCollection_Map` is gone: the engine uses `HashSet<T>` (with the same comparers, and the same iteration order).
- Faster without changing results: a shape's hash is drawn once when its TShape is made; the mesher no longer moves
  its candidate circles when it deletes one; the elementary extrema hold their solutions in fixed-size fields;
  `math_NewtonMinimum` reuses its eigenvalue and LU solvers between iterations; STEP AP214's type tables and MoniTool's
  typed-value library are frozen dictionaries.
- KERF0002 (a map of shapes needs an explicit comparer) works again on Kerf's shape types.
- docs/collections-plan.md: what is left of plan 2.6's collections, type by type.

Phase 10 (part): no copy at entry in construction (unit 10H3).
- Prisms, revolutions, pipes, lofts (`BRepOffsetAPI_ThruSections`, `BRepFill_Generator`), ruled faces and shells,
  evolved shapes, BRepLib's functional twins, the face of a planar wire and the wire maker's merges no longer copy their
  inputs: they run OCCT's in-place code on the caller's shapes in a working overlay, which copies only what they write
  into. Results, and what they share with their inputs, are as before: the prism of a polygon face holds the caller's
  face, its edges and vertices; of a circle face, a new face (the circle takes the cylinder's pcurve, decision 1102).
- Histories answer for the caller's sub-shapes directly. `BRepLib_MakeWire`'s `Modified` records only the edges and
  vertices a merge changed (it recorded every sub-shape of the wire built so far). `BRepPrimAPI_MakeRevol.Degenerated()`
  gives a generating shape's degenerated edge as it stands in the result.
- An algorithm another operation runs (a sweep inside an evolved shape, a loft's generator) writes into that
  operation's overlay, as OCCT writes in place (`WorkingOverlay.BeginUnlessInside`).
- Removed (internal): `BRep_Builder.UpdateVertexOfFrozen`, BRepLib's `UnfreezeResult`, MakeWire's copies for write.
- A construction benchmark (`Kerf.Kernel.Benchmarks -- construction`): allocation falls by up to a third where an input
  was copied whole; Bento's benchmark is unchanged.
- Seven oracle replays whose wires MakeWire builds compare at plan 3.2's second level (`Tape.AcceptOtherArguments`): the
  wires differ from the recordings' only in the Free flag of the caller's edges they now keep.
Phase 10 (part): no copy at entry in offsets, fillets and features (unit 10H5).
- The 3D offset and the thick solid, BRepOffset_Offset's pipes and spheres, BiTgte's rolling ball, the fillet and
  chamfer builder (and FilletSurf's), the 2D fillet, the draft angle, BRepFeat's features and LocOpe's local operations
  no longer copy a frozen input: they run OCCT's in-place code on the caller's shape in a working overlay, and the
  result holds new TShapes only for what changed. Their histories answer for the caller's shapes directly.
- Builders called over several calls (the fillet builder, LocOpe's split and wires on a shape, BRepFeat's makers) keep
  a detached overlay, entered by each call that writes, so nothing stays open between the calls; a second computation
  on one fillet builder starts again from the caller's shape.
- OC5805's thick solid, whose offset surface OCCT enlarges in place, is built as before, the caller's face keeping its
  surface. A second `BRepOffset_MakeOffset` run after `Initialize` no longer fails on the same shapes (OCCT's analysis
  kept the last run's data).
- The simple offset's solid and the offset's thickening mode stay on their copies until construction runs in an overlay:
  their walls are made by BRepLib_MakeFace, which copies frozen edges itself.
- `OffsetInputsUnchangedTests`, `FilletInputsUnchangedTests` and `FeatureInputsUnchangedTests` check the inputs
  untouched; the benchmarks gain `features` (offsets, thick solids, fillets, chamfers, drafts and a boss on frozen
  parts).
Phase 10 (part): OCCT's bugs in Booleans, boxes and distances, and hangs and spurious failures (unit 10K1, batches K1
and K2 of the triage).
- Booleans, boxes, distances: a fuse merges solids whose only shared face OCCT never checked; the boxes of parabola
  and hyperbola arcs contain the arc (they could miss its coordinate extrema, so Booleans and distances could skip a
  touching pair); a distance between a short edge and a face reports the point on the edge, not the origin.
- Hangs: healing's overlap check on a zero-length edge, the B-spline conversion of a surface whose approximation
  always fails, and the corrected Frenet trihedron on offset curves (seconds to minutes; now well under a second) all
  return.
- Spurious failures: a fillet whose radius follows a law (`MakeFillet.Add(law, edge)`); MakeFilling with G2 or C2
  constraints, and C1 now meaning G1 rather than G2; a pipe of a vertex's history; a pipe between two sections on a
  3D path; SweptToElementary on shapes with degenerated edges (most revolved shapes); a loft through closed B-spline
  sections with different knots. The medial axis of a single closed edge reports failure instead of crashing.
- Left for later, with reasons (decision 1441): the 2D offset's two medial-axis failures and some multi-face drafts.
- Recordings these fixes move compare at plan 3.2's third level, each with its reason; occt-issues.md marks the rows
  fixed.

Phase 10 (part): the end of copy at entry (unit 10H6).
- No algorithm copies its input at the entry any more. The offset's thickening mode and the simple offset's solid run
  in working overlays, their walls sharing the caller's edges; the mesher meshes a frozen shape in an overlay of its own
  and returns a result sharing every face it left alone; HLR's outliner works on the caller's shape, so
  `HLRBRep_HLRToShape` finds a caller's sub-shapes in its data as themselves; the reflect lines' SameParameter no
  longer writes into the caller's vertices.
- `BRepTools_CopyAtEntry`, `ExactCopyForWrite`, `HoldsFrozen`, `KeepUnchanged`, `KeepUnchangedIn` and
  `CopyFrozenForWrite` are gone (about 1,000 lines). `CopyData`, `CopyFlags` and `IsSameData` stay for the overlay.
  BRep_Builder's mesher forms `UpdateFaceUnfrozen`/`UpdateEdgeUnfrozen` are `UpdateFaceMesh`/`UpdateEdgeMesh`.
- `WorkingOverlay` has one way to join an operation's overlay, `BeginUnlessWriting` (`BeginUnlessInside` was the same
  rule), and one rule protecting published shapes, `IsReadOnly`, which BRep_Builder's refusals ask through `IsLocked`.
- Copying a mesh out (`MeshExtraction`) writes the normals straight into its buffer instead of computing them on a copy
  of each triangulation: 2.7 MB allocated for a 100,000-triangle sphere instead of 6.5 MB. Its timing test, which failed
  on loaded machines, measures allocation and the best of five runs.

Phase 10 (part): heap and collections, the rest (unit 10J2).
- `NCollection_DataMap` is gone: the engine uses `Dictionary<TKey, TValue>`, with `DictionaryExtensions` (in
  `Kerf.Engine.Collections`) for what the wrapper added: `GetValue` and `GetValueRef` fail with OCCT's
  `NoSuchObjectException` for a missing key, and `SetValueRef`/`GetOrAddValueRef` return the stored value by reference.
- `NCollection_DynamicArray` and `NCollection_Vector` are gone: `List<T>`, with `ListExtensions.SetAt` for OCCT's
  growing `SetValue`.
- `NCollection_IndexedMap` and `NCollection_IndexedDataMap` are gone: `IndexedSet<T>` and `IndexedMap<TKey, TValue>`
  (in `Kerf.Engine.Collections`), numbered from 0; removing an item moves the last one into its place, as in OCCT.
- KERF0002 checks the two indexed collections.
- `tools/collections-rewrite`: the Roslyn rewrite that made these changes, for the types still to go
  (docs/collections-plan.md). Results are unchanged.

Phase 10 (part): OCCT's bugs in construction, properties, extrema and projection (unit 10K2, batches K3, K5 and K6 of
the triage).
- Projection: the pcurve of a helix of more than a turn on a cylinder (a thread), a spiral on a cone or a coil on a
  torus follows the curve (it was off by up to the diameter); a projection's free part after an isoparametric one gets
  its own pcurve; a parabola projected on a plane gets the right focal.
- Extrema: curve-curve extrema run twice report the second run's solutions only; GeomAPI's total curve-curve distance
  keeps the nearest end pair and, on ranges, stays on the ranges; 2D circle intersections give an indirect circle's
  own parameters.
- Properties: BRepGProp's volume by a plane (Vinert) reports its volume and centre; natural faces' adaptive
  integration splits V at the V bounds; curvatures asked before IsCurvatureDefined are the point's; GeomLProp's
  continuity at a B-spline knot is the knot's; a continuous approximation with free ends is done; GProp's elementary
  curves, surfaces and solids (lines, circles, cylinders, cones, spheres, tori) have right centres and inertia.
- Construction: 2D fillets and chamfers keep a face's holes as their own wires, fillet a moved arc the right way and
  trim a reversed neighbour correctly; a distance-angle chamfer between a plane and a cylinder no longer depends on the
  axis direction; NurbsConvert and GTransform keep a meshed face's surface; rational Coons fillings keep their
  boundaries; a revolved feature up to a face at an angle chooses as Perform(Until) does; the sweep section generator
  moves its sections along the path; ExtendFace no longer extends an infinite side.
- Left, with reasons: AdvApp2Var's error estimate (decision 1482), and the row about ExtendFace losing a location,
  which wasn't a bug (1477).
- Recordings these fixes move compare at plan 3.2's third level, each with its reason; occt-issues.md marks the rows.
Phase 10 (part): OCCT's bugs, curve and surface evaluation, conversion, gp and data exchange (unit 10K3, batches K7
and K8).
- Raising the degree of a periodic B-spline keeps the curve (lofts' and fillets' periodic sections and spines).
- Offset curves' third derivatives are right, through the curves and their adaptors, and so the Frenet and corrected
  Frenet frames' derivatives on offset spines; a curve on a surface has the chain rule's third derivative; a torus's
  mixed derivatives have the right signs; a rational bilinear patch's cached second derivatives are right.
- gp: a direction mirrored about an axis, 2D and 3D transformation powers, 2D rotation products, 2D hyperbolas'
  asymptotes and second directrix, and affinities' negative powers are right.
- An offset surface transforms its parameters once; a B-spline surface split in V is reversed in V; a surface's knot
  splitting keeps its U splits when V needs none; canonical recognition finds ellipses at any angle; a cone's implicit
  equation holds away from the origin; an indirect 2D parabola's grid follows its frame; an extrusion adaptor trims
  the direction it is asked to.
- STL: a stream is recognised as a file is (binary files whose header starts "solid" read), and every domain of a
  multi-domain ASCII file is read.
- STEP: rendering methods read as written; styled representations, two complex record kinds and booleans written
  validly; two reflectance styles and TESSELLATED_ITEM read and written; XSControl_Reader reads STEP; 2D trimmed
  curves and rational 2D Bezier curves written; angles written in the file's unit; strings of any length read, and a
  tab ends a string as a space does.
- Binary BRep: a point on a surface keeps its parameters; the files are unchanged, and old ones read right.
- Two rows reclassified to leave (decisions 1493, 1509). The oracle and native comparisons these fixes move compare
  at plan 3.2's third level, each with its reason; occt-issues.md marks the rows fixed.

Phase 10 (part): the public API, part 1: the shared types, geometry and topology (unit 10L1).
- Kerf.Geometry's values read as .NET: `X`, `Y`, `Z`, `Direction`, `Location`, `Radius`… are properties;
  `DistanceTo`, `SquareDistanceTo` and `AngleTo` replace `Distance`, `SquareDistance` and `Angle`; gp's constants are
  static properties (`Point3D.Origin`, `Direction3D.UnitZ`, `Axis3D.ZAxis`, `Frame3D.World`, `Transform3D.Identity`);
  points, vectors and transforms combine with operators (`b - a`, `p + v`, `t * p`); `ToVector3()` and `ToMatrix4x4()`
  convert to System.Numerics.
- Curves and surfaces: `Evaluate`, `Derivatives(u, order)`, `Range`, `Kind` and the surface's `Bounds` replace OCCT's
  `D0`…`DN`, `Value` and out-parameter bounds; `Poles`, `Knots`, `Weights` and the other getters are properties indexed
  from 0; the B-splines and Béziers are built from spans and grids. The concrete classes are sealed.
- Kerf.Topology: `Type`, `Placement` and `Orientation` properties; `IShape<T>`, `TryAs`, `As` and explicit casts;
  `Explore<T>()`, `Children`, `Faces()`, `Edges()`… for every shape type; raw assembly (`Compound.Create`,
  `Wire.Create`…, `WithChildren`, `WithoutChild`); vertices', edges' and faces' data (`Vertex.Point`, `Edge.Range`,
  `TryGetCurve`, `Face.TryGetSurface`, `OuterWire`…); `EdgeCurve` and `FaceSurface`; `ShapeComparer`; `ShapeGraph`.
- New shared types: `Interval`, `Tolerances`, `BoundingBox`, `CurveDerivatives`, `SurfaceDerivatives`, `CurvePoint`,
  `SurfacePoint`, and `ShapeHistory`, which `Built<T>` now carries.
- `Placement` compares by value (its transformation); shapes still compare their placements by identity, as OCCT.
- OCCT-shaped members (`D1(u, out …)`, 1-based `Pole(i)`, members taking `Xyz`) and 277 geometry and topology engine
  types are internal; the kernel's assemblies, its tests, Draw-lite, the benchmarks and, until unit 10M, the modelling
  layer still reach them. `BRepOffset_Interval` is `OffsetInterval`.
- `docs/api/kerf-10l1.map` holds the renames, for kerf-rename to apply to other code (Bento).

Phase 10 (part): the public API, part 2: geometry's operations, construction, Booleans, fillets, offsets and features
(unit 10L2).
- `Kerf.Geometry.GeometryOperations`: arcs and segments (`TrimmedCurve3D.ArcThrough`, `Arc`, `Segment`), fitting
  (`BSplineCurve3D.Interpolate`, `Approximate`, `Join`; `BSplineSurface.Approximate`), conversion (`ToBSpline`,
  `ToCurve2D`, `ToCurve3D`), projection (`Project`, `TryProjectNearest`), intersection (`Intersect`) and
  `surface.TryGetPlane`, each returning a `Result` where it can fail. The splines' `WithPole`, `WithWeight` and
  `WithKnot` are indexed from 0.
- `Kerf.Construction`: `Solid.Box`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `Wedge`, `HalfSpace`; `shape.Extrude`,
  `Revolve`, `Sweeps.Sweep`, `Loft`, `Ruled`; `Edge.Line`, `Wire.FromEdges`, `Wire.Polygon`, `Face.FromWire` and the
  other shapes from geometry; `Transformed`, `Mirrored`, `Scaled`; `ShapeProjections.Project`.
- `Kerf.Booleans`: `a.Fuse(b)`, `Cut`, `Common`, `Section`, and `BooleanOperations.FuseAll`, `Split`, `Partition`, with
  `BooleanOptions`; never changing their inputs, simplified by default.
- `Kerf.Fillets`: `Fillet`, `Chamfer`, `Fillet2D`, `Chamfer2D`; a failure names the edges that failed.
- `Kerf.Offsets` and `Kerf.Features`: `Offset`, `Thicken`, `Offset2D`, `solid.Hollow`; `Draft`, `ExtrudeUntil`,
  `SplitFaces`.
- Every operation returns its shape with its history (`Built<T>`), the engine's statuses and alerts as diagnostics, and
  throws `OperationCanceledException` when cancelled.
- `GlueMode` (members `Off`, `Shift`, `Full`), `FilletShape`, `OffsetMode`, `TransitionMode` and `ParametrizationType`
  moved into the public namespaces; 686 more engine types (construction, Booleans, fillets, offsets, features) are
  internal. `docs/api/kerf-10l2.map` holds the renames.
Phase 10 (part): the public API, part 3: measurement, healing, meshing and exchange (unit 10L3).
- `Kerf.Measurement`: `part.Volume()`, `Area()`, `Length()`, `MassProperties(Dimension)`, `BoundingBox()`,
  `OrientedBox()`, `Distance(other)` (a `Result` with the closest points and their supports), `Intersect(line)` and
  `solid.Classify(point)`. `Kerf.Geometry.OrientedBox` is a value.
- `Kerf.Healing`: `Validate()` (a report of every fault, coded `check.not-closed`…), `Fix()`, `Unify()`, `Sew(faces)`,
  `shell.MakeSolid()`, `FixWireframe`, `ConvertToBSpline`, `ConnectEdges`, `FreeBoundaries`, with options records.
- `Kerf.Meshing`: `part.Mesh(options)` gives a `ShapeMesh` of flat buffers (positions, normals, triangles per face,
  edge polylines) and leaves the shape alone: no triangles are stored on its faces. A big mesh copies out with little
  more than its buffers.
- `Kerf.Exchange`: `Step`, `Stl`, `Gltf`, `ThreeMf`, `Obj` and `Brep` read and write streams with `ReadLimits` and
  cancellation, and return `Result`s; assemblies, names and colours travel as `ModelNode`; `Kerf.Color` is sRGB. Files
  are byte for byte what the engine wrote before. Meshes read from STL and glTF are `ShapeMesh`es.
- Engine types renamed so the public names are free: `LinearColor` (Quantity_Color), `Obb` (Bnd_OBB),
  `GltfReadLimits`, `GltfWriterOptions`, `ObjWriterOptions`, `StepLengthUnit`. 1,229 engine types of these areas are
  internal; the kernel, its tests, Draw-lite, the benchmarks and, until 10M, the modelling layer still reach them.

Phase 10 (part): OCCT's bugs in the 2D constraint solvers (unit 10K4, batch K4).
- GccAna's circle solvers (three tangents; two tangents centred on a line or circle; two tangents and a radius; a
  tangent, a centre locus and a radius) return only circles tangent to their arguments with the qualifiers asked,
  find the ones OCCT lost, report the qualifiers the solutions have, the points where they touch, and the centre as
  CenterOn3; none throws where OCCT's range checks did.
- GccAna's line solvers: tangents oriented from tangency to tangency, each pair of qualifiers picking one; each
  solution's own intersection, IsParallel for a parallel one; lines perpendicular to a circle pass through its centre.
- Geom2dGcc: a free-form curve and a point with a centre locus solve instead of throwing; the iterative solvers give
  their parameters, centres and third tangency; a free-form curve centred on a free-form curve finds its circles; a
  line at an angle parallel to the reference no longer throws.
Phase 10 (part): images, line drawings in SVG and shaded pictures in PNG (unit 10L4).
- `Kerf.Drawing`: `part.ProjectLines(camera)` removes hidden lines (OCCT's exact algorithm, or its polygonal one on a
  mesh) and gives a `LineDrawing`: 2D curves (lines, arcs, ellipses, splines, polylines), visible or hidden, sharp,
  smooth, sewn, outline or isoline, each with the edge or face it comes from. `Camera` is orthographic or perspective.
- `Svg.Write` writes a drawing: visible and hidden layers, true arcs and Bézier curves, page size, margins, scale or fit,
  stroke styles; the same drawing gives the same bytes.
- `Renderer.Render` (and `part.Render`, `mesh.Render`, `ModelNode.ToScene`) draws meshes in managed code: depth buffer,
  flat or smooth shading, colours per part or face, feature edges, supersampling, transparent backgrounds. `Png.Encode`
  writes the picture with Kerf's own deflate, so the bytes are the same on every machine. A part at 800 × 600 takes
  under 100 ms, meshing included.
Phase 10 (part): the public API's leftovers (unit 10L5).
- `Kerf.Geometry.Tangents2D`: lines and circles tangent to 2D curves and through points (`Lines`, `Circles`,
  `CirclesCentredAt`), with each solution's points of contact; `Tangency` is a public enum.
- `Kerf.Construction`: `Surfaces.Fill` (an n-sided filling through boundary edges and points) and `Sweeps.Helix`.
  `Revolve` no longer copies its profile: its result shares the profile, as `Extrude`'s does, and its history answers
  for every edge. Both sweeps' histories now hold their caps: the top one generated, the bottom one modified where
  the sweep rebuilt the profile.
- `Kerf.Exchange`: `StepReadOptions.Heal` and `StepWriteOptions.Schema` (AP214, AP203, AP242). `Kerf.Meshing.Triangle`,
  with `GetTriangles()` on a mesh and its faces. `Fix` warns `healing.not-fixed` when its result is still invalid.
- The mesher finishes each triangulation before attaching it, so meshes are frozen as soon as they are attached; the
  results are unchanged.
- 1,161 more engine types are internal: 184 remain public, all but 26 shown by the modelling layer's public API.
Phase 10 (part): the modelling layer on the public API (unit 10M1).
- The modelling layer calls the public API alone, and `Kerf.Modeling` no longer has `InternalsVisibleTo`. What it
  needed was added where it belongs: `Tangents2D.CirclesCentredOn` (two targets or a radius, centre on a locus) and
  `LinesAtAngle`, a target's range; `Curve2D.Project`, `Curve3D.IsPlanar`, `LengthBetween` and `ParameterAtLength`;
  `WireCurve` (a wire as one curve), and on `EdgeCurve` lengths, discretisation, continuity with another edge, frames
  (`Frames`, Frenet or corrected Frenet); `FaceSurface.TryGetCurvature`; `Wire.Connect`, `Face.FromWires`, holes on
  `FromSurface` and `WithHoles`, `WithCurves3D`, `Solid.HalfSpace` of a shell, `Transformed` with copied geometry,
  `Revolve` with a copied profile, edges projected onto shapes; `Face.Classify`, `ParametersOf`, `IsOnFace`,
  `ClassifyPoint` for any shape; `FixFace`, `FixWireframe`, `MisorientedEdges`, `ApproximateBSplines`; `Chamfer2D` by
  corner, `Draft` with an angle per face, `SplitAlong`, `ThickSolid` of any shape, `Offset2D` in a face;
  `Color.TryFromName` and `NearestName`, `OrientedBox.Contains` and `Diagonal`, a transform's quaternion and rotation,
  `Diagnostic.FromException` and `IsKernelFailure`.
- Options where the modelling layer's calls differed from a public default: `BooleanOptions.SectionCurvesOnFaces`,
  `FixOptions.CheckResult`, `WireframeFixOptions`, `ExtrudeUntilOptions.ModifySketchFace`,
  `SweepOptions.CurvilinearEquivalence`, `ConversionOptions.ForceApproximation`, `ApproximationOptions.Smoothing`,
  periodic interpolation's closing parameter and free tangents, and `Edge.FromCurve(curve2d, surface, computeCurve3D:
  false)`. `Sew`'s history records the edges and vertices it merges. A face's `WithHoles` adds its holes (it failed).
- A mesh read from STL or glTF keeps the shape read, faces holding only triangles (`ShapeMesh.TriangulatedShape`).
- The engine types public only for the modelling layer are internal, but for 17 the WebAssembly sample also shows: 44
  public engine types remain (by reflection, top-level), none for the modelling layer.

Phase 10 (part): Kerf.Modeling's .NET API (unit 10M2). No kernel change.
- The modelling layer's public API is redesigned for .NET (decisions 1575 to 1585): LINQ-friendly typed selections,
  typed results for Part, Sketch and Curve, options records, `Try` forms returning `Result<T>` with modelling codes
  beside the kernel's (`modeling.fillet-failed`…; api/diagnostics.md), no `Mode`, tuple conversions or Python operators,
  Python's emulation rewritten as .NET with the same results. Bento's benchmark builds on the new API.
Phase 10 (part): expected failures are results (unit 10M3).
- `KerfException` is gone, with `Result<T>.GetValueOrThrow` and `Result.ThrowIfFailed`: Kerf has no exception type of
  its own. Reading `Value` of a failed result is a programmer's error and throws `InvalidOperationException`, naming the
  first error's code and message. `Match` gives one value either way, and `Select` and `SelectMany` let a chain of
  operations read as a LINQ query that stops at the first failure.

Phase 10 (part): no Schematiq.Collections; the engine internal (unit 10P1).
- The kernel no longer depends on Schematiq.Collections: lists are returned as `IReadOnlyList<T>`, over `ImmutableArray<T>`,
  and `PackedList<T>` is gone. Bento's benchmark and the Booleans are unchanged.
- Every public type of `Occt` and `Kerf.Engine.*` is internal but `Transient`, which stays public as the base of the public
  geometry, with no public or protected members.
- `PublicSurfaceTests` read every public type, and fail on a public type left in the engine that isn't listed with its reason.
- The WebAssembly sample is on the public API alone. `Kerf.Kernel.Bento.Benchmarks` no longer sees the kernel's internals.
- Kerf's unit 10P2: no exception of the engine reaches a public caller. The public values, geometry and shapes refuse
  misuse with `ArgumentException`, `ArgumentOutOfRangeException`, `InvalidOperationException` or `NotSupportedException`,
  with messages free of OCCT's names; the engine still reads each as the failure it was, so results, codes and recordings
  are unchanged. `LockedShapeException` and `FrozenShapeException` went. A trimmed curve or surface given a NaN
  parameter is refused, where it overflowed the stack and killed the process. KERF0003 covers `Kerf.Modeling`.
**State and the coverage floor (unit 10P3).** BlendFunc's scratch is per instance, Contap's and the legacy classifier's per-thread state is gone, the Booleans' vertices lock is per operation: 5 `[ThreadStatic]`s remain, from 19, each justified in `docs/state.md`. `docs/coverage.md` is measured again, with a reason for each of the 81 packages under the floor; `tools/public-members.cs` writes `docs/public-members.md`, and every public member now runs in a test.

**One set of shapes (unit 10Q).** `IShape`, the non-generic base of `IShape<T>`, and the markers `IShape1D`, `IShape2D` and `IShape3D`, for extension members over kinds of shape, as the modelling layer's now are. `ShapeComparer<T>`. A typed shape's `Reversed()` keeps its type. `tools/b3d-differential.ps1` went, with the modelling layer's log it read.

**One set of values (unit 10Q2).** `Direction3D` converts implicitly to `Vector3D` and scales to one; `IGeometry` is new, and `Point3D` implements it. `IsClosed` means closed, on any shape, replacing `CheckClosed()`; OCCT's stored flags are internal (`ClosedFlag`, `InfiniteFlag`, `OrientableFlag`, `ConvexFlag`). The traversal is `AllVertices()` to `AllSolids()`, the measurements `IntegrateVolume()`, `IntegrateArea()` and `IntegrateLength()`, and the analytic surfaces `Cylinder3D`, `Cone3D`, `Sphere3D` and `Torus3D`, so that the modelling layer's plain names mean one thing in a file importing both. Results are unchanged.

## 0.1.0

Phase 0: groundwork, ported from OCCT V8_0_1 (b8f597c6).

- Licensed under LGPL 2.1 with the Open CASCADE exception, as OCCT.
- `OcctNet.Foundation`:
  - Standard: OCCT's exception hierarchy and its real and integer functions; Precision.
  - The gp package: points, vectors, directions, axes, coordinate systems, matrices, transformations,
    quaternions, conics and elementary surfaces, in 3D and 2D. Values are immutable to callers; each OCCT
    `Set` method has a `With` counterpart that returns a copy, and transformations have static factories
    (`gp_Trsf.Rotation(axis, angle)`).
  - `PackedList<T>`: an `IBentoList<T>` over one contiguous array, readable as a span.
- `OcctNet.Analyzers`: OCCTNET0001 reports a mutating call that would change a defensive copy.
- Tests: OCCT's gp GTests ported; oracle tests against native OCCT; Draw-lite, which parses all 17,237 cases
  of OCCT's Draw test corpus and lists the commands they need.
- `tools/port-status.cs` writes `docs/port-status.md`.

Phase 1 (part): bounding boxes and bounding volume hierarchies.

- `OcctNet.Foundation`: NCollection_Vec2/3/4 and NCollection_Mat3/4 (generic structs), NCollection_LinearVector;
  the Bnd package (Bnd_Box, Bnd_Box2d, Bnd_B2d/f, Bnd_B3d/f, Bnd_OBB, Bnd_Range, Bnd_Sphere, Bnd_BoundSortBox,
  Bnd_Tools); the BVH package (boxes, trees, binned/linear/sweep-plane/spatial-median builders, sorters,
  traversals, box sets, triangulations, distance fields), generic over the element and vector type.
- Tests: the Bnd, BVH and NCollection_Vec4 GTests; oracle tests of Bnd_Box and Bnd_OBB against native OCCT.

Phase 1 (part): meshes.

- `OcctNet.Foundation`: the Poly package (Poly_Triangulation with run-time single or double precision nodes and
  span access for bulk reads, Poly_Triangle, polygons in 2D, 3D and on triangulations, Poly_Connect,
  Poly_MergeNodesTool, Poly_CoherentTriangulation, Poly_MakeLoops, Poly's reading, writing and intersection
  tools); NCollection_AliasedArray, NCollection_PackedMapOfInteger, NCollection_Primes.
- Tests: unit tests of the Poly package (OCCT has no Poly GTests); oracle tests of Poly_Triangulation and
  Poly_Triangle against native OCCT.

Phase 1 (part): B-spline surfaces and conversions.

- `OcctNet.Foundation`: the BSplSLib package (B-spline and Bezier surface evaluation, knot insertion and removal,
  degree elevation, interpolation, BSplSLib_Cache); the Convert package (conics and quadrics to B-splines,
  composite Bezier curves and polynomial curves and grids to B-splines). Converters return copies of their results.
- Tests: the BSplSLib and Convert GTests; oracle tests of BSplCLib, BSplSLib, BSplSLib_Cache and Convert against
  native OCCT, all bit-identical.

Phase 2 (part): approximation leaves.

- `OcctNet.ModelingData`: AdvApprox (approximation of a function by B-splines, with cutting strategies); FEmTool
  (finite elements for AppDef's variational approximation); GProp (global properties of points, curves, surfaces
  and solids, principal properties, point-cloud equations); AppParCurves (MultiPoint, MultiCurve, MultiBSpCurve, and
  its least-squares, constraint and gradient algorithms as generic classes over a MultiLine and its tool,
  `IAppParCurves_LineTool<MultiLine>`).
- Tests: the GProp GTests; unit tests of AdvApprox, FEmTool and AppParCurves on known approximations; oracle tests
  of GProp_GProps and GProp_PrincipalProps against native OCCT (through BRepGProp), all bit-identical.

Phase 2 (part): constructions and sampling.

- `OcctNet.ModelingData`: GC (the makers of Geom and Geom2d curves, surfaces and transformations: arcs of conics,
  segments, lines, circles, ellipses, hyperbolas, parabolas, planes, cylinders, cones, trimmed cylinders and cones,
  mirrors, rotations, scales, translations), whose `Value()` returns its result frozen; CPnts and GCPnts (curve
  lengths, points at a curvilinear distance, uniform-abscissa, uniform-deflection, quasi-uniform and tangential
  deflection samplings), for 3D and 2D adaptors. GCE2d's deprecated aliases are not ported: use the GC 2D makers.
- Tests: the GC and GCPnts GTests; oracle tests of the GC arc makers and of GCPnts_AbscissaPoint,
  GCPnts_UniformDeflection and GCPnts_QuasiUniformDeflection against native OCCT, all bit-identical; test stand-ins
  for GeomAdaptor_Curve and Geom2dAdaptor_Curve, bit-identical to the native adaptors.
Phase 2 (part): surface approximation.

- `OcctNet.ModelingData`: AdvApp2Var (approximation of a function F(U, V) by B-spline surfaces,
  `AdvApp2Var_ApproxAFunc2Var`, with its evaluator, criterion and working classes). Results are frozen surfaces and
  copies of the error tables; approximations run in parallel.
- Tests: the AdvApp2Var GTests; unit tests reproducing planes and polynomial surfaces and approximating a sphere and
  sin(u)cos(v) under each continuity and criterion, in parallel too. Checked against native OCCT on 15,100 random
  approximations (not committed): bit-identical.
Phase 2 (part): discrete approximation.

- `OcctNet.ModelingData`: AppCont (least-squares approximation of a continuous function, with its constant tables
  generated by `tools/gen-appcont-matrices.cs`); AppDef (MultiLine and MultiPointConstraint, BSplineCompute and
  Compute, Variational with its smoothing criteria, and OCCT's instantiation classes); Approx_ComputeLine and
  Approx_BSplComputeLine as generic classes over a MultiLine and its tool (`IApprox_LineTool<MultiLine>`),
  Approx_MCurvesToBSpCurve, Approx_ParametrizationType and Approx_Status.
- Tests: the AppCont GTests; unit tests of AppCont, AppDef and Approx on sampled polynomials, circles and sines;
  oracle tests of AppDef_BSplineCompute and AppDef_Variational against native OCCT (through
  GeomAPI_PointsToBSpline), all bit-identical.

Phase 2 (part): curve extrema.

- `OcctNet.ModelingData`: the curve extrema of Extrema: point/curve (`Extrema_ExtPC`, `Extrema_LocateExtPC` and
  their root-search classes) and curve/curve (`Extrema_ExtCC`, `Extrema_ECC`, `Extrema_LocateExtCC`), in 3D and
  2D, with `Extrema_CurveTool`, `Extrema_Curve2dTool` and `Extrema_GlobOptFuncCCC0/C1/C2`.
- Tests: the Extrema_ExtPC GTest; unit tests on known answers and dense sampling, and 2D against 3D; oracle tests of
  Extrema_ExtPC (directly and through GeomAPI_ProjectPointOnCurve), Extrema_ExtPC2d (through
  Geom2dAPI_ProjectPointOnCurve) and Extrema_ExtCC (through BRepExtrema_DistShapeShape) against native OCCT,
  bit-identical but for last-bit differences on offset curves (test adaptor) and one line/circle pair.
Phase 2 (part): grid evaluation.

- `OcctNet.ModelingData`: GeomGridEval and Geom2dGridEval, OCCT 8's batch evaluators of curves and surfaces on
  parameter lists and grids (points and derivatives up to D3 and DN): one evaluator per curve and surface kind
  (conics, Bezier and B-spline on span caches, offsets, revolutions, extrusions, any other through its adaptor),
  and the dispatchers `GeomGridEval_Curve`, `GeomGridEval_Surface` (trimmed surfaces, the GeomAdaptor and
  transformed-surface adaptors) and `Geom2dGridEval_Curve`. Every evaluator implements
  `GeomGridEval.ICurveEvaluator`, `GeomGridEval.ISurfaceEvaluator` or `Geom2dGridEval.ICurveEvaluator`.
- Tests: the GeomGridEval and Geom2dGridEval GTests (and GeomAdaptor_TransformedSurface's grid test); consistency
  tests of every evaluator against point evaluation on random geometry, bit-identical where OCCT's grid uses the
  same algorithm, and OCCT's grid errors pinned.

Phase 2 (part): surface extrema.

- `OcctNet.ModelingData`: the surface extrema of Extrema: point/surface (`Extrema_ExtPS`, with `Extrema_GenExtPS` on
  a grid or a sphere tree, `Extrema_ExtPExtS`, `Extrema_ExtPRevS`, `Extrema_GenLocateExtPS`), curve/surface
  (`Extrema_GenExtCS`, `Extrema_GenLocateExtCS`, the `Extrema_GlobOptFuncCS/ConicS/CQuadric` functions) and
  surface/surface (`Extrema_ExtSS`, `Extrema_GenExtSS`, `Extrema_GenLocateExtSS`), with `Extrema_ExtFlag`,
  `Extrema_ExtAlgo` and `Extrema_POnSurfParams`; OCCT 8's ExtremaPC package (point/curve extrema per curve kind,
  `ExtremaPC_Curve`).
- `OcctNet.Foundation`: `NCollection_UBTree` and `NCollection_UBTreeFiller` (Bnd_Box and Bnd_Sphere implement
  `INCollection_UBTreeBnd<T>`), and `std_mt19937`.
- Tests: the ExtremaPC and NCollection_UBTree GTests; unit tests of the surface extrema; oracle tests of
  Extrema_ExtPS (through GeomAPI_ProjectPointOnSurf and BRepExtrema_DistShapeShape), Extrema_GenExtCS and
  Extrema_ExtSS (through BRepExtrema_DistShapeShape) against native OCCT, all bit-identical.
Phase 2 (part): bounding boxes.

- `OcctNet.ModelingData`: BndLib (boxes of gp lines, conics and elementary surfaces), BndLib_Add3dCurve,
  BndLib_Add2dCurve and BndLib_AddSurface, and OCCT 8's GeomBndLib: one evaluator per curve and surface kind
  (analytic boxes for conics and quadrics, pole hulls with sampled deflection for Bezier and B-spline geometry,
  offsets, revolutions, extrusions, any other through its adaptor) with `Box` and `BoxOptimal` (PSO + Brent/Powell
  refinement), and the dispatchers `GeomBndLib_Curve`, `GeomBndLib_Curve2d` and `GeomBndLib_Surface`.
  Extrema_ExtCS (curve/surface extrema); its generic paths wait for unit 4A's Extrema_ExtPS and Extrema_GenExtCS.
- Tests: the BndLib and GeomBndLib GTests; OCCT's box errors pinned; oracle tests of the boxes through native
  BRepBndLib (Add, AddOptimal, AddOBB on edges and faces) and of Extrema_ExtCS's analytic cases through
  BRepExtrema_DistShapeShape.

Phase 2 (part): GeomLib and Approx.

- `OcctNet.ModelingData`: GeomLib (the statics: 3D curves from 2D curves and from curves on surfaces, range changes,
  curve and surface extensions, inertia axes, closure and normal estimation; `GeomLib_CheckCurveOnSurface`,
  `GeomLib_Tool`, `GeomLib_IsPlanarSurface`, `GeomLib_Interpolate`, the tangent checks and the small classes); the
  rest of Approx (`Approx_SameParameter`, `Approx_CurveOnSurface`, `Approx_Curve2d`, `Approx_Curve3d`,
  `Approx_FitAndDivide` and `Approx_FitAndDivide2d`, `Approx_CurvlinFunc`, `Approx_CurvilinearParameter`,
  `Approx_SweepFunction`, `Approx_SweepApproximation`); `GeomConvert_SurfToAnaSurf`. GeomLib's functions never change
  the caller's geometry: where OCCT changes it in place, the port changes a copy.
- Tests: the GeomLib_CheckCurveOnSurface GTest; unit tests of every class; oracle tests of GeomLib_IsPlanarSurface,
  and, through BRepLib and BRepTools, of Approx_SameParameter, GeomLib::BuildCurve3d, GeomLib::SameRange and
  GeomLib_CheckCurveOnSurface against native OCCT, all bit-identical.

Phase 2 (part): projection.

- `OcctNet.ModelingData`: the rest of ProjLib (`ProjLib_ProjectedCurve`, `ProjLib_CompProjectedCurve`,
  `ProjLib_ComputeApprox`, `ProjLib_ComputeApproxOnPolarSurface`, `ProjLib_ProjectOnPlane`,
  `ProjLib_ProjectOnSurface`, `ProjLib_PrjFunc`, `ProjLib_PrjResolve`, `ProjLib.MakePCurveOfType`) and GeomProjLib:
  the 2D curves of 3D curves on surfaces (pcurves), normal projections on surfaces and projections on planes. The
  curves a projection keeps are frozen when handed out; GeomProjLib returns curves the caller owns.
- Tests: the ProjLib_ComputeApproxOnPolarSurface GTest; unit tests of every class; oracle tests of
  ProjLib_ProjectedCurve, ProjLib_ProjectOnPlane and GeomProjLib::ProjectOnPlane through GeomAPI::To2d and
  BRep_Tool::CurveOnSurface against native OCCT, all bit-identical.
Phase 3 (part): curve-surface intersection.

- `OcctNet.ModelingAlgorithms`: IntCurveSurface_HInter (curve/surface intersection: analytic for conics on
  quadrics, by the quadric's equation for other curves on quadrics, by polygon/polyhedron interference refined by
  Newton otherwise), generic over its curve, surface and tools as OCCT's templates are;
  Intf_InterferencePolygonPolyhedron, IntImp_IntCS, IntImp_ZerCSParFunc, IntImp_ComputeTangence; IntSurf_Quadric and
  IntSurf_QuadricTool; GeomAPI: To3d, ProjectPointOnCurve, ProjectPointOnSurf, ExtremaCurveCurve, ExtremaCurveSurface,
  ExtremaSurfaceSurface, Interpolate, PointsToBSpline and IntCS.
- Tests: the IntCurveSurface, IntSurf_Quadric and GeomAPI GTests; unit tests on known intersections; oracle tests of
  GeomAPI_IntCS on every kind of curve and surface, and of the GeomAPI projections, Interpolate and PointsToBSpline.
Phase 3 (part): global properties.

- `OcctNet.ModelingAlgorithms`: BRepGProp (length, area, volume, centre of mass and inertia of shapes: the exact,
  adaptive, Gauss-Kronrod and mesh-based forms, with functional overloads returning a new `GProp_GProps`, and the
  integration classes); BRepCheck's statics, `BRepCheck_Status`, `BRepCheck_Result` and `BRepCheck_Shell`;
  `BRepLib_ValidateEdge` and `BRepLib_CheckCurveOnSurface`.
- Tests: the BRepGProp GTests (with BRep_Builder shapes until the primitives are ported); known-answer tests; oracle
  tests of every BRepGProp form and of BRepGProp_Face against native OCCT, all bit-identical, and of BRepCheck_Shell
  and BRepLib_ValidateEdge through BRepCheck_Analyzer.
Phase 3 (part): cross-unit integration.

- `OcctNet.ModelingData`: `BRepTools_NurbsConvertModification` projects pcurves again (ProjLib_ComputeApprox on
  planes, ProjLib_ComputeApproxOnPolarSurface elsewhere and on seams), so shapes with faces convert to NURBS.
- `OcctNet.ModelingAlgorithms`: BRepGProp and `BRepLib_CheckCurveOnSurface` use `BRepAdaptor_Curve`;
  `BRepGProp_Cinert` and `BRepGProp_EdgeTool` take a `BRepAdaptor_Curve`, as in OCCT.
- `OcctNet.Foundation`: `NCollection_DataMap.Bound` replaces the value of a bound key, as OCCT's does; `TryBind` and
  `TryBound` keep it.
- Tests: NCollection_DataMap GTests; NURBS conversion unit tests; oracle tests of the NURBS conversion through
  BRepBuilderAPI_GTransform, and of BRepBuilderAPI_Transform now with faces on planes, all bit-identical.
Phase 3 (part): BRep input and output.

- `OcctNet.ModelingData`: OCCT's text format of shapes (`TopTools_ShapeSet`, `TopTools_LocationSet`,
  `BRepTools_ShapeSet`, `BRepTools.Read`, `Write` and `Dump`, `TopTools.Dump`), versions 1 to 3, with or without
  triangulations and normals, and the binary format (`BinTools`: `BinTools.Read`/`Write`, the shape, location,
  curve and surface sets, `BinTools_ShapeReader`/`ShapeWriter`, `BinTools_IStream`/`OStream`). Shapes read are frozen.
  Untrusted input is read safely: counts are checked against the input's length, nesting is capped, and malformed
  input throws `Storage_StreamFormatError` instead of hanging, overflowing the stack or allocating without bound.
- `OcctNet.Foundation`: `Storage_StreamFormatError`; `Standard_Stream`'s C buffers, getline and atoi, and its input
  length (`SetInputLength`, `InputLength`) and count checks.
- Tests: unit and fuzz tests of both formats; oracle tests against native `BRepTools::Write` and `Read`: random
  native shapes, shapes built identically on both sides, the port's files read natively, the 37 `.brep` files of
  OCCT's data and 2,327 files of its test dataset, all byte-identical.
Phase 3 (part): classification and boxes.

- `OcctNet.ModelingAlgorithms`: BRepClass (`BRepClass_FaceClassifier`: points in, out and on faces, in 2D or
  projected from 3D, with its explorer, intersector and edge), BRepTopAdaptor (`BRepTopAdaptor_FClass2d`, the fast
  polygon classifier of a face, and the topological tool), IntCurvesFace (line and curve against a face or a shape),
  BRepClass3d (`BRepClass3d_SolidClassifier`: points in, out and on solids, the infinite point, the outer shell),
  BRepIntCurveSurface (`BRepIntCurveSurface_Inter`: ray/shape intersection) and BRepBndLib (`Add`, `AddClose`,
  `AddOptimal`, `AddOBB`, with functional forms `Box`, `BoxClose`, `BoxOptimal` and `OBB` returning a new box).
  They only read their inputs.
- Tests: the BRepClass3d_SolidClassifier GTests (with BRep_Builder shapes until the primitives are ported);
  known-answer tests of every package; oracle tests of BRepClass3d_SolidClassifier, BRepIntCurveSurface_Inter and
  every BRepBndLib form against native OCCT on random boxes, cylinders (also as surfaces of extrusion and
  revolution), holed faces, natural and B-spline faces, compounds and meshed shapes, all bit-identical.
Phase 3 (part): Draw-lite shape commands.

- Draw-lite (tests, not packaged): Draw variables held by Tcl variables (`DrawInterpreter.SetDrawable`, `GetShape`,
  Draw::Atof), and Draw's shape commands with Draw's output: `brestore`/`restore`, `bsave`, `readbrep`, `writebrep`,
  `binsave`, `binrestore` (shapes, numbers, DrawTrSurf geometry files), `isdraw`, `dtyp`, `dname`, `whatis`, `copy`,
  `renamevar`, `dump`, `dset`, `dval`, `nbshapes`, `numshapes`, `explode`, `nexplode`, `exwire`, `compound`, `add`,
  `emptycopy`, `orientation`, `treverse`, `complement`, `invert`, `tolerance`, `maxtolerance`, `lprops`, `sprops`,
  `vprops`, `vpropsgk`, `purgeloc`, `checkloc`, `setflags`, `ttranslate`, `trotate`, `tmove`, `reset`, `tscale`,
  `tmirror`, `tcopy`, and the display settings without a viewer. OCCT's `checknbshapes`, `checkprops` and
  `checkmaxtol` run on them. `DrawCorpus.RunAndCheck` runs a corpus case and judges its log with OCCT's `_check_log`.
- Tests: every command's text against Draw's source; oracle tests of the commands on OCCT's 42 data/occ files and of
  the transformation and copy commands against BRepBuilderAPI_Transform and BRepBuilderAPI_Copy, all bit-identical.
Phase 3 (part): BRepLib.

- `OcctNet.ModelingAlgorithms`: the BRepLib makers (`BRepLib_MakeVertex`, `MakeEdge`, `MakeEdge2d`, `MakeWire`,
  `MakePolygon`, `MakeFace`, `MakeShell`, `MakeSolid`), whose results are frozen; `BRepLib_FindSurface`;
  `BRepLib_FuseEdges`; `BRepLib_PointCloudShape` and `BRepLib_ToolTriangulatedShape`; BRepLib's queries
  (`CheckSameRange`, `ContinuityOfFaces`, `SortFaces`, `FindValidRange`, `BoundingVertex`, `ExtendFace`, ...), OCCT's
  non-destructive `SameParameter` and `UpdateTolerances` with a `BRepTools_ReShape`, and functional twins of the
  in-place statics (`SameParametered`, `TolerancesUpdated`, `InnerTolerancesUpdated`, `Curves3dBuilt`,
  `RegularityEncoded`, `DeflectionUpdated`, `NormalConsistencyEnsured`, `ClosedSolidOriented`) that return a new frozen
  shape sharing every sub-shape they leave unchanged. A face made from a frozen wire keeps the wire's edges, so faces
  built on shared edges share them.
- `OcctNet.ModelingData`: an exact copy for write and its counterpart that gives back the original's unchanged
  sub-shapes (internal).
- Tests: the BRepLib_MakeWire GTest; known-answer tests of every class; oracle tests of the makers against
  native BRepBuilderAPI makers, of FindSurface, of the statics and their twins against native in-place statics, and
  of regularity, face extension and mesh statics on random native shapes, byte-identical (apart from the Free flag of
  reused frozen inputs).

Phase 3 (part): shape checking.

- `OcctNet.ModelingAlgorithms`: BRepCheck (`BRepCheck_Analyzer`: validity of a shape and each of its sub-shapes, on
  its own and in context, with OCCT's statuses; topology-only, exact and parallel modes; the vertex, edge, wire,
  face and solid checks; `BRepCheck.SelfIntersection`). The parallel mode gives the serial mode's statuses. Checking
  only reads the shape.
- Tests: the BRepCheck_Face GTests (with BRep_Builder shapes until the makers are ported); known-answer tests on
  valid and deliberately broken shapes in both modes; oracle tests of the analyzer's verdict on every sub-shape
  against native OCCT on random native shapes, natively broken edges, vertices, wires, faces, shells and solids, the
  37 `.brep` files of OCCT's data and 2,320 files of its test dataset: all 396,118 verdicts agree.

Phase 3 (part): Draw-lite checking commands.

- Draw-lite (tests, not packaged): `checkshape` (contextual and structural dumps, `-top`, `-short`, `-parallel`,
  `-exact`, the faulty shapes bound as Draw binds them), `checksection`, `computetolerance`, `bounding` (every option
  but `-shape`, which needs the primitives, and `-dumpJson`), `isbbinterf`, `gbounding`, `getcoords`, `bclassify`,
  `b2dclassify`, `point`, `dchrono`/`chrono` and `dlog`. Corpus runs log each command's words and result as DRAWEXE
  does, so OCCT's log rules judge checkshape's verdicts.
- Tests: every command's text against Draw's source; oracle tests of checkshape's verdicts, bounding's boxes
  (bit-identical) and bclassify's states against native OCCT on OCCT's data files and test dataset. The corpus has
  380 runnable cases (23 before), most of them waiting for data files that aren't public.
Phase 3 (part): distances and makers.

- `OcctNet.ModelingAlgorithms`: BRepExtrema (`BRepExtrema_DistShapeShape` with its supports, parameters and inner
  solutions, single- or multi-threaded; `BRepExtrema_DistanceSS`, the Ext tools, `BRepExtrema_Poly`; the mesh tools
  `BRepExtrema_ShapeProximity`, `ProximityValueTool`, `ProximityDistTool`, `OverlapTool`, `SelfIntersection`,
  `TriangleSet`); BRepBuilderAPI's makers (`BRepBuilderAPI_MakeVertex`, `MakeEdge`, `MakeEdge2d`, `MakePolygon`,
  `MakeWire`, `MakeFace`, `MakeShell`, `MakeSolid`), the public way to build shapes, whose results are frozen, with
  `Modified` answering the copies a maker made of frozen inputs; the modifications `BRepBuilderAPI_Transform`,
  `GTransform`, `Copy` and `NurbsConvert`; `BRepBuilderAPI_Collect` and `BRepBuilderAPI_FindPlane`. Every
  `BRepBuilderAPI_MakeShape` is a `BRepTools_History.IAlgorithm`. Inputs are never changed.
- `OcctNet.ModelingData`: `BRepTools_Modifier` reused on a shape it modified before no longer throws on the
  published result's edges.
- Tests: the BRepExtrema_DistShapeShape and BRepBuilderAPI (Copy, MakeEdge, MakeFace, MakeWire, Transform) GTests, and
  the TopoDS, BRep_Tool, BRepTools_ReShape, BRepAdaptor_CompCurve and BRepGProp GTests on the real makers; Draw-lite's
  t-commands and `tcopy` run the real makers; known-answer tests; oracle tests of BRepExtrema_DistShapeShape on twin
  scenes and native shapes (bit-identical), of the makers through BRep text (byte-identical apart from the Free flag of
  reused frozen inputs) and of Transform, Copy and GTransform on twin scenes and native shapes (bit-identical).

Phase 3 (part): primitives and sweeps.

- `OcctNet.ModelingAlgorithms`: TKPrim. The makers `BRepPrimAPI_MakeBox`, `MakeWedge`, `MakeCylinder`, `MakeCone`,
  `MakeSphere`, `MakeTorus`, `MakeRevolution` (whole and partial, with their faces, shells and solids),
  `MakeHalfSpace`, `MakePrism` (finite, semi-infinite and infinite) and `MakeRevol`, with `Generated`, `IsDeleted`,
  `FirstShape` and `LastShape`; `BRepPreviewAPI_MakeBox`. Results are frozen; BRepPrim, BRepSweep and Sweep are
  internal. Sweeping a frozen profile works on a copy and shares what the sweep didn't change with the profile, which
  is never changed; histories answer for the caller's sub-shapes.
- `OcctNet.ModelingData`: `BRepTools.KeepUnchangedIn` (internal), for operations that build on a copy for write.
- Draw-lite (tests, not packaged): `box` (every form, `-preview`), `wedge`, `pcylinder`, `pcone`, `psphere`, `ptorus`,
  `halfspace`, `prism`, `revol`, and `bounding -shape`.
- Tests: TKPrim's GTests; the phase 3 GTests that used stand-ins now use the real makers, and BRepTools_Test and the
  shape cases of the Bnd_BoundSortBox and Bnd_OBB GTests are ported; known-answer tests; oracle tests of every maker,
  their histories and the Draw commands against native OCCT through BRep text (byte-identical apart from the Free flag
  of frozen inputs). The corpus has 395 runnable cases (28 OK).
Phase 3 (part): Draw-lite construction, distance and modification commands.

- Draw-lite (tests, not packaged): GeomliteTest's curves and surfaces (line, circle, ellipse, parabola, hyperbola,
  Bezier and B-spline curves and surfaces in 3D and 2D, extsurf, revsurf, offset, trim, isos, reverse, segment,
  setorigin, cvalue, svalue, bounds, parameters, length), GeometryTest's `gcarc` and `extrema`, DrawTrSurf's
  transformations, `distmini`, `dist`, `proximity`, `selfintersect`, the topology constructions (`vertex`, `edge`,
  `mkedge`, `wire`, `polyline`, `polyvertex`, `mkcurve`, `mkpoint`, `mk2dcurve`, `uisoedge`, `visoedge`, `profile`,
  `mkface`, `mkshell`, `mksurface`, `mkplane`, `pcurve`) and modifications (`nurbsconvert`, `deform`, `scalexyz`,
  `sameparameter`, `updatetolerance`, `encoderegularity`, `solidorientation`, `mkedgecurve`, `findplane`, `compare`,
  `issubshape`, `wexplo`, `getedgeregularity`, `projponf`), and `getsourcefile`'s `Draw_Files`. Geometry is frozen when
  bound; in-place commands work on a copy of the shape's graph and bind it.
- Tests: unit tests of every command; oracle tests against native OCCT of the geometry (131,947 values), distmini
  (values and solution shapes), the constructions and `profile` (BRep text) and the modifications, all identical but
  for vertex tolerances OCCT's in-place UpdateTolerances writes in address order. The corpus has 714 runnable cases
  (380 before) and 327 OK (13 before); `lappend` no longer formats a whole list again on each call.

Phase 4 (part): Draw-lite mesh readers.

- Draw-lite (tests, not packaged): `tclean`, `trinfo` (with `-lods`), `triepoints`, `trmergenodes`, `correctnormals`,
  `triarea`, `tricheck` (MeshTest_CheckTopology) and `mpparallel`; DrawResources' `checktrinfo` runs. In-place commands
  work on a copy of the shape's graph and bind it. DrawCorpus reads the grids that list their cases' directory in a
  `cases.list` (mesh, heal, chamfer, opengl, xml): 21,068 cases where it saw 17,237. `dgetenv` of an unset variable
  returns an empty result, as Draw's.
- Tests: unit tests of every command; oracle tests against native OCCT on 525 meshed shapes (5,769 values of trinfo,
  triarea and tricheck bit-identical, 974 BRep texts after correctnormals and tclean byte-identical). The corpus has
  961 runnable cases (734 before) and 452 OK (347 before).
Phase 4 (part): ShapeAnalysis's wire analysis.

- `OcctNet.ModelingAlgorithms`: from TKShHealing, ShapeExtend (`ShapeExtend`, `ShapeExtend_Status`,
  `ShapeExtend_WireData`, `ShapeExtend_ComplexCurve`), ShapeAnalysis (`ShapeAnalysis`'s statics including `OuterWire`,
  `IsOuterBound`, `TotCross2D` and `GetFaceUVBounds`; `ShapeAnalysis_Wire`, `WireOrder`, `Edge`, `Curve`, `Surface`,
  `TransferParameters`, `TransferParametersProj`) and ShapeBuild (`ShapeBuild`, `ShapeBuild_Edge`,
  `ShapeBuild_Vertex`). The analyses only read their shapes and work on frozen ones; the shapes and curves they hand
  out are frozen. ShapeBuild_Edge's writers are internal, with functional twins (`RangesCopied`, `PCurveReplaced`,
  `Curve3dBuilt`, ...) that return a new edge sharing what didn't change.
- Tests: OCCT's ShapeAnalysis_Edge GTest; known-answer tests (wire order of shuffled and reversed edges, outer wires of
  faces with holes, every wire check, projections, UV on every surface kind including singular ones); oracle tests of
  ShapeAnalysis_Curve, Surface and Edge against native OCCT, 907,014 values, all bit-identical.
Phase 4 (part): mesh foundations and data model (unit 4A).

- `OcctNet.Foundation`: Message_Algorithm with its statuses (Message_ExecStatus, Message_Status), Message_Msg,
  Message_Messenger and printers, and the `Message` statics, whose default messenger sends nothing;
  NCollection_EBTree and NCollection_MapAlgo; an internal `OSD_Parallel` that every parallel loop of the port now uses,
  with a sequential fallback where there are no threads (WebAssembly).
- `OcctNet.ModelingAlgorithms`: IMeshData, IMeshTools (with `IMeshTools_Parameters`, an equatable value) and
  BRepMeshData; BRepMesh_GeomTool, DefaultRangeSplitter, ShapeTool, Deflection, EdgeParameterProvider, DiscretRoot;
  `BRepMesh_IncrementalMesh`'s copy-for-write shell: a frozen shape is meshed on a copy, `Shape()` returns the frozen
  result with a `History()`, and `Meshed` is the functional form (the meshing algorithms themselves come in later
  units).
- `OcctNet.ModelingData`: the mesher's non-freezing attach of triangulations and polygons, frozen with the shape when
  it is published.
- Tests: the GTests of BRepMesh_GeomTool, Message_Messenger, OSD_Parallel and NCollection_UBTree's EBTree cases; unit
  tests of the data model, the parameters, the shape tools and the copy for write; an oracle of GeomTool and Deflection
  against native OCCT's free-edge polygons (300 edges, 6,126 points, every count equal, values within 5e-15 through
  BRep text).

Phase 4 (part): the discrete model and edge discretisation (unit 4C).

- `OcctNet.ModelingAlgorithms`: BRepMesh_ShapeVisitor and BRepMesh_ModelBuilder (the discrete model of a shape, each
  face's wires ordered by ShapeAnalysis_Wire, the outer one first), BRepMesh_CurveTessellator and
  BRepMesh_EdgeTessellationExtractor, BRepMesh_EdgeDiscret (edges tessellated, or read back from polygons that fit),
  BRepMesh_FaceChecker and BRepMesh_ModelHealer (gaps closed, self-intersecting edges refined), BRepMesh_ModelPreProcessor
  (meshes reused or outdated, cone seams split) and BRepMesh_ModelPostProcessor (polygons, deflections and parameters
  stored); BRepMesh_ConeRangeSplitter, which the pre-processor uses. Their parallel loops give the serial result bit for
  bit; they write into a shape only through BRepMesh_ShapeTool, and never into a frozen one (the mesher copies first).
- Tests: unit tests of the model (faces, wires, edges, pcurves and orientations on primitives and data/occ shapes), the
  edges' polygons (reused from meshes the shapes carry, outdated ones removed from a copy), the healer, the cone seams,
  the post-processor through BRepMesh_IncrementalMesh with a stub face mesher, and parallel against serial; an oracle
  against native BRepMesh_IncrementalMesh's edge polygons on data/occ, random primitives and re-meshed shapes: every
  polygon has the port's points, 47,851 inner nodes bit for bit, parameters within the 15 digits of BRep text.
Phase 4 (part): the Delaunay core (unit 4B).

- `OcctNet.ModelingAlgorithms`: BRepMesh's plain types (`BRepMesh_Vertex`, `Edge`, `OrientedEdge`, `Triangle`,
  `PairOfIndex`, `Circle`, `DegreeOfFreedom`), the cell-filter inspectors and the circle and vertex tools,
  `BRepMesh_DataStructureOfDelaun` and its selector, `BRepMesh_Delaun` (Watson's algorithm with constraint insertion and
  frontier adjustment), `BRepMesh_MeshTool`, `BRepMesh_Classifier` and `BRepMesh_Triangulator`; Delabella, the MIT
  Delaunay library inside OCCT (`IDelaBella`), with its notice kept and listed in THIRD-PARTY-NOTICES.md.
- `OcctNet.Foundation`: `std_heap`, C++'s heap algorithms as MSVC implements them, so that ties sort in native OCCT's
  order; `NCollection_IntDataMap`, OCCT's data map of integer keys iterating in OCCT's order.
- Tests: OCCT's BRepMesh_CircleTool GTest and BRepMesh_Delaun GTests (three of the six wait for unit 4D's face
  meshing); property tests of Watson's triangulation on random points, grids, collinear runs, duplicates and
  near-cocircular points (empty circumcircles, Euler's formula, orientation, frontier and internal links kept, the same
  result on every run and in parallel); Watson against Delabella on the same constrained point sets; the heap order
  against orders recorded from an MSVC-built program; Delabella against output recorded from OCCT's own delabella.cpp
  built with MSVC.

Phase 4 (part): face meshing and the incremental mesh (unit 4D). Shapes now mesh as native OCCT meshes them.

- `OcctNet.ModelingAlgorithms`: the face algorithms (`BRepMesh_BaseMeshAlgo`, `ConstrainedBaseMeshAlgo`,
  `DelaunayBaseMeshAlgo`, `CustomBaseMeshAlgo`, `DelabellaBaseMeshAlgo`; the node-insertion, Delaunay node-insertion and
  deflection-control algorithms as generics over the range splitter, for Watson and, as `...OfDelabella`, for
  Delabella); the range splitters of every surface type (UVParam, NURBS, Cylinder, Sphere, Torus, Extrusion, Undefined,
  BoundaryParams); `BRepMesh_MeshAlgoFactory` and `BRepMesh_DelabellaMeshAlgoFactory`; `BRepMesh_FaceDiscret`;
  `BRepMesh_Context`; the plugin registry (`BRepMesh_DiscretAlgoFactory` with "FastDiscret" and `XBRepMesh_Factory`'s
  "XBRepMesh", `BRepMesh_DiscretFactory`). `BRepMesh_IncrementalMesh` now meshes: a frozen shape on a copy, whose
  frozen result is `Shape()` with a `History()`, or functionally with `BRepMesh_IncrementalMesh.Meshed(shape,
  parameters)`. Parallel meshing gives the serial result bit for bit.
- Tests: OCCT's BRepMesh_BaseMeshAlgo, BRepMesh_DiscretAlgoFactory and BRepMesh_IncrementalMesh GTests (the Fuse case
  waits for phase 6) and the three BRepMesh_Delaun GTests that waited for face meshing; unit tests of the range
  splitters, of Watson and Delabella on every surface type, of the parameters native OCCT's binding can't set, and of
  Delabella in parallel. Oracle milestone 1 against native BRepMesh_IncrementalMesh on the same input, with several
  deflections and angles, relative and absolute, serial and parallel: data/occ, the mesh group's 82 public data
  files, the public dataset's .brep files, random native shapes (primitives, fillets, Booleans, sweeps) and
  re-meshed shapes. Every compared face is bit-identical to native (nodes, UV nodes, triangles, deflection), every
  BRep text is byte-identical but for the Checked flag of faces the mesher keeps (decision 4605), the status flags
  agree, and BRepGProp and BRepBndLib on the meshes are bit-identical; the port's parallel runs equal its serial run.

Phase 4 (part): Draw-lite incmesh and the corpus milestone (unit 4L2).

- `OcctNet.ModelingAlgorithms`: meshing for display as OCCT's presentations do it: `Prs3d_Drawer` (its deflection
  settings: type of deflection, deviation coefficient and angle, maximal chordial deviation, automatic
  triangulation, links), `Prs3d.GetDeflection`, `StdPrs_ToolTriangulatedShape` (`GetDeflection`, `IsTessellated`,
  `Tessellate`, with a twin returning the meshed copy and its history), `Aspect_TypeOfDeflection`.
- Draw-lite (tests, not packaged): `incmesh` with all its options (`-a`, `-min`, `-relative`, `-parallel`, `-algo`,
  `-di`, `-ai`, `-prs`, `-surf_def_all`, `-surf_def_off`, `-int_vert_off`, `-force_face_def`, `-decrease`,
  `-adjust_min`), `tessellate`, `mpnames`, `mpsetdefaultname`, `mpgetdefaultname`, `mpincmesh`; `mpparallel` sets the
  mesher's process-wide default. Commands that change a shape in place in OCCT (incmesh, mpincmesh, tclean,
  trmergenodes, correctnormals, and the viewer's meshing) now rebind every variable that shares the changed
  sub-shapes, as in DRAWEXE. `vdisplay`, `vsetdispmode`, `vdefaults`, `vinit`, `vclose`, `verase`, `vremove`, `vclear`
  and `vdisplayall` mesh what they display as AIS does (selection and shaded presentation); `vglinfo` is ignored.
  The corpus scan knows the mesh group's STEP-only branch. Tcl's `time` returns an integer for one iteration.
- Tests: unit tests of every command and of the StdPrs port; oracle tests of incmesh (deflection, `-relative`, `-a`,
  `-parallel`) and of vdisplay's meshing against native OCCT on 72 shapes: 288 runs, every BRep text byte-identical
  (4 but for the Checked flag of decision 551), every exploded face rebound to native's mesh, 1,656 values equal.
  **The corpus milestone**: 21,068 cases, 1,834 runnable (961 before), 841 OK, 28 BAD (OCCT's own TODOs),
  1 IMPROVEMENT, 947 SKIPPED (data files the public dataset lacks), 17 FAILED (later commands, viewer queries,
  plugin loading, an OCCT test bug; none on a port bug). The mesh group: 684 of its 699 cases runnable, 233 OK,
  16 BAD (OCCT's TODOs), 435 SKIPPED, none FAILED.
Phase 4 (part): the viewer's mesh extraction, benchmarks and the WebAssembly spike (unit 4W).

- `OcctNet.ModelingAlgorithms`: `OcctNet.MeshExtraction` (an OCCT.NET addition, namespace `OcctNet`): a meshed
  shape's faces as located nodes, outward normals and outward-wound triangles, and its edges as polylines, in flat
  `MeshBuffers` laid out as build123d.NET's native `Extras.ExtractMesh`, in the order of build123d.NET's
  `Shape.Faces()` and `Edges()` (`MeshExtraction.Faces`, `Edges`) or of given sub-shapes. It only reads the shape
  (normals are computed on a copy), so it is safe from any thread on a shared shape.
- `benchmarks/OcctNet.Benchmarks`: mesh-and-extract against native OCCT through build123d.NET (BenchmarkDotNet in
  process, a Stopwatch table, allocation sampling). Serially the port is 0.9–1.5× native; in parallel it is
  allocation-bound (1.4–4.5× native under workstation GC). Typical models mesh and extract in 7–65 ms.
- `samples/OcctNet.WasmDemo`: reads a .brep, meshes it, extracts the buffers and draws them with WebGL 2 in the
  browser, built as Bento.Web is (plain WebAssembly, interpreter, single-threaded, trimmed, no workload);
  `run-headless.ps1` runs it over a folder in headless Chrome and compares with win-x64. OcctNet's trimmed
  assemblies are 2.39 MB (1.08 MB Brotli); typical models take 0.24–0.7 s in the browser; 92.5% of faces and every
  edge are bit-identical to win-x64 (the rest come from the browser's sin and cos).
- Tests: unit tests of the extraction (counts, order, normals on planar and curved faces, located, mirrored and
  reversed shapes, edges from polygons on triangulation, 3D polygons and GCPnts, thread safety); the extraction
  oracle against native `Extras.ExtractMesh`: every face and edge float-identical, on native's own meshes read back
  (1,738 faces, 9,724 edges), end to end on data/occ, the mesh group's 82 public files and native shapes (1,448 faces,
  6,509 edges), and on unmeshed shapes (4,786 edges by GCPnts_TangentialDeflection).
Phase 5 (part): intersection of triangulated surfaces (unit 5E).

- `OcctNet.ModelingAlgorithms`: the IntPolyh package, all of it. `IntPolyh_Intersection` intersects two surfaces
  through their triangulations (a 10 x 10 or given sampling, refinement by deflection, the interfering triangles by a
  BVH of their boxes, the four shifted intersections when the standard one finds too few) and gives section lines
  and tangent zones, the start points of IntPatch's parametric intersections. `IntPolyh_MaillageAffinage` (the
  algorithm), `IntPolyh_Triangle`, `Point`, `Edge`, `Couple`, `StartPoint`, `SectionLine`, `Tools` and
  `IntPolyh_Array`, whose items never move as it grows. Serial, and the same every run.
- Tests: OCCT's IntPolyh_Point and IntPolyh_Intersection GTests; unit tests on B-spline pairs (couples are real
  contacts of refined triangles in BVH order, start points lie on both surfaces, every bit the same over repeated,
  fresh and serial runs; the advanced intersection on tangent surfaces; the enlarged-zone retry and the other OCCT
  bugs reproduced); the array, section line, couple hasher and dumps.
Phase 6 (part): sewing (unit 6S).

- `OcctNet.ModelingAlgorithms`: BRepBuilderAPI_Sewing (free, contiguous, multiple and degenerated edges, cutting of
  bounds, non-manifold mode, `Modified`/`ModifiedSubShape`, the context), BRepBuilderAPI_FastSewing,
  BRepBuilderAPI_VertexInspector and BRepBuilderAPI_BndBoxTreeSelector. What sewing hands out is frozen; given frozen
  shapes it sews an exact copy and gives back the caller's own sub-shapes wherever it changed nothing, so the inputs
  are never written.
- Tests: OCCT's BRepBuilderAPI_Sewing GTests; unit tests of sewing and fast sewing; oracle tests against native
  sewing, as Draw's `sewing` sets it up, on the exploded faces of 35 data/occ shapes at 1e-6, 1e-2, 1 and 100 (140
  sewings) and on the sewing group's 501 public cases at their grids' tolerances and their 499 inputs at 1e-6:
  every sewn shape, edge list, `Modified` and `ModifiedSubShape` equal through BRep text (5,261 of 5,807 texts
  byte-identical, the rest but for the Free flag), every count equal; where native OCCT varies from run to run (2
  sewings) the port gives one of its results.
Phase 5 (part): intersection foundations (unit 5A).

- `OcctNet.ModelingAlgorithms`: the rest of IntSurf (IntSurf_PntOn2S, IntSurf_LineOn2S, path and interior points
  and their tools, transitions, the package's MakeTransition and SetPeriod); IntImp's surface/surface generics
  (`IntImp_Int2S`, `IntImp_ZerParFunc`, `IntImp_ZerImpFunc`) and IntStart's (`IntStart_SearchOnBoundaries`,
  `IntStart_SearchInside`), each one generic class over struct tools with static abstract members, which sets the
  pattern for IntPatch, IntWalk, GeomInt and BRepApprox; IntPatch's data model and tools: IntPatch_Point, the lines
  (Line, PointLine, GLine, ALine, WLine, RLine), HInterTool, HCurve2dTool, ArcFunction, CSFunction, CurvIntSurf,
  TheSOnBounds with its path point and segment, TheSearchInside, TheSurfFunction, the polygons (Polygo, PolyArc,
  PolyLine) and RstInt. Lines are working objects with OCCT's mutators; points are values.
- Tests: OCCT's IntSurf_LineOn2S GTest; unit tests of the walking-line containers (insertion, splitting, the lazy
  boxes, WLine's and RLine's vertex parameters), of the boundary search on cylinders and spheres cut by planes
  (points, segments, transitions), the interior search, Int2S and CurvIntSurf, GLine and ALine, the polygons and
  RstInt; the oracle's comparison of intersection lines with native OCCT (decision 5003), checked on native
  GeomAPI_IntSS results.
Phase 6 (part): BOP data and reports, the phase 6 pilot (unit 6A).

- `OcctNet.Foundation`:
  - Message's reports and alerts: Message_Report, Alert, AlertExtended, CompositeAlerts, Attribute,
    AttributeMeter, AttributeObject, AttributeStream, Level, PrinterToReport and MetricType.
  - `std_sort`, which gives MSVC's `std::sort` order on ties.
  - `OSD_Parallel.MaxDegreeOfParallelismScope`, a test hook.
- `OcctNet.ModelingData`: TopoDS_AlertWithShape and TopoDS_AlertAttribute. An alert's `GetShape()` publishes the
  shape (frozen).
- `OcctNet.ModelingAlgorithms`:
  - BOPDS, the Boolean data structure, all of it.
  - IntTools' data classes: Range, Root, CommonPrt, Curve, PntOnFace, PntOn2Faces, MarkedRangeSet, ShrunkRange,
    the range samples and localize data, and the package statics.
  - BOPAlgo_Options, Algo, the alerts (41 classes, checked by type), GlueEnum, Operation, CheckStatus,
    SectionAttribute, BuilderShape and ToolsProvider.
  - From BOPTools: Parallel (each thread gets its own IntTools_Context), the box trees and selectors, and three
    AlgoTools statics. The IntTools_Context shell.
  - **The public Boolean API**: BRepAlgoAPI_Cut, Fuse, Common, Section and Splitter, with BuilderAlgo,
    BooleanOperation and Algo.
    - They keep OCCT's classes and calls, and publish frozen results, histories and section edges.
    - The functional twins `BRepAlgoAPI.Fuse/Common/Cut/Section/Split(args, tools, in BRepAlgoAPI_Options, out
      history)` take an equatable options struct with OCCT's defaults.
    - Until units 6E and 6F port the pave filler and the builders, a stub builder gives OCCT's result for
      arguments that don't interfere, and reports `BOPAlgo_AlertIntersectionFailed` for arguments that do.
- Tests:
  - reports and alerts (gravities, type checks, merging, levels, printers, DumpErrors and DumpWarnings);
  - BOPDS built by hand (paves, pave blocks, common blocks, interference tables, face infos, the iterators'
    sorted pairs, serial against parallel);
  - IntTools' data classes;
  - the API shell (frozen inputs never written, frozen `Shape()`, histories, errors, the options struct, the
    twins);
  - the per-thread context;
  - the harnesses later units compare Booleans with:
    - BRep bytes with the Free flag masked;
    - a canonical match of sub-shapes by geometry, tested on shapes renumbered at random;
    - serial against parallel at several degrees of parallelism;
    - histories through the canonical match.
- OCCT bugs found:
  - Message_Report deletes alerts that merge;
  - Message_CompositeAlerts::Clear(type) never ends (fixed);
  - IntTools_Curve::D0 accepts any parameter;
  - IntTools_CommonPrt's copy loses its flag, and its assignment accumulates ranges;
  - IntTools::RemoveIdenticalRoots skips roots;
  - BOPDS_FaceInfo::Clear and BOPDS_DS::Paves;
  - BOPAlgo_Builder::SetArguments.

Phase 6 (part): healing foundations (unit 6H1).

- `OcctNet.ModelingAlgorithms`:
  - ShapeExtend's rest: `ShapeExtend_CompositeSurface` (a geometry with `With*` twins), `Explorer`,
    `MsgRegistrator`, `BasicMsgRegistrator`, `Parametrisation`.
  - `ShapeBuild_ReShape`, Shape Healing's context.
  - ShapeConstruct: the package statics, `Curve`, `ProjectCurveOnSurface` and `MakeTriangulation`, each with twins of
    its in-place methods.
  - ShapeAnalysis's rest: `FreeBounds`, `FreeBoundData`, `FreeBoundsProperties`, `Shell`, `ShapeContents`,
    `ShapeTolerance`, `CheckSmallFace`, `WireVertex`, `Geom`, `BoxBndTree`, `CanonicalRecognition`.
  - `Geom2dAPI_Interpolate` and `Geom2dAPI_PointsToBSpline`.
  - What these classes hand out is frozen; they only read the shapes they are given, or copy before writing.
- `OcctNet.ModelingData`: the copy at the entry of in-place healing, sewing and clean-up (internal; decision 583).
  - It works on an exact copy of a frozen input and gives back the caller's sub-shapes wherever nothing changed.
  - It re-keys the algorithm's reshaping context and histories onto the caller's shapes.
- Draw-lite's `tolerance` command runs the ported ShapeAnalysis_ShapeTolerance.
- Tests:
  - OCCT's GTests: ShapeBuild_ReShape, ShapeConstruct_ProjectCurveOnSurface, ShapeAnalysis_FreeBounds,
    ShapeAnalysis_CanonicalRecognition, Geom2dAPI_Interpolate and Geom2dAPI_PointsToBSpline. One (ShapeFix_Shape's stability)
    waits for unit 6H3.
  - Unit tests of every class, most on frozen inputs checked unchanged; the copy-at-entry pattern on a toy healing
    algorithm.
  - Oracle tests of ShapeAnalysis_Shell and ShapeAnalysis_FreeBounds against native OCCT on the corpus's shells.
    On 1,570 shapes, 89,646 values and shapes are all identical.
Phase 5 (part): hatching and the API remainder (unit 5H).

- `OcctNet.ModelingAlgorithms`: the Hatch package (Hatch_Hatcher: lines trimmed by oriented 2D segments and lines,
  read back as parameter intervals); HatchGen (points on hatchings and elements, domains, statuses); Geom2dHatch
  (Geom2dHatch_Hatcher, which trims 2D curves by a boundary of 2D curves and computes their domains inside it, with
  its intersector, elements, hatchings and classifier), the hatcher that IntTools_Context and BOPTools use;
  GeomAPI_PointsToBSplineSurface (least squares with each parametrisation, interpolation, periodic forms, the
  variational method, Z grids), whose surface is handed out frozen. GCE2d's deprecated aliases are listed in the GC
  makers they name.
- Tests: OCCT's Geom2dHatch_Elements, Geom2dHatch_Intersector and GeomAPI_PointsToBSplineSurface GTests; unit tests of
  both hatchers on squares, holes, vertices and circles, of the classifier, of HatchGen's points and domains and of
  the surface approximation's forms; oracle tests of GeomAPI_PointsToBSplineSurface against native OCCT on 700
  random grids, every method and continuity: 653 surfaces, all 378,088 values bit-identical.
Phase 5 (part): analytic intersections (unit 5B).

- `OcctNet.ModelingAlgorithms`: IntPatch_ImpImpIntersection, the intersection of two bounded planes, cylinders, cones,
  spheres or tori: conics (IntPatch_GLine), analytic curves (IntPatch_ALine), restrictions lying on the other surface
  (IntPatch_RLine), isolated points and tangent faces, with the coaxial, parallel, tangent and through-the-apex cases
  and OCCT 7.x's cylinder/cylinder walking lines; the points of the domains' boundaries are put on the lines with
  their transitions. `GetStatus` tells a failure from an intersection curve too long to compute
  (`IntStatus_InfiniteSectionCurve`).
- Tests: unit tests of every quadric pair it treats apart and of OCCT's lowalgos/intss bug cases (buc60815, bug20766,
  bug21750, bug23163, bug23177, bug23178, bug24648, bug25782, bug26894, bug28222, bug29807), each line checked to lie
  on both surfaces; an oracle against native GeomAPI_IntSS on 504 random analytic pairs: all 375 conic lines are
  bit-identical to the port's.
Phase 5 (part): approximating intersection lines (unit 5F).

- `OcctNet.ModelingAlgorithms`: ApproxInt, the approximation of a walking line by a B-spline curve in 3D and on the
  surfaces' parameter spaces (`ApproxInt_Approx`, the SvSurfaces that add points on quadric/parametric and
  parametric/parametric pairs, the MultiLine and its tool, `ApproxInt_KnotTools`), as generic classes over the
  surface, its tool and the line; `GeomInt_WLApprox`, its instantiation over surface adaptors and `IntPatch_WLine`,
  used by GeomInt_IntSS and IntTools_FaceFace; BRepApprox (`BRepApprox_Approx` over faces, `BRepApprox_ApproxLine`,
  `BRepApprox_SurfaceTool`); and both packages' instantiations of AppParCurves', Approx's, ApproxInt's and IntImp's
  generics under OCCT's names.
- Tests: GeomInt_WLApprox, BRepApprox_Approx and ApproxInt_KnotTools on walking lines recorded from native
  IntPatch_Intersection on nine surface pairs, against native OCCT's approximations of the same lines: all 128 (degree,
  knots, multiplicities, poles, tolerances reached) bit-identical; the MultiLine, ApproxLine and the OCCT issues
  reproduced natively.
Phase 5 (part): walking (unit 5C).

- `OcctNet.ModelingAlgorithms`: IntWalk, all of it: `IntWalk_IWalking` (the implicit/parametric walking, one generic
  class over the function and the line it builds), `IntWalk_PWalking` (the parametric/parametric walking),
  `IntWalk_TheInt2S`, the walking data and the deflection status; IntPatch's instantiations `IntPatch_TheIWalking`
  and `IntPatch_TheIWLineOfTheIWalking`; `IntPatch_ImpPrmIntersection`, the intersection of a quadric (plane,
  cylinder, sphere, cone) with any parametric surface, and `IntPatch_SpecialPoints`, which it uses at seams and poles.
  Serial, and the same line every run.
- Tests: the walking step by step against native OCCT: every step's status and point and the resulting lines, bit for
  bit, recorded by `tools/walkrec` (OCCT's own walking sources built with MSVC against build123d.NET's native OCCT) for
  nine cases: a cylinder, a sphere, a cone and planes against B-spline patches and a plane against a torus through
  IntPatch_ImpPrmIntersection, and a cylinder, a torus and a patch against a patch through IntWalk_PWalking; the same
  lines serially and in parallel on every run. Unit tests of PWalking (planes, plane and cylinder, boundaries),
  ImpPrmIntersection (every quadric against patches and an extrusion: points on both surfaces, vertices) and
  SpecialPoints (poles, apexes, isolines, periods).

Phase 6 (part): ShapeFix, edges and wires (unit 6H2).

- `OcctNet.ModelingAlgorithms`:
  - `ShapeFix_Root`; `ShapeFix_Wire` (every fix: reorder, small edges, connection, edge curves, degenerated edges,
    self-intersection, lacking edges, closure, gaps in 3D and 2D, seams, shifted pcurves, notches, tails), with
    `ShapeFix_IntersectionTool` and `ShapeFix_SplitTool`; `ShapeFix_Edge`, `ShapeFix_EdgeProjAux`;
    `ShapeFix_ShapeTolerance`; ShapeFix's package statics (`SameParameter`, `EncodeRegularity`, `FixVertexPosition`,
    `LeastEdgeSize`); `ShapeFix_WireVertex`, `ShapeFix_WireSegment`, `ShapeFix_SplitCommonVertex`,
    `ShapeFix_EdgeConnect`.
  - ShapeFix_Wire and ShapeFix_WireVertex given a frozen wire fix a copy of it: their public results give back the
    caller's own edges and vertices wherever nothing changed, and the context answers for the caller's shapes.
  - The in-place tools have public functional twins: `ShapeFix_Edge.AddPCurveFixed` and the other `...Fixed`,
    `ShapeFix_ShapeTolerance.ToleranceLimited`/`WithTolerance`, `ShapeFix.SameParametered`/`RegularityEncoded`/
    `VertexPositionFixed`, `ShapeFix_EdgeConnect.Built`.
- Tests:
  - Unit tests of every class, ShapeFix_Wire's fixes one by one on frozen wires, and ShapeFix_Wire on the wires of the
    heal group's inputs with their faces.
  - Oracle tests of native `ShapeFix_Shape` on wires and edges (the way build123d.NET's `MakeFace` fixes its wires):
    random sketch wires with gaps, tiny, shuffled and reversed edges, and the wires and edges of the heal group's and
    data/occ's shapes. All 14,346 results are byte-identical to native OCCT's.
Phase 5 (part): parametric intersections (unit 5D).

- `OcctNet.ModelingAlgorithms`: `IntPatch_PrmPrmIntersection`, the intersection of two parametric surfaces (B-spline,
  Bézier, offset, swept and the rest): start points from the interference of the surfaces' polyhedra, from
  IntPolyh's triangulations, from a voxel search on dense surfaces, from given points or a start point, then walking
  lines (IntWalk_PWalking) with their vertices on the restrictions; also the self-intersection of a surface. With
  `IntPatch_Polyhedron`, `IntPatch_PolyhedronTool`, `IntPatch_PolyhedronBVH`, `IntPatch_BVHTraversal`,
  `IntPatch_InterferencePolyhedron` and `IntPatch_PrmPrmIntersection_T3Bits`. Serial, and the same lines every run.
- Tests: OCCT's GTests IntPatch_Polyhedron and IntPatch_PolyhedronBVH; seven intersections step by step against
  native OCCT, recorded by `tools/walkrec` with OCCT's own IntPatch_PrmPrmIntersection.cxx (B-spline pairs through
  IntPolyh, polyhedra and the voxel search, an offset surface, a start point, a list of points on a closed surface,
  a self-intersection): every walking step, interference point and line bit-identical; lines on both surfaces, the
  same serially and in parallel.

Phase 6 (part): context and BOP tools (unit 6B).

- `OcctNet.ModelingAlgorithms`:
  - `IntTools_Context` complete: the per-face and per-edge caches of point classifiers, projectors, solid
    classifiers, hatchers, surface adaptors and localization data, and the computations on them (`ComputePE`,
    `ComputeVE`, `ComputeVF`, `IsPointInFace`, `IsValidPointForFace`, `IsValidBlockForFace(s)`, `IsVertexOnLine`,
    ...). Its cached tools were audited for state carried between calls.
  - `IntTools_FClass2d` (the 2D point classifier of a face), `IntTools_TopolTool`, the rest of `IntTools_Tools`;
    `GeomInt.AdjustPeriodic`.
  - BOPTools: `BOPTools_AlgoTools` (states, orientation of splits, faces off an edge, connexity blocks, pcurves,
    edges and vertices, tolerance corrections, dimensions), `AlgoTools2D` (pcurves on faces and their periodic
    adjustment), `AlgoTools3D` (points in faces through the hatcher, normals, seams), `Set`, `ConnexityBlock`,
    `CoupleOfShape`.
  - BOPTools' tolerance corrections run serially whatever their parallel flag says, because native's parallel result
    depends on thread timing.
- Tests:
  - Unit tests of every class: known answers on primitives and data/occ shapes, a reused context answering as fresh
    ones, writes into frozen shapes throwing, and serial = parallel through BOPTools_Parallel.
  - Oracle tests against native OCCT:
    - IntTools_FClass2d on 3,072 faces: 338,880 comparisons, all identical.
    - `ComputeState` against BRepClass3d_SolidClassifier: 4,968 points, all identical.
    - The context's surface and curve projectors: 66,160 values, all bit-identical.

Phase 6 (part): ShapeFix, faces to shapes (unit 6H3).

- `OcctNet.ModelingAlgorithms`:
  - `ShapeFix_Face` (wires, orientation, natural bounds, missing seams, small-area wires, loop and intersecting wires,
    split faces, periodic degenerated edges) with `ShapeFix_ComposeShell`; `ShapeFix_Shell`, `ShapeFix_Solid`
    (`SolidFromShell`), `ShapeFix_Shape`, `ShapeFix_Wireframe` (wire gaps, small edges); `ShapeFix_FixSmallFace`,
    `ShapeFix_FixSmallSolid`, `ShapeFix_FaceConnect`, `ShapeFix_FreeBounds`; `ShapeFix.RemoveSmallEdges`.
  - Given frozen shapes, each fixes a copy: the public results give back the caller's own sub-shapes wherever nothing
    changed (a shape needing nothing is the input itself), are frozen, and the context answers for the caller's shapes.
- Tests:
  - OCCT's ShapeFix_Face and ShapeFix_Shape GTests and ShapeBuild_ReShape's ShapeFix_Shape stability test; unit tests
    of every class on frozen inputs, with the heal group's `drop_small_solids` cases against their reference files.
  - Oracle tests of native `ShapeFix_Shape`, `ShapeFix_Solid::SolidFromShell`, `ShapeFix_Face` and
    `ShapeFix_Wireframe` on the heal group's and data/occ's shapes and on faces and wires as build123d.NET makes them:
    all 26,975 results equal native OCCT's, 26,577 byte-identical and the rest with only the Free and Checked cache
    columns masked.
Phase 5 (part): surface/surface intersection, the milestone (unit 5G).

- `OcctNet.ModelingAlgorithms`: `GeomAPI_IntSS` and `GeomInt_IntSS`, the intersection of two surfaces of any kind as
  approximated curves (conics where exact) with their pcurves; `IntPatch_Intersection`, which dispatches to the
  analytic, quadric/parametric and parametric intersections, with `IntPatch_ALineToWLine`, `IntPatch_WLineTool` (purging,
  joining and extending walking lines) and `IntPatch_LineConstructor`; `GeomInt_LineConstructor`, `GeomInt_LineTool`,
  `GeomInt_ParameterAndOrientation`. Returned curves are frozen. Serial, and the same lines every run.
- Draw-lite: `intersect` (surfaces, and curve/surface), `xdistcs`, `checkoverlapedges`; 56 corpus cases that use
  `intersect` become runnable (31 OK, 8 BAD on OCCT's TODOs, 16 lacking data, 1 IMPROVEMENT from Tcl's variable order).
- Tests: OCCT's GTest GeomAPI_IntSS; unit tests of every class. Oracle: native GeomAPI_IntSS on 1,023 pairs (analytic
  at random and special placements, every kind against revolution, extrusion, Bezier, B-spline, offset and trimmed
  surfaces, the lowalgos/intss corpus' surfaces, data/occ faces): 2,038 lines, every value bit-identical; serial =
  parallel = repeat.
Phase 6 (part): edge intersectors (unit 6C).

- `OcctNet.ModelingAlgorithms`:
  - `IntTools_EdgeEdge` (common parts of two edges: line/line in closed form, otherwise by box refinement and
    coincidence checks), `IntTools_EdgeFace` (common parts of an edge and a face: vertices with touch detection, or the
    whole edge) and `IntTools_BeanFaceIntersector` (the parameter ranges of a curve on a surface, with the sample-box
    localisation on B-spline and Bezier surfaces).
  - The intersector's surface boxes cached in `IntTools_Context` are keyed by what they are computed from, so an
    edge/face run gives the same result whatever its context computed before, and parallel runs give the serial result.
- Tests:
  - Native recordings (tools/edgerec, a C++ recorder built against native OCCT): 2,013 edge/edge and 3,375 edge/face
    runs on data/occ shapes and probe edges, compared bit for bit; all identical but 47 runs on arcs of arbitrary axis,
    which agree within 5.2e-9 (sine and cosine).
  - A reused context gives fresh-context results in any order; serial = parallel through BOPTools_Parallel with edges
    of different tolerances on the same faces; known answers for lines, circles, planes, cylinders and spheres.

Phase 6 (part): face/face intersection (unit 6D).

- `OcctNet.ModelingAlgorithms`: `IntTools_FaceFace`, the intersection of two faces as the Boolean operations need it
  (curves with their 2D curves on both faces, their tolerances and tangential tolerances, isolated points, tangent
  faces), with its own criteria (the faces' tolerances and the fuzzy value), the plane/plane shortcut and the
  approximation flags; `IntTools_WLineTool` (degenerated zones, the decomposition of walking lines at periodic
  boundaries). `Lines()` hands out frozen curves; the faces are only read.
- `tools/ffrec`: records native IntTools_FaceFace (linked against build123d.NET's OCCT) on face pairs.
- Tests: OCCT's GTest IntTools_FaceFace; unit tests of the API and of OCCT's behaviours the port keeps. Against native
  IntTools_FaceFace (recorded): 531 face pairs of primitives, free-form sheets, special placements and data/occ shapes
  with the Boolean operations' and other parameters, 580 lines and 2 points, every value bit-identical; serial =
  parallel = repeat. Oracle: the analytic lines against native GeomAPI_IntSS where the parameters agree, 170 lines
  bit-identical.

Phase 6 (part): UnifySameDomain (unit 6U).

- `OcctNet.ModelingAlgorithms`: `ShapeUpgrade_UnifySameDomain` (build123d.NET's `Clean`): unification of faces on the
  same surface (planes, elementary surfaces through IntPatch_ImpImpIntersection, cylinders made as swept surfaces),
  with seams and periodic surfaces, internal and kept edges and vertices, and of edges on the same line, circle or
  (ConcatBSplines) C1 B-splines; `AllowInternalEdges`, `KeepShape(s)`, `SetSafeInputMode`, the tolerances and the
  history.
  - Given a frozen shape it works on a copy, as native OCCT works on its input even in the safe input mode: the result
    gives back the caller's own sub-shapes wherever nothing changed (a shape needing nothing is the input itself), is
    frozen, and the history answers for the caller's sub-shapes. `InternalShape()` and `InternalHistory()` serve
    ported callers.
- Draw-lite (tests, not packaged): `unifysamedom` with all its options.
- Tests:
  - OCCT's ShapeUpgrade_UnifySameDomain GTests; unit tests on frozen and unfrozen inputs.
  - Oracle tests against native OCCT on native Boolean results (boxes, cylinders, cones, spheres and tori, coplanar,
    coaxial and at random), as build123d.NET's Clean and with eight other option sets, and on the heal group's and
    data/occ's shapes: all 3,455 results equal native OCCT's, 3,410 byte-identical and 33 with the Free column masked
    (12 the same exception); histories equal through the canonical match.
  - The corpus: 51 cases that use `unifysamedom` are now runnable (none before): OK 14, SKIPPED 37 for data files.
Phase 5 (part): analytic constraints (unit 5K1).

- `OcctNet.ModelingAlgorithms`:
  - GccEnt: qualified lines and circles (`GccEnt_QualifiedLin`, `GccEnt_QualifiedCirc`, immutable values), the
    qualifiers and their names.
  - GccInt: the bisecting curves (`GccInt_Bisec` and its line, circle, ellipse, hyperbola, parabola and point kinds).
  - GccAna: circles tangent to three lines, circles or points (`GccAna_Circ2d3Tan`), to two with the centre on a line
    or circle (`Circ2d2TanOn`), to two with a given radius (`Circ2d2TanRad`), to one with the centre on a curve and a
    given radius (`Circ2dTanOnRad`), with a given centre (`Circ2dTanCen`); lines tangent to two circles or points
    (`Lin2d2Tan`), at an angle to a line (`Lin2dTanObl`), parallel (`Lin2dTanPar`) or perpendicular (`Lin2dTanPer`);
    the bisectors of points, lines and circles. Computed by their constructors, with OCCT's accessors.
- Tests: unit tests of every solver and bisector on OCCT's documented configurations (solution counts, qualifier
  filtering, tangency and pass-through of every solution). Native recordings (tools/gccrec, a C++ recorder built
  against native OCCT): every constructor of every GccAna class on 6,618 random and hand-picked configurations, every
  result bit-identical to native, exceptions included, except the results OCCT leaves uninitialised.
Phase 6 (part): the pave filler (unit 6E).

- `OcctNet.ModelingAlgorithms`: `BOPAlgo_PaveFiller`, the intersection part of the Boolean operations: the
  vertex/vertex, vertex/edge, edge/edge, vertex/face, edge/face and face/face interferences with their parallel loops,
  the pave blocks, common blocks and same-domain vertices, the section curves and edges, the split edges, the p-curves,
  the degenerated edges and the self-interference warnings, filling `BOPDS_DS`. Locked (so frozen) arguments make it
  non-destructive by OCCT's own check: published shapes are never written. The members of `BOPAlgo_Tools` it uses.
  The Boolean API's stub builders now report `BOPAlgo_AlertBuilderFailed` for interfering arguments until unit 6F.
- `OcctNet.Foundation`: `NCollection_IntMap`, OCCT's `NCollection_Map<int>` iterating in OCCT's bucket order;
  `NCollection_IntDataMap` gains `Bound`, `TryBound`, `Keys` and an enumerator. BOPDS's and the filler's integer maps
  use them, which native parity needs.
- `tools/pfrec`: records native BOPAlgo_PaveFiller (linked against build123d.NET's OCCT) as a dump of the data
  structure.
- Tests: OCCT's GTests of BOPAlgo_PaveFiller (the filler's part; the Fuse's own checks wait for unit 6F). Against
  native (recorded): 457 cases (the boolean group's simple scripts, some with a fuzzy value; primitives at random
  placements; data/occ shapes), each destructive and non-destructive, 914 of 914 runs bit-identical. Serial =
  parallel on every case; frozen and non-destructive arguments are never written.

Phase 5 (part): faces from wires (unit 5O1).

- `OcctNet.ModelingAlgorithms`:
  - BRepAlgo: `BRepAlgo_FaceRestrictor` (the faces closed wires bound on a face's surface, with or without orientation
    correction), `BRepAlgo_Loop`, `BRepAlgo_Image`, `BRepAlgo_AsDes`, and `BRepAlgo`'s ConvertWire, ConvertFace,
    ConcatenateWire, ConcatenateWireC0, IsValid and IsTopologicallyValid.
  - The face maker the restrictor uses, `TopOpeBRepBuild_WireToFace`, with the part of the legacy Booleans it runs:
    TopOpeBRepBuild's area and loop builders, classifiers, shape sets and face builder; `Builder.MakeFaces`; the
    TopOpeBRepDS_BuildTool and TopOpeBRepTool members they reach (pcurves, shape classifiers, tangents). The Boolean
    algorithm itself is not ported.
  - Given frozen wires, the restrictor and WireToFace work on copies where OCCT writes into them (closed flags,
    pcurves) and give the caller's own wires back where nothing changed; the faces they hand out are frozen.
- Tools: `tools/frrec`, the native recorder of the tests.
- Tests: against native OCCT (recorded): 602 face restrictor and WireToFace cases (nested, disjoint, crossing and
  wrongly oriented wires of lines, arcs, circles and B-splines on three planes, four modes, random sets) and 30 cases of
  the other BRepAlgo tools, all byte-identical; repeatable, serial = parallel, the caller's wires unchanged.

Phase 5 (part): geometric constraints (unit 5K2).

- `OcctNet.ModelingAlgorithms`: Geom2dGcc, all of it. Qualified curves (`Geom2dGcc_QualifiedCurve`, an immutable
  value over its own copy of the adaptor) and the seven solvers: circles tangent to three curves or points
  (`Geom2dGcc_Circ2d3Tan`), to two with the centre on a curve (`Circ2d2TanOn`), to two with a given radius
  (`Circ2d2TanRad`), to one with the centre on a curve and a given radius (`Circ2dTanOnRad`), with a given centre
  (`Circ2dTanCen`); lines tangent to two curves or a curve and a point (`Lin2d2Tan`), at an angle to a line
  (`Lin2dTanObl`). Lines and circles go to GccAna; other curves to the Geo and Iter solvers, which are public too,
  with CurveTool, QCurve and the tangency functions.
- Draw-lite: `cirtang` and `lintan`.
- Tests: OCCT's Geom2dGcc GTests (Circ2d2TanRad, Circ2d3Tan, Lin2d2Tan) and unit tests of every solver; oracle tests
  of the seven solvers against native OCCT on lines, circles, ellipses, B-splines, offset and trimmed curves (36,288 of
  38,000 cases identical token for token, the rest where native reads uninitialised memory or its build skips a range
  check); the corpus's 2dgcc cases (11 runnable, 10 OK, 1 missing its data).
Phase 5 (part): laws and trihedrons (unit 5L1).

- `OcctNet.ModelingAlgorithms`:
  - The Law package: evolution laws (`Law_Constant`, `Law_Linear`, `Law_S`, `Law_Interpol`, `Law_BSpFunc`,
    `Law_Composite`), `Law_BSpline` (a one-dimensional B-spline, immutable to callers with `With`/past-participle
    twins), `Law_Interpolate`, `Law_BSplineKnotSplitting` and the package functions (`Law.MixBnd`, `MixTgt`,
    `Reparametrize`, `Scale`, `ScaleCub`, `BSpline`).
  - GeomFill's trihedron laws for sweeping: `GeomFill_Frenet`, `CorrectedFrenet` (with `EvaluateBestMode`), `Darboux`,
    `ConstantBiNormal`, `Fixed`, `DiscreteTrihedron`, `DraftTrihedron`, `GuideTrihedronAC`, `GuideTrihedronPlan`; the
    location laws `GeomFill_CurveAndTrihedron`, `LocationGuide` and `LocationDraft` with their functions; the section
    law base with `GeomFill_UniformSection` and `GeomFill_SectionPlacement`; `GeomFill_TgtField`.
  - Laws are working objects, as the adaptors: they keep the caller's curves and laws and never change them; a
    `Law_BSpline` a law keeps is frozen.
- Tests: OCCT's GTests GeomFill_CorrectedFrenet and GeomFill_GuideTrihedronAC; unit tests of every law. Oracle: native
  GeomFill_Frenet and GeomFill_CorrectedFrenet on random spines of every curve kind and on planar and twisted B-splines
  (frames and their first and second derivatives, intervals, sub-intervals, copies, the best mode): 2,082,140 values,
  all bit-identical (offsets of free-form curves left out of CorrectedFrenet's runs: OCCT's own sampling is very slow on
  them).

Phase 5 (part): the medial axis (unit 5M).

- `OcctNet.ModelingAlgorithms`:
  - Bisector: the bisecting curves of two lines, circles, conics, B-splines or points (`Bisector_Bisec`, with the
    analytic `Bisector_BisecAna`, point/curve `Bisector_BisecPC` and curve/curve `Bisector_BisecCC` kinds), their
    intersections (`Bisector_Inter`) and the functions and polygons behind them.
  - MAT: the bisecting locus's graph (`MAT_Graph`, arcs, nodes, basic elements), MAT's lists, edges, zones.
  - MAT2d: the locus computation (`MAT2d_Mat2d`, `Tool2d`, `Circuit`, `MiniPath`, `Connexion`, `CutCurve`, `BiInt`).
  - BRepMAT2d: the bisecting locus of a planar face or open wire (`BRepMAT2d_BisectingLocus`, `Explorer`,
    `LinkTopoBilo`), which the 2D wire offset will use. The caller's face is never written.
- `OcctNet.ModelingData`: `Geom2d_TrimmedCurve.ChangeBasisCurve` (internal).
- Tests: native recordings (tools/bisrec, tools/matrec, C++ recorders built against native OCCT): 1,123 bisector and
  bisector-intersection cases and 420 bisecting loci of polygons, arcs, Bézier sides, circles, holed faces and open
  wires (both sides, both join types, open result on and off), every result bit-identical to native, exceptions
  included (16,523 and 5,432 cases checked locally); unit tests on the bisectors, the loci's graphs and MAT's lists.
- Draw-lite: `bisec`; `topoload`, `drawcont`, `side`, `mat`, `result`, `zone`. No corpus case uses `bisec`; the 8
  that use `topoload` still need phase 7's offset commands.
Phase 6 (part): builders and Booleans (unit 6F).

- `OcctNet.ModelingAlgorithms`: the building part of the Boolean operations: `BOPAlgo_Builder` (the General Fuse:
  images of vertices, edges, faces, solids and containers, same-domain faces, internal parts, the history),
  `BOPAlgo_BuilderFace` and `BOPAlgo_WireSplitter` (faces from split edges), `BOPAlgo_BuilderSolid` and
  `BOPAlgo_ShellSplitter` (solids from split faces), `BOPAlgo_BuilderArea`, `BOPAlgo_WireEdgeSet`, `BOPAlgo_BOP`
  (FUSE, COMMON, CUT, CUT21), `BOPAlgo_Section`, `BOPAlgo_Splitter`, and the rest of `BOPAlgo_Tools` (EdgesToWires,
  WiresToFaces, ClassifyFaces, FillInternals). They replace unit 6A's stub builders: `BRepAlgoAPI_Fuse`, `Cut`,
  `Common`, `Section`, `Splitter`, `BuilderAlgo` and the functional twins now give real results, frozen, with
  histories; frozen arguments are never written.
- Tests: OCCT's GTests of BOPAlgo_BOP, and the PaveFiller GTests' result checks; unit tests of every builder. Against
  native OCCT (live, through build123d.NET's bindings): Fuse, Cut, Cut21, Common, Section, Splitter and the General
  Fuse on the boolean group's simple grids, data/occ's shapes, random primitive pairs and triples (with glue, OBB and
  fuzzy values) and near-coincident primitives over a sweep of fuzzy values: 1,853 of 1,853 results byte-identical
  to native, histories equal, serial = parallel on every case.
Phase 5 (part): sections and sweep surfaces (unit 5L2).

- `OcctNet.ModelingAlgorithms`:
  - GeomFill's sweeps: `GeomFill_Sweep` (elementary surfaces where it can, else a B-spline surface approximated by
    `Approx_SweepApproximation`) over `GeomFill_SweepFunction`; the section laws `GeomFill_EvolvedSection` and
    `GeomFill_NSections`; `GeomFill_Pipe` (pipes of constant radius, of a constant section with each trihedron, with a
    fixed binormal, along a curve on a surface, with guides, through two or N sections, and pipes on edges);
    `GeomFill_CircularBlendFunc`, `GeomFill_QuasiAngularConvertor`, `GeomFill_PolynomialConvertor`,
    `GeomFill_SweepSectionGenerator` and `GeomFill_AppSweep`; the package functions of `GeomFill` (ruled surfaces,
    circular sections for blends).
  - Surfaces and curves the sweeps hand out are frozen; GeomFill_NSections leaves the caller's sections unchanged
    (OCCT reparametrises them in place).
- Draw-lite: `sweep` and `tuyau`.
- Tests: OCCT's GTest GeomFill_NSections; unit tests of every class, with frozen inputs. Native recordings
  (tools/sweeprec, a C++ recorder built against native OCCT): 710 cases of pipes, sweeps, section laws, blend functions,
  section generators and package functions, 109,805 doubles and the poles of 474 surfaces, all bit-identical.
Phase 5 (part): surfaces from curves (unit 5L3).

- `OcctNet.ModelingAlgorithms`:
  - Surfaces through sections: `GeomFill_Profiler` (a common B-spline profile), `GeomFill_Generator` (the ruled
    surface through the sections), `GeomFill_SectionGenerator`, `GeomFill_Line` and `GeomFill_AppSurf` (the
    approximated surface: Bezier pieces, B-spline least squares, variational smoothing, pieces of NbMaxP sections),
    over AppBlend: `AppBlend_Approx` and the generic `AppBlend_AppSurf<TheSectionGenerator, TheLine, ThePoint>`.
  - Fillings: `GeomFill_Coons`, `Curved`, `Stretch`, `GeomFill_BSplineCurves` and `GeomFill_BezierCurves` (two, three
    or four curves, three styles), and the constrained filling `GeomFill_ConstrainedFilling` with its boundaries
    (`GeomFill_SimpleBound`, `BoundWithSurf`, `DegeneratedBound`), `GeomFill_CoonsAlgPatch`, `GeomFill_TgtOnCoons`
    and `GeomFill_CornerState`.
  - Gordon surfaces (new in OCCT 8): `GeomFill_Gordon` and `GeomFill_NetworkSurface` over a network of profiles and
    guides, with a parallel contact search that gives the serial result.
  - Surfaces are handed out frozen; the caller's curves are never changed.
- Draw-lite: `appsurf` and `fillcurves`; the four corpus cases they make runnable pass.
- Tests: OCCT's GTests GeomFill_BSplineCurves and GeomFill_Gordon; unit tests of every class. Native recordings (no
  bindings exist): the fillings (tools/fillrec, 880 cases, 568,389 doubles), Gordon and NetworkSurface
  (tools/gordonrec, 380 cases) and the surfaces through sections (tools/appsurfrec, 400 cases), all bit-identical
  but 10 two-section smoothings where native and the port throw different exceptions past an AppDef bug.

Phase 5 (part): 2D fillets and chamfers (unit 5R).

- `OcctNet.ModelingAlgorithms`: ChFi2d, all of it (`ChFi2d_Builder`, the analytical and iterative fillet algorithms,
  `ChFi2d_FilletAPI`, `ChFi2d_ChamferAPI`), and `BRepFilletAPI_MakeFillet2d`: fillets and chamfers on the vertices of
  planar faces and between pairs of edges. Faces given are never written; results, fillets and chamfers are frozen;
  `Modified` answers for the caller's edges.
- Draw-lite: `chfi2d`, `fillet2d`, `chamfer2d`; the corpus scan knows branches on a case's variables.
- Tools: `tools/chfi2drec`, the native recorder of ChFi2d_FilletAPI and ChamferAPI.
- Tests: unit tests (OCCT has no GTests here); 1,200 recorded FilletAPI/ChamferAPI cases, every token identical to
  native; oracle tests of MakeFillet2d on 520 faces (random polygons and arc profiles, build123d.NET's rounded
  rectangle and chamfer patterns), every result byte-identical and every history identical; the corpus's fillet2d cases
  (11 runnable, 10 OK, 1 missing its data).

Phase 5 (part): fair curves (unit 5Q).

- `OcctNet.ModelingAlgorithms`: the FairCurve package: `FairCurve_Batten` (curves simulating physical battens through
  two points with tangency constraints, free or imposed sliding, height and slope) and `FairCurve_MinimalVariation`
  (minimal variation of curvature, with curvature constraints and a physical ratio), working objects whose `Curve()`
  returns a new, frozen 2D B-spline; the energies (`FairCurve_EnergyOfBatten`, `FairCurve_EnergyOfMVC`), the tension,
  sagging and jerk distributions, `FairCurve_BattenLaw`, `FairCurve_Newton` and `FairCurve_AnalysisCode`.
- Tests: native recordings (tools/fairrec, a C++ recorder built against native OCCT): 750 cases of battens, minimal
  variation curves, Draw's command sequences and the energies' values, gradients and hessians, every result
  bit-identical to native, exceptions included; unit tests (OCCT has no FairCurve GTests).
- Draw-lite: `battencurve`, `minvarcurve`, `setpoint`, `setangle`, `setslide`, `freeangle`, `freeslide`, `setheight`,
  `setslope`, `setcurvature`, `freecurvature`, `setphysicalratio`. No corpus case uses them.
Phase 5 (part): plates and filling (unit 5P).

- `OcctNet.ModelingAlgorithms`:
  - Plate (the thin-plate solver `Plate_Plate` and its pinpoint, linear, G1 to G3 and free constraints), NLPlate
    (non-linear plates over a surface), LocalAnalysis (curve and surface continuity analyses).
  - GeomPlate: the plate surface builder `GeomPlate_BuildPlateSurface` with point and curve constraints, the plate
    surface `GeomPlate_Surface` (geometry, immutable to callers, with `WithBounds`), its B-spline approximation
    `GeomPlate_MakeApprox` with the G0 and G1 criteria, and `GeomPlate_BuildAveragePlane`.
  - N-sided filling: `BRepFill_Filling` with its constraint classes, and the maker `BRepOffsetAPI_MakeFilling`
    (build123d.NET's `Face.MakeSurface` and `MakeSurfacePatch`). The face is new topology; the caller's edges and faces
    are never written; `Shape()` and `Generated` publish frozen shapes; a built filling can take more constraints and be
    built again, as build123d.NET does.
  - A solved plate and the plate surface over it evaluate from several threads at once (OCCT's evaluation cache
    dropped; same values).
- Draw-lite: `plate`, `gplate`, `approxplate`, `filling`, `fillingparam`.
- Tests: OCCT's GTests Plate_Plate and GeomPlate_BuildPlateSurface; unit tests of every class. Oracle: MakeFilling
  against native OCCT on build123d.NET's MakeSurface (random contours of lines, arcs and B-splines, rebuilt with
  points), holes in boxes with G1 and G2 supports, cylinder caps, points, points on faces, free constraints and an
  initial surface: all 206 builds byte-identical as BRep text, every Generated list identical, every value
  bit-identical but where native reads uninitialised memory. Corpus: the filling cases (6 runnable, from none; 1 OK, 1
  known problem as OCCT, 3 missing their data, 1 failing as native OCCT 8 does).

Phase 6 (part): BRepAlgoAPI and the public API, the Boolean milestone (unit 6P).

- `OcctNet.ModelingAlgorithms`:
  - `BRepAlgoAPI_BuilderAlgo.SimplifyResult` (and so every Boolean operation's): UnifySameDomain on the result, its
    history merged into the operation's; the arguments are never written, whatever the non-destructive flag.
  - The functional twins (`BRepAlgoAPI.Fuse`, `Common`, `Cut`, `Section`, `Split`): `BRepAlgoAPI_Options.Simplify`
    now works, and each takes an optional `Message_ProgressRange`; a cancelled operation throws
    `OperationCanceledException` over the `BRepAlgoAPI_Failure` that carries OCCT's `BOPAlgo_AlertUserBreak`.
  - `BOPAlgo_PaveFiller` no longer fails when it would write a frozen argument vertex's own tolerance back into it (a
    shape cut again by one of its arguments).
- Tests: OCCT's GTests of BRepAlgoAPI_Fuse, Cut, Common, BuilderAlgo (TKBO) and Cut, Fuse, Section (TKBool), 232
  tests, 7 skipped for fillets (phase 7); unit tests of the simplification, progress and cancellation. Against native
  OCCT: SimplifyResult on random primitives and on the boolean group's and data/occ's arguments, native's destructive
  mode, the inverted-solid check, and the corpus's `bfuzzyvalue`/`bglue` cases (589 comparisons: 513 byte-identical,
  72 where native's destructive mode differs and its non-destructive result is the port's, 4 canonically identical where
  native varies); the determinism test (1,728 repeated and parallel runs at degrees 2 to the processor count, all equal
  to the serial run).

Phase 6 (part): checks and the other builders (unit 6G).

- `OcctNet.ModelingAlgorithms`: BOPAlgo_CheckerSI, ArgumentAnalyzer and CheckResult (self-interference and argument
  checks) with `BRepAlgoAPI_Check`; BOPAlgo_MakerVolume (solids from faces); BOPAlgo_CellsBuilder (cells, materials,
  internal boundaries); BOPAlgo_MakePeriodic and BOPAlgo_MakeConnected (periodic and connected shapes, repetitions,
  twins, materials); BOPAlgo_RemoveFeatures with `BRepAlgoAPI_Defeaturing`. Inputs are never written (a frozen input is
  copied where OCCT writes into it); results, histories and check results hand out frozen shapes. `BOPDS_DS`'s
  interference table iterates in OCCT's order.
- Draw-lite: `bopcheck`, `bopargcheck`, `bopapicheck`, `xdistef`, `checkcurveonsurf`, `mkvolume`, `bcbuild`, `bcadd`,
  `bcaddall`, `bcremove`, `bcremoveall`, `bcremoveint`, `bcmakecontainers`, `makeperiodic`, `periodictwins`,
  `repeatshape`, `clearrepetitions`, `makeconnected`, `cmaterialson`, `cmakeperiodic`, `crepeatshape`,
  `cperiodictwins`, `cclearrepetitions`, `removefeatures`; the Boolean session (`baddobjects`, `baddtools`, `bclear*`,
  `bfillds`, the option commands) and the history commands (`setfillhistory`, `savehistory`, `modified`, `generated`,
  `isdeleted`); `unifysamedom` records its history.
- Tests: unit tests of every class. Against native OCCT (tools/borec, on the corpus's inputs captured by Draw-lite and
  on random primitives and data/occ shapes): checks 488 of 488 identical; volumes and cells 797 of 801 byte-identical,
  4 with the Free flag masked; periodic and connected shapes 488 of 499 byte-identical, 11 with Free masked; feature
  removal 220 of 225 byte-identical, 5 the same error; parallel runs equal to serial ones. Corpus: of the cases using these commands, none
  runnable before; now check commands 97 (OK 84, 1 known problem, 12 missing data), `mkvolume`/`bc*` 152 (OK 142, 1
  known problem, 9 missing data), periodicity and connection 9 (OK 7, 2 missing data), `removefeatures` 42 (OK 25, 17
  missing data); none failing.

Phase 5 (part): sweep laws and sections (unit 5S1).

- `OcctNet.ModelingAlgorithms`: BRepFill's package statics (`Face`, `Shell`: build123d's ruled surfaces; `Axe`,
  `ComputeACR`, `InsertACR`), the location laws (LocationLaw, ACRLaw, Edge3DLaw, EdgeOnSurfLaw), the section laws
  (SectionLaw, ShapeLaw, NSections), Section, SectionPlacement, CompatibleWires, Pipe, TransitionStyle, TypeOfContact,
  ThruSectionErrorStatus; ShapeUpgrade_RemoveLocations; BRepOffsetAPI_MakePipe. Frozen inputs are never written
  (copies where OCCT writes into them). BRepFill_Pipe and MakePipe run BRepFill_Sweep, which unit 5S2 ports: until
  then they throw Standard_NotImplemented through a placeholder.
- Draw-lite: `pipe` (its 58 runnable corpus cases wait for 5S2).
- Tests: unit tests of every class. Against native OCCT: `BRepFill.Face`, `Shell`, `Axe`, `ComputeACR` and `InsertACR`
  through build123d.NET's bindings, 1,900 cases all identical (byte-identical BRep text; 23 Axe cases throwing on both
  sides); the laws, sections, placement and RemoveLocations (tools/brepfillrec) 460 cases all identical;
  CompatibleWires (tools/cwrec) 385 of 390 identical, the other 5 looping sections where native varies from run to run.

Phase 5 (part): the 2D wire offset (unit 5O2).

- `OcctNet.ModelingAlgorithms`: `BRepOffsetAPI_MakeOffset` (the offset of a planar face's or planar wires' contours in
  their plane, arc or intersection joins, closed or open results, `ConvertFace` and the approximation option, as
  build123d's `offset_2d` and slots use it) over BRepFill_OffsetWire, TrimEdgeTool and OffsetAncestors;
  `BRepOffsetAPI_MakeEvolved` over BRepFill_Evolved, TrimSurfaceTool, MultiLine, ComputeCLine and ApproxSeewing (walls
  and faces of horizontal and vertical profile parts; slanted parts wait for unit 5S2's BRepFill_Sweep, the volume mode
  for phase 7). The caller's shapes are never written; results hand out frozen shapes.
- Fixed: line/circle intersections (IntCurve_IntConicConic) differed from OCCT's in the last bits on about 4% of
  inputs; they are now bit-identical.
- Draw-lite: `mkoffset`, `openoffset`, `evolved`; `topoload -approx`. DrawCorpus knows the offset group's `command`
  branches. Corpus: of the 752 cases using `mkoffset` or `openoffset`, none runnable before, 626 now (621 of the offset
  group's wire grids): OK 477, 77 known problems, 72 missing data, none failing. `evolved`: 2 of 29 runnable (missing
  data); 22 wait only for `lastrep`.
- Tests: unit tests of the offset and evolved classes; against native OCCT through build123d.NET's MakeOffset (random
  closed and open profiles with segments and arcs in random planes, holed faces, both joins and signs, build123d.NET's
  `Offset2D` and slot patterns, approximated ellipses and B-splines, `ConvertFace`): 1,228 offsets, every result
  byte-identical (104 not done on both sides), the generated lists the same (74 in another order: native's follow
  addresses); MakeEvolved's values against tools/evorec.

Phase 5 (part): pipe shells (unit 5S2).

- `OcctNet.ModelingAlgorithms`: `BRepOffsetAPI_MakePipeShell` over BRepFill_PipeShell (one or more sections or a
  section with a scaling law, every trihedron mode, transformed, right or round corners, shells and solids,
  `Simulate`, the history), BRepFill_Sweep and BRepFill_TrimShellCorner; `BRepOffsetAPI_MiddlePath`. With the sweep in
  place, `BRepOffsetAPI_MakePipe` and `BRepOffsetAPI_MakeEvolved`'s slanted profiles now build. The caller's profiles
  and spine are never written; results hand out frozen shapes.
- Fixed in the port, against OCCT: a swept planar face no longer reverses the plane of a face already built.
- Draw-lite: `pipe` runs; its 58 runnable corpus cases: OK 53, one known problem, 4 missing data, none failing.
  `evolved` builds slanted profiles.
- Tests: OCCT's BRepFill_PipeShell and BRepOffsetAPI_MakePipeShell GTests; against native OCCT through build123d.NET's
  MakePipeShell, 522 random cases (every mode and transition, solids, several and punctual sections, build123d.NET's
  sweeps): every built result, first and last shape and history byte-identical; MakePipe and MiddlePath through
  tools/piperec, 270 cases all identical; MakeEvolved's slanted profiles against tools/evorec.

Phase 5 (part): lofts and projection (unit 5T).

- `OcctNet.ModelingAlgorithms`: `BRepOffsetAPI_ThruSections` (ruled and smoothed lofts, shells and solids, point end
  sections, CompatibleWires on or off, smoothing, weights, parametrisation, continuity, maximal degree, the history),
  BRepFill_Generator and `BRepProj_Projection` (cylindrical and conical). The caller's sections are never written:
  they are copied at Build where OCCT writes into them, and their unchanged sub-shapes are given back in the result.
- Tests: OCCT's BRepOffsetAPI_ThruSections GTests and the two loft GTests of BOPAlgo_PaveFiller; against native OCCT
  through build123d.NET, 630 random lofts and 248 cases of OCCT's thrusection corpus (every built result, first and
  last shape and history byte-identical; lofts whose first section is also the last vary natively from run to run),
  266 projections (every wire identical, one canonically through decision 577's non-destructive Booleans).

Phase 6 (part): build123d.NET's Booleans (unit 6M, the phase 6 milestone).

- Tests: build123d.NET's own tests' operations replayed against native OCCT, their operands rebuilt with
  build123d.NET's public API.
  - 244 Booleans (fuse, cut, common, section, split, with build123d.NET's options) and 253 cleans
    (UnifySameDomain): all 497 are byte-identical with native, histories included. Native was repeatable on every
    one, and the port's parallel runs equal its serial ones.
  - 179 construction cases (MakePipeShell, ThruSections, MakeFilling, BRepFill, BRepProj_Projection, the Geom2dGcc
    solvers, MakeFillet2d, MakeOffset, PointsToBSplineSurface): 177 are byte-identical, 2 give the same error on both
    sides, and 1 is identical once the port's operands are unfrozen.
- Benchmarks: a Booleans suite (`-- booleans`, `-- booleans-profile`) over build123d.NET's Booleans and the corpus's
  simple and complex grids. Serially the port takes 0.93× to 1.04× native's time (geometric mean 0.83×), within the
  2× target. The clean takes 0.64× to 0.98×. The parallel speed-up on these small Booleans is below native's.
Phase 6 (part): Draw-lite Booleans and healing, the phase 6 corpus milestone (unit 6L).

- Draw-lite: the rest of BOPTest (`bop`, `bopcommon`, `bopfuse`, `bopcut`, `boptuc`, `bopsection`, `bcommon`, `bfuse`,
  `bcut`, `btuc`, `bsection`, `bopcurves`, `bbuild`, `bbop`, `bsplit`, `buildbop`, `bapibuild`, `bapibop`,
  `bapisplit`, `bsimplify`, `edgestofaces`, `edgestowire`, `attachpcurve`, `buildpcurvesonplane`, `btolx`,
  `breducetolerance`, `bopaddpcs` and the data-structure debug commands), `sewing`, `fastsewing`, and SWDRAW's
  ShapeAnalysis, ShapeFix, ShapeTool and ShapeUpgrade commands whose algorithms are ported (`fixshape`, `fixwgaps`,
  `fixsmall`, `fixsmallfaces`, `settolerance`, `freebounds`, `fbclose`, `statshape`, `getanasurf`, `removeloc`,
  `reshape`, ...). Commands that OCCT runs in place work on a copy of Draw's frozen variables and rebind the variables
  that share a changed sub-shape; Booleans of Draw variables are non-destructive. `unifysamedom`'s history now also
  answers for the rebound shapes.
- Corpus: 4,385 cases using the Boolean commands are runnable (621 before): OK 3,687, BAD 84 (OCCT's TODOs), 612
  missing data, 2 failing for attributed reasons (one through DRAWEXE's destructive default, one that native OCCT
  fails too); 903 using the healing commands (none before): OK 602, BAD 12, 289 missing data, none failing. The heal
  group's split and convert grids wait for phase 7's ShapeUpgrade and ShapeCustom.
Phase 5 (part): Draw-lite construction, the phase 5 corpus milestone (unit 5W).

- Draw-lite: the lofts (`thrusections`, `gener`, `pruled`), the pipe-shell commands (`mksweep`, `setsweep`,
  `addsweep`, `deletesweep`, `buildsweep`, `errorsweep`, `simulsweep`), `middlepath`, `geompipe`, the projections
  (`prj`, `cprj`), the continuity commands (`surfaceCcontinuity`, `surfaceGcontinuity`, `curveCcontinuity`,
  `curveGcontinuity`, `getcurvcontinuity`, `getsurfcontinuity`), `ruled`, `partuyau`, the qualified curves (`qcircle`,
  `qline`, `circ2d3Tan`), `interpol` (from a file) and `tanginterpol`. `pipe`, `thrusections` and the sweeps record
  their history for `savehistory`. The corpus scan can count a case's procedures only where they are called.
- Corpus: 1,085 cases use the new commands; none runnable before, 938 after: OK 569, BAD 48 (OCCT's own TODOs),
  321 without their data files; none failing. With the earlier units, 1,807 cases using phase 5's construction
  commands run.
- Tests: the commands on hand-made shapes and curves, their inputs left unchanged.

Phases 5 and 6 complete (integration).

- Oracle tests: the loft and projection oracles' native reruns read the same bytes as the port, so a rerun can't loft
  inputs one ulp apart (decision 944); native calls that reach `AppDef_Variational` are serialised, because OCCT's
  FEmTool criteria share unlocked caches between threads (decision 945, occt-issues.md); the sewing oracle sews
  natively until it meets the port's variant, up to 16 times (decision 946).
- The full suite: 10,008 tests passing, 9 skipped.

Phase 7 (part): ShapeCustom, the healing pilot (unit 7A).

- `OcctNet.ModelingAlgorithms`: ShapeCustom whole: the package functions (`ApplyModifier`, `DirectFaces`,
  `ScaleShape`, `BSplineRestriction`, `ConvertToRevolution`, `SweptToElementary`, `ConvertToBSpline`), the
  modifications (DirectModification, ConvertToRevolution, SweptToElementary, ConvertToBSpline, BSplineRestriction with
  its RestrictionParameters, TrsfModification), and the geometry tools ShapeCustom_Curve, Curve2d and Surface.
  SHMessage: Shape Healing's message texts compiled in, which `ShapeExtend.Init` loads through
  `Message_MsgFile.LoadFromString` (decision 8103).
- The caller's shape is never written: the modifier rebuilds what changes and shares the rest, the results are frozen
  once, at the end, and the caller's curves and surfaces that OCCT changes in place are copied first (decisions 8101,
  8102). The rule for the later healing units is decision 8101's.
- Tests: frozen-input unit tests; oracle tests of the six package functions against build123d.NET's bindings on
  primitives, Boolean results, swept and mirrored shapes, the heal group's inputs and data/occ (5,919 runs, all
  identical); `tools/shcustomrec` records BSplineRestriction with other restriction parameters and the geometry
  tools on 556 inputs (8,878 records, all identical).
Phase 7 (part): Draw-lite's cheap gaps (unit 7W).

- Draw-lite: Draw's `dversion`, `meminfo`, `lastrep`, `dmode`, `discr`, `defle`, `XProgress`, `removeinternals` and
  `repaint` (a viewer no-op); the geometry commands `proj`, `to2d`, `to3d`, `project`, `projonplane`, `crvpoints`,
  `crvtpoints`, `2dproj`, `2dextrema`, `2dintersect`, `2dintanalytical`, `intconcon`, `incdeg`, `insertpole`,
  `cmovepoint`, `convert`, `segsur`, `setweight`; the topology commands `validrange`, `shape`, `subshape`,
  `BRepIntCS`, `xbounds`, `xclassify` and `concatC0wire`. `prism` and `revol` record their history. The corpus scan
  reads the scripts of `repeat` and `do`.
- Fixed: `BRepAlgo.ConcatenateWireC0` threw Standard_ImmutableObject when it reversed a concatenation of several
  pieces.
- Corpus: 512 more cases runnable (10,617 of 21,068): OK 337, BAD 18 (OCCT's TODOs), 138 missing data, 19 failing for
  attributed reasons (progress output, the viewer's mesh, phase 7's evolved volume mode). The four cases the new scan
  default exposed are fixed (bug29586) or attributed (bug23881 and bug29333_2: DRAWEXE's destructive Booleans;
  bug26034: native OCCT built with exceptions fails it too).

- Corpus (unit 7W's follow-up): the Boolean corpus test run whole: 5,530 runnable, OK 4,095, BAD 129, 1,292 missing
  data, 8 failing, all attributed (bugs/modalg_6/bug26393: DRAWEXE's destructive Booleans; bugs/modalg_7/bug83: the
  port's section is about 6x slower than native's on two B-spline sheets).
Phase 7 (part): helix (unit 7D).

- TKHelix, new in OCCT 8: HelixGeom (HelixCurve, the analytical helix adaptor; Tools, the B-spline approximation;
  BuilderApproxCurve, BuilderHelixGen, BuilderHelixCoil, BuilderHelix) and HelixBRep_BuilderHelix (composite helix and
  spiral wires by heights, pitches or numbers of turns). The builders' curves and wires are frozen when handed out;
  the smoothing between parts changes copies of the edges' curves (decision 8502).
- Draw-lite: `setaxis`, `comphelix`, `helix`, `spiral`, `comphelix2`, `helix2`, `spiral2`; `vzfit` as a viewer no-op.
- Tests: TKHelix's six GTests; `tools/helixrec` records native TKHelix on the helix grid's parameters (56 cases,
  all byte-identical, BRep text included). Corpus: the 56 helix cases are runnable: OK 54, BAD 2 (OCCT's own TODO).

Phase 7 (part): the rest of ShapeUpgrade, and ShapeAlgo (unit 7B).

- ShapeUpgrade: the divide tools (ShapeDivide and its Angle, Area, Closed, ClosedEdges and Continuity variants;
  FaceDivide, FaceDivideArea, ClosedFaceDivide, WireDivide, EdgeDivide, ClosedEdgeDivide), the split tools (2D and 3D
  curves, surfaces, with their Continuity, Angle and Area variants), the Bezier converters (curves, surfaces,
  ShapeConvertToBezier), FixSmallCurves, FixSmallBezierCurves, RemoveInternalWires, ShellSewing and
  C0BSplineToSequenceOfC1BSplineCurve. ShapeAlgo: its process-wide AlgoContainer and the ToolContainer.
- The caller's shape is never written: the dividers copy a frozen shape at the entry and give back its unchanged
  sub-shapes, and the geometry OCCT changes in place is changed on copies (decisions 8401, 8402).
- Tests: OCCT's FaceDivide GTest, frozen-input unit tests; `tools/shupgraderec` records twelve tools with the heal
  group's Draw parameters on 527 inputs (6,324 records: 6,323 identical, 1 attributed to OCCT writing into the
  caller's surface).
Phase 7 (part): the offset foundations (unit 7E).

- BRepOffset but BRepOffset_MakeOffset: Analyse (edge types: convex, concave, tangent, mixed; the tangent faces'
  treatment), Tool, Inter2d, Inter3d, Offset (offset faces, pipes and spheres), Interval, MakeLoops, SimpleOffset,
  MakeSimpleOffset, the package (`Surface`, `CollapseSingularities`) and the enums. Also ChFi3d's package functions
  (DefineConnectType, IsTangentFaces, ConcaveSide, NextSide, SameSide) with ChFiDS_TypeOfConcavity, and
  TopOpeBRepTool_TOOL's `Nt`.
- Copy at entry for the offset family (decision 8601): the makers copy a frozen input before OCCT's in-place writes,
  so `Locked(false)` and `Free(true)` act on the copy, and they give back the caller's unchanged sub-shapes.
  MakeSimpleOffset's solid mode and BRepOffset_Offset's pipes and spheres never write into a caller's shape.
- Tests: `tools/offsetrec` records native Analyse, Offset and MakeSimpleOffset on 40 primitives, Boolean and fillet
  results and data/occ shapes: 1,984 records, all byte-identical (two are the same exception).

Phase 7 (part): LocOpe (unit 7H).

- LocOpe, TKFeat's local operations: SplitShape, WiresOnShape, Spliter, BuildWires, BuildShape, Generator,
  GeneratedShape, GluedShape, Gluer, SplitDrafts, Prism, DPrism, Revol, RevolutionForm, Pipe, LinearForm, FindEdges,
  FindEdgesInFace, CSIntersector, CurveShapeIntersector, the package functions, PntFace and Operation.
- The caller's shapes are never written: each operation works on copies of the frozen shapes it is given, all sharing
  their copies, and gives back the caller's sub-shapes it left unchanged (decisions 8901, 8902). Results are frozen;
  BRepFeat (unit 7I) reads the Internal forms.
- Tests: unit tests per class on frozen inputs; `tools/locoperec` records native LocOpe on 119 inputs (primitives,
  data/occ's shapes, the feat group's data files): 5,201 records, all identical (DPrism's shapes compared canonically:
  native varies run to run).
- Fix: BRepFill_OffsetWire down-casts its spine's plane as OCCT does (decision 8906).

Phase 7 (part): ShapeProcess and the healing Draw commands (unit 7C).

- ShapeProcess (the operator registry and performer, Context, ShapeContext, Operator, UOperator, the operator library
  of eighteen operators), ShapeProcessAPI_ApplySequence and TKernel's Resource_Manager. OCCT's default resource files
  (XSTEPResource's IGES and STEP) are compiled in; other files are read only from directories the caller gives, never
  through CSF_ environment variables (decision 8703). The operators run on a copy of a frozen shape and give back its
  unchanged sub-shapes, the history keyed by the caller's sub-shapes (decision 8702).
- Draw-lite: SWDRAW's ShapeUpgrade commands (`DT_ShapeDivide`, `DT_SplitAngle`, `DT_ShapeConvert`,
  `DT_ShapeConvertRev`, `DT_SplitCurve2d`, `DT_SplitCurve`, `DT_SplitSurface`, `offset2dcurve`, `offsetcurve`,
  `splitface`, `DT_ToBspl`, `DT_ClosedSplit`, `DT_SplitByArea`, `DT_SplitByNumber`, `RemoveIntWires`), ShapeCustom
  commands (`directfaces`, `expshape`, `scaleshape`, `bsplres`, `convtorevol`) and ShapeProcess commands (`SPApply`,
  `DT_ApplySeq`).
- Tests: Resource_Manager's GTests; `tools/shprocrec` records native ShapeProcess (OCCT's and the heal group's
  sequences, each operator alone) on 527 inputs: 13,702 records, 13,673 identical, 29 attributed to OCCT's hash-ordered
  maps. Corpus: 2,607 newly runnable cases (OK 1,584, BAD 41, 982 missing data, none failing).
Phase 7 (part): the 3D offset (unit 7F).

- BRepOffset_MakeOffset: the offset shape and the thick solid with the Arc and Intersection joins, the Complete mode's
  splits of the offset faces (BRepOffset_BuildOffsetFaces, planar shapes), thickening with its missing walls, the
  conical faces' correction, the history (`Generated`, `Modified`, `IsDeleted`); BRepOffsetAPI_MakeOffsetShape and
  BRepOffsetAPI_MakeThickSolid (by join and by the simple algorithm).
- The caller's shape is never written: the offset copies a frozen input at the entry, runs OCCT's in-place writes
  (`Locked(false)`, pcurves, tolerances) on the copy and gives back the unchanged sub-shapes; the history answers for
  the caller's sub-shapes (decision 9201).
- Tests: OCCT's GTests (23 cases), frozen-input tests, and the oracle against native OCCT on primitives, Boolean
  results, faces and shells and data/occ's shapes, both joins and signs, with and without intersection, and
  build123d.NET's Hollow, Offset3D and Thicken calls: 724 runs, every result and history identical, canonically equal or
  attributed to native varying from run to run (decision 9206).

Phase 7 (part): the blend functions (unit 7J), the first step to 3D fillets.

- Blend: the function interfaces the walking and the approximation of a blend use (Blend_AppFunction, Function,
  FuncInv, CSFunction, RstRstFunction, SurfRstFunction and the point inverses) and Blend_Point.
- BlendFunc: the functions of constant and evolving radius fillets (ConstRad, EvolRad), chamfers (Chamfer, ConstThroat,
  ConstThroatWithPenetration, ChAsym), ruled blends, and fillets between a surface and a curve (CSConstRad,
  CSCircular), with their inverses on restrictions, and the package's tools.
- ChFiDS: the fillet builders' data: spines (Spine, FilSpine with its radius laws, ChamfSpine, ElSpine), stripes,
  surface data, common points, face interferences and their maps, and the error status build123d.NET reports.
- Tests: `tools/blendrec` records native OCCT's functions on plane, cylinder, cone and B-spline pairs and its spines on
  a box, a cylinder and a B-spline solid: 235 cases, all identical bit for bit; derivatives against finite differences;
  unit tests of ChFiDS's data.
Phase 7 (part): drafts and the rest of TKOffset (unit 7G).

- Draft and BRepOffsetAPI_DraftAngle (taper faces of a shape: planes, cylinders, cones); BRepAlgo_NormalProjection,
  BRepOffsetAPI_NormalProjection and FindContigousEdges; BRepFill_Draft, DraftLaw and BRepOffsetAPI_MakeDraft (draft
  skins); BiTgte_Blend and its curves (the rolling ball); BRepFill_AdvancedEvolved, so BRepOffsetAPI_MakeEvolved's
  volume mode works.
- The caller's shape is never written: the draft angle copies the modifier's result before OCCT's in-place wire
  corrections when it shares the caller's sub-shapes, and the rolling ball and the evolved volume copy a frozen input at
  the entry (decisions 9401, 9403, 9404).
- Draw-lite: the 3D offset commands (`offsetparameter`, `offsetload`, `offsetonface`, `offsetperform`, `offsetshape`,
  `offsetcompshape`, `thickshell`, `mkoffsetshape`, `offsetshapesimple`), the draft commands (`depouille`,
  `ndepouille`, `draft`), `rollingball`/`brollingball`/`trollingball` and `nproject`; a backslash-newline now ends a bare
  Tcl word. 1,687 corpus cases newly runnable (OK 405, BAD 63, 1,216 missing data, 3 failing, attributed).
- Tests: unit and frozen-input tests; the draft angle against native OCCT (870 runs, every result and history identical
  or canonically equal); `tools/draftrec` records MakeDraft, BiTgte_Blend and the evolved volume (181 runs, all
  byte-identical but 4 attributed). Fixed: unit 7F's offset threw reading the history of a run with a null result.
- BRepFeat (unit 7I): the form features (MakePrism, MakeDPrism, MakeRevol, MakePipe), the ribs and grooves
  (MakeLinearForm, MakeRevolutionForm), MakeCylindricalHole, SplitShape, Gluer, Builder; the caller's shapes are never
  written (the features work on copies and give back the unchanged sub-shapes; decision 9501). Draw-lite's feature
  commands (`featprism`, `featperform`, `splitshape`, `hole`...): 280 more corpus cases pass. Oracle: DPrism and
  SplitShape against native OCCT, the other makers against a native recording (tools/featrec). A Boolean argument
  holding frozen sub-shapes now runs non-destructively (decision 9505).

Phase 7 (part): the walking of blends (unit 7K, BRepBlend), the second step to 3D fillets.

- The walkers: BRepBlend_Walking (a blend between two surfaces, stopping on the faces' restrictions and correcting the
  corners), BRepBlend_CSWalking (a curve and a surface), BRepBlend_SurfRstLineBuilder and RstRstLineBuilder (a surface
  and a restriction, two restrictions, with the losses of contact), their line, extremities and points on arcs.
- The surface/restriction and restriction/restriction functions of constant and evolving radius and the inverse
  functions at the walkings' ends; the curve and blend tools.
- The approximation's side: BRepBlend_AppFuncRoot with AppFunc, AppFuncRst and AppFuncRstRst (the walking line as a
  sweep function), BRepBlend_AppSurface and BRepBlend_AppSurf.
- Tests: `tools/blendrec` now records native OCCT's walking step by step on planes, a plane and a cylinder, B-splines,
  a curve and a plane, and restrictions, with the lines' approximations: 112 cases (45 of them walk), all identical
  bit for bit; unit tests of the line, extremities, tools and approximation function.

Phase 7 (part): the legacy data structure's core (unit 7L), which 3D fillets and chamfers write their results into.

- TopOpeBRepDS: the data structure and its handle, points, curves, surfaces, the interferences (curve/point,
  surface/curve, solid/surface, shape/shape, edge/vertex, face/edge), transitions, the iterators and explorers, the
  shape data and the package's functions; TopOpeBRepTool_ShapeTool. Ported whole for what ChFi3d, ChFiKPart and the
  reconstruction of unit 7R need; the legacy builder's filters and reducers are left out (decision 992).
- The data structure is its own context: nothing in it is process-wide.
- `tools/dsrec` records the data structure native fillets and chamfers hand to the legacy builder (15 cases: boxes,
  cylinders, cones, a sphere, a torus, a Boolean result, chamfers), in a text format the port writes too
  (`TopOpeBRepDS_TextDump`), so that the fillet units can compare theirs with native's byte for byte. Every recording
  reads into the port's data structure and writes back as native's.
Phase 7 (part): build123d.NET parity, its first part, and Quantity_Color (unit 7X).

- Quantity_Color and Quantity_ColorRGBA (colours: the 509 named colours, RGB, sRGB, HLS, CIE Lab and Lch, hex strings),
  the last classes build123d.NET calls that the port lacked outside phase 7 and STEP. Identical bit for bit with native
  OCCT on 14,326 records.
- build123d.NET's healing, offset, draft and feature calls (Hollow, Offset3D and the Offset operation, Thicken,
  Face and Shell Offset, Draft, ExtrudeTaper's draft, DPrism, SplitByPerimeter, ToSplines) replayed on the port from its
  tests' inputs, with their histories, failures and build123d.NET's finishing steps: 93 calls, none different (85
  identical or canonically identical where native varies, 2 attributed to native's run-to-run variation, the same
  failure on both sides in the others).
- Benchmarks: `-- phase7` times those calls against native OCCT: 1.56× native serial in all (geometric mean 1.09×);
  offsets and drafts within 2×; DPrism, SplitByPerimeter, ToSplines and three small hollows above it (decision 10304).

Phase 7 (part): the foundations of hidden-line removal (unit 7P).

- Intrv (intervals with tolerances and their Boolean operations), TopBas, TopCnx.
- HLRAlgo: the projector (parallel and perspective), the edge status and its iterator, the min-max boxes, the edge and
  wire blocks, interferences, and the polygonal algorithm's data (bi-points, triangles, hiding data, the internal
  triangulation, shells) and PolyAlgo.
- Contap: the contours (silhouettes) of a surface for a direction, a draft angle or an eye: in closed form on planes,
  spheres, cylinders and cones, otherwise by the boundary search, the interior search and the walking (phase 5's
  generics).
- Fix: `Standard.Epsilon` of the largest reals is 0, as OCCT's.
- Tests: `tools/hlrrec` records native OCCT's projectors, intervals, edge statuses, min-max boxes and contours (60 on
  quadrics, tori, a plane and a B-spline surface): 85 cases, all identical bit for bit; unit tests of the projector,
  the edge status and the contours.

Phase 7 (part): the fillets' construction data (unit 7N1), the first of the fillet builders.

- ChFiKPart: the fillets and chamfers computed analytically, written into the legacy data structure as native OCCT
  writes them: a cylinder between two planes, a torus or sphere between a plane and a cylinder or cone, the chamfers
  (symmetric, two distances, distance and angle, constant throat with or without penetration) on the same pairs, and
  the toric, spherical and rotule corners.
- ChFi3d_Builder_0, whole: the builders' free functions (pcurves and their same parameter, the edges between fillets,
  the intersections of fillets and faces, the elementary spines' curves, writing a stripe into the data structure, the
  topology helpers).
- Tests: tools/dsrec records native OCCT's analytic cases on real faces and Builder_0's functions (compiled from OCCT's
  own source): 66 cases, all identical byte for byte. The fillets write only the builder's working copy of their
  input (unit 7N2 copies it at entry).

Phase 7 (part): the data model of hidden-line removal (unit 7Q1).

- HLRTopoBRep: the outlining of a shape for a projector (its faces' contours as internal outlines and outlines, their
  isolines, its edges split at their vertices), with its data.
- HLRBRep's data model: the edge and face data, the data structure the hider walks (projection, min-max boxes, faces'
  flags and orientations of outlines, interferences of an edge with a face's boundary, classification by a line of
  sight), the loading of a shape, the shape bounds, the interference lists, vertex lists and area limits.
- HLRBRep's projected curves and surfaces with their tools and local properties, and the 2D curve intersection and
  line/surface intersection it uses (phase 5's generics instantiated).
- The outliner copies a frozen shape at the entry: OCCT writes into its input's vertices.
- Tests: `tools/hlrrec` records native OCCT's outlining, data structure and hider's walk on five primitives in eight
  projectors and on the hlr group's shapes: 52 cases, all identical bit for bit; unit tests of the curves, surfaces,
  data, merging and the copy at the entry.
Phase 7 (part): follow-ups on the differences from native (unit 7Y).

- BRepOffset_Tool's `EnLargeFace` puts the enlarged offset surface on the offset's own working face, as OCCT changes it
  in place: OCC5805.brep's thick solid (bugs/modalg_2/bug5805_3 and _4) is one shell of seven faces as natively; the
  offset oracle gains three byte-identical results.
- BRepFill_Filling (BRepOffsetAPI_MakeFilling) accepts a caller's edge on the face swept from it (build123d.NET's
  MakeSurfacePatch threw NullReferenceException); results as native's.
- Public `TopoDS_Builder.MakeWire`, `MakeShell`, `MakeSolid`, `MakeCompSolid` (frozen results, as `MakeCompound`) and a
  `BRepTools_History` copy constructor, for build123d.NET.
- Oracle and corpus: the DPrism oracle reads native's input again for each run (6 differences were native's writes into
  its input); memory-trend corpus cases run once more when they fail; MakeDraft's four and DPrism's eleven tapered fuses
  stay attributed, with their causes narrowed. A stress of build123d.NET's native binding from many threads reproduces
  the gate's native-only access violation (decision 10409).

Phase 7 (part): the fillets' reconstruction on the Boolean engine (unit 7R, option C).

- `ChFi3d_BOPReconstruction` replaces TopOpeBRepBuild_HBuilder for the fillet builders: from the data structure the
  fillets fill, it builds the edges of the curves, the fillet faces, the input's faces and edges cut (concave fillets
  growing a face, holes kept or dropped, a fillet reaching a cone's apex), and the shells and solids, with
  `BOPAlgo_BuilderFace`; it answers `IsSplit`, `Splits`, `Merged`, `NewFaces` and `NewEdges` as ChFi3d asks.
  `ChFi3d_Reconstruction.Create` gives it to the builders. It writes only into the shapes it makes.
- Tests: `tools/dsrec --recon` records native OCCT's data structure, result and history for 120 fillets and chamfers
  (blend/simple A to H, blend/complex's shapes in the dataset, build123d.NET's, shells, voids, compounds, corners of
  four faces). Compared geometrically (volumes, areas, counts, validity, boxes, histories as sets): 114 equal, 3
  attributed to native (two invalid native results, Bezier pieces made B-splines), 3 not comparable.

Phase 7 (part): the fillet builder (unit 7N2), up to the topological reconstruction.

- ChFi3d_Builder, whole: the spines and stripes, the walking and approximation of the fillet surfaces, the analytic
  cases and their splitting by the faces' restrictions, the extremities, the corners of one, two, three and more
  fillets, and the writing of everything into the legacy data structure. ChFi3d_FilBuilder (constant and evolving
  radii) and ChFi3d_ChBuilder (one distance, two distances, distance and angle) with their corners.
- The builders work on a copy of a frozen input and never change the caller's shape.
- `Compute` hands its data structure to `ChFi3d_Reconstruction`, the seam unit 7R implements on the Boolean engine;
  until then it stops there with a NotImplementedException.
- Tests: tools/dsrec records 38 native fillets and chamfers (boxes, cylinders, cones, a sphere, a torus, Boolean
  results, a tangent chain, a four-edge corner, concave edges, a shell, build123d.NET's test inputs and failures): the
  data structure at the reconstruction is native's byte for byte in all 35 that reach it, and the faulty contours and
  stripe statuses build123d.NET reads are native's in all.
Phase 7 (part): the hidden-line removal algorithms (unit 7Q2). TKHLR is complete.

- HLRBRep_Algo: the exact hidden-line removal of shapes for a projector (each shape hidden by itself and by the
  others), and HLRBRep_HLRToShape: its visible and hidden sharp, smooth, sewn, outline and isoline edges as compounds,
  in the view plane or in 3D, for all the shapes, one of them or one of its faces. With HLRBRep_InternalAlgo,
  HLRBRep_Hider, HLRBRep_EdgeBuilder and the package's edge builders.
- HLRBRep_PolyAlgo and HLRBRep_PolyHLRToShape: the polygonal hidden-line removal of meshed shapes, faster and
  approximate.
- HLRAppli_ReflectLines: a shape's reflect lines (outlines and edges) for a view, in 3D or 2D.
- The caller's shapes are not written into (the exact algorithm's outlining copies a frozen shape; the polygonal one
  only reads); every compound is frozen.
- Draw-lite: HLRTest's commands (hprj, houtl, hfill, hload, hhide, hres2d, reflectlines, hlrin3d, hlrin2d and the rest),
  build3d, vcomputehlr with the view projections (vfront, vtop, vright, ...), and Draw's view commands as no-ops: 188
  corpus cases run that didn't (OK 44, BAD 5, two attributed, 137 without public data).
- Tests: `tools/hlrrec` records native OCCT's exact and polygonal hidden-line removal and reflect lines on primitives,
  pairs of shapes and the hlr group's shapes in eight views: 133 cases, 1890 compounds, all identical as BRep text
  (50 with the Free column masked); unit tests of the algorithms, frozen inputs and runs on several threads.

Phase 7 (part): the public fillet and chamfer API (unit 7N3).

- BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer and BRepFilletAPI_LocalOperation: fillets with constant and
  evolving radii and chamfers by one distance, two distances or a distance and an angle, with their histories and the
  diagnostics of failed contours, on 7N2's builders and 7R's reconstruction. The caller's shape is never written and
  the result shares its unchanged faces; results and histories are frozen. FilletSurf_Builder (a fillet's surfaces
  and curves without a shape). The functional twins `BRepFilletAPI.Fillet` and `BRepFilletAPI.Chamfer`.
- Tests: OCCT's GTests for MakeFillet and MakeChamfer; against native OCCT, 1,922 fillets and chamfers on primitives,
  Boolean results, data/occ's solids and the GTests' inputs, geometrically native's where both results are valid but
  for 35 attributed to the reconstruction (a face with a seam keeps the band the fillet removes), and build123d.NET's
  MaxFillet probes, all native's.
- Draw-lite: the fillet and chamfer commands (`blend`, `bfuseblend`, `bcutblend`, `mkevol`, `updatevol`,
  `buildevol`, `tolblend`, `continuityblend`, `checkhist`, `chamf`, `chamf_throat`,
  `chamf_throat_with_penetration`) and the feature commands `fillet` and `bossage`; the rolling-ball commands read
  `tolblend`'s tolerance; `encoderegularity` rebinds the variables sharing the shape. 883 corpus cases newly runnable
  (799 with data): OK 723, BAD 34, IMPROVEMENT 8, FAILED 34, all attributed (mostly the reconstruction's invalid
  results).

Phase 7 (part): build123d.NET parity, its second part: fillets and chamfers (unit 7X).

Phase 7 (part): build123d.NET parity for fillets and chamfers (unit 7X, its second part).
- Tests: every 3D fillet, chamfer and MaxFillet build123d.NET's tests make (63 calls: Shape3D.Fillet, Chamfer and
  MaxFillet and the fillet and chamfer operations, with build123d.NET's validity check and failure messages) replayed
  on the port from native's input bytes: results geometrically native's, the faulty contours, exceptions and MaxFillet's
  probes exactly; none different. One difference attributed to the reconstruction: the input solid's own history
  (removed in the port, modified into the result's solid natively). With part 1, every phase 7 call build123d.NET's
  tests make reproduces native (plan milestone 2).
- Benchmarks: the fillets and chamfers join the `-- phase7` suite, within 2× native serial (0.54× to 1.05× by
  operation). Phase 6's parallel Boolean benchmark measured again.

Phase 7 (part): the reconstruction's tail (unit 7R3).

Phase 7 (part): the reconstruction's tail (unit 7R3).
- Fixed: the band a fillet removes from a face whose seam it crosses (box common sphere, cylinder T, OCC1077,
  OCC426); fillets ending at a vertex given as a point; fillet sections overflowing an edge; fillet faces whose loop
  reads as a hole; zero-length curve edges; the input solid's history (`Modified` and `IsDeleted` of the solid, as
  native's).
- 7N3's oracle: 1,252 of 1,255 primitive runs native's (1,220 before), the GTests' inputs 7 of 7, histories of solids
  compared; the fillet corpus: OK 736 (723), FAILED 20 (34), all attributed.

Phase 7 complete (1 November 2026): finishing.
- Healing (ShapeCustom, ShapeUpgrade, ShapeProcess), helix, offsets and drafts, features (LocOpe, BRepFeat), 3D
  fillets and chamfers (ChFi3d on a reconstruction over the Boolean engine, option C, instead of TopOpeBRepBuild), and
  hidden-line removal, each compared with native OCCT: byte for byte where native repeats itself, otherwise
  canonically or geometrically, with every difference attributed.
- build123d.NET's phase 7 calls (healing, offsets, drafts, features, fillets, chamfers, MaxFillet) replayed from its
  tests: none different. Fillets and chamfers run at 0.5 to 1.05 times native's time.
- The Draw corpus: 16,381 of 21,068 cases runnable (10,066 OK); port-status: 48.1% of OCCT's lines (TKFillet,
  TKOffset, TKFeat, TKShHealing over 99%, TKHLR and TKHelix whole).
- A faster merge gate (Release, about 15 minutes) and the native oracle's use-after-free found and fixed (decision
  1109).

Phase 8 (part): STL (unit 8H1).
- Ported: RWStl and RWStl_Reader (ASCII and binary, reading and writing, multi-domain files, node merging by angle),
  StlAPI, StlAPI_Reader and StlAPI_Writer, BRepBuilderAPI_MakeShapeOnMesh, Standard_ReadLineBuffer; a new
  Standard_IStream (C++ stream rules over a .NET stream). Streams first, file paths on top; limits for hostile input
  (RWStl_ReadLimits: bytes, facets); cancellation through Message_ProgressRange; frozen results. DESTL waits for TKDE.
- Oracle (tools/stlrec): data/occ's shapes meshed and written as ASCII and binary STL byte for byte as native (the
  binary header included); every STL file of data/stl, the dataset and the tests' inputs, and 21 derived inputs
  (concatenated, truncated, lying headers), read every way: triangulations bit for bit, StlAPI_Reader's shapes as BRep
  text; 2 lines attributed (native runs out of memory reading text as binary).
- Draw-lite: `readstl`, `writestl`; 67 corpus cases newly runnable (OK 7, SKIPPED 59 for data not public, FAILED 1
  attributed: the progress output).
Phase 8 (part): the STEP parser (unit 8P).
- `OcctNet.DataExchange`: StepFile_ReadData, and a hand-written, reentrant scanner and parser (`StepFile_Scanner`,
  `StepFile_Parser`) in place of OCCT's flex and bison code, filling it as OCCT's do, error recovery and messages
  included. They read a stream, with limits for hostile input (`StepFile_ReadLimits`: file size, records, nesting) and
  cancellation through `Message_ProgressRange`.
- Tests: tools/steprec compiles OCCT's own scanner and parser. All 340 STEP files of OCCT's dataset and data/step, and
  1,329 hostile and mutated inputs, parse as natively, but for two inputs native crashes on. The port parses in 0.74×
  native's time.
Phase 8 (part): XSBase's foundation (unit 8A1).
- Ported: MoniTool (but its timers and case data), Interface (but STAT and the message dictionary), IFGraph, LibCtl's
  library as C# generics, XSMessage's texts, and TCollection_HAsciiString.
- The statics become a parameter object, `Interface_StaticSet`, passed to each read and write; the protocols'
  module lists are initialised once; no other process-wide state changes.
- Compared with native OCCT: 19,400 recorded lines identical (the formatting of reals STEP writes, the line buffer,
  the statics, the message utilities, bit maps, entity lists); the graph, share, check, copy and file-reading tools
  tested on a toy norm. 24 OCCT bugs found, one fixed (an out-of-bounds shift in EntityCluster::Remove).
- IFSelect's member trace for unit 8A2: 41 of 76 classes reached.
Phase 8 (part): StepData and the STEP header (unit 8B).
- Ported: StepData (the reader data with its text decoding, the reader tool, the model, the writer, the dumper,
  protocols, modules and libraries, select types and members, fields, descriptors, undefined and free-form entities,
  factors), StepFile_Read on unit 8P's parser, HeaderSection, RWHeaderSection, APIHeaderSection_MakeHeader,
  UnitsMethods, DESTEP_Parameters, and Resource_Unicode's decoding over .NET's encodings corrected by OCCT's tables.
- Texts are .NET strings, written back as UTF-8; no process-wide state changes (protocols registered once; the
  CasCade length unit read from the parameter set); StepFile_Read reads streams, without a global mutex, with limits
  and cancellation.
- Compared with native OCCT (tools/stepdatarec): the 340 STEP files of the corpus read into a model and written back
  identically (1.5 million texts decoded, 2.7 million lines written); 248 crafted texts decoded under every code page
  (105 decodings attributed: the system locale, reads past a text's end); the code tables equal OCCT's on all 71,374
  entries its conversions reach. One OCCT bug fixed (an endless recursion copying a named select member).
Phase 8 (part): Transfer, TransferBRep and XSAlgo (unit 8A2).
- Ported: Transfer (the transient and finder processes, binders, actors, iterators, dispatch, input and output tools,
  results), TransferBRep (shape binders and mappers, the shape services, the reader), XSAlgo (the shape processor, the
  algorithm container) and TKDE's DE_ShapeFixParameters.
- XSAlgo reads its parameters from the caller's `Interface_StaticSet`; no process-wide state changes. The shape
  processor never changes its input and returns frozen shapes; the reader publishes frozen shapes.
- Compared with native OCCT (tools/xsalgorec) on data/occ's shapes with STEP's default sequences: 210 records, 207
  identical, 3 attributed. Transfer tested on a toy norm. 12 OCCT bugs found, one fixed (an endless recursion in
  Transfer_ActorDispatch).
Phase 8 (part): the STEP entity generator (unit 8C1).
- tools/stepgen generates STEP's entity classes and their readers and writers from OCCT 8's own C++, translating each
  class's header and source to C# under docs/translation-rules.md; deterministic, with a `--check` mode the tests run.
- Generated: StepBasic, StepGeom, StepShape, StepRepr, RWStepBasic, RWStepGeom, RWStepShape and RWStepRepr (809
  files, 58,631 lines); nothing written by hand.
- Round trips of the dataset's 338 STEP files through the generated readers and writers, with a test protocol: of
  1,451,636 records of these packages (93.9% of all), 1,450,960 are written back the same, and the 676 others differ
  as OCCT's own reading and writing make them (each attributed).
Phase 8 (part): STEP geometry (unit 8D).
- Ported: StepToGeom (STEP's points, directions, vectors, placements, transformation operators, curves and surfaces
  into Geom and Geom2d) and GeomToStep (Geom, Geom2d and gp into STEP entities), with the file's units through
  StepData_Factors; StepToGeom's yaw-pitch-roll conversion waits for StepKinematics.
- Compared with native OCCT (tools/geomsteprec), with the default units and with inches and degrees: 4.2 million
  conversions of the corpus's 338 STEP files' geometric entities, and the conversion of 85 constructed cases and of
  data/occ's geometry into STEP entities, written and read back: identical. 8 OCCT bugs found, one fixed (unbounded
  recursion on cyclic references).
Phase 8 (part): the other STEP entity packages and StepAP214's protocol (unit 8C2).
- Generated by tools/stepgen: StepVisual, StepAP214, StepAP203, StepAP242, StepDimTol, StepKinematics, StepFEA,
  StepElement and their RW packages (804 files, 92,276 lines), with StepAP214's protocol, its general module and its
  read-write module (the dispatch over 826 case numbers), all from OCCT 8's C++. Written by hand: 106 lines (the protocol's
  singleton, the libraries' initialisation, the dispatch's static maps). StepAP209_Construct is left to unit 8F.
- `RWStepAP214.Init()` registers the modules once; `StepAP214.Protocol()` is the protocol. Their static tables are
  filled once by C#'s static initialisation; nothing changes afterwards.
- The H-collections are Standard_Transients, as in OCCT (`NCollection_Array1`, `Array2` and `Sequence` derive from
  `Standard_Transient`); `NCollection_Array1` has OCCT's Iterator.
- Round trips of the dataset's 338 STEP files with the real protocol: 98.9% of the 1,546,314 records are recognised
  (the rest are types OCCT doesn't know either); of those, 1,526,805 are written back the same and the 2,107 others
  differ as OCCT's own reading and writing make them (each attributed, 13 OCCT bugs logged).
Phase 8 (part): IFSelect, XSControl and the STEP header's editor (unit 8A3).
- IFSelect as its member trace reaches (40 of its 76 classes: the work session, work library, model copier,
  share-out, modifiers, selections, signatures and counters, dispatches, editors); XSControl but its command files
  (the controller, work session, reader, writer, transfer reader and writer, selections and signatures on transfers,
  Vars, Utils); APIHeaderSection_EditHeader.
- A work session owns its parameters (an `Interface_StaticSet`) or is given one; a controller declares its norm's
  statics in it (`InitStatics`). Controllers are recorded once, process-wide, as the libraries' modules. Reading takes
  .NET streams and a progress range; writing can go to a stream through the share-out's modifiers
  (`XSControl_Writer.WriteStream`). Readers publish frozen shapes.
- Compared with native OCCT (tools/xscontrolrec) on the corpus's 331 STEP files up to 3 MB: the work session's roots,
  selections, signatures, check lists and label lookups, in its two modes, identical (27,589 lines). Tested on the toy
  norm and on STEP models. 11 OCCT bugs logged.
Phase 8 (part): the assembly tree and the fresh mesh formats (unit 8T).
- XdeDoc, new: an immutable assembly tree in place of XCAF (`XdeDoc_Tree`, `XdeDoc_Product`, `XdeDoc_Instance`):
  shared products, placed instances, names, colours with alpha, visibility, layers, validation properties and
  sub-shape attributes; frozen shapes; `Flatten` for meshing and viewing, `Meshed` to mesh every part.
  `XdeDoc_Document` gives it XCAF's label interface (the shape, colour and layer tools' calls) for STEPCAFControl.
- glTF 2.0 (`.gltf` + `.bin`, `.glb`), OBJ (+ MTL) and 3MF written fresh over the tree, with instancing and colours;
  glTF read fresh into the tree, with limits for hostile input. Streams first, cancellation through
  `Message_ProgressRange`, deterministic output.
- Checked against OCCT's de_mesh cases on the dataset's Lantern (native's counts and structure), round trips, and the
  formats' rules in the tests.

Phase 8 (part): STEP topology (unit 8E).
- StepToTopoDS: the translation of STEP's shape representation items (manifold solid breps, breps with voids, faceted
  breps, shell based surface models, wireframe and face based models, geometric sets, tessellated solids, shells,
  faces and surface sets) into BReps, with its translators of vertices, edges, loops, faces and shells, its tools and
  MakeTransformed. `Value()` gives frozen shapes; a reader works on `InternalValue()` and freezes at the end.
- TopoDSToStep: the conversion of BReps into STEP entities (MakeManifoldSolidBrep, MakeBrepWithVoids,
  MakeFacetedBrep, MakeFacetedBrepAndBrepWithVoids, MakeShellBasedSurfaceModel, MakeGeometricCurveSet, the builders
  and the tessellated items). The shapes converted never change.
- Compared with native OCCT (tools/toporec): all 2,194 of the corpus's shape items translated byte for byte, and
  1,726 conversions of data/occ's shapes written identically.
- 8D's tests now read with StepAP214's protocol; the transient process takes checks on null entities as OCCT's does.
Phase 8 (part): the XDE Draw-lite commands over the document (unit 8G1).
- Draw-lite documents: an `XdeDoc_Document` under a Draw variable (`XNewDoc`, `NewDocument`, `Close`), with XDEDRAW's
  shape, colour, layer, validation property and dump commands (`XAddShape`, `XGetOneShape`, `XStat`, `Xdump`,
  `XSetColor`, `XGetAllColors`, `XAddLayer`, `XGetAllLayers`, `XSetProps`, `XDumpAssemblyTree`... about 70) with
  OCCT's output, and the glTF and OBJ commands (`ReadGltf`, `WriteGltf`, `WriteObj` and their shape forms) over
  unit 8T's reader and writers.
- XdeDoc: XCAF's auto-naming (per document), colour labels, removal of shapes, components, colours and layers; a fix
  of `AddShape` for compounds; glTF's per-face primitives (`RWGltf_WriteOptions.MergeFaces`).
- Corpus: 109 cases using the commands become runnable (none before): 63 OK (xcaf's BRep grids, bugs/xde, de_mesh's
  glTF), 37 skipped for data that isn't public, 9 failures attributed. Most of de and xcaf wait for STEP (unit 8G2).
- ShapeFix_EdgeProjAux: when it reverses the parameters of an edge's ends, the edge now holds the reversed pcurve
  they refer to (as OCCT, which reverses it in place), where before it kept the unreversed one (four edges of
  tr9_r1001-ec.stp lost their pcurves on reading).
Phase 8 (part): STEPControl, the first milestone (unit 8F).
- Ported: STEPControl (the controller with STEP's statics, STEPControl_Reader and STEPControl_Writer, the reading and
  writing actors), STEPConstruct's unit contexts, parts, assemblies, AP203 context and context tool, StepSelect's work
  library and step type, STEPEdit, the STEPSelections selections, StepToGeom's yaw-pitch-roll rotation. STEP shapes
  and assemblies are read and written without native code, from and to .NET streams, with limits for hostile input
  and cancellation; readers publish frozen shapes; writers never change their input; a session's write actor is its
  own, so output doesn't depend on what the process wrote before.
- Compared with native OCCT: the corpus's 338 STEP files read with identical shapes (331 byte for byte, 7 but for the
  Free flag), roots and counts, and identical check messages for 337; data/occ's shapes and assemblies written
  identically in every model type (AP214IS, in process) and in 708 of 720 schema and model type cases (AP203,
  AP214IS, AP242DIS, tools/stepctlrec); round trips identical or canonically identical.
- Draw-lite: XSDRAW's session per interpreter with `param`, `data`, `entity`, `xload`, `writeall`, `writeent`,
  `listtypes`, `newmodel`, `tpstat`, `tpdraw`, `tpcompound`, and `stepread`, `stepwrite`, `testreadstep`,
  `testwritestep`, `stepfileunits`, `steptrans`: 274 corpus cases made runnable (44 OK, 224 skipped for missing data,
  4 BAD as in OCCT, 2 failures attributed).
- Phase 8, unit 8G2: STEPCAFControl's reader and writer over the assembly document (`XdeDoc_Document`): assemblies,
  external references, names, colours (surface, curve, per sub-shape, instance, visibility), layers, validation
  properties, the length unit and sub-shape names to and from STEP, from and to streams; the rest of STEPConstruct
  (Styles, RenderingProperties, ValidationProps, ExternRefs, Tool). GD&T, materials, views, metadata and SHUOs are
  not transferred.
- Compared with native XCAF documents (tools/stepcafrec): the corpus's 338 STEP files read into identical documents
  (332; 6 attributed to GD&T and a native misread name); 458 documents written, 377 identical, 70 identical but for
  the order of the style representations, 11 attributed.
- Draw-lite: `ReadStep` and `WriteStep` over its documents; the de group's STEP grids run (999 corpus cases made
  runnable: 255 OK, 76 BAD as in OCCT, 660 skipped for missing data, 8 failures attributed).
- Phase 8, unit 8Z: STEP external references read safely. `STEPCAFControl_Reader` reads the files a STEP file refers
  to through a resolver (`SetExternFileResolver`; `FileSystemResolver`, OCCT's way and the file API's default;
  `FolderResolver(root)`, confined to a folder; or the caller's, from an archive or memory), can be told to read none
  (`SetExternFilesMode(false)`), applies the caller's read limits and cancellation to every external file within a
  budget across them (`STEPCAFControl_ExternFileLimits`: files, bytes, depth), and stops at cycles (a stack overflow in
  OCCT). `ReadStream` reads no external file unless given a resolver. `STEPCAFControl_Writer.WriteStream(stream,
  externStreams)` writes the multifile mode into the caller's streams.
- Phase 8, unit 8W: the data-exchange corpus (the third milestone). Draw-lite gets `ReadFile`, `readfile`,
  `WriteFile` and `writefile` (STEP, STL, glTF and BRep, with `-conf` configurations), `xnorm`, Tcl's encodings
  (`encoding`, `fconfigure -encoding`), `exec` (removing files only) and `binary format`; the mesh commands take the
  session's length unit. Of 3,366 data-exchange corpus cases, 2,174 run: 631 OK, 97 BAD as in OCCT, 1,436 skipped
  for data that isn't public, 10 failures attributed; the 1,192 others need OCAF storage, IGES, VRML, GD&T, notes,
  views, materials, an OBJ reader or the viewer.

Phase 8 complete (2 November 2026): data exchange.
- XSBase (IFSelect as traced), StepData with a hand-written reentrant STEP parser, STEP's entity classes and their
  readers and writers generated from OCCT 8's own C++ (tools/stepgen: 25 packages, about 151,000 lines), StepToGeom,
  GeomToStep, StepToTopoDS, TopoDSToStep, STEPControl and STEPCAFControl over an immutable assembly tree (XdeDoc) in
  place of OCAF and XCAF; STL ported; glTF, OBJ and 3MF written fresh, with a glTF reader. Each compared with native
  OCCT: STEP read and written as native does on the dataset, every difference attributed.
- Streams first, limits for hostile input and cancellation throughout; no disk access from streams unless asked.
- The Draw corpus: 17,872 of 21,068 cases runnable (10,462 OK); 2,174 data-exchange cases run, 10 failures attributed.
  port-status: 62.0% of OCCT's lines.
- build123d.NET and Bento import and export STEP (with names, colours and assemblies), STL, glTF and 3MF on OCCT.NET;
  the first packages built for nuget.org as 0.1.0-preview.1.

Phase 9 (part): Schematiq.Collections (unit 9A).

- Returned collections are Schematiq.Collections' (`ISchematiqList<T>`), from nuget.org, in place of Bento.Collections
  0.4.0: the same library, renamed and published by Bento's owner. `PackedList<T>` is an `ISchematiqList<T>`.

Phase 9 (part): OCCT.NET as build123d.NET's default backend (unit 9D1).

- build123d.NET now builds on OCCT.NET by default, from the packages on nuget.org; native OCCT is opt-in there and
  kept as the oracle's reference. The oracle tests and benchmarks reference build123d.NET's projects on its native
  backend explicitly, and restore by the project graph so that holds for restore too. `Build123dNativeRoot` lets a
  build123d.NET worktree use another checkout's native build.
Phase 9 (part): the differential run (unit 9D2).

- build123d.NET's whole test suite compared op by op on native OCCT and OCCT.NET, through a shape log in build123d.NET
  and `Build123dDifferentialTests` (`tools/b3d-differential.ps1`): of 16,044 shapes compared, 15,118 are identical as
  BRep text (with the Free and Checked flags masked), 160 canonically and 750 in another order or representation; the
  first divergence of each of the 18 other tests is attributed: native varying from run to run (11 tests), backend-
  specific code in build123d.NET (5), OCCT.NET not writing a caller's face (1), a test mutating a canonically identical
  result's text (1). No port bug.
Phase 9 (part): the triage of five corpus failures (unit 9F).

- Fixed: a frozen face with a seam swept with MakeRevol or MakePrism (and MakePipe's profiles) now answers its history
  as native OCCT does; `Generated` threw TopoDS_LockedShape (bugs/modalg_6/bug26406), and `IsDeleted` called a seam
  deleted (bugs/modalg_7/bug30346_2).
- Fixed: BRepOffsetAPI_MakeFilling with frozen boundary edges that meet only within tolerance lost an edge of the
  face (bugs/filling/bug23380); the face is now native's.
- Attributed, not the port: lowalgos/extss/bug29712_44 (OCCT's script), demo/samples/anc101 (OCCT's sample fails
  natively too), bugs/fclasses/bug1281 (Draw's plugin loader).
Phase 9 (part): the performance review (unit 9E).

- docs/performance.md: where build123d.NET's suite and Bento's examples spend their time on OCCT.NET, against native.
  Five fixes that keep native's results: a quadratic in `Intf_TangentZone` (BRepCheck on filleted solids), the eager
  allocation of every `NCollection_DynamicArray`, the Delaunay mesher's queue of deleted links, the cell filter's
  index copies and a temporary in `math_NewtonMinimum`. The suite now takes 1.9× native's time run serially (3.7×
  before); the rest is mostly the garbage collector and the JIT, with proposals for both.
- `benchmarks/OcctNet.Bento.Benchmarks`: Bento's guide examples, built, meshed and measured, on OCCT.NET or native.

Phase 9 (part): ReadyToRun packages (unit 9R).

- The packages' assemblies are compiled ahead of time for win-x64 (ReadyToRun), keeping their IL for other platforms:
  the first use of each operation no longer waits for the JIT.

Phase 9 complete (2 November 2026): the switch-over.
- On nuget.org as 0.1.0-preview.2 and preview.3, over Schematiq.Collections; built in Release and ReadyToRun, checked
  from a clean feed; Bento's 3D models extension uses them.
- OCCT.NET is build123d.NET's default backend; build123d.NET's whole suite gives native's shapes, every difference
  attributed.
- Two port bugs fixed (sweeps' histories, the filling's edges); a performance review halving build123d.NET's time on
  OCCT.NET (docs/performance.md).
