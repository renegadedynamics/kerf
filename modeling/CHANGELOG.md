# Changelog

All notable changes to the Kerf.Modeling package (until 0.2.0 the build123d.NET packages). Versions follow [SemVer](https://semver.org); before
1.0 a minor version may break the API.

## Unreleased

- **`MaxChamfer` and `MaxFillet` give the proved limit, less twice the shape's tolerance, where the limit itself can't be
  built and that can:** 1.6999998 for the guide's rounded box, whose corners close to a point at 1.7, where the search
  gave 1.594, and 9.9999998 for a fillet using up a face beside a slanted end, where it gave 9.944. Where the limit
  builds, or none is proved, the answers are as they were (kernel decision 26501).
- **Text in DXF:** `ExportDxf` writes a drawing's notes (`Kerf.Drawing.DrawingNote`) on a `"notes"` layer, as closed
  polylines of their outlines by default or as TEXT with a text style for each font (`textOutput: TextOutput.Text`;
  R12's text is ASCII, so other notes stay outlines there, with the warning `modeling.dxf-text-as-outlines`), and
  `AddNotes` adds notes alone. A drawing sheet (`Kerf.Drawing.Sheet`) is written as any drawing is; the guide's
  section 8 has one (kernel decisions 2030 to 2033).
- **Breaking:** Kerf.Modeling's unused `PageSize` enum has gone; `Kerf.Drawing.PageSize` has its values, and ANSI C to E.
- **DXF export:** `ExportDxf`, build123d's `ExportDXF`, writes sketches, faces, wires, edges and line drawings to DXF
  for other CAD programs and laser cutters: layers with colours, line types and weights, units in the header, lines,
  arcs, circles, ellipses and splines exactly (rational ones with their weights, where build123d approximates), and the
  hidden lines of a drawing on their own layer. DXF 2013 by default, R12 for older programs (kernel decisions 2009
  to 2011).
- **Breaking: an object that can't be made from its values throws `ModelingException`,** one type carrying the error a
  `Result` would (`Error`, with its code, and `Diagnostics`), where it threw `ArgumentException`,
  `InvalidOperationException` or, from a few, the kernel's own exceptions. **Each object has a `Make…` operation
  returning a `Result`** instead (`Operations.MakeTriangle`, `MakeSpline`, `MakeConstrainedArcs`, …, as
  `MakeConvexPolyhedron` was). New codes: `modeling.no-solution`, `modeling.limit-not-reached` and
  `modeling.object-failed`. `ModelingException` is an `InvalidOperationException`, and the operators' and topological
  methods' failures are one too; code catching `ArgumentException` for an object's geometric failure must catch
  `ModelingException` (kernel decision 2013). An argument wrong on its own (a null, too few points, a missing angle)
  is still an `ArgumentException`.
- **`RectangleRounded` of radius 0 is the rectangle,** where it threw; a negative radius is an `ArgumentException`.
  **A large `Triangle`** (sides of 1e10, whose lengths come out 8e-6 from the solver's) finds its edges, where it
  threw. **`Rectangle` of a size 0** fails as `Box` does, where it returned a face that isn't valid as a success. **A
  closed `FilletPolyline` of two points** fails saying so, where it threw an index error. **`SlotCenterToCenter`
  and `SlotOverall` too short to have a side** fail as `ModelingException`, where the kernel's internal exception
  escaped. **`ConstrainedArcs` and `ConstrainedLines` with no solution** say so (`modeling.no-solution`), where
  they said the selector returned nothing (kernel decision 2013).
- **An operation reports the kernel's refusal of a value as a failure every time,** `kernel.invalid-argument` in context
  ("box 0 × 1 × 1 failed: …"), where after many calls the JIT's optimisation could make one throw `ArgumentException`
  instead (an extrusion along a zero vector, a box of size 0; kernel decision 2014).
- **`Text`** (unit 13E): build123d's text, a sketch of a string's faces with holes in letters as holes, to emboss or
  engrave, with its font, style, alignment, text on a path, single-line width and rotation; `Operations.MakeText` gives
  it as a `Result`. On the same font file it gives build123d's faces, areas and bounding boxes to 1e-13. Fonts are found
  among the installed ones, or read from a file, by the kernel's own reader; where one isn't installed, Kerf's own
  Liberation Sans stands in, which has Arial's widths, with a warning. A character the font has no glyph for fails,
  naming it (the kernel's decisions 2018 to 2024). **Breaking:** the unused `FontStyle` and `TextAlign` enums of
  Kerf.Modeling are the kernel's `Kerf.Fonts.FontStyle` and `TextAlign`, with the same members.
  or lose its context (an extrusion along a zero vector, a box of size 0, and a cone, sphere, torus, circle or Bezier
  curve refused; kernel decisions 2014 and 2028).
- **A `Cylinder`, `Hole`, `Sphere`, `Torus` or `Wedge` too small to be valid fails, and a `Rectangle` 1e-9 wide,**
  where each returned a shape that isn't valid as a success; **a `Superellipse` about 1.5e-7 wide and a `Triangle`
  with a side under the tolerance** fail as `ModelingException`, where they threw `InvalidCastException` and
  `InvalidOperationException` (kernel decision 2029).
- **Fusing a small part onto a large one is quick again:** a detent bump on an enclosure's door now takes less time than in 0.3.0 (0.4.0 took 4 to 5 times as long). A screw boss inside the door takes 1.3 to 1.4 times 0.3.0's time, against 4 to 5 in 0.4.0. The results are the same (the kernel's decisions 2034 to 2038).

## 0.4.0

- **Silent wrong answers found and fixed,** and failures that say why: a chamfer or fillet too large says what fits,
  2D offsets are within 0.1% of the distance, and `ConvexPolyhedron` builds from any points whose hull has volume. New:
  chamfers past tight corners, see-through parts in pictures, and `Operations.MakeConvexPolyhedron` giving a `Result`.
- **A part built by many cuts and fuses is about a sixth quicker,** at 0.94 times build123d's time on native OCCT for
  ordinary extrude-and-cut parts (the kernel's decisions 2000 and 2001).
- **Breaking:** `Kind.Tangent` and `MeshTolerances.ControlAllSurfaces` have gone.
- **`Chamfer` refuses a length past the tightest convex arc in a face beside its edges, saying what fits,** as `Fillet` does for a radius: "the most that fits is 1.7, the radius of the tightest convex arc in a face beside them", `modeling.chamfer-failed`, where it built a broken shape and failed with `result.invalid`. A chamfer by a length and an angle past it came back valid but wrong, and now fails (kernel decision 1985). **A chamfer that's built broken says it couldn't be built at that length**, after the faults (1986). **`MaxChamfer`**, beside `MaxFillet`, finds the largest length that builds, not trying past what the geometry allows (1987).
- **Chamfer past tight corners:** `Chamfer(…, options: new ChamferOptions { TightCorners = TightCorners.Collapse })` chamfers a rounded prism's end, or a housing's, by more than its corners' radius, letting each corner collapse to a point, where it was refused and needed a hull or loft by hand (kernel decision 1988). Off by default; below the corners' radius it changes nothing. A Kerf addition: build123d has nothing like it.
- **Pictures can show see-through parts:** give a `SceneItem` a colour with an opacity under 1 (kernel decision 1998).
- **`Wire.MakeConvexHull` and `MakeHull` are 100 to 5,000 times quicker and touch curves exactly:** a section of lines and arcs in about a millisecond, where 120 lines took 29 s and 4.6 GB. Areas agree with before to 1e-4 or better, closer to the exact hull (kernel decision 1997).
- **`ConvexPolyhedron` gives a valid solid for any points whose hull has volume, with one face for each flat side,** where points 1e-7 to 1e-6 apart, or a side with a run of close points, threw. It's about 50 times quicker on a few hundred points, and no longer crashes on a few thousand. Points closer than 1e-6 count as one. A set with no volume throws `ArgumentException` saying why; **`Operations.MakeConvexPolyhedron`** returns the same as a `Result<Part>`, failing with `modeling.hull-failed` (kernel decisions 1993 and 1996).
- **`Wire.MakePolygon(…, close: true)` closes a polygon whose ends are within 1e-6,** where ends 1e-7 to 1e-6 apart gave an open wire (kernel decision 1994).
- **A part built by many cuts and fuses spends about a third less on its Booleans' checks,** with the same results: the volume check no longer measures each result twice, and fusing a second boss or rib onto a part allocates about a third less (kernel decisions 2000 and 2001). On eight ordinary parts, the Booleans now take about as long as native OCCT's.
- **`Kind.Tangent` has gone** (a breaking change): an offset's kind is `Kind.Arc` or `Kind.Intersection`. In a 2D offset the tangent join gave the intersection join's result or nothing at all, and `Offset3D`, `Hollow` and a solid's `Offset` threw on it (kernel decision 1982). Use `Kind.Intersection`.
- **An ellipse or spline offset inwards past its sharpest bend is within 0.1% of the distance,** where it was up to 1% off, silently (kernel decision 1980). **A closed curve whose offset comes out open fails** as `modeling.offset-failed`, where `Operations.Offset` threw (decision 1981).
- **`Importers.ImportStep` past `ImportLimits.MaxBytes` or `MaxEntities` fails as `exchange.limit-exceeded`,** as documented and as BREP does, where it was `exchange.failed` (kernel decision 1965).
- **A 2D offset that closes up now fails, saying so, instead of throwing out of `Operations.Offset`** ("leaves nothing: the shape closes up at that distance", `modeling.offset-failed`), and a face that closes up is no longer an empty success. **A shape offset inwards past a waist gives each piece,** a dumbbell its two discs, where it threw. A face of several that closes up is left out with a warning. **A hole whose offset fails now fails the offset,** where it was silently left out (kernel decisions 1956 and 1957; the kernel's `docs/offset2d-survey.md`).
- **`Offset3D` and `Hollow` of a cone with its apex, inwards with round joins, now give the right solid,** made with mitres, with a warning that `Operations.Offset` returns as `offset.join-changed`; before, `Offset3D` gave a shape with no solid in it, and `Hollow` failed. **An offset or hollow solid that comes out nonsense now fails, holding the shape,** where it gave a volume of 2e100, or a repair that filled the wall, as a success (decisions 1947 and 1948).
- **`MeshTolerances.ControlAllSurfaces` has gone** (a breaking change): a mesh keeps to `Linear` on every kind of surface without it, by default, with no more triangles (kernel decision 1954). Remove it from a constructor or `with`; nothing else changes.
- **`Solid.Extrude` along the profile's plane and `Solid.MakeLoft` of sections in one plane now fail,** where they gave a solid of no volume.
- **A loft whose recovery runs out of memory says so,** rather than that it couldn't be recovered (kernel decision 1943).
- **Round all of a cube's edges to half its side and get a sphere, all of a bar's for a capsule,** and likewise slanted ends, a chain round a top, and an L's top. `MaxFillet` gives the radius that just fits, where it gave a little less.

- **Round a disc's rim to its full height, make a full round, a hemisphere or a sphere from a short cylinder:** these used to fail, and now give exact tori and spheres.

- **A fillet too large for a circular edge, such as a cylinder's rim, says what fits,** and `MaxFillet` doesn't try above it.

- **`Center` and `Plane(face)` are exact for a planar face bounded by lines and circles,** and far quicker, so the last digits of what's built on one can change.
- **An extrusion with nothing to merge skips its clean,** with the same shape and order, for a quarter to a half fewer bytes (the kernel's `docs/performance.md`).
- **A fillet that can't fit fails at once and says what does,** where it could take seconds; a radius a hair from the limit, either way, is the limit. `MaxFillet` gives the same answers with fewer tries (the kernel's `CHANGELOG`).

## 0.3.0

- **Bento's examples recalculate about 4.6 times as fast as at 0.2.0, with 72% fewer bytes:** 265 ms from 1,211 ms, and 126 MB from 450 MB, with ReadyToRun (the kernel's `docs/performance.md`).
- **Recalculation and a parallel round are about a fifth quicker, with about a fifth fewer bytes,** on Bento's examples, with the same shapes and meshes (a loaded machine). `Volume` and `Area` are exact on planar faces, so they can differ from before in their last digits.
- **`Volume` of a compound counts its solids at any depth,** and a compsolid's; a box in three nested compounds had a volume of 0.
- **`&` no longer returns an invalid solid** when a clean would spoil its good result.
- **Equal cylinders that nearly cross are right,** and a failing fillet fails sooner (the kernel's `CHANGELOG`).
- **Fillets and chamfers that just fit work:** radius 10 on one edge of a 10 mm cube gives the quarter cylinder it should, as do two fillets that meet on a face and a chamfer of a face's whole width, on straight edges between planes. A radius that's too large says so and says what fits, in place of "no starting solution". `MaxFillet` finds such a limit exactly, and no longer throws on a cylinder's rim when its last try fails (decisions 1847 to 1850).
- **A modelling call can be stopped:** inside `using (Kerf.Cancellation.Use(token))` every operation obeys the token, operators and constructors included (`docs/11t-bento.md`).
- **What used to end the process is a failure:** an assembly or compound nested more than 64 levels, which died at 300 to 3,000; a long loop of moves, which now also stays linear; and files nested too deep.
- **Requests too large to be meant fail at once:** a pattern of more than `LocationList.MaxCount` (10 million) locations, a tessellation of more than 10 million triangles.
- **A Boolean whose result is proved wrong fails, carrying the shape:** a cut larger than its object, a fuse smaller than its larger input, a solid inside out in the result, an empty common of solids that overlap, and the like, which used to be a valid shape with the wrong answer. As for an invalid result, `#VALUE!` in a cell and `ModelingDiagnostics.TryGetInvalid` for the shape (kernel decisions 1860 to 1868).
- **Areas and volumes are integrated precisely (a bug fix; results change):** `Area`, `Volume`, `ComputeVolume`, and sorting and grouping by them, to a millionth of each face's contribution, and the moments of inertia to 1e-9. OCCT's default of a fixed number of points was 6.5e-5 out on an introductory example, and can be 3.7% out on swept and B-spline faces. Over Bento's eleven examples this costs no more. `Center()` and `new Plane(face)`'s origin are on the precise path too, at 1e-9: a curved wall's centre was a whole unit out on a part 100 across.
- **A partial torus is a slice of the tube (a bug fix; results change):** `new Torus(…, minorStartAngle, minorEndAngle)` and `Solid.MakeTorus` close the arc of the section with a straight line. They used to make OCCT's solid under the arc, closed by flat discs to the axis: 210.6 and not 53.5 for the outer half with R = 5, r = 1.
- **A default sweep that fails tries another frame:** the Frenet frame, then the discrete one, as long as the result is valid (OCCT #1630). The operation's result carries a warning (`ModelingDiagnostics.SweepWarning`) naming the frame used; a shape's own `Solid.Sweep`, `Face.Sweep` and `Shell.Sweep` raise it through `Warnings`.
- **A sweep that encloses no volume fails and says why:** a `SweepMode` direction square to the path, or a path in the section's plane, gave a "valid" solid of no volume.
- **`OneD` and `TwoD` have gone from the public API:** `edge.ConnectedEdges(parent)`, `edge.ConnectedFaces(parent)` and `Wire.Combine(edges, tolerance)` replace them (its default tolerance is 1e-9, not 1e-6); `SortWiresByBuildOrder` is internal.
- **Typed parameters where there were `object`s:** a small `SweepMode` for `Solid.Sweep` and `SweepMulti` (a `Vector3D`, `Direction3D`, `Wire` or `Edge` converts to one), `CurveLimit` for `edge.TrimToOther`, `CurveLimit` for `shape.Intersect` too, and overloads of `FindIntersections`, `FindIntersectionPoints` and `ShapeOf`. The wrong type now fails to compile.
- What Bento has to change: [docs/answers-bento.md](docs/answers-bento.md).
- **Lengths are true (a bug fix; results change):** `Length`, `PositionAt` and the kernel's `LengthBetween` and `ParameterAtLength` were up to 1.5% out on uneven curves, as OCCT's are: an ellipse of 10 by 1 measured 41.24 for 40.64, and half way along a symmetric curve wasn't its middle (decision 1768).
- **Bugs that tests of their own found (unit 11J):** `BoundingBox` of a sketch off XY stays where the sketch is; `ProjectPoints` accepts a plane's own origin; `ParseNaca4` refuses five-digit and malformed codes; `(Solid)new Box(...)` works. **These now throw `ArgumentException`:** a scale by zero, unless the shape has no extent on that axis (was a collapsed shape); `Revolve` with an arc of 0 (was a full turn); `Torus` with a minor circle reaching its axis (was a solid through itself). Decisions 1769 to 1774.
- **No operation reports success for an invalid result (breaking).** A Boolean, sweep, loft, offset, fillet or chamfer whose result isn't valid fails
  with `result.invalid`, and the message says why; a sweep's "invalid solid" warning and `modeling.fillet-failed` for an invalid fillet are now this.
  `TryGetInvalid` on a result, or `ModelingDiagnostics.TryGetInvalid` on what a shorthand threw, gives the shape. A clean that would spoil a good Boolean is
  left out, with a warning. `docs/11r-bento.md` says what this means for a cell (decisions 1742, 1791 to 1796).
- **Names, cleaned up (no results change; `docs/clean-up-bento.md` lists what Bento changes):** every object's height is `Height` (was `BoxHeight`,
  `RectangleHeight`, `ConeHeight`, `CylinderHeight`, `TrapezoidHeight`, `SlotHeight`). `Polygon.Pts` is `Points`. `StlExportOptions` says
  `LinearDeflection` and `AngularDeflection`, as the glTF and 3MF options do. `DPrism` is `DraftPrism`. `GeomAdaptor()` is `AsCurve()` on edges and
  wires and `AsSurface()` on faces. `Triangle`'s parameters are `sideA` to `sideC` and `angleA` to `angleC`, as its properties are, and
  a few other parameters have full names (`trsf` → `transform`, `lefthand` → `leftHanded`).
- **Gone:** `SlotArc`, deprecated: use `SlotCurve`. `TopologyUtils` and `OneD.OffsetTopodsFace` are internal.
- **Quicker to open:** the first use in a process takes about a third less time (745 ms to 509 on Bento's examples), for packages about 20% larger. Results don't change. Grouping by a measure no longer boxes its keys.
- **`MeshTolerances.ControlAllSurfaces`,** off by default, and its documentation no longer promises a maximum deflection: a long fillet can stray from it.

- **`shape.BoundBox()` is now `shape.Bounds()`,** beside the kernel's `BoundingBox()`, which has a different default. Its `tolerance`
  argument, which did nothing, has gone.
- **STEP's header names Kerf:** `Kerf.Modeling` as the originating system, and the kernel's other changes.
- **Messages in Kerf's terms:** `Plane`'s and the shapes' messages no longer name OCCT's types.
- **`StepExportOptions.CleanDuplicates`** merges exactly equal entities when writing STEP. Off by default.
- **Faster recalculation:** `Extrude` until a surface no longer measures exact distances when bounds settle which surface it is, and
  `MeshTolerances.For` is about ten times quicker on filleted parts. Results don't change.
- What Bento has to change: [docs/11l-bento.md](docs/11l-bento.md).

## 0.2.0

The first release as Kerf.Modeling, replacing `Build123d` and `Build123d.OcctNet`. Every name and most signatures changed from the `OcctNet.*` and `Build123d` packages, and there's no compatibility
layer. Before 1.0 a minor version may break the API again.

- **One set of shapes and values:** shapes are the kernel's structs, with build123d's conveniences as extension members;
  `Part`, `Sketch`, `Curve` and `Assembly` are the modelling objects; points, vectors and boxes are the kernel's
  (`docs/one-set-of-shapes.md`).
- **A .NET API:** no Python conventions, typed options, and selectors in .NET's terms.
- **Results:** operations, importers and exporters return `Result`; operators throw.
- **Managed only:** it runs on Kerf.Kernel's public API, with no native backend and no Schematiq.Collections.
- **Images:** SVG and PNG through the kernel.

The units, oldest first:

- Kerf's unit 10B: `Build123d` calls OCCT.NET's types directly, under OCCT's names: OCCT.NET's structs, nothing
  disposed. `Build123d.OcctNet` (the generated layer that gave OCCT.NET the native binding's API) is gone, and so is the
  native backend (`Build123d.Occt`, its shim, bindings, generator and native build, `tools/pack.ps1`,
  `-p:Build123dBackend`). `Build123d` builds against the repository's kernel only: `UseOcctNetPackages` and
  `OcctNetVersion` are gone, as it uses kernel members the published packages don't make public.
- Kerf's unit 10E: the package and assembly are `Kerf.Modeling`, the namespace `Kerf.Modeling` (was `Build123d`), over the
  renamed kernel (`Kerf.Kernel.*`): the kernel types it shows are the kernel's new names (`Kerf.Geometry.Point3D`,
  `Kerf.Topology.Shape`…). Its own classes keep their names (`Shape`, `Edge`, `Box`…) until unit 10M merges its shapes
  onto the kernel's. No behaviour changes.
- Kerf's unit 10F: a failure in the kernel says what was being done again, as it did before 10B: the primitives,
  lofts, extrusions, revolutions, sweeps, thickening, triangulation and Booleans throw a `Kerf.KerfException` whose
  message starts with the operation ("box 0.0 × 1.0 × 1.0 failed: …", "extrude until / extrude failed: …"), whose
  diagnostics are the kernel's (a failed Boolean's alerts, with their codes and shapes) and whose inner exception is the
  kernel's own (OCCT.NET's exception type before). A STEP file that can't be read keeps its `ArgumentException`, with
  the exchange status's message. The 2D chamfer's history is read without an exception per vertex.
- Kerf's unit 10H4: `Fix`, the 2D chamfer and `FixDegenerateEdges` hand the shape to ShapeFix directly instead of a
  private copy: the kernel's healing no longer writes into its input, so the results share what wasn't fixed. Results
  are otherwise as before.

- Kerf's unit 10J: the kernel's `NCollection_Map` is `HashSet<T>`; the two places the layer used it (`ShapeHistory`,
  `Wire`) follow. No behaviour changes.
- Kerf's unit 10H3: the kernel's extrusions, revolutions, sweeps and lofts no longer copy their inputs. Results are
  as before: extruding a face taken from another part gives a solid that shares the face, its edges and vertices with
  the part, which stays unchanged (new tests: `ConstructionInputsUnchangedTests`).
- Kerf's unit 10H5: fillets, chamfers, offsets, shells, thickenings and drafts no longer copy the part at the kernel's
  entry: the results share the faces the operation left alone, and the part is never written. Results are otherwise as
  before.
- Kerf's unit 10K1: `Face.MakeSurfacePatch` with `ContinuityLevel.C2` constraints works (the kernel's MakeFilling threw
  on G2), and `C1` keeps meaning tangent continuity.
- Kerf's unit 10H6: triangulating a part no longer copies it in the kernel: the mesh is written in a working overlay,
  and copying it out allocates less than half what it did (the normals go straight into the buffer).
  `ShapeMeshTests.ABigMeshComesOutQuickly` checks the allocation and the best of five runs instead of one timing, which
  failed on loaded machines. Results are as before.
- Kerf's unit 10J2: the kernel's `NCollection_DataMap`, `NCollection_DynamicArray` and indexed maps are `Dictionary`,
  `List`, `IndexedSet` and `IndexedMap`; the layer's uses (`Vertex`, `OneD`, among others) follow. No behaviour changes.
- Kerf's unit 10K2: `Face.Chamfer2D` chamfers the face with the kernel's MakeFillet2d again, as build123d did before it
  worked around OCCT's merging of a holed face's wires into one (the kernel keeps them apart now): a chamfer on a face
  with holes is one operation, and its history comes from the kernel. `Face.Fillet2D` keeps build123d's wire-by-wire
  fillet.
- Kerf's unit 10L1: the kernel's geometry and topology getters became properties (`X`, `Direction`, `Type`,
  `Placement`…) and `Distance`/`Angle` became `DistanceTo`/`AngleTo`; the layer's call sites follow, rewritten by
  the same tool. The layer reaches the kernel's now-internal engine through `InternalsVisibleTo` until 10M. No
  behaviour changes.
- Kerf's unit 10L2: the kernel's `GlueMode`, `FilletShape`, `OffsetMode`, `TransitionMode` and `ParametrizationType`
  moved into its public namespaces (`GlueMode.GlueOff` is `GlueMode.Off`); the layer's call sites follow, rewritten by
  kerf-rename. No behaviour changes.
- Kerf's unit 10L3: STEP is written and read, and STL, glTF and 3MF written, through the kernel's public
  `Kerf.Exchange` (`Step`, `Stl`, `Gltf`, `ThreeMf`, `ModelNode`), and `Triangulate` meshes through `Kerf.Meshing`'s
  `Mesher`, which meshes in a working overlay instead of a private copy. The files are byte for byte as before (the STEP
  native references pass unchanged). The STL and glTF readers and BREP import keep their glue until 10M.
- Kerf's unit 10K4: the kernel's 2D constraint solvers are right, so the constrained arcs and lines use their
  tangency points as given: no parameter offset for enclosed solutions, no re-intersection for tangent and oriented
  lines. Constrained arcs tangent to three curves now work on free-form curves (the third tangency threw). Two
  identical circles raise "Unable to find common tangent line(s)" (they gave no lines).
- Kerf's unit 10M1: the layer calls the kernel's public API alone (the kernel gives it no `InternalsVisibleTo`), and
  its public API shows no engine type: `Vector(Xyz)`, `ToGpXyz`, `Color(ColorWithAlpha)`, `ToQuantityColorRgba`,
  `BoundBox.FromBndBox`, `OrientedBoundBox.FromBndObb`, `ShapeHistory.FromBoolean`, `FromUnify`, `FromSewing` and
  `FromAlgorithm(MakeShape)` are gone or take the kernel's public types (`ToKerfColor`, `FromBoundingBox`,
  `FromOrientedBox`, `FromHistory(Kerf.Topology.ShapeHistory)`); `Edge.GeomAdaptor()` and `Wire.GeomAdaptor()` return
  the kernel's `EdgeCurve` and `WireCurve`. `Meshed` is gone (the kernel never stores a mesh on a shape). Numbers print
  as .NET's invariant text ("1", not Python's "1.0") and `ToString(format)` takes .NET formats ("G6", "F2"); `Repr()`
  is gone. A kernel failure is still a `KerfException` in context; an argument the kernel refuses first is one too
  (`kernel.invalid-argument`), and BREP data that isn't BREP is `exchange.malformed`. STL, glTF and BREP are read
  through `Kerf.Exchange`. Results are unchanged: volumes, face counts, the STEP references byte for byte, the
  Python-reference tests at their level. `docs/10m1-bento.md` lists the members Bento sees renamed or removed.
- Kerf's unit 10M2: the public API is redesigned for .NET; `docs/10m2-bento.md` lists every member renamed, removed or
  retyped, and `docs/name-map.md` maps Python build123d to the new names. Selections are typed extension methods on any
  sequence of shapes, beside LINQ (`Where(GeomType.Plane)`, `ParallelTo(Axis.Z)`, `OrderBy(Axis.Z)`,
  `GroupBy(ShapeKey.Area)`, LINQ's `Last()`); the string selectors, the list operators and `First`/`Last` properties
  went. A Part, Sketch or Curve combines into its own class (`Part + Shape` is a `Part`; `Add`, `Subtract`, `Common`),
  and `Mirror`, `Scale`, `Split`, `Offset` and `Sweep` are typed for them. `Extrude`, `Sweep`, `Offset` and `Thicken`
  take options records (`Operations.ExtrudeOptions`…), and an extrusion up to a target is its own overload. Expected
  failures are `KerfException`s with codes (`ModelingDiagnostics`), or `Result<T>` from the `Try` forms
  (`TryFillet`, `part.TrySubtract`…); `DraftAngleError` went; null inputs throw `ArgumentNullException`; messages are
  .NET sentences. `Mode`, the tuple conversions (`new Vector(1, 2, 3)`, `new Alignment(x, y)`, `new Tangent(edge,
  tangency)`), `%`, `^` and `&` on geometry went. `PySet`, `PyComparer` and `PyMath` are .NET (`PointOrder`,
  `KeyComparer`, `Numerics`) with the same results. `ToGp…` are `ToPoint3D` and the kernel's other names, `Wrapped` is
  `KernelShape`. Results are unchanged: volumes, face counts, the STEP references, the Python-reference tests.
- Kerf's unit 10M3: no `KerfException`. `Operations`' `Fillet`, `Chamfer`, `Extrude`, `Loft`, `Revolve`, `Sweep`,
  `Offset`, `Thicken` and `Draft`, the `Add`, `Subtract` and `Common` of a Part, Sketch or Curve, and the importers
  return `Result<T>`; the `Try` forms went. The operators, the objects' constructors and the topological shapes' own
  methods return the value, as the shorthand for inputs known to work, and throw `InvalidOperationException` with the
  same message on failure. A failed thickening, solid offset or unrecoverable loft is a failure with a code
  (`modeling.thicken-failed`, new; `offset-failed`; `loft-failed`), and data an importer can't read the exchange's
  diagnostic, where they were exceptions. Results are unchanged. `docs/10m3-bento.md` lists what changed for Bento.

Phase 10 (part): no Schematiq.Collections (unit 10P1).
- `ShapeList<T>` holds an `ImmutableArray<T>`, and the package no longer depends on Schematiq.Collections. No public signature
  changed, and results are the same; `Add` and `AddRange` now copy the list. `docs/10p-bento.md` lists what changed for Bento.
- Kerf's unit 10P2: the exporters return `Result` with the exchange's diagnostics, where they returned `bool` and threw,
  and take options records (`StepExportOptions`, `StlExportOptions`, `GltfExportOptions`, `ThreeMfExportOptions`). The
  kernel's values refuse misuse with .NET's exceptions, which the layer reports as before. KERF0003 now covers the layer,
  and finds nothing. Results are unchanged. `docs/10p2-bento.md` lists what changed for Bento.
**Tests (unit 10P3).** Every public member runs in a test (`PublicMemberTests`). The packaging test is `ExtensionLoadTests`, reading `KERF_MODELING_LIB`.

**One set of shapes (unit 10Q, question 56).** The layer uses the kernel's shapes: no `Shape`, `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` or `ShapeType` of its own, and its conveniences are extension members on the kernel's structs. `Part`, `Sketch`, `Curve` and the new `Assembly` derive from `ModelShape`, which carries the label, colour, construction flag and history. A list remembers what it was selected from (`Origin`); what needed a parent reads that or takes it as an argument. `Color` is the kernel's (`ColorExtensions`: `Color.Parse`, `FromCode`, `Rgb`). `ShapeHistory` is `ModelHistory`. `Shape.ProjectFaces` no longer throws where a projection is a shell. Results are unchanged. `docs/one-set-of-shapes.md` is the design, and `docs/10q-bento.md` lists every change. `Vector`, `BoundBox` and `Matrix` are still the layer's, for the reasons there.

**One set of values (unit 10Q2).** `Vector`, `BoundBox`, `Matrix` and `Tangency` went, for the kernel's `Point3D`, `Vector3D`, `Direction3D`, `BoundingBox`, `GeneralTransform3D` and `Tangency`, with extension members. A position is a point, an offset a vector; the tolerance is `IsNear` and `PointComparer`. On a struct, `Edges()` and the rest are the selection, as on a `Part`, and `IsClosed` means closed. `IsUnbounded` is `IsInfinite`. A file importing every namespace qualifies nothing common. Results are unchanged, to the byte on Bento's examples. `docs/10q2-bento.md` lists what changed for Bento.

## 0.1.0

- OCCT's failures reach the caller as OCCT.NET's exceptions (`Standard_Failure` and its subclasses, with `OcctType`),
  until results replace them: `OcctException`, `OcctContext` and the "… failed:" prefix are gone. No more native
  faults or memory pressure (`Extras.AddMemoryPressure`, `MemoryPressureOf`).
- A wire's `Triangulate` and `TriangulateTree` work: they threw `InvalidCastException`.
- `Wire.Fillet2D`'s history matches native's on OCCT.NET: the fillet records the finished wire's edges, not each edge
  MakeWire returned on the way, which OCCT.NET may since have replaced in a copy (`Fillet2DTests.FilletRecord`).
- `ShapeList<T>` is over Schematiq.Collections (`ISchematiqList<T>`, 1.0.0-beta.1 from nuget.org), the version Bento
  ships, in place of Bento.Collections 0.4.0; the local feed and `tools/pack-bento.ps1` are gone.
- OCCT.NET is the default backend, from its packages on nuget.org (`OcctNetVersion`, 0.1.0-preview.3), or from an
  OCCT.NET checkout's projects with `-p:OcctNetRoot=<checkout>/`. A fresh clone builds and tests with `dotnet test`.
  Native OCCT is opt-in with `-p:Build123dBackend=Native` (into `bin-native`, `obj-native`), kept as the reference
  OCCT.NET's oracle compares against, and no longer published; `tools/pack.ps1` packs it locally, as `0.1.0-native`,
  for its load-context test only. `Build123dNativeRoot` takes the native build from another checkout.
- A shape log for the differential run (OCCT.NET's unit 9D2): with `BUILD123D_SHAPE_LOG` naming a folder, every new
  kernel shape a `Shape` wraps is written there as BRep text, one file per test, with the build123d call that made it
  and a digest of its vertices' exact points. Off, it costs one static field read; on, it only reads the shapes. Run
  the suite one test at a time with it (`-- xUnit.ParallelizeTestCollections=false`); OCCT.NET's
  `tools/b3d-differential.ps1` runs it on both backends and compares the logs.
- A spike: `-p:Build123dBackend=OcctNet` builds `Build123d` on OCCT.NET, the managed port of OCCT, with no native
  libraries (`src/Build123d.OcctNet`, generated by `tools/generate-occtnet.ps1`). Native OCCT stays the default.
  On OCCT.NET's phase 7, 3D fillets, chamfers, offsets, thickening, drafts and `BRepFeat` (extrusion until, split by
  perimeter) are forwarded too; what OCCT.NET hasn't ported (STEP) throws `NotSupportedException`.
- Import and export, as build123d's `importers.py` and `exporters3d.py`: `Importers.ImportBrep`, `ImportStep`,
  `ImportStl`, `ImportGltf` and `Exporters3D.ExportBrep`, `ExportStep`, `ExportStl`, `ExportGltf`, `Export3mf`. Streams
  first, file paths on top; `ImportLimits` for hostile data; cancellation. STEP on both backends, written byte for byte
  as native OCCT writes it and read to the same shapes, without names, colours or assemblies yet. STL, glTF and 3MF
  on the OCCT.NET backend only, over OCCT.NET's writers and readers.
- The OCCT.NET backend forwards `STEPControl_Reader` and `Writer` (OCCT.NET's phase 8): no class throws "not yet
  ported" any more. It references `OcctNet.DataExchange`.
- STEP's names, colours and assemblies on the OCCT.NET backend, as build123d's `import_step` and `export_step`, through
  OCCT.NET's STEPCAFControl and its assembly tree (`Extras.ReadStep`, `WriteStep`, `StepKeepsAssemblies`). A STEP
  stream that refers to other files is refused. Natively STEP stays shapes alone (XCAF isn't bound).
- `BoundingBox()` bounds faces holding only triangles (STL, glTF) by their triangles, rather than giving an empty box.
- `IntroductoryExamplesTests`: the 37 introductory examples that need no text, in algebra mode, checked against
  native OCCT's volumes and face counts.
- The native `TopoDS_Iterator` keeps the shape it iterates alive. OCCT 8's iterator holds no reference to it, so a
  shape reachable only through its iterator could be finalized, and its children freed, mid-iteration.

- Licensed under Apache-2.0.

The first packages, for Bento's 3D-models extension.

- Geometry: Vector, Axis, Plane, Location, Rotation, Matrix, Color, BoundBox.
- Topology: Shape, ShapeList and Vertex through Compound, with selectors and history.
- Objects and operations in algebra mode: the part, sketch and curve objects, location lists, and the
  generic, part and sketch operations (not yet `insert`, `add`, `make_brake_formed` or text).
- Native libraries for Windows x64 only.
- Operations no longer change their inputs or the geometry those inputs share, and can run concurrently
  on shared shapes:
  - meshing works on copies (`Shape.Mesh` is replaced by `Meshed`, which returns a meshed copy);
  - booleans run non-destructive;
  - ShapeFix, loft and tapered extrusion work on copies.
- Warnings are collected by `Warnings.Capture()` scopes, which replace the static `Warnings.Raised` event.
- `Shape.Triangulate` gives a structured, immutable `ShapeMesh` for viewers and picking:
  - face meshes, with nodes shared within each face, surface normals, outward winding and the surface type;
  - edge polylines, taken from the face meshes, with the curve type;
  - vertices.
  Each list is in the order of `Faces()`, `Edges()` and `Vertices()`, and the whole mesh is copied out of
  the kernel in bulk. `MeshTolerances.For(shape)` gives tolerances in proportion to the shape.
  `TriangulateTree` gives the assembly tree, with labels, colours and locations, meshing each instanced
  geometry once.
- Native faults on Windows come back as typed `OcctException`s on any thread (`IsFault`). OCCT failures
  say what was being done (`OcctContext.Doing`, `OcctException.Operation`). Failed booleans name OCCT's
  alerts.
- New results tell the GC about their native memory (`GC.AddMemoryPressure`). Triangulation frees its
  temporary native mesh at once.
- Reading BRep data rejects truncated, garbled and overlong input instead of hanging or overrunning a
  buffer.
- Selectors take build123d's `property` arguments by name (`FilterBy("IsClosed")`, `FilterByValue("Radius", 5)`,
  `SortBy("Length")`, `GroupBy("Area")`), and `FilterByValue(SortBy, value)` takes a measure.
- `Selector`: a selection as plain values that can be kept and applied again. `Selector.For` finds the
  simplest selectors for a picked face, edge or vertex.
