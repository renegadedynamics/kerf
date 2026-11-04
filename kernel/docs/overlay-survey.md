# The overlay survey: copy at entry, and the in-place edits behind it

*Unit 10H1, 3 November 2026. The work list for units 10H2 to 10H6 (phase10-plan.md, section 2.4). Counted at Kerf
`main` db338009 plus 10H1, with `tools/overlay-survey.py` (run `python -I tools/overlay-survey.py src --files` from
`kernel/` to recount). Decisions 1359 to 1364. Recounted by unit 10H2 after the Booleans (decisions 1378 to
1384), by unit 10H4 after healing (decisions 1393 to 1399), by unit 10H3 after construction (decisions 1410 to 1418),
by unit 10H5 after offsets, fillets and features (decisions 1419 to 1427) and by unit 10H6, which removed the last
sites and the machinery (decisions 1443 to 1451): the counts below are the finished state. Section 9 closes the
survey.*

## What this is

OCCT's algorithms edit the topology they are given. OCCT.NET freezes every shape a caller holds, so each algorithm that
writes into its input copies it first ("copy at entry") and gives back afterwards what it didn't change. Phase 10 replaces
the copies with a per-operation **working overlay** (`WorkingOverlay`, in ModelingData's BRepTools folder; decision
1359): the algorithm runs on the caller's shape, its writes into frozen TShapes go to working copies that it reads back,
and the result is assembled with new TShapes only for what changed.

This survey lists:
- the copy-at-entry sites, module by module, file by file: what 10H2 to 10H6 remove;
- the frozen-shape checks and workarounds that exist only because of frozen inputs;
- every kind of in-place edit the engine makes, with the `BrepBuilder` method and the TShape member it reaches, and
  how the overlay takes it.

Counts are of source sites found by regular expressions, comments excluded: estimates to plan by. The modules are
the plan's order: Booleans (BOPAlgo, BOPDS, BOPTools, BRepAlgo, BRepAlgoAPI, IntTools, TopOpeBRep\*), construction
(BRepBuilderAPI but sewing, BRepLib, BRepPrim, BRepPrimAPI, BRepSweep, BRepFill, GeomFill, HelixBRep, BRepProj, and
BRepOffsetAPI's pipes, lofts, evolved and filling), healing (Shape\*, sewing), offsets (BRepOffset, BiTgte,
BRepOffsetAPI's offsets), fillets (ChFi\*, FilletSurf, BRepFilletAPI, BRepBlend, Blend\*), features (BRepFeat, LocOpe,
Draft, BRepOffsetAPI's drafts), other (meshing, hidden lines, data exchange, ModelingData itself).

## 1. Copy at entry: none left

Before 10H1 there were 113 by 10H1's count; 10H2 found that the count missed the target-typed constructor
(`CopyAtEntry entry = new(input)`): with it, 119 were left after 10H1's pilot. The units removed them module by module:
the Booleans' 14 (10H2), healing's 34 (10H4), construction's 38 (10H3), the offsets', fillets' and features' 12 (10H5),
and the last 20 (10H6): MakeOffset's thickening mode and the simple offset's solid, the mesher, HLR's outliner, the
STEP reader's regularity and ShapeProcess's modifiers, and BRep_Builder's `…Unfrozen` forms (the mesher's, renamed:
they concern the mesh's freezing, not the shape's; section 9).

| Kind | Booleans | Construction | Healing | Offsets | Fillets | Features | Other | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `new CopyAtEntry(…)`, `CopyAtEntry x = new(…)` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `ExactCopyForWrite` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `KeepUnchanged`, `KeepUnchangedIn` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `KeepUnchangedGenerating` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `CopyFrozenForWrite` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `HoldsFrozen` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| Frozen workarounds (`…OfFrozen`, `…Unfrozen`, `UnfreezeResult`) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| **Total** | **0** | **0** | **0** | **0** | **0** | **0** | **0** | **0** |

The helpers are gone too (10H6): `CopyAtEntry` (301 lines), `BrepTools.ExactCopyForWrite`, `HoldsFrozen`,
`KeepUnchanged`, `KeepUnchangedIn` and `CopyFrozenForWrite` (`BRepTools.ExactCopy.cs` and `BRepTools.CopyFrozen.cs`,
717 lines). `BrepTools.CopyData`, `CopyFlags` and `IsSameData` stay in `BRepTools.TShapeData.cs`: the overlay makes
its working copies and compares them with them (decision 1361). The tests keep an exact copy of their own
(`TestCopies`), to run ported in-place code on shapes it may write into, as OCCT's tests do.
`LockedShapeException` and `FrozenShapeException` stay: BRep_Builder throws the first on a write into a locked or
read-only shape (31 sites, OCCT's own refusals among them), and TopoDS_Builder the second when a shape isn't free, as
OCCT does.

### The work list, by file (done)

- **Booleans** (10H2): 14 sites in 9 files: `BOPAlgo_PaveFiller_10.cs` 1, `BuilderTools.cs` 1, `CellsBuilder.cs` 2,
  `CheckerSI.cs` 1, `MakeConnectedAlgorithm.cs` 1, `MakePeriodicAlgorithm.cs` 5, `BRepAlgo.cs` 1, `FaceRestrictor.cs` 1,
  `WireToFace.cs` 1.
- **Construction** (10H3): 38 sites in 18 files: BRepFill `AdvancedEvolved.cs` 2, `Evolved.cs` 2, `Fill.cs` 7,
  `Generator.cs` 2; BRepLib `BRepLib.Twins.cs` 4, `LibraryMakeFace.cs` 3, `LibraryMakePolygon.cs` 2,
  `LibraryMakeShape.cs` 1, `LibraryMakeSolid.cs` 1, `LibraryMakeWire.cs` 3; BRepOffsetAPI `ApiMakePipe.cs` 1,
  `ThruSections.cs` 2; BRepPrimAPI `ApiMakePrism.cs` 1, `ApiMakeRevol.cs` 1, `MakeSweep.cs` 3; `Projection.cs` 1;
  BRepSweep `Rotation.cs` 1, `Translation.cs` 1. The prisms and revolutions were decisions 468 and 1102's unshared
  faces: a prism of a polygon face now holds the caller's face.
- **Healing** (10H4 and 10H6): 35 sites in 21 files: `Sewing.cs` 3, `ShapeBuild_Edge.Twins.cs` 2, `ShapeConstruct.cs` 2,
  `ComposeShellFixer.cs` 2, `EdgeConnectFixer.cs` 1, `FaceConnectFixer.cs` 1, `FixSmallFaceFixer.cs` 1,
  `FreeBoundsFixer.cs` 1, `ShapeFix.Twins.cs` 3, `ShapeFix_Face.CopyAtEntry.cs` 2 and `ShapeFix_Wire.CopyAtEntry.cs` 2
  (now `….Overlay.cs`), `ShapeFixer.cs` 1, `ShellFixer.cs` 2, `SolidFixer.cs` 1, `WireVertexFixer.cs` 2,
  `WireframeFixer.cs` 2, `OperLibrary.cs` 2, `ShapeContext.cs` 3, `ShapeDivide.cs` 1, `UnifySameDomain.cs` 1. The
  modelling layer's three private copies before ShapeFix went too.
- **Offsets** (10H5 and 10H6): 5 sites: `Offset.cs` 2, BiTgte `OffsetBlend.cs` 1, `MakeOffset.cs` 1 (the thickening
  mode) and `MakeSimpleOffset.cs` 1 (the solid flag). The last two waited for BRepLib_MakeFace to run in place in an
  overlay (decision 1421), which 10H3 did.
- **Fillets** (10H5): ChFi2d `PlanarBuilder.cs` 1 and ChFi3d `Builder_1.cs` 1.
- **Features** (10H5): `MakeLinearForm.cs` 1, `RibSlot.cs` 1, `DraftAngle.cs` 2 and LocOpe `WorkingShapes.cs` 3.
- **Other** (10H6): 17 sites: XSControl `TransferReader.cs` 2, BRepMesh `IncrementalMesh.cs` 3 and `ShapeTool.cs` 6,
  HLRTopoBRep `OutLiner.cs` 2, `BrepBuilder.cs` 4 (`UpdateEdgeUnfrozen`, `UpdateFaceUnfrozen`, now
  `UpdateEdgeMesh` and `UpdateFaceMesh`).

## 2. Frozen-shape checks: 149 sites

| Kind | Booleans | Construction | Healing | Offsets | Fillets | Features | Other | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `.IsFrozen` tests (topology and geometry) | 1 | 15 | 16 | 1 | 3 | 9 | 39 | 84 |
| `.Locked()` tests | 0 | 0 | 0 | 0 | 0 | 0 | 10 | 10 |
| `// OCCT.NET: frozen` markers | 1 | 15 | 29 | 0 | 1 | 0 | 9 | 55 |
| **Total** | **2** | **30** | **45** | **1** | **4** | **9** | **58** | **149** |

10H6's review (decision 1447) found none left that only guarded a copy at entry: the three it removed were the
outliner's, the mesher's and the STEP reader's `HoldsFrozen` walks. What is left is of three kinds:
- **the rule itself**, in ModelingData: BRep_Builder's refusals ask `WorkingOverlay.IsLocked` (OCCT's `Locked()`: the
  flag, or `IsReadOnly`, frozen with no overlay here taking the write), and the TShapes' setters take the write into a
  working copy (`TryWritable`) or throw (`CheckMutable`). The ten `.Locked()` tests are TopoDS_Shape's own accessor;
- **writes skipped outside an overlay** where they would change nothing or only a cache bit (a vertex's tolerance
  already high enough, `BRep_Tool::Range`'s Modified flag, the sweep's `Closed` flag, the Free flag a frozen component
  keeps), and copies of a caller's published wire or shell before a flag is set on it (the draft's and pipe shell's,
  which run outside overlays): they protect published shapes, and inside an overlay most already ask `IsReadOnly`;
- **geometry's**: a curve or surface copied before an edit (10G's), and the read-only representation lists read with
  `Curves()` where OCCT takes `ChangeCurves()` only to read.

Recounted by the script when 10H3 and 10H4 merged; the units' own counts (187 and 196) grouped a few checks
differently. Healing's left are geometry's (a curve copied before an edit: 10G's) and the checks that refuse a write,
which now ask `WorkingOverlay.IsReadOnly` (frozen, and no overlay here takes writes; decision 1396). Other's include the
overlay's own helpers (`WorkingOverlay.Tools.cs`); construction's include BRepLib's SameParameter asking the same.

The Booleans' two left are the pave filler's: the walk deciding the non-destructive mode outside an overlay
(`PaveFiller_10`, decision 1378) and the skipped no-op tolerance write of `PaveFiller_6` (decision 875), which the
general fuse needs because it runs outside any overlay. Construction's 29 left are geometry's (`IsFrozen` of a curve or
plane copied before an edit, 10G's), BRepLib's writes skipped when they change nothing (needed outside an overlay), the
sweep's `Closed` flag on a published result, BRepFill_Filling's bindings of MakeWire's history (10H3 kept them: they
follow `InternalHistory`, which now records only what changed), the pipe shell's, and two of 10H3's own: a point
section of a caller's vertex is frozen with it, and `SameParameter` asks for a frozen edge's writable list in an overlay.
Other's two more are `WorkingOverlay.TakesWrite`'s.

The markers are the places where the port departs from OCCT because an input is frozen: a write skipped when it would
change nothing, a new TShape made instead of an edit, a copy of geometry before an edit. Inside an overlay most are no
longer needed; each unit reviews its module's and reverts to OCCT's code where the overlay takes the write. Geometry's
`IsFrozen` tests (a curve copied before an edit) are 10G's and stay; the offsets', fillets' and features' left are
geometry's. `BrepBuilder`'s 29 `Locked()` tests now ask `WorkingOverlay.IsLocked` (an overlay takes the write), and since
10H5 so do the 5 in its `Make…` methods, which refuse to make a new TShape into a variable holding a locked shape, as
OCCT: inside an overlay a variable holding a frozen shape takes the new TShape, as one holding its copy did (decision
1423). The offsets' 2 `Locked()` tests (BRepOffset_Tool's enlarged offset surface, OC5805) ask it too (decision
1420).

## 3. The kinds of in-place edit

Every call is counted, including the many into topology the algorithm built itself (most of them: an edge built, then
given its pcurves). Only a fraction writes into an input; that fraction is what copy at entry protects today, and what
the overlay takes.

| Kind | Booleans | Construction | Healing | Offsets | Fillets | Features | Other | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Vertex point, tolerance or parameter (`UpdateVertex`) | 22 | 61 | 96 | 41 | 9 | 30 | 17 | 276 |
| Parameters moved between edges (`Transfert`) | 1 | 2 | 0 | 0 | 0 | 0 | 0 | 3 |
| Edge 3D curve, pcurve, polygon or tolerance (`UpdateEdge`) | 25 | 108 | 110 | 75 | 4 | 15 | 59 | 396 |
| Edge range (`Range`) | 13 | 85 | 62 | 76 | 6 | 20 | 38 | 300 |
| Edge flags (`SameParameter`, `SameRange`, `Degenerated`) | 6 | 67 | 73 | 42 | 3 | 4 | 13 | 208 |
| Edge continuity (`Continuity`) | 0 | 24 | 2 | 9 | 2 | 4 | 17 | 58 |
| Face surface, tolerance or mesh (`UpdateFace`) | 0 | 3 | 1 | 7 | 0 | 0 | 16 | 27 |
| Face natural restriction | 1 | 1 | 2 | 0 | 0 | 0 | 3 | 7 |
| BRepLib's in-place statics (`SameParameter`, `UpdateTolerances`, `BuildCurves3d`, `EncodeRegularity`, …) | 8 | 29 | 11 | 42 | 5 | 5 | 3 | 103 |
| Sub-shapes added (`Builder.Add`) | 196 | 358 | 286 | 205 | 34 | 163 | 223 | 1,465 |
| Sub-shapes removed (`Builder.Remove`) | 5 | 14 | 10 | 11 | 2 | 2 | 4 | 48 |
| Shape flags set (`Free`, `Closed`, `Orientable`, `Modified`, `Checked`, `Convex`, `Infinite`) | 13 | 62 | 18 | 21 | 0 | 5 | 145 | 264 |
| TShape data written directly (`ChangeCurves`, `ChangePoints`, the BRep TShapes' setters) | 15 | 18 | 22 | 13 | 0 | 1 | 58 | 127 |
| Child lists edited directly (`TShape._shapes`) | 5 | 0 | 4 | 0 | 0 | 0 | 25 | 34 |
| Representations edited (`Range`, `PCurve`, `SetUVPoints`, `Parameter` on a curve or point representation) | 1 | 0 | 12 | 0 | 1 | 1 | 38 | 53 |
| Orientation and location (on the shape value) | 149 | 296 | 228 | 134 | 366 | 103 | 483 | 1,759 |

### The member each kind reaches, and how the overlay takes it

| Kind | `BrepBuilder` / `Builder` method | TShape member written | In an overlay |
|---|---|---|---|
| Vertex tolerance | `UpdateVertex(v, tol)`, every `UpdateVertex` | `BrepTVertex.UpdateTolerance`; `Tolerance(double)` directly (ShapeFix_ShapeTolerance) | Taken: the setter writes into the working copy |
| Vertex point | `UpdateVertex(v, p, tol)` | `BrepTVertex.Pnt(p)` | Taken |
| Vertex parameter on an edge or face | `UpdateVertex(v, par, e[, f], tol)`, `UpdateVertex(v, u, v, f, tol)`, `Transfert` | `BrepTVertex.ChangePoints()` (`PointOnCurve`, `PointOnCurveOnSurface`, `PointOnSurface`); for an end vertex also the edge's `ChangeCurves()` ranges | Taken: both the vertex and the edge get working copies |
| Pcurve added, replaced or removed | `UpdateEdge(e, c2d[, c2d2], f \| s, l, tol)` | `BrepTEdge.ChangeCurves()` (`CurveOnSurface`, `CurveOnClosedSurface`), `UpdateTolerance` | Taken |
| 3D curve set or removed | `UpdateEdge(e, c3d[, l], tol)` | `ChangeCurves()` (`Curve3D`) | Taken |
| Edge range | `Range(e, first, last[, only3d])`, `Range(e, s \| f, l, first, last)` | the `GCurve`s of `ChangeCurves()` (`SetRange`) | Taken: `ChangeCurves()` gives the working copy's representations, fresh copies |
| Same parameter, same range, degenerated | `SameParameter`, `SameRange`, `Degenerated` | `BrepTEdge`'s flag setters | Taken |
| Continuity | `Continuity(e, f1, f2, c)` | `ChangeCurves()` (`CurveOn2Surfaces`) | Taken |
| Edge tolerance | `UpdateEdge(e, tol)` | `BrepTEdge.UpdateTolerance` | Taken |
| Triangulations and polygons | `UpdateFace(f, triangulation)`, `UpdateEdge(e, Polygon3D \| PolygonOnTriangulation \| Polygon2D…)`, the mesher's `UpdateFaceMesh`/`UpdateEdgeMesh` | `BrepTFace.Triangulation(…)`, `Triangulations(…)`; `ChangeCurves()` | Taken. The mesher's forms (10H6's names for the `…Unfrozen` forms) leave the mesh unfrozen until the shape is published, so its post-processor can finish it (decision 488): they concern the mesh's freezing, not the face's |
| Face surface, location, tolerance, restriction | `UpdateFace(f, s, l, tol)`, `UpdateFace(f, tol)`, `NaturalRestriction` | `BrepTFace`'s setters | Taken |
| Sub-shapes added or removed | `Builder.Add`, `Builder.Remove` | `TShape._shapes`, `Modified` | Taken: the working copy's child list |
| Shape flags | `Shape.Free(bool)`, `Closed(bool)`, … | `TShape`'s flag setters | Taken |
| Child lists edited directly | none: `tShape._shapes.Append/Clear` | `TShape._shapes` | **Not taken**: 42 sites, 13 of them in algorithms (`Sewing`; ShapeFix `FaceFixer`, `WireVertexFixer`, `ShapeFix_Wire.CopyAtEntry`; the rest are the copy helpers and the hooks themselves; BOPTools' 5, in `ConnexityBlock` and `ToolSet`, are those classes' own fields named `_shapes`, not a TShape's). 10H2 removed TopOpeBRepBuild `WireToFace`'s, the only one in the Booleans; 10H3 `LibraryMakeFace`'s two (the face's wire swapped after a copy). 10H4 removed sewing's and ShapeFix_Face's; healing's 4 left fill compounds ShapeFix_Wire and ShapeFix_WireVertex make themselves (so as not to clear the frozen edges' Free flags), and Other's 9 more are the overlay's helpers'. 10H5 removed LocOpe `WorkingShapes`'s (its holder compound: the overlay's `Finish` takes the shapes as they are). Each unit reroutes its module's through `Builder` or `WorkingOverlay.ForWrite`, or shows they act on unfrozen TShapes only |
| Representations edited directly | none: `gc.SetRange(…)`, `pr.Parameter(…)` on a representation | the representation | Taken when the representation came from `ChangeCurves()`/`ChangePoints()`. One read through `Curves()`/`Points()` of a frozen TShape without a working copy is the frozen one: editing it throws, as today (fail fast). 53 sites, mostly ModelingData's own |
| Orientation and location | `Shape.Orientation(o)`, `Location(l)`, `Reverse()`, `Move(l)` | none: they change the shape value, never a TShape | Nothing to take. OCCT reorients a sub-shape in place only by `Remove` and `Add`, counted above |

## 4. What the overlay reads through

While an overlay is open (on the thread, or in tasks it started), these answer from a frozen TShape's working copy:
- `TShape`'s flags (`Free`, `Locked`, `Modified`, `Checked`, `Orientable`, `Closed`, `Infinite`, `Convex`) and
  `ChildCount`;
- `BrepTVertex`: `Tolerance`, `Pnt`, `Points`, `PointRepresentations`, `EmptyCopy`;
- `BrepTEdge`: `Tolerance`, `SameParameter`, `SameRange`, `Degenerated`, `Curves`, `CurveRepresentations`, `EmptyCopy`;
- `BrepTFace`: `Surface`, `Location`, `Tolerance`, `NaturalRestriction`, `Triangulation`, `Triangulations`,
  `TriangulationList`, `TriangulationCount`, `ActiveTriangulation`, `EmptyCopy`;
- `TopoDS_Iterator` (so `TopExp_Explorer`, `TopExp.MapShapes` and everything walking children).

`BRep_Tool` reads through these, so it needs no change, and neither does code that reads a TShape's data directly. Code
that reads `TShape._shapes` itself (section 3's 46 sites) sees the frozen TShape's own children.

## 5. The pilot

ShapeFix_Edge's public twins (`AddPCurveFixed`, `VertexToleranceFixed`, `SameParameterFixed`, `RemovePCurveFixed`,
`Reversed2dFixed`, `AddCurve3dFixed`, `RemoveCurve3dFixed`) run OCCT's in-place fixes on the caller's frozen edge in an
overlay, with ShapeFix_ShapeTolerance's twins (`ToleranceLimited`, `WithTolerance`), where both prototypes were tried
(decision 1360). Their tests are unchanged and stay at level 1; a new test compares each with the copy-at-entry path,
value for value and sharing for sharing (decision 1363).

## 6. How 10H2 to 10H6 convert a site

1. At the public entry, replace `new CopyAtEntry(input, context)` (or `ExactCopyForWrite` and `KeepUnchanged…`) by
   `using var overlay = WorkingOverlay.Begin(input, context);` and run the algorithm on the input itself: drop
   `entry.Shape()` and `Copied(…)`; the caller's sub-shapes are the algorithm's.
2. At the end, `Shape result = overlay.Finish(result, others);` with everything the algorithm hands out (generated
   shapes, free edges, a sweep's first and last shapes): each must pass through `Finish` or `overlay.Final(…)`, or it
   holds the caller's TShapes without the edits. Then freeze as before.
3. Histories: the algorithm's own history is already over the caller's sub-shapes; `overlay.Composed(history)` maps
   its results and adds the in-place changes. `overlay.History()` alone is the direct history of the in-place edits.
4. Remove the module's `// OCCT.NET: frozen` workarounds the overlay makes unnecessary, and its direct `_shapes` edits
   and representation edits on read-only lists (section 3).
5. Tests stay at level 1, or move to level 2 with a reason (plan 3.2). Recount with `tools/overlay-survey.py`.
6. An algorithm that other operations run on shapes they work on (a sweep, a loft's generator, a maker) opens its
   overlay with `WorkingOverlay.BeginUnlessWriting` (10H3's `BeginUnlessInside`, merged into it by 10H6): inside an
   operation's unfinished overlay it writes into that one, as
   OCCT writes in place, and the caller assembles the writes (decision 1411). A functional twin, or the change of a
   result already handed out, keeps its own overlay. A rerouted representation edit asks `ChangeCurves()` of a frozen
   edge the overlay takes (`!WorkingOverlay.IsLocked`), as BRepLib's `SameParameter` of an edge now does.

## 7. What 10H4 added for healing (decisions 1393 to 1399)

- **In place inside another overlay.** OCCT's in-place tools call each other (ShapeFix_Shape → Solid → Shell → Face →
  Wire; ShapeProcess → ShapeFix; the dividers → ComposeShell): the outer one opens the overlay, and an inner one that
  finds an overlay taking writes (`WorkingOverlay.Writing`) runs in place in it (`BeginUnlessWriting`, `ForTool`), as
  OCCT's writes into the shapes the outer one works on. Functional twins (`…Fixed`, `…Copied`) always open their own.
- **Tools with several public calls** (a load, fixes, a result: ShapeFix_Face, _Wire, _WireVertex, _Wireframe,
  _FixSmallFace, _ComposeShell) keep a `Detached` overlay from the first fix to the result, made current only inside
  each call (`Enter`), so nothing stays open between calls. Between a fix and the result the tool's shapes show the
  changes only to its fixes: read the result.
- **A twin's unfrozen input** (a compound a caller built) is copied where unfrozen (`WithUnfrozenCopied`), and the
  copies left equal are the input's again (`UncopiedWhereUnchanged`), so a twin never writes into a caller's shape.
- **The checks that refuse a write** ask `IsReadOnly`, not `IsFrozen`: inside an overlay the write goes where OCCT's
  does, as it went into the copies. Code that edits representations read through `Curves()` needs the same care
  (BRepLib's SameParameter, decision 1396): a frozen edge's own list is read-only, and its edits fail or are lost.

## 8. What 10H5 added for offsets, fillets and features (decisions 1419 to 1427)

- **An unfrozen input of a maker** (a shell a caller built of frozen faces) is worked on with its unfrozen TShapes copied
  (`WithUnfrozenCopied`, as 10H4's twins), so the operation never writes into it; the history maps the caller's
  unfrozen sub-shapes onto their copies (MakeOffset's `InternalCopied`).
- **Builders with several public calls and no single result call** (the fillet builder: `Add`, `Compute`, `Simulate`;
  LocOpe's operations and BRepFeat's features: `Init`, `Add`, `Perform`, the results) keep a detached overlay and
  enter it in each call that writes or reads what was written; the first result finishes it, and a later computation
  starts a new one on the caller's shape as given.
- **Operations sharing one overlay**: LocOpe's working shapes are an overlay shared by an operation and the operations
  it runs (the gluer's wires on the shape, splitter and generator); an operation run inside another's overlay borrows it
  and leaves it to finish (`ForTool`).
- **`BrepBuilder`'s `Make…` refusals ask `IsLocked`**: OCCT refuses to make a new TShape into a variable holding a
  locked shape; inside an overlay a frozen one is not locked, as its copy wasn't.
- **Construction's copy at entry inside an overlay** copies the caller's frozen sub-shapes it is given, where on the
  copies it wrote in place: a face BRepLib_MakeFace builds on a caller's edges then holds copies beside the caller's
  own. MakeOffset's thickening mode and the simple offset's solid stay on their copies until construction runs in an
  overlay (10H3); 10H6 converts them.

## 9. The finished state (unit 10H6, decisions 1443 to 1451)

No algorithm copies its input at the entry any more, and no machinery for it is left. Every operation that edits the
topology it is given runs OCCT's in-place code on the caller's shape in a working overlay, and its result holds new
TShapes only for what changed.

- **The last sites.** MakeOffset's thickening mode and the simple offset's solid run in overlays: their walls, made by
  BRepLib_MakeFace on the caller's free edges, write into the same overlay and share those edges. HLR's outliner
  writes its vertices' representations in an overlay that spans the outlining and HLRBRep_ShapeToHLR's reads of it,
  so the data structure holds the caller's own sub-shapes and HLRBRep_HLRToShape finds them as themselves; the
  reflect lines' SameParameter, which writes into the caller's vertices the 3D edges share, runs in one too. The STEP
  reader's regularity and ShapeProcess's modifiers ask `WorkingOverlay.IsWritableInPlace` (no frozen TShape: ported
  code's own shape) where they asked `HoldsFrozen`.
- **Meshing** (decision 1444). The mesher runs on a frozen shape in an overlay of its own (nested in any open one,
  as the copy was: meshing gives a result, it doesn't write into an operation's shapes), writing the triangulations
  and polygons into working copies of the faces and edges; `Shape()` is the assembled result, sharing every
  sub-shape meshing left alone, and `History()` the overlay's. That is also the way to 10L's `ShapeMesh` (decision
  1349, question 59): mesh in an overlay, read the meshes back while it is open, and drop it unfinished; the faces
  never hold the triangles and the caller's shape is untouched. BRep_Builder's `…Unfrozen` forms, which the survey
  counted as frozen workarounds, are the mesher's attach of a mesh left unfrozen until the shape is published
  (decision 488): they concern the mesh's freezing, not the face's, and stay, renamed `UpdateFaceMesh` and
  `UpdateEdgeMesh`. Making every attach leave the mesh unfrozen was tried and dropped: design 4.1 freezes geometry
  when it is attached, and a triangulation shared by two faces keeps its purpose because it is frozen
  (`FreezingTests`). 10L's `ShapeMesh` retires them with the faces' triangles.
- **One way in** (decision 1446). `BeginUnlessInside` and `BeginUnlessWriting` were the same rule (an unfinished
  overlay is current here) under two names: `BeginUnlessWriting` stays. `WorkingOverlay`'s API is: `Begin` (an
  overlay of the operation's own), `BeginUnlessWriting` (or join the one that takes writes), `Detached`/`ForTool` with
  `Enter` (a tool whose work spans several calls), `Finish`, `Final`, `History`, `Composed`, `Images`, `Initial`; the
  checks `IsReadOnly` (the rule), `IsLocked` (BRep_Builder's: the flag or `IsReadOnly`), `TakesWrite` and
  `IsWritableInPlace`; `WithUnfrozenCopied`, `UncopiedWhereUnchanged` and `CopiedIn` for an unfrozen input; and
  `Current`, `Writing`, `Parent`, `WorkingCount`, `IsFinished` and `AttachContext`. `Input()` went: nothing used it.
- **The rule protecting published shapes** (decision 1447) is one check: `IsReadOnly`, which BRep_Builder's refusals and the port's
  write checks ask, and which the TShapes' setters apply through `TryWritable` and `CheckMutable`. A write outside an
  overlay into a frozen shape still throws, as it always did.
- **The revolution's copy (unit 10L5, decision 17301).** 10L2's public `Revolve` passed `copy: true` because, without,
  a published profile's history came back empty, and decision 1526 took that for a copy at entry the survey had
  missed. It wasn't one: `ApiMakeRevol` sweeps the caller's profile in an overlay (10H3), its tables keyed by the
  caller's sub-shapes. The empty history is OCCT's: `BRepPrimAPI_MakeRevol::Generated` answers only for a sub-shape
  the result itself uses, and a whole turn without copy drops the profile's edges at right angles to the axis. The
  public operation runs without copy and asks the engine for those edges' faces directly; the engine's history stays
  OCCT's. No other maker had a copy left: copy at entry is still at none.
