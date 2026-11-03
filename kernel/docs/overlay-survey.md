# The overlay survey: copy at entry, and the in-place edits behind it

*Unit 10H1, 3 November 2026. The work list for units 10H2 to 10H6 (phase10-plan.md, section 2.4). Counted at Kerf
`main` db338009 plus 10H1, with `tools/overlay-survey.py` (run `python -I tools/overlay-survey.py src --files` from
`kernel/` to recount). Decisions 1359 to 1364. Recounted by unit 10H2 after the Booleans (decisions 15701 to
15707): the counts below are after 10H2.*

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

## 1. Copy at entry: 105 sites

Before 10H1 there were 113 by 10H1's count. The pilot (section 5) removed 3: ShapeFix_Edge's twins (1) and
ShapeFix_ShapeTolerance's (2). 10H2 found that the count missed the target-typed constructor (`CopyAtEntry entry =
new(input)`): with it, 119 were left after 10H1 (Booleans 14, construction 38, features 7). 10H2 removed the Booleans'
14.

| Kind | Booleans | Construction | Healing | Offsets | Fillets | Features | Other | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `new CopyAtEntry(…)`, `CopyAtEntry x = new(…)` | 0 | 6 | 19 | 4 | 1 | 3 | 0 | 33 |
| `ExactCopyForWrite` | 0 | 6 | 3 | 0 | 0 | 1 | 2 | 12 |
| `KeepUnchanged`, `KeepUnchangedIn` | 0 | 8 | 3 | 0 | 0 | 1 | 1 | 13 |
| `KeepUnchangedGenerating` | 0 | 4 | 0 | 0 | 0 | 0 | 0 | 4 |
| `CopyFrozenForWrite` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `HoldsFrozen` | 0 | 5 | 10 | 1 | 1 | 2 | 4 | 23 |
| Frozen workarounds (`…OfFrozen`, `…Unfrozen`, `UnfreezeResult`) | 0 | 9 | 0 | 0 | 0 | 0 | 11 | 20 |
| **Total** | **0** | **38** | **35** | **5** | **2** | **7** | **18** | **105** |

A site is a call; an algorithm's copy at entry is usually two or three of them (`HoldsFrozen`, the copy, the give-back).
By algorithm, the plan's "about 45" holds.

### The work list, by file

- **Booleans**: none left (10H2). They were 14 sites in 9 files: `BOPAlgo_PaveFiller_10.cs` 1, `BuilderTools.cs` 1,
  `CellsBuilder.cs` 2, `CheckerSI.cs` 1, `MakeConnectedAlgorithm.cs` 1, `MakePeriodicAlgorithm.cs` 5, `BRepAlgo.cs` 1,
  `FaceRestrictor.cs` 1, `WireToFace.cs` 1.
- **Construction** (38 sites, 18 files): BRepFill `AdvancedEvolved.cs` 2, `Evolved.cs` 2, `Fill.cs` 7, `Generator.cs` 2;
  BRepLib `BRepLib.Twins.cs` 4,
  `LibraryMakeFace.cs` 3, `LibraryMakePolygon.cs` 2, `LibraryMakeShape.cs` 1, `LibraryMakeSolid.cs` 1,
  `LibraryMakeWire.cs` 3; BRepOffsetAPI `ApiMakePipe.cs` 1, `ThruSections.cs` 2; BRepPrimAPI `ApiMakePrism.cs` 1,
  `ApiMakeRevol.cs` 1, `MakeSweep.cs` 3; `Projection.cs` 1; BRepSweep `Rotation.cs` 1, `Translation.cs` 1. The prisms and
  revolutions are decisions 468 and 1102's unshared faces.
- **Healing** (35 sites, 20 files): `Sewing.cs` 3, `ShapeBuild_Edge.Twins.cs` 2, `ShapeConstruct.cs` 2,
  `ComposeShellFixer.cs` 2, `EdgeConnectFixer.cs` 1, `FaceConnectFixer.cs` 1, `FixSmallFaceFixer.cs` 1,
  `FreeBoundsFixer.cs` 1, `ShapeFix.Twins.cs` 3, `ShapeFix_Face.CopyAtEntry.cs` 2, `ShapeFix_Wire.CopyAtEntry.cs` 2,
  `ShapeFixer.cs` 1, `ShellFixer.cs` 2, `SolidFixer.cs` 1, `WireVertexFixer.cs` 2, `WireframeFixer.cs` 2,
  ShapeProcess `OperLibrary.cs` 2, `ShapeContext.cs` 3, ShapeUpgrade `ShapeDivide.cs` 1, `UnifySameDomain.cs` 1.
- **Offsets** (5 sites, 4 files): `MakeOffset.cs` 1, `MakeSimpleOffset.cs` 1, `Offset.cs` 2, BiTgte `OffsetBlend.cs` 1.
- **Fillets** (2 sites, 2 files): ChFi2d `PlanarBuilder.cs` 1, ChFi3d `Builder_1.cs` 1.
- **Features** (7 sites, 4 files): `MakeLinearForm.cs` 1, `RibSlot.cs` 1, `DraftAngle.cs` 2, LocOpe `WorkingShapes.cs`
  3.
- **Other** (18 sites, 5 files): XSControl `TransferReader.cs` 2, BRepMesh `IncrementalMesh.cs` 3 and `ShapeTool.cs` 6,
  HLRTopoBRep `OutLiner.cs` 2, `BrepBuilder.cs` 5 (`UpdateVertexOfFrozen`, `UpdateEdgeUnfrozen`,
  `UpdateFaceUnfrozen`: the mesher's and sweeps' workarounds). These belong to no 10H unit: 10H6 takes them with the
  helpers.

The helpers themselves go last (10H6): `CopyAtEntry` (301 lines), `BrepTools.ExactCopyForWrite`, `KeepUnchanged`,
`KeepUnchangedIn`, `CopyFrozenForWrite` and `HoldsFrozen` (`BRepTools.ExactCopy.cs`, `BRepTools.CopyFrozen.cs`, 717
lines), and `LockedShapeException`/`FrozenShapeException` where nothing throws them any more. `BrepTools.CopyData`,
`CopyFlags` and `IsSameData` stay: the overlay uses them (decision 1361).

## 2. Frozen-shape checks: 221 sites

| Kind | Booleans | Construction | Healing | Offsets | Fillets | Features | Other | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `.IsFrozen` tests (topology and geometry) | 1 | 21 | 23 | 1 | 4 | 9 | 38 | 97 |
| `.Locked()` tests | 0 | 0 | 0 | 2 | 0 | 0 | 15 | 17 |
| `// OCCT.NET: frozen` markers | 1 | 33 | 60 | 0 | 3 | 0 | 10 | 107 |
| **Total** | **2** | **54** | **83** | **3** | **7** | **9** | **63** | **221** |

The Booleans' two left are the pave filler's: the walk deciding the non-destructive mode outside an overlay
(`PaveFiller_10`, decision 15701) and the skipped no-op tolerance write of `PaveFiller_6` (decision 875), which the
general fuse needs because it runs outside any overlay.

The markers are the places where the port departs from OCCT because an input is frozen: a write skipped when it would
change nothing, a new TShape made instead of an edit, a copy of geometry before an edit. Inside an overlay most are no
longer needed; each unit reviews its module's and reverts to OCCT's code where the overlay takes the write. Geometry's
`IsFrozen` tests (a curve copied before an edit) are 10G's and stay. `BrepBuilder`'s 29 `Locked()` tests now ask
`WorkingOverlay.IsLocked` (an overlay takes the write); the 5 left in its `Make…` methods refuse to make a new TShape into
a variable holding a locked shape, as OCCT, and are untouched.

## 3. The kinds of in-place edit

Every call is counted, including the many into topology the algorithm built itself (most of them: an edge built, then
given its pcurves). Only a fraction writes into an input; that fraction is what copy at entry protects today, and what
the overlay takes.

| Kind | Booleans | Construction | Healing | Offsets | Fillets | Features | Other | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Vertex point, tolerance or parameter (`UpdateVertex`) | 22 | 59 | 96 | 41 | 9 | 30 | 17 | 274 |
| Parameters moved between edges (`Transfert`) | 1 | 2 | 0 | 0 | 0 | 0 | 0 | 3 |
| Edge 3D curve, pcurve, polygon or tolerance (`UpdateEdge`) | 25 | 108 | 110 | 75 | 4 | 15 | 55 | 392 |
| Edge range (`Range`) | 13 | 85 | 62 | 76 | 6 | 20 | 38 | 300 |
| Edge flags (`SameParameter`, `SameRange`, `Degenerated`) | 6 | 67 | 73 | 42 | 3 | 4 | 13 | 208 |
| Edge continuity (`Continuity`) | 0 | 24 | 2 | 9 | 2 | 4 | 17 | 58 |
| Face surface, tolerance or mesh (`UpdateFace`) | 0 | 3 | 1 | 7 | 0 | 0 | 14 | 25 |
| Face natural restriction | 1 | 1 | 2 | 0 | 0 | 0 | 3 | 7 |
| BRepLib's in-place statics (`SameParameter`, `UpdateTolerances`, `BuildCurves3d`, `EncodeRegularity`, …) | 8 | 29 | 11 | 42 | 5 | 5 | 3 | 103 |
| Sub-shapes added (`Builder.Add`) | 196 | 363 | 289 | 206 | 34 | 163 | 223 | 1,474 |
| Sub-shapes removed (`Builder.Remove`) | 5 | 14 | 10 | 11 | 2 | 2 | 4 | 48 |
| Shape flags set (`Free`, `Closed`, `Orientable`, `Modified`, `Checked`, `Convex`, `Infinite`) | 13 | 67 | 18 | 21 | 0 | 5 | 147 | 271 |
| TShape data written directly (`ChangeCurves`, `ChangePoints`, the BRep TShapes' setters) | 15 | 17 | 22 | 13 | 0 | 1 | 59 | 127 |
| Child lists edited directly (`TShape._shapes`) | 5 | 2 | 8 | 0 | 0 | 1 | 29 | 45 |
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
| Triangulations and polygons | `UpdateFace(f, triangulation)`, `UpdateEdge(e, Polygon3D \| PolygonOnTriangulation \| Polygon2D…)`, their `…Unfrozen` forms | `BrepTFace.Triangulation(…)`, `Triangulations(…)`; `ChangeCurves()` | Taken (the `…Unfrozen` forms are the mesher's workaround: 10H6 or the meshing unit retires them) |
| Face surface, location, tolerance, restriction | `UpdateFace(f, s, l, tol)`, `UpdateFace(f, tol)`, `NaturalRestriction` | `BrepTFace`'s setters | Taken |
| Sub-shapes added or removed | `Builder.Add`, `Builder.Remove` | `TShape._shapes`, `Modified` | Taken: the working copy's child list |
| Shape flags | `Shape.Free(bool)`, `Closed(bool)`, … | `TShape`'s flag setters | Taken |
| Child lists edited directly | none: `tShape._shapes.Append/Clear` | `TShape._shapes` | **Not taken**: 45 sites, 16 of them in algorithms (`Sewing`; `LibraryMakeFace`; LocOpe `WorkingShapes`; ShapeFix `FaceFixer`, `WireVertexFixer`, `ShapeFix_Wire.CopyAtEntry`; the rest are the copy helpers and the hooks themselves; BOPTools' 5, in `ConnexityBlock` and `ToolSet`, are those classes' own fields named `_shapes`, not a TShape's). 10H2 removed TopOpeBRepBuild `WireToFace`'s, the only one in the Booleans. Each unit reroutes its module's through `Builder` or `WorkingOverlay.ForWrite`, or shows they act on unfrozen TShapes only |
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
