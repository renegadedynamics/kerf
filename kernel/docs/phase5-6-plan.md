# Phases 5 and 6 plan: construction and Booleans

This plan splits phases 5 and 6 into work units that can be ported in parallel. It is one plan for two phases because
the owner's sequencing interleaves them, and their dependencies cross in both directions:

1. **Phase 5's surface/surface intersections first** (IntPatch, IntWalk, IntPolyh, GeomInt, ApproxInt, IntStart, the
   rest of IntSurf and IntImp, BRepApprox).
2. **Then phase 6's Booleans** (TKBO: IntTools, BOPDS, BOPTools, BOPAlgo, BRepAlgoAPI), **with build123d.NET's
   clean-up** (`ShapeUpgrade_UnifySameDomain`) and the ShapeFix, ShapeAnalysis and sewing pieces its Boolean and
   face-making paths call.
3. **Then the rest of phase 5**: sweeps, lofts and filling (GeomFill, Law, AppBlend, BRepFill, Plate, NLPlate,
   GeomPlate and the BRepOffsetAPI wrappers build123d.NET calls), constrained 2D geometry (GccAna, Geom2dGcc, GccInt,
   GccEnt), hatching, LocalAnalysis, FairCurve, and two 2D pieces pulled forward from phase 7: ChFi2d and the 2D wire
   offset.

Why one document:

- `IntTools_FaceFace` (phase 6) is built on `GeomInt_IntSS` (phase 5), and `ShapeUpgrade_UnifySameDomain` (pulled
  into phase 6) on `IntPatch_ImpImpIntersection` (phase 5).
- `BRepFill_Sweep` (phase 5) calls `BOPTools_AlgoTools` and `IntTools_Context`, and `BRepFill_TrimShellCorner`
  (every pipe shell with corners) runs `BOPAlgo_PaveFiller` and `BRepAlgoAPI_Section` (phase 6). The sweeps can't
  precede the Booleans.
- `BOPTools_AlgoTools3D` and `IntTools_Context` (phase 6) use phase 5's hatcher (`Geom2dHatch_Hatcher`).

So one set of waves, with unit IDs that keep each unit's phase (5A..., 6A...).

The milestones (section 5) are:

- surface/surface intersections as native OCCT computes them,
- **build123d.NET's algebra-mode Boolean tests' operations reproduce native results through BRep bytes**,
- Booleans in parallel give the serial result,
- the corpus's Boolean, sewing and healing cases, then its sweep, loft, filling and constraint cases,
- build123d.NET's extrusions, pockets, Booleans, clean-up, sweeps, lofts, fillings, constrained 2D objects and 2D
  fillets and offsets can run on the port.

The rules are phase 3's ([phase3-design.md](phase3-design.md)) and phase 4's: results freeze on publication, writers
copy first (4.6), exact copy and `KeepUnchanged` (decision 416), makers' `Internal*` accessors (455), one parallel
helper (`OSD_Parallel`, 522), a parallel run gives the serial result (490). Section 2 settles what is specific to
intersections, Booleans, healing and sweeps.

**Size.** About **259,000 lines of OCCT for phase 5** (82,600 of them the surface/surface intersections) and
**135,000 for phase 6** (TKBO 71,200, healing and sewing 61,400, TKernel's reports 2,500): **about 395,000 lines**,
plus about 7,000 lines of C# for Draw-lite and the oracle harnesses. Phases 0 to 4 ported 744,000. Sizes are physical
lines of `.hxx/.cxx/.lxx/.pxx/.gxx`, comments included, as `tools/port-status` counts them (a `.gxx` generic counts
once in its package and once per instantiation file, as port-status does). Dependencies were taken from identifiers
used in code bodies, after removing comments, string literals, `#include`s and `OCCT_DEBUG` code (a throwaway closure
script), and checked at the call sites.

## 1. Scope

### 1.1 What phases 5 and 6 start from

Phases 0 to 4, merged on `main`:

- **TKMath** (99.9%), the TKernel parts used so far (Message_Algorithm and the messenger, NCollection's maps, trees
  and arrays, `OSD_Parallel`), **TKG2d, TKG3d, TKGeomBase** (all but GCE2d's 15 typedef headers).
- **Phase 3:** TKBRep (without BRepGraph), the TKTopAlgo makers, BRepLib, BRepCheck, BRepClass(3d), BRepExtrema,
  BRepGProp, BRepBndLib; TKGeomAlgo's 2D and curve/surface intersections (IntRes2d, IntCurve, Geom2dInt, Intf,
  IntCurveSurface, IntImpParGen, TopTrans, TopClass), IntSurf's Quadric and QuadricTool (6 of 32 files), IntImp's
  curve/surface generics (5 of 11), GeomAPI and Geom2dAPI but for the files below; TKPrim.
- **Phase 4:** TKMesh, TKXMesh, ShapeAnalysis's wire analysis (ShapeExtend, ShapeAnalysis Wire, WireOrder, Edge,
  Curve, Surface, TransferParameters(Proj); ShapeBuild Edge and Vertex).
- `BRepTools.ExactCopyForWrite`/`KeepUnchanged(In)`, `BRepTools_History`, `BRepTools_ReShape`, the freezing of
  published graphs, Draw-lite with its corpus runner and `cases.list` grids.

### 1.2 Phase 5 inventory

**Surface/surface intersections (82,600 lines; wave 1 to 4).**

| Package | Lines | Classes (lines) |
|---|---:|---|
| IntSurf, the rest | 1,824 | LineOn2S 374, PntOn2S 285, PathPoint 218, Transition 213, InteriorPoint 185, IntSurf (package) 148, PathPointTool 143, InteriorPointTool 99, Couple 83, TypeTrans, Situation, Allocator |
| IntImp, the rest | 1,030 | `.gxx` generics: Int2S 361, ZerParFunc 443, ZerImpFunc 225 |
| IntStart | 1,715 | `.gxx` generics: SearchOnBoundaries 1,303, SearchInside 354, SITopolTool 58 |
| IntPatch | 42,472 | **ImpImpIntersection 9,938** (quadric/quadric, the analytic case); PrmPrmIntersection 4,503 (+T3Bits 116); ImpPrmIntersection 4,053; Intersection 2,826; WLineTool 2,241; LineConstructor 2,137; WLine 1,681; GLine 1,432; RstInt 1,395; SpecialPoints 1,287; InterferencePolyhedron 1,270; ALineToWLine 1,192; ALine 928; Polyhedron 858; RLine 852; Point 710; HInterTool 562; HCurve2dTool 383; the instantiations TheIWLineOfTheIWalking 369, TheIWalking 343, TheSOnBounds 204, ThePathPointOfTheSOnBounds 198, TheSegmentOfTheSOnBounds 161, TheSurfFunction 161, TheSearchInside 141; Line 295; PolyArc 290; PolyLine 282; PolyhedronBVH 269; PointLine 247; CSFunction 212; BVHTraversal 210; ArcFunction 200; Polygo 147; CurvIntSurf 146; PolyhedronTool 143; IType, SpecPntType, SearchPnt |
| IntWalk | 8,124 | PWalking 4,487; IWalking (`.gxx`) 3,211; TheInt2S 189, TheFunctionOfTheInt2S 150; StatusDeflection, the two vectors |
| IntPolyh | 7,806 | MaillageAffinage 4,257, Triangle 890, Intersection 771, StartPoint 466, Point 306, Tools 252, Array 221, SectionLine 209, Couple 143, Edge 115, the array typedefs |
| ApproxInt | 4,210 | ImpPrmSvSurfaces 1,020, KnotTools 982, MultiLine 778, Approx 751 (`.gxx`), PrmPrmSvSurfaces 373, MultiLineTool 186, SvSurfaces 120 |
| GeomInt | 9,217 | IntSS 2,071 (with `_1`), LineTool 1,193, LineConstructor 1,119, ParameterAndOrientation 126, package 92; WLApprox 267 and its 24 instantiations of AppParCurves/ApproxInt/IntImp (about 4,350) |
| BRepApprox | 5,269 | Approx (ApproxInt over BRepAdaptor) and its instantiations, SurfaceTool 459, ApproxLine |
| GeomAPI_IntSS | 144 | the public surface/surface intersector |

`IntPatch_ImpImpIntersection` is also what `ShapeUpgrade_UnifySameDomain` uses to merge coaxial cylinders and the
like, so unit 5B is on phase 6's critical path too. BRepApprox is reached only from TopOpeBRepTool (the 2D offset,
unit 5O1); it is ported with the other ApproxInt instantiations because it is the same code.

**Hatching and the API remainder (7,700 lines).** Hatch 805, HatchGen 1,368, Geom2dHatch 3,936 (needed by
`BOPTools_AlgoTools3D::PointInFace` and `IntTools_Context::Hatcher`); `GeomAPI_PointsToBSplineSurface` 1,194 (29 uses in
build123d.NET); GCE2d's 15 typedef headers 432. (`Geom2dAPI_Interpolate` and `PointsToBSpline`, 1,685, go with the
healing foundations, unit 6H1, whose `ShapeConstruct_ProjectCurveOnSurface` calls them.)

**Constrained 2D geometry (33,700).** GccAna 16,486 (Circ2d3Tan 4,361, Circ2d2TanOn 3,716, Circ2d2TanRad 2,679,
Circ2dTanOnRad 1,570, the bisectors used by MAT2d, ...), GccEnt 556, GccInt 670, Geom2dGcc 15,966 (the `Geo` and `Iter`
solvers, QualifiedCurve, QCurve, CurveTool). build123d.NET binds seven Geom2dGcc solvers and QualifiedCurve
(`ConstrainedLines.cs`, 58 uses of QualifiedCurve).

**Sweeps, lofts and filling (about 87,000).**

| Package | Lines | Part ported | Notes |
|---|---:|---|---|
| Law | 5,409 | all | BSpline 2,328, Interpolate 698, package 507, BSpFunc 486, Composite 356, Interpol, Linear, KnotSplitting, Constant, Function, S |
| GeomFill | 33,355 | all | the sweep half 22,234 (trihedrons: Frenet 1,272, CorrectedFrenet 1,223, Darboux, ConstantBiNormal, Fixed, DiscreteTrihedron, DraftTrihedron, GuideTrihedronAC/Plan; location laws: LocationGuide 1,692, LocationDraft 991, CurveAndTrihedron; sections: NSections 1,266, SectionPlacement 1,083, UniformSection, EvolvedSection; Sweep 1,418, Pipe 1,480, SweepFunction, AppSweep, CircularBlendFunc, the convertors) and the filling half 11,121 (Coons, Curved, Stretch, ConstrainedFilling 1,905, BSplineCurves, BezierCurves, Gordon 819 + GordonUtilities 1,729 and NetworkSurface 1,391, new in OCCT 8, Generator, Profiler, AppSurf, SectionGenerator) |
| AppBlend | 1,463 | all | AppSurf (`.gxx`), Approx |
| Plate, NLPlate, GeomPlate | 4,742, 1,464, 6,566 | all | GeomPlate_BuildPlateSurface 3,087 |
| LocalAnalysis | 2,193 | all | used by GeomPlate's error checks |
| FairCurve | 4,481 | all | Batten, MinimalVariation, the energies; needs Law |
| BRepFill (TKBool) | 33,916 | 22,000 | Sweep 4,342, TrimShellCorner 2,957, CompatibleWires 2,738, PipeShell 1,943, Pipe 1,371, Generator 1,351, package 1,232 (`Face`, `Shell`: build123d's ruled faces), Filling 1,100, NSections 965, LocationLaw 888, ShapeLaw 600, SectionPlacement, Section, SectionLaw, the location laws, CurveConstraint, FaceAndOrder, EdgeFaceAndOrder. The offset part goes to 5O2; left for phase 7: Draft 1,077 and DraftLaw (BRepOffsetAPI_MakeDraft), AdvancedEvolved 1,976 (BOP-based evolved solids) |
| BRepOffsetAPI (TKOffset) | 7,762 | 5,300 | MakePipeShell 565, MakePipe 223, ThruSections 1,957, MakeFilling 453, MiddlePath 1,206; with 5O: MakeOffset 615, MakeEvolved 273. Left for phase 7: DraftAngle, MakeDraft, MakeOffsetShape, MakeThickSolid, NormalProjection, FindContigousEdges |
| BRepProj (TKBool) | 407 | all | `BRepProj_Projection` (build123d's projections; needs BRepAlgoAPI_Section) |
| ShapeUpgrade_RemoveLocations | 439 | | BRepFill's `RemoveLocations` |

**The two 2D pieces from phase 7 (section 2.10 recommends both).**

| Piece | Lines | Contents |
|---|---:|---|
| **ChFi2d and `BRepFilletAPI_MakeFillet2d`** | 6,194 | ChFi2d_Builder 2,551, AnaFilletAlgo 1,244, FilletAlgo 1,207, FilletAPI 231, ChamferAPI 219, package, ConstructionError; MakeFillet2d 545. Needs only GccAna's Circ2d2TanRad and Geom2dGcc's (phase 5 anyway). build123d.NET: `Wire.cs` (fillet and chamfer of wires), `Fillet2DTests` |
| **2D wire offset** (`BRepOffsetAPI_MakeOffset`, `BRepFill_OffsetWire`) | ≈ 39,000 | MAT 3,179, MAT2d 5,982, Bisector 7,519, BRepMAT2d 1,642 (the medial axis); BRepFill's OffsetWire 3,075, TrimEdgeTool 859, TrimSurfaceTool 569, MultiLine 907, ApproxSeewing 308, ComputeCLine, OffsetAncestors (6,200); BRepAlgo's package, FaceRestrictor, Loop, Image, AsDes (3,800); **a subset of TopOpeBRepBuild** (`BRepAlgo_FaceRestrictor::Perform` makes faces with `TopOpeBRepBuild_WireToFace`: WireEdgeSet, FaceBuilder, the area/loop builders and classifiers, `Builder::MakeFaces`, and what they call in TopOpeBRepDS_BuildTool and TopOpeBRepTool; estimated 10,000, section 2.11); MakeOffset 615. Optional, cheap afterwards: BRepFill_Evolved 3,644 and MakeEvolved 273 (`evolved`) |

### 1.3 Phase 6 inventory

**TKBO (71,242 lines; GTests 7,470 more).**

| Package | Lines | Classes |
|---|---:|---|
| BOPAlgo | 35,031 | **PaveFiller 11,332** (with `_1` to `_11`: VV, VE, EE, VF, EF, FF interferences, pave blocks, section curves, `_10` the locked-argument check that makes it non-destructive); Builder 4,082 (`_1` to `_4`); RemoveFeatures 2,295; Tools 2,177; BOP 1,913; WireSplitter 1,511; CellsBuilder 1,469; ArgumentAnalyzer 1,225; MakePeriodic 1,210; CheckerSI 1,077; BuilderSolid 1,017; BuilderFace 997; ShellSplitter 764; MakerVolume 701; MakeConnected 666; Section 492; Options 268; Algo 254; CheckResult 221; Splitter 196; BuilderShape, WireEdgeSet, Alerts 147, BuilderArea, msg, ToolsProvider, SectionAttribute, GlueEnum, Operation, CheckStatus |
| IntTools | 18,257 | FaceFace 3,240, BeanFaceIntersector 2,854, EdgeEdge 2,073, WLineTool 1,364, Context 1,306, FClass2d 1,043, EdgeFace 1,037, Tools 993, TopolTool 665, SurfaceRangeLocalizeData 574, package 449, MarkedRangeSet 429, CommonPrt 393, SurfaceRangeSample 314, ShrunkRange 276, Root 272, Curve 216, PntOnFace, PntOn2Faces, Range, CurveRangeSample, CurveRangeLocalizeData, BaseRangeSample |
| BOPTools | 7,980 | AlgoTools 4,622, AlgoTools3D 1,332, AlgoTools2D 1,219, Set, Parallel 187 (the per-thread context functor), PairSelector, BoxSelector, ConnexityBlock, BoxTree, CoupleOfShape |
| BOPDS | 7,225 | DS 2,518, Interf 728, Iterator 595, PaveBlock 554, CommonBlock 439, ShapeInfo, FaceInfo, Curve, SubIterator, Pave, IteratorSI, IndexRange, Tools, Point, CoupleOfPaveBlocks, Pair |
| BRepAlgoAPI | 2,748 | BuilderAlgo 569 (`SimplifyResult` runs UnifySameDomain), Section 548, BooleanOperation 440, Defeaturing 280, Check 272, Splitter 157, Cut, Common, Fuse, Algo |

GTests: BRepAlgoAPI_Fuse 2,051, Cut 1,851 + 1,409, BuilderAlgo 90, Common 32; BOPAlgo_PaveFiller 352, BOP 281;
IntTools_FaceFace 424; their utilities 980. TKBool's GTests add BRepAlgoAPI_Cut 222, Fuse 204, Section 35.

There are **29 parallel loops** (`BOPTools_Parallel::Perform`, which gives each thread its own `IntTools_Context`) in
PaveFiller, Builder, BuilderSolid, ShellSplitter, WireSplitter, CheckerSI, RemoveFeatures, Tools, BOPDS_Iterator and
BOPTools_AlgoTools.

**Healing and sewing pulled forward from phase 7 (61,400).** What build123d.NET's Boolean path and its face making
call, and what they reach:

| Piece | Lines | Why |
|---|---:|---|
| `ShapeUpgrade_UnifySameDomain` | 4,988 | build123d's `Clean()` after every Boolean (`Shape.Booleans.cs`, `Shape.Transforms.cs`); BRepAlgoAPI's `SimplifyResult`, BOPAlgo_CellsBuilder and RemoveFeatures call it too. Needs IntPatch_ImpImpIntersection (5B) |
| ShapeFix, all of it | 31,776 | UnifySameDomain reaches ShapeFix_Face, Wire, Edge, Shell and ShapeFix's statics (24,900 of it). build123d.NET calls `ShapeFix_Shape` (wires before `MakeFace`, faces after it, `TopoUtil.Fix`), `ShapeFix_Solid.SolidFromShell` (`Solid.cs`, `Shell.cs`), `ShapeFix_Face`, `ShapeFix_Wireframe` (`Wire.Fix`). The remaining 6,800 (FixSmallFace, FixSmallSolid, FaceConnect, EdgeConnect, SplitCommonVertex, FreeBounds, Wireframe) complete the package for `fixshape`, `fixsmall` and `fixwgaps` |
| ShapeAnalysis, the rest | 7,700 | FreeBounds 945 and Shell 410 (ShapeFix_Shell/Solid; build123d binds both), ShapeContents (`statshape`, 1,950 corpus cases), ShapeTolerance, CheckSmallFace, WireVertex, Geom, BoxBndTree, CanonicalRecognition |
| ShapeConstruct | 5,043 | ProjectCurveOnSurface 3,085 (ShapeFix_Edge's missing pcurves), Curve, package, MakeTriangulation |
| ShapeExtend, the rest; ShapeBuild_ReShape | 1,900; 459 | CompositeSurface, Explorer, MsgRegistrator; the healing context |
| `Geom2dAPI_Interpolate`, `PointsToBSpline` | 1,685 | ProjectCurveOnSurface's interpolation |
| `BRepBuilderAPI_Sewing` (+ FastSewing, VertexInspector, BndBoxTreeSelector) | 7,700 | ShapeAnalysis_FreeBounds sews free edges; build123d.NET sews shells (`Shell.cs`, `TopoUtil.SewFaces`); `sewing` is in 1,331 corpus cases |

Left for phase 7: the rest of ShapeUpgrade (11,400: the divide and convert tools behind `DT_*`, which block 1,600
heal-group cases), ShapeCustom (6,081; build123d's `ShapeCustom.BSplineRestriction` needs 6,300 with ShapeConstruct's
parts already here), ShapeProcess(API), ShapeAlgo, SHMessage.

**TKernel (2,500).** `Message_Report`, `Message_Alert`, `AlertExtended`, `CompositeAlerts`, `Attribute*`, `Level`,
`PrinterToReport`; `TopoDS_AlertWithShape`, `AlertAttribute`. BOPAlgo's warnings and errors are alerts in a report
(`HasErrors`, `DumpErrors`, `BOPAlgo_AlertSelfInterferingShape`, which build123d.NET tests for).

### 1.4 Dependencies outside phases 0 to 4

| Dependency | Where | Plan |
|---|---|---|
| `OSD_ThreadPool`, `OSD_Thread::Current()` | `BOPTools_Parallel` keys a context per thread id | Onto `OSD_Parallel` (decision 522), with the per-thread context kept (decision 580) |
| `Message_Report` and the alerts | BOPAlgo_Options, every BOPAlgo, BRepAlgoAPI | Unit 6A (decision 584) |
| `NCollection_IncAllocator`, `BaseAllocator` | BOPDS, BOPAlgo, IntPatch | Dropped, by the rules |
| `TopOpeBRepBuild_WireToFace` (legacy Booleans) | `BRepAlgo_FaceRestrictor`, the 2D offset only | A subset (section 2.11, open question 17) |
| `BOPAlgo_*`, `IntTools_Context`, `BRepAlgoAPI_Section` | `BRepFill_Sweep`, `TrimShellCorner`; `BRepProj_Projection` | Phase 6 before those units |
| `DrawTrSurf`, `Draw` | `IntTools_FClass2d`, `IntPolyh` debug blocks | Left out (debug) |
| `TCollection_AsciiString` | BRepAlgoAPI_BooleanOperation's dump file names | `string` |

Nothing in phases 5 and 6 needs TKOffset beyond the wrappers listed, TKFillet beyond ChFi2d, or TKFeat.

### 1.5 Draw commands and the corpus

A scan of today's corpus (21,068 cases with the `cases.list` grids; **1,834 runnable now**) with a scratch script over
`DrawCorpus.Scan`, crediting each stage with the Draw commands of the files it ports:

| After | Commands | New runnable cases | Of which all data public |
|---|---|---:|---:|
| 5G, surface/surface | `intersect` (used by 697 cases, nearly all with Booleans) | 7 | 5 |
| Phase 6, Booleans only | BOPTest's 91 commands (`bop`, `bopcut`..., `bcut`, `bfuse`, `bcommon`, `bsection`, `bfillds`, `bbuild`, `bsplit`, `baddobjects`..., `bfuzzyvalue`, `bglue`, `bnondestructive`, `brunparallel`, `bsimplify`, `bopcheck`, `bopargcheck`, `mkvolume`, `bcbuild`..., `removefeatures`, `makeconnected`, `makeperiodic`, `edgestofaces`), and `setfillhistory`, `savehistory`, `modified`, `generated`, `isdeleted` | 3,862 | 3,336 |
| + sewing and ShapeAnalysis's commands | `sewing`, `fastsewing`, `statshape`, `freebounds`, `fbclose`, ... | +725 | +521 |
| + ShapeFix's and UnifySameDomain | `fixshape`, `fixsmall*`, `fixwgaps`, `settolerance`, `checkoverlapedges`, `unifysamedom`, ... | +293 | +153 |
| **Phase 6 in all** | | **4,880** | **4,010** |
| Phase 5's construction | `thrusections`, `pipe`, `mksweep`/`addsweep`/`setsweep`/`buildsweep`, `middlepath`, `filling`, `gplate`, `cirtang`, `lintan`, `gcarc`, `fillet2d`, `chfi2d`, `prj`, ... | 861 | 601 |
| The 2D offset | `mkoffset`, `openoffset`, `evolved` | 7 by the scan; **about 550 more** (below) | |

- The phase 6 cases are mostly boolean (bop*_simple 379 each, the 2d, complex, bsection and gdml grids) and sewing
  (tol_0_01, tol_1, tol_100, about 230 each).
- The phase 5 ones are pipe/specific 260, thrusection 206+, sweep and the bugs groups.
- **The offset group's wire grids** (621 cases, 552 with their data) set `command` to `mkoffset` and the group's
  `begin` script branches on it. The scan follows the other branch too (OFFSETSHAPE's `offsetload`/`offsetperform`,
  phase 7), so it sees them blocked. Unit 5W teaches DrawCorpus that branch, as decision 496 did for the mesh group's
  data-file branches.
- **What still blocks most after phases 5 and 6:** the heal group's `DT_SplitAngle` 800 and `DT_ShapeDivide` 783
  (ShapeUpgrade's divide tools, phase 7), `blend` 313, `chamf` 285, `depouille` 106 (phase 7), `xdistcs` 115,
  `stepread` 57 (phase 8).

Draw-lite's new command sources: BOPTest (9,200 lines of C++ over 17 files), BRepTest's History, Sweep, Filling,
Fillet2D and Projection commands, `mkoffset`/`openoffset` of BRepTest_CurveCommands, SWDRAW's ShapeFix, ShapeAnalysis
and `unifysamedom`, GeometryTest's Constraint, Surface (`sweep`, `appsurf`), FairCurve and Continuity commands, and
`intersect`.

## 2. Design questions

### 2.1 Generic instantiations

IntPatch, GeomInt, BRepApprox and IntWalk are largely instantiations of `.gxx` generics:

- IntStart's SearchOnBoundaries and SearchInside, IntWalk's IWalking and IntImp's Int2S, ZerImpFunc and ZerParFunc,
  instantiated in IntPatch (`IntPatch_TheSOnBounds`, `TheIWalking`, ...);
- ApproxInt's Approx and AppParCurves' generics, instantiated in GeomInt (`GeomInt_WLApprox` and 24 classes) and in
  BRepApprox.

They follow decision 142 (AppParCurves) and phase 3's IntCurve: one generic class per `.gxx`, the tool a `struct`
implementing an interface of static abstract members, so the JIT specialises each instantiation as OCCT's `_0.cxx`
files do; the instantiation keeps OCCT's name as a closed generic (`using`-alias or a derived class). AppParCurves'
generics are already ported and are reused. Port-status counts the instantiation files as ported when their
generic is.

### 2.2 Intersection results: parity and comparison

A surface/surface intersection is a cascade, like a mesh:

- start points (IntPolyh's triangle intersections, IntStart's boundary search),
- marching with adaptive steps (IntWalk),
- line splitting and purging (WLineTool),
- approximation (ApproxInt, KnotTools).

One different last bit can move a start point and give a different but equally valid line.

- **Analytic pairs** (IntPatch_ImpImpIntersection: plane, cylinder, cone, sphere, torus) are closed-form with IntAna
  and ElSLib. They are expected bit-identical except where transcendental functions differ (oracle-testing.md).
- **Walking lines** (ImpPrm, PrmPrm) are expected bit-identical where the arithmetic is the same. The sources of
  difference are attributed as in decision 494: transcendentals, `std::sort` order on ties (IntTools_Root's sort,
  IntPolyh's couples; decision 491's MSVC helper), and OCCT's own bugs (reproduced).
- **Comparison** (decision 578): GeomAPI_IntSS's lines through the binding:
  - number and type of lines,
  - each line's curve as BRep text of an edge (poles, knots, weights) or the analytic parameters,
  - every double bit for bit; then the largest point-to-point deviation where they differ.

  BRepAlgoAPI_Section's result compares the same lines through BRep bytes, with pcurves.
- **Targets:** every analytic pair bit-identical or attributed; at least 95% of walking-line pairs bit-identical on the
  oracle's random pairs, the rest same count and type, within the tolerance the intersector reports, and attributed.

### 2.3 Booleans on frozen inputs: non-destructive by construction

OCCT's BOP modifies its arguments by default. It grows the tolerances of vertices and edges, adds pcurves to argument
edges and splits them, unless it is non-destructive. `BOPAlgo_PaveFiller` turns non-destructive by itself when an
argument has a `Locked()` sub-shape (PaveFiller_10). Frozen implies Locked (design 4.3).

**So every Boolean of published shapes is non-destructive, with no new code** (decision 577):

- An argument sub-shape whose tolerance must grow, or that needs a pcurve, is replaced by an `EmptyCopied` copy.
  OCCT's own images and origins record the copy, so `Modified`, `Generated` and `IsDeleted` answer for the caller's
  sub-shapes as native's do.
- The result shares every argument sub-shape the operation didn't touch (unsplit faces, edges and vertices), by
  TShape, as native's does.
- `SetNonDestructive(false)` is honoured only for unfrozen arguments (ported code). On frozen ones OCCT's own check
  overrides it, as for locked shapes.
- **The Free flag** (open question 1) appears in every Boolean result. An unsplit argument face put into the result's
  shell keeps the Free flag it was frozen with, where native clears it. The oracle masks that one column and counts
  it, as in phases 3 and 4. Everything else in the BRep text must match.
- **Results are published frozen.** `BRepAlgoAPI_*` stay OCCT's working objects (decision 414's maker pattern):
  - `Shape()`, `SectionEdges()`, `Modified`, `Generated`, `History()` hand out frozen shapes;
  - internal `InternalShape()` serves ported callers that keep building (BRepFill_TrimShellCorner, RemoveFeatures,
    BRepProj, ...);
  - a functional twin per operation for Bento: `BRepAlgoAPI.Fuse(args, tools, in BRepAlgoAPI_Options, out
    BRepTools_History)`, with an equatable options struct (fuzzy value, glue, OBB, inverted-solid check, parallel,
    simplify, non-destructive) that can key a cache (decision 579).
- **Writes into the result.** BOPAlgo builds its result unfrozen and runs its own post-passes
  (`BOPTools_AlgoTools::CorrectTolerances`, `CorrectShapeTolerances`, `OrientFacesOnShell`, ...) before publishing.
  The only frozen things they can meet are shared argument sub-shapes. In non-destructive mode OCCT routes those
  through copies. Any write that still reaches one throws `TopoDS_LockedShape` at the first test that hits it, and is
  fixed by copying there (listed in decisions.md).
- `BRepAlgoAPI_BuilderAlgo::SimplifyResult` runs UnifySameDomain on its own result (unfrozen), not a copy.

### 2.4 Parallel Booleans and determinism

BOPAlgo runs its intersection and building loops in parallel when `SetRunParallel(true)` (build123d.NET always sets it)
or when the global default is on (`BOPAlgo_Options::SetParallelMode`, false by default).

**The rule** (decision 580): **a parallel Boolean gives the serial Boolean's result, bit for bit, every run**, as
decision 490 for the mesher. BOP is designed for it:

- each loop fills its own element of a vector (`BOPAlgo_VertexEdge`, `BOPAlgo_EdgeEdge`, `BOPAlgo_FaceFace`, ...);
- the results are merged afterwards, serially, in vector order;
- `BOPDS_Iterator` sorts its pairs (`std::stable_sort`).

The port must also:

- **Keep the per-thread context.** `BOPTools_Parallel` gives each thread its own `IntTools_Context` (projectors,
  classifiers, hatchers, boxes, cached per face and edge). The port keys it per worker in `OSD_Parallel`'s loop
  (`ThreadLocal` or one per chunk). A context's caches must not change results:
  - porters audit each cached tool (GeomAPI_ProjectPointOnSurf, Extrema_ExtPS's grid, BRepClass3d_SolidClassifier,
    IntTools_FClass2d, the hatcher) for state carried from one call to the next;
  - where a result depends on what the same context computed before (call history), that is a native
    nondeterminism: it goes in occt-issues.md, and the port either reproduces the serial order or gives each item a
    fresh tool.
- **Not iterate in address order.**
  - TKBO declares 216 unordered shape maps (47 iterated) and 34 unordered pave-block or common-block maps keyed by
    handle.
  - In native OCCT they iterate in address order, which **isn't stable from run to run** even serially (phase 3
    design, section 3).
  - The port's maps iterate in insertion order (decision 18): deterministic, but not native's order.
  - Where the result depends on that order (the order of a result's sub-shapes, which split piece keeps an original
    edge, a tolerance reached through a different vertex first), the site is listed in occt-issues.md.
- **Share no evaluation caches** between loop bodies: GeomAdaptor's B-spline cache (decision 431); each thread's
  context has its own adaptors.
- **Test it.** A determinism test runs every oracle and corpus Boolean serially and in parallel, at several degrees
  of parallelism, repeatedly, and requires identical BRep text and identical bits.

**Native's own nondeterminism is measured first.** The oracle runs each native Boolean:

- five times serially and five times in parallel, in one process and in fresh processes,
- recording whether native's BRep bytes vary.

A case where native varies is compared canonically (2.5), never blamed on the port.

### 2.5 How Boolean results are compared

1. **BRep bytes**, with the Free column masked (2.3) and counted. This is the milestone's measure, and it needs the
   same sub-shapes in the same order.
2. Where native's order is address-driven (2.4), or bytes differ, a **canonical comparison** (decision 581):
   - the same number of each shape type (`checknbshapes`);
   - each sub-shape matched to one of the other side by its geometry (a vertex by its point, an edge by its curve's
     BRep text and range, a face by its surface's text and its edges), then its tolerance and flags;
   - volume, area and centroid (BRepGProp) within 1e-9 relative;
   - both valid (`BRepCheck_Analyzer`, `bopargcheck` on the result).

   A case is reported as bytes-identical, canonically identical, or different (then which sub-shapes differ, by how
   much, and the attributed source).
3. **History.** For every argument sub-shape: `Modified`, `Generated` and `IsDeleted` on both sides, compared through
   the canonical match.

### 2.6 Tolerances, fuzzy values and glue

OCCT's Booleans are tolerance-driven:

- vertices and edges grow their tolerances to cover what they touch;
- `SetFuzzyValue` adds to every tolerance during the operation;
- `SetGlue(GlueShift/GlueFull)` skips the intersection of faces it is told only touch;
- `Precision::Confusion` and BRepLib's precision statics enter many thresholds.

So the result depends on the order in which intersections grow tolerances. That order is the serial merge order of
2.4, which the port keeps. Decisions:

- The options keep OCCT's names, defaults and effects exactly. Bento's caller sets them through the options struct
  (2.3); no option is added or reinterpreted (decision 582).
- The glue options give wrong results on shapes that do more than touch, as in OCCT. The port doesn't second-guess
  them; build123d.NET's `Fuse(glue: true)` keeps its native meaning.
- **Fuzzy Booleans** are tested by the corpus's `bfuzzyvalue` cases (37) and an oracle sweep of fuzzy values on
  near-coincident primitives (offsets of 1e-7 to 1e-3).
- `BOPAlgo_Options::SetParallelMode` and `BRepLib::Precision` stay process-wide statics with internal setters
  (decision 415). Draw-lite's `brunparallel` sets the per-algorithm flag, as DRAWEXE does.

### 2.7 Healing and clean-up on frozen shapes

ShapeFix, ShapeUpgrade and sewing edit the shapes they are given (design 1, family 2). They record the replacements
in a `ShapeBuild_ReShape` context, and ShapeFix also edits geometry (pcurves' poles, `Segment`, `IncreaseDegree`: about
40 sites in ShapeFix and 40 in ShapeUpgrade and ShapeConstruct). Decisions (5008):

- **Copy at the entry, as design 4.6 asks.**
  - `ShapeFix_Shape`, `Solid`, `Shell`, `Face`, `Wire`, `Wireframe`, `UnifySameDomain` and `BRepBuilderAPI_Sewing`,
    given a frozen shape, run on `BRepTools.ExactCopyForWrite` of it.
  - On the result, `KeepUnchanged` gives back the input's own sub-shapes wherever nothing changed. A face that needed
    no fix is the caller's face, and a shape that needed nothing is the input itself, after one copy.
  - Their histories (ReShape's, UnifySameDomain's `History()`, Sewing's `Modified`) are composed with the copy's, so
    they answer for the caller's sub-shapes.
  - build123d.NET's `PrivateCopy(copyGeometry: true)` before ShapeFix then becomes unnecessary.
- **Geometry edits.**
  - A curve or surface reached from a representation is frozen, so ShapeFix's in-place edit of a pcurve throws.
  - Each site copies the curve (`Copy()`, unfrozen), edits the copy and attaches it, which is what OCCT's code does on
    curves it built itself.
  - The sites are listed in decisions.md as they are found. The oracle catches any difference, because native edits
    the shared curve, which only matters to other shapes sharing it.
- **`ShapeUpgrade_UnifySameDomain`.**
  - Its default `SafeInputMode(true)` already avoids modifying the input.
  - `SetSafeInputMode(false)` on a frozen shape behaves as `true` (the copy above).
  - Its result is frozen, and `History()` is a frozen-shape `BRepTools_History`.
- **ShapeFix's statics** (`ShapeFix::SameParameter`, `FixVertexPosition`, `ShapeFix_ShapeTolerance::LimitTolerance`,
  `SetTolerance`) are in-place utilities (design family 1): internal in-place forms plus public functional twins, as
  BRepLib's in unit 3I.
- **Messages.** ShapeExtend's message registrators send to Message's process-wide messenger (decision 493), which
  sends nothing by default. ShapeExtend's `Init` message file stays out, as in phase 4.

### 2.8 Sweeps, lofts and filling that share their inputs

Design family 4.

- `BRepFill_PipeShell` and `BRepFill_Sweep` build new topology. The spine and profiles are explored, and edges of the
  spine are shared into the result where OCCT shares them.
- `BRepFill_Sweep` updates the tolerances and pcurves of edges it builds itself. Where it writes into a caller's edge
  (`BuildBoundaries` on the profile's edges, `PerformCorner`'s trimming, TrimShellCorner's section edges), the
  profile is copied first, as BRepSweep's `SetPCurve` in unit 3M (decision 468), and `KeepUnchangedIn` gives back what
  wasn't touched.
- Each place goes in decisions.md, found by the frozen-input tests, which build every sweep from frozen profiles and
  spines.
- The wrappers' `Generated`, `FirstShape`, `LastShape` and `Profiles` hand out frozen shapes, and their histories map
  the caller's sub-shapes.
- `BRepOffsetAPI_MakeFilling` builds a face from frozen boundary edges and keeps them (the 3I pattern for makers).
- `GeomFill_Gordon` and `GordonUtilities` (new in OCCT 8) have parallel loops; they go through `OSD_Parallel` under
  decision 490's rule.

### 2.9 Performance

Booleans are the slowest common operation in Bento's models. Targets (provisional, open question 22):

- the port's serial Boolean within **2×** native serial, over build123d.NET's algebra-mode operations and the
  corpus's simple and complex grids;
- the parallel speed-up comparable with native's;
- allocation reported.

As in phase 4, allocation is the risk:

- BOP's data structure is large: pave blocks, interference vectors, `NCollection_IncAllocator` in OCCT.
- The porters keep OCCT's preallocation sizes and reuse buffers where OCCT does.
- `IntTools_Context`'s caches must not be rebuilt per call.
- The benchmark project of unit 4W gains a Booleans suite (unit 6M).

### 2.10 The two 2D pieces: recommendation

- **ChFi2d: pull it into phase 5 (unit 5R).**
  - It is 6,200 lines and needs only GccAna and Geom2dGcc, which phase 5 ports anyway.
  - build123d.NET calls `BRepFilletAPI_MakeFillet2d` for wire fillets and chamfers (`Wire.cs`, `Fillet2DTests`), and
    `fillet2d`/`chfi2d` have corpus cases.
  - Leaving it for phase 7 would make build123d.NET's sketches wait on a phase whose other content (TKFillet's 3D
    fillets) is unrelated.
- **The 2D wire offset: pull it into phase 5 too, as its last units (5M, 5O1, 5O2).**
  - It is about 39,000 lines (43,000 with `evolved`): the medial axis (MAT, MAT2d, Bisector, BRepMAT2d; 18,300),
    BRepFill's offset wire, BRepAlgo and MakeOffset (10,600), and the TopOpeBRepBuild subset (2.11; about 10,000
    estimated).
  - It gives build123d.NET's `offset` of wires and sketches (`OneD.cs` uses `BRepOffsetAPI_MakeOffset`).
  - It unlocks the offset group's 621 wire cases (552 with data) once the scan knows their branch.
  - `evolved` is cheap afterwards.
  - It needs GccAna's bisectors (5K1) and Geom2dInt (ported). It is independent of the Booleans, so it can move
    earlier if capacity allows.
  - The risk is the TopOpeBRepBuild subset. If it proves much larger than estimated, the unit stops and asks (open
    question 17).

### 2.11 The TopOpeBRepBuild subset for the 2D offset

design.md leaves "the legacy TopOpeBRep Booleans" out. The 2D offset reaches them through one call:

- `BRepOffsetAPI_MakeOffset`'s `BuildDomains` and `BRepFill_Evolved` call `BRepAlgo_FaceRestrictor::Perform`;
- without correction mode, that makes faces with `TopOpeBRepBuild_WireToFace::MakeFaces`: a `WireEdgeSet`, a
  `FaceBuilder` with forced classification, and `TopOpeBRepBuild_Builder::MakeFaces` over a `TopOpeBRepDS_BuildTool`.

By class, that closure is TopOpeBRepBuild, TopOpeBRepDS and TopOpeBRepTool: 40,500 lines, plus BRepApprox through
TopOpeBRepTool_CurveTool. Most of it is the Boolean algorithm proper, which the face maker never runs.

**Provisional** (decision 585, open question 17): port only what `WireToFace::MakeFaces` executes, at member level:

- the area/loop framework: AreaBuilder, Area2dBuilder, FaceAreaBuilder, LoopSet, Loop, LoopClassifier, ShapeSet,
  WireEdgeSet, WireEdgeClassifier, CompositeClassifier, BlockBuilder, BlockIterator, FaceBuilder (about 6,000);
- `Builder::MakeFaces` and the members it calls;
- the TopOpeBRepDS_BuildTool and TopOpeBRepTool members those reach (2D classification, `TOOL` statics, C2DF).

Members not ported are left out of the classes with a one-line comment; no stub that could be called silently. The
unit measures the subset first and reports before going on.

Alternatives:

- the whole closure (about 45,000 lines, the legacy Booleans' data structure with it);
- replace the face maker with `BOPAlgo_BuilderFace` (not a port: different faces, no native parity);
- leave the 2D offset for phase 7.

### 2.12 What the public API looks like to Bento

- The algorithms keep OCCT's classes and calls. build123d.NET's code changes only where a shape was copied to protect
  it (`PrivateCopy` before ShapeFix and meshing) or a native-only workaround existed.
- Every result is frozen, every history answers for the caller's sub-shapes, and every algorithm with options has an
  equatable options value. This follows phase 4's pattern for `IMeshTools_Parameters`.
- Progress and cancellation: BOPAlgo, BRepAlgoAPI, ShapeUpgrade_UnifySameDomain, ShapeFix and BRepFill take
  `Message_ProgressRange` as in OCCT (decision 20), so Bento can cancel a long Boolean through a `CancellationToken`.
  `UserBreak` in a parallel loop stops the loop as OCCT's does.

## 3. The oracle

**What build123d.NET binds** (`bindings/occt.txt`):

- **Booleans:**
  - `BRepAlgoAPI_Algo`, `BuilderAlgo`, `BooleanOperation`, `Cut`, `Fuse`, `Common`, `Section`, `Splitter`;
  - `BuilderAlgo` with `SetFuzzyValue`, `SetGlue`, `SetNonDestructive`, `SetCheckInverted`, `SetUseOBB`,
    `SetRunParallel`, `SetToFillHistory`, `SimplifyResult`, `History`, `Modified`/`Generated`/`IsDeleted`,
    `SectionEdges`;
  - `BRepAlgoAPI_Section` with `Approximation` and `ComputePCurveOn1/2`;
  - `BOPAlgo_Options` with the global `SetParallelMode`, `BOPAlgo_Builder`, `BOPAlgo_Splitter`,
    `BOPAlgo_ToolsProvider`, `HasErrors`/`HasWarnings`.
- **Intersections:** `GeomAPI_IntSS` (`Perform`, `NbLines`, `Line`), `GeomAPI_IntCS`, `IntTools_FClass2d`,
  `Geom2dAPI_InterCurveCurve`.
- **Healing:** `ShapeUpgrade_UnifySameDomain` (with `AllowInternalEdges`, `SetLinear/AngularTolerance`,
  `SetSafeInputMode`, `KeepShape(s)`, `History`), `ShapeFix_Root`, `Shape`, `Solid`, `Face`, `Wireframe`,
  `ShapeAnalysis_FreeBounds`, `Shell`, `Curve`, `Edge`, `Surface`, `BRepBuilderAPI_Sewing`.
- **Construction:**
  - `BRepOffsetAPI_ThruSections`, `MakePipeShell` (modes, transitions, `Simulate`, `Generated`, `FirstShape`,
    `LastShape`), `MakeFilling`, `MakeOffset`;
  - `BRepFill` (`Face`, `Shell`, `Axe`, ACR), `BRepProj_Projection`;
  - `GeomFill_TrihedronLaw`, `Frenet`, `CorrectedFrenet`;
  - seven `Geom2dGcc` solvers with `Geom2dGcc_QualifiedCurve`, `IntAna2d`;
  - `BRepFilletAPI_MakeFillet2d`, `GeomAPI_PointsToBSplineSurface`;
  - BRep text both ways, `BRepGProp`, `BRepCheck_Analyzer`.

**Not bound:** IntPatch, IntWalk, IntPolyh, GeomInt and ApproxInt directly; IntTools' intersectors and context;
BOPDS and PaveFiller; GccAna, GccInt and GccEnt directly (Geom2dGcc dispatches analytic cases to GccAna);
Law, GeomFill beyond the trihedrons, Plate, GeomPlate, FairCurve, LocalAnalysis, Hatch, MAT2d, Bisector; ChFi2d's
FilletAPI and ChamferAPI; ShapeFix_Wire, ShapeFix_Edge, ShapeConstruct; BRepAlgoAPI_Defeaturing, BOPAlgo_MakerVolume,
CellsBuilder, ArgumentAnalyzer. They are checked through the bound classes above, the corpus and the GTests. Open
question 20 asks build123d.NET to bind a short list.

**How the units compare** (decisions 578 and 581):

- **Surface/surface (5B to 5G):**
  - random pairs of every analytic type at random placements (coaxial, tangent, through apexes, at the degenerate
    cases ImpImp special-cases), with B-spline, Bézier, revolution, extrusion and offset surfaces from phase 2's
    generators, and the faces of data/occ shapes;
  - through `GeomAPI_IntSS` (approximated lines) and `BRepAlgoAPI_Section` on faces (BRep bytes with pcurves).
- **Booleans (6E to 6P):** inputs from four sources:
  1. random primitive pairs and triples (box, cylinder, cone, sphere, torus, prism and revolution of random profiles),
     overlapping, touching, coplanar, coaxial and disjoint;
  2. data/occ shapes against primitives;
  3. the corpus's boolean-group inputs whose data is public;
  4. **build123d.NET's algebra-mode operations** (the milestone).

  Each is written as BRep text natively (F1) and read on both sides. Then Cut, Fuse, Common, Section and Splitter run
  with build123d.NET's options (parallel, non-destructive), then with fuzzy values, glue and OBB, serially and in
  parallel. The results are compared as 2.5, histories included.
- **Healing:**
  - UnifySameDomain on every Boolean result above (as build123d.NET's `Clean`), with and without internal edges;
  - ShapeFix_Shape on the corpus heal group's public inputs and on wires and faces as build123d.NET's `MakeFace` builds
    them;
  - Sewing on the sewing group's inputs and on exploded faces of data/occ shapes, at the three tolerances of the
    corpus grids.
- **Construction (5K to 5O):**
  - Geom2dGcc's seven solvers on random qualified circles, lines and B-spline curves (solution count, each solution's
    geometry and qualifiers bit for bit);
  - MakeFillet2d on random polygons and arcs;
  - MakePipeShell with every trihedron mode and transition on random spines and profiles;
  - ThruSections (ruled and smoothed, solid and shell, closed and open);
  - MakeFilling with edge and point constraints;
  - `BRepFill::Face`/`Shell`, `BRepProj_Projection`, `PointsToBSplineSurface`, MakeOffset on random wires (the
    offset grids' values).
- **Native repeatability** (2.4): every Boolean oracle case is also run five times natively, serially and in
  parallel; cases where native varies are reported apart.

**The milestone's harness** (unit 6M, decision 586). build123d.NET's algebra-mode tests are in
`tests/Build123d.Tests` (`Operations/OperationsGenericTests` 89 tests, `OperationsPartTests` 49, `Topology/` Boolean,
Clean, History, Shape3D and Solid tests). Their Boolean operations are reproduced by **capturing each native
`BRepAlgoAPI_*` call and each `UnifySameDomain` that build123d.NET makes while its tests run**:

- the arguments and tools as BRep text,
- the options,
- the native result as BRep text, its history, and the cleaned result.

Each capture is replayed on the port from the same bytes and compared as 2.5. The capture is a test-only hook in
build123d.NET (`TopoUtil`'s Boolean path), which is the owner's to allow (open question 20). Until then the harness
rebuilds the tests' operands with build123d.NET's public API from the oracle project (it already references
Build123d.Occt), which covers the same operations with more transcription.

## 4. Work units and waves

Each unit is one branch and one porter. **Tests:** GT is OCCT's GTests; Oracle is native comparisons; Corpus is
Draw-lite cases. Sizes are thousands of OCCT lines.

### Wave 1: start at once

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5A** Intersection foundations (the phase 5 pilot) | IntSurf's rest (LineOn2S, PntOn2S, PathPoint(Tool), InteriorPoint(Tool), Couple, Transition, TypeTrans, Situation, the package); IntImp's Int2S, ZerImpFunc, ZerParFunc and IntStart's SearchOnBoundaries, SearchInside as generics (2.1); IntPatch's data model and tools: Point, Line, PointLine, ALine, GLine, WLine, RLine, IType, SpecPntType, SearchPnt, HInterTool, HCurve2dTool, ArcFunction, CSFunction, CurvIntSurf, the TheSOnBounds family, TheSearchInside, TheSurfFunction, PolyArc, PolyLine, Polygo, RstInt. **Design:** the generic-instantiation pattern for IntPatch/GeomInt/BRepApprox; lines and points as working objects (not frozen values); the comparison helper of decision 578 | 16.3 | phase 4 | GT: IntSurf_LineOn2S. Unit tests: walking-line containers (insert, split, purge) against OCCT's semantics; IntStart's boundary search on cylinders and spheres cut by planes (points, arcs, transitions). Oracle: indirect (5G) |
| **5E** IntPolyh | All of IntPolyh: MaillageAffinage, Triangle, Point, Edge, Couple, StartPoint, SectionLine, Intersection, Tools, the arrays | 7.8 | phase 4 | GT: IntPolyh_Intersection, IntPolyh_Point. Unit tests: triangle-triangle couples and start points on B-spline pairs, the same every run. Oracle: indirect (5D, 5G) |
| **5H** Hatching and the API remainder | Hatch, HatchGen, Geom2dHatch; GeomAPI_PointsToBSplineSurface; GCE2d's typedefs | 7.7 | phase 4 | GT: Geom2dHatch_Elements, Geom2dHatch_Intersector, GeomAPI_PointsToBSplineSurface. Oracle: PointsToBSplineSurface (bound) on random grids, every parametrisation and continuity; the hatcher through 6B |
| **6A** BOP data and reports (the phase 6 pilot) | TKernel's Message_Report, Alert, AlertExtended, CompositeAlerts, Attribute*, Level, PrinterToReport; TopoDS_AlertWithShape, AlertAttribute; BOPDS (all); IntTools' data classes (Range, Root, CommonPrt, Curve, PntOnFace, PntOn2Faces, MarkedRangeSet, ShrunkRange, the range samples and localize data, the package's statics); BOPAlgo_Options, Algo, Alerts, GlueEnum, Operation, CheckStatus, SectionAttribute. **Design:** the public API shell (2.3, decision 579) with a stub builder, so that 6P only fills it; the per-thread context through `OSD_Parallel` (2.4, decision 580); the determinism and canonical-comparison harnesses (2.5) | 14.7 | phase 4 | Unit tests: reports and alerts (HasErrors, DumpErrors, gravity filters); BOPDS_DS built by hand (paves, pave blocks, common blocks, interference tables, the iterator's sorted pairs); the API shell: frozen inputs never written, `Shape()` frozen, the options struct equatable; the canonical comparer on shapes renumbered at random |
| **6S** Sewing | BRepBuilderAPI_Sewing, FastSewing, VertexInspector, BndBoxTreeSelector. Copy at entry (2.7) | 7.7 | phase 4 | GT: BRepBuilderAPI_Sewing (TKOffset's, 117 lines). Oracle: Sewing (bound) on exploded faces of data/occ shapes and the sewing group's public inputs, at 1e-6, 1e-2, 1 and 100; `Modified`, free, multiple and degenerated edges |
| **6H1** Healing foundations | ShapeExtend's rest (CompositeSurface, Explorer, MsgRegistrator, BasicMsgRegistrator, Parametrisation); ShapeBuild_ReShape; ShapeConstruct (package, Curve, ProjectCurveOnSurface, MakeTriangulation); ShapeAnalysis's rest (FreeBounds after 6S, Shell, ShapeContents, ShapeTolerance, FreeBoundsProperties, CheckSmallFace, WireVertex, Geom, BoxBndTree, CanonicalRecognition); Geom2dAPI_Interpolate, PointsToBSpline. **Design:** the copy-at-entry helper and history composition for all of healing (2.7, decision 583) | 16.8 | phase 4 (6S for FreeBounds) | GT: ShapeBuild_ReShape, ShapeConstruct_ProjectCurveOnSurface (999 lines), ShapeAnalysis_FreeBounds, ShapeAnalysis_CanonicalRecognition, Geom2dAPI_Interpolate, Geom2dAPI_PointsToBSpline. Oracle: ShapeAnalysis_FreeBounds and Shell (bound) on the corpus's shells; ProjectCurveOnSurface indirectly through ShapeFix_Edge (6H2) |

### Wave 2

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5B** Analytic intersections | IntPatch_ImpImpIntersection (`_0` to `_5`: plane, cylinder, cone, sphere and torus pairs, the coaxial and tangent cases, the cylinder/cylinder walking of OCCT 7.x) | 9.9 | 5A | Unit tests on every quadric pair OCCT special-cases (OCCT's bug cases from the corpus's lowalgos and bugs groups as fixed inputs). Oracle: via 5G's `GeomAPI_IntSS` and `BRepAlgoAPI_Section` (6P); bit-identical or attributed |
| **5C** Walking | IntWalk (IWalking as a generic, PWalking, TheInt2S, its function, the vectors, StatusDeflection); IntPatch_TheIWalking, TheIWLineOfTheIWalking; IntPatch_ImpPrmIntersection | 12.8 | 5A | Unit tests: walking a cylinder against a B-spline and a torus against a plane, step by step against recorded native steps (points, statuses); the same line serially every run. Oracle: via 5G |
| **6B** Context and BOP tools | IntTools_Context, FClass2d, TopolTool; BOPTools (AlgoTools, AlgoTools2D, AlgoTools3D, Set, Parallel onto `OSD_Parallel`, PairSelector, BoxSelector, ConnexityBlock, BoxTree, CoupleOfShape). The context's cache audit (2.4) | 11.0 | 6A, 5H | Oracle: IntTools_FClass2d (bound) on random points in data/occ faces against native; BOPTools' statics (`IsSplitToReverse`, `PointInFace`, `ComputeState`, `MakePCurve`, `CorrectTolerances`) on native shapes through BRep text. Unit tests: a context reused across calls gives the same answers as fresh ones |
| **6H2** ShapeFix: edges and wires | ShapeFix (package statics), Root, Edge, EdgeProjAux, Wire, WireSegment, WireVertex, IntersectionTool, ShapeTolerance, SplitCommonVertex, EdgeConnect. Geometry-edit sites copied (2.7) | 15.2 | 6H1 | Oracle: wires as build123d.NET's `MakeFace` fixes them, through `ShapeFix_Shape` (bound) on a wire; `ShapeFix_Wireframe`'s gaps through 6H3. Unit tests: ShapeFix_Wire's fixes one by one (reorder, small, connected, degenerated, self-intersection, lacking, gaps 3D/2D) on OCCT's heal-group inputs |

### Wave 3

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5D** Parametric intersections | IntPatch_PrmPrmIntersection (with T3Bits), Polyhedron, PolyhedronTool, PolyhedronBVH, BVHTraversal, InterferencePolyhedron, SpecialPoints | 8.7 | 5C, 5E | GT: IntPatch_Polyhedron, IntPatch_PolyhedronBVH. Oracle: via 5G on B-spline/B-spline and offset pairs |
| **5F** Approximating intersection lines | ApproxInt (Approx as a generic over the surfaces' tools, ImpPrmSvSurfaces, PrmPrmSvSurfaces, SvSurfaces, KnotTools, MultiLine, MultiLineTool); GeomInt_WLApprox and its 24 instantiations; BRepApprox (Approx, ApproxLine, SurfaceTool, its instantiations), all reusing AppParCurves' generics (decision 142) | 14.0 | 5A | Unit tests: approximating native walking lines read from recorded points (knots, poles, tolerances reached) against recorded native results. Oracle: via 5G (the lines `GeomAPI_IntSS` returns are approximations) |
| **6C** Edge intersectors | IntTools_EdgeEdge, EdgeFace, BeanFaceIntersector, CurveRangeSample/LocalizeData use | 6.0 | 6B | Unit tests: native results recorded from `bopcurves`-style runs on data/occ edges (common parts, ranges, types). Oracle: indirect (6E) |
| **6H3** ShapeFix: faces to shapes | ShapeFix_Face, ComposeShell, SplitTool, Shell, Solid, Shape, Wireframe, FixSmallFace, FixSmallSolid, FaceConnect, FreeBounds | 16.7 | 6H2, 6S | GT: ShapeFix_Face, ShapeFix_Shape. Oracle: `ShapeFix_Shape`, `Solid` (`SolidFromShell`), `Face`, `Wireframe` (bound) on the heal group's public inputs and build123d.NET's face-making cases; frozen inputs: the caller's sub-shapes kept where nothing changed |

### Wave 4

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5G** Surface/surface intersection | IntPatch_Intersection, LineConstructor, ALineToWLine, WLineTool; GeomInt (package, IntSS, LineConstructor, LineTool, ParameterAndOrientation); GeomAPI_IntSS; Draw-lite `intersect` for surfaces | 13.1 + C# | 5B, 5C, 5D, 5F | **The surface/surface milestone** (section 5). GT: GeomAPI_IntSS. Oracle: section 3 in full. Corpus: `intersect` |
| **6U** UnifySameDomain | ShapeUpgrade_UnifySameDomain (with its use of IntPatch_ImpImpIntersection, ShapeFix_Face/Wire/Shell, ShapeConstruct); SafeInputMode on frozen shapes (2.7) | 5.0 | 5B, 6H3 | GT: ShapeUpgrade_UnifySameDomain (591 lines). Oracle: UnifySameDomain (bound) on native Boolean results of section 3 (as build123d.NET's `Clean`: `(shape, true, true, true)` and `AllowInternalEdges(false)`), history included. Corpus: `unifysamedom` |

### Wave 5

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **6D** Face/face intersection | IntTools_FaceFace (its own tolerances, the approximation flags, the analytic shortcuts), IntTools_WLineTool | 4.6 | 5G, 6B | GT: IntTools_FaceFace (424 lines). Oracle: `BRepAlgoAPI_Section` on single faces once 6P lands; before that, the lines against `GeomAPI_IntSS` where the parameters agree |
| **6E** The pave filler | BOPAlgo_PaveFiller (all eleven files: VV, VE, EE, VF, EF, FF, section curves, pave blocks, `_10`'s locked check, `_11`), its parallel loops through 6B's context | 11.3 | 6C, 6D | GT: BOPAlgo_PaveFiller (352 lines). Unit tests: the DS after filling, against native's dumps (`bopds`, `bopinterf` of DRAWEXE, captured once) on the boolean group's simple inputs; serial against parallel |

### Wave 6

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **6F** Builders and Booleans | BOPAlgo_Builder (`_1` to `_4`), BuilderShape, BuilderArea, BuilderFace, BuilderSolid, WireSplitter, ShellSplitter, WireEdgeSet, Tools, BOP, Section, Splitter, ToolsProvider | 13.7 | 6E | GT: BOPAlgo_BOP (281 lines). Oracle: BOPAlgo_Builder and BOPAlgo_Splitter (bound) on section 3's inputs |
| **6G** Checks and the other builders | ArgumentAnalyzer, CheckerSI, CheckResult, MakerVolume, CellsBuilder, MakeConnected, MakePeriodic, RemoveFeatures | 8.9 | 6F, 6U | Corpus: `bopargcheck`, `bopcheck`, `mkvolume`, `bc*`, `makeconnected`, `makeperiodic`, `removefeatures`. Oracle: none bound (open question 20); the corpus's reference values |

### Wave 7

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **6P** BRepAlgoAPI and the public API | BRepAlgoAPI (all ten classes); the API filled in (2.3): frozen results, histories, `InternalShape()`, the functional twins and options struct, `SimplifyResult` through 6U, progress and cancellation | 2.7 | 6F, 6U (6G for Check, Defeaturing) | **The Boolean milestones** (section 5). GT: BRepAlgoAPI_Fuse, Cut, Common, BuilderAlgo, Section (TKBO's and TKBool's, about 6,000 lines). Oracle: section 3 in full; the determinism test |
| **6L** Draw-lite: Booleans and healing | BOPTest's commands (BOPCommands, ObjCommands, PartitionCommands, APICommands, OptionCommands, CellsCommands, CheckCommands, RemoveFeatures, MkConnected, Periodicity, Utility, Toler; the debug commands that read the DS); BRepTest's history commands; `sewing`, `fastsewing`; SWDRAW's ShapeFix, ShapeAnalysis and ShapeTool commands and `unifysamedom`, `removeloc`, `reshape` | ≈ 3.5 (C#) | 6P, 6H3, 6U, 6S | **The corpus milestone for phase 6.** Per command against DRAWEXE output captured once |
| **6M** build123d.NET's Booleans | The capture-and-replay harness (section 3), the canonical comparison report, native repeatability, the Booleans benchmark suite (2.9) | C# (tests) | 6P | **The build123d.NET milestone** (section 5) |

### Wave 8: the rest of phase 5 (independent units may start earlier if porters are free)

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5K1** Analytic constraints | GccEnt, GccInt, GccAna (all) | 17.8 | phase 4 | Oracle: via Geom2dGcc's analytic dispatch (5K2), and GccAna's bisectors through 5M. Unit tests: each GccAna solver on OCCT's documented configurations (counts and qualifiers) |
| **5K2** Geometric constraints | Geom2dGcc (all: the seven solvers, the Geo and Iter variants, QualifiedCurve, QCurve, CurveTool) | 16.0 | 5K1 | GT: Geom2dGcc_Circ2d2TanRad, Circ2d3Tan, Lin2d2Tan. Oracle: the seven bound solvers (section 3). Corpus: `cirtang`, `lintan`, `gcarc` |
| **5L1** Laws and trihedrons | Law (all); GeomFill's trihedrons and location laws (TrihedronLaw, Frenet, CorrectedFrenet, Darboux, ConstantBiNormal, Fixed, DiscreteTrihedron, DraftTrihedron, GuideTrihedronAC/Plan, TrihedronWithGuide, CurveAndTrihedron, LocationLaw, LocationGuide, LocationDraft, LocFunction, FunctionGuide, FunctionDraft, PlanFunc, SnglrFunc, TgtField) | ≈ 14 | phase 4 | GT: GeomFill_CorrectedFrenet, GeomFill_GuideTrihedronAC. Oracle: GeomFill_TrihedronLaw, Frenet, CorrectedFrenet (bound) on random spines (D0 to D2, intervals) |
| **5L2** Sections and sweep surfaces | GeomFill's SectionLaw, UniformSection, EvolvedSection, NSections, SectionPlacement, SweepFunction, Sweep, AppSweep, Pipe, CircularBlendFunc, the convertors, SweepSectionGenerator, Tensor, the package | ≈ 13.6 | 5L1 | GT: GeomFill_NSections. Oracle: indirect (5S2's pipe shells); GeomFill_Pipe through `sweep`/`tuyau` corpus cases |
| **5L3** Surfaces from curves | GeomFill's filling half (Coons, Curved, Stretch, ConstrainedFilling, CoonsAlgPatch, the boundaries, BSplineCurves, BezierCurves, Gordon, GordonUtilities, NetworkSurface, Generator, Profiler, AppSurf, SectionGenerator, Line); AppBlend | 12.6 | 5L1 | GT: GeomFill_BSplineCurves, GeomFill_Gordon. Oracle: indirect (5T's ThruSections) |
| **5P** Plates and filling | Plate, NLPlate, GeomPlate, LocalAnalysis; BRepFill_Filling, CurveConstraint, FaceAndOrder, EdgeFaceAndOrder; BRepOffsetAPI_MakeFilling | 16.9 | 5L1 | GT: Plate_Plate, GeomPlate_BuildPlateSurface. Oracle: MakeFilling (bound) with edge, face and point constraints (build123d.NET's `Face.MakeSurface` cases). Corpus: `filling`, `gplate`, `plate` |
| **5Q** Fair curves | FairCurve (all) | 4.5 | 5L1 | Unit tests on OCCT's `battencurve`/`minvarcurve` examples; the corpus |
| **5M** Medial axis | MAT, MAT2d, Bisector, BRepMAT2d | 18.3 | 5K1 | Unit tests: bisecting loci of polygons and arcs (graph counts, arcs' geometry) against native values recorded once; `bisec` |

### Wave 9

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5S1** Sweep laws and sections | BRepFill's package statics (`Face`, `Shell`, `Axe`, ACR), LocationLaw, ACRLaw, Edge3DLaw, EdgeOnSurfLaw, SectionLaw, ShapeLaw, NSections, Section, SectionPlacement, CompatibleWires, Pipe, TransitionStyle, TypeOfContact; ShapeUpgrade_RemoveLocations; BRepOffsetAPI_MakePipe | ≈ 9 | 5L2, 6B | Oracle: `BRepFill.Face`/`Shell` (bound, build123d.NET's ruled faces); MakePipe through `pipe` cases |
| **5S2** Pipe shells | BRepFill_Sweep, TrimShellCorner, PipeShell; BRepOffsetAPI_MakePipeShell, MiddlePath. Frozen profiles copied where written (2.8) | ≈ 11.6 | 5S1, 6P | GT: BRepFill_PipeShell, BRepOffsetAPI_MakePipeShell. Oracle: MakePipeShell (bound) with every mode and transition, solids and shells, `Simulate`, `Generated` |
| **5T** Lofts and projection | BRepFill_Generator, BRepOffsetAPI_ThruSections, BRepProj_Projection | 3.8 | 5L3, 5S1, 6P | Oracle: ThruSections (bound), ruled and smooth, solid and shell, with the corpus's inconsistent profiles; BRepProj_Projection (bound) on build123d.NET's projection cases |
| **5R** 2D fillets and chamfers | ChFi2d (all), BRepFilletAPI_MakeFillet2d | 6.2 | 5K2 | Oracle: MakeFillet2d (bound) on random polygons and arcs, fillets and chamfers, `Modified`/`IsModified`; ChFi2d_FilletAPI on wires (unit tests against native values recorded once). Corpus: `fillet2d`, `chamfer2d`, `chfi2d` |
| **5O1** Faces from wires (the TopOpeBRepBuild subset) | BRepAlgo (package, FaceRestrictor, Loop, Image, AsDes); the TopOpeBRepBuild/DS/Tool members `WireToFace::MakeFaces` runs (2.11), measured and reported first | ≈ 14 | phase 4 | Unit tests: FaceRestrictor on planar faces with nested and crossing wires against native results recorded once (faces, orientations) |

### Wave 10

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5O2** The 2D wire offset | BRepFill_OffsetWire, TrimEdgeTool, TrimSurfaceTool, MultiLine, ApproxSeewing, ComputeCLine, OffsetAncestors; BRepOffsetAPI_MakeOffset; optionally BRepFill_Evolved and BRepOffsetAPI_MakeEvolved (non-BOP modes) | ≈ 10.7 | 5M, 5O1 | Oracle: MakeOffset (bound) on random wires and the offset grids' values, open and closed, every join type. Corpus: the offset group's wire grids (`mkoffset`, `openoffset`), `evolved` |
| **5W** Draw-lite: construction | BRepTest's sweep commands (`thrusections`, `pipe`, `mksweep`, `addsweep`, `setsweep`, `deletesweep`, `buildsweep`, `simulsweep`, `errorsweep`, `gener`, `middlepath`, `geompipe`), filling commands, Fillet2D commands, `prj`, `mkoffset`, `openoffset`, `evolved`; GeometryTest's constraint, surface (`sweep`, `appsurf`, `ruled`, `tuyau`), fair-curve and continuity commands; DrawCorpus's offset `command` branch | ≈ 2 (C#) | 5K2 to 5O2 | **The corpus milestone for phase 5.** Per command against DRAWEXE output captured once |

### Totals

| Stage | Units | Lines (k) |
|---|---|---:|
| Surface/surface intersections (phase 5) | 5A, 5B, 5C, 5D, 5E, 5F, 5G | 82.6 |
| Phase 5's other wave-1 unit | 5H | 7.7 |
| Booleans (phase 6) | 6A, 6B, 6C, 6D, 6E, 6F, 6G, 6P | 73.9 |
| Healing and sewing (phase 6) | 6S, 6H1, 6H2, 6H3, 6U | 61.4 |
| Constraints, laws, sweeps, lofts, filling (phase 5) | 5K1, 5K2, 5L1, 5L2, 5L3, 5P, 5Q, 5S1, 5S2, 5T | 119.8 |
| The 2D pieces (phase 5) | 5R, 5M, 5O1, 5O2 | 49.2 |
| Draw-lite, harnesses | 6L, 6M, 5W (and 5G's `intersect`) | ≈ 7 (C#) |

That is about **259,000 lines for phase 5** and **135,000 for phase 6**, 395,000 in all, in 33 units.

**The critical path** to the Boolean milestone is 5A → 5C → 5D → 5G → 6D → 6E → 6F → 6P → 6M. 5B and 6H1 → 6H2 →
6H3 → 6U run beside it.

- **5A and 6A are the pilots.** The generic pattern lands in 5A, the Boolean API, context and determinism design in
  6A. Each should be reviewed before 5G and 6E.
- **Six units can start at once:** 5A, 5E, 5H, 6A, 6S and 6H1. None depends on another. 6H1 merges ShapeAnalysis's
  FreeBounds after 6S.
- **Off the critical path, startable at any time:** 5K1, 5K2, 5L1 to 5L3, 5P, 5Q, 5M and 5O1 depend only on phase 4
  and each other. The owner's sequencing puts them after the Booleans; they are the units to give porters who would
  otherwise wait on 6D to 6F.
- The sweeps (5S1, 5S2, 5T) must follow 6B, and for TrimShellCorner and BRepProj, 6P.

## 5. Milestones

1. **After 5G: surface/surface intersections as native OCCT computes them.**
   - Every analytic pair of the oracle (plane, cylinder, cone, sphere, torus at random and special placements)
     intersects bit-identically to native through `GeomAPI_IntSS`, or the difference is attributed.
   - At least 95% of the walking-line pairs (B-spline, Bézier, revolution, extrusion, offset, and data/occ faces) are
     bit-identical. Every other pair has the same number and types of lines, within the reported tolerance, and is
     attributed.
   - The corpus's `intersect` cases that are runnable pass.
2. **After 6P: Booleans as native OCCT computes them.**
   - Cut, Fuse, Common, Section and Splitter on the oracle's four input sources (section 3) give results whose BRep
     bytes equal native's, the Free column apart, wherever native is itself repeatable. Target: 95% of cases
     bytes-identical, the rest canonically identical (2.5) or attributed.
   - Histories agree for every argument sub-shape.
   - Fuzzy, glue and OBB options agree.
   - Frozen inputs are never written. A Boolean that changes nothing an argument holds shares it.
3. **After 6P: parallel Booleans give the serial result**, bit for bit, at every degree of parallelism and on every
   repeat, over the oracle's and the corpus's Booleans.
4. **After 6M: build123d.NET's algebra-mode Boolean tests' operations reproduce native results through BRep bytes.**
   - Every `BRepAlgoAPI_*` call and every `UnifySameDomain` that build123d.NET's tests make (captured or
     transcribed, section 3) is replayed on the port from native's input bytes.
   - The port's result and cleaned result equal native's bytes (Free column apart), or are canonically identical where
     native's own output varies.
   - Histories agree, so build123d.NET's history-based selection (`Select.New`, `Select.Last`) picks the same
     sub-shapes.
   - With the face-making path (ShapeFix_Shape on wires, ShapeFix_Face) and the sewing of shells, this means
     build123d.NET's extrude, pocket (cut of an extrusion), Booleans and `Clean()` can run on the port.
5. **After 6L: the corpus's Boolean, sewing and healing cases.**
   - The 4,880 cases the scan finds runnable after phase 6 (4,010 with their data) run.
   - They pass at the rate DRAWEXE passes them with the reference values in their scripts. Expected failures are
     OCCT's TODOs and missing data.
6. **After 5W: build123d.NET's construction on the port, and the corpus.**
   - MakePipeShell, ThruSections, MakeFilling, `BRepFill.Face`/`Shell`, BRepProj_Projection, the seven Geom2dGcc
     solvers, MakeFillet2d, MakeOffset and PointsToBSplineSurface reproduce native results on the oracle's inputs (bytes
     where native is repeatable, canonically otherwise).
   - build123d.NET's sweep, loft, filling, constrained-object, Fillet2D and 2D-offset tests' operations reproduce
     natively, as milestone 4 for Booleans.
   - The 861 newly runnable construction cases (601 with data) and the offset group's wire cases (552 with data) pass
     at DRAWEXE's rate.
7. **Performance** (2.9): the Booleans benchmark reports the port's serial and parallel times against native's, and
   the 2× target is met or the gap explained.

## 6. Notes for porters

- **Phases 3 and 4's notes apply** (phase3-plan.md section 5, phase4-plan.md section 6):
  - shapes are structs, maps of shapes take a comparer (OCCTNET0002);
  - writing into frozen inputs means copying first;
  - every twin and copy site goes in decisions.md;
  - `OSD_Parallel` for every loop, with OCCT's sequential flag;
  - the MSVC-order helper for `std::sort`/heap ties.
- **Generics** (2.1): port each `.gxx` once as a generic over a struct tool; name the instantiations as OCCT does.
  Check every method an instantiation's tool hides without `virtual`: through a generic, C# calls the constraint's
  member.
- **Handles of algorithm data** (`IntPatch_Line`, `IntSurf_LineOn2S`, `BOPDS_PaveBlock`, `BOPDS_CommonBlock`,
  `IntTools_Context`) are classes and working objects, not frozen values. Maps keyed by them compare by reference and
  iterate in insertion order (decision 490). Where OCCT's result could depend on address order, add it to
  occt-issues.md with the site.
- **BOPTools_Parallel**: `BOPTools_Parallel::Perform(isParallel, vector, context)` → `OSD_Parallel.For` over the
  vector with the per-worker context of decision 580. Keep each `myRunParallel` flag.
- **Exceptions in loop bodies.** BOP's `try { OCC_CATCH_SIGNALS ... } catch (Standard_Failure&)` adds an alert to the
  report: keep it per item. An exception that escapes a parallel loop is unwrapped (decision 431).
- **Reports.** `AddError(new BOPAlgo_AlertXxx(shape))` keeps OCCT's alert classes (one class per
  `DEFINE_ALERT_WITH_SHAPE`) so build123d.NET's checks by type keep working. `DumpErrors(Standard_OStream&)` writes to
  a `TextWriter`.
- **Non-destructive mode.** Don't add a frozen check of your own: `PaveFiller_10`'s `Locked()` test already makes the
  operation non-destructive (2.3). If a write still reaches a frozen sub-shape, copy at that site and record it.
- **ShapeFix's geometry edits** (2.7): where OCCT edits a curve reached from a representation (`->SetPole`,
  `->Segment`, `->IncreaseDegree`, `->Reverse` on a pcurve it didn't create), copy it first if frozen, and record the
  site.
- **ShapeExtend statuses** (`ShapeExtend_Status`, `Status(ShapeExtend_DONE1)`) are ported in phase 4. Keep each
  status bit, because build123d.NET and the corpus read them.
- **Debug and dump code**: `#ifdef OCCT_DEBUG`, `DRAW`, `DEB` blocks go, and so do the `DrawTrSurf` calls in
  IntTools_FClass2d and IntPolyh. Public dumps (`BOPDS_DS::Dump`, `IntPatch_Point::Dump`, `IntPolyh` dumps) are ported,
  writing to a `TextWriter`.
- **Statics.** `BOPAlgo_Options::SetParallelMode` (global), `IntPatch`'s and `IntWalk`'s tuning constants,
  `ShapeFix`'s and `BRepLib`'s precision statics: process-wide, internal setters (decision 415).
- **Progress.** Keep every `Message_ProgressRange` parameter and `UserBreak` check (2.12).

## 7. Questions for the owner

These are in [open-questions.md](open-questions.md) as 16 to 23, with the provisional answers the units follow.

1. **Healing pulled into phase 6** (16). UnifySameDomain reaches most of ShapeFix, and build123d.NET calls
   ShapeFix_Shape, Solid, Face, Wireframe and Sewing. Port ShapeFix, the rest of ShapeAnalysis, ShapeConstruct,
   ShapeBuild_ReShape and Sewing whole (61,400 lines with UnifySameDomain, most of it design.md's phase 7), or only
   the classes reached (about 48,000)?
   Recommended: whole classes and packages, as for ShapeAnalysis in phase 4.
2. **The 2D offset and its TopOpeBRepBuild subset** (17). Pull the 2D wire offset into phase 5, porting only the
   members of TopOpeBRepBuild/DS/Tool that `WireToFace::MakeFaces` runs (2.11)? The alternatives are the whole legacy
   data structure (about 45,000 lines), a non-mechanical replacement, or phase 7.
3. **Booleans of frozen shapes are always non-destructive** (18), through OCCT's own Locked check;
   `SetNonDestructive(false)` applies only to unfrozen shapes. Every result sharing an unsplit argument face shows
   `Free = 1` where native shows 0 (open question 1 grows with it).
4. **Parity where native isn't repeatable** (19). Native BOP iterates address-ordered maps and can vary from run to
   run. The port is deterministic (serial = parallel) and is compared canonically where native varies. Target: 95% of
   Boolean cases bytes-identical, the rest canonically identical or attributed.
5. **The oracle for build123d.NET's tests** (20). Allow a test-only capture hook in build123d.NET's Boolean path
   (operands, options, native results as BRep text)? And bind a short list: `BOPAlgo_MakerVolume`,
   `BOPAlgo_CellsBuilder`, `BRepAlgoAPI_Defeaturing`, `BOPAlgo_ArgumentAnalyzer`, `ShapeFix_Wire`, `ShapeFix_Edge`,
   `ChFi2d_FilletAPI`, `GeomInt_IntSS` (for its approximation flags), `Law_Function` (for `MakePipeShell::SetLaw`)?
6. **Boolean options for Bento** (21). An equatable options struct with OCCT's names and defaults, and functional
   twins, as `IMeshTools_Parameters`?
7. **Performance targets** (22). The port's serial Boolean within 2× native's serial, with a comparable parallel
   speed-up?
8. **Interleaving and porters' order** (23). Follow the owner's order strictly (the rest of phase 5 after the Boolean
   milestone), or let the independent construction units (5K, 5L, 5P, 5Q, 5M, 5O1) start whenever a porter would
   otherwise wait?
