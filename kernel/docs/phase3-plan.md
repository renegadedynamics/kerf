# Phase 3 plan: topology and BRep input/output

This plan splits phase 3 into work units that can be ported in parallel. [phase3-design.md](phase3-design.md)
fixes the rules the units follow:

- shapes are structs,
- geometry freezes when attached,
- TShapes freeze on publication and are then Locked,
- writers copy first,
- meshes stay on the shapes.

Phase 3 covers TKBRep (except OCCT 8's new BRepGraph), the parts of TKTopAlgo that design.md assigns to phase 3, and
everything those parts need. They need more than design.md expected:

- **TKGeomAlgo's 2D curve intersections and curve–surface intersections.** BRepClass classifies points in faces
  with `Geom2dInt_GInter`, BRepCheck_Wire looks for self-intersections with it, and BRepClass3d and IntCurvesFace
  use `IntCurveSurface_HInter`.
- **BRepLib.** BRepCheck_Edge calls `BRepLib_ValidateEdge`, BRepExtrema builds its result edges and faces with
  `BRepBuilderAPI_MakeEdge`/`MakeFace`, and BRepGProp and BRepBndLib call each other.

The phase is therefore about 125,000 lines, not TKBRep's 34,000. Each line of the dependency table was checked at
the call sites.

Sizes are physical lines of `.hxx/.cxx/.lxx/.pxx/.gxx`, comments included, as `tools/port-status` counts them.
Dependencies were taken from identifiers used in the code bodies, after removing `#include`s and the code under
`OCCT_DEBUG`, `DRAW` and `DEB`.

## 1. What phase 3 starts from

- **Foundation:** TKMath, including TopLoc (decision 26), Poly (decisions 61 to 70) and Bnd.
- **Phase 2:** TKG2d, TKG3d and TKGeomBase, including:
  - GeomTools (the geometry sections of a `.brep`, decisions 225 to 234),
  - GProp,
  - BndLib/GeomBndLib,
  - Extrema,
  - the adaptors.
- **Not yet merged from phase 2:** unit 5A (GeomLib, Approx_SameParameter, Approx_CurvilinearParameter) and 6A
  (ProjLib_ProjectedCurve, ComputeApprox, GeomProjLib). Phase 3 uses them in four places:

  | Place | Needs |
  |---|---|
  | `BRep_Tool::CurveOnSurface` on non-planar faces without a pcurve | ProjLib_ProjectedCurve, GeomProjLib (6A) |
  | `BRepTools_NurbsConvertModification` | ProjLib_ComputeApprox* (6A), GeomLib (5A) |
  | `BRepLib_ValidateEdge`, `BRepLib_CheckCurveOnSurface` | GeomLib_CheckCurveOnSurface (5A) |
  | `BRepLib::SameParameter`, `BuildCurves3d` | Approx_SameParameter, Approx_CurvilinearParameter (5A) |

  Units 3A, 3B and 3H may start before they merge. Each such call is then a `// TODO(5A)` or `// TODO(6A)` that
  throws `Standard_NotImplemented`, removed when the unit merges.
- **The design pieces 3A builds first**, before the parallel units start:
  - the freezing hooks on `TopoDS_TShape` and `BRep_*`,
  - `CheckMutable()` in Poly's mutators,
  - the subtype generator,
  - analyzer rule OCCTNET0002.

## 2. Packages

"Oracle" names classes bound by build123d.NET (`bindings/occt.txt`). "GT" is the number of lines of OCCT GTests.

**Native shapes as oracle inputs.** build123d.NET binds BRepPrimAPI, BRepBuilderAPI, BRepAlgoAPI and its shim's
`b3d_brep_write`/`b3d_brep_read`. So from unit 3C on, oracle tests can build their inputs natively, pass them to
the port as BRep text, and compare the port's answers with native OCCT's on the same shape. The transfer itself is
checked byte for byte.

### TKBRep (34,300 lines without BRepGraph)

| Package | Lines | Real dependencies | Oracle | GT |
|---|---:|---|---|---:|
| TopoDS | 2,776 | TopAbs, TopLoc; Message for TopoDS_AlertWithShape | TopoDS_*, Builder, Iterator | 1,370 (Builder, Edge, Iterator, TShape) |
| TopExp | 883 | TopoDS, TopTools_ShapeMapHasher | TopExp, Explorer | 200 |
| TopTools | 1,722 (ShapeSet and LocationSet 1,529) | TopoDS; GeomTools (LocationSet writes gp_Trsf) | ShapeMapHasher | — |
| BRep | 8,105 | TopoDS, TopExp, Geom, Geom2d, GeomAdaptor, Poly; **ProjLib_ProjectedCurve and GeomProjLib in BRep_Tool only** | Tool, Builder | 260 |
| BRepAdaptor | 1,384 | BRep, Adaptor3d/2d, GeomAdaptor, Geom2dAdaptor, GCPnts; **BRepTools_WireExplorer in CompCurve only** | Curve, CompCurve, Curve2d, Surface | 190 |
| BRepLProp | 430 | BRepAdaptor, the LProp_*3d bodies (decision 229) | BRepLProp, SLProps | — |
| BRepTools | 11,663 | BRep, TopExp, TopTools, BRepAdaptor_Surface, GeomTools (ShapeSet), BndLib; **ProjLib, GeomLib, Extrema and the converters in NurbsConvertModification only** | BRepTools, History, ReShape, WireExplorer | 510 (BRepTools, ReShape) |
| BinTools | 7,330 | BRep, TopTools, Geom, Geom2d, Poly; `BRepTools::Update` | none: ask build123d.NET to bind BinTools, or add a `b3d_bin_write`/`b3d_bin_read` to its shim | — |
| BRepGraph, BRepGraphInc | 55,982 | — | — | 39,900 |

**BRepGraph waits.** Nothing in OCCT 8 uses it (no algorithm, no Draw command, no test script); design, section 8.

### TKTopAlgo, the phase 3 part (52,800 lines)

| Package | Lines | Real dependencies | Oracle | GT |
|---|---:|---|---|---:|
| BRepGProp | 7,450 | GProp, Geom/Geom2d, adaptors, BndLib; **BRepCheck_Shell** (closed-shell test in `VolumeProperties`) | BRepGProp, BRepGProp_Face | 250 (needs MakeBox/Cylinder/Sphere; its Cut case waits for phase 6) |
| BRepBndLib | 1,430 | BndLib, adaptors, Extrema_ExtSS, BRepTopAdaptor_FClass2d; BRepGProp (OBB, BRepBndLib_1) | BRepBndLib | — |
| BRepCheck | 7,467 | adaptors, Geom2dInt + IntRes2d (Face, Wire), BndLib, ProjLib/GeomProjLib (Edge); **BRepClass3d** (Solid), **BRepTopAdaptor_FClass2d** (Face), **BRepLib_ValidateEdge** (Edge) | Analyzer | 200 (Face) |
| BRepClass | 1,909 | Geom2dInt, IntRes2d, TopClass, TopTrans, Extrema, Geom2dAPI_ProjectPointOnCurve, GC, GeomLProp | none (indirect: BRepClass3d) | — |
| BRepTopAdaptor | 2,140 | Adaptor3d_TopolTool, BRepClass, Geom2dInt_Geom2dCurveTool, GCPnts | none (indirect) | — |
| IntCurvesFace | 1,172 | IntCurveSurface, Intf_Tool, GeomAPI_ProjectPointOnCurve, BRepTopAdaptor | none (indirect: BRepIntCurveSurface) | — |
| BRepClass3d | 3,427 | IntCurveSurface, IntCurvesFace, BRepClass, BRepTopAdaptor, BRepBndLib, Extrema | SolidClassifier | 80 (needs MakeBox/Sphere) |
| BRepIntCurveSurface | 443 | IntCurveSurface, BRepTopAdaptor, BRepBndLib | Inter | — |
| BRepExtrema | 7,718 | Extrema (ExtPC, ExtPS, ExtCC, ExtCS, ExtSS), GeomAPI projections, BRepClass, BRepClass3d, BRepTopAdaptor, BRepBndLib, GCPnts, BRepGProp; **BRepBuilderAPI_MakeVertex/Edge/Face in DistanceSS** | DistShapeShape | 90 |
| BRepLib | 13,241 | Approx (SameParameter, CurvilinearParameter: 5A), GeomLib (5A), Extrema, GCPnts, the converters, GeomLProp, ProjLib; BRepCheck (statics), BRepClass3d, BRepTopAdaptor, BRepBndLib, BRepGProp | BRepLib, FindSurface; indirect: BRepBuilderAPI_* | 25 (MakeWire) |
| BRepBuilderAPI, without Sewing, FastSewing and MakeShapeOnMesh | 6,340 | BRepLib (makers and errors), BRepTools modifications | MakeVertex/Edge/Wire/Polygon/Face/Shell/Solid, Transform, GTransform, Copy | — (the TopAlgo makers' GTests are phase 5) |

**Inside TKTopAlgo the dependencies run in cycles at class level.** They are broken by ordering:

1. BRepGProp needs only `BRepCheck_Shell`.
2. BRepLib needs BRepCheck's statics (`PrecCurve`, `PrecSurface`), and BRepCheck needs only `BRepLib_ValidateEdge`.
3. BRepBndLib needs BRepGProp only for OBBs.

So BRepCheck's statics, `Status`, `Result` and `Shell`, and BRepLib's `ValidateEdge` and `CheckCurveOnSurface`, go
in an early unit (3H) with BRepGProp.

### TKGeomAlgo, the part phase 3 needs (38,200 lines)

| Package | Lines | Real dependencies | Oracle | GT |
|---|---:|---|---|---:|
| IntRes2d | 1,932 | (phase 2) | none | — |
| IntImpParGen | 1,446 | IntRes2d | none | — |
| Intf | 5,981 (InterferencePolygonPolyhedron.gxx 1,368, instantiated by IntCurveSurface) | (phase 2) | none | Intf_Tool |
| TopTrans, TopClass | 1,365 + 405 | IntRes2d | none | TopTrans_SurfaceTransition |
| IntCurve | 11,791 (5,000 in .gxx: IntCurveCurveGen, IntPolyPolyGen, UserIntConicCurveGen, ...) | IntRes2d, IntImpParGen, Intf (2D) | none | — |
| Geom2dInt | 2,271 (the instantiations of IntCurve's .gxx) | IntCurve, IntRes2d, Intf_Polygon2d | indirect: Geom2dAPI_InterCurveCurve | — |
| Geom2dAPI: ProjectPointOnCurve, ExtremaCurveCurve, InterCurveCurve | 1,021 | Extrema, Geom2dInt | ProjectPointOnCurve, InterCurveCurve | InterCurveCurve |
| IntCurveSurface | 7,676 | Intf, IntImp (IntCS, ZerCSParFunc), IntSurf_Quadric, IntAna | indirect: GeomAPI_IntCS | 4 files |
| IntImp: IntCS, ZerCSParFunc, ComputeTangence | 538 | — | — | — |
| IntSurf: Quadric, QuadricTool | 919 | — | — | IntSurf_Quadric |
| GeomAPI: the package statics (To2d, To3d), ProjectPointOnCurve/Surf, ExtremaCurveCurve/CurveSurface/SurfaceSurface, IntCS | 2,896 | Extrema, IntCurveSurface | ProjectPointOnCurve/Surf, IntCS | ProjectPointOnSurf |

GeomAPI_IntSS (GeomInt, phase 5), GeomAPI's interpolation and approximation classes (phase 5) and the rest of IntSurf
and IntImp (IntPatch and IntWalk, phase 5) are not needed.

## 3. Work units and waves

Each unit is one branch and one porter, as in phase 2. The **Tests** column lists:

- **Oracle:** native comparisons.
- **GT:** OCCT GTests. Many of TKBRep's and TKTopAlgo's build their inputs with `BRepBuilderAPI_MakeEdge`, `MakeFace`
  and `BRepPrimAPI_MakeBox`, so they are ported once 3K (and 3M) merge. Earlier units test through `BRep_Builder`
  (internal, visible to the tests) and through files read by 3C.
- **Corpus:** Draw-lite cases.

### Wave 1

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3A** Topology core | TopoDS (structs and generated subtypes, TShapes, Builder, Iterator, exceptions); TopExp; TopTools (package, ShapeMapHasher, FormatVersion); BRep (TVertex/TEdge/TFace, every representation, Builder, Tool); BRepTools.cxx statics except Read/Write/Dump (a `partial` class); BRepAdaptor_Surface. **Design infrastructure:** `FreezeGraph`, Locked-from-frozen, `CheckMutable` in TShapes, representations and Poly; `TopoDS_Builder.MakeCompound`; analyzer OCCTNET0002; `tools/gen-topods-subtypes.cs` | 14.0 | phase 2 (6A stubbed if needed) | Oracle: shapes built with BRep_Builder identically on both sides (native BRep_Builder is bound), then BRep_Tool, TopExp, Explorer, IsSame/IsEqual/hash agreement, UVBounds, OuterWire. Unit tests of every freezing rule in the design (sections 4.1 to 4.5). GT: TopoDS_TShape, TopoDS_Builder, TopoDS_Iterator, TopExp, BRep_Tool, BRepTools (after 3K and 3M) |
| **3D** 2D intersection bases | IntRes2d, IntImpParGen, Intf except InterferencePolygonPolyhedron, TopTrans, TopClass | 9.8 | phase 2 | Oracle: none (indirect through 3E). GT: Intf_Tool, TopTrans_SurfaceTransition |

### Wave 2

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3B** Modification and adaptors | BRepTools_History, Modification, Modifier, CopyModification, TrsfModification, GTrsfModification, NurbsConvertModification, ReShape, Quilt, Substitution, PurgeLocations, WireExplorer; `BRepTools.CopyForWrite` (design 4.6); the functional twins of BRepTools' in-place statics; BRepAdaptor Curve, Curve2d, CompCurve; BRepLProp | 9.2 | 3A | Oracle: native BRepBuilderAPI_Transform/GTransform/Copy against the port's Modifier with the same modification; ReShape, History, WireExplorer, BRepAdaptor_*, BRepLProp_SLProps on native shapes (from 3C). GT: BRepTools_ReShape, BRepAdaptor_CompCurve (after 3K) |
| **3C** Text and binary I/O | TopTools_ShapeSet, LocationSet; BRepTools_ShapeSet and BRepTools Read/Write/Dump; BinTools (ShapeSet, ShapeReader/Writer, I/O streams, the curve, surface and location sets); the untrusted-input checks (design, section 7) | 10.9 | 3A | Oracle: native `BRepTools::Write` of random native shapes (primitives, sweeps, Boolean results, with and without triangulations, format versions 1 to 3) read by the port and written back, byte for byte; the reverse direction; binary through native BinTools once bound. Corpus: the 37 `.brep` files of data/occ, F1/F2 scheme (design, section 7). Fuzz test on truncated and mutated files |
| **3E** 2D curve intersection | IntCurve (its .gxx as generics, decision 202's approach), Geom2dInt, Geom2dAPI ProjectPointOnCurve, ExtremaCurveCurve, InterCurveCurve | 15.1 | 3D | Oracle: Geom2dAPI_InterCurveCurve, ProjectPointOnCurve on random 2D curves. GT: Geom2dAPI_InterCurveCurve |
| **3F** Curve–surface intersection | IntCurveSurface (HInter and the .gxx instantiations), Intf_InterferencePolygonPolyhedron, IntImp_IntCS/ZerCSParFunc/ComputeTangence, IntSurf_Quadric/QuadricTool, GeomAPI (package, projections, extrema, IntCS) | 13.4 | 3D | Oracle: GeomAPI_IntCS, ProjectPointOnCurve, ProjectPointOnSurf. GT: IntCurveSurface (4 files), GeomAPI_ProjectPointOnSurf, IntSurf_Quadric |
| **3H** Global properties | BRepGProp (all); BRepCheck.cxx statics, Status, Result, ListOfStatus, Shell; BRepLib_ValidateEdge, CheckCurveOnSurface | 9.6 | 3A, phase 2 5A | Oracle: BRepGProp Linear/Surface/VolumeProperties (and the Ex forms, mesh-based on shapes carrying triangulations) on native shapes read through 3C; BRepGProp_Face. GT: BRepGProp (after 3M) |

### Wave 3

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3G** Classification and boxes | BRepClass, BRepTopAdaptor, IntCurvesFace, BRepClass3d, BRepIntCurveSurface, BRepBndLib | 10.5 | 3A, 3B, 3E, 3F, 3H | Oracle: BRepClass3d_SolidClassifier, BRepBndLib (Add, AddClose, AddOptimal, AddOBB), BRepIntCurveSurface_Inter on native shapes. GT: BRepClass3d_SolidClassifier (after 3M) |
| **3L1** Draw-lite: shapes | Drawable variables in DrawInterpreter; `brestore`, `bsave`, `readbrep`, `writebrep`, `binsave`, `binrestore`; `isdraw`, `dtyp`, `dname`, `whatis`, `copy`, `renamevar`, `dump`, `datadir`; `nbshapes`, `numshapes`; `explode`, `nexplode`, `exwire`; `compound`, `add`, `emptycopy`, `orientation`, `treverse`, `complement`, `invert`; `tolerance`, `maxtolerance`; `sprops`, `lprops`, `vprops`; `purgeloc`, `checkloc`, `setflags`; `ttranslate`, `trotate`, `tmove`, `reset`, `tscale`, `tmirror` (over 3B's TrsfModification and GTrsfModification); the viewer commands (`isos`, `triangles`, `polygons`, `discretisation`) as no-ops. Output as Draw prints it | ≈ 3 (C#, from DBRep, Draw_VariableCommands, BRepTest_BasicCommands, BRepTest_GPropCommands) | 3B, 3C, 3H | Draw-lite tests per command against DRAWEXE output captured once; the DrawResources procedures `checknbshapes`, `checkprops`, `checkmaxtol` on data/occ files |

### Wave 4

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3I** BRepLib | BRepLib (statics: SameParameter, UpdateTolerances, BuildCurves3d, EncodeRegularity, OrientClosedSolid, ...; makers: MakeVertex, MakeEdge, MakeEdge2d, MakeWire, MakePolygon, MakeFace, MakeShell, MakeSolid, MakeShape, Command; FindSurface, FuseEdges, PointCloudShape, ToolTriangulatedShape), with the public functional twins of the in-place statics (design 4.6) and the internal non-freezing accessors of the makers (design 4.2) | 12.7 | 3G, 3H, 3B, phase 2 5A | Oracle: native BRepBuilderAPI_MakeEdge/Face/Wire/... against the port's BRepLib makers on the same inputs, compared through BRep bytes; BRepLib::SameParameter, UpdateTolerances, BuildCurves3d against native on copies; BRepLib_FindSurface. GT: BRepLib_MakeWire |
| **3J** BRepCheck | BRepCheck_Analyzer, Vertex, Edge, Wire, Face, Solid (with the parallel mode) | 5.8 | 3G, 3E, 3H | Oracle: BRepCheck_Analyzer statuses on native shapes, valid and deliberately broken (tolerances, pcurves, orientations). GT: BRepCheck_Face (after 3K). Corpus: heal/checkshape's 20 cases that need no external data |

### Wave 5

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3K** Distances and makers | BRepExtrema (DistShapeShape, DistanceSS, ExtPC/PF/CC/CF/FF, the proximity and self-intersection tools, Poly_Util); BRepBuilderAPI without the sewing classes and MakeShapeOnMesh | 14.1 | 3I, 3G, 3H | Oracle: BRepExtrema_DistShapeShape (values, points, supports, parameters) on native shape pairs; BRepBuilderAPI_* against native. GT: BRepExtrema_DistShapeShape, and the GTests of 3A, 3B and 3J that need only BRepBuilderAPI |
| **3M** Primitives (recommended, owner question 2) | TKPrim: BRepPrim, BRepPrimAPI, BRepSweep, Sweep, BRepPreviewAPI | 12.8 | 3K | Oracle: BRepPrimAPI_MakeBox, Wedge, Cylinder, Cone, Sphere, Torus, Prism, Revol, HalfSpace against native through BRep bytes. GT: TKPrim's 960 lines, and every phase 3 GTest that needs MakeBox, Cylinder or Sphere |

### Wave 6

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3L2** Draw-lite: checks and the corpus milestone | `checkshape` (with `-parallel`, `-short`), `checksection`, `bounding`, `distmini`, `tcopy`, `bclassify`; with 3M: `box`, `pcylinder`, `pcone`, `psphere`, `ptorus`, `prism`, `revol`, `wedge`, `halfspace` | ≈ 2 (C#) | 3J, 3K (3M) | The corpus milestone (section 4) |

### Totals

| Wave | Units | Lines (k) |
|---|---|---:|
| 1 | 3A, 3D | 23.8 |
| 2 | 3B, 3C, 3E, 3F, 3H | 58.2 |
| 3 | 3G, 3L1 | 10.5 + Draw-lite |
| 4 | 3I, 3J | 18.5 |
| 5 | 3K (3M) | 14.1 (+ 12.8) |
| 6 | 3L2 | Draw-lite |

That is 125,000 lines of OCCT, or 138,000 with TKPrim, plus about 5,000 lines of Draw-lite C#.

**The critical path** is 3A → 3H → 3G → 3I → 3K → 3L2:

- 3A is the design pilot and should merge, and be reviewed against the design, before wave 2 starts.
- Wave 2 has five independent units.
- After that the TKTopAlgo cycles leave one or two units per wave.

## 4. Milestones

1. **After 3C and 3L1: BRep round trip.**
   - All 37 `.brep` files of `data/occ` round-trip byte-identically: native F1 → port → F1, text and (once bound)
     binary.
   - Random native shapes do the same in both directions.
   - `restore`, `nbshapes`/`checknbshapes`, `sprops`/`vprops`/`lprops`/`checkprops`, `explode`, `tolerance` and
     `checkmaxtol` run on them.
   - Bento can store, load and measure shapes without native code (design.md's phase 3 milestone, without
     checkshape).
2. **After 3L2: the corpus.**
   - Every corpus case whose commands are all implemented runs and passes, with the reference values in the scripts.
   - Every `.brep` and `.rle` that the corpus restores round-trips byte-identically against native F1.
   - A spike scan of today's corpus (17,237 cases, 15 runnable) found:
     - **373 cases runnable** with 3L1 and 3L2's commands,
     - **of which only 20 need no external data file** (heal/checkshape),
     - 323 of the 373 are boolean/gdml_private, which `source` a Tcl file from the test data, so their real needs
       are unknown until that data is present.
   - The scan also shows what comes next: the cases missing exactly one command miss `bcut` (303), `thrusections`
     (246), `blend` (175), `bcommon` (169), `bfuse` (163), `bsection` (148), `prj` (76), `triarea` (75), `fixshape`
     (63) and `box` (14). With 3M, `box` and `pcylinder`, which 2,868 and 1,031 cases use, would be available.
   - The command counts are from DrawCorpus's scan; the most used commands are `isdraw` (13,869 cases),
     `checkshape` (13,045), `sprops` (12,066), `dtyp` (11,030), `dname` (10,929), `lprops` (9,805), `vprops`
     (9,667), `brestore` (9,625), `explode` (7,886), `nbshapes` (6,547) and `tolerance` (4,416).
   - **The corpus needs OCCT's test data** (owner question 3). The corpus restores 5,815 distinct data files, and
     the OCCT source has 37 of them.

## 5. Notes for porters

- **Shapes are structs.**
  - Write `new TopoDS_Shape()`, never `default`.
  - Narrow with `TopoDS.Edge(S)`.
  - Pass a subtype variable to a `ref TopoDS_Shape` that OCCT changes only through `TopoDS.AsShapeRef(ref e)`.
  - `TopoDS_Builder.Add` and `Remove` take the parent by `in`.
- **Maps of shapes** take an explicit comparer: `TopTools_ShapeMapHasher.Instance` for every `TopTools_*` typedef
  (OCCTNET0002).
- **Freezing.**
  - Mutators of TShapes, representations and attached Poly objects start with `CheckMutable()`.
  - `BRep_Builder` freezes what it attaches.
  - A public method returning a shape it built freezes the graph first; ported code that keeps building on it uses
    the internal accessor.
  - Reads freeze at the end.
- **Writing into a frozen input** in ported code: don't. Copy for write (design 4.6), compose the history, and add a
  line to decisions.md. Cache completions on frozen shapes (`BRepTools::Update(Face)`) return early, marked
  `// OCCT.NET: frozen`.
- **Locked.** `Locked()` includes frozen. Writers of files use the raw bit (`LockedFlag()`).
- **Free.** `TopoDS_Builder.Add` leaves a frozen child's Free unchanged (design 4.4). Everything else about Free is
  OCCT's.
- **Twins.** Each public in-place static gets a functional twin returning the new shape (and its history where
  OCCT keeps one), listed in decisions.md.
- **Streams** follow decisions 225 and 226. Writers write `'\n'`. The untrusted-input checks are marked
  `// OCCT.NET: untrusted input`.
- **Oracle inputs** are native shapes passed as BRep text (`b3d_brep_write`). Mirror native call paths as
  oracle-testing.md says.

## 6. Questions for the owner

1. **Free on frozen children.** Leave a frozen shape's Free flag as it was when frozen (recommended: a shape's BRep
   text never changes), or clear it as OCCT does when the shape is added to a compound (OCCT's bytes in every
   history, but a shared shape's text changes when another cell uses it)?
2. **Scope.** Phase 3 needs about 38,000 lines of TKGeomAlgo and BRepLib with BRepBuilderAPI's makers, which
   design.md placed in phase 5. Should it also take TKPrim (unit 3M, 12,800 lines)? Without it, most of TKBRep's and
   TKTopAlgo's GTests can't be ported, and the corpus can't run `box` or `pcylinder`. Recommended: yes, and update
   design.md's phase table.
3. **OCCT's test data.** The corpus milestone needs OCCT's separate test-data archive (5,815 files that `restore`
   reads; several GB). Should `tools/get-occt.ps1` fetch it into `.deps`?
4. **BinTools oracle.** build123d.NET doesn't bind BinTools. Ask it to bind BinTools, or to add a binary twin of
   `b3d_brep_write`/`read` to its shim?
5. **Public construction.** `BRep_Builder` and `TopoDS_Builder` become internal; callers use the makers and
   `TopoDS_Builder.MakeCompound`. build123d.NET uses `BRep_Builder` directly today. Is a functional compound builder
   enough, or does it need more, such as editing a compound's children?
