# Phase 4 plan: meshing

This plan splits phase 4 into work units that can be ported in parallel. Phase 4 ports TKMesh (BRepMesh and its
interfaces) and TKXMesh, and what they need beyond phases 0 to 3. Its milestones:

- shapes mesh as native OCCT meshes them,
- the corpus's mesh commands run,
- Bento's viewer meshes without native code,
- a WebAssembly demo.

The rules it follows are phase 3's ([phase3-design.md](phase3-design.md)):

- meshes stay on the shapes,
- meshing returns a meshed copy (section 5 there),
- writers copy first (4.6).

Section 2 settles what is specific to meshing.

TKMesh is 20,000 lines. It needs about 14,000 lines of TKShHealing (ShapeAnalysis's wire analysis, design.md's
phase 7) and 3,000 of TKernel. So the phase is about **37,000 lines of OCCT**, plus about 2,500 lines of C# for
Draw-lite and the viewer. Sizes are physical lines of `.hxx/.cxx/.lxx/.pxx/.gxx` (and Delabella's `.cpp`),
comments included, as `tools/port-status` counts them. Dependencies were taken from identifiers used in the code
bodies, after removing `#include`s and debug code, and checked at the call sites.

## 1. Scope

### 1.1 What phase 4 starts from

Phases 0 to 3, merged on `main`:

- **Phase 0 and 1:** TKMath, including Poly (all 34 files) and BVH, and NCollection's CellFilter, UBTree,
  UBTreeFiller, LinearVector, DynamicArray and the maps.
- **Phase 2:** TKG2d, TKG3d and TKGeomBase, including GCPnts_TangentialDeflection, Extrema_LocateExtPC and CSLib.
- **Phase 3:** TKBRep (without BRepGraph), the TKTopAlgo makers and the BRepLib statics that BRepMesh calls:
  - `UpdateDeflection` (internal),
  - `ReverseSortFaces`,
  - `BRepLib_ToolTriangulatedShape`.

  Also phase 3's TKGeomAlgo intersections, TKPrim and Draw-lite. Phase 3 also has:

  - `BRepTools.ExactCopyForWrite` and `KeepUnchanged`/`KeepUnchangedIn` (decisions 416, 468),
  - the twins `BRepTools.Clean`, `BRepLib.UpdateDeflection` and the `Triangulation*` statics,
  - BRep text and binary I/O with triangulations.

### 1.2 TKMesh, file by file (19,930 lines; GTests 1,113 more)

| Package | Lines | Classes |
|---|---:|---|
| IMeshData | 1,058 | Types (the typedefs), Shape, TessellatedShape, StatusOwner, Status, Model, Face, Wire, Edge, PCurve, Curve, ParametersList, ParametersListArrayAdaptor: abstract interfaces of the discrete model |
| IMeshTools | 1,092 | Parameters (struct), Context, MeshBuilder (a `Message_Algorithm`), ModelBuilder, ModelAlgo, MeshAlgo, MeshAlgoFactory, MeshAlgoType, CurveTessellator, ShapeVisitor, ShapeExplorer |
| BRepMeshData | 913 | Model, Face, Wire, Edge, PCurve, Curve: the default model |
| BRepMesh: plain types | 1,000 | Vertex 125, Edge 99, OrientedEdge 83, Triangle 135, PairOfIndex 113, Circle 62, DegreeOfFreedom 28; the inspectors CircleInspector 131, VertexInspector 148 |
| BRepMesh: Delaunay core | 5,700 | Delaun 3,121, DataStructureOfDelaun 768, SelectorOfDataStructureOfDelaun 216, CircleTool 316, VertexTool 262, Classifier 190, MeshTool 552, Triangulator 356 |
| BRepMesh: Delabella | 1,225 | `delabella.cpp` and `.pxx`: a third-party Delaunay library under the MIT licence (GUMIX, Marcin Sokalski) |
| BRepMesh: geometry tools | 2,050 | GeomTool 779, DefaultRangeSplitter 355, ShapeTool 466, Deflection 235, EdgeParameterProvider 165 (a template over the parameter collection) |
| BRepMesh: model and edges | 3,069 | ShapeVisitor 194, ModelBuilder 121, CurveTessellator 441, EdgeTessellationExtractor 124, EdgeDiscret 420, FaceChecker 410, ModelHealer 683, ModelPreProcessor 396, ModelPostProcessor 280 |
| BRepMesh: face algorithms | 3,983 | BaseMeshAlgo 440, ConstrainedBaseMeshAlgo 69, DelaunayBaseMeshAlgo 92, CustomBaseMeshAlgo 101, CustomDelaunayBaseMeshAlgo 47; the templates NodeInsertionMeshAlgo 227, DelaunayNodeInsertionMeshAlgo 161, DelaunayDeflectionControlMeshAlgo 465; the range splitters UVParam 66, NURBS 739, Cone 125, Cylinder 145, Sphere 106, Torus 262, Extrusion 80, Undefined 65, BoundaryParams 45; MeshAlgoFactory 158, DelabellaBaseMeshAlgo 227, DelabellaMeshAlgoFactory 174, FaceDiscret 189 |
| BRepMesh: entry points | 949 | IncrementalMesh 287, Context 115, DiscretRoot 92, DiscretFactory 129, DiscretAlgoFactory 195, IncrementalMeshFactory 102, FastDiscret 29 (a typedef) |
| TKXMesh: XBRepMesh | 100 | XBRepMesh_Factory: a second registered factory, an IncrementalMesh by another name |

GTests: BaseMeshAlgo (277), CircleTool (84), Delaun (237), DiscretAlgoFactory (145), GeomTool (86) and IncrementalMesh
(273). One IncrementalMesh case fuses a box and a ring with `BRepAlgoAPI_Fuse` (phase 6).

BRepMesh has no mutex. Its parallel loops (`OSD_Parallel::For` and `ForEach`) each write their own face or edge of the
discrete model. The post-processor's edge loop is forced serial by OCCT ("data races on edges sharing the same
TShape").

### 1.3 Dependencies outside phases 0 to 3

| Dependency | Lines | Where | Plan |
|---|---:|---|---|
| **ShapeAnalysis's wire analysis** (TKShHealing, design.md's phase 7) | 14,100 | `BRepMesh_ShapeVisitor` orders each wire with `ShapeAnalysis_Wire::CheckOrder` over `ShapeExtend_WireData` and `ShapeAnalysis_WireOrder`, and takes the outer wire from `ShapeAnalysis::OuterWire`; `BRepMesh_ShapeTool::Range` uses `ShapeAnalysis_Edge::PCurve` and `Curve3d` | Unit 4S ports the whole classes (owner question 1): ShapeExtend (package, Status, WireData, ComplexCurve), ShapeAnalysis (package, Wire, WireOrder, Edge, Curve, Surface, TransferParameters, TransferParametersProj), ShapeBuild (package, Edge, Vertex). `ShapeAnalysis_Wire`'s constructor makes a `ShapeAnalysis_Surface`, and its other checks reach Curve, TransferParametersProj and ShapeBuild_Edge, which is why it is 14,100 lines and not the 600 BRepMesh calls. All their own dependencies are ported (Geom2dInt, IntRes2d, BRepTopAdaptor_FClass2d, BRepLib_ValidateEdge, GeomAPI_ProjectPointOnSurf, Geom2dConvert_ApproxCurve, BRepBuilderAPI_MakeEdge/MakeWire, BRepTools_WireExplorer). `ShapeExtend::Init`'s message file is left for phase 7 |
| **Message_Algorithm** and its status types (TKernel) | ≈ 1,500 | `IMeshTools_MeshBuilder` and `IMeshTools_ModelBuilder` derive from it (`SetStatus(Message_Fail1)`, ...) | Unit 4A: Message_Algorithm, ExecStatus, Status, and Msg without MsgFile's resource files |
| **Message's package statics and Messenger** | ≈ 900 | `Message::SendWarning` (Context), `Message::Send` (Delabella), `Message::DefaultMessenger` (Triangulator) | Unit 4A, as a process-wide messenger whose default sends nothing (decision 493) |
| **NCollection_EBTree, NCollection_MapAlgo** | 518 | IMeshData's `BndBox2dTree` (FaceChecker); `NCollection_MapAlgo::Unite` (FaceChecker, ModelHealer) | Unit 4A. `NCollection_Shared` and the allocators (`IncAllocator`, `OccAllocator`) go, by the rules |
| **OSD_Parallel** | — | 9 loops in BRepMesh (`For`, `ForEach` over a map) | One internal helper for the whole port, with a sequential fallback for single-threaded WebAssembly (decision 487); unit 4A |
| **OSD_Environment** | — | `BRepMesh_Context` reads `CSF_MeshAlgo` to choose Watson or Delabella | Not read: a result must not depend on the process environment (decision 492) |
| **The plugin factory** | 325 + 100 | `BRepMesh_DiscretFactory` (a global singleton with a default name), `BRepMesh_DiscretAlgoFactory` (a global registry, new in OCCT 8), `BRepMesh_IncrementalMeshFactory` ("FastDiscret"), `XBRepMesh_Factory` (self-registering) | Ported, with the two factories registered when the type initialises and the registry's mutators internal (decision 492). No plugin loading: OCCT 8 has none either (`Plugin` appears only in a comment) |
| **MoniTool_TimerSentry** | — | Triangulator, under `CHECK_PERF` only | Left out |
| **BRepBuilderAPI_MakeVertex/Edge/Polygon/Face, BRepTools::Write** | — | The debug dumps of DataStructureOfDelaun, Delaun (`OCCT_DEBUG`), MeshTool and ModelHealer (`DEBUG_HEALER`) | `BRepMesh_MeshTool::DumpTriangles` and `DataStructureOfDelaun::Statistics`/dump are public: ported (all their callees exist); the `#ifdef`'d ones are left out |

Nothing else is missing. Poly (Triangulation, PolygonOnTriangulation, Polygon3D, TriangulationParameters,
MergeNodesTool), Bnd_B2d, Bnd_Box2d, CSLib, GCPnts, Extrema_LocateExtPC, the adaptors, BRepLib and BRepTools are
ported. There is no Standard_Mutex or atomic in TKMesh.

### 1.4 Draw commands and the corpus

The mesh commands are in TKTopTest's MeshTest (2,997 lines): MeshTest.cxx, MeshTest_CheckTopology and
MeshTest_PluginCommands. Also needed are DBRep's `tclean` and CheckCommands.tcl's `checktrinfo` (a procedure,
already loaded, over `trinfo`).

A scan of today's corpus (17,237 cases, 734 runnable after phase 3) with a scratch script over `DrawCorpus.Scan`:

| Command | Cases using it | Cases it alone blocks |
|---|---:|---:|
| `triarea` | 4,244 (every `bugs` case: `bugs/end` calls it when the case sets `rel_tol`) | 178 |
| `trinfo` (and `checktrinfo` over it) | 424 (388) | 3 |
| `incmesh` | 355 | 22 |
| `tclean` | 287 | 7 |
| `tricheck` | 63 | 0 |
| `tessellate`, `wavefront` | 5, 5 | 0, 1 |
| `trmergenodes`, `trlateload`, `mpparallel`, `triepoints`, `correctnormals` | 2, 2, 2, 1, 1 | 0 |

- **With all the mesh commands, 313 more cases become runnable.**
  - By group: bugs/moddata_2 69, bugs/mesh 53, bugs/moddata_1 47, bugs/moddata_3 47, v3d/bugs 29,
    lowalgos/proximity 21, bugs/fclasses 18, bugs/demo 14, bugs/modalg_8 10, and 5 others.
  - 72 of them need no data file, 96 have all their data in the public dataset, and 145 miss a file and will be
    SKIPPED.
- **The scan does not see the `mesh` group.** DrawCorpus skips grids whose cases live in a `cases.list` directory:
  - mesh, 699 cases: 233 data scripts run as `standard_incmesh`, `standard_incmesh_parallel`, `standard_shading`
    and the `advanced_*` grids,
  - heal's split and surface-to-revolution grids, 2,259 cases,
  - chamfer, 279,
  - opengl and xml.

  The mesh group is OCCT's main mesh test:
  - its `end` script runs `tclean`, `incmesh -parallel`, `checkshape`, `trinfo`, `tricheck` and `triarea`, and
    compares area, free links, cross faces and free nodes with the expected values;
  - 83 of its 233 data files (227 `.brep`, 1 `.rle`, 3 STEP) are in the public dataset, so about 250 of its cases
    can run.

  Unit 4L1 adds `cases.list` to DrawCorpus.
- **The `*_shading` grids** mesh through the viewer (`vdisplay`, AIS's default deflection), which Draw-lite makes a
  no-op. Unit 4L2 makes `vdisplay` mesh the shape as AIS does (StdPrs_ToolTriangulatedShape with Prs3d_Drawer's
  defaults), so those grids can run (decision 495).
- **What still blocks the mesh-using cases**, by the one other command they miss:
  - `vdefaults` 101 (a viewer setting, a no-op candidate),
  - `fixshape` 65 (phase 7), `unifysamedom` 51,
  - `bfuse` 37, `bcommon` 18, `bcut` 11 (phase 6),
  - `stepread` 34 (phase 8).
- **`incmesh`'s options in the corpus:** `-a` 132, `-parallel` 15, `-min` 9, `-algo` 6 (bugs/mesh/bug28089_1 and
  _2 compare Watson with Delabella), `-relative` 5, `-surf_def_all` 3, `-force_face_def` 2, and one each of
  `-surf_def_off`, `-int_vert_off`, `-decrease`, `-adjust_min`.

## 2. Design questions

### 2.1 Publishing a meshed shape

`BRepMesh_IncrementalMesh` writes into the shape it is given:

- triangulations into its faces (`BRepMesh_ShapeTool::AddInFace`),
- polygons on triangulation (and 3D polygons for free edges) into its edges,
- then each triangulation's deflection and parameters (`ModelPostProcessor`: `BRepLib::UpdateDeflection`,
  `Poly_Triangulation::Parameters`).

Phase 3 settled that meshes stay on the shapes and that meshing returns a meshed copy. That gives two forms:

1. **A meshed shape**, from `ExactCopyForWrite` + mesh in place + `KeepUnchanged` + freeze, the pattern of decisions
   416 and 468. Recommended, and the only form OCCT's readers of meshes can use:
   - BRepGProp's and BRepBndLib's `useTriangulation`, BRepExtrema's proximity tools, BRepCheck, HLR later, and the
     STL, glTF and OBJ writers of phase 8 all read `BRep_Tool::Triangulation`.
   - Its BRep text is native OCCT's, so the oracle compares whole meshed shapes.
   - Because of `KeepUnchanged`, the result shares the input's vertices, and every face whose triangulation the
     mesher kept ("incremental": an existing one fine enough is reused). Meshing a shape that is already meshed
     finely enough gives back the input itself, which costs one exact copy and no triangulation.
2. **Mesh data returned separately for the viewer**: buffers of nodes, normals, triangles and edge polylines, as
   build123d.NET's `ShapeMesh` holds. That is an extraction from form 1 (unit 4W), not a second mesher.

**The API** (decision 489):

- `BRepMesh_IncrementalMesh` keeps OCCT's constructors and `Perform`, `Shape()`, `IsDone()`, `GetStatusFlags()`.
  - On an unfrozen shape (ported code) it meshes in place, as OCCT does.
  - On a shape whose graph holds a frozen TShape it meshes an exact copy. `Shape()` then returns the frozen meshed
    result, and `History()` maps the input's sub-shapes to the result's.
  - So build123d.NET's `new BRepMesh_IncrementalMesh(copy, tol, rel, ang, true)` becomes
    `new BRepMesh_IncrementalMesh(shape, tol, rel, ang, true).Shape()`. No `PrivateCopy`, no meshing lock.
- A functional twin, `BRepMesh_IncrementalMesh.Meshed(in TopoDS_Shape, in IMeshTools_Parameters)` (and with
  `out BRepTools_History`), as the other twins in decisions.md.
- `IMeshTools_Parameters` stays OCCT's struct with OCCT's public fields:
  - a caller writes `new IMeshTools_Parameters { Deflection = 0.1, Angle = 0.5, InParallel = true }`;
  - it is a value, so changing a copy changes nothing shared;
  - it gets `IEquatable`, so `(shape, parameters)` can key Bento's mesh cache.

**Freezing inside the mesher** (decision 488). `BRep_Builder.UpdateFace(face, triangulation)` freezes the
triangulation it attaches (decision 272). The post-processor then writes `Deflection` and `Parameters` into it.

- Phase 3's internal `BRepLib.UpdateDeflection` replaces frozen triangulations with copies (`WritableTriangulations`).
  Doing that for every face would copy every mesh.
- So the mesher attaches its own triangulations and polygons **unfrozen**, through an internal non-freezing form of
  `BRep_Builder.UpdateFace`/`UpdateEdge` used only by `BRepMesh_ShapeTool`.
- The result is frozen when it is published (`FreezeGraph`, which freezes representations and their Poly objects).
- A triangulation reused from the input stays frozen and is never written: the mesher marks its face Reused and skips
  it in the post-processor.

Rejected: mesh the input's TShapes in place under a lock (the side effect phase 3 removed), and a side cache of meshes
(phase3-design 5).

### 2.2 Parallel meshing must not depend on scheduling

OCCT meshes edges, then faces, in parallel when `InParallel` is set. build123d.NET always sets it.

**The rule** (decision 490): **a parallel run gives the serial run's result bit for bit, every run.** It holds by
construction if each loop body writes only its own face or edge, which BRepMesh's design intends. The port must also:

- **Not iterate in address order.** IMeshData's maps keyed by pointer (`MapOfIEdgePtr`, `MapOfIFacePtr`,
  `DMapOfIFacePtrsMapOfIEdgePtrs`) iterate in address order in OCCT. The healer's amplification loop iterates them.
  The port's maps are keyed by reference and iterate in insertion order, which is deterministic. Where OCCT's result
  could depend on address order, the issue goes in occt-issues.md.
- **Share no evaluation caches.**
  - Each `IMeshData_Face` holds one `BRepAdaptor_Surface`, which the face's edges may evaluate from several threads
    in `EdgeDiscret` and `ModelHealer`. GeomAdaptor's B-spline cache isn't thread-safe; this is the race decision
    431 fixed in BRepCheck_Edge.
  - Porters audit every adaptor reached from two loop bodies and give each thread a shallow copy where needed.
  - Frozen geometry is safe: the Resolution caches are volatile (decisions 126, 135) and the triangulation's range
    takes a lock (decision 69).
- **Test it.** The determinism test meshes each corpus shape serially and in parallel (several degrees of
  parallelism, repeated) and requires identical BRep text and identical node bits. The oracle also compares native
  parallel with native serial, so that a native nondeterminism isn't blamed on the port.

### 2.3 Floating-point parity

A triangulation is a cascade of decisions:

- tangential deflection steps,
- Delaunay circumcircle tests with tolerances,
- node insertion by deflection.

One different last bit upstream can flip a decision, and the result is then a different but equally valid mesh.

**The input.** BRep text gives both sides bit-identical inputs: the oracle meshes native `BRepTools::Read(F1)` and
the port's read of the same F1. The F1/F2 lesson of unit 3C applies: mesh the re-read shape on both sides, not
native's original.

**The expectation** (decision 494):

- Where the arithmetic is the same, the meshes are bit-identical: node coordinates, UV nodes, triangles and their
  order, polygons on triangulation with their parameters, and deflections.
- Differences can come from three sources. Each is investigated and attributed, as in phases 2 and 3:
  - **Transcendental functions.** MSVC's `sin`/`cos`/`acos` and .NET's can round differently (oracle-testing.md).
    They are used by GCPnts on circles, ElSLib on spheres and tori, and the range splitters.
  - **Heap and sort order on ties.** `BRepMesh_Delaun` orders vertices with `std::make_heap` + `std::sort_heap`,
    whose element order on equal keys is the C++ library's. The port reproduces MSVC's heap algorithm, because the
    oracle is MSVC-built (decision 491). Delabella's `std::sort` of points has the same issue; its ties are
    duplicate points.
  - **OCCT's own bugs.** Reproduced and pinned (rules, section 7).
- **The targets:**
  - at least 99% of faces bit-identical on data/occ and on the mesh group's available data;
  - every other face a valid mesh (`tricheck` clean) whose area is within the test's tolerance of native's.

**The comparison** (decision 494). BRep text writes nodes with 15 significant digits, so equal text doesn't prove
equal bits. The oracle compares:

1. BRep text with triangulations, byte for byte. This is unit 3C's writer, and covers the shape, the polygon
   parameters and the deflections.
2. Then each face's `Poly_Triangulation` node by node and triangle by triangle through the native binding: every
   double bit for bit.

It reports, per shape: bit-identical; same topology within 1e-12; or a different triangulation (then the node,
triangle and free-link counts, the area difference and `tricheck`'s verdict).

### 2.4 WebAssembly

Bento.Web, the engine of published workbooks, is **plain WebAssembly**:

- no Blazor, no AOT (the Mono interpreter), single-threaded,
- trimmed, built without the wasm-tools workload,
- `net10.0`.

The port must run there unchanged.

- **Threads.** On single-threaded WebAssembly, `Parallel.For` can't rely on blocking waits.
  - The port routes every parallel loop through one internal `OSD_Parallel.For`/`ForEach` (decision 487). It runs a
    plain loop when threads aren't available (`OperatingSystem.IsBrowser()` without threading, checked once).
  - The loops of phases 1 to 3 move onto it: BVH, BRepCheck_Analyzer, BRepExtrema_DistShapeShape,
    GeomLib_CheckCurveOnSurface.
  - Because of 2.2, the fallback gives the same result.
- **Floating point.** WebAssembly doubles are IEEE 754 without implicit fused multiply-add. But `Math.Sin` and the
  other transcendental functions come from the WebAssembly runtime's libm, not MSVC's. So a mesh made in the browser
  may differ in last bits from one made on Windows (2.3). The spike measures how often. The reference stays win-x64
  against the native oracle.
- **Trimming and size.** The OcctNet assemblies are trimmed with Bento.Web's settings (no reflection, so it should
  hold). The spike reports their size.
- **Stack.** The interpreter's stack is smaller. The spike runs the deepest recursions (UBTree, Delaun's frontier
  walks, BRepTools' readers) on the largest available data files.
- **Speed.** The interpreter is slower than the JIT, by a factor the spike measures. The spike times the interpreter,
  and AOT where the workload is installed, so the owner can choose (owner question 4).

### 2.5 Performance targets

These are provisional (owner question 4), measured by a benchmark project over data/occ and the mesh group's available
data at the corpus's deflections:

- **Desktop** (win-x64, .NET 10): the port's serial meshing within **2×** native serial, and its parallel speed-up
  comparable with native's.
- **Bento's viewer:** a typical model (up to a few hundred faces, at `MeshTolerances.For`'s default 0.1% of the
  diagonal) meshes and extracts in **under 100 ms** on the desktop.
- **WebAssembly** (interpreter): the same model in **under 2 s**, to be confirmed by the spike.

Allocation matters more than in C++, where IMeshData uses an incremental allocator:

- the porters keep OCCT's preallocation sizes,
- the porters reuse buffers where OCCT does,
- the benchmark reports allocated bytes.

## 3. The oracle

**What build123d.NET binds for meshing** (`bindings/occt.txt`):

- `BRepMesh_DiscretRoot`;
- `BRepMesh_IncrementalMesh`, with the five-argument constructor (shape, deflection, relative, angle, parallel) and
  `Perform`, `IsDone`, `GetStatusFlags`, `IsParallelDefault`/`SetParallelDefault`;
- `Poly_Triangulation`: nodes, UV nodes, normals, triangles, deflection, the cached box;
- `Poly_Triangle`.

Also available:

- `BRepTools::Write`/`Read` with triangulations and normals, used by unit 3C's byte comparison;
- `BRepGProp` (the mesh-based properties) and `BRepBndLib` with triangulations;
- its shim's `b3d_mesh_extract` (`Extras.ExtractMesh`), which fills `ShapeMesh`'s buffers. That is exactly what
  unit 4W reproduces.

**Not bound:**

- `IMeshTools_Parameters` and the `(shape, parameters)` constructor, so MinSize, InternalVerticesMode,
  ControlSurfaceDeflection, the interior deflections, AdjustMinSize, ForceFaceDeflection, AllowQualityDecrease and
  the Delabella algorithm can't be set natively;
- `Poly_PolygonOnTriangulation` and `Poly_Polygon3D` (compared through BRep text only);
- `BRepLib_ToolTriangulatedShape`;
- ShapeAnalysis_Wire and WireOrder.

ShapeAnalysis_Curve, Edge and Surface are bound.

**How the units compare** (decision 494):

- **Shapes:** native shapes from three sources:
  - random primitives, sweeps, Booleans and fillets built natively,
  - data/occ's 37 `.brep` files,
  - the public dataset's 83 mesh-group files and the other `.brep` files under 10 MB.

  For each, write F1 natively without triangulations. Read it on both sides. Mesh with the same five arguments:
  deflections absolute and relative, angles 0.1 to 0.5, serial and parallel. Compare as in 2.3.
- **Re-meshing:** shapes carrying a native mesh, finer and coarser than asked, to exercise the incremental reuse.
  Native and the port must keep or replace the same faces' meshes.
- **The parameters native can't set** are checked by the corpus (`incmesh -min`, `-surf_def_all`, `-int_vert_off`,
  ...) against the values in OCCT's scripts, and by the GTests. Owner question 3 asks build123d.NET to bind them.
- **Below the mesher:** the Delaunay core (unit 4B) has no native binding. It is tested by its GTests and by
  property tests: empty circumcircles, Euler's formula, every constraint edge present. It is compared through
  whole meshes.

## 4. Work units and waves

Each unit is one branch and one porter, as in phases 2 and 3. **Tests:** GT is OCCT's GTests; Oracle is native
comparisons; Corpus is Draw-lite cases.

### Wave 1

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **4A** Mesh foundations and data model (the pilot) | TKernel: Message_Algorithm, Message_ExecStatus, Message_Status, Message_Msg (no MsgFile resources), Message_Messenger and Printer, the Message package's Send statics (decision 493); NCollection_EBTree, NCollection_MapAlgo. `OSD_Parallel` (decision 487), with the parallel loops of phases 1 to 3 moved onto it. IMeshData, IMeshTools, BRepMeshData. BRepMesh_GeomTool, DefaultRangeSplitter, ShapeTool, Deflection, EdgeParameterProvider. **Design:** the non-freezing attach for the mesher (decision 488), with FreezeGraph freezing attached Poly objects (checked); `IMeshTools_Parameters` as an equatable struct; the copy-for-write shell of the public API (decision 489) with a stub mesher, so that 4D only fills it | 8.0 | phase 3 | GT: BRepMesh_GeomTool. Unit tests: Message_Algorithm's statuses; BRepMeshData's model built by hand; ShapeTool's ranges and UpdateEdge/AddInFace through the non-freezing attach; the API shell: a frozen input is never written, `KeepUnchanged` gives back the input when the stub changes nothing; OSD_Parallel's serial fallback. Oracle: GeomTool's curve discretisation through GCPnts (indirect) |
| **4S** ShapeAnalysis for wires (owner question 1) | ShapeExtend (package, Status, WireData, ComplexCurve); ShapeAnalysis (package statics including OuterWire and TotCross2D, Wire, WireOrder, Edge, Curve, Surface, TransferParameters, TransferParametersProj); ShapeBuild (package, Edge, Vertex). Writes into shapes (ShapeBuild_Edge's Copy/Replace, TransferParametersProj) follow design 4.6 and are listed in decisions.md | 14.1 | phase 3 | Oracle: ShapeAnalysis_Edge (vertices, curves, pcurves, CheckSameParameter, CheckVerticesWithCurve3d), ShapeAnalysis_Curve (Project, ValidateRange, GetSamplePoints), ShapeAnalysis_Surface (ValueOfUV, NextValueOfUV, IsDegenerated, singularities) on native shapes and surfaces read through 3C; Wire and WireOrder indirectly through 4C's models. GT: ShapeAnalysis_Edge_Test (91 lines) |
| **4L1** Draw-lite: mesh readers and the hidden grids | DrawCorpus: grids with `cases.list` (mesh, heal, chamfer, opengl, xml), counted in the scans. Commands that read meshes already on shapes: `tclean` (on a copy of the graph, as decision 480), `trinfo`, `tricheck` (MeshTest_CheckTopology), `triarea`, `triepoints`, `trmergenodes` (Poly_MergeNodesTool), `correctnormals`, `mpparallel` | ≈ 1.2 (C#) | phase 3 | Draw-lite tests per command against DRAWEXE output captured once, on data/occ and dataset `.brep` files that carry triangulations; `checktrinfo` over `trinfo`. The scan's new totals with the `cases.list` grids |

### Wave 2

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **4B** Delaunay core | The plain types (Vertex, Edge, OrientedEdge, Triangle, PairOfIndex, Circle, DegreeOfFreedom), CircleInspector, VertexInspector, CircleTool, VertexTool, DataStructureOfDelaun, SelectorOfDataStructureOfDelaun, Delaun, Classifier, MeshTool, Triangulator; Delabella's library (`delabella.cpp`/`.pxx`, MIT notice kept, decision 497). MSVC's heap order for Delaun (decision 491) | 7.9 | 4A | GT: BRepMesh_Delaun (6 tests), BRepMesh_CircleTool. Property tests on random and degenerate point sets (grids, collinear runs, duplicates, near-cocircular points): empty circumcircles, Euler counts, constraint edges kept, the same triangulation for the same input every time; Watson against Delabella on the same points (same point set covered). Oracle: indirect (4D). The heap-order helper against recorded MSVC orders on tied keys |
| **4C** Model and edge discretisation | ShapeVisitor, ModelBuilder (with IMeshTools_ShapeExplorer from 4A), CurveTessellator, EdgeTessellationExtractor, EdgeDiscret, FaceChecker, ModelHealer, ModelPreProcessor, ModelPostProcessor. The parallel audit of 2.2 for the edge and healer loops | 3.1 | 4A, 4S | Unit tests on data/occ shapes: the model's faces, wires, edges, pcurves and orientations; edge polygons with reused tessellations (ModelPreProcessor on shapes carrying meshes); the healer's self-intersection fixes on OCCT's examples (bug428, `mesh standard C7`); serial against parallel. Oracle: indirect (4D's polygons on triangulation) |

### Wave 3

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **4D** Face meshing and the incremental mesh | BaseMeshAlgo, ConstrainedBaseMeshAlgo, DelaunayBaseMeshAlgo, CustomBaseMeshAlgo, CustomDelaunayBaseMeshAlgo; NodeInsertionMeshAlgo, DelaunayNodeInsertionMeshAlgo, DelaunayDeflectionControlMeshAlgo as generics over the range splitter (notes, section 6); the range splitters (UVParam, NURBS, Cone, Cylinder, Sphere, Torus, Extrusion, Undefined, BoundaryParams); MeshAlgoFactory, DelabellaBaseMeshAlgo, DelabellaMeshAlgoFactory, FaceDiscret; Context, DiscretRoot, IncrementalMesh, IncrementalMeshFactory, DiscretAlgoFactory, DiscretFactory, FastDiscret; XBRepMesh_Factory. The public API filled in (decision 489), the determinism test (2.2) | 5.0 | 4B, 4C | **Oracle milestone 1** (section 5). GT: BRepMesh_BaseMeshAlgo, BRepMesh_DiscretAlgoFactory, BRepMesh_IncrementalMesh (its Fuse case waits for phase 6). Oracle: section 3 in full; BRepGProp and BRepBndLib with `useTriangulation` on the port's meshes against native's on native meshes |

### Wave 4

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **4L2** Draw-lite: incmesh and the corpus | `incmesh` with all its options (`-a`, `-min`, `-parallel`, `-relative`, `-algo`, `-surf_def_all`, `-surf_def_off`, `-int_vert_off`, `-force_face_def`, `-decrease`, `-adjust_min`, ...), rebinding the name to the meshed result and propagating to variables that share its sub-shapes (decision 495); `tessellate`, `mpincmesh`, `mpnames`, `mpsetdefaultname`, `mpgetdefaultname`; `vdisplay` meshing as AIS does, so the shading grids run (decision 495) | ≈ 0.8 (C#) | 4D, 4L1 | **The corpus milestone** (section 5). Per command against DRAWEXE output captured once |
| **4W** Viewer and WebAssembly | A managed equivalent of build123d.NET's `b3d_mesh_extract` (owner question 2): every face's located nodes, normals (BRepLib_ToolTriangulatedShape.ComputeNormals on an unattached copy, design 5) and outward triangles, every edge's polyline (from a face's polygon on triangulation, else the mesher's 3D polygon, else GCPnts_TangentialDeflection), in the order of the shape's faces and edges. The benchmark project (2.5). The WebAssembly spike (2.4): a `samples/OcctNet.WasmDemo` built as Bento.Web is (plain WebAssembly, interpreter, single-threaded, trimmed) that reads a `.brep`, meshes it, extracts the buffers and draws them with WebGL | ≈ 0.5 (C#) + the sample | 4D | Oracle: the managed extraction's buffers against native `Extras.ExtractMesh` on the same native shapes, float for float. Benchmarks against native. In the browser: the same buffers as on win-x64 (bit-identical share reported), timings, trimmed size, deepest recursion |

### Totals

| Wave | Units | Lines (k) |
|---|---|---:|
| 1 | 4A, 4S, 4L1 | 22.1 + Draw-lite |
| 2 | 4B, 4C | 11.0 |
| 3 | 4D | 5.0 |
| 4 | 4L2, 4W | Draw-lite, viewer, sample |

That is about 37,000 lines of OCCT: TKMesh 19,900, TKXMesh 100, ShapeAnalysis 14,100 and TKernel 2,900. Add about
2,500 lines of C# for Draw-lite, the extraction and the sample.

**The critical path** is 4A → 4B → 4D → 4L2 / 4W.

- **4A is the pilot:** the freezing and API design lands there, and should be reviewed before 4D.
- **Three units can start at once:** 4A, 4S and 4L1. 4S is the largest unit and sits off the critical path until 4C.
- **4B's independent part can start before 4A merges:** the plain types, CircleTool, VertexTool,
  DataStructureOfDelaun and Delabella's library. It merges after 4A, because Delaun calls BRepMesh_GeomTool's
  statics.

## 5. Milestones

1. **After 4D: shapes mesh as native OCCT meshes them.**
   - Every `.brep` of data/occ, and every mesh-group data file in the public dataset, meshes with the corpus's
     deflections (and three others, absolute and relative, serial and parallel) into BRep text byte-identical to
     native.
   - At least 99% of faces are bit-identical node for node. Every other face is attributed (2.3) and valid.
   - Random native shapes (primitives, sweeps, Booleans, fillets built natively) do the same.
   - The port's parallel result equals its serial result on all of them, bit for bit.
   - Meshing a frozen shape never writes it, and meshing an adequately meshed shape returns it unchanged.
2. **After 4L2: the corpus.**
   - Every runnable case with a mesh command passes with the reference values in its script.
   - Those are the 313 cases the scan finds (168 with their data), and the mesh group's `standard_incmesh`,
     `standard_incmesh_parallel`, `advanced_incmesh` and `advanced_incmesh_parallel` cases whose data is public
     (about 165), plus the shading grids if decision 495's `vdisplay` meshing holds.
   - Expected failures are OCCT's own TODOs (the mesh `end` script turns `bug_*` variables into TODOs) and missing
     data.
3. **After 4W: Bento's viewer without native code.**
   - The managed extraction gives build123d.NET's `ShapeMesh` buffers equal to native's, float for float, on the
     oracle's shapes.
   - Bento's viewer can mesh any shape OCCT.NET holds: shapes read from BRep, built with the phase 3 makers and
     TKPrim, or passed from native as BRep text.
   - The desktop targets of 2.5 are met or the gap is explained.
4. **After 4W: the WebAssembly demo.**
   - The sample meshes and draws data/occ shapes in a browser, single-threaded, under Bento.Web's build settings.
   - It reports its timings, its trimmed size, and how many meshes are bit-identical to win-x64's.

## 6. Notes for porters

- **Phase 3's notes apply** (phase3-plan.md, section 5): shapes are structs, maps of shapes take a comparer, writing
  into frozen inputs means copying first, and every twin goes in decisions.md.
- **Handles of IMeshData.** `IMeshData::IFaceHandle` and the other handles are references to the classes.
  - `IMeshData::IFacePtr` (a raw pointer used as a map key) is the same reference, in a map compared by reference
    that iterates in insertion order (decision 490).
  - `NCollection_Shared<X>` is `X`.
  - The allocator arguments go.
- **Multiple inheritance.** `IMeshData_Face`, `IMeshData_Wire` and `IMeshData_Edge` derive from
  `IMeshData_TessellatedShape` and the mix-in `IMeshData_StatusOwner`. StatusOwner becomes an interface whose state
  (the status mask) lives in each of the three classes, with the same members.
- **The templates.** C# can't derive from a type parameter.
  - `BRepMesh_NodeInsertionMeshAlgo<RangeSplitter, BaseAlgo>`, `BRepMesh_DelaunayNodeInsertionMeshAlgo` and
    `BRepMesh_DelaunayDeflectionControlMeshAlgo` are instantiated over two bases:
    - `BRepMesh_DelaunayBaseMeshAlgo` (Watson, the default),
    - `BRepMesh_CustomDelaunayBaseMeshAlgo<BRepMesh_DelabellaBaseMeshAlgo>` (Delabella, `-algo 1`).
  - **Recommended:** port each as a generic over the range splitter only, deriving from the Watson base, plus a
    second set for the Delabella base written once by a scratch script from the same bodies (decision 202's
    approach to `.gxx` instantiations).
  - The range splitter's constraint is `where RangeSplitter : BRepMesh_DefaultRangeSplitter, new()`. Its methods
    that the subclasses redefine are `virtual` in OCCT, so dispatch is unchanged. Check every method a subclass
    hides without `virtual`: through a generic, C# calls the base's.
  - `BRepMesh_EdgeParameterProvider<ParametersCollection>` is a generic.
  - `IMeshData_ParametersListArrayAdaptor<T>` is a generic over the list.
- **Functors.** `OSD_Parallel::For(0, n, *this, isSequential)` → `OSD_Parallel.For(0, n, process, isSequential)` with a
  method group (decision 487). Keep each `!myParameters.InParallel`, including OCCT's forced `true` in the
  post-processor.
- **Exceptions in loop bodies.** OCCT's `try { OCC_CATCH_SIGNALS ... } catch (Standard_Failure&)` sets a face's
  `IMeshData_Exception` status. Keep it per face. An exception that escapes a parallel loop is unwrapped from
  `AggregateException` to its first inner exception (decision 431).
- **Sorting.**
  - `std::make_heap` + `std::sort_heap` → the MSVC-order helper of 4B (decision 491), never `Array.Sort`, which
    orders ties differently.
  - `std::sort` on doubles or on three integers (NURBSRangeSplitter, Triangle) can use `Array.Sort`.
  - Delabella's `std::sort` of points: use the helper or record the difference.
- **Writing into shapes.**
  - Only `BRepMesh_ShapeTool` attaches meshes, through the non-freezing attach (decision 488). `BRepTools::Update(face)`
    in ShapeVisitor returns early on frozen faces, as in phase 3.
  - Anything else that writes into the shape being meshed goes in decisions.md.
- **Process-wide state** (decision 492).
  - `BRepMesh_IncrementalMesh::IS_IN_PARALLEL` and `BRepMesh_DiscretFactory`'s default name stay statics with
    internal setters, as `BRepLib.Precision` (decision 415).
  - `CSF_MeshAlgo` isn't read.
  - `BRepMesh_DiscretAlgoFactory.RegisterFactory` and `UnregisterFactory` are internal. The two factories register
    when the type initialises.
- **Messages.** `Message::SendWarning`, `Send` and `DefaultMessenger()` go to `Message`'s process-wide messenger
  (decision 493). Draw-lite installs one that prints as DRAWEXE does.
- **Debug code.** `#ifdef OCCT_DEBUG`, `DEBUG_HEALER` and `CHECK_PERF` blocks go. Public dump methods
  (`BRepMesh_MeshTool::DumpTriangles`, DataStructureOfDelaun's) are ported, writing through BRepTools.
- **Delabella's licence.** `delabella.cpp` is MIT-licensed third-party code inside OCCT. Its C# port keeps the MIT
  notice at the top of the file, beside OCCT's provenance. The notice is added to a THIRD-PARTY-NOTICES file
  (decision 497).

## 7. Questions for the owner

These are in [open-questions.md](open-questions.md) as 11 to 15, with the provisional answers the units follow.

1. **ShapeAnalysis in phase 4.** BRepMesh orders wires with `ShapeAnalysis_Wire::CheckOrder` and finds the outer wire
   with `ShapeAnalysis::OuterWire`. Should phase 4 port those classes whole (unit 4S, 14,100 lines of design.md's
   phase 7)? The alternative is only the five methods BRepMesh calls (about 600 lines, the rest `TODO(phase 7)`).
   Recommended: whole classes. They are needed again in phases 5 and 7, and a partial `ShapeAnalysis_Wire` would be
   reworked then.
2. **Where the viewer's mesh extraction lives.** build123d.NET's `ShapeMesh` is filled by its native shim
   (`b3d_mesh_extract`, 225 lines). Unit 4W writes the managed equivalent. Should it be:
   - an OCCT.NET addition (a public helper in OcctNet.ModelingAlgorithms, outside OCCT's names), which build123d.NET
     calls when it moves; or
   - build123d.NET's own C# from the start?

   Should build123d.NET's viewer path switch to OCCT.NET already in phase 4 (native shape → BRep text → managed mesh
   and extraction), ahead of phase 9? Recommended: the helper in OCCT.NET, and the switch left to you.
3. **Binding the mesher's parameters natively.** build123d.NET binds only BRepMesh_IncrementalMesh's five-argument
   constructor. Ask it to bind `IMeshTools_Parameters`, the `(shape, parameters)` constructor,
   `Poly_PolygonOnTriangulation` and `Poly_Polygon3D`? Without them, MinSize, the interior deflections, the
   surface-deflection controls and Delabella are checked only by the corpus and the GTests.
4. **The WebAssembly demo and the targets.**
   - The demo could be a standalone sample in this repository (recommended), or a Bento.Web page.
   - It could run under Bento.Web's interpreter only, or AOT as well, which needs the wasm-tools workload.
   - Are section 2.5's provisional targets the right ones? 2× native on the desktop, under 100 ms for a typical
     model, under 2 s in the browser.
5. **Meshing a frozen shape through OCCT's own class.** `new BRepMesh_IncrementalMesh(frozenShape, ...)` meshes a
   copy, and `Shape()` returns the meshed result (recommended: build123d.NET's calls barely change). The alternative
   is to throw on frozen input and offer only the functional `Meshed`, which is stricter but breaks OCCT's usual
   call.
