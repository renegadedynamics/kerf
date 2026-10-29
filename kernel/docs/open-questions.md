# Open questions for the owner

Choices made while working unattended that need your decision. Each has a provisional answer the work follows
until you say otherwise; changing one later is cheap unless the entry says so. Numbers refer to
[decisions.md](decisions.md).

## Phase 3 (topology), from the design in [phase3-design.md](phase3-design.md)

1. **Free flag on frozen shapes.** OCCT clears a sub-shape's `Free` flag when it is put into a compound, which
   changes the shared sub-shape (and its BRep text). *Provisional:* a frozen child keeps its flag, so a shape's BRep
   text never changes once published; the cost is that a file written after building a compound from frozen shapes
   shows `Free = 1` where native OCCT shows 0. Byte-identical round trips of files *read* are unaffected.
   Alternative: clear it as OCCT does (OCCT's bytes in every history, but a shared shape's text changes).
   *Evidence from unit 3I (decision 414 onwards):* the BRepLib makers reuse the caller's frozen sub-shapes, so 570
   of 2,015 maker outputs in the oracle differ from native only in the Free column, among them every face made
   from a planar wire and every solid made from those faces. Everything else in those files is identical.
   *Unit 3M (decisions 467, 468):* prisms and revolutions of frozen profiles share the profile's unchanged
   sub-shapes: 122 of 474 prisms and 31 of 443 revolutions in the oracle differ from native only in the Free column
   (and 184 box faces asked for before the box was built, which the port hands out already in the shell).
2. **Scope of phase 3.** BRepClass, BRepCheck, BRepClass3d and BRepExtrema need about 38k lines of TKGeomAlgo
   intersections (IntRes2d, IntCurve, Geom2dInt, IntCurveSurface, ...) plus BRepLib and BRepBuilderAPI's makers,
   which [design.md](design.md) put in phase 5; and most topology GTests and 3,900 corpus cases need `MakeBox` /
   `pcylinder` (TKPrim). *Provisional:* pulled into phase 3 (units 3D–3K, and 3M for TKPrim), ~138k lines in all.
   design.md's phase table is updated to match.
3. **OCCT's test data.** *Decided by the owner (29 October): download it.* `tools/get-occt-testdata.ps1` fetches the dataset OCCT V8_0_1's own CI uses (opencascade-dataset-7.9.0, 99 MB, SHA-256 in occt-version.json) into `.deps/occt-testdata`. It holds 2,907 of the 6,939 files the corpus names (1,287 of 4,039 .brep, 870 of 1,156 .rle, 170 of 414 STEP); the rest are not published. MSBuild property `OcctTestDataDir` points at it.
4. **A native oracle for BinTools.** build123d.NET doesn't bind BinTools (binary BRep). Changing that repository is
   yours to decide. *Provisional:* BinTools is checked by round trips and against the format as OCCT writes it, not
   against native output. Options: bind `BinTools` there, or add a binary twin of its `b3d_brep_write`/`read`.
5. **Public construction once `BRep_Builder` is internal.** *Provisional:* a functional `TopoDS_Builder.MakeCompound`
   plus the BRepBuilderAPI makers is the public surface; build123d.NET will tell us what else it needs.

## Phase 4 (meshing), from the plan in [phase4-plan.md](phase4-plan.md)

The numbers in brackets are the plan's (section 7).

11. **ShapeAnalysis in phase 4** (plan 1). BRepMesh needs `ShapeAnalysis_Wire::CheckOrder`,
    `ShapeAnalysis_WireOrder`, `ShapeExtend_WireData`, `ShapeAnalysis::OuterWire` and `ShapeAnalysis_Edge`.
    *Provisional:* unit 4S ports the classes whole, with ShapeAnalysis_Curve, Surface, TransferParameters(Proj) and
    ShapeBuild_Edge that they reach: 14,100 lines of design.md's phase 7. design.md's phase table is updated to
    match. Alternative: only the methods BRepMesh calls (about 600 lines).
12. **The viewer's mesh extraction** (plan 2). *Provisional:* a managed equivalent of build123d.NET's
    `b3d_mesh_extract` goes into OcctNet.ModelingAlgorithms as a public helper (an addition, not a port), checked
    float for float against the native shim. Whether build123d.NET's viewer switches to it in phase 4 (through BRep
    text) or in phase 9 is yours to decide.
    *Evidence (unit 4W, decisions 4801–4806):* OcctNet.MeshExtraction.Extract gives build123d.NET's MeshBuffers
    layout and order; its buffers are float-identical to the native shim's on every compared face and edge (native's
    mesh read back as BRep: 1,738 faces, 9,724 edges; mesh and extract end to end on data/occ, the mesh group's 82
    files and native shapes with fillets, Booleans, mirrored and reversed parts and free edges: 1,448 faces, 6,509
    edges; unmeshed edges by GCPnts: 4,786). Extraction is about 2% of mesh-and-extract. On the desktop the port's
    serial mesh-and-extract is 0.9–1.5× native's, its parallel one 1.4–4.5× (allocation-bound, decision 4806); BRep
    text read by the port costs 2–35 ms on typical models.
13. **A native oracle for the mesher's parameters** (plan 3). build123d.NET binds only BRepMesh_IncrementalMesh's
    five-argument constructor. *Provisional:* the oracle uses that constructor, and the other parameters (MinSize,
    interior deflections, surface-deflection control, Delabella) are checked by the corpus and the GTests. Options:
    bind `IMeshTools_Parameters`, the `(shape, parameters)` constructor, `Poly_PolygonOnTriangulation` and
    `Poly_Polygon3D` there.
14. **The WebAssembly demo and performance targets** (plan 4). *Provisional:*
    - a standalone sample, `samples/OcctNet.WasmDemo`, built as Bento.Web is (plain WebAssembly, the interpreter,
      single-threaded, trimmed), timed with AOT too where the workload is installed;
    - targets: the port within 2× native serial on the desktop; a typical model meshed and extracted in under
      100 ms on the desktop and under 2 s in the browser.

    *Evidence (unit 4W, decisions 4805–4809):*
    - The sample builds with the SDK alone, as Bento.Web does (no workload is installed, so AOT is not measured).
      Trimmed, the OcctNet assemblies are 2.39 MB (1.08 MB Brotli); rooted, 5.80 MB (2.14 MB).
    - Browser (interpreter, headless Chrome): typical models mesh and extract in 0.24–0.7 s warm; 223–324-face models
      in 1.9–3.7 s; a single 30k-node B-spline face in 8.8 s. 8–19× the desktop JIT. terrain.brep runs out of memory
      (5.6 GB of heap on the desktop too). 92.5% of faces and every edge bit-identical to win-x64; the rest are
      tori, spheres, planes bounded by circles and cylinders' normals (transcendental functions).
    - Desktop: serial within 2× native (0.9–1.5×; one huge face 2–2.7×); parallel 1.4–4.5× native, because the
      port allocates 12–260 MB per mesh and workstation GC serialises the threads (server GC recovers most of it).
      Typical models: 13–65 ms serial, 7–44 ms parallel, under 100 ms; Motor-c (223 faces, 26k nodes) 155–209 ms.
    - Options: a follow-up that pools NCollection_CellFilter's nodes and BRepMesh's per-face structures (results
      unchanged); server GC or a larger gen0 budget in Bento's host; a lower 2 s ambition or a coarser default
      deflection in the browser.
15. **Meshing a frozen shape through OCCT's class** (plan 5). *Provisional:*
    `new BRepMesh_IncrementalMesh(frozenShape, ...)` meshes an exact copy, and `Shape()` returns the frozen meshed
    result, sharing what meshing left unchanged; a functional `Meshed` twin as well. Alternative: throw on frozen
    input.

## Earlier judgement calls worth a look

6. **Decision 94: abstract geometry mutators are `protected internal`**, so callers can derive their own curves and
   surfaces as OCCT allows, while objects they are handed still can't be changed.
7. **Decision 44: BVH's trees, sets and builders keep OCCT's public mutators** (like math and BSplCLib, decision 21);
   only BVH's value types follow the internal-mutator rule.
8. **Decision 206: `FEmTool_Curve::D2` on a degree-1 element** reads memory before its coefficients in OCCT, so native
   C0 variational fits differ from run to run. The port reads the same in-array values and zeros before the array:
   deterministic, but it can't be oracle-tested.
9. **Decision 113 and 198: AdvApp2Var's thread safety.** OCCT's Hermite cache is a racy shared static; the port's is
   per thread. Where OCCT overruns its work zone (and crashes), the port allocates the excess and carries on.
10. **Fixing OCCT's bugs.** [occt-issues.md](occt-issues.md) lists every bug found, all reproduced so results match
    native OCCT. You said to fix them later once consistent behaviour is proven; several (offset-curve D3,
    AdvApp2Var's error estimate, `GeomLProp::Continuity`, `Extrema_ExtCC` not clearing old solutions) affect results
    that Bento users would see, and are good first candidates.
