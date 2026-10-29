# Unit 3K, BRepExtrema (fork): decisions to fold into decisions.md

Placeholders 3101-3149; renumber on merge.

3101. **BRepExtrema's classes are public working objects** (decision 21) with OCCT's API: DistShapeShape, DistanceSS,
      the Ext tools (ExtPC, ExtPF, ExtCC, ExtCF, ExtFF), TriangleSet, OverlapTool, SelfIntersection, ShapeProximity,
      ProximityValueTool, ProximityDistTool, ElementFilter and BRepExtrema_VertexInspector; BRepExtrema_Poly is a
      static class. OCCT uses the Ext tools by value, but they hold Extrema state and adaptors, so they are sealed
      classes. `BRepExtrema_SolutionElem` has no mutators and is a readonly struct with field equality (`default` is
      OCCT's empty constructor); `BRepExtrema_SupportType` is an enum; `BRepExtrema_UnCompatibleShape` derives from
      Standard_DomainError. OCCT's adaptor handles copied from locals (`new BRepAdaptor_Curve(Curv)`) are the locals
      themselves (decision 394).
3102. **The thread count is the machine's.** OCCT splits DistShapeShape's work into tasks by
      `OSD_ThreadPool::DefaultPool()->NbThreads()` (one per logical processor) in single-threaded mode too, and the
      split decides which pairs of sub-shapes each task measures (each task stops early on its own best distance) and
      the order of the solutions. The port takes `Environment.ProcessorCount`, as GeomLib_CheckCurveOnSurface does, so
      on the same machine it measures and orders as native OCCT (oracle: every value bit-identical, solutions in
      order). `OSD_Parallel::For(..., !myIsMultiThread)` is `Parallel.For` or a plain loop; the functors are private
      sealed classes with an `Invoke(int)`; SolidTreatment's atomics are volatile fields copied back after the loop, its
      mutex a `Lock`. In multi-threaded mode the inner solutions depend on which task finds a point inside first (and
      how many tasks do before the others see the flag), on both sides; the oracle checks those as valid rather than
      equal. `std::stable_sort` is LINQ's stable `OrderBy`.
3103. **Supports and frozen inputs.** DistShapeShape and DistanceSS only read their shapes (and the Proximity tools
      only their meshes). The supports handed out are the inputs' own sub-shapes (TopExp maps with
      TopTools_ShapeMapHasher), except where DistanceSS trims an infinite edge or face facing a finite shape: the
      trimmed copy (`BRepBuilderAPI_MakeEdge(Geom_TrimmedCurve)`, `BRepBuilderAPI_MakeFace(RectangularTrimmedSurface,
      tol)` in OCCT) is a public support, so it is built with the maker's freezing form. Until BRepBuilderAPI merges
      these are `BRepLib_MakeEdge(...).Edge()`, `BRepLib_MakeFace(...).Face()` and `BRepLib_MakeVertex(...).Vertex()`
      (the local vertices of the C0 paths too), marked `TODO(3K-merge)`: BRepBuilderAPI's makers wrap exactly these.
3104. **DistanceSS** is a public sealed class computing in its constructor. `Seq1Value()`/`Seq2Value()` return
      `IBentoList` copies; DistShapeShape reads the sequences through internal `InternalSeq1Value()`/`InternalSeq2Value()`
      and copies them as OCCT does. The default deflection is the literal 1e-7 (`Precision::Confusion()` isn't a C#
      constant). OCCT's static helpers (TRI_SOLUTION, MIN_SOLUTION, TRIM_INFINIT_EDGE/FACE, PERFORM_C0, isOnBoundary)
      are private statics; their `bool&`/`TopoDS_Edge&` outputs are `ref` (read or left as the caller set them) and
      TRIM_INFINIT_FACE's flag is `out`. `double arrU[8]` is a stack span (zeroed, see occt-issues).
3105. **Signatures.** Outputs written unconditionally are `out` (ParOnEdgeS1/2, ParOnFaceS1/2, SolutionElem's
      EdgeParameter/FaceParameter, the Ext tools' Parameter and TrimmedSquareDistances, TriangleSet.GetVertices and
      GetVtxIndices, the proximity points and statuses); `BRepExtrema_Poly.Distance(S1, S2, ref P1, ref P2, out dist)`
      leaves the points unchanged when it fails, as OCCT; `BRepExtrema_ProximityValueTool.Perform(ref theTolerance)`
      writes it only when done. `Dump(TextWriter)` writes OCCT's text with reals at the stream's default precision
      (`Standard_Stream.FormatReal(x, 6)`). DistShapeShape's obsolete flag and algorithm are kept unread with CS0414
      suppressed (as decision 392).
3106. **The mesh tools over the ported BVH.** `BRepExtrema_TriangleSet` derives from
      `BVH_PrimitiveSet<double, NCollection_Vec3<double>>` (OCCT's BVH_PrimitiveSet3d) with OCCT's linear builder; OCCT's
      BVH typedefs are written out (`BVH_Vec3d` is a file-level alias). `GetVertices()` returns the set's own vector, a
      working object's array as OCCT's const reference. Its triangle map is an `NCollection_DataMap<int, int>` (no
      `ReSize`). `Init` builds the BVH (OCCT's `Standard_ASSERT_RETURN(!BVH().IsNull())`). `BRepExtrema_OverlapTool`
      derives from `BVH_PairTraverse<double, Vec3d, double>` (OCCT's set-less form) and traverses the sets' trees;
      its `RejectNode` writes a metric of 0 where OCCT leaves it unwritten. `BRepExtrema_ProximityDistTool` derives from
      `BVH_Distance<double, Vec3d, Vec3d, BRepExtrema_TriangleSet>`; its `ComputeDistance` hides BVH_Distance's
      (`protected new`), as OCCT's does; `PrjState` is a readonly struct. `BRepExtrema_ElementFilter` is a class with a
      virtual `PreCheckElements`, which SelfIntersection overrides publicly (OCCT's override is protected; C# can't
      narrow it). `BRepExtrema_VertexInspector` is a sealed class implementing `INCollection_CellFilter_Inspector<int,
      gp_XYZ>` (the filter's `Inspect` takes it by `ref`). Standard_ASSERT_RAISE throws Standard_ProgramError, as
      OCCT's release build does. The prism's fixed arrays are small heap arrays (a struct with arrays), C arrays of
      vectors are stack spans or collection expressions, `DBL_EPSILON` a constant.
3107. **Tests.**
      - Ported GTest: BRepExtrema_DistShapeShape (3), with the BRepLib makers in place of BRepBuilderAPI's
        (`TODO(3K-merge)`); the edges whose 3D curve OCCT removes are made with `InternalEdge()` and frozen after.
      - Unit tests (32 with the GTest): vertex/vertex, vertex/edge (foot inside and beyond the end), vertex/face,
        skew and parallel edges, two equidistant edges, edge/face above and through, parallel faces, inner solutions
        in a box, two boxes single- and multi-threaded with inputs unchanged (ShapeSnapshot), located shapes,
        deflection, an infinite line (trimmed, frozen support), errors and Dump, each Ext tool, DistanceSS's box
        rejection; TriangleSet indices with an edge polygon, ShapeProximity overlap (tolerance and exact), proximity
        values of edges and faces with refinement, SelfIntersection exact and with tolerance, BRepExtrema_Poly,
        orientations; and four pinned OCCT issues.
      - Oracle tests (`tests/OcctNet.Oracle.Tests/BRepExtrema`): BRepExtrema_DistShapeShape on 800 pairs of the
        ClassificationScenes (shapes, vertices, edges, faces against each other, points in and around solids,
        infinite lines and planes): 641 computations, 667 solutions (84 inner), 14,821 values all bit-identical, 52
        trimmed supports byte-identical as BRep text, 43 multi-threaded inner solutions checked as valid; and on 160
        pairs of random native shapes (primitives, sweeps, Booleans, fillets, lofts) read by both sides from native
        OCCT's file, placed apart or overlapping: 3,907 values, all bit-identical. Each run draws the deflection, the
        threading mode and the obsolete flag and algorithm. The proximity, overlap and self-intersection tools have no
        native binding and are unit-tested.
3108. **For later units.**
      - 3K merge: replace the four `TODO(3K-merge)` makers in BRepExtrema_DistanceSS (and those of the GTest and the
        oracle test) with BRepBuilderAPI's.
      - 3L2 (`distmini name S1 S2 [deflection] [-parallel]`, BRepTest_ExtremaCommands.cxx): LoadS1, LoadS2,
        SetDeflection (default Precision::Confusion()), SetMultiThread, Perform with a Draw progress indicator; if
        done, prints `"distmini" command returns:` then binds the number `name_val` (Value) and, per solution, a
        vertex at PointOnShape1 (`BRepLib_MakeVertex`) when Value <= 1e-9, else the edge `BRepLib_MakeEdge(P1, P2)`,
        named `name` for the first and `name<i>` after, printing each name, then `Output is complete.`; otherwise
        `problem: no distance is found`. The Dump is only in OCCT_DEBUG builds. The solutions' order is OCCT's when the
        processor count matches native DRAWEXE's (3102), so the names bind the same edges. The same file has
        `proximity` (BRepExtrema_ShapeProximity, `-tol`/`-value`/`-profile`) and `selfintersection` (OCCT's
        BRepExtrema_SelfIntersection), both ported here.
