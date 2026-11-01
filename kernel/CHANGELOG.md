# Changelog

All notable changes to the OCCT.NET packages. Versions follow [SemVer](https://semver.org); before 1.0 a minor
version may break the API.

## 0.1.0 (unreleased)

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
