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
