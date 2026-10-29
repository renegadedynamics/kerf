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
