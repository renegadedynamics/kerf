# Phase 2 plan: TKG2d, TKG3d, TKGeomBase

This plan splits the geometry toolkits into work units that can be ported in parallel. It covers 410,000 lines of
OCCT, of which 148,000 are generated numeric tables. The packages of these toolkits call each other in cycles. To
break the cycles, branch `phase2/skeleton` ported an abstract layer first: base classes, interfaces and a few
leaves. Every unit below then depends only on:

- Foundation,
- the abstract layer, and
- units of earlier waves.

Units in the same wave are independent.

Sizes are physical lines of `.hxx/.cxx/.lxx/.pxx/.gxx`, comments included, as `tools/port-status` counts them.
The dependencies were taken from identifiers used in the code bodies, not from `#include`s. Code under
`#ifdef OCCT_DEBUG`, `DRAW`, `DEB` and `DEBUGLINES` was removed first, and the notable edges were checked at their
call sites.

## 1. What phase 2 starts from

- **Foundation** (merged): gp, NCollection, math and the Math* packages, ElCLib, ElSLib, GeomAbs, TopLoc, PLib,
  BSplCLib, CSLib, Bnd, BVH, Poly, Precision, Message.
- **Ported in parallel**: BSplSLib and Convert. They are not on this branch. Units marked **[BSplSLib]** or
  **[Convert]** start once those packages are merged.
- **The abstract layer**, branch `phase2/skeleton`, 10,700 lines:

  | Package | Ported |
  |---|---|
  | TopAbs | Everything |
  | Geom | Geometry, Curve, Surface, BoundedCurve, BoundedSurface, Conic, ElementarySurface, SweptSurface, Point, Vector, AxisPlacement, CartesianPoint, Direction, VectorWithMagnitude, Axis1Placement, Axis2Placement, Transformation, UndefinedDerivative, UndefinedValue |
  | Geom2d | The same set, except the surfaces |
  | Adaptor2d | Curve2d, Line2d |
  | Adaptor3d | Curve, Surface, HVertex, InterFunc |
  | GeomEval, Geom2dEval | RepCurveDesc, RepSurfaceDesc, RepUtils |

  The Geom/Geom2d classes use the "evaluation representation" hook, which only references the base classes. Porting
  it with the abstract layer removes the Geom ⇄ GeomEval and Geom2d ⇄ Geom2dEval cycles.

  The bases refer to Geom_BezierCurve, Geom_BSplineCurve, Geom_OffsetCurve, Geom_BezierSurface,
  Geom_BSplineSurface, Geom2d_BezierCurve and Geom2d_BSplineCurve. Each of these exists as an empty abstract
  **placeholder** in the file its port will replace (decision 99).
- **Outside these toolkits:** the only package that real code uses is `OSD_ThreadPool`, in
  GeomLib_CheckCurveOnSurface; use `Parallel.For`. Draw, DrawTrSurf and OSD appear only in debug code.
- **Not needed in TKGeomBase:** the `DumpJson` code. TKG2d has it in 27 files and TKG3d in 50; it is left out by
  the rules.

## 2. Packages

"(a)" means the dependency is available (Foundation or the abstract layer). Other names are packages of these
toolkits. "Oracle" means classes bound by build123d.NET, from `bindings/occt.txt` with wrappers in
`src/Build123d.Occt/Generated`. "GT" is the number of lines of OCCT GTests.

### TKG2d (23,500 lines)

| Package | Lines | Real dependencies | Oracle | GT |
|---|---:|---|---|---:|
| Geom2d | 11,019 | (a) gp, BSplCLib, PLib, ElCLib; Geom2dEval Rep* (a) | 11 of 22. Not bound: Bezier, Hyperbola, Parabola, OffsetCurve | 3,800 |
| Geom2dEval | 4,405 | Geom2d bases (a) | none | 1,680 |
| Adaptor2d | 1,720 | Geom2d (Bezier/BSpline handles); OffsetCurve uses `Geom2d_OffsetCurveUtils.pxx` | Curve2d | 440 |
| Geom2dAdaptor | 1,826 | Geom2d (pervasive), Adaptor2d, Geom2dEval Rep* | Curve | 240 |
| Geom2dGridEval | 3,491 | Geom2d (+ OffsetCurveUtils), Adaptor2d, Geom2dAdaptor (1 downcast) | none | 1,330 |
| Geom2dHash | 1,027 | Geom2d (one hasher per concrete class) | none | 650 |

### TKG3d (70,300 lines)

| Package | Lines | Real dependencies | Oracle | GT |
|---|---:|---|---|---:|
| Geom | 30,528 | (a) gp, BSplCLib, PLib, ElCLib/ElSLib, CSLib (OffsetSurfaceUtils); **BSplSLib** (Bezier/BSpline surfaces, OsculatingSurface); **Convert** (OsculatingSurface, 1 use); GeomEval Rep* (a); **AdvApprox + GeomAdaptor_Surface, only in `Geom_OffsetSurface::UIso/VIso`** | 30 of 39 (all curves and surfaces). Not bound: placements, vectors, Transformation | 5,300 |
| GeomEval | 9,820 | Geom bases (a) | none | 3,440 |
| Adaptor3d | 7,038 | Geom (handles; IsoCurve and CurveOnSurface build B-splines), Adaptor2d, TopAbs, math; **GeomAdaptor_Curve, only in `Adaptor3d_HSurfaceTool.cxx:149`** | Curve, Surface | — |
| GeomAdaptor | 7,175 | Geom (pervasive, + the `*Utils.pxx` of offsets, revolution and extrusion), Adaptor3d, GeomEval Rep*, BSplCLib, **BSplSLib** | Curve, Surface | 950 |
| GeomGridEval | 10,200 | Geom (+ Utils.pxx), GeomAdaptor (dispatch), Adaptor3d, BSplCLib, BSplSLib | none | 6,400 |
| GeomHash | 2,800 | Geom, Poly | none | 1,640 |
| AdvApprox | 2,320 | (a) only: PLib, math, BSplCLib, **Convert** | none (indirect: GeomConvert_ApproxCurve) | — |
| TopAbs | 418 | done | none | — |

### TKGeomBase (317,000 lines, of which 148,000 are generated tables)

| Package | Lines | Real dependencies | Oracle | GT |
|---|---:|---|---|---:|
| AdvApp2Var | 51,697: 23,300 tables, 28,400 f2c code (MathBase 10,900, ApproxF2var 9,000, SysBase 3,200) | AdvApprox_Cutting; Geom_BSplineSurface (ApproxAFunc2Var only) | none | 380 |
| AppCont | 125,352: 124,400 tables, 950 code | AppParCurves | none | 280 |
| AppDef | 10,572: 24 `*_0.cxx` instantiations, about 4,500 lines with headers; Variational 3,700 | AppParCurves (pervasive), FEmTool, Approx (ParametrizationType + 2 .gxx) | indirect: GeomAPI_PointsToBSpline | — |
| AppParCurves | 6,330: 4,550 in 6 .gxx | (a) only | none | — |
| Approx | 10,308: 3,860 .gxx | AppParCurves, Adaptor3d/2d, AdvApprox, Geom/Geom2d, AppCont, adaptors; **GeomLib, Extrema, GCPnts, GeomConvert only in SameParameter, CurvlinFunc and CurveOnSurface** | indirect: BRepLib::SameParameter | — |
| BndLib | 2,537 | BndLib.cxx is elementary (Bnd, ElCLib); the Add2dCurve/Add3dCurve/AddSurface wrappers call GeomBndLib | indirect: BRepBndLib | 1,480 |
| GeomBndLib | 9,343 | Geom, Geom2d, Adaptor3d/2d, GeomAdaptor, Geom2dAdaptor, GeomGridEval_Surface, BndLib (torus), MathOpt | indirect: BRepBndLib | 3,570 |
| CPnts | 1,665 | Adaptor3d, Adaptor2d | none | — |
| GCPnts | 5,022 | Adaptor3d/2d (pervasive), CPnts, Geom/Geom2d BSpline/Bezier | AbscissaPoint, QuasiUniformDeflection, UniformDeflection | 80 |
| Extrema | 22,611 (72 headers; 8 templates over the curve type, 3d and 2d instances) | Adaptor3d/2d (pervasive), GeomAdaptor (ExtPS, ExtPExtS, ExtPRevS), Geom, IntAna (ExtElCS only), GeomGridEval_Surface (GenExtPS/CS/SS), GCPnts (CurveTool, GGenExtCC), BndLib_AddSurface (ExtCS), IntAna2d (ExtElC) | ExtPC, POnCurv; indirect: GeomAPI_ProjectPointOnCurve/Surf, Geom2dAPI_ProjectPointOnCurve, BRepExtrema | 60 |
| ExtremaPC | 3,990 | Geom, GeomAdaptor (Curve, TransformedCurve), Adaptor3d, GeomGridEval, MathUtils/MathPoly/MathRoot | none | 8,800 |
| FEmTool | 3,114 | (a) only | none | — |
| GC | 5,462 | gce (pervasive), Geom, Geom2d, Extrema_ExtElC/ExtPElC (2 classes), IntAna2d (1) | Root, MakeArcOfCircle/Ellipse/Hyperbola/Parabola | 370 |
| GCE2d | 432 | only `using` aliases of GC's 2d makers | — | — |
| gce | 4,513 | Extrema_ExtElC (MakeCirc), IntAna2d (MakeCirc2d) | Root, MakeLin | 640 |
| GProp | 2,776 | (a) only | GProps, PrincipalProps; indirect: BRepGProp | 190 |
| Geom2dConvert | 4,160 | Geom2d (pervasive), Adaptor2d, AdvApprox, Hermit, Convert; GC + GCPnts (ApproxArcsSegments only) | static class (ConcatC1/G1 not bound) | 60 |
| GeomConvert | 9,390 | Geom (pervasive), Geom2d, Adaptor3d, GeomAdaptor, AdvApprox, GeomLProp, Hermit, AdvApp2Var (ApproxSurface), Geom2dConvert, gce (CurveToAnaCurve); **IntAna, Extrema and GeomLib_IsPlanarSurface only in SurfToAnaSurf** | static class, ApproxCurve, CompCurveToBSplineCurve | 460 |
| GeomLProp, LProp | 2,214 + 843 | cycle: GeomLProp uses LProp's status enums, exceptions and Curve/SurfaceUtils; LProp_CLProps3d/SLProps3d are `using` aliases of GeomLProp templates | indirect: BRepLProp_SLProps | 580 |
| GeomLib | 7,032 | Geom, Geom2d, adaptors, AdvApprox, Hermit, Extrema (ExtPC, ExtPS, LocateExtPC), GeomConvert, Geom2dConvert, OSD_ThreadPool | IsPlanarSurface; indirect: BRepLib | 200 |
| GeomProjLib | 491 | Geom, Geom2d, GeomAdaptor, ProjLib, Approx_CurveOnSurface | indirect: GeomAPI (To2d) | — |
| GeomTools | 3,846 | every concrete Geom/Geom2d class, Message | none | — |
| Hermit | 1,280 | Geom_BSplineCurve, Geom2d_BSplineCurve | none | 250 |
| IntAna | 7,605 | IntAna2d; **ProjLib::Project(gp_Pln, gp_Lin) and gce_MakePln only in QuadQuadGeo** | none (add IntAna_QuadQuadGeo, IntAna_IntConicQuad to the whitelist) | 270 |
| IntAna2d | 1,971 | (a) only | AnaIntersection, IntPoint | — |
| ProjLib | 12,846 | Adaptor3d, Geom, Geom2d (pervasive), Extrema, AppParCurves, GeomAdaptor, GeomLib, Approx, AppCont, LProp, gce, GCPnts, GeomConvert, Geom2dConvert. ProjLib.cxx, ProjLib_Plane/Cylinder/Cone/Sphere/Torus and ProjLib_Projector are elementary | indirect: GeomAPI To2d/To3d, BRepProj_Projection | 140 |

**No GTests:** AppDef, Approx, AppParCurves, CPnts, FEmTool, GeomProjLib, GeomTools and IntAna2d. Draw-lite
scripts will cover them once the geometry commands exist.

### Generic code

The C++ templates and `.gxx` files below become C# generics, under the rules of section 9 and decisions 41 and 75:
a generic method or class constrained on a static-abstract tool interface, with a struct tool per instantiation.

- **AppParCurves .gxx** (Function, LeastSquare, ResolConstraint, Gradient, BSpFunction, BSpGradient). They are
  instantiated only by AppDef's 24 `*_0.cxx` files, over three or four "line tools".
- **Approx .gxx.** ComputeLine and BSplComputeLine are instantiated once each, by AppDef. ComputeCLine is
  instantiated twice, by Approx_FitAndDivide(2d) over AppCont_Function.
- **Extrema's templates** (GGExtPC, GGenExtPC, GFuncExtPC, GLocateExtPC, GenLocateExtPC, GGenExtCC, GFuncExtCC,
  GenLocateExtCC). Each has a 3d instance over Adaptor3d_Curve and a 2d instance over Adaptor2d_Curve2d.

### The cycles and how they are broken

1. **{Geom, GeomEval, GeomAdaptor, Adaptor3d}.**
   - The Rep* files are in the abstract layer.
   - Geom_OffsetSurface (its UIso/VIso need GeomAdaptor_Surface and AdvApprox) is ported in the same unit as
     GeomAdaptor.
   - Adaptor3d_HSurfaceTool (it constructs a GeomAdaptor_Curve) also goes in that unit.
2. **{Geom2d, Geom2dEval}.** Rep* is in the abstract layer.
3. **{GeomLProp, LProp}.** Both go in one unit.
4. **{BndLib, GeomBndLib}.** Both go in one unit.
5. **{gce, GC, Extrema, IntAna, ProjLib, Approx, GeomLib, GeomConvert, Geom2dConvert}.** Every edge is
   class-level:
   - gce, GC and IntAna use only the elementary Extrema and ProjLib classes. These go to early units (1G and 2F).
   - GeomConvert uses GeomLib only in SurfToAnaSurf, so SurfToAnaSurf moves to the GeomLib unit.
   - Approx uses GeomLib, GeomConvert and Extrema only in SameParameter, CurveOnSurface and CurvlinFunc. These are
     ported with GeomLib.

## 3. Work units and waves

Each unit is one branch and one porter. A unit lists its packages (or classes) and their size in thousands of
lines, what it depends on, and its tests:

- **Oracle:** whether build123d.NET binds its classes.
- **GT:** OCCT GTests to port.

"(a)" means Foundation plus the abstract layer.

### Wave 1: needs only (a)

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **1A** Geom curves | Line, Circle, Ellipse, Hyperbola, Parabola, TrimmedCurve, BezierCurve, BSplineCurve (+_1), OffsetCurve + OffsetCurveUtils.pxx | 8.6 | (a) | Oracle: full. GT: BSplineCurve, BezierCurve, Circle, Line, OffsetCurve, CurveEval (its GeomAPI_PointsToBSpline cases wait) |
| **1B** Geom2d curves | the same Geom2d curves + Geom2d_OffsetCurveUtils.pxx; Adaptor2d_OffsetCurve | 9.4 | (a) | Oracle: Line, Circle, Ellipse, Trimmed, BSpline (whitelist Bezier, Hyperbola, Parabola, OffsetCurve). GT: 3,800 lines of Geom2d + Adaptor2d_OffsetCurve |
| **1C** Geom surfaces | Plane, Cylindrical/Conical/Spherical/ToroidalSurface, RectangularTrimmedSurface, SurfaceOfRevolution + RevolutionUtils, SurfaceOfLinearExtrusion + ExtrusionUtils, BezierSurface, BSplineSurface (+_1) | 13.9 | (a), **[BSplSLib]** | Oracle: full. GT: BSplineSurface, BezierSurface, Plane, SurfaceEval |
| **1D** Eval geometries | GeomEval (TBezier, AHTBezier curve/surface, Ellipsoid, Hyperboloid, HypParaboloid, Paraboloid, CircularHelicoid, CircularHelix, SineWave) and Geom2dEval (TBezier, AHTBezier, Archimedean/Logarithmic spiral, CircleInvolute, SineWave) | 12.5 | (a) | Oracle: none. GT: 5,100 lines |
| **1E** Approximation leaves | AdvApprox, AppParCurves (the .gxx as generics), FEmTool, GProp | 14.5 | (a), **[Convert]** for AdvApprox | Oracle: GProp_GProps, PrincipalProps. GT: GProp |
| **1F** AdvApp2Var numeric core | AdvApp2Var_Data* tables (generated, as decision 77), SysBase, MathBase, and the other f2c files without Geom | 14.1 + tables | (a) | Oracle: none. GT: later, with 2D. Check against the C on random inputs if needed |
| **1G** Elementary analytics | IntAna2d; gce; the elementary Extrema: ExtElC, ExtElC2d, ExtPElC, ExtPElC2d, ExtPElS, ExtElSS, POnCurv(2d), POnSurf | 11.0 | (a) | Oracle: gce_MakeLin, IntAna2d_AnaIntersection, Extrema_POnCurv. GT: gce |

### Wave 2

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **2A** Offset surfaces and GeomAdaptor | Geom_OffsetSurface + OffsetSurfaceUtils.pxx + OsculatingSurface; GeomAdaptor (all of it); Adaptor3d_HSurfaceTool | 11.9 | 1A, 1C, 1E | Oracle: Geom_OffsetSurface, GeomAdaptor_Curve/Surface. GT: OffsetSurface, GeomAdaptor |
| **2B** Adaptors on surfaces | Adaptor3d_CurveOnSurface, IsoCurve, TopolTool; Geom2dAdaptor; Hermit | 8.3 | 1A, 1B, 1C | Oracle: Geom2dAdaptor_Curve. GT: Geom2dAdaptor, Hermit |
| **2C** Discrete approximation | AppCont (tables generated); AppDef with the Approx .gxx it instantiates (ComputeLine, BSplComputeLine) and Approx_ParametrizationType | 14.7 + tables | 1E | Oracle: indirect (native GeomAPI_PointsToBSpline against AppDef_BSplineCompute on the same points). GT: AppCont |
| **2D** AdvApp2Var | ApproxF2var, Context, Framework, Iso, Node, Network, Patch, Strip, ApproxAFunc2Var | 15.0 | 1F, 1E, 1C | Oracle: none (whitelist GeomConvert_ApproxSurface). GT: AdvApp2Var |
| **2E** Constructions and sampling | GC, GCE2d; CPnts, GCPnts | 12.6 | 1A, 1B, 1C, 1G | Oracle: GC_MakeArcOf*, GCPnts_AbscissaPoint, QuasiUniformDeflection, UniformDeflection. GT: GC, GCPnts |
| **2F** Quadric intersections | IntAna; Extrema_ExtElCS; the elementary ProjLib (ProjLib.cxx's `Project` overloads and IsAnaSurf, in a `partial` ProjLib class; Plane, Cylinder, Cone, Sphere, Torus, Projector). `MakePCurveOfType` needs ProjLib_ProjectedCurve and goes to 6A | 10.6 | 1G | Oracle: none (whitelist IntAna). GT: IntAna_IntQuadQuad, ProjLib_Cone |

### Wave 3

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **3A** Grid evaluation | GeomGridEval, Geom2dGridEval | 13.7 | 2A, 2B | Oracle: none. GT: 7,700 lines |
| **3B** Curve extrema | Extrema's curve templates (PC, CC, Locate, GlobOptFunc*) and their instances, CurveTool, Curve2dTool, ExtPC(2d), ExtCC(2d), ECC(2d), LocateExtPC/CC | 10.0 | 2E, 2B | Oracle: Extrema_ExtPC; native GeomAPI_ProjectPointOnCurve and Geom2dAPI_ProjectPointOnCurve against the port's ExtPC. GT: Extrema_ExtPC |
| **3C** Properties, persistence, hashes | LProp + GeomLProp; GeomTools; GeomHash; Geom2dHash | 10.6 | 2A, 2B | Oracle: none (BRepLProp indirectly later). GT: GeomLProp, LProp, the hashers |

### Wave 4

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **4A** Surface extrema | Extrema's surface classes (ExtPS, GenExtPS, GenExtCS, GenExtSS, ExtPExtS, ExtPRevS, ExtSS, FuncExt*, GenLocateExtPS, ...) except ExtCS; ExtremaPC | 11.3 | 3A, 3B, 2A | Oracle: native GeomAPI_ProjectPointOnSurf. GT: ExtremaPC, 8,800 lines |
| **4B** Bounding boxes | BndLib; GeomBndLib; Extrema_ExtCS | 12.6 | 3A, 3B | Oracle: Bnd_Box/OBB through native BRepBndLib on edges and faces. GT: BndLib, GeomBndLib |
| **4C** Conversion | Geom2dConvert; GeomConvert except SurfToAnaSurf | 12.3 | 3C, 2D, 2E, 2F, 2B | Oracle: GeomConvert and Geom2dConvert statics, ApproxCurve, CompCurveToBSplineCurve. GT: GeomConvert, Geom2dConvert |

### Wave 5

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **5A** GeomLib and Approx | GeomLib; Approx except the .gxx done in 2C (SameParameter, CurveOnSurface, CurvlinFunc, CurvilinearParameter, SweepApproximation, MCurvesToBSpCurve, FitAndDivide(2d)); GeomConvert_SurfToAnaSurf | 14.9 | 4A, 4C, 2C, 2E | Oracle: GeomLib_IsPlanarSurface; BRepLib::SameParameter indirectly. GT: GeomLib_CheckCurveOnSurface |

### Wave 6

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **6A** Projection | the rest of ProjLib (ProjectedCurve, CompProjectedCurve, ComputeApprox, ComputeApproxOnPolarSurface, ProjectOnPlane, ProjectOnSurface, ...); GeomProjLib | 11.3 | 5A | Oracle: native GeomAPI To2d and BRepProj against the port. GT: ProjLib_ComputeApproxOnPolarSurface |

### Totals

| Wave | Units | Lines (k) |
|---|---|---:|
| 1 | 7 | 84 |
| 2 | 6 | 73 |
| 3 | 3 | 34 |
| 4 | 3 | 36 |
| 5 | 1 | 15 |
| 6 | 1 | 11 |

That is 254,000 lines of code, plus 148,000 lines of tables to generate and the abstract layer's 10,700. Waves 5
and 6 are single units on the critical path. GeomLib and ProjLib can't be split further without porting halves of
classes.

## 4. Notes for porters

- **Mutators.** Mutators of Geom, Geom2d and Geom*_Transformation are `internal` and start with `CheckMutable()`.
  Overrides of the abstract mutators are `protected internal override` (decision 94):
  - Geom_Geometry: `Transform`
  - Geom_Curve: `Reverse`
  - Geom_Surface: `UReverse`, `VReverse`
  - Geom_AxisPlacement: `SetDirection`
  - Geom_Vector: `Cross`, `CrossCross`
- **Twins.** Every `SetFoo` gets a `WithFoo` returning a changed `Copy()` of the concrete type (decision 95).
  Every other mutator needs its OCCT `-ed` twin, or one you add.
- **Evaluation.** `EvalD0` to `EvalDN` are the virtuals. They return `Geom_Curve.ResD1/2/3` or
  `Geom_Surface.ResD1/2/3` (public-field structs). `D0` to `DN` are non-virtual wrappers with `out` parameters,
  except in Adaptor2d_Curve2d, where OCCT makes D0 to DN virtual and Eval* call them.
- **Evaluation representations.** `SetEvalRepresentation` is a mutator (internal, with `CheckMutable`).
  Descriptors are mutable objects with public fields, as in OCCT. Clone them with `GeomEval_RepUtils.CloneCurveDesc`
  in `Copy`, and don't hand a stored descriptor to callers (decision 100).
- **Adaptors** are working objects: `Load` and the other setters are public (decision 101). `GetType()` hides
  `object.GetType`; derived adaptors write `public override GeomAbs_CurveType GetType()` (decision 98).
- **Placeholders.** Units 1A, 1B and 1C replace the placeholder files of their classes.
- **Freezing.** When a ported algorithm mutates geometry it didn't create, copy first and note the place in
  decisions.md (section 8 of the rules).
