# Public members no test calls directly

Written by `tools/direct-members.cs` from the IL of the Release test assemblies (unit 11J). `public-members.md`
counts a member as called if any line of it ran, through any other member; this counts it only if a test assembly's
own code names it. "Checked" means that the test naming it, or a test reaching the helper naming it, also reaches
an assertion: a filter against tests that only call, not a proof that this member's result is what was asserted.
An override counts when its base's declaration is named, since C# names the least derived one. The Draw-lite
harness isn't a test assembly, so what only its commands name doesn't count.

Measured: 4,526 members; called directly 4,319; of them checked 4,317; no direct test 207. From 12,052 test methods. Not measured: 32 abstract, 813 compiler-written, 23 overrides of `object`'s members on classes (the IL can't show them).

| Assembly | Members | Direct | Checked | No direct test |
|---|---:|---:|---:|---:|
| Kerf.Kernel.Foundation | 1,281 | 1,243 | 1,242 | 38 |
| Kerf.Kernel.ModelingData | 1,012 | 1,002 | 1,002 | 10 |
| Kerf.Kernel.ModelingAlgorithms | 846 | 719 | 719 | 127 |
| Kerf.Kernel.DataExchange | 117 | 90 | 90 | 27 |
| Kerf.Modeling | 1,270 | 1,265 | 1,264 | 5 |

| Namespace | Members | Direct | Checked | No direct test |
|---|---:|---:|---:|---:|
| `Kerf` | 77 | 75 | 75 | 2 |
| `Kerf.Booleans` | 35 | 34 | 34 | 1 |
| `Kerf.Construction` | 185 | 177 | 177 | 8 |
| `Kerf.Drawing` | 149 | 124 | 124 | 25 |
| `Kerf.Exchange` | 117 | 90 | 90 | 27 |
| `Kerf.Features` | 25 | 16 | 16 | 9 |
| `Kerf.Fillets` | 52 | 49 | 49 | 3 |
| `Kerf.Geometry` | 1,960 | 1,876 | 1,875 | 84 |
| `Kerf.Healing` | 125 | 110 | 110 | 15 |
| `Kerf.Measurement` | 71 | 57 | 57 | 14 |
| `Kerf.Meshing` | 56 | 53 | 53 | 3 |
| `Kerf.Modeling` | 1,270 | 1,265 | 1,264 | 5 |
| `Kerf.Offsets` | 33 | 29 | 29 | 4 |
| `Kerf.Topology` | 371 | 364 | 364 | 7 |

## `Kerf.Booleans.BooleanOptions`

- `get_CheckResult()`

## `Kerf.Cancellation`

- `get_Current()`

## `Kerf.Color`

- `Equals(Color)`

## `Kerf.Construction.ExtrudeOptions`

- `get_CheckResult()`

## `Kerf.Construction.FillBoundary`

- `set_Support(Face?)`

## `Kerf.Construction.FillOptions`

- `get_CheckResult()`
- `set_CheckResult(Boolean)`

## `Kerf.Construction.LoftOptions`

- `get_CheckResult()`

## `Kerf.Construction.SweepOptions`

- `get_CheckResult()`
- `get_TryOtherFrames()`
- `set_Transition(TransitionMode)`

## `Kerf.Drawing.DirectionalLight`

- `get_Direction()`
- `get_Intensity()`

## `Kerf.Drawing.PngOptions`

- `get_Compression()`
- `get_Default()`

## `Kerf.Drawing.RenderOptions`

- `get_Ambient()`
- `get_Background()`
- `get_EdgeColor()`
- `get_EdgeWidth()`
- `get_Edges()`
- `get_FeatureAngle()`
- `get_Height()`
- `get_Margin()`
- `get_Parallel()`
- `get_Scale()`
- `get_Shading()`
- `get_Supersampling()`
- `get_Width()`

## `Kerf.Drawing.Renderer`

- `DefaultMeshOptions(Shape)`

## `Kerf.Drawing.Scene`

- `.ctor(IEnumerable<SceneItem>)`

## `Kerf.Drawing.SceneItem`

- `get_DefaultColor()`
- `get_FaceColors()`
- `get_Transform()`
- `set_Color(Color)`
- `set_FaceColors(ImmutableDictionary<Int32, Color>)`
- `set_Mesh(ShapeMesh)`

## `Kerf.Exchange.GltfReadOptions`

- `.ctor()`
- `get_Default()`
- `get_Limits()`
- `get_Unit()`
- `set_Limits(ReadLimits)`
- `set_Unit(LengthUnit)`

## `Kerf.Exchange.GltfWriteOptions`

- `get_Binary()`
- `get_Default()`
- `get_Generator()`
- `get_Mesh()`
- `get_Unit()`
- `set_Generator(String)`
- `set_Unit(LengthUnit)`

## `Kerf.Exchange.ModelNode`

- `set_Mesh(ShapeMesh)`

## `Kerf.Exchange.ObjWriteOptions`

- `get_Default()`
- `get_Mesh()`
- `get_Normals()`
- `get_Scale()`
- `get_YUp()`

## `Kerf.Exchange.StlReadOptions`

- `get_Default()`
- `get_Limits()`
- `get_Unit()`
- `set_Limits(ReadLimits)`

## `Kerf.Exchange.StlWriteOptions`

- `get_Ascii()`
- `get_Default()`

## `Kerf.Exchange.ThreeMfWriteOptions`

- `get_Mesh()`
- `get_Unit()`

## `Kerf.Features.ExtrudeUntilOptions`

- `get_CheckResult()`
- `get_Face()`
- `get_Fuse()`
- `get_Height()`
- `get_Until()`
- `set_CheckResult(Boolean)`
- `set_Face(Face?)`
- `set_Fuse(Boolean)`

## `Kerf.Features.SplitFacesResult`

- `.ctor(Shape, ShapeHistory, ImmutableArray<Face>, ImmutableArray<Face>)`

## `Kerf.Fillets.ChamferOptions`

- `get_CheckResult()`
- `set_CheckResult(Boolean)`

## `Kerf.Fillets.FilletOptions`

- `get_CheckResult()`

## `Kerf.Geometry.ApproximationOptions`

- `get_Continuity()`
- `get_Default()`
- `get_MaxDegree()`
- `get_MinDegree()`
- `get_Parametrization()`
- `get_Smoothing()`
- `get_Tolerance()`
- `set_Continuity(Continuity)`

## `Kerf.Geometry.BSplineCurve2D`

- `get_MaxDegree()`

## `Kerf.Geometry.ConversionOptions`

- `get_Continuity()`
- `get_Default()`
- `get_ForceApproximation()`
- `get_MaxDegree()`
- `get_MaxSegments()`
- `get_Tolerance()`
- `set_Continuity(Continuity)`
- `set_MaxDegree(Int32)`
- `set_MaxSegments(Int32)`
- `set_Tolerance(Double)`

## `Kerf.Geometry.CurveCurveHit2D`

- `.ctor(Point2D, Double, Double)`
- `get_Point()`

## `Kerf.Geometry.CurveDerivatives2D`

- `.ctor(Int32, Point2D, Vector2D, Vector2D, Vector2D)`

## `Kerf.Geometry.CurveFrame`

- `.ctor(Point3D, Vector3D, Vector3D, Vector3D)`
- `get_Point()`

## `Kerf.Geometry.CurvePoint`

- `get_Point()`

## `Kerf.Geometry.CurvePoint2D`

- `.ctor(Double, Point2D, Double)`
- `get_Parameter()`

## `Kerf.Geometry.CurveSurfaceHit`

- `.ctor(Point3D, Double, Double, Double)`

## `Kerf.Geometry.Direction3D`

- `Equals(Direction3D)`

## `Kerf.Geometry.Geometry2D`

- `Translated(Point2D, Point2D)`

## `Kerf.Geometry.GeometryOperations`

- `Arc(Hyperbola3D, Double, Double, Boolean)`
- `Arc(Parabola3D, Double, Double, Boolean)`
- `get_ApproximationFailed()`
- `get_IntersectionFailed()`

## `Kerf.Geometry.InterpolationOptions`

- `get_Default()`
- `get_Parameters()`
- `get_Periodic()`
- `get_ScaleTangents()`
- `get_Tangents()`
- `get_Tolerance()`
- `set_ScaleTangents(Boolean)`
- `set_Tolerance(Double)`

## `Kerf.Geometry.Line3D`

- `Equals(Line3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Reversed()`
- `Scaled(Point3D, Double)`
- `Translated(Point3D, Point3D)`
- `WithDirection(Direction3D)` (called, no assertion reached)
- `WithLocation(Point3D)`
- `WithPosition(Axis3D)`
- `op_Equality(Line3D, Line3D)`
- `op_Inequality(Line3D, Line3D)`

## `Kerf.Geometry.LineCurve2D`

- `get_Position()`

## `Kerf.Geometry.OrientedBox`

- `Equals(OrientedBox)`
- `get_Empty()`
- `get_HalfSize()`
- `get_XAxis()`
- `get_YAxis()`
- `get_ZAxis()`
- `op_Equality(OrientedBox, OrientedBox)`
- `op_Inequality(OrientedBox, OrientedBox)`

## `Kerf.Geometry.Plane3D`

- `Equals(Plane3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Point3D)`
- `Translated(Point3D, Point3D)`
- `WithLocation(Point3D)`
- `WithPosition(CoordinateSystem)`
- `op_Equality(Plane3D, Plane3D)`
- `op_Inequality(Plane3D, Plane3D)`

## `Kerf.Geometry.SurfaceDerivatives`

- `get_Point()`

## `Kerf.Geometry.SurfacePoint`

- `get_U()`
- `get_V()`

## `Kerf.Geometry.TangentCircle2D`

- `.ctor(Circle2D, ImmutableArray<TangentContact2D>)`

## `Kerf.Geometry.TangentContact2D`

- `.ctor(Tangency, Point2D, Double, Double, Boolean)`
- `get_SolutionParameter()`

## `Kerf.Geometry.TangentLine2D`

- `.ctor(Line2D, ImmutableArray<TangentContact2D>)`

## `Kerf.Geometry.TangentTarget2D`

- `Through(Point2D)`
- `get_Curve()`
- `get_IsPoint()`
- `get_Point()`
- `get_Tangency()`

## `Kerf.Geometry.Tangents2D`

- `get_TangentsFailed()`

## `Kerf.Geometry.Vector3D`

- `op_Equality(Vector3D, Vector3D)`
- `op_Inequality(Vector3D, Vector3D)`

## `Kerf.Healing.BSplineApproximationOptions`

- `get_CheckResult()`

## `Kerf.Healing.BSplineConversionOptions`

- `get_CheckResult()`
- `set_CheckResult(Boolean)`

## `Kerf.Healing.SewOptions`

- `get_CheckResult()`
- `get_Tolerance()`
- `set_CheckResult(Boolean)`

## `Kerf.Healing.UnifyOptions`

- `get_CheckResult()`
- `set_AllowInternalEdges(Boolean)`
- `set_ConcatBSplines(Boolean)`

## `Kerf.Healing.ValidationReport`

- `.ctor()`
- `ToDiagnostic(String)`
- `set_IsValid(Boolean)`
- `set_Problems(ImmutableArray<ShapeProblem>)`

## `Kerf.Healing.WireframeFixOptions`

- `get_CheckResult()`
- `set_CheckResult(Boolean)`

## `Kerf.Measurement.DistanceResult`

- `.ctor()`
- `set_IsInnerSolution(Boolean)`
- `set_Solutions(ImmutableArray<DistanceSolution>)`
- `set_Value(Double)`

## `Kerf.Measurement.DistanceSolution`

- `.ctor(Point3D, Point3D, SupportKind, Shape, Point2D, SupportKind, Shape, Point2D)`

## `Kerf.Measurement.LineHit`

- `.ctor(Point3D, Double, Face, Double, Double, Containment)`

## `Kerf.Measurement.MassProperties`

- `.ctor()`
- `set_CentreOfMass(Point3D)`
- `set_Dimension(Dimension)`
- `set_Inertia(Matrix3x3)`
- `set_Mass(Double)`
- `set_PrincipalAxes(ValueTuple<Direction3D, Direction3D, Direction3D>)`
- `set_PrincipalMoments(ValueTuple<Double, Double, Double>)`
- `set_StaticMoments(Vector3D)`

## `Kerf.Meshing.MeshOptions`

- `get_MaxTriangles()`
- `set_MaxTriangles(Int64)`

## `Kerf.Meshing.Mesher`

- `get_TooLarge()`

## `Kerf.Modeling.ColorExtensions`

- `get_White()` (called, no assertion reached)

## `Kerf.Modeling.EdgeExtensions`

- `FindIntersections(Edge, Axis, Double)`
- `FindIntersections(Edge, Double)`

## `Kerf.Modeling.ModelShape`

- `op_Explicit(ModelShape)`

## `Kerf.Modeling.Selector`

- `Equals(Selector)`
- `get_Of()`

## `Kerf.Offsets.Offset2DOptions`

- `get_CheckResult()`
- `set_CheckResult(Boolean)`

## `Kerf.Offsets.OffsetOptions`

- `get_CheckResult()`
- `set_CheckResult(Boolean)`

## `Kerf.Topology.CompSolid`

- `get_Placement()`

## `Kerf.Topology.Compound`

- `get_Placement()`

## `Kerf.Topology.ShapeHistoryEntry`

- `get_Initial()`
- `get_Results()`

## `Kerf.Topology.Shell`

- `get_Placement()`

## `Kerf.Topology.SurfaceCurvature`

- `.ctor(Double, Double, Direction3D)`

## `Kerf.Topology.Wire`

- `get_Placement()`
