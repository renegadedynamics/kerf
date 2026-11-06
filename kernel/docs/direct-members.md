# Public members no test calls directly

Written by `tools/direct-members.cs` from the IL of the Release test assemblies (unit 11J). `public-members.md`
counts a member as called if any line of it ran, through any other member; this counts it only if a test assembly's
own code names it. "Checked" means that the test naming it, or a test reaching the helper naming it, also reaches
an assertion: a filter against tests that only call, not a proof that this member's result is what was asserted.
An override counts when its base's declaration is named, since C# names the least derived one. The Draw-lite
harness isn't a test assembly, so what only its commands name doesn't count.

Measured: 4,463 members; called directly 3,470; of them checked 3,465; no direct test 993. From 11,476 test methods. Not measured: 32 abstract, 814 compiler-written, 23 overrides of `object`'s members on classes (the IL can't show them).

| Assembly | Members | Direct | Checked | No direct test |
|---|---:|---:|---:|---:|
| Kerf.Kernel.Foundation | 1,274 | 888 | 888 | 386 |
| Kerf.Kernel.ModelingData | 1,012 | 926 | 926 | 86 |
| Kerf.Kernel.ModelingAlgorithms | 802 | 471 | 466 | 331 |
| Kerf.Kernel.DataExchange | 117 | 58 | 58 | 59 |
| Kerf.Modeling | 1,258 | 1,127 | 1,127 | 131 |

| Namespace | Members | Direct | Checked | No direct test |
|---|---:|---:|---:|---:|
| `Kerf` | 70 | 69 | 69 | 1 |
| `Kerf.Booleans` | 31 | 20 | 20 | 11 |
| `Kerf.Construction` | 172 | 91 | 91 | 81 |
| `Kerf.Drawing` | 149 | 92 | 87 | 57 |
| `Kerf.Exchange` | 117 | 58 | 58 | 59 |
| `Kerf.Features` | 23 | 15 | 15 | 8 |
| `Kerf.Fillets` | 46 | 23 | 23 | 23 |
| `Kerf.Geometry` | 1,960 | 1,529 | 1,529 | 431 |
| `Kerf.Healing` | 115 | 66 | 66 | 49 |
| `Kerf.Measurement` | 70 | 42 | 42 | 28 |
| `Kerf.Meshing` | 54 | 41 | 41 | 13 |
| `Kerf.Modeling` | 1,258 | 1,127 | 1,127 | 131 |
| `Kerf.Offsets` | 27 | 11 | 11 | 16 |
| `Kerf.Topology` | 371 | 286 | 286 | 85 |

## `Kerf.Booleans.BooleanOperations`

- `Common(IReadOnlyList<Shape>, IReadOnlyList<Shape>, BooleanOptions, IProgress<Double>, CancellationToken)`
- `Cut(IReadOnlyList<Shape>, IReadOnlyList<Shape>, BooleanOptions, IProgress<Double>, CancellationToken)`
- `Fuse(IReadOnlyList<Shape>, IReadOnlyList<Shape>, BooleanOptions, IProgress<Double>, CancellationToken)`

## `Kerf.Booleans.BooleanOptions`

- `get_ApproximateSection()`
- `get_FuzzyValue()`
- `get_Glue()`
- `get_SectionCurvesOnFaces()`
- `get_Simplify()`
- `set_ApproximateSection(Boolean)`
- `set_FuzzyValue(Double)`
- `set_Glue(GlueMode)`

## `Kerf.Color`

- `Equals(Color)`

## `Kerf.Construction.ExtrudeOptions`

- `get_Canonize()`
- `get_Default()`

## `Kerf.Construction.FillBoundary`

- `get_Continuity()`
- `get_Edge()`
- `get_IsBoundary()`
- `get_Support()`
- `set_Support(Face?)`

## `Kerf.Construction.FillOptions`

- `get_AngularTolerance()`
- `get_Anisotropy()`
- `get_CurvatureTolerance()`
- `get_Default()`
- `get_Degree()`
- `get_InitialSurface()`
- `get_Iterations()`
- `get_MaxDegree()`
- `get_MaxSegments()`
- `get_Points()`
- `get_PointsOnCurve()`
- `get_Tolerance2D()`
- `get_Tolerance3D()`

## `Kerf.Construction.HelixOptions`

- `get_Continuity()`
- `get_Default()`
- `get_EndRadius()`
- `get_MaxDegree()`
- `get_Tolerance()`

## `Kerf.Construction.LoftOptions`

- `get_CheckCompatibility()`
- `get_Continuity()`
- `get_Default()`
- `get_MakeSolid()`
- `get_MaxDegree()`
- `get_Parametrization()`
- `get_Ruled()`
- `get_Smoothing()`
- `get_Tolerance()`
- `set_MakeSolid(Boolean)`

## `Kerf.Construction.Primitives`

- `Cone(Double, Double, Double, Frame3D, Double)`
- `Sphere(Double, Frame3D, SphereOptions)`
- `Torus(Double, Double, Frame3D, TorusOptions)`
- `Wedge(Double, Double, Double, WedgeOptions, Frame3D)`
- `get_Failed()`

## `Kerf.Construction.ShapeProjections`

- `Project(Wire, Shape, Point3D)`

## `Kerf.Construction.ShapeTransforms`

- `get_TransformFailed()`

## `Kerf.Construction.SphereOptions`

- `.ctor()`
- `get_Angle()`
- `get_Default()`
- `get_MaxLatitude()`
- `get_MinLatitude()`
- `set_Angle(Double)`
- `set_MaxLatitude(Double)`
- `set_MinLatitude(Double)`

## `Kerf.Construction.Surfaces`

- `get_FillFailed()`

## `Kerf.Construction.SweepOptions`

- `get_AuxiliarySpine()`
- `get_Binormal()`
- `get_Default()`
- `get_FixedFrame()`
- `get_Frame()`
- `get_MakeSolid()`
- `get_Tolerance()`
- `get_Transition()`
- `get_WithContact()`
- `get_WithCorrection()`
- `set_AuxiliarySpine(Wire?)`
- `set_CurvilinearEquivalence(Boolean)`
- `set_FixedFrame(Frame3D?)`
- `set_Transition(TransitionMode)`
- `set_WithCorrection(Boolean)`

## `Kerf.Construction.Sweeps`

- `Sweep(IReadOnlyList<Shape>, Wire, SweepOptions, IProgress<Double>, CancellationToken)`
- `get_SweepFailed()`

## `Kerf.Construction.TorusOptions`

- `.ctor()`
- `get_Angle()`
- `get_Default()`
- `get_MinorEnd()`
- `get_MinorStart()`
- `set_Angle(Double)`
- `set_MinorEnd(Double)`
- `set_MinorStart(Double)`

## `Kerf.Construction.WedgeOptions`

- `get_TopLength()`
- `get_TopMax()`
- `get_TopMin()`
- `set_TopMax(ValueTuple<Double, Double>?)`
- `set_TopMin(ValueTuple<Double, Double>?)`

## `Kerf.Drawing.Camera`

- `.ctor()`
- `get_Distance()`
- `get_Eye()`
- `get_Projection()`
- `get_Target()`
- `get_ViewFrame()`
- `set_Target(Point3D)`

## `Kerf.Drawing.DirectionalLight`

- `get_Direction()`
- `get_Intensity()`

## `Kerf.Drawing.DrawingOptions`

- `get_Default()`
- `get_IncludeHidden()`
- `get_IsoLineCount()`
- `get_Mesh()`
- `get_Method()`
- `get_Tolerance()`

## `Kerf.Drawing.DrawnEdge`

- `get_IsVisible()`

## `Kerf.Drawing.LineStyle`

- `get_Color()`
- `get_Dashes()`
- `get_Hidden()`
- `get_Visible()`
- `get_Width()`
- `set_Color(Color)` (called, no assertion reached)
- `set_Dashes(ImmutableArray<Double>)`

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

## `Kerf.Drawing.SvgOptions`

- `get_Background()`
- `get_Decimals()`
- `get_Default()`
- `get_HiddenStyle()`
- `get_IncludeHidden()`
- `get_Margin()`
- `get_PageHeight()`
- `get_PageWidth()`
- `get_Scale()`
- `get_Tolerance()`
- `get_Unit()`
- `get_VisibleStyle()`
- `set_Background(Color?)` (called, no assertion reached)
- `set_Scale(Double?)` (called, no assertion reached)
- `set_Tolerance(Double)` (called, no assertion reached)
- `set_VisibleStyle(LineStyle)` (called, no assertion reached)

## `Kerf.Exchange.BrepWriteOptions`

- `get_Binary()`
- `get_Default()`
- `get_WithNormals()`
- `get_WithTriangles()`

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

- `.ctor()`
- `get_Placement()`
- `set_Children(ImmutableArray<ModelNode>)`
- `set_Color(Color?)`
- `set_Mesh(ShapeMesh)`
- `set_Name(String)`
- `set_Placement(Placement)`
- `set_Shape(Shape?)`

## `Kerf.Exchange.ObjWriteOptions`

- `get_Default()`
- `get_Mesh()`
- `get_Normals()`
- `get_Scale()`
- `get_YUp()`

## `Kerf.Exchange.ReadLimits`

- `get_MaxDepth()`
- `get_MaxEntities()`
- `get_MaxNodes()`
- `get_MaxTriangles()`
- `set_MaxDepth(Int32)`
- `set_MaxEntities(Int32)`
- `set_MaxNodes(Int32)`
- `set_MaxTriangles(Int64)`

## `Kerf.Exchange.Step`

- `Read(String, StepReadOptions, IProgress<Double>, CancellationToken)`

## `Kerf.Exchange.StepReadOptions`

- `get_Default()`
- `get_ExternalReferences()`
- `get_Heal()`
- `get_Limits()`
- `get_ProductNames()`
- `set_ExternalReferences(Boolean)`
- `set_Limits(ReadLimits)`

## `Kerf.Exchange.StepWriteOptions`

- `get_Precision()`
- `get_Schema()`
- `get_Unit()`
- `get_WritePcurves()`
- `set_WritePcurves(Boolean)`

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

- `get_Face()`
- `get_Fuse()`
- `get_Height()`
- `get_Until()`
- `set_Face(Face?)`
- `set_Fuse(Boolean)`

## `Kerf.Features.FeatureOperations`

- `get_Failed()`

## `Kerf.Features.SplitFacesResult`

- `.ctor(Shape, ShapeHistory, ImmutableArray<Face>, ImmutableArray<Face>)`

## `Kerf.Fillets.ChamferOptions`

- `get_Default()`
- `get_Mode()`

## `Kerf.Fillets.ChamferSpec`

- `get_Angle()`
- `get_Distance()`
- `get_Distance2()`
- `get_Edge()`
- `get_ReferenceFace()`

## `Kerf.Fillets.FilletOperations`

- `get_Failed()`

## `Kerf.Fillets.FilletOptions`

- `get_AngularTolerance()`
- `get_Default()`
- `get_InternalContinuity()`
- `get_Shape()`
- `get_Tolerances()`

## `Kerf.Fillets.FilletSpec`

- `get_Edge()`
- `get_EndRadius()`
- `get_Radius()`

## `Kerf.Fillets.FilletTolerances`

- `get_Angular()`
- `get_Approximation2D()`
- `get_Approximation3D()`
- `get_Default()`
- `get_Deflection()`
- `get_Parametric()`
- `get_Spine()`

## `Kerf.Geometry.ApproximationOptions`

- `get_Continuity()`
- `get_Default()`
- `get_MaxDegree()`
- `get_MinDegree()`
- `get_Parametrization()`
- `get_Smoothing()`
- `get_Tolerance()`
- `set_Continuity(Continuity)`

## `Kerf.Geometry.Axis2D`

- `AngleTo(Axis2D)`
- `Equals(Axis2D)`
- `IsParallel(Axis2D, Double)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `Reversed()`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `Transformed(Transform2D)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `WithDirection(Direction2D)`
- `WithLocation(Point2D)`
- `op_Equality(Axis2D, Axis2D)`
- `op_Inequality(Axis2D, Axis2D)`

## `Kerf.Geometry.Axis3D`

- `AngleTo(Axis3D)`
- `Equals(Axis3D)`
- `IsNormal(Axis3D, Double)`
- `IsOpposite(Axis3D, Double)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `WithDirection(Direction3D)`
- `WithLocation(Point3D)`
- `op_Equality(Axis3D, Axis3D)`
- `op_Inequality(Axis3D, Axis3D)`

## `Kerf.Geometry.BSplineCurve2D`

- `get_MaxDegree()`

## `Kerf.Geometry.BoundingBox`

- `Equals(BoundingBox)`
- `op_Equality(BoundingBox, BoundingBox)`
- `op_Inequality(BoundingBox, BoundingBox)`

## `Kerf.Geometry.Circle2D`

- `Coefficients(Double, Double, Double, Double, Double, Double)`
- `Equals(Circle2D)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `Reversed()`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `WithLocation(Point2D)`
- `WithRadius(Double)`
- `get_Axis()`
- `op_Equality(Circle2D, Circle2D)`
- `op_Inequality(Circle2D, Circle2D)`

## `Kerf.Geometry.Circle3D`

- `Equals(Circle3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Scaled(Point3D, Double)`
- `SquareDistanceTo(Point3D)`
- `Translated(Point3D, Point3D)`
- `WithPosition(Frame3D)`
- `WithRadius(Double)`
- `get_YAxis()`
- `op_Equality(Circle3D, Circle3D)`
- `op_Inequality(Circle3D, Circle3D)`

## `Kerf.Geometry.Cone3D`

- `Equals(Cone3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `WithPosition(CoordinateSystem)`
- `WithRadius(Double)`
- `WithSemiAngle(Double)`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Cone3D, Cone3D)`
- `op_Inequality(Cone3D, Cone3D)`

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

## `Kerf.Geometry.CoordinateSystem`

- `Equals(CoordinateSystem)`
- `IsCoplanar(CoordinateSystem, Double, Double)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `get_Axis()`
- `op_Equality(CoordinateSystem, CoordinateSystem)`
- `op_Inequality(CoordinateSystem, CoordinateSystem)`

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

## `Kerf.Geometry.Cylinder3D`

- `Coefficients(Double, Double, Double, Double, Double, Double, Double, Double, Double, Double)`
- `Equals(Cylinder3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `WithAxis(Axis3D)`
- `WithPosition(CoordinateSystem)`
- `WithRadius(Double)`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Cylinder3D, Cylinder3D)`
- `op_Inequality(Cylinder3D, Cylinder3D)`

## `Kerf.Geometry.Direction2D`

- `Equals(Direction2D)`
- `Reversed()`
- `op_ExclusiveOr(Direction2D, Direction2D)`
- `op_Inequality(Direction2D, Direction2D)`
- `op_Multiply(Direction2D, Direction2D)`

## `Kerf.Geometry.Direction3D`

- `DotCross(Direction3D, Direction3D)`
- `Equals(Direction3D)`
- `Mirrored(Direction3D)`
- `op_Inequality(Direction3D, Direction3D)`

## `Kerf.Geometry.Ellipse2D`

- `Coefficients(Double, Double, Double, Double, Double, Double)`
- `Equals(Ellipse2D)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `Reversed()`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `Transformed(Transform2D)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `WithMajorRadius(Double)`
- `WithMinorRadius(Double)`
- `get_IsDirect()`
- `op_Equality(Ellipse2D, Ellipse2D)`
- `op_Inequality(Ellipse2D, Ellipse2D)`

## `Kerf.Geometry.Ellipse3D`

- `Equals(Ellipse3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `WithMajorRadius(Double)`
- `WithMinorRadius(Double)`
- `WithPosition(Frame3D)`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Ellipse3D, Ellipse3D)`
- `op_Inequality(Ellipse3D, Ellipse3D)`

## `Kerf.Geometry.Frame2D`

- `.ctor(Axis2D, Boolean)`
- `Equals(Frame2D)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `Transformed(Transform2D)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `WithAxis(Frame2D)`
- `WithLocation(Point2D)`
- `get_YAxis()`
- `op_Equality(Frame2D, Frame2D)`
- `op_Inequality(Frame2D, Frame2D)`

## `Kerf.Geometry.Frame3D`

- `Equals(Frame3D)`
- `IsCoplanar(Axis3D, Double, Double)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Scaled(Point3D, Double)`
- `Translated(Point3D, Point3D)`
- `WithAxis(Axis3D)`
- `op_Equality(Frame3D, Frame3D)`
- `op_Inequality(Frame3D, Frame3D)`

## `Kerf.Geometry.GeneralTransform2D`

- `Equals(GeneralTransform2D)`
- `ToTransform()`
- `WithTransform(Transform2D)`
- `op_Equality(GeneralTransform2D, GeneralTransform2D)`
- `op_Inequality(GeneralTransform2D, GeneralTransform2D)`

## `Kerf.Geometry.GeneralTransform3D`

- `Equals(GeneralTransform3D)`
- `Mat4(ReadOnlySpan<Double>)`
- `Multiplied(GeneralTransform3D)`
- `WithMat4(ReadOnlySpan<Double>)`
- `WithVectorialPart(Matrix3x3)`
- `get_IsNegative()`
- `get_Item(Int32, Int32)`
- `get_VectorialPart()`
- `op_Equality(GeneralTransform3D, GeneralTransform3D)`
- `op_Inequality(GeneralTransform3D, GeneralTransform3D)`

## `Kerf.Geometry.Geometry2D`

- `Translated(Point2D, Point2D)`

## `Kerf.Geometry.GeometryOperations`

- `Arc(Hyperbola3D, Double, Double, Boolean)`
- `Arc(Parabola3D, Double, Double, Boolean)`
- `get_ApproximationFailed()`
- `get_IntersectionFailed()`

## `Kerf.Geometry.Hyperbola2D`

- `Coefficients(Double, Double, Double, Double, Double, Double)`
- `Equals(Hyperbola2D)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `OtherBranch()`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `get_IsDirect()`
- `op_Equality(Hyperbola2D, Hyperbola2D)`
- `op_Inequality(Hyperbola2D, Hyperbola2D)`

## `Kerf.Geometry.Hyperbola3D`

- `Equals(Hyperbola3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Hyperbola3D, Hyperbola3D)`
- `op_Inequality(Hyperbola3D, Hyperbola3D)`

## `Kerf.Geometry.InterpolationOptions`

- `get_Default()`
- `get_Parameters()`
- `get_Periodic()`
- `get_ScaleTangents()`
- `get_Tangents()`
- `get_Tolerance()`
- `set_ScaleTangents(Boolean)`
- `set_Tolerance(Double)`

## `Kerf.Geometry.Line2D`

- `AngleTo(Line2D)`
- `Coefficients(Double, Double, Double)`
- `Contains(Point2D, Double)`
- `DistanceTo(Line2D)`
- `Equals(Line2D)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `Reversed()`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `SquareDistanceTo(Point2D)`
- `Transformed(Transform2D)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `WithDirection(Direction2D)`
- `WithLocation(Point2D)`
- `op_Equality(Line2D, Line2D)`
- `op_Inequality(Line2D, Line2D)`

## `Kerf.Geometry.Line3D`

- `Equals(Line3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Normal(Point3D)`
- `Reversed()`
- `Scaled(Point3D, Double)`
- `SquareDistanceTo(Line3D)`
- `SquareDistanceTo(Point3D)`
- `Translated(Point3D, Point3D)`
- `WithDirection(Direction3D)`
- `WithLocation(Point3D)`
- `WithPosition(Axis3D)`
- `op_Equality(Line3D, Line3D)`
- `op_Inequality(Line3D, Line3D)`

## `Kerf.Geometry.LineCurve2D`

- `get_Position()`

## `Kerf.Geometry.Matrix3x3`

- `Added(Matrix3x3)`
- `CopyTo(Span<Double>)`
- `Divided(Double)`
- `Equals(Matrix3x3)`
- `Multiplied(Double)`
- `Powered(Int32)`
- `PreMultiplied(Matrix3x3)`
- `Subtracted(Matrix3x3)`
- `WithDiagonal(Double, Double, Double)`
- `WithScale(Double)`
- `op_Addition(Matrix3x3, Matrix3x3)`
- `op_Inequality(Matrix3x3, Matrix3x3)`

## `Kerf.Geometry.OrientedBox`

- `Equals(OrientedBox)`
- `get_Empty()`
- `get_HalfSize()`
- `get_XAxis()`
- `get_YAxis()`
- `get_ZAxis()`
- `op_Equality(OrientedBox, OrientedBox)`
- `op_Inequality(OrientedBox, OrientedBox)`

## `Kerf.Geometry.Parabola2D`

- `.ctor(Axis2D, Point2D, Boolean)`
- `Equals(Parabola2D)`
- `Mirrored(Axis2D)`
- `Mirrored(Point2D)`
- `Rotated(Point2D, Double)`
- `Scaled(Point2D, Double)`
- `Translated(Point2D, Point2D)`
- `Translated(Vector2D)`
- `get_IsDirect()`
- `op_Equality(Parabola2D, Parabola2D)`
- `op_Inequality(Parabola2D, Parabola2D)`

## `Kerf.Geometry.Parabola3D`

- `Equals(Parabola3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `get_Parameter()`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Parabola3D, Parabola3D)`
- `op_Inequality(Parabola3D, Parabola3D)`

## `Kerf.Geometry.Plane3D`

- `Contains(Line3D, Double, Double)`
- `DistanceTo(Line3D)`
- `DistanceTo(Plane3D)`
- `Equals(Plane3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Point3D)`
- `SignedDistance(Line3D)`
- `SignedDistance(Plane3D)`
- `SignedDistance(Point3D)`
- `SquareDistanceTo(Line3D)`
- `SquareDistanceTo(Plane3D)`
- `SquareDistanceTo(Point3D)`
- `Translated(Point3D, Point3D)`
- `WithLocation(Point3D)`
- `WithPosition(CoordinateSystem)`
- `get_YAxis()`
- `op_Equality(Plane3D, Plane3D)`
- `op_Inequality(Plane3D, Plane3D)`

## `Kerf.Geometry.Point2D`

- `Equals(Point2D)`
- `op_Equality(Point2D, Point2D)`
- `op_Inequality(Point2D, Point2D)`

## `Kerf.Geometry.Sphere3D`

- `Coefficients(Double, Double, Double, Double, Double, Double, Double, Double, Double, Double)`
- `Equals(Sphere3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `WithPosition(CoordinateSystem)`
- `WithRadius(Double)`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Sphere3D, Sphere3D)`
- `op_Inequality(Sphere3D, Sphere3D)`

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

## `Kerf.Geometry.Torus3D`

- `Equals(Torus3D)`
- `Mirrored(Axis3D)`
- `Mirrored(Frame3D)`
- `Mirrored(Point3D)`
- `Rotated(Axis3D, Double)`
- `Scaled(Point3D, Double)`
- `Transformed(Transform3D)`
- `Translated(Point3D, Point3D)`
- `Translated(Vector3D)`
- `VReversed()`
- `get_XAxis()`
- `get_YAxis()`
- `op_Equality(Torus3D, Torus3D)`
- `op_Inequality(Torus3D, Torus3D)`

## `Kerf.Geometry.Transform2D`

- `Equals(Transform2D)`
- `WithTransformation(Axis2D)`
- `WithTransformation(Axis2D, Axis2D)`
- `WithValues(Double, Double, Double, Double, Double, Double)`
- `get_IsNegative()`
- `get_RotationAngle()`
- `op_Equality(Transform2D, Transform2D)`
- `op_Inequality(Transform2D, Transform2D)`
- `op_Multiply(Transform2D, Transform2D)`

## `Kerf.Geometry.Transform3D`

- `Equals(Transform3D)`
- `WithMirror(Axis3D)`
- `WithTransformation(CoordinateSystem, CoordinateSystem)`
- `WithTranslation(Point3D, Point3D)`
- `WithValues(Double, Double, Double, Double, Double, Double, Double, Double, Double, Double, Double, Double)`
- `op_Equality(Transform3D, Transform3D)`
- `op_Inequality(Transform3D, Transform3D)`

## `Kerf.Geometry.Vector2D`

- `Added(Vector2D)`
- `CrossMagnitude(Vector2D)`
- `CrossSquareMagnitude(Vector2D)`
- `Equals(Vector2D)`
- `Multiplied(Double)`
- `Rotated(Double)`
- `Transformed(Transform2D)`
- `WithX(Double)`
- `WithY(Double)`
- `op_Equality(Vector2D, Vector2D)`
- `op_Inequality(Vector2D, Vector2D)`
- `op_Multiply(Double, Vector2D)`
- `op_UnaryNegation(Vector2D)`

## `Kerf.Geometry.Vector3D`

- `Added(Vector3D)`
- `AngleWithRef(Vector3D, Vector3D)`
- `CrossSquareMagnitude(Vector3D)`
- `LinearForm(Double, Vector3D, Double, Vector3D, Double, Vector3D)`
- `LinearForm(Double, Vector3D, Double, Vector3D, Double, Vector3D, Vector3D)`
- `LinearForm(Double, Vector3D, Double, Vector3D, Vector3D)`
- `LinearForm(Double, Vector3D, Vector3D)`
- `LinearForm(Vector3D, Vector3D)`
- `WithX(Double)`
- `WithY(Double)`
- `WithZ(Double)`
- `op_Equality(Vector3D, Vector3D)`
- `op_Inequality(Vector3D, Vector3D)`

## `Kerf.Healing.BSplineApproximationOptions`

- `get_Continuity2D()`
- `get_Continuity3D()`
- `get_MaxDegree()`
- `get_MaxSegments()`
- `get_Polynomial()`
- `get_PreferDegree()`
- `get_Tolerance2D()`
- `get_Tolerance3D()`

## `Kerf.Healing.BSplineConversionOptions`

- `get_Planes()`

## `Kerf.Healing.FixOptions`

- `get_CheckResult()`
- `get_Default()`
- `get_FixFaces()`
- `get_FixSameParameter()`
- `get_FixShells()`
- `get_FixSolids()`
- `get_FixWires()`
- `get_MaxTolerance()`
- `get_MinTolerance()`
- `get_Precision()`

## `Kerf.Healing.HealingOperations`

- `get_ConversionFailed()`
- `get_FixFailed()`
- `get_SewingFailed()`
- `get_UnifyFailed()`

## `Kerf.Healing.SewOptions`

- `get_Default()`
- `get_MaxTolerance()`
- `get_MinTolerance()`
- `get_NonManifold()`
- `get_Tolerance()`

## `Kerf.Healing.UnifyOptions`

- `get_AllowInternalEdges()`
- `get_AngularTolerance()`
- `get_ConcatBSplines()`
- `get_Default()`
- `get_Edges()`
- `get_Faces()`
- `get_LinearTolerance()`
- `set_AllowInternalEdges(Boolean)`
- `set_ConcatBSplines(Boolean)`
- `set_Edges(Boolean)`
- `set_Faces(Boolean)`

## `Kerf.Healing.ValidationOptions`

- `get_Default()`
- `get_Exact()`
- `get_Geometry()`
- `get_Parallel()`

## `Kerf.Healing.ValidationReport`

- `.ctor()`
- `set_IsValid(Boolean)`
- `set_Problems(ImmutableArray<ShapeProblem>)`

## `Kerf.Healing.WireframeFixOptions`

- `get_Default()`
- `get_MaxTolerance()`
- `get_SmallEdgesFirst()`

## `Kerf.Measurement.DistanceOptions`

- `get_Default()`
- `get_Deflection()`
- `get_Parallel()`
- `set_Deflection(Double)`

## `Kerf.Measurement.DistanceResult`

- `.ctor()`
- `set_IsInnerSolution(Boolean)`
- `set_Solutions(ImmutableArray<DistanceSolution>)`
- `set_Value(Double)`

## `Kerf.Measurement.DistanceSolution`

- `.ctor(Point3D, Point3D, SupportKind, Shape, Point2D, SupportKind, Shape, Point2D)`
- `get_ParametersB()`
- `get_SupportB()`
- `get_SupportShapeB()`

## `Kerf.Measurement.LineHit`

- `.ctor(Point3D, Double, Face, Double, Double, Containment)`
- `get_Face()`

## `Kerf.Measurement.MassOptions`

- `get_Default()`
- `get_OnlyClosed()`
- `get_SkipShared()`
- `get_Tolerance()`
- `get_UseTriangulation()`

## `Kerf.Measurement.MassProperties`

- `.ctor()`
- `get_Inertia()`
- `set_CentreOfMass(Point3D)`
- `set_Dimension(Dimension)`
- `set_Inertia(Matrix3x3)`
- `set_Mass(Double)`
- `set_PrincipalAxes(ValueTuple<Direction3D, Direction3D, Direction3D>)`
- `set_PrincipalMoments(ValueTuple<Double, Double, Double>)`
- `set_StaticMoments(Vector3D)`

## `Kerf.Meshing.EdgePolyline`

- `get_Kind()`
- `get_Points()`

## `Kerf.Meshing.FaceMesh`

- `get_Normals()`
- `get_Positions()`

## `Kerf.Meshing.MeshOptions`

- `get_AngularDeflection()`
- `get_EdgeDeflection()`
- `get_InternalVertices()`
- `get_LinearDeflection()`
- `get_MinSize()`
- `get_Parallel()`
- `get_Relative()`
- `set_EdgeDeflection(Double?)`

## `Kerf.Meshing.ShapeMesh`

- `get_EdgePoints()`

## `Kerf.Modeling.Airfoil`

- `ParseNaca4(String)`

## `Kerf.Modeling.Alignment`

- `Offset(Point3D, Point3D, Point3D?)`
- `get_Count()`

## `Kerf.Modeling.Assembly`

- `.ctor(Shape, String, Color?)`

## `Kerf.Modeling.Axis`

- `Equals(Axis)`

## `Kerf.Modeling.BaseLineObject`

- `get_Wire()`

## `Kerf.Modeling.Bezier`

- `.ctor(Point3D[])`

## `Kerf.Modeling.Curve`

- `.ctor(IEnumerable<Shape>, String, Color?)`
- `.ctor(IEnumerable<Wire>, String, Color?)`
- `Common(IEnumerable<Shape>)`
- `Subtract(IEnumerable<Shape>)`

## `Kerf.Modeling.CurvePosition`

- `get_Point()`
- `get_Value()`

## `Kerf.Modeling.GltfExportOptions`

- `get_AngularDeflection()`
- `get_Binary()`
- `get_Default()`
- `get_LinearDeflection()`
- `get_Unit()`

## `Kerf.Modeling.GridLocations`

- `get_Min()`
- `get_Size()`

## `Kerf.Modeling.ImportLimits`

- `get_Default()`
- `get_MaxBytes()`
- `get_MaxDepth()`
- `get_MaxEntities()`
- `get_MaxNodes()`
- `get_MaxTriangles()`

## `Kerf.Modeling.Location`

- `Equals(Location)`
- `op_Inequality(Location, Location)`

## `Kerf.Modeling.ModelHistory`

- `Of(IEnumerable<T>)`

## `Kerf.Modeling.ModelShape`

- `op_Explicit(ModelShape)`
- `op_Explicit(ModelShape)`
- `op_Explicit(ModelShape)`
- `op_Explicit(ModelShape)`
- `op_Explicit(ModelShape)`
- `op_Explicit(ModelShape)`

## `Kerf.Modeling.ModelShapeOperations`

- `Moved(T, Plane)`
- `Scale(T, Vector3D, Point3D?)`

## `Kerf.Modeling.ModelingDiagnostics`

- `get_KernelInvalidArgument()`

## `Kerf.Modeling.Operations`

- `BoundingBox(IEnumerable<Shape>)`
- `Chamfer(Shape, IEnumerable<Vertex>, Double, Double?, Double?, Edge?)`
- `Fillet(Shape, IEnumerable<Vertex>, Double)`
- `ProjectPoints(IEnumerable<Point3D>, Plane)`
- `Revolve(IEnumerable<T>, Axis?, Double, Boolean)`
- `Scale(Curve, Double, Point3D?)`
- `Scale(Curve, Vector3D, Point3D?)`
- `Scale(IEnumerable<Shape>, Vector3D, Point3D?)`
- `Scale(IEnumerable<T>, Double, Point3D?)`
- `Scale(IEnumerable<T>, Vector3D, Point3D?)`
- `Split(IEnumerable<Shape>, IShape2D, Keep)`

## `Kerf.Modeling.Operations.ExtrudeOptions`

- `get_Both()`
- `get_Clean()`
- `get_Default()`
- `get_Direction()`
- `get_Taper()`

## `Kerf.Modeling.Operations.OffsetOptions`

- `get_Closed()`
- `get_Default()`
- `get_Kind()`
- `get_MinEdgeLength()`
- `get_Openings()`
- `get_Side()`

## `Kerf.Modeling.Operations.SweepOptions`

- `get_Binormal()`
- `get_Clean()`
- `get_Default()`
- `get_IsFrenet()`
- `get_Multisection()`
- `get_Normal()`
- `get_Transition()`

## `Kerf.Modeling.Operations.ThickenOptions`

- `get_Both()`
- `get_Clean()`
- `get_Default()`
- `get_NormalOverride()`

## `Kerf.Modeling.Part`

- `Common(IEnumerable<Shape>)`
- `op_Subtraction(Part, IEnumerable<Shape>)`

## `Kerf.Modeling.Plane`

- `Contains(Axis, Double)`
- `Contains(Point3D, Double)`
- `FromLocalCoords(T)`
- `get_Bottom()`
- `get_Left()`
- `get_Right()`
- `get_Top()`

## `Kerf.Modeling.PointComparer`

- `Equals(Point3D, Point3D)`
- `GetHashCode(Point3D)`

## `Kerf.Modeling.PointExtensions`

- `Describe(Direction3D, String)`
- `GetSignedAngle(Direction3D, Vector3D, Vector3D?)`
- `Round(Vector3D, Int32)`
- `Transform(Point3D, GeneralTransform3D)`

## `Kerf.Modeling.RegularPolygon`

- `get_Radius()`

## `Kerf.Modeling.Selector`

- `Equals(Selector)`
- `get_Of()`

## `Kerf.Modeling.Selector.GroupByAxis`

- `get_Axis()`
- `get_Index()`

## `Kerf.Modeling.Selector.GroupByKey`

- `get_Index()`
- `get_Key()`

## `Kerf.Modeling.Selector.OfGeomType`

- `get_GeomType()`

## `Kerf.Modeling.Selector.ParallelTo`

- `get_Axis()`

## `Kerf.Modeling.Shape1DExtensions`

- `CommonPlane(TSelf, IEnumerable<IShape1D>, Double)`

## `Kerf.Modeling.Shape2DExtensions`

- `FindIntersectionPoints(TSelf, Axis, Double)`

## `Kerf.Modeling.ShapeExtensions`

- `ClosestPoints(TSelf, Point3D)`
- `ConvexityOf(TSelf, Face)`
- `Cut(TSelf, IEnumerable<T>)`

## `Kerf.Modeling.ShapeGroups`

- `GetEnumerator()`

## `Kerf.Modeling.ShapeOperations`

- `Moved(T, Plane)`
- `WithLocation(T, Location)`

## `Kerf.Modeling.ShapeSelection`

- `OrderByDescending(IEnumerable<T>, IShape1D)`
- `OrderByDistanceTo(IEnumerable<T>, Shape)`

## `Kerf.Modeling.ShellExtensions`

- `MakeSheet(IEnumerable<Face>, Boolean)`

## `Kerf.Modeling.Sketch`

- `Common(IEnumerable<Shape>)`
- `Subtract(IEnumerable<Shape>)`

## `Kerf.Modeling.SolidExtensions`

- `SweepMulti(IEnumerable<Wire>, IShape1D, Boolean, Boolean, Object)`

## `Kerf.Modeling.StepExportOptions`

- `get_CleanDuplicates()`
- `get_Default()`
- `get_PrecisionMode()`
- `get_Timestamp()`
- `get_Unit()`
- `get_WritePcurves()`

## `Kerf.Modeling.StlExportOptions`

- `get_AngularDeflection()`
- `get_Ascii()`
- `get_Default()`
- `get_LinearDeflection()`

## `Kerf.Modeling.Tangent`

- `get_Edge()`
- `get_Point()`
- `get_Tangency()`

## `Kerf.Modeling.ThreeMfExportOptions`

- `get_AngularDeflection()`
- `get_Default()`
- `get_LinearDeflection()`
- `get_Unit()`

## `Kerf.Modeling.Triangle`

- `get_EdgeA()`
- `get_EdgeB()`
- `get_EdgeC()`

## `Kerf.Modeling.WireExtensions`

- `Combine(IEnumerable<IShape1D>, Double)`
- `OrderChamferEdges(Edge?, ValueTuple<Edge, Edge>)`

## `Kerf.Offsets.Offset2DOptions`

- `get_Default()`
- `get_Join()`
- `get_Open()`

## `Kerf.Offsets.OffsetOperations`

- `get_Failed()`

## `Kerf.Offsets.OffsetOptions`

- `get_Default()`
- `get_Intersection()`
- `get_Join()`
- `get_Mode()`
- `get_RemoveInternalEdges()`
- `get_SelfIntersection()`
- `get_Tolerance()`
- `set_Intersection(Boolean)`
- `set_Mode(OffsetMode)`
- `set_RemoveInternalEdges(Boolean)`
- `set_SelfIntersection(Boolean)`
- `set_Tolerance(Double)`

## `Kerf.Topology.CompSolid`

- `Complemented()`
- `Composed(Orientation)`
- `Equals(CompSolid)`
- `IsEqual(Shape)`
- `IsPartner(Shape)`
- `IsSame(Shape)`
- `Located(Placement, Boolean)`
- `Moved(Placement, Boolean)`
- `Oriented(Orientation)`
- `Reversed()`
- `get_Orientation()`
- `get_Placement()`
- `op_Inequality(CompSolid, CompSolid)`

## `Kerf.Topology.Compound`

- `Complemented()`
- `Composed(Orientation)`
- `Equals(Compound)`
- `IsEqual(Shape)`
- `IsPartner(Shape)`
- `IsSame(Shape)`
- `Located(Placement, Boolean)`
- `Oriented(Orientation)`
- `Reversed()`
- `get_Placement()`
- `op_Inequality(Compound, Compound)`

## `Kerf.Topology.Edge`

- `Complemented()`
- `Equals(Edge)`
- `Explore(ShapeType?)`
- `op_Equality(Edge, Edge)`
- `op_Inequality(Edge, Edge)`

## `Kerf.Topology.EdgeCurve`

- `TryGetBSpline(BSplineCurve3D)`
- `TryGetBezier(BezierCurve3D)`
- `TryGetEllipse(Ellipse3D)`
- `TryGetHyperbola(Hyperbola3D)`
- `TryGetParabola(Parabola3D)`
- `get_Period()`

## `Kerf.Topology.Face`

- `Complemented()`
- `Composed(Orientation)`
- `IsPartner(Shape)`
- `op_Inequality(Face, Face)`

## `Kerf.Topology.FaceSurface`

- `TryGetCone(Cone3D)`
- `TryGetSphere(Sphere3D)`
- `TryGetTorus(Torus3D)`
- `get_Face()`

## `Kerf.Topology.Placement`

- `Divided(Placement)`
- `Multiplied(Placement)`

## `Kerf.Topology.Shape`

- `Explore(ShapeType?)`

## `Kerf.Topology.ShapeHistoryEntry`

- `get_Initial()`
- `get_Results()`

## `Kerf.Topology.Shell`

- `Complemented()`
- `Composed(Orientation)`
- `Equals(Shell)`
- `IsEqual(Shape)`
- `IsPartner(Shape)`
- `Located(Placement, Boolean)`
- `Oriented(Orientation)`
- `get_Placement()`
- `op_Inequality(Shell, Shell)`

## `Kerf.Topology.Solid`

- `Complemented()`
- `Composed(Orientation)`
- `Equals(Solid)`
- `IsPartner(Shape)`
- `Located(Placement, Boolean)`
- `op_Inequality(Solid, Solid)`

## `Kerf.Topology.SurfaceCurvature`

- `.ctor(Double, Double, Direction3D)`
- `get_Max()`
- `get_Min()`
- `get_Normal()`

## `Kerf.Topology.Vertex`

- `Complemented()`
- `Composed(Orientation)`
- `Equals(Vertex)`
- `IsEqual(Shape)`
- `IsPartner(Shape)`
- `get_Placement()`
- `get_Type()`
- `op_Equality(Vertex, Vertex)`
- `op_Inequality(Vertex, Vertex)`

## `Kerf.Topology.Wire`

- `Complemented()`
- `Composed(Orientation)`
- `Equals(Wire)`
- `IsEqual(Shape)`
- `IsPartner(Shape)`
- `Located(Placement, Boolean)`
- `Oriented(Orientation)`
- `get_Placement()`
- `op_Inequality(Wire, Wire)`
