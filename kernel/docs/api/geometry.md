# API review: geometry

*Unit 10D, draft. Namespace `Kerf.Geometry`. Values in Foundation, curves and surfaces in ModelingData; the fitting,
projection and intersection members are extension members declared in ModelingAlgorithms, in the same namespace.*

## Main types

**Values** (gp, `readonly struct`; README section 4): `Point3D`, `Vector3D`, `Direction3D`, `Axis3D`, `Frame3D`,
`CoordinateSystem`, `Transform3D`, `GeneralTransform3D`, `Matrix3x3`, `Line3D`, `Circle3D`, `Ellipse3D`,
`Hyperbola3D`, `Parabola3D`, `Plane3D`, `Cylinder`, `Cone`, `Sphere`, `Torus`, and the 2D twins; enums
`EulerSequence`, `TransformForm`, `Continuity`, `CurveKind`, `SurfaceKind`, `JoinKind`. New: `BoundingBox`,
`OrientedBox`, `Interval`, `Tolerances`, `Grid<T>`, `CurvePoint`, `SurfacePoint`.

```csharp
public readonly struct Point3D : IEquatable<Point3D>
{
    public Point3D(double x, double y, double z);
    public double X { get; }  public double Y { get; }  public double Z { get; }
    public static Point3D Origin { get; }
    public double DistanceTo(Point3D other);
    public double SquareDistanceTo(Point3D other);
    public bool IsEqual(Point3D other, double tolerance);
    public Point3D Transformed(in Transform3D transform);
    public Point3D Mirrored(in Axis3D axis);           // and Mirrored(Point3D), Mirrored(Frame3D)
    public static Vector3D operator -(Point3D a, Point3D b);
    public static Point3D operator +(Point3D point, Vector3D offset);
}

public readonly struct Transform3D
{
    public static Transform3D Identity { get; }
    public static Transform3D Translation(Vector3D offset);
    public static Transform3D Rotation(in Axis3D axis, double angle);
    public static Transform3D Rotation(EulerSequence sequence, double alpha, double beta, double gamma);
    public static Transform3D Scale(Point3D centre, double factor);
    public static Transform3D Mirror(in Frame3D plane);
    public static Transform3D Between(in CoordinateSystem from, in CoordinateSystem to);
    public TransformForm Form { get; }
    public double ScaleFactor { get; }
    public Vector3D TranslationPart { get; }
    public Matrix3x3 RotationPart { get; }
    public bool IsNegative { get; }
    public (double Alpha, double Beta, double Gamma) GetEulerAngles(EulerSequence sequence);
    public Transform3D Inverted();
    public static Transform3D operator *(in Transform3D left, in Transform3D right);
    public static Point3D operator *(in Transform3D transform, Point3D point);
}

public readonly struct BoundingBox                       // replaces Bnd_Box (a mutable class) in the public API
{
    public static BoundingBox Empty { get; }
    public bool IsEmpty { get; }
    public Point3D Min { get; }  public Point3D Max { get; }
    public Vector3D Size { get; }  public Point3D Centre { get; }
    public BoundingBox Union(in BoundingBox other);
    public BoundingBox Enlarged(double gap);
    public bool Contains(Point3D point);
    public bool Intersects(in BoundingBox other);
}
```

**Curves and surfaces** (Geom, Geom2d): sealed immutable classes, abstract where OCCT's are.

```csharp
public abstract class Curve3D : Geometry3D
{
    public Interval Range { get; }                       // FirstParameter..LastParameter
    public bool IsClosed { get; }  public bool IsPeriodic { get; }  public double Period { get; }
    public Continuity Continuity { get; }
    public CurveKind Kind { get; }
    public Point3D Evaluate(double u);
    public CurveDerivatives Derivatives(double u, int order);     // replaces D1/D2/D3/DN and ResD1…ResD3
    public Curve3D Reversed();
    public double ReversedParameter(double u);
    public Curve3D Transformed(in Transform3D transform);       // from Geometry3D
}

public sealed class BSplineCurve3D : BoundedCurve3D
{
    public BSplineCurve3D(ReadOnlySpan<Point3D> poles, ReadOnlySpan<double> knots, ReadOnlySpan<int> multiplicities,
                          int degree, bool periodic = false, ReadOnlySpan<double> weights = default);
    public int Degree { get; }
    public bool IsRational { get; }
    public ReadOnlySpan<Point3D> Poles { get; }                 // 0-based
    public ReadOnlySpan<double> Weights { get; }               // empty when not rational
    public ReadOnlySpan<double> Knots { get; }
    public ReadOnlySpan<int> Multiplicities { get; }
    public BSplineCurve3D WithPole(int index, Point3D pole);   // copies (immutability)
    public BSplineCurve3D Segment(double u1, double u2);
    public BSplineCurve3D WithDegree(int degree);
}

public abstract class Surface : Geometry3D
{
    public (Interval U, Interval V) Bounds { get; }
    public SurfaceKind Kind { get; }
    public Point3D Evaluate(double u, double v);
    public SurfaceDerivatives Derivatives(double u, double v, int order);
    public Curve3D UIso(double u);  public Curve3D VIso(double v);
    public bool TryGetPlane(double tolerance, out Plane3D plane);   // GeomLib_IsPlanarSurface
}

public sealed class PlaneSurface : ElementarySurface { public PlaneSurface(in Plane3D plane); public Plane3D Plane { get; } }
public sealed class TrimmedCurve3D : BoundedCurve3D { public Curve3D Basis { get; } … }
```

Static extension members (ModelingAlgorithms and ModelingData), each an expected-failure operation:

```csharp
// construction of curves (GC_*, gce_*, GeomAPI_*)
TrimmedCurve3D.ArcThrough(Point3D start, Point3D middle, Point3D end)            -> Result<TrimmedCurve3D>
TrimmedCurve3D.Arc(in Circle3D circle, double startAngle, double endAngle, bool counterClockwise = true)
TrimmedCurve3D.Arc(in Ellipse3D ellipse, …), Arc(in Hyperbola3D …), Arc(in Parabola3D …)
TrimmedCurve3D.Segment(Point3D start, Point3D end)                               -> Result<TrimmedCurve3D>
BSplineCurve3D.Interpolate(ReadOnlySpan<Point3D> points, InterpolationOptions? options = null)  -> Result<BSplineCurve3D>
BSplineCurve3D.Approximate(ReadOnlySpan<Point3D> points, ApproximationOptions? options = null)  -> Result<BSplineCurve3D>
BSplineCurve3D.Join(IEnumerable<BoundedCurve3D> curves, double tolerance)                    -> Result<BSplineCurve3D>
BSplineSurface.Approximate(Grid<Point3D> points, SurfaceApproximationOptions? options = null)  -> Result<BSplineSurface>
curve.ToBSpline(ConversionOptions? options = null)                              -> Result<BSplineCurve3D>
curve.ToCurve2D(in Plane3D plane) / curve2d.ToCurve3D(in Plane3D plane)          // GeomAPI.To2d/To3d

// projection and intersection (GeomAPI_*, Geom2dAPI_*, Extrema)
curve.Project(Point3D point)            -> ImmutableArray<CurvePoint>      // nearest first
curve.TryProjectNearest(Point3D point, out CurvePoint nearest)
surface.Project(Point3D point)          -> ImmutableArray<SurfacePoint>
curve.Intersect(Surface surface, double tolerance)  -> ImmutableArray<CurveSurfaceHit>
surface.Intersect(Surface other, double tolerance)  -> Result<ImmutableArray<Curve3D>>
curve2d.Intersect(Curve2D other, double tolerance)  -> ImmutableArray<CurveCurveHit2D>

// 2D constraints (GccAna, Geom2dGcc): circles and lines tangent to curves, through points, of a radius
Tangents2D.Lines(TangentTarget2D first, TangentTarget2D second, double tolerance)            -> ImmutableArray<Line2D>
Tangents2D.Circles(TangentTarget2D first, TangentTarget2D second, double radius, double tol) -> ImmutableArray<Circle2D>
Tangents2D.Circles(TangentTarget2D a, TangentTarget2D b, TangentTarget2D c, double tolerance)
// TangentTarget2D: a Curve2D or a Point2D, with a Tangency (Unqualified, Enclosing, Enclosed, Outside: GccEnt_Position)
```

Options: `InterpolationOptions` (`Periodic`, `Tangents`, `Parameters`, `Tolerance`), `ApproximationOptions`
(`MinDegree`, `MaxDegree`, `Continuity`, `Tolerance`, `Parametrisation`), `ConversionOptions` (`Tolerance`,
`MaxDegree`, `MaxSegments`).

## Examples

```csharp
var frame = new Frame3D(Point3D.Origin, Direction3D.Z, Direction3D.X);
var placed = Transform3D.Rotation(new Axis3D(Point3D.Origin, Direction3D.Z), Math.PI / 4) * new Point3D(10, 0, 0);
```

```csharp
var spline = BSplineCurve3D.Interpolate([new(0, 0, 0), new(10, 5, 0), new(20, 0, 0)],
                                        new InterpolationOptions { Tangents = [Vector3D.X, Vector3D.X] });
if (!spline.TryGetValue(out var curve)) return spline.Error!.Message;
Point3D mid = curve.Evaluate(curve.Range.Middle);
```

```csharp
var arc = TrimmedCurve3D.ArcThrough(new(0, 0, 0), new(5, 5, 0), new(10, 0, 0)).Value;
double radius = ((CircleCurve3D)arc.Basis).Radius;
```

```csharp
if (surface.TryGetPlane(Tolerances.Confusion, out Plane3D plane))
    Console.WriteLine(plane.Normal);
```

```csharp
var nearest = curve.Project(point);            // ImmutableArray<CurvePoint>, nearest first
double gap = nearest.IsEmpty ? double.NaN : nearest[0].Distance;
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `gp_Pnt`, `gp_Vec`, `gp_Dir`, `gp_Ax1`/`2`/`3`, `gp_Trsf`… | The values above (README section 4) |
| `gp_Pnt.X()`, `SetX(…)`, `Coord()` | `X` property; `WithX(…)`; no `gp_XYZ` in public |
| `gp::Origin()`, `gp::DZ()` | `Point3D.Origin`, `Direction3D.Z` |
| `gp_Quaternion` | `Transform3D.Rotation(…)`, `GetEulerAngles(…)` |
| `Geom_Curve.D0/D1/D2/D3/DN` and `ResD1…ResD3` | `Evaluate`, `Derivatives(u, order)` → `CurveDerivatives` |
| `Geom_BSplineCurve.Pole(i)`, `NbPoles()`, `Poles(TColgp_Array1OfPnt)` | `Poles[i - 1]`, `Poles.Length`, `Poles` span |
| `Geom_BSplineCurve.SetPole`, `Segment`, `IncreaseDegree` (internal mutators) | `WithPole`, `Segment`, `WithDegree` (copies) |
| `GeomAdaptor_Curve`, `GeomAdaptor_Surface` | The curve's and surface's own members |
| `GC_MakeArcOfCircle`, `…OfEllipse/Hyperbola/Parabola`, `GC_MakeSegment` | `TrimmedCurve3D.ArcThrough`, `Arc`, `Segment` |
| `gce_Make…` | The values' constructors and factories |
| `GeomAPI_Interpolate`, `GeomAPI_PointsToBSpline(Surface)` | `BSplineCurve3D.Interpolate`, `Approximate`; `BSplineSurface.Approximate` |
| `GeomConvert`, `GeomConvert_CompCurveToBSplineCurve`, `…ApproxCurve` | `ToBSpline`, `BSplineCurve3D.Join` |
| `GeomAPI_ProjectPointOnCurve/Surf`, `Extrema_ExtPC`, `Geom2dAPI_ProjectPointOnCurve` | `Project`, `TryProjectNearest` |
| `GeomAPI_IntCS`, `GeomAPI_IntSS`, `Geom2dAPI_InterCurveCurve`, `IntAna2d_AnaIntersection` | `Intersect` |
| `Geom2dGcc_*`, `GccAna_*`, `Geom2dGcc_QualifiedCurve`, `GccEnt_Position` | `Tangents2D`, `TangentTarget2D`, `Tangency` |
| `GeomLib_IsPlanarSurface` | `Surface.TryGetPlane` |
| `GeomAPI.To2d/To3d` | `ToCurve2D`, `ToCurve3D` |
| `Bnd_Box`, `Bnd_OBB` | `BoundingBox`, `OrientedBox` (values; built by Measurement) |
| `Precision.Confusion()` | `Tolerances.Confusion` |
| `TColgp_Array1OfPnt`, `TColStd_Array1OfReal`, `TColgp_Array2OfPnt` | `ReadOnlySpan<Point3D>`, `ReadOnlySpan<double>`, `Grid<Point3D>` |

**Disappears from the public API:** `Geom_CartesianPoint`, `Geom_Direction`, `Geom_Vector`, the `…Placement` classes and
`Geom_Transformation` (values do their work); the `Adaptor` hierarchies; `GeomAbs_IsoType`, `GeomAbs_BSplKnotDistribution`;
`gp_VectorWithNullMagnitude` and Geom's `Undefined…` exceptions (an `ArgumentException`, or a result).

## Open points

- `Curve3D`/`Curve2D` keep separate hierarchies (as Geom and Geom2d do); no shared generic base. Agreed by default.
- `Interval` (a `readonly record struct` of `Min`, `Max`) is new; OCCT uses pairs of doubles and `Bnd_Range`.
- Whether `CurveDerivatives` holds `ReadOnlySpan<Vector3D>` (any order, a `ref struct`) or three fixed fields (orders
  1 to 3, a plain struct). Proposed: fixed fields, with `DerivativeN(u, n)` for higher orders.
- Units: radians in the kernel throughout; the modelling layer converts degrees.
