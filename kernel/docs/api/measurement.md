# API review: properties and measurement

*Unit 10D, draft. Namespace `Kerf.Measurement`, in ModelingAlgorithms. Engine: `Kerf.Engine.Measurement` (BRepGProp,
GProp, BRepBndLib, BRepExtrema, CPnts, GCPnts, LocalAnalysis), `Kerf.Engine.Classification` (BRepClass3d,
BRepClass, TopClass) and `Kerf.Engine.Intersection.CurveSurface`.*

## Main types

New: `MeasureOperations`, `MassProperties`, `MassOptions`, `DistanceResult`, `DistanceSolution`, `SupportKind`
(Vertex, Edge, Face), `LineHit`. Values from Geometry: `BoundingBox`, `OrientedBox`.

```csharp
public static class MeasureOperations   // extension members on Shape (and Edge, Face, Solid where they fit)
{
    // BRepGProp: Volume/Surface/LinearProperties
    public static double Volume(Shape shape, MassOptions? options = null);
    public static double Area(Shape shape, MassOptions? options = null);
    public static double Length(Shape shape, MassOptions? options = null);
    public static MassProperties MassProperties(Shape shape, Dimension dimension, MassOptions? options = null);
    // The centre of the faces' area alone: exact for one planar face bounded by lines and circles
    public static Point3D CentreOfArea(Shape shape, MassOptions? options = null);
    // BRepBndLib: Add, AddOptimal, AddOBB
    public static BoundingBox BoundingBox(Shape shape, bool optimal = false, bool useTriangulation = false);
    public static OrientedBox OrientedBox(Shape shape, bool optimal = false);
    // BRepExtrema_DistShapeShape
    public static Result<DistanceResult> Distance(Shape a, Shape b, DistanceOptions? options = null, CancellationToken cancellationToken = default);
    // BRepClass3d_SolidClassifier
    public static Containment Classify(Solid solid, Point3D point, double tolerance = Tolerances.Confusion);
    // BRepIntCurveSurface_Inter: where a line meets the faces, sorted along the line
    public static ImmutableArray<LineHit> Intersect(Shape shape, in Line3D line, double tolerance = Tolerances.Confusion);
}

public sealed record MassProperties
{
    public double Mass { get; init; }                         // volume, area or length, by Dimension (density 1)
    public Point3D CentreOfMass { get; init; }
    public Matrix3x3 Inertia { get; init; }                   // about the centre of mass
    public (double I1, double I2, double I3) PrincipalMoments { get; init; }
    public (Direction3D A1, Direction3D A2, Direction3D A3) PrincipalAxes { get; init; }
    public double RadiusOfGyration(in Axis3D axis);
    public double MomentOfInertia(in Axis3D axis);
}

public enum Dimension { Length = 1, Area = 2, Volume = 3 }
public sealed record MassOptions { public double Tolerance { get; init; } = 1e-6; public bool SkipShared { get; init; } public bool UseTriangulation { get; init; }
                                   public bool OnlyClosed { get; init; } public bool? ParallelVolumes { get; init; } }

public sealed record DistanceResult
{
    public double Value { get; init; }
    public ImmutableArray<DistanceSolution> Solutions { get; init; }   // each: points on a and b, their supports and parameters
    public bool IsInnerSolution { get; init; }                         // a point of one inside the other (a solid)
}
public readonly record struct DistanceSolution(Point3D OnA, Point3D OnB, SupportKind SupportA, Shape SupportShapeA,
                                               SupportKind SupportB, Shape SupportShapeB);
public readonly record struct LineHit(Point3D Point, double LineParameter, Face Face, double U, double V, Containment State);
```

- Volume, area and length are plain `double`s, not results: OCCT's integration can't fail on a valid shape; on an
  invalid one the number is what it is. Distance can fail (no solution, a degenerate input), so it is a `Result`.
- An area or volume with a `Tolerance` (the default, 1e-6) is right to it on every face: each face that isn't a plane,
  cylinder, cone, sphere or torus is integrated again ten times finer and the finer kept where they differ by more than the tolerance (or 1e-10, where that's finer), and the
  integration breaks at the surface's spans (an offset's, extrusion's or revolution's basis's too) and where a boundary
  crosses them (decisions 34801, 34802). Where nothing confirms a face, or a face's boundary can't be read, the figures
  are NaN, not a number off by an unknown amount. `Tolerance = 0` still asks for OCCT's fixed points, unchecked.
- Edge lengths along a parameter (`GCPnts_AbscissaPoint`) are on `EdgeCurve` (topology.md): `Length()`,
  `ParameterAtLength`.

## Examples

```csharp
double volume = part.IntegrateVolume();
Point3D centre = part.MassProperties(Dimension.Volume).CentreOfMass;
```

```csharp
BoundingBox box = part.BoundingBox(optimal: true);
Console.WriteLine($"{box.Size.X} × {box.Size.Y} × {box.Size.Z}");
```

```csharp
var gap = MeasureOperations.Distance(a, b);
if (gap.TryGetValue(out var d) && d.Value < 0.1) Warn(d.Solutions[0].OnA);
```

```csharp
bool inside = MeasureOperations.Classify(solid, probe) == Containment.In;
```

Points asked one at a time of a frozen shape (anything an operation returned) share one classifier, kept with the
shape for the next point: asking many points in a loop costs about one classifier and the points, not a classifier
each. Each point gets what a new classifier would give it (decision 2110).

```csharp
var hits = part.Intersect(new Line3D(eye, viewDirection));      // the faces a ray meets, nearest first
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `GProp_GProps props; BRepGProp.VolumeProperties(shape, props); props.Mass()` | `shape.IntegrateVolume()`, `MassProperties(Dimension.Volume)` |
| `SurfaceProperties`, `LinearProperties` | `Area`, `Length`, `MassProperties(Dimension.Area/Length)` |
| `props.CentreOfMass()`, `MatrixOfInertia()`, `PrincipalProperties()`, `GProp_PrincipalProps` | `MassProperties` record |
| `BRepGProp_Face` (normals for area integration in the modelling layer) | `FaceSurface.Normal` (topology.md) |
| `Bnd_Box box; BRepBndLib.Add(shape, box)`, `AddOptimal`, `AddOBB`, `Bnd_OBB` | `BoundingBox(…)`, `OrientedBox(…)` |
| `BRepExtrema_DistShapeShape` (`Value`, `NbSolution`, `PointOnShape1(i)`, `SupportTypeShape1`, `ParOnEdgeS1`…) | `Distance` → `DistanceResult` |
| `BRepExtrema_SupportType` (`IsVertex`, `IsOnEdge`, `IsInFace`) | `SupportKind` |
| `BRepClass3d_SolidClassifier.Perform(point, tol)`, `State()` | `Classify` → `Containment` |
| `BRepIntCurveSurface_Inter` (`Init`, `More`, `Next`, `Pnt`, `Face`, `W`, `U`, `V`) | `Intersect(shape, line)` → `LineHit`s |
| `GCPnts_AbscissaPoint.Length`, `GCPnts_UniformDeflection` | `EdgeCurve.Length`, `Discretize` |

**Disappears:** `GProp_GProps` as a mutable accumulator, `Bnd_Box`'s `Add`/`Enlarge`/`IsVoid` protocol, the
`Init/More/Next` loops.

## Open points

- Mass properties use density 1 and the shape's own units; a `Density` option is easy to add if Bento wants mass.
- `BoundingBox(shape)` without `optimal` is OCCT's fast, loose box; build123d asks for `optimal` by default. Keep the
  kernel's default loose and fast, and let the modelling layer choose.
