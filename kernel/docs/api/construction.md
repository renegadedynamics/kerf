# API review: construction

*Unit 10D, draft; unit 10L2 implemented it (README.md, section 11, has what differs and what is left). Namespace `Kerf.Construction`, in ModelingAlgorithms.*

## Main types

All new, over the engine's `BRepPrimAPI`, `BRepBuilderAPI`, `BRepOffsetAPI_MakePipeShell`/`ThruSections`/`MakeFilling`,
`BRepFill`, `BRepLib` and `BRepProj`:

- `Primitives`: `Box`, `Wedge`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `HalfSpace`.
- `Sweeps`: `Extrude`, `Revolve`, `Sweep`, `Loft`, `Ruled`.
- `ShapeFactory`: edges, wires, faces, shells and solids from geometry, as static extension members on the shape types
  (`Edge.Line`, `Wire.FromEdges`, `Face.FromWire`), checked and connected, unlike Topology's raw `Create`.
- `ShapeTransforms`: `Transformed`, `Mirrored`, `Scaled` (copying geometry where a placement can't express it).
- `Projections`: wires projected onto shapes. `Surfaces`: `Fill` (plate surfaces through boundary curves).
- Options: `BoxOptions`… where a primitive has more than its sizes, `ExtrudeOptions`, `RevolveOptions`, `SweepOptions`,
  `LoftOptions`, `FillOptions`; enums `Transition` (Transformed, RightCorner, RoundCorner), `SweepFrame` (Frenet,
  CorrectedFrenet, Fixed, Binormal, Auxiliary).

```csharp
public static class Primitives
{
    // in extension(Solid) blocks: Solid.Box(…), Solid.Cylinder(…)
    public static Result<Built<Solid>> Box(double dx, double dy, double dz, in Frame3D frame = default /* World */);
    public static Result<Built<Solid>> Box(Point3D corner1, Point3D corner2);
    public static Result<Built<Solid>> Cylinder(double radius, double height, in Frame3D frame = default, double angle = 2 * Math.PI);
    public static Result<Built<Solid>> Cone(double radius1, double radius2, double height, in Frame3D frame = default, double angle = 2 * Math.PI);
    public static Result<Built<Solid>> Sphere(double radius, in Frame3D frame = default, SphereOptions? options = null);  // angles
    public static Result<Built<Solid>> Torus(double majorRadius, double minorRadius, in Frame3D frame = default, TorusOptions? options = null);   // part of the minor circle: a slice of the tube, not OCCT's solid under the arc
    public static Result<Built<Solid>> Wedge(double dx, double dy, double dz, WedgeOptions options, in Frame3D frame = default);
    public static Result<Solid> HalfSpace(Face boundary, Point3D inside);
}

public static class Sweeps
{
    // extension(Shape profile): profile.Extrude(…), profile.Revolve(…)
    public static Result<Built<Shape>> Extrude(Shape profile, Vector3D direction, ExtrudeOptions? options = null);
    public static Result<Built<Shape>> Revolve(Shape profile, in Axis3D axis, double angle, RevolveOptions? options = null);
    public static Result<Built<Shape>> Sweep(Shape profile, Wire spine, SweepOptions? options = null,
                                             IProgress<double>? progress = null, CancellationToken cancellationToken = default);
    public static Result<Built<Shape>> Sweep(IReadOnlyList<Shape> profiles, Wire spine, SweepOptions? options = null, …);
    public static Result<Built<Shape>> Loft(IReadOnlyList<Shape> sections, LoftOptions? options = null, …);
    public static Result<Built<Face>> Ruled(Edge first, Edge second);         // and (Wire, Wire) -> Shell: BRepFill.Face/Shell
}

public sealed record SweepOptions
{
    public bool MakeSolid { get; init; } = true;
    public Transition Transition { get; init; } = Transition.Transformed;
    public SweepFrame Frame { get; init; } = SweepFrame.CorrectedFrenet;
    public bool TryOtherFrames { get; init; } = true;           // on failure, Frenet then discrete, with a warning (SweptWithFrenet, SweptWithDiscrete)
    public Direction3D? Binormal { get; init; }                 // SweepFrame.Binormal
    public Shape? AuxiliarySpine { get; init; }                 // SweepFrame.Auxiliary
    public bool WithContact { get; init; }  public bool WithCorrection { get; init; }
    public double Tolerance { get; init; } = 1e-4;
}

public static class ShapeFactory   // static extension members
{
    // Vertex.At(Point3D)
    // Edge.Line(Point3D, Point3D), Edge.FromCurve(Curve3D, Interval? range = null), Edge.FromCurve(Curve2D, Surface, Interval?),
    // Edge.Circle(Circle3D), Edge.Arc(Circle3D, double, double)                                  -> Result<Edge>
    // Wire.FromEdges(IEnumerable<Edge>, double tolerance = Tolerances.Confusion)                  -> Result<Wire>   (connects; WireError → diagnostics)
    // Wire.Polygon(ReadOnlySpan<Point3D>, bool closed)                                           -> Result<Wire>
    // Face.FromWire(Wire, bool onlyPlane = false), Face.FromWires(Wire outer, IEnumerable<Wire> holes)  -> Result<Face>
    // Face.FromSurface(Surface, (Interval U, Interval V)? bounds = null, Wire? boundary = null)    -> Result<Face>
    // Face.FromPlane(Plane3D, Wire? boundary = null)
    // Shell.FromSurface(Surface, …), Solid.FromShells(IEnumerable<Shell>)                       -> Result<Solid>
    // Wire.TryGetPlane(out Plane3D), Shape.TryFindSurface(double tolerance, out Surface)         (BRepLib_FindSurface)
}

public static class ShapeTransforms  // extension(Shape shape)
{
    public static Result<Built<Shape>> Transformed(Shape shape, in Transform3D transform);        // BRepBuilderAPI_Transform: copies when scaling
    public static Result<Built<Shape>> Transformed(Shape shape, in GeneralTransform3D transform); // BRepBuilderAPI_GTransform: non-uniform scale
    public static Result<Built<Shape>> Mirrored(Shape shape, in Frame3D plane);
    public static Result<Built<Shape>> Scaled(Shape shape, Point3D centre, double factor);
}

public static class Projections
{
    public static Result<ImmutableArray<Wire>> Project(Wire wire, Shape target, Direction3D direction);  // BRepProj_Projection, cylindrical
    public static Result<ImmutableArray<Wire>> Project(Wire wire, Shape target, Point3D eye);             // conical
}

public static class Surfaces
{
    public static Result<Built<Face>> Fill(IReadOnlyList<FillBoundary> boundaries, FillOptions? options = null, …);
    // FillBoundary: an Edge with a Continuity (C0, G1, G2) and an optional support Face; FillOptions: degree, points on curve, iterations, tolerances
}
```

## Examples

```csharp
Solid box = Solid.Box(80, 60, 10).Value;                                         // or Primitives.Box(80, 60, 10)
Solid post = Solid.Cylinder(5, 30, new Frame3D(new Point3D(40, 30, 10), Direction3D.Z)).Value;
```

```csharp
Wire outline = Wire.Polygon([new(0, 0, 0), new(40, 0, 0), new(40, 20, 0), new(0, 20, 0)], closed: true).Value;
var extruded = Face.FromWire(outline).Bind(face => face.Extrude(new Vector3D(0, 0, 15)));
```

```csharp
var swept = Sweeps.Sweep(profile, path, new SweepOptions { Transition = Transition.RoundCorner, Frame = SweepFrame.Frenet });
```

```csharp
var lofted = Sweeps.Loft([bottomWire, middleWire, topWire], new LoftOptions { Ruled = false, MakeSolid = true });
```

```csharp
var spun = Sweeps.Revolve(profile, new Axis3D(Point3D.Origin, Direction3D.Z), Math.PI);
IReadOnlyList<Shape> capFaces = spun.Value.History.Generated(profile);   // what build123d's revolve history needs
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `BRepPrimAPI_MakeBox/Cylinder/Cone/Sphere/Torus/Wedge`, `.Solid()` | `Primitives.Box`… (`Solid.Box`…) |
| `BRepPrimAPI_MakeHalfSpace` | `Primitives.HalfSpace` |
| `BRepPrimAPI_MakePrism`, `MakeRevol` (and their `FirstShape`/`LastShape`) | `Sweeps.Extrude`, `Revolve` (history: `Generated`, plus `Built.History` entries for the caps) |
| `BRepOffsetAPI_MakePipeShell` (`Add`, `SetMode`, `SetTransitionMode`, `Build`, `MakeSolid`) | `Sweeps.Sweep` with `SweepOptions` |
| `GeomFill_Frenet`, `GeomFill_CorrectedFrenet`, `GeomFill_TrihedronLaw` | `SweepFrame` |
| `BRepBuilderAPI_TransitionMode` | `Transition` |
| `BRepOffsetAPI_ThruSections` | `Sweeps.Loft` |
| `BRepFill.Face`, `BRepFill.Shell` | `Sweeps.Ruled` |
| `BRepOffsetAPI_MakeFilling` | `Surfaces.Fill` |
| `BRepBuilderAPI_MakeVertex/Edge/Wire/Polygon/Face/Solid` | `Vertex.At`, `Edge.Line`/`FromCurve`, `Wire.FromEdges`/`Polygon`, `Face.FromWire`/`FromSurface`, `Solid.FromShells` |
| `BRepBuilderAPI_WireError`, `FaceError`, `EdgeError` | Diagnostics (`construction.disconnected-wire`, `construction.non-manifold-wire`…) |
| `BRepBuilderAPI_Transform`, `GTransform` | `ShapeTransforms.Transformed` |
| `BRepBuilderAPI_Copy` | Not needed: shapes are immutable; `Transformed` copies when it must |
| `BRepLib.BuildCurves3d` | Done by the factories that make edges on surfaces |
| `BRepLib_FindSurface` | `Wire.TryGetPlane`, `Shape.TryFindSurface` |
| `BRepProj_Projection` | `Projections.Project` |

**Disappears:** the `Make…` objects' `Build`/`IsDone`/`Shape()` protocol; `BRepBuilderAPI_MakeShape`'s
`Modified`/`Generated` (now `Built.History`); `BRepPrimAPI_MakeOneAxis`/`MakeSweep` bases; `BRepPreviewAPI`.

## Open points

- `Frame3D` defaults: `in Frame3D frame = default` can't express `Frame3D.World`; 10L uses overloads (with and without a
  frame) rather than a `default` that means the world frame, since `default(Frame3D)` is invalid.
- Helices (`HelixBRep`, `HelixGeom`) aren't used by the modelling layer, which builds its helix from geometry; add
  `Sweeps.Helix` when wanted.
- Evolved solids (`BRepOffsetAPI_MakeEvolved`), the middle path and normal projection stay engine-only for now.
