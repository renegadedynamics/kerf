# API review: topology

*Unit 10D, draft; unit 10L1 implemented it but for `WireCurve`, meshes and the measuring members (README.md, section
10). Namespace `Kerf.Topology`, in ModelingData.*

## Main types

`Shape` and the typed `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `CompSolid`, `Compound` (README section 5);
`ShapeType`, `Orientation`, `Containment` (from TopAbs); `Placement` (from `TopLoc_Location`). New: `IShape`, `IShape<T>`, `IShape1D`, `IShape2D`, `IShape3D`,
`ShapeComparer`, `ShapeHistory`, `ShapeGraph`, `EdgeCurve`, `FaceSurface`, `WireCurve`, `SubShapes<T>`.

```csharp
public readonly struct Shape
{
    // identity and placement
    public ShapeType Type { get; }  public Orientation Orientation { get; }  public Placement Placement { get; }
    public bool IsNull { get; }
    public Shape Reversed();  public Shape Oriented(Orientation orientation);
    public Shape Located(Placement placement);                 // replaces the placement
    public Shape Moved(in Transform3D transform);               // composes; shares the data
    public bool IsSame(Shape other);  public bool IsPartner(Shape other);

    // traversal
    public int ChildCount { get; }
    public SubShapes<Shape> Children { get; }                   // TopoDS_Iterator: struct enumerator, no allocation
    public SubShapes<TShape> Explore<TShape>(ShapeType? avoid = null) where TShape : struct, IShape<TShape>;  // TopExp_Explorer
    public ImmutableArray<Vertex> Vertices();                   // unique (IsSame), in exploration order: TopExp.MapShapes
    public ImmutableArray<Edge> Edges();  public ImmutableArray<Wire> Wires();  public ImmutableArray<Face> Faces();
    public ImmutableArray<Shell> Shells();  public ImmutableArray<Solid> Solids();
    public ImmutableArray<Shape> TopLevelShapes();              // a compound's leaves, compounds flattened

    // flags kept on the data
    public bool IsClosed { get; }  public bool IsInfinite { get; }  public bool IsOrientable { get; }

    // assembling without checks (TopoDS_Builder): what the modelling layer's UnsafeAccessors reach today
    public Shape WithChildren(IEnumerable<Shape> children);
    public Shape WithoutChild(Shape child);
}

public static class ShapeFactories   // static extension members, raw assembly: no sewing, no checks
{
    // Compound.Create(IEnumerable<Shape>), Wire.Create(IEnumerable<Edge>), Shell.Create(IEnumerable<Face>),
    // Solid.Create(IEnumerable<Shell>), CompSolid.Create(IEnumerable<Solid>)
}

public readonly struct Vertex { public Point3D Point { get; } public double Tolerance { get; } }

public readonly struct Edge
{
    public Interval Range { get; }  public double Tolerance { get; }
    public bool IsDegenerated { get; }  public bool IsClosed { get; }  public bool IsSeam(Face face);
    public Vertex FirstVertex { get; }  public Vertex LastVertex { get; }      // orientation-aware: TopExp.FirstVertex(E, true)
    public bool TryGetCurve(out Curve3D curve, out Interval range);           // placed: BRep_Tool.Curve
    public bool TryGetCurveOnFace(Face face, out Curve2D curve, out Interval range);  // BRep_Tool.CurveOnSurface
    public double ParameterOf(Vertex vertex);
    public EdgeCurve ToCurve();                                               // the evaluator below
}

public readonly struct Face
{
    public double Tolerance { get; }
    public bool TryGetSurface(out Surface surface);            // false for a face that is only a mesh (STL)
    public Wire OuterWire { get; }                              // BRepTools.OuterWire
    public (Interval U, Interval V) UVBounds { get; }           // BRepTools.UVBounds
    public bool TryGetMesh(out FaceMesh mesh);                  // a stored triangulation (STL, a BREP with triangles)
    public FaceSurface ToSurface();
}

public readonly struct Wire { public ImmutableArray<Edge> OrderedEdges(); public WireCurve ToCurve(); }
```

**Evaluators** (from BRepAdaptor; sealed classes, made once and queried many times, in world coordinates):

```csharp
public sealed class EdgeCurve
{
    public Edge Edge { get; }  public Interval Range { get; }  public CurveKind Kind { get; }
    public bool IsClosed { get; }  public bool IsPeriodic { get; }  public double Period { get; }
    public Point3D Evaluate(double u);
    public CurveDerivatives Derivatives(double u, int order);
    public Vector3D Tangent(double u);
    public bool TryGetLine(out Line3D line);  public bool TryGetCircle(out Circle3D circle);   // and Ellipse, Hyperbola, Parabola
    public bool TryGetBSpline(out BSplineCurve3D curve);  public bool TryGetBezier(out BezierCurve3D curve);
    public double IntegrateLength();                                   // Measurement area, GCPnts_AbscissaPoint
    public double ParameterAtLength(double length, double fromParameter);
    public ImmutableArray<CurvePoint> Discretize(DiscretizeOptions options);   // GCPnts_(Quasi)UniformDeflection
}

public sealed class FaceSurface
{
    public Face Face { get; }  public SurfaceKind Kind { get; }  public (Interval U, Interval V) Bounds { get; }
    public Point3D Evaluate(double u, double v);
    public Direction3D Normal(double u, double v);              // the face's orientation applied
    public SurfaceCurvature Curvature(double u, double v);      // BRepLProp_SLProps: min, max, mean, Gaussian, directions
    public (double U, double V) Parameters(Point3D point, double tolerance);  // ShapeAnalysis_Surface.ValueOfUV
    public Containment Classify(double u, double v, double tolerance);        // IntTools_FClass2d
    public bool TryGetPlane(out Plane3D plane);  public bool TryGetCylinder(out Cylinder cylinder);  // and Cone, Sphere, Torus
}

public sealed class WireCurve { /* BRepAdaptor_CompCurve: Evaluate, Derivatives, Range over the whole wire */ }
```

**History** (`BRepTools_History`, immutable, shared by every operation's `Built<T>`):

```csharp
public sealed class ShapeHistory
{
    public static ShapeHistory Empty { get; }
    public IReadOnlyList<Shape> Modified(Shape initial);
    public IReadOnlyList<Shape> Generated(Shape initial);
    public bool IsRemoved(Shape initial);
    public IEnumerable<ShapeHistoryEntry> Entries { get; }      // (Initial, Kind, Results): the maps the modelling layer copies today
    public ShapeHistory Then(ShapeHistory next);                // composition: this operation, then the next
}

public sealed class ShapeGraph                                  // TopExp.MapShapesAndAncestors, built once per shape
{
    public static ShapeGraph Build(Shape root);
    public IReadOnlyList<Shape> Ancestors(Shape shape, ShapeType ancestorType);
    public int IndexOf(Shape shape);                            // 0-based, -1 when absent
    public IReadOnlyList<Shape> All(ShapeType type);
}
```

## Examples

```csharp
foreach (Face face in solid.AllFaces())
    if (face.ToSurface().TryGetPlane(out Plane3D plane) && plane.Normal.IsParallel(Direction3D.Z, 1e-9))
        topFaces.Add(face);
```

```csharp
var graph = ShapeGraph.Build(solid);
IReadOnlyList<Shape> facesOfEdge = graph.Ancestors(edge, ShapeType.Face);
```

```csharp
if (edge.ToCurve().TryGetCircle(out Circle3D circle))
    Console.WriteLine($"radius {circle.Radius}");
```

```csharp
Compound assembly = Compound.Create([part1, part2.Moved(Transform3D.Translation(new(50, 0, 0)))]);
```

```csharp
switch (shape.Type)
{
    case ShapeType.Solid: Measure((Solid)shape); break;
    case ShapeType.Face when shape.TryAs(out Face face): Show(face); break;
}
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `TopoDS_Shape`, `TopoDS_Edge`… | `Shape`, `Edge`… |
| `TopoDS.Edge(s)` | `(Edge)s`, `s.As<Edge>()`, `s.TryAs(out Edge e)` |
| `TopAbs_ShapeEnum.TopAbs_FACE`, `TopAbs_Orientation`, `TopAbs_State` | `ShapeType.Face`, `Orientation`, `Containment` |
| `TopLoc_Location` | `Placement` |
| `shape.ShapeType()`, `NbChildren()`, `Closed()`, `Location()` | `Type`, `ChildCount`, `IsClosed`, `Placement` |
| `TopoDS_Iterator` | `shape.Children` |
| `TopExp_Explorer` | `shape.Explore<T>()` |
| `TopExp.MapShapes`, `MapShapesAndAncestors`, `FirstVertex`, `LastVertex` | `Faces()`…, `ShapeGraph`, `Edge.FirstVertex`… |
| `TopTools_IndexedMapOfShape`, `…DataMapOfShapeShape`, `…ListOfShape` | `ShapeGraph`; `Dictionary<Shape, T>(ShapeComparer.Same)`; `ImmutableArray<Shape>` |
| `TopTools_ShapeMapHasher` | `ShapeComparer.Same`, `ShapeComparer.Equal` |
| `BRep_Tool.Pnt/Tolerance/Range/Curve/CurveOnSurface/Surface/Degenerated/IsClosed/Parameter/Triangulation` | `Vertex.Point`, `Tolerance`, `Edge.Range`, `TryGetCurve`, `TryGetCurveOnFace`, `Face.TryGetSurface`, `IsDegenerated`, `IsClosed`, `ParameterOf`, `Face.TryGetMesh` |
| `BRepTools.UVBounds`, `OuterWire`; `BRepTools_WireExplorer` | `Face.UVBounds`, `Face.OuterWire`; `Wire.OrderedEdges()` |
| `BRepAdaptor_Curve`, `_Surface`, `_CompCurve`, `_Curve2d` | `EdgeCurve`, `FaceSurface`, `WireCurve`, `Edge.TryGetCurveOnFace` |
| `BRepLProp_SLProps`, `BRepLProp.Continuity` | `FaceSurface.Curvature`, `Normal`; `EdgeCurve` continuity |
| `BRepTools_History` | `ShapeHistory` |
| `TopoDS_Builder` (`MakeCompound`, `Add`, `Remove`), `BRep_Builder` | `Compound.Create`, `Wire.Create`…, `WithChildren`, `WithoutChild` |
| `TopoDS_TShape` and subclasses, `Free/Locked/Modified/Checked` flags, `FreezeGraph` | Engine; shapes are immutable |

**Disappears:** `TopoDS_HShape`, `TopoDS_FrozenShape`/`LockedShape` (with copy at entry, 10H), the alerts, OCCT's
`HashCode`/`DumpJson`.

## Open points

- `Faces()` and the others return `ImmutableArray<T>` per call. The data could cache its unique sub-shapes (shapes are
  immutable once published), making repeated calls cheap; 10L measures before caching.
- `SubShapes<T>` is a struct implementing `IEnumerable<T>` with a struct enumerator: `foreach` allocates nothing, LINQ
  boxes once.
- `Shape ==` is OCCT's `IsEqual` (orientation included), matching design-rules.md's exact equality; `IsSame` is the
  method most callers want for "the same face". Document it in the type's summary.
