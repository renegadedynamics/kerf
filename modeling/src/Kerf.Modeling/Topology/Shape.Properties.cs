using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Modeling;

public abstract partial class Shape
{
    // ---- Kind ----

    /// <summary>The kind of shape. An empty shape reports its own class's kind.</summary>
    public ShapeType ShapeType
    {
        get
        {
            if (IsEmpty)
                for (var t = GetType(); t != typeof(Shape); t = t.BaseType!)
                    if (Enum.TryParse<ShapeType>(t.Name, out var kind))
                        return kind;
            return TopoUtil.FromTopAbs(TopoUtil.ShapeTypeOf(_wrapped));
        }
    }

    /// <summary>The type of the underlying curve (of an edge) or surface (of a face); <see cref="GeomType.Other"/> for other shapes.</summary>
    /// <exception cref="InvalidOperationException">The shape is empty.</exception>
    public GeomType GeomType
    {
        get
        {
            if (IsEmpty)
                throw new InvalidOperationException("Cannot determine geometry type of an empty shape");
            switch (_wrapped.Type)
            {
                case global::Kerf.Topology.ShapeType.Edge:
                    return GeomTypeOf(((global::Kerf.Topology.Edge)_wrapped).ToCurve().Kind);
                case global::Kerf.Topology.ShapeType.Face:
                    return GeomTypeOf(((global::Kerf.Topology.Face)_wrapped).ToSurface().Kind);
                default:
                    return GeomType.Other;
            }
        }
    }

    /// <summary>The geometry type of a kind of curve.</summary>
    internal static GeomType GeomTypeOf(CurveKind type) => type switch
    {
        CurveKind.Line => GeomType.Line,
        CurveKind.Circle => GeomType.Circle,
        CurveKind.Ellipse => GeomType.Ellipse,
        CurveKind.Hyperbola => GeomType.Hyperbola,
        CurveKind.Parabola => GeomType.Parabola,
        CurveKind.BezierCurve => GeomType.Bezier,
        CurveKind.BSplineCurve => GeomType.BSpline,
        CurveKind.OffsetCurve => GeomType.Offset,
        _ => GeomType.Other,
    };

    /// <summary>The geometry type of a kind of surface.</summary>
    internal static GeomType GeomTypeOf(SurfaceKind type) => type switch
    {
        SurfaceKind.Plane => GeomType.Plane,
        SurfaceKind.Cylinder => GeomType.Cylinder,
        SurfaceKind.Cone => GeomType.Cone,
        SurfaceKind.Sphere => GeomType.Sphere,
        SurfaceKind.Torus => GeomType.Torus,
        SurfaceKind.BezierSurface => GeomType.Bezier,
        SurfaceKind.BSplineSurface => GeomType.BSpline,
        SurfaceKind.SurfaceOfRevolution => GeomType.Revolution,
        SurfaceKind.SurfaceOfExtrusion => GeomType.Extrusion,
        SurfaceKind.OffsetSurface => GeomType.Offset,
        _ => GeomType.Other,
    };

    // ---- Placement ----

    /// <summary>The shape's Location.</summary>
    /// <exception cref="InvalidOperationException">The shape is empty.</exception>
    public Location Location
    {
        get
        {
            NeedsGeometry("location");
            return new Location(_wrapped.Placement);
        }
    }

    /// <summary>The position part of the shape's Location.</summary>
    public Vector Position
    {
        get
        {
            NeedsGeometry("position");
            return Location.Position;
        }
    }

    /// <summary>The orientation part of the shape's Location, as rotations in degrees.</summary>
    public Vector Orientation
    {
        get
        {
            NeedsGeometry("orientation");
            return Location.Orientation;
        }
    }

    // ---- Measures ----

    /// <summary>The surface area of all the faces in this shape.</summary>
    public double Area => IsEmpty ? 0.0 : TopoUtil.Area(_wrapped);

    /// <summary>The volume of the shape: zero for shapes without one.</summary>
    public abstract double Volume { get; }

    /// <summary>The mass properties for the shape's kind (length, area or volume); null for a vertex, which has none.</summary>
    internal static MassProperties? MassPropertiesOf(global::Kerf.Topology.Shape shape) => TopoUtil.ShapeTypeOf(shape) switch
    {
        global::Kerf.Topology.ShapeType.Vertex => null,
        global::Kerf.Topology.ShapeType.Edge or global::Kerf.Topology.ShapeType.Wire => shape.MassProperties(Dimension.Length),
        global::Kerf.Topology.ShapeType.Face or global::Kerf.Topology.ShapeType.Shell => shape.MassProperties(Dimension.Area),
        _ => shape.MassProperties(Dimension.Volume),
    };

    /// <summary>The measure OCCT computes for the shape's kind: length, area or volume (build123d's <c>compute_volume</c>).</summary>
    public double ComputeVolume()
    {
        if (IsEmpty || _wrapped.IsVertex())
            return 0.0;
        return TopoUtil.ShapeTypeOf(_wrapped) switch
        {
            global::Kerf.Topology.ShapeType.Edge or global::Kerf.Topology.ShapeType.Wire => _wrapped.Length(),
            global::Kerf.Topology.ShapeType.Face or global::Kerf.Topology.ShapeType.Shell => _wrapped.Area(),
            _ => _wrapped.Volume(),
        };
    }

    /// <summary>The inertia tensor of the shape about its centre of mass, as rows.</summary>
    public double[][] MatrixOfInertia
    {
        get
        {
            if (IsEmpty)
                return [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
            Span<double> m = stackalloc double[9];
            _wrapped.MassProperties(Dimension.Volume).Inertia.CopyTo(m);
            return [[m[0], m[1], m[2]], [m[3], m[4], m[5]], [m[6], m[7], m[8]]];
        }
    }

    /// <summary>The principal axes of inertia, each with its moment.</summary>
    public IReadOnlyList<(Vector Axis, double Moment)> PrincipalProperties
    {
        get
        {
            NeedsGeometry("principal axes");
            var props = _wrapped.MassProperties(Dimension.Volume);
            var (m1, m2, m3) = props.PrincipalMoments;
            var (a1, a2, a3) = props.PrincipalAxes;
            return [(new Vector(a1), m1), (new Vector(a2), m2), (new Vector(a3), m3)];
        }
    }

    /// <summary>The first moments of mass (Mx, My, Mz).</summary>
    public (double Mx, double My, double Mz) StaticMoments
    {
        get
        {
            if (IsEmpty)
                return (0, 0, 0);
            var moments = _wrapped.MassProperties(Dimension.Volume).StaticMoments;
            return (moments.X, moments.Y, moments.Z);
        }
    }

    /// <summary>The radius of gyration of the shape about an axis.</summary>
    public double RadiusOfGyration(Axis axis)
    {
        NeedsGeometry("radius of gyration");
        var ax1 = axis.ToGpAx1();
        return _wrapped.MassProperties(Dimension.Volume).RadiusOfGyration(ax1);
    }

    /// <summary>The centre of the shape, by the calculation its class uses by default.</summary>
    public abstract Vector Center();

    /// <summary>The centre of the shape by a chosen calculation.</summary>
    /// <exception cref="NotSupportedException">The class doesn't offer a choice.</exception>
    public virtual Vector Center(CenterOf centerOf) =>
        throw new NotSupportedException($"{GetType().Name}.Center doesn't take a CenterOf");

    /// <summary>The centre of several shapes, weighted by their volume (mass) or by their bounding boxes.</summary>
    /// <exception cref="ArgumentException"><see cref="CenterOf.Geometry"/> isn't implemented.</exception>
    public static Vector CombinedCenter(IEnumerable<Shape> objects, CenterOf centerOf = CenterOf.Mass)
    {
        var list = objects.ToList();
        switch (centerOf)
        {
            case CenterOf.Mass:
            {
                var totalMass = list.Sum(o => o.ComputeVolume());
                var sum = list.Select(o => o.Center(CenterOf.Mass) * o.ComputeVolume()).Aggregate((a, b) => a + b);
                return sum * (1.0 / totalMass);
            }
            case CenterOf.BoundingBox:
            {
                var sum = list.Select(o => o.BoundingBox().Center()).Aggregate((a, b) => a + b);
                return sum * (1.0 / list.Count);
            }
            default:
                throw new ArgumentException("CenterOf.Geometry not implemented");
        }
    }

    // ---- Checks ----

    /// <summary>True if OCCT finds no defect in the shape or its sub-shapes (<c>BRepCheck_Analyzer</c>).</summary>
    public bool IsValid
    {
        get
        {
            if (IsEmpty)
                return true;
            return _wrapped.Validate().IsValid;
        }
    }

    /// <summary>
    /// True if each non-degenerate edge belongs to exactly two faces and the faces are consistently oriented: the shape
    /// is watertight. Faces that overlap or cross each other in space aren't detected; use <see cref="IsValid"/> for that.
    /// </summary>
    public bool IsManifold
    {
        get
        {
            if (IsEmpty)
                return false;
            foreach (var shape in TopoUtil.TopLevelShapes(_wrapped))
            {
                // The faces each edge is used by, once per use (TopExp::MapShapesAndAncestors): a seam counts its face twice
                var uses = new Dictionary<global::Kerf.Topology.Shape, int>(ShapeComparer.Same);
                foreach (var face in shape.Explore<global::Kerf.Topology.Face>())
                    foreach (var e in face.Explore<global::Kerf.Topology.Edge>())
                        uses[e] = uses.GetValueOrDefault(e) + 1;
                foreach (var (e, count) in uses)
                {
                    // A degenerate edge (a sphere's pole, a cone's apex) belongs to one face only
                    if (((global::Kerf.Topology.Edge)e).IsDegenerated)
                        continue;
                    if (count != 2)
                        return false;
                }
                // In a closed shell each edge is used once in each direction
                if (shape.MisorientedEdges(checkInternalEdges: true).Length > 0)
                    return false;
            }
            return true;
        }
    }

    // ---- Bounding boxes ----

    /// <summary>A box aligned with the axes containing this shape (tight when <paramref name="optimal"/>).</summary>
    public BoundBox BoundingBox(double? tolerance = null, bool optimal = true) => Kerf.Modeling.BoundBox.FromShape(this, tolerance, optimal);

    /// <summary>The smallest box in any orientation containing this shape.</summary>
    public OrientedBoundBox OrientedBoundingBox()
    {
        return Kerf.Modeling.OrientedBoundBox.FromOrientedBox(IsEmpty ? Kerf.Geometry.OrientedBox.Empty : _wrapped.OrientedBox());
    }

    // ---- Distances ----

    /// <summary>The minimal distance between this shape and another.</summary>
    /// <exception cref="InvalidOperationException">Either shape is empty.</exception>
    public double Distance(Shape other)
    {
        if (IsEmpty || other.IsEmpty)
            throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
        return Failures.Get(_wrapped.Distance(other._wrapped), () => "distance").Value;
    }

    /// <summary>The minimal distance between this shape and another shape.</summary>
    public double DistanceTo(Shape other) => DistanceToWithClosestPoints(other).Distance;

    /// <summary>The minimal distance between this shape and a point.</summary>
    public double DistanceTo(Vector point) => DistanceToWithClosestPoints(point).Distance;

    /// <summary>The points on this shape and another where they are closest.</summary>
    public (Vector OnThis, Vector OnOther) ClosestPoints(Shape other)
    {
        var (_, a, b) = DistanceToWithClosestPoints(other);
        return (a, b);
    }

    /// <summary>The points on this shape and at <paramref name="point"/> where they are closest.</summary>
    public (Vector OnThis, Vector OnOther) ClosestPoints(Vector point)
    {
        var (_, a, b) = DistanceToWithClosestPoints(point);
        return (a, b);
    }

    /// <summary>The minimal distance to another shape and the points on each where it is found.</summary>
    public (double Distance, Vector OnThis, Vector OnOther) DistanceToWithClosestPoints(Shape other)
    {
        if (IsEmpty || other.IsEmpty)
            throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
        return DistanceToTopo(other._wrapped);
    }

    /// <summary>The minimal distance to a point and the points on each where it is found.</summary>
    public (double Distance, Vector OnThis, Vector OnOther) DistanceToWithClosestPoints(Vector point)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
        var pnt = point.ToGpPnt();
        return DistanceToTopo(global::Kerf.Topology.Vertex.At(pnt));
    }

    private (double, Vector, Vector) DistanceToTopo(global::Kerf.Topology.Shape other)
    {
        var distance = Failures.Get(_wrapped.Distance(other), () => "distance");
        var nearest = distance.Solutions[0];
        return (distance.Value, new Vector(nearest.OnA), new Vector(nearest.OnB));
    }

    /// <summary>The minimal distances from this shape to each of <paramref name="others"/>.</summary>
    public IEnumerable<double> Distances(params IEnumerable<Shape> others)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
        foreach (var other in others)
        {
            if (other.IsEmpty)
                throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
            yield return Failures.Get(_wrapped.Distance(other._wrapped), () => "distance").Value;
        }
    }

    /// <summary>The faces of this shape an axis passes through, nearest the axis's position first.</summary>
    public ShapeList<Face> FacesIntersectedByAxis(Axis axis, double tolerance = 1e-4)
    {
        if (IsEmpty)
            return [];
        var ax1 = axis.ToGpAx1();
        var line = new Line3D(ax1);
        var origin = axis.Position.ToGpPnt();
        var found = new List<(global::Kerf.Topology.Face Face, double Distance)>();
        foreach (var hit in _wrapped.Intersect(line, tolerance))
            found.Add((hit.Face, Math.Abs(origin.SquareDistanceTo(hit.Point))));
        return new ShapeList<Face>(found.OrderBy(f => f.Distance).Select(f => (Face)CastLike(f.Face)));
    }

    // ---- Meshing ----

    /// <summary>A triangulated approximation: the vertices and the triangles as index triples into them.</summary>
    public (IReadOnlyList<Vector> Vertices, IReadOnlyList<(int, int, int)> Triangles) Tessellate(double tolerance, double angularTolerance = 0.1)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot tessellate an empty shape");
        // The kernel's mesher leaves the shape alone and hands the meshes back (unit 10L3)
        var options = new MeshOptions { LinearDeflection = tolerance, AngularDeflection = angularTolerance, Relative = true, Parallel = true };
        var mesh = Failures.Get(_wrapped.Mesh(options), () => "tessellate");
        var meshes = new Dictionary<global::Kerf.Topology.Shape, global::Kerf.Meshing.FaceMesh>(ShapeComparer.Same);
        foreach (var faceMesh in mesh.Faces)
            meshes.TryAdd(faceMesh.Face, faceMesh);
        var vertices = new List<Vector>();
        var triangles = new List<(int, int, int)>();
        int offset = 0;
        foreach (var face in Faces())
        {
            if (!meshes.TryGetValue(face.TopoShape, out var faceMesh) || faceMesh.TriangleCount == 0)
                continue;
            bool reverse = face.TopoShape.Orientation == global::Kerf.Topology.Orientation.Reversed;
            // The mesh's triangles follow the surface's normal, flipped again by a mirroring placement: this follows the face
            bool mirrored = faceMesh.Face.Placement.Transformation.IsNegative;
            var nodes = faceMesh.GetNodes();
            foreach (var node in nodes)
                vertices.Add(new Vector(node.X, node.Y, node.Z));
            foreach (var t in faceMesh.GetTriangles())
            {
                int a = t.A + offset, b = t.B + offset, c = t.C + offset;
                triangles.Add(mirrored ? (a, c, b) : (a, b, c));
            }
            offset += nodes.Length;
        }
        return (vertices, triangles);
    }
}
