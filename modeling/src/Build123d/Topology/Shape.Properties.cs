using Build123d.Occt;

namespace Build123d;

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
            switch (_wrapped)
            {
                case TopoDS_Edge edge:
                {
                    using var curve = new BRepAdaptor_Curve(edge);
                    return GeomTypeOf(curve.GetType_());
                }
                case TopoDS_Face face:
                {
                    using var surface = new BRepAdaptor_Surface(face);
                    return GeomTypeOf(surface.GetType_());
                }
                default:
                    return GeomType.Other;
            }
        }
    }

    /// <summary>The geometry type of a kind of curve.</summary>
    internal static GeomType GeomTypeOf(GeomAbs_CurveType type) => type switch
    {
        GeomAbs_CurveType.GeomAbs_Line => GeomType.Line,
        GeomAbs_CurveType.GeomAbs_Circle => GeomType.Circle,
        GeomAbs_CurveType.GeomAbs_Ellipse => GeomType.Ellipse,
        GeomAbs_CurveType.GeomAbs_Hyperbola => GeomType.Hyperbola,
        GeomAbs_CurveType.GeomAbs_Parabola => GeomType.Parabola,
        GeomAbs_CurveType.GeomAbs_BezierCurve => GeomType.Bezier,
        GeomAbs_CurveType.GeomAbs_BSplineCurve => GeomType.BSpline,
        GeomAbs_CurveType.GeomAbs_OffsetCurve => GeomType.Offset,
        _ => GeomType.Other,
    };

    /// <summary>The geometry type of a kind of surface.</summary>
    internal static GeomType GeomTypeOf(GeomAbs_SurfaceType type) => type switch
    {
        GeomAbs_SurfaceType.GeomAbs_Plane => GeomType.Plane,
        GeomAbs_SurfaceType.GeomAbs_Cylinder => GeomType.Cylinder,
        GeomAbs_SurfaceType.GeomAbs_Cone => GeomType.Cone,
        GeomAbs_SurfaceType.GeomAbs_Sphere => GeomType.Sphere,
        GeomAbs_SurfaceType.GeomAbs_Torus => GeomType.Torus,
        GeomAbs_SurfaceType.GeomAbs_BezierSurface => GeomType.Bezier,
        GeomAbs_SurfaceType.GeomAbs_BSplineSurface => GeomType.BSpline,
        GeomAbs_SurfaceType.GeomAbs_SurfaceOfRevolution => GeomType.Revolution,
        GeomAbs_SurfaceType.GeomAbs_SurfaceOfExtrusion => GeomType.Extrusion,
        GeomAbs_SurfaceType.GeomAbs_OffsetSurface => GeomType.Offset,
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
            return new Location(_wrapped.Location());
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

    /// <summary>Fills <paramref name="props"/> with the mass properties for the shape's kind; false for a vertex, which has none.</summary>
    internal static bool MassProperties(TopoDS_Shape shape, GProp_GProps props)
    {
        switch (TopoUtil.ShapeTypeOf(shape))
        {
            case TopAbs_ShapeEnum.TopAbs_VERTEX:
                return false;
            case TopAbs_ShapeEnum.TopAbs_EDGE or TopAbs_ShapeEnum.TopAbs_WIRE:
                BRepGProp.LinearProperties(shape, props);
                return true;
            case TopAbs_ShapeEnum.TopAbs_FACE or TopAbs_ShapeEnum.TopAbs_SHELL:
                BRepGProp.SurfaceProperties(shape, props);
                return true;
            default:
                BRepGProp.VolumeProperties(shape, props);
                return true;
        }
    }

    /// <summary>The measure OCCT computes for the shape's kind: length, area or volume (build123d's <c>compute_volume</c>).</summary>
    public double ComputeVolume()
    {
        if (IsEmpty || _wrapped is TopoDS_Vertex)
            return 0.0;
        using var props = new GProp_GProps();
        MassProperties(_wrapped, props);
        return props.Mass();
    }

    /// <summary>The inertia tensor of the shape about its centre of mass, as rows.</summary>
    public double[][] MatrixOfInertia
    {
        get
        {
            if (IsEmpty)
                return [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
            using var props = new GProp_GProps();
            BRepGProp.VolumeProperties(_wrapped, props);
            using var m = props.MatrixOfInertia();
            return [.. Enumerable.Range(1, 3).Select(i => Enumerable.Range(1, 3).Select(j => m.Value(i, j)).ToArray())];
        }
    }

    /// <summary>The principal axes of inertia, each with its moment.</summary>
    public IReadOnlyList<(Vector Axis, double Moment)> PrincipalProperties
    {
        get
        {
            NeedsGeometry("principal axes");
            using var props = new GProp_GProps();
            BRepGProp.VolumeProperties(_wrapped, props);
            using var principal = props.PrincipalProperties();
            double m1 = 0, m2 = 0, m3 = 0;
            principal.Moments(ref m1, ref m2, ref m3);
            using var a1 = principal.FirstAxisOfInertia();
            using var a2 = principal.SecondAxisOfInertia();
            using var a3 = principal.ThirdAxisOfInertia();
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
            using var props = new GProp_GProps();
            BRepGProp.VolumeProperties(_wrapped, props);
            double x = 0, y = 0, z = 0;
            props.StaticMoments(ref x, ref y, ref z);
            return (x, y, z);
        }
    }

    /// <summary>The radius of gyration of the shape about an axis.</summary>
    public double RadiusOfGyration(Axis axis)
    {
        NeedsGeometry("radius of gyration");
        using var props = new GProp_GProps();
        BRepGProp.VolumeProperties(_wrapped, props);
        using var ax1 = axis.ToGpAx1();
        return props.RadiusOfGyration(ax1);
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
            using var check = new BRepCheck_Analyzer(_wrapped);
            check.SetParallel(true);
            return check.IsValid();
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
                using var map = new TopTools_IndexedDataMapOfShapeListOfShape();
                TopExp.MapShapesAndAncestors(shape, TopAbs_ShapeEnum.TopAbs_EDGE, TopAbs_ShapeEnum.TopAbs_FACE, map);
                for (int i = 1; i <= map.Extent(); i++)
                {
                    using var edge = TopoDS.Edge(map.FindKey(i));
                    // A degenerate edge (a sphere's pole, a cone's apex) belongs to one face only
                    if (BRep_Tool.Degenerated(edge))
                        continue;
                    using var faces = map.FindFromIndex(i);
                    if (faces.Extent() != 2)
                        return false;
                }
                // In a closed shell each edge is used once in each direction
                using var orientation = new ShapeAnalysis_Shell();
                orientation.LoadShells(shape);
                orientation.CheckOrientedShells(shape, true, true);
                if (orientation.HasBadEdges())
                    return false;
            }
            return true;
        }
    }

    // ---- Bounding boxes ----

    /// <summary>A box aligned with the axes containing this shape (tight when <paramref name="optimal"/>).</summary>
    public BoundBox BoundingBox(double? tolerance = null, bool optimal = true) => Build123d.BoundBox.FromShape(this, tolerance, optimal);

    /// <summary>The smallest box in any orientation containing this shape.</summary>
    public OrientedBoundBox OrientedBoundingBox()
    {
        var obb = new Bnd_OBB();
        if (!IsEmpty)
            BRepBndLib.AddOBB(_wrapped, obb, true);
        return Build123d.OrientedBoundBox.Own(obb);
    }

    // ---- Distances ----

    /// <summary>The minimal distance between this shape and another.</summary>
    /// <exception cref="InvalidOperationException">Either shape is empty.</exception>
    public double Distance(Shape other)
    {
        if (IsEmpty || other.IsEmpty)
            throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
        using var calc = new BRepExtrema_DistShapeShape(_wrapped, other._wrapped);
        return calc.Value();
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
        using var pnt = point.ToGpPnt();
        using var maker = new BRepBuilderAPI_MakeVertex(pnt);
        using var vertex = maker.Vertex();
        return DistanceToTopo(vertex);
    }

    private (double, Vector, Vector) DistanceToTopo(TopoDS_Shape other)
    {
        using var calc = new BRepExtrema_DistShapeShape();
        calc.LoadS1(_wrapped);
        calc.LoadS2(other);
        calc.Perform();
        using var p1 = calc.PointOnShape1(1);
        using var p2 = calc.PointOnShape2(1);
        return (calc.Value(), new Vector(p1), new Vector(p2));
    }

    /// <summary>The minimal distances from this shape to each of <paramref name="others"/>.</summary>
    public IEnumerable<double> Distances(params IEnumerable<Shape> others)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
        using var calc = new BRepExtrema_DistShapeShape();
        calc.LoadS1(_wrapped);
        foreach (var other in others)
        {
            if (other.IsEmpty)
                throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
            calc.LoadS2(other._wrapped);
            calc.Perform();
            yield return calc.Value();
        }
    }

    /// <summary>The faces of this shape an axis passes through, nearest the axis's position first.</summary>
    public ShapeList<Face> FacesIntersectedByAxis(Axis axis, double tolerance = 1e-4)
    {
        if (IsEmpty)
            return [];
        using var ax1 = axis.ToGpAx1();
        using var makeLin = new gce_MakeLin(ax1);
        using var line = makeLin.Value();
        using var inter = new BRepIntCurveSurface_Inter();
        inter.Init(_wrapped, line, tolerance);
        using var origin = axis.Position.ToGpPnt();
        var found = new List<(TopoDS_Face Face, double Distance)>();
        for (; inter.More(); inter.Next())
        {
            using var point = inter.Pnt();
            found.Add((inter.Face(), Math.Abs(origin.SquareDistance(point))));
        }
        return new ShapeList<Face>(found.OrderBy(f => f.Distance).Select(f => (Face)CastLike(f.Face)));
    }

    // ---- Meshing ----

    /// <summary>
    /// A copy of the kernel shape carrying a triangulation within <paramref name="tolerance"/>. Meshing stores the
    /// triangulation in the faces it's given, so it meshes a copy: the shape itself, and everything sharing its
    /// geometry, is untouched.
    /// </summary>
    internal TopoDS_Shape MeshedCopy(double tolerance, double angularTolerance, bool relative = true)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot mesh an empty shape");
        var copy = TopoUtil.PrivateCopy(_wrapped);
        new BRepMesh_IncrementalMesh(copy, tolerance, relative, angularTolerance, true).Dispose();
        return copy;
    }

    /// <summary>A triangulated approximation: the vertices and the triangles as index triples into them.</summary>
    public (IReadOnlyList<Vector> Vertices, IReadOnlyList<(int, int, int)> Triangles) Tessellate(double tolerance, double angularTolerance = 0.1)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot tessellate an empty shape");
        var meshed = Rewrapped(MeshedCopy(tolerance, angularTolerance));
        var vertices = new List<Vector>();
        var triangles = new List<(int, int, int)>();
        int offset = 0;
        foreach (var face in meshed.Faces())
        {
            using var loc = new TopLoc_Location();
            using var poly = BRep_Tool.Triangulation(face.TopoFace, loc);
            if (poly is null)
                continue;
            using var trsf = loc.Transformation();
            bool reverse = face.TopoShape.Orientation() == TopAbs_Orientation.TopAbs_REVERSED;
            int nodes = poly.NbNodes();
            for (int i = 1; i <= nodes; i++)
            {
                using var node = poly.Node(i);
                using var placed = node.Transformed(trsf);
                vertices.Add(new Vector(placed.X(), placed.Y(), placed.Z()));
            }
            for (int i = 1; i <= poly.NbTriangles(); i++)
            {
                using var t = poly.Triangle(i);
                int a = t.Value(1) + offset - 1, b = t.Value(2) + offset - 1, c = t.Value(3) + offset - 1;
                triangles.Add(reverse ? (a, c, b) : (a, b, c));
            }
            offset += nodes;
        }
        return (vertices, triangles);
    }
}
