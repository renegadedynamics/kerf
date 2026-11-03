using Occt;

namespace Build123d;

/// <summary>
/// A zero-dimensional point in the topology: it marks the ends of edges and holds their coordinates.
/// </summary>
public sealed class Vertex : Shape
{
    /// <summary>The empty Vertex; <c>new Vertex(0, 0, 0)</c> is the origin.</summary>
    public Vertex() : base(null) { }

    /// <summary>A vertex wrapping an OCCT vertex.</summary>
    public Vertex(TopoDS_Vertex vertex) : base(vertex) { }

    /// <summary>A vertex at a point.</summary>
    public Vertex(double x, double y, double z = 0) : base(Make(new Vector(x, y, z))) { }

    /// <summary>A vertex at a point.</summary>
    public Vertex(Vector point) : base(Make(point)) { }

    private static TopoDS_Vertex Make(Vector point)
    {
        var pnt = point.ToGpPnt();
        var maker = new BRepBuilderAPI_MakeVertex(pnt);
        return maker.Vertex();
    }

    internal TopoDS_Vertex TopoVertex => TopoDS.Vertex(TopoShape);

    internal override int? Dim => 0;

    internal override double Order => 0.0;

    internal override Shape MakeZero() => new Vertex();

    /// <summary>The volume of a vertex, which is zero.</summary>
    public override double Volume => 0.0;

    private gp_Pnt Point
    {
        get
        {
            NeedsGeometry("position");
            return BRep_Tool.Pnt(TopoVertex);
        }
    }

    /// <summary>The X coordinate, including the vertex's Location.</summary>
    public double X { get { var p = Point; return p.X(); } }

    /// <summary>The Y coordinate, including the vertex's Location.</summary>
    public double Y { get { var p = Point; return p.Y(); } }

    /// <summary>The Z coordinate, including the vertex's Location.</summary>
    public double Z { get { var p = Point; return p.Z(); } }

    /// <summary>The coordinates of the vertex.</summary>
    public void Deconstruct(out double x, out double y, out double z)
    {
        var p = Point;
        (x, y, z) = (p.X(), p.Y(), p.Z());
    }

    /// <summary>The point the vertex is at.</summary>
    public Vector ToVector()
    {
        var p = Point;
        return new Vector(p);
    }

    /// <summary>The centre of a vertex is itself.</summary>
    public override Vector Center()
    {
        NeedsGeometry("center");
        return ToVector();
    }

    /// <summary>This vertex.</summary>
    public override ShapeList<Vertex> Vertices(Select select = Select.All) =>
        SelectFrom<Vertex>(IsEmpty ? [] : [this], select);

    /// <summary>A vertex at the sum of two vertices' coordinates.</summary>
    public static Vertex operator +(Vertex a, Vertex b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    /// <summary>A vertex moved by a vector.</summary>
    public static Vertex operator +(Vertex a, Vector b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    /// <summary>A vertex at the difference of two vertices' coordinates.</summary>
    public static Vertex operator -(Vertex a, Vertex b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    /// <summary>A vertex moved back by a vector.</summary>
    public static Vertex operator -(Vertex a, Vector b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    /// <summary>Vertices can't be intersected with <c>&amp;</c>.</summary>
    public override Shape CommonWith(params IEnumerable<object> others) =>
        throw new NotSupportedException("Vertices can't be intersected");

    internal override Shape TransformShapeCore(Matrix matrix) => new Vertex(matrix.Multiply(ToVector()));

    /// <inheritdoc/>
    public override string ToString() => IsEmpty ? "Vertex()" : $"Vertex({PyFormat.Repr(X)}, {PyFormat.Repr(Y)}, {PyFormat.Repr(Z)})";

    internal override ShapeList<Shape>? IntersectCore(object other, double tolerance, bool includeTouched)
    {
        switch (other)
        {
            case Vector v:
                other = new Vertex(v);
                break;
            case Location l:
                other = new Vertex(l.Position);
                break;
            case Axis axis:
                return axis.Intersect(Center()) is not null ? [this] : null;
            case Plane plane:
                return plane.Contains(Center(), tolerance) ? [this] : null;
        }
        if (other is Vertex vertex)
            return DistanceTo(vertex) <= tolerance ? [this] : null;
        // Delegate to the higher-dimensional shape (including a compound)
        return ((Shape)other).IntersectCore(this, tolerance, includeTouched);
    }

    // ---- Convexity ----

    /// <summary>
    /// How the shape this vertex was selected from sits around it.
    /// </summary>
    /// <remarks>
    /// <para>
    /// Selected through a face (<c>face.Vertices()</c>, or <c>face.Edges()[0].Vertices()</c>) the vertex is a corner of
    /// that face: Convex at the corner of a plate, Concave at the corner of a slot or the step in an L, Smooth where two
    /// edges meet in line. The corner is read in the face's own parameters, so a corner of a curved face classifies the
    /// same way as one of a flat face. Selected straight off a sketch or another shape of faces alone, the vertex is a
    /// corner of the one face that holds it.
    /// </para>
    /// <para>
    /// Selected straight off a solid or shell, the vertex is classified by the edges meeting there: Convex where every
    /// crease is convex (the corner of a box), Concave where every crease is concave (the bottom corner of a pocket), and
    /// Saddle where both kinds meet (the inner corner of a step). Smooth edges don't count.
    /// </para>
    /// </remarks>
    /// <exception cref="InvalidOperationException">
    /// The vertex has no <see cref="Shape.TopoParent"/>, or its face or edges can't be classified.
    /// </exception>
    public override Convexity Convexity
    {
        get
        {
            var face = OwningFace() ?? OnlyFace();
            return face is { } f ? CornerConvexity(f) : CreaseConvexity();
        }
    }

    /// <summary>The innermost face this vertex was selected through, if any.</summary>
    private TopoDS_Face? OwningFace()
    {
        for (int i = TopoPath.Count - 1; i >= 0; i--)
            if (!TopoPath[i].IsEmpty && TopoPath[i].TopoShape.ShapeType() == TopAbs_ShapeEnum.TopAbs_FACE)
                return TopoDS.Face(TopoPath[i].TopoShape);
        return null;
    }

    /// <summary>The one face of a sketch-like parent (no solids or shells) that holds this vertex.</summary>
    private TopoDS_Face? OnlyFace()
    {
        var parent = TopoParent;
        if (parent is null || parent.IsEmpty || parent.Solids().Count > 0 || parent.Shells().Count > 0)
            return null;
        var holding = new List<TopoDS_Face>();
        var faces = new TopExp_Explorer(parent.TopoShape, TopAbs_ShapeEnum.TopAbs_FACE);
        for (; faces.More(); faces.Next())
        {
            var face = TopoDS.Face(faces.Current());
            var vertices = new TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_VERTEX);
            for (; vertices.More(); vertices.Next())
                if (vertices.Current().IsSame(TopoShape))
                {
                    holding.Add(face);
                    break;
                }
        }
        if (holding.Count > 1)
            throw new InvalidOperationException(
                $"this vertex is a corner of {holding.Count} faces of the {parent.GetType().Name} it was selected from, "
                + "and classifies differently in each - select it through the face meant");
        return holding.Count == 1 ? holding[0] : null;
    }

    /// <summary>Unit directions leading away from this vertex in the face's (u, v) parameters.</summary>
    private List<(double U, double V)> FaceTangents(TopoDS_Face face)
    {
        var here = BRep_Tool.Parameters(TopoVertex, face);
        var seen = new List<TopoDS_Edge>();
        var directions = new List<(double, double)>();
        var explorer = new TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_EDGE);
        for (; explorer.More(); explorer.Next())
        {
            var edge = TopoDS.Edge(explorer.Current());
            if (seen.Any(e => edge.IsSame(e)))
                continue;   // a seam edge is met once per side
            seen.Add(edge);
            var curve = new BRepAdaptor_Curve2d(edge, face);
            foreach (var (param, sign) in new[] { (curve.FirstParameter(), 1.0), (curve.LastParameter(), -1.0) })
            {
                var end = curve.Value(param);
                if (Math.Pow(end.X() - here.X(), 2) + Math.Pow(end.Y() - here.Y(), 2) > Constants.Tolerance)
                    continue;
                var point = new gp_Pnt2d();
                var tangent = new gp_Vec2d();
                curve.D1(param, out point, out tangent);
                var length = Math.Sqrt(tangent.X() * tangent.X() + tangent.Y() * tangent.Y());
                if (length > 0)
                    directions.Add((sign * tangent.X() / length, sign * tangent.Y() / length));
            }
        }
        return directions;
    }

    /// <summary>
    /// Which way the face's boundary turns at this corner. The two edges leaving the vertex bound a wedge of less than
    /// half a turn, and their bisector points into it: material there and the corner is convex, else concave.
    /// </summary>
    private Convexity CornerConvexity(TopoDS_Face face)
    {
        var directions = FaceTangents(face);
        if (directions.Count != 2)
            throw new InvalidOperationException(
                $"{directions.Count} edge end(s) meet this vertex on the face, expected 2 - a corner is where exactly two of them do");
        var ((u1, v1), (u2, v2)) = (directions[0], directions[1]);
        var bisector = (U: u1 + u2, V: v1 + v2);
        var span = Math.Sqrt(bisector.U * bisector.U + bisector.V * bisector.V);
        if (span < Constants.Tolerance)
            return Convexity.Smooth;   // the boundary runs straight through
        double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
        BRepTools.UVBounds(face, out uMin, out uMax, out vMin, out vMax);
        var step = 1e-4 * Math.Sqrt(Math.Pow(uMax - uMin, 2) + Math.Pow(vMax - vMin, 2)) / span;
        var here = BRep_Tool.Parameters(TopoVertex, face);
        var probe = new gp_Pnt2d(here.X() + bisector.U * step, here.Y() + bisector.V * step);
        var classifier = new IntTools_FClass2d(face, Constants.Tolerance);
        return classifier.Perform(probe) == TopAbs_State.TopAbs_IN ? Convexity.Convex : Convexity.Concave;
    }

    /// <summary>Classifies by the creases meeting at this vertex of a solid or shell.</summary>
    private Convexity CreaseConvexity()
    {
        var parent = TopoParent;
        if (parent is null || parent.IsEmpty)
            throw new InvalidOperationException(
                "this vertex was not selected from a shape, so there is nothing to classify it against - take it from a face or solid");
        var map = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(parent.TopoShape, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, map);
        var own = TopoUtil.FindSame(TopoShape, Enumerable.Range(1, map.Extent()).Select(map.FindKey))
                  ?? throw new InvalidOperationException("this vertex is not part of its topo_parent");
        var kinds = new HashSet<Convexity>();
        var edges = map.FindFromKey(own);
        foreach (var topoEdge in edges)
            kinds.Add(((Edge)Cast(topoEdge).WithTopoPath(TopoPath)).Convexity);
        kinds.Remove(Convexity.Smooth);
        return kinds.Count switch
        {
            0 => Convexity.Smooth,
            1 => kinds.Single(),
            _ => Convexity.Saddle,
        };
    }

    /// <summary>The vertex the two edges share, or the empty Vertex if they share none.</summary>
    public static Vertex CommonVertex(Edge edge1, Edge edge2) => CommonVertex(edge1.TopoEdge, edge2.TopoEdge);

    internal static Vertex CommonVertex(TopoDS_Edge edge1, TopoDS_Edge edge2)
    {
        var first = new TopExp_Explorer(edge1, TopAbs_ShapeEnum.TopAbs_VERTEX);
        for (; first.More(); first.Next())
        {
            var vertex1 = first.Current();
            var second = new TopExp_Explorer(edge2, TopAbs_ShapeEnum.TopAbs_VERTEX);
            for (; second.More(); second.Next())
                if (vertex1.IsSame(second.Current()))
                    return new Vertex(TopoDS.Vertex(vertex1));
        }
        return new Vertex();
    }
}
