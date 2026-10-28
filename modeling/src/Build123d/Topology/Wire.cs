using Build123d.Occt;

namespace Build123d;

/// <summary>
/// A sequence of connected edges forming a continuous path: the boundary of a face, or a path to sweep along.
/// </summary>
public class Wire : Shape1D
{
    /// <summary>The empty Wire.</summary>
    public Wire() : base(null) { }

    /// <summary>A wire wrapping an OCCT wire.</summary>
    public Wire(TopoDS_Wire wire, string label = "", Color? color = null) : base(wire, label, color) { }

    /// <summary>A copy of a wire (sharing its geometry).</summary>
    public Wire(Wire wire, string label = "", Color? color = null) : base(wire.TopoWire, label, color) { }

    /// <summary>A wire of one edge.</summary>
    public Wire(Edge edge, string label = "", Color? color = null) : base(MakeWire([edge]), label, color) { }

    /// <summary>A wire of a curve's edges.</summary>
    public Wire(Curve curve, string label = "", Color? color = null) : base(MakeWire(curve.Edges()), label, color) { }

    /// <summary>
    /// A wire of edges. With <paramref name="sequenced"/> the edges are first put in order, the end of each at the start
    /// of the next (which is slow).
    /// </summary>
    public Wire(IEnumerable<Edge> edges, bool sequenced = false, string label = "", Color? color = null)
        : base(MakeWireOrNull(edges, sequenced), label, color) { }

    internal TopoDS_Wire TopoWire => (TopoDS_Wire)TopoShape;

    internal override double Order => 1.5;

    internal override Shape MakeZero() => new Wire();

    private static TopoDS_Wire? MakeWireOrNull(IEnumerable<Edge> edges, bool sequenced)
    {
        var list = edges.ToList();
        return list.Count == 0 ? null : MakeWire(list, sequenced);
    }

    /// <summary>An OCCT wire of edges, put in order first with <paramref name="sequenced"/>.</summary>
    /// <exception cref="ArgumentException">The edges are disconnected.</exception>
    /// <exception cref="InvalidOperationException">The wire is empty.</exception>
    internal static TopoDS_Wire MakeWire(IEnumerable<Edge> edges, bool sequenced = false)
    {
        var list = edges.ToList();
        if (sequenced && list.Count > 0)
        {
            var placed = new List<Edge> { list[0] };
            var unplaced = list.Skip(1).ToList();
            while (unplaced.Count > 0)
            {
                var target = new Wire(placed).PositionAt(1);
                var next = unplaced.OrderBy(e => Math.Min((target - e.PositionAt(0)).Length, (target - e.PositionAt(1)).Length)).First();
                unplaced.Remove(next);
                placed.Add(next);
            }
            list = placed;
        }
        using var builder = new BRepBuilderAPI_MakeWire();
        using var combined = TopoUtil.ListOf(list.Where(e => !e.IsEmpty).Select(e => e.TopoShape));
        builder.Add(combined);
        builder.Build();
        if (!builder.IsDone())
        {
            switch (builder.Error())
            {
                case BRepBuilderAPI_WireError.BRepBuilderAPI_NonManifoldWire:
                    Warnings.Warn("Wire is non manifold (e.g. branching, self intersecting)");
                    break;
                case BRepBuilderAPI_WireError.BRepBuilderAPI_EmptyWire:
                    throw new InvalidOperationException("Wire is empty");
                case BRepBuilderAPI_WireError.BRepBuilderAPI_DisconnectedWire:
                    throw new ArgumentException("Edges are disconnected");
            }
        }
        return builder.Wire();
    }

    // ---- Factories ----

    /// <summary>The wires made by connecting the edges of the given wires and edges.</summary>
    public static ShapeList<Wire> Combine(IEnumerable<Shape1D> wires, double tolerance = 1e-9) =>
        OneD.EdgesToWires(wires.SelectMany(w => w.Edges()), tolerance);

    /// <summary>A circle centred on the plane's origin (default XY).</summary>
    public static Wire MakeCircle(double radius, Plane? plane = null) => new([Edge.MakeCircle(radius, plane)]);

    /// <summary>An ellipse centred on the plane's origin (default XY); an arc is closed with a line unless <paramref name="closed"/> is false.</summary>
    public static Wire MakeEllipse(double xRadius, double yRadius, Plane? plane = null, double startAngle = 360.0, double endAngle = 360.0,
                                   AngularDirection angularDirection = AngularDirection.CounterClockwise, bool closed = true)
    {
        var ellipse = Edge.MakeEllipse(xRadius, yRadius, plane, startAngle, endAngle, angularDirection);
        if (startAngle != endAngle && closed)
            return new Wire([ellipse, Edge.MakeLine(ellipse.EndPoint(), ellipse.StartPoint())]);
        return new Wire([ellipse]);
    }

    /// <summary>A polygon through points, closed unless <paramref name="close"/> is false.</summary>
    public static Wire MakePolygon(IEnumerable<Vector> vertices, bool close = true)
    {
        var vectors = vertices.ToList();
        if ((vectors[0] - vectors[^1]).Length > Constants.Tolerance && close)
            vectors.Add(vectors[0]);
        using var builder = new BRepBuilderAPI_MakePolygon();
        foreach (var v in vectors)
        {
            using var p = v.ToGpPnt();
            builder.Add(p);
        }
        return new Wire(builder.Wire());
    }

    /// <summary>A rectangle centred on the plane's origin (default XY).</summary>
    public static Wire MakeRect(double width, double height, Plane? plane = null)
    {
        plane ??= Plane.XY;
        (double, double)[] corners = [(width / 2, height / 2), (width / 2, height / -2), (width / -2, height / -2), (width / -2, height / 2)];
        return MakePolygon(corners.Select(c => plane.FromLocalCoords(new Vector(c.Item1, c.Item2))), close: true);
    }

    /// <summary>
    /// The shortest wire enclosing the edges (which mustn't overlap), in the XY plane: their convex hull, built of
    /// trimmed pieces of the edges and lines joining them.
    /// </summary>
    /// <param name="edges">The edges to enclose.</param>
    /// <param name="tolerance">The allowable error, as a fraction of each edge's length.</param>
    public static Wire MakeConvexHull(IEnumerable<Edge> edges, double tolerance = 1e-3)
    {
        // A cloud of points along the edges, its convex hull as pairs of point indices, and from those the lines
        // joining the edges and the pieces of each edge on the hull
        var edgeList = edges.ToList();
        int fragments = (int)(2 / tolerance);
        var lookup = new Dictionary<int, (int Edge, double Param)>();
        var points = new List<(double X, double Y)>();
        for (int e = 0; e < edgeList.Count; e++)
            for (int i = 0; i < fragments; i++)
            {
                var param = i / (double)(fragments - 1);
                var p = edgeList[e].PositionAt(param);
                points.Add((p.X, p.Y));
                lookup[e * fragments + i] = (e, param);
            }

        var connecting = new List<((int Edge, double Param), (int Edge, double Param))>();
        var trimPoints = new Dictionary<int, List<int>>();
        void AddTrim(int edge, params int[] indices)
        {
            if (!trimPoints.TryGetValue(edge, out var list))
                trimPoints[edge] = list = [];
            list.AddRange(indices);
        }
        foreach (var (s0, s1) in ConvexHull2D.Simplices(points))
        {
            int edge0 = lookup[s0].Edge, edge1 = lookup[s1].Edge;
            if (edge0 != edge1)
            {
                // A line joining two edges
                AddTrim(edge0, s0);
                AddTrim(edge1, s1);
                connecting.Add((lookup[s0], lookup[s1]));
            }
            else if (Math.Abs(s0 - s1) != 1)
            {
                // A line across one edge
                int start = Math.Min(s0, s1), end = Math.Max(s0, s1);
                AddTrim(edge0, start, end);
                connecting.Add((lookup[start], lookup[end]));
            }
        }
        var trimmedEdges = new List<Edge>();
        foreach (var (edgeIndex, indices) in trimPoints)
        {
            var sorted = indices.Order().ToList();
            for (int i = 0; i + 1 < sorted.Count; i += 2)
                if (sorted[i] != sorted[i + 1])
                    trimmedEdges.Add(edgeList[edgeIndex].Trim(lookup[sorted[i]].Param, lookup[sorted[i + 1]].Param));
        }
        var connectingEdges = connecting.Select(c =>
            Edge.MakeLine(edgeList[c.Item1.Edge].PositionAt(c.Item1.Param), edgeList[c.Item2.Edge].PositionAt(c.Item2.Param)));
        return new Wire([.. connectingEdges, .. trimmedEdges], sequenced: true);
    }

    /// <summary>A wire extruded: not possible.</summary>
    public static Wire Extrude(Shape obj, Vector direction) => throw new ArgumentException("Wires can't be created by extrusion");

    // ---- Edges ----

    /// <summary>
    /// The edges in the order they connect (as <c>BRepTools_WireExplorer</c> walks them), then any edges at branches of a
    /// non-manifold wire, which the explorer skips.
    /// </summary>
    public override ShapeList<Edge> Edges(Select select = Select.All)
    {
        if (IsEmpty)
            return [];
        var edges = new List<Edge>();
        using var explored = new TopTools_MapOfShape();
        using (var explorer = new BRepTools_WireExplorer(TopoWire))
            for (; explorer.More(); explorer.Next())
            {
                var edge = explorer.Current();
                explored.Add(edge);
                edges.Add((Edge)new Edge(edge).ExtractedFrom(this));
            }
        using (var all = new TopExp_Explorer(TopoShape, TopAbs_ShapeEnum.TopAbs_EDGE))
            for (; all.More(); all.Next())
            {
                var edge = all.Current();
                if (explored.Add(edge))
                    edges.Add((Edge)new Edge(TopoDS.Edge(edge)).ExtractedFrom(this));
            }
        return SelectFrom(new ShapeList<Edge>(edges), select);
    }

    /// <summary>The edges sorted along the wire, each turned to run the way the wire does.</summary>
    public ShapeList<Edge> OrderEdges()
    {
        // Sorting by a reversed wire runs against the orientation of its edges
        var sorted = Edges().SortBy(this, reverse: !IsForward);
        var ordered = new List<Edge> { sorted[0] };
        foreach (var edge in sorted.Skip(1))
            ordered.Add((ordered[^1].PositionAt(1) - edge.PositionAt(0)).Length < Constants.Tolerance ? edge : edge.Reversed());
        return new ShapeList<Edge>(ordered);
    }

    // ---- Curve parameters ----

    /// <inheritdoc/>
    public override BRepAdaptor_CompCurve GeomAdaptor()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't get geom adaptor of empty wire");
        return new BRepAdaptor_CompCurve(TopoWire);
    }

    /// <summary>The parameter of the wire's <c>BRepAdaptor_CompCurve</c> at a position (not an edge parameter).</summary>
    public override double ParamAt(double position)
    {
        using var curve = GeomAdaptor();
        var target = PositionAt(position);
        using var pnt = target.ToGpPnt();
        using var extrema = new Extrema_ExtPC(pnt, curve);
        if (!extrema.IsDone() || extrema.NbExt() == 0)
            throw new InvalidOperationException("Failed to find point on curve");
        double minDist = double.PositiveInfinity;
        Vector? closest = null;
        double closestParam = 0;
        for (int i = 1; i <= extrema.NbExt(); i++)
        {
            var dist = extrema.SquareDistance(i);
            if (dist < minDist)
            {
                minDist = dist;
                using var point = extrema.Point(i);
                using var value = point.Value();
                closest = new Vector(value);
                closestParam = point.Parameter();
            }
        }
        if (closest is null || (closest.Value - target).Length > Constants.Tolerance)
            throw new InvalidOperationException("Failed to find point on curve");
        return closestParam;
    }

    /// <inheritdoc/>
    /// <exception cref="ArgumentException">The point isn't on the wire.</exception>
    public override double ParamAtPoint(Vector point, bool normalize = true)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find point on empty wire");
        var vertex = new Vertex(point);
        var separation = DistanceTo(point);
        if (!TopologyUtils.IsCloseB(separation, 0, absTol: Constants.Tolerance))
            throw new ArgumentException($"point ({point}) is {separation} from wire");
        using var extrema = new BRepExtrema_DistShapeShape(vertex.TopoShape, TopoShape);
        extrema.Perform();
        if (!extrema.IsDone() || extrema.NbSolution() == 0)
            throw new ArgumentException("point is not on Wire");

        TopoDS_Edge closestEdge;
        double closestParam;
        switch (extrema.SupportTypeShape2(1))
        {
            case BRepExtrema_SupportType.BRepExtrema_IsOnEdge:
                closestEdge = TopoDS.Edge(extrema.SupportOnShape2(1));
                closestParam = 0;
                extrema.ParOnEdgeS2(1, ref closestParam);
                break;
            case BRepExtrema_SupportType.BRepExtrema_IsVertex:
            {
                using var hit = TopoDS.Vertex(extrema.SupportOnShape2(1));
                using var map = new TopTools_IndexedDataMapOfShapeListOfShape();
                TopExp.MapShapesAndAncestors(TopoShape, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, map);
                using var edges = map.FindFromKey(hit);
                closestEdge = TopoDS.Edge(edges.First());
                closestParam = BRep_Tool.Parameter(hit, closestEdge);
                break;
            }
            default:
                throw new ArgumentException("point is not on Wire");
        }

        double distanceAlongWire;
        using (var adaptor = new BRepAdaptor_Curve(closestEdge))
        {
            double paramMin = 0, paramMax = 0;
            BRep_Tool.Range(closestEdge, ref paramMin, ref paramMax);
            if (adaptor.IsPeriodic())
                closestParam = PyMath.Mod(closestParam - paramMin, adaptor.Period()) + paramMin;
            distanceAlongWire = closestEdge.Orientation() == TopAbs_Orientation.TopAbs_FORWARD
                ? GCPnts_AbscissaPoint.Length(adaptor, paramMin, closestParam)
                : GCPnts_AbscissaPoint.Length(adaptor, closestParam, paramMax);
        }
        // The lengths of the edges before the closest one
        using (var explorer = new BRepTools_WireExplorer(TopoWire))
            for (; explorer.More(); explorer.Next())
            {
                var edge = explorer.Current();
                if (BRep_Tool.Degenerated(edge))
                    continue;
                if (edge.IsEqual(closestEdge))
                    break;
                using var adaptor = new BRepAdaptor_Curve(edge);
                distanceAlongWire += GCPnts_AbscissaPoint.Length(adaptor);
            }
        // The explorer follows the wire's orientation while positions measure a reversed wire from its other end
        var u = distanceAlongWire / Length;
        var normalized = IsForward ? u : 1 - u;
        return normalize ? normalized : ParamAt(normalized);
    }

    internal override (BRepAdaptor_Curve Curve, double Param, bool Forward) OcctParamAt(double position, PositionMode mode = PositionMode.Parameter)
    {
        // The absolute distance along the wire
        double distance;
        if (mode == PositionMode.Parameter)
            distance = (IsForward ? position : 1.0 - position) * Length;
        else
            distance = IsForward ? position : Length - position;

        var edges = Edges();
        var lengths = edges.Select(e => e.Length).ToList();
        var cumulative = new List<double>();
        double total = 0.0;
        foreach (var l in lengths)
            cumulative.Add(total += l);

        int index;
        double local;
        if (distance <= 0.0)
            (index, local) = (0, 0.0);
        else if (distance >= total)
            (index, local) = (edges.Count - 1, lengths[^1]);
        else
        {
            // bisect_right
            index = cumulative.FindIndex(c => c > distance);
            if (index < 0)
                index = cumulative.Count;
            local = distance - (index > 0 ? cumulative[index - 1] : 0.0);
        }
        var target = edges[index];
        var fraction = target.Length == 0 ? 0.0 : local / target.Length;
        // The fraction is measured the way the edge is traversed, which ParamAt honours for a reversed edge
        var param = target.ParamAt(fraction);
        return (target.GeomAdaptor(), param, target.IsForward);
    }

    // ---- Edits ----

    /// <summary>This wire if closed, else this wire closed with a line from its end to its start.</summary>
    public Wire Close()
    {
        if (IsClosed)
            return this;
        return Combine([this, Edge.MakeLine(EndPoint(), StartPoint())])[0];
    }

    /// <summary>The two edges at a chamfered corner, the reference edge first if there is one.</summary>
    public static (Edge, Edge) OrderChamferEdges(Edge? referenceEdge, (Edge, Edge) edges)
    {
        if (referenceEdge is null)
            return edges;
        var (edge1, edge2) = edges;
        if (edge1 == referenceEdge)
            return (edge1, edge2);
        if (edge2 == referenceEdge)
            return (edge2, edge1);
        throw new ArgumentException("reference edge not in edges");
    }

    /// <summary>
    /// This planar wire with the given corners chamfered: <paramref name="distance"/> along the first edge at each corner
    /// (along <paramref name="edge"/> if given) and <paramref name="distance2"/> along the other.
    /// </summary>
    public Wire Chamfer2D(double distance, double distance2, IEnumerable<Vertex> vertices, Edge? edge = null)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't chamfer empty wire");
        using var unchamfered = TopoUtil.MakeFaceFromWires(TopoWire);
        using var builder = new BRepFilletAPI_MakeFillet2d(unchamfered);
        using var map = new TopTools_IndexedDataMapOfShapeListOfShape();
        TopExp.MapShapesAndAncestors(unchamfered, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, map);
        foreach (var v in vertices)
        {
            if (v.IsEmpty)
                continue;
            using var list = map.FindFromKey(v.TopoShape);
            var pair = (new Edge(TopoDS.Edge(list.First())), new Edge(TopoDS.Edge(list.Last())));
            var (edge1, edge2) = OrderChamferEdges(edge, pair);
            if (!edge1.IsEmpty && !edge2.IsEmpty)
                builder.AddChamfer(edge1.TopoEdge, edge2.TopoEdge, distance, distance2);
        }
        builder.Build();
        using var chamfered = builder.Shape();
        using var fix = new ShapeFix_Shape(chamfered);
        fix.Perform();
        var fixedFace = TopoUtil.Downcast(fix.Shape()) as TopoDS_Face
                        ?? throw new InvalidOperationException("An internal error occured creating the chamfer");
        var result = new Wire(BRepTools.OuterWire(fixedFace));
        return (Wire)result.MadeBy(ShapeHistory.FromAlgorithm(builder, [TopoShape], result.TopoShape));
    }

    /// <summary>This wire with its very short edges (below <paramref name="precision"/>) and gaps fixed.</summary>
    public Wire FixDegenerateEdges(double precision)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't fix an empty edge");
        using var fix = new ShapeFix_Wireframe(TopoShape);
        fix.SetPrecision(precision);
        fix.SetMaxTolerance(1e-6);
        fix.FixSmallEdges();
        fix.FixWireGaps();
        return new Wire((TopoDS_Wire)TopoUtil.Downcast(fix.Shape()));
    }

    /// <summary>This wire joined to another.</summary>
    public Wire Stitch(Wire other)
    {
        if (IsEmpty || other.IsEmpty)
            throw new InvalidOperationException("Can't stitch empty wires");
        using var builder = new BRepBuilderAPI_MakeWire();
        builder.Add(TopoWire);
        builder.Add(other.TopoWire);
        builder.Build();
        return (Wire)CastLike(builder.Wire());
    }

    /// <summary>
    /// This wire as one B-spline edge, its edges concatenated in order. The junctions become knots rather than vertices,
    /// which can surprise later operations: for internal use.
    /// </summary>
    internal Edge ToBSpline()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't convert an empty wire");
        using var builder = new GeomConvert_CompCurveToBSplineCurve();
        using (var explorer = new BRepTools_WireExplorer(TopoWire))
            for (; explorer.More(); explorer.Next())
            {
                var edge = explorer.Current();
                if (BRep_Tool.Degenerated(edge))
                    continue;
                double first = 0, last = 0;
                BRep_Tool.Range(edge, ref first, ref last);
                double f = 0, l = 0;
                using var curve = BRep_Tool.Curve(edge, ref f, ref l);
                using var trimmed = new Geom_TrimmedCurve(curve, first, last);
                if (!builder.Add(trimmed, Constants.Tolerance))
                    throw new InvalidOperationException("Failed to build bspline.");
            }
        using var spline = builder.BSplineCurve();
        using var maker = new BRepBuilderAPI_MakeEdge(spline);
        if (!maker.IsDone())
            throw new InvalidOperationException("Failed to build bspline.");
        return new Edge(maker.Edge());
    }

    /// <summary>A new wire of the section between two positions (normalised parameters or points on the wire).</summary>
    public Wire Trim(CurvePosition start, CurvePosition end)
    {
        var startU = start.ToParam(this);
        var endU = end.ToParam(this);
        (startU, endU) = (Math.Min(startU, endU), Math.Max(startU, endU));
        var ordered = Edges().SortBy(this);
        if (!IsForward)
            // The edges of a reversed wire run against its positions, so turn each around
            ordered = new ShapeList<Edge>(ordered.Select(e => e.Reversed()));
        if (ordered.Count == 1)
            return new Wire([ordered[0].Trim(startU, endU)]);

        var total = Length;
        var startLen = startU * total;
        var endLen = endU * total;
        var trimmed = new List<Edge>();
        double current = 0.0;
        foreach (var edge in ordered)
        {
            var edgeLen = edge.Length;
            var edgeStart = current;
            var edgeEnd = current + edgeLen;
            current = edgeEnd;
            if (edgeEnd <= startLen || edgeStart >= endLen)
                continue;
            if (edgeStart >= startLen && edgeEnd <= endLen)
                trimmed.Add(edge);   // the whole edge
            else
            {
                var u0 = (Math.Max(startLen, edgeStart) - edgeStart) / edgeLen;
                var u1 = (Math.Min(endLen, edgeEnd) - edgeStart) / edgeLen;
                if (Math.Abs(u1 - u0) > Constants.Tolerance)
                    trimmed.Add(edge.Trim(u0, u1));
            }
        }
        return new Wire(trimmed);
    }

    /// <summary>
    /// Whether two wires have the same geometry, comparing their edges in order: two wires made independently along the
    /// same curves are equal.
    /// </summary>
    public bool GeomEqual(Wire other, double tolerance = 1e-6, int numInterpolationPoints = 5)
    {
        var edges1 = OrderEdges();
        var edges2 = other.OrderEdges();
        return edges1.Count == edges2.Count
               && edges1.Zip(edges2).All(p => p.First.GeomEqual(p.Second, tolerance, numInterpolationPoints));
    }

    /// <summary>
    /// The wire projected onto a shape along <paramref name="direction"/>, or from a <paramref name="center"/>; one of the two
    /// must be given. The projected wires run the same way as this one, nearest first.
    /// </summary>
    public ShapeList<Wire> ProjectToShape(Shape targetObject, Vector? direction = null, Vector? center = null)
    {
        if (IsEmpty || targetObject.IsEmpty)
            throw new InvalidOperationException("Can't project empty Wires or to empty Shapes");
        Vector? directionVector;
        Vector centerPoint;
        if (direction is { } d && center is null)
            (directionVector, centerPoint) = (d.Normalized(), new Vector(0, 0, 0));
        else if (center is { } c && direction is null)
            (directionVector, centerPoint) = (null, c);
        else
            throw new ArgumentException("Provide exactly one of direction or center");

        BRepProj_Projection projection;
        if (directionVector is { } dv)
        {
            using var dir = dv.ToGpDir();
            projection = new BRepProj_Projection(TopoShape, targetObject.TopoShape, dir);
        }
        else
        {
            using var pnt = centerPoint.ToGpPnt();
            projection = new BRepProj_Projection(TopoShape, targetObject.TopoShape, pnt);
        }
        var output = new List<Wire>();
        using (projection)
        {
            var orientation = TopoShape.Orientation();
            for (; projection.More(); projection.Next())
            {
                var projected = projection.Current();
                output.Add(orientation == projected.Orientation()
                    ? new Wire(projected)
                    : new Wire(TopoDS.Wire(projected.Reversed())));
            }
        }
        // BRepProj_Projection is inconsistent in the order it returns the wires: sort them by distance from this wire
        if (output.Count > 1)
        {
            var planarCenter = Center();
            var withDistance = new List<(Wire Wire, double Distance)>();
            foreach (var w in output)
            {
                var wireCenter = w.Center();
                if (directionVector is { } dv2)
                {
                    if ((wireCenter - planarCenter).Normalized().Dot(dv2) >= 0)
                        withDistance.Add((w, (wireCenter - planarCenter).Length));
                }
                else
                    withDistance.Add((w, (wireCenter - centerPoint).Length));
            }
            output = withDistance.OrderBy(x => x.Distance).Select(x => x.Wire).ToList();
        }
        // Clean the wires: projection can split edges artificially
        return new ShapeList<Wire>(output.Select(w => w.Clean()));
    }

    /// <summary>The 2D fillet of a wire's corners isn't ported yet: it needs the constrained-arc solvers (phase 6).</summary>
    public Wire Fillet2D(double radius, IEnumerable<Vertex> vertices) =>
        throw new NotImplementedException("Wire.Fillet2D needs the constrained-arc solvers, which come with phase 6");
}

/// <summary>Computes the edges of a 2D convex hull, as <c>scipy.spatial.ConvexHull(points).simplices</c> does.</summary>
internal static class ConvexHull2D
{
    /// <summary>
    /// The hull's edges as pairs of point indices (Andrew's monotone chain). Collinear points are left out, and of points
    /// closer than Qhull's precision only the first counts, as with Qhull.
    /// </summary>
    internal static List<(int, int)> Simplices(IReadOnlyList<(double X, double Y)> points)
    {
        var scale = points.Count == 0 ? 1 : points.Max(p => Math.Max(Math.Abs(p.X), Math.Abs(p.Y)));
        var merge = 1e-13 * Math.Max(1, scale);
        var kept = new List<int>();
        var cells = new Dictionary<(long, long), List<int>>();
        for (int i = 0; i < points.Count; i++)
        {
            var key = ((long)Math.Floor(points[i].X / merge), (long)Math.Floor(points[i].Y / merge));
            bool duplicate = false;
            for (long dx = -1; dx <= 1 && !duplicate; dx++)
                for (long dy = -1; dy <= 1 && !duplicate; dy++)
                    if (cells.TryGetValue((key.Item1 + dx, key.Item2 + dy), out var near))
                        duplicate = near.Any(j => Math.Abs(points[j].X - points[i].X) <= merge && Math.Abs(points[j].Y - points[i].Y) <= merge);
            if (duplicate)
                continue;
            if (!cells.TryGetValue(key, out var cell))
                cells[key] = cell = [];
            cell.Add(i);
            kept.Add(i);
        }
        var order = kept.OrderBy(i => points[i].X).ThenBy(i => points[i].Y).ToList();
        double Cross(int o, int a, int b) =>
            (points[a].X - points[o].X) * (points[b].Y - points[o].Y) - (points[a].Y - points[o].Y) * (points[b].X - points[o].X);
        var hull = new List<int>();
        foreach (var pass in new[] { order, Enumerable.Reverse(order).ToList() })
        {
            int start = hull.Count;
            foreach (var i in pass)
            {
                while (hull.Count >= start + 2 && Cross(hull[^2], hull[^1], i) <= 0)
                    hull.RemoveAt(hull.Count - 1);
                hull.Add(i);
            }
            hull.RemoveAt(hull.Count - 1);
        }
        var simplices = new List<(int, int)>();
        for (int i = 0; i < hull.Count; i++)
            simplices.Add((hull[i], hull[(i + 1) % hull.Count]));
        return simplices;
    }
}
