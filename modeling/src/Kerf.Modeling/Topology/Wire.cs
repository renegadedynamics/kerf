using Kerf.Topology;
using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Geometry;

namespace Kerf.Modeling;

/// <summary>
/// A sequence of connected edges forming a continuous path: the boundary of a face, or a path to sweep along.
/// </summary>
public partial class Wire : Shape1D
{
    /// <summary>The empty Wire.</summary>
    public Wire() : base(null) { }

    /// <summary>A wire wrapping an OCCT wire.</summary>
    public Wire(global::Kerf.Topology.Wire wire, string label = "", Color? color = null) : base(wire, label, color) { }

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

    internal global::Kerf.Topology.Wire TopoWire => (global::Kerf.Topology.Wire)TopoShape;

    internal override double Order => 1.5;

    internal override Shape MakeZero() => new Wire();

    private static global::Kerf.Topology.Wire? MakeWireOrNull(IEnumerable<Edge> edges, bool sequenced)
    {
        var list = edges.ToList();
        return list.Count == 0 ? null : MakeWire(list, sequenced);
    }

    /// <summary>An OCCT wire of edges, put in order first with <paramref name="sequenced"/>.</summary>
    /// <exception cref="ArgumentException">The edges are disconnected.</exception>
    /// <exception cref="InvalidOperationException">The wire is empty.</exception>
    internal static global::Kerf.Topology.Wire MakeWire(IEnumerable<Edge> edges, bool sequenced = false)
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
        // The edges in any order, connected where they meet (BRepBuilderAPI_MakeWire given the list)
        var made = global::Kerf.Topology.Wire.Connect(list.Where(e => !e.IsEmpty).Select(e => e.TopoEdge));
        if (!made.TryGetValue(out var wire))
        {
            switch (made.Error!.Code.Value)
            {
                case "construction.non-manifold-wire":
                    Warnings.Warn("Wire is non manifold (e.g. branching, self intersecting)");
                    break;
                case "construction.empty-wire":
                    throw new InvalidOperationException("Wire is empty");
                case "construction.disconnected-wire":
                    throw new ArgumentException("Edges are disconnected");
            }
            Failures.Get(made, () => "wire");
        }
        return wire;
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
        var points = vectors.Select(v => v.ToGpPnt()).ToArray();
        return new Wire(Failures.Get(global::Kerf.Topology.Wire.Polygon(points), () => "polygon"));
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
        var explored = new HashSet<global::Kerf.Topology.Shape>(ShapeComparer.Same);
        foreach (var edge in TopoWire.OrderedEdges())
        {
            explored.Add(edge);
            edges.Add((Edge)new Edge(edge).ExtractedFrom(this));
        }
        foreach (var edge in TopoShape.Explore<global::Kerf.Topology.Edge>())
        {
            if (explored.Add(edge))
                edges.Add((Edge)new Edge(edge).ExtractedFrom(this));
        }
        return SelectFrom(new ShapeList<Edge>(edges), select);
    }

    /// <summary>The edges sorted along the wire, each turned to run the way the wire does.</summary>
    public ShapeList<Edge> OrderEdges()
    {
        // Sorting by a reversed wire runs against the orientation of its edges
        var sorted = (IsForward ? Edges().OrderBy(this) : Edges().OrderByDescending(this));
        var ordered = new List<Edge> { sorted[0] };
        foreach (var edge in sorted.Skip(1))
            ordered.Add((ordered[^1].PositionAt(1) - edge.PositionAt(0)).Length < Constants.Tolerance ? edge : edge.Reversed());
        return new ShapeList<Edge>(ordered);
    }

    // ---- Curve parameters ----

    /// <summary>The kernel's curve of the wire (OCCT's <c>BRepAdaptor_CompCurve</c>), its edges joined end to end.</summary>
    public WireCurve GeomAdaptor()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't get geom adaptor of empty wire");
        return TopoWire.ToCurve();
    }

    internal override ShapeCurve Curve() => new(GeomAdaptor());

    /// <summary>The parameter of the wire's <c>BRepAdaptor_CompCurve</c> at a position (not an edge parameter).</summary>
    public override double ParamAt(double position)
    {
        var curve = GeomAdaptor();
        var target = PositionAt(position);
        var pnt = target.ToGpPnt();
        // The extrema of the distance, nearest first
        var extrema = curve.Project(pnt);
        if (extrema.IsEmpty)
            throw new InvalidOperationException("Failed to find point on curve");
        var closest = new Vector(extrema[0].Point);
        if ((closest - target).Length > Constants.Tolerance)
            throw new InvalidOperationException("Failed to find point on curve");
        return extrema[0].Parameter;
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
        var distance = vertex.TopoShape.Distance(TopoShape);
        if (!distance.TryGetValue(out var extrema) || extrema.Solutions.IsEmpty)
            throw new ArgumentException("point is not on Wire");

        global::Kerf.Topology.Edge closestEdge;
        double closestParam;
        var nearest = extrema.Solutions[0];
        switch (nearest.SupportB)
        {
            case SupportKind.Edge:
                closestEdge = (global::Kerf.Topology.Edge)nearest.SupportShapeB;
                closestParam = nearest.ParametersB.X;
                break;
            case SupportKind.Vertex:
            {
                var hit = (global::Kerf.Topology.Vertex)nearest.SupportShapeB;
                var edges = ShapeGraph.Build(TopoShape).Ancestors(hit, global::Kerf.Topology.ShapeType.Edge);
                closestEdge = (global::Kerf.Topology.Edge)edges.First();
                closestParam = closestEdge.ParameterOf(hit);
                break;
            }
            default:
                throw new ArgumentException("point is not on Wire");
        }

        double distanceAlongWire;
        {
            var adaptor = closestEdge.ToCurve();
            var (paramMin, paramMax) = closestEdge.Range;
            if (adaptor.IsPeriodic)
                closestParam = PyMath.Mod(closestParam - paramMin, adaptor.Period) + paramMin;
            distanceAlongWire = closestEdge.Orientation == global::Kerf.Topology.Orientation.Forward
                ? adaptor.LengthBetween(paramMin, closestParam)
                : adaptor.LengthBetween(closestParam, paramMax);
        }
        // The lengths of the edges before the closest one
        foreach (var edge in TopoWire.OrderedEdges())
        {
            if (edge.IsDegenerated)
                continue;
            if (edge.IsEqual(closestEdge))
                break;
            distanceAlongWire += edge.ToCurve().Length;
        }
        // The explorer follows the wire's orientation while positions measure a reversed wire from its other end
        var u = distanceAlongWire / Length;
        var normalized = IsForward ? u : 1 - u;
        return normalize ? normalized : ParamAt(normalized);
    }

    internal override (EdgeCurve Curve, double Param, bool Forward) OcctParamAt(double position, PositionMode mode = PositionMode.Parameter)
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
        var unchamfered = TopoUtil.MakeFaceFromWires(TopoWire);
        var graph = ShapeGraph.Build(unchamfered);
        var corners = new List<(global::Kerf.Topology.Edge, global::Kerf.Topology.Edge)>();
        foreach (var v in vertices)
        {
            if (v.IsEmpty)
                continue;
            var list = graph.Ancestors(v.TopoShape, global::Kerf.Topology.ShapeType.Edge);
            var pair = (new Edge((global::Kerf.Topology.Edge)list.First()), new Edge((global::Kerf.Topology.Edge)list.Last()));
            var (edge1, edge2) = OrderChamferEdges(edge, pair);
            if (!edge1.IsEmpty && !edge2.IsEmpty)
                corners.Add((edge1.TopoEdge, edge2.TopoEdge));
        }
        var built = Failures.Get(unchamfered.Chamfer2D(corners, distance, distance2), () => "2D chamfer");
        var chamfered = built.Shape;
        // The chamfered face shares the wire's other edges, which ShapeFix leaves untouched (it edits in a working
        // overlay)
        var fixedShape = TopoUtil.Downcast(TopoUtil.Fix(chamfered));
        if (!fixedShape.IsFace())
            throw new InvalidOperationException("An internal error occured creating the chamfer");
        var result = new Wire(((global::Kerf.Topology.Face)fixedShape).OuterWire);
        // The edges the chamfers trimmed are modified (OCCT's DescendantEdge); the builder answers nothing as generated
        return (Wire)result.MadeBy(ShapeHistory.FromAlgorithm(s => built.History.Modified(s), _ => [], [TopoShape], result.TopoShape));
    }

    /// <summary>This wire with its very short edges (below <paramref name="precision"/>) and gaps fixed.</summary>
    public Wire FixDegenerateEdges(double precision)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't fix an empty edge");
        // ShapeFix_Wireframe fixes in a working overlay, the wire untouched
        var options = new WireframeFixOptions { MaxTolerance = 1e-6, SmallEdgesFirst = true };
        var fixedShape = Failures.Get(TopoShape.FixWireframe(precision, options), () => "fix degenerate edges").Shape;
        return new Wire((global::Kerf.Topology.Wire)TopoUtil.Downcast(fixedShape));
    }

    /// <summary>This wire joined to another.</summary>
    public Wire Stitch(Wire other)
    {
        if (IsEmpty || other.IsEmpty)
            throw new InvalidOperationException("Can't stitch empty wires");
        // BRepBuilderAPI_MakeWire given each wire in turn: their edges added one by one
        var made = global::Kerf.Topology.Wire.FromEdges([.. TopoWire.Children.Select(e => (global::Kerf.Topology.Edge)e),
                                                         .. other.TopoWire.Children.Select(e => (global::Kerf.Topology.Edge)e)]);
        return (Wire)CastLike(Failures.Get(made, () => "stitch"));
    }

    /// <summary>
    /// This wire as one B-spline edge, its edges concatenated in order. The junctions become knots rather than vertices,
    /// which can surprise later operations: for internal use.
    /// </summary>
    internal Edge ToBSpline()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't convert an empty wire");
        var curves = new List<BoundedCurve3D>();
        foreach (var edge in TopoWire.OrderedEdges())
        {
            if (edge.IsDegenerated)
                continue;
            edge.TryGetCurve(out var curve, out var range);
            curves.Add(new TrimmedCurve3D(curve!, range.Min, range.Max));
        }
        if (!BSplineCurve3D.Join(curves, Constants.Tolerance).TryGetValue(out var spline)
            || !global::Kerf.Topology.Edge.FromCurve(spline).TryGetValue(out var made))
            throw new InvalidOperationException("Failed to build bspline.");
        return new Edge(made);
    }

    /// <summary>A new wire of the section between two positions (normalised parameters or points on the wire).</summary>
    public Wire Trim(CurvePosition start, CurvePosition end)
    {
        var startU = start.ToParam(this);
        var endU = end.ToParam(this);
        (startU, endU) = (Math.Min(startU, endU), Math.Max(startU, endU));
        var ordered = Edges().OrderBy(this);
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

        var projection = directionVector is { } dv
            ? ShapeProjections.Project(TopoWire, targetObject.TopoShape, dv.ToGpDir())
            : ShapeProjections.Project(TopoWire, targetObject.TopoShape, centerPoint.ToGpPnt());
        var output = new List<Wire>();
        {
            var orientation = TopoShape.Orientation;
            // A projection that meets nothing is no wires
            foreach (var projected in projection.TryGetValue(out var wires) ? wires : [])
            {
                output.Add(orientation == projected.Orientation
                    ? new Wire(projected)
                    : new Wire((global::Kerf.Topology.Wire)projected.Reversed()));
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
