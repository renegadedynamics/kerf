using Kerf.Topology;
using Occt;

namespace Build123d;

public partial class Wire
{
    /// <summary>
    /// This (planar) wire with the corners at <paramref name="vertices"/> rounded to <paramref name="radius"/>; corners
    /// that are already tangent-continuous are left as they are.
    /// </summary>
    /// <exception cref="InvalidOperationException">The wire is empty.</exception>
    /// <exception cref="ArgumentException">The wire isn't planar, a vertex isn't on it, or a corner can't be filleted.</exception>
    public Wire Fillet2D(double radius, IEnumerable<Vertex> vertices)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't fillet an empty wire");
        var plane = CommonPlane() ?? throw new ArgumentException($"Wire is not planar {this}");

        // Fillet on Plane.XY
        var filleted = plane.ToLocalCoords(this);
        var record = ShapeHistory.FromRelocation(TopoShape, filleted.TopoShape);
        foreach (var vertex in vertices)
        {
            var local = plane.ToLocalCoords(vertex);
            var current = filleted.Vertices().SortByDistance(local)[0];
            if ((current.ToVector() - local.ToVector()).Length > Constants.Tolerance)
                throw new ArgumentException($"Could not find fillet vertex on wire: {vertex}");
            var beforeCorner = filleted;
            filleted = WireFillet.FilletCorner(filleted, current, radius);
            if (!ReferenceEquals(filleted, beforeCorner))
                record = record.Merge(filleted.History);
        }

        // Back to the wire's plane
        var globalized = plane.FromLocalCoords(filleted);
        record = record.Merge(ShapeHistory.FromRelocation(filleted.TopoShape, globalized.TopoShape));

        // Back to the original location rather than the plane's
        var ownLocation = TopoShape.Location();
        var geometryAdjust = new Location(ownLocation).Inverse() * globalized.Location;
        var trsf = geometryAdjust.ToTopLoc().Transformation();
        var identity = new Placement();
        var baseWire = globalized.TopoShape.Located(identity);
        record = record.Merge(ShapeHistory.FromRelocation(globalized.TopoShape, baseWire));
        var transformer = new BRepBuilderAPI_Transform(baseWire, trsf, true);
        var transformed = TopoDS.Wire(transformer.Shape());
        record = record.Merge(ShapeHistory.FromAlgorithm(transformer, [baseWire], transformed));
        var located = TopoDS.Wire(transformed.Located(ownLocation));
        record = record.Merge(ShapeHistory.FromRelocation(transformed, located));

        // The same direction as this wire
        var final = new Wire(located);
        if (IsForward != final.IsForward)
            final = new Wire(TopoDS.Wire(located.Reversed()));
        return (Wire)final.MadeBy(record);
    }
}

/// <summary>The single-corner wire fillet of build123d's one_d.py.</summary>
internal static class WireFillet
{
    internal sealed record Corner(Wire Wire, Vertex Vertex, ShapeList<Edge> AllEdges, List<Edge> ConnectedEdges, List<int> ConnectedEdgeIndices);

    internal sealed record Solution(List<TopoDS_Edge?> TrimmedEdges, TopoDS_Edge FilletEdge);

    internal static Wire FilletCorner(Wire wire, Vertex vertex, double radius)
    {
        var corner = Analyze(wire, vertex);
        if (IsTangentContinuous(corner))
            return wire;
        var solution = Solve(corner, radius)
                       ?? throw new ArgumentException($"Fillet algorithm failed for {vertex} with radius {radius}");
        var result = Splice(corner, solution);
        if (!wire.IsClosed || result.IsClosed)
            return result;
        throw new ArgumentException("Filleting failed to create a closed wire.");
    }

    internal static Corner Analyze(Wire wire, Vertex vertex)
    {
        var all = wire.Edges();
        var connected = all.Where(e => e.Vertices().Any(v => v.TopoShape.IsSame(vertex.TopoShape))).ToList();
        if (connected.Count == 0)
            throw new ArgumentException($"Could not find shared vertex on wire: {vertex}");
        if (connected.Count != 2)
            throw new ArgumentException($"Vertex must connect exactly two edges: {vertex}");
        var indices = connected.Select(e => all.ToList().IndexOf(e)).ToList();
        return new Corner(wire, vertex, all, connected, indices);
    }

    private static bool EdgeContainsVertex(TopoDS_Edge edge, TopoDS_Vertex vertex)
    {
        var v0 = TopExp.FirstVertex(edge);
        var v1 = TopExp.LastVertex(edge);
        var p = BRep_Tool.Pnt(vertex);
        var p0 = BRep_Tool.Pnt(v0);
        var p1 = BRep_Tool.Pnt(v1);
        return p0.Distance(p) < Constants.Tolerance || p1.Distance(p) < Constants.Tolerance;
    }

    private static List<TopoDS_Edge> SplitEdgeAtVertex(Edge edge, Vertex splitVertex)
    {
        var splitter = new BOPAlgo_Splitter();
        splitter.SetNonDestructive(true);   // leave the inputs' tolerances alone
        splitter.AddArgument(edge.TopoShape);
        splitter.AddTool(splitVertex.TopoShape);
        splitter.Perform();
        var split = splitter.Shape();
        var edges = new List<TopoDS_Edge>();
        var explorer = new TopExp_Explorer(split, TopAbs_ShapeEnum.TopAbs_EDGE, TopAbs_ShapeEnum.TopAbs_SHAPE);
        for (; explorer.More(); explorer.Next())
            edges.Add(TopoDS.Edge(explorer.Current()));
        return edges;
    }

    /// <summary>Whether the two edges at the corner are tangent-continuous there.</summary>
    internal static bool IsTangentContinuous(Corner corner)
    {
        var (e0, e1) = (corner.ConnectedEdges[0], corner.ConnectedEdges[1]);
        var tangent0 = e0.TangentAt(corner.Vertex);
        var tangent1 = e1.TangentAt(corner.Vertex);
        if (tangent0.Cross(tangent1).Length > Constants.Tolerance)
            return false;

        var sampleLength = corner.ConnectedEdges.Min(e => e.Length) * 1e-3;
        sampleLength = Math.Min(sampleLength, corner.ConnectedEdges.Min(e => e.Length * 0.25));
        sampleLength = Math.Max(sampleLength, Constants.Tolerance * 100);

        var reflected = new List<Vector>();
        foreach (var edge in corner.ConnectedEdges)
        {
            var first = TopExp.FirstVertex(edge.TopoEdge);
            var samplePoint = first.IsSame(corner.Vertex.TopoShape)
                ? edge.PositionAt(sampleLength, PositionMode.Length)
                : edge.PositionAt(edge.Length - sampleLength, PositionMode.Length);
            reflected.Add(corner.Vertex.ToVector() * 2 - samplePoint);
        }
        return Math.Min(e1.DistanceTo(reflected[0]), e0.DistanceTo(reflected[1])) <= Math.Max(Constants.Tolerance * 10, sampleLength * 0.25);
    }

    /// <summary>The fillet from the Geom2dGcc_Circ2d2TanRad tangent-arc solver, or null if it finds none.</summary>
    internal static Solution? Solve(Corner corner, double radius)
    {
        var arcs = ConstrainedSolvers.TwoTanRadArcs([corner.ConnectedEdges[0], corner.ConnectedEdges[1]], radius, Sagitta.Both);
        if (arcs.Count == 0)
            return null;
        var arc = arcs.SortByDistance(corner.Vertex)[0];

        var trimmed = new List<TopoDS_Edge?>();
        foreach (var connected in corner.ConnectedEdges)
        {
            var other = connected.Vertices().First(v => v != corner.Vertex);
            var filletVertex = arc.Vertices().SortByDistance(connected)[0];
            var pnt = filletVertex.ToVector().ToGpPnt();
            var maker = new BRepBuilderAPI_MakeVertex(pnt);
            var splitVertex = new Vertex(maker.Vertex());
            // A copy is split, as build123d splits a deepcopy, so the pieces share nothing with the wire
            var copier = new BRepBuilderAPI_Copy(connected.TopoShape);
            var pieces = SplitEdgeAtVertex(new Edge(TopoDS.Edge(copier.Shape())), splitVertex);
            var otherV = TopoDS.Vertex(other.TopoShape);
            var cornerV = TopoDS.Vertex(corner.Vertex.TopoShape);
            trimmed.Add(pieces.Where(e => EdgeContainsVertex(e, otherV) && !EdgeContainsVertex(e, cornerV)).Select(e => (TopoDS_Edge?)e).FirstOrDefault());
        }
        return new Solution(trimmed, arc.TopoEdge);
    }

    /// <summary>
    /// The wire with the two edges at the corner trimmed and the fillet between them, carrying the record of the corner: the
    /// vertex removed, the edges beside it trimmed, the arc generated from the vertex, and every other edge modified into
    /// the copy <c>BRepBuilderAPI_MakeWire</c> makes of it.
    /// </summary>
    private static Wire Splice(Corner corner, Solution solution)
    {
        var edges = corner.AllEdges.Select(e => (TopoDS_Edge?)e.TopoEdge).ToList();
        // Which original edge each entry came from, for the record
        var sources = corner.AllEdges.Select(e => (TopoDS_Shape?)e.TopoShape).ToList();
        var history = new BRepTools_History();

        var toRemove = new HashSet<int>();
        for (int i = 0; i < solution.TrimmedEdges.Count; i++)
        {
            var index = corner.ConnectedEdgeIndices[i];
            var original = corner.ConnectedEdges[i].TopoEdge;
            if (solution.TrimmedEdges[i] is not { } trimmed)
            {
                toRemove.Add(index);
                history.Remove(original);
                continue;
            }
            // Flip any edge reversed by the trim
            edges[index] = trimmed.Orientation() != original.Orientation() ? TopoDS.Edge(trimmed.Reversed()) : trimmed;
        }

        // The insert index, in the original index space
        int n = edges.Count;
        var (i0, i1) = (corner.ConnectedEdgeIndices[0], corner.ConnectedEdgeIndices[1]);
        var insertIndex = i1 == (i0 + 1) % n ? i0 + 1 : i1 + 1;
        foreach (var index in toRemove.OrderByDescending(i => i))
        {
            edges.RemoveAt(index);
            sources.RemoveAt(index);
            if (index < insertIndex)
                insertIndex--;
        }
        edges.Insert(insertIndex, solution.FilletEdge);
        sources.Insert(insertIndex, null);   // the arc comes from the vertex, not an edge

        // MakeWire copies each edge so neighbours share vertices: record each as the finished wire places it
        var cornerVertex = corner.Vertex.TopoShape;
        var builder = new BRepBuilderAPI_MakeWire();
        foreach (var edge in edges)
            builder.Add(edge!.Value);
        builder.Build();
        // The edges as the finished wire holds them, in the order added: a later merge may have moved or enlarged an
        // earlier edge's vertex, which native OCCT does in place and OCCT.NET in a copy of the wire built so far
        var placedEdges = new List<TopoDS_Shape>();
        var wireIt = new TopoDS_Iterator(builder.Wire());
        for (; wireIt.More(); wireIt.Next())
            placedEdges.Add(wireIt.Value());
        for (int i = 0; i < edges.Count; i++)
        {
            var placed = placedEdges[i];
            if (sources[i] is not { } source)
            {
                history.AddGenerated(cornerVertex, placed);
                foreach (var arcEnd in ShapeHistory.TrackedSubshapes([placed]))
                    if (arcEnd.ShapeType() == TopAbs_ShapeEnum.TopAbs_VERTEX)
                        history.AddGenerated(cornerVertex, arcEnd);
            }
            else
            {
                if (!placed.IsSame(source))
                    history.AddModified(source, placed);
                OneD.RecordMatchingVertices(history, source, placed);
            }
        }
        history.Remove(cornerVertex);
        builder.Build();
        return (Wire)new Wire(builder.Wire()).MadeBy(ShapeHistory.FromHistory(history));
    }
}
