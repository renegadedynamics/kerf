using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Modeling;

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
            var current = filleted.Vertices().OrderByDistanceTo(local)[0];
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
        var ownLocation = TopoShape.Placement;
        var geometryAdjust = new Location(ownLocation).Inverse() * globalized.Location;
        var trsf = geometryAdjust.ToTopLoc().Transformation;
        var identity = new Placement();
        var baseWire = globalized.TopoShape.Located(identity);
        record = record.Merge(ShapeHistory.FromRelocation(globalized.TopoShape, baseWire));
        var transformer = Failures.Get(baseWire.Transformed(trsf, copyGeometry: true), () => "transform");
        var transformed = (global::Kerf.Topology.Wire)transformer.Shape;
        record = record.Merge(ShapeHistory.FromAlgorithm(s => transformer.History.Modified(s), s => transformer.History.Generated(s),
                                                         [baseWire], transformed));
        var located = (global::Kerf.Topology.Wire)transformed.Located(ownLocation);
        record = record.Merge(ShapeHistory.FromRelocation(transformed, located));

        // The same direction as this wire
        var final = new Wire(located);
        if (IsForward != final.IsForward)
            final = new Wire((global::Kerf.Topology.Wire)located.Reversed());
        return (Wire)final.MadeBy(record);
    }
}

/// <summary>The single-corner wire fillet of build123d's one_d.py.</summary>
internal static class WireFillet
{
    internal sealed record Corner(Wire Wire, Vertex Vertex, ShapeList<Edge> AllEdges, List<Edge> ConnectedEdges, List<int> ConnectedEdgeIndices);

    internal sealed record Solution(List<global::Kerf.Topology.Edge?> TrimmedEdges, global::Kerf.Topology.Edge FilletEdge);

    internal static Wire FilletCorner(Wire wire, Vertex vertex, double radius)
    {
        var corner = Analyze(wire, vertex);
        if (IsTangentContinuous(corner))
            return wire;
        var solution = Solve(corner, radius)
                       ?? throw Failures.Expected(ModelingDiagnostics.FilletFailed,
                                                  $"The corner at {vertex} can't be filleted with a radius of {Formatting.Number(radius)}.");
        var result = Splice(corner, solution);
        if (!wire.IsClosed || result.IsClosed)
            return result;
        throw Failures.Expected(ModelingDiagnostics.FilletFailed, "The filleted wire isn't closed.");
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

    private static bool EdgeContainsVertex(global::Kerf.Topology.Edge edge, global::Kerf.Topology.Vertex vertex)
    {
        var p = vertex.Point;
        var p0 = edge.FirstVertex.Point;
        var p1 = edge.LastVertex.Point;
        return p0.DistanceTo(p) < Constants.Tolerance || p1.DistanceTo(p) < Constants.Tolerance;
    }

    private static List<global::Kerf.Topology.Edge> SplitEdgeAtVertex(Edge edge, Vertex splitVertex)
    {
        // Non-destructive, as all the modelling layer's Booleans: the inputs' tolerances are left alone
        var split = Failures.Get(TopoUtil.Boolean(BooleanKind.Split, [edge.TopoShape], [splitVertex.TopoShape]), () => "split").Shape;
        return [.. split.Explore<global::Kerf.Topology.Edge>()];
    }

    /// <summary>The vertex an edge starts at in its own sense, whatever its orientation (TopExp::FirstVertex without CumOri).</summary>
    private static global::Kerf.Topology.Vertex NaturalFirstVertex(global::Kerf.Topology.Edge edge) =>
        ((global::Kerf.Topology.Edge)edge.Oriented(Orientation.Forward)).FirstVertex;

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
            var first = NaturalFirstVertex(edge.TopoEdge);
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
        var arc = arcs.OrderByDistanceTo(corner.Vertex)[0];

        var trimmed = new List<global::Kerf.Topology.Edge?>();
        foreach (var connected in corner.ConnectedEdges)
        {
            var other = connected.Vertices().First(v => v != corner.Vertex);
            var filletVertex = arc.Vertices().OrderByDistanceTo(connected)[0];
            var pnt = filletVertex.ToVector().ToGpPnt();
            var splitVertex = new Vertex(global::Kerf.Topology.Vertex.At(pnt));
            // A copy is split, as build123d splits a deepcopy, so the pieces share nothing with the wire: the identity
            // applied with its geometry copied (BRepBuilderAPI_Transform with copy) rebuilds the edge throughout
            var copied = Failures.Get(connected.TopoShape.Transformed(Transform3D.Identity, copyGeometry: true), () => "copy").Shape;
            var pieces = SplitEdgeAtVertex(new Edge((global::Kerf.Topology.Edge)copied), splitVertex);
            var otherV = (global::Kerf.Topology.Vertex)other.TopoShape;
            var cornerV = (global::Kerf.Topology.Vertex)corner.Vertex.TopoShape;
            trimmed.Add(pieces.Where(e => EdgeContainsVertex(e, otherV) && !EdgeContainsVertex(e, cornerV)).Select(e => (global::Kerf.Topology.Edge?)e).FirstOrDefault());
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
        var edges = corner.AllEdges.Select(e => (global::Kerf.Topology.Edge?)e.TopoEdge).ToList();
        // Which original edge each entry came from, for the record
        var sources = corner.AllEdges.Select(e => (global::Kerf.Topology.Shape?)e.TopoShape).ToList();
        var history = new List<ShapeHistoryEntry>();

        var toRemove = new HashSet<int>();
        for (int i = 0; i < solution.TrimmedEdges.Count; i++)
        {
            var index = corner.ConnectedEdgeIndices[i];
            var original = corner.ConnectedEdges[i].TopoEdge;
            if (solution.TrimmedEdges[i] is not { } trimmed)
            {
                toRemove.Add(index);
                history.Add(new(original, ShapeHistoryKind.Removed, []));
                continue;
            }
            // Flip any edge reversed by the trim
            edges[index] = trimmed.Orientation != original.Orientation ? (global::Kerf.Topology.Edge)trimmed.Reversed() : trimmed;
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
        // BRepBuilderAPI_MakeWire, the edges added one by one
        var made = Failures.Get(global::Kerf.Topology.Wire.FromEdges(edges.Select(e => e!.Value)), () => "wire");
        // The edges as the finished wire holds them, in the order added: a later merge may have moved or enlarged an
        // earlier edge's vertex, which native OCCT does in place and OCCT.NET in a copy of the wire built so far
        var placedEdges = made.Children.ToList();
        for (int i = 0; i < edges.Count; i++)
        {
            var placed = placedEdges[i];
            if (sources[i] is not { } source)
            {
                history.Add(new(cornerVertex, ShapeHistoryKind.Generated, [placed]));
                foreach (var arcEnd in ShapeHistory.TrackedSubshapes([placed]))
                    if (arcEnd.Type == global::Kerf.Topology.ShapeType.Vertex)
                        history.Add(new(cornerVertex, ShapeHistoryKind.Generated, [arcEnd]));
            }
            else
            {
                if (!placed.IsSame(source))
                    history.Add(new(source, ShapeHistoryKind.Modified, [placed]));
                OneD.RecordMatchingVertices(history, source, placed);
            }
        }
        history.Add(new(cornerVertex, ShapeHistoryKind.Removed, []));
        return (Wire)new Wire(made).MadeBy(ShapeHistory.FromHistory(global::Kerf.Topology.ShapeHistory.Create(history)));
    }
}
