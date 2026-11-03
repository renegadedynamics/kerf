using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Modeling;

/// <summary>The module-level functions of build123d's topology/one_d.py.</summary>
public static class OneD
{
    /// <summary>The wires made by connecting edges end to end within <paramref name="tolerance"/>.</summary>
    public static ShapeList<Wire> EdgesToWires(IEnumerable<Edge> edges, double tolerance = 1e-6)
    {
        var edgesIn = new NCollection_HSequence<global::Kerf.Topology.Shape>();
        foreach (var edge in edges)
            if (!edge.IsEmpty)
                edgesIn.Append(edge.TopoShape);
        var wiresOut = FreeBounds.ConnectEdgesToWires(edgesIn, tolerance, false);
        if (wiresOut is null)
            return [];
        return new ShapeList<Wire>(wiresOut.Select(w => new Wire(ShapeCasts.Wire(w))));
    }

    internal static Shape Add1D(Shape self, IEnumerable<Shape?> other)
    {
        var topoSummands = other.OfType<Shape>().Where(o => !o.IsEmpty)
                                .SelectMany(o => TopoUtil.TopLevelShapes(o.TopoShape)).ToList();
        if (topoSummands.Count == 0)
            return self;
        if (topoSummands.Any(s => TopoUtil.Dim(s) != 1))
            throw new ArgumentException("Only shapes with the same dimension can be added");
        var summands = topoSummands.Select(s => (Shape1D)Shape.Cast(s)).ToList();
        var summandEdges = summands.SelectMany(s => s.Edges()).ToList();
        var brought = summands.Select(s => s.TopoShape).ToList();

        Shape sum;
        if (self.IsEmpty)
        {
            if (summands.Count == 1)
                sum = summands[0];
            else
            {
                try
                {
                    sum = JoinedWire(summandEdges, [], brought);
                }
                catch (Exception e) when (e is ArgumentException or InvalidOperationException or FailureException or KerfException)
                {
                    sum = summands[0].Fuse(summands.Skip(1));
                    if (sum.History is not null)
                        sum = sum.MadeBy(sum.History.WithInputs([], brought));
                }
            }
        }
        else
        {
            try
            {
                sum = JoinedWire([.. self.Edges(), .. summandEdges], [self.TopoShape], brought);
            }
            catch (Exception e) when (e is ArgumentException or InvalidOperationException or FailureException or KerfException)
            {
                sum = self.Fuse(summands);
            }
        }
        if (SkipClean.Clean)
            sum = sum.Clean();
        // A single edge is returned as an Edge, with the record of how it was made
        return sum.Edges().Count == 1 ? sum.SingleEdge().MadeBy(sum.History) : sum;
    }

    /// <summary>A face offset by <paramref name="amount"/> (<c>BRepOffset_MakeOffset</c>).</summary>
    public static global::Kerf.Topology.Shape OffsetTopodsFace(global::Kerf.Topology.Face face, double amount)
    {
        var offsetter = new MakeOffset();
        offsetter.Initialize(face, amount, Constants.Tolerance);
        offsetter.MakeOffsetShape();
        return offsetter.Shape();
    }

    /// <summary>
    /// The edges of <paramref name="parent"/> (default the edge's <see cref="Shape.TopoParent"/>) connected to
    /// <paramref name="edge"/> with at least the given continuity. A moved copy of one of the parent's edges gets its
    /// neighbours in its own frame.
    /// </summary>
    public static ShapeList<Edge> ConnectedEdges(Edge edge, Shape? parent = null, ContinuityLevel continuity = ContinuityLevel.C0)
    {
        parent ??= edge.TopoParent ?? throw new ArgumentException("edge has no valid parent");
        if (edge.IsEmpty)
            throw new ArgumentException("edge is empty");
        var topoEdges = parent.Edges().Where(e => !e.IsEmpty).Select(e => e.TopoShape).ToList();
        // Work with the parent's own copy of the edge so vertices match
        var parentEdge = TopoUtil.FindSame(edge.TopoShape, topoEdges);
        if (parentEdge is null)
            return [];
        var given = ShapeCasts.Edge(parentEdge.Value);
        var relocation = TopoUtil.RelocationBetween(given, edge.TopoShape);
        var connected = new List<global::Kerf.Topology.Shape>();
        foreach (var other in topoEdges)
        {
            if (given.IsSame(other))
                continue;
            var otherEdge = ShapeCasts.Edge(other);
            var common = Vertex.CommonVertex(given, otherEdge);
            if (common.IsEmpty)
                continue;
            var u1 = Tool.Parameter(common.TopoVertex, given);
            var u2 = Tool.Parameter(common.TopoVertex, otherEdge);
            var curve1 = new EdgeCurveAdaptor(given);
            var curve2 = new EdgeCurveAdaptor(otherEdge);
            var actual = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(curve1, curve2, u1, u2, Constants.Tolerance, Constants.Tolerance);
            var level = actual switch
            {
                Continuity.C0 => ContinuityLevel.C0,
                Continuity.G1 or Continuity.C1 => ContinuityLevel.C1,
                _ => ContinuityLevel.C2,
            };
            if (level >= continuity && !connected.Any(c => c.IsSame(other)))
                connected.Add(other);
        }
        return relocation.IsIdentity
            ? new ShapeList<Edge>(connected.Select(e => new Edge(ShapeCasts.Edge(e))))
            : new ShapeList<Edge>(connected.Select(e => new Edge(ShapeCasts.Edge(e.Moved(relocation)))));
    }

    /// <summary>
    /// The faces of <paramref name="parent"/> (default the edge's <see cref="Shape.TopoParent"/>) that meet at an edge.
    /// A moved copy of one of the parent's edges gets the faces in its own frame; an edge the parent doesn't contain has none.
    /// </summary>
    public static List<global::Kerf.Topology.Face> ConnectedFaces(Edge edge, Shape? parent = null) => ConnectedFaces((Shape1D)edge, parent);

    internal static List<global::Kerf.Topology.Face> ConnectedFaces(Shape1D edge, Shape? parent = null)
    {
        if (edge.IsEmpty)
            throw new ArgumentException("Can't explore from an empty edge");
        parent ??= edge.TopoParent;
        if (parent is null || parent.IsEmpty)
            throw new InvalidOperationException("edge has no valid parent");
        var map = new IndexedMap<global::Kerf.Topology.Shape, NCollection_List<global::Kerf.Topology.Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(parent.TopoShape, global::Kerf.Topology.ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, map);
        // The map keys on TShape and Location, so a moved edge isn't found by Contains
        var parentEdge = TopoUtil.FindSame(edge.TopoShape, map.Keys);
        if (parentEdge is null)
            return [];
        var relocation = TopoUtil.RelocationBetween(parentEdge.Value, edge.TopoShape);
        var unique = new IndexedSet<global::Kerf.Topology.Shape>(ShapeHasher.Instance);
        var faces = map.GetValue(parentEdge.Value);
        foreach (var face in faces)
            unique.Add(face);
        return [.. unique.Select(f => ShapeCasts.Face(f.Moved(relocation)))];
    }

    /// <summary>Whether the face meets itself along the edge, as a cylinder does at its seam.</summary>
    internal static bool IsSeamOf(global::Kerf.Topology.Edge edge, global::Kerf.Topology.Face face)
    {
        int count = 0;
        var explorer = new Explorer(face, global::Kerf.Topology.ShapeType.Edge);
        for (; explorer.More(); explorer.Next())
            if (explorer.Current().IsSame(edge))
                count++;
        return count > 1;
    }

    /// <summary>The edge with the orientation it has in the face's boundary.</summary>
    private static global::Kerf.Topology.Edge AsBoundedBy(global::Kerf.Topology.Face face, global::Kerf.Topology.Edge edge)
    {
        var explorer = new Explorer(face, global::Kerf.Topology.ShapeType.Edge);
        for (; explorer.More(); explorer.Next())
            if (explorer.Current().IsSame(edge))
                return ShapeCasts.Edge(explorer.Current());
        throw new ArgumentException("edge is not on the face");
    }

    /// <summary>
    /// Classifies the crease between two faces along their shared edge. At each sample the faces' outward normals and the
    /// edge tangent (the way the second face's boundary runs) give the direction from the edge into the second face,
    /// which dips behind the first face's tangent plane at a convex crease and rises above it at a concave one. Normals
    /// that agree within <paramref name="sinTolerance"/> are a smooth join; degenerate samples are skipped.
    /// </summary>
    internal static Convexity DihedralConvexity(global::Kerf.Topology.Edge edge, global::Kerf.Topology.Face face1, global::Kerf.Topology.Face face2,
                                                int samples = 7, double sinTolerance = 1e-4)
    {
        var curve = new EdgeCurveAdaptor(edge);
        double first = curve.FirstParameter(), last = curve.LastParameter();
        double f1 = first, l1 = last, f2 = first, l2 = last;
        var pcurve1 = Tool.CurveOnSurface(edge, face1, out f1, out l1)!;
        var pcurve2 = Tool.CurveOnSurface(edge, face2, out f2, out l2)!;
        bool reversedInFace2 = AsBoundedBy(face2, edge).Orientation == Orientation.Reversed;
        var props1 = new FaceIntegrand(face1);
        var props2 = new FaceIntegrand(face2);
        var kinds = new HashSet<Convexity>();
        for (int i = 0; i < samples; i++)
        {
            var param = first + (last - first) * (i + 0.5) / samples;
            var point = new Point3D();
            var tangent = new Vector3D();
            curve.D1(param, out point, out tangent);
            var uv1 = pcurve1.Value(param);
            var uv2 = pcurve2.Value(param);
            var normal1 = new Vector3D();
            var normal2 = new Vector3D();
            props1.Normal(uv1.X, uv1.Y, out point, out normal1);
            props2.Normal(uv2.X, uv2.Y, out point, out normal2);
            if (Math.Min(tangent.Magnitude, Math.Min(normal1.Magnitude, normal2.Magnitude)) < 1e-12)
                continue;
            tangent = tangent.Normalized();
            normal1 = normal1.Normalized();
            normal2 = normal2.Normalized();
            var cross = normal1.Crossed(normal2);
            if (cross.Magnitude < sinTolerance)
            {
                kinds.Add(Convexity.Smooth);
                continue;
            }
            if (reversedInFace2)
                tangent = tangent.Reversed();
            var intoFace2 = normal2.Crossed(tangent);
            kinds.Add(intoFace2.Dot(normal1) < 0 ? Convexity.Convex : Convexity.Concave);
        }
        if (kinds.Count == 0)
            throw new InvalidOperationException("the faces are degenerate along this edge");
        kinds.Remove(Convexity.Smooth);
        return kinds.Count switch
        {
            0 => Convexity.Smooth,
            1 => kinds.Single(),
            _ => Convexity.Saddle,
        };
    }

    /// <summary>
    /// A wire from connected edges, with the record of what became of each: <c>BRepBuilderAPI_MakeWire</c> copies every
    /// edge so neighbours share vertices, so each input edge is recorded as modified into the result edge with the same
    /// geometry, and its vertices into those at the same points.
    /// </summary>
    internal static Wire JoinedWire(IReadOnlyList<Edge> edges, IEnumerable<global::Kerf.Topology.Shape> before, IEnumerable<global::Kerf.Topology.Shape> brought)
    {
        var wire = new Wire(edges);
        var history = new History();
        var placed = wire.Edges();
        var midpoints = placed.Select(e => e.PositionAt(0.5)).ToList();
        foreach (var source in edges)
        {
            var midpoint = source.PositionAt(0.5);
            var distances = midpoints.Select(m => (m - midpoint).Length).ToList();
            var min = distances.Min();
            if (min > Constants.Tolerance)
                continue;   // MakeWire keeps every edge it is given
            var match = placed[distances.IndexOf(min)];
            if (!match.TopoShape.IsSame(source.TopoShape))
                history.AddModified(source.TopoShape, match.TopoShape);
            RecordMatchingVertices(history, source.TopoShape, match.TopoShape);
        }
        return (Wire)wire.MadeBy(ShapeHistory.FromHistory(history, before, brought));
    }

    /// <summary>Records the vertices of a rebuilt edge as modified from those at the same points.</summary>
    internal static void RecordMatchingVertices(History history, global::Kerf.Topology.Shape source, global::Kerf.Topology.Shape placed)
    {
        var placedVertices = ShapeHistory.TrackedSubshapes([placed]).Where(v => v.Type == global::Kerf.Topology.ShapeType.Vertex).ToList();
        foreach (var old in ShapeHistory.TrackedSubshapes([source]))
        {
            if (old.Type != global::Kerf.Topology.ShapeType.Vertex)
                continue;
            var oldVertex = ShapeCasts.Vertex(old);
            var oldPoint = Tool.Pnt(oldVertex);
            foreach (var @new in placedVertices)
            {
                if (@new.IsSame(old))
                    break;
                var newVertex = ShapeCasts.Vertex(@new);
                var newPoint = Tool.Pnt(newVertex);
                if (newPoint.DistanceTo(oldPoint) < Constants.Tolerance)
                {
                    history.AddModified(old, @new);
                    break;
                }
            }
        }
    }

    /// <summary>
    /// The wire with its offset curves (which STEP export, among others, can't handle) replaced by B-splines within a
    /// tenth of the tolerance; a wire without offset curves is returned as it is.
    /// </summary>
    internal static Wire OffsetCurvesToBSplines(Wire wire)
    {
        var edges = wire.Edges();
        if (edges.All(e => e.GeomType != GeomType.Offset))
            return wire;
        var fitted = new List<Edge>();
        foreach (var edge in edges)
        {
            if (edge.GeomType != GeomType.Offset)
            {
                fitted.Add(edge);
                continue;
            }
            var adaptor = edge.GeomAdaptor();
            // Fit only the part of the offset curve the edge uses, as it lies before the edge's location
            var curveLocation = new Placement();
            double f = 0, l = 0;
            var curve = Tool.Curve(edge.TopoEdge, out curveLocation, out f, out l);
            var trimmed = new TrimmedCurve3D(curve!, adaptor.FirstParameter(), adaptor.LastParameter());
            var approximation = new ApproxCurve(trimmed, Constants.Tolerance / 10, Continuity.C2, 100, 9);
            if (!approximation.HasResult)
                throw new InvalidOperationException("2D offset curve could not be converted to a spline");
            var spline = approximation.Curve();
            var maker = new MakeEdge(spline!);
            // The curve is stored without the placement or direction of its edge
            var placed = maker.Edge().Located(edge.TopoShape.Placement);
            fitted.Add(new Edge(ShapeCasts.Edge(placed.Oriented(edge.TopoShape.Orientation))));
        }
        // The edges came from the wire with its direction applied, so the new wire winds the same way
        return new Wire(fitted);
    }
}

public abstract partial class Shape1D
{
    /// <summary>
    /// This planar edge or wire offset within its plane.
    /// </summary>
    /// <remarks>
    /// Left and right are those of someone travelling along the curve from its start, the plane's normal pointing up. The
    /// plane is <paramref name="plane"/> if given, else the one the curve lies in, with its normal taken from the curve's
    /// location or from Plane.XY where one of them matches; a straight curve lies in many planes, and that of its location
    /// is used if the line is in it, then Plane.XY. The offset of a line is a line and of a circle a circle; other curves
    /// offset into offset curves, which are replaced by B-splines unless <paramref name="asBSpline"/> is false.
    /// </remarks>
    /// <param name="distance">How far to offset.</param>
    /// <param name="kind">The transition at corners.</param>
    /// <param name="side">Which side to offset to.</param>
    /// <param name="closed">For one side of an open curve, close the offset back to the curve.</param>
    /// <param name="plane">The plane of the offset.</param>
    /// <param name="asBSpline">Replace offset curves by B-splines.</param>
    /// <returns>An edge, or a wire.</returns>
    public Shape1D Offset2D(double distance, Kind kind = Kind.Arc, Side side = Side.Both, bool closed = true, Plane? plane = null,
                            bool asBSpline = true)
    {
        var join = kind switch
        {
            Kind.Arc => JoinKind.Arc,
            Kind.Intersection => JoinKind.Intersection,
            _ => JoinKind.Tangent,
        };
        var line = this as Wire ?? new Wire([(Edge)this]);
        var lineEdges = line.Edges();
        var start = line.PositionAt(0);
        var tangent = line.TangentAt(0);
        var startAxis = new Axis(start, tangent);
        bool isStraight = lineEdges.All(e => e.GeomType == GeomType.Line
                                             && new Axis(e.PositionAt(0), e.TangentAt(0)).IsCoaxial(startAxis));

        // The normal of the plane of the offset: the curve fixes the plane but not which way its normal points (and a
        // straight one doesn't fix the plane), so both are settled by the curve's location, or failing that Plane.XY
        Vector[] references = [Location.ZAxis.Direction, lineEdges[0].Location.ZAxis.Direction, new Vector(0, 0, 1)];
        Vector? normal = null;
        if (plane is not null)
            normal = plane.ZDir;
        else if (isStraight)
        {
            foreach (var r in references)
                if (Math.Abs(r.Dot(tangent)) < Constants.Tolerance)
                {
                    normal = r;
                    break;
                }
        }
        else if (line.CommonPlane() is { } own)
        {
            normal = own.ZDir;
            foreach (var r in references)
                if (Math.Abs(own.ZDir.Dot(r)) > 1 - Constants.Tolerance)
                {
                    normal = r;
                    break;
                }
        }

        var signedDistance = distance;
        if (side != Side.Both && line.IsClosed && normal is { } n)
        {
            // OCCT offsets a closed wire outwards for a positive distance; which side is outside depends on the winding
            int steps = Math.Max(16, 8 * lineEdges.Count);
            var points = Enumerable.Range(0, steps).Select(i => line.PositionAt(i / (double)steps)).ToList();
            double twiceArea = 0;
            for (int i = 1; i < steps - 1; i++)
                twiceArea += (points[i] - points[0]).Cross(points[i + 1] - points[0]).Dot(n);
            bool leftIsOutside = twiceArea < 0;
            bool outwards = leftIsOutside == (side == Side.Left);
            signedDistance = outwards ? Math.Abs(distance) : -Math.Abs(distance);
        }

        // Avoiding a bug when the wire contains a single edge
        global::Kerf.Topology.Wire topoWire = lineEdges.Count == 1
            ? new Wire([lineEdges[0].Trim(0.0, 0.5), lineEdges[0].Trim(0.5, 1.0)]).TopoWire
            : line.TopoWire;
        var builder = new MakeWireOffset();
        if (isStraight && normal is { } sn)
        {
            // A straight line has no plane of its own to offset within
            var spine = MakePlaneFace(new Plane(start, zDir: sn));
            builder.Init(spine, join);
        }
        else
            builder.Init(join);
        builder.AddWire(topoWire);
        builder.Perform(signedDistance);
        if (!builder.IsDone)
            throw new InvalidOperationException($"2D offset failed with distance {distance}");
        var obj = TopoUtil.Downcast(builder.Shape());
        if (obj.IsCompound())
            obj = TopoUtil.UnwrapCompound(ShapeCasts.Compound(obj));
        if (!obj.IsWire())
            throw new InvalidOperationException("Unexpected result type");   // likely several wires
        var offsetWire = new Wire(ShapeCasts.Wire(obj));
        if (asBSpline)
            offsetWire = OneD.OffsetCurvesToBSplines(offsetWire);

        if (side != Side.Both && !line.IsClosed)
        {
            // Remove the end arcs
            Vector[] endpoints = [line.PositionAt(0), line.PositionAt(1)];
            var offsetEdges = offsetWire.Edges().FilterBy(
                e => e.GeomType == GeomType.Circle && endpoints.Any(pt => (e.ArcCenter - pt).Length < Constants.Tolerance),
                reverse: true);
            var wires = OneD.EdgesToWires(offsetEdges);
            if (normal is null)
            {
                var angles = wires.Select(w => line.TangentAt(0).GetSignedAngle(w.PositionAt(0.5) - line.PositionAt(0))).ToList();
                offsetWire = side == Side.Left ? wires[angles[0] > angles[1] ? 1 : 0] : wires[angles[0] <= angles[1] ? 1 : 0];
            }
            else
            {
                // Each side starts beside the start of the line; a side can be missing, as when an arc is offset inwards
                // by more than its radius
                var toTheLeft = normal.Value.Cross(tangent);
                var onSide = wires.Where(w =>
                {
                    var besideStart = new[] { w.PositionAt(0), w.PositionAt(1) }.MinBy(e => (e - start).Length);
                    return (besideStart - start).Dot(toTheLeft) > 0 == (side == Side.Left);
                }).ToList();
                if (onSide.Count == 0)
                    throw new ArgumentException(
                        $"Nothing is left of an offset of {distance} on the {side.ToString().ToUpperInvariant()} side - "
                        + "reduce the distance or use the other side");
                offsetWire = onSide[0];
            }
            if (closed)
            {
                var self0 = line.PositionAt(0);
                var self1 = line.PositionAt(1);
                var end0 = offsetWire.PositionAt(0);
                var end1 = offsetWire.PositionAt(1);
                Edge edge0, edge1;
                if ((self0 - end0).Length - Math.Abs(distance) <= Constants.Tolerance)
                    (edge0, edge1) = (Edge.MakeLine(self0, end0), Edge.MakeLine(self1, end1));
                else
                    (edge0, edge1) = (Edge.MakeLine(self0, end1), Edge.MakeLine(self1, end0));
                offsetWire = new Wire([.. line.Edges(), .. offsetWire.Edges(), edge0, edge1]);
            }
        }
        var result = offsetWire.Edges();
        return result.Count == 1 ? result[0] : offsetWire;
    }
}
