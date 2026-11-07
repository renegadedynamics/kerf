using Kerf.Construction;
using Kerf.Healing;
using Kerf.Measurement;

namespace Kerf.Modeling;

// The face, shell and extrusion helpers of build123d's topology/utils.py
internal static partial class TopoUtil
{
    /// <summary>The point of a face at normalised (u, v), each from 0 to 1 across the face's own parameter range.</summary>
    internal static Point3D FacePosition(Face face, double u, double v)
    {
        var (us, vs) = face.UVBounds;
        face.TryGetSurface(out var surface);
        var p = surface!.Evaluate(us.Min + u * (us.Max - us.Min), vs.Min + v * (vs.Max - vs.Min));
        return p;
    }

    /// <summary>
    /// The unit normal of a face at normalised (u, v), with the face's own orientation: reversed faces point the
    /// other way from their surface.
    /// </summary>
    internal static Vector3D FaceNormal(Face face, double u = 0.5, double v = 0.5)
    {
        var (us, vs) = face.UVBounds;
        return FaceNormalVector(face, us.Min + u * (us.Max - us.Min), vs.Min + v * (vs.Max - vs.Min)).Normalized();
    }

    /// <summary>The centre of mass of a face's area.</summary>
    internal static Point3D FaceCenter(Face face) => face.CentreOfArea(MeasuredMoments);

    // How much further apart than the clean's own 1e-7 and 1e-12 two neighbours must be for this to be sure of them
    private const double SurelyAtAnAngle = 1e-6, SurelyApart = 1e-5;

    /// <summary>
    /// Whether a clean is sure to hand back what extruding this profile along its normal makes (decision 1895). It merges only
    /// through an edge two faces share, so only neighbouring sides could merge, and a side's edges only once its neighbour has gone.
    /// A case in doubt answers false and is cleaned: any curve but a line or a circle, any edge that isn't plainly part of a
    /// boundary, a vertex with other than two ends of edges, lines within a millionth of a radian of one line, circles
    /// within 1e-5 of one circle. A line and a circle make a plane and a cylinder, which the clean never merges.
    /// </summary>
    internal static bool NothingToCleanAfterExtruding(Face profile, Vector3D direction)
    {
        if (!profile.TryGetSurface(out var surface) || surface is not PlaneSurface plane)
            return false;
        if (Math.Abs(new Vector3D(plane.Position.Direction).Dot(direction.Normalized())) < 1 - 1e-12)
            return false;

        var edges = Explore(profile, ShapeType.Edge);
        var seen = new HashSet<Shape>(ShapeComparer.Same);
        var curves = new Curve3D[edges.Count];
        var joints = new Dictionary<Shape, (int First, int Second, int Ends)>(ShapeComparer.Same);
        for (int i = 0; i < edges.Count; i++)
        {
            var edge = (Edge)edges[i];
            if (edge.Orientation is not (Orientation.Forward or Orientation.Reversed) || !seen.Add(edge))
                return false;
            if (!edge.TryGetCurve(out var curve, out _) || curve is not (LineCurve3D or CircleCurve3D))
                return false;
            curves[i] = curve;
            Shape first = edge.FirstVertex, last = edge.LastVertex;
            if (first.IsNull || last.IsNull)
                return false;
            foreach (var vertex in new[] { first, last })
            {
                joints[vertex] = joints.TryGetValue(vertex, out var joint) ? (joint.First, i, joint.Ends + 1) : (i, -1, 1);
            }
        }

        foreach (var (_, joint) in joints)
        {
            if (joint.Ends != 2)
                return false;
            if (joint.First != joint.Second && MightBeOne(curves[joint.First], curves[joint.Second]))
                return false;
        }
        return edges.Count > 0;
    }

    private static bool MightBeOne(Curve3D a, Curve3D b) => (a, b) switch
    {
        (LineCurve3D p, LineCurve3D q) => new Vector3D(p.Position.Direction).Crossed(new Vector3D(q.Position.Direction)).Magnitude < SurelyAtAnAngle,
        (CircleCurve3D p, CircleCurve3D q) => Math.Abs(p.Radius - q.Radius) < SurelyApart
            && (p.Position.Location - q.Position.Location).Magnitude < SurelyApart,
        _ => false,
    };

    /// <summary>Whether a point lies on a face, its boundary included, within a tolerance.</summary>
    internal static bool PointOnFace(Face face, Point3D point, double tolerance = Constants.Tolerance) =>
        face.IsOnFace(point, tolerance);

    /// <summary>BRepCheck_Analyzer's own defaults, serial: for the checks of single faces the modelling layer makes.</summary>
    internal static readonly ValidationOptions SerialCheck = new() { Parallel = false };

    /// <summary>One shell from faces: a single face held in a shell of its own, or several sewn together.</summary>
    /// <exception cref="ArgumentException">No faces, or faces that don't sew into one shell.</exception>
    internal static Shell MakeShell(IEnumerable<Face> faces, double tolerance = Constants.Tolerance)
    {
        var list = faces.ToList();
        if (list.Count == 0)
            throw new ArgumentException("a shell needs at least one face");
        if (list.Count == 1)
            return Shell.Create([list[0]]);
        var sewn = SewFaces(list.Select(f => (Shape)f), tolerance);
        return sewn.IsShell() ? (Shell)sewn : throw new ArgumentException("the faces do not sew into one connected shell");
    }

    /// <summary>
    /// The direction into a face's material across one of its edges. A valid face keeps its material on the left of
    /// every wire as it walks it, so the normal crossed with the edge's tangent (as the face walks the edge) points in.
    /// </summary>
    /// <exception cref="ArgumentException">The edge isn't an edge of the face.</exception>
    internal static Vector3D MaterialSide(Face face, Edge edge)
    {
        Edge? occurrence = null;
        foreach (var current in face.Explore<Edge>())
        {
            if (current.IsSame(edge))
            {
                occurrence = current;
                break;
            }
            if (occurrence is null && current.IsPartner(edge))
                occurrence = current;
        }
        if (occurrence is not { } found)
            throw new ArgumentException("the edge is not an edge of the face");
        var curve = found.ToCurve();
        var d = curve.Derivatives((curve.Range.Min + curve.Range.Max) / 2, 1);
        var tangent = d.D1;
        if (found.Orientation == Orientation.Reversed)
            tangent = tangent.Reversed();
        return FaceNormalAt(face, d.Point).Cross(tangent.Normalized());
    }

    /// <summary>
    /// The shape extruded along <paramref name="direction"/>: a vertex makes an edge, an edge a face, a wire a shell,
    /// a face a solid and a shell a compound of solids.
    /// </summary>
    internal static Shape Extrude(Shape obj, Vector3D direction)
    {
        if (!(obj.IsVertex() || obj.IsEdge() || obj.IsWire() || obj.IsFace() || obj.IsShell()))
            throw new ArgumentException($"extrude not supported for {TypeName(obj)}");
        var vec = direction;
        var extrusion = Downcast(Failures.Get(obj.Extrude(vec), () => "extrude").Shape);
        if (extrusion.Type == ShapeType.CompSolid)
            extrusion = MakeCompound(Explore(extrusion, ShapeType.Solid));
        return extrusion;
    }

    /// <summary>
    /// A loft through wires, optionally starting or ending with a vertex (but with no vertex between wires).
    /// </summary>
    internal static Shape MakeLoft(IEnumerable<Shape> objs, bool filled, bool ruled = false)
    {
        var list = objs.ToList();
        if (list.Count < 2)
            throw new ArgumentException("More than one wire is required");
        int vertexCount = list.Count(o => o.IsVertex());
        if (vertexCount > 2)
            throw new ArgumentException("Only two vertices are allowed");
        if (vertexCount == 1 && !(list[0].IsVertex() || list[^1].IsVertex()))
            throw new ArgumentException("The vertex must be either at the beginning or end of the list");
        if (vertexCount == 2)
        {
            if (list.Count == 2)
                throw new ArgumentException("You can't have only 2 vertices to loft; try adding some wires");
            if (!(list[0].IsVertex() && list[^1].IsVertex()))
                throw new ArgumentException("The vertices must be at the beginning and end of the list");
        }
        // The loft no longer writes pcurves into the sections' edges (the kernel's unit 10H3), so they aren't copied
        var sections = list.Where(o => o.IsVertex() || o.IsWire()).Select(o => o).ToList();
        var loft = Failures.Get(Sweeps.Loft(sections, new LoftOptions { MakeSolid = filled, Ruled = ruled, CheckResult = false }), () => "loft").Shape;
        // Sections in one plane make a "valid" solid of nothing (decision 1927)
        if (filled && Math.Abs(loft.IntegrateVolume()) <= 1e-10 * Math.Pow(loft.IntegrateArea(), 1.5))
            throw Failures.Expected(Sweeps.NoVolume, "The loft encloses no volume: its sections lie in one plane.", shapes: loft);
        return loft;
    }

    /// <summary>A planar face from an outer wire and holes.</summary>
    /// <exception cref="ArgumentException">A wire isn't closed, the wires aren't coplanar, or OCCT can't build the face.</exception>
    /// <param name="outerWire">The outer wire.</param>
    /// <param name="innerWires">The holes.</param>
    /// <param name="checkResult">Whether a face that isn't valid fails: not for a face only used to reach the wire's corners or
    /// its area, which may be a hole's wire, or open.</param>
    internal static Face MakeFaceFromWires(Wire outerWire, IEnumerable<Wire>? innerWires = null, bool checkResult = true)
    {
        var inner = innerWires?.ToList() ?? [];
        if (inner.Count > 0 && !outerWire.IsClosed)
            throw new ArgumentException("Cannot build face(s): outer wire is not closed");
        var verification = MakeCompound([outerWire, .. inner]);
        if (!verification.TryFindSurface(-1, out _, onlyPlane: true))
            throw new ArgumentException("Cannot build face(s): wires not planar");

        var topoWire = (Wire)Fix(outerWire);
        var holes = new List<Wire>();
        foreach (var innerWire in inner)
        {
            if (!innerWire.IsClosed)
                throw new ArgumentException("Cannot build face(s): inner wire is not closed");
            holes.Add((Wire)Fix(innerWire));
        }
        // The fix below checks the face
        var made = Face.FromWires(topoWire, holes, onlyPlane: true, checkResult: false);
        if (!made.TryGetValue(out var face))
            throw new ArgumentException($"Cannot build face(s): {made.Error!.Code.Value}");
        return (Face)Failures.Get(face.FixFace(checkResult), () => "fix face").Shape;
    }

    /// <summary>
    /// A face on the (untrimmed) surface of <paramref name="surfaceFace"/> bounded by wires lying on it. The face is
    /// fixed afterwards, which supplies the seam of a periodic surface the wires wrap around.
    /// </summary>
    internal static Face MakeFaceOnSurface(Face surfaceFace, Wire outerWire, IEnumerable<Wire>? innerWires = null)
    {
        surfaceFace.TryGetSurface(out var surface);
        var made = Face.FromSurface(surface!, boundary: outerWire, holes: innerWires ?? [], checkResult: false);
        if (!made.TryGetValue(out var built))
            throw new ArgumentException($"Cannot build face(s): {made.Error!.Code.Value}");
        var fixedShape = Fix(built);
        if (fixedShape.Type != ShapeType.Face)
            throw new InvalidOperationException("Cannot build face(s): the wires do not bound one face");
        var result = (Face)fixedShape;
        if (!result.Validate(SerialCheck).IsValid)
            throw new InvalidOperationException("Cannot build face(s): the wires do not lie on the surface");
        return result;
    }

    /// <summary>The kernel type of a shape, looking into a compound whose children are all of one type.</summary>
    internal static ShapeType UnwrappedShapeType(Shape obj)
    {
        if (obj.IsCompound() && obj.TryAs(out Compound compound))
        {
            var types = compound.ChildShapes().Select(c => ShapeTypeOf(c)).Distinct().ToList();
            if (types.Count == 1)
                return types[0];
        }
        return ShapeTypeOf(obj);
    }
}

/// <summary>Small helpers from build123d's topology utils that are part of its public API.</summary>
internal static partial class TopologyUtils
{
    /// <summary>Cartesian coordinates of a polar point (angle in degrees).</summary>
    public static (double X, double Y) Polar(double length, double angle) =>
        (length * Math.Cos(angle * Constants.DegToRad), length * Math.Sin(angle * Constants.DegToRad));

    /// <summary>Whether two numbers are close: <c>math.isclose</c> with an absolute tolerance of 1e-14 by default.</summary>
    public static bool IsCloseB(double x, double y, double relTol = 1e-9, double absTol = 1e-14) =>
        x == y || Math.Abs(x - y) <= Math.Max(relTol * Math.Max(Math.Abs(x), Math.Abs(y)), absTol);

    /// <summary>The shapes of either list whose OCCT shape isn't in both.</summary>
    public static List<Shape> Delta(IEnumerable<Shape> shapesOne, IEnumerable<Shape> shapesTwo)
    {
        var one = shapesOne.ToList();
        var two = shapesTwo.ToList();
        var twoSet = two.ToHashSet(ShapeComparer.Same);
        var onlyOne = one.Where(s => !twoSet.Contains(s)).ToHashSet(ShapeComparer.Same);
        return one.Concat(two).Where(onlyOne.Contains).ToList();
    }

    /// <summary>The diagonal of the bounding box of one or more shapes.</summary>
    public static double FindMaxDimension(IEnumerable<Shape> shapes)
    {
        var composite = TopoUtil.MakeCompound(shapes.Select(s => s));
        return composite.Bounds().Diagonal;
    }

    /// <summary>The edges of <paramref name="combined"/> that none of <paramref name="objects"/> had.</summary>
    public static ShapeList<Edge> NewEdges(IEnumerable<Shape> objects, Shape combined)
    {
        var combinedEdges = combined.Edges().Where(e => !e.IsEmpty).Select(e => e).ToList();
        var originalEdges = objects.SelectMany(o => o.Edges()).Where(e => !e.IsEmpty).Select(e => e).ToList();
        if (combinedEdges.Count == 0)
            return [];
        // Non-destructive, as all the modelling layer's Booleans: the inputs' tolerances are left alone
        var result = originalEdges.Count == 0
            ? TopoUtil.MakeCompound(combinedEdges.Select(e => (Shape)e))
            : Failures.Get(TopoUtil.Boolean(BooleanKind.Cut, [.. combinedEdges.Select(e => (Shape)e)], [.. originalEdges.Select(e => (Shape)e)]), () => "cut").Shape;
        var edges = new List<Edge>();
        foreach (var edge in TopoUtil.Explore(result, ShapeType.Edge))
            edges.Add((Edge)(edge));
        return new ShapeList<Edge>(edges);
    }
}

internal static partial class TopologyUtils
{
    /// <summary>
    /// A key function giving each shape's topological distance from <paramref name="other"/>, for
    /// <c>OrderBy</c> and <c>GroupBy</c>. Distances are measured over the
    /// whole topology of <paramref name="parent"/>: the references are 0, shapes
    /// connected to them 1, and so on; unreachable shapes are infinitely far.
    /// </summary>
    /// <remarks>
    /// The shapes must all be of one kind: vertices (connected by edges), edges or wires (by vertices), faces (by edges),
    /// or shells or solids (by faces).
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// No references, mixed or unsupported kinds, or sources not in <paramref name="parent"/>.
    /// </exception>
    public static Func<T, double> TopoDistanceTo<T>(Shape parent, params IEnumerable<T> other) where T : IShape
    {
        var distance = TopoDistanceTo(parent, other.Select(s => s.AsShape()));
        return s => distance(s.AsShape());
    }

    /// <summary>
    /// A key function giving each shape's topological distance from <paramref name="other"/> (see the other form).
    /// </summary>
    public static Func<Shape, double> TopoDistanceTo(Shape parent, IEnumerable<Shape> other)
    {
        var sources = other.ToList();
        if (sources.Count == 0)
            throw new ArgumentException("Cannot measure topological distance to an empty object");
        var peerType = sources[0].Type;
        if (sources.Any(s => s.Type != peerType))
            throw new ArgumentException("Topological distance requires shapes of the same type");
        ShapeType? connector = peerType switch
        {
            ShapeType.Vertex => null,
            ShapeType.Edge or ShapeType.Wire => ShapeType.Vertex,
            ShapeType.Face => ShapeType.Edge,
            ShapeType.Shell or ShapeType.Solid => ShapeType.Face,
            _ => throw new ArgumentException($"Topological distance is not supported for {peerType}"),
        };
        IEnumerable<Shape> peers = peerType switch
        {
            ShapeType.Vertex => parent.Vertices().Upcast(),
            ShapeType.Edge => parent.Edges().Upcast(),
            ShapeType.Wire => parent.Wires().Upcast(),
            ShapeType.Face => parent.Faces().Upcast(),
            ShapeType.Shell => parent.Shells().Upcast(),
            _ => parent.Solids().Upcast(),
        };
        var peerList = peers.Where(p => !p.IsEmpty).ToList();
        // Shapes are equal (and hash) by IsSame, as build123d's hasher lookup is
        var lookup = new Dictionary<Shape, Shape>(ShapeComparer.Same);
        foreach (var p in peerList)
            lookup.TryAdd(p, p);
        var peerTopo = peerList.Select(p => p).ToList();
        if (sources.Any(s => !lookup.ContainsKey(s) && TopoUtil.FindSame(s, peerTopo) is null))
            throw new ArgumentException("Topological distance needs shapes selected from the parent shape.");
        Shape? PeerOf(Shape shape)
        {
            if (shape.IsEmpty)
                return null;
            if (lookup.TryGetValue(shape, out var peer))
                return peer;
            // Allow for the shape having been moved
            return TopoUtil.FindSame(shape, peerTopo) is { } match ? lookup[(match)] : null;
        }

        var neighbours = peerList.Distinct(ShapeComparer.Same).ToDictionary(p => p, _ => new HashSet<Shape>(ShapeComparer.Same), ShapeComparer.Same);
        {
            var graph = ShapeGraph.Build(parent);
            if (connector is null)
            {
                // Vertices are neighbours across an edge
                foreach (var key in graph.All(ShapeType.Vertex))
                {
                    if (!lookup.TryGetValue((key), out var vertexPeer))
                        continue;
                    foreach (var e in graph.Ancestors(key, ShapeType.Edge))
                    {
                        var edge = (Edge)e;
                        foreach (var end in new[] { edge.FirstVertex, edge.LastVertex })
                            if (lookup.TryGetValue((end), out var n) && n != vertexPeer)
                                neighbours[vertexPeer].Add(n);
                    }
                }
            }
            else
            {
                // Peers are neighbours across a shared connector
                foreach (var key in graph.All(connector.Value))
                {
                    var list = graph.Ancestors(key, (peerType));
                    var connected = list.Select(s => lookup.GetValueOrDefault((s))).OfType<Shape>().ToList();
                    foreach (var a in connected)
                        foreach (var b in connected)
                            if (a != b)
                                neighbours[a].Add(b);
                }
            }
        }

        // Breadth-first from the references
        var distances = new Dictionary<Shape, int>(ShapeComparer.Same);
        var frontier = new Queue<Shape>();
        foreach (var source in sources)
            if (PeerOf(source) is { } peer && distances.TryAdd(peer, 0))
                frontier.Enqueue(peer);
        while (frontier.Count > 0)
        {
            var current = frontier.Dequeue();
            foreach (var n in neighbours[current])
                if (distances.TryAdd(n, distances[current] + 1))
                    frontier.Enqueue(n);
        }

        return obj =>
        {
            if (obj.Type != peerType)
                throw new ArgumentException("Topological distance requires shapes of the same type");
            if (PeerOf(obj) is not { } peer)
                throw new ArgumentException("Topological distance needs shapes selected from the parent shape.");
            return distances.TryGetValue(peer, out var d) ? d : double.PositiveInfinity;
        };
    }
}
