using Occt;

namespace Build123d;

// The face, shell and extrusion helpers of build123d's topology/utils.py
internal static partial class TopoUtil
{
    /// <summary>The point of a face at normalised (u, v), each from 0 to 1 across the face's own parameter range.</summary>
    internal static Vector FacePosition(TopoDS_Face face, double u, double v)
    {
        double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
        BRepTools.UVBounds(face, out uMin, out uMax, out vMin, out vMax);
        var surface = BRep_Tool.Surface(face)!;
        var p = surface.Value(uMin + u * (uMax - uMin), vMin + v * (vMax - vMin));
        return new Vector(p);
    }

    /// <summary>
    /// The unit normal of a face at normalised (u, v), with the face's own orientation: reversed faces point the
    /// other way from their surface.
    /// </summary>
    internal static Vector FaceNormal(TopoDS_Face face, double u = 0.5, double v = 0.5)
    {
        double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
        BRepTools.UVBounds(face, out uMin, out uMax, out vMin, out vMax);
        var point = new gp_Pnt();
        var normal = new gp_Vec();
        var props = new BRepGProp_Face(face);
        props.Normal(uMin + u * (uMax - uMin), vMin + v * (vMax - vMin), out point, out normal);
        return new Vector(normal).Normalized();
    }

    /// <summary>The centre of mass of a face's area.</summary>
    internal static Vector FaceCenter(TopoDS_Face face)
    {
        var props = new GProp_GProps();
        BRepGProp.SurfaceProperties(face, props);
        var c = props.CentreOfMass();
        return new Vector(c);
    }

    /// <summary>Whether a point lies on a face, its boundary included, within a tolerance.</summary>
    internal static bool PointOnFace(TopoDS_Face face, Vector point, double tolerance = Constants.Tolerance)
    {
        var classifier = new BRepClass3d_SolidClassifier(face);
        var p = point.ToGpPnt();
        classifier.Perform(p, tolerance);
        return classifier.IsOnAFace();
    }

    /// <summary>One shell from faces: a single face held in a shell of its own, or several sewn together.</summary>
    /// <exception cref="ArgumentException">No faces, or faces that don't sew into one shell.</exception>
    internal static TopoDS_Shell MakeShell(IEnumerable<TopoDS_Face> faces, double tolerance = Constants.Tolerance)
    {
        var list = faces.ToList();
        if (list.Count == 0)
            throw new ArgumentException("a shell needs at least one face");
        if (list.Count == 1)
            return TopoDS_Builder.MakeShell([list[0]]);
        var sewn = SewFaces(list.Select(f => (TopoDS_Shape)f), tolerance);
        return sewn.IsShell() ? TopoDS.Shell(sewn) : throw new ArgumentException("the faces do not sew into one connected shell");
    }

    /// <summary>
    /// The direction into a face's material across one of its edges. A valid face keeps its material on the left of
    /// every wire as it walks it, so the normal crossed with the edge's tangent (as the face walks the edge) points in.
    /// </summary>
    /// <exception cref="ArgumentException">The edge isn't an edge of the face.</exception>
    internal static Vector MaterialSide(TopoDS_Face face, TopoDS_Edge edge)
    {
        TopoDS_Edge? occurrence = null;
        var explorer = new TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_EDGE);
        for (; explorer.More(); explorer.Next())
        {
            var current = explorer.Current();
            if (current.IsSame(edge))
            {
                occurrence = TopoDS.Edge(current);
                break;
            }
            if (occurrence is null && current.IsPartner(edge))
                occurrence = TopoDS.Edge(current);
        }
        if (occurrence is not { } found)
            throw new ArgumentException("the edge is not an edge of the face");
        var curve = new BRepAdaptor_Curve(found);
        curve.D1((curve.FirstParameter() + curve.LastParameter()) / 2, out var point, out var tangent);
        if (found.Orientation() == TopAbs_Orientation.TopAbs_REVERSED)
            tangent = tangent.Reversed();
        return FaceNormalAt(face, point).Cross(new Vector(tangent).Normalized());
    }

    /// <summary>
    /// The shape extruded along <paramref name="direction"/>: a vertex makes an edge, an edge a face, a wire a shell,
    /// a face a solid and a shell a compound of solids.
    /// </summary>
    internal static TopoDS_Shape Extrude(TopoDS_Shape obj, Vector direction)
    {
        if (!(obj.IsVertex() || obj.IsEdge() || obj.IsWire() || obj.IsFace() || obj.IsShell()))
            throw new ArgumentException($"extrude not supported for {TypeName(obj)}");
        var vec = direction.ToGpVec();
        var prism = new BRepPrimAPI_MakePrism(obj, vec);
        var extrusion = Downcast(prism.Shape());
        if (extrusion.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPSOLID)
        {
            var solids = new List<TopoDS_Shape>();
            var explorer = new TopExp_Explorer(extrusion, TopAbs_ShapeEnum.TopAbs_SOLID);
            for (; explorer.More(); explorer.Next())
                solids.Add(Downcast(explorer.Current()));
            extrusion = MakeCompound(solids);
        }
        return extrusion;
    }

    /// <summary>
    /// A loft through wires, optionally starting or ending with a vertex (but with no vertex between wires).
    /// </summary>
    internal static TopoDS_Shape MakeLoft(IEnumerable<Shape> objs, bool filled, bool ruled = false)
    {
        var list = objs.ToList();
        if (list.Count < 2)
            throw new ArgumentException("More than one wire is required");
        int vertexCount = list.Count(o => o.TopoShape.IsVertex());
        if (vertexCount > 2)
            throw new ArgumentException("Only two vertices are allowed");
        if (vertexCount == 1 && !(list[0].TopoShape.IsVertex() || list[^1].TopoShape.IsVertex()))
            throw new ArgumentException("The vertex must be either at the beginning or end of the list");
        if (vertexCount == 2)
        {
            if (list.Count == 2)
                throw new ArgumentException("You can't have only 2 vertices to loft; try adding some wires");
            if (!(list[0].TopoShape.IsVertex() && list[^1].TopoShape.IsVertex()))
                throw new ArgumentException("The vertices must be at the beginning and end of the list");
        }
        var loft = new BRepOffsetAPI_ThruSections(filled, ruled);
        foreach (var obj in list)
        {
            if (obj.TopoShape.IsVertex())
                loft.AddVertex(TopoDS.Vertex(obj.TopoShape));
            else if (obj.TopoShape.IsWire())
                loft.AddWire(TopoDS.Wire(PrivateCopy(obj.TopoShape)));   // the loft writes pcurves into the sections' edges
        }
        loft.Build();
        return loft.Shape();
    }

    /// <summary>A planar face from an outer wire and holes.</summary>
    /// <exception cref="ArgumentException">A wire isn't closed, the wires aren't coplanar, or OCCT can't build the face.</exception>
    internal static TopoDS_Face MakeFaceFromWires(TopoDS_Wire outerWire, IEnumerable<TopoDS_Wire>? innerWires = null)
    {
        var inner = innerWires?.ToList() ?? [];
        if (inner.Count > 0 && !BRep_Tool.IsClosed(outerWire))
            throw new ArgumentException("Cannot build face(s): outer wire is not closed");
        var verification = MakeCompound([outerWire, .. inner]);
        var finder = new BRepLib_FindSurface(verification, -1, true);
        if (!finder.Found())
            throw new ArgumentException("Cannot build face(s): wires not planar");

        var fixOuter = new ShapeFix_Shape(outerWire);
        fixOuter.Perform();
        var topoWire = TopoDS.Wire(fixOuter.Shape());
        var builder = new BRepBuilderAPI_MakeFace(topoWire, true);
        foreach (var innerWire in inner)
        {
            if (!BRep_Tool.IsClosed(innerWire))
                throw new ArgumentException("Cannot build face(s): inner wire is not closed");
            var fixInner = new ShapeFix_Shape(innerWire);
            fixInner.Perform();
            var fixedInner = TopoDS.Wire(fixInner.Shape());
            builder.Add(fixedInner);
        }
        builder.Build();
        if (!builder.IsDone())
            throw new ArgumentException($"Cannot build face(s): {builder.Error()}");
        var face = builder.Face();
        var fixFace = new ShapeFix_Face(face);
        fixFace.FixOrientation();
        fixFace.Perform();
        return TopoDS.Face(fixFace.Result());
    }

    /// <summary>
    /// A face on the (untrimmed) surface of <paramref name="surfaceFace"/> bounded by wires lying on it. The face is
    /// fixed afterwards, which supplies the seam of a periodic surface the wires wrap around.
    /// </summary>
    internal static TopoDS_Face MakeFaceOnSurface(TopoDS_Face surfaceFace, TopoDS_Wire outerWire, IEnumerable<TopoDS_Wire>? innerWires = null)
    {
        var surface = BRep_Tool.Surface(surfaceFace);
        var builder = new BRepBuilderAPI_MakeFace(surface!, outerWire, true);
        foreach (var innerWire in innerWires ?? [])
            builder.Add(innerWire);
        if (!builder.IsDone())
            throw new ArgumentException($"Cannot build face(s): {builder.Error()}");
        var built = builder.Face();
        var fix = new ShapeFix_Shape(built);
        fix.Perform();
        var fixedShape = fix.Shape();
        if (fixedShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_FACE)
            throw new InvalidOperationException("Cannot build face(s): the wires do not bound one face");
        var result = TopoDS.Face(fixedShape);
        var check = new BRepCheck_Analyzer(result);
        if (!check.IsValid())
            throw new InvalidOperationException("Cannot build face(s): the wires do not lie on the surface");
        return result;
    }

    /// <summary>The kernel type of a shape, looking into a compound whose children are all of one type.</summary>
    internal static TopAbs_ShapeEnum UnwrappedShapeType(Shape obj)
    {
        if (obj.TopoShape.IsCompound() && obj is Compound compound)
        {
            var types = compound.Children().Select(c => ShapeTypeOf(c.TopoShape)).Distinct().ToList();
            if (types.Count == 1)
                return types[0];
        }
        return ShapeTypeOf(obj.TopoShape);
    }
}

/// <summary>Small helpers from build123d's topology utils that are part of its public API.</summary>
public static partial class TopologyUtils
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
        var twoSet = two.ToHashSet();
        var onlyOne = one.Where(s => !twoSet.Contains(s)).ToHashSet();
        return one.Concat(two).Where(onlyOne.Contains).ToList();
    }

    /// <summary>The diagonal of the bounding box of one or more shapes.</summary>
    public static double FindMaxDimension(IEnumerable<Shape> shapes)
    {
        var composite = TopoUtil.MakeCompound(shapes.Select(s => s.TopoShape));
        return BoundBox.FromTopoDS(composite, Constants.Tolerance, optimal: true).Diagonal;
    }

    /// <summary>The edges of <paramref name="combined"/> that none of <paramref name="objects"/> had.</summary>
    public static ShapeList<Edge> NewEdges(IEnumerable<Shape> objects, Shape combined)
    {
        var combinedEdges = TopoUtil.ListOf(combined.Edges().Where(e => !e.IsEmpty).Select(e => e.TopoShape));
        var originalEdges = TopoUtil.ListOf(objects.SelectMany(o => o.Edges()).Where(e => !e.IsEmpty).Select(e => e.TopoShape));
        var operation = new BRepAlgoAPI_Cut();
        operation.SetNonDestructive(true);   // leave the inputs' tolerances alone
        operation.SetArguments(combinedEdges);
        operation.SetTools(originalEdges);
        operation.SetRunParallel(true);
        operation.Build();
        var edges = new List<Edge>();
        var result = operation.Shape();
        var explorer = new TopExp_Explorer(result, TopAbs_ShapeEnum.TopAbs_EDGE);
        for (; explorer.More(); explorer.Next())
            edges.Add((Edge)combined.CastLike(TopoUtil.Downcast(explorer.Current())).WithTopoPath([combined]));
        return new ShapeList<Edge>(edges);
    }
}

public static partial class TopologyUtils
{
    /// <summary>
    /// A key function giving each shape's topological distance from <paramref name="other"/>, for
    /// <see cref="ShapeList{T}.SortBy{TKey}(Func{T, TKey}, bool)"/> and <c>GroupBy</c>. Distances are measured over the
    /// whole topology of the reference shapes' shared <see cref="Shape.TopoParent"/>: the references are 0, shapes
    /// connected to them 1, and so on; unreachable shapes are infinitely far.
    /// </summary>
    /// <remarks>
    /// The shapes must all be of one kind: vertices (connected by edges), edges or wires (by vertices), faces (by edges),
    /// or shells or solids (by faces).
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// No references, mixed or unsupported kinds, or shapes without one shared <see cref="Shape.TopoParent"/>.
    /// </exception>
    public static Func<Shape, double> TopoDistanceTo(params IEnumerable<Shape> other)
    {
        var sources = other.ToList();
        if (sources.Count == 0)
            throw new ArgumentException("Cannot measure topological distance to an empty object");
        var peerType = sources[0].ShapeType;
        if (sources.Any(s => s.ShapeType != peerType))
            throw new ArgumentException("Topological distance requires shapes of the same type");
        TopAbs_ShapeEnum? connector = peerType switch
        {
            ShapeType.Vertex => null,
            ShapeType.Edge or ShapeType.Wire => TopAbs_ShapeEnum.TopAbs_VERTEX,
            ShapeType.Face => TopAbs_ShapeEnum.TopAbs_EDGE,
            ShapeType.Shell or ShapeType.Solid => TopAbs_ShapeEnum.TopAbs_FACE,
            _ => throw new ArgumentException($"Topological distance is not supported for {peerType}"),
        };
        var parents = sources.Select(s => s.TopoParent).ToList();
        if (parents.Any(p => p is null))
            throw new ArgumentException("Topological distance requires shapes with a topo_parent");
        var parent = parents[0]!;
        if (parents.Skip(1).Any(p => !parent.IsSame(p!)))
            throw new ArgumentException("Topological distance requires a shared topo_parent");

        IEnumerable<Shape> peers = peerType switch
        {
            ShapeType.Vertex => parent.Vertices(),
            ShapeType.Edge => parent.Edges(),
            ShapeType.Wire => parent.Wires(),
            ShapeType.Face => parent.Faces(),
            ShapeType.Shell => parent.Shells(),
            _ => parent.Solids(),
        };
        var peerList = peers.Where(p => !p.IsEmpty).ToList();
        // Shapes are equal (and hash) by IsSame, as build123d's hasher lookup is
        var lookup = new Dictionary<Shape, Shape>();
        foreach (var p in peerList)
            lookup.TryAdd(p, p);
        var peerTopo = peerList.Select(p => p.TopoShape).ToList();
        Shape? PeerOf(Shape shape)
        {
            if (shape.IsEmpty)
                return null;
            if (lookup.TryGetValue(shape, out var peer))
                return peer;
            // Allow for the shape having been moved
            return TopoUtil.FindSame(shape.TopoShape, peerTopo) is { } match ? lookup[Shape.Cast(match)] : null;
        }

        var neighbours = peerList.Distinct().ToDictionary(p => p, _ => new HashSet<Shape>());
        {
            var map = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            if (connector is null)
            {
                // Vertices are neighbours across an edge
                TopExp.MapShapesAndAncestors(parent.TopoShape, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, map);
                for (int i = 1; i <= map.Extent(); i++)
                {
                    if (!lookup.TryGetValue(Shape.Cast(map.FindKey(i)), out var vertexPeer))
                        continue;
                    var edges = map.FindFromIndex(i);
                    foreach (var e in edges)
                    {
                        var edge = TopoDS.Edge(e);
                        foreach (var end in new[] { TopExp.FirstVertex(edge), TopExp.LastVertex(edge) })
                            if (lookup.TryGetValue(Shape.Cast(end), out var n) && n != vertexPeer)
                                neighbours[vertexPeer].Add(n);
                    }
                }
            }
            else
            {
                // Peers are neighbours across a shared connector
                TopExp.MapShapesAndAncestors(parent.TopoShape, connector.Value, TopoUtil.ToTopAbs(peerType), map);
                for (int i = 1; i <= map.Extent(); i++)
                {
                    var list = map.FindFromIndex(i);
                    var connected = list.Select(s => lookup.GetValueOrDefault(Shape.Cast(s))).OfType<Shape>().ToList();
                    foreach (var a in connected)
                        foreach (var b in connected)
                            if (a != b)
                                neighbours[a].Add(b);
                }
            }
        }

        // Breadth-first from the references
        var distances = new Dictionary<Shape, int>();
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
            if (obj.ShapeType != peerType)
                throw new ArgumentException("Topological distance requires shapes of the same type");
            if (obj.TopoParent is null)
                throw new ArgumentException("Topological distance requires shapes with a topo_parent");
            if (!parent.IsSame(obj.TopoParent))
                throw new ArgumentException("Topological distance requires a shared topo_parent");
            return PeerOf(obj) is { } peer && distances.TryGetValue(peer, out var d) ? d : double.PositiveInfinity;
        };
    }
}
