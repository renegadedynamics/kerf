using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Build123d;

/// <summary>The methods shared by solids and compounds (build123d's <c>Mixin3D</c>).</summary>
public abstract partial class Shape3D : Shape
{
    private protected Shape3D(global::Kerf.Topology.Shape? obj, string label = "", Color? color = null) : base(obj, label, color) { }

    internal override int? Dim => 3;

    /// <summary>A 3D operation's result as topology rather than as the source's class: a Solid, or a Part of several.</summary>
    internal static Shape Make3DResult(global::Kerf.Topology.Shape shape)
    {
        var result = TopoUtil.Downcast(shape);
        if (result.IsCompound())
            result = TopoUtil.Downcast(TopoUtil.UnwrapCompound(ShapeCasts.Compound(result)));
        if (result.IsCompound())
            return MakeComposite(TopoUtil.TopLevelShapes(result).Select(s => new Solid(ShapeCasts.Solid(s))), 3);
        return new Solid(ShapeCasts.Solid(result));
    }

    /// <summary>The centre of mass (by default) or of the bounding box; the geometric centre isn't supported.</summary>
    public override Vector Center() => Center(CenterOf.Mass);

    /// <inheritdoc cref="Center()"/>
    public override Vector Center(CenterOf centerOf)
    {
        NeedsGeometry("center");
        switch (centerOf)
        {
            case CenterOf.Geometry:
                throw new ArgumentException("Center of GEOMETRY is not supported for this object");
            case CenterOf.Mass:
            {
                var props = new Properties();
                MassProperties(TopoShape, props);
                var c = props.CentreOfMass();
                return new Vector(c);
            }
            default:
                return BoundingBox().Center();
        }
    }

    /// <summary>
    /// The edges of this shape chamfered by <paramref name="length"/> (and <paramref name="length2"/> on the other side, for an
    /// asymmetric chamfer), measured on <paramref name="face"/> if given (which the edges must belong to).
    /// </summary>
    /// <exception cref="ArgumentException">An edge isn't on the face, or the chamfer can't be made.</exception>
    public Shape Chamfer(double length, double? length2, IEnumerable<Edge> edgeList, Face? face = null)
    {
        var edges = edgeList.ToList();
        if (face is not null)
        {
            var faceEdges = face.Edges();
            if (edges.Any(e => !faceEdges.Contains(e)))
                throw new ArgumentException("Some edges are not part of the face");
        }
        var map = new NCollection_IndexedDataMap<global::Kerf.Topology.Shape, NCollection_List<global::Kerf.Topology.Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(TopoShape, global::Kerf.Topology.ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, map);
        var builder = new BRepFilletAPI_MakeChamfer(TopoShape);
        var distance2 = length2 is { } l2 && l2 != 0 ? l2 : length;
        foreach (var edge in edges)
        {
            global::Kerf.Topology.Face topoFace;
            if (face is not null)
                topoFace = face.TopoFace;
            else
            {
                var ancestors = map.FindFromKey(edge.TopoShape);
                topoFace = ShapeCasts.Face(ancestors.First());
            }
            builder.Add(length, distance2, edge.TopoEdge, topoFace);
        }
        Shape result;
        try
        {
            result = Make3DResult(builder.Shape());
            if (!result.IsValid)
                throw new ArgumentException("Failed creating a chamfer, try a smaller length value(s)");
        }
        catch (FailureException e)
        {
            throw new ArgumentException("Failed creating a chamfer, try a smaller length value(s)", e);
        }
        return result.MadeBy(ShapeHistory.FromAlgorithm(builder, [TopoShape], result.TopoShape));
    }

    /// <summary>
    /// A prismatic feature, added or subtracted, from profiles (faces, or wires grouped into faces) on an optional basis
    /// face: through all, to a <paramref name="depth"/>, or up to a face, with an optional taper in degrees.
    /// </summary>
    public Shape DPrism(Face? basis, IReadOnlyList<Shape2D> bounds, double? depth = null, double taper = 0, Face? upToFace = null,
                        bool thruAll = true, bool additive = true) =>
        DPrismCore(basis, bounds.SelectMany(b => b.Faces()), depth, taper, upToFace, thruAll, additive);

    /// <inheritdoc cref="DPrism(Face?, IReadOnlyList{Shape2D}, double?, double, Face?, bool, bool)"/>
    public Shape DPrism(Face? basis, IReadOnlyList<Wire> bounds, double? depth = null, double taper = 0, Face? upToFace = null,
                        bool thruAll = true, bool additive = true) =>
        DPrismCore(basis, TwoD.SortWiresByBuildOrder(bounds).Select(p => new Face(p[0], p.Skip(1))), depth, taper, upToFace, thruAll, additive);

    private Shape DPrismCore(Face? basis, IEnumerable<Face> faces, double? depth, double taper, Face? upToFace, bool thruAll, bool additive)
    {
        var shape = TopoShape;
        var noBasis = new global::Kerf.Topology.Face();
        foreach (var face in faces)
        {
            var feature = new BRepFeat_MakeDPrism(shape, face.TopoFace, basis?.TopoFace ?? noBasis,
                                                        taper * Constants.DegToRad, additive ? 1 : 0, false);
            if (upToFace is not null)
                feature.Perform(upToFace.TopoShape);
            else if (thruAll || depth is null)
                feature.PerformThruAll();
            else
                feature.Perform(depth.Value);
            shape = feature.Shape();
        }
        return CastLike(shape);
    }

    /// <summary>The edges of this shape filleted with a radius.</summary>
    /// <exception cref="ArgumentException">The fillet can't be made; the message says which edges failed and why.</exception>
    public Shape Fillet(double radius, IEnumerable<Edge> edgeList)
    {
        var edges = edgeList.Select(e => e.TopoEdge).ToList();
        var builder = new BRepFilletAPI_MakeFillet(TopoShape);
        foreach (var edge in edges)
            builder.Add(radius, edge);
        Shape result;
        try
        {
            result = Make3DResult(builder.Shape());
            if (!result.IsValid)
                throw new InvalidOperationException();
        }
        catch (Exception e) when (e is FailureException or InvalidOperationException)
        {
            string details = "";
            try
            {
                details = FilletFailureDetails(builder, edges);
            }
            catch (Exception)
            {
                // Diagnostics mustn't mask the kernel's failure
            }
            throw new ArgumentException(
                $"Failed creating a fillet with radius of {PyFormat.Repr(radius)}{details}, try a smaller value or use "
                + "max_fillet() to find the largest valid fillet radius", e);
        }
        return result.MadeBy(ShapeHistory.FromAlgorithm(builder, [TopoShape], result.TopoShape));
    }

    /// <summary>The faulty contours of a failed fillet, in terms of the caller's selected edges.</summary>
    private static string FilletFailureDetails(BRepFilletAPI_MakeFillet builder, List<global::Kerf.Topology.Edge> edges)
    {
        var faulty = new Dictionary<string, HashSet<Edge>>();
        var order = new List<string>();
        for (int i = 1; i <= builder.NbFaultyContours(); i++)
        {
            var contour = builder.FaultyContour(i);
            // A faulty stripe past NbContours() can lack a spine, which OCCT dereferences unchecked
            if (contour < 1 || contour > builder.NbContours())
                continue;
            var reason = builder.StripeStatus(contour) switch
            {
                ChFiDS_ErrorStatus.ChFiDS_WalkingFailure => "walking failure",
                ChFiDS_ErrorStatus.ChFiDS_StartsolFailure => "no starting solution",
                ChFiDS_ErrorStatus.ChFiDS_TwistedSurface => "twisted surface",
                _ => "error",
            };
            if (!faulty.TryGetValue(reason, out var set))
            {
                faulty[reason] = set = [];
                order.Add(reason);
            }
            for (int j = 1; j <= builder.NbEdges(contour); j++)
                set.Add(new Edge(builder.Edge(contour, j)));
        }
        if (faulty.Count == 0)
            return "";
        var selected = edges.Select(e => new Edge(e)).ToList();
        var rejected = new HashSet<int>();
        var groups = new List<string>();
        foreach (var reason in order)
        {
            var set = faulty[reason];
            var entries = new List<string>();
            for (int i = 0; i < selected.Count; i++)
                if (set.Contains(selected[i]))
                {
                    rejected.Add(i);
                    entries.Add($"[{i}] at {selected[i].Center().ToString(".6g")}");
                }
            var tangentCount = set.Count(e => !selected.Contains(e));
            if (tangentCount > 0)
                entries.Add($"{tangentCount} additional tangent {(tangentCount == 1 ? "edge" : "edges")}");
            groups.Add($"{reason}: {string.Join(", ", entries)}");
        }
        return $" on {rejected.Count} of {edges.Count} edges ({string.Join("; ", groups)})";
    }

    /// <summary>
    /// The largest fillet radius the edges take, found by a binary search to within <paramref name="tolerance"/>.
    /// </summary>
    /// <exception cref="InvalidOperationException">No value was found within <paramref name="maxIterations"/>.</exception>
    /// <exception cref="ArgumentException">The shape isn't valid.</exception>
    public double MaxFillet(IEnumerable<Edge> edgeList, double tolerance = 0.1, int maxIterations = 10)
    {
        if (!IsValid)
            throw new ArgumentException("Invalid Shape");
        var edges = edgeList.Select(e => e.TopoEdge).ToList();
        double Search(double windowMin, double windowMax, int iteration)
        {
            var windowMid = (windowMin + windowMax) / 2;
            if (iteration == maxIterations)
                throw new InvalidOperationException($"Failed to find the max value within {PyFormat.Repr(tolerance)} in {maxIterations}");
            var builder = new BRepFilletAPI_MakeFillet(TopoShape);
            foreach (var edge in edges)
                builder.Add(windowMid, edge);
            // Do these numbers work? If not, try the smaller window
            try
            {
                if (!Make3DResult(builder.Shape()).IsValid)
                    return Search(windowMin, windowMid, iteration + 1);
            }
            catch (FailureException)
            {
                return Search(windowMin, windowMid, iteration + 1);
            }
            // They work: close enough, or try the larger window
            return windowMid - windowMin <= tolerance ? windowMid : Search(windowMid, windowMax, iteration + 1);
        }
        return Search(0.0, 2 * BoundingBox().Diagonal, 0);
    }

    /// <summary>
    /// This shape shelled to a <paramref name="thickness"/> (positive outwards, negative inwards), the given faces removed to
    /// open it; with no faces, a closed hollow solid.
    /// </summary>
    /// <exception cref="ArgumentException">Kind.Tangent isn't supported.</exception>
    /// <exception cref="InvalidOperationException">OCCT can't build the shell.</exception>
    public Shape Hollow(IEnumerable<Face>? faces, double thickness, double tolerance = 0.0001, Kind kind = Kind.Arc)
    {
        var faceList = faces?.ToList() ?? [];
        if (kind == Kind.Tangent)
            throw new ArgumentException("Kind.TANGENT not supported");
        var join = kind == Kind.Arc ? JoinKind.Arc : JoinKind.Intersection;
        var openings = TopoUtil.ListOf(faceList.Select(f => f.TopoShape));
        var builder = new BRepOffsetAPI_MakeThickSolid();
        builder.MakeThickSolidByJoin(TopoShape, openings, thickness, tolerance, BRepOffset_Mode.BRepOffset_Skin, true, false, join);
        builder.Build();
        var shellShape = builder.Shape();
        // OCCT can report success and still hand back a null shape
        if (shellShape.IsNull)
            throw new InvalidOperationException("hollow Error, an alternative kind may resolve this error");
        if (faceList.Count > 0)
            return CastLike(shellShape);
        // With no faces removed, make a watertight solid of the two shells
        var shell1 = CastLike(shellShape).Shells()[0].TopoShell;
        var shell2 = Shells()[0].TopoShell;
        // The first can be the outer or inner shell depending on the thickness's sign
        var solid = thickness > 0 ? new BRepBuilderAPI_MakeSolid(shell1, shell2) : new BRepBuilderAPI_MakeSolid(shell2, shell1);
        return CastLike(solid.Shape()).Fix();
    }

    /// <summary>Whether a point is inside the solid (or on its surface) within a tolerance.</summary>
    public bool IsInside(Vector point, double tolerance = 1.0e-6)
    {
        var classifier = new BRepClass3d_SolidClassifier(TopoShape);
        var p = point.ToGpPnt();
        classifier.Perform(p, tolerance);
        return classifier.State() == Containment.In || classifier.IsOnAFace();
    }

    /// <summary>
    /// This shape offset by <paramref name="thickness"/> (positive outwards), with <paramref name="openings"/> removed.
    /// </summary>
    /// <exception cref="ArgumentException">Kind.Tangent isn't supported.</exception>
    /// <exception cref="InvalidOperationException">OCCT can't build the offset.</exception>
    public Shape Offset3D(IEnumerable<Face>? openings, double thickness, double tolerance = 0.0001, Kind kind = Kind.Arc)
    {
        var faceList = openings?.ToList() ?? [];
        if (kind == Kind.Tangent)
            throw new ArgumentException("Kind.TANGENT not supported");
        var join = kind == Kind.Arc ? JoinKind.Arc : JoinKind.Intersection;
        var faces = TopoUtil.ListOf(faceList.Select(f => f.TopoShape));
        var builder = new BRepOffsetAPI_MakeThickSolid();
        builder.MakeThickSolidByJoin(TopoShape, faces, thickness, tolerance, BRepOffset_Mode.BRepOffset_Skin, true, false, join, true);
        builder.Build();
        global::Kerf.Topology.Shape solid;
        try
        {
            solid = builder.Shape();
        }
        catch (FailureException e)
        {
            throw new InvalidOperationException("offset Error, an alternative kind may resolve this error", e);
        }
        if (solid.IsNull)
            throw new InvalidOperationException("offset Error, an alternative kind may resolve this error");
        if (solid.ShapeType() == global::Kerf.Topology.ShapeType.Solid)
            solid = ThreeD.ForwardSolid(ShapeCasts.Solid(solid));
        var result = CastLike(solid);
        // The solid can be inverted: if so, reverse it
        return result.Volume < 0 ? result.Rewrapped(result.TopoShape.Reversed()) : result;
    }

    internal override ShapeList<Shape>? IntersectCore(object other, double tolerance, bool includeTouched)
    {
        var shape = AsShape(other);
        var results = new List<Shape>();
        // Trim infinite edges before OCCT operations
        if (shape is Edge { IsInfinite: true } infinite)
        {
            var box = BoundingBox(optimal: false);
            shape = infinite.TrimInfinite(box.Diagonal + (infinite.Center() - box.Center()).Length);
        }
        switch (shape)
        {
            case Solid or Face or Shell or Edge or Wire:
            {
                var common = new BRepAlgoAPI_Common();
                results.AddRange(BoolOpList([this], [shape], common).Expand());
                break;
            }
            case Vertex vertex:
                if (IsInside(vertex.ToVector(), tolerance))
                    results.Add(vertex);
                break;
            default:
                // Delegate to higher-order shapes (a compound); the outer caller handles touches
                if (shape.IntersectCore(this, tolerance, includeTouched: false) is { } found)
                    results.AddRange(found);
                break;
        }
        if (includeTouched && this is Solid solid)
        {
            results.AddRange(solid.Touch(shape, tolerance));
            results = FilterRedundantTouches(results.Distinct().ToList(), tolerance);
        }
        return results.Count > 0 ? new ShapeList<Shape>(results) : null;
    }

    /// <summary>The results without vertices and edges that lie on higher-dimensional ones.</summary>
    private static List<Shape> FilterRedundantTouches(List<Shape> items, double tolerance)
    {
        var edges = items.OfType<Edge>().ToList();
        var faces = items.OfType<Face>().ToList();
        var solids = items.OfType<Solid>().ToList();
        return items.Where(r =>
            !(r is Vertex v && (edges.Any(e => e.DistanceTo(v) <= tolerance) || faces.Any(f => f.DistanceTo(v) <= tolerance)
                                || solids.SelectMany(s => s.Faces()).Any(sf => sf.DistanceTo(v) <= tolerance)))
            && !(r is Edge e2 && faces.Any(f => f.DistanceTo(e2.Center()) <= tolerance))).ToList();
    }
}

/// <summary>The module-level functions of build123d's topology/three_d.py.</summary>
internal static class ThreeD
{
    /// <summary>
    /// The same solid with FORWARD flags on itself and its shells. <c>BRepOffsetAPI_MakeThickSolid</c> can return a solid
    /// whose faces point inward under a REVERSED flag on the solid, which <c>BRepFilletAPI_MakeFillet</c> misreads as an
    /// inside-out solid; rebuilding it FORWARD, each face carrying the orientation it had in effect, keeps the geometry.
    /// </summary>
    internal static global::Kerf.Topology.Shape ForwardSolid(global::Kerf.Topology.Solid solid)
    {
        var flags = new List<Orientation> { solid.Orientation() };
        var shells = new Iterator(solid, true, true);
        for (; shells.More(); shells.Next())
            flags.Add(shells.Value().Orientation());
        if (flags.All(f => f == Orientation.Forward))
            return solid;
        var forwardShells = new List<global::Kerf.Topology.Shape>();
        for (shells = new Iterator(solid, true, true); shells.More(); shells.Next())   // orientations composed
        {
            var faces = new List<global::Kerf.Topology.Shape>();
            for (var it = new Iterator(shells.Value(), true, true); it.More(); it.Next())
                faces.Add(it.Value());
            forwardShells.Add(Builder.MakeShell(faces));
        }
        return Builder.MakeSolid(forwardShells);
    }
}
