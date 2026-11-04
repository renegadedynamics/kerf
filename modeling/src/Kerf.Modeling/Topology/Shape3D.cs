using Kerf.Construction;
using Kerf.Features;
using Kerf.Fillets;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Modeling;

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
            result = TopoUtil.Downcast(TopoUtil.UnwrapCompound((global::Kerf.Topology.Compound)result));
        if (result.IsCompound())
            return MakeComposite(TopoUtil.TopLevelShapes(result).Select(s => new Solid((global::Kerf.Topology.Solid)s)), 3);
        return new Solid((global::Kerf.Topology.Solid)result);
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
                return new Vector(MassPropertiesOf(TopoShape)!.CentreOfMass);
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
        var graph = ShapeGraph.Build(TopoShape);
        var distance2 = length2 is { } l2 && l2 != 0 ? l2 : length;
        var specs = new List<ChamferSpec>();
        foreach (var edge in edges)
        {
            global::Kerf.Topology.Face topoFace;
            if (face is not null)
                topoFace = face.TopoFace;
            else
                topoFace = (global::Kerf.Topology.Face)graph.Ancestors(edge.TopoShape, global::Kerf.Topology.ShapeType.Face).First();
            // BRepFilletAPI_MakeChamfer's Add of two distances on the face
            specs.Add(new ChamferSpec(edge.TopoEdge, length, distance2, ReferenceFace: topoFace));
        }
        var chamfered = TopoShape.Chamfer(specs);
        if (!chamfered.TryGetValue(out var built))
            throw new ArgumentException("Failed creating a chamfer, try a smaller length value(s)", new KerfException(chamfered.Diagnostics));
        var result = Make3DResult(built.Shape);
        if (!result.IsValid)
            throw new ArgumentException("Failed creating a chamfer, try a smaller length value(s)");
        return result.MadeBy(ShapeHistory.FromAlgorithm(s => built.History.Modified(s), s => built.History.Generated(s), [TopoShape], result.TopoShape));
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
        // BRepFeat_MakeDPrism, its sketch face not modified
        var options = new ExtrudeUntilOptions
        {
            Until = upToFace is not null ? ExtrudeLimit.Face : thruAll || depth is null ? ExtrudeLimit.ThroughAll : ExtrudeLimit.Height,
            Face = upToFace?.TopoFace,
            Height = upToFace is null && !(thruAll || depth is null) ? depth!.Value : 0,
            Fuse = additive,
            ModifySketchFace = false,
        };
        foreach (var face in faces)
            shape = Failures.Get(shape.ExtrudeUntil(face.TopoFace, basis?.TopoFace ?? noBasis, taper * Constants.DegToRad, options),
                                 () => "draft prism").Shape;
        return CastLike(shape);
    }

    /// <summary>The edges of this shape filleted with a radius.</summary>
    /// <exception cref="ArgumentException">The fillet can't be made; the message says which edges failed and why.</exception>
    public Shape Fillet(double radius, IEnumerable<Edge> edgeList)
    {
        var edges = edgeList.Select(e => e.TopoEdge).ToList();
        var filleted = TopoShape.Fillet(edges, radius);
        Shape? result = null;
        Exception? failure = null;
        if (filleted.TryGetValue(out var built))
        {
            result = Make3DResult(built.Shape);
            if (!result.IsValid)
                failure = new InvalidOperationException();
        }
        else
            failure = new KerfException(filleted.Diagnostics);
        if (failure is not null)
        {
            string details = "";
            try
            {
                details = filleted.IsFailure ? FilletFailureDetails(filleted.Diagnostics, edges) : "";
            }
            catch (Exception)
            {
                // Diagnostics mustn't mask the kernel's failure
            }
            throw new ArgumentException(
                $"Failed creating a fillet with radius of {PyFormat.Repr(radius)}{details}, try a smaller value or use "
                + "max_fillet() to find the largest valid fillet radius", failure);
        }
        return result!.MadeBy(ShapeHistory.FromAlgorithm(s => built!.History.Modified(s), s => built!.History.Generated(s),
                                                         [TopoShape], result!.TopoShape));
    }

    /// <summary>
    /// The faulty contours of a failed fillet, in terms of the caller's selected edges: the kernel reports each faulty contour
    /// after its first error, with the contour's status as its code and the contour's edges.
    /// </summary>
    private static string FilletFailureDetails(System.Collections.Immutable.ImmutableArray<Diagnostic> diagnostics, List<global::Kerf.Topology.Edge> edges)
    {
        var faulty = new Dictionary<string, HashSet<Edge>>();
        var order = new List<string>();
        foreach (var contour in diagnostics.Where(d => d.IsError).Skip(1))
        {
            var reason = contour.Code.Value switch
            {
                "fillet.walking-failed" => "walking failure",
                "fillet.no-starting-solution" => "no starting solution",
                "fillet.twisted-surface" => "twisted surface",
                _ => "error",
            };
            if (!faulty.TryGetValue(reason, out var set))
            {
                faulty[reason] = set = [];
                order.Add(reason);
            }
            foreach (var edge in contour.Shapes)
                set.Add(new Edge((global::Kerf.Topology.Edge)edge));
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
            // Do these numbers work? If not, try the smaller window
            if (!TopoShape.Fillet(edges, windowMid).TryGetValue(out var built) || !Make3DResult(built.Shape).IsValid)
                return Search(windowMin, windowMid, iteration + 1);
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
        // BRepOffsetAPI_MakeThickSolid by join, in skin mode, with intersections
        var options = new OffsetOptions { Tolerance = tolerance, Mode = OffsetMode.Skin, Intersection = true, SelfIntersection = false, Join = join };
        var made = TopoShape.ThickSolid(faceList.Select(f => f.TopoFace), thickness, options);
        // OCCT can report success and still hand back a null shape
        if (!made.TryGetValue(out var thick) || thick.Shape.IsNull)
            throw new InvalidOperationException("hollow Error, an alternative kind may resolve this error");
        var shellShape = thick.Shape;
        if (faceList.Count > 0)
            return CastLike(shellShape);
        // With no faces removed, make a watertight solid of the two shells
        var shell1 = CastLike(shellShape).Shells()[0].TopoShell;
        var shell2 = Shells()[0].TopoShell;
        // The first can be the outer or inner shell depending on the thickness's sign
        var solid = Failures.Get(global::Kerf.Topology.Solid.FromShells(thickness > 0 ? [shell1, shell2] : [shell2, shell1]), () => "solid");
        return CastLike(solid).Fix();
    }

    /// <summary>Whether a point is inside the solid (or on its surface) within a tolerance.</summary>
    public bool IsInside(Vector point, double tolerance = 1.0e-6)
    {
        var p = point.ToGpPnt();
        return TopoShape.ClassifyPoint(p, tolerance) == Containment.In || TopoShape.IsOnFace(p, tolerance);
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
        // BRepOffsetAPI_MakeThickSolid by join, in skin mode, with intersections, internal edges removed
        var options = new OffsetOptions
        {
            Tolerance = tolerance,
            Mode = OffsetMode.Skin,
            Intersection = true,
            SelfIntersection = false,
            Join = join,
            RemoveInternalEdges = true,
        };
        var made = TopoShape.ThickSolid(faceList.Select(f => f.TopoFace), thickness, options);
        if (!made.TryGetValue(out var thick))
            throw new InvalidOperationException("offset Error, an alternative kind may resolve this error", new KerfException(made.Diagnostics));
        var solid = thick.Shape;
        if (solid.IsNull)
            throw new InvalidOperationException("offset Error, an alternative kind may resolve this error");
        if (solid.Type == global::Kerf.Topology.ShapeType.Solid)
            solid = ThreeD.ForwardSolid((global::Kerf.Topology.Solid)solid);
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
                results.AddRange(BoolOpList([this], [shape], BooleanKind.Common).Expand());
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
        var flags = new List<Orientation> { solid.Orientation };
        foreach (var shell in solid.Children)
            flags.Add(shell.Orientation);
        if (flags.All(f => f == Orientation.Forward))
            return solid;
        var forwardShells = new List<global::Kerf.Topology.Shell>();
        foreach (var shell in solid.Children)   // orientations composed
            forwardShells.Add(global::Kerf.Topology.Shell.Create(shell.Children.Select(f => (global::Kerf.Topology.Face)f)));
        return global::Kerf.Topology.Solid.Create(forwardShells);
    }
}
