using Kerf.Construction;
using Kerf.Features;
using Kerf.Fillets;
using Kerf.Measurement;
using Kerf.Offsets;

namespace Kerf.Modeling;

/// <summary>The methods shared by solids and compounds (build123d's <c>Mixin3D</c>).</summary>
public static partial class Shape3DExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape3D
    {

        /// <summary>The centre of mass (by default) or of the bounding box; the geometric centre isn't supported.</summary>
        internal Point3D CenterOfShape3D() => self.Center(CenterOf.Mass);

        /// <summary>As the other forms of <c>Center</c>.</summary>
        internal Point3D CenterOfShape3D(CenterOf centerOf)
        {
            self.NeedsGeometry("center");
            switch (centerOf)
            {
                case CenterOf.Geometry:
                    throw new ArgumentException("Center of GEOMETRY is not supported for this object");
                case CenterOf.Mass:
                    return (ShapeExtensions.MassPropertiesOf(self.AsShape())!.CentreOfMass);
                default:
                    return self.Bounds().Center();
            }
        }

        /// <summary>
        /// The edges of this shape chamfered by <paramref name="length"/> (and <paramref name="length2"/> on the other side, for an
        /// asymmetric chamfer), measured on <paramref name="face"/> if given (which the edges must belong to).
        /// </summary>
        /// <exception cref="ArgumentException">An edge isn't on the face.</exception>
        /// <exception cref="InvalidOperationException">
        /// The chamfer can't be made: the shorthand's failure; <see cref="Operations.Chamfer(IEnumerable{Edge},
        /// double, double?, double?, Face?)"/> returns it as a result, with the code <see cref="ModelingDiagnostics.ChamferFailed"/>.
        /// </exception>
        public Shape Chamfer(double length, double? length2, IEnumerable<Edge> edgeList, Face? face = null)
        {
            var edges = edgeList.ToList();
            if (face is { } onFace)
            {
                var faceEdges = onFace.Edges();
                if (edges.Any(e => !faceEdges.Contains(e, ShapeComparer<Edge>.Same)))
                    throw new ArgumentException("Some edges are not part of the face");
            }
            var graph = ShapeGraph.Build(self.AsShape());
            var distance2 = length2 is { } l2 && l2 != 0 ? l2 : length;
            var specs = new List<ChamferSpec>();
            foreach (var edge in edges)
            {
                Face topoFace;
                if (face is { } given)
                    topoFace = given;
                else
                    topoFace = (Face)graph.Ancestors(edge, ShapeType.Face).First();
                // BRepFilletAPI_MakeChamfer's Add of two distances on the face
                specs.Add(new ChamferSpec(edge, length, distance2, ReferenceFace: topoFace));
            }
            var chamfered = self.AsShape().Chamfer(specs);
            if (!chamfered.TryGetValue(out var built))
                throw Failures.Expected(ModelingDiagnostics.ChamferFailed, "The chamfer failed; try a smaller length.", chamfered.Diagnostics);
            var result = Shape3DExtensions.Make3DResult(built.Shape);
            if (!result.IsValid)
                throw Failures.Expected(ModelingDiagnostics.ChamferFailed, "The chamfer's result isn't valid; try a smaller length.");
            return result.MadeBy(ModelHistory.FromAlgorithm(s => built.History.Modified(s), s => built.History.Generated(s), [self.AsShape()], result));
        }

        /// <summary>
        /// A prismatic feature, added or subtracted, from profiles (faces, or wires grouped into faces) on an optional basis
        /// face: through all, to a <paramref name="depth"/>, or up to a face, with an optional taper in degrees.
        /// </summary>
        public Shape DraftPrism(Face? basis, IReadOnlyList<IShape2D> bounds, double? depth = null, double taper = 0, Face? upToFace = null,
                            bool thruAll = true, bool additive = true) =>
            self.DPrismCore(basis, bounds.SelectMany(b => b.Faces()), depth, taper, upToFace, thruAll, additive);

        /// <summary>As the other forms of <c>DraftPrism</c>.</summary>
        public Shape DraftPrism(Face? basis, IReadOnlyList<Wire> bounds, double? depth = null, double taper = 0, Face? upToFace = null,
                            bool thruAll = true, bool additive = true) =>
            self.DPrismCore(basis, TwoD.SortWiresByBuildOrder(bounds).Select(p => Face.Make(p[0], p.Skip(1))), depth, taper, upToFace, thruAll, additive);

        private Shape DPrismCore(Face? basis, IEnumerable<Face> faces, double? depth, double taper, Face? upToFace, bool thruAll, bool additive)
        {
            var shape = self.AsShape();
            var noBasis = new Face();
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
                shape = Failures.Get(shape.ExtrudeUntil(face, basis?.TopoFace ?? noBasis, taper * Constants.DegToRad, options),
                                     () => "draft prism").Shape;
            return (shape);
        }

        /// <summary>The edges of this shape filleted with a radius.</summary>
        /// <exception cref="InvalidOperationException">
        /// The fillet can't be made: the shorthand's failure, whose message says which edges failed and why.
        /// <see cref="Operations.Fillet(IEnumerable{Edge}, double)"/> returns it as a result, with the code
        /// <see cref="ModelingDiagnostics.FilletFailed"/> and the kernel's diagnostics.
        /// </exception>
        public Shape Fillet(double radius, IEnumerable<Edge> edgeList)
        {
            var edges = edgeList.Select(e => e).ToList();
            var filleted = self.AsShape().Fillet(edges, radius);
            Shape result = default;
            Exception? failure = null;
            if (filleted.TryGetValue(out var built))
            {
                result = Shape3DExtensions.Make3DResult(built.Shape);
                if (!result.IsValid)
                    failure = new InvalidOperationException();
            }
            else
                failure = Failures.Failed(filleted.Diagnostics);
            if (failure is not null)
            {
                string details = "";
                try
                {
                    details = filleted.IsFailure ? Shape3DExtensions.FilletFailureDetails(filleted.Diagnostics, edges) : "";
                }
                catch (Exception)
                {
                    // Diagnostics mustn't mask the kernel's failure
                }
                throw Failures.Expected(ModelingDiagnostics.FilletFailed,
                    $"The fillet with a radius of {Formatting.Number(radius)}{details} failed; try a smaller radius, or MaxFillet "
                    + "to find the largest that works.", filleted.IsFailure ? filleted.Diagnostics : default, failure);
            }
            return result.MadeBy(ModelHistory.FromAlgorithm(s => built!.History.Modified(s), s => built!.History.Generated(s),
                                                             [self.AsShape()], result));
        }

        /// <summary>
        /// The largest fillet radius the edges take, found by a binary search to within <paramref name="tolerance"/>.
        /// </summary>
        /// <exception cref="InvalidOperationException">No value was found within <paramref name="maxIterations"/>.</exception>
        /// <exception cref="ArgumentException">The shape isn't valid.</exception>
        public double MaxFillet(IEnumerable<Edge> edgeList, double tolerance = 0.1, int maxIterations = 10)
        {
            if (!self.IsValid)
                throw new ArgumentException("Invalid Shape");
            var edges = edgeList.Select(e => e).ToList();
            double Search(double windowMin, double windowMax, int iteration)
            {
                var windowMid = (windowMin + windowMax) / 2;
                if (iteration == maxIterations)
                    throw new InvalidOperationException($"Failed to find the max value within {Formatting.Number(tolerance)} in {maxIterations}");
                // Do these numbers work? If not, try the smaller window
                if (!self.AsShape().Fillet(edges, windowMid).TryGetValue(out var built) || !Shape3DExtensions.Make3DResult(built.Shape).IsValid)
                    return Search(windowMin, windowMid, iteration + 1);
                // They work: close enough, or try the larger window
                return windowMid - windowMin <= tolerance ? windowMid : Search(windowMid, windowMax, iteration + 1);
            }
            return Search(0.0, 2 * self.Bounds().Diagonal, 0);
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
            var made = self.AsShape().ThickSolid(faceList.Select(f => f), thickness, options);
            // OCCT can report success and still hand back a null shape
            if (!made.TryGetValue(out var thick) || thick.Shape.IsNull)
                throw new InvalidOperationException("hollow Error, an alternative kind may resolve this error");
            var shellShape = thick.Shape;
            if (faceList.Count > 0)
                return (shellShape);
            // With no faces removed, make a watertight solid of the two shells
            var shell1 = (shellShape).Shells()[0];
            var shell2 = self.Shells()[0];
            // The first can be the outer or inner shell depending on the thickness's sign
            var solid = Failures.Get(Solid.FromShells(thickness > 0 ? [shell1, shell2] : [shell2, shell1]), () => "solid");
            return (solid).Fix();
        }

        /// <summary>Whether a point is inside the solid (or on its surface) within a tolerance.</summary>
        public bool IsInside(Point3D point, double tolerance = 1.0e-6)
        {
            var p = point;
            return self.AsShape().ClassifyPoint(p, tolerance) == Containment.In || self.AsShape().IsOnFace(p, tolerance);
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
            var made = self.AsShape().ThickSolid(faceList.Select(f => f), thickness, options);
            if (!made.TryGetValue(out var thick))
                throw Failures.Expected(ModelingDiagnostics.OffsetFailed, "The offset failed; another kind of join may work.", made.Diagnostics);
            var solid = thick.Shape;
            if (solid.IsNull)
                throw new InvalidOperationException("offset Error, an alternative kind may resolve this error");
            if (solid.Type == ShapeType.Solid)
                solid = ThreeD.ForwardSolid((Solid)solid);
            var result = (solid);
            // The solid can be inverted: if so, reverse it
            return result.Volume < 0 ? (result.Reversed()) : result;
        }

        internal ShapeList<Shape>? IntersectCoreOfShape3D(object other, double tolerance, bool includeTouched)
        {
            var shape = ShapeExtensions.ShapeOf(other);
            var results = new List<Shape>();
            // Trim infinite edges before OCCT operations
            if (TopoUtil.Typed(shape) is Edge { IsInfinite: true } infinite)
            {
                var box = self.Bounds(optimal: false);
                shape = infinite.TrimInfinite(box.Diagonal + (infinite.Center() - box.Center()).Magnitude);
            }
            switch (TopoUtil.Typed(shape))
            {
                case Solid or Face or Shell or Edge or Wire:
                {
                    results.AddRange(ShapeExtensions.BoolOpList([self.AsShape()], [shape], BooleanKind.Common).Expand());
                    break;
                }
                case Vertex vertex:
                    if (self.IsInside(vertex.Point, tolerance))
                        results.Add(vertex);
                    break;
                default:
                    // Delegate to higher-order shapes (a compound); the outer caller handles touches
                    if (ShapeExtensions.IntersectOne(shape.AsShape(), self.AsShape(), tolerance, includeTouched: false) is { } found)
                        results.AddRange(found);
                    break;
            }
            if (includeTouched && self.AsShape().TryAs(out Solid solid))
            {
                results.AddRange(solid.Touch(shape, tolerance));
                results = Shape3DExtensions.FilterRedundantTouches(results.Distinct(ShapeComparer.Same).ToList(), tolerance);
            }
            return results.Count > 0 ? new ShapeList<Shape>(results) : null;
        }
    }



    /// <summary>A 3D operation's result as topology rather than as the source's class: a Solid, or a Part of several.</summary>
    internal static Shape Make3DResult(Shape shape)
    {
        var result = TopoUtil.Downcast(shape);
        if (result.IsCompound())
            result = TopoUtil.Downcast(TopoUtil.UnwrapCompound((Compound)result));
        if (result.IsCompound())
            return ShapeExtensions.MakeComposite(TopoUtil.TopLevelShapes(result));
        return (Solid)result;
    }

    /// <summary>
    /// The faulty contours of a failed fillet, in terms of the caller's selected edges: the kernel reports each faulty contour
    /// after its first error, with the contour's status as its code and the contour's edges.
    /// </summary>
    private static string FilletFailureDetails(System.Collections.Immutable.ImmutableArray<Diagnostic> diagnostics, List<Edge> edges)
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
                faulty[reason] = set = new(ShapeComparer<Edge>.Same);
                order.Add(reason);
            }
            foreach (var edge in contour.Shapes)
                set.Add((Edge)edge);
        }
        if (faulty.Count == 0)
            return "";
        var selected = edges.Select(e => e).ToList();
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
                    entries.Add($"[{i}] at {selected[i].Center().Describe("G6")}");
                }
            var tangentCount = set.Count(e => !selected.Contains(e, ShapeComparer<Edge>.Same));
            if (tangentCount > 0)
                entries.Add($"{tangentCount} additional tangent {(tangentCount == 1 ? "edge" : "edges")}");
            groups.Add($"{reason}: {string.Join(", ", entries)}");
        }
        return $" on {rejected.Count} of {edges.Count} edges ({string.Join("; ", groups)})";
    }

    /// <summary>The results without vertices and edges that lie on higher-dimensional ones.</summary>
    private static List<Shape> FilterRedundantTouches(List<Shape> items, double tolerance)
    {
        var edges = items.OfKind<Edge>().ToList();
        var faces = items.OfKind<Face>().ToList();
        var solids = items.OfKind<Solid>().ToList();
        return items.Where(r =>
            !(r.TryAs(out Vertex v) && (edges.Any(e => e.DistanceTo(v) <= tolerance) || faces.Any(f => f.DistanceTo(v) <= tolerance)
                                || solids.SelectMany(s => s.Faces()).Any(sf => sf.DistanceTo(v) <= tolerance)))
            && !(r.TryAs(out Edge e2) && faces.Any(f => f.DistanceTo(e2.Center()) <= tolerance))).ToList();
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
    internal static Shape ForwardSolid(Solid solid)
    {
        var flags = new List<Orientation> { solid.Orientation };
        foreach (var shell in solid.Children)
            flags.Add(shell.Orientation);
        if (flags.All(f => f == Orientation.Forward))
            return solid;
        var forwardShells = new List<Shell>();
        foreach (var shell in solid.Children)   // orientations composed
            forwardShells.Add(Shell.Create(shell.Children.Select(f => (Face)f)));
        return Solid.Create(forwardShells);
    }
}
