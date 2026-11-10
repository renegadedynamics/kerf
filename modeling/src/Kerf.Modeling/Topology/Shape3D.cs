using Kerf.Construction;
using Kerf.Features;
using Kerf.Fillets;
using Kerf.Healing;
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
        /// asymmetric chamfer), measured on <paramref name="face"/> if given (which the edges must belong to), and past tight
        /// corners as <paramref name="options"/> say.
        /// </summary>
        /// <exception cref="ArgumentException">An edge isn't on the face.</exception>
        /// <exception cref="InvalidOperationException">
        /// The chamfer can't be made: the shorthand's failure; <see cref="Operations.Chamfer(IEnumerable{Edge},
        /// double, double?, double?, Face?, Operations.ChamferOptions?)"/> returns it as a result, with the code <see cref="ModelingDiagnostics.ChamferFailed"/>.
        /// </exception>
        public Shape Chamfer(double length, double? length2, IEnumerable<Edge> edgeList, Face? face = null, Operations.ChamferOptions? options = null)
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
            var kernelOptions = options?.TightCorners == TightCorners.Collapse ? new ChamferOptions { TightCorners = Kerf.Fillets.TightCorners.Collapse } : null;
            var chamfered = self.AsShape().Chamfer(specs, kernelOptions);
            // A broken result's error already says the chamfer couldn't be built at its length (decision 1985)
            if (!chamfered.TryGetValue(out var built))
                throw Failures.Expected(ModelingDiagnostics.ChamferFailed,
                                        Shape3DExtensions.TooLargeMessage(chamfered.Diagnostics)
                                        ?? (chamfered.TryGetInvalid(out _) ? "" : $"The chamfer with a length of {Formatting.Number(length)} failed; try a smaller length."),
                                        chamfered);
            var result = Shape3DExtensions.Make3DResult(built.Shape);
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
            if (!filleted.TryGetValue(out var built))
            {
                string details = "";
                try
                {
                    details = Shape3DExtensions.FilletFailureDetails(filleted.Diagnostics, edges);
                }
                catch (Exception e) when (e is not OutOfMemoryException and not OperationCanceledException)
                {
                    // Diagnostics mustn't mask the kernel's failure (nor running out of memory, nor a cancel: decision 1943)
                }
                // What doesn't fit says so, and with several edges, which of them the algorithm failed on
                string message = Shape3DExtensions.TooLargeMessage(filleted.Diagnostics) is { } tooLarge
                    ? edges.Count > 1 && details.Length > 0 ? $"{tooLarge} It failed{details}." : tooLarge
                    : $"The fillet with a radius of {Formatting.Number(radius)}{details} failed; try a smaller radius, or MaxFillet "
                      + "to find the largest that works.";
                const string advice = "Try a smaller radius, or MaxFillet to find the largest that works.";
                if (filleted.TryGetInvalid(out var invalid))
                    // A result that isn't valid is described from its own report; one the kernel failed for a reason of its own (its
                    // edges cross, fillet.self-intersecting) keeps that error and its edges, carrying the shape (decision 2123)
                    throw filleted.Error!.Code == DiagnosticCode.InvalidResult
                        ? Failures.Invalid(invalid.Shape, invalid.Shape.Validate(), $"The fillet with a radius of {Formatting.Number(radius)}", advice)
                        : Failures.Expected(ModelingDiagnostics.FilletFailed, advice, filleted);
                throw Failures.Expected(ModelingDiagnostics.FilletFailed, message, filleted.Diagnostics, Failures.Failed(filleted.Diagnostics));
            }
            var result = Shape3DExtensions.Make3DResult(built.Shape);
            return result.MadeBy(ModelHistory.FromAlgorithm(s => built!.History.Modified(s), s => built!.History.Generated(s),
                                                             [self.AsShape()], result));
        }

        /// <summary>
        /// The largest fillet radius the edges take: for straight edges between planes and circles between planes, cylinders and cones, what the geometry gives, if that can be
        /// built, or else that less what the shape's tolerance there needs, if that can; otherwise found by a binary search to within
        /// <paramref name="tolerance"/>, which doesn't try what the geometry says can't fit.
        /// </summary>
        /// <exception cref="InvalidOperationException">No value was found within <paramref name="maxIterations"/>.</exception>
        /// <exception cref="ArgumentException">The shape isn't valid.</exception>
        public double MaxFillet(IEnumerable<Edge> edgeList, double tolerance = 0.1, int maxIterations = 10)
        {
            if (!self.IsValid)
                throw new ArgumentException("Invalid Shape");
            var edges = edgeList.Select(e => e).ToList();
            return Shape3DExtensions.Largest(self.AsShape().FilletSizeLimit(edges), r => self.AsShape().Fillet(edges, r).IsSuccess,
                                             2 * self.Bounds().Diagonal, tolerance, maxIterations);
        }

        /// <summary>
        /// The largest chamfer length the edges take, as one length on both faces: what the geometry gives (the tightest convex arc
        /// in a face beside them, or the width of a face beside straight edges and rims), if that can be built, or else that less what
        /// the shape's tolerance there needs, if that can (an arc's radius, where the edge closes to a point, less twice its tolerance);
        /// otherwise found by the same binary search as <c>MaxFillet</c>, to within <paramref name="tolerance"/>, which doesn't try
        /// what the geometry says can't fit. build123d has no <c>max_chamfer</c>.
        /// </summary>
        /// <exception cref="InvalidOperationException">No value was found within <paramref name="maxIterations"/>.</exception>
        /// <exception cref="ArgumentException">The shape isn't valid.</exception>
        public double MaxChamfer(IEnumerable<Edge> edgeList, double tolerance = 0.1, int maxIterations = 10)
        {
            if (!self.IsValid)
                throw new ArgumentException("Invalid Shape");
            var edges = edgeList.Select(e => e).ToList();
            return Shape3DExtensions.Largest(self.AsShape().ChamferSizeLimit(edges), d => self.AsShape().Chamfer(edges, d).IsSuccess,
                                             2 * self.Bounds().Diagonal, tolerance, maxIterations);
        }

        /// <summary>
        /// This shape shelled to a <paramref name="thickness"/> (positive outwards, negative inwards), the given faces removed to
        /// open it; with no faces, a closed hollow solid.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="kind"/> is neither Kind.Arc nor Kind.Intersection.</exception>
        /// <exception cref="InvalidOperationException">OCCT can't build the shell.</exception>
        public Shape Hollow(IEnumerable<Face>? faces, double thickness, double tolerance = 0.0001, Kind kind = Kind.Arc)
        {
            var faceList = faces?.ToList() ?? [];
            if (kind is not (Kind.Arc or Kind.Intersection))
                throw new ArgumentOutOfRangeException(nameof(kind), kind, "An offset's kind is Kind.Arc or Kind.Intersection.");
            var join = kind == Kind.Arc ? JoinKind.Arc : JoinKind.Intersection;
            // BRepOffsetAPI_MakeThickSolid by join, in skin mode, with intersections
            var options = new OffsetOptions { Tolerance = tolerance, Mode = OffsetMode.Skin, Intersection = true, SelfIntersection = false, Join = join };
            var made = ThickSolid(self.AsShape(), faceList, thickness, options, "The hollow solid");
            if (made.TryGetInvalid(out _))
                throw Failures.Expected(ModelingDiagnostics.OffsetFailed, "An alternative kind may resolve this error.", made);
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
            var solid = Failures.Get(Solid.FromShells(thickness > 0 ? [shell1, shell2] : [shell2, shell1], checkResult: false), () => "solid");
            return Failures.Valid(solid.Fix(), "The hollow solid", "An alternative kind may resolve this error.");
        }

        /// <summary>Whether a point is inside the solid (or on its surface) within a tolerance.</summary>
        public bool IsInside(Point3D point, double tolerance = 1.0e-6)
        {
            // One classification: its state is ON exactly when the point is on a face (unit 15C)
            return self.AsShape().ClassifyPoint(point, tolerance) is Containment.In or Containment.On;
        }

        /// <summary>
        /// This shape offset by <paramref name="thickness"/> (positive outwards), with <paramref name="openings"/> removed.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="kind"/> is neither Kind.Arc nor Kind.Intersection.</exception>
        /// <exception cref="InvalidOperationException">OCCT can't build the offset.</exception>
        public Shape Offset3D(IEnumerable<Face>? openings, double thickness, double tolerance = 0.0001, Kind kind = Kind.Arc)
        {
            var faceList = openings?.ToList() ?? [];
            if (kind is not (Kind.Arc or Kind.Intersection))
                throw new ArgumentOutOfRangeException(nameof(kind), kind, "An offset's kind is Kind.Arc or Kind.Intersection.");
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
            var made = ThickSolid(self.AsShape(), faceList, thickness, options, "The offset's result");
            if (!made.TryGetValue(out var thick))
                throw Failures.Expected(ModelingDiagnostics.OffsetFailed, "The offset failed; another kind of join may work.", made);
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
            var shape = ShapeExtensions.ShapeOfAny(other);
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

    /// <summary>
    /// The kernel's thick solid, which checks its size and makes it again with mitres if round joins failed at an apex
    /// (decisions 1918 and 1938), with an invalid one repaired only if the repair passes the same check of size; on success the
    /// kernel's warnings are raised (<c>offset.join-changed</c>).
    /// </summary>
    private static Result<Built<Shape>> ThickSolid(Shape shape, List<Face> openings, double thickness, OffsetOptions options, string what)
    {
        // With no openings the kernel's thick solid is the offset turned inside out when outwards
        var made = TopoUtil.Repaired(shape.ThickSolid(openings, thickness, options),
                                     repaired => OffsetVolumes.Check(shape, thickness, options.Join,
                                                                     openings.Count == 0 && thickness > 0 ? repaired.Reversed() : repaired,
                                                                     what, isWall: openings.Count > 0));
        if (made.IsSuccess)
            foreach (var warning in made.Diagnostics)
                Warnings.Warn(warning);
        return made;
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
    /// The largest size that works, to within <paramref name="tolerance"/>: the geometry's <paramref name="proved"/> limit if it works,
    /// then the largest size below it that isn't taken as the limit, if that works, and otherwise halving (0, <paramref name="windowMax"/>),
    /// without trying at or past the limit, which can't fit (decision 1900).
    /// </summary>
    internal static double Largest(SizeLimit? proved, Func<double, bool> works, double windowMax, double tolerance, int maxIterations)
    {
        double? limit = proved?.Value;
        double Search(double windowMin, double windowMax, int iteration)
        {
            var windowMid = (windowMin + windowMax) / 2;
            // The last try failed, but the window has closed on one that worked
            if (iteration == maxIterations && windowMin > 0 && windowMid - windowMin <= tolerance)
                return windowMin;
            if (iteration == maxIterations)
                throw new InvalidOperationException($"Failed to find the max value within {Formatting.Number(tolerance)} in {maxIterations}");
            // Do these numbers work? If not, try the smaller window. Past the limit they don't, so it isn't asked (decision 1900):
            // the same halving, so the same answer, without the tries that can't fit
            if (windowMid >= limit || !works(windowMid))
                return Search(windowMin, windowMid, iteration + 1);
            // They work: close enough, or try the larger window
            return windowMid - windowMin <= tolerance ? windowMid : Search(windowMid, windowMax, iteration + 1);
        }
        if (proved is { } most)
        {
            if (works(most.Value))
                return most.Value;
            // Where the limit itself can't be built, just below it often can: an arc's chamfer to a point, a fillet using up a face
            if (most.JustBelow > 0 && works(most.JustBelow))
                return most.JustBelow;
        }
        return Search(0.0, windowMax, 0);
    }

    /// <summary>The kernel's own account of a fillet or chamfer that doesn't fit, which says more than a general failure can.</summary>
    private static string? TooLargeMessage(System.Collections.Immutable.ImmutableArray<Diagnostic> diagnostics) =>
        diagnostics.FirstOrDefault(d => d.Code == FilletOperations.TooLarge)?.Message;

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
                "fillet.too-large" => "too large",
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
