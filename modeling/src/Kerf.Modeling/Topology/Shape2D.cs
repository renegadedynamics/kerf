using Kerf.Features;
using Kerf.Measurement;
using Kerf.Offsets;


namespace Kerf.Modeling;

/// <summary>The methods shared by faces and shells (build123d's <c>Mixin2D</c>).</summary>
public static partial class Shape2DExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape2D
    {

        /// <summary>
        /// This face or shell joined with other 2D shapes (build123d's <c>+</c>). When a shell is involved the faces are
        /// sewn into one shell, which joining sheet surfaces along shared edges needs; otherwise they are fused as for any
        /// shape, so coplanar faces make a Face or Sketch.
        /// </summary>
        /// <exception cref="ArgumentException">The operands aren't all 2D, or the faces don't sew into one connected shell.</exception>
        internal Shape AddCoreOfShape2D(IEnumerable<Shape> other)
        {
            var summands = ShapeExtensions.Operands(other);
            if (summands.Count == 0)
                return self.AsShape();
            // Only sew when a shell is being built up
            if (self is not Shell && !self.AsShape().IsShell() && !summands.Any(s => s.IsShell()))
                return ShapeAlgebra.AddPlain(self.AsShape(), null, other);
            if (summands.Any(s => s.Dim != 2))
                throw new ArgumentException("Only shapes with the same dimension can be added");
            var faces = self.Faces().Concat(summands.SelectMany(s => s.Faces())).ToList();
            Shape sum;
            try
            {
                sum = Shell.Make(faces);
            }
            catch (Exception e) when (Failures.IsKernelFailure(e) || e is ArgumentException or InvalidOperationException)
            {
                throw new ArgumentException(
                    "Unable to sew faces into a single connected Shell - faces must meet along shared edges, "
                    + "with at most two faces on an edge", e);
            }
            if (SkipClean.Clean)
                sum = sum.Clean();
            return sum;
        }

        /// <summary>
        /// The faces of this shape divided by a closed perimeter lying on it: those inside and those outside it. Each side is
        /// one face, a shell of several, or (when they don't sew) a list; the empty Face for a side with nothing on it.
        /// </summary>
        /// <remarks>This may fail if the perimeter crosses the shape's edges.</remarks>
        /// <exception cref="ArgumentException">The perimeter isn't closed.</exception>
        public (IReadOnlyList<Face> Inside, IReadOnlyList<Face> Outside) SplitByPerimeter(IShape1D perimeter)
        {
            var (inside, outside) = self.SplitByPerimeterCore(perimeter);
            return (inside.Faces(), outside.Faces());
        }

        /// <summary>
        /// The part of this shape inside (<see cref="Keep.Inside"/>, the default) or outside (<see cref="Keep.Outside"/>) a
        /// closed perimeter lying on it: a Face, a Shell, a Compound of faces that don't sew, or the empty Face.
        /// </summary>
        public Shape SplitByPerimeter(IShape1D perimeter, Keep keep)
        {
            if (keep is not (Keep.Inside or Keep.Outside))
                throw new ArgumentException("keep must be one of Keep.INSIDE, Keep.OUTSIDE, or Keep.BOTH (use the overload without keep for both)");
            if (self.IsEmpty)
                return default;
            var (inside, outside) = self.SplitByPerimeterCore(perimeter);
            return keep == Keep.Inside ? inside : outside;
        }

        private (Shape Inside, Shape Outside) SplitByPerimeterCore(IShape1D perimeter)
        {
            if (!perimeter.IsClosed)
                throw new ArgumentException("perimeter must be a closed Wire or Edge");

            Shape ProcessSides(List<Shape> sides)
            {
                if (sides.Count == 0)
                    return default(Face);
                if (sides.Count == 1)
                    return sides[0];
                // Try to make a shell
                var shell = TopoUtil.SewFaces(sides.Select(s => s));
                return shell.IsShell() ? (shell) : Compound.Make(sides);
            }

            // Split each perimeter edge where it crosses a seam, so the pieces lie in one patch of the surface
            var perimeterEdges = new List<Edge>();
            var seams = self.Faces().SelectMany(f => f.Seams).ToList();
            foreach (var edge in perimeter.Edges())
            {
                if (edge.IsEmpty)
                    continue;
                var seamVertices = new List<Vertex>();
                foreach (var seam in seams)
                {
                    var crossing = edge.Intersect(seam);
                    if (crossing.Count == 0)
                        continue;
                    foreach (var vertex in crossing.Vertices())
                        if (edge.Vertices().All(ev => vertex.DistanceTo(ev) > Constants.Tolerance)
                            && seamVertices.All(existing => vertex.DistanceTo(existing) > Constants.Tolerance))
                            seamVertices.Add(vertex);
                }
                foreach (var piece in Shape2DExtensions.SplitEdgeAtVertices(edge, seamVertices))
                    perimeterEdges.Add(piece);
            }

            var constructor = Failures.Get(FeatureOperations.SplitAlong(self.AsShape(), perimeterEdges), () => "split");
            List<Shape> Sides(IEnumerable<Face> list) => [.. list.Where(s => !s.IsNull).Select(s => (s))];
            Shape left, right;
            left = ProcessSides(Sides(constructor.Left));
            right = ProcessSides(Sides(constructor.Right));

            // Is the left or the right the inside?
            var perimeterLength = perimeter.Length;
            var leftLength = left.IsEmpty ? 0 : left.Edges().Sum(e => e.Length);
            var rightLength = right.IsEmpty ? 0 : right.Edges().Sum(e => e.Length);
            bool leftInside = Math.Abs(perimeterLength - leftLength) < Math.Abs(perimeterLength - rightLength);
            return leftInside ? (left, right) : (right, left);
        }

        /// <summary>The points where an axis meets the surface, nearest the axis's start first, each with the surface normal there.</summary>
        public IReadOnlyList<(Point3D Point, Vector3D Normal)> FindIntersectionPoints(Axis other, double tolerance = Constants.Tolerance) =>
            SurfaceIntersection.FindPoints(self.AsShape(), other, tolerance);

        internal ShapeList<Shape>? IntersectCoreOfShape2D(object other, double tolerance, bool includeTouched)
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
                case IShape2D surface:
                {
                    // The common part (coplanar overlap) and the section (crossing curves)
                    ShapeList<Shape> commonFaces;
                    commonFaces = ShapeExtensions.BoolOpList([self.AsShape()], [surface.AsShape()], BooleanKind.Common).Expand();
                    results.AddRange(commonFaces);
                    ShapeList<Shape> sectionEdges;
                    // The section's vertices are boundary contacts (touches), not intersections
                    sectionEdges = new ShapeList<Shape>(ShapeExtensions.BoolOpList([self.AsShape()], [surface.AsShape()], BooleanKind.Section).OfType<Edge>().Upcast()).Expand();
                    if (commonFaces.Count == 0)
                        results.AddRange(sectionEdges);
                    else
                    {
                        // Leave out edges on the boundary of the overlap, which aren't crossings
                        var commonEdges = commonFaces.SelectMany(f => f.Edges()).Select(ce => (Edge: ce, Box: ce.Bounds(optimal: false))).ToList();
                        foreach (var edge in sectionEdges)
                        {
                            var box = edge.Bounds(optimal: false);
                            if (!commonEdges.Any(ce => box.IntersectsNear(ce.Box, tolerance) && edge.DistanceTo(ce.Edge) <= tolerance))
                                results.Add(edge);
                        }
                    }
                    break;
                }
                case IShape1D line:
                {
                    results.AddRange(ShapeExtensions.BoolOpList([self.AsShape()], [line.AsShape()], BooleanKind.Section));
                    break;
                }
                case Vertex vertex:
                    if (vertex.DistanceTo(self.AsShape()) <= tolerance)
                        results.Add(vertex);
                    break;
                default:
                    // Delegate to higher-order shapes
                    if (ShapeExtensions.IntersectOne(shape.AsShape(), self.AsShape(), tolerance, includeTouched) is { } found)
                        results.AddRange(found);
                    break;
            }
            if (includeTouched && shape.Dim == 2)
                results.AddRange(self.TouchCore(shape, tolerance, results.OfKind<Face>().ToList(), results.OfKind<Edge>().ToList()));
            return results.Count > 0 ? new ShapeList<Shape>(results) : null;
        }

        /// <summary>
        /// Where this face or shell touches another at its boundary: for two faces, the vertices where they meet at a corner
        /// or cross at a point without an edge or face in common.
        /// </summary>
        internal ShapeList<Shape> TouchOfShape2D(Shape other, double tolerance = 1e-6) => self.TouchCore(other, tolerance, null, null);

        internal ShapeList<Shape> TouchCore(Shape other, double tolerance, List<Face>? foundFaces, List<Edge>? foundEdges)
        {
            if (other.Dim != 2)
                // Face + edge or vertex: no touch (the intersection already covers them); a compound iterates
                return other.Touch(self.AsShape(), tolerance);

            bool OnEdges(Vertex v, IEnumerable<Edge> edges) => edges.Any(e => v.DistanceTo(e) <= tolerance);
            bool OnFaces(Vertex v, IEnumerable<Face> faces) => faces.Any(f => v.DistanceTo(f) <= tolerance);
            static bool IsDuplicate(Vertex v, IEnumerable<Vertex> vertices)
            {
                var vec = v.Point;
                return vertices.Any(o => vec.IsNear(o.Point));
            }

            if (foundFaces is null)
            {
                foundFaces = [];
                foundEdges = [];
                if (ShapeExtensions.IntersectOne(self.AsShape(), other, tolerance, includeTouched: false) is { } found)
                {
                    foundFaces.AddRange(found.OfType<Face>());
                    foundEdges.AddRange(found.OfType<Edge>());
                }
            }
            foundEdges ??= [];

            var results = new List<Shape>();
            var foundVertices = new List<Vertex>();
            // All contact points (vertex-vertex, vertex-edge, vertex-face), with high precision to avoid duplicates
            var distance = self.AsShape().Distance(other, new DistanceOptions { Deflection = tolerance * 1e-3 });
            if (distance.TryGetValue(out var extrema) && extrema.Value <= tolerance)
            {
                var selfEdges = self.Edges();
                var otherEdges = other.Edges();
                var selfVertices = self.Vertices();
                var otherVertices = other.Vertices();
                foreach (var solution in extrema.Solutions)
                {
                    var (p1, p2) = (solution.OnA, solution.OnB);
                    if (p1.DistanceTo(p2) > tolerance)
                        continue;
                    var vertex = Vertex.Make(p1.X, p1.Y, p1.Z);
                    if (IsDuplicate(vertex, foundVertices))
                        continue;
                    // Skip edge-edge crossings, but keep corner touches
                    if (OnEdges(vertex, selfEdges) && OnEdges(vertex, otherEdges)
                        && !IsDuplicate(vertex, selfVertices) && !IsDuplicate(vertex, otherVertices))
                        continue;
                    // Keep only vertices that aren't boundaries of higher-dimensional contacts
                    if (!OnFaces(vertex, foundFaces) && !OnEdges(vertex, foundEdges))
                    {
                        results.Add(vertex);
                        foundVertices.Add(vertex);
                    }
                }
            }
            return new ShapeList<Shape>(results);
        }

        /// <summary>
        /// This face or shell offset along its own normals: every point moves <paramref name="amount"/> along the normal there,
        /// so a cylinder changes radius and the faces of a shell stay joined.
        /// </summary>
        /// <remarks>
        /// A curved surface offset by exactly its radius of curvature collapses and is rejected. Offsetting further turns the
        /// surface inside out, which OCCT reports as success and isn't detected.
        /// </remarks>
        /// <exception cref="InvalidOperationException">The offset collapsed or failed.</exception>
        internal Shape OffsetCore(double amount)
        {
            if (amount == 0)
                return self.AsShape();
            // BRepOffset_MakeOffset in skin mode, not thickening
            var options = new OffsetOptions
            {
                Tolerance = Constants.Tolerance,
                Mode = OffsetMode.Skin,
                Intersection = true,
                SelfIntersection = false,
                Join = JoinKind.Intersection,
                RemoveInternalEdges = true,
            };
            var offsetResult = self.AsShape().Offset(amount, options);
            if (!offsetResult.TryGetValue(out var built) || built.Shape.IsNull)
                throw Failures.Expected(ModelingDiagnostics.OffsetFailed, $"The offset of the {typeof(TSelf).Name} by {Formatting.Number(amount)} failed.",
                                        offsetResult.IsFailure ? offsetResult.Diagnostics : default);
            var offset = built.Shape;
            var result = (offset);
            // The builder always returns a shell; a face offsets to a single face
            if (self.AsShape().TryAs<Face>(out _) && result.Faces().Count == 1)
                result = result.Faces()[0];
            if (result.Area <= Constants.Tolerance)
                throw new InvalidOperationException($"Offsetting by {amount} collapsed the {typeof(TSelf).Name} onto itself");
            return result;
        }
    }



    /// <summary>An edge split at all the given interior vertices.</summary>
    private static List<Edge> SplitEdgeAtVertices(Edge edge, List<Vertex> vertices)
    {
        var segments = new List<Edge> { edge };
        foreach (var vertex in vertices)
        {
            var next = new List<Edge>();
            foreach (var segment in segments)
            {
                if (segment.DistanceTo(vertex) > Constants.Tolerance
                    || segment.Vertices().Any(v => vertex.DistanceTo(v) <= Constants.Tolerance))
                {
                    next.Add(segment);
                    continue;
                }
                next.AddRange(TwoD.SplitEdgeAtVertex(segment, vertex).Select(e => e));
            }
            segments = next;
        }
        return segments;
    }
}

/// <summary>A face or shell with its normal reversed.</summary>
public static class Shape2DOperators
{
    extension<T>(T) where T : struct, IShape<T>, IShape2D
    {
        /// <summary>This face or shell with its normal reversed (build123d's unary <c>-</c>).</summary>
        public static T operator -(T shape)
        {
            if (shape.IsEmpty)
                throw new InvalidOperationException("Invalid Shape");
            return shape.AsShape().Complemented().As<T>();
        }
    }
}

/// <summary>The offset of faces and shells, keeping the caller's class.</summary>
public static class Shape2DOperations
{
    extension<T>(T shape) where T : IShape2D
    {
        /// <summary>
        /// The face or shell offset along its own normals (see <see cref="IShape2D"/>): a positive amount follows the normal.
        /// </summary>
        public T Offset(double amount) => TopoUtil.Rewrap(shape, shape.OffsetCore(amount));
    }
}
