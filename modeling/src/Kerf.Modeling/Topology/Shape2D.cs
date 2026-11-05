using Kerf.Features;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Geometry;


namespace Kerf.Modeling;

/// <summary>The methods shared by faces and shells (build123d's <c>Mixin2D</c>).</summary>
public static partial class Shape2DExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape2D
    {

        internal int? Dim_Shape2D => 2;

        /// <summary>
        /// This face or shell joined with other 2D shapes (build123d's <c>+</c>). When a shell is involved the faces are
        /// sewn into one shell, which joining sheet surfaces along shared edges needs; otherwise they are fused as for any
        /// shape, so coplanar faces make a Face or Sketch.
        /// </summary>
        /// <exception cref="ArgumentException">The operands aren't all 2D, or the faces don't sew into one connected shell.</exception>
        internal Shape AddCore_Shape2D(IEnumerable<Shape> other)
        {
            var summands = ShapeExtensions.Operands(other);
            if (summands.Count == 0)
                return self.AsShape();
            // Only sew when a shell is being built up
            if (self.AsShape() is not Shell && !summands.Any(s => s.TryAs<Shell>(out _)))
                return base.AddCore(other);
            if (summands.Any(s => s.Dim != 2))
                throw new ArgumentException("Only shapes with the same dimension can be added");
            var faces = self.SelectFaces().Concat(summands.SelectMany(s => s.SelectFaces())).ToList();
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
            return self.CopyAttributesTo(sum);
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
            return (inside.SelectFaces(), outside.SelectFaces());
        }

        /// <summary>
        /// The part of this shape inside (<see cref="Keep.Inside"/>, the default) or outside (<see cref="Keep.Outside"/>) a
        /// closed perimeter lying on it: a Face, a Shell, a Compound of faces that don't sew, or the empty Face.
        /// </summary>
        public Shape SplitByPerimeter(IShape1D perimeter, Keep keep)
        {
            if (keep is not (Keep.Inside or Keep.Outside))
                throw new ArgumentException("keep must be one of Keep.INSIDE, Keep.OUTSIDE, or Keep.BOTH (use the overload without keep for both)");
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
            var seams = self.SelectFaces().SelectMany(f => f.Seams).ToList();
            foreach (var edge in perimeter.SelectEdges())
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
                        if (edge.SelectVertices().All(ev => vertex.DistanceTo(ev) > Constants.Tolerance)
                            && seamVertices.All(existing => vertex.DistanceTo(existing) > Constants.Tolerance))
                            seamVertices.Add(vertex);
                }
                foreach (var piece in Shape2DExtensions.SplitEdgeAtVertices(edge, seamVertices))
                    perimeterEdges.Add(piece);
            }

            var constructor = Failures.Get(FeatureOperations.SplitAlong(self.AsShape(), perimeterEdges), () => "split");
            List<Shape> Sides(IEnumerable<Face> list) => [.. list.Where(s => !s.IsNull).Select(s => (s))];
            Shape left, right;
            left = self.ProcessSides(self.Sides(constructor.Left));
            right = self.ProcessSides(self.Sides(constructor.Right));

            // Is the left or the right the inside?
            var perimeterLength = perimeter.Length;
            var leftLength = left.IsEmpty ? 0 : left.SelectEdges().Sum(e => e.Length);
            var rightLength = right.IsEmpty ? 0 : right.SelectEdges().Sum(e => e.Length);
            bool leftInside = Math.Abs(perimeterLength - leftLength) < Math.Abs(perimeterLength - rightLength);
            return leftInside ? (left, right) : (right, left);
        }

        /// <summary>The points where an axis meets the surface, nearest the axis's start first, each with the surface normal there.</summary>
        public IReadOnlyList<(Vector Point, Vector Normal)> FindIntersectionPoints(Axis other, double tolerance = Constants.Tolerance) =>
            SurfaceIntersection.FindPoints(self.AsShape(), other, tolerance);

        internal ShapeList<Shape>? IntersectCore_Shape2D(object other, double tolerance, bool includeTouched)
        {
            var shape = Shape.AsShape(other);
            var results = new List<Shape>();
            // Trim infinite edges before OCCT operations
            if (shape is Edge { IsInfinite: true } infinite)
            {
                var box = self.BoundingBox(optimal: false);
                shape = infinite.TrimInfinite(box.Diagonal + (infinite.Center() - box.Center()).Length);
            }
            switch (shape)
            {
                case IShape2D surface:
                {
                    // The common part (coplanar overlap) and the section (crossing curves)
                    ShapeList<Shape> commonFaces;
                    commonFaces = self.BoolOpList([self.AsShape()], [surface], BooleanKind.Common).Expand();
                    results.AddRange(commonFaces);
                    ShapeList<Shape> sectionEdges;
                    // The section's vertices are boundary contacts (touches), not intersections
                    sectionEdges = new ShapeList<Shape>(self.BoolOpList([self.AsShape()], [surface], BooleanKind.Section).OfType<Edge>()).Expand();
                    if (commonFaces.Count == 0)
                        results.AddRange(sectionEdges);
                    else
                    {
                        // Leave out edges on the boundary of the overlap, which aren't crossings
                        var commonEdges = commonFaces.SelectMany(f => f.SelectEdges()).Select(ce => (Edge: ce, Box: ce.BoundingBox(optimal: false))).ToList();
                        foreach (var edge in sectionEdges)
                        {
                            var box = edge.BoundingBox(optimal: false);
                            if (!commonEdges.Any(ce => box.Intersects(ce.Box, tolerance) && edge.DistanceTo(ce.Edge) <= tolerance))
                                results.Add(edge);
                        }
                    }
                    break;
                }
                case IShape1D line:
                {
                    results.AddRange(self.BoolOpList([self.AsShape()], [line], BooleanKind.Section));
                    break;
                }
                case Vertex vertex:
                    if (vertex.DistanceTo(self.AsShape()) <= tolerance)
                        results.Add(vertex);
                    break;
                default:
                    // Delegate to higher-order shapes
                    if (shape.IntersectCore(self.AsShape(), tolerance, includeTouched) is { } found)
                        results.AddRange(found);
                    break;
            }
            if (includeTouched && shape is IShape2D)
                results.AddRange(self.TouchCore(shape, tolerance, results.OfType<Face>().ToList(), results.OfType<Edge>().ToList()));
            return results.Count > 0 ? new ShapeList<Shape>(results) : null;
        }

        /// <summary>
        /// Where this face or shell touches another at its boundary: for two faces, the vertices where they meet at a corner
        /// or cross at a point without an edge or face in common.
        /// </summary>
        internal ShapeList<Shape> Touch_Shape2D(Shape other, double tolerance = 1e-6) => self.TouchCore(other, tolerance, null, null);

        internal ShapeList<Shape> TouchCore(Shape other, double tolerance, List<Face>? foundFaces, List<Edge>? foundEdges)
        {
            if (other is not IShape2D)
                // Face + edge or vertex: no touch (the intersection already covers them); a compound iterates
                return other.Touch(self.AsShape(), tolerance);

            bool OnEdges(Vertex v, IEnumerable<Edge> edges) => edges.Any(e => v.DistanceTo(e) <= tolerance);
            bool OnFaces(Vertex v, IEnumerable<Face> faces) => faces.Any(f => v.DistanceTo(f) <= tolerance);
            static bool IsDuplicate(Vertex v, IEnumerable<Vertex> vertices)
            {
                var vec = v.ToVector();
                return vertices.Any(o => vec == o.ToVector());
            }

            if (foundFaces is null)
            {
                foundFaces = [];
                foundEdges = [];
                if (self.IntersectCore(other, tolerance, includeTouched: false) is { } found)
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
                var selfEdges = self.SelectEdges();
                var otherEdges = other.SelectEdges();
                var selfVertices = self.SelectVertices();
                var otherVertices = other.SelectVertices();
                foreach (var solution in extrema.Solutions)
                {
                    var (p1, p2) = (solution.OnA, solution.OnB);
                    if (p1.DistanceTo(p2) > tolerance)
                        continue;
                    var vertex = Vertex.Make(p1.X, p1.Y, p1.Z);
                    if (Shape2DExtensions.IsDuplicate(vertex, foundVertices))
                        continue;
                    // Skip edge-edge crossings, but keep corner touches
                    if (self.OnEdges(vertex, selfEdges) && self.OnEdges(vertex, otherEdges)
                        && !Shape2DExtensions.IsDuplicate(vertex, selfVertices) && !Shape2DExtensions.IsDuplicate(vertex, otherVertices))
                        continue;
                    // Keep only vertices that aren't boundaries of higher-dimensional contacts
                    if (!self.OnFaces(vertex, foundFaces) && !self.OnEdges(vertex, foundEdges))
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
                throw Failures.Expected(ModelingDiagnostics.OffsetFailed, $"The offset of the {GetType().Name} by {Formatting.Number(amount)} failed.",
                                        offsetResult.IsFailure ? offsetResult.Diagnostics : default);
            var offset = built.Shape;
            var result = Shape.Cast(offset);
            // The builder always returns a shell; a face offsets to a single face
            if (self.AsShape().TryAs<Face>(out _) && result.SelectFaces().Count == 1)
                result = result.SelectFaces()[0];
            if (result.Area <= Constants.Tolerance)
                throw new InvalidOperationException($"Offsetting by {amount} collapsed the {GetType().Name} onto itself");
            return self.CopyAttributesTo(result);
        }
    }

    extension(IShape2D)
    {
        public static IShape2D Make(Shape? obj) => (IShape2D)TopoUtil.Downcast(obj!);
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
                    || segment.SelectVertices().Any(v => vertex.DistanceTo(v) <= Constants.Tolerance))
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

/// <summary>The offset of faces and shells, keeping the caller's class.</summary>
public static class Shape2DOperations
{
    extension<T>(T shape) where T : IShape2D
    {
        /// <summary>
        /// The face or shell offset along its own normals (see <see cref="IShape2D"/>): a positive amount follows the normal.
        /// </summary>
        public T Offset(double amount) => (T)shape.OffsetCore(amount);
    }
}
