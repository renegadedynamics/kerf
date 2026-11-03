using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Features;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Offsets;
using Kerf.Geometry;
using Occt;

namespace Build123d;

/// <summary>The methods shared by faces and shells (build123d's <c>Mixin2D</c>).</summary>
public abstract partial class Shape2D : Shape
{
    private protected Shape2D(global::Kerf.Topology.Shape? obj, string label = "", Color? color = null) : base(obj, label, color) { }

    internal override int? Dim => 2;

    /// <summary>
    /// This face or shell joined with other 2D shapes (build123d's <c>+</c>). When a shell is involved the faces are
    /// sewn into one shell, which joining sheet surfaces along shared edges needs; otherwise they are fused as for any
    /// shape, so coplanar faces make a Face or Sketch.
    /// </summary>
    /// <exception cref="ArgumentException">The operands aren't all 2D, or the faces don't sew into one connected shell.</exception>
    public override Shape Add(params IEnumerable<Shape?> other)
    {
        var summands = Operands(other);
        if (summands.Count == 0)
            return this;
        // Only sew when a shell is being built up
        if (this is not Shell && !summands.Any(s => s is Shell))
            return base.Add(other);
        if (summands.Any(s => s.Dim != 2))
            throw new ArgumentException("Only shapes with the same dimension can be added");
        var faces = Faces().Concat(summands.SelectMany(s => s.Faces())).ToList();
        Shape sum;
        try
        {
            sum = new Shell(faces);
        }
        catch (Exception e) when (e is ArgumentException or InvalidOperationException or FailureException)
        {
            throw new ArgumentException(
                "Unable to sew faces into a single connected Shell - faces must meet along shared edges, "
                + "with at most two faces on an edge", e);
        }
        if (SkipClean.Clean)
            sum = sum.Clean();
        return CopyAttributesTo(sum);
    }

    /// <summary>This face or shell with its normal reversed (build123d's unary <c>-</c>).</summary>
    public static Shape2D operator -(Shape2D shape)
    {
        if (shape.IsEmpty)
            throw new InvalidOperationException("Invalid Shape");
        // As the surface has changed, where it was taken from no longer applies
        return (Shape2D)shape.Rewrapped(shape.TopoShape.Complemented()).WithTopoPath([]);
    }

    /// <summary>
    /// The faces of this shape divided by a closed perimeter lying on it: those inside and those outside it. Each side is
    /// one face, a shell of several, or (when they don't sew) a list; the empty Face for a side with nothing on it.
    /// </summary>
    /// <remarks>This may fail if the perimeter crosses the shape's edges.</remarks>
    /// <exception cref="ArgumentException">The perimeter isn't closed.</exception>
    public (IReadOnlyList<Face> Inside, IReadOnlyList<Face> Outside) SplitByPerimeter(Shape1D perimeter)
    {
        var (inside, outside) = SplitByPerimeterCore(perimeter);
        return (inside.Faces(), outside.Faces());
    }

    /// <summary>
    /// The part of this shape inside (<see cref="Keep.Inside"/>, the default) or outside (<see cref="Keep.Outside"/>) a
    /// closed perimeter lying on it: a Face, a Shell, a Compound of faces that don't sew, or the empty Face.
    /// </summary>
    public Shape SplitByPerimeter(Shape1D perimeter, Keep keep)
    {
        if (keep is not (Keep.Inside or Keep.Outside))
            throw new ArgumentException("keep must be one of Keep.INSIDE, Keep.OUTSIDE, or Keep.BOTH (use the overload without keep for both)");
        var (inside, outside) = SplitByPerimeterCore(perimeter);
        return keep == Keep.Inside ? inside : outside;
    }

    private (Shape Inside, Shape Outside) SplitByPerimeterCore(Shape1D perimeter)
    {
        if (!perimeter.IsClosed)
            throw new ArgumentException("perimeter must be a closed Wire or Edge");

        Shape ProcessSides(List<Shape> sides)
        {
            if (sides.Count == 0)
                return new Face();
            if (sides.Count == 1)
                return sides[0];
            // Try to make a shell
            var shell = TopoUtil.SewFaces(sides.Select(s => s.TopoShape));
            return shell.IsShell() ? CastLike(shell) : new Compound(sides);
        }

        // Split each perimeter edge where it crosses a seam, so the pieces lie in one patch of the surface
        var perimeterEdges = new NCollection_Sequence<global::Kerf.Topology.Shape>();
        var seams = Faces().SelectMany(f => f.Seams).ToList();
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
            foreach (var piece in SplitEdgeAtVertices(edge, seamVertices))
                perimeterEdges.Append(piece.TopoShape);
        }

        var constructor = new SplitShape(TopoShape);
        constructor.Add(perimeterEdges);
        constructor.Build();
        List<Shape> Sides(IEnumerable<global::Kerf.Topology.Shape> list) => [.. list.Where(s => !s.IsNull).Select(CastLike)];
        Shape left, right;
        var lefts = constructor.Left();
        left = ProcessSides(Sides(lefts));
        var rights = constructor.Right();
        right = ProcessSides(Sides(rights));

        // Is the left or the right the inside?
        var perimeterLength = perimeter.Length;
        var leftLength = left.IsEmpty ? 0 : left.Edges().Sum(e => e.Length);
        var rightLength = right.IsEmpty ? 0 : right.Edges().Sum(e => e.Length);
        bool leftInside = Math.Abs(perimeterLength - leftLength) < Math.Abs(perimeterLength - rightLength);
        return leftInside ? (left, right) : (right, left);
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
                next.AddRange(TwoD.SplitEdgeAtVertex(segment, vertex).Select(e => new Edge(e)));
            }
            segments = next;
        }
        return segments;
    }

    /// <summary>The points where an axis meets the surface, nearest the axis's start first, each with the surface normal there.</summary>
    public IReadOnlyList<(Vector Point, Vector Normal)> FindIntersectionPoints(Axis other, double tolerance = Constants.Tolerance) =>
        SurfaceIntersection.FindPoints(this, other, tolerance);

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
            case Shape2D surface:
            {
                // The common part (coplanar overlap) and the section (crossing curves)
                ShapeList<Shape> commonFaces;
                var common = new Common();
                commonFaces = BoolOpList([this], [surface], common).Expand();
                results.AddRange(commonFaces);
                ShapeList<Shape> sectionEdges;
                var section = new SectionOperation();
                // The section's vertices are boundary contacts (touches), not intersections
                sectionEdges = new ShapeList<Shape>(BoolOpList([this], [surface], section).OfType<Edge>()).Expand();
                if (commonFaces.Count == 0)
                    results.AddRange(sectionEdges);
                else
                {
                    // Leave out edges on the boundary of the overlap, which aren't crossings
                    var commonEdges = commonFaces.SelectMany(f => f.Edges()).Select(ce => (Edge: ce, Box: ce.BoundingBox(optimal: false))).ToList();
                    foreach (var edge in sectionEdges)
                    {
                        var box = edge.BoundingBox(optimal: false);
                        if (!commonEdges.Any(ce => box.Intersects(ce.Box, tolerance) && edge.DistanceTo(ce.Edge) <= tolerance))
                            results.Add(edge);
                    }
                }
                break;
            }
            case Shape1D line:
            {
                var section = new SectionOperation();
                results.AddRange(BoolOpList([this], [line], section));
                break;
            }
            case Vertex vertex:
                if (vertex.DistanceTo(this) <= tolerance)
                    results.Add(vertex);
                break;
            default:
                // Delegate to higher-order shapes
                if (shape.IntersectCore(this, tolerance, includeTouched) is { } found)
                    results.AddRange(found);
                break;
        }
        if (includeTouched && shape is Shape2D)
            results.AddRange(TouchCore(shape, tolerance, results.OfType<Face>().ToList(), results.OfType<Edge>().ToList()));
        return results.Count > 0 ? new ShapeList<Shape>(results) : null;
    }

    /// <summary>
    /// Where this face or shell touches another at its boundary: for two faces, the vertices where they meet at a corner
    /// or cross at a point without an edge or face in common.
    /// </summary>
    public override ShapeList<Shape> Touch(Shape other, double tolerance = 1e-6) => TouchCore(other, tolerance, null, null);

    internal ShapeList<Shape> TouchCore(Shape other, double tolerance, List<Face>? foundFaces, List<Edge>? foundEdges)
    {
        if (other is not Shape2D)
            // Face + edge or vertex: no touch (the intersection already covers them); a compound iterates
            return other.Touch(this, tolerance);

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
            if (IntersectCore(other, tolerance, includeTouched: false) is { } found)
            {
                foundFaces.AddRange(found.OfType<Face>());
                foundEdges.AddRange(found.OfType<Edge>());
            }
        }
        foundEdges ??= [];

        var results = new List<Shape>();
        var foundVertices = new List<Vertex>();
        // All contact points (vertex-vertex, vertex-edge, vertex-face), with high precision to avoid duplicates
        var extrema = new DistanceShapeShape();
        extrema.SetDeflection(tolerance * 1e-3);
        extrema.LoadS1(TopoShape);
        extrema.LoadS2(other.TopoShape);
        extrema.Perform();
        if (extrema.IsDone && extrema.Value() <= tolerance)
        {
            var selfEdges = Edges();
            var otherEdges = other.Edges();
            var selfVertices = Vertices();
            var otherVertices = other.Vertices();
            for (int i = 1; i <= extrema.SolutionCount; i++)
            {
                var p1 = extrema.PointOnShape1(i);
                var p2 = extrema.PointOnShape2(i);
                if (p1.Distance(p2) > tolerance)
                    continue;
                var vertex = new Vertex(p1.X(), p1.Y(), p1.Z());
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
            return this;
        var builder = new MakeOffset();
        builder.Initialize(TopoShape, amount, Constants.Tolerance, OffsetMode.Skin, true, false,
                           JoinKind.Intersection, false, true);
        builder.MakeOffsetShape();
        var offset = builder.Shape();
        if (offset.IsNull)
            throw new InvalidOperationException($"Unable to offset {GetType().Name} by {amount}");
        var result = Cast(offset);
        // The builder always returns a shell; a face offsets to a single face
        if (this is Face && result.Faces().Count == 1)
            result = result.Faces()[0];
        if (result.Area <= Constants.Tolerance)
            throw new InvalidOperationException($"Offsetting by {amount} collapsed the {GetType().Name} onto itself");
        return CopyAttributesTo(result);
    }
}

/// <summary>The offset of faces and shells, keeping the caller's class.</summary>
public static class Shape2DOperations
{
    extension<T>(T shape) where T : Shape2D
    {
        /// <summary>
        /// The face or shell offset along its own normals (see <see cref="Shape2D"/>): a positive amount follows the normal.
        /// </summary>
        public T Offset(double amount) => (T)shape.OffsetCore(amount);
    }
}
