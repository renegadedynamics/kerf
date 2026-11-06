using Kerf.Construction;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>
/// A zero-dimensional point in the topology: it marks the ends of edges and holds their coordinates.
/// </summary>
public static partial class VertexExtensions
{
    extension(Vertex self)
    {

        internal Vertex TopoVertex => (Vertex)self;

        /// <summary>The X coordinate, including the vertex's Location.</summary>
        public double X { get { self.NeedsGeometry("position"); return self.Point.X; } }

        /// <summary>The Y coordinate, including the vertex's Location.</summary>
        public double Y { get { self.NeedsGeometry("position"); return self.Point.Y; } }

        /// <summary>The Z coordinate, including the vertex's Location.</summary>
        public double Z { get { self.NeedsGeometry("position"); return self.Point.Z; } }

        /// <summary>The coordinates of the vertex.</summary>
        public void Deconstruct(out double x, out double y, out double z)
        {
            var p = self.Point;
            (x, y, z) = (p.X, p.Y, p.Z);
        }

        /// <summary>The centre of a vertex is itself.</summary>
        internal Point3D CenterOfVertex()
        {
            self.NeedsGeometry("center");
            return self.Point;
        }

        
        internal ShapeList<Shape>? IntersectCoreOfVertex(object other, double tolerance, bool includeTouched)
        {
            switch (other)
            {
                case Point3D v:
                    other = Vertex.Make(v);
                    break;
                case Location l:
                    other = Vertex.Make(l.Position);
                    break;
                case Axis axis:
                    return axis.Intersect(self.Center()) is not null ? [self] : null;
                case Plane plane:
                    return plane.Contains(self.Center(), tolerance) ? [self] : null;
            }
            if (other is Vertex vertex)
                return self.DistanceTo(vertex) <= tolerance ? [self] : null;
            // Delegate to the higher-dimensional shape (including a compound)
            return ShapeExtensions.IntersectOne(((IShape)other).AsShape(), self, tolerance, includeTouched);
        }

        // ---- Convexity ----

        /// <summary>The one face of a sketch-like parent (no solids or shells) that holds this vertex.</summary>
        private Face? OnlyFace(Shape parent)
        {
            if (parent.IsEmpty || parent.Solids().Count > 0 || parent.Shells().Count > 0)
                return null;
            var holding = new List<Face>();
            foreach (var face in parent.Explore<Face>())
            {
                foreach (var vertex in face.Explore<Vertex>())
                    if (vertex.IsSame(self))
                    {
                        holding.Add(face);
                        break;
                    }
            }
            if (holding.Count > 1)
                throw new InvalidOperationException(
                    $"this vertex is a corner of {holding.Count} faces of the {TopoUtil.KindName(parent)} it was selected from, "
                    + "and classifies differently in each - select it through the face meant");
            return holding.Count == 1 ? holding[0] : null;
        }

        /// <summary>Unit directions leading away from this vertex in the face's (u, v) parameters.</summary>
        private List<(double U, double V)> FaceTangents(Face face)
        {
            var here = self.ParametersOn(face);
            var seen = new List<Edge>();
            var directions = new List<(double, double)>();
            foreach (var edge in face.Explore<Edge>())
            {
                if (seen.Any(e => edge.IsSame(e)))
                    continue;   // a seam edge is met once per side
                seen.Add(edge);
                if (!edge.TryGetCurveOnFace(face, out var curve, out var range))
                    continue;
                foreach (var (param, sign) in new[] { (range.Min, 1.0), (range.Max, -1.0) })
                {
                    var end = curve.Evaluate(param);
                    if (Math.Pow(end.X - here.X, 2) + Math.Pow(end.Y - here.Y, 2) > Constants.Tolerance)
                        continue;
                    var tangent = curve.Derivatives(param, 1).D1;
                    var length = Math.Sqrt(tangent.X * tangent.X + tangent.Y * tangent.Y);
                    if (length > 0)
                        directions.Add((sign * tangent.X / length, sign * tangent.Y / length));
                }
            }
            return directions;
        }

        /// <summary>
        /// Which way the face's boundary turns at this corner. The two edges leaving the vertex bound a wedge of less than
        /// half a turn, and their bisector points into it: material there and the corner is convex, else concave.
        /// </summary>
        private Convexity CornerConvexity(Face face)
        {
            var directions = self.FaceTangents(face);
            if (directions.Count != 2)
                throw new InvalidOperationException(
                    $"{directions.Count} edge end(s) meet this vertex on the face, expected 2 - a corner is where exactly two of them do");
            var ((u1, v1), (u2, v2)) = (directions[0], directions[1]);
            var bisector = (U: u1 + u2, V: v1 + v2);
            var span = Math.Sqrt(bisector.U * bisector.U + bisector.V * bisector.V);
            if (span < Constants.Tolerance)
                return Convexity.Smooth;   // the boundary runs straight through
            var (us, vs) = face.UVBounds;
            var step = 1e-4 * Math.Sqrt(Math.Pow(us.Max - us.Min, 2) + Math.Pow(vs.Max - vs.Min, 2)) / span;
            var here = self.ParametersOn(face);
            var probe = new Point2D(here.X + bisector.U * step, here.Y + bisector.V * step);
            return face.Classify(probe, Constants.Tolerance) == Containment.In ? Convexity.Convex : Convexity.Concave;
        }

        /// <summary>Classifies by the creases meeting at this vertex of a solid or shell.</summary>
        private Convexity CreaseConvexity(Shape parent)
        {
            if (parent.IsEmpty)
                throw new InvalidOperationException(
                    "this vertex was not selected from a shape, so there is nothing to classify it against - take it from a face or solid");
            var graph = ShapeGraph.Build(parent);
            var own = TopoUtil.FindSame(self, graph.All(ShapeType.Vertex))
                      ?? throw new InvalidOperationException("The vertex isn't part of the shape it was selected from.");
            var kinds = new HashSet<Convexity>();
            var edges = graph.Ancestors(own, ShapeType.Edge);
            foreach (var topoEdge in edges)
                kinds.Add(Shape1DExtensions.ConvexityIn((Edge)topoEdge, parent));
            kinds.Remove(Convexity.Smooth);
            return kinds.Count switch
            {
                0 => Convexity.Smooth,
                1 => kinds.Single(),
                _ => Convexity.Saddle,
            };
        }
    }

    extension(Vertex)
    {

        /// <summary>The vertex the two edges share, or the empty Vertex if they share none.</summary>
        public static Vertex CommonVertex(Edge edge1, Edge edge2)
        {
            foreach (var vertex1 in edge1.Explore<Vertex>())
                foreach (var vertex2 in edge2.Explore<Vertex>())
                    if (vertex1.IsSame(vertex2))
                        return vertex1;
            return default;
        }
    }



    extension(Vertex)
    {
        /// <summary>A vertex at a point.</summary>
        public static Vertex Make(double x, double y, double z = 0) => Vertex.Make(new Point3D(x, y, z));
    }


    extension(Vertex)
    {
        /// <summary>A vertex at a point.</summary>
        public static Vertex Make(Point3D point) => Vertex.At(point);
    }



    /// <summary>How <paramref name="parent"/> sits around a vertex of it (see <c>ConvexityOf</c>).</summary>
    internal static Convexity ConvexityIn(Vertex vertex, Shape parent, Face? through)
    {
        if (TopoUtil.IsNothing(parent))
            throw new InvalidOperationException(
                "this vertex was not selected from a shape, so there is nothing to classify it against - take it from a face or solid");
        var face = through ?? (parent.IsFace() ? (Face)parent : vertex.OnlyFace(parent));
        return face is { } f ? vertex.CornerConvexity(f) : vertex.CreaseConvexity(parent);
    }
}
