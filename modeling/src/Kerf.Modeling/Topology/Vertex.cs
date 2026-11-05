using Kerf.Construction;
using Kerf.Measurement;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// A zero-dimensional point in the topology: it marks the ends of edges and holds their coordinates.
/// </summary>
public static partial class VertexExtensions
{
    extension(Vertex self)
    {

        internal Vertex TopoVertex => (Vertex)self;

        internal int? Dim_Vertex => 0;

        internal double Order_Vertex => 0.0;

        internal Shape MakeZero_Vertex() => default(Vertex);

        /// <summary>The volume of a vertex, which is zero.</summary>
        internal double Volume_Vertex => 0.0;

        private Point3D Point
        {
            get
            {
                self.NeedsGeometry("position");
                return self.Point;
            }
        }

        /// <summary>The X coordinate, including the vertex's Location.</summary>
        public double X { get { var p = self.Point; return p.X; } }

        /// <summary>The Y coordinate, including the vertex's Location.</summary>
        public double Y { get { var p = self.Point; return p.Y; } }

        /// <summary>The Z coordinate, including the vertex's Location.</summary>
        public double Z { get { var p = self.Point; return p.Z; } }

        /// <summary>The coordinates of the vertex.</summary>
        public void Deconstruct(out double x, out double y, out double z)
        {
            var p = self.Point;
            (x, y, z) = (p.X, p.Y, p.Z);
        }

        /// <summary>The point the vertex is at.</summary>
        public Vector ToVector()
        {
            var p = self.Point;
            return new Vector(p);
        }

        /// <summary>The centre of a vertex is itself.</summary>
        internal Vector Center_Vertex()
        {
            self.NeedsGeometry("center");
            return self.ToVector();
        }

        /// <summary>This vertex.</summary>
        internal ShapeList<Vertex> Vertices_Vertex(Select select = Select.All) =>
            self.SelectFrom<Vertex>(self.IsEmpty ? [] : [self], select);

        /// <summary>Vertices can't be intersected with <c>&amp;</c>.</summary>
        internal Shape CommonCore_Vertex(IEnumerable<object> others) =>
            throw new NotSupportedException("Vertices can't be intersected");

        internal Shape TransformShapeCore_Vertex(Matrix matrix) => Vertex.Make(matrix.Multiply(self.ToVector()));

        internal ShapeList<Shape>? IntersectCore_Vertex(object other, double tolerance, bool includeTouched)
        {
            switch (other)
            {
                case Vector v:
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
            return ((Shape)other).IntersectCore(self, tolerance, includeTouched);
        }

        // ---- Convexity ----

        /// <summary>
        /// How the shape this vertex was selected from sits around it.
        /// </summary>
        /// <remarks>
        /// <para>
        /// Selected through a face (<c>face.Vertices()</c>, or <c>face.Edges()[0].Vertices()</c>) the vertex is a corner of
        /// that face: Convex at the corner of a plate, Concave at the corner of a slot or the step in an L, Smooth where two
        /// edges meet in line. The corner is read in the face's own parameters, so a corner of a curved face classifies the
        /// same way as one of a flat face. Selected straight off a sketch or another shape of faces alone, the vertex is a
        /// corner of the one face that holds it.
        /// </para>
        /// <para>
        /// Selected straight off a solid or shell, the vertex is classified by the edges meeting there: Convex where every
        /// crease is convex (the corner of a box), Concave where every crease is concave (the bottom corner of a pocket), and
        /// Saddle where both kinds meet (the inner corner of a step). Smooth edges don't count.
        /// </para>
        /// </remarks>
        /// <exception cref="InvalidOperationException">
        /// The vertex has no <see cref="Shape.TopoParent"/>, or its face or edges can't be classified.
        /// </exception>
        internal Convexity Convexity_Vertex
        {
            get
            {
                var face = self.OwningFace() ?? self.OnlyFace();
                return face is { } f ? self.CornerConvexity(f) : self.CreaseConvexity();
            }
        }

        /// <summary>The innermost face this vertex was selected through, if any.</summary>
        private Face? OwningFace()
        {
            for (int i = self.TopoPath.Count - 1; i >= 0; i--)
                if (!self.TopoPath[i].IsEmpty && self.TopoPath[i].Type == ShapeType.Face)
                    return (Face)self.TopoPath[i];
            return null;
        }

        /// <summary>The one face of a sketch-like parent (no solids or shells) that holds this vertex.</summary>
        private Face? OnlyFace()
        {
            var parent = self.TopoParent;
            if (parent is null || parent.IsEmpty || parent.SelectSolids().Count > 0 || parent.SelectShells().Count > 0)
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
                    $"this vertex is a corner of {holding.Count} faces of the {parent.GetType().Name} it was selected from, "
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
        private Convexity CreaseConvexity()
        {
            var parent = self.TopoParent;
            if (parent is null || parent.IsEmpty)
                throw new InvalidOperationException(
                    "this vertex was not selected from a shape, so there is nothing to classify it against - take it from a face or solid");
            var graph = ShapeGraph.Build(parent);
            var own = TopoUtil.FindSame(self, graph.All(ShapeType.Vertex))
                      ?? throw new InvalidOperationException("The vertex isn't part of the shape it was selected from.");
            var kinds = new HashSet<Convexity>();
            var edges = graph.Ancestors(own, ShapeType.Edge);
            foreach (var topoEdge in edges)
                kinds.Add(((Edge)Shape.Cast(topoEdge)).Convexity);
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
        /// <summary>A vertex wrapping an OCCT vertex.</summary>
        public static Vertex Make(Vertex vertex) => (Vertex)TopoUtil.Downcast(vertex!);
    }


    extension(Vertex)
    {
        /// <summary>A vertex at a point.</summary>
        public static Vertex Make(double x, double y, double z = 0) => (Vertex)TopoUtil.Downcast(VertexExtensions.Make(new Vector(x, y, z))!);
    }


    extension(Vertex)
    {
        /// <summary>A vertex at a point.</summary>
        public static Vertex Make(Vector point) => (Vertex)TopoUtil.Downcast(VertexExtensions.Make(point)!);
    }



}
