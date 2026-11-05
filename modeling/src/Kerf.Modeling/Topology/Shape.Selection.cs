namespace Kerf.Modeling;

public static partial class ShapeExtensions
{
    /// <summary>The sub-shapes of a type, the list noting that they were taken out of this shape.</summary>
    internal static ShapeList<T> ShapeListOf<T>(IShape origin) where T : struct, IShape<T>
    {
        if (origin.IsEmpty)
            return ShapeList<T>.From(origin);
        return new ShapeList<T>(TopoUtil.Entities(origin.AsShape(), T.Type).Select(e => e.As<T>()), origin);
    }

    /// <summary>The one sub-shape of a type.</summary>
    /// <exception cref="InvalidOperationException">There isn't exactly one.</exception>
    internal static T SingleShapeOf<T>(IShape origin) where T : struct, IShape<T>
    {
        var list = ShapeListOf<T>(origin);
        if (list.Count != 1)
            throw new InvalidOperationException($"Expected exactly one {T.Type.ToString().ToLowerInvariant()}, found {list.Count}");
        return list[0];
    }

    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>The distinct kernel sub-shapes of a type.</summary>
        public IReadOnlyList<Shape> Entities(ShapeType type) =>
            self.IsEmpty ? [] : TopoUtil.Entities(self.AsShape(), type);

        /// <summary>All the vertices in this shape, as a selection (a vertex holds itself).</summary>
        public ShapeList<Vertex> SelectVertices() =>
            self.AsShape() is { IsNull: false } s && s.IsVertex() ? new ShapeList<Vertex>([(Vertex)s], (IShape)self) : ShapeListOf<Vertex>(self);

        /// <summary>All the non-degenerate edges in this shape, as a selection (a wire's in the order they connect).</summary>
        public ShapeList<Edge> SelectEdges()
        {
            var shape = self.AsShape();
            if (!shape.IsNull && shape.IsWire())
                return ((Wire)shape).OrderedEdgeList(self);
            var all = ShapeListOf<Edge>(self);
            return all.Where(e => !e.IsDegenerated);
        }

        /// <summary>All the wires in this shape, as a selection.</summary>
        public ShapeList<Wire> SelectWires() => ShapeListOf<Wire>(self);

        /// <summary>All the faces in this shape, as a selection.</summary>
        public ShapeList<Face> SelectFaces() => ShapeListOf<Face>(self);

        /// <summary>All the shells in this shape, as a selection.</summary>
        public ShapeList<Shell> SelectShells() => ShapeListOf<Shell>(self);

        /// <summary>All the solids in this shape, as a selection.</summary>
        public ShapeList<Solid> SelectSolids() => ShapeListOf<Solid>(self);

        /// <summary>The one vertex in this shape (build123d's <c>vertex()</c>).</summary>
        /// <exception cref="InvalidOperationException">The shape doesn't have exactly one.</exception>
        public Vertex SingleVertex() => SingleShapeOf<Vertex>(self);

        /// <summary>The one edge in this shape (build123d's <c>edge()</c>).</summary>
        public Edge SingleEdge() => SingleShapeOf<Edge>(self);

        /// <summary>The one wire in this shape (build123d's <c>wire()</c>).</summary>
        public Wire SingleWire() => SingleShapeOf<Wire>(self);

        /// <summary>The one face in this shape (build123d's <c>face()</c>).</summary>
        public Face SingleFace() => SingleShapeOf<Face>(self);

        /// <summary>The one shell in this shape (build123d's <c>shell()</c>).</summary>
        public Shell SingleShell() => SingleShapeOf<Shell>(self);

        /// <summary>The one solid in this shape (build123d's <c>solid()</c>).</summary>
        public Solid SingleSolid() => SingleShapeOf<Solid>(self);

        /// <summary>The one compound in this shape (build123d's <c>compound()</c>).</summary>
        public Compound SingleCompound() => SingleShapeOf<Compound>(self);

        /// <summary>
        /// The non-compound shapes at this shape's first level: a compound's children, looking through nested compounds;
        /// any other shape is its own first level.
        /// </summary>
        public ShapeList<Shape> GetTopLevelShapes() =>
            self.IsEmpty ? [] : new ShapeList<Shape>(TopoUtil.TopLevelShapes(self.AsShape()));

        // ---- Convexity ----

        /// <summary>
        /// How this shape sits around a face, edge or vertex of it: Convex on the outer edge of a box, Concave at the inner
        /// corner of a pocket, Smooth where a fillet meets the face it blends into. See <c>ConvexityOf</c> and
        /// <c>ConvexityOf</c>.
        /// </summary>
        public Convexity ConvexityOf(Face face) => face.Convexity;

        /// <summary>
        /// How this shape sits around an edge of it: Convex on the outer edge of a box, Concave at the inner corner of a
        /// pocket, Smooth where a fillet meets the face it blends into, and Saddle for an edge that turns one way at one end
        /// and the other way at the other. A seam edge is Smooth.
        /// </summary>
        /// <exception cref="InvalidOperationException">The edge lies on one face only, or more than two faces meet at it.</exception>
        public Convexity ConvexityOf(Edge edge) => Shape1DExtensions.ConvexityIn(edge, self.AsShape());

        /// <summary>
        /// How this shape sits around a vertex of it. Through a face, the vertex is a corner of that face: Convex at the
        /// corner of a plate, Concave at the corner of a slot or the step in an L, Smooth where two edges meet in line.
        /// Without one, on a shape of faces alone it is a corner of the one face that holds it; on a solid or shell it is
        /// classified by the edges meeting there, Convex where every crease is convex, Concave where every crease is
        /// concave, and Saddle where both kinds meet.
        /// </summary>
        /// <exception cref="InvalidOperationException">The vertex's face or edges can't be classified.</exception>
        public Convexity ConvexityOf(Vertex vertex, Face? through = null) => VertexExtensions.ConvexityIn(vertex, self.AsShape(), through);

        /// <summary>The convexity of a face, edge or vertex of this shape, as its kind has one.</summary>
        /// <exception cref="InvalidOperationException">The shape is of a kind that has none.</exception>
        internal Convexity ConvexityOfAny(Shape sub, Face? through)
        {
            if (!sub.IsNull)
            {
                if (sub.TryAs(out Face face) && sub.IsFace())
                    return self.ConvexityOf(face);
                if (sub.TryAs(out Edge edge) && sub.IsEdge())
                    return self.ConvexityOf(edge);
                if (sub.TryAs(out Vertex vertex) && sub.IsVertex())
                    return self.ConvexityOf(vertex, through);
            }
            throw new InvalidOperationException(
                $"a {TopoUtil.KindName(sub)} has no convexity - only a Vertex, Edge or Face selected from a larger shape does");
        }
    }
}
