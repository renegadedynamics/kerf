namespace Kerf.Modeling;

public static partial class Operations
{
    // ---- full_round ----

    /// <summary>
    /// The face an edge was taken from, with the edge replaced by the arc of the largest empty circle that fits in the
    /// face between the edge and its two neighbours: the end of the face rounded off (or, with <paramref name="invert"/>,
    /// hollowed out).
    /// </summary>
    /// <param name="edge">The edge to replace, taken from a face or sketch.</param>
    /// <param name="parent">The shape the edge is an edge of, which is rebuilt.</param>
    /// <param name="invert">Make the arc concave instead of convex.</param>
    /// <param name="voronoiPointCount">
    /// The number of points along each edge used to find the Voronoi vertices, the candidates for the centre of the
    /// largest empty circle.
    /// </param>
    /// <exception cref="ArgumentException">The edge wasn't taken from a shape, or the geometry is unsuitable.</exception>
    public static Sketch FullRound(Edge edge, Shape parent, bool invert = false, int voronoiPointCount = 100)
    {
        // Phase 5: in builder mode the result is combined with mode (replacing the builder's sketch by default)

        // Generate a set of evenly spaced points along the given edge and the edges connected to it and use them to
        // generate the Voronoi vertices as possible locations for the center of the largest empty circle. Note:
        // full_round could be enhanced to handle the case of a face composed of two edges.
        var connectedEdges = OneD.ConnectedEdges(edge, parent);
        if (connectedEdges.Count != 2)
            throw new ArgumentException("Invalid geometry - 3 or more edges required");

        List<Edge> edgeGroup = [edge, .. connectedEdges];
        var fractions = Enumerable.Range(0, voronoiPointCount + 1).Select(i => i / (double)voronoiPointCount).ToList();
        var voronoiEdgePoints = edgeGroup.SelectMany(e => e.Positions(fractions)).ToList();
        var voronoiVertices = VoronoiVertices(voronoiEdgePoints.Select(p => (p.X, p.Y)).ToList())
            .Select(v => new Point3D(v.X, v.Y, 0)).ToList();

        // Refine the largest empty circle center estimate by averaging the best three candidates. The minimum distance
        // between the edges and this center is the circle radius.
        var bestThree = new List<(double Difference, int Index)>
        {
            (double.PositiveInfinity, 0), (double.PositiveInfinity, 0), (double.PositiveInfinity, 0),
        };
        for (int i = 0; i < voronoiVertices.Count; i++)
        {
            var distances = edgeGroup.Select(e => e.DistanceTo(voronoiVertices[i])).ToList();
            var avgDistance = distances.Sum() / 3;
            var difference = distances.Max(d => Math.Abs(d - avgDistance));

            // Prefer vertices with minimal difference
            if (difference < bestThree[^1].Difference)
            {
                bestThree[^1] = (difference, i);
                bestThree = [.. bestThree.OrderBy(b => b.Difference)];
            }
        }

        // Refine by averaging the best three
        var voronoiCircleCenter = (bestThree.Aggregate(Vector3D.Zero, (sum, b) => sum + voronoiVertices[b.Index].ToVector()) * (1.0 / 3)).ToPoint();

        // Determine where the connected edges intersect with the largest empty circle
        var connectedEdgesEndPoints = connectedEdges.Select(e => e.DistanceToWithClosestPoints(voronoiCircleCenter).OnThis).ToList();

        // Determine where the target edge intersects with the largest empty circle
        var middleEdgeArcPoint = edge.DistanceToWithClosestPoints(voronoiCircleCenter).OnThis;

        // Trim the connected edges to allow room for the circular feature
        var origin = (connectedEdgesEndPoints.Aggregate(Vector3D.Zero, (sum, p) => sum + p.ToVector()) / 2).ToPoint();
        var xDir = (connectedEdgesEndPoints[1] - connectedEdgesEndPoints[0]).Normalized();
        var toArcVec = origin - middleEdgeArcPoint;
        // Project `to_arc_vec` onto the plane perpendicular to `x_dir`
        var zDir = (toArcVec - xDir * toArcVec.Dot(xDir)).Normalized();

        var splitPlane = new Plane(origin, xDir, zDir);
        var trimmedConnectedEdges = new List<Edge>();
        foreach (var connected in connectedEdges)
        {
            var trimmed = connected.Split(splitPlane);
            if (trimmed.Count == 0)
                throw new ArgumentException("Invalid geometry to create the end arc");
            trimmedConnectedEdges.Add((Edge)trimmed[0]);
        }

        // Flip the middle point if the user wants the concave solution
        if (invert)
            middleEdgeArcPoint = (voronoiCircleCenter.ToVector() * 2 - middleEdgeArcPoint.ToVector()).ToPoint();

        // Generate the new circular edge
        var newArc = Edge.MakeThreePointArc(connectedEdgesEndPoints[0], middleEdgeArcPoint, connectedEdgesEndPoints[1]);

        // Recover other edges
        var otherEdges = parent.Edges().Except(OneD.ConnectedEdges(edge, parent)).Except([edge]);

        // Rebuild the face. Note that the longest wire must be the perimeter and others holes
        var faceWires = Wire.Combine([.. trimmedConnectedEdges, newArc, .. otherEdges]).OrderByDescending(ShapeKey.Length);
        var pendingFace = Face.Make(faceWires[0], faceWires.Skip(1));

        // Flip the face to match the original parent
        if (!parent.Faces()[0].NormalAt().IsNear(pendingFace.NormalAt()))
            pendingFace = (Face)(-pendingFace);

        return new Sketch([pendingFace]);
    }

    /// <summary>
    /// The vertices of the Voronoi diagram of points in the plane (scipy's <c>Voronoi(points).vertices</c>): the centres of
    /// the circumcircles of their Delaunay triangulation, which is built by Bowyer-Watson insertion. Coincident points
    /// count once, and coincident centres (of cocircular points) are one vertex, as Qhull merges them.
    /// </summary>
    internal static List<(double X, double Y)> VoronoiVertices(IReadOnlyList<(double X, double Y)> input)
    {
        double minX = input.Min(p => p.X), maxX = input.Max(p => p.X), minY = input.Min(p => p.Y), maxY = input.Max(p => p.Y);
        var extent = Math.Max(Math.Max(maxX - minX, maxY - minY), double.Epsilon);
        var pointTolerance = extent * 1e-12;

        var points = new List<(double X, double Y)>();
        foreach (var p in input)
            if (!points.Any(q => Math.Abs(q.X - p.X) <= pointTolerance && Math.Abs(q.Y - p.Y) <= pointTolerance))
                points.Add(p);
        int n = points.Count;

        // A triangle large enough to hold every point, whose corners are removed at the end
        double cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, size = extent * 100;
        points.Add((cx - 2 * size, cy - size));
        points.Add((cx + 2 * size, cy - size));
        points.Add((cx, cy + 2 * size));

        var triangles = new List<DelaunayTriangle> { DelaunayTriangle.Of(points, n, n + 1, n + 2) };
        for (int i = 0; i < n; i++)
        {
            var (px, py) = points[i];
            var bad = triangles.Where(t => t.Encloses(px, py)).ToList();
            // The boundary of the cavity: the edges of the bad triangles not shared between two of them
            var edgeCounts = new Dictionary<(int, int), int>();
            foreach (var t in bad)
                foreach (var e in t.Edges())
                    edgeCounts[e] = edgeCounts.GetValueOrDefault(e) + 1;
            triangles.RemoveAll(bad.Contains);
            foreach (var t in bad)
                foreach (var (a, b) in t.Edges())
                    if (edgeCounts[(a, b)] == 1)
                        triangles.Add(DelaunayTriangle.Of(points, a, b, i));
        }

        var centerTolerance = extent * 1e-9;
        var vertices = new List<(double X, double Y)>();
        foreach (var t in triangles)
        {
            if (t.A >= n || t.B >= n || t.C >= n || double.IsInfinity(t.RadiusSquared))
                continue;
            if (!vertices.Any(v => Math.Abs(v.X - t.X) <= centerTolerance && Math.Abs(v.Y - t.Y) <= centerTolerance))
                vertices.Add((t.X, t.Y));
        }
        return vertices;
    }

    /// <summary>A triangle of a Delaunay triangulation: its corners (indices) and circumcircle.</summary>
    private sealed record DelaunayTriangle(int A, int B, int C, double X, double Y, double RadiusSquared)
    {
        public static DelaunayTriangle Of(List<(double X, double Y)> points, int a, int b, int c)
        {
            var (ax, ay) = points[a];
            var (bx, by) = points[b];
            var (qx, qy) = points[c];
            var d = 2 * (ax * (by - qy) + bx * (qy - ay) + qx * (ay - by));
            if (d == 0)
                // Degenerate: every point is taken as inside, so the triangle is replaced as soon as possible
                return new DelaunayTriangle(a, b, c, 0, 0, double.PositiveInfinity);
            var a2 = ax * ax + ay * ay;
            var b2 = bx * bx + by * by;
            var c2 = qx * qx + qy * qy;
            var x = (a2 * (by - qy) + b2 * (qy - ay) + c2 * (ay - by)) / d;
            var y = (a2 * (qx - bx) + b2 * (ax - qx) + c2 * (bx - ax)) / d;
            return new DelaunayTriangle(a, b, c, x, y, (ax - x) * (ax - x) + (ay - y) * (ay - y));
        }

        /// <summary>Whether a point is strictly inside the circumcircle (points on it leave the triangle as it is).</summary>
        public bool Encloses(double px, double py)
        {
            if (double.IsInfinity(RadiusSquared))
                return true;
            var dx = px - X;
            var dy = py - Y;
            return dx * dx + dy * dy < RadiusSquared * (1 - 1e-12);
        }

        /// <summary>The edges, each as its corner indices in increasing order.</summary>
        public IEnumerable<(int, int)> Edges()
        {
            yield return A < B ? (A, B) : (B, A);
            yield return B < C ? (B, C) : (C, B);
            yield return C < A ? (C, A) : (A, C);
        }
    }

    // ---- make_face ----

    /// <summary>A face (in a Sketch) from perimeter edges, wires or curves, which must join into one closed wire.</summary>
    /// <exception cref="ArgumentException">An object isn't an edge, wire or curve.</exception>
    public static Sketch MakeFace(IEnumerable<Shape> edges)
    {
        // Phase 5: in builder mode the edges default to the pending edges, and the face is combined with mode
        if (edges is null)
            throw new ArgumentException("No objects to create a face");
        var outerEdges = new List<Edge>();
        foreach (var item in edges)
        {
            if (item.IsWire() || item.IsCompound())
                outerEdges.AddRange(item.Edges());
            else
                outerEdges.Add(item.IsEdge() ? (Edge)item : throw new ArgumentException($"Unable to make a face from a {TopoUtil.KindName(item)}"));
        }
        if (outerEdges.Count == 0)
            return new Sketch();   // a face of no edges is nothing

        var pendingFace = Face.Make(Wire.Combine(outerEdges)[0]);
        if (pendingFace.NormalAt().Z < 0)   // flip up-side-down faces
            pendingFace = (Face)(-pendingFace);
        return new Sketch([pendingFace]);
    }

    /// <inheritdoc cref="MakeFace(IEnumerable{Shape})"/>
    public static Sketch MakeFace(Shape edges) => MakeFace([edges]);

    // ---- make_hull ----

    /// <summary>A face (in a Sketch) from the convex hull of the edges.</summary>
    public static Sketch MakeHull(IEnumerable<Edge> edges)
    {
        // Phase 5: in builder mode the edges default to the pending edges and the sketch's edges, and the face is
        // combined with mode
        if (edges is null)
            throw new ArgumentException("No objects to create a hull");
        var hullEdges = edges.ToList();
        if (hullEdges.Count == 0)
            return new Sketch();   // a hull of no edges is nothing

        var pendingFace = Face.Make(Wire.MakeConvexHull(hullEdges));
        if (pendingFace.NormalAt().Z < 0)   // flip up-side-down faces
            pendingFace = (Face)(-pendingFace);
        return new Sketch([pendingFace]);
    }

    /// <inheritdoc cref="MakeHull(IEnumerable{Edge})"/>
    public static Sketch MakeHull(Edge edge) => MakeHull([edge]);

    // ---- trace ----

    /// <summary>
    /// Faces made from edges, wires or curves by sweeping a perpendicular line of <paramref name="lineWidth"/> along them,
    /// fused into a Sketch. Connected edges are traced together, for neat transitions.
    /// </summary>
    public static Sketch Trace(IEnumerable<Shape> lines, double lineWidth = 1)
    {
        // Phase 5: in builder mode the lines default to the pending edges, and the faces are combined with mode
        if (lines is null)
            throw new ArgumentException("No objects to trace");
        var traceEdges = lines.SelectMany(l => l.Edges()).ToList();
        if (traceEdges.Count == 0)
            return new Sketch();   // tracing nothing is nothing

        // Group the edges into wires to allow for nice transitions
        var traceWires = Wire.Combine(traceEdges);

        var newFaces = new List<Face>();
        foreach (var toTrace in traceWires)
        {
            var tracePen = toTrace.PerpendicularLine(lineWidth, 0);
            newFaces.AddRange(Shell.Sweep(tracePen, toTrace, Transition.Right).Faces());
        }

        var combinedFaces = newFaces.Count > 1 ? newFaces[0].Fuse(newFaces.Skip(1)) : newFaces[0];
        return new Sketch(combinedFaces);
    }

    /// <inheritdoc cref="Trace(IEnumerable{Shape}, double)"/>
    public static Sketch Trace(Shape lines, double lineWidth = 1) => Trace([lines], lineWidth);
}
