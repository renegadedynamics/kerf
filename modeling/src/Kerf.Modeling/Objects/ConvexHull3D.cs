using System.Numerics;

namespace Kerf.Modeling;

/// <summary>
/// The convex hull of a set of points, as a solid with one planar face for each flat side (decision 24001).
/// </summary>
/// <remarks>
/// <para>The steps, and what each guarantees:</para>
/// <list type="number">
/// <item>Points closer than <see cref="MergeDistance"/> are one point (the first). Every point dropped is within that
/// distance of one kept, so the hull moves by no more than it.</item>
/// <item>The hull of what's left is computed with exact orientation tests (a floating-point test with Shewchuk's error
/// bound, and exact integers when that can't decide), so it is a correct triangulated hull of those points: a closed
/// surface, every triangle facing out, the triangles seen from a point outside always one connected patch.</item>
/// <item>Neighbouring triangles are grouped into flat sides: a triangle joins the side grown from the largest
/// triangle when its three corners are within <see cref="PlaneTolerance"/> of that triangle's plane and it faces the
/// same way (within 60 degrees: a thin triangle's normal can point almost anywhere, and one turned sideways is left
/// to the next step). Each side's corners are then within that distance of its plane, which is half the edges' and
/// vertices' tolerance (1e-7), so each side is a valid planar face.</item>
/// <item>A side made only of triangles thinner than <see cref="MergeDistance"/> (a triangle's height above its longest
/// side) would be a face OCCT's tolerances can't tell from a line. Each such triangle's far corner is within that
/// distance of the side opposite, so it is left out and the hull taken again. A thin triangle in a side with a thicker
/// one is only part of a polygon, and stays.</item>
/// <item>Each side is the polygon around its triangles. Because they all face the same way and lie on a convex
/// surface, a line along the plane's normal crosses them at most once (it enters the hull through one triangle facing
/// away from the normal and leaves through one facing it), so the polygon has no overlaps; its outline is
/// checked to be one simple loop anyway, and a side that isn't one is left as its triangles.</item>
/// <item>A corner shared by only two sides, within <see cref="PlaneTolerance"/> of the line between its neighbours, is
/// a point on an edge rather than a corner, and is dropped from both.</item>
/// </list>
/// <para>Point sets whose hull has no volume (fewer than four distinct points, or all on one line or in one plane, to
/// within <see cref="MergeDistance"/>) throw <see cref="ArgumentException"/>, saying which.</para>
/// </remarks>
internal static class ConvexHull3D
{
    /// <summary>Points closer than this are one point, and a triangle thinner than this is a line (the modelling tolerance).</summary>
    internal const double MergeDistance = Constants.Tolerance;

    /// <summary>How far a corner of a flat side may be from its plane: half of OCCT's confusion tolerance (1e-7).</summary>
    internal const double PlaneTolerance = 5e-8;

    /// <summary>The least cosine between a side's normal and a triangle's for the triangle to join it (60 degrees).</summary>
    private const double SameWay = 0.5;

    /// <summary>A flat side of the hull: its plane (normal out) and its corners, anticlockwise seen from outside.</summary>
    internal readonly record struct Side(Point3D Origin, Vector3D Normal, int[] Corners);

    /// <summary>The hull's points (merged) and its flat sides, as indices into them.</summary>
    internal sealed record Hull(IReadOnlyList<Point3D> Points, IReadOnlyList<Side> Sides);

    /// <summary>The hull of <paramref name="points"/>, as flat sides.</summary>
    /// <exception cref="ArgumentException">A coordinate isn't finite, or the hull has no volume.</exception>
    internal static Hull Sides(IEnumerable<Point3D> points)
    {
        ArgumentNullException.ThrowIfNull(points);
        var all = points.ToList();
        if (all.Any(p => !double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z)))
            throw new ArgumentException("The points of a convex polyhedron must have finite coordinates");
        var pts = Merge(all, MergeDistance);
        var removed = new bool[pts.Count];
        while (true)
        {
            var facets = Triangulate(pts, removed);
            var normals = facets.Select(f => (pts[f[1]] - pts[f[0]]).Cross(pts[f[2]] - pts[f[0]])).ToList();
            var (group, sides) = GroupFacets(pts, facets, normals);
            var thin = ThinCorners(pts, facets, group);
            if (thin.Count == 0)
                return new Hull(pts, Outlines(pts, facets, normals, sides));
            foreach (var corner in thin)
                removed[corner] = true;
        }
    }

    /// <summary>The hull's triangles, outward-facing, as point-index triples (for the tests).</summary>
    internal static List<int[]> Facets(IReadOnlyList<Point3D> points) => Triangulate(points, new bool[points.Count]);

    // ---- 1. Merging ----

    /// <summary>The points, those closer than <paramref name="distance"/> to one already kept left out.</summary>
    internal static List<Point3D> Merge(IReadOnlyList<Point3D> points, double distance)
    {
        var kept = new List<Point3D>(points.Count);
        var cells = new Dictionary<(long, long, long), List<int>>();
        (long, long, long) Cell(Point3D p) =>
            ((long)Math.Floor(p.X / distance), (long)Math.Floor(p.Y / distance), (long)Math.Floor(p.Z / distance));
        foreach (var p in points)
        {
            var (cx, cy, cz) = Cell(p);
            bool near = false;
            for (long dx = -1; dx <= 1 && !near; dx++)
                for (long dy = -1; dy <= 1 && !near; dy++)
                    for (long dz = -1; dz <= 1 && !near; dz++)
                        if (cells.TryGetValue((cx + dx, cy + dy, cz + dz), out var list))
                            near = list.Any(j => (kept[j] - p).Magnitude <= distance);
            if (near)
                continue;
            if (!cells.TryGetValue((cx, cy, cz), out var cell))
                cells[(cx, cy, cz)] = cell = [];
            cell.Add(kept.Count);
            kept.Add(p);
        }
        return kept;
    }

    // ---- 2. The exact hull ----

    /// <summary>
    /// The hull of the points not <paramref name="removed"/> (QuickHull: each point outside is held by one triangle it
    /// sees, and the furthest is added next), as outward-facing triangles.
    /// </summary>
    /// <exception cref="ArgumentException">Fewer than four points, or all on a line or in a plane (to within <see cref="MergeDistance"/>).</exception>
    private static List<int[]> Triangulate(IReadOnlyList<Point3D> pts, bool[] removed)
    {
        var live = Enumerable.Range(0, pts.Count).Where(i => !removed[i]).ToList();
        if (live.Count < 4)
            throw new ArgumentException(
                $"A convex polyhedron needs at least four distinct points (points closer than {MergeDistance:G} are one); there are {live.Count}");
        var (a, b, c, d) = InitialTetrahedron(pts, live);

        var facets = new List<int[]>();
        var alive = new List<bool>();
        var outside = new List<List<int>?>();
        var byEdge = new Dictionary<(int, int), int>();   // a directed edge to the facet that has it
        int AddFacet(int p, int q, int r)
        {
            int f = facets.Count;
            facets.Add([p, q, r]);
            alive.Add(true);
            outside.Add(null);
            byEdge[(p, q)] = f;
            byEdge[(q, r)] = f;
            byEdge[(r, p)] = f;
            return f;
        }
        void Orient(int p, int q, int r, int inside)
        {
            if (Orientation(pts[p], pts[q], pts[r], pts[inside]) > 0)
                AddFacet(p, r, q);
            else
                AddFacet(p, q, r);
        }
        Orient(a, b, c, d);
        Orient(a, b, d, c);
        Orient(a, c, d, b);
        Orient(b, c, d, a);

        // Each other point to the first triangle it's in front of; a point in front of none is inside
        void Assign(IEnumerable<int> candidates, IReadOnlyList<int> among)
        {
            foreach (int i in candidates)
                foreach (int f in among)
                    if (Orientation(pts[facets[f][0]], pts[facets[f][1]], pts[facets[f][2]], pts[i]) > 0)
                    {
                        (outside[f] ??= []).Add(i);
                        break;
                    }
        }
        Assign(live.Where(i => i != a && i != b && i != c && i != d), [0, 1, 2, 3]);

        var pending = new Stack<int>([0, 1, 2, 3]);
        while (pending.Count > 0)
        {
            int f = pending.Pop();
            if (!alive[f] || outside[f] is not { Count: > 0 } held)
                continue;
            // The furthest point in front of the triangle (by the floating-point volume: any point in front will do)
            int apex = held.MaxBy(i => Volume(pts[facets[f][0]], pts[facets[f][1]], pts[facets[f][2]], pts[i]));

            // The triangles it sees: connected, as the tests are exact
            var visible = new List<int> { f };
            var seen = new HashSet<int> { f };
            var horizon = new List<(int, int)>();
            for (int k = 0; k < visible.Count; k++)
            {
                var t = facets[visible[k]];
                for (int e = 0; e < 3; e++)
                {
                    int u = t[e], v = t[(e + 1) % 3];
                    int n = byEdge[(v, u)];
                    if (seen.Contains(n))
                        continue;
                    var m = facets[n];
                    if (Orientation(pts[m[0]], pts[m[1]], pts[m[2]], pts[apex]) > 0)
                    {
                        seen.Add(n);
                        visible.Add(n);
                    }
                    else
                        horizon.Add((u, v));
                }
            }
            var orphans = new List<int>();
            foreach (int v in visible)
            {
                alive[v] = false;
                var t = facets[v];
                for (int e = 0; e < 3; e++)
                    if (byEdge.TryGetValue((t[e], t[(e + 1) % 3]), out int owner) && owner == v)
                        byEdge.Remove((t[e], t[(e + 1) % 3]));
                if (outside[v] is { } list)
                    orphans.AddRange(list.Where(i => i != apex));
                outside[v] = null;
            }
            var added = new List<int>(horizon.Count);
            foreach (var (u, v) in horizon)
                added.Add(AddFacet(u, v, apex));
            Assign(orphans, added);
            foreach (int n in added)
                pending.Push(n);
        }

        var result = new List<int[]>();
        for (int f = 0; f < facets.Count; f++)
            if (alive[f])
                result.Add(facets[f]);
        CheckVolume(pts, result);
        return result;
    }

    /// <summary>Four points far from each other's line and plane: the first hull.</summary>
    private static (int, int, int, int) InitialTetrahedron(IReadOnlyList<Point3D> pts, List<int> live)
    {
        // The lowest point in x, the furthest from it, the furthest from their line, the furthest from their plane
        int a = live.MinBy(i => (pts[i].X, pts[i].Y, pts[i].Z));
        int b = live.MaxBy(i => (pts[i] - pts[a]).Magnitude);
        var ab = pts[b] - pts[a];
        int c = live.MaxBy(i => ab.Cross(pts[i] - pts[a]).Magnitude);
        if (ab.Cross(pts[c] - pts[a]).Magnitude / ab.Magnitude <= MergeDistance)
            throw new ArgumentException($"The points of a convex polyhedron all lie on one line (to within {MergeDistance:G}), so their hull has no volume");
        var normal = ab.Cross(pts[c] - pts[a]).Normalized();
        int d = live.MaxBy(i => Math.Abs(normal.Dot(pts[i] - pts[a])));
        if (Math.Abs(normal.Dot(pts[d] - pts[a])) <= MergeDistance)
            throw NoVolume();
        return (a, b, c, d);
    }

    /// <summary>Throws when the hull is thinner than <see cref="MergeDistance"/> everywhere it could be measured across.</summary>
    private static void CheckVolume(IReadOnlyList<Point3D> pts, List<int[]> facets)
    {
        // The hull's width is the least, over its faces, of the furthest corner behind each; a facet with every corner
        // within the merge distance of its plane means the hull is flat
        var corners = facets.SelectMany(f => f).Distinct().ToList();
        foreach (var f in facets)
        {
            var normal = (pts[f[1]] - pts[f[0]]).Cross(pts[f[2]] - pts[f[0]]);
            var length = normal.Magnitude;
            if (length == 0)
                continue;
            normal /= length;
            if (corners.All(i => normal.Dot(pts[i] - pts[f[0]]) >= -MergeDistance))
                throw NoVolume();
        }
    }

    private static ArgumentException NoVolume() =>
        new($"The points of a convex polyhedron all lie in one plane (to within {MergeDistance:G}), so their hull has no volume");

    // ---- 3. Thin triangles ----

    /// <summary>
    /// The far corners of triangles thinner than <see cref="MergeDistance"/> in sides of only such triangles, each within
    /// that distance of the hull without it; no two from triangles sharing a corner, so each is that close to what's left.
    /// </summary>
    private static List<int> ThinCorners(IReadOnlyList<Point3D> pts, List<int[]> facets, int[] group)
    {
        var thin = new List<(double Height, int Apex, int P, int Q)>();
        var thick = new HashSet<int>();   // the sides with a triangle at least that thick
        var candidates = new List<(int Group, (double, int, int, int) Thin)>();
        foreach (var (f, index) in facets.Select((f, i) => (f, i)))
        {
            // The longest side, and the height of the corner opposite above it
            int longest = 0;
            double length = 0;
            for (int e = 0; e < 3; e++)
            {
                var l = (pts[f[(e + 1) % 3]] - pts[f[e]]).Magnitude;
                if (l > length)
                    (longest, length) = (e, l);
            }
            double area2 = (pts[f[1]] - pts[f[0]]).Cross(pts[f[2]] - pts[f[0]]).Magnitude;
            double height = area2 / length;
            if (height < MergeDistance)
                candidates.Add((group[index], (height, f[(longest + 2) % 3], f[longest], f[(longest + 1) % 3])));
            else
                thick.Add(group[index]);
        }
        thin.AddRange(candidates.Where(c => !thick.Contains(c.Group)).Select(c => c.Thin));
        var locked = new HashSet<int>();
        var corners = new List<int>();
        foreach (var (_, apex, p, q) in thin.OrderBy(t => t.Height).ThenBy(t => t.Apex))
        {
            if (locked.Contains(apex) || locked.Contains(p) || locked.Contains(q))
                continue;
            locked.UnionWith([apex, p, q]);
            corners.Add(apex);
        }
        return corners;
    }

    // ---- 4. Flat sides ----

    /// <summary>The flat sides, grown from the largest triangle first, and the side of each triangle.</summary>
    private static (int[] Group, List<(Point3D Origin, Vector3D Normal, List<int> Facets)> Sides) GroupFacets(
        IReadOnlyList<Point3D> pts, List<int[]> facets, List<Vector3D> normals)
    {
        var byEdge = new Dictionary<(int, int), int>();
        for (int f = 0; f < facets.Count; f++)
            for (int e = 0; e < 3; e++)
                byEdge[(facets[f][e], facets[f][(e + 1) % 3])] = f;
        var group = Enumerable.Repeat(-1, facets.Count).ToArray();
        var sides = new List<(Point3D Origin, Vector3D Normal, List<int> Facets)>();
        foreach (int seed in Enumerable.Range(0, facets.Count).OrderByDescending(f => normals[f].Magnitude).ThenBy(f => f))
        {
            if (group[seed] >= 0)
                continue;
            var origin = pts[facets[seed][0]];
            var normal = normals[seed].Normalized();
            var members = new List<int> { seed };
            group[seed] = sides.Count;
            for (int k = 0; k < members.Count; k++)
            {
                var t = facets[members[k]];
                for (int e = 0; e < 3; e++)
                {
                    int n = byEdge[(t[(e + 1) % 3], t[e])];
                    // Facing the same way, by a margin: a thin triangle's normal can point almost anywhere, and one
                    // nearly sideways would fold the side's outline back on itself
                    if (group[n] >= 0 || normal.Dot(normals[n]) <= SameWay * normals[n].Magnitude)
                        continue;
                    if (facets[n].All(i => Math.Abs(normal.Dot(pts[i] - origin)) <= PlaneTolerance))
                    {
                        group[n] = sides.Count;
                        members.Add(n);
                    }
                }
            }
            sides.Add((origin, normal, members));
        }
        return (group, sides);
    }

    /// <summary>The sides as polygons.</summary>
    private static List<Side> Outlines(IReadOnlyList<Point3D> pts, List<int[]> facets, List<Vector3D> normals,
        List<(Point3D Origin, Vector3D Normal, List<int> Facets)> sides)
    {
        // Each side's outline; a side whose outline isn't one simple loop is left as its triangles
        var loops = new List<(Point3D Origin, Vector3D Normal, List<int> Loop)>();
        foreach (var (origin, normal, members) in sides)
        {
            if (Outline(facets, members) is { } loop)
                loops.Add((origin, normal, loop));
            else
                foreach (int f in members)
                    loops.Add((pts[facets[f][0]], normals[f].Normalized(), [.. facets[f]]));
        }
        DropEdgePoints(pts, loops);
        return loops.Select(l => new Side(l.Origin, l.Normal, [.. l.Loop])).ToList();
    }

    /// <summary>The outline of a set of triangles, if it is one loop visiting each corner once; otherwise null.</summary>
    private static List<int>? Outline(List<int[]> facets, List<int> members)
    {
        var inside = new HashSet<(int, int)>();
        foreach (int f in members)
            for (int e = 0; e < 3; e++)
                inside.Add((facets[f][e], facets[f][(e + 1) % 3]));
        var next = new Dictionary<int, int>();
        foreach (var (u, v) in inside)
            if (!inside.Contains((v, u)) && !next.TryAdd(u, v))
                return null;   // a corner the outline passes twice
        if (next.Count < 3)
            return null;
        int start = next.Keys.Min();
        var loop = new List<int> { start };
        for (int v = next[start]; v != start; v = next[v])
        {
            if (loop.Count > next.Count)
                return null;
            loop.Add(v);
        }
        return loop.Count == next.Count ? loop : null;   // otherwise more than one loop
    }

    /// <summary>
    /// Drops corners on only two sides that lie on the line between their neighbours, so an edge between two sides is
    /// one edge. A corner dropped is within <see cref="PlaneTolerance"/> of the line between the corners kept either side,
    /// counting the corners dropped before it.
    /// </summary>
    private static void DropEdgePoints(IReadOnlyList<Point3D> pts, List<(Point3D Origin, Vector3D Normal, List<int> Loop)> sides)
    {
        var sidesOf = new Dictionary<int, List<int>>();
        for (int s = 0; s < sides.Count; s++)
            foreach (int v in sides[s].Loop)
                (sidesOf.TryGetValue(v, out var l) ? l : sidesOf[v] = []).Add(s);
        // The corners dropped between two kept corners, by the side's directed edge
        var between = new Dictionary<(int, int), List<int>>();
        List<int> Between(int u, int v) => between.TryGetValue((u, v), out var l) ? l : [];
        bool changed = true;
        while (changed)
        {
            changed = false;
            foreach (var (v, owners) in sidesOf)
            {
                if (owners.Count != 2)
                    continue;
                var (s1, s2) = (sides[owners[0]].Loop, sides[owners[1]].Loop);
                int i1 = s1.IndexOf(v), i2 = s2.IndexOf(v);
                if (s1.Count <= 3 || s2.Count <= 3)
                    continue;
                int a = s1[(i1 + s1.Count - 1) % s1.Count], b = s1[(i1 + 1) % s1.Count];
                if (s2[(i2 + s2.Count - 1) % s2.Count] != b || s2[(i2 + 1) % s2.Count] != a)
                    continue;
                List<int> run = [.. Between(a, v), v, .. Between(v, b)];
                if (!run.All(p => DistanceToSegment(pts[p], pts[a], pts[b]) <= PlaneTolerance))
                    continue;
                s1.RemoveAt(i1);
                s2.RemoveAt(i2);
                between[(a, b)] = run;
                between[(b, a)] = [.. Enumerable.Reverse(run)];
                owners.Clear();
                changed = true;
            }
        }
    }

    private static double DistanceToSegment(Point3D p, Point3D a, Point3D b)
    {
        var ab = b - a;
        double t = Math.Clamp((p - a).Dot(ab) / ab.Dot(ab), 0, 1);
        return (p - (a + ab * t)).Magnitude;
    }

    // ---- Exact orientation ----

    /// <summary>Six times the signed volume of (p, q, r, s), in floating point: positive when s is in front of p, q, r.</summary>
    private static double Volume(Point3D p, Point3D q, Point3D r, Point3D s) => (q - p).Cross(r - p).Dot(s - p);

    /// <summary>
    /// The exact sign of <see cref="Volume"/>: +1 when s is in front of the triangle p, q, r (anticlockwise seen from s),
    /// -1 behind, 0 in its plane.
    /// </summary>
    internal static int Orientation(Point3D p, Point3D q, Point3D r, Point3D s)
    {
        double ux = q.X - p.X, uy = q.Y - p.Y, uz = q.Z - p.Z;
        double vx = r.X - p.X, vy = r.Y - p.Y, vz = r.Z - p.Z;
        double wx = s.X - p.X, wy = s.Y - p.Y, wz = s.Z - p.Z;
        double det = ux * (vy * wz - vz * wy) + uy * (vz * wx - vx * wz) + uz * (vx * wy - vy * wx);
        double permanent = Math.Abs(ux) * (Math.Abs(vy * wz) + Math.Abs(vz * wy))
                         + Math.Abs(uy) * (Math.Abs(vz * wx) + Math.Abs(vx * wz))
                         + Math.Abs(uz) * (Math.Abs(vx * wy) + Math.Abs(vy * wx));
        // Shewchuk's bound for this expression is (7 + 56e)e times the permanent, e = 2^-53 (about 7.8e-16); outside
        // the range where products might underflow, a determinant beyond 1e-15 times the permanent has the right sign
        if (permanent > 1e-280 && Math.Abs(det) > 1e-15 * permanent)
            return Math.Sign(det);
        return ExactOrientation(p, q, r, s);
    }

    private static int ExactOrientation(Point3D p, Point3D q, Point3D r, Point3D s)
    {
        // Each coordinate as an integer times 2^e, all scaled to the smallest e, so the differences and products are exact
        Span<double> values = [p.X, p.Y, p.Z, q.X, q.Y, q.Z, r.X, r.Y, r.Z, s.X, s.Y, s.Z];
        Span<long> mantissas = stackalloc long[12];
        Span<int> exponents = stackalloc int[12];
        int least = int.MaxValue;
        for (int i = 0; i < 12; i++)
        {
            (mantissas[i], exponents[i]) = Decompose(values[i]);
            if (mantissas[i] != 0)
                least = Math.Min(least, exponents[i]);
        }
        var big = new BigInteger[12];
        for (int i = 0; i < 12; i++)
            big[i] = mantissas[i] == 0 ? BigInteger.Zero : new BigInteger(mantissas[i]) << (exponents[i] - least);
        BigInteger ux = big[3] - big[0], uy = big[4] - big[1], uz = big[5] - big[2];
        BigInteger vx = big[6] - big[0], vy = big[7] - big[1], vz = big[8] - big[2];
        BigInteger wx = big[9] - big[0], wy = big[10] - big[1], wz = big[11] - big[2];
        var det = ux * (vy * wz - vz * wy) + uy * (vz * wx - vx * wz) + uz * (vx * wy - vy * wx);
        return det.Sign;
    }

    /// <summary>A finite double as m * 2^e, m an integer.</summary>
    private static (long Mantissa, int Exponent) Decompose(double value)
    {
        long bits = BitConverter.DoubleToInt64Bits(value);
        bool negative = bits < 0;
        int exponent = (int)((bits >> 52) & 0x7FF);
        long mantissa = bits & 0xFFFFFFFFFFFFFL;
        if (exponent == 0)
            exponent = 1;   // subnormal
        else
            mantissa |= 1L << 52;
        if (mantissa == 0)
            return (0, 0);
        return (negative ? -mantissa : mantissa, exponent - 1075);
    }
}
