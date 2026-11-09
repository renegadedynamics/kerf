using Kerf.Construction;

namespace Kerf.Modeling;

/// <summary>
/// The convex hull of edges in the XY plane, as a wire of pieces of the edges and lines touching them (decision 24005).
/// </summary>
/// <remarks>
/// <para>build123d samples 2 / tolerance points along every edge, by arc length, and keeps the hull of the samples;
/// each sample by arc length inverts the curve's length numerically, which was most of the time. Here a line adds its
/// ends only, a curve is sampled to a deflection of tolerance times its length (checked, as the kernel's sampling can
/// miss a lobe), so the samples' hull is within that of the edges'. Then:</para>
/// <list type="bullet">
/// <item>each joining line found from the samples is moved to touch its curves (Newton's method), kept only if the curves
/// stay inside it; two curves crossing make a corner where they meet;</item>
/// <item>a step along a curve that bends inwards between its samples is crossed by a touching line instead;</item>
/// <item>a curve poking past one of the hull's lines (or past a step along another curve, by more than that curve's own
/// bulge) gets a sample at its furthest point, and the hull is taken again, at most <see cref="MaxRounds"/> times;</item>
/// <item>the points along the result must go once round, turning left: if not, the samples' hull alone is kept, as
/// build123d makes it, within the deflection.</item>
/// </list>
/// </remarks>
internal static class ConvexHullOfEdges
{
    private readonly record struct Sample(int Edge, int Index);

    private sealed class Piece(Edge edge, EdgeCurve curve, double[] parameters, bool closed, bool line)
    {
        public Edge Edge { get; } = edge;
        public EdgeCurve Curve { get; } = curve;
        public double[] Parameters { get; } = parameters;
        public bool Closed { get; } = closed;   // the last sample isn't kept: it is the first
        public bool Line { get; } = line;
        public double Min => Curve.Range.Min;
        public double Max => Curve.Range.Max;

        public bool Adjacent(int i, int j) =>
            Math.Abs(i - j) == 1 || (Closed && Math.Abs(i - j) == Parameters.Length - 1);
    }

    /// <summary>Rounds of adding a sample where a curve pokes past a joining line: each adds at least one.</summary>
    private const int MaxRounds = 8;

    internal static Wire Make(IEnumerable<Edge> edges, double tolerance)
    {
        // Each edge's samples: a line's ends, a curve's to a deflection of tolerance times its length
        var bases = new List<(Edge Edge, EdgeCurve Curve, List<double> Parameters, bool Closed, bool Line, double Deflection)>();
        foreach (var edge in edges)
        {
            var curve = edge.AsCurve();
            var range = curve.Range;
            bool line = edge.GeomType == GeomType.Line;
            double deflection = Math.Max(tolerance * edge.Length, Tolerances.Confusion);
            List<double> parameters;
            if (line)
                parameters = [range.Min, range.Max];
            else
                parameters = Samples(curve, deflection);
            bool closed = !line && (curve.Evaluate(range.Min) - curve.Evaluate(range.Max)).Magnitude <= Constants.Tolerance;
            if (closed)
                parameters.RemoveAt(parameters.Count - 1);
            bases.Add((edge, curve, parameters, closed, line, deflection));
        }
        for (int round = 0; ; round++)
        {
            var pieces = bases.Select(b => new Piece(b.Edge, b.Curve, [.. b.Parameters], b.Closed, b.Line)).ToList();
            if (Walk(pieces, out var wire, out var trace, round < MaxRounds ? bases.Select(b => b.Deflection).ToList() : null, refine: true) is { Count: > 0 } pokes)
            {
                foreach (var (e, t) in pokes)
                {
                    var list = bases[e].Parameters;
                    int at = list.BinarySearch(t);
                    if (at < 0)
                        list.Insert(~at, t);
                }
                continue;
            }
            if (Convex(trace))
                return wire!.Value;
            // Where touching lines and crossings tangle (curves crossing near where a line leaves them), the samples'
            // hull alone, as build123d builds it, with the curves sampled a thousand times finer: within a thousandth
            // of the deflection of the edges' hull
            var fine = bases.Select(b => new Piece(b.Edge, b.Curve,
                b.Line ? [.. b.Parameters] : [.. Finer(b.Curve, b.Deflection / 1000, b.Closed)], b.Closed, b.Line)).ToList();
            Walk(fine, out wire, out _, null, refine: false);
            return wire!.Value;
        }
    }

    private static List<double> Finer(EdgeCurve curve, double deflection, bool closed)
    {
        var parameters = Samples(curve, deflection);
        if (closed)
            parameters.RemoveAt(parameters.Count - 1);
        return parameters;
    }

    /// <summary>
    /// Whether points along the hull, in order, go once round anticlockwise, turning left at each (to rounding): a
    /// tangle of touching lines would turn back or loop.
    /// </summary>
    private static bool Convex(List<Point3D> trace)
    {
        var points = new List<Point3D>(trace.Count);
        double size = trace.Count == 0 ? 1 : trace.Max(p => Math.Abs(p.X) + Math.Abs(p.Y)) + 1;
        foreach (var p in trace)
            if (points.Count == 0 || (p - points[^1]).Magnitude > 1e-9 * size)
                points.Add(p);
        while (points.Count > 1 && (points[0] - points[^1]).Magnitude <= 1e-9 * size)
            points.RemoveAt(points.Count - 1);
        if (points.Count < 3)
            return points.Count > 0;   // one closed edge, all the way round
        double turned = 0;
        for (int i = 0; i < points.Count; i++)
        {
            var a = points[(i + points.Count - 1) % points.Count];
            var b = points[i];
            var c = points[(i + 1) % points.Count];
            double ux = b.X - a.X, uy = b.Y - a.Y, vx = c.X - b.X, vy = c.Y - b.Y;
            double cross = ux * vy - uy * vx, dot = ux * vx + uy * vy;
            if (cross < -1e-6 * Math.Sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy)))
                return false;
            turned += Math.Atan2(cross, dot);
        }
        return Math.Abs(turned - 2 * Math.PI) < 1e-6;
    }

    /// <summary>
    /// A curve's samples, the polygon through them within <paramref name="deflection"/> of it, as checked at each span's
    /// quarter points: the kernel's deflection sampling (OCCT's) with 16 even steps, each span that strays further split
    /// in two. The kernel's alone isn't enough: on a spline with a tight lobe it left the lobe 5.5 out at a deflection of
    /// 0.2 (occt-issues.md).
    /// </summary>
    private static List<double> Samples(EdgeCurve curve, double deflection)
    {
        var range = curve.Range;
        var start = new SortedSet<double>(curve.Discretize(deflection));
        for (int k = 0; k <= 16; k++)
            start.Add(range.Min + (range.Max - range.Min) * k / 16);
        var result = new List<double>();
        var pending = new Stack<(double, double, int)>();
        var list = start.ToList();
        for (int i = list.Count - 2; i >= 0; i--)
            pending.Push((list[i], list[i + 1], 0));
        result.Add(list[0]);
        const int maxDepth = 24, maxSamples = 200_000;
        while (pending.Count > 0)
        {
            var (t0, t1, depth) = pending.Pop();
            var p = curve.Evaluate(t0);
            var q = curve.Evaluate(t1);
            double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
            double Off(double t)
            {
                var x = curve.Evaluate(t);
                return length == 0 ? Math.Sqrt((x.X - p.X) * (x.X - p.X) + (x.Y - p.Y) * (x.Y - p.Y))
                                   : Math.Abs(dx * (x.Y - p.Y) - dy * (x.X - p.X)) / length;
            }
            double h = t1 - t0;
            if (depth < maxDepth && result.Count + pending.Count < maxSamples
                && (Off(t0 + h / 4) > deflection || Off(t0 + h / 2) > deflection || Off(t0 + 3 * h / 4) > deflection))
            {
                pending.Push((t0 + h / 2, t1, depth + 1));
                pending.Push((t0, t0 + h / 2, depth + 1));
                continue;
            }
            result.Add(t1);
        }
        return result;
    }

    /// <summary>
    /// The hull of the pieces' samples, its joining lines made to touch; then, if <paramref name="deflections"/> is given,
    /// the places where a curve pokes past a joining line (to add as samples, with no wire), or else the wire.
    /// </summary>
    private static List<(int Edge, double T)> Walk(List<Piece> pieces, out Wire? wire, out List<Point3D> trace, List<double>? deflections,
        bool refine)
    {
        wire = null;
        trace = [];
        var samples = new List<Sample>();
        var points = new List<(double X, double Y)>();
        for (int e = 0; e < pieces.Count; e++)
            for (int i = 0; i < pieces[e].Parameters.Length; i++)
            {
                var p = pieces[e].Curve.Evaluate(pieces[e].Parameters[i]);
                samples.Add(new Sample(e, i));
                points.Add((p.X, p.Y));
            }

        // The samples' hull, anticlockwise. A point where edges meet is one sample to the hull, but each edge's sample
        // there counts: the hull may arrive along one edge and leave along the other
        var cycle = ConvexHull2D.Simplices(points).Select(s => s.Item1).ToList();
        int n = cycle.Count;
        // (within the tolerance: two edges' shared end can differ in its last digits)
        var cells = new Dictionary<(long, long), List<int>>();
        (long, long) Cell(int s) => ((long)Math.Floor(points[s].X / Constants.Tolerance), (long)Math.Floor(points[s].Y / Constants.Tolerance));
        for (int s = 0; s < samples.Count; s++)
            (cells.TryGetValue(Cell(s), out var list) ? list : cells[Cell(s)] = []).Add(s);
        List<int> Aliases(int s)
        {
            var (cx, cy) = Cell(s);
            var near = new List<int> { s };
            for (long dx = -1; dx <= 1; dx++)
                for (long dy = -1; dy <= 1; dy++)
                    if (cells.TryGetValue((cx + dx, cy + dy), out var list))
                        near.AddRange(list.Where(o => o != s && Math.Abs(points[o].X - points[s].X) <= Constants.Tolerance
                                                             && Math.Abs(points[o].Y - points[s].Y) <= Constants.Tolerance));
            return near;
        }
        bool Follows(Sample a, Sample b) => a.Edge == b.Edge && pieces[a.Edge].Adjacent(a.Index, b.Index);

        // Each step round the hull: along an edge (with the samples it goes between) or not
        var along = new (Sample From, Sample To)?[n];
        for (int i = 0; i < n; i++)
            foreach (int a in Aliases(cycle[i]))
            {
                foreach (int b in Aliases(cycle[(i + 1) % n]))
                    if (Follows(samples[a], samples[b]))
                    {
                        along[i] = (samples[a], samples[b]);
                        break;
                    }
                if (along[i] is not null)
                    break;
            }
        // A step along a curve that bends inwards between its two samples (an inflection) isn't on the hull: the hull
        // crosses the dent on a line touching the curve either side, so the step is left to be joined
        for (int i = 0; i < n; i++)
            if (refine && along[i] is { } step && Dented(pieces[step.From.Edge], step.From.Index, step.To.Index))
                along[i] = null;
        if (along.All(s => s is not null) && along.All(s => s!.Value.From.Edge == along[0]!.Value.From.Edge))
        {
            wire = Wire.Make(pieces[along[0]!.Value.From.Edge].Edge);   // one closed edge, all the way round
            return [];
        }

        // Each step that leaves the edges is a joining line, its ends on the edges the hull arrives along and leaves
        // along, moved until it touches them
        var joins = new (int From, double FromT, int To, double ToT)?[n];
        for (int i = 0; i < n; i++)
        {
            if (along[i] is not null)
                continue;
            var from = along[(i + n - 1) % n]?.To ?? samples[cycle[i]];
            var to = along[(i + 1) % n]?.From ?? samples[cycle[(i + 1) % n]];
            joins[i] = refine ? Touch(pieces, from, to)
                : (from.Edge, pieces[from.Edge].Parameters[from.Index], to.Edge, pieces[to.Edge].Parameters[to.Index]);
        }
        if (deflections is not null)
        {
            // The hull's pieces as lines: the joining lines and the steps along lines, and the steps along curves by
            // their chords, which another curve must pass by more than the curve's own bulge to be outside it
            var lines = joins.Where(j => j is not null).Select(j => (Line: j!.Value, Skip: -1, Allowance: 0.0)).ToList();
            foreach (var step in along)
            {
                if (step is not { } s)
                    continue;
                var piece = pieces[s.From.Edge];
                double t0 = piece.Parameters[s.From.Index], t1 = piece.Parameters[s.To.Index];
                if (piece.Line)
                {
                    lines.Add(((s.From.Edge, t0, s.To.Edge, t1), -1, 0.0));
                    continue;
                }
                var p = piece.Curve.Evaluate(t0);
                var q = piece.Curve.Evaluate(t1);
                double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
                if (length <= Constants.Tolerance || (piece.Closed && Math.Abs(s.To.Index - s.From.Index) != 1))
                    continue;
                double Past(Point3D x) => -(dx * (x.Y - p.Y) - dy * (x.X - p.X)) / length;
                var (_, bulge) = Furthest(piece, Math.Min(t0, t1), Math.Max(t0, t1), Past);
                lines.Add(((s.From.Edge, t0, s.To.Edge, t1), s.From.Edge, Math.Max(bulge, 0)));
            }
            if (Pokes(pieces, lines, deflections) is { Count: > 0 } pokes)
                return pokes;
        }

        // The pieces in order round the hull: runs along an edge, from where the hull reaches it to where it leaves it,
        // and the joining lines
        var result = new List<Edge>();
        int startAt = Enumerable.Range(0, n).FirstOrDefault(i => along[i] is null ||
            along[(i + n - 1) % n] is null || along[(i + n - 1) % n]!.Value.To.Edge != along[i]!.Value.From.Edge);
        for (int step = 0; step < n;)
        {
            int i = (startAt + step) % n;
            if (joins[i] is { } join)
            {
                var start = pieces[join.From].Curve.Evaluate(join.FromT);
                var end = pieces[join.To].Curve.Evaluate(join.ToT);
                if ((end - start).Magnitude > Constants.Tolerance)
                    result.Add(Edge.MakeLine(start, end));
                trace.Add(start);
                trace.Add(end);
                // Two joining lines at one sample: the hull follows the edge between where one ends and the next starts
                int j = (i + 1) % n;
                if (joins[j] is { } nextJoin && nextJoin.From == join.To)
                    AddRun(result, trace, pieces[join.To], join.ToT, nextJoin.FromT, null);
                step++;
                continue;
            }
            // A run of steps along one edge
            var first = along[i]!.Value;
            int e = first.From.Edge, count = 0;
            while (count < n - step && along[(i + count) % n] is { } s && s.From.Edge == e)
                count++;
            var last = along[(i + count - 1) % n]!.Value;
            var piece = pieces[e];
            double t0 = joins[(i + n - 1) % n] is { } before && before.To == e ? before.ToT : piece.Parameters[first.From.Index];
            double t1 = joins[(i + count) % n] is { } after && after.From == e ? after.FromT : piece.Parameters[last.To.Index];
            bool increasing = (first.To.Index - first.From.Index + piece.Parameters.Length) % piece.Parameters.Length == 1;
            AddRun(result, trace, piece, t0, t1, increasing);
            step += count;
        }
        wire = Wire.Make(result);   // already in order round the hull: sequencing them again is quadratic
        return [];
    }

    /// <summary>
    /// Where a curve pokes past a straight piece of the hull: the samples' hull can miss a bump of a curve between two of its samples,
    /// by up to its deflection. Each curve's samples within two deflections of a joining line, near it, are searched
    /// either side for the furthest point past it (golden section); one past by more than 1e-10 of the line's length
    /// is a sample to add.
    /// </summary>
    private static List<(int Edge, double T)> Pokes(List<Piece> pieces,
        List<((int From, double FromT, int To, double ToT) Line, int Skip, double Allowance)> lines, List<double> deflections)
    {
        var found = new List<(int, double)>();
        foreach (var (j, skip, allowance) in lines)
        {
            var p = pieces[j.From].Curve.Evaluate(j.FromT);
            var q = pieces[j.To].Curve.Evaluate(j.ToT);
            double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
            if (length <= Constants.Tolerance)
                continue;
            // How far a point is past the line, outwards (the hull runs anticlockwise, so outside is on the right)
            double Past(Point3D x) => -(dx * (x.Y - p.Y) - dy * (x.X - p.X)) / length;
            double Along(Point3D x) => (dx * (x.X - p.X) + dy * (x.Y - p.Y)) / length;
            for (int e = 0; e < pieces.Count; e++)
            {
                var piece = pieces[e];
                if (piece.Line || e == skip)
                    continue;   // a line's furthest points are its ends, which are samples; a curve's own step is its own
                var t = piece.Parameters;
                for (int i = 0; i < t.Length; i++)
                {
                    var x = piece.Curve.Evaluate(t[i]);
                    double s = Along(x);
                    if (Past(x) < allowance - 2 * deflections[e] || s < -deflections[e] || s > length + deflections[e])
                        continue;
                    var (lo, hi) = Bracket(piece, i);
                    if (!piece.Curve.IsPeriodic)
                        (lo, hi) = (Math.Max(lo, piece.Min), Math.Min(hi, piece.Max));
                    // Anything past a hull edge's line is outside the hull, wherever along the line it is
                    var (best, distance) = Furthest(piece, lo, hi, Past);
                    if (distance > allowance + 1e-10 * length)
                    {
                        double period = piece.Max - piece.Min;
                        if (piece.Closed && best >= piece.Max)
                            best -= period;
                        else if (piece.Closed && best < piece.Min)
                            best += period;
                        if (!t.Contains(best))
                            found.Add((e, best));
                    }
                }
            }
        }
        return found;
    }

    /// <summary>
    /// The parameter in [lo, hi] furthest past a line, and how far past it is: the best of 16 steps, then a golden
    /// section either side of it (a curve can bend more than once between two samples).
    /// </summary>
    private static (double, double) Furthest(Piece piece, double lo, double hi, Func<Point3D, double> past)
    {
        const int steps = 16;
        int bestStep = 0;
        double bestPast = double.NegativeInfinity;
        for (int k = 0; k <= steps; k++)
        {
            double v = past(piece.Curve.Evaluate(lo + (hi - lo) * k / steps));
            if (v > bestPast)
                (bestStep, bestPast) = (k, v);
        }
        double width = (hi - lo) / steps;
        (lo, hi) = (lo + width * Math.Max(bestStep - 1, 0), lo + width * Math.Min(bestStep + 1, steps));
        double ratio = (Math.Sqrt(5) - 1) / 2;
        double a = lo, b = hi;
        double c = b - ratio * (b - a), d = a + ratio * (b - a);
        double fc = past(piece.Curve.Evaluate(c)), fd = past(piece.Curve.Evaluate(d));
        for (int k = 0; k < 60 && b - a > 1e-13 * Math.Max(1, Math.Abs(b)); k++)
        {
            if (fc > fd)
            {
                (b, d, fd) = (d, c, fc);
                c = b - ratio * (b - a);
                fc = past(piece.Curve.Evaluate(c));
            }
            else
            {
                (a, c, fc) = (c, d, fd);
                d = a + ratio * (b - a);
                fd = past(piece.Curve.Evaluate(d));
            }
        }
        double best = (a + b) / 2;
        return (best, past(piece.Curve.Evaluate(best)));
    }

    /// <summary>The piece of an edge the hull follows, from <paramref name="t0"/> to <paramref name="t1"/>.</summary>
    private static void AddRun(List<Edge> result, List<Point3D> trace, Piece piece, double t0, double t1, bool? increasing)
    {
        double period = piece.Max - piece.Min;
        double lo, hi;
        if (increasing is { } up)
            (lo, hi) = up ? (t0, t1) : (t1, t0);
        else
            (lo, hi) = (Math.Min(t0, t1), Math.Max(t0, t1));
        if (hi < lo)
        {
            if (piece.Closed)
                hi += period;   // across the seam of a closed edge
            else
                (lo, hi) = (hi, lo);
        }
        else if (increasing is null && piece.Closed && hi - lo > period / 2)
            (lo, hi) = (hi, lo + period);   // the short way round, across the seam
        if (piece.Curve.LengthBetween(lo, Math.Min(hi, piece.Max)) + (hi > piece.Max ? piece.Curve.LengthBetween(piece.Min, hi - period) : 0)
            > Constants.Tolerance)
            result.AddRange(Trimmed(piece, lo, hi));
        // Points along it, the way the hull goes: from where it arrives
        Point3D Along(double t) => piece.Curve.Evaluate(t > piece.Max && piece.Closed ? t - period : t);
        var entry = Along(t0);
        bool forward = (Along(lo) - entry).Magnitude <= (Along(hi) - entry).Magnitude;
        for (int k = 0; k <= 8; k++)
            trace.Add(Along(forward ? lo + (hi - lo) * k / 8 : hi - (hi - lo) * k / 8));
    }

    /// <summary>The piece of an edge between two of its curve's parameters (past its end on a closed edge).</summary>
    private static IEnumerable<Edge> Trimmed(Piece piece, double lo, double hi)
    {
        piece.Edge.TryGetCurve(out var curve, out _);
        if (hi > piece.Max && !curve!.IsPeriodic)
        {
            // A closed curve that isn't periodic: the two pieces either side of its seam
            yield return Failures.Get(Edge.FromCurve(curve, new Interval(lo, piece.Max)), () => "convex hull: trim");
            yield return Failures.Get(Edge.FromCurve(curve, new Interval(piece.Min, hi - (piece.Max - piece.Min))), () => "convex hull: trim");
            yield break;
        }
        yield return Failures.Get(Edge.FromCurve(curve!, new Interval(lo, hi)), () => "convex hull: trim");
    }

    /// <summary>
    /// The joining line from sample <paramref name="a"/> to sample <paramref name="b"/>, its ends moved along their
    /// curves, within the samples either side, until it touches both (or ends at an edge's end).
    /// </summary>
    private static (int From, double FromT, int To, double ToT) Touch(List<Piece> pieces, Sample a, Sample b)
    {
        var (pa, pb) = (pieces[a.Edge], pieces[b.Edge]);
        double ta = pa.Parameters[a.Index], tb = pb.Parameters[b.Index];
        var (loA, hiA) = Bracket(pa, a.Index);
        var (loB, hiB) = Bracket(pb, b.Index);
        bool freeA = !pa.Line, freeB = !pb.Line;
        if ((pa.Curve.Evaluate(ta) - pb.Curve.Evaluate(tb)).Magnitude <= Constants.Tolerance)
            return (a.Edge, ta, b.Edge, tb);   // they meet: there's no line to move
        // Where the samples' line can't be made to touch, two curves may cross there: the hull has a corner where they meet
        (int, double, int, double) Fallback()
        {
            // A corner of the hull is on or outside the samples' line; a crossing inside it is inside the hull
            if (!pa.Line && !pb.Line && a.Edge != b.Edge && Meet(pa, pb, pa.Parameters[a.Index], pb.Parameters[b.Index]) is var (u, v, met)
                && met && u >= loA && u <= hiA && v >= loB && v <= hiB)
            {
                var p = pa.Curve.Evaluate(pa.Parameters[a.Index]);
                var q = pb.Curve.Evaluate(pb.Parameters[b.Index]);
                var x = pa.Curve.Evaluate(u);
                double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
                if (-(dx * (x.Y - p.Y) - dy * (x.X - p.X)) / length >= -1e-12 * length)
                    return (a.Edge, u, b.Edge, v);
            }
            return (a.Edge, pa.Parameters[a.Index], b.Edge, pb.Parameters[b.Index]);   // the samples' line, within the bound
        }
        for (int round = 0; round < 3 && (freeA || freeB); round++)
        {
            var (ua, ub, converged) = Newton(pa, pb, ta, tb, freeA, freeB);
            if (!converged)
                return Fallback();
            // An end past the samples either side touches beyond this edge's run: it stays at the edge's end
            bool outA = freeA && (ua < loA || ua > hiA), outB = freeB && (ub < loB || ub > hiB);
            if (!outA && !outB)
                return Inside(pa, ua, loA, hiA, pb, ub, loB, hiB) ? (a.Edge, ua, b.Edge, ub) : Fallback();
            // Past the edge's end from its end sample, the line leaves from that end, a corner; anything else isn't trusted
            if (outA)
            {
                freeA = false;
                ta = Math.Clamp(ua, loA, hiA);
                if (!AtEnd(pa, a.Index, ta))
                    return Fallback();
            }
            if (outB)
            {
                freeB = false;
                tb = Math.Clamp(ub, loB, hiB);
                if (!AtEnd(pb, b.Index, tb))
                    return Fallback();
            }
        }
        return (a.Edge, ta, b.Edge, tb);
    }

    /// <summary>Whether the curve between two adjacent samples goes inside the line between them (the hull runs anticlockwise).</summary>
    private static bool Dented(Piece piece, int from, int to)
    {
        if (piece.Line)
            return false;
        double t0 = piece.Parameters[from], t1 = piece.Parameters[to];
        if (piece.Closed && Math.Abs(to - from) != 1)
            (t0, t1) = from > to ? (t0, t1 + (piece.Max - piece.Min)) : (t0 + (piece.Max - piece.Min), t1);
        var p = piece.Curve.Evaluate(t0);
        var q = piece.Curve.Evaluate(t1);
        double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
        // Deeper than rounding can make it: a millionth of the chord, and 1e-11 of the coordinates' size
        double depth = Math.Max(1e-6 * length, 1e-11 * (1 + Math.Abs(p.X) + Math.Abs(p.Y)));
        if (length == 0)
            return false;
        for (int k = 1; k < 16; k++)
        {
            var x = piece.Curve.Evaluate(t0 + (t1 - t0) * k / 16);
            if ((dx * (x.Y - p.Y) - dy * (x.X - p.X)) / length > depth)
                return true;   // on the left of the line, inside
        }
        return false;
    }

    // Whether a parameter is the end of an open edge that the sample is at
    private static bool AtEnd(Piece piece, int index, double t) =>
        !piece.Closed && ((index == 0 && t == piece.Min) || (index == piece.Parameters.Length - 1 && t == piece.Max));

    /// <summary>
    /// Whether each curve stays inside the line from A at <paramref name="ua"/> to B at <paramref name="ub"/> either side
    /// of where it touches: a curve that bends both ways can touch a line where it crosses it.
    /// </summary>
    private static bool Inside(Piece pa, double ua, double loA, double hiA, Piece pb, double ub, double loB, double hiB)
    {
        var p = pa.Curve.Evaluate(ua);
        var q = pb.Curve.Evaluate(ub);
        double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
        bool Stays(Piece piece, double u, double lo, double hi)
        {
            if (piece.Line)
                return true;
            foreach (double fraction in (ReadOnlySpan<double>)[0.5, 0.25, 0.0625])
                foreach (double t in (ReadOnlySpan<double>)[u - (u - lo) * fraction, u + (hi - u) * fraction])
                {
                    if (!piece.Curve.IsPeriodic && (t < piece.Min || t > piece.Max))
                        continue;
                    var x = piece.Curve.Evaluate(t);
                    if (-(dx * (x.Y - p.Y) - dy * (x.X - p.X)) / length > 1e-9 * length)
                        return false;
                }
            return true;
        }
        return Stays(pa, ua, loA, hiA) && Stays(pb, ub, loB, hiB);
    }

    // The parameters of the samples either side of a sample (its edge's ends at its ends)
    private static (double, double) Bracket(Piece piece, int index)
    {
        var t = piece.Parameters;
        int count = t.Length;
        if (piece.Closed)
        {
            double period = piece.Max - piece.Min;
            double before = index > 0 ? t[index - 1] : t[count - 1] - period;
            double after = index < count - 1 ? t[index + 1] : t[0] + period;
            return (before, after);
        }
        return (index > 0 ? t[index - 1] : t[0], index < count - 1 ? t[index + 1] : t[count - 1]);
    }

    /// <summary>Where curves A and B cross, near <paramref name="ta"/> and <paramref name="tb"/> (Newton's method in the plane).</summary>
    private static (double, double, bool) Meet(Piece pa, Piece pb, double ta, double tb)
    {
        double scaleA = pa.Max - pa.Min, scaleB = pb.Max - pb.Min;
        for (int iteration = 0; iteration < 30; iteration++)
        {
            var p = pa.Curve.Evaluate(ta);
            var q = pb.Curve.Evaluate(tb);
            double rx = p.X - q.X, ry = p.Y - q.Y;
            if (Math.Sqrt(rx * rx + ry * ry) <= 1e-12 * Math.Max(1, p.ToVector().Magnitude))
                return (ta, tb, true);
            var da = pa.Curve.Tangent(ta);
            var db = pb.Curve.Tangent(tb);
            // p(ta + x) - q(tb + y) = 0: da x - db y = -r
            double det = -da.X * db.Y + db.X * da.Y;
            if (det == 0 || !double.IsFinite(det))
                return (ta, tb, false);
            double x = (-rx * -db.Y - -ry * -db.X) / det;
            double y = (da.X * -ry - da.Y * -rx) / det;
            ta += x;
            tb += y;
            if (!double.IsFinite(ta) || !double.IsFinite(tb))
                return (ta, tb, false);
            if (Math.Abs(x) <= 1e-13 * scaleA && Math.Abs(y) <= 1e-13 * scaleB)
                return (ta, tb, true);
        }
        return (ta, tb, false);
    }

    /// <summary>
    /// Solves for the line from curve A at <paramref name="ta"/> to curve B at <paramref name="tb"/> touching each curve
    /// whose end is free: the tangent there parallel to the line.
    /// </summary>
    private static (double, double, bool) Newton(Piece pa, Piece pb, double ta, double tb, bool freeA, bool freeB)
    {
        double scaleA = pa.Max - pa.Min, scaleB = pb.Max - pb.Min;
        (double, double) F(double u, double v)
        {
            var p = pa.Curve.Evaluate(u);
            var q = pb.Curve.Evaluate(v);
            double dx = q.X - p.X, dy = q.Y - p.Y, length = Math.Sqrt(dx * dx + dy * dy);
            double f = 0, g = 0;
            if (freeA)
            {
                var d = pa.Curve.Tangent(u);
                f = (d.X * dy - d.Y * dx) / (length * Math.Max(Math.Sqrt(d.X * d.X + d.Y * d.Y), 1e-300));
            }
            if (freeB)
            {
                var d = pb.Curve.Tangent(v);
                g = (d.X * dy - d.Y * dx) / (length * Math.Max(Math.Sqrt(d.X * d.X + d.Y * d.Y), 1e-300));
            }
            return (f, g);
        }
        for (int iteration = 0; iteration < 30; iteration++)
        {
            var (f, g) = F(ta, tb);
            if (Math.Abs(f) < 1e-14 && Math.Abs(g) < 1e-14)
                return (ta, tb, true);
            double ha = 1e-7 * scaleA, hb = 1e-7 * scaleB;
            double fa = 0, ga = 0, fb = 0, gb = 0;
            if (freeA)
            {
                var (f1, g1) = F(ta + ha, tb);
                var (f0, g0) = F(ta - ha, tb);
                (fa, ga) = ((f1 - f0) / (2 * ha), (g1 - g0) / (2 * ha));
            }
            if (freeB)
            {
                var (f1, g1) = F(ta, tb + hb);
                var (f0, g0) = F(ta, tb - hb);
                (fb, gb) = ((f1 - f0) / (2 * hb), (g1 - g0) / (2 * hb));
            }
            double da, db;
            if (freeA && freeB)
            {
                double det = fa * gb - fb * ga;
                if (det == 0 || !double.IsFinite(det))
                    return (ta, tb, false);
                da = (-f * gb + g * fb) / det;
                db = (-g * fa + f * ga) / det;
            }
            else if (freeA)
            {
                if (fa == 0)
                    return (ta, tb, false);
                (da, db) = (-f / fa, 0);
            }
            else
            {
                if (gb == 0)
                    return (ta, tb, false);
                (da, db) = (0, -g / gb);
            }
            ta += da;
            tb += db;
            if (!double.IsFinite(ta) || !double.IsFinite(tb))
                return (ta, tb, false);
            if (Math.Abs(da) <= 1e-12 * scaleA && Math.Abs(db) <= 1e-12 * scaleB)
                return (ta, tb, true);
        }
        return (ta, tb, false);
    }
}
