// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// Chamfers that run on through a vertex, their edges tangent there, which ChFi3d split into stripes meeting at a corner (unit
/// 15V): built as one band of Kerf's own, each section's contacts solved where it lies on the input's faces, so no corner is
/// needed there and the other chamfers at the vertex end on the band.
/// </summary>
/// <remarks>
/// <para>
/// A loft from a square to a circle has its rim's arcs tangent at each top vertex, and its two side faces there tangent too:
/// ChFi3d's corner of three chamfers has no crossing to start from (decision 2136), and its arcs' stripes, carried past the
/// vertex, lie along each other, so the general fuse can't split them cleanly either. One band through the vertex has neither.
/// </para>
/// <para>
/// <b>The band.</b> At points of the chain, the section in the plane square to the edge: on each side the point of the face at
/// the chamfer's distance from the edge's point (ChFi3d's equal-distance chamfer), by Newton's method on the face's surface,
/// on whichever of that side's faces holds it. A cubic through each side's points (periodic round a closed chain) and the
/// ruled surface between them: its contacts are its u-isos, u = 0 on the first side, u = 1 on the second, as a stripe's are.
/// Only one length on both faces is asked, and one side has to be one face for the whole chain.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    // Sections on each edge of a chain, gathered towards its ends: 16 left the loft's contacts off the length, and 48 made the
    // fuse a tenth slower, its bands' poles one to a section
    private const int SectionsPerEdge = 24;

    // Edges are taken as running on where their tangents at the vertex are this close, in radians
    private const double Tangent = 1e-6;

    private sealed record Chain(Stripe[] Stripes, (Edge Edge, bool Reversed)[] Edges, bool Closed, Face Common, Vertex First, Vertex Last);

    // The stripes that run on through a corner, as chains of two or more, and each stripe on its own where asked; the rest are
    // left as they are
    private static List<Chain> Chains(Face[] faces, Stripe[] stripes, List<Vertex> corners, bool alone = false)
    {
        var chains = new List<Chain>();
        var used = new HashSet<Stripe>();
        foreach (Stripe start in stripes)
        {
            if (used.Contains(start))
            {
                continue;
            }

            // Walk back to the chain's first stripe, then forward, joining at each end where the next stripe runs on
            Stripe first = start;
            bool firstReversed = false;
            var seen = new HashSet<Stripe> { start };
            while (Next(first, !firstReversed, stripes) is { } back && seen.Add(back.Stripe))
            {
                (first, firstReversed) = (back.Stripe, !back.Reversed);
            }
            var order = new List<(Stripe Stripe, bool Reversed)> { (first, firstReversed) };
            var inChain = new HashSet<Stripe> { first };
            bool closed = false;
            while (Next(order[^1].Stripe, order[^1].Reversed, stripes) is { } next)
            {
                if (ReferenceEquals(next.Stripe, first))
                {
                    closed = true;
                    break;
                }
                if (!inChain.Add(next.Stripe))
                {
                    break;
                }
                order.Add(next);
            }
            if (order.Count < 2 && !alone)
            {
                continue;
            }
            foreach ((Stripe s, _) in order)
            {
                used.Add(s);
            }

            // The edges in order, each with its direction along the chain
            var edges = new List<(Edge, bool)>();
            Spine head = order[0].Stripe.Spine()!;
            Vertex at = order[0].Reversed ? head.LastVertex() : head.FirstVertex();
            foreach ((Stripe s, bool reversed) in order)
            {
                Spine spine = s.Spine()!;
                foreach (int k in reversed ? Enumerable.Range(1, spine.EdgeCount).Reverse() : Enumerable.Range(1, spine.EdgeCount))
                {
                    Edge e = spine.Edges(k);
                    (Vertex a, Vertex b) = Ends(e);
                    bool back = !a.IsSame(at);
                    if (back && !b.IsSame(at))
                    {
                        return [];
                    }
                    edges.Add((e, back));
                    at = back ? a : b;
                }
            }
            Face[] common = [.. faces.Where(f => edges.All(x => OnFace(x.Item1, f)))];
            if (common.Length == 0 || (common.Length > 1 && order.Count > 1))
            {
                return [];
            }
            Vertex v0 = closed ? default : StartOf(edges[0]), v1 = closed ? default : EndOf(edges[^1]);
            chains.Add(new Chain([.. order.Select(static o => o.Stripe)], [.. edges], closed, common[0],
                                 closed || !corners.Any(c => c.IsSame(v0)) ? default : v0, closed || !corners.Any(c => c.IsSame(v1)) ? default : v1));
        }
        return chains;
    }

    private static (Vertex First, Vertex Last) Ends(Edge e)
    {
        Vertex a = default, b = default;
        foreach (Vertex v in e.AllVertices())
        {
            if (v.Orientation == Orientation.Forward)
            {
                a = v;
            }
            else if (v.Orientation == Orientation.Reversed)
            {
                b = v;
            }
        }
        return (a, b);
    }

    private static Vertex StartOf((Edge Edge, bool Reversed) e) => e.Reversed ? Ends(e.Edge).Last : Ends(e.Edge).First;

    private static Vertex EndOf((Edge Edge, bool Reversed) e) => e.Reversed ? Ends(e.Edge).First : Ends(e.Edge).Last;

    // The stripe that runs on from this one's end (its last, or its first if reversed), and whether it's then reversed
    private static (Stripe Stripe, bool Reversed)? Next(Stripe stripe, bool reversed, Stripe[] stripes)
    {
        Spine spine = stripe.Spine()!;
        Vertex end = reversed ? spine.FirstVertex() : spine.LastVertex();
        Edge last = reversed ? spine.Edges(1) : spine.Edges(spine.EdgeCount);
        if (TangentAt(last, end) is not { } t)
        {
            return null;
        }
        (Stripe, bool)? found = null;
        foreach (Stripe other in stripes)
        {
            if (ReferenceEquals(other, stripe) && !spine.FirstVertex().IsSame(spine.LastVertex()))
            {
                continue;
            }
            Spine o = other.Spine()!;
            foreach ((Vertex v, Edge e, bool r) in new[] { (o.FirstVertex(), o.Edges(1), false), (o.LastVertex(), o.Edges(o.EdgeCount), true) })
            {
                if (!v.IsSame(end) || (ReferenceEquals(other, stripe) && r == reversed) || TangentAt(e, v) is not { } u)
                {
                    continue;
                }
                // Running on: the tangent out of the vertex along the other is the tangent into it along this one
                if (Math.Abs(Math.Abs(t.Dot(u)) - 1) < Tangent)
                {
                    if (found is not null)
                    {
                        return null; // more than one way on
                    }
                    found = (other, r);
                }
            }
        }
        return found;
    }

    // A chamfer's cut as its spec has it: the distance on the reference face, and on the other face the second distance (the
    // same where there's none), or the angle its section makes there with the reference face's (ChFi3d's BlendFunc_ChAsym)
    private readonly record struct Cut(double Distance, double? Second, double? Angle)
    {
        public bool Equal => Second is null && Angle is null;
    }

    // The one cut of the specs on these edges, with their reference faces; null where there's none or they differ
    private static (Cut Cut, Face[] References)? CutOf(ChamferSpec[] specs, Face[] referenceFaces, Edge[] edges)
    {
        int[] own = [.. Enumerable.Range(0, specs.Length).Where(i => edges.Any(e => e.IsSame(specs[i].Edge)))];
        if (own.Length == 0)
        {
            return null;
        }
        static Cut Of(ChamferSpec s) => new(s.Distance, s.Angle is null && s.Distance2 is { } d2 && d2 != s.Distance ? d2 : null, s.Angle);
        Cut cut = Of(specs[own[0]]);
        return own.Any(i => Of(specs[i]) != cut) ? null : (cut, [.. own.Select(i => referenceFaces[i])]);
    }

    // Whether the face is the cut's reference face: any, for one length on both; null where the specs' reference faces don't agree
    private static bool? IsReference(Cut cut, Face[] references, Face face)
    {
        if (cut.Equal)
        {
            return true;
        }
        int same = references.Count(r => !r.IsNull && r.IsSame(face));
        return same == references.Length && same > 0 ? true : same == 0 && references.All(static r => !r.IsNull) ? false : null;
    }

    // One side of an edge along which sections are solved: the face to start on, those that may hold the contact, and where
    // the last one was found
    private sealed class Side
    {
        public Face Start { get; set; }

        public Face[] Candidates { get; set; } = [];

        public Edge InStart { get; set; }

        public Point2D? Last;

        // The next contact's solution started at the last one's parameters, where it's on the same face, rather than at the
        // last one's point put back on the face (unit 15U4: the same start, without the search)
        public bool FromLast { get; init; }

        public Face Face;

        public Point3D? At(double t, Point3D c, Vector3D tangent, double distance, IntersectionContext context) =>
            Contact(Start, Candidates, InStart, t, c, tangent, distance, ref Last, ref Face, context, FromLast);
    }

    // An angle's contact is solved to this, in radians; where the secant stalls on the contacts' own rounding, this is taken
    private const double AngleSolved = 1e-11;

    private const double AngleStalled = 1e-5;

    // A section's two contacts: the reference side's at the cut's distance, and the other's at its own, or where the line
    // between them makes the cut's angle with the reference face's section at its contact (the secant method on the distance)
    private static (Point3D Reference, Point3D Other)? Contacts(Cut cut, Side reference, Side other, double t, Point3D c, Vector3D tangent, IntersectionContext context)
    {
        if (reference.At(t, c, tangent, cut.Distance, context) is not { } p)
        {
            return null;
        }
        if (cut.Angle is not { } angle)
        {
            return other.At(t, c, tangent, cut.Second ?? cut.Distance, context) is { } q ? (p, q) : null;
        }
        if (reference.Last is not { } uv || reference.Face.IsNull)
        {
            return null;
        }
        // The reference face's section at its contact, towards the edge
        Vector3D along = Normal(Tool.Surface(reference.Face)!, uv.X, uv.Y).Crossed(tangent);
        if (along.Dot(new Vector3D(p, c)) < 0)
        {
            along = -along;
        }
        double? Off(double d, out Point3D q)
        {
            q = default;
            if (other.At(t, c, tangent, d, context) is not { } found)
            {
                return null;
            }
            q = found;
            var chord = new Vector3D(p, q);
            return Math.Atan2(chord.Crossed(along).Magnitude, chord.Dot(along)) - angle;
        }
        double d0 = cut.Distance * Math.Tan(angle), d1 = 1.05 * d0;
        if (Off(d0, out Point3D q0) is not { } f0)
        {
            return null;
        }
        (double Off, Point3D Point) best = (Math.Abs(f0), q0);
        for (int iteration = 0; iteration < 40 && best.Off >= AngleSolved; iteration++)
        {
            if (Off(d1, out Point3D q1) is not { } f1)
            {
                return null;
            }
            if (Math.Abs(f1) < best.Off)
            {
                best = (Math.Abs(f1), q1);
            }
            if (f1 == f0)
            {
                break;
            }
            double next = d1 - (f1 * (d1 - d0) / (f1 - f0));
            (d0, f0) = (d1, f1);
            d1 = next > 0 ? next : 0.5 * d1;
        }
        return best.Off < AngleStalled ? (p, best.Point) : null;
    }

    // ChFi3d's band at the chamfer's distance from its edge along both contacts, to its approximation's tolerance and the edge's
    private static bool OnSpec(Band band, double distance, double tolerance) => OnSpec(band, distance, distance, tolerance);

    // The same with a distance for each contact, the first face's and the second's; a contact without one isn't looked at
    private static bool OnSpec(Band band, double? first, double? second, double tolerance)
    {
        for (int k = 0; k <= 8; k++)
        {
            double v = band.VLo + ((band.VHi - band.VLo) * k / 8);
            foreach ((double u, Face face, double? asked) in new[] { (0.0, band.Face1, first), (band.UMax, band.Face2, second) })
            {
                if (asked is not { } distance)
                {
                    continue;
                }
                // The contact where the band crosses its face, as the fuse will find it: off the face by the approximation's
                // error, it moves along the face by that over the sine of the angle between them
                Point3D p = band.Original.EvalD0(u, v);
                double moved = 0;
                if (!face.IsNull && Tool.Surface(face) is { } support)
                {
                    var onFace = new ProjectPointOnSurface(p, support);
                    if (onFace.NbPoints() == 0)
                    {
                        return false;
                    }
                    onFace.LowerDistanceParameters(out double fu, out double fv);
                    double sine = Normal(support, fu, fv).Crossed(Normal(band.Original, u, v)).Magnitude;
                    moved = sine > 0 ? onFace.LowerDistance() / sine : double.MaxValue;
                    p = support.EvalD0(fu, fv);
                }
                double best = double.MaxValue, allowed = tolerance;
                foreach (Edge edge in band.Edges)
                {
                    if (Tool.Curve(edge, out double a, out double b) is not { } curve)
                    {
                        continue;
                    }
                    var projection = new ProjectPointOnCurve(p, curve, a, b);
                    if (projection.NbPoints() > 0 && projection.LowerDistance() < best)
                    {
                        double t = projection.LowerDistanceParameter();
                        if (t - a <= 1e-3 * (b - a) || b - t <= 1e-3 * (b - a))
                        {
                            continue; // past the edge's end, at a corner
                        }
                        best = projection.LowerDistance();
                        allowed = tolerance + Tool.Tolerance(edge);
                    }
                }
                if (best < double.MaxValue && Math.Abs(best - distance) + moved > allowed)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // The unit tangent of the edge at one of its vertices
    private static Vector3D? TangentAt(Edge edge, Vertex vertex)
    {
        if (Tool.Curve(edge, out double a, out double b) is not { } curve)
        {
            return null;
        }
        (Vertex first, _) = Ends(edge);
        curve.D1(first.IsSame(vertex) ? a : b, out _, out Vector3D d);
        return d.Magnitude > 0 ? d / d.Magnitude : null;
    }

    // The band of a chain: sections solved on the input's faces, a cubic through each side's contacts, the ruled surface between
    private static List<Band>? ChainBands(Face[] faces, Chain chain, double distance, bool convex)
    {
        var cut = new Cut(distance, null, null);
        var context = new IntersectionContext();
        Face[] other = [.. chain.Edges.Select(x => faces.First(f => !f.IsSame(chain.Common) && OnFace(x.Edge, f))).Distinct()];
        var onCommon = new List<Point3D>();
        var onOther = new List<Point3D>();
        var spine = new List<Point3D>();
        var commonSide = new Side { Start = chain.Common, Candidates = [chain.Common] };
        var otherSide = new Side();
        for (int i = 0; i < chain.Edges.Length; i++)
        {
            (Edge edge, bool reversed) = chain.Edges[i];
            if (Tool.Curve(edge, out double a, out double b) is not { } curve)
            {
                return null;
            }
            Face own = faces.First(f => !f.IsSame(chain.Common) && OnFace(edge, f));
            int count = SectionsPerEdge;
            for (int k = 0; k < count; k++)
            {
                if (k == 0 && i > 0)
                {
                    continue; // the vertex, already taken as the last of the edge before
                }
                double s = 0.5 - (0.5 * Math.Cos(Math.PI * k / count)); // gathered towards the vertices
                if (chain.Closed && i == chain.Edges.Length - 1 && k == count)
                {
                    break;
                }
                double t = reversed ? b - (s * (b - a)) : a + (s * (b - a));
                curve.D1(t, out Point3D c, out Vector3D d1);
                Vector3D tangent = (reversed ? -d1 : d1) / d1.Magnitude;
                commonSide.InStart = chain.Common.AllEdges().First(e => e.IsSame(edge));
                (otherSide.Start, otherSide.Candidates, otherSide.InStart) = (own, [own, .. other.Where(f => !f.IsSame(own))], own.AllEdges().First(e => e.IsSame(edge)));
                if (Contacts(cut, commonSide, otherSide, t, c, tangent, context) is not var (p, q))
                {
                    return null;
                }
                spine.Add(c);
                onCommon.Add(p);
                onOther.Add(q);
            }
            if (i == chain.Edges.Length - 1 && !chain.Closed)
            {
                double t = reversed ? a : b;
                curve.D1(t, out Point3D c, out Vector3D d1);
                Vector3D tangent = (reversed ? -d1 : d1) / d1.Magnitude;
                commonSide.InStart = chain.Common.AllEdges().First(e => e.IsSame(edge));
                (otherSide.Start, otherSide.Candidates, otherSide.InStart) = (own, [own, .. other.Where(f => !f.IsSame(own))], own.AllEdges().First(e => e.IsSame(edge)));
                if (Contacts(cut, commonSide, otherSide, t, c, tangent, context) is not var (p, q))
                {
                    return null;
                }
                spine.Add(c);
                onCommon.Add(p);
                onOther.Add(q);
            }
        }

        // The parameters: the spine's length along the chain
        int n = spine.Count;
        var parameters = new double[chain.Closed ? n + 1 : n];
        for (int k = 1; k < parameters.Length; k++)
        {
            parameters[k] = parameters[k - 1] + spine[k - 1].DistanceTo(spine[k % n]);
        }
        if (Fit(onCommon, parameters, chain.Closed) is not { } c1 || Fit(onOther, parameters, chain.Closed) is not { } c2
            || !c1.Knots.SequenceEqual(c2.Knots) || !c1.Multiplicities.SequenceEqual(c2.Multiplicities) || c1.PoleCount != c2.PoleCount)
        {
            return null;
        }
        var poles = new Point3D[2 * c1.PoleCount];
        for (int k = 0; k < c1.PoleCount; k++)
        {
            poles[k] = c1.Poles[k];
            poles[c1.PoleCount + k] = c2.Poles[k];
        }
        var surface = new BSplineSurface(new Grid<Point3D>(poles, 2, c1.PoleCount), [0.0, 1.0], [.. c1.Knots], [2, 2], [.. c1.Multiplicities],
                                         1, c1.Degree, false, chain.Closed);
        double v0 = c1.FirstParameter, v1 = c1.LastParameter;
        double width = onCommon.Zip(onOther).Max(static x => x.First.DistanceTo(x.Second));

        // A closed band in two halves, as a face of the fuse can't be its own seam; an open one ends at its corners
        (double Lo, double Hi, Vertex First, Vertex Last)[] spans = chain.Closed
            ? [(Split(0), Split(0.5), default, default), (Split(0.5), Split(1), default, default)]
            : [(v0, v1, chain.First, chain.Last)];
        Edge[] edges = [.. chain.Edges.Select(static x => x.Edge)];

        // Where a closed band is cut: from the middle of its first edge, clear of the vertices, where other chamfers end
        double Split(double fraction) => parameters[SectionsPerEdge / 2] + (fraction * (v1 - v0));
        var bands = new List<Band>();
        foreach ((double lo, double hi, Vertex first, Vertex last) in spans)
        {
            int middle = Array.FindIndex(parameters, p => p >= 0.5 * (lo + hi));
            bands.Add(new Band
            {
                Original = surface,
                UMax = 1,
                A1 = new Point2D(0, lo),
                A2 = new Point2D(1, lo),
                B1 = new Point2D(0, hi),
                B2 = new Point2D(1, hi),
                Width = width,
                Convex = convex,
                FirstCorner = first,
                LastCorner = last,
                EdgePoint = spine[Math.Clamp(middle, 0, n - 1)],
                Edges = edges,
                Own = true,
            });
        }
        return bands;
    }

    private static BSplineCurve3D? Fit(List<Point3D> points, double[] parameters, bool periodic)
    {
        var p = new NCollection_HArray1<Point3D>(1, points.Count);
        for (int k = 0; k < points.Count; k++)
        {
            p.SetValue(k + 1, points[k]);
        }
        var t = new NCollection_HArray1<double>(1, parameters.Length);
        for (int k = 0; k < parameters.Length; k++)
        {
            t.SetValue(k + 1, parameters[k]);
        }
        var interpolation = new Kerf.Engine.Geometry.Api.ApiInterpolate(p, t, periodic, Precision.Confusion());
        interpolation.Perform();
        return interpolation.IsDone ? interpolation.Curve() : null;
    }

    // The point of one of the faces at the distance from the edge's point, in the plane square to the edge there, by Newton's
    // method from the last one found (or, at first, from a point of the face beside the edge); the face it's on returned
    private static Point3D? Contact(Face start, Face[] candidates, Edge inStart, double t, Point3D c, Vector3D tangent, double distance,
                                    ref Point2D? last, ref Face lastFace, IntersectionContext context, bool fromLast = false)
    {
        Point2D? lastUv = last;
        Point3D guess;
        if (last is { } previous && !lastFace.IsNull)
        {
            guess = Tool.Surface(lastFace)!.EvalD0(previous.X, previous.Y);
        }
        else
        {
            Point2D uv = default;
            Point3D into = default;
            if (AlgoTools3D.PointNearEdge(inStart, start, t, 1e-3, ref uv, ref into, context) != 0)
            {
                return null;
            }
            Vector3D across = new Vector3D(c, into);
            across -= tangent * across.Dot(tangent);
            if (across.Magnitude <= 0)
            {
                return null;
            }
            guess = c + (across / across.Magnitude * distance);
        }
        Face previousFace = lastFace;
        IEnumerable<Face> order = previousFace.IsNull ? candidates : [previousFace, .. candidates.Where(f => !f.IsSame(previousFace))];
        (Point3D Point, Point2D Uv, Face Face)? fallback = null;
        foreach (Face face in order)
        {
            Surface surface = Tool.Surface(face)!;
            double u, v;
            if (fromLast && lastUv is { } at && face.IsSame(previousFace))
            {
                (u, v) = (at.X, at.Y);
            }
            else
            {
                var projection = new ProjectPointOnSurface(guess, surface);
                if (projection.NbPoints() == 0)
                {
                    continue;
                }
                projection.LowerDistanceParameters(out u, out v);
            }
            if (Solve(surface, c, tangent, distance, ref u, ref v) is not { } p)
            {
                continue;
            }
            if (context.StatePointFace(face, new Point2D(u, v)) != Containment.Out)
            {
                last = new Point2D(u, v);
                lastFace = face;
                return p;
            }
            fallback ??= (p, new Point2D(u, v), face);
        }

        // At a corner the section can leave the faces: the first face's surface carried on, as the band is past a corner
        if (fallback is { } outside)
        {
            last = outside.Uv;
            lastFace = outside.Face;
            return outside.Point;
        }
        return null;
    }

    // Newton's method on (S - c) . t = 0, |S - c|^2 = d^2
    private static Point3D? Solve(Surface surface, Point3D c, Vector3D t, double d, ref double u, ref double v)
    {
        for (int iteration = 0; iteration < 40; iteration++)
        {
            surface.D1(u, v, out Point3D s, out Vector3D su, out Vector3D sv);
            var r = new Vector3D(c, s);
            double g1 = r.Dot(t), g2 = r.Dot(r) - (d * d);
            if (Math.Abs(g1) < 1e-13 * Math.Max(1, d) && Math.Abs(g2) < 1e-13 * Math.Max(1, d * d))
            {
                return s;
            }
            double j00 = su.Dot(t), j01 = sv.Dot(t), j10 = 2 * r.Dot(su), j11 = 2 * r.Dot(sv);
            double det = (j00 * j11) - (j01 * j10);
            if (Math.Abs(det) < 1e-300)
            {
                return null;
            }
            u -= ((g1 * j11) - (g2 * j01)) / det;
            v -= ((g2 * j00) - (g1 * j10)) / det;
        }
        surface.D1(u, v, out Point3D last, out _, out _);
        var rl = new Vector3D(c, last);
        return Math.Abs(rl.Dot(t)) < 1e-9 && Math.Abs(rl.Magnitude - d) < 1e-9 ? last : null;
    }

    // A ruled band carried on across both contacts by a fraction of its width, its parameters kept: the lines of its sections
    // run on straight
    private static BSplineSurface Across1(BSplineSurface band, double fraction)
    {
        int n = band.VPoleCount;
        Grid<Point3D> old = band.Poles;
        var poles = new Point3D[2 * n];
        for (int k = 0; k < n; k++)
        {
            Point3D p = old[0, k], q = old[1, k];
            var pq = new Vector3D(p, q);
            poles[k] = p - (pq * fraction);
            poles[n + k] = q + (pq * fraction);
        }
        double[] vKnots = [.. band.VKnots];
        int[] vMults = [.. band.VMultiplicities];
        return new BSplineSurface(new Grid<Point3D>(poles, 2, n), [-fraction, 1 + fraction], vKnots, [2, 2], vMults, 1, band.VDegree, false, band.IsVPeriodic);
    }
}
