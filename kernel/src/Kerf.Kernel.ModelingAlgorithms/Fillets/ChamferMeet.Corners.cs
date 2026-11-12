// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// What the chamfers at a convex corner take off together beyond their edges' sections (unit 15V, slice 2): integrated along
/// lines through a box about the corner, each line cut exactly where it crosses one of the corner's surfaces, and the lines'
/// lengths integrated over the box's face by Gauss's points.
/// </summary>
/// <remarks>
/// <para>
/// Near a corner every edge there is convex, so the input is what's behind all of the corner's faces; a chamfer takes off
/// what's past its surface, and an edge's section region is what's past its chamfer and beyond the plane square to the edge
/// at the corner. Each is the sign of a smooth function along a line: the distance from a face's or a chamfer's surface, by
/// its nearest point (Newton's method from the last), or from the plane. So along each line the excess (in the input and past
/// any chamfer, less each edge's section region it's in) is constant between the places where one of them changes sign,
/// found by the Illinois rule to the box's size times 1e-13, and its length is exact to that.
/// </para>
/// <para>
/// The lengths are continuous over the box's face, smooth between the curves where a line grazes a surface, so Gauss's
/// points converge on them, as the square of the panels' size: the excess with two panels and with four of eight points each
/// way, the finer taken and their difference allowed, three times what that rate leaves in the finer.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    // Samples along each line, between which the signs are followed
    private const int LineSamples = 12;

    // Points each way on each side of a tight box, where the excess has to be nothing
    private const int SideSamples = 9;

    // A surface or plane whose sign along a line is followed: positive outside a face, past a chamfer, beyond a plane
    private sealed class Signed
    {
        private readonly Surface? _surface;
        private readonly double _u0, _u1, _v0, _v1, _sign;
        private readonly bool _clampV, _clampU;
        private readonly double _within = double.NaN;
        private readonly Point3D _origin;
        private readonly Vector3D _normal;
        private double _u, _v;

        public Signed(Surface surface, double u0, double u1, double v0, double v1, double sign, Point3D near, bool carriedOn = false)
        {
            (_surface, _u0, _u1, _v0, _v1, _sign) = (surface, u0, u1, v0, v1, sign);
            _clampV = !surface.IsVPeriodic && !carriedOn;
            _clampU = !carriedOn;
            var start = new ProjectPointOnSurface(near, surface, u0, u1, v0, v1);
            if (start.NbPoints() == 0)
            {
                Failed = true;
                return;
            }
            start.LowerDistanceParameters(out _u, out _v);
        }

        public Signed(Point3D origin, Vector3D normal) => (_origin, _normal) = (origin, normal);

        // Positive within the distance of the surface's patch, rather than on one side of it
        public Signed(Surface surface, double u0, double u1, double v0, double v1, Point3D near, double within)
            : this(surface, u0, u1, v0, v1, 1, near) => _within = within;

        public bool Failed { get; private set; }

        // Each point's nearest from where the corner's was, not from the last point's (unit 15U4): the same value for a point
        // whatever was asked before it. From the last point's, a face carried on that bends within the box could be followed
        // onto a far part of itself and stay there along a line
        public bool FromCorner { get; init; }

        // The value at the point; from the corner's, the nearest point may be left once it moves no more than rest
        public double At(Point3D x, double rest = 0)
        {
            if (_surface is null)
            {
                return new Vector3D(_origin, x).Dot(_normal);
            }
            double u = _u, v = _v;
            Point3D foot = default;
            Vector3D su = default, sv = default;
            for (int iteration = 0; iteration < 20; iteration++)
            {
                _surface.D1(u, v, out foot, out su, out sv);
                var r = new Vector3D(x, foot);
                double gu = r.Dot(su), gv = r.Dot(sv);
                double a = su.Dot(su), b = su.Dot(sv), c = sv.Dot(sv), det = (a * c) - (b * b);
                if (det <= 0)
                {
                    Failed = true;
                    return double.NaN;
                }
                double du = ((c * gu) - (b * gv)) / det, dv = ((a * gv) - (b * gu)) / det;
                double nextU = _clampU ? Math.Clamp(u - du, _u0, _u1) : u - du;
                double nextV = _clampV ? Math.Clamp(v - dv, _v0, _v1) : v - dv;
                // From the corner's, it has come to rest once it moves no more than 1e-11 of the distance, held at the patch's
                // side or not (unit 15U4)
                bool rested = FromCorner && (Math.Abs(nextU - u) * Math.Sqrt(a)) + (Math.Abs(nextV - v) * Math.Sqrt(c)) < Math.Max(rest, 1e-11 * (1 + r.Magnitude));
                (u, v) = (nextU, nextV);
                if (rested || (Math.Abs(du) * Math.Sqrt(a)) + (Math.Abs(dv) * Math.Sqrt(c)) < 1e-14 * (1 + r.Magnitude))
                {
                    break;
                }
            }
            _surface.D1(u, v, out foot, out su, out sv);
            if (!FromCorner)
            {
                (_u, _v) = (u, v);
            }
            if (!double.IsNaN(_within))
            {
                return _within - foot.DistanceTo(x);
            }
            Vector3D n = su.Crossed(sv);
            double length = n.Magnitude;
            return length == 0 ? double.NaN : _sign * new Vector3D(foot, x).Dot(n) / length;
        }
    }

    // The excess at a convex corner, and its quadrature's error, or NaN where it can't be worked out. In a tight box (one cut
    // down to the room the corner has, unit 15U4) a chamfer takes off only what is within its depth of its surface's patch, so
    // that the patch's side carried on without end, which on faces that bend within the box comes back into the input far
    // from the edge, counts for nothing; every nearest point is found from the corner's, so that a point's value doesn't hang
    // on the points before it; the lines are sampled at a third of the shallowest chamfer's depth, and only where the excess
    // can be other than nothing; and it has to be nothing on the box's sides (81 points on each), or the box didn't hold it.
    // The box is as small as that allows, from three of the deepest chamfer's depths up to the room
    private static (double Excess, double Error) CornerExcess(Corner corner, Face[] faces, Edge[] edges, List<Band> bands, double reach, bool tight = false)
    {
        Point3D o = corner.Point;
        Face[] around = [.. faces.Where(f => f.AllVertices().Any(v => v.IsSame(corner.Vertex)))];
        Edge[] meeting = [.. around.SelectMany(static f => f.AllEdges()).Where(e => e.AllVertices().Any(v => v.IsSame(corner.Vertex)))
                                   .DistinctBy(static e => (Shape)e, ShapeHasher.Instance)];
        if (meeting.Any(e => !Tool.Degenerated(e) && IsConvex(faces, e) != true))
        {
            return (double.NaN, double.NaN);
        }
        var inside = new List<Signed>();
        foreach (Face face in around)
        {
            Surface surface = Tool.Surface(face)!;
            BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
            inside.Add(new Signed(surface, u0, u1, v0, v1, face.Orientation == Orientation.Reversed ? -1 : 1, o, carriedOn: true) { FromCorner = tight });
        }
        var past = new List<Signed>();
        var within = new List<Signed>();
        var beyond = new List<Signed>();
        var sides = new List<(int One, int Two)>();
        double shallowest = double.MaxValue, deepest = 0;
        foreach (Edge edge in edges)
        {
            int[] two = [.. Enumerable.Range(0, around.Length).Where(k => OnFace(edge, around[k]))];
            if (two.Length != 2)
            {
                return (double.NaN, double.NaN);
            }
            sides.Add((two[0], two[1]));
            if ((corner.Bands.FirstOrDefault(b => b.Edges.Any(x => x.IsSame(edge))) ?? bands.FirstOrDefault(b => b.Edges.Any(x => x.IsSame(edge)))) is not { } band || TangentAt(edge, corner.Vertex) is not { } t)
            {
                return (double.NaN, double.NaN);
            }
            past.Add(new Signed(band.Surface, band.EU0, band.EU1, band.EV0, band.EV1, band.EdgeSign, o) { FromCorner = tight });
            if (tight)
            {
                // The chamfer's depth: how far its edge is from its surface, which no point it takes off is farther than
                if (Tool.Curve(edge, out double a, out double b) is not { } curve)
                {
                    return (double.NaN, double.NaN);
                }
                double depth = 0;
                for (int k = 0; k <= 8; k++)
                {
                    var onBand = new ProjectPointOnSurface(curve.EvalD0(a + ((b - a) * k / 8)), band.Surface, band.EU0, band.EU1, band.EV0, band.EV1);
                    if (onBand.NbPoints() == 0)
                    {
                        return (double.NaN, double.NaN);
                    }
                    depth = Math.Max(depth, onBand.LowerDistance());
                }
                shallowest = Math.Min(shallowest, depth);
                deepest = Math.Max(deepest, depth);
                within.Add(new Signed(band.Surface, band.EU0, band.EU1, band.EV0, band.EV1, o, (1.05 * depth) + 1e-7) { FromCorner = true });
            }
            beyond.Add(new Signed(o, Ends(edge).First.IsSame(corner.Vertex) ? t : -t)); // the tangent out of the corner along the edge
        }
        if (inside.Concat(past).Concat(within).Any(static s => s.Failed) || (tight && !(shallowest > 0)))
        {
            return (double.NaN, double.NaN);
        }
        Signed[] all = [.. inside, .. past, .. within, .. beyond];
        bool Past(int e, Point3D x) => past[e].At(x) > 0 && (!tight || within[e].At(x) > 0);

        double Excess(Point3D x)
        {
            if (inside.Any(s => !(s.At(x) <= 0)))
            {
                return 0;
            }
            bool any = false;
            for (int e = 0; e < past.Count; e++)
            {
                if (Past(e, x))
                {
                    any = true;
                }
            }
            return any ? 1 : 0;
        }

        // In an edge's section region, as the sections along the edge counted it: past its chamfer, beyond the plane at the
        // corner, and behind its own two faces (carried on past the corner as the sections' contacts were), whatever the
        // corner's other faces say
        double Sections(Point3D x)
        {
            double sum = 0;
            for (int e = 0; e < past.Count; e++)
            {
                if (Past(e, x) && beyond[e].At(x) > 0 && !(inside[sides[e].One].At(x) > 0) && !(inside[sides[e].Two].At(x) > 0))
                {
                    sum++;
                }
            }
            return sum;
        }

        if (!tight)
        {
            return Integrated(o, reach, all, x => Excess(x) - Sections(x));
        }

        // The box: three of the deepest chamfer's depths at first, half as large again while the excess is something on its
        // sides, and no larger than the room
        bool Clear(double half)
        {
            for (int side = 0; side < 6; side++)
            {
                for (int i = 0; i < SideSamples; i++)
                {
                    for (int j = 0; j < SideSamples; j++)
                    {
                        double p = half * ((((2 * i) + 1.0) / SideSamples) - 1), q = half * ((((2 * j) + 1.0) / SideSamples) - 1), r = side % 2 == 0 ? -half : half;
                        Point3D x = (side / 2) switch
                        {
                            0 => new Point3D(o.X + r, o.Y + p, o.Z + q),
                            1 => new Point3D(o.X + p, o.Y + r, o.Z + q),
                            _ => new Point3D(o.X + p, o.Y + q, o.Z + r),
                        };
                        if (!(Excess(x) - Sections(x) == 0))
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }
        double room = reach;
        reach = Math.Min(room, 3 * deepest);
        while (!Clear(reach))
        {
            if (reach >= room)
            {
                return (double.NaN, double.NaN);
            }
            reach = Math.Min(room, 1.5 * reach);
        }

        // Where the excess can be other than nothing: within a chamfer's depth of its patch, and there within another's too
        // (two sections), not beyond that edge's plane at the corner (past the chamfer, in no section), or outside a face
        // of the corner that isn't the edge's (a section outside the input). The greatest over the chamfers of the least of
        // those is positive wherever the excess isn't nothing, and each is a distance, so it falls no faster than length
        int samples = Math.Clamp((int)Math.Ceiling(6 * reach / shallowest), LineSamples, 8 * LineSamples);
        double rest = 0.01 * 2 * reach / samples; // a hundredth of a sample: the gate is asked how far, not where
        double Gate(Point3D x)
        {
            double most = double.NegativeInfinity;
            for (int e = 0; e < within.Count; e++)
            {
                double near = within[e].At(x, rest);
                if (double.IsNaN(near))
                {
                    return double.NaN;
                }
                if (near <= most)
                {
                    continue;
                }
                double other = -beyond[e].At(x);
                for (int f = 0; f < within.Count && other < near; f++)
                {
                    if (f != e)
                    {
                        other = Math.Max(other, within[f].At(x, rest));
                    }
                }
                for (int k = 0; k < inside.Count && other < near; k++)
                {
                    if (k != sides[e].One && k != sides[e].Two)
                    {
                        other = Math.Max(other, inside[k].At(x, rest));
                    }
                }
                if (double.IsNaN(other))
                {
                    return double.NaN;
                }
                most = Math.Max(most, Math.Min(near, other));
            }
            return most;
        }
        return Integrated(o, reach, all, x => Excess(x) - Sections(x), samples, Gate);
    }

    // The excess at a corner of convex and concave edges (unit 15U3): what its formula holds without its chamfers and not
    // with them, less each edge's section region, in a box that lies inside the reach where the formula says what's built
    private static (double Excess, double Error) MixedExcess(Corner corner, Term formula, Edge[] edges, List<Carried> carried, double reach)
    {
        Point3D o = corner.Point;
        Leaf[] leaves = [.. Leaves(formula)];
        var inside = new List<Signed>();
        foreach (Leaf leaf in leaves)
        {
            double sign = leaf.Face.Orientation == Orientation.Reversed ? -1 : 1;
            if (carried.FirstOrDefault(c => c.Input.IsSame(leaf.Face)) is { } c)
            {
                inside.Add(new Signed(c.Surface, c.U0, c.U1, c.V0, c.V1, sign, o, carriedOn: true));
                continue;
            }
            BrepTools.UVBounds(leaf.Face, out double u0, out double u1, out double v0, out double v1);
            inside.Add(new Signed(Tool.Surface(leaf.Face)!, u0, u1, v0, v1, sign, o, carriedOn: true));
        }
        var joined = new Dictionary<Band, Signed>(ReferenceEqualityComparer.Instance);
        foreach (Band band in leaves.SelectMany(static l => l.Joined))
        {
            joined[band] = new Signed(band.Surface, band.EU0, band.EU1, band.EV0, band.EV1, band.EdgeSign, o);
        }
        var past = new List<Signed>();
        var beyond = new List<Signed>();
        var sides = new List<(int One, int Two)>();
        foreach (Edge edge in edges)
        {
            int[] two = [.. Enumerable.Range(0, leaves.Length).Where(k => OnFace(edge, leaves[k].Face))];
            if (two.Length != 2 || corner.Bands.FirstOrDefault(b => b.Edges.Any(x => x.IsSame(edge))) is not { Convex: true } band
                || !joined.TryGetValue(band, out Signed? side) || TangentAt(edge, corner.Vertex) is not { } t)
            {
                return (double.NaN, double.NaN);
            }
            sides.Add((two[0], two[1]));
            past.Add(side);
            beyond.Add(new Signed(o, Ends(edge).First.IsSame(corner.Vertex) ? t : -t));
        }
        if (inside.Concat(joined.Values).Any(static s => s.Failed) || joined.Count != past.Count)
        {
            return (double.NaN, double.NaN);
        }
        Signed[] all = [.. inside, .. past, .. beyond];

        bool Behind(Face face, Point3D x) => !(inside[Array.FindIndex(leaves, l => l.Face.IsSame(face))].At(x) > 0);

        // What the chamfers take off: in the formula without them, and not with them
        double Excess(Point3D x)
        {
            bool input = Holds(formula, x, Behind, static (_, _) => null);
            bool result = Holds(formula, x, Behind, (b, p) => joined[b].At(p) > 0 != b.Convex);
            return (input ? 1 : 0) - (result ? 1 : 0);
        }

        // In an edge's section region, as the sections along the edge counted it: behind its own two faces carried on
        double Sections(Point3D x)
        {
            double sum = 0;
            for (int e = 0; e < past.Count; e++)
            {
                if (past[e].At(x) > 0 && beyond[e].At(x) > 0 && !(inside[sides[e].One].At(x) > 0) && !(inside[sides[e].Two].At(x) > 0))
                {
                    sum++;
                }
            }
            return sum;
        }

        return Integrated(o, reach, all, x => Excess(x) - Sections(x));
    }

    // The integral over the box of half-side reach about o of a count that changes only where one of the functions changes sign.
    // With a gate, the count is nothing wherever the gate isn't positive, and the gate falls no faster than the distance along
    // a line: a line is then followed only where it is within a sample of positive, stepping over half of what it lacks
    // elsewhere, and its sign changes are found to 1e-9 of the box
    private static (double Excess, double Error) Integrated(Point3D o, double reach, Signed[] all, Func<Point3D, double> count, int samples = LineSamples,
                                                            Func<Point3D, double>? gate = null)
    {
        // Along the line through (a, b) square to the box's face: the length where the excess is, between its sign changes
        double Line(double a, double b)
        {
            Cancellation.ThrowIfRequested();
            Point3D At(double z) => new(o.X + a, o.Y + b, o.Z + z);
            var cuts = new List<double> { -reach, reach };
            bool[]? near = null;
            if (gate is not null)
            {
                near = new bool[samples + 1];
                double step = 2 * reach / samples;
                bool any = false;
                for (int i = 0; i <= samples;)
                {
                    double most = gate(At(-reach + (2 * reach * i / samples)));
                    if (double.IsNaN(most))
                    {
                        return double.NaN;
                    }
                    if (most > -step)
                    {
                        near[i] = true;
                        any = true;
                        i++;
                    }
                    else
                    {
                        i += Math.Max(1, (int)(-0.5 * most / step));
                    }
                }
                if (!any)
                {
                    return 0;
                }
            }
            bool Followed(int i) => near is null || near[i] || near[i + 1];
            var values = new double[all.Length, samples + 1];
            for (int i = 0; i <= samples; i++)
            {
                if (!(Followed(Math.Min(i, samples - 1)) || Followed(Math.Max(i - 1, 0))))
                {
                    continue;
                }
                Point3D x = At(-reach + (2 * reach * i / samples));
                for (int k = 0; k < all.Length; k++)
                {
                    values[k, i] = all[k].At(x);
                    if (double.IsNaN(values[k, i]))
                    {
                        return double.NaN;
                    }
                }
            }
            for (int k = 0; k < all.Length; k++)
            {
                for (int i = 0; i < samples; i++)
                {
                    if (!Followed(i) || Math.Sign(values[k, i]) == Math.Sign(values[k, i + 1]))
                    {
                        continue;
                    }
                    double lo = -reach + (2 * reach * i / samples), hi = lo + (2 * reach / samples);
                    double flo = values[k, i], fhi = values[k, i + 1];
                    int side = 0;
                    for (int iteration = 0; iteration < 100 && hi - lo > (gate is null ? 1e-13 : 1e-9) * reach; iteration++)
                    {
                        // The Illinois rule: the secant, the stale end's value halved when the same end moves twice
                        double mid = flo == fhi ? 0.5 * (lo + hi) : Math.Clamp(hi - (fhi * (hi - lo) / (fhi - flo)), lo, hi);
                        double f = all[k].At(At(mid));
                        if (double.IsNaN(f))
                        {
                            return double.NaN;
                        }
                        if (f == 0)
                        {
                            (lo, hi) = (mid, mid);
                            break;
                        }
                        if (Math.Sign(f) == Math.Sign(flo))
                        {
                            (lo, flo) = (mid, f);
                            if (side == -1)
                            {
                                fhi *= 0.5;
                            }
                            side = -1;
                        }
                        else
                        {
                            (hi, fhi) = (mid, f);
                            if (side == 1)
                            {
                                flo *= 0.5;
                            }
                            side = 1;
                        }
                    }
                    cuts.Add(0.5 * (lo + hi));
                }
            }
            cuts.Sort();
            double length = 0;
            for (int i = 0; i + 1 < cuts.Count; i++)
            {
                if (cuts[i + 1] > cuts[i])
                {
                    Point3D mid = At(0.5 * (cuts[i] + cuts[i + 1]));
                    length += (cuts[i + 1] - cuts[i]) * count(mid);
                }
            }
            return length;
        }

        double Quadrature(int panels)
        {
            double h = 2 * reach / panels, sum = 0;
            for (int pa = 0; pa < panels; pa++)
            {
                for (int i = 0; i < GaussNodes.Length; i++)
                {
                    double a = -reach + (h * (pa + (0.5 * (1 + GaussNodes[i]))));
                    for (int pb = 0; pb < panels; pb++)
                    {
                        for (int j = 0; j < GaussNodes.Length; j++)
                        {
                            double b = -reach + (h * (pb + (0.5 * (1 + GaussNodes[j]))));
                            sum += 0.25 * h * h * GaussWeights[i] * GaussWeights[j] * Line(a, b);
                        }
                    }
                }
            }
            return sum;
        }

        double coarse = Quadrature(2), fine = Quadrature(4);
        return double.IsNaN(coarse) || double.IsNaN(fine) ? (double.NaN, double.NaN) : (fine, Math.Abs(fine - coarse));
    }
}
