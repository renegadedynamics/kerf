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

    // A surface or plane whose sign along a line is followed: positive outside a face, past a chamfer, beyond a plane
    private sealed class Signed
    {
        private readonly Surface? _surface;
        private readonly double _u0, _u1, _v0, _v1, _sign;
        private readonly bool _clampV, _clampU;
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

        public bool Failed { get; private set; }

        public double At(Point3D x)
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
                u = _clampU ? Math.Clamp(u - du, _u0, _u1) : u - du;
                v = _clampV ? Math.Clamp(v - dv, _v0, _v1) : v - dv;
                if ((Math.Abs(du) * Math.Sqrt(a)) + (Math.Abs(dv) * Math.Sqrt(c)) < 1e-14 * (1 + r.Magnitude))
                {
                    break;
                }
            }
            _surface.D1(u, v, out foot, out su, out sv);
            (_u, _v) = (u, v);
            Vector3D n = su.Crossed(sv);
            double length = n.Magnitude;
            return length == 0 ? double.NaN : _sign * new Vector3D(foot, x).Dot(n) / length;
        }
    }

    // The excess at a convex corner, and its quadrature's error, or NaN where it can't be worked out
    private static (double Excess, double Error) CornerExcess(Corner corner, Face[] faces, Edge[] edges, List<Band> bands, double reach)
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
            inside.Add(new Signed(surface, u0, u1, v0, v1, face.Orientation == Orientation.Reversed ? -1 : 1, o, carriedOn: true));
        }
        var past = new List<Signed>();
        var beyond = new List<Signed>();
        var sides = new List<(int One, int Two)>();
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
            past.Add(new Signed(band.Surface, band.EU0, band.EU1, band.EV0, band.EV1, band.EdgeSign, o));
            beyond.Add(new Signed(o, Ends(edge).First.IsSame(corner.Vertex) ? t : -t)); // the tangent out of the corner along the edge
        }
        if (inside.Concat(past).Any(static s => s.Failed))
        {
            return (double.NaN, double.NaN);
        }
        Signed[] all = [.. inside, .. past, .. beyond];

        double Excess(Point3D x)
        {
            if (inside.Any(s => !(s.At(x) <= 0)))
            {
                return 0;
            }
            bool any = false;
            for (int e = 0; e < past.Count; e++)
            {
                if (past[e].At(x) > 0)
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
                if (past[e].At(x) > 0 && beyond[e].At(x) > 0 && !(inside[sides[e].One].At(x) > 0) && !(inside[sides[e].Two].At(x) > 0))
                {
                    sum++;
                }
            }
            return sum;
        }

        // Along the line through (a, b) square to the box's face: the length where the excess is, between its sign changes
        double Line(double a, double b)
        {
            Cancellation.ThrowIfRequested();
            Point3D At(double z) => new(o.X + a, o.Y + b, o.Z + z);
            var cuts = new List<double> { -reach, reach };
            var values = new double[all.Length, LineSamples + 1];
            for (int i = 0; i <= LineSamples; i++)
            {
                Point3D x = At(-reach + (2 * reach * i / LineSamples));
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
                for (int i = 0; i < LineSamples; i++)
                {
                    if (Math.Sign(values[k, i]) == Math.Sign(values[k, i + 1]))
                    {
                        continue;
                    }
                    double lo = -reach + (2 * reach * i / LineSamples), hi = lo + (2 * reach / LineSamples);
                    double flo = values[k, i], fhi = values[k, i + 1];
                    int side = 0;
                    for (int iteration = 0; iteration < 100 && hi - lo > 1e-13 * reach; iteration++)
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
                    length += (cuts[i + 1] - cuts[i]) * (Excess(mid) - Sections(mid));
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
