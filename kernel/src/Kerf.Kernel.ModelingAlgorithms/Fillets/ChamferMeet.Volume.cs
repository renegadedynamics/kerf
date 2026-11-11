// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// The volume of a chamfered solid against an integral apart from the fuse's faces (unit 15V): the input's, less the region
/// each chamfer takes off along its edge, by its exact sections, less what the chamfers take off at their corners past that.
/// </summary>
/// <remarks>
/// <para>
/// <b>Along an edge,</b> between the planes square to it at its ends, each section is the region between the edge's point,
/// its contacts on the two faces (solved where they lie, at the length from the point in the plane square to the edge) and the
/// chord between them: the triangle, with the segment between each face's section and its side, by the face's sag at the
/// side's middle (a parabola's two thirds of chord times sag). Swept along the edge, each element of the section moves by
/// one less the edge's curvature vector dotted with its offset, so the volume is the integral of the section's area less its
/// first moment against the curvature, by Gauss's points. Where edges run on through a vertex their planes there are one.
/// </para>
/// <para>
/// <b>At a corner,</b> what the chamfers take off together is not the sum of their sections: in a box round the corner,
/// what's in the input and past any chamfer's surface less what's in each section's region, by the midpoint rule on two
/// grids a half cell apart, their difference for the error. Only corners of convex chamfers, which take off what's past any
/// of them.
/// </para>
/// <para>
/// The result's volume, measured, has to match to a part in a million of it, the approximation's tolerance over the chamfer's
/// area, and twice the corners' difference between grids. Everything is local to the call.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    private static ReadOnlySpan<double> GaussNodes => [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                                       0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];

    private static ReadOnlySpan<double> GaussWeights => [0.1012285362903763, 0.2223810344533745, 0.3137066661925477, 0.3626837833783620,
                                                         0.3626837833783620, 0.3137066661925477, 0.2223810344533745, 0.1012285362903763];

    // Panels of Gauss's eight points along each edge; cells along each side of a corner's box, and how far it reaches from
    // the corner, in its widest chamfer's widths
    private const int VolumePanels = 16;

    private const int CornerCells = 24;

    private const double CornerReach = 3;

    /// <summary>The expected volume and how far from it the result may be, or null where it can't be worked out.</summary>
    private static (double Expected, double Allowed)? ExpectedVolume(Shape input, Face[] faces, List<Band> bands, List<Corner> corners,
                                                                     Edge[] chamfered, ChamferSpec[] specs, Func<Point3D, bool> material,
                                                                     Func<Band, Point3D, bool, bool?> sideOf, double tolerance)
    {
        if (bands.Any(static b => !b.Convex))
        {
            return null;
        }
        var context = new IntersectionContext();
        double removed = 0, area = 0;
        foreach (Edge edge in chamfered.DistinctBy(static e => (Shape)e, ShapeHasher.Instance))
        {
            if (EqualDistance(specs, [edge]) is not { } distance || Swept(faces, edge, distance, context) is not { } swept)
            {
                return null;
            }
            removed += swept.Volume;
            area += swept.Width;
        }

        // Each corner: what's past any chamfer less what's in each edge's sections, in a box about it
        double cornersRemoved = 0, cornersError = 0;
        foreach (Corner corner in corners)
        {
            Edge[] at = [.. chamfered.Where(e => e.AllVertices().Any(v => v.IsSame(corner.Vertex))).DistinctBy(static e => (Shape)e, ShapeHasher.Instance)];
            double reach = CornerReach * corner.Bands.Max(static b => b.Width);
            double one = CornerExcess(corner, at, bands, reach, CornerCells, 0, material, sideOf);
            double other = CornerExcess(corner, at, bands, reach, CornerCells, 0.5, material, sideOf);
            if (double.IsNaN(one) || double.IsNaN(other))
            {
                return null;
            }
            cornersRemoved += 0.5 * (one + other);
            cornersError += 2 * Math.Abs(one - other);
        }
        double expected = MeasuredVolume(input) - removed - cornersRemoved;
        return (expected, (1e-6 * Math.Abs(expected)) + (tolerance * area) + cornersError);
    }

    private static double MeasuredVolume(Shape shape) => Kerf.Measurement.MeasureOperations.IntegrateVolume(shape, new Kerf.Measurement.MassOptions { Tolerance = 1e-8 });

    // The region the chamfer takes off along the edge, between the planes square to it at its ends, and the chamfer's area there
    private static (double Volume, double Width)? Swept(Face[] faces, Edge edge, double distance, IntersectionContext context)
    {
        Face[] two = [.. faces.Where(f => OnFace(edge, f))];
        if (two.Length != 2 || Tool.Curve(edge, out double a, out double b) is not { } curve)
        {
            return null;
        }
        Face[] ordered1 = [two[0], .. faces.Where(f => !f.IsSame(two[0]))], ordered2 = [two[1], .. faces.Where(f => !f.IsSame(two[1]))];
        Edge in1 = two[0].AllEdges().First(e => e.IsSame(edge)), in2 = two[1].AllEdges().First(e => e.IsSame(edge));
        Point2D? last1 = null, last2 = null;
        Face face1 = default, face2 = default;
        double volume = 0, width = 0;
        for (int panel = 0; panel < VolumePanels; panel++)
        {
            double lo = a + ((b - a) * panel / VolumePanels), hi = a + ((b - a) * (panel + 1) / VolumePanels);
            for (int i = 0; i < GaussNodes.Length; i++)
            {
                double t = (0.5 * (lo + hi)) + (0.5 * (hi - lo) * GaussNodes[i]);
                curve.D2(t, out Point3D c, out Vector3D d1, out Vector3D d2);
                double speed = d1.Magnitude;
                Vector3D tangent = d1 / speed;
                Vector3D bend = (d2 - (tangent * d2.Dot(tangent))) / (speed * speed);
                if (Contact(two[0], ordered1, in1, t, c, tangent, distance, ref last1, ref face1, context) is not { } p1
                    || Contact(two[1], ordered2, in2, t, c, tangent, distance, ref last2, ref face2, context) is not { } p2)
                {
                    return null;
                }
                (double sectionArea, Vector3D moment) = Section(c, p1, p2, face1, face2);
                double weight = 0.5 * (hi - lo) * GaussWeights[i] * speed;
                volume += weight * (sectionArea - bend.Dot(moment));
                width += weight * p1.DistanceTo(p2);
            }
        }
        return (volume, width);
    }

    // A section's area and first moment about the edge's point: the triangle, and each face's segment beside it
    private static (double Area, Vector3D Moment) Section(Point3D c, Point3D p1, Point3D p2, Face face1, Face face2)
    {
        var e1 = new Vector3D(c, p1);
        var e2 = new Vector3D(c, p2);
        double area = 0.5 * e1.Crossed(e2).Magnitude;
        Vector3D moment = (e1 + e2) / 3 * area;
        foreach ((Point3D p, Point3D other, Face face) in new[] { (p1, p2, face1), (p2, p1, face2) })
        {
            Point3D middle = new(0.5 * (c.X + p.X), 0.5 * (c.Y + p.Y), 0.5 * (c.Z + p.Z));
            var projection = new ProjectPointOnSurface(middle, Tool.Surface(face)!);
            if (projection.NbPoints() == 0)
            {
                continue;
            }
            var sag = new Vector3D(middle, projection.NearestPoint());
            var chord = new Vector3D(c, p);
            Vector3D toward = new Vector3D(middle, other);
            toward -= chord * (toward.Dot(chord) / chord.Dot(chord));
            double segment = 2.0 / 3 * chord.Magnitude * sag.Magnitude * (sag.Dot(toward) < 0 ? 1 : -1);
            area += segment;
            moment += (new Vector3D(c, middle) + (sag * 0.4)) * segment;
        }
        return (area, moment);
    }

    // At a corner, in the box about it: what's in the input and past any of its chamfers, less what's in each of its edges'
    // sections (past the chamfer and between the planes at the edge's ends), by the midpoint rule
    private static double CornerExcess(Corner corner, Edge[] edges, List<Band> bands, double reach, int cells, double shift,
                                       Func<Point3D, bool> material, Func<Band, Point3D, bool, bool?> sideOf)
    {
        Band?[] own = [.. edges.Select(e => bands.FirstOrDefault(b => b.Edges.Any(x => x.IsSame(e))))];
        if (own.Any(static b => b is null))
        {
            return double.NaN;
        }
        var curves = new (Curve3D Curve, double A, double B)[edges.Length];
        for (int k = 0; k < edges.Length; k++)
        {
            if (Tool.Curve(edges[k], out double a, out double b) is not { } curve)
            {
                return double.NaN;
            }
            curves[k] = (curve, a, b);
        }
        double h = 2 * reach / cells, sum = 0;
        Point3D o = corner.Point;
        var past = new bool[edges.Length];
        var feet = new (double U, double V)[edges.Length];
        for (int e = 0; e < edges.Length; e++)
        {
            var start = new ProjectPointOnSurface(o, own[e]!.Surface, own[e]!.EU0, own[e]!.EU1, own[e]!.EV0, own[e]!.EV1);
            if (start.NbPoints() == 0)
            {
                return double.NaN;
            }
            start.LowerDistanceParameters(out double u0, out double v0);
            feet[e] = (u0, v0);
        }
        for (int i = 0; i < cells; i++)
        {
            for (int j = 0; j < cells; j++)
            {
                for (int k = 0; k < cells; k++)
                {
                    var x = new Point3D(o.X - reach + ((i + shift) * h), o.Y - reach + ((j + shift) * h), o.Z - reach + ((k + shift) * h));
                    bool any = false;
                    for (int e = 0; e < edges.Length; e++)
                    {
                        past[e] = Beyond(own[e]!, x, ref feet[e]) ?? sideOf(own[e]!, x, true) == false;
                        any |= past[e];
                    }
                    if (!any || !material(x))
                    {
                        continue;
                    }
                    int count = 1;
                    for (int e = 0; e < edges.Length; e++)
                    {
                        if (past[e] && Inside(curves[e], x))
                        {
                            count--;
                        }
                    }
                    sum += count;
                }
            }
        }
        return sum * h * h * h;
    }

    // Past the band's surface, on its edge's side (a convex chamfer's): the foot by Newton's method from the last one, as the
    // grid's points are near each other; null where it doesn't settle
    private static bool? Beyond(Band band, Point3D x, ref (double U, double V) foot)
    {
        (double u, double v) = foot;
        for (int iteration = 0; iteration < 12; iteration++)
        {
            band.Surface.D1(u, v, out Point3D s, out Vector3D su, out Vector3D sv);
            var r = new Vector3D(x, s);
            double gu = r.Dot(su), gv = r.Dot(sv);
            double a = su.Dot(su), b = su.Dot(sv), c = sv.Dot(sv), det = (a * c) - (b * b);
            if (det <= 0)
            {
                return null;
            }
            double du = ((c * gu) - (b * gv)) / det, dv = ((a * gv) - (b * gu)) / det;
            u = Math.Clamp(u - du, band.EU0, band.EU1);
            v = band.Surface.IsVPeriodic ? v - dv : Math.Clamp(v - dv, band.EV0, band.EV1); // round a closed band, on through its seam
            if (Math.Abs(du) * Math.Sqrt(a) + (Math.Abs(dv) * Math.Sqrt(c)) < 1e-12 * (1 + r.Magnitude))
            {
                break;
            }
        }
        band.Surface.D1(u, v, out Point3D foot3, out Vector3D fu, out Vector3D fv);
        Vector3D normal = fu.Crossed(fv);
        var to = new Vector3D(foot3, x);
        if (normal.Magnitude == 0 || Math.Abs(to.Dot(normal)) < 0.99 * to.Magnitude * normal.Magnitude)
        {
            return null; // not square to the surface: at its edge, or not settled
        }
        foot = (u, v);
        return Math.Sign(to.Dot(normal)) == band.EdgeSign;
    }

    // The point's nearest point of the edge is inside it, not at an end: it's between the planes square to the edge there
    private static bool Inside((Curve3D Curve, double A, double B) edge, Point3D x)
    {
        var projection = new ProjectPointOnCurve(x, edge.Curve, edge.A, edge.B);
        if (projection.NbPoints() == 0)
        {
            return false;
        }
        double t = projection.LowerDistanceParameter();
        return t > edge.A + (1e-9 * (edge.B - edge.A)) && t < edge.B - (1e-9 * (edge.B - edge.A));
    }
}
