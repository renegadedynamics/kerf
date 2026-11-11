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
/// what's in the input and past any chamfer's surface less what's in each section's region, along lines cut exactly where
/// they cross the corner's surfaces (<c>ChamferMeet.Corners.cs</c>). Only corners where every edge is convex.
/// </para>
/// <para>
/// The result's volume, measured, has to match to a part in a million of it, the approximation's tolerance over the chamfer's
/// area, and twice the corners' difference between two quadratures. Everything is local to the call.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    private static ReadOnlySpan<double> GaussNodes => [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                                       0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];

    private static ReadOnlySpan<double> GaussWeights => [0.1012285362903763, 0.2223810344533745, 0.3137066661925477, 0.3626837833783620,
                                                         0.3626837833783620, 0.3137066661925477, 0.2223810344533745, 0.1012285362903763];

    // Panels of Gauss's eight points along each edge, and how far a corner's box reaches from the corner, in its widest
    // chamfer's widths
    private const int VolumePanels = 16;

    private const double CornerReach = 3;

    /// <summary>The expected volume and how far from it the result may be, or null where it can't be worked out.</summary>
    private static (double Expected, double Allowed)? ExpectedVolume(Shape input, Face[] faces, List<Band> bands, List<Corner> corners,
                                                                     Edge[] chamfered, ChamferSpec[] specs, double tolerance)
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
            (double excess, double error) = CornerExcess(corner, faces, at, bands, reach);
            if (double.IsNaN(excess) || double.IsNaN(error))
            {
                return null;
            }
            cornersRemoved += excess;
            cornersError += error;
        }
        double expected = MeasuredVolume(input) - removed - cornersRemoved;
        return (expected, (1e-6 * Math.Abs(expected)) + (tolerance * area) + cornersError);
    }

    private static double MeasuredVolume(Shape shape) => Kerf.Measurement.MeasureOperations.IntegrateVolume(shape, new Kerf.Measurement.MassOptions { Tolerance = 1e-8 });

    internal static double? SweptVolume(Face[] faces, Edge edge, double distance) => Swept(faces, edge, distance, new IntersectionContext())?.Volume;

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
}
