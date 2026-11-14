// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
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
/// its contacts on the two faces (solved where they lie, in the plane square to the edge: at the chamfer's length from the
/// point, at its two distances, or at its distance on the reference face and where the chord makes its angle with that face's
/// section, as ChFi3d's BlendFunc_ChAsym has it) and the chord between them: the triangle, with the segment between each face's section and its side, by the face's sag at the
/// side's middle (a parabola's two thirds of chord times sag). Swept along the edge, each element of the section moves by
/// one less the edge's curvature vector dotted with its offset, so the volume is the integral of the section's area less its
/// first moment against the curvature, by Gauss's points on panels halved where their halves disagree (unit 15AP: a contact
/// that crosses onto a face carried on, or an edge's knot, puts a kink in the integrand that fixed panels converge on slowly).
/// Where edges run on through a vertex their planes there are one.
/// </para>
/// <para>
/// <b>At a corner,</b> what the chamfers take off together is not the sum of their sections: in a box round the corner,
/// what's in the input and past any chamfer's surface less what's in each section's region, along lines cut exactly where
/// they cross the corner's surfaces (<c>ChamferMeet.Corners.cs</c>). Where every edge at the corner is convex the input is
/// what's behind all of its faces; at a corner of convex and concave edges (unit 15U3) it's what the corner's formula holds
/// without its chamfers, and what's built what it holds with them, in a box inside the reach where the formula decides the
/// fuse's pieces. Only convex chamfers.
/// </para>
/// <para>
/// <b>What's allowed</b> (unit 15AP), each term an error measured or bounded, where it was a part in a million of the whole
/// solid: the two volumes' (each measured at 1e-10, its error what it moved from 1e-8), the sections' (what their panels'
/// halving moved), the corners' (the most two Gauss rules, and Gauss's against the midpoint rule, differ), and the
/// approximation's tolerance over the chamfers' area, which the faces may lie off their exact sections by. If that comes to
/// more than a twentieth of what the chamfers take off, the volume proves too little, and the chamfer isn't built.
/// </para>
/// <para>
/// <b>Refined only until it's decided:</b> first the sections on eight panels and sixteen, and the corners by Gauss's two panels
/// and the midpoint rule on as many lines; the result is proved if it's within that allowance, refused if it's outside the
/// faces' tolerance by more than the errors so far, and only otherwise are the integrals refined (the sections' panels halved
/// where they disagree, the corners' finer Gauss rule) and asked again. Everything is local to the call.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    private static ReadOnlySpan<double> GaussNodes => [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                                       0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];

    private static ReadOnlySpan<double> GaussWeights => [0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                                         0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];

    // Panels of Gauss's eight points along each edge to start with, how many times one may be halved, and how near its halves
    // have to come to it, of itself, first and then refined; and how far a corner's box reaches from the corner, in its
    // widest chamfer's widths
    private const int FirstPanels = 8;

    private const int VolumePanels = 8;

    private const int VolumeHalvings = 4;

    private const double VolumeAgreed = 1e-7;

    private const double CornerReach = 3;

    // The most of what the chamfers take off that the volume's allowance may be, for it to prove anything (unit 15AP)
    internal const double ProvedShare = 0.05;

    // What the volume says of the result: proved, refused (outside what's allowed, or not worked out), or proving too little
    private enum Verdict
    {
        Proved,
        Refused,
        TooLittle,
    }

    /// <summary>The result's volume, <paramref name="measured"/> to <paramref name="measuredError"/>, against the expected,
    /// the integrals refined only until that's decided; with the allowance and what the chamfers take off at the last level asked.</summary>
    private static (Verdict Verdict, double Allowed, double Removed) VolumeJudged(Shape input, Face[] faces, List<Band> bands, List<Corner> corners,
                                                                                  List<Carried> carried, Edge[] chamfered, ChamferSpec[] specs, Face[] referenceFaces,
                                                                                  double tolerance, double near, double measured, double measuredError, double share)
    {
        (double before, double beforeError) = MeasuredVolume(input);
        // A volume that couldn't be confirmed proves nothing (decision 2460)
        if (double.IsNaN(before) || double.IsNaN(beforeError) || double.IsNaN(measured) || double.IsNaN(measuredError))
        {
            return (Verdict.Refused, double.NaN, double.NaN);
        }
        for (int level = 1; level <= 2; level++)
        {
            if (ExpectedVolume(input, faces, bands, corners, carried, chamfered, specs, referenceFaces, near, level) is not { } v)
            {
                return (Verdict.Refused, double.NaN, double.NaN);
            }
            double removed = Math.Abs(v.Removed), licence = tolerance * v.Area, integrals = beforeError + measuredError + v.Error;
            double allowed = licence + integrals, gap = Math.Abs(measured - (before - v.Removed));
            if (allowed <= share * removed && gap <= allowed)
            {
                return (Verdict.Proved, allowed, v.Removed);
            }
            // Past the faces' tolerance by more than every error so far: refining can't bring it in
            if (gap - integrals > licence && allowed <= share * removed)
            {
                return (Verdict.Refused, allowed, v.Removed);
            }
            if (level == 2 || licence + beforeError + measuredError > share * removed)
            {
                return (allowed > share * removed ? Verdict.TooLittle : Verdict.Refused, allowed, v.Removed);
            }
        }
        return (Verdict.Refused, double.NaN, double.NaN);
    }

    // What the chamfers take off, their area and the integrals' error, at a level of refinement, or null where it can't be
    // worked out
    private readonly record struct VolumeTaken(double Removed, double Area, double Error);

    private static VolumeTaken? ExpectedVolume(Shape input, Face[] faces, List<Band> bands, List<Corner> corners, List<Carried> carried,
                                               Edge[] chamfered, ChamferSpec[] specs, Face[] referenceFaces, double near, int level)
    {
        if (bands.Any(static b => !b.Convex))
        {
            return null;
        }
        var context = new IntersectionContext();
        double removed = 0, area = 0, sectionsError = 0;
        foreach (Edge edge in chamfered.DistinctBy(static e => (Shape)e, ShapeHasher.Instance))
        {
            // The edge's cut is its own spec's, or its contour's where the spec names another of its edges
            Edge[] contour = bands.FirstOrDefault(b => b.Edges.Any(e => e.IsSame(edge)))?.Edges ?? [edge];
            if ((CutOf(specs, referenceFaces, [edge]) ?? CutOf(specs, referenceFaces, contour)) is not { } cut
                || Swept(faces, edge, cut.Cut, cut.References, context, level) is not { } swept)
            {
                return null;
            }
            removed += swept.Volume;
            area += swept.Width;
            sectionsError += swept.Error;
        }

        // Each corner: what's past any chamfer less what's in each edge's sections, in a box about it
        double cornersRemoved = 0, cornersError = 0;
        Vertex[] vertices = [.. input.AllVertices().DistinctBy(static v => (Shape)v, ShapeHasher.Instance)];
        foreach (Corner corner in corners)
        {
            Edge[] at = [.. chamfered.Where(e => e.AllVertices().Any(v => v.IsSame(corner.Vertex))).DistinctBy(static e => (Shape)e, ShapeHasher.Instance)];
            // A convex corner's box is three of its widest chamfer's widths at most; one of convex and concave edges has its
            // box inside the reach where its formula says what's built (unit 15U3)
            // On a solid small beside its chamfers that box would reach the next corner's faces, where this corner's say
            // nothing of the input: there the box stays inside the sphere half-way to the nearest other vertex (unit 15U4).
            // Every convex corner's box is drawn round what its chamfers' depths hold, as such a tight one was (unit 15AP)
            double reach = CornerReach * corner.Bands.Max(static b => b.Width);
            double room = vertices.Where(v => !v.IsSame(corner.Vertex)).Select(v => v.Point.DistanceTo(corner.Point)).DefaultIfEmpty(double.MaxValue).Min()
                          / (2 * Math.Sqrt(3));
            (double excess, double error) = corner.Mixed is { } formula
                ? MixedExcess(corner, formula, at, carried, near / Math.Sqrt(3), level)
                : CornerExcess(corner, faces, at, bands, Math.Min(reach, room), level);
            if (double.IsNaN(excess) || double.IsNaN(error))
            {
                return null;
            }
            cornersRemoved += excess;
            cornersError += error;
        }
        return new VolumeTaken(removed + cornersRemoved, area, sectionsError + cornersError);
    }

    // A solid's volume at 1e-10, and its error: what it moved from 1e-8, and the sum's rounding (unit 15AP). Where it was taken
    // at 1e-8 and a part in a million of it allowed, that was 7.4e-4 on the loft, which the integrals achieve to about 1e-9. NaN
    // where either couldn't be confirmed (decision 2460)
    private static (double Volume, double Error) MeasuredVolume(Shape shape)
    {
        double coarse = Kerf.Measurement.MeasureOperations.VolumeOrNaN(shape, new Kerf.Measurement.MassOptions { Tolerance = 1e-8 });
        double fine = Kerf.Measurement.MeasureOperations.VolumeOrNaN(shape, new Kerf.Measurement.MassOptions { Tolerance = 1e-10 });
        return (fine, Math.Abs(fine - coarse) + (1e-13 * Math.Abs(fine)));
    }

    internal static double? SweptVolume(Face[] faces, Edge edge, double distance) =>
        Swept(faces, edge, new Cut(distance, null, null), [], new IntersectionContext(), 2)?.Volume;

    internal static double? SweptVolume(Face[] faces, Edge edge, double distance, double? second, double? angle, Face reference) =>
        Swept(faces, edge, new Cut(distance, second, angle), [reference], new IntersectionContext(), 2)?.Volume;

    // The region the chamfer takes off along the edge, between the planes square to it at its ends, the chamfer's area there,
    // and the quadrature's error: on each panel, what its halves moved it by where they agreed with it. At the first level,
    // eight panels each halved once; refined, halved again while they disagree
    private static (double Volume, double Width, double Error)? Swept(Face[] faces, Edge edge, Cut cut, Face[] references, IntersectionContext context,
                                                                     int level)
    {
        int panels = level == 1 ? FirstPanels : VolumePanels, halvings = level == 1 ? 1 : VolumeHalvings;
        Face[] two = [.. faces.Where(f => OnFace(edge, f))];
        if (two.Length != 2 || Tool.Curve(edge, out double a, out double b) is not { } curve || IsReference(cut, references, two[0]) is not { } firstIsReference
            || (!firstIsReference && IsReference(cut, references, two[1]) != true))
        {
            return null;
        }
        // Each side's contact on its own face, carried on where the section leaves it at a corner, as the corners' integrals
        // count the sections: behind the edge's own two faces. Taken on whichever face held it (unit 15V), a side whose section
        // left its face found a point at its distance on the other side's face, and the section between two points of one
        // face is nothing (unit 15U4)
        var side1 = new Side { Start = two[0], Candidates = [two[0]], InStart = two[0].AllEdges().First(e => e.IsSame(edge)), FromLast = true };
        var side2 = new Side { Start = two[1], Candidates = [two[1]], InStart = two[1].AllEdges().First(e => e.IsSame(edge)), FromLast = true };
        var onto = new Dictionary<Shape, ProjectPointOnSurface>(ShapeHasher.Instance);

        // Gauss's eight points on [lo, hi], in order along the edge
        (double Volume, double Width)? Panel(double lo, double hi)
        {
            double volume = 0, width = 0;
            for (int i = 0; i < GaussNodes.Length; i++)
            {
                double t = (0.5 * (lo + hi)) + (0.5 * (hi - lo) * GaussNodes[i]);
                curve.D2(t, out Point3D c, out Vector3D d1, out Vector3D d2);
                double speed = d1.Magnitude;
                Vector3D tangent = d1 / speed;
                Vector3D bend = (d2 - (tangent * d2.Dot(tangent))) / (speed * speed);
                if (Contacts(cut, firstIsReference ? side1 : side2, firstIsReference ? side2 : side1, t, c, tangent, context) is not { } contacts)
                {
                    return null;
                }
                (Point3D p1, Point3D p2) = firstIsReference ? contacts : (contacts.Other, contacts.Reference);
                (double sectionArea, Vector3D moment) = Section(c, p1, p2, side1.Face, side2.Face, onto);
                double weight = 0.5 * (hi - lo) * GaussWeights[i] * speed;
                volume += weight * (sectionArea - bend.Dot(moment));
                width += weight * p1.DistanceTo(p2);
            }
            return (volume, width);
        }

        // Where each side's last contact was, so that a panel's halves start from where the panel did: each contact is solved
        // from the last one's, and one a panel back may not be found from
        (Point2D? Last, Face Face, Point2D? OtherLast, Face OtherFace) Where() => (side1.Last, side1.Face, side2.Last, side2.Face);
        void From((Point2D? Last, Face Face, Point2D? OtherLast, Face OtherFace) at) =>
            (side1.Last, side1.Face, side2.Last, side2.Face) = at;

        // The panel, given its own value and where its contacts started, by its halves, each halved again while they move it
        // by more than VolumeAgreed of itself
        double total = 0, area = 0, error = 0;
        bool Halved(double lo, double hi, double whole, (Point2D?, Face, Point2D?, Face) start, int halvings)
        {
            double middle = 0.5 * (lo + hi);
            From(start);
            if (Panel(lo, middle) is not { } left)
            {
                return false;
            }
            var half = Where();
            if (Panel(middle, hi) is not { } right)
            {
                return false;
            }
            var end = Where();
            double moved = Math.Abs(left.Volume + right.Volume - whole);
            if (moved <= VolumeAgreed * Math.Abs(whole) || halvings == 0)
            {
                total += left.Volume + right.Volume;
                area += left.Width + right.Width;
                error += moved;
                return true;
            }
            bool both = Halved(lo, middle, left.Volume, start, halvings - 1) && Halved(middle, hi, right.Volume, half, halvings - 1);
            From(end);
            return both;
        }

        double rough = 0;
        for (int panel = 0; panel < panels; panel++)
        {
            double lo = a + ((b - a) * panel / panels), hi = a + ((b - a) * (panel + 1) / panels);
            var start = Where();
            if (Panel(lo, hi) is not { } whole || !Halved(lo, hi, whole.Volume, start, halvings - 1))
            {
                return null;
            }
            rough += whole.Volume;
        }
        // Where a panel is still halved at the last, across a contact that jumps where it crosses onto a face carried on, what
        // the halvings moved it by isn't all its error: then the panels as they were at first say more (on C1, 1.2e-8 where
        // the halvings said 8e-10)
        return (total, area, Math.Max(error, Math.Abs(total - rough)));
    }

    // A section's area and first moment about the edge's point: the triangle, and each face's segment beside it. Each face's
    // sag is by a projector loaded once for the face along the edge (onto), whose grid of samples is built at its first point
    // and not again at each (unit 15AQ): the same values as one made for each point
    private static (double Area, Vector3D Moment) Section(Point3D c, Point3D p1, Point3D p2, Face face1, Face face2, Dictionary<Shape, ProjectPointOnSurface> onto)
    {
        var e1 = new Vector3D(c, p1);
        var e2 = new Vector3D(c, p2);
        double area = 0.5 * e1.Crossed(e2).Magnitude;
        Vector3D moment = (e1 + e2) / 3 * area;
        foreach ((Point3D p, Point3D other, Face face) in new[] { (p1, p2, face1), (p2, p1, face2) })
        {
            Point3D middle = new(0.5 * (c.X + p.X), 0.5 * (c.Y + p.Y), 0.5 * (c.Z + p.Z));
            if (!onto.TryGetValue(face, out ProjectPointOnSurface? projection))
            {
                Surface surface = Tool.Surface(face)!;
                surface.ParameterBounds(out double u0, out double u1, out double v0, out double v1);
                projection = new ProjectPointOnSurface();
                projection.Init(surface, u0, u1, v0, v1, Precision.Confusion());
                onto[face] = projection;
            }
            projection.Perform(middle);
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
