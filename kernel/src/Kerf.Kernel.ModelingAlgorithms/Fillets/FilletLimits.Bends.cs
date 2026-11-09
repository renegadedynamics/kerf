// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// How long a chamfer can be where its edge bends in a plane beside it (decision 1985; fillet-design.md, section 8). On a plane, a
/// chamfer's contact is the edge stepped in square to itself by the length measured on that face. Along a circle of radius r, with the
/// face on the centre's side, that's the concentric circle of radius r − d: at d = r it closes to the centre, and past it it comes out
/// on the far side, running backwards, so every ruling of the chamfer crosses the circle's axis at one point and the face crosses
/// itself. An ellipse's stepped curve has a cusp where d is the radius of curvature, and past its tightest, b²/a, the curve
/// doubles back on itself, so the face folds over. Either way nothing valid is there to build. On the far side of the centre
/// (a hole's rim, a concave arc) the stepped curve is r + d, which has no such limit.
/// </summary>
internal static partial class FilletLimits
{
    /// <summary>One edge of a chamfer's contour, on a plane where it bends with the face on its inside.</summary>
    /// <param name="Edge">The edge.</param>
    /// <param name="Radius">Its tightest radius of curvature: the circle's radius, or an ellipse's b²/a if that point is on the edge.</param>
    /// <param name="Asked">The chamfer's length measured on that plane.</param>
    /// <param name="Tolerance">How closely the shape knows that: the largest of the face's, the edge's and its vertices'.</param>
    /// <param name="IsCircle">A circle, rather than an ellipse.</param>
    public readonly record struct Bend(Edge Edge, double Radius, double Asked, double Tolerance, bool IsCircle)
    {
        /// <summary>Past the radius by more than the tolerance: proved not to fit.</summary>
        public bool Above => Asked > Radius + Tolerance;

        /// <summary>Within the tolerance of the radius, either way: the stepped curve closes to a point.</summary>
        public bool AtLimit => !Above && Asked >= Radius - Tolerance;
    }

    /// <summary>The bends in the contours <paramref name="chamfer"/> has found for these specs, edges it carries on to along a tangent
    /// chain included; empty where the mode isn't classic, as a throat isn't measured along the face.</summary>
    public static IReadOnlyList<Bend> Bends(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options)
    {
        if (options.Mode != ChamferMode.Classic)
        {
            return [];
        }
        try
        {
            return FindBends(shape, chamfer, specs, referenceFaces);
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            // A shape this can't read has no limit it knows of: the algorithm's turn
            return [];
        }
    }

    private static List<Bend> FindBends(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces)
    {
        var bends = new List<Bend>();
        Neighbours? neighbours = null;
        int contours = chamfer.ContourCount;

        // Only edges that bend can say anything, and a chamfer of straight edges shouldn't pay for the map of faces
        var chains = new Edge[contours + 1][];
        var bending = new bool[contours + 1];
        var contourOf = new Dictionary<Shape, int>(ShapeHasher.Instance);
        for (int ic = 1; ic <= contours; ic++)
        {
            int count = chamfer.NbEdges(ic);
            chains[ic] = new Edge[count];
            for (int j = 0; j < count; j++)
            {
                Edge edge = chains[ic][j] = chamfer.Edge(ic, j + 1);
                contourOf.TryAdd(edge, ic);
                bending[ic] |= new EdgeCurveAdaptor(edge).GetType() is CurveKind.Circle or CurveKind.Ellipse;
            }
        }

        // A contour's lengths are those of the first spec added on it, as ChFi3d skips an edge it already has
        var firstSpec = new int[contours + 1];
        Array.Fill(firstSpec, -1);
        for (int i = 0; i < specs.Length; i++)
        {
            if (contourOf.TryGetValue(specs[i].Edge, out int ic) && firstSpec[ic] < 0)
            {
                firstSpec[ic] = i;
            }
        }
        for (int ic = 1; ic <= contours; ic++)
        {
            int first = firstSpec[ic];
            if (!bending[ic] || first < 0)
            {
                continue;
            }
            Cancellation.ThrowIfRequested();
            Edge[] edges = chains[ic];
            int count = edges.Length;
            neighbours ??= new Neighbours(shape);
            ChamferSpec spec = specs[first];
            int start = Array.FindIndex(edges, e => e.IsSame(spec.Edge));
            IReadOnlyList<Face> faces = neighbours.Faces(spec.Edge);
            if (start < 0 || faces.Count != 2)
            {
                continue;
            }
            Face reference = referenceFaces[first];
            Face other = faces[0].IsSame(reference) ? faces[1] : faces[0];

            // Each edge's faces, the reference's side first, carried along the chain by the face two edges share
            var sides = new (Face Reference, Face Other)?[count];
            sides[start] = (reference, other);
            foreach (int step in new[] { 1, -1 })
            {
                (Face Reference, Face Other) last = (reference, other);
                for (int j = start + step; j >= 0 && j < count; j += step)
                {
                    IReadOnlyList<Face> on = neighbours.Faces(edges[j]);
                    if (on.Count != 2 || Carried(last, on[0], on[1]) is not { } carried)
                    {
                        break;
                    }
                    sides[j] = last = carried;
                }
            }
            for (int j = 0; j < count; j++)
            {
                if (sides[j] is not { } side)
                {
                    continue;
                }
                foreach ((Face plane, Face beside, bool isReference) in new[] { (side.Reference, side.Other, true), (side.Other, side.Reference, false) })
                {
                    if (LengthOn(neighbours, spec, plane, beside, isReference) is { } asked && Bending(neighbours, edges[j], plane) is { } bend)
                    {
                        bends.Add(new Bend(edges[j], bend.Radius, asked, bend.Tolerance, bend.IsCircle));
                    }
                }
            }
        }
        return bends;
    }

    // The next edge's faces as the last's sides: by the face they share
    private static (Face Reference, Face Other)? Carried((Face Reference, Face Other) last, Face one, Face two)
    {
        if (one.IsSame(last.Reference) || two.IsSame(last.Other))
        {
            return (one, two);
        }
        if (two.IsSame(last.Reference) || one.IsSame(last.Other))
        {
            return (two, one);
        }
        return null;
    }

    // The length measured on the face: the one length, either of two, or for a length and an angle, the length on its face, and on
    // the other where the faces are square, as a cylinder about the plane's normal is, where it's the length times the angle's tangent
    private static double? LengthOn(Neighbours neighbours, in ChamferSpec spec, Face face, Face beside, bool isReference)
    {
        if (spec.Angle is { } angle)
        {
            if (isReference)
            {
                return spec.Distance;
            }
            if (neighbours.Normal(face) is { } normal && angle > 0 && angle < Math.PI / 2 && IsSquareCylinder(beside, normal))
            {
                return spec.Distance * Math.Tan(angle);
            }
            return null;
        }
        return isReference ? spec.Distance : spec.Distance2 ?? spec.Distance;
    }

    private static bool IsSquareCylinder(Face face, Direction3D normal)
    {
        var surface = new FaceSurfaceAdaptor(face);
        return surface.GetType() == SurfaceKind.Cylinder && Math.Abs(surface.Cylinder().Position.Direction.Dot(normal)) >= 1 - 1e-9;
    }

    // The edge's tightest radius of curvature in the plane, if it's a circle or an ellipse in it with the face on the inside
    private static (double Radius, double Tolerance, bool IsCircle)? Bending(Neighbours neighbours, Edge edge, Face face)
    {
        if (Tool.Degenerated(edge) || neighbours.Normal(face) is not { } normal)
        {
            return null;
        }
        var curve = new EdgeCurveAdaptor(edge);
        CurveKind kind = curve.GetType();
        double first = curve.FirstParameter(), last = curve.LastParameter();
        Point3D centre;
        Direction3D axis;
        double radius;
        if (kind == CurveKind.Circle)
        {
            Circle3D circle = curve.Circle();
            (centre, axis, radius) = (circle.Location, circle.Axis.Direction, circle.Radius);
        }
        else if (kind == CurveKind.Ellipse)
        {
            Ellipse3D ellipse = curve.Ellipse();
            (centre, axis) = (ellipse.Location, ellipse.Axis.Direction);
            radius = Tightest(ellipse.MajorRadius, ellipse.MinorRadius, first, last);
        }
        else
        {
            return null;
        }
        if (Math.Abs(axis.Dot(normal)) < 1 - 1e-9)
        {
            return null;
        }

        // The face lies to the left of its oriented edge, seen from outside: is that the centre's side?
        curve.D1((first + last) / 2, out Point3D at, out Vector3D d1);
        if (d1.Magnitude <= Tolerances.Confusion)
        {
            return null;
        }
        Direction3D tangent = new(d1);
        if (neighbours.Oriented(face, edge).Orientation == Orientation.Reversed)
        {
            tangent = tangent.Reversed();
        }
        Direction3D inward = normal.Crossed(tangent);
        if (new Vector3D(inward).Dot(new Vector3D(at, centre)) <= 0)
        {
            return null;
        }
        double tolerance = Math.Max(Tolerance, Math.Max(Tool.Tolerance(face), Tool.Tolerance(edge)));
        foreach (Vertex vertex in edge.AllVertices())
        {
            tolerance = Math.Max(tolerance, Tool.Tolerance(vertex));
        }
        return (radius, tolerance, kind == CurveKind.Circle);
    }

    /// <summary>The least radius of curvature of an ellipse over parameters [first, last]: b²/a at the ends of the major axis, at
    /// t = kπ, and otherwise at an end, as between two of those it rises to a²/b at t = kπ + π/2 and falls again.</summary>
    internal static double Tightest(double a, double b, double first, double last)
    {
        static double At(double a, double b, double t)
        {
            double s = Math.Sin(t), c = Math.Cos(t);
            return Math.Pow(a * a * s * s + b * b * c * c, 1.5) / (a * b);
        }
        if (Math.Ceiling(first / Math.PI) * Math.PI <= last)
        {
            return b * b / a;
        }
        return Math.Min(At(a, b, first), At(a, b, last));
    }

    /// <summary>The error for a chamfer too long for a bend beside its edges, or null if none is past one.</summary>
    public static Diagnostic? TooTight(IReadOnlyList<Bend> bends, ChamferSpec[] specs) =>
        BendMessage(bends.Where(static b => b.Above).ToList(), specs, atLimit: false);

    /// <summary>The error for a chamfer that closes its edge on a face to a point at a bend, where ChFi3d hasn't built it, or null.</summary>
    public static Diagnostic? AtBend(IReadOnlyList<Bend> bends, ChamferSpec[] specs) =>
        BendMessage(bends.Where(static b => b.AtLimit).ToList(), specs, atLimit: true);

    private static Diagnostic? BendMessage(List<Bend> bends, ChamferSpec[] specs, bool atLimit)
    {
        if (bends.Count == 0)
        {
            return null;
        }
        Bend tightest = bends.MinBy(static b => b.Radius - b.Asked);
        Edge[] named = [.. specs.Select(s => s.Edge)];
        string them = named.Length == 1 ? "it" : "them";
        string what = tightest.IsCircle ? $"the radius of the tightest convex arc in a face beside {them}" : $"the tightest radius of curvature of a convex curve in a face beside {them}";
        Shape[] shapes = [.. bends.Select(static b => b.Edge).DistinctBy(static e => e, ShapeHasher.Instance).Select(static e => (Shape)e)];
        if (atLimit)
        {
            return Diagnostic.Error(FilletOperations.TooLarge,
                $"The chamfer with a length of {tightest.Asked.ToString("G6", CultureInfo.CurrentCulture)} brings its edge on a face beside {Edges(named)} to a point, as that's {what}, and this case can't be built yet: try a slightly smaller length.")
                .WithShapes(shapes);
        }

        // To twelve figures, as for a fillet: given back, what this says fits is within tolerance of the limit
        double limit = Math.Round(tightest.Radius, 9);
        string most = limit.ToString("G12", CultureInfo.CurrentCulture);
        string asked = tightest.Asked.ToString("G6", CultureInfo.CurrentCulture);
        asked = asked == limit.ToString("G6", CultureInfo.CurrentCulture) ? tightest.Asked.ToString("G15", CultureInfo.CurrentCulture) : asked;
        bool oneLength = specs.All(s => s.Angle is null && (s.Distance2 is null || s.Distance2 == s.Distance)) && specs.All(s => s.Distance == specs[0].Distance);
        string length = oneLength ? $"with a length of {asked}" : $"with a length of {asked} on a face beside {them}";
        return Diagnostic.Error(FilletOperations.TooLarge,
            $"The chamfer {length} is too large for {Edges(named)}: the most that fits is {most}, {what}, where its edge on that face {(tightest.IsCircle ? "closes to a point" : "comes to a cusp")}.")
            .WithShapes(shapes);
    }
}
