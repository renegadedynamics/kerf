// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// Corners whose edges are both convex and concave (unit 15U2): the input's material near the corner as its faces'
/// half-spaces combine, each chamfer joined to the face it trims, and the corner's other faces carried on past their edges.
/// </summary>
/// <remarks>
/// <para>
/// <b>The formula.</b> Near a corner the input is what's behind its faces, combined as its edges say: two faces whose edges
/// are convex meet as both (what's behind each), two whose edges are concave as either. The faces near the corner are taken
/// two groups at a time, joined where every edge between them is of one kind, until one group is left; an order that leaves
/// two groups whose edges between them are of both kinds is given up for another, and a corner no order joins isn't built.
/// On <c>8_J1</c>'s floor with two walls meeting at a convex edge, the third wall's foot chamfered before, that's the floor
/// or else the first wall and either the second wall or its chamfer.
/// </para>
/// <para>
/// <b>The chamfers.</b> A chamfer joins the face of its two that the formula holds deeper (the wall, not the floor that
/// combines with the walls together), as that face or the chamfer's side for a concave edge, and that face and the chamfer's
/// side for a convex one. So on <c>8_J1</c> the chamfer along the first wall's foot fills only where the second wall or its
/// chamfer is behind it, and ends against them; with two walls chamfered it fills where both do, round their convex corner
/// (decision 2148's floor ∪ ((wall ∪ chamfer) ∩ (wall ∪ chamfer))).
/// </para>
/// <para>
/// <b>The faces carried on.</b> Each face of the formula that isn't one of its chamfers' own two is carried on past its
/// edges, as the chamfers' surfaces are: a plane, or a surface that already runs on past the face by the length (a chamfer
/// ChFi3d made before), as it is; otherwise its surface, each parameter scaled to the face's length along it (unit 15Y), and
/// extended by the length on every side. It goes to the fuse as the face on that surface and a ring of it round the face, so
/// that the pieces of the two that are kept are one face again after the fuse. Only the ring's pieces near a corner of this
/// kind can be kept.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    // A formula of half-spaces: a face's (behind it, as the shell has it), with the chamfers joined to it, or a join of terms
    private abstract class Term;

    private sealed class Leaf(Face face) : Term
    {
        public Face Face { get; } = face;

        public List<Band> Joined { get; } = [];
    }

    private sealed class Join(bool union, Term[] parts) : Term
    {
        public bool Union { get; } = union;

        public Term[] Parts { get; } = parts;
    }

    // A face carried on past its edges: its surface extended, the face on it, and the ring of it round the face
    private sealed record Carried(Face Input, Face Replacement, Face Ring, Surface Surface, double U0, double U1, double V0, double V1);

    // The formula of the faces near a corner of convex and concave edges, with its chamfers joined; null where none fits
    private static Term? MixedFormula(Face[] faces, Vertex corner, double reach, Band[] at)
    {
        if (at.Any(static b => b.Own || b.Face1.IsNull || b.Face2.IsNull))
        {
            return null;
        }
        Face[] cluster = [.. faces.Where(f => f.AllVertices().Any(v => v.Point.DistanceTo(corner.Point) <= reach))];
        int n = cluster.Length;
        if (n < 3 || n > 8)
        {
            return null;
        }
        var kind = new bool?[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                Edge[] shared = [.. cluster[i].AllEdges().Where(e => !Tool.Degenerated(e) && OnFace(e, cluster[j]))];
                if (shared.Length == 0)
                {
                    continue;
                }
                if (shared.Select(e => IsConvex(faces, e)).Distinct().ToArray() is not [{ } convex])
                {
                    return null;
                }
                kind[i, j] = kind[j, i] = convex;
            }
        }
        Leaf[] leaves = [.. cluster.Select(static f => new Leaf(f))];
        if (Joined([.. Enumerable.Range(0, n).Select(i => ((Term)leaves[i], new[] { i }))], kind) is not { } formula)
        {
            return null;
        }

        // Each chamfer joins the deeper of its two faces
        foreach (Band band in at)
        {
            int d1 = Depth(formula, band.Face1, 0), d2 = Depth(formula, band.Face2, 0);
            if (d1 < 0 || d2 < 0 || d1 == d2)
            {
                return null;
            }
            Face deeper = d1 > d2 ? band.Face1 : band.Face2;
            leaves.First(l => l.Face.IsSame(deeper)).Joined.Add(band);
        }
        return formula;
    }

    // The groups joined two at a time where every edge between them is of one kind, until one is left
    private static Term? Joined(List<(Term Term, int[] Members)> groups, bool?[,] kind)
    {
        if (groups.Count == 1)
        {
            return groups[0].Term;
        }
        for (int a = 0; a < groups.Count; a++)
        {
            for (int b = a + 1; b < groups.Count; b++)
            {
                bool[] kinds = [.. (from i in groups[a].Members from j in groups[b].Members where kind[i, j] is not null select kind[i, j]!.Value).Distinct()];
                if (kinds is not [bool convex])
                {
                    continue;
                }
                bool union = !convex;
                var next = new List<(Term Term, int[] Members)>(groups.Count - 1)
                {
                    (new Join(union, [.. Parts(groups[a].Term, union), .. Parts(groups[b].Term, union)]), [.. groups[a].Members, .. groups[b].Members]),
                };
                next.AddRange(groups.Where((_, k) => k != a && k != b));
                if (Joined(next, kind) is { } done)
                {
                    return done;
                }
            }
        }
        return null;
    }

    private static Term[] Parts(Term term, bool union) => term is Join join && join.Union == union ? join.Parts : [term];

    private static int Depth(Term term, Face face, int depth) => term switch
    {
        Leaf leaf => leaf.Face.IsSame(face) ? depth : -1,
        Join join => join.Parts.Select(p => Depth(p, face, depth + 1)).DefaultIfEmpty(-1).Max(),
        _ => -1,
    };

    private static IEnumerable<Leaf> Leaves(Term term) => term is Join join ? join.Parts.SelectMany(Leaves) : [(Leaf)term];

    // Whether the formula holds at x: each face's half-space by behind, each chamfer's side by side (null where it says nothing)
    private static bool Holds(Term term, Point3D x, Func<Face, Point3D, bool> behind, Func<Band, Point3D, bool?> side)
    {
        if (term is Join join)
        {
            return join.Union ? join.Parts.Any(p => Holds(p, x, behind, side)) : join.Parts.All(p => Holds(p, x, behind, side));
        }
        var leaf = (Leaf)term;
        bool held = behind(leaf.Face, x);
        foreach (Band band in leaf.Joined)
        {
            if (side(band, x) is { } s)
            {
                held = band.Convex ? held && s : held || s;
            }
        }
        return held;
    }

    // The face carried on past its edges by length: a plane, or a surface that runs on past it that far, as it is; a Bézier or
    // B-spline surface scaled and extended on every side; null where it can't be (a periodic surface, a seam, a placed
    // surface, more than one wire)
    private static Carried? Carry(Face face, double length)
    {
        Face forward = ShapeCasts.Face(face.Oriented(Orientation.Forward));
        Wire[] wires = [.. forward.Explore<Wire>(ShapeType.Wire)];
        if (wires.Length != 1 || Tool.Surface(forward, out Placement placed) is not { } surface || !placed.IsIdentity)
        {
            return null;
        }
        BrepTools.UVBounds(forward, out double fu0, out double fu1, out double fv0, out double fv1);
        var builder = new BrepBuilder();
        if (surface is PlaneSurface)
        {
            double u0 = fu0 - length, u1 = fu1 + length, v0 = fv0 - length, v1 = fv1 + length;
            Face planeRing = Polygon(surface, [new(u0, v0), new(u1, v0), new(u1, v1), new(u0, v1)]);
            builder.Add(planeRing, wires[0].Reversed());
            return new Carried(face, face, planeRing, surface, u0, u1, v0, v1);
        }
        // A surface that already runs on past the face by the length on every side (a chamfer ChFi3d made before, which it
        // extended and trimmed) is carried on as it is: extended again, a stripe's parameter across, by its length along
        // the edge, makes curves the fuse can't follow (unit 15Y)
        surface.ParameterBounds(out double su0, out double su1, out double sv0, out double sv1);
        double mu = (fu0 + fu1) / 2, mv = (fv0 + fv1) / 2;
        if (!surface.IsUPeriodic && !surface.IsVPeriodic && surface is BoundedSurface
            && surface.EvalD0(fu0, mv).DistanceTo(surface.EvalD0(su0, mv)) >= length && surface.EvalD0(fu1, mv).DistanceTo(surface.EvalD0(su1, mv)) >= length
            && surface.EvalD0(mu, fv0).DistanceTo(surface.EvalD0(mu, sv0)) >= length && surface.EvalD0(mu, fv1).DistanceTo(surface.EvalD0(mu, sv1)) >= length)
        {
            Face ownRing = Polygon(surface, [new(su0, sv0), new(su1, sv0), new(su1, sv1), new(su0, sv1)]);
            builder.Add(ownRing, wires[0].Reversed());
            return new Carried(face, face, ownRing, surface, su0, su1, sv0, sv1);
        }
        BSplineSurface? spline = surface switch
        {
            BSplineSurface b => b,
            BezierSurface => GeomConvert.SurfaceToBSplineSurface(surface),
            _ => null,
        };
        if (spline is null || spline.IsUPeriodic || spline.IsVPeriodic)
        {
            return null;
        }
        // Each parameter by the face's length along it first, so that the extension, which adds its length in parameter, runs
        // as fast as the face: a chamfer ChFi3d made before has its parameter across by its length along the edge, and
        // carried on across as it was, it made curves the fuse couldn't follow (unit 15Y's slivers)
        double su = Speed(surface.EvalD0(fu0, mv), surface.EvalD0(fu1, mv), fu1 - fu0), sv = Speed(surface.EvalD0(mu, fv0), surface.EvalD0(mu, fv1), fv1 - fv0);
        spline = new BSplineSurface(spline.Poles, [.. spline.UKnots.Select(k => k * su)], [.. spline.VKnots.Select(k => k * sv)],
                                    [.. spline.UMultiplicities], [.. spline.VMultiplicities], spline.UDegree, spline.VDegree,
                                    false, false, spline.IsURational || spline.IsVRational ? spline.Weights : default);
        BoundedSurface extended = spline;
        foreach ((bool inU, bool after) in (ReadOnlySpan<(bool, bool)>)[(true, true), (true, false), (false, true), (false, false)])
        {
            GeomLib.ExtendSurfByLength(ref extended, length, 1, inU, after);
        }
        if (extended is not BSplineSurface carried || carried.EvalD0(mu * su, mv * sv).DistanceTo(surface.EvalD0(mu, mv)) > Precision.Confusion())
        {
            return null; // the extension moved the parameters
        }

        // The face again on the extended surface: a copy, its edges given their curves on it, the same as on the face's own
        var copy = ShapeCasts.Face(new Copy(forward, false).InternalModifiedShape(forward));
        var curves = new List<(Edge Edge, Curve2D Curve)>();
        foreach (Edge edge in copy.Explore<Edge>(ShapeType.Edge))
        {
            if (Tool.IsClosed(edge, copy) || Tool.CurveOnSurface(edge, copy, out double first, out double last) is not { } curve)
            {
                return null;
            }
            // The edge's curve on the face, its parameters scaled as the surface's
            Curve2D scaled = Scaled(curve, first, last, su, sv);
            foreach (double t in (ReadOnlySpan<double>)[first, (first + last) / 2, last])
            {
                Point2D was = curve.EvalD0(t), now = scaled.EvalD0(t);
                if (carried.EvalD0(now.X, now.Y).DistanceTo(surface.EvalD0(was.X, was.Y)) > Precision.Confusion())
                {
                    return null;
                }
            }
            curves.Add((edge, scaled));
        }
        builder.UpdateFace(copy, carried, new Placement(), Tool.Tolerance(copy));
        foreach ((Edge edge, Curve2D curve) in curves)
        {
            builder.UpdateEdge(edge, curve, copy, Tool.Tolerance(edge));
        }
        carried.ParameterBounds(out double cu0, out double cu1, out double cv0, out double cv1);
        Face ring = Polygon(carried, [new(cu0, cv0), new(cu1, cv0), new(cu1, cv1), new(cu0, cv1)]);
        builder.Add(ring, copy.Explore<Wire>(ShapeType.Wire).Single().Reversed());
        return new Carried(face, ShapeCasts.Face(copy.Oriented(face.Orientation)), ring, carried, cu0, cu1, cv0, cv1);
    }

    // The curve on [first, last] with its points' coordinates scaled, its parameter as it was: a line as a B-spline of degree
    // 1, a B-spline or a Bézier curve by its poles, a trimmed curve by its basis
    private static BSplineCurve2D Scaled(Curve2D curve, double first, double last, double su, double sv)
    {
        BSplineCurve2D spline = curve switch
        {
            TrimmedCurve2D trimmed => Scaled(trimmed.BasisCurve, first, last, 1, 1),
            LineCurve2D => new BSplineCurve2D([curve.EvalD0(first), curve.EvalD0(last)], [first, last], [2, 2], 1),
            BSplineCurve2D { IsPeriodic: false } b => b,
            BezierCurve2D z => new BSplineCurve2D([.. z.Poles], [0, 1], [z.Degree + 1, z.Degree + 1], z.Degree, false, z.IsRational ? [.. z.Weights] : default),
            _ => Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(curve, first, last)),
        };
        return new BSplineCurve2D([.. spline.Poles.Select(q => new Point2D(q.X * su, q.Y * sv))], [.. spline.Knots], [.. spline.Multiplicities],
                                  spline.Degree, false, spline.IsRational ? [.. spline.Weights] : default);
    }

    // A parameter's speed over a face: the chord across it over its span, or 1 where it's too short to say
    private static double Speed(Point3D a, Point3D b, double span) =>
        span > Precision.PConfusion() && a.DistanceTo(b) > Precision.Confusion() ? a.DistanceTo(b) / span : 1;

    // Behind a face's surface, carried on where it is, as the shell orients the face: on the material's side of it
    private static bool Behind(Face oriented, Surface surface, double u0, double u1, double v0, double v1, Point3D x, int material)
    {
        var projection = double.IsInfinity(u0) ? new ProjectPointOnSurface(x, surface) : new ProjectPointOnSurface(x, surface, u0, u1, v0, v1);
        if (projection.NbPoints() == 0)
        {
            return false;
        }
        projection.LowerDistanceParameters(out double u, out double v);
        Vector3D normal = Normal(surface, u, v);
        if (oriented.Orientation == Orientation.Reversed)
        {
            normal = -normal;
        }
        return Math.Sign(new Vector3D(surface.EvalD0(u, v), x).Dot(normal)) == material;
    }
}
