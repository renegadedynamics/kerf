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
/// edges, as the chamfers' surfaces are: its surface extended by the same length on every side, its parameters kept, and
/// given to the fuse as the face itself on that surface and a ring of it round the face, so that the pieces of the two that
/// are kept are one face again after the fuse. Only the ring's pieces near a corner of this kind can be kept.
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

    // The face carried on past its edges by length: a plane as it is, a Bézier or B-spline surface extended on every side,
    // its parameters kept; null where it can't be (a periodic surface, a seam, a placed surface, more than one wire)
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
        BoundedSurface extended = spline;
        foreach ((bool inU, bool after) in (ReadOnlySpan<(bool, bool)>)[(true, true), (true, false), (false, true), (false, false)])
        {
            GeomLib.ExtendSurfByLength(ref extended, length, 1, inU, after);
        }
        double mu = (fu0 + fu1) / 2, mv = (fv0 + fv1) / 2;
        if (extended is not BSplineSurface carried || carried.EvalD0(mu, mv).DistanceTo(surface.EvalD0(mu, mv)) > Precision.Confusion())
        {
            return null; // the extension moved the parameters
        }

        // The face again on the extended surface: a copy, its edges given their curves on it, the same as on the face's own
        var copy = ShapeCasts.Face(new Copy(forward, false).InternalModifiedShape(forward));
        var curves = new List<(Edge Edge, Curve2D Curve)>();
        foreach (Edge edge in copy.Explore<Edge>(ShapeType.Edge))
        {
            if (Tool.IsClosed(edge, copy) || Tool.CurveOnSurface(edge, copy, out _, out _) is not { } curve)
            {
                return null;
            }
            curves.Add((edge, curve));
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
