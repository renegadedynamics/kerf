// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// Tight corners that are B-splines, ellipses or Béziers rather than circular arcs (unit 13I; docs/chamfer-corners.md): a corner swallowed
/// whole by the collapse, every point of it at least the chamfer's length inside the point P its neighbours' insets meet at, is the cone from P
/// over the curve, exactly a B-spline surface of the curve's own poles and weights in one row and P in the other. A curve that isn't
/// swallowed whole would need its inset, which isn't such a curve, so it's refused.
/// </summary>
internal static partial class FilletLimits
{
    // Samples of a curve between its knots, for the searches over it
    private const int CurveSamples = 32;

    /// <summary>The curve kinds read as a curved edge of the loop rather than a line or arc.</summary>
    private static bool IsFreeCurve(CurveKind kind) => kind is CurveKind.BSplineCurve or CurveKind.BezierCurve or CurveKind.Ellipse;

    /// <summary>The edge's curve as a B-spline, exactly (a conic's rational form), in place, running the way <paramref name="oriented"/>
    /// runs it.</summary>
    private static BSplineCurve3D EdgeSpline(Edge oriented)
    {
        Curve3D curve = (Curve3D)Tool.Curve(oriented, out Placement placement, out double first, out double last)!.Transformed(placement.Transformation);
        BSplineCurve3D spline = GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(curve, first, last));
        return oriented.Orientation == Orientation.Reversed ? (BSplineCurve3D)spline.Reversed() : spline;
    }

    // The distinct knots of a spline, its spans' ends
    private static double[] Spans(BSplineCurve3D spline) => [.. spline.Knots.Where(k => k >= spline.FirstParameter - 1e-12 && k <= spline.LastParameter + 1e-12)];

    // A point of the curve on F and its first two derivatives, flat
    private static (Flat P, Flat D1, Flat D2) Flatten(in FacePlane frame, BSplineCurve3D spline, double u)
    {
        spline.D2(u, out Point3D p, out Vector3D d1, out Vector3D d2);
        Vector3D x = new(frame.X), y = new(frame.Y);
        return (frame.Flatten(p), new Flat(d1.Dot(x), d1.Dot(y)), new Flat(d2.Dot(x), d2.Dot(y)));
    }

    /// <summary>∫ f(u) du over the spline, by Gauss-Legendre on each knot span in two panels.</summary>
    private static double OverSpline(BSplineCurve3D spline, Func<double, double> f)
    {
        double[] spans = Spans(spline);
        double sum = 0;
        for (int i = 0; i + 1 < spans.Length; i++)
        {
            for (int panel = 0; panel < 2; panel++)
            {
                double from = spans[i] + (spans[i + 1] - spans[i]) * panel / 2, half = (spans[i + 1] - spans[i]) / 4, middle = from + half;
                for (int g = 0; g < GaussNodes.Length; g++)
                {
                    sum += GaussWeights[g] * half * (f(middle + GaussNodes[g] * half) + f(middle - GaussNodes[g] * half));
                }
            }
        }
        return sum;
    }

    // Parameters along the spline: each span in equal steps, its ends included
    private static IEnumerable<double> Samples(BSplineCurve3D spline)
    {
        double[] spans = Spans(spline);
        for (int i = 0; i + 1 < spans.Length; i++)
        {
            for (int j = 0; j < CurveSamples; j++)
            {
                yield return spans[i] + (spans[i + 1] - spans[i]) * j / CurveSamples;
            }
        }
        yield return spans[^1];
    }

    /// <summary>A curved edge of the loop, its curve on F as a B-spline running the way the loop does; or null, saying why, if its side isn't
    /// swept straight down from it.</summary>
    private static Piece? ReadCurve(Neighbours neighbours, Face face, in FacePlane frame, Edge edge, Face side, out string? why)
    {
        why = null;
        var surface = new FaceSurfaceAdaptor(side);
        if (surface.GetType() != SurfaceKind.SurfaceOfExtrusion || Math.Abs(Math.Abs(surface.Direction().Dot(frame.Up)) - 1) > 1e-9)
        {
            why = "a face beside a curved edge isn't swept square to the face";
            return null;
        }
        BSplineCurve3D spline = EdgeSpline(neighbours.Oriented(face, edge));
        FacePlane at = frame;
        if (spline.Poles.Any(p => Math.Abs(at.Depth(p)) > TightTolerance(edge, face)))
        {
            why = "a curved edge of the loop doesn't lie in the face's plane";
            return null;
        }
        (Flat start, Flat startD1, _) = Flatten(frame, spline, spline.FirstParameter);
        (Flat end, Flat endD1, _) = Flatten(frame, spline, spline.LastParameter);
        double length = OverSpline(spline, u => Flatten(at, spline, u).D1.Length);
        double twice = OverSpline(spline, u =>
        {
            (Flat p, Flat d1, _) = Flatten(at, spline, u);
            return p.Cross(d1);
        });
        return new Piece(edge, side, start, end, false, default, 0, 0, 0)
        {
            Spline = spline,
            CurveStartTangent = startD1 * (1 / startD1.Length),
            CurveEndTangent = endD1 * (1 / endD1.Length),
            CurveLength = length,
            CurveTwice = twice,
            CurveTightest = Tightest(frame, spline) ?? double.PositiveInfinity,
        };
    }

    /// <summary>The tightest radius of curvature of a curved edge where it bends round the material of a planar face, by samples; null
    /// where it never does.</summary>
    private static double? TightestConvex(Neighbours neighbours, Face face, Edge edge)
    {
        if (neighbours.Normal(face) is not { } up)
        {
            return null;
        }
        Plane3D plane = new FaceSurfaceAdaptor(face).Plane();
        var frame = new FacePlane(plane.Location, plane.Position.XDirection, up.Crossed(plane.Position.XDirection), up);
        return Tightest(frame, EdgeSpline(neighbours.Oriented(face, edge)));
    }

    private static double? Tightest(in FacePlane frame, BSplineCurve3D spline)
    {
        double most = 0;
        foreach (double u in Samples(spline))
        {
            (_, Flat d1, Flat d2) = Flatten(frame, spline, u);
            double speed = d1.Length;
            if (speed > 1e-12)
            {
                // Anticlockwise about the outward normal, the material on the left: a left turn bends round it
                most = Math.Max(most, d1.Cross(d2) / (speed * speed * speed));
            }
        }
        return most > 0 ? 1 / most : null;
    }

    /// <summary>Whether a curved piece is swallowed whole at P: it turns one way, round the material, and every point of it is the length
    /// on F or more from P, measured inwards square to its tangent, so P is inside each of its tangent lines stepped in, and E has its
    /// normals only at P (docs/chamfer-corners.md).</summary>
    private static bool SwallowedWhole(in FacePlane frame, Piece piece, Flat apex, double a, double join)
    {
        BSplineCurve3D spline = piece.Spline!;
        foreach (double u in Samples(spline))
        {
            (Flat p, Flat d1, Flat d2) = Flatten(frame, spline, u);
            double speed = d1.Length;
            if (speed < 1e-12 || d1.Cross(d2) < -1e-9 * speed * speed * speed)
            {
                return false;
            }
            Flat inward = d1.Left * (1 / speed);
            if ((apex - p).Dot(inward) < a - join)
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>The section of a swallowed curve at λ: its curve moved λ of the way to P, so its part of twice the area is
    /// (1 - λ)² ∫ γ × γ' + λ (1 - λ) P × (end - start).</summary>
    private static double CurveSection(Piece piece, double lambda) =>
        (1 - lambda) * (1 - lambda) * piece.CurveTwice + lambda * (1 - lambda) * piece.Apex.Cross(piece.End - piece.Start);

    /// <summary>The cone from P to a swallowed curve b deep: half the integral of |γ' × (P - γ, b)|.</summary>
    private static double CurveConeArea(in FacePlane frame, Piece piece, double b)
    {
        BSplineCurve3D spline = piece.Spline!;
        FacePlane at = frame;
        Flat apex = piece.Apex;
        return OverSpline(spline, u =>
        {
            (Flat p, Flat d1, _) = Flatten(at, spline, u);
            double across = d1.Cross(apex - p);
            return Math.Sqrt(b * b * d1.Dot(d1) + across * across);
        }) / 2;
    }

    // How far a point is from a swallowed curve's section at its depth: the nearest sample, then Newton's steps along the curve
    private static double OffCurveChamfer(in FacePlane frame, Piece piece, Flat q, double lambda)
    {
        BSplineCurve3D spline = piece.Spline!;
        Flat apex = piece.Apex;
        FacePlane at = frame;
        (Flat P, Flat D1, Flat D2) At(double u)
        {
            (Flat p, Flat d1, Flat d2) = Flatten(at, spline, u);
            return (Flat.Mix(apex, p, 1 - lambda), d1 * (1 - lambda), d2 * (1 - lambda));
        }
        double best = spline.FirstParameter, nearest = double.MaxValue;
        foreach (double u in Samples(spline))
        {
            double distance = (At(u).P - q).Length;
            if (distance < nearest)
            {
                (nearest, best) = (distance, u);
            }
        }
        for (int step = 0; step < 8; step++)
        {
            (Flat p, Flat d1, Flat d2) = At(best);
            Flat off = p - q;
            double slope = off.Dot(d1), curve = d1.Dot(d1) + off.Dot(d2);
            if (Math.Abs(curve) < 1e-300)
            {
                break;
            }
            best = Math.Clamp(best - slope / curve, spline.FirstParameter, spline.LastParameter);
        }
        return Math.Min(nearest, (At(best).P - q).Length);
    }

    /// <summary>A swallowed curve's part of the tool: the cone from P carried down to <paramref name="deep"/>, the wall straight up from its
    /// lower edge, and the top in F's plane from P out to the wall.</summary>
    private static void CurveTool(Sewing sewing, in FacePlane frame, Piece piece, double deep, double k)
    {
        BSplineCurve3D spline = piece.Spline!;
        Flat apex = piece.Apex;
        int count = spline.PoleCount;
        IReadOnlyList<Point3D> poles = spline.Poles;
        IReadOnlyList<double> weights = spline.Weights;
        var flat = new Flat[count];
        for (int i = 0; i < count; i++)
        {
            flat[i] = Flat.Mix(apex, frame.Flatten(poles[i]), k);
        }
        BSplineSurface Rows(Func<int, Point3D> first, Func<int, Point3D> second)
        {
            var grid = new Point3D[count * 2];
            var w = new double[count * 2];
            for (int i = 0; i < count; i++)
            {
                grid[i * 2] = first(i);
                grid[i * 2 + 1] = second(i);
                w[i * 2] = w[i * 2 + 1] = weights[i];
            }
            return new BSplineSurface(new Grid<Point3D>(grid, count, 2), [.. spline.Knots], [0.0, 1.0], [.. spline.Multiplicities], [2, 2], spline.Degree, 1,
                                      weights: spline.IsRational ? new Grid<double>(w, count, 2) : default);
        }
        FacePlane at = frame;
        double u0 = spline.FirstParameter, u1 = spline.LastParameter;
        sewing.Add(new MakeFace(Rows(_ => at.Lift(apex, 0), i => at.Lift(flat[i], deep)), u0, u1, 0, 1, Tolerance).Face());
        sewing.Add(new MakeFace(Rows(i => at.Lift(flat[i], deep), i => at.Lift(flat[i], 0)), u0, u1, 0, 1, Tolerance).Face());
        Point3D[] top = [.. flat.Select(p => at.Lift(p, 0))];
        var rim = new BSplineCurve3D(top, [.. spline.Knots], [.. spline.Multiplicities], spline.Degree, false, spline.IsRational ? [.. weights] : default);
        var wire = new MakeWire();
        wire.Add(new MakeEdge(frame.Lift(apex, 0), top[0]).Edge());
        wire.Add(new MakeEdge(rim).Edge());
        wire.Add(new MakeEdge(top[^1], frame.Lift(apex, 0)).Edge());
        sewing.Add(new MakeFace(wire.Wire(), true).Face());
    }
}
