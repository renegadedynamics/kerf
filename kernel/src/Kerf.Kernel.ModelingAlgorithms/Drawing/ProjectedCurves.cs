// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Drawing;

/// <summary>
/// The hidden-line removal's projected curves as public 2D curves: <c>HLRBRep::MakeEdge</c>'s cases without the edge,
/// and a polyline within a tolerance where OCCT's makes a coarse one (fifteen points) for other curves.
/// </summary>
internal static class ProjectedCurves
{
    /// <summary>The part of the projected curve between the 3D parameters; null where it is a point.</summary>
    public static Curve2D? Make(HiddenLineCurve ec, double u1, double u2, double tolerance)
    {
        if (!(Math.Abs(u2 - u1) > 1e-12))
        {
            return null;
        }

        double sta = ec.Parameter2d(u1), end = ec.Parameter2d(u2);
        try
        {
            switch (ec.GetType())
            {
                case CurveKind.Line:
                    return Trimmed(new LineCurve2D(ec.Line()), sta, end, periodic: false);
                case CurveKind.Circle:
                    return Trimmed(new CircleCurve2D(ec.Circle()), sta, end, periodic: true);
                case CurveKind.Ellipse:
                    return Trimmed(new EllipseCurve2D(ec.Ellipse()), sta, end, periodic: true);
                case CurveKind.BezierCurve:
                {
                    var poles = new NCollection_Array1<Point2D>(1, ec.PoleCount);
                    BezierCurve2D bezier;
                    if (ec.IsRational)
                    {
                        var weights = new NCollection_Array1<double>(1, ec.PoleCount);
                        ec.PolesAndWeights(poles, weights);
                        bezier = new BezierCurve2D(poles, weights);
                    }
                    else
                    {
                        ec.Poles(poles);
                        bezier = new BezierCurve2D(poles);
                    }

                    return Within(bezier, sta, end);
                }

                case CurveKind.BSplineCurve when BSpline(ec, sta, end) is { } spline:
                    return Within(spline, sta, end);
            }
        }
        catch (Exception e) when (e is FailureException || e is ArgumentException)
        {
            // A curve the cases can't make: drawn as a polyline below
        }

        return Sampled(ec, u1, u2, tolerance);
    }

    private static Curve2D? Trimmed(Curve2D basis, double sta, double end, bool periodic)
    {
        if (!periodic && sta > end)
        {
            (sta, end) = (end, sta);
        }

        return Math.Abs(end - sta) > 1e-12 ? new TrimmedCurve2D(basis, sta, end) : null;
    }

    // A bounded curve, segmented where the part isn't the whole
    private static Curve2D? Within(BoundedCurve2D curve, double sta, double end)
    {
        if (sta > end)
        {
            (sta, end) = (end, sta);
        }

        if (!(end - sta > 1e-12))
        {
            return null;
        }

        double first = curve.FirstParameter, last = curve.LastParameter;
        if (Math.Abs(sta - first) <= Precision.PConfusion() && Math.Abs(end - last) <= Precision.PConfusion())
        {
            return curve;
        }

        return curve switch
        {
            BSplineCurve2D spline => spline.Segmented(Math.Max(sta, first), Math.Min(end, last)),
            BezierCurve2D bezier => new TrimmedCurve2D(bezier, sta, end),
            _ => new TrimmedCurve2D(curve, sta, end),
        };
    }

    // HLRBRep::MakeEdge's B-spline case
    private static BSplineCurve2D? BSpline(HiddenLineCurve ec, double sta, double end)
    {
        Edge edge = ec.GetCurve().Edge();
        Curve3D? curve = Tool.Curve(edge, out _, out _);
        if (curve is TrimmedCurve3D trimmed)
        {
            curve = trimmed.BasisCurve;
        }

        if (curve is not BSplineCurve3D bspline)
        {
            return null;
        }

        if (bspline.IsPeriodic && !ec.GetCurve().Curve().IsClosed)
        {
            var theCurve = (BSplineCurve3D)bspline.Copy();
            theCurve.Segment(sta, end);
            var poles = new NCollection_Array1<Point2D>(1, theCurve.PoleCount);
            var knots = new NCollection_Array1<double>(theCurve.InternalKnots());
            var mults = new NCollection_Array1<int>(theCurve.InternalMultiplicities());
            if (theCurve.IsRational)
            {
                var weights = new NCollection_Array1<double>(1, theCurve.PoleCount);
                ec.PolesAndWeights(theCurve, poles, weights);
                return new BSplineCurve2D(poles, weights, knots, mults, theCurve.Degree, theCurve.IsPeriodic);
            }

            ec.Poles(theCurve, poles);
            return new BSplineCurve2D(poles, knots, mults, theCurve.Degree, theCurve.IsPeriodic);
        }
        else
        {
            var poles = new NCollection_Array1<Point2D>(1, ec.PoleCount);
            var knots = new NCollection_Array1<double>(1, ec.KnotCount);
            var mults = new NCollection_Array1<int>(1, ec.KnotCount);
            ec.Knots(knots);
            ec.Multiplicities(mults);
            if (ec.IsRational)
            {
                var weights = new NCollection_Array1<double>(1, ec.PoleCount);
                ec.PolesAndWeights(poles, weights);
                return new BSplineCurve2D(poles, weights, knots, mults, ec.Degree(), ec.IsPeriodic);
            }

            ec.Poles(poles);
            return new BSplineCurve2D(poles, knots, mults, ec.Degree(), ec.IsPeriodic);
        }
    }

    // A curve with no exact 2D form: a polyline within the tolerance, on the 3D curve's parameters
    private static Curve2D? Sampled(HiddenLineCurve ec, double u1, double u2, double tolerance)
    {
        List<Point2D> points = Flattening.Flatten(ec.Value, Math.Min(u1, u2), Math.Max(u1, u2), tolerance);
        return points.Count >= 2 && points[0].SquareDistanceTo(points[^1]) + Length2(points) > 1e-20 ? Polyline(points) : null;
    }

    private static double Length2(List<Point2D> points)
    {
        double sum = 0;
        for (int i = 1; i < points.Count; i++)
        {
            sum += points[i - 1].SquareDistanceTo(points[i]);
        }

        return sum;
    }

    /// <summary>A polyline: a line segment for two points, else a B-spline of degree 1 through the points.</summary>
    public static Curve2D Polyline(List<Point2D> points)
    {
        if (points.Count == 2)
        {
            Point2D p = points[0], q = points[1];
            var line = new LineCurve2D(p, new Direction2D(q.X - p.X, q.Y - p.Y));
            return new TrimmedCurve2D(line, 0, p.DistanceTo(q));
        }

        var knots = new double[points.Count];
        var mults = new int[points.Count];
        for (int i = 0; i < knots.Length; i++)
        {
            knots[i] = i;
            mults[i] = 1;
        }

        mults[0] = mults[^1] = 2;
        return new BSplineCurve2D([.. points], knots, mults, 1);
    }
}

/// <summary>Curves flattened to polylines within a tolerance, deterministically.</summary>
internal static class Flattening
{
    /// <summary>The points of a polyline within <paramref name="tolerance"/> of the curve between the parameters:
    /// eight equal steps, each halved while its midpoint is farther than the tolerance from its chord.</summary>
    public static List<Point2D> Flatten(Func<double, Point2D> curve, double u0, double u1, double tolerance)
    {
        var points = new List<Point2D> { curve(u0) };
        const int steps = 8;
        Point2D previous = points[0];
        for (int i = 1; i <= steps; i++)
        {
            double a = u0 + (u1 - u0) * (i - 1) / steps, b = i == steps ? u1 : u0 + (u1 - u0) * i / steps;
            Point2D next = curve(b);
            Subdivide(curve, a, previous, b, next, tolerance, 0, points);
            previous = next;
        }

        return points;
    }

    private static void Subdivide(Func<double, Point2D> curve, double a, Point2D pa, double b, Point2D pb, double tolerance, int depth,
                                  List<Point2D> points)
    {
        double m = 0.5 * (a + b);
        Point2D pm = curve(m);
        if (depth < 16 && DistanceToChord(pm, pa, pb) > tolerance)
        {
            Subdivide(curve, a, pa, m, pm, tolerance, depth + 1, points);
            Subdivide(curve, m, pm, b, pb, tolerance, depth + 1, points);
            return;
        }

        points.Add(pb);
    }

    private static double DistanceToChord(Point2D p, Point2D a, Point2D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
        if (l2 <= 0)
        {
            return p.DistanceTo(a);
        }

        double t = Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1);
        double ex = a.X + t * dx - p.X, ey = a.Y + t * dy - p.Y;
        return Math.Sqrt(ex * ex + ey * ey);
    }
}
