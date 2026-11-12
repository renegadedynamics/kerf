// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A solid's classifier doesn't look for the extrema of its line, or point, with a spline edge whose poles keep it further than
/// the edge's tolerance (unit 13K, <c>SplineHull</c>): those extrema are a global search, and were nearly all of the points'
/// time on a fuse of cylinders nearly crossing. The states are the geometry's, and the extrema asked are counted: 182 here, of
/// 614 a classifier asked before, and 2 since a span whose poles come within the distance is halved (unit 15AH3).
/// </summary>
public class SplineHullTests(ITestOutputHelper output)
{
    private const double Radius = 5, Thin = 3;

    // A cylinder of radius 5 on the Z axis, 10 high, and one of radius 3 along X through it at height 5: their section is two
    // walked curves, splines
    private static Shape Crossed()
    {
        Shape upright = Solid.Cylinder(Radius, 2 * Radius).Value.Shape;
        var along = new Frame3D(new Point3D(-2 * Radius, 0, Radius), Direction3D.UnitX);
        Shape across = Solid.Cylinder(Thin, 4 * Radius, along).Value.Shape;
        return upright.Fuse(across).Value.Shape;
    }

    // Inside either cylinder by the geometry, or null within a hundredth of a surface
    private static bool? Inside(in Point3D p)
    {
        double upright = Math.Min(Radius - Math.Sqrt(p.X * p.X + p.Y * p.Y), Math.Min(p.Z, 2 * Radius - p.Z));
        double across = Math.Min(Thin - Math.Sqrt(p.Y * p.Y + (p.Z - Radius) * (p.Z - Radius)), Math.Min(p.X + 2 * Radius, 2 * Radius - p.X));
        double depth = Math.Max(upright, across);
        return Math.Abs(upright) < 0.01 || Math.Abs(across) < 0.01 ? null : depth > 0;
    }

    [Fact]
    public void A_classifier_skips_the_spline_edges_its_line_keeps_away_from()
    {
        Shape fused = Crossed();
        var classifier = new SolidClassifier(fused);
        SplineHull.s_keptAway = 0;
        SplineHull.s_halvedAway = 0;
        SplineHull.s_extremaAsked = 0;
        int points = 0, inside = 0;
        for (int i = 0; i <= 8; i++)
        {
            for (int j = 0; j <= 8; j++)
            {
                for (int k = 0; k <= 8; k++)
                {
                    var point = new Point3D(-11 + 22 * i / 8.0, -6 + 12 * j / 8.0, -1 + 12 * k / 8.0);
                    if (Inside(point) is not { } expected)
                    {
                        continue;
                    }
                    classifier.Perform(point, 1e-7);
                    Assert.Equal(expected ? Containment.In : Containment.Out, classifier.State());
                    points++;
                    inside += expected ? 1 : 0;
                }
            }
        }
        output.WriteLine($"{points} points, {inside} inside; spline edges kept away {SplineHull.s_keptAway} ({SplineHull.s_halvedAway} spans halved), extrema asked {SplineHull.s_extremaAsked}");
        Assert.True(inside > 100);
        Assert.True(SplineHull.s_keptAway > 0);
        Assert.True(SplineHull.s_extremaAsked * 20 <= SplineHull.s_keptAway, $"{SplineHull.s_extremaAsked} extrema asked");
    }

    // A rational cubic of four spans with uneven knots and weights, on part of its range, placed off the origin
    private static Edge Rational()
    {
        var poles = new NCollection_Array1<Point3D>(1, 7);
        var weights = new NCollection_Array1<double>(1, 7);
        double[] x = [0, 2, 3, 5, 6, 8, 9], y = [0, 4, -1, 3, 5, -2, 1], z = [0, 1, 3, 0, 2, 4, 1], w = [1, 3, 0.5, 2, 0.7, 4, 1];
        for (int i = 0; i < 7; i++)
        {
            poles[i + 1] = new Point3D(x[i], y[i], z[i]);
            weights[i + 1] = w[i];
        }
        var knots = new NCollection_Array1<double>(1, 5);
        var mults = new NCollection_Array1<int>(1, 5);
        double[] k = [0, 0.7, 1, 2.5, 3];
        int[] m = [4, 1, 1, 1, 4];
        for (int i = 0; i < 5; i++)
        {
            knots[i + 1] = k[i];
            mults[i + 1] = m[i];
        }
        var curve = new BSplineCurve3D(poles, weights, knots, mults, 3);
        Edge edge = new MakeEdge(curve, 0.4, 2.8).Edge();
        return ShapeCasts.Edge(edge.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(3, -2, 1)))));
    }

    // A rational Bezier of degree 5
    private static Edge Bezier()
    {
        var poles = new NCollection_Array1<Point3D>(1, 6);
        var weights = new NCollection_Array1<double>(1, 6);
        double[] x = [0, 1, 4, 5, 7, 9], y = [0, 5, 6, -3, 2, 0], z = [1, -2, 3, 2, -1, 0], w = [1, 0.4, 2.5, 1, 3, 1];
        for (int i = 0; i < 6; i++)
        {
            poles[i + 1] = new Point3D(x[i], y[i], z[i]);
            weights[i + 1] = w[i];
        }
        return new MakeEdge(new BezierCurve3D(poles, weights)).Edge();
    }

    // The nearest of many points along the edge's range to a point, or to the line through it along a direction
    private static double Nearest(in Edge edge, in Point3D origin, Direction3D? along)
    {
        Curve3D curve = Tool.Curve(edge, out Placement placement, out double from, out double to)!;
        double nearest = double.MaxValue;
        const int Samples = 4000;
        for (int i = 0; i <= Samples; i++)
        {
            Point3D p = curve.Evaluate(from + ((to - from) * i / Samples)).Transformed(placement.Transformation);
            var offset = new Vector3D(origin, p);
            if (along is { } d)
            {
                offset -= new Vector3D(d) * offset.Dot(new Vector3D(d));
            }
            nearest = Math.Min(nearest, offset.Magnitude);
        }
        return nearest;
    }

    // Halving a span whose poles come within the distance (unit 15AH3) only keeps away an edge no point of which is within it, so
    // the selectors keep what they kept: over lines and points all round the crossed cylinders' section splines, a rational spline
    // on part of its range and placed, and a rational Bezier, each edge kept away is further than the distance at every one of
    // 4,001 points along it, and the halving is what keeps many of them away
    [Fact]
    public void Halving_a_span_keeps_away_only_an_edge_that_is_away()
    {
        List<Edge> edges = [.. Crossed().AllEdges().Where(static e => Tool.Curve(e, out _, out _) is BSplineCurve3D or BezierCurve3D), Rational(), Bezier()];
        Assert.True(edges.Count >= 4, $"{edges.Count} spline edges");
        Direction3D[] directions = [Direction3D.UnitX, Direction3D.UnitZ, new Direction3D(1, 2, -1), new Direction3D(-3, 1, 2)];
        SplineHull.s_keptAway = 0;
        SplineHull.s_halvedAway = 0;
        SplineHull.s_extremaAsked = 0;
        int asked = 0;
        foreach (Edge edge in edges)
        {
            BoundingBox box = ((Shape)edge).BoundingBox();
            foreach (double distance in new[] { 1e-7, 0.05, 0.4 })
            {
                for (int i = 0; i <= 4; i++)
                {
                    for (int j = 0; j <= 4; j++)
                    {
                        for (int k = 0; k <= 4; k++)
                        {
                            var origin = new Point3D(box.Min.X + ((box.Max.X - box.Min.X) * i / 4), box.Min.Y + ((box.Max.Y - box.Min.Y) * j / 4), box.Min.Z + ((box.Max.Z - box.Min.Z) * k / 4));
                            foreach (Direction3D? along in directions.Cast<Direction3D?>().Append(null))
                            {
                                asked++;
                                if (SplineHull.KeepsAway(edge, distance, origin, along))
                                {
                                    double nearest = Nearest(edge, origin, along);
                                    Assert.True(nearest > distance, $"kept away at {nearest} for {distance}");
                                }
                            }
                        }
                    }
                }
            }
        }
        output.WriteLine($"{asked} asked: kept away {SplineHull.s_keptAway} ({SplineHull.s_halvedAway} spans halved), extrema asked {SplineHull.s_extremaAsked}");
        Assert.True(SplineHull.s_halvedAway > 100, $"{SplineHull.s_halvedAway} spans halved");
    }
}
