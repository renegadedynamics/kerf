// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A solid's classifier doesn't look for the extrema of its line, or point, with a spline edge whose poles keep it further than
/// the edge's tolerance (unit 13K, <c>SplineHull</c>): those extrema are a global search, and were nearly all of the points'
/// time on a fuse of cylinders nearly crossing. The states are the geometry's, and the extrema asked are counted: 182 here, of
/// 614 a classifier asked before.
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
        output.WriteLine($"{points} points, {inside} inside; spline edges kept away {SplineHull.s_keptAway}, extrema asked {SplineHull.s_extremaAsked}");
        Assert.True(inside > 100);
        Assert.True(SplineHull.s_keptAway > 0);
        Assert.True(SplineHull.s_extremaAsked * 2 <= SplineHull.s_keptAway, $"{SplineHull.s_extremaAsked} extrema asked");
    }
}
