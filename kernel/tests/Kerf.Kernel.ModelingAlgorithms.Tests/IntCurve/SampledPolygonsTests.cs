// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Polygon = Kerf.Engine.Intersection.Curves.Polygon2dGen<Kerf.Engine.Geometry.Adaptors.Curve2DAdaptorBase, Kerf.Engine.Intersection.TwoD.Geom2dCurveTool>;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>A table of first samplings gives the polygons that sampling gives, and refining one touches no other.</summary>
public class SampledPolygonsTests
{
    private const int NbPoints = 40;
    private const double Tol = 1e-7;

    private static Curve2DAdaptorBase Arc(CircleCurve2D curve) => new CurveAdaptor2D(curve, 0.5, 2.5);

    private static TwoDDomain Domain(Curve2DAdaptorBase c) => new(c.Value(0.5), 0.5, Tol, c.Value(2.5), 2.5, Tol);

    private static CircleCurve2D NewCurve() => new(new Axis2D(new Point2D(1, 2), new Direction2D(1, 0)), 5.0);

    private static List<(double, double, double, double)> Segments(Polygon polygon)
    {
        List<(double, double, double, double)> segments = [];
        for (int i = 1; i <= polygon.SegmentCount; i++)
        {
            polygon.Segment(i, out Point2D begin, out Point2D end);
            segments.Add((begin.X, begin.Y, end.X, end.Y));
        }
        return segments;
    }

    [Fact]
    public void A_polygon_from_a_table_is_what_sampling_makes()
    {
        CircleCurve2D curve = NewCurve();
        Curve2DAdaptorBase c = Arc(curve);
        TwoDDomain d = Domain(c);
        Polygon sampled = new(c, NbPoints, d, Tol);
        Polygon first, second;
        using (new SampledPolygons().Use())
        {
            first = Polygon.Sample(c, NbPoints, d, Tol);
            second = Polygon.Sample(Arc(curve), NbPoints, d, Tol); // Another adaptor of the same curve
        }

        Assert.Equal(Segments(sampled), Segments(first));
        Assert.Equal(Segments(sampled), Segments(second));
        Assert.Equal(sampled.DeflectionOverEstimation(), second.DeflectionOverEstimation());
        Assert.Equal(sampled.Bounding(), second.Bounding());
        Assert.Equal(sampled.InfParameter(), second.InfParameter());
        Assert.Equal(sampled.SupParameter(), second.SupParameter());
    }

    [Fact]
    public void Refining_a_polygon_changes_neither_the_table_nor_the_polygons_sharing_it()
    {
        CircleCurve2D curve = NewCurve();
        Curve2DAdaptorBase c = Arc(curve);
        TwoDDomain d = Domain(c);
        Box2d box = new();
        box.Add(new Point2D(-10, -10));
        box.Add(new Point2D(10, 10));
        // A coarse sampling and a small deflection, so that ComputeWithBox has to add points
        Polygon plain = new(c, 5, d, Tol);
        plain.SetDeflectionOverEstimation(1e-6);
        plain.ComputeWithBox(c, box);

        using (new SampledPolygons().Use())
        {
            Polygon first = Polygon.Sample(c, 5, d, Tol);
            Polygon second = Polygon.Sample(c, 5, d, Tol);
            List<(double, double, double, double)> before = Segments(second);

            first.SetDeflectionOverEstimation(1e-6);
            first.ComputeWithBox(c, box);
            Assert.Equal(Segments(plain), Segments(first));
            Assert.True(first.SegmentCount > 4);

            Assert.Equal(before, Segments(second));
            Polygon third = Polygon.Sample(c, 5, d, Tol);
            Assert.Equal(before, Segments(third));
            third.SetDeflectionOverEstimation(1e-6);
            third.ComputeWithBox(c, box);
            Assert.Equal(Segments(plain), Segments(third));
        }
    }

    [Fact]
    public void Without_a_table_or_after_one_nothing_is_kept()
    {
        Assert.Null(SampledPolygons.Current);
        SampledPolygons table = new();
        using (table.Use())
        {
            Assert.Same(table, SampledPolygons.Current);
        }
        Assert.Null(SampledPolygons.Current);
    }
}
