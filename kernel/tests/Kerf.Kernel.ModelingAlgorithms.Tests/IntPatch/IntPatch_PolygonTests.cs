// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>The 2D polygons of IntPatch_RstInt: an arc's (IntPatch_PolyArc) and a line's (IntPatch_PolyLine).</summary>
public class IntPatch_PolygonTests
{
    private static Box2d Box(double x0, double y0, double x1, double y1)
    {
        Box2d aBox = new();
        aBox.Update(x0, y0, x1, y1);
        return aBox;
    }

    [Fact]
    public void PolyArc_on_a_half_circle()
    {
        CurveAdaptor2D aCircle = new(new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0));
        PolyArc aPoly = new(aCircle, 10, 0.0, Math.PI, Box(-1, 0, 1, 1));

        Assert.Equal(10, aPoly.PointCount);
        Assert.Equal(9, aPoly.SegmentCount);
        Assert.False(aPoly.Closed());
        for (int i = 1; i <= 10; i++)
        {
            double t = (i - 1) * (Math.PI / 9);
            Assert.Equal(t, aPoly.Parameter(i), 1e-15);
            Assert.True(aPoly.Point(i).DistanceTo(new Point2D(Math.Cos(t), Math.Sin(t))) < 1e-15);
        }
        // The deflection: 1.2 times the largest sagitta of a chord, 1 - cos(pi / 18).
        Assert.Equal(1.2 * (1 - Math.Cos(Math.PI / 18)), aPoly.Error(), 1e-12);
        Assert.Equal(aPoly.Error(), aPoly.DeflectionOverEstimation());

        aPoly.Segment(2, out Point2D aB, out Point2D aE);
        Assert.Equal(aPoly.Point(2), aB);
        Assert.Equal(aPoly.Point(3), aE);

        aPoly.SetOffset(2 * Math.PI, 0);
        Assert.Equal(1 + 2 * Math.PI, aPoly.Point(1).X, 1e-15);

        // A full circle is closed; infinite bounds or one sample are refused.
        Assert.True(new PolyArc(aCircle, 12, 0.0, 2 * Math.PI, Box(-1, -1, 1, 1)).Closed());
        Assert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new PolyArc(aCircle, 1, 0.0, 1.0, Box(-1, -1, 1, 1)));
    }

    [Fact]
    public void PolyArc_resamples_the_part_near_the_other_polygon()
    {
        // A long segment whose middle only is near the box: the polygon is resampled between the indices around it.
        CurveAdaptor2D aLine = new(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)));
        PolyArc aPoly = new(aLine, 11, 0.0, 1000.0, Box(499, -0.5, 501, 0.5));
        Assert.Equal(11, aPoly.PointCount);
        Assert.True(aPoly.Parameter(1) > 0.0 && aPoly.Parameter(11) < 1000.0);
        Assert.True(aPoly.Parameter(1) <= 500.0 && aPoly.Parameter(11) >= 500.0);
    }

    [Fact]
    public void PolyLine_on_a_straight_WLine_pushes_the_ends_out()
    {
        WLine aW = new(IntPatch_WLineTests.StraightLine(3), false);
        PolyLine aPoly = new();
        double anInit = aPoly.Error();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.PConfusion() * 100.0, anInit);
        aPoly.SetWLine(true, aW);
        Assert.Equal(3, aPoly.PointCount);
        Assert.Equal(new Point2D(0 + 1e-7 * (0 - 1), 0), aPoly.Point(1));
        Assert.Equal(new Point2D(1, 0), aPoly.Point(2));
        Assert.Equal(new Point2D(2 + 1e-7 * (2 - 1), 0), aPoly.Point(3));
        Assert.Equal(anInit, aPoly.Error()); // collinear: no deflection

        aPoly.SetWLine(false, aW); // on S2: v = 1
        Assert.Equal(new Point2D(1, 1), aPoly.Point(2));
    }

    [Fact]
    public void PolyLine_on_a_curved_line_estimates_its_deflection()
    {
        LineOn2S aCurve = new();
        for (int k = 0; k <= 8; k++)
        {
            double t = k * Math.PI / 8;
            PntOn2S aP = new();
            aP.SetValue(new Point3D(Math.Cos(t), Math.Sin(t), 0), Math.Cos(t), Math.Sin(t), 0, 0);
            aCurve.Add(aP);
        }
        RLine aR = new(false);
        aR.Add(aCurve);
        PolyLine aPoly = new(0.0);
        aPoly.SetRLine(true, aR);
        // The sagitta of a chord of pi / 8 on the unit circle, within the parabola's estimate.
        double aSagitta = 1 - Math.Cos(Math.PI / 16);
        Assert.InRange(aPoly.Error(), 0.5 * aSagitta, 2 * aSagitta);
        aPoly.ResetError();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.PConfusion() * 100.0, aPoly.Error());

        StringWriter aOut = new();
        aPoly.Dump(aOut);
        Assert.Contains("pol2d ", aOut.ToString());
        Assert.Contains("pnt2d ", aOut.ToString());
    }
}
