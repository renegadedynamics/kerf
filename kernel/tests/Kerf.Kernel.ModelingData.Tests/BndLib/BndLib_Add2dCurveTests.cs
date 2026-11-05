// Port of OCCT's src/ModelingData/TKGeomBase/GTests/BndLib_Test.cxx (V8_0_1), suite BndLib_Add2dCurveTest.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class BndLib_Add2dCurveTest
{
    [Fact]
    public void Circle_Full()
    {
        // Create a 2D circle using Geom2d_Circle
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.Add(aCircle, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(-5.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_Arc()
    {
        // Circle arc from 0 to PI/2
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.Add(aCircle, 0.0, M_PI / 2.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Ellipse_Full()
    {
        // Ellipse with major radius 8, minor radius 4
        EllipseCurve2D anEllipse = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 8.0, 4.0);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.Add(anEllipse, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(-8.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Line_Segment()
    {
        // Line segment from (0,0) to (10,5)
        LineCurve2D aLine = new(new Point2D(0.0, 0.0), new Direction2D(2.0, 1.0));
        TrimmedCurve2D aTrimmed = new(aLine, 0.0, Math.Sqrt(125.0)); // length = sqrt(100+25)
        CurveAdaptor2D aCurve = new(aTrimmed);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aXmax, 0.01);
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, 0.01);
    }

    [Fact]
    public void BezierCurve()
    {
        // Create a cubic Bezier curve in 2D
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(0.0, 0.0);
        aPoles[2] = new Point2D(1.0, 3.0);
        aPoles[3] = new Point2D(3.0, 3.0);
        aPoles[4] = new Point2D(4.0, 0.0);

        BezierCurve2D aBezier = new(aPoles);
        CurveAdaptor2D aCurve = new(aBezier);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        // Box should contain the actual curve extents (not necessarily the control polygon).
        // GeomBndLib computes a tight box based on the actual curve, not the convex hull of poles.
        Assert.True(aXmin <= 0.0);
        Assert.True(aXmax >= 4.0);
        Assert.True(aYmin <= 0.0);
        Assert.True(aYmax > 1.0, "Y max should be > 1 (curve rises above Y=0)");
    }

    [Fact]
    public void AddOptimal_Ellipse()
    {
        // Test AddOptimal method with an ellipse
        EllipseCurve2D anEllipse = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 6.0, 3.0);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.AddOptimal(anEllipse, 0.0, 2.0 * M_PI, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(-6.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(6.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-3.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Adaptor_Circle()
    {
        // Test using Adaptor interface
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(5.0, 5.0), new Direction2D(1.0, 0.0)), 3.0);
        CurveAdaptor2D aCurve = new(aCircle);
        Box2d aBox = new();
        double aTol = 0.0;

        Add2dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(2.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
