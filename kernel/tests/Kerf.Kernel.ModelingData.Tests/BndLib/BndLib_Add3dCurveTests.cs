// Port of OCCT's src/ModelingData/TKGeomBase/GTests/BndLib_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// The BndLib_Add3dCurveTest suite of BndLib_Test.cxx.
public class BndLib_Add3dCurveTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private static Frame3D Ax2(double theDX, double theDY, double theDZ)
        => new(new Point3D(0.0, 0.0, 0.0), new Direction3D(theDX, theDY, theDZ));

    [Fact]
    public void Circle_Full()
    {
        // Create a 3D circle using Geom_Circle
        CircleCurve3D aCircle = new(Ax2(0.0, 0.0, 1.0), 5.0);
        CurveAdaptor aCurve = new(aCircle);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-5.0, aXmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(-5.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Circle_Arc()
    {
        // Circle arc from 0 to PI/2
        CircleCurve3D aCircle = new(Ax2(0.0, 0.0, 1.0), 5.0);
        CurveAdaptor aCurve = new(aCircle);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, 0.0, M_PI / 2.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(0.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
    }

    [Fact]
    public void Ellipse_Full()
    {
        // Ellipse with major radius 10, minor radius 5
        EllipseCurve3D anEllipse = new(Ax2(0.0, 0.0, 1.0), 10.0, 5.0);
        CurveAdaptor aCurve = new(anEllipse);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        Assert.Equal(-10.0, aXmin, Conf);
        Assert.Equal(10.0, aXmax, Conf);
        Assert.Equal(-5.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
    }

    [Fact]
    public void Line_Segment()
    {
        // Line segment from (0,0,0) to (10,0,0)
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        TrimmedCurve3D aTrimmed = new(aLine, 0.0, 10.0);
        CurveAdaptor aCurve = new(aTrimmed);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(10.0, aXmax, Conf);
        Assert.Equal(0.0, aYmin, Conf);
        Assert.Equal(0.0, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void BezierCurve()
    {
        // Create a cubic Bezier curve
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles[3] = new Point3D(3.0, 2.0, 0.0);
        aPoles[4] = new Point3D(4.0, 0.0, 0.0);

        BezierCurve3D aBezier = new(aPoles);
        CurveAdaptor aCurve = new(aBezier);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // Box should contain curve endpoints and the actual curve
        // Note: The curve doesn't necessarily pass through control points,
        // so the box may be smaller than the convex hull of control points
        Assert.False(aBox.IsVoid, "Box should not be void");
        Assert.True(aXmin <= 0.0);
        Assert.True(aXmax >= 4.0);
        Assert.True(aYmin <= 0.0);
        // Y max is at curve's maximum, not necessarily at control point Y
        Assert.True(aYmax > 1.0, "Y max should be > 1 (curve rises above Y=0)");
    }

    [Fact]
    public void BSplineCurve()
    {
        // Create a simple B-spline curve
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles[4] = new Point3D(3.0, 0.0, 0.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aBSpline = new(aPoles, aKnots, aMults, 3);
        CurveAdaptor aCurve = new(aBSpline);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out _, out _, out double aXmax, out _, out _);

        Assert.False(aBox.IsVoid, "Box should not be void");
        Assert.True(aXmin <= 0.0);
        Assert.True(aXmax >= 3.0);
    }

    [Fact]
    public void AddOptimal_Circle()
    {
        // Test AddOptimal method with a circle
        CircleCurve3D aCircle = new(Ax2(0.0, 0.0, 1.0), 5.0);
        CurveAdaptor aCurve = new(aCircle);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.AddOptimal(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        Assert.Equal(-5.0, aXmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(-5.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
    }

    [Fact]
    public void Circle_Rotated()
    {
        // Circle in YZ plane (axis along X)
        CircleCurve3D aCircle = new(Ax2(1.0, 0.0, 0.0), 4.0);
        CurveAdaptor aCurve = new(aCircle);
        Box aBox = new();
        double aTol = 0.0;

        Add3dCurve.Add(aCurve, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(0.0, aXmax, Conf);
        Assert.Equal(-4.0, aYmin, Conf);
        Assert.Equal(4.0, aYmax, Conf);
        Assert.Equal(-4.0, aZmin, Conf);
        Assert.Equal(4.0, aZmax, Conf);
    }
}
