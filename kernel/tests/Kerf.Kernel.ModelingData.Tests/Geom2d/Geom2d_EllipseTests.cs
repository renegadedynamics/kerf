// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Ellipse_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_EllipseTest
{
    // Ellipse centered at origin, major radius 10, minor radius 5
    private readonly EllipseCurve2D myEllipse =
        new EllipseCurve2D(new Ellipse2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 10.0, 5.0));

    [Fact]
    public void Radii()
    {
        Assert.Equal(10.0, myEllipse.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, myEllipse.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ParameterBounds()
    {
        Assert.Equal(0.0, myEllipse.FirstParameter());
        Assert.Equal(2.0 * M_PI, myEllipse.LastParameter());
    }

    [Fact]
    public void IsClosedAndPeriodic()
    {
        Assert.True(myEllipse.IsClosed);
        Assert.True(myEllipse.IsPeriodic);
    }

    [Fact]
    public void Eccentricity()
    {
        // e = sqrt(1 - (b/a)^2) = sqrt(1 - 0.25) = sqrt(0.75)
        double anExpected = Math.Sqrt(1.0 - (5.0 * 5.0) / (10.0 * 10.0));
        Assert.Equal(anExpected, myEllipse.Eccentricity(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Focal()
    {
        // Focal distance = sqrt(a^2 - b^2) * 2
        double anExpected = 2.0 * Math.Sqrt(10.0 * 10.0 - 5.0 * 5.0);
        Assert.Equal(anExpected, myEllipse.Focal(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Foci()
    {
        Point2D aFocus1 = myEllipse.Focus1();
        Point2D aFocus2 = myEllipse.Focus2();
        double c = Math.Sqrt(100.0 - 25.0); // sqrt(a^2 - b^2)
        Assert.Equal(c, aFocus1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aFocus1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-c, aFocus2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aFocus2.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parameter()
    {
        // Semi-latus rectum = b^2/a = 25/10 = 2.5
        Assert.Equal(2.5, myEllipse.Parameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MajorVertex()
    {
        // At U=0: point on major axis = (a, 0) = (10, 0)
        Point2D aPnt = myEllipse.EvalD0(0.0);
        Assert.Equal(10.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MinorVertex()
    {
        // At U=PI/2: point on minor axis = (0, b) = (0, 5)
        Point2D aPnt = myEllipse.EvalD0(M_PI / 2.0);
        Assert.Equal(0.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Curve2D.ResD1 aR = myEllipse.EvalD1(0.0);
        // At U=0: tangent is (0, b) = (0, 5)
        Assert.Equal(0.0, aR.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aR.D1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD2_AtZero()
    {
        Curve2D.ResD2 aR = myEllipse.EvalD2(0.0);
        // At U=0: second derivative is (-a, 0) = (-10, 0)
        Assert.Equal(-10.0, aR.D2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aR.D2.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetMajorRadius()
    {
        myEllipse.SetMajorRadius(20.0);
        Assert.Equal(20.0, myEllipse.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetMinorRadius()
    {
        myEllipse.SetMinorRadius(3.0);
        Assert.Equal(3.0, myEllipse.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PointOnEllipse_FociDistanceSum()
    {
        // For any point on ellipse: d(P, F1) + d(P, F2) = 2a
        Point2D aF1 = myEllipse.Focus1();
        Point2D aF2 = myEllipse.Focus2();
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 6.0)
        {
            Point2D aPnt = myEllipse.EvalD0(u);
            double aSum = aPnt.Distance(aF1) + aPnt.Distance(aF2);
            Assert.Equal(2.0 * 10.0, aSum, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void Copy()
    {
        Geometry2D aCopyGeom = myEllipse.Copy();
        EllipseCurve2D? aCopy = aCopyGeom as EllipseCurve2D;
        Assert.NotNull(aCopy);
        Assert.Equal(10.0, aCopy.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aCopy.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Transform2D aTrsf = new Transform2D();
        aTrsf = aTrsf.WithTranslation(new Vector2D(5.0, 10.0));
        myEllipse.Transform(aTrsf);
        Assert.Equal(5.0, myEllipse.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, myEllipse.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, myEllipse.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ReversedParameter()
    {
        double aU = M_PI / 4.0;
        Assert.Equal(2.0 * M_PI - aU, myEllipse.ReversedParameter(aU), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
