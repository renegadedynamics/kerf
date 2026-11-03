// Port of: src/ModelingData/TKGeomBase/GTests/GeomLProp_CLProps2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomLProp_CLProps2dTest
{
    private readonly CircleCurve2D myCircle;
    private readonly LineCurve2D myLine;
    private readonly EllipseCurve2D myEllipse;

    public GeomLProp_CLProps2dTest()
    {
        // Circle centered at origin, radius 5
        var aCirc = new Circle2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        myCircle = new CircleCurve2D(aCirc);

        // Line through origin along X
        var aLin = new Line2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        myLine = new LineCurve2D(aLin);

        // Ellipse centered at origin, major radius 10, minor radius 5
        var anElips = new Ellipse2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 10.0, 5.0);
        myEllipse = new EllipseCurve2D(anElips);
    }

    [Fact]
    public void Circle_Value()
    {
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        Point2D aPnt = aProps.Value();
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_SetParameter()
    {
        var aProps = new CLProps2d(myCircle, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        aProps.SetParameter(Math.PI / 2.0);
        Point2D aPnt = aProps.Value();
        Assert.Equal(0.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_TangentDefined()
    {
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aProps.IsTangentDefined());
    }

    [Fact]
    public void Circle_TangentAtZero()
    {
        // At U=0 on a circle, the tangent should point in +Y direction
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        aProps.Tangent(out Direction2D aTangent);
        Assert.Equal(0.0, aTangent.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aTangent.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_Curvature()
    {
        // Curvature of a circle of radius R is 1/R
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        double aCurvature = aProps.Curvature();
        Assert.Equal(1.0 / 5.0, aCurvature, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_CurvatureConstant()
    {
        // Curvature on a circle should be the same at every point
        var aProps = new CLProps2d(myCircle, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        for (double u = 0.0; u < 2.0 * Math.PI; u += Math.PI / 4.0)
        {
            aProps.SetParameter(u);
            Assert.Equal(1.0 / 5.0, aProps.Curvature(), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void Circle_Normal()
    {
        // Normal at U=0 on a circle centered at origin should point inward (-X)
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        aProps.Normal(out Direction2D aNormal);
        // Normal should be perpendicular to tangent
        aProps.Tangent(out Direction2D aTangent);
        Assert.Equal(0.0, Math.Abs(aTangent.X() * aNormal.X() + aTangent.Y() * aNormal.Y()), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_CentreOfCurvature()
    {
        // Centre of curvature of a circle is its center
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        aProps.CentreOfCurvature(out Point2D aCenter);
        Assert.Equal(0.0, aCenter.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCenter.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_CentreOfCurvature_AtPiHalf()
    {
        var aProps = new CLProps2d(myCircle, Math.PI / 2.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        aProps.CentreOfCurvature(out Point2D aCenter);
        Assert.Equal(0.0, aCenter.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCenter.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Line_TangentDefined()
    {
        var aProps = new CLProps2d(myLine, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aProps.IsTangentDefined());
    }

    [Fact]
    public void Line_Tangent()
    {
        var aProps = new CLProps2d(myLine, 5.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        aProps.Tangent(out Direction2D aTangent);
        Assert.Equal(1.0, aTangent.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aTangent.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Line_CurvatureIsZero()
    {
        var aProps = new CLProps2d(myLine, 5.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        double aCurvature = aProps.Curvature();
        Assert.Equal(0.0, aCurvature, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Line_CentreOfCurvature_Throws()
    {
        // Centre of curvature is not defined for a line (zero curvature)
        var aProps = new CLProps2d(myLine, 5.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.ThrowsAny<NotDefinedException>(() => aProps.CentreOfCurvature(out _));
    }

    [Fact]
    public void Ellipse_CurvatureAtMajorVertex()
    {
        // Radius of curvature at the major vertex = b^2/a, so curvature = a/b^2
        var aProps = new CLProps2d(myEllipse, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        double aCurvature = aProps.Curvature();
        double aExpected = 10.0 / (5.0 * 5.0); // a/b^2 = 10/25 = 0.4
        Assert.Equal(aExpected, aCurvature, 1e-6);
    }

    [Fact]
    public void Ellipse_CurvatureAtMinorVertex()
    {
        // At U=PI/2, ellipse is at minor vertex. Curvature = b/a^2
        var aProps = new CLProps2d(myEllipse, Math.PI / 2.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());

        double aCurvature = aProps.Curvature();
        double aExpected = 5.0 / (10.0 * 10.0); // b/a^2 = 5/100 = 0.05
        Assert.Equal(aExpected, aCurvature, 1e-6);
    }

    [Fact]
    public void SetCurve()
    {
        var aProps = new CLProps2d(myEllipse, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        aProps.SetCurve(myCircle);
        aProps.SetParameter(0.0);

        Point2D aPnt = aProps.Value();
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D1_Circle()
    {
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aD1 = aProps.D1();

        // First derivative at U=0 on circle(R=5) is (0, 5)
        Assert.Equal(0.0, aD1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aD1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D2_Circle()
    {
        var aProps = new CLProps2d(myCircle, 0.0, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aD2 = aProps.D2();

        // Second derivative at U=0 on circle(R=5) is (-5, 0)
        Assert.Equal(-5.0, aD2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD2.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
