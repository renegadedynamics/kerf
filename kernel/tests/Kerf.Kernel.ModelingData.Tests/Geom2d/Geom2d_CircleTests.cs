// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Circle_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_CircleTest
{
    // Circle centered at origin, radius 5
    private readonly CircleCurve2D myCircle =
        new CircleCurve2D(new Circle2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0));

    [Fact]
    public void ConstructFromCirc2d()
    {
        Assert.Equal(5.0, myCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromAxisAndRadius()
    {
        CircleCurve2D aCircle = new CircleCurve2D(new Axis2D(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0)), 3.0);
        Assert.Equal(3.0, aCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCircle.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aCircle.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromAx22d()
    {
        Frame2D anAxis = new Frame2D(new Point2D(3.0, 4.0), new Direction2D(1.0, 0.0), new Direction2D(0.0, 1.0));
        CircleCurve2D aCircle = new CircleCurve2D(anAxis, 7.0);
        Assert.Equal(7.0, aCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCircle.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ParameterBounds()
    {
        Assert.Equal(0.0, myCircle.FirstParameter);
        Assert.Equal(2.0 * M_PI, myCircle.LastParameter);
    }

    [Fact]
    public void IsClosedAndPeriodic()
    {
        Assert.True(myCircle.IsClosed);
        Assert.True(myCircle.IsPeriodic);
    }

    [Fact]
    public void Eccentricity()
    {
        Assert.Equal(0.0, myCircle.Eccentricity, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetRadius()
    {
        myCircle.SetRadius(10.0);
        Assert.Equal(10.0, myCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circ2d()
    {
        Circle2D aCirc = myCircle.Circle;
        Assert.Equal(5.0, aCirc.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtZero()
    {
        Point2D aPnt = myCircle.EvalD0(0.0);
        Assert.Equal(5.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtPiHalf()
    {
        Point2D aPnt = myCircle.EvalD0(M_PI / 2.0);
        Assert.Equal(0.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtPi()
    {
        Point2D aPnt = myCircle.EvalD0(M_PI);
        Assert.Equal(-5.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Curve2D.ResD1 aR = myCircle.EvalD1(0.0);
        // Tangent at U=0 is (0, R) = (0, 5)
        Assert.Equal(0.0, aR.D1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aR.D1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD2_AtZero()
    {
        Curve2D.ResD2 aR = myCircle.EvalD2(0.0);
        // Second derivative at U=0 is (-R, 0) = (-5, 0)
        Assert.Equal(-5.0, aR.D2.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aR.D2.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalDN_Order1()
    {
        Vector2D aD1 = myCircle.EvalDN(0.0, 1);
        Assert.Equal(0.0, aD1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aD1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ReversedParameter()
    {
        double aU = M_PI / 3.0;
        double aRev = myCircle.ReversedParameter(aU);
        Assert.Equal(2.0 * M_PI - aU, aRev, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geometry2D aCopyGeom = myCircle.Copy();
        CircleCurve2D? aCopy = aCopyGeom as CircleCurve2D;
        Assert.NotNull(aCopy);
        Assert.Equal(5.0, aCopy.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
        // Verify independence
        aCopy.SetRadius(1.0);
        Assert.Equal(5.0, myCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(10.0, 20.0));
        myCircle.Transform(aTrsf);
        Assert.Equal(10.0, myCircle.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(20.0, myCircle.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, myCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PointOnCircle_DistanceFromCenter()
    {
        // All points on circle should be at distance R from center
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 6.0)
        {
            Point2D aPnt = myCircle.EvalD0(u);
            double aDist = aPnt.DistanceTo(myCircle.Location);
            Assert.Equal(5.0, aDist, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }
}
