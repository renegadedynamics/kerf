// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Circle_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_CircleTest
{
    // Circle centered at origin, radius 5
    private readonly Geom2d_Circle myCircle =
        new Geom2d_Circle(new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0));

    [Fact]
    public void ConstructFromCirc2d()
    {
        Assert.Equal(5.0, myCircle.Radius(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromAxisAndRadius()
    {
        Geom2d_Circle aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        Assert.Equal(3.0, aCircle.Radius(), Precision.Confusion());
        Assert.Equal(1.0, aCircle.Location().X(), Precision.Confusion());
        Assert.Equal(2.0, aCircle.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromAx22d()
    {
        gp_Ax22d anAxis = new gp_Ax22d(new gp_Pnt2d(3.0, 4.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle = new Geom2d_Circle(anAxis, 7.0);
        Assert.Equal(7.0, aCircle.Radius(), Precision.Confusion());
        Assert.Equal(3.0, aCircle.Location().X(), Precision.Confusion());
    }

    [Fact]
    public void ParameterBounds()
    {
        Assert.Equal(0.0, myCircle.FirstParameter());
        Assert.Equal(2.0 * M_PI, myCircle.LastParameter());
    }

    [Fact]
    public void IsClosedAndPeriodic()
    {
        Assert.True(myCircle.IsClosed());
        Assert.True(myCircle.IsPeriodic());
    }

    [Fact]
    public void Eccentricity()
    {
        Assert.Equal(0.0, myCircle.Eccentricity(), Precision.Confusion());
    }

    [Fact]
    public void SetRadius()
    {
        myCircle.SetRadius(10.0);
        Assert.Equal(10.0, myCircle.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Circ2d()
    {
        gp_Circ2d aCirc = myCircle.Circ2d();
        Assert.Equal(5.0, aCirc.Radius(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtZero()
    {
        gp_Pnt2d aPnt = myCircle.EvalD0(0.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtPiHalf()
    {
        gp_Pnt2d aPnt = myCircle.EvalD0(M_PI / 2.0);
        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(5.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtPi()
    {
        gp_Pnt2d aPnt = myCircle.EvalD0(M_PI);
        Assert.Equal(-5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Geom2d_Curve.ResD1 aR = myCircle.EvalD1(0.0);
        // Tangent at U=0 is (0, R) = (0, 5)
        Assert.Equal(0.0, aR.D1.X(), Precision.Confusion());
        Assert.Equal(5.0, aR.D1.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD2_AtZero()
    {
        Geom2d_Curve.ResD2 aR = myCircle.EvalD2(0.0);
        // Second derivative at U=0 is (-R, 0) = (-5, 0)
        Assert.Equal(-5.0, aR.D2.X(), Precision.Confusion());
        Assert.Equal(0.0, aR.D2.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalDN_Order1()
    {
        gp_Vec2d aD1 = myCircle.EvalDN(0.0, 1);
        Assert.Equal(0.0, aD1.X(), Precision.Confusion());
        Assert.Equal(5.0, aD1.Y(), Precision.Confusion());
    }

    [Fact]
    public void ReversedParameter()
    {
        double aU = M_PI / 3.0;
        double aRev = myCircle.ReversedParameter(aU);
        Assert.Equal(2.0 * M_PI - aU, aRev, Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Geometry aCopyGeom = myCircle.Copy();
        Geom2d_Circle? aCopy = aCopyGeom as Geom2d_Circle;
        Assert.NotNull(aCopy);
        Assert.Equal(5.0, aCopy.Radius(), Precision.Confusion());
        // Verify independence
        aCopy.SetRadius(1.0);
        Assert.Equal(5.0, myCircle.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(10.0, 20.0));
        myCircle.Transform(aTrsf);
        Assert.Equal(10.0, myCircle.Location().X(), Precision.Confusion());
        Assert.Equal(20.0, myCircle.Location().Y(), Precision.Confusion());
        Assert.Equal(5.0, myCircle.Radius(), Precision.Confusion());
    }

    [Fact]
    public void PointOnCircle_DistanceFromCenter()
    {
        // All points on circle should be at distance R from center
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 6.0)
        {
            gp_Pnt2d aPnt = myCircle.EvalD0(u);
            double aDist = aPnt.Distance(myCircle.Location());
            Assert.Equal(5.0, aDist, Precision.Confusion());
        }
    }
}
