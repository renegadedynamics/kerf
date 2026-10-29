// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Hyperbola_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_HyperbolaTest
{
    // Hyperbola centered at origin, a=5, b=3
    private readonly Geom2d_Hyperbola myHyperbola =
        new Geom2d_Hyperbola(new gp_Hypr2d(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0, 3.0));

    [Fact]
    public void Radii()
    {
        Assert.Equal(5.0, myHyperbola.MajorRadius(), Precision.Confusion());
        Assert.Equal(3.0, myHyperbola.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void IsNotClosedNotPeriodic()
    {
        Assert.False(myHyperbola.IsClosed());
        Assert.False(myHyperbola.IsPeriodic());
    }

    [Fact]
    public void Eccentricity()
    {
        // e = c/a where c = sqrt(a^2 + b^2) = sqrt(25+9) = sqrt(34)
        double anExpected = Math.Sqrt(34.0) / 5.0;
        Assert.Equal(anExpected, myHyperbola.Eccentricity(), Precision.Confusion());
    }

    [Fact]
    public void Focal()
    {
        // 2c = 2*sqrt(a^2 + b^2)
        double anExpected = 2.0 * Math.Sqrt(25.0 + 9.0);
        Assert.Equal(anExpected, myHyperbola.Focal(), Precision.Confusion());
    }

    [Fact]
    public void Foci()
    {
        double c = Math.Sqrt(25.0 + 9.0);
        gp_Pnt2d aF1 = myHyperbola.Focus1();
        gp_Pnt2d aF2 = myHyperbola.Focus2();
        Assert.Equal(c, aF1.X(), Precision.Confusion());
        Assert.Equal(-c, aF2.X(), Precision.Confusion());
    }

    [Fact]
    public void Parameter()
    {
        // Semi-latus rectum = b^2/a = 9/5 = 1.8
        Assert.Equal(1.8, myHyperbola.Parameter(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtZero()
    {
        // At U=0: vertex = (a, 0) = (5, 0) since x = a*cosh(0) = a, y = b*sinh(0) = 0
        gp_Pnt2d aPnt = myHyperbola.EvalD0(0.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Geom2d_Curve.ResD1 aR = myHyperbola.EvalD1(0.0);
        // At U=0: dx/du = a*sinh(0) = 0, dy/du = b*cosh(0) = b = 3
        Assert.Equal(0.0, aR.D1.X(), Precision.Confusion());
        Assert.Equal(3.0, aR.D1.Y(), Precision.Confusion());
    }

    [Fact]
    public void PointOnHyperbola_FociDistanceDifference()
    {
        // For any point on hyperbola: |d(P,F1) - d(P,F2)| = 2a
        gp_Pnt2d aF1 = myHyperbola.Focus1();
        gp_Pnt2d aF2 = myHyperbola.Focus2();
        for (double u = -2.0; u <= 2.0; u += 0.5)
        {
            gp_Pnt2d aPnt = myHyperbola.EvalD0(u);
            double aDiff = Math.Abs(aPnt.Distance(aF1) - aPnt.Distance(aF2));
            Assert.Equal(2.0 * 5.0, aDiff, Precision.Confusion());
        }
    }

    [Fact]
    public void SetMajorRadius()
    {
        myHyperbola.SetMajorRadius(8.0);
        Assert.Equal(8.0, myHyperbola.MajorRadius(), Precision.Confusion());
    }

    [Fact]
    public void SetMinorRadius()
    {
        myHyperbola.SetMinorRadius(4.0);
        Assert.Equal(4.0, myHyperbola.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Geometry aCopyGeom = myHyperbola.Copy();
        Geom2d_Hyperbola? aCopy = aCopyGeom as Geom2d_Hyperbola;
        Assert.NotNull(aCopy);
        Assert.Equal(5.0, aCopy.MajorRadius(), Precision.Confusion());
        Assert.Equal(3.0, aCopy.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(10.0, 20.0));
        myHyperbola.Transform(aTrsf);
        Assert.Equal(10.0, myHyperbola.Location().X(), Precision.Confusion());
        Assert.Equal(20.0, myHyperbola.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Asymptotes()
    {
        gp_Ax2d anAsym1 = myHyperbola.Asymptote1();
        gp_Ax2d anAsym2 = myHyperbola.Asymptote2();
        // Asymptotes pass through center
        Assert.Equal(0.0, anAsym1.Location().X(), Precision.Confusion());
        Assert.Equal(0.0, anAsym2.Location().X(), Precision.Confusion());
    }
}
