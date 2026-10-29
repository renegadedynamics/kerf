// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Parabola_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_ParabolaTest
{
    // Parabola centered at origin with focal distance 4
    private readonly Geom2d_Parabola myParabola =
        new Geom2d_Parabola(new gp_Parab2d(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 4.0));

    [Fact]
    public void Focal()
    {
        Assert.Equal(4.0, myParabola.Focal(), Precision.Confusion());
    }

    [Fact]
    public void IsNotClosedNotPeriodic()
    {
        Assert.False(myParabola.IsClosed());
        Assert.False(myParabola.IsPeriodic());
    }

    [Fact]
    public void Eccentricity()
    {
        Assert.Equal(1.0, myParabola.Eccentricity(), Precision.Confusion());
    }

    [Fact]
    public void Focus()
    {
        gp_Pnt2d aFocus = myParabola.Focus();
        Assert.Equal(4.0, aFocus.X(), Precision.Confusion());
        Assert.Equal(0.0, aFocus.Y(), Precision.Confusion());
    }

    [Fact]
    public void Parameter()
    {
        // p = 2*Focal = 8
        Assert.Equal(8.0, myParabola.Parameter(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtZero()
    {
        // At U=0: vertex
        gp_Pnt2d aPnt = myParabola.EvalD0(0.0);
        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Symmetric()
    {
        // Parabola is symmetric: P(U) and P(-U) should have same X, opposite Y
        double aU = 3.0;
        gp_Pnt2d aPntPos = myParabola.EvalD0(aU);
        gp_Pnt2d aPntNeg = myParabola.EvalD0(-aU);
        Assert.Equal(aPntNeg.X(), aPntPos.X(), Precision.Confusion());
        Assert.Equal(-aPntNeg.Y(), aPntPos.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Geom2d_Curve.ResD1 aR = myParabola.EvalD1(0.0);
        // At vertex: tangent is along Y axis
        Assert.Equal(0.0, aR.D1.X(), Precision.Confusion());
        Assert.True(aR.D1.Magnitude() > Precision.Confusion());
    }

    [Fact]
    public void EvalD2_AtZero()
    {
        Geom2d_Curve.ResD2 aR = myParabola.EvalD2(0.0);
        // Second derivative at vertex should be along X (opening direction)
        Assert.True(aR.D2.X() > 0.0);
    }

    [Fact]
    public void PointOnParabola_FocusDirectrixProperty()
    {
        // For any point P on parabola: distance(P, Focus) = distance(P, Directrix)
        gp_Pnt2d aFocus = myParabola.Focus();
        gp_Ax2d aDirectrix = myParabola.Directrix();
        gp_Lin2d aDirLine = new gp_Lin2d(aDirectrix);
        for (double u = -5.0; u <= 5.0; u += 1.0)
        {
            gp_Pnt2d aPnt = myParabola.EvalD0(u);
            double aDistFocus = aPnt.Distance(aFocus);
            double aDistDir = aDirLine.Distance(aPnt);
            Assert.Equal(aDistDir, aDistFocus, 1e-6);
        }
    }

    [Fact]
    public void SetFocal()
    {
        myParabola.SetFocal(10.0);
        Assert.Equal(10.0, myParabola.Focal(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Geometry aCopyGeom = myParabola.Copy();
        Geom2d_Parabola? aCopy = aCopyGeom as Geom2d_Parabola;
        Assert.NotNull(aCopy);
        Assert.Equal(4.0, aCopy.Focal(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(5.0, 10.0));
        myParabola.Transform(aTrsf);
        Assert.Equal(5.0, myParabola.Location().X(), Precision.Confusion());
        Assert.Equal(10.0, myParabola.Location().Y(), Precision.Confusion());
    }
}
