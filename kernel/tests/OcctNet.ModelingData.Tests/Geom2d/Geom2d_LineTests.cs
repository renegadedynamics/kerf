// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Line_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

public class Geom2d_LineTest
{
    // Horizontal line through origin
    private readonly Geom2d_Line myLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));

    [Fact]
    public void ConstructFromPointAndDir()
    {
        Assert.Equal(1.0, myLine.Direction().X(), Precision.Confusion());
        Assert.Equal(0.0, myLine.Direction().Y(), Precision.Confusion());
        Assert.Equal(0.0, myLine.Location().X(), Precision.Confusion());
        Assert.Equal(0.0, myLine.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromLin2d()
    {
        gp_Lin2d aLin = new gp_Lin2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Line aLine = new Geom2d_Line(aLin);
        Assert.Equal(0.0, aLine.Direction().X(), Precision.Confusion());
        Assert.Equal(1.0, aLine.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromAxis()
    {
        gp_Ax2d anAxis = new gp_Ax2d(new gp_Pnt2d(3.0, 4.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_Line aLine = new Geom2d_Line(anAxis);
        Assert.Equal(3.0, aLine.Location().X(), Precision.Confusion());
    }

    [Fact]
    public void ParameterBounds()
    {
        // OCCT uses large finite bounds (not numeric_limits::max)
        Assert.True(myLine.FirstParameter() < -1e+90);
        Assert.True(myLine.LastParameter() > 1e+90);
    }

    [Fact]
    public void IsNotClosedNotPeriodic()
    {
        Assert.False(myLine.IsClosed());
        Assert.False(myLine.IsPeriodic());
    }

    [Fact]
    public void Continuity()
    {
        Assert.Equal(GeomAbs_CN, myLine.Continuity());
        Assert.True(myLine.IsCN(100));
    }

    [Fact]
    public void EvalD0()
    {
        gp_Pnt2d aPnt = myLine.EvalD0(5.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_DirectionIsConstant()
    {
        Geom2d_Curve.ResD1 aR = myLine.EvalD1(5.0);
        Assert.Equal(1.0, aR.D1.X(), Precision.Confusion());
        Assert.Equal(0.0, aR.D1.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD2_IsZero()
    {
        Geom2d_Curve.ResD2 aR = myLine.EvalD2(5.0);
        Assert.Equal(0.0, aR.D2.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void EvalD3_IsZero()
    {
        Geom2d_Curve.ResD3 aR = myLine.EvalD3(5.0);
        Assert.Equal(0.0, aR.D3.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void EvalDN_HigherOrder()
    {
        gp_Vec2d aD4 = myLine.EvalDN(0.0, 4);
        Assert.Equal(0.0, aD4.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void Distance()
    {
        Assert.Equal(3.0, myLine.Distance(new gp_Pnt2d(5.0, 3.0)), Precision.Confusion());
        Assert.Equal(0.0, myLine.Distance(new gp_Pnt2d(0.0, 0.0)), Precision.Confusion());
    }

    [Fact]
    public void SetDirection()
    {
        myLine.SetDirection(new gp_Dir2d(0.0, 1.0));
        Assert.Equal(0.0, myLine.Direction().X(), Precision.Confusion());
        Assert.Equal(1.0, myLine.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void SetLocation()
    {
        myLine.SetLocation(new gp_Pnt2d(3.0, 4.0));
        Assert.Equal(3.0, myLine.Location().X(), Precision.Confusion());
        Assert.Equal(4.0, myLine.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Lin2d()
    {
        gp_Lin2d aLin = myLine.Lin2d();
        Assert.Equal(1.0, aLin.Direction().X(), Precision.Confusion());
        Assert.Equal(0.0, aLin.Location().X(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Geometry aCopyGeom = myLine.Copy();
        Geom2d_Line? aCopy = aCopyGeom as Geom2d_Line;
        Assert.NotNull(aCopy);
        Assert.Equal(1.0, aCopy.Direction().X(), Precision.Confusion());
        aCopy.SetDirection(new gp_Dir2d(0.0, 1.0));
        Assert.Equal(1.0, myLine.Direction().X(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(5.0, 10.0));
        myLine.Transform(aTrsf);
        Assert.Equal(5.0, myLine.Location().X(), Precision.Confusion());
        Assert.Equal(10.0, myLine.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);
        myLine.Transform(aTrsf);
        // Horizontal line rotated 90 degrees becomes vertical
        Assert.Equal(0.0, myLine.Direction().X(), Precision.Confusion());
        Assert.Equal(1.0, Math.Abs(myLine.Direction().Y()), Precision.Confusion());
    }

    [Fact]
    public void ReversedParameter()
    {
        Assert.Equal(-5.0, myLine.ReversedParameter(5.0));
    }

    [Fact]
    public void DiagonalLine_Value()
    {
        Geom2d_Line aDiag = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 1.0));
        gp_Pnt2d aPnt = aDiag.EvalD0(Math.Sqrt(2.0));
        Assert.Equal(1.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(1.0, aPnt.Y(), Precision.Confusion());
    }
}
