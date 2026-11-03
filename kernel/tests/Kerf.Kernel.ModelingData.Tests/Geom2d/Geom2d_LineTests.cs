// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Line_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_LineTest
{
    // Horizontal line through origin
    private readonly LineCurve2D myLine = new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));

    [Fact]
    public void ConstructFromPointAndDir()
    {
        Assert.Equal(1.0, myLine.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, myLine.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, myLine.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, myLine.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromLin2d()
    {
        Line2D aLin = new Line2D(new Point2D(1.0, 2.0), new Direction2D(0.0, 1.0));
        LineCurve2D aLine = new LineCurve2D(aLin);
        Assert.Equal(0.0, aLine.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aLine.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromAxis()
    {
        Axis2D anAxis = new Axis2D(new Point2D(3.0, 4.0), new Direction2D(1.0, 0.0));
        LineCurve2D aLine = new LineCurve2D(anAxis);
        Assert.Equal(3.0, aLine.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
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
        Assert.False(myLine.IsClosed);
        Assert.False(myLine.IsPeriodic);
    }

    [Fact]
    public void Continuity()
    {
        Assert.Equal(CN, myLine.Continuity());
        Assert.True(myLine.IsCN(100));
    }

    [Fact]
    public void EvalD0()
    {
        Point2D aPnt = myLine.EvalD0(5.0);
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_DirectionIsConstant()
    {
        Curve2D.ResD1 aR = myLine.EvalD1(5.0);
        Assert.Equal(1.0, aR.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aR.D1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD2_IsZero()
    {
        Curve2D.ResD2 aR = myLine.EvalD2(5.0);
        Assert.Equal(0.0, aR.D2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD3_IsZero()
    {
        Curve2D.ResD3 aR = myLine.EvalD3(5.0);
        Assert.Equal(0.0, aR.D3.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalDN_HigherOrder()
    {
        Vector2D aD4 = myLine.EvalDN(0.0, 4);
        Assert.Equal(0.0, aD4.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Distance()
    {
        Assert.Equal(3.0, myLine.Distance(new Point2D(5.0, 3.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, myLine.Distance(new Point2D(0.0, 0.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetDirection()
    {
        myLine.SetDirection(new Direction2D(0.0, 1.0));
        Assert.Equal(0.0, myLine.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, myLine.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetLocation()
    {
        myLine.SetLocation(new Point2D(3.0, 4.0));
        Assert.Equal(3.0, myLine.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, myLine.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Lin2d()
    {
        Line2D aLin = myLine.Lin2d();
        Assert.Equal(1.0, aLin.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aLin.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geometry2D aCopyGeom = myLine.Copy();
        LineCurve2D? aCopy = aCopyGeom as LineCurve2D;
        Assert.NotNull(aCopy);
        Assert.Equal(1.0, aCopy.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aCopy.SetDirection(new Direction2D(0.0, 1.0));
        Assert.Equal(1.0, myLine.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Transform2D aTrsf = new Transform2D();
        aTrsf.SetTranslation(new Vector2D(5.0, 10.0));
        myLine.Transform(aTrsf);
        Assert.Equal(5.0, myLine.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, myLine.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        Transform2D aTrsf = new Transform2D();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI / 2.0);
        myLine.Transform(aTrsf);
        // Horizontal line rotated 90 degrees becomes vertical
        Assert.Equal(0.0, myLine.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, Math.Abs(myLine.Direction().Y()), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ReversedParameter()
    {
        Assert.Equal(-5.0, myLine.ReversedParameter(5.0));
    }

    [Fact]
    public void DiagonalLine_Value()
    {
        LineCurve2D aDiag = new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 1.0));
        Point2D aPnt = aDiag.EvalD0(Math.Sqrt(2.0));
        Assert.Equal(1.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
