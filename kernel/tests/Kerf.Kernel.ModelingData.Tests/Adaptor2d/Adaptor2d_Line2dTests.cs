// Port of: src/ModelingData/TKG2d/GTests/Adaptor2d_Line2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

public class Adaptor2d_Line2dTest
{
    private readonly Line2d myLine;
    private readonly Line2d myDiagLine;

    public Adaptor2d_Line2dTest()
    {
        // Line through origin along direction (1, 0) with bounds [0, 10]
        myLine = new Line2d(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0), 0.0, 10.0);

        // Line through (1, 2) along direction (3, 4)/5 with bounds [-5, 5]
        myDiagLine =
          new Line2d(new Point2D(1.0, 2.0), new Direction2D(3.0 / 5.0, 4.0 / 5.0), -5.0, 5.0);
    }

    [Fact]
    public void DefaultConstructor()
    {
        Line2d aLine = new Line2d();
        Assert.Equal(Line, aLine.GetType());
    }

    [Fact]
    public void ParameterBounds()
    {
        Assert.Equal(0.0, myLine.FirstParameter());
        Assert.Equal(10.0, myLine.LastParameter());
        Assert.Equal(-5.0, myDiagLine.FirstParameter());
        Assert.Equal(5.0, myDiagLine.LastParameter());
    }

    [Fact]
    public void Continuity()
    {
        Assert.Equal(CN, myLine.Continuity());
    }

    [Fact]
    public void NbIntervals()
    {
        Assert.Equal(1, myLine.NbIntervals(C0));
        Assert.Equal(1, myLine.NbIntervals(CN));
    }

    [Fact]
    public void Intervals()
    {
        NCollection_Array1<double> anIntervals = new(1, 2);
        myLine.Intervals(anIntervals, CN);
        Assert.Equal(0.0, anIntervals[1]);
        Assert.Equal(10.0, anIntervals[2]);
    }

    [Fact]
    public void IsNotClosed()
    {
        Assert.False(myLine.IsClosed);
    }

    [Fact]
    public void IsNotPeriodic()
    {
        Assert.False(myLine.IsPeriodic);
    }

    [Fact]
    public void GetType_() // GetType in OCCT; renamed, as it would hide object.GetType
    {
        Assert.Equal(Line, myLine.GetType());
    }

    [Fact]
    public void Value_AtOrigin()
    {
        Point2D aPnt = myLine.Value(0.0);
        Assert.Equal(0.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Value_AtParameter()
    {
        Point2D aPnt = myLine.Value(5.0);
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Value_DiagonalLine()
    {
        // At U=0, point should be origin (1, 2)
        Point2D aPnt0 = myDiagLine.Value(0.0);
        Assert.Equal(1.0, aPnt0.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aPnt0.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // At U=5, point should be (1 + 3, 2 + 4) = (4, 6)
        Point2D aPnt5 = myDiagLine.Value(5.0);
        Assert.Equal(4.0, aPnt5.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(6.0, aPnt5.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D0()
    {
        Point2D aPnt = new();
        myLine.D0(3.0, out aPnt);
        Assert.Equal(3.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D1_FirstDerivativeIsDirection()
    {
        Point2D aPnt = new();
        Vector2D aVec = new();
        myLine.D1(5.0, out aPnt, out aVec);

        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aVec.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aVec.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D2_SecondDerivativeIsZero()
    {
        Point2D aPnt = new();
        Vector2D aV1, aV2;
        myLine.D2(5.0, out aPnt, out aV1, out aV2);

        Assert.Equal(0.0, aV2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D3_ThirdDerivativeIsZero()
    {
        Point2D aPnt = new();
        Vector2D aV1, aV2, aV3;
        myLine.D3(5.0, out aPnt, out aV1, out aV2, out aV3);

        Assert.Equal(0.0, aV3.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DN_FirstOrderMatchesD1()
    {
        Vector2D aDN1 = myLine.DN(5.0, 1);
        Assert.Equal(1.0, aDN1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDN1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DN_HigherOrderIsZero()
    {
        Vector2D aDN2 = myLine.DN(5.0, 2);
        Assert.Equal(0.0, aDN2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aDN3 = myLine.DN(5.0, 3);
        Assert.Equal(0.0, aDN3.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Resolution()
    {
        // For a unit-speed line, resolution should equal the input tolerance
        double aResolution = myLine.Resolution(0.001);
        Assert.Equal(0.001, aResolution, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void LineGeometry()
    {
        Line2D aLin = myLine.Line();
        Assert.Equal(1.0, aLin.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aLin.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aLin.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aLin.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Load_WithLine()
    {
        Line2d aLine = new Line2d();
        Line2D         aLin = new(new Point2D(1.0, 1.0), new Direction2D(0.0, 1.0));
        aLine.Load(aLin);

        Point2D aPnt = aLine.Value(3.0);
        Assert.Equal(1.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Load_WithBounds()
    {
        Line2d aLine = new Line2d();
        Line2D         aLin = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        aLine.Load(aLin, 2.0, 8.0);

        Assert.Equal(2.0, aLine.FirstParameter());
        Assert.Equal(8.0, aLine.LastParameter());
    }

    [Fact]
    public void Trim()
    {
        Curve2DAdaptorBase aTrimmed = myLine.Trim(2.0, 7.0, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aTrimmed.FirstParameter());
        Assert.Equal(7.0, aTrimmed.LastParameter());

        Point2D aPnt = aTrimmed.Value(2.0);
        Assert.Equal(2.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ShallowCopy()
    {
        Curve2DAdaptorBase aCopy = myLine.ShallowCopy();
        Assert.Equal(0.0, aCopy.FirstParameter());
        Assert.Equal(10.0, aCopy.LastParameter());
        Assert.Equal(Line, aCopy.GetType());

        Point2D aPnt = aCopy.Value(5.0);
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
