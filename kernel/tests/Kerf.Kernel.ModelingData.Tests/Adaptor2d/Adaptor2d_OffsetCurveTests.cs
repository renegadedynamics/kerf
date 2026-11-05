// Port of: src/ModelingData/TKG2d/GTests/Adaptor2d_OffsetCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

public class Adaptor2d_OffsetCurveTest
{
    // Horizontal line through origin, bounds [0, 10]
    private readonly Line2d myBaseLine =
        new Line2d(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0), 0.0, 10.0);

    [Fact]
    public void DefaultConstructor()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D();
        Assert.Equal(0.0, anOffset.Offset());
    }

    [Fact]
    public void ConstructWithCurve_OffsetIsZero()
    {
        // Constructor with only curve sets offset to 0 and bounds to 0,0
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine);
        Assert.Equal(0.0, anOffset.Offset());
        Assert.Equal(0.0, anOffset.FirstParameter());
        Assert.Equal(0.0, anOffset.LastParameter());
    }

    [Fact]
    public void ConstructWithOffset()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Assert.Equal(3.0, anOffset.Offset());
        Assert.Equal(0.0, anOffset.FirstParameter());
        Assert.Equal(10.0, anOffset.LastParameter());
    }

    [Fact]
    public void ConstructWithOffsetAndBounds()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 2.0, 1.0, 8.0);
        Assert.Equal(2.0, anOffset.Offset());
        Assert.Equal(1.0, anOffset.FirstParameter());
        Assert.Equal(8.0, anOffset.LastParameter());
    }

    [Fact]
    public void Value_PositiveOffset()
    {
        // Offsetting a horizontal +X line by +3 shifts Y by -3
        // (OCCT 2D offset uses right-hand normal: Z cross tangent)
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Point2D aPnt = anOffset.Value(5.0);
        Assert.Equal(5.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-3.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Value_NegativeOffset()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, -2.0);
        Point2D aPnt = anOffset.Value(5.0);
        Assert.Equal(5.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Value_ZeroOffset_MatchesBaseCurve()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 0.0);
        for (double u = 0.0; u <= 10.0; u += 2.0)
        {
            Point2D aOffPnt = anOffset.Value(u);
            Point2D aBasePnt = myBaseLine.Value(u);
            Assert.True(aOffPnt.IsEqual(aBasePnt, global::Kerf.Engine.Foundation.Precision.Confusion()));
        }
    }

    [Fact]
    public void D1_OffsetLine()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        anOffset.D1(5.0, out _, out Vector2D aVec);
        // First derivative of an offset line is same as the base line direction
        Assert.Equal(1.0, aVec.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aVec.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Continuity()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        // Offset of CN line is CN
        Assert.Equal(CN, anOffset.Continuity());
    }

    [Fact]
    public void NbIntervals()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Assert.Equal(1, anOffset.NbIntervals(CN));
    }

    [Fact]
    public void GetType_Line()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Assert.Equal(Line, anOffset.GetType());
    }

    [Fact]
    public void IsNotClosed()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Assert.False(anOffset.IsClosed);
    }

    [Fact]
    public void IsNotPeriodic()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Assert.False(anOffset.IsPeriodic);
    }

    [Fact]
    public void LoadCurve_ResetsOffset()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 5.0);
        Assert.Equal(5.0, anOffset.Offset());
        // Load a new curve resets offset to 0
        Line2d aNewLine = new Line2d(new Point2D(0.0, 0.0), new Direction2D(0.0, 1.0), 0.0, 5.0);
        anOffset.Load(aNewLine);
        Assert.Equal(0.0, anOffset.Offset());
    }

    [Fact]
    public void LoadOffset()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine);
        Assert.Equal(0.0, anOffset.Offset());
        anOffset.Load(7.5);
        Assert.Equal(7.5, anOffset.Offset());
    }

    [Fact]
    public void LoadOffsetWithBounds()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine);
        anOffset.Load(4.0, 2.0, 6.0);
        Assert.Equal(4.0, anOffset.Offset());
        Assert.Equal(2.0, anOffset.FirstParameter());
        Assert.Equal(6.0, anOffset.LastParameter());
    }

    [Fact]
    public void Curve_ReturnsBaseCurve()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Curve2DAdaptorBase? aCurve = anOffset.Curve();
        Assert.NotNull(aCurve);
        Assert.Equal(Line, aCurve.GetType());
    }

    [Fact]
    public void Trim()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        Curve2DAdaptorBase aTrimmed = anOffset.Trim(2.0, 7.0, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aTrimmed.FirstParameter());
        Assert.Equal(7.0, aTrimmed.LastParameter());
    }

    [Fact]
    public void ShallowCopy()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0, 1.0, 9.0);
        Curve2DAdaptorBase aCopy = anOffset.ShallowCopy();
        Assert.Equal(1.0, aCopy.FirstParameter());
        Assert.Equal(9.0, aCopy.LastParameter());
        Point2D aPntOrig = anOffset.Value(5.0);
        Point2D aPntCopy = aCopy.Value(5.0);
        Assert.True(aPntOrig.IsEqual(aPntCopy, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void Resolution()
    {
        OffsetCurveAdaptor2D anOffset = new OffsetCurveAdaptor2D(myBaseLine, 3.0);
        double aRes = anOffset.Resolution(0.001);
        Assert.True(aRes > 0.0);
    }
}
