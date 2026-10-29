// Port of: src/ModelingData/TKG2d/GTests/Adaptor2d_OffsetCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

public class Adaptor2d_OffsetCurveTest
{
    // Horizontal line through origin, bounds [0, 10]
    private readonly Adaptor2d_Line2d myBaseLine =
        new Adaptor2d_Line2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), 0.0, 10.0);

    [Fact]
    public void DefaultConstructor()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve();
        Assert.Equal(0.0, anOffset.Offset());
    }

    [Fact]
    public void ConstructWithCurve_OffsetIsZero()
    {
        // Constructor with only curve sets offset to 0 and bounds to 0,0
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine);
        Assert.Equal(0.0, anOffset.Offset());
        Assert.Equal(0.0, anOffset.FirstParameter());
        Assert.Equal(0.0, anOffset.LastParameter());
    }

    [Fact]
    public void ConstructWithOffset()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Assert.Equal(3.0, anOffset.Offset());
        Assert.Equal(0.0, anOffset.FirstParameter());
        Assert.Equal(10.0, anOffset.LastParameter());
    }

    [Fact]
    public void ConstructWithOffsetAndBounds()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 2.0, 1.0, 8.0);
        Assert.Equal(2.0, anOffset.Offset());
        Assert.Equal(1.0, anOffset.FirstParameter());
        Assert.Equal(8.0, anOffset.LastParameter());
    }

    [Fact]
    public void Value_PositiveOffset()
    {
        // Offsetting a horizontal +X line by +3 shifts Y by -3
        // (OCCT 2D offset uses right-hand normal: Z cross tangent)
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        gp_Pnt2d aPnt = anOffset.Value(5.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(-3.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Value_NegativeOffset()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, -2.0);
        gp_Pnt2d aPnt = anOffset.Value(5.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(2.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Value_ZeroOffset_MatchesBaseCurve()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 0.0);
        for (double u = 0.0; u <= 10.0; u += 2.0)
        {
            gp_Pnt2d aOffPnt = anOffset.Value(u);
            gp_Pnt2d aBasePnt = myBaseLine.Value(u);
            Assert.True(aOffPnt.IsEqual(aBasePnt, Precision.Confusion()));
        }
    }

    [Fact]
    public void D1_OffsetLine()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        anOffset.D1(5.0, out _, out gp_Vec2d aVec);
        // First derivative of an offset line is same as the base line direction
        Assert.Equal(1.0, aVec.X(), Precision.Confusion());
        Assert.Equal(0.0, aVec.Y(), Precision.Confusion());
    }

    [Fact]
    public void Continuity()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        // Offset of CN line is CN
        Assert.Equal(GeomAbs_CN, anOffset.Continuity());
    }

    [Fact]
    public void NbIntervals()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Assert.Equal(1, anOffset.NbIntervals(GeomAbs_CN));
    }

    [Fact]
    public void GetType_Line()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Assert.Equal(GeomAbs_Line, anOffset.GetType());
    }

    [Fact]
    public void IsNotClosed()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Assert.False(anOffset.IsClosed());
    }

    [Fact]
    public void IsNotPeriodic()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Assert.False(anOffset.IsPeriodic());
    }

    [Fact]
    public void LoadCurve_ResetsOffset()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 5.0);
        Assert.Equal(5.0, anOffset.Offset());
        // Load a new curve resets offset to 0
        Adaptor2d_Line2d aNewLine = new Adaptor2d_Line2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(0.0, 1.0), 0.0, 5.0);
        anOffset.Load(aNewLine);
        Assert.Equal(0.0, anOffset.Offset());
    }

    [Fact]
    public void LoadOffset()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine);
        Assert.Equal(0.0, anOffset.Offset());
        anOffset.Load(7.5);
        Assert.Equal(7.5, anOffset.Offset());
    }

    [Fact]
    public void LoadOffsetWithBounds()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine);
        anOffset.Load(4.0, 2.0, 6.0);
        Assert.Equal(4.0, anOffset.Offset());
        Assert.Equal(2.0, anOffset.FirstParameter());
        Assert.Equal(6.0, anOffset.LastParameter());
    }

    [Fact]
    public void Curve_ReturnsBaseCurve()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Adaptor2d_Curve2d? aCurve = anOffset.Curve();
        Assert.NotNull(aCurve);
        Assert.Equal(GeomAbs_Line, aCurve.GetType());
    }

    [Fact]
    public void Trim()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        Adaptor2d_Curve2d aTrimmed = anOffset.Trim(2.0, 7.0, Precision.Confusion());
        Assert.Equal(2.0, aTrimmed.FirstParameter());
        Assert.Equal(7.0, aTrimmed.LastParameter());
    }

    [Fact]
    public void ShallowCopy()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0, 1.0, 9.0);
        Adaptor2d_Curve2d aCopy = anOffset.ShallowCopy();
        Assert.Equal(1.0, aCopy.FirstParameter());
        Assert.Equal(9.0, aCopy.LastParameter());
        gp_Pnt2d aPntOrig = anOffset.Value(5.0);
        gp_Pnt2d aPntCopy = aCopy.Value(5.0);
        Assert.True(aPntOrig.IsEqual(aPntCopy, Precision.Confusion()));
    }

    [Fact]
    public void Resolution()
    {
        Adaptor2d_OffsetCurve anOffset = new Adaptor2d_OffsetCurve(myBaseLine, 3.0);
        double aRes = anOffset.Resolution(0.001);
        Assert.True(aRes > 0.0);
    }
}
