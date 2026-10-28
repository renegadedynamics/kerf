// Port of: src/FoundationClasses/TKMath/GTests/gp_Pnt2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_Pnt2dTests
{
    [Fact]
    public void DefaultConstructor()
    {
        gp_Pnt2d aPnt = new();
        Assert.Equal(0.0, aPnt.X());
        Assert.Equal(0.0, aPnt.Y());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        gp_Pnt2d aPnt = new(1.0, 2.0);
        Assert.Equal(1.0, aPnt.X());
        Assert.Equal(2.0, aPnt.Y());
    }

    [Fact]
    public void XYConstructor()
    {
        gp_XY aXY = new(3.0, 4.0);
        gp_Pnt2d aPnt = new(aXY);
        Assert.Equal(3.0, aPnt.X());
        Assert.Equal(4.0, aPnt.Y());
    }

    [Fact]
    public void SetCoord()
    {
        gp_Pnt2d aPnt = new();
        aPnt.SetX(10.0);
        aPnt.SetY(20.0);
        Assert.Equal(10.0, aPnt.X());
        Assert.Equal(20.0, aPnt.Y());

        aPnt.SetCoord(7.0, 8.0);
        Assert.Equal(7.0, aPnt.X());
        Assert.Equal(8.0, aPnt.Y());
    }

    [Fact]
    public void Distance()
    {
        gp_Pnt2d aP1 = new(0.0, 0.0);
        gp_Pnt2d aP2 = new(3.0, 4.0);
        Assert.Equal(5.0, aP1.Distance(aP2), Precision.Confusion());
    }

    [Fact]
    public void SquareDistance()
    {
        gp_Pnt2d aP1 = new(1.0, 2.0);
        gp_Pnt2d aP2 = new(4.0, 6.0);
        // (4-1)^2 + (6-2)^2 = 9 + 16 = 25
        Assert.Equal(25.0, aP1.SquareDistance(aP2), Precision.Confusion());
    }

    [Fact]
    public void IsEqual()
    {
        gp_Pnt2d aP1 = new(1.0, 2.0);
        gp_Pnt2d aP2 = new(1.0, 2.0);
        gp_Pnt2d aP3 = new(1.0, 3.0);
        Assert.True(aP1.IsEqual(aP2, Precision.Confusion()));
        Assert.False(aP1.IsEqual(aP3, Precision.Confusion()));
    }

    [Fact]
    public void Translate_ByVec2d()
    {
        gp_Pnt2d aPnt = new(1.0, 2.0);
        gp_Vec2d aVec = new(10.0, 20.0);
        gp_Pnt2d aResult = aPnt.Translated(aVec);
        Assert.Equal(11.0, aResult.X(), Precision.Confusion());
        Assert.Equal(22.0, aResult.Y(), Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        gp_Pnt2d aPnt = new(2.0, 4.0);
        gp_Pnt2d aCenter = new(0.0, 0.0);
        gp_Pnt2d aResult = aPnt.Scaled(aCenter, 2.0);
        Assert.Equal(4.0, aResult.X(), Precision.Confusion());
        Assert.Equal(8.0, aResult.Y(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Pnt2d aPnt = new(1.0, 0.0);
        gp_Pnt2d aCenter = new(0.0, 0.0);
        gp_Pnt2d aResult = aPnt.Rotated(aCenter, M_PI_2);
        Assert.Equal(0.0, aResult.X(), Precision.Confusion());
        Assert.Equal(1.0, aResult.Y(), Precision.Confusion());
    }

    [Fact]
    public void Mirror_Point()
    {
        gp_Pnt2d aPnt = new(1.0, 0.0);
        gp_Pnt2d aCenter = new(0.0, 0.0);
        gp_Pnt2d aResult = aPnt.Mirrored(aCenter);
        Assert.Equal(-1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Y(), Precision.Confusion());
    }

    [Fact]
    public void Mirror_Axis()
    {
        gp_Pnt2d aPnt = new(1.0, 1.0);
        // Mirror through the X axis
        gp_Ax2d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        gp_Pnt2d aResult = aPnt.Mirrored(anAxis);
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(-1.0, aResult.Y(), Precision.Confusion());
    }

    [Fact]
    public void Transformed()
    {
        gp_Pnt2d aPnt = new(1.0, 0.0);
        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(0.0, 5.0));
        gp_Pnt2d aResult = aPnt.Transformed(aTrsf);
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(5.0, aResult.Y(), Precision.Confusion());
    }

    // OCC22736: Verifies that composing two identical mirror transformations through the same axis
    // is equivalent to the identity transform. Mirror p1 once through the axis to get p2, then
    // mirror p2 to get back p1. Composing M1*M2 and applying to p1 must also return p1.
    [Fact]
    public void OCC22736_TwoIdenticalMirrorCompositionIsIdentity()
    {
        gp_Pnt2d aMirrorFirst = new(2.0, 1.0);
        gp_Pnt2d aMirrorSecond = new(3.0, 1.0);
        gp_Ax2d aMirrorAxis = new(aMirrorFirst, new gp_Dir2d(new gp_Vec2d(aMirrorFirst, aMirrorSecond)));

        gp_Pnt2d aP1 = new(1.0, 0.0);
        gp_Pnt2d aP2 = new(1.0, 2.0);

        gp_Trsf2d aM1 = new();
        aM1.SetMirror(aMirrorAxis);
        gp_Trsf2d aM2 = new();
        aM2.SetMirror(aMirrorAxis);

        // Applying M1 to p1 should give p2
        gp_Pnt2d aP1MirrorM1 = aP1.Transformed(aM1);
        Assert.Equal(aP2.X(), aP1MirrorM1.X(), Precision.Confusion());
        Assert.Equal(aP2.Y(), aP1MirrorM1.Y(), Precision.Confusion());

        // Applying M2 to the mirrored point should give back p1
        gp_Pnt2d aP1MirrorM1M2 = aP1MirrorM1.Transformed(aM2);
        Assert.Equal(aP1.X(), aP1MirrorM1M2.X(), Precision.Confusion());
        Assert.Equal(aP1.Y(), aP1MirrorM1M2.Y(), Precision.Confusion());

        // Composing M2*M1 must also be identity on p1
        gp_Trsf2d aComp = aM2.Multiplied(aM1);
        gp_Pnt2d aP1Comp = aP1.Transformed(aComp);
        Assert.Equal(aP1.X(), aP1Comp.X(), Precision.Confusion());
        Assert.Equal(aP1.Y(), aP1Comp.Y(), Precision.Confusion());
    }
}
