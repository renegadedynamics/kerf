// Port of: src/FoundationClasses/TKMath/GTests/gp_Pnt2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

public class gp_Pnt2dTests
{
    [Fact]
    public void DefaultConstructor()
    {
        Point2D aPnt = new();
        Assert.Equal(0.0, aPnt.X());
        Assert.Equal(0.0, aPnt.Y());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        Point2D aPnt = new(1.0, 2.0);
        Assert.Equal(1.0, aPnt.X());
        Assert.Equal(2.0, aPnt.Y());
    }

    [Fact]
    public void XYConstructor()
    {
        Xy aXY = new(3.0, 4.0);
        Point2D aPnt = new(aXY);
        Assert.Equal(3.0, aPnt.X());
        Assert.Equal(4.0, aPnt.Y());
    }

    [Fact]
    public void SetCoord()
    {
        Point2D aPnt = new();
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
        Point2D aP1 = new(0.0, 0.0);
        Point2D aP2 = new(3.0, 4.0);
        Assert.Equal(5.0, aP1.Distance(aP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SquareDistance()
    {
        Point2D aP1 = new(1.0, 2.0);
        Point2D aP2 = new(4.0, 6.0);
        // (4-1)^2 + (6-2)^2 = 9 + 16 = 25
        Assert.Equal(25.0, aP1.SquareDistance(aP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsEqual()
    {
        Point2D aP1 = new(1.0, 2.0);
        Point2D aP2 = new(1.0, 2.0);
        Point2D aP3 = new(1.0, 3.0);
        Assert.True(aP1.IsEqual(aP2, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.False(aP1.IsEqual(aP3, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void Translate_ByVec2d()
    {
        Point2D aPnt = new(1.0, 2.0);
        Vector2D aVec = new(10.0, 20.0);
        Point2D aResult = aPnt.Translated(aVec);
        Assert.Equal(11.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(22.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        Point2D aPnt = new(2.0, 4.0);
        Point2D aCenter = new(0.0, 0.0);
        Point2D aResult = aPnt.Scaled(aCenter, 2.0);
        Assert.Equal(4.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Point2D aPnt = new(1.0, 0.0);
        Point2D aCenter = new(0.0, 0.0);
        Point2D aResult = aPnt.Rotated(aCenter, M_PI_2);
        Assert.Equal(0.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror_Point()
    {
        Point2D aPnt = new(1.0, 0.0);
        Point2D aCenter = new(0.0, 0.0);
        Point2D aResult = aPnt.Mirrored(aCenter);
        Assert.Equal(-1.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror_Axis()
    {
        Point2D aPnt = new(1.0, 1.0);
        // Mirror through the X axis
        Axis2D anAxis = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        Point2D aResult = aPnt.Mirrored(anAxis);
        Assert.Equal(1.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-1.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transformed()
    {
        Point2D aPnt = new(1.0, 0.0);
        Transform2D aTrsf = new();
        aTrsf.SetTranslation(new Vector2D(0.0, 5.0));
        Point2D aResult = aPnt.Transformed(aTrsf);
        Assert.Equal(1.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCC22736: Verifies that composing two identical mirror transformations through the same axis
    // is equivalent to the identity transform. Mirror p1 once through the axis to get p2, then
    // mirror p2 to get back p1. Composing M1*M2 and applying to p1 must also return p1.
    [Fact]
    public void OCC22736_TwoIdenticalMirrorCompositionIsIdentity()
    {
        Point2D aMirrorFirst = new(2.0, 1.0);
        Point2D aMirrorSecond = new(3.0, 1.0);
        Axis2D aMirrorAxis = new(aMirrorFirst, new Direction2D(new Vector2D(aMirrorFirst, aMirrorSecond)));

        Point2D aP1 = new(1.0, 0.0);
        Point2D aP2 = new(1.0, 2.0);

        Transform2D aM1 = new();
        aM1.SetMirror(aMirrorAxis);
        Transform2D aM2 = new();
        aM2.SetMirror(aMirrorAxis);

        // Applying M1 to p1 should give p2
        Point2D aP1MirrorM1 = aP1.Transformed(aM1);
        Assert.Equal(aP2.X(), aP1MirrorM1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP2.Y(), aP1MirrorM1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Applying M2 to the mirrored point should give back p1
        Point2D aP1MirrorM1M2 = aP1MirrorM1.Transformed(aM2);
        Assert.Equal(aP1.X(), aP1MirrorM1M2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP1.Y(), aP1MirrorM1M2.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Composing M2*M1 must also be identity on p1
        Transform2D aComp = aM2.Multiplied(aM1);
        Point2D aP1Comp = aP1.Transformed(aComp);
        Assert.Equal(aP1.X(), aP1Comp.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP1.Y(), aP1Comp.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
