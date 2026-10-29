// Port of: src/FoundationClasses/TKMath/GTests/Bnd_B2_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Bnd_B2dTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Bnd_B2d aBox = new();
        Assert.True(aBox.IsVoid());
    }

    [Fact]
    public void ConstructorWithCenterAndHSize()
    {
        gp_XY aCenter = new(5.0, 10.0);
        gp_XY aHSize = new(2.0, 3.0);

        Bnd_B2d aBox = new(aCenter, aHSize);

        Assert.False(aBox.IsVoid());

        gp_XY aMin = aBox.CornerMin();
        gp_XY aMax = aBox.CornerMax();

        Assert.Equal(3.0, aMin.X(), Precision.Confusion());
        Assert.Equal(7.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(7.0, aMax.X(), Precision.Confusion());
        Assert.Equal(13.0, aMax.Y(), Precision.Confusion());
    }

    [Fact]
    public void Clear()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));
        Assert.False(aBox.IsVoid());

        aBox.Clear();
        Assert.True(aBox.IsVoid());
    }

    [Fact]
    public void AddPoint()
    {
        Bnd_B2d aBox = new();

        aBox.Add(new gp_XY(1.0, 2.0));
        Assert.False(aBox.IsVoid());

        gp_XY aMin = aBox.CornerMin();
        gp_XY aMax = aBox.CornerMax();
        Assert.Equal(1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(1.0, aMax.X(), Precision.Confusion());
        Assert.Equal(2.0, aMax.Y(), Precision.Confusion());

        aBox.Add(new gp_XY(4.0, 5.0));
        aMin = aBox.CornerMin();
        aMax = aBox.CornerMax();
        Assert.Equal(1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), Precision.Confusion());
    }

    [Fact]
    public void AddPnt2d()
    {
        Bnd_B2d aBox = new();
        aBox.Add(new gp_Pnt2d(1.0, 2.0));

        Assert.False(aBox.IsVoid());
        gp_XY aMin = aBox.CornerMin();
        Assert.Equal(1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), Precision.Confusion());
    }

    [Fact]
    public void AddBox()
    {
        Bnd_B2d aBox1 = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));
        Bnd_B2d aBox2 = new(new gp_XY(3.0, 3.0), new gp_XY(1.0, 1.0));

        aBox1.Add(aBox2);

        gp_XY aMin = aBox1.CornerMin();
        gp_XY aMax = aBox1.CornerMax();
        Assert.Equal(-1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(-1.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), Precision.Confusion());
        Assert.Equal(4.0, aMax.Y(), Precision.Confusion());
    }

    [Fact]
    public void SquareExtent()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(3.0, 4.0));
        double  aSqExtent = aBox.SquareExtent();

        // Square diagonal = 4 * (3^2 + 4^2) = 4 * 25 = 100
        Assert.Equal(100.0, aSqExtent, Precision.Confusion());
    }

    [Fact]
    public void Enlarge()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));
        aBox.Enlarge(0.5);

        gp_XY aMin = aBox.CornerMin();
        gp_XY aMax = aBox.CornerMax();
        Assert.Equal(-1.5, aMin.X(), Precision.Confusion());
        Assert.Equal(-1.5, aMin.Y(), Precision.Confusion());
        Assert.Equal(1.5, aMax.X(), Precision.Confusion());
        Assert.Equal(1.5, aMax.Y(), Precision.Confusion());
    }

    [Fact]
    public void Limit()
    {
        // Test limiting a large box by a smaller box inside it
        // aBox1: (-5, -5) to (5, 5)
        // aBox2: (0, 0) to (4, 4)
        Bnd_B2d aBox1 = new(new gp_XY(0.0, 0.0), new gp_XY(5.0, 5.0));
        Bnd_B2d aBox2 = new(new gp_XY(2.0, 2.0), new gp_XY(2.0, 2.0));

        bool isLimited = aBox1.Limit(aBox2);
        Assert.True(isLimited);

        // After limiting, aBox1's min corner should align with aBox2's min corner
        gp_XY aMin = aBox1.CornerMin();
        gp_XY aMax = aBox1.CornerMax();
        Assert.Equal(0.0, aMin.X(), Precision.Confusion());
        Assert.Equal(0.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(5.0, aMax.X(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), Precision.Confusion());

        // Test with non-intersecting boxes
        Bnd_B2d aBox3 = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));
        Bnd_B2d aBox4 = new(new gp_XY(10.0, 10.0), new gp_XY(1.0, 1.0));
        bool    isLimited2 = aBox3.Limit(aBox4);
        Assert.False(isLimited2);
    }

    [Fact]
    public void IsOutPoint()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));

        Assert.False(aBox.IsOut(new gp_XY(0.0, 0.0)));
        Assert.False(aBox.IsOut(new gp_XY(0.5, 0.5)));
        Assert.True(aBox.IsOut(new gp_XY(2.0, 0.0)));
        Assert.True(aBox.IsOut(new gp_XY(0.0, 2.0)));
    }

    [Fact]
    public void IsOutCircle()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));

        // Circle at (0, 0) with small radius - should intersect
        Assert.False(aBox.IsOut(new gp_XY(0.0, 0.0), 0.5));

        // Circle far away - should not intersect
        Assert.True(aBox.IsOut(new gp_XY(10.0, 10.0), 1.0));
    }

    [Fact]
    public void IsOutBox()
    {
        Bnd_B2d aBox1 = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));
        Bnd_B2d aBox2 = new(new gp_XY(0.5, 0.5), new gp_XY(1.0, 1.0));
        Bnd_B2d aBox3 = new(new gp_XY(5.0, 5.0), new gp_XY(1.0, 1.0));

        Assert.False(aBox1.IsOut(aBox2));
        Assert.True(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void IsOutLine()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));

        // Line passing through box
        gp_Ax2d aLine1 = new(new gp_Pnt2d(-2.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Assert.False(aBox.IsOut(aLine1));

        // Line not intersecting box
        gp_Ax2d aLine2 = new(new gp_Pnt2d(-2.0, 5.0), new gp_Dir2d(1.0, 0.0));
        Assert.True(aBox.IsOut(aLine2));
    }

    [Fact]
    public void IsOutSegment()
    {
        Bnd_B2d aBox = new(new gp_XY(0.0, 0.0), new gp_XY(1.0, 1.0));

        // Segment intersecting box
        Assert.False(aBox.IsOut(new gp_XY(-2.0, 0.0), new gp_XY(2.0, 0.0)));

        // Segment not intersecting box
        Assert.True(aBox.IsOut(new gp_XY(5.0, 5.0), new gp_XY(6.0, 6.0)));
    }

    [Fact]
    public void IsInBox()
    {
        Bnd_B2d aBox1 = new(new gp_XY(0.0, 0.0), new gp_XY(0.5, 0.5));
        Bnd_B2d aBox2 = new(new gp_XY(0.0, 0.0), new gp_XY(2.0, 2.0));

        Assert.True(aBox1.IsIn(aBox2));
        Assert.False(aBox2.IsIn(aBox1));
    }

    [Fact]
    public void Transformed()
    {
        Bnd_B2d aBox = new(new gp_XY(1.0, 1.0), new gp_XY(1.0, 1.0));

        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(2.0, 3.0));

        Bnd_B2d aTransformedBox = aBox.Transformed(aTrsf);

        gp_XY aMin = aTransformedBox.CornerMin();
        gp_XY aMax = aTransformedBox.CornerMax();

        Assert.Equal(2.0, aMin.X(), Precision.Confusion());
        Assert.Equal(3.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetCenterAndHSize()
    {
        Bnd_B2d aBox = new();

        aBox.SetCenter(new gp_XY(5.0, 10.0));
        aBox.SetHSize(new gp_XY(2.0, 3.0));

        gp_XY aMin = aBox.CornerMin();
        gp_XY aMax = aBox.CornerMax();

        Assert.Equal(3.0, aMin.X(), Precision.Confusion());
        Assert.Equal(7.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(7.0, aMax.X(), Precision.Confusion());
        Assert.Equal(13.0, aMax.Y(), Precision.Confusion());
    }
}

public class Bnd_B2fTest
{
    [Fact]
    public void FloatPrecision()
    {
        Bnd_B2f aBox = new(new gp_XY(1.0f, 2.0f), new gp_XY(0.5f, 0.5f));

        Assert.False(aBox.IsVoid());

        gp_XY aMin = aBox.CornerMin();
        gp_XY aMax = aBox.CornerMax();

        Assert.Equal(0.5, aMin.X(), 1e-5);
        Assert.Equal(1.5, aMin.Y(), 1e-5);
        Assert.Equal(1.5, aMax.X(), 1e-5);
        Assert.Equal(2.5, aMax.Y(), 1e-5);
    }
}
