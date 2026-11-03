// Port of: src/FoundationClasses/TKMath/GTests/Bnd_B2_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Bnd_B2dTest
{
    [Fact]
    public void DefaultConstructor()
    {
        B2d aBox = new();
        Assert.True(aBox.IsVoid);
    }

    [Fact]
    public void ConstructorWithCenterAndHSize()
    {
        Xy aCenter = new(5.0, 10.0);
        Xy aHSize = new(2.0, 3.0);

        B2d aBox = new(aCenter, aHSize);

        Assert.False(aBox.IsVoid);

        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();

        Assert.Equal(3.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(13.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Clear()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));
        Assert.False(aBox.IsVoid);

        aBox.Clear();
        Assert.True(aBox.IsVoid);
    }

    [Fact]
    public void AddPoint()
    {
        B2d aBox = new();

        aBox.Add(new Xy(1.0, 2.0));
        Assert.False(aBox.IsVoid);

        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();
        Assert.Equal(1.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        aBox.Add(new Xy(4.0, 5.0));
        aMin = aBox.CornerMin();
        aMax = aBox.CornerMax();
        Assert.Equal(1.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void AddPnt2d()
    {
        B2d aBox = new();
        aBox.Add(new Point2D(1.0, 2.0));

        Assert.False(aBox.IsVoid);
        Xy aMin = aBox.CornerMin();
        Assert.Equal(1.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void AddBox()
    {
        B2d aBox1 = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));
        B2d aBox2 = new(new Xy(3.0, 3.0), new Xy(1.0, 1.0));

        aBox1.Add(aBox2);

        Xy aMin = aBox1.CornerMin();
        Xy aMax = aBox1.CornerMax();
        Assert.Equal(-1.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-1.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SquareExtent()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(3.0, 4.0));
        double  aSqExtent = aBox.SquareExtent();

        // Square diagonal = 4 * (3^2 + 4^2) = 4 * 25 = 100
        Assert.Equal(100.0, aSqExtent, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Enlarge()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));
        aBox.Enlarge(0.5);

        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();
        Assert.Equal(-1.5, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-1.5, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.5, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.5, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Limit()
    {
        // Test limiting a large box by a smaller box inside it
        // aBox1: (-5, -5) to (5, 5)
        // aBox2: (0, 0) to (4, 4)
        B2d aBox1 = new(new Xy(0.0, 0.0), new Xy(5.0, 5.0));
        B2d aBox2 = new(new Xy(2.0, 2.0), new Xy(2.0, 2.0));

        bool isLimited = aBox1.Limit(aBox2);
        Assert.True(isLimited);

        // After limiting, aBox1's min corner should align with aBox2's min corner
        Xy aMin = aBox1.CornerMin();
        Xy aMax = aBox1.CornerMax();
        Assert.Equal(0.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test with non-intersecting boxes
        B2d aBox3 = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));
        B2d aBox4 = new(new Xy(10.0, 10.0), new Xy(1.0, 1.0));
        bool    isLimited2 = aBox3.Limit(aBox4);
        Assert.False(isLimited2);
    }

    [Fact]
    public void IsOutPoint()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));

        Assert.False(aBox.IsOut(new Xy(0.0, 0.0)));
        Assert.False(aBox.IsOut(new Xy(0.5, 0.5)));
        Assert.True(aBox.IsOut(new Xy(2.0, 0.0)));
        Assert.True(aBox.IsOut(new Xy(0.0, 2.0)));
    }

    [Fact]
    public void IsOutCircle()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));

        // Circle at (0, 0) with small radius - should intersect
        Assert.False(aBox.IsOut(new Xy(0.0, 0.0), 0.5));

        // Circle far away - should not intersect
        Assert.True(aBox.IsOut(new Xy(10.0, 10.0), 1.0));
    }

    [Fact]
    public void IsOutBox()
    {
        B2d aBox1 = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));
        B2d aBox2 = new(new Xy(0.5, 0.5), new Xy(1.0, 1.0));
        B2d aBox3 = new(new Xy(5.0, 5.0), new Xy(1.0, 1.0));

        Assert.False(aBox1.IsOut(aBox2));
        Assert.True(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void IsOutLine()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));

        // Line passing through box
        Axis2D aLine1 = new(new Point2D(-2.0, 0.0), new Direction2D(1.0, 0.0));
        Assert.False(aBox.IsOut(aLine1));

        // Line not intersecting box
        Axis2D aLine2 = new(new Point2D(-2.0, 5.0), new Direction2D(1.0, 0.0));
        Assert.True(aBox.IsOut(aLine2));
    }

    [Fact]
    public void IsOutSegment()
    {
        B2d aBox = new(new Xy(0.0, 0.0), new Xy(1.0, 1.0));

        // Segment intersecting box
        Assert.False(aBox.IsOut(new Xy(-2.0, 0.0), new Xy(2.0, 0.0)));

        // Segment not intersecting box
        Assert.True(aBox.IsOut(new Xy(5.0, 5.0), new Xy(6.0, 6.0)));
    }

    [Fact]
    public void IsInBox()
    {
        B2d aBox1 = new(new Xy(0.0, 0.0), new Xy(0.5, 0.5));
        B2d aBox2 = new(new Xy(0.0, 0.0), new Xy(2.0, 2.0));

        Assert.True(aBox1.IsIn(aBox2));
        Assert.False(aBox2.IsIn(aBox1));
    }

    [Fact]
    public void Transformed()
    {
        B2d aBox = new(new Xy(1.0, 1.0), new Xy(1.0, 1.0));

        Transform2D aTrsf = new();
        aTrsf = aTrsf.WithTranslation(new Vector2D(2.0, 3.0));

        B2d aTransformedBox = aBox.Transformed(aTrsf);

        Xy aMin = aTransformedBox.CornerMin();
        Xy aMax = aTransformedBox.CornerMax();

        Assert.Equal(2.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetCenterAndHSize()
    {
        B2d aBox = new();

        aBox.SetCenter(new Xy(5.0, 10.0));
        aBox.SetHSize(new Xy(2.0, 3.0));

        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();

        Assert.Equal(3.0, aMin.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aMin.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aMax.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(13.0, aMax.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}

public class Bnd_B2fTest
{
    [Fact]
    public void FloatPrecision()
    {
        B2f aBox = new(new Xy(1.0f, 2.0f), new Xy(0.5f, 0.5f));

        Assert.False(aBox.IsVoid);

        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();

        Assert.Equal(0.5, aMin.X(), 1e-5);
        Assert.Equal(1.5, aMin.Y(), 1e-5);
        Assert.Equal(1.5, aMax.X(), 1e-5);
        Assert.Equal(2.5, aMax.Y(), 1e-5);
    }
}
