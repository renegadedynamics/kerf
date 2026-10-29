// Port of: src/FoundationClasses/TKMath/GTests/BVH_Box_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class BVH_BoxTest
{
    [Fact]
    public void DefaultConstructor()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new();
        Assert.False(aBox.IsValid());
    }

    [Fact]
    public void ConstructorWithCorners()
    {
        BVH_Vec3d aMin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aMax = new(1.0, 2.0, 3.0);

        BVH_Box<double, BVH_Vec3d> aBox = new(aMin, aMax);

        Assert.True(aBox.IsValid());
        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(2.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(3.0, aBox.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void Add()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new();

        aBox.Add(new BVH_Vec3d(1.0, 2.0, 3.0));
        Assert.True(aBox.IsValid());

        aBox.Add(new BVH_Vec3d(-1.0, -2.0, -3.0));

        Assert.Equal(-1.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(-2.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(-3.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(2.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(3.0, aBox.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void Combine()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(3.0, 3.0, 3.0));

        aBox1.Combine(aBox2);

        Assert.Equal(0.0, aBox1.CornerMin().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox1.CornerMin().y(), Precision.Confusion());
        Assert.Equal(0.0, aBox1.CornerMin().z(), Precision.Confusion());
        Assert.Equal(3.0, aBox1.CornerMax().x(), Precision.Confusion());
        Assert.Equal(3.0, aBox1.CornerMax().y(), Precision.Confusion());
        Assert.Equal(3.0, aBox1.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void Size()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(2.0, 3.0, 4.0));

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(2.0, aSize.x(), Precision.Confusion());
        Assert.Equal(3.0, aSize.y(), Precision.Confusion());
        Assert.Equal(4.0, aSize.z(), Precision.Confusion());
    }

    [Fact]
    public void Center()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(2.0, 4.0, 6.0));

        BVH_Vec3d aCenter = aBox.Center();
        Assert.Equal(1.0, aCenter.x(), Precision.Confusion());
        Assert.Equal(2.0, aCenter.y(), Precision.Confusion());
        Assert.Equal(3.0, aCenter.z(), Precision.Confusion());
    }

    [Fact]
    public void Area()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 2.0, 3.0));

        // Surface area = 2 * (1*2 + 2*3 + 1*3) = 2 * (2 + 6 + 3) = 22
        double anArea = aBox.Area();
        Assert.Equal(22.0, anArea, Precision.Confusion());
    }

    [Fact]
    public void IsOut()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(3.0, 3.0, 3.0));
        BVH_Box<double, BVH_Vec3d> aBox3 = new(new BVH_Vec3d(0.5, 0.5, 0.5), new BVH_Vec3d(1.5, 1.5, 1.5));

        Assert.True(aBox1.IsOut(aBox2));
        Assert.False(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void Clear()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        Assert.True(aBox.IsValid());

        aBox.Clear();
        Assert.False(aBox.IsValid());
    }

    [Fact]
    public void Box2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new();

        aBox.Add(new BVH_Vec2d(0.0, 0.0));
        aBox.Add(new BVH_Vec2d(1.0, 1.0));

        Assert.True(aBox.IsValid());
        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().y(), Precision.Confusion());

        // Area in 2D is width * height (actual area)
        double anArea = aBox.Area();
        Assert.Equal(1.0, anArea, Precision.Confusion()); // 1 * 1 = 1
    }

    [Fact]
    public void Box4D()
    {
        BVH_Box<double, BVH_Vec4d> aBox = new();

        aBox.Add(new BVH_Vec4d(0.0, 0.0, 0.0, 0.0));
        aBox.Add(new BVH_Vec4d(1.0, 2.0, 3.0, 4.0));

        Assert.True(aBox.IsValid());
        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().w(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(2.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(3.0, aBox.CornerMax().z(), Precision.Confusion());
        Assert.Equal(4.0, aBox.CornerMax().w(), Precision.Confusion());
    }

    [Fact]
    public void FloatPrecision()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new();

        aBox.Add(new BVH_Vec3f(0.0f, 0.0f, 0.0f));
        aBox.Add(new BVH_Vec3f(1.0f, 2.0f, 3.0f));

        Assert.True(aBox.IsValid());
        Assert.Equal(0.0f, aBox.CornerMin().x(), 1e-5f);
        Assert.Equal(1.0f, aBox.CornerMax().x(), 1e-5f);
        Assert.Equal(2.0f, aBox.CornerMax().y(), 1e-5f);
        Assert.Equal(3.0f, aBox.CornerMax().z(), 1e-5f);
    }

    [Fact]
    public void SinglePointBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new();

        aBox.Add(new BVH_Vec3d(5.0, 5.0, 5.0));

        Assert.True(aBox.IsValid());
        Assert.Equal(5.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(5.0, aBox.CornerMax().x(), Precision.Confusion());

        // Size should be zero
        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(0.0, aSize.x(), Precision.Confusion());
        Assert.Equal(0.0, aSize.y(), Precision.Confusion());
        Assert.Equal(0.0, aSize.z(), Precision.Confusion());

        // Area should be zero
        Assert.Equal(0.0, aBox.Area(), Precision.Confusion());
    }

    [Fact]
    public void NegativeCoordinates()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-10.0, -20.0, -30.0), new BVH_Vec3d(-5.0, -10.0, -15.0));

        Assert.True(aBox.IsValid());

        BVH_Vec3d aCenter = aBox.Center();
        Assert.Equal(-7.5, aCenter.x(), Precision.Confusion());
        Assert.Equal(-15.0, aCenter.y(), Precision.Confusion());
        Assert.Equal(-22.5, aCenter.z(), Precision.Confusion());

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(5.0, aSize.x(), Precision.Confusion());
        Assert.Equal(10.0, aSize.y(), Precision.Confusion());
        Assert.Equal(15.0, aSize.z(), Precision.Confusion());
    }

    [Fact]
    public void LargeValues()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(1e10, 1e10, 1e10),
                                                        new BVH_Vec3d(1e10 + 1.0, 1e10 + 2.0, 1e10 + 3.0));

        Assert.True(aBox.IsValid());

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(1.0, aSize.x(), 1e-5);
        Assert.Equal(2.0, aSize.y(), 1e-5);
        Assert.Equal(3.0, aSize.z(), 1e-5);
    }

    [Fact]
    public void CombineWithInvalid()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(); // Invalid box

        aBox1.Combine(aBox2);

        // Box1 should remain unchanged when combining with invalid box
        Assert.True(aBox1.IsValid());
        Assert.Equal(0.0, aBox1.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox1.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void AddToInvalid()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(); // Invalid box
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        aBox1.Combine(aBox2);

        // Box1 should now be valid and equal to aBox2
        Assert.True(aBox1.IsValid());
        Assert.Equal(0.0, aBox1.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox1.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void IsOutTouchingBoxes()
    {
        // Boxes that touch at a face
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(1.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0));

        // Touching boxes should NOT be "out"
        Assert.False(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void IsOutTouchingAtEdge()
    {
        // Boxes that touch at an edge
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(1.0, 1.0, 0.0), new BVH_Vec3d(2.0, 2.0, 1.0));

        // Touching at edge should NOT be "out"
        Assert.False(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void IsOutTouchingAtCorner()
    {
        // Boxes that touch at a corner
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(1.0, 1.0, 1.0), new BVH_Vec3d(2.0, 2.0, 2.0));

        // Touching at corner should NOT be "out"
        Assert.False(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void AreaUnitCube()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Surface area of unit cube = 6 faces * 1 = 6
        // Formula: 2 * (xy + yz + xz) = 2 * (1 + 1 + 1) = 6
        Assert.Equal(6.0, aBox.Area(), Precision.Confusion());
    }

    [Fact]
    public void CenterAtOrigin()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-1.0, -1.0, -1.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        BVH_Vec3d aCenter = aBox.Center();
        Assert.Equal(0.0, aCenter.x(), Precision.Confusion());
        Assert.Equal(0.0, aCenter.y(), Precision.Confusion());
        Assert.Equal(0.0, aCenter.z(), Precision.Confusion());
    }

    [Fact]
    public void MultipleAdds()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new();

        // Add points in random order
        aBox.Add(new BVH_Vec3d(5.0, 3.0, 1.0));
        aBox.Add(new BVH_Vec3d(-2.0, 7.0, 4.0));
        aBox.Add(new BVH_Vec3d(1.0, -1.0, 8.0));
        aBox.Add(new BVH_Vec3d(0.0, 2.0, -3.0));

        Assert.Equal(-2.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(-1.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(-3.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(5.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(7.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(8.0, aBox.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void FlatBox2D()
    {
        // Box with zero extent in Z (flat)
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 5.0), new BVH_Vec3d(3.0, 4.0, 5.0));

        Assert.True(aBox.IsValid());

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(3.0, aSize.x(), Precision.Confusion());
        Assert.Equal(4.0, aSize.y(), Precision.Confusion());
        Assert.Equal(0.0, aSize.z(), Precision.Confusion());

        // Area = 2 * (3*4 + 4*0 + 3*0) = 24
        Assert.Equal(24.0, aBox.Area(), Precision.Confusion());
    }

    [Fact]
    public void FlatBox1D()
    {
        // Box with zero extent in Y and Z (line)
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 5.0, 5.0), new BVH_Vec3d(10.0, 5.0, 5.0));

        Assert.True(aBox.IsValid());

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(10.0, aSize.x(), Precision.Confusion());
        Assert.Equal(0.0, aSize.y(), Precision.Confusion());
        Assert.Equal(0.0, aSize.z(), Precision.Confusion());

        // For degenerate box (line), Area returns length of longest dimension
        // This is an implementation detail - the surface area formula gives 0,
        // but Area() returns the length (10) for degenerate cases
        double anArea = aBox.Area();
        Assert.True(anArea >= 0.0);
    }

    [Fact]
    public void CombineMultiple()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox3 = new(new BVH_Vec3d(0.0, 2.0, 0.0), new BVH_Vec3d(1.0, 3.0, 1.0));

        aBox1.Combine(aBox2);
        aBox1.Combine(aBox3);

        Assert.Equal(0.0, aBox1.CornerMin().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox1.CornerMin().y(), Precision.Confusion());
        Assert.Equal(0.0, aBox1.CornerMin().z(), Precision.Confusion());
        Assert.Equal(3.0, aBox1.CornerMax().x(), Precision.Confusion());
        Assert.Equal(3.0, aBox1.CornerMax().y(), Precision.Confusion());
        Assert.Equal(1.0, aBox1.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void IsOutPartialOverlap()
    {
        // Boxes that partially overlap
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(2.0, 2.0, 2.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(1.0, 1.0, 1.0), new BVH_Vec3d(3.0, 3.0, 3.0));

        Assert.False(aBox1.IsOut(aBox2));
        Assert.False(aBox2.IsOut(aBox1));
    }

    [Fact]
    public void IsOutContained()
    {
        // One box completely inside another
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(3.0, 3.0, 3.0));

        Assert.False(aBox1.IsOut(aBox2));
        Assert.False(aBox2.IsOut(aBox1));
    }

    [Fact]
    public void IsOutFloat()
    {
        BVH_Box<float, BVH_Vec3f> aBox1 = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));
        BVH_Box<float, BVH_Vec3f> aBox2 = new(new BVH_Vec3f(2.0f, 2.0f, 2.0f), new BVH_Vec3f(3.0f, 3.0f, 3.0f));
        BVH_Box<float, BVH_Vec3f> aBox3 = new(new BVH_Vec3f(0.5f, 0.5f, 0.5f), new BVH_Vec3f(1.5f, 1.5f, 1.5f));

        Assert.True(aBox1.IsOut(aBox2));
        Assert.False(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void Box2DIsOut()
    {
        BVH_Box<double, BVH_Vec2d> aBox1 = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0));
        BVH_Box<double, BVH_Vec2d> aBox2 = new(new BVH_Vec2d(2.0, 2.0), new BVH_Vec2d(3.0, 3.0));
        BVH_Box<double, BVH_Vec2d> aBox3 = new(new BVH_Vec2d(0.5, 0.5), new BVH_Vec2d(1.5, 1.5));

        Assert.True(aBox1.IsOut(aBox2));
        Assert.False(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void Box2DCenter()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(4.0, 6.0));

        BVH_Vec2d aCenter = aBox.Center();
        Assert.Equal(2.0, aCenter.x(), Precision.Confusion());
        Assert.Equal(3.0, aCenter.y(), Precision.Confusion());
    }

    [Fact]
    public void Box2DSize()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(1.0, 2.0), new BVH_Vec2d(4.0, 7.0));

        BVH_Vec2d aSize = aBox.Size();
        Assert.Equal(3.0, aSize.x(), Precision.Confusion());
        Assert.Equal(5.0, aSize.y(), Precision.Confusion());
    }

    [Fact]
    public void VerySmallBox()
    {
        // Test with very small values
        double aSmall = 1e-10;
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(aSmall, aSmall, aSmall));

        Assert.True(aBox.IsValid());

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(aSmall, aSize.x(), 1e-15);
        Assert.Equal(aSmall, aSize.y(), 1e-15);
        Assert.Equal(aSmall, aSize.z(), 1e-15);
    }

    [Fact]
    public void SymmetricBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-5.0, -5.0, -5.0), new BVH_Vec3d(5.0, 5.0, 5.0));

        BVH_Vec3d aCenter = aBox.Center();
        Assert.Equal(0.0, aCenter.x(), Precision.Confusion());
        Assert.Equal(0.0, aCenter.y(), Precision.Confusion());
        Assert.Equal(0.0, aCenter.z(), Precision.Confusion());

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(10.0, aSize.x(), Precision.Confusion());
        Assert.Equal(10.0, aSize.y(), Precision.Confusion());
        Assert.Equal(10.0, aSize.z(), Precision.Confusion());
    }

    [Fact]
    public void NonCubicBox()
    {
        // Box with different dimensions on each axis
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 10.0, 100.0));

        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(1.0, aSize.x(), Precision.Confusion());
        Assert.Equal(10.0, aSize.y(), Precision.Confusion());
        Assert.Equal(100.0, aSize.z(), Precision.Confusion());

        // Area = 2 * (1*10 + 10*100 + 1*100) = 2 * (10 + 1000 + 100) = 2220
        Assert.Equal(2220.0, aBox.Area(), Precision.Confusion());
    }

    [Fact]
    public void ClearAndReuse()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        Assert.True(aBox.IsValid());

        aBox.Clear();
        Assert.False(aBox.IsValid());

        // Reuse the box
        aBox.Add(new BVH_Vec3d(5.0, 5.0, 5.0));
        aBox.Add(new BVH_Vec3d(10.0, 10.0, 10.0));

        Assert.True(aBox.IsValid());
        Assert.Equal(5.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(10.0, aBox.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void IsOutSameBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Box should not be "out" of itself
        Assert.False(aBox.IsOut(aBox));
    }

    [Fact]
    public void CombineSameBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        aBox1.Combine(aBox2);

        // Should remain unchanged
        Assert.Equal(0.0, aBox1.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox1.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void IsOutNearMiss()
    {
        // Boxes that are very close but not touching
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(1.0001, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0));

        Assert.True(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void AddDuplicatePoint()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new();

        aBox.Add(new BVH_Vec3d(1.0, 2.0, 3.0));
        aBox.Add(new BVH_Vec3d(1.0, 2.0, 3.0));
        aBox.Add(new BVH_Vec3d(1.0, 2.0, 3.0));

        // Should still be a single point box
        Assert.Equal(1.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox.Size().x(), Precision.Confusion());
    }

    // =======================================================================================
    // Tests for Single-Point Constructor
    // =======================================================================================

    [Fact]
    public void SinglePointConstructor()
    {
        BVH_Vec3d aPoint = new(5.0, 10.0, 15.0);
        BVH_Box<double, BVH_Vec3d> aBox = new(aPoint);

        Assert.True(aBox.IsValid());
        Assert.Equal(5.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(10.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(15.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(5.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(10.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(15.0, aBox.CornerMax().z(), Precision.Confusion());

        // Size should be zero
        BVH_Vec3d aSize = aBox.Size();
        Assert.Equal(0.0, aSize.x(), Precision.Confusion());
        Assert.Equal(0.0, aSize.y(), Precision.Confusion());
        Assert.Equal(0.0, aSize.z(), Precision.Confusion());
    }

    [Fact]
    public void SinglePointConstructor2D()
    {
        BVH_Vec2d aPoint = new(3.0, 7.0);
        BVH_Box<double, BVH_Vec2d> aBox = new(aPoint);

        Assert.True(aBox.IsValid());
        Assert.Equal(3.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(7.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(3.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(7.0, aBox.CornerMax().y(), Precision.Confusion());
    }

    [Fact]
    public void SinglePointConstructorOrigin()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Box<double, BVH_Vec3d> aBox = new(aOrigin);

        Assert.True(aBox.IsValid());
        Assert.Equal(0.0, aBox.Center().x(), Precision.Confusion());
        Assert.Equal(0.0, aBox.Center().y(), Precision.Confusion());
        Assert.Equal(0.0, aBox.Center().z(), Precision.Confusion());
    }

    // =======================================================================================
    // Tests for Center(axis) method
    // =======================================================================================

    [Fact]
    public void CenterByAxis()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 20.0, 30.0));

        Assert.Equal(5.0, aBox.Center(0), Precision.Confusion());  // X axis
        Assert.Equal(10.0, aBox.Center(1), Precision.Confusion()); // Y axis
        Assert.Equal(15.0, aBox.Center(2), Precision.Confusion()); // Z axis
    }

    [Fact]
    public void CenterByAxis2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(2.0, 4.0), new BVH_Vec2d(8.0, 12.0));

        Assert.Equal(5.0, aBox.Center(0), Precision.Confusion()); // X axis
        Assert.Equal(8.0, aBox.Center(1), Precision.Confusion()); // Y axis
    }

    [Fact]
    public void CenterByAxisNegative()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-10.0, -20.0, -30.0), new BVH_Vec3d(10.0, 20.0, 30.0));

        Assert.Equal(0.0, aBox.Center(0), Precision.Confusion());
        Assert.Equal(0.0, aBox.Center(1), Precision.Confusion());
        Assert.Equal(0.0, aBox.Center(2), Precision.Confusion());
    }

    [Fact]
    public void CenterByAxis4D()
    {
        BVH_Box<double, BVH_Vec4d> aBox = new(new BVH_Vec4d(0.0, 0.0, 0.0, 0.0), new BVH_Vec4d(4.0, 6.0, 8.0, 10.0));

        Assert.Equal(2.0, aBox.Center(0), Precision.Confusion());
        Assert.Equal(3.0, aBox.Center(1), Precision.Confusion());
        Assert.Equal(4.0, aBox.Center(2), Precision.Confusion());
    }

    // =======================================================================================
    // Tests for IsOut(point) method
    // =======================================================================================

    [Fact]
    public void IsOutPoint_Inside()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Point inside
        Assert.False(aBox.IsOut(new BVH_Vec3d(5.0, 5.0, 5.0)));
        Assert.False(aBox.IsOut(new BVH_Vec3d(1.0, 1.0, 1.0)));
        Assert.False(aBox.IsOut(new BVH_Vec3d(9.0, 9.0, 9.0)));
    }

    [Fact]
    public void IsOutPoint_Outside()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Points outside
        Assert.True(aBox.IsOut(new BVH_Vec3d(11.0, 5.0, 5.0)));   // Outside X
        Assert.True(aBox.IsOut(new BVH_Vec3d(5.0, 11.0, 5.0)));   // Outside Y
        Assert.True(aBox.IsOut(new BVH_Vec3d(5.0, 5.0, 11.0)));   // Outside Z
        Assert.True(aBox.IsOut(new BVH_Vec3d(-1.0, 5.0, 5.0)));   // Outside negative X
        Assert.True(aBox.IsOut(new BVH_Vec3d(15.0, 15.0, 15.0))); // Far outside
    }

    [Fact]
    public void IsOutPoint_OnBoundary()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Points on boundary (should be considered inside)
        Assert.False(aBox.IsOut(new BVH_Vec3d(0.0, 5.0, 5.0)));    // On min X face
        Assert.False(aBox.IsOut(new BVH_Vec3d(10.0, 5.0, 5.0)));   // On max X face
        Assert.False(aBox.IsOut(new BVH_Vec3d(5.0, 0.0, 5.0)));    // On min Y face
        Assert.False(aBox.IsOut(new BVH_Vec3d(5.0, 10.0, 5.0)));   // On max Y face
        Assert.False(aBox.IsOut(new BVH_Vec3d(0.0, 0.0, 0.0)));    // At corner
        Assert.False(aBox.IsOut(new BVH_Vec3d(10.0, 10.0, 10.0))); // At opposite corner
    }

    [Fact]
    public void IsOutPoint_InvalidBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(); // Invalid box

        Assert.True(aBox.IsOut(new BVH_Vec3d(0.0, 0.0, 0.0)));
        Assert.True(aBox.IsOut(new BVH_Vec3d(5.0, 5.0, 5.0)));
    }

    [Fact]
    public void IsOutPoint_2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(5.0, 5.0));

        Assert.False(aBox.IsOut(new BVH_Vec2d(2.5, 2.5))); // Inside
        Assert.True(aBox.IsOut(new BVH_Vec2d(6.0, 2.5)));  // Outside X
        Assert.True(aBox.IsOut(new BVH_Vec2d(2.5, 6.0)));  // Outside Y
        Assert.False(aBox.IsOut(new BVH_Vec2d(0.0, 0.0))); // On corner
    }

    [Fact]
    public void IsOutPoint_NegativeCoords()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-5.0, -5.0, -5.0), new BVH_Vec3d(5.0, 5.0, 5.0));

        Assert.False(aBox.IsOut(new BVH_Vec3d(0.0, 0.0, 0.0)));    // At center
        Assert.False(aBox.IsOut(new BVH_Vec3d(-3.0, -3.0, -3.0))); // Inside negative
        Assert.True(aBox.IsOut(new BVH_Vec3d(-6.0, 0.0, 0.0)));    // Outside negative X
    }

    // =======================================================================================
    // Tests for Contains() methods
    // =======================================================================================

    [Fact]
    public void Contains_FullyContained()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(8.0, 8.0, 8.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.True(isContained);
        Assert.True(hasOverlap);
    }

    [Fact]
    public void Contains_NotContained()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(8.0, 8.0, 8.0), new BVH_Vec3d(12.0, 12.0, 12.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.False(isContained);
        Assert.True(hasOverlap); // They overlap but box2 is not fully contained
    }

    [Fact]
    public void Contains_NoOverlap()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(5.0, 5.0, 5.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(10.0, 10.0, 10.0), new BVH_Vec3d(15.0, 15.0, 15.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.False(isContained);
        Assert.False(hasOverlap);
    }

    [Fact]
    public void Contains_SameBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.True(isContained);
        Assert.True(hasOverlap);
    }

    [Fact]
    public void Contains_TouchingBoundary()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(5.0, 5.0, 5.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.True(isContained);
        Assert.True(hasOverlap);
    }

    [Fact]
    public void Contains_InvalidBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(); // Invalid

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.False(isContained);
        Assert.False(hasOverlap);
    }

    [Fact]
    public void Contains_InvalidContainer()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(); // Invalid
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.False(isContained);
        Assert.False(hasOverlap);
    }

    [Fact]
    public void Contains_PartialOverlapOneAxis()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(5.0, 5.0, 5.0), new BVH_Vec3d(15.0, 8.0, 8.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.False(isContained); // Not fully contained (extends in X)
        Assert.True(hasOverlap);
    }

    [Fact]
    public void Contains_2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox1 = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(10.0, 10.0));
        BVH_Box<double, BVH_Vec2d> aBox2 = new(new BVH_Vec2d(2.0, 2.0), new BVH_Vec2d(8.0, 8.0));

        bool hasOverlap  = false;
        bool isContained = aBox1.Contains(aBox2, out hasOverlap);

        Assert.True(isContained);
        Assert.True(hasOverlap);
    }

    [Fact]
    public void ContainsByCorners_FullyContained()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        bool hasOverlap  = false;
        bool isContained = aBox.Contains(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(8.0, 8.0, 8.0), out hasOverlap);

        Assert.True(isContained);
        Assert.True(hasOverlap);
    }

    [Fact]
    public void ContainsByCorners_NotContained()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        bool hasOverlap = false;
        bool isContained =
            aBox.Contains(new BVH_Vec3d(8.0, 8.0, 8.0), new BVH_Vec3d(12.0, 12.0, 12.0), out hasOverlap);

        Assert.False(isContained);
        Assert.True(hasOverlap);
    }

    // =======================================================================================
    // Tests for Transform() and Transformed() methods
    // =======================================================================================

    [Fact]
    public void Transform_Identity()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        NCollection_Mat4<double> aIdentity = new();
        aIdentity.InitIdentity();

        aBox.Transform(aIdentity);

        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        NCollection_Mat4<double> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<double>(5.0, 10.0, 15.0));

        aBox.Transform(aTransform);

        Assert.Equal(5.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(10.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(15.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(6.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(11.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(16.0, aBox.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Scale()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        NCollection_Mat4<double> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetValue(0, 0, 2.0); // Scale X by 2
        aTransform.SetValue(1, 1, 3.0); // Scale Y by 3
        aTransform.SetValue(2, 2, 4.0); // Scale Z by 4

        aBox.Transform(aTransform);

        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(2.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(3.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(4.0, aBox.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void Transformed_Identity()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        NCollection_Mat4<double> aIdentity = new();
        aIdentity.InitIdentity();

        BVH_Box<double, BVH_Vec3d> aTransformed = aBox.Transformed(aIdentity);

        // Original should be unchanged
        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());

        // Transformed should be the same
        Assert.Equal(0.0, aTransformed.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aTransformed.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void Transformed_Translation()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        NCollection_Mat4<double> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<double>(10.0, 20.0, 30.0));

        BVH_Box<double, BVH_Vec3d> aTransformed = aBox.Transformed(aTransform);

        // Original should be unchanged
        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(1.0, aBox.CornerMax().x(), Precision.Confusion());

        // Transformed should be translated
        Assert.Equal(10.0, aTransformed.CornerMin().x(), Precision.Confusion());
        Assert.Equal(20.0, aTransformed.CornerMin().y(), Precision.Confusion());
        Assert.Equal(30.0, aTransformed.CornerMin().z(), Precision.Confusion());
        Assert.Equal(11.0, aTransformed.CornerMax().x(), Precision.Confusion());
        Assert.Equal(21.0, aTransformed.CornerMax().y(), Precision.Confusion());
        Assert.Equal(31.0, aTransformed.CornerMax().z(), Precision.Confusion());
    }

    [Fact]
    public void Transform_InvalidBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(); // Invalid box

        NCollection_Mat4<double> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<double>(10.0, 20.0, 30.0));

        aBox.Transform(aTransform);

        // Should remain invalid
        Assert.False(aBox.IsValid());
    }

    [Fact]
    public void Transformed_InvalidBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(); // Invalid box

        NCollection_Mat4<double> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<double>(10.0, 20.0, 30.0));

        BVH_Box<double, BVH_Vec3d> aTransformed = aBox.Transformed(aTransform);

        // Should remain invalid
        Assert.False(aTransformed.IsValid());
    }

    // =======================================================================================
    // Tests for Corner Modification
    // =======================================================================================

    [Fact]
    public void ModifyCornerMin()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Modify via non-const reference
        aBox.ChangeCornerMin() = new BVH_Vec3d(-5.0, -5.0, -5.0);

        Assert.Equal(-5.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(-5.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(-5.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(10.0, aBox.CornerMax().x(), Precision.Confusion());
    }

    [Fact]
    public void ModifyCornerMax()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Modify via non-const reference
        aBox.ChangeCornerMax() = new BVH_Vec3d(20.0, 30.0, 40.0);

        Assert.Equal(20.0, aBox.CornerMax().x(), Precision.Confusion());
        Assert.Equal(30.0, aBox.CornerMax().y(), Precision.Confusion());
        Assert.Equal(40.0, aBox.CornerMax().z(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().x(), Precision.Confusion());
    }

    [Fact]
    public void ModifyCornerComponents()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Modify individual components
        aBox.ChangeCornerMin()[0] = -1.0;
        aBox.ChangeCornerMin()[1] = -2.0;
        aBox.ChangeCornerMax()[2] = 15.0;

        Assert.Equal(-1.0, aBox.CornerMin().x(), Precision.Confusion());
        Assert.Equal(-2.0, aBox.CornerMin().y(), Precision.Confusion());
        Assert.Equal(0.0, aBox.CornerMin().z(), Precision.Confusion());
        Assert.Equal(15.0, aBox.CornerMax().z(), Precision.Confusion());
    }

    // =======================================================================================
    // Additional Edge Cases
    // =======================================================================================

    [Fact]
    public void Area_DegenerateBox2D()
    {
        // 2D box with zero area should return perimeter
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(5.0, 0.0));

        double anArea = aBox.Area();
        // For degenerate case, returns sum of dimensions
        Assert.True(anArea >= 0.0);
        Assert.Equal(5.0, anArea, Precision.Confusion()); // Width + height = 5 + 0
    }

    [Fact]
    public void IsOut_TwoCorners_Overlapping()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Test with overlapping region
        Assert.False(aBox.IsOut(new BVH_Vec3d(5.0, 5.0, 5.0), new BVH_Vec3d(15.0, 15.0, 15.0)));
    }

    [Fact]
    public void IsOut_TwoCorners_Disjoint()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Test with disjoint region
        Assert.True(aBox.IsOut(new BVH_Vec3d(20.0, 20.0, 20.0), new BVH_Vec3d(30.0, 30.0, 30.0)));
    }

    [Fact]
    public void IsOut_TwoCorners_Contained()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));

        // Test with fully contained region
        Assert.False(aBox.IsOut(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(8.0, 8.0, 8.0)));
    }

    [Fact]
    public void Constexpr_Construction()
    {
        // C++ checks this at compile time (constexpr, static_assert); the port checks at run time
        // Test that constexpr constructors work at compile time
        BVH_Vec3d aMin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aMax = new(1.0, 1.0, 1.0);
        BVH_Box<double, BVH_Vec3d> aBox = new(aMin, aMax);

        Assert.True(aBox.IsValid(), "Constexpr box should be valid");
    }

    [Fact]
    public void Constexpr_SinglePoint()
    {
        // C++ checks this at compile time (constexpr, static_assert); the port checks at run time
        BVH_Vec3d aPoint = new(5.0, 5.0, 5.0);
        BVH_Box<double, BVH_Vec3d> aBox = new(aPoint);

        Assert.True(aBox.IsValid(), "Constexpr single-point box should be valid");
    }

    [Fact]
    public void Constexpr_DefaultInvalid()
    {
        // C++ checks this at compile time (constexpr, static_assert); the port checks at run time
        BVH_Box<double, BVH_Vec3d> aBox = new();

        Assert.True(!aBox.IsValid(), "Default constexpr box should be invalid");
    }

    // =======================================================================================
    // Tests for Transform/Transformed with float precision
    // =======================================================================================

    [Fact]
    public void Transform_Float_Identity()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        NCollection_Mat4<float> aIdentity = new();
        aIdentity.InitIdentity();

        aBox.Transform(aIdentity);

        Assert.Equal(0.0f, aBox.CornerMin().x(), 1e-5f);
        Assert.Equal(1.0f, aBox.CornerMax().x(), 1e-5f);
    }

    [Fact]
    public void Transform_Float_Translation()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        NCollection_Mat4<float> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<float>(5.0f, 10.0f, 15.0f));

        aBox.Transform(aTransform);

        Assert.Equal(5.0f, aBox.CornerMin().x(), 1e-5f);
        Assert.Equal(10.0f, aBox.CornerMin().y(), 1e-5f);
        Assert.Equal(15.0f, aBox.CornerMin().z(), 1e-5f);
        Assert.Equal(6.0f, aBox.CornerMax().x(), 1e-5f);
        Assert.Equal(11.0f, aBox.CornerMax().y(), 1e-5f);
        Assert.Equal(16.0f, aBox.CornerMax().z(), 1e-5f);
    }

    [Fact]
    public void Transform_Float_Scale()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        NCollection_Mat4<float> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetValue(0, 0, 2.0f); // Scale X by 2
        aTransform.SetValue(1, 1, 3.0f); // Scale Y by 3
        aTransform.SetValue(2, 2, 4.0f); // Scale Z by 4

        aBox.Transform(aTransform);

        Assert.Equal(0.0f, aBox.CornerMin().x(), 1e-5f);
        Assert.Equal(2.0f, aBox.CornerMax().x(), 1e-5f);
        Assert.Equal(3.0f, aBox.CornerMax().y(), 1e-5f);
        Assert.Equal(4.0f, aBox.CornerMax().z(), 1e-5f);
    }

    [Fact]
    public void Transformed_Float_Translation()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        NCollection_Mat4<float> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<float>(10.0f, 20.0f, 30.0f));

        BVH_Box<float, BVH_Vec3f> aTransformed = aBox.Transformed(aTransform);

        // Original should be unchanged
        Assert.Equal(0.0f, aBox.CornerMin().x(), 1e-5f);
        Assert.Equal(1.0f, aBox.CornerMax().x(), 1e-5f);

        // Transformed should be translated
        Assert.Equal(10.0f, aTransformed.CornerMin().x(), 1e-5f);
        Assert.Equal(20.0f, aTransformed.CornerMin().y(), 1e-5f);
        Assert.Equal(30.0f, aTransformed.CornerMin().z(), 1e-5f);
        Assert.Equal(11.0f, aTransformed.CornerMax().x(), 1e-5f);
        Assert.Equal(21.0f, aTransformed.CornerMax().y(), 1e-5f);
        Assert.Equal(31.0f, aTransformed.CornerMax().z(), 1e-5f);
    }

    [Fact]
    public void Transform_Float_InvalidBox()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(); // Invalid box

        NCollection_Mat4<float> aTransform = new();
        aTransform.InitIdentity();
        aTransform.SetColumn(3, new NCollection_Vec3<float>(10.0f, 20.0f, 30.0f));

        aBox.Transform(aTransform);

        // Should remain invalid
        Assert.False(aBox.IsValid());
    }
}
