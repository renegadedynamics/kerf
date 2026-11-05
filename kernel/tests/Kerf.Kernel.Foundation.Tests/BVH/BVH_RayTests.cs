// Port of: src/FoundationClasses/TKMath/GTests/BVH_Ray_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;

namespace Kerf.Kernel.Foundation.Tests;

public class BVH_RayTest
{
    // =======================================================================================
    // Tests for BVH_Ray
    // =======================================================================================

    [Fact]
    public void ConstructorBasic()
    {
        BVH_Vec3d aOrigin = new(1.0, 2.0, 3.0);
        BVH_Vec3d aDirection = new(0.0, 1.0, 0.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1.0, aRay.Origin.X());
        Assert.Equal(2.0, aRay.Origin.Y());
        Assert.Equal(3.0, aRay.Origin.Z());

        Assert.Equal(0.0, aRay.Direct.X());
        Assert.Equal(1.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void DirectionStorage()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(2.0, 4.0, 8.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        // Verify direction is stored correctly
        Assert.Equal(2.0, aRay.Direct.X());
        Assert.Equal(4.0, aRay.Direct.Y());
        Assert.Equal(8.0, aRay.Direct.Z());
    }

    [Fact]
    public void NegativeDirection()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(-2.0, -4.0, -1.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(-2.0, aRay.Direct.X());
        Assert.Equal(-4.0, aRay.Direct.Y());
        Assert.Equal(-1.0, aRay.Direct.Z());
    }

    [Fact]
    public void ZeroComponent()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(1.0, 0.0, 1.0); // Zero Y component

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1.0, aRay.Direct.X());
        Assert.Equal(0.0, aRay.Direct.Y());
        Assert.Equal(1.0, aRay.Direct.Z());
    }

    [Fact]
    public void AllZeroDirection()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(0.0, 0.0, 0.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        // All components should be zero
        Assert.Equal(0.0, aRay.Direct.X());
        Assert.Equal(0.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void DefaultConstructor()
    {
        Ray<double, BVH_Vec3d> aRay = new();

        // Default constructed ray should have zero origin and direction
        Assert.Equal(0.0, aRay.Origin.X());
        Assert.Equal(0.0, aRay.Origin.Y());
        Assert.Equal(0.0, aRay.Origin.Z());

        Assert.Equal(0.0, aRay.Direct.X());
        Assert.Equal(0.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void Ray2D()
    {
        BVH_Vec2d aOrigin = new(1.0, 2.0);
        BVH_Vec2d aDirection = new(3.0, 4.0);

        Ray<double, BVH_Vec2d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1.0, aRay.Origin.X());
        Assert.Equal(2.0, aRay.Origin.Y());

        Assert.Equal(3.0, aRay.Direct.X());
        Assert.Equal(4.0, aRay.Direct.Y());
    }

    [Fact]
    public void NormalizedDirection()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(1.0, 0.0, 0.0); // Unit vector along X

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1.0, aRay.Direct.X());
        Assert.Equal(0.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void VerySmallDirection()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(1e-20, 1e-20, 1e-20);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        // Should handle very small values correctly
        Assert.Equal(1e-20, aRay.Direct.X());
        Assert.Equal(1e-20, aRay.Direct.Y());
        Assert.Equal(1e-20, aRay.Direct.Z());
    }

    [Fact]
    public void MixedZeroNonZero()
    {
        BVH_Vec3d aOrigin = new(1.0, 2.0, 3.0);
        BVH_Vec3d aDirection = new(0.0, 2.0, 0.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(0.0, aRay.Direct.X());
        Assert.Equal(2.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void FloatPrecision()
    {
        BVH_Vec3f aOrigin = new(1.0f, 2.0f, 3.0f);
        BVH_Vec3f aDirection = new(2.0f, 4.0f, 8.0f);

        Ray<float, BVH_Vec3f> aRay = new(aOrigin, aDirection);

        Assert.Equal(2.0f, aRay.Direct.X());
        Assert.Equal(4.0f, aRay.Direct.Y());
        Assert.Equal(8.0f, aRay.Direct.Z());
    }

    [Fact]
    public void ConstexprConstructor()
    {
        // C++ checks that the constructor is constexpr; C# has no equivalent, so this checks the values only
        // This test verifies that the constructor is truly constexpr
        BVH_Vec3d aOrigin = new(1.0, 2.0, 3.0);
        BVH_Vec3d aDirection = new(1.0, 1.0, 1.0);
        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        // If this compiles, constexpr works
        Assert.Equal(1.0, aRay.Origin.X());
    }

    [Fact]
    public void DiagonalRay()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(1.0, 1.0, 1.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1.0, aRay.Direct.X());
        Assert.Equal(1.0, aRay.Direct.Y());
        Assert.Equal(1.0, aRay.Direct.Z());
    }

    [Fact]
    public void NegativeOrigin()
    {
        BVH_Vec3d aOrigin = new(-10.0, -20.0, -30.0);
        BVH_Vec3d aDirection = new(1.0, 2.0, 3.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(-10.0, aRay.Origin.X());
        Assert.Equal(-20.0, aRay.Origin.Y());
        Assert.Equal(-30.0, aRay.Origin.Z());

        Assert.Equal(1.0, aRay.Direct.X());
        Assert.Equal(2.0, aRay.Direct.Y());
        Assert.Equal(3.0, aRay.Direct.Z());
    }

    [Fact]
    public void AxisAlignedX()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(1.0, 0.0, 0.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1.0, aRay.Direct.X());
        Assert.Equal(0.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void AxisAlignedY()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(0.0, 1.0, 0.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(0.0, aRay.Direct.X());
        Assert.Equal(1.0, aRay.Direct.Y());
        Assert.Equal(0.0, aRay.Direct.Z());
    }

    [Fact]
    public void AxisAlignedZ()
    {
        BVH_Vec3d aOrigin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aDirection = new(0.0, 0.0, 1.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(0.0, aRay.Direct.X());
        Assert.Equal(0.0, aRay.Direct.Y());
        Assert.Equal(1.0, aRay.Direct.Z());
    }

    [Fact]
    public void LargeValues()
    {
        BVH_Vec3d aOrigin = new(1e6, 2e6, 3e6);
        BVH_Vec3d aDirection = new(1e3, 2e3, 4e3);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        Assert.Equal(1e3, aRay.Direct.X());
        Assert.Equal(2e3, aRay.Direct.Y());
        Assert.Equal(4e3, aRay.Direct.Z());
    }

    [Fact]
    public void OriginAndDirectionPreservation()
    {
        BVH_Vec3d aOrigin = new(1.0, 2.0, 3.0);
        BVH_Vec3d aDirection = new(2.0, 4.0, 8.0);

        Ray<double, BVH_Vec3d> aRay = new(aOrigin, aDirection);

        // Verify that both origin and direction are preserved exactly
        Assert.Equal(aOrigin.X(), aRay.Origin.X());
        Assert.Equal(aOrigin.Y(), aRay.Origin.Y());
        Assert.Equal(aOrigin.Z(), aRay.Origin.Z());

        Assert.Equal(aDirection.X(), aRay.Direct.X());
        Assert.Equal(aDirection.Y(), aRay.Direct.Y());
        Assert.Equal(aDirection.Z(), aRay.Direct.Z());
    }
}
