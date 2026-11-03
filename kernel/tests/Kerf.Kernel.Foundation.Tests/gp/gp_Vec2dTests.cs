// Port of: src/FoundationClasses/TKMath/GTests/gp_Vec2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_Vec2dTests
{
    [Fact]
    public void DefaultConstructor()
    {
        Vector2D aVec = new();
        Assert.Equal(0.0, aVec.X);
        Assert.Equal(0.0, aVec.Y);
    }

    [Fact]
    public void CoordinateConstructor()
    {
        Vector2D aVec = new(1.0, 2.0);
        Assert.Equal(1.0, aVec.X);
        Assert.Equal(2.0, aVec.Y);
    }

    [Fact]
    public void TwoPointConstructor()
    {
        Point2D aP1 = new(1.0, 2.0);
        Point2D aP2 = new(4.0, 6.0);
        Vector2D aVec = new(aP1, aP2);
        Assert.Equal(3.0, aVec.X);
        Assert.Equal(4.0, aVec.Y);
    }

    [Fact]
    public void Dir2dConstructor()
    {
        Direction2D aDir = new(0.0, 1.0);
        Vector2D aVec = new(aDir);
        Assert.Equal(1.0, aVec.Magnitude, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aVec.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void AddSubtract()
    {
        Vector2D aV1 = new(1.0, 2.0);
        Vector2D aV2 = new(4.0, 5.0);

        Vector2D aSum = aV1 + aV2;
        Assert.Equal(5.0, aSum.X);
        Assert.Equal(7.0, aSum.Y);

        Vector2D aDiff = aV2 - aV1;
        Assert.Equal(3.0, aDiff.X);
        Assert.Equal(3.0, aDiff.Y);
    }

    [Fact]
    public void MultiplyDivide()
    {
        Vector2D aVec = new(2.0, 4.0);

        Vector2D aScaled = aVec * 3.0;
        Assert.Equal(6.0, aScaled.X);
        Assert.Equal(12.0, aScaled.Y);

        Vector2D aDivided = aVec / 2.0;
        Assert.Equal(1.0, aDivided.X);
        Assert.Equal(2.0, aDivided.Y);
    }

    [Fact]
    public void Dot()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(0.0, 1.0);
        Assert.Equal(0.0, aV1.Dot(aV2));

        Vector2D aV3 = new(1.0, 2.0);
        Vector2D aV4 = new(4.0, 5.0);
        // 1*4 + 2*5 = 14
        Assert.Equal(14.0, aV3.Dot(aV4));
    }

    [Fact]
    public void Crossed()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(0.0, 1.0);
        // V1.X*V2.Y - V1.Y*V2.X = 1*1 - 0*0 = 1
        Assert.Equal(1.0, aV1.Crossed(aV2), global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aV3 = new(3.0, 4.0);
        Vector2D aV4 = new(2.0, 5.0);
        // 3*5 - 4*2 = 15 - 8 = 7
        Assert.Equal(7.0, aV3.Crossed(aV4), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Magnitude()
    {
        Vector2D aVec = new(3.0, 4.0);
        Assert.Equal(5.0, aVec.Magnitude, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(25.0, aVec.SquareMagnitude, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Normalize()
    {
        Vector2D aVec = new(3.0, 4.0);
        Vector2D aNormalized = aVec.Normalized();
        Assert.Equal(1.0, aNormalized.Magnitude, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.6, aNormalized.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.8, aNormalized.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(0.0, 1.0);
        Assert.Equal(M_PI_2, aV1.AngleTo(aV2), global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void IsParallel()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(5.0, 0.0);
        Assert.True(aV1.IsParallel(aV2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(0.0, 1.0);
        Assert.True(aV1.IsNormal(aV2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(-3.0, 0.0);
        Assert.True(aV1.IsOpposite(aV2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Reverse()
    {
        Vector2D aVec = new(1.0, 2.0);
        Vector2D aReversed = aVec.Reversed();
        Assert.Equal(-1.0, aReversed.X);
        Assert.Equal(-2.0, aReversed.Y);
    }

    [Fact]
    public void SetLinearForm()
    {
        Vector2D aV1 = new(1.0, 0.0);
        Vector2D aV2 = new(0.0, 1.0);
        Vector2D aResult = new();
        aResult = Vector2D.LinearForm(2.0, aV1, 3.0, aV2);
        Assert.Equal(2.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void OCC26750_IsNormal_NegativeAngle()
    {
        // OCC26750: gp_Vec2d::IsNormal() returned FALSE when the angle is -PI/2 (not only +PI/2).
        Vector2D aVec1 = new(1.0, 0.0);
        Vector2D aVec2 = new(0.0, -1.0); // -90 degrees
        Assert.True(aVec1.IsNormal(aVec2, global::Kerf.Engine.Foundation.Precision.Angular()), "Vectors at -90 degrees should be recognized as normal");

        // Verify gp_Dir2d as well
        Direction2D aD1 = new(Direction2D.D.X);
        Direction2D aD2 = new(Direction2D.D.NY);
        Assert.True(aD1.IsNormal(aD2, global::Kerf.Engine.Foundation.Precision.Angular()), "Direction X and direction -Y should be recognized as normal");
    }
}
