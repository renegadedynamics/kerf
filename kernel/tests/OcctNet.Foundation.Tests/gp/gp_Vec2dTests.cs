// Port of: src/FoundationClasses/TKMath/GTests/gp_Vec2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_Vec2dTests
{
    [Fact]
    public void DefaultConstructor()
    {
        gp_Vec2d aVec = new();
        Assert.Equal(0.0, aVec.X());
        Assert.Equal(0.0, aVec.Y());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        gp_Vec2d aVec = new(1.0, 2.0);
        Assert.Equal(1.0, aVec.X());
        Assert.Equal(2.0, aVec.Y());
    }

    [Fact]
    public void TwoPointConstructor()
    {
        gp_Pnt2d aP1 = new(1.0, 2.0);
        gp_Pnt2d aP2 = new(4.0, 6.0);
        gp_Vec2d aVec = new(aP1, aP2);
        Assert.Equal(3.0, aVec.X());
        Assert.Equal(4.0, aVec.Y());
    }

    [Fact]
    public void Dir2dConstructor()
    {
        gp_Dir2d aDir = new(0.0, 1.0);
        gp_Vec2d aVec = new(aDir);
        Assert.Equal(1.0, aVec.Magnitude(), Precision.Confusion());
        Assert.Equal(1.0, aVec.Y(), Precision.Confusion());
    }

    [Fact]
    public void AddSubtract()
    {
        gp_Vec2d aV1 = new(1.0, 2.0);
        gp_Vec2d aV2 = new(4.0, 5.0);

        gp_Vec2d aSum = aV1 + aV2;
        Assert.Equal(5.0, aSum.X());
        Assert.Equal(7.0, aSum.Y());

        gp_Vec2d aDiff = aV2 - aV1;
        Assert.Equal(3.0, aDiff.X());
        Assert.Equal(3.0, aDiff.Y());
    }

    [Fact]
    public void MultiplyDivide()
    {
        gp_Vec2d aVec = new(2.0, 4.0);

        gp_Vec2d aScaled = aVec * 3.0;
        Assert.Equal(6.0, aScaled.X());
        Assert.Equal(12.0, aScaled.Y());

        gp_Vec2d aDivided = aVec / 2.0;
        Assert.Equal(1.0, aDivided.X());
        Assert.Equal(2.0, aDivided.Y());
    }

    [Fact]
    public void Dot()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(0.0, 1.0);
        Assert.Equal(0.0, aV1.Dot(aV2));

        gp_Vec2d aV3 = new(1.0, 2.0);
        gp_Vec2d aV4 = new(4.0, 5.0);
        // 1*4 + 2*5 = 14
        Assert.Equal(14.0, aV3.Dot(aV4));
    }

    [Fact]
    public void Crossed()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(0.0, 1.0);
        // V1.X*V2.Y - V1.Y*V2.X = 1*1 - 0*0 = 1
        Assert.Equal(1.0, aV1.Crossed(aV2), Precision.Confusion());

        gp_Vec2d aV3 = new(3.0, 4.0);
        gp_Vec2d aV4 = new(2.0, 5.0);
        // 3*5 - 4*2 = 15 - 8 = 7
        Assert.Equal(7.0, aV3.Crossed(aV4), Precision.Confusion());
    }

    [Fact]
    public void Magnitude()
    {
        gp_Vec2d aVec = new(3.0, 4.0);
        Assert.Equal(5.0, aVec.Magnitude(), Precision.Confusion());
        Assert.Equal(25.0, aVec.SquareMagnitude(), Precision.Confusion());
    }

    [Fact]
    public void Normalize()
    {
        gp_Vec2d aVec = new(3.0, 4.0);
        gp_Vec2d aNormalized = aVec.Normalized();
        Assert.Equal(1.0, aNormalized.Magnitude(), Precision.Confusion());
        Assert.Equal(0.6, aNormalized.X(), Precision.Confusion());
        Assert.Equal(0.8, aNormalized.Y(), Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(0.0, 1.0);
        Assert.Equal(M_PI_2, aV1.Angle(aV2), Precision.Angular());
    }

    [Fact]
    public void IsParallel()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(5.0, 0.0);
        Assert.True(aV1.IsParallel(aV2, Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(0.0, 1.0);
        Assert.True(aV1.IsNormal(aV2, Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(-3.0, 0.0);
        Assert.True(aV1.IsOpposite(aV2, Precision.Angular()));
    }

    [Fact]
    public void Reverse()
    {
        gp_Vec2d aVec = new(1.0, 2.0);
        gp_Vec2d aReversed = aVec.Reversed();
        Assert.Equal(-1.0, aReversed.X());
        Assert.Equal(-2.0, aReversed.Y());
    }

    [Fact]
    public void SetLinearForm()
    {
        gp_Vec2d aV1 = new(1.0, 0.0);
        gp_Vec2d aV2 = new(0.0, 1.0);
        gp_Vec2d aResult = new();
        aResult.SetLinearForm(2.0, aV1, 3.0, aV2);
        Assert.Equal(2.0, aResult.X(), Precision.Confusion());
        Assert.Equal(3.0, aResult.Y(), Precision.Confusion());
    }

    [Fact]
    public void OCC26750_IsNormal_NegativeAngle()
    {
        // OCC26750: gp_Vec2d::IsNormal() returned FALSE when the angle is -PI/2 (not only +PI/2).
        gp_Vec2d aVec1 = new(1.0, 0.0);
        gp_Vec2d aVec2 = new(0.0, -1.0); // -90 degrees
        Assert.True(aVec1.IsNormal(aVec2, Precision.Angular()), "Vectors at -90 degrees should be recognized as normal");

        // Verify gp_Dir2d as well
        gp_Dir2d aD1 = new(gp_Dir2d.D.X);
        gp_Dir2d aD2 = new(gp_Dir2d.D.NY);
        Assert.True(aD1.IsNormal(aD2, Precision.Angular()), "Direction X and direction -Y should be recognized as normal");
    }
}
