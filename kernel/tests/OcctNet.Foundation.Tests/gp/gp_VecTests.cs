// Port of: src/FoundationClasses/TKMath/GTests/gp_Vec_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_VecTest
{
    [Fact]
    public void DefaultConstructor()
    {
        gp_Vec aVec = new();
        Assert.Equal(0.0, aVec.X());
        Assert.Equal(0.0, aVec.Y());
        Assert.Equal(0.0, aVec.Z());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        gp_Vec aVec = new(1.0, 2.0, 3.0);
        Assert.Equal(1.0, aVec.X());
        Assert.Equal(2.0, aVec.Y());
        Assert.Equal(3.0, aVec.Z());
    }

    [Fact]
    public void TwoPointConstructor()
    {
        gp_Pnt aP1 = new(1.0, 2.0, 3.0);
        gp_Pnt aP2 = new(4.0, 6.0, 8.0);
        gp_Vec aVec = new(aP1, aP2);
        Assert.Equal(3.0, aVec.X());
        Assert.Equal(4.0, aVec.Y());
        Assert.Equal(5.0, aVec.Z());
    }

    [Fact]
    public void DirConstructor()
    {
        gp_Dir aDir = new(0.0, 0.0, 1.0);
        gp_Vec aVec = new(aDir);
        Assert.Equal(1.0, aVec.Magnitude(), Precision.Confusion());
        Assert.Equal(1.0, aVec.Z(), Precision.Confusion());
    }

    [Fact]
    public void AddSubtract()
    {
        gp_Vec aV1 = new(1.0, 2.0, 3.0);
        gp_Vec aV2 = new(4.0, 5.0, 6.0);

        gp_Vec aSum = aV1 + aV2;
        Assert.Equal(5.0, aSum.X());
        Assert.Equal(7.0, aSum.Y());
        Assert.Equal(9.0, aSum.Z());

        gp_Vec aDiff = aV2 - aV1;
        Assert.Equal(3.0, aDiff.X());
        Assert.Equal(3.0, aDiff.Y());
        Assert.Equal(3.0, aDiff.Z());
    }

    [Fact]
    public void MultiplyDivide()
    {
        gp_Vec aVec = new(2.0, 4.0, 6.0);

        gp_Vec aScaled = aVec * 3.0;
        Assert.Equal(6.0, aScaled.X());
        Assert.Equal(12.0, aScaled.Y());
        Assert.Equal(18.0, aScaled.Z());

        gp_Vec aDivided = aVec / 2.0;
        Assert.Equal(1.0, aDivided.X());
        Assert.Equal(2.0, aDivided.Y());
        Assert.Equal(3.0, aDivided.Z());
    }

    [Fact]
    public void Dot()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(0.0, 1.0, 0.0);
        Assert.Equal(0.0, aV1.Dot(aV2));

        gp_Vec aV3 = new(1.0, 2.0, 3.0);
        gp_Vec aV4 = new(4.0, 5.0, 6.0);
        // 1*4 + 2*5 + 3*6 = 32
        Assert.Equal(32.0, aV3.Dot(aV4));
    }

    [Fact]
    public void Cross()
    {
        gp_Vec aVx = new(1.0, 0.0, 0.0);
        gp_Vec aVy = new(0.0, 1.0, 0.0);
        gp_Vec aCross = aVx.Crossed(aVy);
        Assert.Equal(0.0, aCross.X(), Precision.Confusion());
        Assert.Equal(0.0, aCross.Y(), Precision.Confusion());
        Assert.Equal(1.0, aCross.Z(), Precision.Confusion());
    }

    [Fact]
    public void CrossMagnitude()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(0.0, 1.0, 0.0);
        Assert.Equal(1.0, aV1.CrossMagnitude(aV2), Precision.Confusion());
    }

    [Fact]
    public void DotCross_TripleScalarProduct()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(0.0, 1.0, 0.0);
        gp_Vec aV3 = new(0.0, 0.0, 1.0);
        // Triple scalar product of orthogonal unit vectors = 1
        Assert.Equal(1.0, aV1.DotCross(aV2, aV3), Precision.Confusion());
    }

    [Fact]
    public void Magnitude()
    {
        gp_Vec aVec = new(3.0, 4.0, 0.0);
        Assert.Equal(5.0, aVec.Magnitude(), Precision.Confusion());
        Assert.Equal(25.0, aVec.SquareMagnitude(), Precision.Confusion());
    }

    [Fact]
    public void Normalize()
    {
        gp_Vec aVec = new(3.0, 4.0, 0.0);
        gp_Vec aNormalized = aVec.Normalized();
        Assert.Equal(1.0, aNormalized.Magnitude(), Precision.Confusion());
        Assert.Equal(0.6, aNormalized.X(), Precision.Confusion());
        Assert.Equal(0.8, aNormalized.Y(), Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(0.0, 1.0, 0.0);
        Assert.Equal(M_PI_2, aV1.Angle(aV2), Precision.Angular());
    }

    [Fact]
    public void IsParallel()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(5.0, 0.0, 0.0);
        Assert.True(aV1.IsParallel(aV2, Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(0.0, 1.0, 0.0);
        Assert.True(aV1.IsNormal(aV2, Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(-3.0, 0.0, 0.0);
        Assert.True(aV1.IsOpposite(aV2, Precision.Angular()));
    }

    [Fact]
    public void Reverse()
    {
        gp_Vec aVec = new(1.0, 2.0, 3.0);
        gp_Vec aReversed = aVec.Reversed();
        Assert.Equal(-1.0, aReversed.X());
        Assert.Equal(-2.0, aReversed.Y());
        Assert.Equal(-3.0, aReversed.Z());
    }

    [Fact]
    public void SetLinearForm()
    {
        gp_Vec aV1 = new(1.0, 0.0, 0.0);
        gp_Vec aV2 = new(0.0, 1.0, 0.0);
        gp_Vec aResult = new();
        aResult.SetLinearForm(2.0, aV1, 3.0, aV2);
        Assert.Equal(2.0, aResult.X(), Precision.Confusion());
        Assert.Equal(3.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), Precision.Confusion());
    }
}
