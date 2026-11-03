// Port of: src/FoundationClasses/TKMath/GTests/gp_Vec_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_VecTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Vector3D aVec = new();
        Assert.Equal(0.0, aVec.X());
        Assert.Equal(0.0, aVec.Y());
        Assert.Equal(0.0, aVec.Z());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        Vector3D aVec = new(1.0, 2.0, 3.0);
        Assert.Equal(1.0, aVec.X());
        Assert.Equal(2.0, aVec.Y());
        Assert.Equal(3.0, aVec.Z());
    }

    [Fact]
    public void TwoPointConstructor()
    {
        Point3D aP1 = new(1.0, 2.0, 3.0);
        Point3D aP2 = new(4.0, 6.0, 8.0);
        Vector3D aVec = new(aP1, aP2);
        Assert.Equal(3.0, aVec.X());
        Assert.Equal(4.0, aVec.Y());
        Assert.Equal(5.0, aVec.Z());
    }

    [Fact]
    public void DirConstructor()
    {
        Direction3D aDir = new(0.0, 0.0, 1.0);
        Vector3D aVec = new(aDir);
        Assert.Equal(1.0, aVec.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aVec.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void AddSubtract()
    {
        Vector3D aV1 = new(1.0, 2.0, 3.0);
        Vector3D aV2 = new(4.0, 5.0, 6.0);

        Vector3D aSum = aV1 + aV2;
        Assert.Equal(5.0, aSum.X());
        Assert.Equal(7.0, aSum.Y());
        Assert.Equal(9.0, aSum.Z());

        Vector3D aDiff = aV2 - aV1;
        Assert.Equal(3.0, aDiff.X());
        Assert.Equal(3.0, aDiff.Y());
        Assert.Equal(3.0, aDiff.Z());
    }

    [Fact]
    public void MultiplyDivide()
    {
        Vector3D aVec = new(2.0, 4.0, 6.0);

        Vector3D aScaled = aVec * 3.0;
        Assert.Equal(6.0, aScaled.X());
        Assert.Equal(12.0, aScaled.Y());
        Assert.Equal(18.0, aScaled.Z());

        Vector3D aDivided = aVec / 2.0;
        Assert.Equal(1.0, aDivided.X());
        Assert.Equal(2.0, aDivided.Y());
        Assert.Equal(3.0, aDivided.Z());
    }

    [Fact]
    public void Dot()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(0.0, 1.0, 0.0);
        Assert.Equal(0.0, aV1.Dot(aV2));

        Vector3D aV3 = new(1.0, 2.0, 3.0);
        Vector3D aV4 = new(4.0, 5.0, 6.0);
        // 1*4 + 2*5 + 3*6 = 32
        Assert.Equal(32.0, aV3.Dot(aV4));
    }

    [Fact]
    public void Cross()
    {
        Vector3D aVx = new(1.0, 0.0, 0.0);
        Vector3D aVy = new(0.0, 1.0, 0.0);
        Vector3D aCross = aVx.Crossed(aVy);
        Assert.Equal(0.0, aCross.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCross.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCross.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CrossMagnitude()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(0.0, 1.0, 0.0);
        Assert.Equal(1.0, aV1.CrossMagnitude(aV2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DotCross_TripleScalarProduct()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(0.0, 1.0, 0.0);
        Vector3D aV3 = new(0.0, 0.0, 1.0);
        // Triple scalar product of orthogonal unit vectors = 1
        Assert.Equal(1.0, aV1.DotCross(aV2, aV3), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Magnitude()
    {
        Vector3D aVec = new(3.0, 4.0, 0.0);
        Assert.Equal(5.0, aVec.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(25.0, aVec.SquareMagnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Normalize()
    {
        Vector3D aVec = new(3.0, 4.0, 0.0);
        Vector3D aNormalized = aVec.Normalized();
        Assert.Equal(1.0, aNormalized.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.6, aNormalized.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.8, aNormalized.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(0.0, 1.0, 0.0);
        Assert.Equal(M_PI_2, aV1.Angle(aV2), global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void IsParallel()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(5.0, 0.0, 0.0);
        Assert.True(aV1.IsParallel(aV2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(0.0, 1.0, 0.0);
        Assert.True(aV1.IsNormal(aV2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(-3.0, 0.0, 0.0);
        Assert.True(aV1.IsOpposite(aV2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Reverse()
    {
        Vector3D aVec = new(1.0, 2.0, 3.0);
        Vector3D aReversed = aVec.Reversed();
        Assert.Equal(-1.0, aReversed.X());
        Assert.Equal(-2.0, aReversed.Y());
        Assert.Equal(-3.0, aReversed.Z());
    }

    [Fact]
    public void SetLinearForm()
    {
        Vector3D aV1 = new(1.0, 0.0, 0.0);
        Vector3D aV2 = new(0.0, 1.0, 0.0);
        Vector3D aResult = new();
        aResult.SetLinearForm(2.0, aV1, 3.0, aV2);
        Assert.Equal(2.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
