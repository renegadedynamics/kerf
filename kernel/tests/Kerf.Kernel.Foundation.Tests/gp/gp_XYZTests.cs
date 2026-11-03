// Port of: src/FoundationClasses/TKMath/GTests/gp_XYZ_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_XYZTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Xyz aXYZ = new();
        Assert.Equal(0.0, aXYZ.X());
        Assert.Equal(0.0, aXYZ.Y());
        Assert.Equal(0.0, aXYZ.Z());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        Xyz aXYZ = new(1.0, 2.0, 3.0);
        Assert.Equal(1.0, aXYZ.X());
        Assert.Equal(2.0, aXYZ.Y());
        Assert.Equal(3.0, aXYZ.Z());
    }

    [Fact]
    public void SetCoord()
    {
        Xyz aXYZ = new();
        aXYZ.SetCoord(7.0, 8.0, 9.0);
        Assert.Equal(7.0, aXYZ.X());
        Assert.Equal(8.0, aXYZ.Y());
        Assert.Equal(9.0, aXYZ.Z());
    }

    [Fact]
    public void Accessors()
    {
        Xyz aXYZ = new(1.0, 2.0, 3.0);
        Assert.Equal(1.0, aXYZ.Coord(1));
        Assert.Equal(2.0, aXYZ.Coord(2));
        Assert.Equal(3.0, aXYZ.Coord(3));
    }

    [Fact]
    public void Add()
    {
        Xyz aA = new(1.0, 2.0, 3.0);
        Xyz aB = new(4.0, 5.0, 6.0);
        Xyz aSum = aA + aB;
        Assert.Equal(5.0, aSum.X());
        Assert.Equal(7.0, aSum.Y());
        Assert.Equal(9.0, aSum.Z());
    }

    [Fact]
    public void Subtract()
    {
        Xyz aA = new(4.0, 5.0, 6.0);
        Xyz aB = new(1.0, 2.0, 3.0);
        Xyz aDiff = aA - aB;
        Assert.Equal(3.0, aDiff.X());
        Assert.Equal(3.0, aDiff.Y());
        Assert.Equal(3.0, aDiff.Z());
    }

    [Fact]
    public void Multiply()
    {
        Xyz aXYZ = new(1.0, 2.0, 3.0);
        Xyz aScaled = aXYZ * 2.0;
        Assert.Equal(2.0, aScaled.X());
        Assert.Equal(4.0, aScaled.Y());
        Assert.Equal(6.0, aScaled.Z());
    }

    [Fact]
    public void Divide()
    {
        Xyz aXYZ = new(4.0, 6.0, 8.0);
        Xyz aDivided = aXYZ / 2.0;
        Assert.Equal(2.0, aDivided.X());
        Assert.Equal(3.0, aDivided.Y());
        Assert.Equal(4.0, aDivided.Z());
    }

    [Fact]
    public void Cross()
    {
        Xyz aA = new(1.0, 0.0, 0.0);
        Xyz aB = new(0.0, 1.0, 0.0);
        Xyz aCross = aA.Crossed(aB);
        Assert.Equal(0.0, aCross.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCross.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCross.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Dot()
    {
        Xyz aA = new(1.0, 2.0, 3.0);
        Xyz aB = new(4.0, 5.0, 6.0);
        // 1*4 + 2*5 + 3*6 = 32
        Assert.Equal(32.0, aA.Dot(aB));
    }

    [Fact]
    public void CrossMagnitude()
    {
        Xyz aA = new(1.0, 0.0, 0.0);
        Xyz aB = new(0.0, 1.0, 0.0);
        Assert.Equal(1.0, aA.CrossMagnitude(aB), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CrossSquareMagnitude()
    {
        Xyz aA = new(1.0, 0.0, 0.0);
        Xyz aB = new(0.0, 1.0, 0.0);
        Assert.Equal(1.0, aA.CrossSquareMagnitude(aB), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DotCross()
    {
        Xyz aA = new(1.0, 0.0, 0.0);
        Xyz aB = new(0.0, 1.0, 0.0);
        Xyz aC = new(0.0, 0.0, 1.0);
        Assert.Equal(1.0, aA.DotCross(aB, aC), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Modulus()
    {
        Xyz aXYZ = new(3.0, 4.0, 0.0);
        Assert.Equal(5.0, aXYZ.Modulus(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SquareModulus()
    {
        Xyz aXYZ = new(3.0, 4.0, 0.0);
        Assert.Equal(25.0, aXYZ.SquareModulus(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetLinearForm()
    {
        Xyz aA = new(1.0, 0.0, 0.0);
        Xyz aB = new(0.0, 1.0, 0.0);
        Xyz aResult = new();
        aResult.SetLinearForm(2.0, aA, 3.0, aB);
        Assert.Equal(2.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsEqual()
    {
        Xyz aA = new(1.0, 2.0, 3.0);
        Xyz aB = new(1.0, 2.0, 3.0);
        Xyz aC = new(1.0, 2.0, 4.0);
        Assert.True(aA.IsEqual(aB, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.False(aA.IsEqual(aC, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }
}
