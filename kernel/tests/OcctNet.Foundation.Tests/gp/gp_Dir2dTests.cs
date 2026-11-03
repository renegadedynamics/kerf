// Port of: src/FoundationClasses/TKMath/GTests/gp_Dir2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

public class gp_Dir2dTests
{
    [Fact]
    public void CoordinateConstructor()
    {
        Direction2D aDir = new(1.0, 0.0);
        Assert.Equal(1.0, aDir.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void XYConstructor()
    {
        Xy aXY = new(0.0, 3.0);
        Direction2D aDir = new(aXY);
        Assert.Equal(0.0, aDir.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aDir.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Vec2dConstructor()
    {
        Vector2D aVec = new(3.0, 0.0);
        Direction2D aDir = new(aVec);
        Assert.Equal(1.0, aDir.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(0.0, 1.0);
        Assert.Equal(M_PI_2, aD1.Angle(aD2), global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void IsEqual()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(1.0, 0.0);
        Assert.True(aD1.IsEqual(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(0.0, 1.0);
        Assert.True(aD1.IsNormal(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(-1.0, 0.0);
        Assert.True(aD1.IsOpposite(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsParallel()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(-1.0, 0.0);
        Assert.True(aD1.IsParallel(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Dot()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(0.0, 1.0);
        Assert.Equal(0.0, aD1.Dot(aD2), global::Kerf.Engine.Foundation.Precision.Confusion());

        Direction2D aD3 = new(1.0, 0.0);
        Assert.Equal(1.0, aD1.Dot(aD3), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Crossed()
    {
        Direction2D aD1 = new(1.0, 0.0);
        Direction2D aD2 = new(0.0, 1.0);
        // D1.X*D2.Y - D1.Y*D2.X = 1*1 - 0*0 = 1
        Assert.Equal(1.0, aD1.Crossed(aD2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Direction2D aDir = new(1.0, 0.0);
        Direction2D aRotated = aDir.Rotated(M_PI_2);
        Assert.Equal(0.0, aRotated.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aRotated.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        Direction2D aDir = new(1.0, 0.0);
        // Mirror through the Y axis
        Axis2D anAxis = new(new Point2D(0.0, 0.0), new Direction2D(0.0, 1.0));
        Direction2D aMirrored = aDir.Mirrored(anAxis);
        // Mirroring (1,0) through axis with direction (0,1):
        // d_mirror = 2*(d.axis_dir)*axis_dir - d = 2*0*(0,1) - (1,0) = (-1,0)
        Assert.Equal(-1.0, aMirrored.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aMirrored.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Direction2D aDir = new(1.0, 0.0);
        Transform2D aTrsf = new();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI);
        Direction2D aTransformed = aDir.Transformed(aTrsf);
        Assert.Equal(-1.0, aTransformed.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aTransformed.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ZeroMagnitudeThrows()
    {
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new Direction2D(0.0, 0.0));
    }
}
