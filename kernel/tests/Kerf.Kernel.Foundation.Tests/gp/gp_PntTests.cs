// Port of: src/FoundationClasses/TKMath/GTests/gp_Pnt_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_PntTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Point3D aPnt = new();
        Assert.Equal(0.0, aPnt.X);
        Assert.Equal(0.0, aPnt.Y);
        Assert.Equal(0.0, aPnt.Z);
    }

    [Fact]
    public void CoordinateConstructor()
    {
        Point3D aPnt = new(1.0, 2.0, 3.0);
        Assert.Equal(1.0, aPnt.X);
        Assert.Equal(2.0, aPnt.Y);
        Assert.Equal(3.0, aPnt.Z);
    }

    [Fact]
    public void XYZConstructor()
    {
        Xyz aXYZ = new(4.0, 5.0, 6.0);
        Point3D aPnt = new(aXYZ);
        Assert.Equal(4.0, aPnt.X);
        Assert.Equal(5.0, aPnt.Y);
        Assert.Equal(6.0, aPnt.Z);
    }

    [Fact]
    public void SetCoord()
    {
        Point3D aPnt = new Point3D().WithX(10.0);
        aPnt = aPnt.WithY(20.0);
        aPnt = aPnt.WithZ(30.0);
        Assert.Equal(10.0, aPnt.X);
        Assert.Equal(20.0, aPnt.Y);
        Assert.Equal(30.0, aPnt.Z);

        aPnt = aPnt.WithCoord(7.0, 8.0, 9.0);
        Assert.Equal(7.0, aPnt.X);
        Assert.Equal(8.0, aPnt.Y);
        Assert.Equal(9.0, aPnt.Z);
    }

    [Fact]
    public void Distance()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(3.0, 4.0, 0.0);
        Assert.Equal(5.0, aP1.DistanceTo(aP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SquareDistance()
    {
        Point3D aP1 = new(1.0, 2.0, 3.0);
        Point3D aP2 = new(4.0, 6.0, 3.0);
        // (4-1)^2 + (6-2)^2 + (3-3)^2 = 9 + 16 + 0 = 25
        Assert.Equal(25.0, aP1.SquareDistanceTo(aP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsEqual()
    {
        Point3D aP1 = new(1.0, 2.0, 3.0);
        Point3D aP2 = new(1.0, 2.0, 3.0);
        Point3D aP3 = new(1.0, 2.0, 4.0);
        Assert.True(aP1.IsEqual(aP2, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.False(aP1.IsEqual(aP3, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void BaryCenter()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(10.0, 0.0, 0.0);
        // BaryCenter with equal weights: midpoint
        aP1 = aP1.BaryCentered(1.0, aP2, 1.0);
        Assert.Equal(5.0, aP1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP1.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Translate_ByVec()
    {
        Point3D aPnt = new(1.0, 2.0, 3.0);
        Vector3D aVec = new(10.0, 20.0, 30.0);
        Point3D aResult = aPnt.Translated(aVec);
        Assert.Equal(11.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(22.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(33.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Translate_ByTwoPoints()
    {
        Point3D aPnt = new(1.0, 2.0, 3.0);
        Point3D aFrom = new(0.0, 0.0, 0.0);
        Point3D aTo = new(5.0, 5.0, 5.0);
        Point3D aResult = aPnt.Translated(aFrom, aTo);
        Assert.Equal(6.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        Point3D aPnt = new(2.0, 4.0, 6.0);
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aResult = aPnt.Scaled(aCenter, 2.0);
        Assert.Equal(4.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(12.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Point3D aPnt = new(1.0, 0.0, 0.0);
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aResult = aPnt.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror_Point()
    {
        Point3D aPnt = new(1.0, 0.0, 0.0);
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aResult = aPnt.Mirrored(aCenter);
        Assert.Equal(-1.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror_Axis()
    {
        Point3D aPnt = new(1.0, 1.0, 0.0);
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Point3D aResult = aPnt.Mirrored(anAxis);
        Assert.Equal(1.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-1.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror_Plane()
    {
        Point3D aPnt = new(1.0, 2.0, 3.0);
        // Mirror through XY plane (Z=0)
        Frame3D anAx2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aResult = aPnt.Mirrored(anAx2);
        Assert.Equal(1.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-3.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transformed()
    {
        Point3D aPnt = new(1.0, 0.0, 0.0);
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0.0, 0.0, 5.0));
        Point3D aResult = aPnt.Transformed(aTrsf);
        Assert.Equal(1.0, aResult.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aResult.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
