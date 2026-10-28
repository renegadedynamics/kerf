// Port of: src/FoundationClasses/TKMath/GTests/gp_Pnt_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_PntTest
{
    [Fact]
    public void DefaultConstructor()
    {
        gp_Pnt aPnt = new();
        Assert.Equal(0.0, aPnt.X());
        Assert.Equal(0.0, aPnt.Y());
        Assert.Equal(0.0, aPnt.Z());
    }

    [Fact]
    public void CoordinateConstructor()
    {
        gp_Pnt aPnt = new(1.0, 2.0, 3.0);
        Assert.Equal(1.0, aPnt.X());
        Assert.Equal(2.0, aPnt.Y());
        Assert.Equal(3.0, aPnt.Z());
    }

    [Fact]
    public void XYZConstructor()
    {
        gp_XYZ aXYZ = new(4.0, 5.0, 6.0);
        gp_Pnt aPnt = new(aXYZ);
        Assert.Equal(4.0, aPnt.X());
        Assert.Equal(5.0, aPnt.Y());
        Assert.Equal(6.0, aPnt.Z());
    }

    [Fact]
    public void SetCoord()
    {
        gp_Pnt aPnt = new();
        aPnt.SetX(10.0);
        aPnt.SetY(20.0);
        aPnt.SetZ(30.0);
        Assert.Equal(10.0, aPnt.X());
        Assert.Equal(20.0, aPnt.Y());
        Assert.Equal(30.0, aPnt.Z());

        aPnt.SetCoord(7.0, 8.0, 9.0);
        Assert.Equal(7.0, aPnt.X());
        Assert.Equal(8.0, aPnt.Y());
        Assert.Equal(9.0, aPnt.Z());
    }

    [Fact]
    public void Distance()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(3.0, 4.0, 0.0);
        Assert.Equal(5.0, aP1.Distance(aP2), Precision.Confusion());
    }

    [Fact]
    public void SquareDistance()
    {
        gp_Pnt aP1 = new(1.0, 2.0, 3.0);
        gp_Pnt aP2 = new(4.0, 6.0, 3.0);
        // (4-1)^2 + (6-2)^2 + (3-3)^2 = 9 + 16 + 0 = 25
        Assert.Equal(25.0, aP1.SquareDistance(aP2), Precision.Confusion());
    }

    [Fact]
    public void IsEqual()
    {
        gp_Pnt aP1 = new(1.0, 2.0, 3.0);
        gp_Pnt aP2 = new(1.0, 2.0, 3.0);
        gp_Pnt aP3 = new(1.0, 2.0, 4.0);
        Assert.True(aP1.IsEqual(aP2, Precision.Confusion()));
        Assert.False(aP1.IsEqual(aP3, Precision.Confusion()));
    }

    [Fact]
    public void BaryCenter()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(10.0, 0.0, 0.0);
        // BaryCenter with equal weights: midpoint
        aP1.BaryCenter(1.0, aP2, 1.0);
        Assert.Equal(5.0, aP1.X(), Precision.Confusion());
        Assert.Equal(0.0, aP1.Y(), Precision.Confusion());
        Assert.Equal(0.0, aP1.Z(), Precision.Confusion());
    }

    [Fact]
    public void Translate_ByVec()
    {
        gp_Pnt aPnt = new(1.0, 2.0, 3.0);
        gp_Vec aVec = new(10.0, 20.0, 30.0);
        gp_Pnt aResult = aPnt.Translated(aVec);
        Assert.Equal(11.0, aResult.X(), Precision.Confusion());
        Assert.Equal(22.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(33.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Translate_ByTwoPoints()
    {
        gp_Pnt aPnt = new(1.0, 2.0, 3.0);
        gp_Pnt aFrom = new(0.0, 0.0, 0.0);
        gp_Pnt aTo = new(5.0, 5.0, 5.0);
        gp_Pnt aResult = aPnt.Translated(aFrom, aTo);
        Assert.Equal(6.0, aResult.X(), Precision.Confusion());
        Assert.Equal(7.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(8.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        gp_Pnt aPnt = new(2.0, 4.0, 6.0);
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        gp_Pnt aResult = aPnt.Scaled(aCenter, 2.0);
        Assert.Equal(4.0, aResult.X(), Precision.Confusion());
        Assert.Equal(8.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(12.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Pnt aPnt = new(1.0, 0.0, 0.0);
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aResult = aPnt.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aResult.X(), Precision.Confusion());
        Assert.Equal(1.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Mirror_Point()
    {
        gp_Pnt aPnt = new(1.0, 0.0, 0.0);
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        gp_Pnt aResult = aPnt.Mirrored(aCenter);
        Assert.Equal(-1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Mirror_Axis()
    {
        gp_Pnt aPnt = new(1.0, 1.0, 0.0);
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Pnt aResult = aPnt.Mirrored(anAxis);
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(-1.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Mirror_Plane()
    {
        gp_Pnt aPnt = new(1.0, 2.0, 3.0);
        // Mirror through XY plane (Z=0)
        gp_Ax2 anAx2 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aResult = aPnt.Mirrored(anAx2);
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(2.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(-3.0, aResult.Z(), Precision.Confusion());
    }

    [Fact]
    public void Transformed()
    {
        gp_Pnt aPnt = new(1.0, 0.0, 0.0);
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(0.0, 0.0, 5.0));
        gp_Pnt aResult = aPnt.Transformed(aTrsf);
        Assert.Equal(1.0, aResult.X(), Precision.Confusion());
        Assert.Equal(0.0, aResult.Y(), Precision.Confusion());
        Assert.Equal(5.0, aResult.Z(), Precision.Confusion());
    }
}
