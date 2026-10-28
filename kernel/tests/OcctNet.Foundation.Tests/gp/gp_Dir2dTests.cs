// Port of: src/FoundationClasses/TKMath/GTests/gp_Dir2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_Dir2dTests
{
    [Fact]
    public void CoordinateConstructor()
    {
        gp_Dir2d aDir = new(1.0, 0.0);
        Assert.Equal(1.0, aDir.X(), Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void XYConstructor()
    {
        gp_XY aXY = new(0.0, 3.0);
        gp_Dir2d aDir = new(aXY);
        Assert.Equal(0.0, aDir.X(), Precision.Confusion());
        Assert.Equal(1.0, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void Vec2dConstructor()
    {
        gp_Vec2d aVec = new(3.0, 0.0);
        gp_Dir2d aDir = new(aVec);
        Assert.Equal(1.0, aDir.X(), Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(0.0, 1.0);
        Assert.Equal(M_PI_2, aD1.Angle(aD2), Precision.Angular());
    }

    [Fact]
    public void IsEqual()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(1.0, 0.0);
        Assert.True(aD1.IsEqual(aD2, Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(0.0, 1.0);
        Assert.True(aD1.IsNormal(aD2, Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(-1.0, 0.0);
        Assert.True(aD1.IsOpposite(aD2, Precision.Angular()));
    }

    [Fact]
    public void IsParallel()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(-1.0, 0.0);
        Assert.True(aD1.IsParallel(aD2, Precision.Angular()));
    }

    [Fact]
    public void Dot()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(0.0, 1.0);
        Assert.Equal(0.0, aD1.Dot(aD2), Precision.Confusion());

        gp_Dir2d aD3 = new(1.0, 0.0);
        Assert.Equal(1.0, aD1.Dot(aD3), Precision.Confusion());
    }

    [Fact]
    public void Crossed()
    {
        gp_Dir2d aD1 = new(1.0, 0.0);
        gp_Dir2d aD2 = new(0.0, 1.0);
        // D1.X*D2.Y - D1.Y*D2.X = 1*1 - 0*0 = 1
        Assert.Equal(1.0, aD1.Crossed(aD2), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Dir2d aDir = new(1.0, 0.0);
        gp_Dir2d aRotated = aDir.Rotated(M_PI_2);
        Assert.Equal(0.0, aRotated.X(), Precision.Confusion());
        Assert.Equal(1.0, aRotated.Y(), Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        gp_Dir2d aDir = new(1.0, 0.0);
        // Mirror through the Y axis
        gp_Ax2d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(0.0, 1.0));
        gp_Dir2d aMirrored = aDir.Mirrored(anAxis);
        // Mirroring (1,0) through axis with direction (0,1):
        // d_mirror = 2*(d.axis_dir)*axis_dir - d = 2*0*(0,1) - (1,0) = (-1,0)
        Assert.Equal(-1.0, aMirrored.X(), Precision.Confusion());
        Assert.Equal(0.0, aMirrored.Y(), Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        gp_Dir2d aDir = new(1.0, 0.0);
        gp_Trsf2d aTrsf = new();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI);
        gp_Dir2d aTransformed = aDir.Transformed(aTrsf);
        Assert.Equal(-1.0, aTransformed.X(), Precision.Confusion());
        Assert.Equal(0.0, aTransformed.Y(), Precision.Confusion());
    }

    [Fact]
    public void ZeroMagnitudeThrows()
    {
        Assert.ThrowsAny<Standard_ConstructionError>(() => new gp_Dir2d(0.0, 0.0));
    }
}
