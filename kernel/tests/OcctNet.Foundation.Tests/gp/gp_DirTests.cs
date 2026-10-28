// Port of: src/FoundationClasses/TKMath/GTests/gp_Dir_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_DirTest
{
    [Fact]
    public void CoordinateConstructor()
    {
        gp_Dir aDir = new(0.0, 0.0, 1.0);
        Assert.Equal(0.0, aDir.X(), Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), Precision.Confusion());
        Assert.Equal(1.0, aDir.Z(), Precision.Confusion());
    }

    [Fact]
    public void XYZConstructor()
    {
        gp_XYZ aXYZ = new(0.0, 3.0, 4.0);
        gp_Dir aDir = new(aXYZ);
        Assert.Equal(0.0, aDir.X(), Precision.Confusion());
        Assert.Equal(0.6, aDir.Y(), Precision.Confusion());
        Assert.Equal(0.8, aDir.Z(), Precision.Confusion());
    }

    [Fact]
    public void VecConstructor()
    {
        gp_Vec aVec = new(3.0, 0.0, 0.0);
        gp_Dir aDir = new(aVec);
        Assert.Equal(1.0, aDir.X(), Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), Precision.Confusion());
        Assert.Equal(0.0, aDir.Z(), Precision.Confusion());
    }

    [Fact]
    public void PredefinedDirections()
    {
        Assert.True(new gp_Dir(1.0, 0.0, 0.0).IsEqual(new gp_Dir(gp_Dir.D.X), Precision.Angular()));
        Assert.True(new gp_Dir(0.0, 1.0, 0.0).IsEqual(new gp_Dir(gp_Dir.D.Y), Precision.Angular()));
        Assert.True(new gp_Dir(0.0, 0.0, 1.0).IsEqual(new gp_Dir(gp_Dir.D.Z), Precision.Angular()));
    }

    [Fact]
    public void Angle()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(0.0, 1.0, 0.0);
        Assert.Equal(M_PI_2, aD1.Angle(aD2), Precision.Angular());
    }

    [Fact]
    public void AngleWithRef()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(0.0, 1.0, 0.0);
        gp_Dir aRef = new(0.0, 0.0, 1.0);
        double anAngle = aD1.AngleWithRef(aD2, aRef);
        Assert.Equal(M_PI_2, anAngle, Precision.Angular());

        // Reverse reference direction should negate the angle
        double anAngleReversed = aD1.AngleWithRef(aD2, new gp_Dir(0.0, 0.0, -1.0));
        Assert.Equal(-M_PI_2, anAngleReversed, Precision.Angular());
    }

    [Fact]
    public void IsEqual()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(1.0, 0.0, 0.0);
        Assert.True(aD1.IsEqual(aD2, Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(0.0, 1.0, 0.0);
        Assert.True(aD1.IsNormal(aD2, Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(-1.0, 0.0, 0.0);
        Assert.True(aD1.IsOpposite(aD2, Precision.Angular()));
    }

    [Fact]
    public void IsParallel()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(-1.0, 0.0, 0.0);
        Assert.True(aD1.IsParallel(aD2, Precision.Angular()));
    }

    [Fact]
    public void Cross()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(0.0, 1.0, 0.0);
        gp_Dir aCross = aD1.Crossed(aD2);
        Assert.Equal(0.0, aCross.X(), Precision.Confusion());
        Assert.Equal(0.0, aCross.Y(), Precision.Confusion());
        Assert.Equal(1.0, aCross.Z(), Precision.Confusion());
    }

    [Fact]
    public void CrossCross()
    {
        gp_Dir aA = new(1.0, 0.0, 0.0);
        gp_Dir aB = new(1.0, 1.0, 0.0);
        gp_Dir aC = new(0.0, 0.0, 1.0);
        gp_Dir aCrossCross = aA.CrossCrossed(aB, aC);
        // Result should be a valid normalized direction
        double aMag = Math.Sqrt(aCrossCross.X() * aCrossCross.X() + aCrossCross.Y() * aCrossCross.Y()
                                + aCrossCross.Z() * aCrossCross.Z());
        Assert.Equal(1.0, aMag, Precision.Confusion());
    }

    [Fact]
    public void Dot()
    {
        gp_Dir aD1 = new(1.0, 0.0, 0.0);
        gp_Dir aD2 = new(0.0, 1.0, 0.0);
        Assert.Equal(0.0, aD1.Dot(aD2), Precision.Confusion());

        gp_Dir aD3 = new(1.0, 0.0, 0.0);
        Assert.Equal(1.0, aD1.Dot(aD3), Precision.Confusion());
    }

    [Fact]
    public void Mirror_Point()
    {
        gp_Dir aDir = new(1.0, 0.0, 0.0);
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 0.0));
        gp_Dir aMirrored = aDir.Mirrored(anAxis);
        // For axis mirror of a direction, it reflects in the axis:
        // d_mirror = 2*(d.axis)*axis - d
        // d.axis = (1,0,0).(0,1,0) = 0
        // result = 2*0*(0,1,0) - (1,0,0) = (-1,0,0)
        Assert.Equal(-1.0, aMirrored.X(), Precision.Confusion());
        Assert.Equal(0.0, aMirrored.Y(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Dir aDir = new(1.0, 0.0, 0.0);
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Dir aRotated = aDir.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.X(), Precision.Confusion());
        Assert.Equal(1.0, aRotated.Y(), Precision.Confusion());
        Assert.Equal(0.0, aRotated.Z(), Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        gp_Dir aDir = new(1.0, 0.0, 0.0);
        gp_Trsf aTrsf = new();
        aTrsf.SetRotation(new gp_Ax1(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), M_PI);
        gp_Dir aTransformed = aDir.Transformed(aTrsf);
        Assert.Equal(-1.0, aTransformed.X(), Precision.Confusion());
        Assert.Equal(0.0, aTransformed.Y(), Precision.Confusion());
    }

    [Fact]
    public void ZeroMagnitudeInput_ThrowsException()
    {
        Assert.ThrowsAny<Standard_ConstructionError>(() => new gp_Dir(0.0, 0.0, 0.0));
    }

    // OCC22558: Mirror of gp_Dir(1,1,1) through gp_Ax2 at origin with main dir (1,0,0) gives
    // (-1/sqrt(3), 1/sqrt(3), 1/sqrt(3)).  The TCL test checks int(res * 100000) == {-57735, 57735,
    // 57735}.
    [Fact]
    public void OCC22558_MirrorThroughAx2_CorrectResult()
    {
        gp_Dir aToSym = new(1.0, 1.0, 1.0);
        gp_Ax2 anAx2 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        aToSym.Mirror(anAx2);

        // Expected: (-1/sqrt(3), 1/sqrt(3), 1/sqrt(3))
        double aExpected = 1.0 / Math.Sqrt(3.0);
        Assert.Equal(-aExpected, aToSym.X(), Precision.Confusion());
        Assert.Equal(aExpected, aToSym.Y(), Precision.Confusion());
        Assert.Equal(aExpected, aToSym.Z(), Precision.Confusion());
    }
}
