// Port of: src/FoundationClasses/TKMath/GTests/gp_Dir_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

public class gp_DirTest
{
    [Fact]
    public void CoordinateConstructor()
    {
        Direction3D aDir = new(0.0, 0.0, 1.0);
        Assert.Equal(0.0, aDir.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aDir.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void XYZConstructor()
    {
        Xyz aXYZ = new(0.0, 3.0, 4.0);
        Direction3D aDir = new(aXYZ);
        Assert.Equal(0.0, aDir.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.6, aDir.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.8, aDir.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void VecConstructor()
    {
        Vector3D aVec = new(3.0, 0.0, 0.0);
        Direction3D aDir = new(aVec);
        Assert.Equal(1.0, aDir.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDir.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PredefinedDirections()
    {
        Assert.True(new Direction3D(1.0, 0.0, 0.0).IsEqual(new Direction3D(Direction3D.D.X), global::Kerf.Engine.Foundation.Precision.Angular()));
        Assert.True(new Direction3D(0.0, 1.0, 0.0).IsEqual(new Direction3D(Direction3D.D.Y), global::Kerf.Engine.Foundation.Precision.Angular()));
        Assert.True(new Direction3D(0.0, 0.0, 1.0).IsEqual(new Direction3D(Direction3D.D.Z), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Angle()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(0.0, 1.0, 0.0);
        Assert.Equal(M_PI_2, aD1.Angle(aD2), global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void AngleWithRef()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(0.0, 1.0, 0.0);
        Direction3D aRef = new(0.0, 0.0, 1.0);
        double anAngle = aD1.AngleWithRef(aD2, aRef);
        Assert.Equal(M_PI_2, anAngle, global::Kerf.Engine.Foundation.Precision.Angular());

        // Reverse reference direction should negate the angle
        double anAngleReversed = aD1.AngleWithRef(aD2, new Direction3D(0.0, 0.0, -1.0));
        Assert.Equal(-M_PI_2, anAngleReversed, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void IsEqual()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(1.0, 0.0, 0.0);
        Assert.True(aD1.IsEqual(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsNormal()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(0.0, 1.0, 0.0);
        Assert.True(aD1.IsNormal(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsOpposite()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(-1.0, 0.0, 0.0);
        Assert.True(aD1.IsOpposite(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void IsParallel()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(-1.0, 0.0, 0.0);
        Assert.True(aD1.IsParallel(aD2, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Cross()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(0.0, 1.0, 0.0);
        Direction3D aCross = aD1.Crossed(aD2);
        Assert.Equal(0.0, aCross.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCross.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCross.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CrossCross()
    {
        Direction3D aA = new(1.0, 0.0, 0.0);
        Direction3D aB = new(1.0, 1.0, 0.0);
        Direction3D aC = new(0.0, 0.0, 1.0);
        Direction3D aCrossCross = aA.CrossCrossed(aB, aC);
        // Result should be a valid normalized direction
        double aMag = Math.Sqrt(aCrossCross.X() * aCrossCross.X() + aCrossCross.Y() * aCrossCross.Y()
                                + aCrossCross.Z() * aCrossCross.Z());
        Assert.Equal(1.0, aMag, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Dot()
    {
        Direction3D aD1 = new(1.0, 0.0, 0.0);
        Direction3D aD2 = new(0.0, 1.0, 0.0);
        Assert.Equal(0.0, aD1.Dot(aD2), global::Kerf.Engine.Foundation.Precision.Confusion());

        Direction3D aD3 = new(1.0, 0.0, 0.0);
        Assert.Equal(1.0, aD1.Dot(aD3), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror_Point()
    {
        Direction3D aDir = new(1.0, 0.0, 0.0);
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        Direction3D aMirrored = aDir.Mirrored(anAxis);
        // For axis mirror of a direction, it reflects in the axis:
        // d_mirror = 2*(d.axis)*axis - d
        // d.axis = (1,0,0).(0,1,0) = 0
        // result = 2*0*(0,1,0) - (1,0,0) = (-1,0,0)
        Assert.Equal(-1.0, aMirrored.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aMirrored.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Direction3D aDir = new(1.0, 0.0, 0.0);
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Direction3D aRotated = aDir.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aRotated.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aRotated.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Direction3D aDir = new(1.0, 0.0, 0.0);
        Transform3D aTrsf = new();
        aTrsf.SetRotation(new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), M_PI);
        Direction3D aTransformed = aDir.Transformed(aTrsf);
        Assert.Equal(-1.0, aTransformed.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aTransformed.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ZeroMagnitudeInput_ThrowsException()
    {
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new Direction3D(0.0, 0.0, 0.0));
    }

    // OCC22558: Mirror of gp_Dir(1,1,1) through gp_Ax2 at origin with main dir (1,0,0) gives
    // (-1/sqrt(3), 1/sqrt(3), 1/sqrt(3)).  The TCL test checks int(res * 100000) == {-57735, 57735,
    // 57735}.
    [Fact]
    public void OCC22558_MirrorThroughAx2_CorrectResult()
    {
        Direction3D aToSym = new(1.0, 1.0, 1.0);
        Frame3D anAx2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        aToSym.Mirror(anAx2);

        // Expected: (-1/sqrt(3), 1/sqrt(3), 1/sqrt(3))
        double aExpected = 1.0 / Math.Sqrt(3.0);
        Assert.Equal(-aExpected, aToSym.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpected, aToSym.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpected, aToSym.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
