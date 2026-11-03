// Port of: src/FoundationClasses/TKMath/GTests/gp_Pln_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_PlnTest
{
    [Fact]
    public void ConstructorFromAx3()
    {
        CoordinateSystem anAx3 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Plane3D aPln = new(anAx3);
        Assert.True(aPln.Location.IsEqual(new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aPln.Axis.Direction.IsEqual(new Direction3D(0.0, 0.0, 1.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void ConstructorFromPointAndDir()
    {
        Point3D aLoc = new(1.0, 2.0, 3.0);
        Direction3D aNorm = new(0.0, 0.0, 1.0);
        Plane3D aPln = new(aLoc, aNorm);
        Assert.True(aPln.Location.IsEqual(aLoc, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aPln.Axis.Direction.IsEqual(aNorm, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void ConstructorFromCoefficients()
    {
        // Plane Z = 5 => 0*X + 0*Y + 1*Z - 5 = 0
        Plane3D aPln = new(0.0, 0.0, 1.0, -5.0);
        aPln.Coefficients(out double anA, out double aB, out double aC, out double aD);
        Assert.Equal(0.0, anA, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aB, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aC, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aD, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DistanceToPoint()
    {
        Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aPnt = new(3.0, 4.0, 5.0);
        Assert.Equal(5.0, aPln.DistanceTo(aPnt), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Contains()
    {
        Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aOnPlane = new(3.0, 4.0, 0.0);
        Point3D aOffPlane = new(3.0, 4.0, 1.0);
        Assert.True(aPln.Contains(aOnPlane, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.False(aPln.Contains(aOffPlane, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void Coefficients()
    {
        Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aPln.Coefficients(out double anA, out double aB, out double aC, out double aD);
        Assert.Equal(0.0, anA, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aB, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aC, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        Plane3D aPln = new(new Point3D(0.0, 0.0, 5.0), new Direction3D(0.0, 0.0, 1.0));
        Frame3D aMirrorPlane = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Plane3D aMirrored = aPln.Mirrored(aMirrorPlane);
        Assert.Equal(-5.0, aMirrored.Location.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Plane3D aPln = new(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Plane3D aRotated = aPln.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aRotated.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        Plane3D aPln = new(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Plane3D aScaled = aPln.Scaled(aCenter, 3.0);
        Assert.Equal(3.0, aScaled.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Translate()
    {
        Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Vector3D aVec = new(1.0, 2.0, 3.0);
        Plane3D aTranslated = aPln.Translated(aVec);
        Assert.Equal(1.0, aTranslated.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aTranslated.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aTranslated.Location.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0.0, 0.0, 10.0));
        Plane3D aTransformed = aPln.Transformed(aTrsf);
        Assert.Equal(10.0, aTransformed.Location.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
