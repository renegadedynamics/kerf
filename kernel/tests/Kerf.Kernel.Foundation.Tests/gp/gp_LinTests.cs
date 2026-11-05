// Port of: src/FoundationClasses/TKMath/GTests/gp_Lin_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_LinTest
{
    [Fact]
    public void ConstructorFromAx1()
    {
        Axis3D anAx1 = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(1.0, 0.0, 0.0));
        Line3D aLin = new(anAx1);
        Assert.True(aLin.Location.IsEqual(new Point3D(1.0, 2.0, 3.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aLin.Direction.IsEqual(new Direction3D(1.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void ConstructorFromPointAndDir()
    {
        Line3D aLin = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Assert.True(aLin.Location.IsEqual(new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aLin.Direction.IsEqual(new Direction3D(0.0, 0.0, 1.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void DistanceToPoint()
    {
        Line3D aLin = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Point3D aPnt = new(5.0, 3.0, 4.0);
        // Distance from point to X axis: sqrt(3^2 + 4^2) = 5
        Assert.Equal(5.0, aLin.DistanceTo(aPnt), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DistanceToLine_Parallel()
    {
        Line3D aL1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Line3D aL2 = new(new Point3D(0.0, 3.0, 4.0), new Direction3D(1.0, 0.0, 0.0));
        Assert.Equal(5.0, aL1.DistanceTo(aL2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DistanceToLine_Intersecting()
    {
        Line3D aL1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Line3D aL2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        Assert.Equal(0.0, aL1.DistanceTo(aL2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Contains()
    {
        Line3D aLin = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Point3D aOnLine = new(5.0, 0.0, 0.0);
        Point3D aOffLine = new(5.0, 1.0, 0.0);
        Assert.True(aLin.Contains(aOnLine, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.False(aLin.Contains(aOffLine, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void Angle()
    {
        Line3D aL1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Line3D aL2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        Assert.Equal(M_PI_2, aL1.AngleTo(aL2), global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void Translate()
    {
        Line3D aLin = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Vector3D aVec = new(0.0, 5.0, 0.0);
        Line3D aTranslated = aLin.Translated(aVec);
        Assert.Equal(5.0, aTranslated.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Line3D aLin = new(new Point3D(1.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Line3D aRotated = aLin.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.Location.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aRotated.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aRotated.Direction.IsEqual(new Direction3D(0.0, 1.0, 0.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Mirror()
    {
        Line3D aLin = new(new Point3D(1.0, 1.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Line3D aMirrored = aLin.Mirrored(anAxis);
        Assert.Equal(-1.0, aMirrored.Location.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Line3D aLin = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0.0, 0.0, 10.0));
        Line3D aTransformed = aLin.Transformed(aTrsf);
        Assert.Equal(10.0, aTransformed.Location.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}

/// <summary>The gp_Lin2d tests that OCCT keeps in gp_Lin_Test.cxx (OCC15489).</summary>
public class gp_Lin2dTest
{
    [Fact]
    public void ConstructFromEquation_ZeroDirection_ThrowsException()
    {
        // A=0, B=0 -> direction vector is zero -> must throw
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new Line2D(0.0, 0.0, 1.0));
    }

    [Fact]
    public void ConstructFromEquation_ValidCoefficients_CorrectOrigin()
    {
        // A=1e-20, B=-1, C=2 gives the line   1e-20 * x - y + 2 = 0
        // Origin (closest point to global origin on the line) must be
        // X_0 ~= -1.9999999999999999e-20, Y_0 ~= 2
        Line2D aLin2d = new(1e-20, -1.0, 2.0);
        Point2D anOrigin = aLin2d.Location;
        Assert.Equal(-1.9999999999999999e-20, anOrigin.X, 1e-25);
        Assert.Equal(2.0, anOrigin.Y, 0.001);
    }
}
