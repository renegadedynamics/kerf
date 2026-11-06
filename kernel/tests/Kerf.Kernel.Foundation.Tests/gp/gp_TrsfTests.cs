// Port of: src/FoundationClasses/TKMath/GTests/gp_Trsf_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_TrsfTest
{
    [Fact]
    public void OCC23361_TransformationComposition()
    {
        Point3D aP = new(0, 0, 2);

        Transform3D aT1 = new(), aT2 = new();
        aT1 = aT1.WithRotation(new Axis3D(aP, new Direction3D(Direction3D.D.Y)), -0.49328285294022267);
        aT2 = aT2.WithRotation(new Axis3D(aP, new Direction3D(Direction3D.D.Z)), 0.87538474718473880);

        Transform3D aTComp = aT2 * aT1;

        Point3D aP1 = new(10, 3, 4);
        Point3D aP2 = aP1.Transformed(aTComp);
        Point3D aP3 = aP1.Transformed(aT1);
        aP3 = aP3.Transformed(aT2);

        // Points must be equal: equivalent transformations should produce equal points
        Assert.True(aP2.IsEqual(aP3, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void PointMirror_IsItsOwnInverse()
    {
        // OCCT 3270746444: 8.0.1 negated the translation part, which is right for a translation only
        Point3D aCentre = new(1, 2, 3), aP = new(4, -5, 6);
        Transform3D aMirror = Transform3D.Mirror(aCentre);
        Point3D aBack = aP.Transformed(aMirror).Transformed(aMirror.Inverted());
        Assert.True(aBack.DistanceTo(aP) < 1.0e-12);

        Transform2D aMirror2d = Transform2D.Mirror(new Point2D(1, 2));
        Point2D aBack2d = new Point2D(4, -5).Transformed(aMirror2d).Transformed(aMirror2d.Inverted());
        Assert.True(aBack2d.DistanceTo(new Point2D(4, -5)) < 1.0e-12);
    }

    [Fact]
    public void A_direction_refuses_what_has_no_direction()
    {
        // OCCT #1618, decision 20610: NaN passed the test for a null vector, and an infinite length normalises to NaN
        foreach (double aBad in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
        {
            Assert.Throws<ArgumentException>(() => new Direction3D(aBad, 0.0, 0.0));
            Assert.Throws<ArgumentException>(() => new Direction3D(0.0, 0.0, aBad));
            Assert.Throws<ArgumentException>(() => new Direction2D(aBad, 0.0));
            Assert.Throws<ArgumentException>(() => new Direction2D(0.0, aBad));
        }
        Assert.Equal(1.0, new Direction3D(1e150, 0.0, 0.0).X);
    }

    [Fact]
    public void Elementary_geometry_refuses_a_NaN_radius_or_angle()
    {
        // OCCT #1618: every comparison with NaN is false, so the old guards let it through
        Frame3D aFrame = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        CoordinateSystem aSystem = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        double aNaN = double.NaN;
        Assert.Throws<ArgumentException>(() => new Circle3D(aFrame, aNaN));
        Assert.Throws<ArgumentException>(() => new Ellipse3D(aFrame, aNaN, 1.0));
        Assert.Throws<ArgumentException>(() => new Ellipse3D(aFrame, 2.0, aNaN));
        Assert.Throws<ArgumentException>(() => new Hyperbola3D(aFrame, aNaN, 1.0));
        Assert.Throws<ArgumentException>(() => new Parabola3D(aFrame, aNaN));
        Assert.Throws<ArgumentException>(() => new Sphere3D(aSystem, aNaN));
        Assert.Throws<ArgumentException>(() => new Cylinder3D(aSystem, aNaN));
        Assert.Throws<ArgumentException>(() => new Cone3D(aSystem, aNaN, 1.0));
        Assert.Throws<ArgumentException>(() => new Cone3D(aSystem, 0.5, aNaN));
        Assert.Throws<ArgumentException>(() => new Torus3D(aSystem, aNaN, 1.0));
    }
}
