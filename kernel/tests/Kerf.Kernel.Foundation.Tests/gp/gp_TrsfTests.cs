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
}
