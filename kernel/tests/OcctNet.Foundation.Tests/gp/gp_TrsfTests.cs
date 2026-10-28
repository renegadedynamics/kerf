// Port of: src/FoundationClasses/TKMath/GTests/gp_Trsf_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_TrsfTest
{
    [Fact]
    public void OCC23361_TransformationComposition()
    {
        gp_Pnt aP = new(0, 0, 2);

        gp_Trsf aT1 = new(), aT2 = new();
        aT1.SetRotation(new gp_Ax1(aP, new gp_Dir(gp_Dir.D.Y)), -0.49328285294022267);
        aT2.SetRotation(new gp_Ax1(aP, new gp_Dir(gp_Dir.D.Z)), 0.87538474718473880);

        gp_Trsf aTComp = aT2 * aT1;

        gp_Pnt aP1 = new(10, 3, 4);
        gp_Pnt aP2 = aP1.Transformed(aTComp);
        gp_Pnt aP3 = aP1.Transformed(aT1);
        aP3.Transform(aT2);

        // Points must be equal: equivalent transformations should produce equal points
        Assert.True(aP2.IsEqual(aP3, Precision.Confusion()));
    }
}
