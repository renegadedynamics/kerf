// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeElips_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class gce_MakeElipsTest
{
    [Fact]
    public void FromPoints_Valid_Done()
    {
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        gp_Pnt aS1 = new(10.0, 0.0, 0.0);
        gp_Pnt aS2 = new(0.0, 5.0, 0.0);

        gce_MakeElips aMaker = new(aS1, aS2, aCenter);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Elips anElips = aMaker.Value();
        Assert.Equal(10.0, anElips.MajorRadius(), Precision.Confusion());
        Assert.Equal(5.0, anElips.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void FromPoints_Collinear_InvertAxis()
    {
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        gp_Pnt aS1 = new(10.0, 0.0, 0.0);
        gp_Pnt aS2 = new(5.0, 0.0, 0.0);

        gce_MakeElips aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_InvertAxis, aMaker.Status());
    }

    [Fact]
    public void FromPoints_TinyScale_NullAxis()
    {
        double aRes = gp.Resolution();
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        gp_Pnt aS1 = new(2.0 * aRes, 0.0, 0.0);
        gp_Pnt aS2 = new(0.0, 1.1 * aRes, 0.0);

        gce_MakeElips aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_NullAxis, aMaker.Status());
    }
}
