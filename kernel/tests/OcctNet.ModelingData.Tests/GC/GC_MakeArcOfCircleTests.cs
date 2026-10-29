// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeArcOfCircle_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakeArcOfCircleTest
{
    [Fact]
    public void FromPointsAndTangent_Done()
    {
        gp_Pnt aP1 = new(1.0, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, 1.0, 0.0);
        gp_Vec aTangentAtP1 = new(0.0, 1.0, 0.0);
        GC_MakeArcOfCircle aMaker = new(aP1, aTangentAtP1, aP2);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
    }

    [Fact]
    public void FromThreePoints_Done()
    {
        GC_MakeArcOfCircle aMaker = new(new gp_Pnt(1.0, 0.0, 0.0), new gp_Pnt(0.0, 1.0, 0.0), new gp_Pnt(-1.0, 0.0, 0.0));
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
    }

    [Fact]
    public void FromPointsAndTangent_ConfusionScale_Done()
    {
        double aScale = 2.0 * Precision.Confusion();
        gp_Pnt aP1 = new(aScale, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, aScale, 0.0);
        gp_Vec aTangentAtP1 = new(0.0, 1.0, 0.0);
        GC_MakeArcOfCircle aMaker = new(aP1, aTangentAtP1, aP2);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
        Assert.True(aMaker.Value().StartPoint().IsEqual(aP1, Precision.Confusion()));
    }
}
