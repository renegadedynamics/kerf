// Port of: src/ModelingData/TKGeomBase/GTests/AdvApp2Var_Network_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;

namespace OcctNet.ModelingData.Tests;

public class AdvApp2Var_NetworkTest
{
    [Fact]
    public void UpdateInV_PreservesPatchOrdersPerColumn()
    {
        NCollection_Sequence<Patch> aNet = new();
        aNet.Append(new Patch(0.0, 1.0, 0.0, 1.0, 0, 1));
        aNet.Append(new Patch(1.0, 2.0, 0.0, 1.0, 2, 3));

        NCollection_Sequence<double> aUParams = new();
        aUParams.Append(0.0);
        aUParams.Append(1.0);
        aUParams.Append(2.0);

        NCollection_Sequence<double> aVParams = new();
        aVParams.Append(0.0);
        aVParams.Append(1.0);

        Network aNetwork = new(aNet, aUParams, aVParams);
        aNetwork.UpdateInV(0.5);

        Assert.Equal(4, aNetwork.PatchCount);
        Assert.Equal(2, aNetwork.PatchInUCount);
        Assert.Equal(2, aNetwork.PatchInVCount);
        Assert.Equal(0.5, aNetwork.VParameter(2));

        Patch aBottomLeft = aNetwork.Patch(1, 1);
        Patch aBottomRight = aNetwork.Patch(2, 1);
        Patch aTopLeft = aNetwork.Patch(1, 2);
        Patch aTopRight = aNetwork.Patch(2, 2);

        Assert.Equal(0, aBottomLeft.UOrder());
        Assert.Equal(1, aBottomLeft.VOrder());
        Assert.Equal(2, aBottomRight.UOrder());
        Assert.Equal(3, aBottomRight.VOrder());
        Assert.Equal(0, aTopLeft.UOrder());
        Assert.Equal(1, aTopLeft.VOrder());
        Assert.Equal(2, aTopRight.UOrder());
        Assert.Equal(3, aTopRight.VOrder());

        Assert.Equal(0.0, aBottomLeft.V0());
        Assert.Equal(0.5, aBottomLeft.V1());
        Assert.Equal(0.5, aTopLeft.V0());
        Assert.Equal(1.0, aTopLeft.V1());
    }

    [Fact]
    public void UpdateInU_PreservesPatchOrdersPerRow()
    {
        NCollection_Sequence<Patch> aNet = new();
        aNet.Append(new Patch(0.0, 1.0, 0.0, 1.0, 0, 1));
        aNet.Append(new Patch(1.0, 2.0, 0.0, 1.0, 2, 3));
        aNet.Append(new Patch(0.0, 1.0, 1.0, 2.0, 4, 5));
        aNet.Append(new Patch(1.0, 2.0, 1.0, 2.0, 6, 7));

        NCollection_Sequence<double> aUParams = new();
        aUParams.Append(0.0);
        aUParams.Append(1.0);
        aUParams.Append(2.0);

        NCollection_Sequence<double> aVParams = new();
        aVParams.Append(0.0);
        aVParams.Append(1.0);
        aVParams.Append(2.0);

        Network aNetwork = new(aNet, aUParams, aVParams);
        aNetwork.UpdateInU(0.5);

        Assert.Equal(6, aNetwork.PatchCount);
        Assert.Equal(3, aNetwork.PatchInUCount);
        Assert.Equal(2, aNetwork.PatchInVCount);
        Assert.Equal(0.5, aNetwork.UParameter(2));

        Patch aBottomLeft = aNetwork.Patch(1, 1);
        Patch aBottomMiddle = aNetwork.Patch(2, 1);
        Patch aBottomRight = aNetwork.Patch(3, 1);
        Patch aTopLeft = aNetwork.Patch(1, 2);
        Patch aTopMiddle = aNetwork.Patch(2, 2);
        Patch aTopRight = aNetwork.Patch(3, 2);

        Assert.Equal(0, aBottomLeft.UOrder());
        Assert.Equal(1, aBottomLeft.VOrder());
        Assert.Equal(0, aBottomMiddle.UOrder());
        Assert.Equal(1, aBottomMiddle.VOrder());
        Assert.Equal(2, aBottomRight.UOrder());
        Assert.Equal(3, aBottomRight.VOrder());

        Assert.Equal(4, aTopLeft.UOrder());
        Assert.Equal(5, aTopLeft.VOrder());
        Assert.Equal(4, aTopMiddle.UOrder());
        Assert.Equal(5, aTopMiddle.VOrder());
        Assert.Equal(6, aTopRight.UOrder());
        Assert.Equal(7, aTopRight.VOrder());

        Assert.Equal(0.0, aBottomLeft.U0());
        Assert.Equal(0.5, aBottomLeft.U1());
        Assert.Equal(0.5, aBottomMiddle.U0());
        Assert.Equal(1.0, aBottomMiddle.U1());
    }

    [Fact]
    public void SameDegree_RaisesEveryPatchToTheLargestNumbers()
    {
        NCollection_Sequence<Patch> aNet = new();
        Patch aP1 = new(0.0, 1.0, 0.0, 1.0, 1, 1);
        Patch aP2 = new(1.0, 2.0, 0.0, 1.0, 1, 1);
        aP1.ChangeNbCoeff(7, 3);
        aP2.ChangeNbCoeff(5, 9);
        aNet.Append(aP1);
        aNet.Append(aP2);
        NCollection_Sequence<double> aUParams = new();
        aUParams.Append(0.0);
        aUParams.Append(1.0);
        aUParams.Append(2.0);
        NCollection_Sequence<double> aVParams = new();
        aVParams.Append(0.0);
        aVParams.Append(1.0);
        Network aNetwork = new(aNet, aUParams, aVParams);

        aNetwork.SameDegree(1, 1, out int aNcfU, out int aNcfV);
        Assert.Equal(7, aNcfU);
        Assert.Equal(9, aNcfV);
        Assert.Equal(7, aP2.CoeffInUCount);
        Assert.Equal(9, aP1.CoeffInVCount);

        int anIndex = 0;
        Assert.True(aNetwork.FirstNotApprox(ref anIndex));
        Assert.Equal(1, anIndex);
    }
}
