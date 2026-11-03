// Port of: src/ModelingData/TKGeomBase/GTests/AdvApp2Var_Iso_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using static Kerf.Engine.Geometry.IsoType;

namespace Kerf.Kernel.ModelingData.Tests;

public class AdvApp2Var_IsoTest
{
    [Fact]
    public void ConstructorWithOrders_AssignsOrdersByIsoType()
    {
        Iso anIsoU = new(IsoU, 3, 1);
        Assert.Equal(IsoU, anIsoU.Type());
        Assert.Equal(3, anIsoU.UOrder());
        Assert.Equal(1, anIsoU.VOrder());
        Assert.Equal(0.5, anIsoU.Constante());
        Assert.Equal(0.0, anIsoU.U0());
        Assert.Equal(1.0, anIsoU.U1());
        Assert.Equal(0.0, anIsoU.V0());
        Assert.Equal(1.0, anIsoU.V1());
        Assert.Equal(0.0, anIsoU.T0());
        Assert.Equal(1.0, anIsoU.T1());
        Assert.Equal(0, anIsoU.Position());

        Iso anIsoV = new(IsoV, 3, 1);
        Assert.Equal(IsoV, anIsoV.Type());
        Assert.Equal(3, anIsoV.UOrder());
        Assert.Equal(1, anIsoV.VOrder());
    }
}
