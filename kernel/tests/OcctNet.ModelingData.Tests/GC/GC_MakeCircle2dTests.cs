// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeCircle2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakeCircle2dTest
{
    [Fact]
    public void FromCircAndPoint_Done()
    {
        gp_Circ2d aBase = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0);
        gp_Pnt2d aPoint = new(0.0, 10.0);
        GC_MakeCircle2d aMaker = new(aBase, aPoint);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.Equal(10.0, aMaker.Value().Circ2d().Radius(), Precision.Confusion());
    }

    [Fact]
    public void FromCircAndDist_Done()
    {
        gp_Circ2d aBase = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0);
        GC_MakeCircle2d aMaker = new(aBase, -2.0);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.Equal(3.0, aMaker.Value().Circ2d().Radius(), Precision.Confusion());
    }

    [Fact]
    public void FromCircAndPoint_AtCenter_DoneZeroRadius()
    {
        gp_Circ2d aBase = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0);
        GC_MakeCircle2d aMaker = new(aBase, new gp_Pnt2d(0.0, 0.0));
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
        Assert.Equal(0.0, aMaker.Value().Circ2d().Radius(), Precision.Confusion());
    }
}
