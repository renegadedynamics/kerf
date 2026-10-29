// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeSegment2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakeSegment2dTest
{
    [Fact]
    public void FromPoints_BelowResolution_ConfusedPoints()
    {
        GC_MakeSegment2d aMaker = new(new gp_Pnt2d(0.0, 0.0), new gp_Pnt2d(0.5 * gp.Resolution(), 0.0));
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_AboveResolution_Done()
    {
        GC_MakeSegment2d aMaker = new(new gp_Pnt2d(0.0, 0.0), new gp_Pnt2d(1.0e-150, 0.0));
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
    }

    [Fact]
    public void FromPoints_AtResolution_ConfusedPoints()
    {
        GC_MakeSegment2d aMaker = new(new gp_Pnt2d(0.0, 0.0), new gp_Pnt2d(gp.Resolution(), 0.0));
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPointDir_BelowResolutionParameter_ConfusedPoints()
    {
        GC_MakeSegment2d aMaker = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0),
                                      new gp_Pnt2d(0.5 * gp.Resolution(), 3.0));
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPointDir_AboveResolutionParameter_Done()
    {
        GC_MakeSegment2d aMaker = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0),
                                      new gp_Pnt2d(2.0 * gp.Resolution(), 3.0));
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
    }

    [Fact]
    public void FromPointDir_AtResolutionParameter_ConfusedPoints()
    {
        GC_MakeSegment2d aMaker = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0),
                                      new gp_Pnt2d(gp.Resolution(), 5.0));
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_ConfusedPoints, aMaker.Status());
    }
}
