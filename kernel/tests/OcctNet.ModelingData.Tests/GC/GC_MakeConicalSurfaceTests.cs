// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeConicalSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakeConicalSurfaceTest
{
    [Fact]
    public void FromAxis_AngleBelowResolution_BadAngle()
    {
        gp_Ax2 aAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        GC_MakeConicalSurface aMaker = new(aAxis, 0.5 * gp.Resolution(), 2.0);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_BadAngle, aMaker.Status());
    }

    [Fact]
    public void FromAxis_ValidAngle_Done()
    {
        gp_Ax2 aAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        GC_MakeConicalSurface aMaker = new(aAxis, M_PI / 6.0, 2.0);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
    }
}
