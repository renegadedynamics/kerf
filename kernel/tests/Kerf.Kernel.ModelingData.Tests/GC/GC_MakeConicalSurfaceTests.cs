// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeConicalSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

public class GC_MakeConicalSurfaceTest
{
    [Fact]
    public void FromAxis_AngleBelowResolution_BadAngle()
    {
        Frame3D aAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        MakeConicalSurface aMaker = new(aAxis, 0.5 * Constants.Resolution(), 2.0);
        Assert.False(aMaker.IsDone);
        Assert.Equal(BadAngle, aMaker.Status());
    }

    [Fact]
    public void FromAxis_ValidAngle_Done()
    {
        Frame3D aAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        MakeConicalSurface aMaker = new(aAxis, M_PI / 6.0, 2.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
    }
}
