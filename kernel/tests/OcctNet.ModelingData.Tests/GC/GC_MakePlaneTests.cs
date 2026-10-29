// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakePlane_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakePlaneTest
{
    [Fact]
    public void FromPlaneAndDist_Done()
    {
        gp_Pln aBase = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        GC_MakePlane aMaker = new(aBase, 5.0);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.Equal(5.0, aMaker.Value().Pln().Location().Z(), Precision.Confusion());
    }

    [Fact]
    public void FromPlaneAndPoint_Done()
    {
        gp_Pln aBase = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aPoint = new(2.0, -3.0, 7.0);
        GC_MakePlane aMaker = new(aBase, aPoint);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.True(aMaker.Value().Pln().Location().IsEqual(aPoint, Precision.Confusion()));
    }

    [Fact]
    public void FromAxis_Done()
    {
        gp_Ax1 aAxis = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 1.0, 0.0));
        GC_MakePlane aMaker = new(aAxis);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());
        Assert.True(aMaker.Value().Pln().Axis().Direction().IsParallel(aAxis.Direction(), Precision.Angular()));
    }
}
