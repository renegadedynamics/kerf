// Port of: src/ModelingData/TKGeomBase/GTests/ProjLib_Cone_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class ProjLib_ConeTest
{
    [Fact]
    public void ProjectCircle_ParallelAxes_DoneLine()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0))), M_PI / 6.0, 5.0);
        gp_Circ aCircle = new(new gp_Ax2(new gp_Pnt(0.0, 0.0, 10.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)),
                              2.0);

        ProjLib_Cone aProjector = new(aCone, aCircle);
        Assert.True(aProjector.IsDone());
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Line, aProjector.GetType());
        Assert.Equal(0.0, aProjector.Line().Direction().Y(), Precision.Angular());
    }

    [Fact]
    public void ProjectCircle_NonParallelAxes_NotDone()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0))), M_PI / 6.0, 5.0);
        gp_Circ aCircle = new(new gp_Ax2(new gp_Pnt(0.0, 0.0, 10.0), new gp_Dir(1.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 0.0)),
                              2.0);

        ProjLib_Cone aProjector = new(aCone, aCircle);
        Assert.False(aProjector.IsDone());
    }

    [Fact]
    public void ProjectCircle_OppositeNormal_DoneLineWithNegativeDirection()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0))), M_PI / 6.0, 5.0);
        gp_Circ aCircle = new(new gp_Ax2(new gp_Pnt(0.0, 0.0, 6.0), new gp_Dir(0.0, 0.0, -1.0), new gp_Dir(1.0, 0.0, 0.0)),
                              2.0);

        ProjLib_Cone aProjector = new(aCone, aCircle);
        Assert.True(aProjector.IsDone());
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Line, aProjector.GetType());
        Assert.True(aProjector.Line().Direction().X() < 0.0);
    }
}
