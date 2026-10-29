// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeCylinder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class gce_MakeCylinderTest
{
    [Fact]
    public void FromAxis_YBranch_DoneAndOrthogonalXAxis()
    {
        gp_Ax1 aAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 1.0));

        gce_MakeCylinder aMaker = new(aAxis, 5.0);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cylinder aCylinder = aMaker.Value();
        Assert.Equal(5.0, aCylinder.Radius(), Precision.Confusion());
        Assert.True(aCylinder.Axis().Direction().IsParallel(aAxis.Direction(), Precision.Angular()));

        double aDot = aCylinder.Axis().Direction().XYZ().Dot(aCylinder.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, Precision.Angular());
    }

    [Fact]
    public void FromPoints_ZBranch_DoneAndOrthogonalXAxis()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, 0.0, 10.0);
        gp_Pnt aP3 = new(3.0, 4.0, 0.0);

        gce_MakeCylinder aMaker = new(aP1, aP2, aP3);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cylinder aCylinder = aMaker.Value();
        Assert.Equal(5.0, aCylinder.Radius(), Precision.Confusion());
        Assert.True(aCylinder.Axis().Direction().IsParallel(new gp_Dir(aP2.XYZ() - aP1.XYZ()), Precision.Angular()));

        double aDot = aCylinder.Axis().Direction().XYZ().Dot(aCylinder.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, Precision.Angular());
    }

    [Fact]
    public void FromAxis_NegativeRadius_NegativeRadius()
    {
        gp_Ax1 aAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));

        gce_MakeCylinder aMaker = new(aAxis, -1.0);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_NegativeRadius, aMaker.Status());
    }
}
