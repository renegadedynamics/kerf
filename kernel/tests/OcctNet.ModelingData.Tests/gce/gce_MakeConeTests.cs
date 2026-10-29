// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeCone_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class gce_MakeConeTest
{
    private static gp_Cone BaseCone() => new(new gp_Ax3(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0))), M_PI / 6.0, 10.0);

    [Fact]
    public void FromConeAndDist_Valid_Done()
    {
        gp_Cone aBaseCone = BaseCone();
        double aDist = 2.0;

        gce_MakeCone aMaker = new(aBaseCone, aDist);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cone aCone = aMaker.Value();
        double anExpectedRadius = aBaseCone.RefRadius() + aDist / Math.Cos(aBaseCone.SemiAngle());
        Assert.Equal(anExpectedRadius, aCone.RefRadius(), Precision.Confusion());
        Assert.Equal(aBaseCone.SemiAngle(), aCone.SemiAngle(), Precision.Confusion());
        Assert.True(aCone.Location().IsEqual(aBaseCone.Location(), Precision.Confusion()));
        Assert.True(aCone.Axis().Direction().IsParallel(aBaseCone.Axis().Direction(), Precision.Angular()));
    }

    [Fact]
    public void FromConeAndDist_TooNegative_NegativeRadius()
    {
        gp_Cone aBaseCone = BaseCone();
        double aDist = -(aBaseCone.RefRadius() * Math.Cos(aBaseCone.SemiAngle()) + 1.0);

        gce_MakeCone aMaker = new(aBaseCone, aDist);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromConeAndPoint_Valid_Done()
    {
        gp_Cone aBaseCone = BaseCone();
        double aVParam = 4.0;
        double aTargetRadius = 12.0;
        double aPointRadius = aTargetRadius + aVParam * Math.Tan(aBaseCone.SemiAngle());
        gp_Pnt aPoint = new(aPointRadius, 0.0, aVParam);

        gce_MakeCone aMaker = new(aBaseCone, aPoint);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cone aCone = aMaker.Value();
        Assert.Equal(aTargetRadius, aCone.RefRadius(), Precision.Confusion());
    }

    [Fact]
    public void FromConeAndPoint_NoValidParallelCone_NegativeRadius()
    {
        gp_Cone aBaseCone = BaseCone();
        gp_Pnt aPoint = new(1.0, 0.0, 20.0);

        gce_MakeCone aMaker = new(aBaseCone, aPoint);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromConeAndPoint_TwoCandidates_ChoosesNearestToBaseCone()
    {
        gp_Cone aBaseCone = BaseCone();
        gp_Pnt aPoint = new(5.0, 0.0, -20.0);

        gce_MakeCone aMaker = new(aBaseCone, aPoint);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        double aRadius = new gp_Lin(aBaseCone.Axis()).Distance(aPoint);
        double aVParam = (aPoint.XYZ() - aBaseCone.Location().XYZ()).Dot(aBaseCone.Axis().Direction().XYZ());
        double aTan = Math.Tan(aBaseCone.SemiAngle());
        double aCos = Math.Cos(aBaseCone.SemiAngle());
        double aCand1 = aRadius - aVParam * aTan;
        double aCand2 = -aRadius - aVParam * aTan;
        double aDist1 = Math.Abs((aCand1 - aBaseCone.RefRadius()) * aCos);
        double aDist2 = Math.Abs((aCand2 - aBaseCone.RefRadius()) * aCos);
        double anExpR = (aDist1 <= aDist2) ? aCand1 : aCand2;

        Assert.Equal(anExpR, aMaker.Value().RefRadius(), Precision.Confusion());
    }

    [Fact]
    public void FromTwoPointsAndTwoRadii_YBranch_DoneAndOrthogonalXAxis()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, 10.0, 10.0);

        gce_MakeCone aMaker = new(aP1, aP2, 3.0, 2.0);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cone aCone = aMaker.Value();
        gp_Dir aAxis = new(aP2.XYZ() - aP1.XYZ());
        Assert.True(aCone.Axis().Direction().IsParallel(aAxis, Precision.Angular()));
        double aDot = aCone.Axis().Direction().XYZ().Dot(aCone.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, Precision.Angular());
    }

    [Fact]
    public void FromTwoPointsAndTwoRadii_ZBranch_DoneAndOrthogonalXAxis()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, 0.0, 10.0);

        gce_MakeCone aMaker = new(aP1, aP2, 4.0, 2.0);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cone aCone = aMaker.Value();
        Assert.True(aCone.Axis().Direction().IsParallel(new gp_Dir(aP2.XYZ() - aP1.XYZ()), Precision.Angular()));
        double aDot = aCone.Axis().Direction().XYZ().Dot(aCone.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, Precision.Angular());
    }

    [Fact]
    public void FromFourPoints_YBranch_DoneAndOrthogonalXAxis()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, 10.0, 10.0);
        gp_Pnt aP3 = new(3.0, 0.0, 0.0);
        gp_Pnt aP4 = new(5.0, 4.0, 4.0);

        gce_MakeCone aMaker = new(aP1, aP2, aP3, aP4);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cone aCone = aMaker.Value();
        gp_Dir aAxis = new(aP2.XYZ() - aP1.XYZ());
        Assert.True(aCone.Axis().Direction().IsParallel(aAxis, Precision.Angular()));
        double aDot = aCone.Axis().Direction().XYZ().Dot(aCone.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, Precision.Angular());
    }

    [Fact]
    public void FromFourPoints_ZBranch_DoneAndOrthogonalXAxis()
    {
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Pnt aP2 = new(0.0, 0.0, 10.0);
        gp_Pnt aP3 = new(4.0, 0.0, 0.0);
        gp_Pnt aP4 = new(2.0, 0.0, 5.0);

        gce_MakeCone aMaker = new(aP1, aP2, aP3, aP4);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        gp_Cone aCone = aMaker.Value();
        Assert.True(aCone.Axis().Direction().IsParallel(new gp_Dir(aP2.XYZ() - aP1.XYZ()), Precision.Angular()));
        double aDot = aCone.Axis().Direction().XYZ().Dot(aCone.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, Precision.Angular());
    }
}
