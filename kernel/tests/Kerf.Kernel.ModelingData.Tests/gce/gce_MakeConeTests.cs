// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeCone_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

public class gce_MakeConeTest
{
    private static Cone3D BaseCone() => new(new CoordinateSystem(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0))), M_PI / 6.0, 10.0);

    [Fact]
    public void FromConeAndDist_Valid_Done()
    {
        Cone3D aBaseCone = BaseCone();
        double aDist = 2.0;

        MakeCone aMaker = new(aBaseCone, aDist);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cone3D aCone = aMaker.Value();
        double anExpectedRadius = aBaseCone.RefRadius + aDist / Math.Cos(aBaseCone.SemiAngle);
        Assert.Equal(anExpectedRadius, aCone.RefRadius, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aBaseCone.SemiAngle, aCone.SemiAngle, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aCone.Location.IsEqual(aBaseCone.Location, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aCone.Axis.Direction.IsParallel(aBaseCone.Axis.Direction, global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void FromConeAndDist_TooNegative_NegativeRadius()
    {
        Cone3D aBaseCone = BaseCone();
        double aDist = -(aBaseCone.RefRadius * Math.Cos(aBaseCone.SemiAngle) + 1.0);

        MakeCone aMaker = new(aBaseCone, aDist);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromConeAndPoint_Valid_Done()
    {
        Cone3D aBaseCone = BaseCone();
        double aVParam = 4.0;
        double aTargetRadius = 12.0;
        double aPointRadius = aTargetRadius + aVParam * Math.Tan(aBaseCone.SemiAngle);
        Point3D aPoint = new(aPointRadius, 0.0, aVParam);

        MakeCone aMaker = new(aBaseCone, aPoint);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cone3D aCone = aMaker.Value();
        Assert.Equal(aTargetRadius, aCone.RefRadius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromConeAndPoint_NoValidParallelCone_NegativeRadius()
    {
        Cone3D aBaseCone = BaseCone();
        Point3D aPoint = new(1.0, 0.0, 20.0);

        MakeCone aMaker = new(aBaseCone, aPoint);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromConeAndPoint_TwoCandidates_ChoosesNearestToBaseCone()
    {
        Cone3D aBaseCone = BaseCone();
        Point3D aPoint = new(5.0, 0.0, -20.0);

        MakeCone aMaker = new(aBaseCone, aPoint);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        double aRadius = new Line3D(aBaseCone.Axis).DistanceTo(aPoint);
        double aVParam = (aPoint.XYZ() - aBaseCone.Location.XYZ()).Dot(aBaseCone.Axis.Direction.XYZ());
        double aTan = Math.Tan(aBaseCone.SemiAngle);
        double aCos = Math.Cos(aBaseCone.SemiAngle);
        double aCand1 = aRadius - aVParam * aTan;
        double aCand2 = -aRadius - aVParam * aTan;
        double aDist1 = Math.Abs((aCand1 - aBaseCone.RefRadius) * aCos);
        double aDist2 = Math.Abs((aCand2 - aBaseCone.RefRadius) * aCos);
        double anExpR = (aDist1 <= aDist2) ? aCand1 : aCand2;

        Assert.Equal(anExpR, aMaker.Value().RefRadius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromTwoPointsAndTwoRadii_YBranch_DoneAndOrthogonalXAxis()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(0.0, 10.0, 10.0);

        MakeCone aMaker = new(aP1, aP2, 3.0, 2.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cone3D aCone = aMaker.Value();
        Direction3D aAxis = new(aP2.XYZ() - aP1.XYZ());
        Assert.True(aCone.Axis.Direction.IsParallel(aAxis, global::Kerf.Engine.Foundation.Precision.Angular()));
        double aDot = aCone.Axis.Direction.XYZ().Dot(aCone.Position.XDirection.XYZ());
        Assert.Equal(0.0, aDot, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void FromTwoPointsAndTwoRadii_ZBranch_DoneAndOrthogonalXAxis()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(0.0, 0.0, 10.0);

        MakeCone aMaker = new(aP1, aP2, 4.0, 2.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cone3D aCone = aMaker.Value();
        Assert.True(aCone.Axis.Direction.IsParallel(new Direction3D(aP2.XYZ() - aP1.XYZ()), global::Kerf.Engine.Foundation.Precision.Angular()));
        double aDot = aCone.Axis.Direction.XYZ().Dot(aCone.Position.XDirection.XYZ());
        Assert.Equal(0.0, aDot, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void FromFourPoints_YBranch_DoneAndOrthogonalXAxis()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(0.0, 10.0, 10.0);
        Point3D aP3 = new(3.0, 0.0, 0.0);
        Point3D aP4 = new(5.0, 4.0, 4.0);

        MakeCone aMaker = new(aP1, aP2, aP3, aP4);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cone3D aCone = aMaker.Value();
        Direction3D aAxis = new(aP2.XYZ() - aP1.XYZ());
        Assert.True(aCone.Axis.Direction.IsParallel(aAxis, global::Kerf.Engine.Foundation.Precision.Angular()));
        double aDot = aCone.Axis.Direction.XYZ().Dot(aCone.Position.XDirection.XYZ());
        Assert.Equal(0.0, aDot, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void FromFourPoints_ZBranch_DoneAndOrthogonalXAxis()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(0.0, 0.0, 10.0);
        Point3D aP3 = new(4.0, 0.0, 0.0);
        Point3D aP4 = new(2.0, 0.0, 5.0);

        MakeCone aMaker = new(aP1, aP2, aP3, aP4);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cone3D aCone = aMaker.Value();
        Assert.True(aCone.Axis.Direction.IsParallel(new Direction3D(aP2.XYZ() - aP1.XYZ()), global::Kerf.Engine.Foundation.Precision.Angular()));
        double aDot = aCone.Axis.Direction.XYZ().Dot(aCone.Position.XDirection.XYZ());
        Assert.Equal(0.0, aDot, global::Kerf.Engine.Foundation.Precision.Angular());
    }
}
