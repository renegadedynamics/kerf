// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeCylinder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

public class gce_MakeCylinderTest
{
    [Fact]
    public void FromAxis_YBranch_DoneAndOrthogonalXAxis()
    {
        Axis3D aAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 1.0));

        MakeCylinder aMaker = new(aAxis, 5.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cylinder aCylinder = aMaker.Value();
        Assert.Equal(5.0, aCylinder.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aCylinder.Axis().Direction().IsParallel(aAxis.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));

        double aDot = aCylinder.Axis().Direction().XYZ().Dot(aCylinder.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void FromPoints_ZBranch_DoneAndOrthogonalXAxis()
    {
        Point3D aP1 = new(0.0, 0.0, 0.0);
        Point3D aP2 = new(0.0, 0.0, 10.0);
        Point3D aP3 = new(3.0, 4.0, 0.0);

        MakeCylinder aMaker = new(aP1, aP2, aP3);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Cylinder aCylinder = aMaker.Value();
        Assert.Equal(5.0, aCylinder.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aCylinder.Axis().Direction().IsParallel(new Direction3D(aP2.XYZ() - aP1.XYZ()), global::Kerf.Engine.Foundation.Precision.Angular()));

        double aDot = aCylinder.Axis().Direction().XYZ().Dot(aCylinder.Position().XDirection().XYZ());
        Assert.Equal(0.0, aDot, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void FromAxis_NegativeRadius_NegativeRadius()
    {
        Axis3D aAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

        MakeCylinder aMaker = new(aAxis, -1.0);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }
}
