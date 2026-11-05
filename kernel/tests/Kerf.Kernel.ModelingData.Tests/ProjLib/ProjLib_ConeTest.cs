// Port of: src/ModelingData/TKGeomBase/GTests/ProjLib_Cone_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ProjLib_ConeTest
{
    [Fact]
    public void ProjectCircle_ParallelAxes_DoneLine()
    {
        Cone3D aCone = new(new CoordinateSystem(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0))), M_PI / 6.0, 5.0);
        Circle3D aCircle = new(new Frame3D(new Point3D(0.0, 0.0, 10.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)),
                              2.0);

        ConeProjector aProjector = new(aCone, aCircle);
        Assert.True(aProjector.IsDone);
        Assert.Equal(CurveKind.Line, aProjector.GetType());
        Assert.Equal(0.0, aProjector.Line().Direction.Y, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void ProjectCircle_NonParallelAxes_NotDone()
    {
        Cone3D aCone = new(new CoordinateSystem(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0))), M_PI / 6.0, 5.0);
        Circle3D aCircle = new(new Frame3D(new Point3D(0.0, 0.0, 10.0), new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0)),
                              2.0);

        ConeProjector aProjector = new(aCone, aCircle);
        Assert.False(aProjector.IsDone);
    }

    [Fact]
    public void ProjectCircle_OppositeNormal_DoneLineWithNegativeDirection()
    {
        Cone3D aCone = new(new CoordinateSystem(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0))), M_PI / 6.0, 5.0);
        Circle3D aCircle = new(new Frame3D(new Point3D(0.0, 0.0, 6.0), new Direction3D(0.0, 0.0, -1.0), new Direction3D(1.0, 0.0, 0.0)),
                              2.0);

        ConeProjector aProjector = new(aCone, aCircle);
        Assert.True(aProjector.IsDone);
        Assert.Equal(CurveKind.Line, aProjector.GetType());
        Assert.True(aProjector.Line().Direction.X < 0.0);
    }
}
