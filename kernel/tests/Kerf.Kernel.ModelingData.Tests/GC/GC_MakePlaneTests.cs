// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakePlane_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

public class GC_MakePlaneTest
{
    [Fact]
    public void FromPlaneAndDist_Done()
    {
        Plane3D aBase = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        MakePlane aMaker = new(aBase, 5.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.Equal(5.0, aMaker.Value().Pln().Location().Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromPlaneAndPoint_Done()
    {
        Plane3D aBase = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aPoint = new(2.0, -3.0, 7.0);
        MakePlane aMaker = new(aBase, aPoint);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.True(aMaker.Value().Pln().Location().IsEqual(aPoint, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void FromAxis_Done()
    {
        Axis3D aAxis = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 1.0, 0.0));
        MakePlane aMaker = new(aAxis);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.True(aMaker.Value().Pln().Axis().Direction().IsParallel(aAxis.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));
    }
}
