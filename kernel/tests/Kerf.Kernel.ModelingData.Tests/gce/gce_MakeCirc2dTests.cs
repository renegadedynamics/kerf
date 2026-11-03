// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeCirc2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

public class gce_MakeCirc2dTest
{
    [Fact]
    public void ThreePoints_AllCoincidentWithinResolution_DoneZeroRadius()
    {
        // Intentional: points inside gp::Resolution() are treated as coincident
        // and should produce a valid zero-radius circle.
        double aDelta = 0.25 * Constants.Resolution();

        Point2D aP1 = new(0.0, 0.0);
        Point2D aP2 = new(aDelta, 0.0);
        Point2D aP3 = new(0.0, aDelta);

        MakeCirc2d aMaker = new(aP1, aP2, aP3);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.Equal(0.0, aMaker.Value().Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ThreePoints_OneNearCoincidentPair_Done()
    {
        Point2D aP1 = new(0.0, 0.0);
        Point2D aP2 = new(1.0e-150, 0.0);
        Point2D aP3 = new(0.0, 1.0);

        MakeCirc2d aMaker = new(aP1, aP2, aP3);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.True(aMaker.Value().Radius > 0.0);
    }

    [Fact]
    public void ThreePoints_SmallDistinct_Done()
    {
        Point2D aP1 = new(0.0, 0.0);
        Point2D aP2 = new(2.0e-150, 0.0);
        Point2D aP3 = new(0.0, 1.0);

        MakeCirc2d aMaker = new(aP1, aP2, aP3);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.True(aMaker.Value().Radius > 0.0);
    }
}
