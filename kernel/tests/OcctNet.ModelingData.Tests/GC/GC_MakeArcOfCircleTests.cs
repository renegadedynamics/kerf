// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeArcOfCircle_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakeArcOfCircleTest
{
    [Fact]
    public void FromPointsAndTangent_Done()
    {
        Point3D aP1 = new(1.0, 0.0, 0.0);
        Point3D aP2 = new(0.0, 1.0, 0.0);
        Vector3D aTangentAtP1 = new(0.0, 1.0, 0.0);
        MakeArcOfCircle aMaker = new(aP1, aTangentAtP1, aP2);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
    }

    [Fact]
    public void FromThreePoints_Done()
    {
        MakeArcOfCircle aMaker = new(new Point3D(1.0, 0.0, 0.0), new Point3D(0.0, 1.0, 0.0), new Point3D(-1.0, 0.0, 0.0));
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
    }

    [Fact]
    public void FromPointsAndTangent_ConfusionScale_Done()
    {
        double aScale = 2.0 * global::Kerf.Engine.Foundation.Precision.Confusion();
        Point3D aP1 = new(aScale, 0.0, 0.0);
        Point3D aP2 = new(0.0, aScale, 0.0);
        Vector3D aTangentAtP1 = new(0.0, 1.0, 0.0);
        MakeArcOfCircle aMaker = new(aP1, aTangentAtP1, aP2);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
        Assert.True(aMaker.Value().StartPoint().IsEqual(aP1, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }
}
