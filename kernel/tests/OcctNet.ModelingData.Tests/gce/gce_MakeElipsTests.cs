// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeElips_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

public class gce_MakeElipsTest
{
    [Fact]
    public void FromPoints_Valid_Done()
    {
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(10.0, 0.0, 0.0);
        Point3D aS2 = new(0.0, 5.0, 0.0);

        gce_MakeElips aMaker = new(aS1, aS2, aCenter);
        Assert.True(aMaker.IsDone());
        Assert.Equal(gce_Done, aMaker.Status());

        Ellipse3D anElips = aMaker.Value();
        Assert.Equal(10.0, anElips.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, anElips.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromPoints_Collinear_InvertAxis()
    {
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(10.0, 0.0, 0.0);
        Point3D aS2 = new(5.0, 0.0, 0.0);

        gce_MakeElips aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_InvertAxis, aMaker.Status());
    }

    [Fact]
    public void FromPoints_TinyScale_NullAxis()
    {
        double aRes = Constants.Resolution();
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(2.0 * aRes, 0.0, 0.0);
        Point3D aS2 = new(0.0, 1.1 * aRes, 0.0);

        gce_MakeElips aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_NullAxis, aMaker.Status());
    }
}
