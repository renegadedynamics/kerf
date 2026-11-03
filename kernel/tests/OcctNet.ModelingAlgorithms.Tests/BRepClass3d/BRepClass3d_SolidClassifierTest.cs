// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepClass3d_SolidClassifier_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepClass3d_SolidClassifier GTests, on BRepPrimAPI_MakeBox's and MakeSphere's (frozen) solids.
/// </summary>
public class BRepClass3d_SolidClassifierTest
{
    private static Shape Box10() => new BRepPrimAPI_MakeBox(new Point3D(0.0, 0.0, 0.0), 10.0, 10.0, 10.0).Shape();

    [Fact]
    public void PointInside_Box()
    {
        Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new Point3D(5.0, 5.0, 5.0), global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(In, aClassifier.State()); // Point (5,5,5) should be inside the box
    }

    [Fact]
    public void PointOutside_Box()
    {
        Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new Point3D(20.0, 20.0, 20.0), global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(Out, aClassifier.State()); // Point (20,20,20) should be outside the box
    }

    [Fact]
    public void PointOnFace_Box()
    {
        Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new Point3D(5.0, 5.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(On, aClassifier.State()); // Point (5,5,0) should be on the bottom face of the box
    }

    [Fact]
    public void PointInside_Sphere()
    {
        Shape aSphere = new BRepPrimAPI_MakeSphere(10.0).Shape();

        var aClassifier = new BRepClass3d_SolidClassifier(aSphere);
        aClassifier.Perform(new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(In, aClassifier.State()); // Origin should be inside the sphere
    }

    [Fact]
    public void PerformInfinitePoint()
    {
        Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(Out, aClassifier.State()); // Infinite point should be outside the box
    }
}
