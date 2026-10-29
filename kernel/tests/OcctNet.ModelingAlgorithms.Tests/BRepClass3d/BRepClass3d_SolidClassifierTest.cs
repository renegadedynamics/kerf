// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepClass3d_SolidClassifier_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepClass3d_SolidClassifier GTests, on BRepPrimAPI_MakeBox's and MakeSphere's (frozen) solids.
/// </summary>
public class BRepClass3d_SolidClassifierTest
{
    private static TopoDS_Shape Box10() => new BRepPrimAPI_MakeBox(new gp_Pnt(0.0, 0.0, 0.0), 10.0, 10.0, 10.0).Shape();

    [Fact]
    public void PointInside_Box()
    {
        TopoDS_Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new gp_Pnt(5.0, 5.0, 5.0), Precision.Confusion());

        Assert.Equal(TopAbs_IN, aClassifier.State()); // Point (5,5,5) should be inside the box
    }

    [Fact]
    public void PointOutside_Box()
    {
        TopoDS_Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new gp_Pnt(20.0, 20.0, 20.0), Precision.Confusion());

        Assert.Equal(TopAbs_OUT, aClassifier.State()); // Point (20,20,20) should be outside the box
    }

    [Fact]
    public void PointOnFace_Box()
    {
        TopoDS_Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.Perform(new gp_Pnt(5.0, 5.0, 0.0), Precision.Confusion());

        Assert.Equal(TopAbs_ON, aClassifier.State()); // Point (5,5,0) should be on the bottom face of the box
    }

    [Fact]
    public void PointInside_Sphere()
    {
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(10.0).Shape();

        var aClassifier = new BRepClass3d_SolidClassifier(aSphere);
        aClassifier.Perform(new gp_Pnt(0.0, 0.0, 0.0), Precision.Confusion());

        Assert.Equal(TopAbs_IN, aClassifier.State()); // Origin should be inside the sphere
    }

    [Fact]
    public void PerformInfinitePoint()
    {
        TopoDS_Shape aBox = Box10();

        var aClassifier = new BRepClass3d_SolidClassifier(aBox);
        aClassifier.PerformInfinitePoint(Precision.Confusion());

        Assert.Equal(TopAbs_OUT, aClassifier.State()); // Infinite point should be outside the box
    }
}
