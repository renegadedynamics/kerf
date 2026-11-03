// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.BOPAlgo_CheckStatus;
using static Occt.BOPAlgo_Operation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepAlgoAPI_Check (unit 6G): the validity of shapes for Boolean operations.</summary>
public class BRepAlgoAPI_CheckTests
{
    private static TopoDS_Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        BOPAlgo_CheckerSITests.Box(x, y, z, dx, dy, dz);

    [Fact]
    public void A_box_is_valid()
    {
        BRepAlgoAPI_Check aCheck = new(Box(0, 0, 0, 1, 1, 1));
        Assert.True(aCheck.IsValid());
        Assert.Empty(aCheck.Result());
    }

    [Fact]
    public void Two_overlapping_boxes_are_valid_for_any_operation()
    {
        foreach (BOPAlgo_Operation anOp in new[] { BOPAlgo_COMMON, BOPAlgo_FUSE, BOPAlgo_CUT, BOPAlgo_CUT21, BOPAlgo_SECTION, BOPAlgo_UNKNOWN })
        {
            Assert.True(new BRepAlgoAPI_Check(Box(0, 0, 0, 2, 2, 2), Box(1, 1, 1, 2, 2, 2), anOp).IsValid());
        }
    }

    [Fact]
    public void A_self_interfering_compound_is_invalid_unless_not_checked()
    {
        TopoDS_Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        BRepAlgoAPI_Check aCheck = new(aC);
        Assert.False(aCheck.IsValid());
        Assert.All(aCheck.Result(), r => Assert.Equal(BOPAlgo_SelfIntersect, r.GetCheckStatus()));
        Assert.True(new BRepAlgoAPI_Check(aC, true, false).IsValid());
    }

    [Fact]
    public void Types_invalid_for_the_operation()
    {
        TopoDS_Shape aVertex = new BRepBuilderAPI_MakeVertex(new Point3D(1, 2, 3)).Shape();
        BRepAlgoAPI_Check aCheck = new(Box(0, 0, 0, 1, 1, 1), aVertex, BOPAlgo_FUSE);
        Assert.False(aCheck.IsValid());
        Assert.Equal([BOPAlgo_BadType], aCheck.Result().Select(r => r.GetCheckStatus()));
        // an empty shape
        aCheck = new BRepAlgoAPI_Check(BOPAlgo_CheckerSITests.Compound());
        Assert.Equal([BOPAlgo_BadType], aCheck.Result().Select(r => r.GetCheckStatus()));
    }

    [Fact]
    public void A_topologically_invalid_shape_is_not_valid()
    {
        // a face whose wire isn't closed
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        TopoDS_Wire aWire = new BRepBuilderAPI_MakeWire(anEdge).Wire();
        TopoDS_Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(), aWire, true).Shape();
        BRepAlgoAPI_Check aCheck = new(aFace, false, false);
        Assert.False(aCheck.IsValid());
        Assert.Equal(BOPAlgo_NotValid, aCheck.Result().Last().GetCheckStatus());
        Assert.True(aCheck.Result().Last().GetShape1().IsSame(aFace));
        Assert.True(aCheck.Result().Last().GetShape2().IsNull());
    }

    [Fact]
    public void SetData_reuses_the_algorithm_and_clears_the_results()
    {
        TopoDS_Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        BRepAlgoAPI_Check aCheck = new();
        aCheck.SetData(aC);
        aCheck.Perform();
        Assert.False(aCheck.IsValid());
        aCheck.SetData(Box(0, 0, 0, 1, 1, 1), Box(2, 0, 0, 1, 1, 1), BOPAlgo_FUSE);
        Assert.True(aCheck.IsValid());
        aCheck.Perform();
        Assert.True(aCheck.IsValid());
    }
}
