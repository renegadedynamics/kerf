// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Booleans.CheckStatus;
using static Kerf.Engine.Booleans.Operation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepAlgoAPI_Check (unit 6G): the validity of shapes for Boolean operations.</summary>
public class BRepAlgoAPI_CheckTests
{
    private static Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        BOPAlgo_CheckerSITests.Box(x, y, z, dx, dy, dz);

    [Fact]
    public void A_box_is_valid()
    {
        Check aCheck = new(Box(0, 0, 0, 1, 1, 1));
        Assert.True(aCheck.IsValid);
        Assert.Empty(aCheck.Result());
    }

    [Fact]
    public void Two_overlapping_boxes_are_valid_for_any_operation()
    {
        foreach (Operation anOp in new[] { Operation.Common, Operation.Fuse, Operation.Cut, Operation.CUT21, Operation.Section, Operation.Unknown })
        {
            Assert.True(new Check(Box(0, 0, 0, 2, 2, 2), Box(1, 1, 1, 2, 2, 2), anOp).IsValid);
        }
    }

    [Fact]
    public void A_self_interfering_compound_is_invalid_unless_not_checked()
    {
        Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        Check aCheck = new(aC);
        Assert.False(aCheck.IsValid);
        Assert.All(aCheck.Result(), r => Assert.Equal(CheckStatus.SelfIntersect, r.GetCheckStatus()));
        Assert.True(new Check(aC, true, false).IsValid);
    }

    [Fact]
    public void Types_invalid_for_the_operation()
    {
        Shape aVertex = new MakeVertex(new Point3D(1, 2, 3)).Shape();
        Check aCheck = new(Box(0, 0, 0, 1, 1, 1), aVertex, Operation.Fuse);
        Assert.False(aCheck.IsValid);
        Assert.Equal([CheckStatus.BadType], aCheck.Result().Select(r => r.GetCheckStatus()));
        // an empty shape
        aCheck = new Check(BOPAlgo_CheckerSITests.Compound());
        Assert.Equal([CheckStatus.BadType], aCheck.Result().Select(r => r.GetCheckStatus()));
    }

    [Fact]
    public void A_topologically_invalid_shape_is_not_valid()
    {
        // a face whose wire isn't closed
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Wire aWire = new MakeWire(anEdge).Wire();
        Shape aFace = new MakeFace(new Plane3D(), aWire, true).Shape();
        Check aCheck = new(aFace, false, false);
        Assert.False(aCheck.IsValid);
        Assert.Equal(CheckStatus.NotValid, aCheck.Result().Last().GetCheckStatus());
        Assert.True(aCheck.Result().Last().GetShape1().IsSame(aFace));
        Assert.True(aCheck.Result().Last().GetShape2().IsNull);
    }

    [Fact]
    public void SetData_reuses_the_algorithm_and_clears_the_results()
    {
        Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        Check aCheck = new();
        aCheck.SetData(aC);
        aCheck.Perform();
        Assert.False(aCheck.IsValid);
        aCheck.SetData(Box(0, 0, 0, 1, 1, 1), Box(2, 0, 0, 1, 1, 1), Operation.Fuse);
        Assert.True(aCheck.IsValid);
        aCheck.Perform();
        Assert.True(aCheck.IsValid);
    }
}
