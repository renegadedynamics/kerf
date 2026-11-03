// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>The ShapeAnalysis package statics: periods, wire bounds, signed areas, outer wires and UV bounds.</summary>
public class ShapeAnalysisTests
{
    private static Wire Polygon(bool theClose, params Point3D[] thePoints)
    {
        var aMaker = new BRepBuilderAPI_MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aMaker.Add(aP);
        }

        if (theClose)
        {
            aMaker.Close();
        }

        return aMaker.Wire();
    }

    [Fact]
    public void AdjustByPeriod_and_AdjustToPeriod()
    {
        Assert.Equal(0.0, ShapeAnalysis.AdjustByPeriod(1.0, 0.0, 2 * Math.PI));
        Assert.Equal(-2 * Math.PI, ShapeAnalysis.AdjustByPeriod(7.0, 0.0, 2 * Math.PI));
        Assert.Equal(4 * Math.PI, ShapeAnalysis.AdjustByPeriod(-11.0, 0.0, 2 * Math.PI));
        Assert.Equal(3.0, ShapeAnalysis.AdjustByPeriod(3.0, 0.0, 1e-200)); // no period: the difference
        Assert.Equal(-10.0, ShapeAnalysis.AdjustToPeriod(12.0, 0.0, 10.0));
        Assert.Equal(0.0, ShapeAnalysis.AdjustToPeriod(5.0, 0.0, 10.0));
    }

    [Fact]
    public void FindBounds_OfWireEdgeAndVertex()
    {
        Point3D aP1 = new(0, 0, 0), aP2 = new(1, 0, 0), aP3 = new(1, 2, 0);
        Wire aW = Polygon(false, aP1, aP2, aP3);
        ShapeAnalysis.FindBounds(aW, out Vertex aV1, out Vertex aV2);
        Assert.True(Tool.Pnt(aV1).IsEqual(aP1, 0.0));
        Assert.True(Tool.Pnt(aV2).IsEqual(aP3, 0.0));

        Edge anE = ShapeCasts.Edge(new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge().Reversed());
        ShapeAnalysis.FindBounds(anE, out aV1, out aV2);
        Assert.True(Tool.Pnt(aV1).IsEqual(aP2, 0.0));
        Assert.True(Tool.Pnt(aV2).IsEqual(aP1, 0.0));

        ShapeAnalysis.FindBounds(aV1, out Vertex aV3, out Vertex aV4);
        Assert.True(aV3.IsSame(aV1) && aV4.IsSame(aV1));

        Face aF = new BRepBuilderAPI_MakeFace(new Plane3D()).Face();
        ShapeAnalysis.FindBounds(aF, out aV3, out aV4);
        Assert.True(aV3.IsNull && aV4.IsNull);
    }

    [Fact]
    public void TotCross2D_and_ContourArea_OfSquares()
    {
        Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out Wire anOuter, out Wire aHole, out _);
        Assert.Equal(100.0, ShapeAnalysis.TotCross2D(new ShapeExtend_WireData(anOuter), aFace), 1e-9);
        Assert.Equal(-4.0, ShapeAnalysis.TotCross2D(new ShapeExtend_WireData(aHole), aFace), 1e-9);
        Assert.Equal(100.0, ShapeAnalysis.ContourArea(anOuter), 1e-9);
        Assert.Equal(4.0, ShapeAnalysis.ContourArea(aHole), 1e-9);
    }

    [Fact]
    public void OuterWire_and_IsOuterBound_WithPermutedWires()
    {
        // the face's wires are hole, outer, hole
        Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out Wire anOuter, out Wire aHole1, out _);
        Assert.True(ShapeAnalysis.OuterWire(aFace).IsSame(anOuter));
        Assert.True(ShapeAnalysis.OuterWire(ShapeCasts.Face(aFace.Reversed())).IsSame(anOuter));
        Assert.True(ShapeAnalysis.IsOuterBound(aFace));

        // a face with the hole only: not outer; its only wire is still returned
        var aB = new BrepBuilder();
        Face aHoleFace = new();
        aB.MakeFace(ref aHoleFace, Tool.Surface(aFace), 1e-7);
        aB.Add(aHoleFace, aHole1);
        Assert.False(ShapeAnalysis.IsOuterBound(aHoleFace));
        Assert.True(ShapeAnalysis.OuterWire(aHoleFace).IsSame(aHole1));

        // no wire at all
        Assert.True(ShapeAnalysis.OuterWire(new BRepBuilderAPI_MakeFace(new Plane3D()).Face()).IsNull);
    }

    [Fact]
    public void OuterWire_OfPrimitiveFaces()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aF = ShapeCasts.Face(anExp.Current());
            Assert.True(ShapeAnalysis.OuterWire(aF).IsSame(BrepTools.OuterWire(aF)));
            Assert.True(ShapeAnalysis.IsOuterBound(aF));
        }
    }

    [Fact]
    public void GetFaceUVBounds_OfPlaneCylinderAndSphere()
    {
        Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out _, out _, out _);
        ShapeAnalysis.GetFaceUVBounds(aFace, out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.0, aU1, 1e-9);
        Assert.Equal(10.0, aU2, 1e-9);
        Assert.Equal(0.0, aV1, 1e-9);
        Assert.Equal(10.0, aV2, 1e-9);

        ShapeAnalysis.GetFaceUVBounds(new BRepPrimAPI_MakeCylinder(2, 5).Face(), out aU1, out aU2, out aV1, out aV2);
        Assert.Equal(0.0, aU1, 1e-9);
        Assert.Equal(2 * Math.PI, aU2, 1e-9);
        Assert.Equal(0.0, aV1, 1e-9);
        Assert.Equal(5.0, aV2, 1e-9);

        ShapeAnalysis.GetFaceUVBounds(new BRepPrimAPI_MakeSphere(3).Face(), out aU1, out aU2, out aV1, out aV2);
        Assert.Equal(0.0, aU1, 1e-9);
        Assert.Equal(2 * Math.PI, aU2, 1e-9);
        Assert.Equal(-Math.PI / 2, aV1, 1e-9);
        Assert.Equal(Math.PI / 2, aV2, 1e-9);

        // no edges: the surface's bounds
        var aB = new BrepBuilder();
        Face aBare = new();
        aB.MakeFace(ref aBare, new TrimmedSurface(new PlaneSurface(new Plane3D()), -1, 2, -3, 4), 1e-7);
        ShapeAnalysis.GetFaceUVBounds(aBare, out aU1, out aU2, out aV1, out aV2);
        Assert.Equal([-1.0, 2.0, -3.0, 4.0], new[] { aU1, aU2, aV1, aV2 });
    }
}
