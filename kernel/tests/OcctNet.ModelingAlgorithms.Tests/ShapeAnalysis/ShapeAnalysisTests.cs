// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>The ShapeAnalysis package statics: periods, wire bounds, signed areas, outer wires and UV bounds.</summary>
public class ShapeAnalysisTests
{
    private static TopoDS_Wire Polygon(bool theClose, params gp_Pnt[] thePoints)
    {
        var aMaker = new BRepBuilderAPI_MakePolygon();
        foreach (gp_Pnt aP in thePoints)
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
        gp_Pnt aP1 = new(0, 0, 0), aP2 = new(1, 0, 0), aP3 = new(1, 2, 0);
        TopoDS_Wire aW = Polygon(false, aP1, aP2, aP3);
        ShapeAnalysis.FindBounds(aW, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.True(BRep_Tool.Pnt(aV1).IsEqual(aP1, 0.0));
        Assert.True(BRep_Tool.Pnt(aV2).IsEqual(aP3, 0.0));

        TopoDS_Edge anE = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge().Reversed());
        ShapeAnalysis.FindBounds(anE, out aV1, out aV2);
        Assert.True(BRep_Tool.Pnt(aV1).IsEqual(aP2, 0.0));
        Assert.True(BRep_Tool.Pnt(aV2).IsEqual(aP1, 0.0));

        ShapeAnalysis.FindBounds(aV1, out TopoDS_Vertex aV3, out TopoDS_Vertex aV4);
        Assert.True(aV3.IsSame(aV1) && aV4.IsSame(aV1));

        TopoDS_Face aF = new BRepBuilderAPI_MakeFace(new gp_Pln()).Face();
        ShapeAnalysis.FindBounds(aF, out aV3, out aV4);
        Assert.True(aV3.IsNull() && aV4.IsNull());
    }

    [Fact]
    public void TotCross2D_and_ContourArea_OfSquares()
    {
        TopoDS_Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out TopoDS_Wire anOuter, out TopoDS_Wire aHole, out _);
        Assert.Equal(100.0, ShapeAnalysis.TotCross2D(new ShapeExtend_WireData(anOuter), aFace), 1e-9);
        Assert.Equal(-4.0, ShapeAnalysis.TotCross2D(new ShapeExtend_WireData(aHole), aFace), 1e-9);
        Assert.Equal(100.0, ShapeAnalysis.ContourArea(anOuter), 1e-9);
        Assert.Equal(4.0, ShapeAnalysis.ContourArea(aHole), 1e-9);
    }

    [Fact]
    public void OuterWire_and_IsOuterBound_WithPermutedWires()
    {
        // the face's wires are hole, outer, hole
        TopoDS_Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out TopoDS_Wire anOuter, out TopoDS_Wire aHole1, out _);
        Assert.True(ShapeAnalysis.OuterWire(aFace).IsSame(anOuter));
        Assert.True(ShapeAnalysis.OuterWire(TopoDS.Face(aFace.Reversed())).IsSame(anOuter));
        Assert.True(ShapeAnalysis.IsOuterBound(aFace));

        // a face with the hole only: not outer; its only wire is still returned
        var aB = new BRep_Builder();
        TopoDS_Face aHoleFace = new();
        aB.MakeFace(ref aHoleFace, BRep_Tool.Surface(aFace), 1e-7);
        aB.Add(aHoleFace, aHole1);
        Assert.False(ShapeAnalysis.IsOuterBound(aHoleFace));
        Assert.True(ShapeAnalysis.OuterWire(aHoleFace).IsSame(aHole1));

        // no wire at all
        Assert.True(ShapeAnalysis.OuterWire(new BRepBuilderAPI_MakeFace(new gp_Pln()).Face()).IsNull());
    }

    [Fact]
    public void OuterWire_OfPrimitiveFaces()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExp.Current());
            Assert.True(ShapeAnalysis.OuterWire(aF).IsSame(BRepTools.OuterWire(aF)));
            Assert.True(ShapeAnalysis.IsOuterBound(aF));
        }
    }

    [Fact]
    public void GetFaceUVBounds_OfPlaneCylinderAndSphere()
    {
        TopoDS_Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out _, out _, out _);
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
        var aB = new BRep_Builder();
        TopoDS_Face aBare = new();
        aB.MakeFace(ref aBare, new Geom_RectangularTrimmedSurface(new Geom_Plane(new gp_Pln()), -1, 2, -3, 4), 1e-7);
        ShapeAnalysis.GetFaceUVBounds(aBare, out aU1, out aU2, out aV1, out aV2);
        Assert.Equal([-1.0, 2.0, -3.0, 4.0], new[] { aU1, aU2, aV1, aV2 });
    }
}
