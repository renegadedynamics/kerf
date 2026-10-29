// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepExtrema_DistShapeShape_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepExtrema_DistShapeShape GTests, with the BRepBuilderAPI makers. The edges whose 3D curve OCCT removes
/// with BRep_Builder are built unfrozen (InternalEdge) and frozen after.
/// </summary>
public class BRepExtrema_DistShapeShapeTest
{
    // Bug BUC60870: BRepExtrema_DistShapeShape should correctly compute the minimum distance
    // between a linear edge and a vertex.
    // The edge runs from (0,0,0) to (0,1,0); the vertex is at (0,0.3,1).
    // The theoretical minimum distance is 1.0 (perpendicular from (0,0.3,1) to (0,0.3,0)).
    [Fact]
    public void BUC60870_EdgeToVertexMinimumDistance()
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 1, 0)).Edge();
        TopoDS_Vertex aVertex = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0.3, 1)).Vertex();

        var aDist = new BRepExtrema_DistShapeShape(anEdge, aVertex, 2.0);

        Assert.True(aDist.IsDone(), "BRepExtrema_DistShapeShape failed to compute");
        Assert.True(aDist.NbSolution() >= 1, "Expected at least one solution");

        const double aTheorDist = 1.0;
        const double aPercentMax = 0.01; // 1% relative tolerance
        Assert.Equal(aTheorDist, aDist.Value(), aTheorDist * aPercentMax);
    }

    // Verify that BRepExtrema_DistShapeShape does not crash when computing the distance
    // between two edges and one of them has no 3D curve (null Geom_Curve from BRep_Tool::Curve).
    // This exercises the null-check in the PERFORM_C0 helper.
    [Fact]
    public void EdgeEdge_Null3DCurve_NoCrash()
    {
        TopoDS_Edge aEdge1 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();

        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        TopoDS_Edge aEdge2 = new BRepBuilderAPI_MakeEdge(aCircle).InternalEdge();

        var aBuilder = new BRep_Builder();
        aBuilder.UpdateEdge(aEdge2, (Geom_Curve?)null, 1e-7);
        aEdge2.TShape()!.FreezeGraph();

        var aDist = new BRepExtrema_DistShapeShape(aEdge1, aEdge2, 10.0);
        _ = aDist;
    }

    // Verify that BRepExtrema_DistShapeShape does not crash when computing the distance
    // between an edge with no 3D curve and a face.
    // This exercises the null-check in Perform(Edge, Face).
    [Fact]
    public void EdgeFace_Null3DCurve_NoCrash()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        TopoDS_Edge aEdge = new BRepBuilderAPI_MakeEdge(aCircle).InternalEdge();

        var aBuilder = new BRep_Builder();
        aBuilder.UpdateEdge(aEdge, (Geom_Curve?)null, 1e-7);
        aEdge.TShape()!.FreezeGraph();

        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Pnt(0, 0, 5), new gp_Dir(0, 0, 1)), -10, 10, -10, 10).Face();

        var aDist = new BRepExtrema_DistShapeShape(aEdge, aFace, 10.0);
        _ = aDist;
    }
}
