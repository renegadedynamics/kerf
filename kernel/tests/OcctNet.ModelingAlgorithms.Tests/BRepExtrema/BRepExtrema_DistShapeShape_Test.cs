// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepExtrema_DistShapeShape_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
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
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(0, 1, 0)).Edge();
        Vertex aVertex = new MakeVertex(new Point3D(0, 0.3, 1)).Vertex();

        var aDist = new DistanceShapeShape(anEdge, aVertex, 2.0);

        Assert.True(aDist.IsDone, "BRepExtrema_DistShapeShape failed to compute");
        Assert.True(aDist.SolutionCount >= 1, "Expected at least one solution");

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
        Edge aEdge1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();

        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Edge aEdge2 = new MakeEdge(aCircle).InternalEdge();

        var aBuilder = new BrepBuilder();
        aBuilder.UpdateEdge(aEdge2, (Curve3D?)null, 1e-7);
        aEdge2.TShape()!.FreezeGraph();

        var aDist = new DistanceShapeShape(aEdge1, aEdge2, 10.0);
        _ = aDist;
    }

    // Verify that BRepExtrema_DistShapeShape does not crash when computing the distance
    // between an edge with no 3D curve and a face.
    // This exercises the null-check in Perform(Edge, Face).
    [Fact]
    public void EdgeFace_Null3DCurve_NoCrash()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Edge aEdge = new MakeEdge(aCircle).InternalEdge();

        var aBuilder = new BrepBuilder();
        aBuilder.UpdateEdge(aEdge, (Curve3D?)null, 1e-7);
        aEdge.TShape()!.FreezeGraph();

        Face aFace = new MakeFace(new Plane3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), -10, 10, -10, 10).Face();

        var aDist = new DistanceShapeShape(aEdge, aFace, 10.0);
        _ = aDist;
    }
}
