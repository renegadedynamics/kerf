// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_IncrementalMesh_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

// A shape holding frozen sub-shapes is meshed on a copy, which the tests read from Shape() (decision 489).
public class BRepMesh_IncrementalMeshTest
{
    // Test OCC26407: BRepMesh_Delaun must not fail on a planar polygon with frontier edges.
    // The key check is that GetStatusFlags() == 0 (success) after meshing.
    // Migrated from QABugs_19.cxx OCC26407
    [Fact]
    public void OCC26407_PlanarPolygonMeshStatus()
    {
        // Hardcoded octagon-like polygon lying in the Z=88.5 plane
        Point3D[] aPnts =
        [
            new(587.90000000000009094947, 40.6758179230516248026106, 88.5),
            new(807.824182076948432040808, 260.599999999999965893949, 88.5),
            new(644.174182076948454778176, 424.249999999999943156581, 88.5000000000000142108547),
            new(629.978025792618950617907, 424.25, 88.5),
            new(793.628025792618700506864, 260.599999999999852207111, 88.5),
            new(587.900000000000204636308, 54.8719742073813492311274, 88.5),
            new(218.521974207381418864315, 424.250000000000056843419, 88.5),
            new(204.325817923051886282337, 424.249999999999943156581, 88.5),
        ];

        List<Vertex> aVertices = new(aPnts.Length);
        foreach (Point3D aPnt in aPnts)
        {
            aVertices.Add(new MakeVertex(aPnt).Vertex());
        }

        MakeWire aWireBuilder = new();
        for (int i = 0; i < aVertices.Count; ++i)
        {
            Vertex aV = aVertices[i];
            Vertex aW = aVertices[(i + 1) % aVertices.Count];
            aWireBuilder.Add(new MakeEdge(aV, aW).Edge());
        }
        Assert.True(aWireBuilder.IsDone, "Wire construction failed");

        Point3D aV0 = aPnts[0];
        Point3D aV1 = aPnts[1];
        Point3D aV2 = aPnts[^1];
        Vector3D aFaceNormal = new Vector3D(aV0, aV1).Crossed(new Vector3D(aV0, aV2));

        Face aFace = new MakeFace(new Plane3D(aV0, new Direction3D(aFaceNormal)), aWireBuilder.Wire()).Face();

        IncrementalMesh aMesher = new(aFace, 1.0e-7);
        Assert.Equal(0, aMesher.GetStatusFlags());
    }

    [Fact]
    public void TrimmedCylinder_RespectsUVRangeAndDeflection()
    {
        const double THE_RADIUS = 5.0;
        const double THE_FIRST_U = 0.4;
        const double THE_LAST_U = 5.7;
        const double THE_FIRST_V = -2.0;
        const double THE_LAST_V = 7.0;
        const double THE_DEFLECTION = 0.05;

        CylindricalSurface aCylinder = new(new CoordinateSystem(Constants.Origin(), Constants.DZ()), THE_RADIUS);
        MakeFace aFaceBuilder = new(aCylinder,
                                                   THE_FIRST_U,
                                                   THE_LAST_U,
                                                   THE_FIRST_V,
                                                   THE_LAST_V,
                                                   global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFaceBuilder.IsDone);
        Face aFace = aFaceBuilder.Face();

        IncrementalMesh aMesher = new(aFace, THE_DEFLECTION);
        Assert.True(aMesher.IsDone);
        aFace = ShapeCasts.Face(aMesher.Shape()); // OCCT.NET: the meshed shape (a copy if the input holds frozen sub-shapes, decision 489)

        Triangulation? aTriangulation = Tool.Triangulation(aFace, out Placement aLocation);
        Assert.NotNull(aTriangulation);
        Assert.True(aTriangulation.HasUVNodes);
        Assert.True(aTriangulation.TriangleCount > 0);

        Transform3D aTrsf = aLocation.Transformation();
        for (int aNodeIdx = 1; aNodeIdx <= aTriangulation.NodeCount; ++aNodeIdx)
        {
            Point2D aUV = aTriangulation.UVNode(aNodeIdx);
            Assert.True(aUV.X() >= THE_FIRST_U - global::Kerf.Engine.Foundation.Precision.PConfusion());
            Assert.True(aUV.X() <= THE_LAST_U + global::Kerf.Engine.Foundation.Precision.PConfusion());
            Assert.True(aUV.Y() >= THE_FIRST_V - global::Kerf.Engine.Foundation.Precision.PConfusion());
            Assert.True(aUV.Y() <= THE_LAST_V + global::Kerf.Engine.Foundation.Precision.PConfusion());

            Point3D anExpectedPoint = aCylinder.Value(aUV.X(), aUV.Y());
            Point3D anActualPoint = aTriangulation.Node(aNodeIdx).Transformed(aTrsf);
            Assert.True(anActualPoint.Distance(anExpectedPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        for (int aTriangleIdx = 1; aTriangleIdx <= aTriangulation.TriangleCount; ++aTriangleIdx)
        {
            aTriangulation.Triangle(aTriangleIdx).Get(out int aNode1, out int aNode2, out int aNode3);

            Point2D aUV1 = aTriangulation.UVNode(aNode1);
            Point2D aUV2 = aTriangulation.UVNode(aNode2);
            Point2D aUV3 = aTriangulation.UVNode(aNode3);
            Point3D anExpectedPoint = aCylinder.Value((aUV1.X() + aUV2.X() + aUV3.X()) / 3.0,
                                                     (aUV1.Y() + aUV2.Y() + aUV3.Y()) / 3.0);
            Point3D aPoint1 = aTriangulation.Node(aNode1).Transformed(aTrsf);
            Point3D aPoint2 = aTriangulation.Node(aNode2).Transformed(aTrsf);
            Point3D aPoint3 = aTriangulation.Node(aNode3).Transformed(aTrsf);
            Point3D anActualPoint = new((aPoint1.XYZ() + aPoint2.XYZ() + aPoint3.XYZ()) / 3.0);
            Assert.True(anActualPoint.Distance(anExpectedPoint) <= THE_DEFLECTION);
        }
    }

    [Fact(Skip = "Fuses a box and a ring with BRepAlgoAPI_Fuse, which waits for phase 6 (Booleans)")]
    public void BooleanTrimmedTorus_CrossingSeamProducesDenseMesh()
    {
    }

    [Fact]
    public void ClosedCylinder_SeamPolygonsReferenceValidMeshNodes()
    {
        ApiMakeCylinder aCylinderBuilder = new(5.0, 10.0);
        Shape aCylinder = aCylinderBuilder.Shape();
        Assert.True(aCylinderBuilder.IsDone);

        IncrementalMesh aMesher = new(aCylinder, 0.1);
        Assert.True(aMesher.IsDone);
        // OCCT.NET: a maker's result may hold frozen sub-shapes; the meshed shape is Shape() (decision 489)
        aCylinder = aMesher.Shape();

        int aSeamEdges = 0;
        for (Explorer aFaceExplorer = new(aCylinder, ShapeType.Face); aFaceExplorer.More(); aFaceExplorer.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExplorer.Current());
            Triangulation? aTriangulation = Tool.Triangulation(aFace, out Placement aLocation);
            Assert.NotNull(aTriangulation);

            for (Explorer anEdgeExplorer = new(aFace, ShapeType.Edge); anEdgeExplorer.More(); anEdgeExplorer.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anEdgeExplorer.Current());
                if (!Tool.IsClosed(anEdge, aFace))
                {
                    continue;
                }

                PolygonOnTriangulation? aForwardPolygon =
                    Tool.PolygonOnTriangulation(anEdge, aTriangulation, aLocation);
                Edge aReversedEdge = ShapeCasts.Edge(anEdge.Reversed());
                PolygonOnTriangulation? aReversedPolygon =
                    Tool.PolygonOnTriangulation(aReversedEdge, aTriangulation, aLocation);
                Assert.NotNull(aForwardPolygon);
                Assert.NotNull(aReversedPolygon);

                for (int aNodeIndex = 1; aNodeIndex <= aForwardPolygon.NodeCount; ++aNodeIndex)
                {
                    Assert.True(aForwardPolygon.Node(aNodeIndex) >= 1);
                    Assert.True(aForwardPolygon.Node(aNodeIndex) <= aTriangulation.NodeCount);
                }
                for (int aNodeIndex = 1; aNodeIndex <= aReversedPolygon.NodeCount; ++aNodeIndex)
                {
                    Assert.True(aReversedPolygon.Node(aNodeIndex) >= 1);
                    Assert.True(aReversedPolygon.Node(aNodeIndex) <= aTriangulation.NodeCount);
                }

                ++aSeamEdges;
            }
        }

        Assert.True(aSeamEdges > 0);
    }
}
