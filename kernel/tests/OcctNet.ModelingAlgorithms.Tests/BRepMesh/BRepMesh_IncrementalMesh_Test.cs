// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_IncrementalMesh_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

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
        gp_Pnt[] aPnts =
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

        List<TopoDS_Vertex> aVertices = new(aPnts.Length);
        foreach (gp_Pnt aPnt in aPnts)
        {
            aVertices.Add(new BRepBuilderAPI_MakeVertex(aPnt).Vertex());
        }

        BRepBuilderAPI_MakeWire aWireBuilder = new();
        for (int i = 0; i < aVertices.Count; ++i)
        {
            TopoDS_Vertex aV = aVertices[i];
            TopoDS_Vertex aW = aVertices[(i + 1) % aVertices.Count];
            aWireBuilder.Add(new BRepBuilderAPI_MakeEdge(aV, aW).Edge());
        }
        Assert.True(aWireBuilder.IsDone(), "Wire construction failed");

        gp_Pnt aV0 = aPnts[0];
        gp_Pnt aV1 = aPnts[1];
        gp_Pnt aV2 = aPnts[^1];
        gp_Vec aFaceNormal = new gp_Vec(aV0, aV1).Crossed(new gp_Vec(aV0, aV2));

        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(aV0, new gp_Dir(aFaceNormal)), aWireBuilder.Wire()).Face();

        BRepMesh_IncrementalMesh aMesher = new(aFace, 1.0e-7);
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

        Geom_CylindricalSurface aCylinder = new(new gp_Ax3(gp.Origin(), gp.DZ()), THE_RADIUS);
        BRepBuilderAPI_MakeFace aFaceBuilder = new(aCylinder,
                                                   THE_FIRST_U,
                                                   THE_LAST_U,
                                                   THE_FIRST_V,
                                                   THE_LAST_V,
                                                   Precision.Confusion());
        Assert.True(aFaceBuilder.IsDone());
        TopoDS_Face aFace = aFaceBuilder.Face();

        BRepMesh_IncrementalMesh aMesher = new(aFace, THE_DEFLECTION);
        Assert.True(aMesher.IsDone());
        aFace = TopoDS.Face(aMesher.Shape()); // OCCT.NET: the meshed shape (a copy if the input holds frozen sub-shapes, decision 489)

        Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLocation);
        Assert.NotNull(aTriangulation);
        Assert.True(aTriangulation.HasUVNodes());
        Assert.True(aTriangulation.NbTriangles() > 0);

        gp_Trsf aTrsf = aLocation.Transformation();
        for (int aNodeIdx = 1; aNodeIdx <= aTriangulation.NbNodes(); ++aNodeIdx)
        {
            gp_Pnt2d aUV = aTriangulation.UVNode(aNodeIdx);
            Assert.True(aUV.X() >= THE_FIRST_U - Precision.PConfusion());
            Assert.True(aUV.X() <= THE_LAST_U + Precision.PConfusion());
            Assert.True(aUV.Y() >= THE_FIRST_V - Precision.PConfusion());
            Assert.True(aUV.Y() <= THE_LAST_V + Precision.PConfusion());

            gp_Pnt anExpectedPoint = aCylinder.Value(aUV.X(), aUV.Y());
            gp_Pnt anActualPoint = aTriangulation.Node(aNodeIdx).Transformed(aTrsf);
            Assert.True(anActualPoint.Distance(anExpectedPoint) <= Precision.Confusion());
        }

        for (int aTriangleIdx = 1; aTriangleIdx <= aTriangulation.NbTriangles(); ++aTriangleIdx)
        {
            aTriangulation.Triangle(aTriangleIdx).Get(out int aNode1, out int aNode2, out int aNode3);

            gp_Pnt2d aUV1 = aTriangulation.UVNode(aNode1);
            gp_Pnt2d aUV2 = aTriangulation.UVNode(aNode2);
            gp_Pnt2d aUV3 = aTriangulation.UVNode(aNode3);
            gp_Pnt anExpectedPoint = aCylinder.Value((aUV1.X() + aUV2.X() + aUV3.X()) / 3.0,
                                                     (aUV1.Y() + aUV2.Y() + aUV3.Y()) / 3.0);
            gp_Pnt aPoint1 = aTriangulation.Node(aNode1).Transformed(aTrsf);
            gp_Pnt aPoint2 = aTriangulation.Node(aNode2).Transformed(aTrsf);
            gp_Pnt aPoint3 = aTriangulation.Node(aNode3).Transformed(aTrsf);
            gp_Pnt anActualPoint = new((aPoint1.XYZ() + aPoint2.XYZ() + aPoint3.XYZ()) / 3.0);
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
        BRepPrimAPI_MakeCylinder aCylinderBuilder = new(5.0, 10.0);
        TopoDS_Shape aCylinder = aCylinderBuilder.Shape();
        Assert.True(aCylinderBuilder.IsDone());

        BRepMesh_IncrementalMesh aMesher = new(aCylinder, 0.1);
        Assert.True(aMesher.IsDone());
        // OCCT.NET: a maker's result may hold frozen sub-shapes; the meshed shape is Shape() (decision 489)
        aCylinder = aMesher.Shape();

        int aSeamEdges = 0;
        for (TopExp_Explorer aFaceExplorer = new(aCylinder, TopAbs_ShapeEnum.TopAbs_FACE); aFaceExplorer.More(); aFaceExplorer.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceExplorer.Current());
            Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLocation);
            Assert.NotNull(aTriangulation);

            for (TopExp_Explorer anEdgeExplorer = new(aFace, TopAbs_ShapeEnum.TopAbs_EDGE); anEdgeExplorer.More(); anEdgeExplorer.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExplorer.Current());
                if (!BRep_Tool.IsClosed(anEdge, aFace))
                {
                    continue;
                }

                Poly_PolygonOnTriangulation? aForwardPolygon =
                    BRep_Tool.PolygonOnTriangulation(anEdge, aTriangulation, aLocation);
                TopoDS_Edge aReversedEdge = TopoDS.Edge(anEdge.Reversed());
                Poly_PolygonOnTriangulation? aReversedPolygon =
                    BRep_Tool.PolygonOnTriangulation(aReversedEdge, aTriangulation, aLocation);
                Assert.NotNull(aForwardPolygon);
                Assert.NotNull(aReversedPolygon);

                for (int aNodeIndex = 1; aNodeIndex <= aForwardPolygon.NbNodes(); ++aNodeIndex)
                {
                    Assert.True(aForwardPolygon.Node(aNodeIndex) >= 1);
                    Assert.True(aForwardPolygon.Node(aNodeIndex) <= aTriangulation.NbNodes());
                }
                for (int aNodeIndex = 1; aNodeIndex <= aReversedPolygon.NbNodes(); ++aNodeIndex)
                {
                    Assert.True(aReversedPolygon.Node(aNodeIndex) >= 1);
                    Assert.True(aReversedPolygon.Node(aNodeIndex) <= aTriangulation.NbNodes());
                }

                ++aSeamEdges;
            }
        }

        Assert.True(aSeamEdges > 0);
    }
}
