// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_Delaun_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

// Test for bug 0032395: BRepMesh_Delaun::isVertexInsidePolygon
// The method uses winding number algorithm to check if a point is inside a polygon.
// For CCW polygons, the cumulative angle is -2*PI, but the original code only
// checked for +2*PI. This test verifies that both orientations are handled correctly.
public class BRepMesh_DelaunTest
{
    private const string NeedsIncrementalMesh = "Needs BRepMesh_IncrementalMesh's face meshing (unit 4D); "
                                                + "BRepMeshDelaunTests.Square_with_a_square_hole covers the Delaunay part";

    [Fact]
    public void Vec2dAngleSignConvention()
    {
        // Verify gp_Vec2d::Angle sign convention used in isVertexInsidePolygon
        // The angle from aPrev to aCur is computed as aCur.Angle(aPrev)

        // Test CCW rotation: from (1,0) to (0,1) is 90 degrees CCW
        gp_Vec2d aVecRight = new(1.0, 0.0);
        gp_Vec2d aVecUp = new(0.0, 1.0);

        // aCur.Angle(aPrev) where aCur=up, aPrev=right
        // Going from right to up is CCW, angle should be negative in OCCT convention
        double anAngleCCW = aVecUp.Angle(aVecRight);
        Assert.True(anAngleCCW < 0.0, "CCW rotation should give negative angle in OCCT convention");
        Assert.Equal(-Math.PI / 2, anAngleCCW, Precision.Angular());

        // Test CW rotation: from (0,1) to (1,0) is 90 degrees CW
        double anAngleCW = aVecRight.Angle(aVecUp);
        Assert.True(anAngleCW > 0.0, "CW rotation should give positive angle in OCCT convention");
        Assert.Equal(Math.PI / 2, anAngleCW, Precision.Angular());
    }

    [Fact]
    public void WindingAngleCCWPolygon()
    {
        // Simulate the winding angle calculation for a CCW square polygon
        // with a point at the center
        // Vertices: (1,0), (0,1), (-1,0), (0,-1) - CCW order
        // Center point: (0,0)

        gp_XY aCenter = new(0.0, 0.0);
        gp_XY[] aVertices =
        [
            new(1.0, 0.0),
            new(0.0, 1.0),
            new(-1.0, 0.0),
            new(0.0, -1.0),
            new(1.0, 0.0), // Close the polygon
        ];

        gp_Vec2d aPrevDir = new(aVertices[0] - aCenter);
        double aTotalAngle = 0.0;

        for (int i = 1; i < 5; ++i)
        {
            gp_Vec2d aCurDir = new(aVertices[i] - aCenter);
            aTotalAngle += aCurDir.Angle(aPrevDir);
            aPrevDir = aCurDir;
        }

        // For CCW polygon with point inside, total angle should be -2*PI
        Assert.Equal(2.0 * Math.PI, Math.Abs(aTotalAngle), Precision.Angular());

        // The fix checks std::abs(std::abs(aTotalAng) - Angle2PI), so both +2PI and -2PI work
        double Angle2PI = 2.0 * Math.PI;
        bool isInsideOldFix = Math.Abs(Angle2PI - aTotalAngle) <= Precision.Angular();
        bool isInsideNewFix = Math.Abs(Math.Abs(aTotalAngle) - Angle2PI) <= Precision.Angular();

        Assert.False(isInsideOldFix, "Old check fails for CCW polygon (angle is negative)");
        Assert.True(isInsideNewFix, "New check handles both CCW and CW polygons");
    }

    [Fact]
    public void WindingAngleCWPolygon()
    {
        // Simulate the winding angle calculation for a CW square polygon
        // Vertices in CW order: (1,0), (0,-1), (-1,0), (0,1)
        // Center point: (0,0)

        gp_XY aCenter = new(0.0, 0.0);
        gp_XY[] aVertices =
        [
            new(1.0, 0.0),
            new(0.0, -1.0),
            new(-1.0, 0.0),
            new(0.0, 1.0),
            new(1.0, 0.0), // Close the polygon
        ];

        gp_Vec2d aPrevDir = new(aVertices[0] - aCenter);
        double aTotalAngle = 0.0;

        for (int i = 1; i < 5; ++i)
        {
            gp_Vec2d aCurDir = new(aVertices[i] - aCenter);
            aTotalAngle += aCurDir.Angle(aPrevDir);
            aPrevDir = aCurDir;
        }

        // For CW polygon with point inside, total angle should be +2*PI
        Assert.Equal(2.0 * Math.PI, aTotalAngle, Precision.Angular());

        double Angle2PI = 2.0 * Math.PI;
        bool isInsideOldFix = Math.Abs(Angle2PI - aTotalAngle) <= Precision.Angular();
        bool isInsideNewFix = Math.Abs(Math.Abs(aTotalAngle) - Angle2PI) <= Precision.Angular();

        Assert.True(isInsideOldFix, "Old check works for CW polygon");
        Assert.True(isInsideNewFix, "New check also works for CW polygon");
    }

    [Fact(Skip = NeedsIncrementalMesh)]
    public void MeshPlanarFaceWithHole()
    {
        // Create a planar face with a hole - this exercises the Delaunay triangulation
        // code path that uses isVertexInsidePolygon during mesh refinement

        // Outer wire: square 10x10 centered at origin
        gp_Pnt aP1 = new(-5.0, -5.0, 0.0);
        gp_Pnt aP2 = new(5.0, -5.0, 0.0);
        gp_Pnt aP3 = new(5.0, 5.0, 0.0);
        gp_Pnt aP4 = new(-5.0, 5.0, 0.0);

        BRepBuilderAPI_MakeWire anOuterWireMaker = new();
        anOuterWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge());
        anOuterWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP2, aP3).Edge());
        anOuterWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP3, aP4).Edge());
        anOuterWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP4, aP1).Edge());
        Assert.True(anOuterWireMaker.IsDone());

        // Inner wire (hole): smaller square 2x2 centered at origin
        gp_Pnt aH1 = new(-1.0, -1.0, 0.0);
        gp_Pnt aH2 = new(1.0, -1.0, 0.0);
        gp_Pnt aH3 = new(1.0, 1.0, 0.0);
        gp_Pnt aH4 = new(-1.0, 1.0, 0.0);

        BRepBuilderAPI_MakeWire anInnerWireMaker = new();
        anInnerWireMaker.Add(new BRepBuilderAPI_MakeEdge(aH1, aH2).Edge());
        anInnerWireMaker.Add(new BRepBuilderAPI_MakeEdge(aH2, aH3).Edge());
        anInnerWireMaker.Add(new BRepBuilderAPI_MakeEdge(aH3, aH4).Edge());
        anInnerWireMaker.Add(new BRepBuilderAPI_MakeEdge(aH4, aH1).Edge());
        Assert.True(anInnerWireMaker.IsDone());

        // Create face with hole
        Geom_Plane aPlane = new(new gp_Pln(gp.Origin(), gp.DZ()));
        BRepBuilderAPI_MakeFace aFaceMaker = new(aPlane, anOuterWireMaker.Wire());
        aFaceMaker.Add(anInnerWireMaker.Wire());
        Assert.True(aFaceMaker.IsDone());

        TopoDS_Face aFace = aFaceMaker.Face();

        // Mesh the face
        BRepMesh_IncrementalMesh aMesher = new(aFace, 0.1);
        Assert.True(aMesher.IsDone(), "Meshing should succeed");

        // Verify triangulation exists
        Poly_Triangulation? aTri = BRep_Tool.Triangulation(TopoDS.Face(aMesher.Shape()), out TopLoc_Location _);
        Assert.NotNull(aTri);
        Assert.True(aTri.NbTriangles() > 0, "Should have triangles");
        Assert.True(aTri.NbNodes() > 0, "Should have nodes");
    }

    [Fact(Skip = NeedsIncrementalMesh)]
    public void MeshBoxAllFaces()
    {
        // Create and mesh a box - tests triangulation on faces with different orientations
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

        BRepMesh_IncrementalMesh aMesher = new(aBox, 0.5);
        Assert.True(aMesher.IsDone(), "Meshing should succeed");

        // Verify all faces are triangulated
        int aFaceCount = 0;
        for (TopExp_Explorer anExp = new(aMesher.Shape(), TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            Poly_Triangulation? aTri = BRep_Tool.Triangulation(aFace, out TopLoc_Location _);

            Assert.True(aTri is not null, $"Face {aFaceCount} should have triangulation");
            Assert.True(aTri.NbTriangles() > 0, $"Face {aFaceCount} should have triangles");
            ++aFaceCount;
        }

        Assert.Equal(6, aFaceCount);
    }

    [Fact(Skip = NeedsIncrementalMesh)]
    public void MeshCylinderCurvedFaces()
    {
        // Create and mesh a cylinder - curved faces may have different polygon orientations
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape();

        BRepMesh_IncrementalMesh aMesher = new(aCylinder, 0.5);
        Assert.True(aMesher.IsDone(), "Meshing should succeed");

        // Verify faces are triangulated
        int aTotalTriangles = 0;
        for (TopExp_Explorer anExp = new(aMesher.Shape(), TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            Poly_Triangulation? aTri = BRep_Tool.Triangulation(aFace, out TopLoc_Location _);

            Assert.True(aTri is not null, "Face should have triangulation");
            aTotalTriangles += aTri.NbTriangles();
        }

        Assert.True(aTotalTriangles > 0, "Cylinder should have triangles");
    }
}
