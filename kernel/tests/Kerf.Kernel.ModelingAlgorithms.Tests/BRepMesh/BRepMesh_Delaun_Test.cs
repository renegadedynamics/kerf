// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_Delaun_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

// Test for bug 0032395: BRepMesh_Delaun::isVertexInsidePolygon
// The method uses winding number algorithm to check if a point is inside a polygon.
// For CCW polygons, the cumulative angle is -2*PI, but the original code only
// checked for +2*PI. This test verifies that both orientations are handled correctly.
public class BRepMesh_DelaunTest
{
    [Fact]
    public void Vec2dAngleSignConvention()
    {
        // Verify gp_Vec2d::Angle sign convention used in isVertexInsidePolygon
        // The angle from aPrev to aCur is computed as aCur.Angle(aPrev)

        // Test CCW rotation: from (1,0) to (0,1) is 90 degrees CCW
        Vector2D aVecRight = new(1.0, 0.0);
        Vector2D aVecUp = new(0.0, 1.0);

        // aCur.Angle(aPrev) where aCur=up, aPrev=right
        // Going from right to up is CCW, angle should be negative in OCCT convention
        double anAngleCCW = aVecUp.AngleTo(aVecRight);
        Assert.True(anAngleCCW < 0.0, "CCW rotation should give negative angle in OCCT convention");
        Assert.Equal(-Math.PI / 2, anAngleCCW, global::Kerf.Engine.Foundation.Precision.Angular());

        // Test CW rotation: from (0,1) to (1,0) is 90 degrees CW
        double anAngleCW = aVecRight.AngleTo(aVecUp);
        Assert.True(anAngleCW > 0.0, "CW rotation should give positive angle in OCCT convention");
        Assert.Equal(Math.PI / 2, anAngleCW, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    [Fact]
    public void WindingAngleCCWPolygon()
    {
        // Simulate the winding angle calculation for a CCW square polygon
        // with a point at the center
        // Vertices: (1,0), (0,1), (-1,0), (0,-1) - CCW order
        // Center point: (0,0)

        Xy aCenter = new(0.0, 0.0);
        Xy[] aVertices =
        [
            new(1.0, 0.0),
            new(0.0, 1.0),
            new(-1.0, 0.0),
            new(0.0, -1.0),
            new(1.0, 0.0), // Close the polygon
        ];

        Vector2D aPrevDir = new(aVertices[0] - aCenter);
        double aTotalAngle = 0.0;

        for (int i = 1; i < 5; ++i)
        {
            Vector2D aCurDir = new(aVertices[i] - aCenter);
            aTotalAngle += aCurDir.AngleTo(aPrevDir);
            aPrevDir = aCurDir;
        }

        // For CCW polygon with point inside, total angle should be -2*PI
        Assert.Equal(2.0 * Math.PI, Math.Abs(aTotalAngle), global::Kerf.Engine.Foundation.Precision.Angular());

        // The fix checks std::abs(std::abs(aTotalAng) - Angle2PI), so both +2PI and -2PI work
        double Angle2PI = 2.0 * Math.PI;
        bool isInsideOldFix = Math.Abs(Angle2PI - aTotalAngle) <= global::Kerf.Engine.Foundation.Precision.Angular();
        bool isInsideNewFix = Math.Abs(Math.Abs(aTotalAngle) - Angle2PI) <= global::Kerf.Engine.Foundation.Precision.Angular();

        Assert.False(isInsideOldFix, "Old check fails for CCW polygon (angle is negative)");
        Assert.True(isInsideNewFix, "New check handles both CCW and CW polygons");
    }

    [Fact]
    public void WindingAngleCWPolygon()
    {
        // Simulate the winding angle calculation for a CW square polygon
        // Vertices in CW order: (1,0), (0,-1), (-1,0), (0,1)
        // Center point: (0,0)

        Xy aCenter = new(0.0, 0.0);
        Xy[] aVertices =
        [
            new(1.0, 0.0),
            new(0.0, -1.0),
            new(-1.0, 0.0),
            new(0.0, 1.0),
            new(1.0, 0.0), // Close the polygon
        ];

        Vector2D aPrevDir = new(aVertices[0] - aCenter);
        double aTotalAngle = 0.0;

        for (int i = 1; i < 5; ++i)
        {
            Vector2D aCurDir = new(aVertices[i] - aCenter);
            aTotalAngle += aCurDir.AngleTo(aPrevDir);
            aPrevDir = aCurDir;
        }

        // For CW polygon with point inside, total angle should be +2*PI
        Assert.Equal(2.0 * Math.PI, aTotalAngle, global::Kerf.Engine.Foundation.Precision.Angular());

        double Angle2PI = 2.0 * Math.PI;
        bool isInsideOldFix = Math.Abs(Angle2PI - aTotalAngle) <= global::Kerf.Engine.Foundation.Precision.Angular();
        bool isInsideNewFix = Math.Abs(Math.Abs(aTotalAngle) - Angle2PI) <= global::Kerf.Engine.Foundation.Precision.Angular();

        Assert.True(isInsideOldFix, "Old check works for CW polygon");
        Assert.True(isInsideNewFix, "New check also works for CW polygon");
    }

    [Fact]
    public void MeshPlanarFaceWithHole()
    {
        // Create a planar face with a hole - this exercises the Delaunay triangulation
        // code path that uses isVertexInsidePolygon during mesh refinement

        // Outer wire: square 10x10 centered at origin
        Point3D aP1 = new(-5.0, -5.0, 0.0);
        Point3D aP2 = new(5.0, -5.0, 0.0);
        Point3D aP3 = new(5.0, 5.0, 0.0);
        Point3D aP4 = new(-5.0, 5.0, 0.0);

        MakeWire anOuterWireMaker = new();
        anOuterWireMaker.Add(new MakeEdge(aP1, aP2).Edge());
        anOuterWireMaker.Add(new MakeEdge(aP2, aP3).Edge());
        anOuterWireMaker.Add(new MakeEdge(aP3, aP4).Edge());
        anOuterWireMaker.Add(new MakeEdge(aP4, aP1).Edge());
        Assert.True(anOuterWireMaker.IsDone);

        // Inner wire (hole): smaller square 2x2 centered at origin
        Point3D aH1 = new(-1.0, -1.0, 0.0);
        Point3D aH2 = new(1.0, -1.0, 0.0);
        Point3D aH3 = new(1.0, 1.0, 0.0);
        Point3D aH4 = new(-1.0, 1.0, 0.0);

        MakeWire anInnerWireMaker = new();
        anInnerWireMaker.Add(new MakeEdge(aH1, aH2).Edge());
        anInnerWireMaker.Add(new MakeEdge(aH2, aH3).Edge());
        anInnerWireMaker.Add(new MakeEdge(aH3, aH4).Edge());
        anInnerWireMaker.Add(new MakeEdge(aH4, aH1).Edge());
        Assert.True(anInnerWireMaker.IsDone);

        // Create face with hole
        PlaneSurface aPlane = new(new Plane3D(Constants.Origin(), Constants.DZ()));
        MakeFace aFaceMaker = new(aPlane, anOuterWireMaker.Wire());
        aFaceMaker.Add(anInnerWireMaker.Wire());
        Assert.True(aFaceMaker.IsDone);

        Face aFace = aFaceMaker.Face();

        // Mesh the face
        IncrementalMesh aMesher = new(aFace, 0.1);
        Assert.True(aMesher.IsDone, "Meshing should succeed");

        // Verify triangulation exists
        Triangulation? aTri = Tool.Triangulation(ShapeCasts.Face(aMesher.Shape()), out Placement _);
        Assert.NotNull(aTri);
        Assert.True(aTri.TriangleCount > 0, "Should have triangles");
        Assert.True(aTri.NodeCount > 0, "Should have nodes");
    }

    [Fact]
    public void MeshBoxAllFaces()
    {
        // Create and mesh a box - tests triangulation on faces with different orientations
        Shape aBox = new MakeBox(10.0, 10.0, 10.0).Shape();

        IncrementalMesh aMesher = new(aBox, 0.5);
        Assert.True(aMesher.IsDone, "Meshing should succeed");

        // Verify all faces are triangulated
        int aFaceCount = 0;
        for (Explorer anExp = new(aMesher.Shape(), ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aFace = ShapeCasts.Face(anExp.Current());
            Triangulation? aTri = Tool.Triangulation(aFace, out Placement _);

            Assert.True(aTri is not null, $"Face {aFaceCount} should have triangulation");
            Assert.True(aTri.TriangleCount > 0, $"Face {aFaceCount} should have triangles");
            ++aFaceCount;
        }

        Assert.Equal(6, aFaceCount);
    }

    [Fact]
    public void MeshCylinderCurvedFaces()
    {
        // Create and mesh a cylinder - curved faces may have different polygon orientations
        Shape aCylinder = new ApiMakeCylinder(5.0, 10.0).Shape();

        IncrementalMesh aMesher = new(aCylinder, 0.5);
        Assert.True(aMesher.IsDone, "Meshing should succeed");

        // Verify faces are triangulated
        int aTotalTriangles = 0;
        for (Explorer anExp = new(aMesher.Shape(), ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aFace = ShapeCasts.Face(anExp.Current());
            Triangulation? aTri = Tool.Triangulation(aFace, out Placement _);

            Assert.True(aTri is not null, "Face should have triangulation");
            aTotalTriangles += aTri.TriangleCount;
        }

        Assert.True(aTotalTriangles > 0, "Cylinder should have triangles");
    }
}
