// Port of: src/FoundationClasses/TKMath/GTests/BVH_Tools_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class BVH_ToolsTest
{
    [Fact]
    public void PointBoxSquareDistance()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Point inside box
        double aDist1 = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(0.5, 0.5, 0.5), aBox);
        Assert.Equal(0.0, aDist1, Precision.Confusion());

        // Point on face
        double aDist2 = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(0.5, 0.5, 1.0), aBox);
        Assert.Equal(0.0, aDist2, Precision.Confusion());

        // Point outside box (distance = 1)
        double aDist3 = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(2.0, 0.5, 0.5), aBox);
        Assert.Equal(1.0, aDist3, Precision.Confusion());

        // Point at corner outside (distance^2 = 1 + 1 + 1 = 3)
        double aDist4 = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(2.0, 2.0, 2.0), aBox);
        Assert.Equal(3.0, aDist4, Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistance()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox3 = new(new BVH_Vec3d(0.5, 0.5, 0.5), new BVH_Vec3d(1.5, 1.5, 1.5));

        // Separated boxes
        double aDist1 = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(1.0, aDist1, Precision.Confusion());

        // Overlapping boxes
        double aDist2 = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox1, aBox3);
        Assert.Equal(0.0, aDist2, Precision.Confusion());
    }

    [Fact]
    public void PointBoxProjection()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Point inside - should return same point
        BVH_Vec3d aProj1 = BVH_Tools<double, BVH_Vec3d>.PointBoxProjection(new BVH_Vec3d(0.5, 0.5, 0.5), aBox);
        Assert.Equal(0.5, aProj1.x(), Precision.Confusion());
        Assert.Equal(0.5, aProj1.y(), Precision.Confusion());
        Assert.Equal(0.5, aProj1.z(), Precision.Confusion());

        // Point outside - should clamp to box
        BVH_Vec3d aProj2 = BVH_Tools<double, BVH_Vec3d>.PointBoxProjection(new BVH_Vec3d(2.0, 0.5, 0.5), aBox);
        Assert.Equal(1.0, aProj2.x(), Precision.Confusion());
        Assert.Equal(0.5, aProj2.y(), Precision.Confusion());
        Assert.Equal(0.5, aProj2.z(), Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersection()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray hitting the box
        bool aHit1 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, 0.5, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit1);
        Assert.Equal(1.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(2.0, aTimeLeave, Precision.Confusion());

        // Ray missing the box
        bool aHit2 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, 5.0, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit2);

        // Ray starting inside box
        bool aHit3 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(0.5, 0.5, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit3);
        Assert.True(aTimeEnter <= 0.0);
        Assert.Equal(0.5, aTimeLeave, Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersectionParallelRay()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray parallel to X-axis, passing through box
        bool aHit1 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, 0.5, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit1);

        // Ray parallel to X-axis, missing box (Y out of range)
        bool aHit2 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, 2.0, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit2);
    }

    [Fact]
    public void PointTriangleProjection()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(1.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 1.0, 0.0);

        // Point projects to vertex
        BVH_Vec3d aProj1 = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(-1.0, -1.0, 0.0),
                                                                                                                                          aNode0,
                                                                                                                                          aNode1,
                                                                                                                                          aNode2);
        Assert.Equal(0.0, aProj1.x(), Precision.Confusion());
        Assert.Equal(0.0, aProj1.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj1.z(), Precision.Confusion());

        // Point projects to edge
        BVH_Vec3d aProj2 = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(0.5, -1.0, 0.0),
                                                                                                                                          aNode0,
                                                                                                                                          aNode1,
                                                                                                                                          aNode2);
        Assert.Equal(0.5, aProj2.x(), Precision.Confusion());
        Assert.Equal(0.0, aProj2.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj2.z(), Precision.Confusion());

        // Point projects inside triangle
        BVH_Vec3d aProj3 = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(0.25, 0.25, 1.0),
                                                                                                                                          aNode0,
                                                                                                                                          aNode1,
                                                                                                                                          aNode2);
        Assert.Equal(0.25, aProj3.x(), Precision.Confusion());
        Assert.Equal(0.25, aProj3.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj3.z(), Precision.Confusion());
    }

    [Fact]
    public void PointBoxSquareDistance2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0));

        // Point inside box
        double aDist1 = BVH_Tools<double, BVH_Vec2d>.PointBoxSquareDistance(new BVH_Vec2d(0.5, 0.5), aBox);
        Assert.Equal(0.0, aDist1, Precision.Confusion());

        // Point outside box
        double aDist2 = BVH_Tools<double, BVH_Vec2d>.PointBoxSquareDistance(new BVH_Vec2d(2.0, 0.5), aBox);
        Assert.Equal(1.0, aDist2, Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersectionDiagonalRay()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Diagonal ray through box center
        BVH_Vec3d aDir = new(1.0, 1.0, 1.0);
        double aNorm = Math.Sqrt(3.0);
        aDir            = new BVH_Vec3d(aDir.x() / aNorm, aDir.y() / aNorm, aDir.z() / aNorm);

        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, -1.0, -1.0), aDir, aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
        Assert.True(aTimeEnter > 0.0);
        Assert.True(aTimeLeave > aTimeEnter);
    }

    [Fact]
    public void RayBoxIntersectionNegativeDirection()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray going in negative X direction
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(2.0, 0.5, 0.5), new BVH_Vec3d(-1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
        Assert.Equal(1.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(2.0, aTimeLeave, Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersectionTouchingEdge()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray touching edge of box
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, 0.0, 0.0), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
    }

    [Fact]
    public void RayBoxIntersectionTouchingCorner()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray through corner
        BVH_Vec3d aDir = new(1.0, 1.0, 1.0);
        double aNorm = Math.Sqrt(3.0);
        aDir            = new BVH_Vec3d(aDir.x() / aNorm, aDir.y() / aNorm, aDir.z() / aNorm);

        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, -1.0, -1.0), aDir, aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
    }

    [Fact]
    public void BoxBoxSquareDistanceTouching()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(1.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0));

        // Touching boxes (sharing a face)
        double aDist = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(0.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistanceOneInsideOther()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(10.0, 10.0, 10.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(3.0, 3.0, 3.0));

        // Small box inside large box
        double aDist = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(0.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistanceCornerToCorner()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 2.0), new BVH_Vec3d(3.0, 3.0, 3.0));

        // Distance from corner to corner: sqrt(1^2 + 1^2 + 1^2) = sqrt(3), squared = 3
        double aDist = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(3.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void PointBoxProjectionNegativeCoords()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-1.0, -1.0, -1.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Point far outside in negative direction
        BVH_Vec3d aProj = BVH_Tools<double, BVH_Vec3d>.PointBoxProjection(new BVH_Vec3d(-5.0, -5.0, -5.0), aBox);
        Assert.Equal(-1.0, aProj.x(), Precision.Confusion());
        Assert.Equal(-1.0, aProj.y(), Precision.Confusion());
        Assert.Equal(-1.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionOnEdge01()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(2.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 2.0, 0.0);

        // Point projects onto edge between Node0 and Node1
        BVH_Vec3d aProj = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(1.0, -1.0, 0.0),
                                                                                                                                        aNode0,
                                                                                                                                        aNode1,
                                                                                                                                        aNode2);
        Assert.Equal(1.0, aProj.x(), Precision.Confusion());
        Assert.Equal(0.0, aProj.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionOnEdge12()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(2.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 2.0, 0.0);

        // Point projects onto edge between Node1 and Node2
        BVH_Vec3d aProj =
            BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(2.0, 2.0, 0.0), aNode0, aNode1, aNode2);
        Assert.Equal(1.0, aProj.x(), Precision.Confusion());
        Assert.Equal(1.0, aProj.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionOnEdge20()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(2.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 2.0, 0.0);

        // Point projects onto edge between Node2 and Node0
        BVH_Vec3d aProj = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(-1.0, 1.0, 0.0),
                                                                                                                                        aNode0,
                                                                                                                                        aNode1,
                                                                                                                                        aNode2);
        Assert.Equal(0.0, aProj.x(), Precision.Confusion());
        Assert.Equal(1.0, aProj.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionVertex1()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(2.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 2.0, 0.0);

        // Point projects to Node1
        BVH_Vec3d aProj = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(3.0, -1.0, 0.0),
                                                                                                                                        aNode0,
                                                                                                                                        aNode1,
                                                                                                                                        aNode2);
        Assert.Equal(2.0, aProj.x(), Precision.Confusion());
        Assert.Equal(0.0, aProj.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionVertex2()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(2.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 2.0, 0.0);

        // Point projects to Node2
        BVH_Vec3d aProj = BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(-1.0, 3.0, 0.0),
                                                                                                                                        aNode0,
                                                                                                                                        aNode1,
                                                                                                                                        aNode2);
        Assert.Equal(0.0, aProj.x(), Precision.Confusion());
        Assert.Equal(2.0, aProj.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjection3D()
    {
        // Triangle in 3D space (not in XY plane)
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(1.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 1.0, 1.0);

        // Point above triangle center
        BVH_Vec3d aProj =
            BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(0.3, 0.3, 0.8), aNode0, aNode1, aNode2);

        // Should project inside triangle
        Assert.True(aProj.x() >= 0.0);
        Assert.True(aProj.y() >= 0.0);
        Assert.True(aProj.x() + aProj.y() <= 1.0 + Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistance2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox1 = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0));
        BVH_Box<double, BVH_Vec2d> aBox2 = new(new BVH_Vec2d(3.0, 0.0), new BVH_Vec2d(4.0, 1.0));

        // Distance = 2
        double aDist = BVH_Tools<double, BVH_Vec2d>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(4.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void PointBoxProjection2D()
    {
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0));

        // Point outside
        BVH_Vec2d aProj = BVH_Tools<double, BVH_Vec2d>.PointBoxProjection(new BVH_Vec2d(2.0, 2.0), aBox);
        Assert.Equal(1.0, aProj.x(), Precision.Confusion());
        Assert.Equal(1.0, aProj.y(), Precision.Confusion());
    }

    [Fact]
    public void PointBoxSquareDistanceAtVertex()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Point on vertex
        double aDist = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(0.0, 0.0, 0.0), aBox);
        Assert.Equal(0.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void PointBoxSquareDistanceAtEdge()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Point on edge
        double aDist = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(0.5, 0.0, 0.0), aBox);
        Assert.Equal(0.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void PointBoxSquareDistanceFloat()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        // Point inside box
        float aDist1 = BVH_Tools<float, BVH_Vec3f>.PointBoxSquareDistance(new BVH_Vec3f(0.5f, 0.5f, 0.5f), aBox);
        Assert.Equal(0.0f, aDist1, 1e-5f);

        // Point outside box
        float aDist2 = BVH_Tools<float, BVH_Vec3f>.PointBoxSquareDistance(new BVH_Vec3f(2.0f, 0.5f, 0.5f), aBox);
        Assert.Equal(1.0f, aDist2, 1e-5f);
    }

    [Fact]
    public void RayBoxIntersectionFloat()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        float aTimeEnter = 0, aTimeLeave = 0;

        // Ray hitting the box
        bool aHit = BVH_Tools<float, BVH_Vec3f>.RayBoxIntersection(new BVH_Vec3f(-1.0f, 0.5f, 0.5f), new BVH_Vec3f(1.0f, 0.0f, 0.0f), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
        Assert.Equal(1.0f, aTimeEnter, 1e-5f);
        Assert.Equal(2.0f, aTimeLeave, 1e-5f);
    }

    [Fact]
    public void RayBoxIntersectionBehindRay()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray pointing away from box
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(2.0, 0.5, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        // The box is behind the ray origin
        Assert.True(aTimeLeave < 0.0 || !aHit);
    }

    [Fact]
    public void RayBoxIntersectionYAxis()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray along Y axis
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(0.5, -1.0, 0.5), new BVH_Vec3d(0.0, 1.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
        Assert.Equal(1.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(2.0, aTimeLeave, Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersectionZAxis()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray along Z axis
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(0.5, 0.5, -1.0), new BVH_Vec3d(0.0, 0.0, 1.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
        Assert.Equal(1.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(2.0, aTimeLeave, Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistanceFloat()
    {
        BVH_Box<float, BVH_Vec3f> aBox1 = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));
        BVH_Box<float, BVH_Vec3f> aBox2 = new(new BVH_Vec3f(2.0f, 0.0f, 0.0f), new BVH_Vec3f(3.0f, 1.0f, 1.0f));

        float aDist = BVH_Tools<float, BVH_Vec3f>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(1.0f, aDist, 1e-5f);
    }

    [Fact]
    public void PointBoxProjectionFloat()
    {
        BVH_Box<float, BVH_Vec3f> aBox = new(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f));

        BVH_Vec3f aProj = BVH_Tools<float, BVH_Vec3f>.PointBoxProjection(new BVH_Vec3f(2.0f, 0.5f, 0.5f), aBox);
        Assert.Equal(1.0f, aProj.x(), 1e-5f);
        Assert.Equal(0.5f, aProj.y(), 1e-5f);
        Assert.Equal(0.5f, aProj.z(), 1e-5f);
    }

    [Fact]
    public void PointBoxSquareDistanceNegativeBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(-2.0, -2.0, -2.0), new BVH_Vec3d(-1.0, -1.0, -1.0));

        // Point at origin
        double aDist = BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(0.0, 0.0, 0.0), aBox);
        // Distance = sqrt(1^2 + 1^2 + 1^2) = sqrt(3), squared = 3
        Assert.Equal(3.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistanceEdgeToEdge()
    {
        BVH_Box<double, BVH_Vec3d> aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        BVH_Box<double, BVH_Vec3d> aBox2 = new(new BVH_Vec3d(2.0, 2.0, 0.0), new BVH_Vec3d(3.0, 3.0, 1.0));

        // Closest points are on edges, distance = sqrt(1^2 + 1^2) = sqrt(2), squared = 2
        double aDist = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox1, aBox2);
        Assert.Equal(2.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionFloat()
    {
        BVH_Vec3f aNode0 = new(0.0f, 0.0f, 0.0f);
        BVH_Vec3f aNode1 = new(1.0f, 0.0f, 0.0f);
        BVH_Vec3f aNode2 = new(0.0f, 1.0f, 0.0f);

        // Point projects inside triangle
        BVH_Vec3f aProj = BVH_Tools<float, BVH_Vec3f>.PointTriangleProjection(new BVH_Vec3f(0.25f, 0.25f, 1.0f),
                                                                                                                                      aNode0,
                                                                                                                                      aNode1,
                                                                                                                                      aNode2);
        Assert.Equal(0.25f, aProj.x(), 1e-5f);
        Assert.Equal(0.25f, aProj.y(), 1e-5f);
        Assert.Equal(0.0f, aProj.z(), 1e-5f);
    }

    [Fact]
    public void PointBoxSquareDistanceLargeBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1000.0, 1000.0, 1000.0));

        // Point inside
        double aDist1 =
            BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(500.0, 500.0, 500.0), aBox);
        Assert.Equal(0.0, aDist1, Precision.Confusion());

        // Point outside
        double aDist2 =
            BVH_Tools<double, BVH_Vec3d>.PointBoxSquareDistance(new BVH_Vec3d(1001.0, 500.0, 500.0), aBox);
        Assert.Equal(1.0, aDist2, Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersectionLargeBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1000.0, 1000.0, 1000.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-100.0, 500.0, 500.0), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit);
        Assert.Equal(100.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(1100.0, aTimeLeave, Precision.Confusion());
    }

    [Fact]
    public void PointTriangleProjectionCentroid()
    {
        BVH_Vec3d aNode0 = new(0.0, 0.0, 0.0);
        BVH_Vec3d aNode1 = new(3.0, 0.0, 0.0);
        BVH_Vec3d aNode2 = new(0.0, 3.0, 0.0);

        // Point directly above centroid
        BVH_Vec3d aProj =
            BVH_Tools<double, BVH_Vec3d>.PointTriangleProjection(new BVH_Vec3d(1.0, 1.0, 5.0), aNode0, aNode1, aNode2);
        Assert.Equal(1.0, aProj.x(), Precision.Confusion());
        Assert.Equal(1.0, aProj.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj.z(), Precision.Confusion());
    }

    [Fact]
    public void BoxBoxSquareDistanceSameBox()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Distance to itself should be 0
        double aDist = BVH_Tools<double, BVH_Vec3d>.BoxBoxSquareDistance(aBox, aBox);
        Assert.Equal(0.0, aDist, Precision.Confusion());
    }

    [Fact]
    public void PointBoxProjectionAllCorners()
    {
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        // Test projection from each octant
        BVH_Vec3d aProj1 = BVH_Tools<double, BVH_Vec3d>.PointBoxProjection(new BVH_Vec3d(-1.0, -1.0, -1.0), aBox);
        Assert.Equal(0.0, aProj1.x(), Precision.Confusion());
        Assert.Equal(0.0, aProj1.y(), Precision.Confusion());
        Assert.Equal(0.0, aProj1.z(), Precision.Confusion());

        BVH_Vec3d aProj2 = BVH_Tools<double, BVH_Vec3d>.PointBoxProjection(new BVH_Vec3d(2.0, 2.0, 2.0), aBox);
        Assert.Equal(1.0, aProj2.x(), Precision.Confusion());
        Assert.Equal(1.0, aProj2.y(), Precision.Confusion());
        Assert.Equal(1.0, aProj2.z(), Precision.Confusion());
    }

    // =======================================================================================
    // Tests for improved RayBoxIntersection (dimension-independent, early exit)
    // =======================================================================================

    [Fact]
    public void RayBoxIntersection2D()
    {
        // Test 2D ray-box intersection (old code was hardcoded for 3D)
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0));

        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray hitting the box
        bool aHit1 = BVH_Tools<double, BVH_Vec2d>.RayBoxIntersection(new BVH_Vec2d(-1.0, 0.5), new BVH_Vec2d(1.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit1);
        Assert.Equal(1.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(2.0, aTimeLeave, Precision.Confusion());

        // Ray missing the box
        bool aHit2 = BVH_Tools<double, BVH_Vec2d>.RayBoxIntersection(new BVH_Vec2d(-1.0, 2.0), new BVH_Vec2d(1.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit2);

        // Ray parallel to X axis, inside Y bounds
        bool aHit3 = BVH_Tools<double, BVH_Vec2d>.RayBoxIntersection(new BVH_Vec2d(-1.0, 0.5), new BVH_Vec2d(1.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit3);

        // Ray parallel to Y axis, inside X bounds
        bool aHit4 = BVH_Tools<double, BVH_Vec2d>.RayBoxIntersection(new BVH_Vec2d(0.5, -1.0), new BVH_Vec2d(0.0, 1.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit4);
        Assert.Equal(1.0, aTimeEnter, Precision.Confusion());
        Assert.Equal(2.0, aTimeLeave, Precision.Confusion());
    }

    [Fact]
    public void RayBoxIntersectionEarlyExit()
    {
        // Test early exit optimization when ray misses on first axis
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray clearly misses in X direction - should exit immediately
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-2.0, 0.5, 0.5), new BVH_Vec3d(-1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit);

        // Ray with early mismatch in Y direction
        bool aHit2 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(0.5, -2.0, 0.5), new BVH_Vec3d(0.0, -1.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit2);
    }

    [Fact]
    public void RayBoxIntersectionParallelRayEarlyExit()
    {
        // Test parallel ray that misses - should exit immediately
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray parallel to X axis but Y coordinate outside box - should reject immediately
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, 2.0, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit);

        // Ray parallel to Y axis but X coordinate outside box
        bool aHit2 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(2.0, -1.0, 0.5), new BVH_Vec3d(0.0, 1.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit2);

        // Ray parallel to Z axis but X and Y outside
        bool aHit3 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(-1.0, -1.0, 0.5), new BVH_Vec3d(0.0, 0.0, 1.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit3);
    }

    [Fact]
    public void RayBoxIntersection2DParallelBothAxes()
    {
        // 2D test with ray parallel to both axes (direction = 0,0)
        BVH_Box<double, BVH_Vec2d> aBox = new(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0));
        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray origin inside box, direction = 0
        bool aHit1 = BVH_Tools<double, BVH_Vec2d>.RayBoxIntersection(new BVH_Vec2d(0.5, 0.5), new BVH_Vec2d(0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit1); // Should still hit because origin is inside

        // Ray origin outside box, direction = 0
        bool aHit2 = BVH_Tools<double, BVH_Vec2d>.RayBoxIntersection(new BVH_Vec2d(2.0, 2.0), new BVH_Vec2d(0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit2); // Should miss because origin is outside
    }

    [Fact]
    public void RayBoxIntersectionNegativeTime()
    {
        // Test that ray doesn't report intersection behind the origin
        BVH_Box<double, BVH_Vec3d> aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        double aTimeEnter = 0, aTimeLeave = 0;

        // Ray origin is past the box, pointing away
        bool aHit = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(2.0, 0.5, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.False(aHit);

        // Ray origin inside box, pointing away - should still hit (leave point is in front)
        bool aHit2 = BVH_Tools<double, BVH_Vec3d>.RayBoxIntersection(new BVH_Vec3d(0.5, 0.5, 0.5), new BVH_Vec3d(1.0, 0.0, 0.0), aBox, ref aTimeEnter, ref aTimeLeave);
        Assert.True(aHit2);
        Assert.True(aTimeLeave >= 0.0);
    }
}
