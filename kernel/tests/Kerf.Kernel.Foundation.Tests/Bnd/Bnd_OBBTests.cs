// Port of: src/FoundationClasses/TKMath/GTests/Bnd_OBB_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bvh;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Bnd_OBB_Test
{
    [Fact]
    public void OCC33009_ReBuildWithPoints()
    {
        Obb aBndBox = new();

        NCollection_Array1<Point3D> aPoints = new(1, 5);
        aPoints[1] = new Point3D(1, 2, 3);
        aPoints[2] = new Point3D(3, 2, 1);
        aPoints[3] = new Point3D(2, 3, 1);
        aPoints[4] = new Point3D(1, 3, 2);
        aPoints[5] = new Point3D(2, 1, 3);

        // Should not throw exception when rebuilding with points
        aBndBox.ReBuild(aPoints, null, true);
        Assert.False(aBndBox.IsVoid);
    }

    [Fact]
    public void OCC30704_AddBoundingBoxToVoidBox()
    {
        // Bug OCC30704: adding a box to a void box must set the centre of the void box.
        // OCCT builds the OBB of a box shape (100, 100, 100)-(200, 200, 200) with BRepBndLib::AddOBB: that form is in
        // Kerf.Kernel.ModelingAlgorithms.Tests (Bnd/Bnd_ShapeGTests.cs); here the OBB is built from the box's eight vertices,
        // which is what AddOBB uses for a box.
        NCollection_Array1<Point3D> aVertices = new(1, 8);
        for (int i = 0; i < 8; ++i)
        {
            aVertices[i + 1] = new Point3D((i & 1) != 0 ? 200 : 100, (i & 2) != 0 ? 200 : 100, (i & 4) != 0 ? 200 : 100);
        }

        Obb aVoidBox = new(), aOBB = new();
        aOBB.ReBuild(aVertices);
        aVoidBox.Add(aOBB);

        Xyz aCenter = aVoidBox.Center();
        GTest.ExpectDoubleEq(150.0, aCenter.X());
        GTest.ExpectDoubleEq(150.0, aCenter.Y());
        GTest.ExpectDoubleEq(150.0, aCenter.Z());
    }

    [Fact]
    public void OCC30704_AddPointToVoidBox()
    {
        Point3D aP = new(100, 200, 300);

        Obb aVoidBox = new();
        aVoidBox.Add(aP);

        Xyz aCenter = aVoidBox.Center();
        GTest.ExpectDoubleEq(100.0, aCenter.X());
        GTest.ExpectDoubleEq(200.0, aCenter.Y());
        GTest.ExpectDoubleEq(300.0, aCenter.Z());
    }

    [Fact]
    public void Contains_Point()
    {
        Obb anOBB = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 5.0, 5.0, 5.0);
        Assert.True(anOBB.Contains(new Point3D(0, 0, 0)));
        Assert.True(anOBB.Contains(new Point3D(4, 4, 4)));
        Assert.False(anOBB.Contains(new Point3D(10, 0, 0)));
    }

    [Fact]
    public void Intersects_OBB()
    {
        Obb anOBB1 = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 5.0, 5.0, 5.0);
        Obb anOBB2 = new(new Point3D(8, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 5.0, 5.0, 5.0);
        Obb anOBB3 = new(new Point3D(20, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 5.0, 5.0, 5.0);
        Assert.True(anOBB1.Intersects(anOBB2));
        Assert.False(anOBB1.Intersects(anOBB3));
    }

    [Fact]
    public void GetHalfSizes_StructuredBindings()
    {
        Obb anOBB = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 3.0, 5.0, 7.0);
        var (aHX, aHY, aHZ) = anOBB.GetHalfSizes();
        GTest.ExpectDoubleEq(3.0, aHX);
        GTest.ExpectDoubleEq(5.0, aHY);
        GTest.ExpectDoubleEq(7.0, aHZ);
    }
}

/// <summary>Tests of the port's own for Bnd_OBB and Bnd_Tools.</summary>
public class Bnd_OBB_PortTest
{
    [Fact]
    public void DefaultIsVoid_AndValueSemantics()
    {
        Obb aBox = new();
        Assert.True(aBox.IsVoid);
        Assert.True(Obb.Void().IsVoid);
        Obb anEnlarged = new Obb(new Point3D(1, 2, 3), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 1.0, 2.0, 3.0).Enlarged(-0.5);
        Assert.Equal(1.5, anEnlarged.XHSize());
        Assert.Equal(4.0 * (1.5 * 1.5 + 2.5 * 2.5 + 3.5 * 3.5), anEnlarged.SquareExtent());
        Assert.Equal(anEnlarged, anEnlarged.WithAABox(false));
        Assert.NotEqual(anEnlarged, anEnlarged.WithAABox(true));
    }

    [Fact]
    public void FromBndBox_AndCompletelyInside()
    {
        Box aBox = new(new Point3D(-1, -2, -3), new Point3D(1, 2, 3));
        Obb anOBB = new(aBox);
        Assert.True(anOBB.IsAABox);
        Assert.Equal(new Xyz(0, 0, 0), anOBB.Center());
        Assert.Equal(3.0, anOBB.ZHSize());
        Assert.True(anOBB.Enlarged(1.0).IsCompletelyInside(anOBB));
        Assert.False(anOBB.IsCompletelyInside(anOBB.Enlarged(1.0)));
        Assert.True(new Obb(new Box()).IsVoid);
    }

    [Fact]
    public void ReBuild_PointSets()
    {
        var aRandom = new Random(5);
        foreach (bool isOptimal in new[] { false, true })
        {
            for (int aCase = 0; aCase < 20; ++aCase)
            {
                int aNb = 1 + aCase;
                NCollection_Array1<Point3D> aPoints = new(1, aNb);
                NCollection_Array1<double> aTols = new(1, aNb);
                for (int i = 1; i <= aNb; ++i)
                {
                    aPoints[i] = new Point3D(aRandom.NextDouble() * 10, aRandom.NextDouble() * 5, aRandom.NextDouble());
                    aTols[i] = aRandom.NextDouble() * 0.01;
                }

                Obb anOBB = Obb.FromPoints(aPoints, aTols, isOptimal);
                Assert.False(anOBB.IsVoid);
                for (int i = 1; i <= aNb; ++i)
                {
                    Assert.False(anOBB.Enlarged(1.0e-9).IsOut(aPoints[i]));
                }
            }
        }
    }

    [Fact]
    public void ReBuild_TwoCoincidentPoints_DropTolerance()
    {
        // OCCT: the two-point case of coincident points loses the tolerances (enlarged by Precision::Confusion only)
        NCollection_Array1<Point3D> aPoints = new(1, 2);
        aPoints[1] = new Point3D(1, 1, 1);
        aPoints[2] = new Point3D(1, 1, 1);
        NCollection_Array1<double> aTols = new(1, 2);
        aTols[1] = 0.5;
        aTols[2] = 0.5;
        Obb anOBB = Obb.FromPoints(aPoints, aTols);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), anOBB.XHSize());
    }

    [Fact]
    public void Bnd_Tools_Bnd2BVH()
    {
        Box aBox = new(new Point3D(-1, -2, -3), new Point3D(1, 2, 3));
        Box<double, NCollection_Vec3<double>> aBVH = global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(aBox);
        Assert.Equal(new NCollection_Vec3<double>(-1, -2, -3), aBVH.CornerMin());
        Assert.Equal(new NCollection_Vec3<double>(1, 2, 3), aBVH.CornerMax());

        Box2d aBox2d = Box2d.FromPoint(new Point2D(1, 2)).Added(new Point2D(3, 4));
        Box<double, NCollection_Vec2<double>> aBVH2d = global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(aBox2d);
        Assert.Equal(new NCollection_Vec2<double>(1, 2), aBVH2d.CornerMin());
        Assert.Equal(new NCollection_Vec2<double>(3, 4), aBVH2d.CornerMax());
    }
}
