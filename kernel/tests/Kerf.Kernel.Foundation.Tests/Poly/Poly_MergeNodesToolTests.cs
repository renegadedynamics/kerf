// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Meshing;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>Poly_MergeNodesTool (OCCT has no GTests for the Poly package).</summary>
public class Poly_MergeNodesToolTest
{
    /// <summary>
    /// Two triangles with their own nodes, sharing the edge (0,0,0)-(1,0,0); the second is folded by
    /// <paramref name="theFoldAngle"/> about that edge (0: coplanar).
    /// </summary>
    private static Triangulation Soup(double theFoldAngle)
    {
        var aTri = new Triangulation(6, 2, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(0.5, 1, 0));
        aTri.SetNode(4, new Point3D(1, 0, 0));
        aTri.SetNode(5, new Point3D(0, 0, 0));
        aTri.SetNode(6, new Point3D(0.5, -Math.Cos(theFoldAngle), Math.Sin(theFoldAngle)));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        aTri.SetTriangle(2, new Triangle1(4, 5, 6));
        return aTri;
    }

    [Fact]
    public void MergeNodes_Coplanar()
    {
        Triangulation aResult = MergeNodesTool.MergeNodes(Soup(0), new Transform3D(), false, 0.5)!;
        Assert.Equal(4, aResult.NodeCount);
        Assert.Equal(2, aResult.TriangleCount);
        Assert.Equal(new Triangle1(1, 2, 3), aResult.Triangle(1));
        Assert.Equal(new Triangle1(2, 1, 4), aResult.Triangle(2));
        Assert.Equal(new Point3D(0.5, -1, 0), aResult.Node(4));
    }

    [Fact]
    public void MergeNodes_SharpEdgeKept()
    {
        // folded by 90 degrees: the normals differ by 90 degrees
        Triangulation aSoup = Soup(Math.PI / 2);
        Triangulation aSharp = MergeNodesTool.MergeNodes(aSoup, new Transform3D(), false, 30 * Math.PI / 180)!;
        Assert.Equal(6, aSharp.NodeCount);

        // a smooth angle near 90 degrees merges any angle
        Triangulation aSmooth = MergeNodesTool.MergeNodes(aSoup, new Transform3D(), false, 100 * Math.PI / 180)!;
        Assert.Equal(4, aSmooth.NodeCount);

        // nothing merged: null unless forced
        Assert.Null(MergeNodesTool.MergeNodes(aSoup, new Transform3D(), false, 30 * Math.PI / 180, 0.0, false));
    }

    [Fact]
    public void MergeNodes_Opposite()
    {
        // the second triangle reversed, coplanar: opposite normals
        Triangulation aSoup = Soup(0);
        aSoup.SetTriangle(2, new Triangle1(5, 4, 6));
        var aTool = new MergeNodesTool(0.1);
        aTool.AddTriangulation(aSoup);
        Assert.Equal(6, aTool.NodeCount);

        var anOpposite = new MergeNodesTool(0.1);
        anOpposite.SetMergeOpposite(true);
        Assert.True(anOpposite.ToMergeOpposite());
        anOpposite.AddTriangulation(aSoup);
        Assert.Equal(4, anOpposite.NodeCount);
    }

    [Fact]
    public void MergeNodes_Tolerance()
    {
        Triangulation aSoup = Soup(0);
        aSoup.SetNode(4, new Point3D(1.0005, 0.0003, 0));
        aSoup.SetNode(5, new Point3D(-0.0004, 0, 0.0002));

        // exact merging finds nothing to merge
        Assert.Equal(6, MergeNodesTool.MergeNodes(aSoup, new Transform3D(), false, 0.5)!.NodeCount);

        Triangulation aMerged = MergeNodesTool.MergeNodes(aSoup, new Transform3D(), false, 0.5, 0.001)!;
        Assert.Equal(4, aMerged.NodeCount);
        // the first node at a place is kept
        Assert.Equal(new Point3D(1, 0, 0), aMerged.Node(2));

        Assert.Equal(6, MergeNodesTool.MergeNodes(aSoup, new Transform3D(), false, 0.5, 0.0001)!.NodeCount);
    }

    [Fact]
    public void MergeNodes_TransformReverseAndUnits()
    {
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(0, 0, 10));
        Triangulation aResult = MergeNodesTool.MergeNodes(Soup(0), aTrsf, true, 0.5)!;
        Assert.Equal(new Triangle1(1, 2, 3), aResult.Triangle(1));
        Assert.Equal(new Point3D(0, 0, 10), aResult.Node(1));
        Assert.Equal(new Point3D(0.5, 1, 10), aResult.Node(2));

        var aTool = new MergeNodesTool(0.5);
        aTool.SetUnitFactor(1000);
        aTool.AddTriangulation(Soup(0));
        Assert.Equal(new Point3D(500, 1000, 0), aTool.Result()!.Node(3));
    }

    [Fact]
    public void NoMerging_WithoutAngleOrTolerance()
    {
        var aTool = new MergeNodesTool(0.0);
        Assert.Equal(0.0, aTool.MergeAngle());
        Assert.Equal(0.0, aTool.MergeTolerance());
        aTool.AddTriangulation(Soup(0));
        Assert.Equal(6, aTool.NodeCount);
        Assert.Equal(2, aTool.ElementCount);
        Assert.Equal(6, aTool.Result()!.NodeCount);
    }

    [Fact]
    public void DegenerateAndEqualElements()
    {
        var aTool = new MergeNodesTool(0.5, 0.01);
        Assert.True(aTool.ToDropDegenerative());
        Assert.False(aTool.ToMergeElems());
        aTool.SetMergeElems(true);

        Xyz[] aTri = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0)];
        aTool.AddTriangle(aTri);
        // the same triangle with its nodes rotated
        aTool.AddTriangle([aTri[1], aTri[2], aTri[0]]);
        // a triangle whose first two nodes merge
        Xyz[] aSliver = [new(0, 0, 0), new(0.001, 0, 0), new(0, 1, 0)];
        aTool.AddTriangle(aSliver);

        Assert.Equal(1, aTool.ElementCount);
        Assert.Equal(1, aTool.MergedElemCount);
        Assert.Equal(1, aTool.DegenerativeElemCount);
        Assert.Equal(3, aTool.NodeCount);

        var aKeep = new MergeNodesTool(0.5, 0.01);
        aKeep.SetDropDegenerative(false);
        aKeep.AddTriangle(aSliver);
        Assert.Equal(1, aKeep.ElementCount);
        Assert.Equal(2, aKeep.NodeCount);

        // a triangle of zero area has a zero normal, so its nodes merge with nothing, not even each other
        var aFlat = new MergeNodesTool(0.5);
        aFlat.AddTriangle([aTri[0], aTri[0], aTri[1]]);
        Assert.Equal(3, aFlat.NodeCount);
        Assert.Equal(1, aFlat.ElementCount);
    }

    [Fact]
    public void Quads_AndPushedElements()
    {
        var aTool = new MergeNodesTool(0.5);
        aTool.AddQuad([new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)]);
        Assert.Equal(2, aTool.ElementCount);
        Assert.Equal(4, aTool.NodeCount);

        aTool.ChangeElementNode(0) = new Xyz(1, 0, 0);
        aTool.ChangeElementNode(1) = new Xyz(2, 0, 0);
        aTool.ChangeElementNode(2) = new Xyz(1, 1, 0);
        aTool.PushLastTriangle();
        Assert.Equal(1, aTool.ElementNodeIndex(0));
        Assert.Equal(4, aTool.ElementNodeIndex(1));
        Assert.Equal(2, aTool.ElementNodeIndex(2));

        Triangulation aResult = aTool.Result()!;
        Assert.Equal(5, aResult.NodeCount);
        Assert.Equal(3, aResult.TriangleCount);
        Assert.Equal(new Triangle1(1, 3, 4), aResult.Triangle(2));
        Assert.Equal(new Triangle1(2, 5, 3), aResult.Triangle(3));

        EngineAssert.ThrowsAny<ProgramErrorException>(() => aTool.PushLastElement(5));
    }

    [Fact]
    public void AsMergeMapOnly()
    {
        var aTool = new MergeNodesTool(0.5);
        aTool.ChangeOutput() = null;
        aTool.AddTriangulation(Soup(0));
        Assert.Equal(4, aTool.NodeCount);
        Assert.Null(aTool.Result());
    }

    [Fact]
    public void ManyNodes_ResizeKeepsMerging()
    {
        // a grid of quads, each with its own nodes, merged: (n + 1)^2 nodes; exercises the map's resizing
        const int n = 40;
        var aTool = new MergeNodesTool(0.5, 0.0, 1);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                aTool.AddQuad([new(i, j, 0), new(i + 1, j, 0), new(i + 1, j + 1, 0), new(i, j + 1, 0)]);
            }
        }

        Assert.Equal((n + 1) * (n + 1), aTool.NodeCount);
        Assert.Equal(2 * n * n, aTool.ElementCount);

        var aTolerant = new MergeNodesTool(0.5, 0.01, 1);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                double e = 0.001 * ((i * 7 + j * 3) % 5);
                aTolerant.AddQuad([new(i + e, j, 0), new(i + 1, j + e, 0), new(i + 1, j + 1, e), new(i, j + 1, 0)]);
            }
        }

        Assert.Equal((n + 1) * (n + 1), aTolerant.NodeCount);
    }
}
