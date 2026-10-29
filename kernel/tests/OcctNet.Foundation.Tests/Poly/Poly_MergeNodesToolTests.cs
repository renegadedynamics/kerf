// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

/// <summary>Poly_MergeNodesTool (OCCT has no GTests for the Poly package).</summary>
public class Poly_MergeNodesToolTest
{
    /// <summary>
    /// Two triangles with their own nodes, sharing the edge (0,0,0)-(1,0,0); the second is folded by
    /// <paramref name="theFoldAngle"/> about that edge (0: coplanar).
    /// </summary>
    private static Poly_Triangulation Soup(double theFoldAngle)
    {
        var aTri = new Poly_Triangulation(6, 2, false);
        aTri.SetNode(1, new gp_Pnt(0, 0, 0));
        aTri.SetNode(2, new gp_Pnt(1, 0, 0));
        aTri.SetNode(3, new gp_Pnt(0.5, 1, 0));
        aTri.SetNode(4, new gp_Pnt(1, 0, 0));
        aTri.SetNode(5, new gp_Pnt(0, 0, 0));
        aTri.SetNode(6, new gp_Pnt(0.5, -Math.Cos(theFoldAngle), Math.Sin(theFoldAngle)));
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aTri.SetTriangle(2, new Poly_Triangle(4, 5, 6));
        return aTri;
    }

    [Fact]
    public void MergeNodes_Coplanar()
    {
        Poly_Triangulation aResult = Poly_MergeNodesTool.MergeNodes(Soup(0), new gp_Trsf(), false, 0.5)!;
        Assert.Equal(4, aResult.NbNodes());
        Assert.Equal(2, aResult.NbTriangles());
        Assert.Equal(new Poly_Triangle(1, 2, 3), aResult.Triangle(1));
        Assert.Equal(new Poly_Triangle(2, 1, 4), aResult.Triangle(2));
        Assert.Equal(new gp_Pnt(0.5, -1, 0), aResult.Node(4));
    }

    [Fact]
    public void MergeNodes_SharpEdgeKept()
    {
        // folded by 90 degrees: the normals differ by 90 degrees
        Poly_Triangulation aSoup = Soup(Math.PI / 2);
        Poly_Triangulation aSharp = Poly_MergeNodesTool.MergeNodes(aSoup, new gp_Trsf(), false, 30 * Math.PI / 180)!;
        Assert.Equal(6, aSharp.NbNodes());

        // a smooth angle near 90 degrees merges any angle
        Poly_Triangulation aSmooth = Poly_MergeNodesTool.MergeNodes(aSoup, new gp_Trsf(), false, 100 * Math.PI / 180)!;
        Assert.Equal(4, aSmooth.NbNodes());

        // nothing merged: null unless forced
        Assert.Null(Poly_MergeNodesTool.MergeNodes(aSoup, new gp_Trsf(), false, 30 * Math.PI / 180, 0.0, false));
    }

    [Fact]
    public void MergeNodes_Opposite()
    {
        // the second triangle reversed, coplanar: opposite normals
        Poly_Triangulation aSoup = Soup(0);
        aSoup.SetTriangle(2, new Poly_Triangle(5, 4, 6));
        var aTool = new Poly_MergeNodesTool(0.1);
        aTool.AddTriangulation(aSoup);
        Assert.Equal(6, aTool.NbNodes());

        var anOpposite = new Poly_MergeNodesTool(0.1);
        anOpposite.SetMergeOpposite(true);
        Assert.True(anOpposite.ToMergeOpposite());
        anOpposite.AddTriangulation(aSoup);
        Assert.Equal(4, anOpposite.NbNodes());
    }

    [Fact]
    public void MergeNodes_Tolerance()
    {
        Poly_Triangulation aSoup = Soup(0);
        aSoup.SetNode(4, new gp_Pnt(1.0005, 0.0003, 0));
        aSoup.SetNode(5, new gp_Pnt(-0.0004, 0, 0.0002));

        // exact merging finds nothing to merge
        Assert.Equal(6, Poly_MergeNodesTool.MergeNodes(aSoup, new gp_Trsf(), false, 0.5)!.NbNodes());

        Poly_Triangulation aMerged = Poly_MergeNodesTool.MergeNodes(aSoup, new gp_Trsf(), false, 0.5, 0.001)!;
        Assert.Equal(4, aMerged.NbNodes());
        // the first node at a place is kept
        Assert.Equal(new gp_Pnt(1, 0, 0), aMerged.Node(2));

        Assert.Equal(6, Poly_MergeNodesTool.MergeNodes(aSoup, new gp_Trsf(), false, 0.5, 0.0001)!.NbNodes());
    }

    [Fact]
    public void MergeNodes_TransformReverseAndUnits()
    {
        var aTrsf = new gp_Trsf();
        aTrsf.SetTranslation(new gp_Vec(0, 0, 10));
        Poly_Triangulation aResult = Poly_MergeNodesTool.MergeNodes(Soup(0), aTrsf, true, 0.5)!;
        Assert.Equal(new Poly_Triangle(1, 2, 3), aResult.Triangle(1));
        Assert.Equal(new gp_Pnt(0, 0, 10), aResult.Node(1));
        Assert.Equal(new gp_Pnt(0.5, 1, 10), aResult.Node(2));

        var aTool = new Poly_MergeNodesTool(0.5);
        aTool.SetUnitFactor(1000);
        aTool.AddTriangulation(Soup(0));
        Assert.Equal(new gp_Pnt(500, 1000, 0), aTool.Result()!.Node(3));
    }

    [Fact]
    public void NoMerging_WithoutAngleOrTolerance()
    {
        var aTool = new Poly_MergeNodesTool(0.0);
        Assert.Equal(0.0, aTool.MergeAngle());
        Assert.Equal(0.0, aTool.MergeTolerance());
        aTool.AddTriangulation(Soup(0));
        Assert.Equal(6, aTool.NbNodes());
        Assert.Equal(2, aTool.NbElements());
        Assert.Equal(6, aTool.Result()!.NbNodes());
    }

    [Fact]
    public void DegenerateAndEqualElements()
    {
        var aTool = new Poly_MergeNodesTool(0.5, 0.01);
        Assert.True(aTool.ToDropDegenerative());
        Assert.False(aTool.ToMergeElems());
        aTool.SetMergeElems(true);

        gp_XYZ[] aTri = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0)];
        aTool.AddTriangle(aTri);
        // the same triangle with its nodes rotated
        aTool.AddTriangle([aTri[1], aTri[2], aTri[0]]);
        // a triangle whose first two nodes merge
        gp_XYZ[] aSliver = [new(0, 0, 0), new(0.001, 0, 0), new(0, 1, 0)];
        aTool.AddTriangle(aSliver);

        Assert.Equal(1, aTool.NbElements());
        Assert.Equal(1, aTool.NbMergedElems());
        Assert.Equal(1, aTool.NbDegenerativeElems());
        Assert.Equal(3, aTool.NbNodes());

        var aKeep = new Poly_MergeNodesTool(0.5, 0.01);
        aKeep.SetDropDegenerative(false);
        aKeep.AddTriangle(aSliver);
        Assert.Equal(1, aKeep.NbElements());
        Assert.Equal(2, aKeep.NbNodes());

        // a triangle of zero area has a zero normal, so its nodes merge with nothing, not even each other
        var aFlat = new Poly_MergeNodesTool(0.5);
        aFlat.AddTriangle([aTri[0], aTri[0], aTri[1]]);
        Assert.Equal(3, aFlat.NbNodes());
        Assert.Equal(1, aFlat.NbElements());
    }

    [Fact]
    public void Quads_AndPushedElements()
    {
        var aTool = new Poly_MergeNodesTool(0.5);
        aTool.AddQuad([new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)]);
        Assert.Equal(2, aTool.NbElements());
        Assert.Equal(4, aTool.NbNodes());

        aTool.ChangeElementNode(0) = new gp_XYZ(1, 0, 0);
        aTool.ChangeElementNode(1) = new gp_XYZ(2, 0, 0);
        aTool.ChangeElementNode(2) = new gp_XYZ(1, 1, 0);
        aTool.PushLastTriangle();
        Assert.Equal(1, aTool.ElementNodeIndex(0));
        Assert.Equal(4, aTool.ElementNodeIndex(1));
        Assert.Equal(2, aTool.ElementNodeIndex(2));

        Poly_Triangulation aResult = aTool.Result()!;
        Assert.Equal(5, aResult.NbNodes());
        Assert.Equal(3, aResult.NbTriangles());
        Assert.Equal(new Poly_Triangle(1, 3, 4), aResult.Triangle(2));
        Assert.Equal(new Poly_Triangle(2, 5, 3), aResult.Triangle(3));

        Assert.ThrowsAny<Standard_ProgramError>(() => aTool.PushLastElement(5));
    }

    [Fact]
    public void AsMergeMapOnly()
    {
        var aTool = new Poly_MergeNodesTool(0.5);
        aTool.ChangeOutput() = null;
        aTool.AddTriangulation(Soup(0));
        Assert.Equal(4, aTool.NbNodes());
        Assert.Null(aTool.Result());
    }

    [Fact]
    public void ManyNodes_ResizeKeepsMerging()
    {
        // a grid of quads, each with its own nodes, merged: (n + 1)^2 nodes; exercises the map's resizing
        const int n = 40;
        var aTool = new Poly_MergeNodesTool(0.5, 0.0, 1);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                aTool.AddQuad([new(i, j, 0), new(i + 1, j, 0), new(i + 1, j + 1, 0), new(i, j + 1, 0)]);
            }
        }

        Assert.Equal((n + 1) * (n + 1), aTool.NbNodes());
        Assert.Equal(2 * n * n, aTool.NbElements());

        var aTolerant = new Poly_MergeNodesTool(0.5, 0.01, 1);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                double e = 0.001 * ((i * 7 + j * 3) % 5);
                aTolerant.AddQuad([new(i + e, j, 0), new(i + 1, j + e, 0), new(i + 1, j + 1, e), new(i, j + 1, 0)]);
            }
        }

        Assert.Equal((n + 1) * (n + 1), aTolerant.NbNodes());
    }
}
