// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Meshing;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

/// <summary>Poly_CoherentTriangulation and its node, triangle and link classes (OCCT has no GTests for Poly).</summary>
public class Poly_CoherentTriangulationTest
{
    [Fact]
    public void FromTriangulation_Connectivity()
    {
        var aCoh = new CoherentTriangulation(Poly_TriangulationTest.Square());
        Assert.Equal(4, aCoh.NNodes());
        Assert.Equal(2, aCoh.NTriangles());
        Assert.Equal(0, aCoh.NLinks());
        Assert.Equal(3, aCoh.MaxNode());
        Assert.Equal(1, aCoh.MaxTriangle());
        Assert.Equal(2, aCoh.Node(1).GetIndex());
        Assert.Equal(new Xyz(1, 1, 0), aCoh.Node(2).XYZ());
        Assert.Equal(1.0, aCoh.Node(2).GetV());

        CoherentTriangle aTri0 = aCoh.Triangle(0);
        CoherentTriangle aTri1 = aCoh.Triangle(1);
        Assert.Equal((0, 1, 2), (aTri0.Node(0), aTri0.Node(1), aTri0.Node(2)));
        Assert.Equal(1, aTri0.NConnections());
        Assert.Equal(1, aTri1.NConnections());
        // tri0 = (0, 1, 2) and tri1 = (0, 2, 3) share the side 0-2, opposite node 1 of tri0 and node 3 of tri1
        Assert.Same(aTri1, aTri0.GetConnectedTri(1));
        Assert.Equal(3, aTri0.GetConnectedNode(1));
        Assert.Same(aTri0, aTri1.GetConnectedTri(2));
        Assert.Equal(1, aTri1.GetConnectedNode(2));
        Assert.Equal(1, aTri0.FindConnection(aTri1));
        Assert.Null(aTri0.GetConnectedTri(0));

        // the triangles around node 0
        var aTris = new List<CoherentTriangle>();
        for (CoherentTriPtr.Iterator anIter = aCoh.Node(0).TriangleIterator(); anIter.More(); anIter.Next())
        {
            aTris.Add(anIter.Value());
        }

        Assert.Equal(2, aTris.Count);
        Assert.Contains(aTri0, aTris);
        Assert.Contains(aTri1, aTris);
    }

    [Fact]
    public void ComputeLinks_AndFindTriangle()
    {
        var aCoh = new CoherentTriangulation(Poly_TriangulationTest.Square());
        Assert.Equal(5, aCoh.ComputeLinks());
        Assert.Equal(5, aCoh.NLinks());

        CoherentTriangle aTri0 = aCoh.Triangle(0);
        CoherentTriangle aTri1 = aCoh.Triangle(1);
        CoherentLink aShared = aTri0.GetLink(1)!;
        Assert.Same(aShared, aTri1.GetLink(2));
        Assert.Equal((0, 2), (aShared.Node(0), aShared.Node(1)));
        Assert.Equal((3, 1), (aShared.OppositeNode(0), aShared.OppositeNode(1)));

        var pTri = new CoherentTriangle?[2];
        Assert.True(aCoh.FindTriangle(new CoherentLink(0, 2), pTri));
        Assert.Same(aTri1, pTri[0]);
        Assert.Same(aTri0, pTri[1]);
        Assert.True(aCoh.FindTriangle(new CoherentLink(0, 1), pTri));
        Assert.Same(aTri0, pTri[0]);
        Assert.Null(pTri[1]);
        Assert.False(aCoh.FindTriangle(new CoherentLink(1, 3), pTri));

        int aCount = 0;
        for (var anIter = new CoherentTriangulation.IteratorOfLink(aCoh); anIter.More(); anIter.Next())
        {
            Assert.False(anIter.Value().IsEmpty);
            aCount++;
        }

        Assert.Equal(5, aCount);

        aShared.SetAttribute("data");
        Assert.Equal("data", aShared.GetAttribute());

        aCoh.ClearLinks();
        Assert.Equal(0, aCoh.NLinks());
        Assert.Null(aTri0.GetLink(1));
    }

    [Fact]
    public void RemoveTriangle_FreesNodes()
    {
        var aCoh = new CoherentTriangulation(Poly_TriangulationTest.Square());
        aCoh.ComputeLinks();
        CoherentTriangle aTri1 = aCoh.Triangle(1);
        Assert.True(aCoh.RemoveTriangle(aTri1));
        Assert.True(aTri1.IsEmpty);
        Assert.Equal(1, aCoh.NTriangles());
        Assert.Equal(3, aCoh.NNodes());
        Assert.Equal(0, aCoh.Triangle(0).NConnections());
        // the shared link stays, as a boundary link now; the two others of the triangle go
        Assert.Equal(3, aCoh.NLinks());

        var aFree = new NCollection_List<int>();
        Assert.True(aCoh.GetFreeNodes(aFree));
        Assert.Equal([3], aFree);

        var aNodes = new List<int>();
        for (var anIter = new CoherentTriangulation.IteratorOfNode(aCoh); anIter.More(); anIter.Next())
        {
            aNodes.Add(anIter.Index());
        }

        Assert.Equal([0, 1, 2], aNodes);

        Triangulation aResult = aCoh.GetTriangulation()!;
        Assert.Equal(3, aResult.NodeCount);
        Assert.Equal(1, aResult.TriangleCount);
        Assert.Equal(new Triangle1(1, 2, 3), aResult.Triangle(1));
    }

    [Fact]
    public void AddTriangle_ConnectsAndAddsLinks()
    {
        var aCoh = new CoherentTriangulation();
        Assert.Equal(0, aCoh.SetNode(new Xyz(0, 0, 0)));
        Assert.Equal(1, aCoh.SetNode(new Xyz(1, 0, 0)));
        Assert.Equal(2, aCoh.SetNode(new Xyz(0, 1, 0)));
        Assert.Equal(4, aCoh.SetNode(new Xyz(1, 1, 0), 4));
        Assert.True(aCoh.Node(3).IsFreeNode);

        CoherentTriangle aFirst = aCoh.AddTriangle(0, 1, 2)!;
        aCoh.ComputeLinks();
        Assert.Equal(3, aCoh.NLinks());
        CoherentTriangle aSecond = aCoh.AddTriangle(1, 4, 2)!;
        Assert.Same(aSecond, aFirst.GetConnectedTri(0));
        Assert.Equal(5, aCoh.NLinks());
        Assert.Null(aCoh.AddTriangle(-1, 1, 2));

        // the same nodes in the other orientation: incoherent with the first triangle, so not connected
        Assert.True(aCoh.ReplaceNodes(aSecond, 1, 2, 4));
        Assert.Null(aFirst.GetConnectedTri(0));
        Assert.Equal(2, aCoh.NTriangles());

        aCoh.SetDeflection(0.5);
        CoherentTriangulation aClone = aCoh.Clone()!;
        Assert.Equal(4, aClone.NNodes());
        Assert.Equal(0.5, aClone.Deflection());

        var aDump = new StringWriter();
        aCoh.Dump(aDump);
        Assert.StartsWith("Node 0  X =   0.0000; Y =   0.0000; Z =   0.0000\n", aDump.ToString());
        Assert.Null(new CoherentTriangulation().Clone());
        Assert.Null(new CoherentTriangulation().GetTriangulation());
    }

    [Fact]
    public void RemoveDegenerated_Reconnects()
    {
        var aTri = new Triangulation(5, 3, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(1, 1, 0));
        aTri.SetNode(4, new Point3D(1, 1 + 1e-9, 0));
        aTri.SetNode(5, new Point3D(0, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        aTri.SetTriangle(2, new Triangle1(1, 3, 4));
        aTri.SetTriangle(3, new Triangle1(1, 4, 5));
        var aCoh = new CoherentTriangulation(aTri);

        var aRemoved = new NCollection_List<CoherentTriangulation.TwoIntegers>();
        Assert.True(aCoh.RemoveDegenerated(1e-6, aRemoved));
        Assert.Equal([new CoherentTriangulation.TwoIntegers(2, 3)], aRemoved);
        Assert.Equal(2, aCoh.NTriangles());
        Assert.True(aCoh.Node(2).IsFreeNode);

        var aTris = new List<(int, int, int)>();
        for (var anIter = new CoherentTriangulation.IteratorOfTriangle(aCoh); anIter.More(); anIter.Next())
        {
            CoherentTriangle aT = anIter.Value();
            aTris.Add((aT.Node(0), aT.Node(1), aT.Node(2)));
            Assert.Equal(1, aT.NConnections());
        }

        Assert.Equal([(0, 3, 4), (0, 1, 3)], aTris);
        Assert.False(aCoh.RemoveDegenerated(1e-6));
    }

    [Fact]
    public void Node_Attributes()
    {
        var aNode = new CoherentNode(new Xyz(1, 2, 3));
        Assert.True(aNode.IsFreeNode);
        Assert.False(aNode.HasNormal);
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aNode.GetU()));
        aNode.SetNormal(new Xyz(0, 0, 1));
        Assert.True(aNode.HasNormal);
        Assert.Equal(new Xyz(0, 0, 1), aNode.GetNormal());
        aNode.SetUV(0.5, 0.25);
        Assert.Equal((0.5, 0.25), (aNode.GetU(), aNode.GetV()));
        aNode.Clear();
        Assert.Equal(new Xyz(0, 0, 0), aNode.XYZ());
        Assert.False(aNode.HasNormal);

        var aDefault = new CoherentNode();
        Assert.Equal(-1, aDefault.GetIndex());
        Assert.False(aDefault.HasNormal);
    }

    [Fact]
    public void TriangleConnections()
    {
        var aT1 = new CoherentTriangle(0, 1, 2);
        var aT2 = new CoherentTriangle(2, 1, 3);
        var aT3 = new CoherentTriangle(5, 6, 7);
        Assert.True(aT1.SetConnection(aT2));
        Assert.Equal(0, aT1.FindConnection(aT2));
        Assert.Equal(3, aT1.GetConnectedNode(0));
        Assert.Equal(0, aT2.GetConnectedNode(2));
        Assert.False(aT1.SetConnection(aT3));
        Assert.True(aT1.RemoveConnection(aT2));
        Assert.Equal(0, aT1.NConnections());
        Assert.Equal(0, aT2.NConnections());
        Assert.False(aT1.RemoveConnection(aT2));

        Assert.True(aT1.SetConnection(0, aT2));
        Assert.Same(aT1, aT2.GetConnectedTri(2));
        Assert.True(new CoherentTriangle().IsEmpty);
    }

    [Fact]
    public void TriPtr_RingList()
    {
        var aT1 = new CoherentTriangle(0, 1, 2);
        var aT2 = new CoherentTriangle(0, 2, 3);
        var aT3 = new CoherentTriangle(0, 3, 4);
        var aList = new CoherentTriPtr(aT1);
        aList.Append(aT2);
        aList.Prepend(aT3);
        var anOrder = new List<CoherentTriangle>();
        var anIter = new CoherentTriPtr.Iterator(aList.Next());
        Assert.Same(aT2, anIter.First());
        for (; anIter.More(); anIter.Next())
        {
            anOrder.Add(anIter.Value());
        }

        Assert.Equal([aT2, aT3, aT1], anOrder);
        CoherentTriPtr.Remove(aList.Next());
        Assert.Same(aT3, aList.Next().GetTriangle());
        Assert.Same(aT3, aList.Previous().GetTriangle());
        Assert.False(new CoherentTriPtr.Iterator().More());
    }
}
