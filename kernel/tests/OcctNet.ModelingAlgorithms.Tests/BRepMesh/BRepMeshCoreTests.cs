// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the Delaunay core's pieces (unit 4B): the plain types, the vertex and circle tools, the data
/// structure (with the OCCT semantics decision 4404 keeps), the mesh tool, the classifier and the triangulator.
/// </summary>
public class BRepMeshCoreTests
{
    // ---- Plain types

    [Fact]
    public void PairOfIndex_holds_at_most_two_indices()
    {
        BRepMesh_PairOfIndex aPair = new();
        Assert.True(aPair.IsEmpty());
        Assert.Equal(0, aPair.Extent());
        aPair = aPair.Appended(5).Appended(7);
        Assert.Equal(2, aPair.Extent());
        Assert.Equal(5, aPair.FirstIndex());
        Assert.Equal(7, aPair.LastIndex());
        Assert.Throws<Standard_OutOfRange>(() => aPair.Appended(9));
        Assert.Throws<Standard_OutOfRange>(() => aPair.Index(3));
        BRepMesh_PairOfIndex aRemoved = aPair.IndexRemoved(1);
        Assert.Equal(1, aRemoved.Extent());
        Assert.Equal(7, aRemoved.FirstIndex());
        Assert.Equal(7, aRemoved.LastIndex());
        Assert.True(default(BRepMesh_PairOfIndex) != new BRepMesh_PairOfIndex());
    }

    [Fact]
    public void Edge_equality_is_OCCTs_in_IsEqual_and_exact_in_Equals()
    {
        BRepMesh_Edge anEdge = new(1, 2, BRepMesh_Free);
        BRepMesh_Edge aReversed = new(2, 1, BRepMesh_Frontier);
        Assert.True(anEdge.IsEqual(aReversed));
        Assert.False(anEdge.IsSameOrientation(aReversed));
        Assert.False(anEdge.Equals(aReversed));
        Assert.True(BRepMesh_Edge.Hasher.Instance.Equals(anEdge, aReversed));
        Assert.Equal(BRepMesh_Edge.Hasher.Instance.GetHashCode(anEdge), BRepMesh_Edge.Hasher.Instance.GetHashCode(aReversed));

        // A deleted link equals nothing, not even itself (OCCT's operator==).
        BRepMesh_Edge aDeleted = anEdge.WithMovability(BRepMesh_Deleted);
        Assert.False(aDeleted.IsEqual(aDeleted));
        Assert.True(aDeleted.Equals(aDeleted));
        Assert.Equal(BRepMesh_Deleted, new BRepMesh_Edge().Movability());

        BRepMesh_OrientedEdge anOriented = anEdge;
        Assert.Equal(new BRepMesh_OrientedEdge(1, 2), anOriented);
        Assert.Equal(1 | (2 << 16), anOriented.GetHashCode());
    }

    [Fact]
    public void Triangle_and_vertex_equality()
    {
        BRepMesh_Triangle aTriangle = new([1, 2, 3], [true, false, true], BRepMesh_Free);
        BRepMesh_Triangle aRotated = new([2, 3, 1], [false, true, true], BRepMesh_Free);
        Assert.True(aTriangle.IsEqual(aRotated));
        Assert.False(aTriangle.Equals(aRotated));
        Assert.False(aTriangle.IsEqual(new BRepMesh_Triangle([1, 3, 2], [true, true, true], BRepMesh_Free)));
        Assert.False(aTriangle.WithMovability(BRepMesh_Deleted).IsEqual(aRotated));
        Span<int> e = stackalloc int[3];
        Span<bool> o = stackalloc bool[3];
        aTriangle.Edges(e, o);
        Assert.Equal([1, 2, 3], e.ToArray());
        Assert.Equal([true, false, true], o.ToArray());

        BRepMesh_Vertex aVertex = new(new gp_XY(1.0, 2.0), 7, BRepMesh_Frontier);
        BRepMesh_Vertex aNear = new(1.0 + 1e-10, 2.0, BRepMesh_Free);
        Assert.True(aVertex.IsEqual(aNear));
        Assert.False(aVertex.Equals(aNear));
        Assert.False(aVertex.IsEqual(aNear.WithMovability(BRepMesh_Deleted)));
        Assert.Equal(new BRepMesh_Vertex(), default);
    }

    // ---- Tools

    [Fact]
    public void VertexTool_merges_coincident_vertices_and_reuses_deleted_indices()
    {
        BRepMesh_VertexTool aTool = new();
        int a = aTool.Add(new BRepMesh_Vertex(0.0, 0.0, BRepMesh_Free), false);
        int b = aTool.Add(new BRepMesh_Vertex(1.0, 0.0, BRepMesh_Free), false);
        int aSame = aTool.Add(new BRepMesh_Vertex(1.0 + 1e-8, 0.0, BRepMesh_Free), false);
        int aForced = aTool.Add(new BRepMesh_Vertex(1.0 + 1e-8, 0.0, BRepMesh_Free), true);
        Assert.Equal((1, 2, 2, 3), (a, b, aSame, aForced));
        Assert.Equal(2, aTool.FindIndex(new BRepMesh_Vertex(1.0, 0.0, BRepMesh_Free)));

        aTool.DeleteVertex(1);
        Assert.Equal(0, aTool.FindIndex(new BRepMesh_Vertex(0.0, 0.0, BRepMesh_Free)));
        Assert.Equal(1, aTool.Add(new BRepMesh_Vertex(5.0, 5.0, BRepMesh_Free), false));
        Assert.Equal(1, aTool.FindIndex(new BRepMesh_Vertex(5.0, 5.0, BRepMesh_Free)));
    }

    [Fact]
    public void CircleTool_selects_the_circles_containing_a_point()
    {
        BRepMesh_CircleTool aTool = new();
        aTool.SetMinMaxSize(new gp_XY(-10, -10), new gp_XY(10, 10));
        aTool.SetCellSize(1.0, 1.0);
        Assert.True(aTool.Bind(1, new gp_XY(0, 0), new gp_XY(2, 0), new gp_XY(0, 2)));
        Assert.True(aTool.Bind(2, new gp_XY(5, 5), new gp_XY(6, 5), new gp_XY(5, 6)));
        Assert.False(aTool.Bind(3, new gp_XY(0, 0), new gp_XY(1, 1), new gp_XY(2, 2)));
        aTool.MocBind(3);

        Assert.Equal([1], aTool.Select(new gp_XY(1.0, 1.0)));
        Assert.Equal([2], aTool.Select(new gp_XY(5.4, 5.4)));
        Assert.Empty(aTool.Select(new gp_XY(-5.0, 5.0)));

        aTool.Delete(1);
        Assert.Empty(aTool.Select(new gp_XY(1.0, 1.0)));
    }

    // ---- Data structure

    private static BRepMesh_DataStructureOfDelaun Quad(out int[] theNodes)
    {
        // Two triangles (1, 2, 3) and (1, 3, 4) of the unit square.
        BRepMesh_DataStructureOfDelaun aStructure = new();
        theNodes =
        [
            aStructure.AddNode(new BRepMesh_Vertex(0, 0, BRepMesh_Free)),
            aStructure.AddNode(new BRepMesh_Vertex(1, 0, BRepMesh_Free)),
            aStructure.AddNode(new BRepMesh_Vertex(1, 1, BRepMesh_Free)),
            aStructure.AddNode(new BRepMesh_Vertex(0, 1, BRepMesh_Free)),
        ];
        BRepMesh_MeshTool aTool = new(aStructure);
        Span<int> e = stackalloc int[3];
        aTool.AddTriangle(1, 2, 3, e);
        aTool.AddTriangle(1, 3, 4, e);
        return aStructure;
    }

    [Fact]
    public void Links_are_found_in_either_orientation_and_deleted_indices_are_reused()
    {
        BRepMesh_DataStructureOfDelaun aStructure = Quad(out _);
        Assert.Equal(5, aStructure.NbLinks());
        int aDiagonal = aStructure.IndexOf(new BRepMesh_Edge(3, 1, BRepMesh_Free));
        // The triangles stored the diagonal as 3 -> 1: the other orientation gives a negative index.
        Assert.Equal(aDiagonal, aStructure.AddLink(new BRepMesh_Edge(3, 1, BRepMesh_Free)));
        Assert.Equal(-aDiagonal, aStructure.AddLink(new BRepMesh_Edge(1, 3, BRepMesh_Free)));
        Assert.Equal(2, aStructure.ElementsConnectedTo(aDiagonal).Extent());

        // ElementsConnectedTo is a reference that sees the removal of a triangle.
        ref readonly BRepMesh_PairOfIndex aPair = ref aStructure.ElementsConnectedTo(aDiagonal);
        aStructure.RemoveElement(1);
        Assert.Equal(1, aPair.Extent());
        Assert.Equal(2, aPair.FirstIndex());
        aStructure.RemoveElement(2);
        Assert.True(aPair.IsEmpty());

        aStructure.RemoveLink(aDiagonal);
        Assert.Equal(BRepMesh_Deleted, aStructure.GetLink(aDiagonal).Movability());
        Assert.Equal(0, aStructure.IndexOf(new BRepMesh_Edge(1, 3, BRepMesh_Free)));
        Assert.DoesNotContain(aDiagonal, aStructure.LinksConnectedTo(1));
        Assert.Equal(aDiagonal, aStructure.AddLink(new BRepMesh_Edge(2, 4, BRepMesh_Free)));
        Assert.Contains(aDiagonal, aStructure.LinksConnectedTo(4));
    }

    [Fact]
    public void A_removed_link_list_node_keeps_its_successor()
    {
        // OCCT's NCollection_List over an incremental allocator: a loop standing on a node that another loop removes
        // goes on to that node's old successor (BRepMesh_Delaun's killTrianglesAroundVertex relies on it).
        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger aList = new();
        foreach (int i in new[] { 1, 2, 3, 4 })
        {
            aList.Append(i);
        }

        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator anOuter = new(aList);
        anOuter.Next(); // on 2
        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator anInner = new(aList);
        anInner.Next();
        aList.Remove(anInner); // removes 2
        Assert.Equal(3, anInner.Value());
        Assert.Equal(2, anOuter.Value());
        anOuter.Next();
        Assert.Equal(3, anOuter.Value());
        Assert.Equal([1, 3, 4], aList);
        Assert.Equal(3, aList.Extent());

        // Removing the last node: Last moves back.
        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator aLast = new(aList);
        aLast.Next();
        aLast.Next();
        aList.Remove(aLast);
        Assert.False(aLast.More());
        Assert.Equal(3, aList.Last());
        aList.Append(5);
        Assert.Equal([1, 3, 5], aList);
    }

    [Fact]
    public void ClearDeleted_renumbers_links_and_nodes_ReproducesOcctError()
    {
        BRepMesh_DataStructureOfDelaun aStructure = new();
        for (int i = 0; i < 5; ++i)
        {
            aStructure.AddNode(new BRepMesh_Vertex(i, i * i, BRepMesh_Free));
        }

        int l12 = aStructure.AddLink(new BRepMesh_Edge(1, 2, BRepMesh_Free));
        int l23 = aStructure.AddLink(new BRepMesh_Edge(2, 3, BRepMesh_Free));
        int l45 = aStructure.AddLink(new BRepMesh_Edge(4, 5, BRepMesh_Free));
        Assert.Equal((1, 2, 3), (l12, l23, l45));
        aStructure.RemoveLink(l12);
        aStructure.RemoveLink(l23);
        aStructure.RemoveNode(3);
        aStructure.RemoveNode(1);

        aStructure.ClearDeleted();

        // The links: link 3 (4-5) moves to index 1.
        Assert.Equal(1, aStructure.NbLinks());
        BRepMesh_Edge aLink = aStructure.GetLink(1);
        Assert.Equal(new gp_XY(3, 9), aStructure.GetNode(aLink.FirstNode()).Coord());
        Assert.Equal(new gp_XY(4, 16), aStructure.GetNode(aLink.LastNode()).Coord());
        Assert.Equal((1, 3), (aLink.FirstNode(), aLink.LastNode()));

        // OCCT: BRepMesh_VertexTool::RemoveLast deletes vertex Extent() but doesn't shrink the vector, so the second
        // RemoveLast of clearDeletedNodes deletes vertex 5 again instead of vertex 4: node 4 is left live, a duplicate of
        // the node moved to index 1, and NbNodes() stays 5 (occt-issues.md, unit 4B).
        Assert.Equal(5, aStructure.NbNodes());
        Assert.Equal(new gp_XY(3, 9), aStructure.GetNode(4).Coord());
        Assert.Equal(BRepMesh_Free, aStructure.GetNode(4).Movability());
        Assert.Equal(BRepMesh_Deleted, aStructure.GetNode(5).Movability());
    }

    [Fact]
    public void SubstituteElement_writes_one_triangle_too_far_ReproducesOcctError()
    {
        // OCCT: myElements(theIndex) indexes a vector from 0, so the triangle after theIndex is replaced
        // (occt-issues.md, unit 4B).
        BRepMesh_DataStructureOfDelaun aStructure = Quad(out _);
        BRepMesh_Triangle aFirst = aStructure.GetElement(1);
        BRepMesh_Triangle aSecond = aStructure.GetElement(2);
        aStructure.RemoveElement(1);
        Assert.True(aStructure.SubstituteElement(1, aSecond));
        Assert.Equal(BRepMesh_Deleted, aStructure.GetElement(1).Movability());
        Assert.Equal(aSecond, aStructure.GetElement(2));
        Assert.Equal(aFirst.WithMovability(BRepMesh_Deleted), aStructure.GetElement(1));
    }

    [Fact]
    public void ClearDomain_and_selector()
    {
        BRepMesh_DataStructureOfDelaun aStructure = Quad(out _);
        BRepMesh_SelectorOfDataStructureOfDelaun aSelector = new(aStructure);
        aSelector.NeighboursOfNode(2);
        Assert.Equal(1, aSelector.Elements().Extent());
        aSelector.NeighboursOfNode(4);
        Assert.Equal(2, aSelector.Elements().Extent());

        aStructure.ClearDomain();
        Assert.True(aStructure.ElementsOfDomain().IsEmpty());
        Assert.True(aStructure.LinksOfDomain().IsEmpty());
    }

    // ---- Mesh tool, classifier, triangulator

    [Fact]
    public void Legalize_flips_a_non_Delaunay_diagonal()
    {
        // A thin rhombus split along its long diagonal is flipped to its short one.
        BRepMesh_DataStructureOfDelaun aStructure = new();
        aStructure.AddNode(new BRepMesh_Vertex(0, 0, BRepMesh_Free));
        aStructure.AddNode(new BRepMesh_Vertex(2, -0.2, BRepMesh_Free));
        aStructure.AddNode(new BRepMesh_Vertex(4, 0, BRepMesh_Free));
        aStructure.AddNode(new BRepMesh_Vertex(2, 0.2, BRepMesh_Free));
        BRepMesh_MeshTool aTool = new(aStructure);
        aTool.AddAndLegalizeTriangle(1, 2, 3);
        aTool.AddAndLegalizeTriangle(1, 3, 4);
        aTool.EraseFreeLinks();

        List<(int, int, int)> aTriangles = BRepMeshDelaunTests.Triangles(aStructure);
        Assert.Equal(2, aTriangles.Count);
        Assert.All(aTriangles, t => Assert.True(new[] { t.Item1, t.Item2, t.Item3 }.Contains(2)
                                               && new[] { t.Item1, t.Item2, t.Item3 }.Contains(4)));
        Assert.Equal(0, aStructure.IndexOf(new BRepMesh_Edge(1, 3, BRepMesh_Free)));

        BRepMesh_MeshTool.NodeClassifier aClassifier = new(new BRepMesh_Edge(1, 3, BRepMesh_Free), aStructure);
        Assert.True(aClassifier.IsAbove(4) != aClassifier.IsAbove(2));
    }

    [Fact]
    public void Classifier_with_a_hole()
    {
        BRepMesh_Classifier aClassifier = new();
        NCollection_Sequence<gp_Pnt2d> anOuter = new();
        foreach ((double x, double y) in new[] { (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0) })
        {
            anOuter.Append(new gp_Pnt2d(x, y));
        }

        NCollection_Sequence<gp_Pnt2d> aHole = new();
        foreach ((double x, double y) in new[] { (1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0) })
        {
            aHole.Append(new gp_Pnt2d(x, y));
        }

        aClassifier.RegisterWire(anOuter, (1e-7, 1e-7), (0.0, 4.0), (0.0, 4.0));
        aClassifier.RegisterWire(aHole, (1e-7, 1e-7), (0.0, 4.0), (0.0, 4.0));

        Assert.Equal(TopAbs_State.TopAbs_IN, aClassifier.Perform(new gp_Pnt2d(0.5, 0.5)));
        Assert.Equal(TopAbs_State.TopAbs_OUT, aClassifier.Perform(new gp_Pnt2d(2.0, 2.0)));
        Assert.Equal(TopAbs_State.TopAbs_OUT, aClassifier.Perform(new gp_Pnt2d(5.0, 2.0)));
        Assert.Equal(TopAbs_State.TopAbs_OUT, aClassifier.Perform(new gp_Pnt2d(0.0, 2.0)));
    }

    [Fact]
    public void Triangulator_triangulates_a_polygon_with_a_hole()
    {
        NCollection_DynamicArray<gp_XYZ> aXYZs = new();
        // The outer square (indices 0..3) and the hole (4..7), in the plane Z = 1, counter-clockwise and clockwise.
        foreach ((double x, double y) in new[] { (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0) })
        {
            aXYZs.Append(new gp_XYZ(x, y, 1.0));
        }

        NCollection_Sequence<int> anOuter = new();
        NCollection_Sequence<int> aHole = new();
        for (int i = 0; i < 4; ++i)
        {
            anOuter.Append(i);
            aHole.Append(i + 4);
        }

        NCollection_List<NCollection_Sequence<int>> aWires = new();
        aWires.Append(anOuter);
        aWires.Append(aHole);

        NCollection_List<Poly_Triangle> aTriangles = new();
        Assert.True(new BRepMesh_Triangulator(aXYZs, aWires, gp.DZ()).Perform(aTriangles));
        Assert.Equal(8, aTriangles.Extent());

        NCollection_Array1<gp_Pnt> aNodes = new(1, 8);
        for (int i = 0; i < 8; ++i)
        {
            aNodes[i + 1] = new gp_Pnt(aXYZs[i]);
        }

        Poly_Triangulation aPoly = BRepMesh_Triangulator.ToPolyTriangulation(aNodes, aTriangles);
        double anArea = 0.0;
        for (int i = 1; i <= aPoly.NbTriangles(); ++i)
        {
            aPoly.Triangle(i).Get(out int n1, out int n2, out int n3);
            gp_XYZ a = aPoly.Node(n1).XYZ(), b = aPoly.Node(n2).XYZ(), c = aPoly.Node(n3).XYZ();
            anArea += 0.5 * (b - a).Crossed(c - a).Modulus();
        }

        Assert.Equal(12.0, anArea, 1e-12);
    }

    [Fact]
    public void Triangulator_simple_wires()
    {
        NCollection_DynamicArray<gp_XYZ> aXYZs = new();
        foreach ((double x, double y) in new[] { (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0) })
        {
            aXYZs.Append(new gp_XYZ(x, y, 0.0));
        }

        NCollection_Sequence<int> aWire = new();
        for (int i = 0; i < 4; ++i)
        {
            aWire.Append(i);
        }

        NCollection_List<NCollection_Sequence<int>> aWires = new();
        aWires.Append(aWire);
        NCollection_List<Poly_Triangle> aTriangles = new();
        Assert.True(new BRepMesh_Triangulator(aXYZs, aWires, gp.DZ()).Perform(aTriangles));
        // The diagonals are equal: d24 < d13 is false, and the inner-corner check keeps the diagonal 1-3.
        Assert.Equal([new Poly_Triangle(1, 2, 3), new Poly_Triangle(3, 4, 1)], aTriangles);
    }
}
