// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using System.Reflection;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the Delaunay core's pieces (unit 4B): the plain types, the vertex and circle tools, the data
/// structure (with the OCCT semantics decision 541 keeps), the mesh tool, the classifier and the triangulator.
/// </summary>
public class BRepMeshCoreTests
{
    // ---- A thread's lists

    [Fact]
    public void MeshWorkList_behaves_as_a_list()
    {
        MeshWorkList<int> aList = new(2);
        aList.Add(1);
        aList.Add(2);
        aList.Add(3);
        Assert.Equal(3, aList.Count);
        aList[1] = 20;
        Assert.Equal(20, aList[1]);
        Assert.Throws<ArgumentOutOfRangeException>(() => aList[3]);
        Assert.Throws<ArgumentOutOfRangeException>(() => aList[-1]);
    }

    [Fact]
    public void MeshWorkList_SetAt_past_the_end_fills_with_defaults_even_over_an_earlier_use()
    {
        MeshWorkList<int> aList = new(8);
        for (int i = 0; i < 8; i++)
        {
            aList.Add(7);
        }
        aList.Empty();
        Assert.Equal(0, aList.Count);
        aList.SetAt(3, 5);
        Assert.Equal(4, aList.Count);
        Assert.Equal([0, 0, 0, 5], [aList[0], aList[1], aList[2], aList[3]]);
    }

    [Fact]
    public void MeshWorkList_holds_a_large_array_only_until_a_full_collection()
    {
        MeshWorkList<long> aList = new(256);
        WeakReference anArray = UseALargeArrayTwice(aList);
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
        Assert.False(anArray.IsAlive);
        aList.TakeHeldArray();
        Assert.True(ArrayOf(aList).Target is long[] { Length: <= 8192 });

        // In a method of its own, so no temporary on this frame's stack holds the array
        [MethodImpl(MethodImplOptions.NoInlining)]
        static WeakReference UseALargeArrayTwice(MeshWorkList<long> theList)
        {
            for (int i = 0; i < 20_000; i++)
            {
                theList.Add(i);
            }
            WeakReference anArray = ArrayOf(theList);
            theList.Empty();
            Assert.Equal(0, theList.Count);
            Assert.True(ArrayOf(theList).Target is long[] { Length: <= 8192 }); // Back on the array that stays

            theList.TakeHeldArray();
            Assert.Same(anArray.Target, ArrayOf(theList).Target); // The next large face takes it
            theList.Empty();
            return anArray;
        }

        static WeakReference ArrayOf(MeshWorkList<long> theList) => new(
            typeof(MeshWorkList<long>).GetField("_items", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(theList));
    }

    [Fact]
    public void MeshWorkStorage_gives_empty_lists_each_time()
    {
        MeshWorkStorage aStorage = MeshWorkStorage.Rent();
        aStorage.Elements.Add(new Triangle());
        aStorage.Circles.Add(new MeshCircle());
        aStorage.CirclesTaken = true;
        MeshWorkStorage.Return(aStorage);
        MeshWorkStorage aNext = MeshWorkStorage.Rent();
        Assert.Same(aStorage, aNext);
        Assert.Equal(0, aNext.Elements.Count);
        Assert.Equal(0, aNext.Circles.Count);
        Assert.False(aNext.CirclesTaken);
        Assert.NotSame(aNext, MeshWorkStorage.Rent()); // In use: a nested use makes its own
        MeshWorkStorage.Return(aNext);
    }

    // ---- Plain types

    [Fact]
    public void PairOfIndex_holds_at_most_two_indices()
    {
        PairOfIndex aPair = new();
        Assert.True(aPair.IsEmpty);
        Assert.Equal(0, aPair.Extent());
        aPair = aPair.Appended(5).Appended(7);
        Assert.Equal(2, aPair.Extent());
        Assert.Equal(5, aPair.FirstIndex());
        Assert.Equal(7, aPair.LastIndex());
        EngineAssert.Throws<OutOfRangeException>(() => aPair.Appended(9));
        EngineAssert.Throws<OutOfRangeException>(() => aPair.Index(3));
        PairOfIndex aRemoved = aPair.IndexRemoved(1);
        Assert.Equal(1, aRemoved.Extent());
        Assert.Equal(7, aRemoved.FirstIndex());
        Assert.Equal(7, aRemoved.LastIndex());
        Assert.True(default(PairOfIndex) != new PairOfIndex());
    }

    [Fact]
    public void Edge_equality_is_OCCTs_in_IsEqual_and_exact_in_Equals()
    {
        MeshEdge anEdge = new(1, 2, DegreeOfFreedom.Free);
        MeshEdge aReversed = new(2, 1, DegreeOfFreedom.Frontier);
        Assert.True(anEdge.IsEqual(aReversed));
        Assert.False(anEdge.IsSameOrientation(aReversed));
        Assert.False(anEdge.Equals(aReversed));
        Assert.True(MeshEdge.Hasher.Instance.Equals(anEdge, aReversed));
        Assert.Equal(MeshEdge.Hasher.Instance.GetHashCode(anEdge), MeshEdge.Hasher.Instance.GetHashCode(aReversed));

        // A deleted link equals nothing, not even itself (OCCT's operator==).
        MeshEdge aDeleted = anEdge.WithMovability(DegreeOfFreedom.Deleted);
        Assert.False(aDeleted.IsEqual(aDeleted));
        Assert.True(aDeleted.Equals(aDeleted));
        Assert.Equal(DegreeOfFreedom.Deleted, new MeshEdge().Movability());

        OrientedEdge anOriented = anEdge;
        Assert.Equal(new OrientedEdge(1, 2), anOriented);
        Assert.Equal(1 | (2 << 16), anOriented.GetHashCode());
    }

    [Fact]
    public void Triangle_and_vertex_equality()
    {
        Triangle aTriangle = new([1, 2, 3], [true, false, true], DegreeOfFreedom.Free);
        Triangle aRotated = new([2, 3, 1], [false, true, true], DegreeOfFreedom.Free);
        Assert.True(aTriangle.IsEqual(aRotated));
        Assert.False(aTriangle.Equals(aRotated));
        Assert.False(aTriangle.IsEqual(new Triangle([1, 3, 2], [true, true, true], DegreeOfFreedom.Free)));
        Assert.False(aTriangle.WithMovability(DegreeOfFreedom.Deleted).IsEqual(aRotated));
        Span<int> e = stackalloc int[3];
        Span<bool> o = stackalloc bool[3];
        aTriangle.Edges(e, o);
        Assert.Equal([1, 2, 3], e.ToArray());
        Assert.Equal([true, false, true], o.ToArray());

        MeshVertex aVertex = new(new Xy(1.0, 2.0), 7, DegreeOfFreedom.Frontier);
        MeshVertex aNear = new(1.0 + 1e-10, 2.0, DegreeOfFreedom.Free);
        Assert.True(aVertex.IsEqual(aNear));
        Assert.False(aVertex.Equals(aNear));
        Assert.False(aVertex.IsEqual(aNear.WithMovability(DegreeOfFreedom.Deleted)));
        Assert.Equal(new MeshVertex(), default);
    }

    // ---- Tools

    [Fact]
    public void VertexTool_merges_coincident_vertices_and_reuses_deleted_indices()
    {
        VertexTool aTool = new();
        int a = aTool.Add(new MeshVertex(0.0, 0.0, DegreeOfFreedom.Free), false);
        int b = aTool.Add(new MeshVertex(1.0, 0.0, DegreeOfFreedom.Free), false);
        int aSame = aTool.Add(new MeshVertex(1.0 + 1e-8, 0.0, DegreeOfFreedom.Free), false);
        int aForced = aTool.Add(new MeshVertex(1.0 + 1e-8, 0.0, DegreeOfFreedom.Free), true);
        Assert.Equal((1, 2, 2, 3), (a, b, aSame, aForced));
        Assert.Equal(2, aTool.FindIndex(new MeshVertex(1.0, 0.0, DegreeOfFreedom.Free)));

        aTool.DeleteVertex(1);
        Assert.Equal(0, aTool.FindIndex(new MeshVertex(0.0, 0.0, DegreeOfFreedom.Free)));
        Assert.Equal(1, aTool.Add(new MeshVertex(5.0, 5.0, DegreeOfFreedom.Free), false));
        Assert.Equal(1, aTool.FindIndex(new MeshVertex(5.0, 5.0, DegreeOfFreedom.Free)));
    }

    [Fact]
    public void CircleTool_selects_the_circles_containing_a_point()
    {
        CircleTool aTool = new();
        aTool.SetMinMaxSize(new Xy(-10, -10), new Xy(10, 10));
        aTool.SetCellSize(1.0, 1.0);
        Assert.True(aTool.Bind(1, new Xy(0, 0), new Xy(2, 0), new Xy(0, 2)));
        Assert.True(aTool.Bind(2, new Xy(5, 5), new Xy(6, 5), new Xy(5, 6)));
        Assert.False(aTool.Bind(3, new Xy(0, 0), new Xy(1, 1), new Xy(2, 2)));
        aTool.MocBind(3);

        Assert.Equal([1], aTool.Select(new Xy(1.0, 1.0)));
        Assert.Equal([2], aTool.Select(new Xy(5.4, 5.4)));
        Assert.Empty(aTool.Select(new Xy(-5.0, 5.0)));

        aTool.Delete(1);
        Assert.Empty(aTool.Select(new Xy(1.0, 1.0)));
    }

    // ---- Data structure

    private static DataStructureOfDelaun Quad(out int[] theNodes)
    {
        // Two triangles (1, 2, 3) and (1, 3, 4) of the unit square.
        DataStructureOfDelaun aStructure = new();
        theNodes =
        [
            aStructure.AddNode(new MeshVertex(0, 0, DegreeOfFreedom.Free)),
            aStructure.AddNode(new MeshVertex(1, 0, DegreeOfFreedom.Free)),
            aStructure.AddNode(new MeshVertex(1, 1, DegreeOfFreedom.Free)),
            aStructure.AddNode(new MeshVertex(0, 1, DegreeOfFreedom.Free)),
        ];
        MeshTool aTool = new(aStructure);
        Span<int> e = stackalloc int[3];
        aTool.AddTriangle(1, 2, 3, e);
        aTool.AddTriangle(1, 3, 4, e);
        return aStructure;
    }

    [Fact]
    public void Links_are_found_in_either_orientation_and_deleted_indices_are_reused()
    {
        DataStructureOfDelaun aStructure = Quad(out _);
        Assert.Equal(5, aStructure.LinkCount);
        int aDiagonal = aStructure.IndexOf(new MeshEdge(3, 1, DegreeOfFreedom.Free));
        // The triangles stored the diagonal as 3 -> 1: the other orientation gives a negative index.
        Assert.Equal(aDiagonal, aStructure.AddLink(new MeshEdge(3, 1, DegreeOfFreedom.Free)));
        Assert.Equal(-aDiagonal, aStructure.AddLink(new MeshEdge(1, 3, DegreeOfFreedom.Free)));
        Assert.Equal(2, aStructure.ElementsConnectedTo(aDiagonal).Extent());

        // ElementsConnectedTo is a reference that sees the removal of a triangle.
        ref readonly PairOfIndex aPair = ref aStructure.ElementsConnectedTo(aDiagonal);
        aStructure.RemoveElement(1);
        Assert.Equal(1, aPair.Extent());
        Assert.Equal(2, aPair.FirstIndex());
        aStructure.RemoveElement(2);
        Assert.True(aPair.IsEmpty);

        aStructure.RemoveLink(aDiagonal);
        Assert.Equal(DegreeOfFreedom.Deleted, aStructure.GetLink(aDiagonal).Movability());
        Assert.Equal(0, aStructure.IndexOf(new MeshEdge(1, 3, DegreeOfFreedom.Free)));
        Assert.DoesNotContain(aDiagonal, aStructure.LinksConnectedTo(1));
        Assert.Equal(aDiagonal, aStructure.AddLink(new MeshEdge(2, 4, DegreeOfFreedom.Free)));
        Assert.Contains(aDiagonal, aStructure.LinksConnectedTo(4));
    }

    [Fact]
    public void A_removed_link_list_node_keeps_its_successor()
    {
        // OCCT's NCollection_List over an incremental allocator: a loop standing on a node that another loop removes
        // goes on to that node's old successor (BRepMesh_Delaun's killTrianglesAroundVertex relies on it).
        DataStructureOfDelaun.LinkedListOfInteger aList = new();
        foreach (int i in new[] { 1, 2, 3, 4 })
        {
            aList.Append(i);
        }

        DataStructureOfDelaun.LinkedListOfInteger.Iterator anOuter = new(aList);
        anOuter.Next(); // on 2
        DataStructureOfDelaun.LinkedListOfInteger.Iterator anInner = new(aList);
        anInner.Next();
        aList.Remove(anInner); // removes 2
        Assert.Equal(3, anInner.Value());
        Assert.Equal(2, anOuter.Value());
        anOuter.Next();
        Assert.Equal(3, anOuter.Value());
        Assert.Equal([1, 3, 4], aList);
        Assert.Equal(3, aList.Extent());

        // Removing the last node: Last moves back.
        DataStructureOfDelaun.LinkedListOfInteger.Iterator aLast = new(aList);
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
        DataStructureOfDelaun aStructure = new();
        for (int i = 0; i < 5; ++i)
        {
            aStructure.AddNode(new MeshVertex(i, i * i, DegreeOfFreedom.Free));
        }

        int l12 = aStructure.AddLink(new MeshEdge(1, 2, DegreeOfFreedom.Free));
        int l23 = aStructure.AddLink(new MeshEdge(2, 3, DegreeOfFreedom.Free));
        int l45 = aStructure.AddLink(new MeshEdge(4, 5, DegreeOfFreedom.Free));
        Assert.Equal((1, 2, 3), (l12, l23, l45));
        aStructure.RemoveLink(l12);
        aStructure.RemoveLink(l23);
        aStructure.RemoveNode(3);
        aStructure.RemoveNode(1);

        aStructure.ClearDeleted();

        // The links: link 3 (4-5) moves to index 1.
        Assert.Equal(1, aStructure.LinkCount);
        MeshEdge aLink = aStructure.GetLink(1);
        Assert.Equal(new Xy(3, 9), aStructure.GetNode(aLink.FirstNode()).Coord());
        Assert.Equal(new Xy(4, 16), aStructure.GetNode(aLink.LastNode()).Coord());
        Assert.Equal((1, 3), (aLink.FirstNode(), aLink.LastNode()));

        // OCCT: BRepMesh_VertexTool::RemoveLast deletes vertex Extent() but doesn't shrink the vector, so the second
        // RemoveLast of clearDeletedNodes deletes vertex 5 again instead of vertex 4: node 4 is left live, a duplicate of
        // the node moved to index 1, and NbNodes() stays 5 (occt-issues.md, unit 4B).
        Assert.Equal(5, aStructure.NodeCount);
        Assert.Equal(new Xy(3, 9), aStructure.GetNode(4).Coord());
        Assert.Equal(DegreeOfFreedom.Free, aStructure.GetNode(4).Movability());
        Assert.Equal(DegreeOfFreedom.Deleted, aStructure.GetNode(5).Movability());
    }

    [Fact]
    public void SubstituteElement_writes_one_triangle_too_far_ReproducesOcctError()
    {
        // OCCT: myElements(theIndex) indexes a vector from 0, so the triangle after theIndex is replaced
        // (occt-issues.md, unit 4B).
        DataStructureOfDelaun aStructure = Quad(out _);
        Triangle aFirst = aStructure.GetElement(1);
        Triangle aSecond = aStructure.GetElement(2);
        aStructure.RemoveElement(1);
        Assert.True(aStructure.SubstituteElement(1, aSecond));
        Assert.Equal(DegreeOfFreedom.Deleted, aStructure.GetElement(1).Movability());
        Assert.Equal(aSecond, aStructure.GetElement(2));
        Assert.Equal(aFirst.WithMovability(DegreeOfFreedom.Deleted), aStructure.GetElement(1));
    }

    [Fact]
    public void ClearDomain_and_selector()
    {
        DataStructureOfDelaun aStructure = Quad(out _);
        SelectorOfDataStructureOfDelaun aSelector = new(aStructure);
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
        DataStructureOfDelaun aStructure = new();
        aStructure.AddNode(new MeshVertex(0, 0, DegreeOfFreedom.Free));
        aStructure.AddNode(new MeshVertex(2, -0.2, DegreeOfFreedom.Free));
        aStructure.AddNode(new MeshVertex(4, 0, DegreeOfFreedom.Free));
        aStructure.AddNode(new MeshVertex(2, 0.2, DegreeOfFreedom.Free));
        MeshTool aTool = new(aStructure);
        aTool.AddAndLegalizeTriangle(1, 2, 3);
        aTool.AddAndLegalizeTriangle(1, 3, 4);
        aTool.EraseFreeLinks();

        List<(int, int, int)> aTriangles = BRepMeshDelaunTests.Triangles(aStructure);
        Assert.Equal(2, aTriangles.Count);
        Assert.All(aTriangles, t => Assert.True(new[] { t.Item1, t.Item2, t.Item3 }.Contains(2)
                                               && new[] { t.Item1, t.Item2, t.Item3 }.Contains(4)));
        Assert.Equal(0, aStructure.IndexOf(new MeshEdge(1, 3, DegreeOfFreedom.Free)));

        MeshTool.NodeClassifier aClassifier = new(new MeshEdge(1, 3, DegreeOfFreedom.Free), aStructure);
        Assert.True(aClassifier.IsAbove(4) != aClassifier.IsAbove(2));
    }

    [Fact]
    public void Classifier_with_a_hole()
    {
        Classifier aClassifier = new();
        NCollection_Sequence<Point2D> anOuter = new();
        foreach ((double x, double y) in new[] { (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0) })
        {
            anOuter.Append(new Point2D(x, y));
        }

        NCollection_Sequence<Point2D> aHole = new();
        foreach ((double x, double y) in new[] { (1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0) })
        {
            aHole.Append(new Point2D(x, y));
        }

        aClassifier.RegisterWire(anOuter, (1e-7, 1e-7), (0.0, 4.0), (0.0, 4.0));
        aClassifier.RegisterWire(aHole, (1e-7, 1e-7), (0.0, 4.0), (0.0, 4.0));

        Assert.Equal(Containment.In, aClassifier.Perform(new Point2D(0.5, 0.5)));
        Assert.Equal(Containment.Out, aClassifier.Perform(new Point2D(2.0, 2.0)));
        Assert.Equal(Containment.Out, aClassifier.Perform(new Point2D(5.0, 2.0)));
        Assert.Equal(Containment.Out, aClassifier.Perform(new Point2D(0.0, 2.0)));
    }

    [Fact]
    public void Triangulator_triangulates_a_polygon_with_a_hole()
    {
        List<Xyz> aXYZs = new();
        // The outer square (indices 0..3) and the hole (4..7), in the plane Z = 1, counter-clockwise and clockwise.
        foreach ((double x, double y) in new[] { (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0) })
        {
            aXYZs.Add(new Xyz(x, y, 1.0));
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

        NCollection_List<Triangle1> aTriangles = new();
        Assert.True(new Triangulator(aXYZs, aWires, Constants.DZ()).Perform(aTriangles));
        Assert.Equal(8, aTriangles.Extent());

        NCollection_Array1<Point3D> aNodes = new(1, 8);
        for (int i = 0; i < 8; ++i)
        {
            aNodes[i + 1] = new Point3D(aXYZs[i]);
        }

        Triangulation aPoly = Triangulator.ToPolyTriangulation(aNodes, aTriangles);
        double anArea = 0.0;
        for (int i = 1; i <= aPoly.TriangleCount; ++i)
        {
            aPoly.Triangle(i).Get(out int n1, out int n2, out int n3);
            Xyz a = aPoly.Node(n1).XYZ(), b = aPoly.Node(n2).XYZ(), c = aPoly.Node(n3).XYZ();
            anArea += 0.5 * (b - a).Crossed(c - a).Modulus();
        }

        Assert.Equal(12.0, anArea, 1e-12);
    }

    [Fact]
    public void Triangulator_simple_wires()
    {
        List<Xyz> aXYZs = new();
        foreach ((double x, double y) in new[] { (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0) })
        {
            aXYZs.Add(new Xyz(x, y, 0.0));
        }

        NCollection_Sequence<int> aWire = new();
        for (int i = 0; i < 4; ++i)
        {
            aWire.Append(i);
        }

        NCollection_List<NCollection_Sequence<int>> aWires = new();
        aWires.Append(aWire);
        NCollection_List<Triangle1> aTriangles = new();
        Assert.True(new Triangulator(aXYZs, aWires, Constants.DZ()).Perform(aTriangles));
        // The diagonals are equal: d24 < d13 is false, and the inner-corner check keeps the diagonal 1-3.
        Assert.Equal([new Triangle1(1, 2, 3), new Triangle1(3, 4, 1)], aTriangles);
    }
}
