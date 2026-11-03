// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SelectorOfDataStructureOfDelaun.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SelectorOfDataStructureOfDelaun.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>Collects the triangles around nodes, links or triangles of a <see cref="BRepMesh_DataStructureOfDelaun"/>.</summary>
/// <remarks>A working object, as OCCT's. Only <see cref="Elements"/> is ever filled, as in OCCT.</remarks>
public sealed class BRepMesh_SelectorOfDataStructureOfDelaun : Transient
{
    private BRepMesh_DataStructureOfDelaun? myMesh;
    private readonly NCollection_PackedMapOfInteger myNodes = new();
    private readonly NCollection_PackedMapOfInteger myLinks = new();
    private readonly NCollection_PackedMapOfInteger myElements = new();
    private readonly NCollection_PackedMapOfInteger myFrontier = new();

    /// <summary>A selector without a mesh.</summary>
    public BRepMesh_SelectorOfDataStructureOfDelaun() { }

    /// <summary>A selector of the given mesh.</summary>
    public BRepMesh_SelectorOfDataStructureOfDelaun(BRepMesh_DataStructureOfDelaun theMesh) => myMesh = theMesh;

    /// <summary>Sets the mesh and clears the selection.</summary>
    public void Initialize(BRepMesh_DataStructureOfDelaun theMesh)
    {
        myMesh = theMesh;
        myNodes.Clear();
        myLinks.Clear();
        myElements.Clear();
        myFrontier.Clear();
    }

    /// <summary>Selects the triangles around the node coinciding with the given one.</summary>
    public void NeighboursOf(in BRepMesh_Vertex theNode) => NeighboursOfNode(myMesh!.IndexOf(theNode));

    /// <summary>Selects the triangles around the node of the given index.</summary>
    public void NeighboursOfNode(int theNodeIndex)
    {
        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator aLinkIt = new(myMesh!.LinksConnectedTo(theNodeIndex));

        for (; aLinkIt.More(); aLinkIt.Next())
        {
            elementsOfLink(aLinkIt.Value());
        }
    }

    /// <summary>Selects the triangles around both nodes of the link.</summary>
    public void NeighboursOf(in BRepMesh_Edge theLink)
    {
        NeighboursOfNode(theLink.FirstNode());
        NeighboursOfNode(theLink.LastNode());
    }

    /// <summary>Selects the triangles around both nodes of the link of the given index.</summary>
    public void NeighboursOfLink(int theLinkIndex) => NeighboursOf(myMesh!.GetLink(theLinkIndex));

    /// <summary>Selects the triangles around the three nodes of the triangle.</summary>
    public void NeighboursOf(in BRepMesh_Triangle theElement)
    {
        Span<int> v = stackalloc int[3];
        myMesh!.ElementNodes(theElement, v);

        for (int i = 0; i < 3; ++i)
        {
            NeighboursOfNode(v[i]);
        }
    }

    /// <summary>Selects the triangles around the three nodes of the triangle of the given index.</summary>
    public void NeighboursOfElement(int theElementIndex) => NeighboursOf(myMesh!.GetElement(theElementIndex));

    /// <summary>Selects the triangles sharing a link with the triangle.</summary>
    public void NeighboursByEdgeOf(in BRepMesh_Triangle theElement)
    {
        BRepMesh_Triangle.Int3 e = theElement.myEdges;
        for (int i = 0; i < 3; ++i)
        {
            elementsOfLink(e[i]);
        }
    }

    /// <summary>Does nothing (as OCCT's).</summary>
    public void NeighboursOf(BRepMesh_SelectorOfDataStructureOfDelaun theSelector) => _ = theSelector;

    /// <summary>Does nothing (as OCCT's).</summary>
    public void AddNeighbours() { }

    /// <summary>The selected nodes (never filled, as in OCCT).</summary>
    public NCollection_PackedMapOfInteger Nodes() => myNodes;

    /// <summary>The selected links (never filled, as in OCCT).</summary>
    public NCollection_PackedMapOfInteger Links() => myLinks;

    /// <summary>The selected triangles.</summary>
    public NCollection_PackedMapOfInteger Elements() => myElements;

    /// <summary>The selected frontier links (never filled, as in OCCT).</summary>
    public NCollection_PackedMapOfInteger FrontierLinks() => myFrontier;

    private void elementsOfLink(int theIndex)
    {
        ref readonly BRepMesh_PairOfIndex aPair = ref myMesh!.ElementsConnectedTo(theIndex);
        for (int j = 1, jn = aPair.Extent(); j <= jn; ++j)
        {
            myElements.Add(aPair.Index(j));
        }
    }
}
