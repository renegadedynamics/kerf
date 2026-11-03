// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SelectorOfDataStructureOfDelaun.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SelectorOfDataStructureOfDelaun.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>Collects the triangles around nodes, links or triangles of a <see cref="DataStructureOfDelaun"/>.</summary>
/// <remarks>A working object, as OCCT's. Only <see cref="Elements"/> is ever filled, as in OCCT.</remarks>
public sealed class SelectorOfDataStructureOfDelaun : Transient
{
    private DataStructureOfDelaun? _mesh;
    private readonly NCollection_PackedMapOfInteger _nodes = new();
    private readonly NCollection_PackedMapOfInteger _links = new();
    private readonly NCollection_PackedMapOfInteger _elements = new();
    private readonly NCollection_PackedMapOfInteger _frontier = new();

    /// <summary>A selector without a mesh.</summary>
    public SelectorOfDataStructureOfDelaun() { }

    /// <summary>A selector of the given mesh.</summary>
    public SelectorOfDataStructureOfDelaun(DataStructureOfDelaun mesh) => _mesh = mesh;

    /// <summary>Sets the mesh and clears the selection.</summary>
    public void Initialize(DataStructureOfDelaun mesh)
    {
        _mesh = mesh;
        _nodes.Clear();
        _links.Clear();
        _elements.Clear();
        _frontier.Clear();
    }

    /// <summary>Selects the triangles around the node coinciding with the given one.</summary>
    public void NeighboursOf(in MeshVertex node) => NeighboursOfNode(_mesh!.IndexOf(node));

    /// <summary>Selects the triangles around the node of the given index.</summary>
    public void NeighboursOfNode(int nodeIndex)
    {
        DataStructureOfDelaun.LinkedListOfInteger.Iterator linkIt = new(_mesh!.LinksConnectedTo(nodeIndex));

        for (; linkIt.More(); linkIt.Next())
        {
            ElementsOfLink(linkIt.Value());
        }
    }

    /// <summary>Selects the triangles around both nodes of the link.</summary>
    public void NeighboursOf(in MeshEdge link)
    {
        NeighboursOfNode(link.FirstNode());
        NeighboursOfNode(link.LastNode());
    }

    /// <summary>Selects the triangles around both nodes of the link of the given index.</summary>
    public void NeighboursOfLink(int linkIndex) => NeighboursOf(_mesh!.GetLink(linkIndex));

    /// <summary>Selects the triangles around the three nodes of the triangle.</summary>
    public void NeighboursOf(in Triangle element)
    {
        Span<int> v = stackalloc int[3];
        _mesh!.ElementNodes(element, v);

        for (int i = 0; i < 3; ++i)
        {
            NeighboursOfNode(v[i]);
        }
    }

    /// <summary>Selects the triangles around the three nodes of the triangle of the given index.</summary>
    public void NeighboursOfElement(int elementIndex) => NeighboursOf(_mesh!.GetElement(elementIndex));

    /// <summary>Selects the triangles sharing a link with the triangle.</summary>
    public void NeighboursByEdgeOf(in Triangle element)
    {
        Triangle.Int3 e = element._edges;
        for (int i = 0; i < 3; ++i)
        {
            ElementsOfLink(e[i]);
        }
    }

    /// <summary>Does nothing (as OCCT's).</summary>
    public void NeighboursOf(SelectorOfDataStructureOfDelaun selector) => _ = selector;

    /// <summary>Does nothing (as OCCT's).</summary>
    public void AddNeighbours() { }

    /// <summary>The selected nodes (never filled, as in OCCT).</summary>
    public NCollection_PackedMapOfInteger Nodes() => _nodes;

    /// <summary>The selected links (never filled, as in OCCT).</summary>
    public NCollection_PackedMapOfInteger Links() => _links;

    /// <summary>The selected triangles.</summary>
    public NCollection_PackedMapOfInteger Elements() => _elements;

    /// <summary>The selected frontier links (never filled, as in OCCT).</summary>
    public NCollection_PackedMapOfInteger FrontierLinks() => _frontier;

    private void ElementsOfLink(int index)
    {
        ref readonly PairOfIndex pair = ref _mesh!.ElementsConnectedTo(index);
        for (int j = 1, jn = pair.Extent(); j <= jn; ++j)
        {
            _elements.Add(pair.Index(j));
        }
    }
}
