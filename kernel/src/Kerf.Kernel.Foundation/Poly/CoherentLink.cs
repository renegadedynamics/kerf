// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentLink.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentLink.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A link between two nodes of a <see cref="CoherentTriangulation"/>, made by its triangles: the two nodes (the
/// smaller index first) and the opposite node of the triangle on each side (left, then right; -1 if none). A link can
/// carry an arbitrary attribute.
/// </summary>
/// <remarks>
/// A class, as triangles refer to their links (OCCT stores pointers to them). OCCT's <c>void*</c> attribute is an
/// <c>object</c>.
/// </remarks>
internal class CoherentLink
{
    internal NCollection_Buffer2<int> _node;
    internal NCollection_Buffer2<int> _oppositeNode;
    private object? _attribute;

    /// <summary>An empty (invalid) link.</summary>
    public CoherentLink()
    {
        _attribute = null;
        _node[0] = -1;
        _node[1] = -1;
        _oppositeNode[0] = -1;
        _oppositeNode[1] = -1;
    }

    /// <summary>A link between two nodes, without opposite nodes: a temporary link outside any triangulation.</summary>
    public CoherentLink(int iNode0, int iNode1)
    {
        _attribute = null;
        _node[0] = iNode0;
        _node[1] = iNode1;
        _oppositeNode[0] = -1;
        _oppositeNode[1] = -1;
    }

    /// <summary>
    /// The link on side <paramref name="iSide"/> (0, 1 or 2: the index of the opposite node) of a triangle, with the
    /// smaller node first and the opposite nodes set.
    /// </summary>
    public CoherentLink(CoherentTriangle tri, int iSide)
    {
        ReadOnlySpan<int> ind = [1, 2, 0, 1];
        _attribute = null;
        Standard_ProgramError_Raise_if(iSide < 0 || iSide > 2, "Poly_CoherentLink::Poly_CoherentLink: Wrong iSide parameter");
        ReadOnlySpan<int> nodeInd = [tri.Node(ind[iSide + 0]), tri.Node(ind[iSide + 1])];
        if (nodeInd[0] < nodeInd[1])
        {
            _node[0] = nodeInd[0];
            _node[1] = nodeInd[1];
            _oppositeNode[0] = tri.Node(iSide);
            _oppositeNode[1] = tri.GetConnectedNode(iSide);
        }
        else
        {
            _node[0] = nodeInd[1];
            _node[1] = nodeInd[0];
            _oppositeNode[0] = tri.GetConnectedNode(iSide);
            _oppositeNode[1] = tri.Node(iSide);
        }
    }

    /// <summary>A node of the link: 0 or 1; Node(0) is the smaller index.</summary>
    public int Node(int ind) => _node[ind & 0x1];

    /// <summary>The opposite node of the triangle on the left (0) or right (1) side of the link, or -1.</summary>
    public int OppositeNode(int ind) => _oppositeNode[ind & 0x1];

    /// <summary>The attribute of the link.</summary>
    public object? GetAttribute() => _attribute;

    /// <summary>Sets the attribute of the link.</summary>
    public void SetAttribute(object? att) => _attribute = att;

    /// <summary>True if the link is invalid (a node is -1).</summary>
    public bool IsEmpty => _node[0] < 0 || _node[1] < 0;

    /// <summary>Invalidates this link.</summary>
    public void Nullify()
    {
        _node[0] = -1;
        _node[1] = -1;
        _oppositeNode[0] = -1;
        _oppositeNode[1] = -1;
    }

    // C++ assignment (theLink = Poly_CoherentLink()), used by Poly_CoherentTriangulation
    internal void Assign(CoherentLink other)
    {
        _node = other._node;
        _oppositeNode = other._oppositeNode;
        _attribute = other._attribute;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_CoherentLink({_node[0]}, {_node[1]}; opposite {_oppositeNode[0]}, {_oppositeNode[1]})");
}
