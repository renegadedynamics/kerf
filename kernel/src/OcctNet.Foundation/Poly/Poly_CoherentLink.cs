// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentLink.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentLink.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A link between two nodes of a <see cref="Poly_CoherentTriangulation"/>, made by its triangles: the two nodes (the
/// smaller index first) and the opposite node of the triangle on each side (left, then right; -1 if none). A link can
/// carry an arbitrary attribute.
/// </summary>
/// <remarks>
/// A class, as triangles refer to their links (OCCT stores pointers to them). OCCT's <c>void*</c> attribute is an
/// <c>object</c>.
/// </remarks>
public class Poly_CoherentLink
{
    internal NCollection_Buffer2<int> myNode;
    internal NCollection_Buffer2<int> myOppositeNode;
    private object? myAttribute;

    /// <summary>An empty (invalid) link.</summary>
    public Poly_CoherentLink()
    {
        myAttribute = null;
        myNode[0] = -1;
        myNode[1] = -1;
        myOppositeNode[0] = -1;
        myOppositeNode[1] = -1;
    }

    /// <summary>A link between two nodes, without opposite nodes: a temporary link outside any triangulation.</summary>
    public Poly_CoherentLink(int iNode0, int iNode1)
    {
        myAttribute = null;
        myNode[0] = iNode0;
        myNode[1] = iNode1;
        myOppositeNode[0] = -1;
        myOppositeNode[1] = -1;
    }

    /// <summary>
    /// The link on side <paramref name="iSide"/> (0, 1 or 2: the index of the opposite node) of a triangle, with the
    /// smaller node first and the opposite nodes set.
    /// </summary>
    public Poly_CoherentLink(Poly_CoherentTriangle theTri, int iSide)
    {
        ReadOnlySpan<int> ind = [1, 2, 0, 1];
        myAttribute = null;
        Standard_ProgramError_Raise_if(iSide < 0 || iSide > 2, "Poly_CoherentLink::Poly_CoherentLink: Wrong iSide parameter");
        ReadOnlySpan<int> aNodeInd = [theTri.Node(ind[iSide + 0]), theTri.Node(ind[iSide + 1])];
        if (aNodeInd[0] < aNodeInd[1])
        {
            myNode[0] = aNodeInd[0];
            myNode[1] = aNodeInd[1];
            myOppositeNode[0] = theTri.Node(iSide);
            myOppositeNode[1] = theTri.GetConnectedNode(iSide);
        }
        else
        {
            myNode[0] = aNodeInd[1];
            myNode[1] = aNodeInd[0];
            myOppositeNode[0] = theTri.GetConnectedNode(iSide);
            myOppositeNode[1] = theTri.Node(iSide);
        }
    }

    /// <summary>A node of the link: 0 or 1; Node(0) is the smaller index.</summary>
    public int Node(int ind) => myNode[ind & 0x1];

    /// <summary>The opposite node of the triangle on the left (0) or right (1) side of the link, or -1.</summary>
    public int OppositeNode(int ind) => myOppositeNode[ind & 0x1];

    /// <summary>The attribute of the link.</summary>
    public object? GetAttribute() => myAttribute;

    /// <summary>Sets the attribute of the link.</summary>
    public void SetAttribute(object? theAtt) => myAttribute = theAtt;

    /// <summary>True if the link is invalid (a node is -1).</summary>
    public bool IsEmpty() => myNode[0] < 0 || myNode[1] < 0;

    /// <summary>Invalidates this link.</summary>
    public void Nullify()
    {
        myNode[0] = -1;
        myNode[1] = -1;
        myOppositeNode[0] = -1;
        myOppositeNode[1] = -1;
    }

    // C++ assignment (theLink = Poly_CoherentLink()), used by Poly_CoherentTriangulation
    internal void Assign(Poly_CoherentLink theOther)
    {
        myNode = theOther.myNode;
        myOppositeNode = theOther.myOppositeNode;
        myAttribute = theOther.myAttribute;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_CoherentLink({myNode[0]}, {myNode[1]}; opposite {myOppositeNode[0]}, {myOppositeNode[1]})");
}
