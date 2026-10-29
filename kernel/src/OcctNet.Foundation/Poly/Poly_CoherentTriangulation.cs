// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangulation.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A triangulation for algorithms that analyse or edit a mesh (refining, decimation): each triangle knows its up to
/// three neighbours and their third nodes, each node the triangles using it, and links between nodes can be computed.
/// Nodes and triangles can be added, triangles removed, and the result converted back to a
/// <see cref="Poly_Triangulation"/>.
/// </summary>
/// <remarks>
/// <para>
/// All triangles are expected to be oriented coherently, as on the surface of a solid; a link shared by more than
/// two triangles isn't supported. Nodes are never removed, but a node used by no triangle is "free" and isn't exported.
/// Removed triangles and links stay in their arrays, empty (nodes -1); the iterators skip them.
/// </para>
/// <para>
/// OCCT's allocator arguments are dropped (<c>Allocator()</c>, the allocator of <c>Clone</c>). Nodes, triangles and
/// links are classes, so the references OCCT keeps as pointers stay valid as the arrays grow.
/// </para>
/// </remarks>
public class Poly_CoherentTriangulation : Standard_Transient
{
    /// <summary>Iterates the triangles, skipping removed ones. Triangles added during the iteration aren't visited.</summary>
    public sealed class IteratorOfTriangle
    {
        private NCollection_DynamicArray<Poly_CoherentTriangle>? myArray;
        private int myIndex;
        private int myLast;

        public IteratorOfTriangle(Poly_CoherentTriangulation? theTri)
        {
            if (theTri is not null)
            {
                myArray = theTri.myTriangles;
                myIndex = 0;
                myLast = myArray.Length();
                while (More())
                {
                    Poly_CoherentTriangle aTri = Value();
                    if (!aTri.IsEmpty())
                    {
                        break;
                    }

                    myIndex++;
                }
            }
        }

        public bool More() => myArray is not null && myIndex < myLast;

        public void Next()
        {
            myIndex++;
            while (More())
            {
                Poly_CoherentTriangle aTri = Value();
                if (!aTri.IsEmpty())
                {
                    break;
                }

                myIndex++;
            }
        }

        public Poly_CoherentTriangle Value() => myArray!.Value(myIndex);

        public Poly_CoherentTriangle ChangeValue() => myArray!.Value(myIndex);
    }

    /// <summary>Iterates the nodes, skipping free ones.</summary>
    public sealed class IteratorOfNode
    {
        private NCollection_DynamicArray<Poly_CoherentNode>? myArray;
        private int myIndex;
        private int myLast;

        public IteratorOfNode(Poly_CoherentTriangulation? theTri)
        {
            if (theTri is not null)
            {
                myArray = theTri.myNodes;
                myIndex = 0;
                myLast = myArray.Length();
                while (More())
                {
                    if (!Value().IsFreeNode())
                    {
                        break;
                    }

                    myIndex++;
                }
            }
        }

        public bool More() => myArray is not null && myIndex < myLast;

        public void Next()
        {
            myIndex++;
            while (More())
            {
                if (!Value().IsFreeNode())
                {
                    break;
                }

                myIndex++;
            }
        }

        public Poly_CoherentNode Value() => myArray!.Value(myIndex);

        public Poly_CoherentNode ChangeValue() => myArray!.Value(myIndex);

        /// <summary>The index of the current node.</summary>
        public int Index() => myIndex;
    }

    /// <summary>Iterates the links, skipping removed ones.</summary>
    public sealed class IteratorOfLink
    {
        private NCollection_DynamicArray<Poly_CoherentLink>? myArray;
        private int myIndex;
        private int myLast;

        public IteratorOfLink(Poly_CoherentTriangulation? theTri)
        {
            if (theTri is not null)
            {
                myArray = theTri.myLinks;
                myIndex = 0;
                myLast = myArray.Length();
                while (More())
                {
                    if (!Value().IsEmpty())
                    {
                        break;
                    }

                    myIndex++;
                }
            }
        }

        public bool More() => myArray is not null && myIndex < myLast;

        public void Next()
        {
            myIndex++;
            while (More())
            {
                if (!Value().IsEmpty())
                {
                    break;
                }

                myIndex++;
            }
        }

        public Poly_CoherentLink Value() => myArray!.Value(myIndex);

        public Poly_CoherentLink ChangeValue() => myArray!.Value(myIndex);
    }

    /// <summary>A pair of node indices (used by <see cref="RemoveDegenerated"/>: the removed node, then the one kept).</summary>
    public readonly record struct TwoIntegers(int myValue0, int myValue1);

    protected readonly NCollection_DynamicArray<Poly_CoherentTriangle> myTriangles = new();
    protected readonly NCollection_DynamicArray<Poly_CoherentNode> myNodes = new();
    protected readonly NCollection_DynamicArray<Poly_CoherentLink> myLinks = new();
    protected double myDeflection;

    /// <summary>An empty triangulation.</summary>
    public Poly_CoherentTriangulation()
    {
        myDeflection = 0.0;
    }

    /// <summary>
    /// The coherent form of a triangulation: its nodes (indexed from 0, each keeping its original index), its
    /// non-degenerate triangles, UV nodes and normals. Links aren't computed (<see cref="ComputeLinks"/>).
    /// </summary>
    public Poly_CoherentTriangulation(Poly_Triangulation? theTriangulation)
    {
        // OCCT leaves the deflection uninitialised for a null triangulation
        if (theTriangulation is not null)
        {
            int nNodes = theTriangulation.NbNodes();
            int i;
            // Copy the nodes
            for (i = 0; i < nNodes; i++)
            {
                int anOldInd = i + 1;
                int aNewInd = SetNode(theTriangulation.Node(anOldInd).XYZ(), i);
                Poly_CoherentNode aCopiedNode = myNodes[aNewInd];
                aCopiedNode.SetIndex(anOldInd);
            }

            // Copy the triangles
            Span<int> iNode = [0, 0, 0];
            for (i = 1; i <= theTriangulation.NbTriangles(); i++)
            {
                theTriangulation.Triangle(i).Get(out iNode[0], out iNode[1], out iNode[2]);
                if (iNode[0] != iNode[1] && iNode[1] != iNode[2] && iNode[2] != iNode[0])
                {
                    AddTriangle(iNode[0] - 1, iNode[1] - 1, iNode[2] - 1);
                }
            }

            // Copy UV coordinates of nodes
            if (theTriangulation.HasUVNodes())
            {
                for (i = 0; i < nNodes; i++)
                {
                    gp_Pnt2d anUV = theTriangulation.UVNode(i + 1);
                    myNodes[i].SetUV(anUV.X(), anUV.Y());
                }
            }

            // Copy the normals at nodes
            if (theTriangulation.HasNormals())
            {
                for (i = 0; i < nNodes; i++)
                {
                    theTriangulation.Normal(i + 1, out NCollection_Vec3<float> aNormal);
                    myNodes[i].SetNormal(new gp_XYZ(aNormal.x(), aNormal.y(), aNormal.z()));
                }
            }

            myDeflection = theTriangulation.Deflection();
        }
    }

    /// <summary>
    /// A <see cref="Poly_Triangulation"/> of the non-free nodes and the triangles, with UV nodes and normals where the
    /// nodes have them; null if there are no nodes or no triangles.
    /// </summary>
    public Poly_Triangulation? GetTriangulation()
    {
        int nNodes = NNodes();
        int nTriangles = NTriangles();
        if (nNodes == 0 || nTriangles == 0)
        {
            return null;
        }

        var aResult = new Poly_Triangulation(nNodes, nTriangles, false);

        var vecNodeId = new NCollection_DynamicArray<int>();
        int aCount = 0;

        // Copy the nodes (3D and 2D coordinates)
        for (int i = 0; i < myNodes.Length(); i++)
        {
            Poly_CoherentNode aNode = myNodes[i];
            if (aNode.IsFreeNode())
            {
                vecNodeId.SetValue(i, 0);
                continue;
            }

            vecNodeId.SetValue(i, ++aCount);
            gp_XYZ aNormal = aNode.GetNormal();
            if (aNormal.SquareModulus() > Precision.Confusion())
            {
                aResult.AddNormals();
                aResult.SetNormal(aCount, new gp_Dir(aNormal));
            }

            aResult.SetNode(aCount, new gp_Pnt(aNode.XYZ()));
            if (aNode.GetU() * aNode.GetU() + aNode.GetV() * aNode.GetV() > Precision.Confusion())
            {
                aResult.AddUVNodes();
                aResult.SetUVNode(aCount, new gp_Pnt2d(aNode.GetU(), aNode.GetV()));
            }
        }

        // Copy the triangles
        aCount = 0;
        for (int anIterT = 0, aLast = myTriangles.Length(); anIterT < aLast; anIterT++)
        {
            Poly_CoherentTriangle aTri = myTriangles[anIterT];
            if (!aTri.IsEmpty())
            {
                aResult.SetTriangle(++aCount,
                                    new Poly_Triangle(vecNodeId[aTri.Node(0)], vecNodeId[aTri.Node(1)], vecNodeId[aTri.Node(2)]));
            }
        }

        aResult.Deflection(myDeflection);
        return aResult;
    }

    /// <summary>
    /// Removes the degenerate triangles: those with two nodes closer than <paramref name="theTol"/>. The triangles
    /// around one of the two nodes are reconnected to the other; <paramref name="pLstRemovedNode"/>, if given, receives
    /// each pair (removed node, node kept). True if a triangle was removed.
    /// </summary>
    public bool RemoveDegenerated(double theTol, NCollection_List<TwoIntegers>? pLstRemovedNode = null)
    {
        bool aResult = false;
        double aTol2 = theTol * theTol;
        ReadOnlySpan<int> ind0 = [2, 0, 1, 2, 0];
        const int ind = 1; // ind = &ind0[1]

        pLstRemovedNode?.Clear();

        var anIterT = new IteratorOfTriangle(this);
        for (; anIterT.More(); anIterT.Next())
        {
            Poly_CoherentTriangle aTri = anIterT.ChangeValue();
            NCollection_Buffer3<Poly_CoherentNode> pNode = default;
            pNode[0] = ChangeNode(aTri.Node(0));
            pNode[1] = ChangeNode(aTri.Node(1));
            pNode[2] = ChangeNode(aTri.Node(2));
            ReadOnlySpan<double> aLen2 =
            [
                pNode[2].XYZ().Subtracted(pNode[1].XYZ()).SquareModulus(),
                pNode[0].XYZ().Subtracted(pNode[2].XYZ()).SquareModulus(),
                pNode[1].XYZ().Subtracted(pNode[0].XYZ()).SquareModulus(),
            ];
            for (int i = 0; i < 3; i++)
            {
                if (aLen2[i] < aTol2)
                {
                    int im1 = aTri.Node(ind0[ind + i - 1]);
                    int ip1 = aTri.Node(ind0[ind + i + 1]);

                    // Disconnect from both neighbours
                    RemoveTriangle(aTri);

                    // Reconnect all triangles from Node(ind[i+1]) to Node(ind[i-1])
                    for (;;)
                    {
                        Poly_CoherentTriPtr.Iterator anIterConn = pNode[ind0[ind + i + 1]].TriangleIterator();
                        if (!anIterConn.More())
                        {
                            break;
                        }

                        Poly_CoherentTriangle aTriConn = anIterConn.ChangeValue();
                        Span<int> aNewTriConn = [aTriConn.Node(0), aTriConn.Node(1), aTriConn.Node(2)];
                        if (aTriConn != aTri)
                        {
                            if (aNewTriConn[0] == ip1)
                            {
                                aNewTriConn[0] = im1;
                            }
                            else if (aNewTriConn[1] == ip1)
                            {
                                aNewTriConn[1] = im1;
                            }
                            else if (aNewTriConn[2] == ip1)
                            {
                                aNewTriConn[2] = im1;
                            }

                            RemoveTriangle(aTriConn);
                            AddTriangle(aNewTriConn[0], aNewTriConn[1], aNewTriConn[2]);
                        }
                        else
                        {
                            anIterConn.Next();
                            if (!anIterConn.More())
                            {
                                break;
                            }
                        }
                    }

                    pLstRemovedNode?.Append(new TwoIntegers(ip1, im1));
                    aResult = true;
                    break;
                }
            }
        }

        return aResult;
    }

    /// <summary>The indices of the free nodes (used by no triangle) in <paramref name="lstNodes"/>; true if there are any.</summary>
    public bool GetFreeNodes(NCollection_List<int> lstNodes)
    {
        lstNodes.Clear();
        int i;
        for (i = 0; i < myNodes.Length(); i++)
        {
            Poly_CoherentNode aNode = myNodes[i];
            if (aNode.IsFreeNode())
            {
                lstNodes.Append(i);
            }
        }

        return !lstNodes.IsEmpty();
    }

    /// <summary>The index of the last node.</summary>
    public int MaxNode() => myNodes.Length() - 1;

    /// <summary>The index of the last triangle.</summary>
    public int MaxTriangle() => myTriangles.Length() - 1;

    /// <summary>Sets the deflection.</summary>
    public void SetDeflection(double theDefl) => myDeflection = theDefl;

    /// <summary>The deflection (0 if never set).</summary>
    public double Deflection() => myDeflection;

    /// <summary>
    /// Sets a node at index <paramref name="iN"/>, or adds it at the end if <paramref name="iN"/> is negative, and
    /// returns its index. Setting beyond the end adds free nodes up to it.
    /// </summary>
    public int SetNode(in gp_XYZ thePnt, int iN = -1)
    {
        int aResult = myNodes.Length();
        if (iN < 0)
        {
            myNodes.Append(new Poly_CoherentNode(thePnt));
        }
        else
        {
            // NCollection_DynamicArray::SetValue default-constructs the elements up to iN
            while (myNodes.Length() < iN)
            {
                myNodes.Append(new Poly_CoherentNode());
            }

            myNodes.SetValue(iN, new Poly_CoherentNode(thePnt));
            aResult = iN;
        }

        return aResult;
    }

    /// <summary>The node at an index.</summary>
    public Poly_CoherentNode Node(int i) => myNodes.Value(i);

    /// <summary>The node at an index, for writing.</summary>
    public Poly_CoherentNode ChangeNode(int i) => myNodes.Value(i);

    /// <summary>The number of nodes used by triangles.</summary>
    public int NNodes()
    {
        int aCount = 0;
        foreach (Poly_CoherentNode aNode in myNodes)
        {
            if (!aNode.IsFreeNode())
            {
                aCount++;
            }
        }

        return aCount;
    }

    /// <summary>The triangle at an index.</summary>
    public Poly_CoherentTriangle Triangle(int i) => myTriangles.Value(i);

    /// <summary>The number of triangles not removed.</summary>
    public int NTriangles()
    {
        int aCount = 0;
        foreach (Poly_CoherentTriangle aTri in myTriangles)
        {
            if (!aTri.IsEmpty())
            {
                aCount++;
            }
        }

        return aCount;
    }

    /// <summary>The number of links not removed.</summary>
    public int NLinks()
    {
        int aCount = 0;
        foreach (Poly_CoherentLink aLink in myLinks)
        {
            if (!aLink.IsEmpty())
            {
                aCount++;
            }
        }

        return aCount;
    }

    /// <summary>Removes a triangle (it stays in the array, empty), updating its neighbours, nodes and links.</summary>
    public bool RemoveTriangle(Poly_CoherentTriangle theTriangle)
    {
        bool aResult = false;
        for (int i = 0; i < 3; i++)
        {
            if (theTriangle.Node(i) >= 0)
            {
                Poly_CoherentNode aNode = myNodes[theTriangle.Node(i)];
                // If Links exist in this Triangulation, remove or update a Link
                Poly_CoherentLink? aLink = theTriangle.mypLink[i];
                if (aLink is not null)
                {
                    Poly_CoherentTriangle? pTriOpp = theTriangle.GetConnectedTri(i);
                    bool toRemoveLink = true;
                    if (pTriOpp is not null)
                    {
                        // A neighbour is detected. If a Link exists on it, update it,
                        // otherwise remove this link
                        for (int j = 0; j < 3; j++)
                        {
                            if (aLink == pTriOpp.GetLink(j))
                            {
                                if (aLink.OppositeNode(0) == theTriangle.Node(i))
                                {
                                    aLink.myOppositeNode[0] = -1;
                                    toRemoveLink = false;
                                }
                                else if (aLink.OppositeNode(1) == theTriangle.Node(i))
                                {
                                    aLink.myOppositeNode[1] = -1;
                                    toRemoveLink = false;
                                }

                                break;
                            }
                        }
                    }

                    if (toRemoveLink)
                    {
                        RemoveLink(aLink);
                    }
                }

                if (aNode.RemoveTriangle(theTriangle))
                {
                    theTriangle.myNodes[i] = -1;
                    aResult = true;
                }
            }

            theTriangle.RemoveConnection(i);
        }

        return aResult;
    }

    /// <summary>Removes a link (it stays in the array, empty), clearing the references to it in its triangles.</summary>
    public void RemoveLink(Poly_CoherentLink theLink)
    {
        var pTri = new Poly_CoherentTriangle?[2];
        if (FindTriangle(theLink, pTri))
        {
            for (int i = 0; i < 2; i++)
            {
                int iNode = theLink.OppositeNode(i);
                Poly_CoherentTriangle? aTri = pTri[i];
                if (iNode >= 0 && aTri is not null)
                {
                    if (iNode == aTri.Node(0))
                    {
                        aTri.mypLink[0] = null;
                    }
                    else if (iNode == aTri.Node(1))
                    {
                        aTri.mypLink[1] = null;
                    }
                    else if (iNode == aTri.Node(2))
                    {
                        aTri.mypLink[2] = null;
                    }
                    else
                    {
                        throw new Standard_ProgramError("Poly_CoherentTriangulation::RemoveLink:  wrong connectivity between triangles");
                    }
                }
            }
        }

        theLink.Assign(new Poly_CoherentLink());
    }

    /// <summary>Adds a triangle of three nodes, connecting it to its neighbours; null if a node index is negative.</summary>
    public Poly_CoherentTriangle? AddTriangle(int iNode0, int iNode1, int iNode2)
    {
        Poly_CoherentTriangle? pTriangle = null;
        if (iNode0 >= 0 && iNode1 >= 0 && iNode2 >= 0)
        {
            pTriangle = myTriangles.Append(new Poly_CoherentTriangle());
            ReplaceNodes(pTriangle, iNode0, iNode1, iNode2);
        }

        return pTriangle;
    }

    /// <summary>Replaces the nodes of a triangle, updating its connections and links; false if a node index is negative.</summary>
    public bool ReplaceNodes(Poly_CoherentTriangle theTriangle, int iNode0, int iNode1, int iNode2)
    {
        if (!theTriangle.IsEmpty())
        {
            RemoveTriangle(theTriangle);
        }

        if (iNode0 >= 0 && iNode1 >= 0 && iNode2 >= 0)
        {
            theTriangle.Assign(new Poly_CoherentTriangle(iNode0, iNode1, iNode2));
            for (int i = 0; i < 3; i++)
            {
                Poly_CoherentNode aNode = myNodes[theTriangle.Node(i)];
                Poly_CoherentTriPtr.Iterator anIterT = aNode.TriangleIterator();
                for (; anIterT.More(); anIterT.Next())
                {
                    anIterT.ChangeValue().SetConnection(theTriangle);
                }

                aNode.AddTriangle(theTriangle);
            }

            // If Links exist in this Triangulation, create or update a Link
            if (myLinks.Length() > 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    Poly_CoherentTriangle? pTriOpp = theTriangle.GetConnectedTri(i);
                    bool toAddLink = true;
                    if (pTriOpp is not null)
                    {
                        // A neighbour is detected. If a Link exists on it, update it,
                        // otherwise create a new link.
                        for (int j = 0; j < 3; j++)
                        {
                            if (theTriangle.Node(i) == pTriOpp.GetConnectedNode(j))
                            {
                                Poly_CoherentLink? aLink = pTriOpp.GetLink(j);
                                if (aLink is not null)
                                {
                                    if (aLink.OppositeNode(0) == pTriOpp.Node(j))
                                    {
                                        aLink.myOppositeNode[1] = theTriangle.Node(i);
                                        toAddLink = false;
                                    }
                                    else if (aLink.OppositeNode(1) == pTriOpp.Node(j))
                                    {
                                        aLink.myOppositeNode[0] = theTriangle.Node(i);
                                        toAddLink = false;
                                    }
                                }
                            }
                        }
                    }

                    if (toAddLink)
                    {
                        // No neighbor on this side, the new Link is created.
                        AddLink(theTriangle, i);
                    }
                }
            }

            return true;
        }

        return false;
    }

    /// <summary>
    /// Adds the link on side <paramref name="theConn"/> of a triangle (without checking for an existing one), and sets
    /// it in the triangle and its neighbour on that side. Null if the triangle is empty.
    /// </summary>
    public Poly_CoherentLink? AddLink(Poly_CoherentTriangle theTri, int theConn)
    {
        Poly_CoherentLink? pLink = null;
        if (!theTri.IsEmpty())
        {
            pLink = myLinks.Append(new Poly_CoherentLink(theTri, theConn));
            theTri.mypLink[theConn] = pLink;
            Poly_CoherentTriangle? pTriOpp = theTri.GetConnectedTri(theConn);
            if (pTriOpp is null)
            {
                return pLink;
            }

            if (pTriOpp.IsEmpty())
            {
                return pLink;
            }

            if (pTriOpp.Node(0) == theTri.GetConnectedNode(theConn))
            {
                pTriOpp.mypLink[0] = pLink;
            }
            else if (pTriOpp.Node(1) == theTri.GetConnectedNode(theConn))
            {
                pTriOpp.mypLink[1] = pLink;
            }
            else if (pTriOpp.Node(2) == theTri.GetConnectedNode(theConn))
            {
                pTriOpp.mypLink[2] = pLink;
            }
            else
            {
                throw new Standard_ProgramError("Poly_CoherentTriangulation::AddLink: Bad connectivity of triangles");
            }
        }

        return pLink;
    }

    /// <summary>
    /// The triangles sharing the two nodes of a link: <c>pTri[0]</c> on its left, <c>pTri[1]</c> on its right (an array
    /// of two). True if at least one is found.
    /// </summary>
    public bool FindTriangle(Poly_CoherentLink theLink, Poly_CoherentTriangle?[] pTri)
    {
        pTri[0] = null;
        pTri[1] = null;
        int iNode0 = theLink.Node(0);
        if (!theLink.IsEmpty() && iNode0 < myNodes.Length() && theLink.Node(1) < myNodes.Length())
        {
            Poly_CoherentTriPtr.Iterator anIter0 = myNodes[iNode0].TriangleIterator();
            for (; anIter0.More(); anIter0.Next())
            {
                Poly_CoherentTriangle aTri = anIter0.Value();
                if (aTri.Node(0) == iNode0)
                {
                    if (aTri.Node(1) == theLink.Node(1))
                    {
                        pTri[0] = aTri;
                    }
                    else if (aTri.Node(2) == theLink.Node(1))
                    {
                        pTri[1] = aTri;
                    }
                }
                else if (aTri.Node(1) == iNode0)
                {
                    if (aTri.Node(2) == theLink.Node(1))
                    {
                        pTri[0] = aTri;
                    }
                    else if (aTri.Node(0) == theLink.Node(1))
                    {
                        pTri[1] = aTri;
                    }
                }
                else if (aTri.Node(2) == iNode0)
                {
                    if (aTri.Node(0) == theLink.Node(1))
                    {
                        pTri[0] = aTri;
                    }
                    else if (aTri.Node(1) == theLink.Node(1))
                    {
                        pTri[1] = aTri;
                    }
                }
                else
                {
                    throw new Standard_ProgramError("Poly_CoherentTriangulation::FindTriangle :  Data incoherence detected");
                }

                if (pTri[0] is not null && pTri[1] is not null)
                {
                    break;
                }
            }
        }

        return pTri[0] is not null || pTri[1] is not null;
    }

    /// <summary>(Re)computes all the links; returns their number.</summary>
    /// <remarks>
    /// As in OCCT, the triangles' references to the previous links aren't cleared first (call <see cref="ClearLinks"/>
    /// before recomputing): sides that had a link keep it and get no new one.
    /// </remarks>
    public int ComputeLinks()
    {
        myLinks.Clear(); // OCCT: the triangles keep (dangling) pointers to the cleared links
        int aLast = myTriangles.Length();
        for (int anIter = 0; anIter < aLast; anIter++)
        {
            Poly_CoherentTriangle aTriangle = myTriangles[anIter];
            if (aTriangle.IsEmpty())
            {
                continue;
            }

            if (aTriangle.Node(0) < aTriangle.Node(1))
            {
                AddLink(aTriangle, 2);
            }

            if (aTriangle.Node(1) < aTriangle.Node(2))
            {
                AddLink(aTriangle, 0);
            }

            if (aTriangle.Node(2) < aTriangle.Node(0))
            {
                AddLink(aTriangle, 1);
            }
        }

        // Above algorithm does not create all boundary links, so
        // it is necessary to check triangles and add absentee links
        aLast = myTriangles.Length();
        int i;
        for (int anIter = 0; anIter < aLast; anIter++)
        {
            Poly_CoherentTriangle aTriangle = myTriangles[anIter];
            if (aTriangle.IsEmpty())
            {
                continue;
            }

            for (i = 0; i < 3; ++i)
            {
                if (aTriangle.mypLink[i] is null)
                {
                    AddLink(aTriangle, i);
                }
            }
        }

        return myLinks.Length();
    }

    /// <summary>Removes all the links, and the triangles' references to them.</summary>
    public void ClearLinks()
    {
        myLinks.Clear();
        foreach (Poly_CoherentTriangle aTriangle in myTriangles)
        {
            aTriangle.mypLink[0] = null;
            aTriangle.mypLink[1] = null;
            aTriangle.mypLink[2] = null;
        }
    }

    /// <summary>
    /// A copy of this triangulation (through <see cref="GetTriangulation"/>: free nodes and removed triangles are
    /// dropped, and links aren't copied); null if it has no triangles or nodes.
    /// </summary>
    public Poly_CoherentTriangulation? Clone()
    {
        Poly_CoherentTriangulation? newTri = null;

        if (NTriangles() != 0 && NNodes() != 0)
        {
            Poly_Triangulation theTriangulation = GetTriangulation()!;
            newTri = new Poly_CoherentTriangulation(theTriangulation);
            newTri.SetDeflection(theTriangulation.Deflection());
        }

        return newTri;
    }

    /// <summary>Writes the non-free nodes and their triangles (debugging output).</summary>
    public void Dump(TextWriter theStream)
    {
        for (int iNode = 0; iNode < myNodes.Length(); iNode++)
        {
            Poly_CoherentNode aNode = myNodes[iNode];
            if (aNode.IsFreeNode())
            {
                continue;
            }

            theStream.Write("Node " + iNode.ToString(CultureInfo.InvariantCulture));
            aNode.Dump(theStream);
        }
    }
}
