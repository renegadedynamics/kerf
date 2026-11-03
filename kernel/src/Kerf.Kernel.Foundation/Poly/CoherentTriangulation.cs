// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangulation.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A triangulation for algorithms that analyse or edit a mesh (refining, decimation): each triangle knows its up to
/// three neighbours and their third nodes, each node the triangles using it, and links between nodes can be computed.
/// Nodes and triangles can be added, triangles removed, and the result converted back to a
/// <see cref="Triangulation"/>.
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
public class CoherentTriangulation : Transient
{
    /// <summary>Iterates the triangles, skipping removed ones. Triangles added during the iteration aren't visited.</summary>
    public sealed class IteratorOfTriangle
    {
        private NCollection_DynamicArray<CoherentTriangle>? _array;
        private int _index;
        private int _last;

        public IteratorOfTriangle(CoherentTriangulation? tri)
        {
            if (tri is not null)
            {
                _array = tri._triangles;
                _index = 0;
                _last = _array.Length();
                while (More())
                {
                    CoherentTriangle aTri = Value();
                    if (!aTri.IsEmpty)
                    {
                        break;
                    }

                    _index++;
                }
            }
        }

        public bool More() => _array is not null && _index < _last;

        public void Next()
        {
            _index++;
            while (More())
            {
                CoherentTriangle tri = Value();
                if (!tri.IsEmpty)
                {
                    break;
                }

                _index++;
            }
        }

        public CoherentTriangle Value() => _array!.Value(_index);

        public CoherentTriangle ChangeValue() => _array!.Value(_index);
    }

    /// <summary>Iterates the nodes, skipping free ones.</summary>
    public sealed class IteratorOfNode
    {
        private NCollection_DynamicArray<CoherentNode>? _array;
        private int _index;
        private int _last;

        public IteratorOfNode(CoherentTriangulation? tri)
        {
            if (tri is not null)
            {
                _array = tri._nodes;
                _index = 0;
                _last = _array.Length();
                while (More())
                {
                    if (!Value().IsFreeNode)
                    {
                        break;
                    }

                    _index++;
                }
            }
        }

        public bool More() => _array is not null && _index < _last;

        public void Next()
        {
            _index++;
            while (More())
            {
                if (!Value().IsFreeNode)
                {
                    break;
                }

                _index++;
            }
        }

        public CoherentNode Value() => _array!.Value(_index);

        public CoherentNode ChangeValue() => _array!.Value(_index);

        /// <summary>The index of the current node.</summary>
        public int Index() => _index;
    }

    /// <summary>Iterates the links, skipping removed ones.</summary>
    public sealed class IteratorOfLink
    {
        private NCollection_DynamicArray<CoherentLink>? _array;
        private int _index;
        private int _last;

        public IteratorOfLink(CoherentTriangulation? tri)
        {
            if (tri is not null)
            {
                _array = tri._links;
                _index = 0;
                _last = _array.Length();
                while (More())
                {
                    if (!Value().IsEmpty)
                    {
                        break;
                    }

                    _index++;
                }
            }
        }

        public bool More() => _array is not null && _index < _last;

        public void Next()
        {
            _index++;
            while (More())
            {
                if (!Value().IsEmpty)
                {
                    break;
                }

                _index++;
            }
        }

        public CoherentLink Value() => _array!.Value(_index);

        public CoherentLink ChangeValue() => _array!.Value(_index);
    }

    /// <summary>A pair of node indices (used by <see cref="RemoveDegenerated"/>: the removed node, then the one kept).</summary>
    public readonly record struct TwoIntegers(int myValue0, int myValue1);

    protected readonly NCollection_DynamicArray<CoherentTriangle> _triangles = new();
    protected readonly NCollection_DynamicArray<CoherentNode> _nodes = new();
    protected readonly NCollection_DynamicArray<CoherentLink> _links = new();
    protected double _deflection;

    /// <summary>An empty triangulation.</summary>
    public CoherentTriangulation()
    {
        _deflection = 0.0;
    }

    /// <summary>
    /// The coherent form of a triangulation: its nodes (indexed from 0, each keeping its original index), its
    /// non-degenerate triangles, UV nodes and normals. Links aren't computed (<see cref="ComputeLinks"/>).
    /// </summary>
    public CoherentTriangulation(Triangulation? triangulation)
    {
        // OCCT leaves the deflection uninitialised for a null triangulation
        if (triangulation is not null)
        {
            int nNodes = triangulation.NodeCount;
            int i;
            // Copy the nodes
            for (i = 0; i < nNodes; i++)
            {
                int oldInd = i + 1;
                int newInd = SetNode(triangulation.Node(oldInd).XYZ(), i);
                CoherentNode copiedNode = _nodes[newInd];
                copiedNode.SetIndex(oldInd);
            }

            // Copy the triangles
            Span<int> iNode = [0, 0, 0];
            for (i = 1; i <= triangulation.TriangleCount; i++)
            {
                triangulation.Triangle(i).Get(out iNode[0], out iNode[1], out iNode[2]);
                if (iNode[0] != iNode[1] && iNode[1] != iNode[2] && iNode[2] != iNode[0])
                {
                    AddTriangle(iNode[0] - 1, iNode[1] - 1, iNode[2] - 1);
                }
            }

            // Copy UV coordinates of nodes
            if (triangulation.HasUVNodes)
            {
                for (i = 0; i < nNodes; i++)
                {
                    Point2D uv = triangulation.UVNode(i + 1);
                    _nodes[i].SetUV(uv.X(), uv.Y());
                }
            }

            // Copy the normals at nodes
            if (triangulation.HasNormals)
            {
                for (i = 0; i < nNodes; i++)
                {
                    triangulation.Normal(i + 1, out NCollection_Vec3<float> normal);
                    _nodes[i].SetNormal(new Xyz(normal.X(), normal.Y(), normal.Z()));
                }
            }

            _deflection = triangulation.Deflection();
        }
    }

    /// <summary>
    /// A <see cref="Triangulation"/> of the non-free nodes and the triangles, with UV nodes and normals where the
    /// nodes have them; null if there are no nodes or no triangles.
    /// </summary>
    public Triangulation? GetTriangulation()
    {
        int nNodes = NNodes();
        int nTriangles = NTriangles();
        if (nNodes == 0 || nTriangles == 0)
        {
            return null;
        }

        var result = new Triangulation(nNodes, nTriangles, false);

        var vecNodeId = new NCollection_DynamicArray<int>();
        int count = 0;

        // Copy the nodes (3D and 2D coordinates)
        for (int i = 0; i < _nodes.Length(); i++)
        {
            CoherentNode node = _nodes[i];
            if (node.IsFreeNode)
            {
                vecNodeId.SetValue(i, 0);
                continue;
            }

            vecNodeId.SetValue(i, ++count);
            Xyz normal = node.GetNormal();
            if (normal.SquareModulus() > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                result.AddNormals();
                result.SetNormal(count, new Direction3D(normal));
            }

            result.SetNode(count, new Point3D(node.XYZ()));
            if (node.GetU() * node.GetU() + node.GetV() * node.GetV() > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                result.AddUVNodes();
                result.SetUVNode(count, new Point2D(node.GetU(), node.GetV()));
            }
        }

        // Copy the triangles
        count = 0;
        for (int iterT = 0, last = _triangles.Length(); iterT < last; iterT++)
        {
            CoherentTriangle tri = _triangles[iterT];
            if (!tri.IsEmpty)
            {
                result.SetTriangle(++count,
                                    new Triangle1(vecNodeId[tri.Node(0)], vecNodeId[tri.Node(1)], vecNodeId[tri.Node(2)]));
            }
        }

        result.Deflection(_deflection);
        return result;
    }

    /// <summary>
    /// Removes the degenerate triangles: those with two nodes closer than <paramref name="tol"/>. The triangles
    /// around one of the two nodes are reconnected to the other; <paramref name="pLstRemovedNode"/>, if given, receives
    /// each pair (removed node, node kept). True if a triangle was removed.
    /// </summary>
    public bool RemoveDegenerated(double tol, NCollection_List<TwoIntegers>? pLstRemovedNode = null)
    {
        bool result = false;
        double tol2 = tol * tol;
        ReadOnlySpan<int> ind0 = [2, 0, 1, 2, 0];
        const int ind = 1; // ind = &ind0[1]

        pLstRemovedNode?.Clear();

        var iterT = new IteratorOfTriangle(this);
        for (; iterT.More(); iterT.Next())
        {
            CoherentTriangle tri = iterT.ChangeValue();
            NCollection_Buffer3<CoherentNode> pNode = default;
            pNode[0] = ChangeNode(tri.Node(0));
            pNode[1] = ChangeNode(tri.Node(1));
            pNode[2] = ChangeNode(tri.Node(2));
            ReadOnlySpan<double> len2 =
            [
                pNode[2].XYZ().Subtracted(pNode[1].XYZ()).SquareModulus(),
                pNode[0].XYZ().Subtracted(pNode[2].XYZ()).SquareModulus(),
                pNode[1].XYZ().Subtracted(pNode[0].XYZ()).SquareModulus(),
            ];
            for (int i = 0; i < 3; i++)
            {
                if (len2[i] < tol2)
                {
                    int im1 = tri.Node(ind0[ind + i - 1]);
                    int ip1 = tri.Node(ind0[ind + i + 1]);

                    // Disconnect from both neighbours
                    RemoveTriangle(tri);

                    // Reconnect all triangles from Node(ind[i+1]) to Node(ind[i-1])
                    for (;;)
                    {
                        CoherentTriPtr.Iterator iterConn = pNode[ind0[ind + i + 1]].TriangleIterator();
                        if (!iterConn.More())
                        {
                            break;
                        }

                        CoherentTriangle triConn = iterConn.ChangeValue();
                        Span<int> newTriConn = [triConn.Node(0), triConn.Node(1), triConn.Node(2)];
                        if (triConn != tri)
                        {
                            if (newTriConn[0] == ip1)
                            {
                                newTriConn[0] = im1;
                            }
                            else if (newTriConn[1] == ip1)
                            {
                                newTriConn[1] = im1;
                            }
                            else if (newTriConn[2] == ip1)
                            {
                                newTriConn[2] = im1;
                            }

                            RemoveTriangle(triConn);
                            AddTriangle(newTriConn[0], newTriConn[1], newTriConn[2]);
                        }
                        else
                        {
                            iterConn.Next();
                            if (!iterConn.More())
                            {
                                break;
                            }
                        }
                    }

                    pLstRemovedNode?.Append(new TwoIntegers(ip1, im1));
                    result = true;
                    break;
                }
            }
        }

        return result;
    }

    /// <summary>The indices of the free nodes (used by no triangle) in <paramref name="lstNodes"/>; true if there are any.</summary>
    public bool GetFreeNodes(NCollection_List<int> lstNodes)
    {
        lstNodes.Clear();
        int i;
        for (i = 0; i < _nodes.Length(); i++)
        {
            CoherentNode node = _nodes[i];
            if (node.IsFreeNode)
            {
                lstNodes.Append(i);
            }
        }

        return !lstNodes.IsEmpty();
    }

    /// <summary>The index of the last node.</summary>
    public int MaxNode() => _nodes.Length() - 1;

    /// <summary>The index of the last triangle.</summary>
    public int MaxTriangle() => _triangles.Length() - 1;

    /// <summary>Sets the deflection.</summary>
    public void SetDeflection(double defl) => _deflection = defl;

    /// <summary>The deflection (0 if never set).</summary>
    public double Deflection() => _deflection;

    /// <summary>
    /// Sets a node at index <paramref name="iN"/>, or adds it at the end if <paramref name="iN"/> is negative, and
    /// returns its index. Setting beyond the end adds free nodes up to it.
    /// </summary>
    public int SetNode(in Xyz pnt, int iN = -1)
    {
        int result = _nodes.Length();
        if (iN < 0)
        {
            _nodes.Append(new CoherentNode(pnt));
        }
        else
        {
            // NCollection_DynamicArray::SetValue default-constructs the elements up to iN
            while (_nodes.Length() < iN)
            {
                _nodes.Append(new CoherentNode());
            }

            _nodes.SetValue(iN, new CoherentNode(pnt));
            result = iN;
        }

        return result;
    }

    /// <summary>The node at an index.</summary>
    public CoherentNode Node(int i) => _nodes.Value(i);

    /// <summary>The node at an index, for writing.</summary>
    public CoherentNode ChangeNode(int i) => _nodes.Value(i);

    /// <summary>The number of nodes used by triangles.</summary>
    public int NNodes()
    {
        int count = 0;
        foreach (CoherentNode node in _nodes)
        {
            if (!node.IsFreeNode)
            {
                count++;
            }
        }

        return count;
    }

    /// <summary>The triangle at an index.</summary>
    public CoherentTriangle Triangle(int i) => _triangles.Value(i);

    /// <summary>The number of triangles not removed.</summary>
    public int NTriangles()
    {
        int count = 0;
        foreach (CoherentTriangle tri in _triangles)
        {
            if (!tri.IsEmpty)
            {
                count++;
            }
        }

        return count;
    }

    /// <summary>The number of links not removed.</summary>
    public int NLinks()
    {
        int count = 0;
        foreach (CoherentLink link in _links)
        {
            if (!link.IsEmpty)
            {
                count++;
            }
        }

        return count;
    }

    /// <summary>Removes a triangle (it stays in the array, empty), updating its neighbours, nodes and links.</summary>
    public bool RemoveTriangle(CoherentTriangle triangle)
    {
        bool result = false;
        for (int i = 0; i < 3; i++)
        {
            if (triangle.Node(i) >= 0)
            {
                CoherentNode node = _nodes[triangle.Node(i)];
                // If Links exist in this Triangulation, remove or update a Link
                CoherentLink? link = triangle._mypLink[i];
                if (link is not null)
                {
                    CoherentTriangle? pTriOpp = triangle.GetConnectedTri(i);
                    bool toRemoveLink = true;
                    if (pTriOpp is not null)
                    {
                        // A neighbour is detected. If a Link exists on it, update it,
                        // otherwise remove this link
                        for (int j = 0; j < 3; j++)
                        {
                            if (link == pTriOpp.GetLink(j))
                            {
                                if (link.OppositeNode(0) == triangle.Node(i))
                                {
                                    link._oppositeNode[0] = -1;
                                    toRemoveLink = false;
                                }
                                else if (link.OppositeNode(1) == triangle.Node(i))
                                {
                                    link._oppositeNode[1] = -1;
                                    toRemoveLink = false;
                                }

                                break;
                            }
                        }
                    }

                    if (toRemoveLink)
                    {
                        RemoveLink(link);
                    }
                }

                if (node.RemoveTriangle(triangle))
                {
                    triangle._nodes[i] = -1;
                    result = true;
                }
            }

            triangle.RemoveConnection(i);
        }

        return result;
    }

    /// <summary>Removes a link (it stays in the array, empty), clearing the references to it in its triangles.</summary>
    public void RemoveLink(CoherentLink link)
    {
        var pTri = new CoherentTriangle?[2];
        if (FindTriangle(link, pTri))
        {
            for (int i = 0; i < 2; i++)
            {
                int iNode = link.OppositeNode(i);
                CoherentTriangle? tri = pTri[i];
                if (iNode >= 0 && tri is not null)
                {
                    if (iNode == tri.Node(0))
                    {
                        tri._mypLink[0] = null;
                    }
                    else if (iNode == tri.Node(1))
                    {
                        tri._mypLink[1] = null;
                    }
                    else if (iNode == tri.Node(2))
                    {
                        tri._mypLink[2] = null;
                    }
                    else
                    {
                        throw new ProgramErrorException("Poly_CoherentTriangulation::RemoveLink:  wrong connectivity between triangles");
                    }
                }
            }
        }

        link.Assign(new CoherentLink());
    }

    /// <summary>Adds a triangle of three nodes, connecting it to its neighbours; null if a node index is negative.</summary>
    public CoherentTriangle? AddTriangle(int iNode0, int iNode1, int iNode2)
    {
        CoherentTriangle? pTriangle = null;
        if (iNode0 >= 0 && iNode1 >= 0 && iNode2 >= 0)
        {
            pTriangle = _triangles.Append(new CoherentTriangle());
            ReplaceNodes(pTriangle, iNode0, iNode1, iNode2);
        }

        return pTriangle;
    }

    /// <summary>Replaces the nodes of a triangle, updating its connections and links; false if a node index is negative.</summary>
    public bool ReplaceNodes(CoherentTriangle triangle, int iNode0, int iNode1, int iNode2)
    {
        if (!triangle.IsEmpty)
        {
            RemoveTriangle(triangle);
        }

        if (iNode0 >= 0 && iNode1 >= 0 && iNode2 >= 0)
        {
            triangle.Assign(new CoherentTriangle(iNode0, iNode1, iNode2));
            for (int i = 0; i < 3; i++)
            {
                CoherentNode node = _nodes[triangle.Node(i)];
                CoherentTriPtr.Iterator iterT = node.TriangleIterator();
                for (; iterT.More(); iterT.Next())
                {
                    iterT.ChangeValue().SetConnection(triangle);
                }

                node.AddTriangle(triangle);
            }

            // If Links exist in this Triangulation, create or update a Link
            if (_links.Length() > 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    CoherentTriangle? pTriOpp = triangle.GetConnectedTri(i);
                    bool toAddLink = true;
                    if (pTriOpp is not null)
                    {
                        // A neighbour is detected. If a Link exists on it, update it,
                        // otherwise create a new link.
                        for (int j = 0; j < 3; j++)
                        {
                            if (triangle.Node(i) == pTriOpp.GetConnectedNode(j))
                            {
                                CoherentLink? link = pTriOpp.GetLink(j);
                                if (link is not null)
                                {
                                    if (link.OppositeNode(0) == pTriOpp.Node(j))
                                    {
                                        link._oppositeNode[1] = triangle.Node(i);
                                        toAddLink = false;
                                    }
                                    else if (link.OppositeNode(1) == pTriOpp.Node(j))
                                    {
                                        link._oppositeNode[0] = triangle.Node(i);
                                        toAddLink = false;
                                    }
                                }
                            }
                        }
                    }

                    if (toAddLink)
                    {
                        // No neighbor on this side, the new Link is created.
                        AddLink(triangle, i);
                    }
                }
            }

            return true;
        }

        return false;
    }

    /// <summary>
    /// Adds the link on side <paramref name="conn"/> of a triangle (without checking for an existing one), and sets
    /// it in the triangle and its neighbour on that side. Null if the triangle is empty.
    /// </summary>
    public CoherentLink? AddLink(CoherentTriangle tri, int conn)
    {
        CoherentLink? pLink = null;
        if (!tri.IsEmpty)
        {
            pLink = _links.Append(new CoherentLink(tri, conn));
            tri._mypLink[conn] = pLink;
            CoherentTriangle? pTriOpp = tri.GetConnectedTri(conn);
            if (pTriOpp is null)
            {
                return pLink;
            }

            if (pTriOpp.IsEmpty)
            {
                return pLink;
            }

            if (pTriOpp.Node(0) == tri.GetConnectedNode(conn))
            {
                pTriOpp._mypLink[0] = pLink;
            }
            else if (pTriOpp.Node(1) == tri.GetConnectedNode(conn))
            {
                pTriOpp._mypLink[1] = pLink;
            }
            else if (pTriOpp.Node(2) == tri.GetConnectedNode(conn))
            {
                pTriOpp._mypLink[2] = pLink;
            }
            else
            {
                throw new ProgramErrorException("Poly_CoherentTriangulation::AddLink: Bad connectivity of triangles");
            }
        }

        return pLink;
    }

    /// <summary>
    /// The triangles sharing the two nodes of a link: <c>pTri[0]</c> on its left, <c>pTri[1]</c> on its right (an array
    /// of two). True if at least one is found.
    /// </summary>
    public bool FindTriangle(CoherentLink link, CoherentTriangle?[] pTri)
    {
        pTri[0] = null;
        pTri[1] = null;
        int iNode0 = link.Node(0);
        if (!link.IsEmpty && iNode0 < _nodes.Length() && link.Node(1) < _nodes.Length())
        {
            CoherentTriPtr.Iterator iter0 = _nodes[iNode0].TriangleIterator();
            for (; iter0.More(); iter0.Next())
            {
                CoherentTriangle tri = iter0.Value();
                if (tri.Node(0) == iNode0)
                {
                    if (tri.Node(1) == link.Node(1))
                    {
                        pTri[0] = tri;
                    }
                    else if (tri.Node(2) == link.Node(1))
                    {
                        pTri[1] = tri;
                    }
                }
                else if (tri.Node(1) == iNode0)
                {
                    if (tri.Node(2) == link.Node(1))
                    {
                        pTri[0] = tri;
                    }
                    else if (tri.Node(0) == link.Node(1))
                    {
                        pTri[1] = tri;
                    }
                }
                else if (tri.Node(2) == iNode0)
                {
                    if (tri.Node(0) == link.Node(1))
                    {
                        pTri[0] = tri;
                    }
                    else if (tri.Node(1) == link.Node(1))
                    {
                        pTri[1] = tri;
                    }
                }
                else
                {
                    throw new ProgramErrorException("Poly_CoherentTriangulation::FindTriangle :  Data incoherence detected");
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
        _links.Clear(); // OCCT: the triangles keep (dangling) pointers to the cleared links
        int last = _triangles.Length();
        for (int iter = 0; iter < last; iter++)
        {
            CoherentTriangle triangle = _triangles[iter];
            if (triangle.IsEmpty)
            {
                continue;
            }

            if (triangle.Node(0) < triangle.Node(1))
            {
                AddLink(triangle, 2);
            }

            if (triangle.Node(1) < triangle.Node(2))
            {
                AddLink(triangle, 0);
            }

            if (triangle.Node(2) < triangle.Node(0))
            {
                AddLink(triangle, 1);
            }
        }

        // Above algorithm does not create all boundary links, so
        // it is necessary to check triangles and add absentee links
        last = _triangles.Length();
        int i;
        for (int iter = 0; iter < last; iter++)
        {
            CoherentTriangle triangle = _triangles[iter];
            if (triangle.IsEmpty)
            {
                continue;
            }

            for (i = 0; i < 3; ++i)
            {
                if (triangle._mypLink[i] is null)
                {
                    AddLink(triangle, i);
                }
            }
        }

        return _links.Length();
    }

    /// <summary>Removes all the links, and the triangles' references to them.</summary>
    public void ClearLinks()
    {
        _links.Clear();
        foreach (CoherentTriangle triangle in _triangles)
        {
            triangle._mypLink[0] = null;
            triangle._mypLink[1] = null;
            triangle._mypLink[2] = null;
        }
    }

    /// <summary>
    /// A copy of this triangulation (through <see cref="GetTriangulation"/>: free nodes and removed triangles are
    /// dropped, and links aren't copied); null if it has no triangles or nodes.
    /// </summary>
    public CoherentTriangulation? Clone()
    {
        CoherentTriangulation? newTri = null;

        if (NTriangles() != 0 && NNodes() != 0)
        {
            Triangulation theTriangulation = GetTriangulation()!;
            newTri = new CoherentTriangulation(theTriangulation);
            newTri.SetDeflection(theTriangulation.Deflection());
        }

        return newTri;
    }

    /// <summary>Writes the non-free nodes and their triangles (debugging output).</summary>
    public void Dump(TextWriter stream)
    {
        for (int iNode = 0; iNode < _nodes.Length(); iNode++)
        {
            CoherentNode node = _nodes[iNode];
            if (node.IsFreeNode)
            {
                continue;
            }

            stream.Write("Node " + iNode.ToString(CultureInfo.InvariantCulture));
            node.Dump(stream);
        }
    }
}
