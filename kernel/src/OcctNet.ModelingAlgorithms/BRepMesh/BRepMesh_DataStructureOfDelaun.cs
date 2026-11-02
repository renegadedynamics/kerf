// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DataStructureOfDelaun.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DataStructureOfDelaun.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using static Occt.BRepMesh_DegreeOfFreedom;

namespace Occt;

/// <summary>
/// The mesh of the Delaunay triangulation in the parametric space: nodes (<see cref="BRepMesh_Vertex"/>), links
/// (<see cref="BRepMesh_Edge"/>) with the triangles connected to each, and triangles (<see cref="BRepMesh_Triangle"/>),
/// all indexed from 1. Deleted links and nodes keep their indices until <see cref="ClearDeleted"/> renumbers.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (decision 21; the allocator argument is dropped). Two containers keep OCCT's node
/// semantics because the Delaunay algorithm depends on them (decision 541):
/// <list type="bullet">
///   <item>the links connected to a node are a singly linked list (<see cref="LinkedListOfInteger"/>), as OCCT's
///   NCollection_List over an incremental allocator: removing a link while another loop walks the same list leaves
///   that loop where OCCT's continues;</item>
///   <item>the links are an indexed map with OCCT's equality (either orientation, never a deleted link) whose keys and
///   values stay at the same place, so <see cref="ElementsConnectedTo"/> returns a reference that sees later changes,
///   as the C++ <c>const BRepMesh_PairOfIndex&amp;</c> does.</item>
/// </list>
/// </remarks>
public sealed class BRepMesh_DataStructureOfDelaun : Standard_Transient
{
    private readonly BRepMesh_VertexTool myNodes;
    private readonly NCollection_DataMap<int, LinkedListOfInteger> myNodeLinks;
    private readonly LinkMap myLinks;
    // OCCT's NCollection_List<int>, used only as a first-in first-out queue: a .NET queue, whose RemoveFirst
    // doesn't move the rest (unit 9E)
    private readonly Queue<int> myDelLinks;
    private readonly NCollection_DynamicArray<BRepMesh_Triangle> myElements;
    private readonly NCollection_PackedMapOfInteger myElementsOfDomain;
    private readonly NCollection_PackedMapOfInteger myLinksOfDomain;

    /// <summary>An empty structure reserving room for about <paramref name="theReservedNodeSize"/> nodes.</summary>
    public BRepMesh_DataStructureOfDelaun(int theReservedNodeSize = 100)
    {
        myNodes = new BRepMesh_VertexTool();
        myNodeLinks = new NCollection_DataMap<int, LinkedListOfInteger>();
        myLinks = new LinkMap(theReservedNodeSize * 3);
        myDelLinks = new Queue<int>();
        myElements = new NCollection_DynamicArray<BRepMesh_Triangle>(theReservedNodeSize * 2);
        myElementsOfDomain = new NCollection_PackedMapOfInteger();
        myLinksOfDomain = new NCollection_PackedMapOfInteger();
    }

    // Nodes

    /// <summary>The number of nodes (deleted ones included).</summary>
    public int NbNodes() => myNodes.Extent();

    /// <summary>
    /// Adds a node, unless one coincides with it within the tolerance of <see cref="Data"/> (or
    /// <paramref name="isForceAdd"/>); returns the index of the node added or found.
    /// </summary>
    public int AddNode(in BRepMesh_Vertex theNode, bool isForceAdd = false)
    {
        int aNodeId = myNodes.Add(theNode, isForceAdd);
        if (!myNodeLinks.IsBound(aNodeId))
        {
            myNodeLinks.Bind(aNodeId, new LinkedListOfInteger());
        }

        return aNodeId;
    }

    /// <summary>The index of the node coinciding with the given one, or 0.</summary>
    public int IndexOf(in BRepMesh_Vertex theNode) => myNodes.FindIndex(theNode);

    /// <summary>The node of the given index.</summary>
    public BRepMesh_Vertex GetNode(int theIndex) => myNodes.FindKey(theIndex);

    /// <summary>The node of the given index (OCCT's <c>operator()</c>).</summary>
    public BRepMesh_Vertex this[int theIndex] => GetNode(theIndex);

    /// <summary>Replaces the node of the given index; false if another node coincides with the new one.</summary>
    public bool SubstituteNode(int theIndex, in BRepMesh_Vertex theNewNode)
    {
        if (myNodes.FindIndex(theNewNode) != 0)
        {
            return false;
        }

        myNodes.Substitute(theIndex, theNewNode);
        return true;
    }

    /// <summary>Removes a free node (or any, with <paramref name="isForce"/>) to which no link is connected.</summary>
    public void RemoveNode(int theIndex, bool isForce = false)
    {
        if (isForce || myNodes.FindKey(theIndex).Movability() == BRepMesh_Free)
        {
            if (LinksConnectedTo(theIndex).Extent() == 0)
            {
                myNodes.DeleteVertex(theIndex);
            }
        }
    }

    /// <summary>The indices of the links connected to the node (the structure's own list).</summary>
    public LinkedListOfInteger LinksConnectedTo(int theIndex) => linksConnectedTo(theIndex);

    // Links

    /// <summary>The number of links (deleted ones included).</summary>
    public int NbLinks() => myLinks.Extent();

    /// <summary>
    /// Adds a link, or finds the same link in either orientation; returns its index, negative if the existing link is
    /// reversed. A deleted link's index is reused.
    /// </summary>
    public int AddLink(in BRepMesh_Edge theLink)
    {
        int aLinkIndex = IndexOf(theLink);
        if (aLinkIndex > 0)
        {
            return theLink.IsSameOrientation(GetLink(aLinkIndex)) ? aLinkIndex : -aLinkIndex;
        }

        BRepMesh_PairOfIndex aPair = new();
        if (myDelLinks.Count != 0)
        {
            aLinkIndex = myDelLinks.Peek();
            myLinks.Substitute(aLinkIndex, theLink, aPair);
            myDelLinks.Dequeue();
        }
        else
        {
            aLinkIndex = myLinks.Add(theLink, aPair);
        }

        int aLinkId = Math.Abs(aLinkIndex);
        linksConnectedTo(theLink.FirstNode()).Append(aLinkId);
        linksConnectedTo(theLink.LastNode()).Append(aLinkId);
        myLinksOfDomain.Add(aLinkIndex);

        return aLinkIndex;
    }

    /// <summary>The index of the link joining the same nodes (either orientation), or 0.</summary>
    public int IndexOf(in BRepMesh_Edge theLink) => myLinks.FindIndex(theLink);

    /// <summary>The link of the given index.</summary>
    public BRepMesh_Edge GetLink(int theIndex) => myLinks.FindKey(theIndex);

    /// <summary>The link of the given index, for writing in place (OCCT's <c>const_cast</c> of <c>GetLink</c>).</summary>
    internal ref BRepMesh_Edge ChangeLink(int theIndex) => ref myLinks.FindKey(theIndex);

    /// <summary>The indices of the links of the domain.</summary>
    public NCollection_PackedMapOfInteger LinksOfDomain() => myLinksOfDomain;

    /// <summary>Replaces the link of the given index; false if the new link already exists.</summary>
    public bool SubstituteLink(int theIndex, in BRepMesh_Edge theNewLink)
    {
        BRepMesh_PairOfIndex aPair = new();
        BRepMesh_Edge aLink = GetLink(theIndex);
        if (aLink.Movability() == BRepMesh_Deleted)
        {
            myLinks.Substitute(theIndex, theNewLink, aPair);
            return true;
        }

        if (IndexOf(theNewLink) != 0)
        {
            return false;
        }

        aLink.SetMovability(BRepMesh_Deleted);
        myLinks.Substitute(theIndex, aLink, aPair);
        cleanLink(theIndex, aLink);

        int aLinkId = Math.Abs(theIndex);
        linksConnectedTo(theNewLink.FirstNode()).Append(aLinkId);
        linksConnectedTo(theNewLink.LastNode()).Append(aLinkId);
        myLinks.Substitute(theIndex, theNewLink, aPair);

        return true;
    }

    /// <summary>
    /// Removes a free link (or any, with <paramref name="isForce"/>) to which no triangle is connected: it is marked
    /// deleted and its index is reused by the next <see cref="AddLink"/>.
    /// </summary>
    public void RemoveLink(int theIndex, bool isForce = false)
    {
        ref BRepMesh_Edge aLink = ref myLinks.FindKey(theIndex);
        if (aLink.Movability() == BRepMesh_Deleted || (!isForce && aLink.Movability() != BRepMesh_Free)
            || ElementsConnectedTo(theIndex).Extent() != 0)
        {
            return;
        }

        cleanLink(theIndex, aLink);
        aLink.SetMovability(BRepMesh_Deleted);

        myLinksOfDomain.Remove(theIndex);
        myDelLinks.Enqueue(theIndex);
    }

    /// <summary>
    /// The triangles connected to the link of the given index: a reference into the structure, which sees later
    /// changes (as OCCT's <c>const&amp;</c>).
    /// </summary>
    public ref readonly BRepMesh_PairOfIndex ElementsConnectedTo(int theLinkIndex) => ref myLinks.FindFromIndex(theLinkIndex);

    // Elements

    /// <summary>The number of triangles (deleted ones included).</summary>
    public int NbElements() => myElements.Length();

    /// <summary>Adds a triangle; returns its index.</summary>
    public int AddElement(in BRepMesh_Triangle theElement)
    {
        myElements.Append(theElement);
        int aElementIndex = myElements.Length();
        myElementsOfDomain.Add(aElementIndex);

        BRepMesh_Triangle.Int3 e = theElement.myEdges;
        for (int i = 0; i < 3; ++i)
        {
            myLinks.FindFromIndex(e[i]).Append(aElementIndex);
        }

        return aElementIndex;
    }

    /// <summary>The triangle of the given index.</summary>
    public BRepMesh_Triangle GetElement(int theIndex) => myElements.Value(theIndex - 1);

    /// <summary>The triangle of the given index, for writing in place (valid until the next <see cref="AddElement"/>).</summary>
    internal ref BRepMesh_Triangle ChangeElement(int theIndex) => ref myElements.ChangeValue(theIndex - 1);

    /// <summary>The indices of the triangles of the domain.</summary>
    public NCollection_PackedMapOfInteger ElementsOfDomain() => myElementsOfDomain;

    /// <summary>Replaces the triangle of the given index.</summary>
    public bool SubstituteElement(int theIndex, in BRepMesh_Triangle theNewElement)
    {
        BRepMesh_Triangle aElement = GetElement(theIndex);
        if (aElement.Movability() == BRepMesh_Deleted)
        {
            // OCCT: myElements(theIndex) writes at index theIndex of a vector from 0, one after the triangle theIndex
            // (from 1) that GetElement reads; kept (occt-issues.md).
            setElementAt0(theIndex, theNewElement);
            return true;
        }

        cleanElement(theIndex, aElement);
        // Warning: here new element and old element should have different Hash code
        setElementAt0(theIndex, theNewElement); // OCCT: the same off-by-one write as above

        BRepMesh_Triangle.Int3 e = theNewElement.myEdges;
        for (int i = 0; i < 3; ++i)
        {
            myLinks.FindFromIndex(e[i]).Append(theIndex);
        }

        return true;
    }

    /// <summary>Removes the triangle of the given index: it is marked deleted and leaves the domain.</summary>
    public void RemoveElement(int theIndex)
    {
        ref BRepMesh_Triangle aElement = ref myElements.ChangeValue(theIndex - 1);
        if (aElement.Movability() == BRepMesh_Deleted)
        {
            return;
        }

        cleanElement(theIndex, aElement);
        aElement.SetMovability(BRepMesh_Deleted);
        myElementsOfDomain.Remove(theIndex);
    }

    /// <summary>The indices of the three nodes of the triangle, in its orientation, into a span of three.</summary>
    public void ElementNodes(in BRepMesh_Triangle theElement, Span<int> theNodes)
    {
        BRepMesh_Triangle.Int3 e = theElement.myEdges;
        BRepMesh_Triangle.Bool3 o = theElement.myOrientations;

        BRepMesh_Edge aLink1 = GetLink(e[0]);
        if (o[0])
        {
            theNodes[0] = aLink1.FirstNode();
            theNodes[1] = aLink1.LastNode();
        }
        else
        {
            theNodes[1] = aLink1.FirstNode();
            theNodes[0] = aLink1.LastNode();
        }

        BRepMesh_Edge aLink2 = GetLink(e[2]);
        if (o[2])
        {
            theNodes[2] = aLink2.FirstNode();
        }
        else
        {
            theNodes[2] = aLink2.LastNode();
        }
    }

    /// <summary>
    /// Writes the links of the domain (or the nodes, if there is no link) as edges (vertices) of a compound in the plane
    /// Z = 0 to a BRep file; returns the file name, or an error message.
    /// </summary>
    public string Dump(string theFileNameStr) => BRepMesh_Dump(this, theFileNameStr);

    // Auxiliary API

    /// <summary>The tool holding the nodes (cell sizes and tolerances are set through it).</summary>
    public BRepMesh_VertexTool Data() => myNodes;

    /// <summary>Removes every triangle of the domain, and the free links they leave.</summary>
    public void ClearDomain()
    {
        NCollection_PackedMapOfInteger aFreeEdges = new();
        NCollection_PackedMapOfInteger.Iterator aElementIt = new(myElementsOfDomain);
        for (; aElementIt.More(); aElementIt.Next())
        {
            int aElementId = aElementIt.Key();
            ref BRepMesh_Triangle aElement = ref myElements.ChangeValue(aElementId - 1);

            BRepMesh_Triangle.Int3 e = aElement.myEdges;

            for (int i = 0; i < 3; ++i)
            {
                aFreeEdges.Add(e[i]);
            }

            cleanElement(aElementId, aElement);
            aElement.SetMovability(BRepMesh_Deleted);
        }

        myElementsOfDomain.Clear();

        NCollection_PackedMapOfInteger.Iterator aEdgeIt = new(aFreeEdges);
        for (; aEdgeIt.More(); aEdgeIt.Next())
        {
            RemoveLink(aEdgeIt.Key());
        }
    }

    /// <summary>Renumbers the links and nodes to remove the deleted ones.</summary>
    public void ClearDeleted()
    {
        clearDeletedLinks();
        clearDeletedNodes();
    }

    private LinkedListOfInteger linksConnectedTo(int theIndex) => myNodeLinks.Find(theIndex);

    private void clearDeletedLinks()
    {
        Span<int> e = stackalloc int[3];
        Span<bool> o = stackalloc bool[3];
        int aLastLiveItem = NbLinks();
        while (myDelLinks.Count != 0)
        {
            while (aLastLiveItem > 0)
            {
                if (GetLink(aLastLiveItem).Movability() != BRepMesh_Deleted)
                {
                    break;
                }

                myLinks.RemoveLast();
                --aLastLiveItem;
            }

            int aDelItem = myDelLinks.Peek();
            myDelLinks.Dequeue();

            if (aDelItem > aLastLiveItem)
            {
                continue;
            }

            BRepMesh_Edge aLink = GetLink(aLastLiveItem);
            // The pair is read from the removed node, as OCCT reads it after RemoveLast (its incremental allocator
            // keeps the memory).
            ref BRepMesh_PairOfIndex aPair = ref myLinks.FindFromIndex(aLastLiveItem);

            myLinks.RemoveLast();
            myLinks.Substitute(aDelItem, aLink, aPair);

            myLinksOfDomain.Remove(aLastLiveItem);
            myLinksOfDomain.Add(aDelItem);
            --aLastLiveItem;

            int aLastLiveItemId = aLastLiveItem + 1;
            LinkedListOfInteger.Iterator aLinkIt = new();
            // update link references
            for (int i = 0; i < 2; ++i)
            {
                int aCurNodeId = (i == 0) ? aLink.FirstNode() : aLink.LastNode();

                for (aLinkIt.Init(linksConnectedTo(aCurNodeId)); aLinkIt.More(); aLinkIt.Next())
                {
                    ref int aLinkId = ref aLinkIt.ChangeValue();
                    if (aLinkId == aLastLiveItemId)
                    {
                        aLinkId = aDelItem;
                        break;
                    }
                }
            }

            // update elements references
            for (int j = 1, jn = aPair.Extent(); j <= jn; ++j)
            {
                int anElementId = aPair.Index(j);
                BRepMesh_Triangle aElement = GetElement(anElementId);
                aElement.Edges(e, o);
                for (int i = 0; i < 3; ++i)
                {
                    if (e[i] == aLastLiveItemId)
                    {
                        e[i] = aDelItem;
                        break;
                    }
                }

                // OCCT: myElements(anElementId) writes one after the triangle anElementId that GetElement read
                // (occt-issues.md).
                setElementAt0(anElementId, new BRepMesh_Triangle(e, o, aElement.Movability()));
            }
        }
    }

    private void clearDeletedNodes()
    {
        NCollection_List<int> aDelNodes = myNodes.GetListOfDelNodes();

        int aLastLiveItem = NbNodes();
        while (!aDelNodes.IsEmpty())
        {
            while (aLastLiveItem > 0)
            {
                if (GetNode(aLastLiveItem).Movability() != BRepMesh_Deleted)
                {
                    break;
                }

                myNodes.RemoveLast();
                --aLastLiveItem;
            }

            int aDelItem = aDelNodes.First();
            aDelNodes.RemoveFirst();

            if (aDelItem > aLastLiveItem)
            {
                continue;
            }

            BRepMesh_Vertex aNode = GetNode(aLastLiveItem);
            LinkedListOfInteger aLinkList = linksConnectedTo(aLastLiveItem);

            myNodes.RemoveLast();
            --aLastLiveItem;

            myNodes.Substitute(aDelItem, aNode);
            myNodeLinks.ChangeFind(aDelItem).Assign(aLinkList);

            int aLastLiveItemId = aLastLiveItem + 1;
            LinkedListOfInteger.Iterator aLinkIt = new(aLinkList);
            for (; aLinkIt.More(); aLinkIt.Next())
            {
                int aLinkId = aLinkIt.Value();
                BRepMesh_Edge aLink = GetLink(aLinkId);
                BRepMesh_PairOfIndex aPair = myLinks.FindFromIndex(aLinkId);

                int v0 = aLink.FirstNode();
                int v1 = aLink.LastNode();
                if (v0 == aLastLiveItemId)
                {
                    v0 = aDelItem;
                }
                else if (v1 == aLastLiveItemId)
                {
                    v1 = aDelItem;
                }

                myLinks.Substitute(aLinkId, new BRepMesh_Edge(v0, v1, aLink.Movability()), aPair);
            }
        }
    }

    // OCCT's myElements(theIndex) = ...: writes the triangle at index theIndex from 0. Past the last triangle OCCT
    // writes into the vector's unused capacity, which the next Append overwrites (or, at a block boundary, dereferences
    // a missing block); the port drops that write.
    private void setElementAt0(int theIndex, in BRepMesh_Triangle theElement)
    {
        if (theIndex < myElements.Length())
        {
            myElements[theIndex] = theElement;
        }
    }

    private void cleanLink(int theIndex, in BRepMesh_Edge theLink)
    {
        for (int i = 0; i < 2; ++i)
        {
            int aNodeId = (i == 0) ? theLink.FirstNode() : theLink.LastNode();

            LinkedListOfInteger aLinkList = linksConnectedTo(aNodeId);
            LinkedListOfInteger.Iterator aLinkIt = new(aLinkList);
            for (; aLinkIt.More(); aLinkIt.Next())
            {
                if (aLinkIt.Value() == theIndex)
                {
                    aLinkList.Remove(aLinkIt);
                    break;
                }
            }
        }
    }

    private void cleanElement(int theIndex, in BRepMesh_Triangle theElement)
    {
        if (theElement.Movability() != BRepMesh_Free)
        {
            return;
        }

        BRepMesh_Triangle.Int3 e = theElement.myEdges;
        for (int i = 0; i < 3; ++i)
        {
            removeElementIndex(theIndex, ref myLinks.FindFromIndex(e[i]));
        }
    }

    private static void removeElementIndex(int theIndex, ref BRepMesh_PairOfIndex thePair)
    {
        for (int i = 1, n = thePair.Extent(); i <= n; ++i)
        {
            if (thePair.Index(i) == theIndex)
            {
                thePair.RemoveIndex(i);
                return;
            }
        }
    }

    // OCCT's global BRepMesh_Dump (meant for the debugger); Dump calls it.
    private static string BRepMesh_Dump(BRepMesh_DataStructureOfDelaun? theMeshData, string? theFileNameStr)
    {
        if (theMeshData is null || theFileNameStr is null)
        {
            return "Error: file name or mesh data is null";
        }

        TopoDS_Compound aMesh = new();
        BRep_Builder aBuilder = new();
        aBuilder.MakeCompound(ref aMesh);

        try
        {
            if (theMeshData.LinksOfDomain().IsEmpty())
            {
                int aNodesNb = theMeshData.NbNodes();
                for (int i = 1; i <= aNodesNb; ++i)
                {
                    gp_XY aNode = theMeshData.GetNode(i).Coord();
                    gp_Pnt aPnt = new(aNode.X(), aNode.Y(), 0.0);
                    aBuilder.Add(aMesh, new BRepBuilderAPI_MakeVertex(aPnt).Shape());
                }
            }
            else
            {
                NCollection_PackedMapOfInteger.Iterator aLinksIt = new(theMeshData.LinksOfDomain());
                Span<gp_Pnt> aPnt = stackalloc gp_Pnt[2];
                for (; aLinksIt.More(); aLinksIt.Next())
                {
                    BRepMesh_Edge aLink = theMeshData.GetLink(aLinksIt.Key());
                    for (int i = 0; i < 2; ++i)
                    {
                        int aNodeId = (i == 0) ? aLink.FirstNode() : aLink.LastNode();

                        gp_XY aNode = theMeshData.GetNode(aNodeId).Coord();
                        aPnt[i] = new gp_Pnt(aNode.X(), aNode.Y(), 0.0);
                    }

                    if (aPnt[0].SquareDistance(aPnt[1]) < Precision.SquareConfusion())
                    {
                        continue;
                    }

                    aBuilder.Add(aMesh, new BRepBuilderAPI_MakeEdge(aPnt[0], aPnt[1]).Shape());
                }
            }

            if (!BRepTools.Write(aMesh, theFileNameStr))
            {
                return "Error: write failed";
            }
        }
        catch (Standard_Failure anException)
        {
            return anException.Message;
        }

        return theFileNameStr;
    }

    /// <summary>
    /// A singly linked list of integers with OCCT's NCollection_List semantics over an incremental allocator: a removed
    /// node keeps its link to the next one, so an iterator standing on it goes on where OCCT's does.
    /// </summary>
    public sealed class LinkedListOfInteger : IEnumerable<int>
    {
        internal sealed class Node(int theValue)
        {
            public int Value = theValue;
            public Node? Next;
        }

        private Node? myFirst;
        private Node? myLast;
        private int myLength;

        /// <summary>The number of items.</summary>
        public int Extent() => myLength;

        /// <summary>The number of items.</summary>
        public int Size() => myLength;

        /// <summary>Whether the list is empty.</summary>
        public bool IsEmpty() => myFirst is null;

        /// <summary>The first item; throws <see cref="Standard_NoSuchObject"/> on an empty list.</summary>
        public int First() => myFirst is not null ? myFirst.Value : throw new Standard_NoSuchObject("NCollection_List::First");

        /// <summary>The last item; throws <see cref="Standard_NoSuchObject"/> on an empty list.</summary>
        public int Last() => myLast is not null ? myLast.Value : throw new Standard_NoSuchObject("NCollection_List::Last");

        /// <summary>Appends an item.</summary>
        public void Append(int theItem)
        {
            Node aNode = new(theItem);
            if (myLength == 0)
            {
                myFirst = aNode;
            }
            else
            {
                myLast!.Next = aNode;
            }

            myLast = aNode;
            myLength++;
        }

        /// <summary>Removes every item.</summary>
        public void Clear()
        {
            myFirst = myLast = null;
            myLength = 0;
        }

        /// <summary>Replaces the items with a copy of the other list's (OCCT's assignment).</summary>
        public LinkedListOfInteger Assign(LinkedListOfInteger theOther)
        {
            if (ReferenceEquals(this, theOther))
            {
                return this;
            }

            Clear();
            for (Node? aNode = theOther.myFirst; aNode is not null; aNode = aNode.Next)
            {
                Append(aNode.Value);
            }

            return this;
        }

        /// <summary>Removes the iterator's item; the iterator moves to the next one.</summary>
        public void Remove(Iterator theIter)
        {
            Node aCurrent = theIter.myCurrent!;
            Node? aNode = aCurrent.Next;
            if (theIter.myPrevious is null)
            {
                myFirst = aCurrent.Next;
                myLength--;
                if (myLength == 0)
                {
                    myLast = null;
                }
            }
            else
            {
                theIter.myPrevious.Next = aNode;
                if (ReferenceEquals(myLast, aCurrent))
                {
                    myLast = theIter.myPrevious;
                }

                myLength--;
            }

            // The removed node keeps its Next, as OCCT's freed node in incremental-allocator memory does.
            theIter.myCurrent = aNode;
        }

        public IEnumerator<int> GetEnumerator()
        {
            for (Node? aNode = myFirst; aNode is not null; aNode = aNode.Next)
            {
                yield return aNode.Value;
            }
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        /// <summary>OCCT's list iterator: <c>for (It it(list); it.More(); it.Next()) it.Value();</c></summary>
        public sealed class Iterator
        {
            internal Node? myCurrent;
            internal Node? myPrevious;

            public Iterator() { }

            public Iterator(LinkedListOfInteger theList) => Init(theList);

            public void Init(LinkedListOfInteger theList)
            {
                myCurrent = theList.myFirst;
                myPrevious = null;
            }

            public void Initialize(LinkedListOfInteger theList) => Init(theList);

            public bool More() => myCurrent is not null;

            public void Next()
            {
                myPrevious = myCurrent;
                myCurrent = myCurrent!.Next;
            }

            public int Value() => myCurrent!.Value;

            public ref int ChangeValue() => ref myCurrent!.Value;
        }
    }

    /// <summary>
    /// OCCT's <c>NCollection_IndexedDataMap&lt;BRepMesh_Edge, BRepMesh_PairOfIndex&gt;</c> (IMeshData::IDMapOfLink): links
    /// by index from 1 with OCCT's hash and equality, in nodes that stay put, so references to keys and values stay
    /// valid as in C++.
    /// </summary>
    private sealed class LinkMap
    {
        private sealed class Node(in BRepMesh_Edge theKey, in BRepMesh_PairOfIndex theValue, int theIndex, Node? theNext)
        {
            public BRepMesh_Edge Key = theKey;
            public BRepMesh_PairOfIndex Value = theValue;
            public readonly int Index = theIndex;
            public Node? Next = theNext;
        }

        private Node?[]? myData1;
        private readonly List<Node> myData2 = [];
        private long myNbBuckets;

        public LinkMap(int theNbBuckets)
        {
            myNbBuckets = theNbBuckets > 0 ? theNbBuckets : 1;
        }

        public int Extent() => myData2.Count;

        private static long HashCode(in BRepMesh_Edge theKey, long theUpperBound) => theKey.OcctHash() % theUpperBound + 1;

        private void ReSize(long N)
        {
            long newBuck = NCollection_Primes.NextPrimeForMap(N);
            if (newBuck <= myNbBuckets)
            {
                if (myData1 is null)
                {
                    newBuck = myNbBuckets;
                }
                else
                {
                    return;
                }
            }

            Node?[] newdata = new Node?[newBuck + 1];
            if (myData1 is not null)
            {
                for (long i = 0; i <= myNbBuckets; ++i)
                {
                    Node? p = myData1[i];
                    while (p is not null)
                    {
                        long k = HashCode(p.Key, newBuck);
                        Node? q = p.Next;
                        p.Next = newdata[k];
                        newdata[k] = p;
                        p = q;
                    }
                }
            }

            myNbBuckets = newBuck;
            myData1 = newdata;
        }

        private Node? lookup(in BRepMesh_Edge theKey, out long theHash)
        {
            theHash = 0;
            if (myData2.Count == 0)
            {
                return null;
            }

            theHash = HashCode(theKey, myNbBuckets);
            for (Node? aNode = myData1![theHash]; aNode is not null; aNode = aNode.Next)
            {
                if (aNode.Key.IsEqual(theKey))
                {
                    return aNode;
                }
            }

            return null;
        }

        public int Add(in BRepMesh_Edge theKey, in BRepMesh_PairOfIndex theItem)
        {
            if (myData2.Count == 0 || myData2.Count > myNbBuckets)
            {
                ReSize(myData2.Count);
            }

            Node? aNode = lookup(theKey, out long aHash);
            if (aNode is not null)
            {
                return aNode.Index;
            }

            aHash = HashCode(theKey, myNbBuckets);
            int aNewIndex = myData2.Count + 1;
            aNode = new Node(theKey, theItem, aNewIndex, myData1![aHash]);
            myData1[aHash] = aNode;
            myData2.Add(aNode);
            return aNewIndex;
        }

        public int FindIndex(in BRepMesh_Edge theKey) => lookup(theKey, out _) is Node aNode ? aNode.Index : 0;

        private Node nodeOf(int theIndex)
        {
            Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myData2.Count, "NCollection_IndexedDataMap::FindKey");
            return myData2[theIndex - 1];
        }

        public ref BRepMesh_Edge FindKey(int theIndex) => ref nodeOf(theIndex).Key;

        public ref BRepMesh_PairOfIndex FindFromIndex(int theIndex) => ref nodeOf(theIndex).Value;

        public void Substitute(int theIndex, in BRepMesh_Edge theKey1, in BRepMesh_PairOfIndex theItem)
        {
            Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myData2.Count,
                                         "NCollection_IndexedDataMap::Substitute : Index is out of range");

            // check if theKey1 is not already in the map
            Node? aNode = lookup(theKey1, out long aHash);
            if (aNode is not null)
            {
                if (aNode.Index != theIndex)
                {
                    throw new Standard_DomainError("NCollection_IndexedDataMap::Substitute : Attempt to substitute existing key");
                }

                aNode.Key = theKey1;
                aNode.Value = theItem;
                return;
            }

            // Find the node for the index I
            aNode = myData2[theIndex - 1];

            // remove the old key
            long iK = HashCode(aNode.Key, myNbBuckets);
            Node? q = myData1![iK];
            if (ReferenceEquals(q, aNode))
            {
                myData1[iK] = aNode.Next;
            }
            else
            {
                while (!ReferenceEquals(q!.Next, aNode))
                {
                    q = q.Next;
                }

                q.Next = aNode.Next;
            }

            // update the node
            aNode.Key = theKey1;
            aNode.Value = theItem;
            aNode.Next = myData1[aHash];
            myData1[aHash] = aNode;
        }

        public void RemoveLast()
        {
            int aLastIndex = myData2.Count;
            Standard_OutOfRange_Raise_if(aLastIndex == 0, "NCollection_IndexedDataMap::RemoveLast");

            // Find the node for the last index and remove it
            Node p = myData2[aLastIndex - 1];
            myData2.RemoveAt(aLastIndex - 1);

            // remove the key
            long iK = HashCode(p.Key, myNbBuckets);
            Node? q = myData1![iK];
            if (ReferenceEquals(q, p))
            {
                myData1[iK] = p.Next;
            }
            else
            {
                while (!ReferenceEquals(q!.Next, p))
                {
                    q = q.Next;
                }

                q.Next = p.Next;
            }
        }
    }
}
