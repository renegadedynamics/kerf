// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DataStructureOfDelaun.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DataStructureOfDelaun.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Collections;
using System.Runtime.InteropServices;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The mesh of the Delaunay triangulation in the parametric space: nodes (<see cref="MeshVertex"/>), links
/// (<see cref="MeshEdge"/>) with the triangles connected to each, and triangles (<see cref="Triangle"/>),
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
public sealed class DataStructureOfDelaun : Transient
{
    private readonly VertexTool _nodes;
    private readonly Dictionary<int, LinkedListOfInteger> _nodeLinks;
    private readonly LinkMap _links;
    // OCCT's NCollection_List<int>, used only as a first-in first-out queue: a .NET queue, whose RemoveFirst
    // doesn't move the rest (unit 9E)
    private readonly Queue<int> _delLinks;
    private readonly List<Triangle> _elements;
    private readonly NCollection_PackedMapOfInteger _elementsOfDomain;
    private readonly NCollection_PackedMapOfInteger _linksOfDomain;

    /// <summary>An empty structure reserving room for about <paramref name="reservedNodeSize"/> nodes.</summary>
    public DataStructureOfDelaun(int reservedNodeSize = 100)
    {
        _nodes = new VertexTool();
        _nodeLinks = new Dictionary<int, LinkedListOfInteger>();
        _links = new LinkMap(reservedNodeSize * 3);
        _delLinks = new Queue<int>();
        _elements = new List<Triangle>(reservedNodeSize * 2);
        _elementsOfDomain = new NCollection_PackedMapOfInteger();
        _linksOfDomain = new NCollection_PackedMapOfInteger();
    }

    // Nodes

    /// <summary>The number of nodes (deleted ones included).</summary>
    public int NodeCount => _nodes.Extent();

    /// <summary>
    /// Adds a node, unless one coincides with it within the tolerance of <see cref="Data"/> (or
    /// <paramref name="isForceAdd"/>); returns the index of the node added or found.
    /// </summary>
    public int AddNode(in MeshVertex node, bool isForceAdd = false)
    {
        int nodeId = _nodes.Add(node, isForceAdd);
        if (!_nodeLinks.ContainsKey(nodeId))
        {
            _nodeLinks[nodeId] = new LinkedListOfInteger();
        }

        return nodeId;
    }

    /// <summary>The index of the node coinciding with the given one, or 0.</summary>
    public int IndexOf(in MeshVertex node) => _nodes.FindIndex(node);

    /// <summary>The node of the given index.</summary>
    public MeshVertex GetNode(int index) => _nodes.FindKey(index);

    /// <summary>The node of the given index (OCCT's <c>operator()</c>).</summary>
    public MeshVertex this[int index] => GetNode(index);

    /// <summary>Replaces the node of the given index; false if another node coincides with the new one.</summary>
    public bool SubstituteNode(int index, in MeshVertex newNode)
    {
        if (_nodes.FindIndex(newNode) != 0)
        {
            return false;
        }

        _nodes.Substitute(index, newNode);
        return true;
    }

    /// <summary>Removes a free node (or any, with <paramref name="isForce"/>) to which no link is connected.</summary>
    public void RemoveNode(int index, bool isForce = false)
    {
        if (isForce || _nodes.FindKey(index).Movability() == DegreeOfFreedom.Free)
        {
            if (LinksConnectedTo(index).Extent() == 0)
            {
                _nodes.DeleteVertex(index);
            }
        }
    }

    /// <summary>The indices of the links connected to the node (the structure's own list).</summary>
    public LinkedListOfInteger LinksConnectedTo(int index) => LinksConnectedToCore(index);

    // Links

    /// <summary>The number of links (deleted ones included).</summary>
    public int LinkCount => _links.Extent();

    /// <summary>
    /// Adds a link, or finds the same link in either orientation; returns its index, negative if the existing link is
    /// reversed. A deleted link's index is reused.
    /// </summary>
    public int AddLink(in MeshEdge link)
    {
        int linkIndex = IndexOf(link);
        if (linkIndex > 0)
        {
            return link.IsSameOrientation(GetLink(linkIndex)) ? linkIndex : -linkIndex;
        }

        PairOfIndex pair = new();
        if (_delLinks.Count != 0)
        {
            linkIndex = _delLinks.Peek();
            _links.Substitute(linkIndex, link, pair);
            _delLinks.Dequeue();
        }
        else
        {
            linkIndex = _links.Add(link, pair);
        }

        int linkId = Math.Abs(linkIndex);
        LinksConnectedToCore(link.FirstNode()).Append(linkId);
        LinksConnectedToCore(link.LastNode()).Append(linkId);
        _linksOfDomain.Add(linkIndex);

        return linkIndex;
    }

    /// <summary>The index of the link joining the same nodes (either orientation), or 0.</summary>
    public int IndexOf(in MeshEdge link) => _links.FindIndex(link);

    /// <summary>The link of the given index.</summary>
    public MeshEdge GetLink(int index) => _links.FindKey(index);

    /// <summary>The link of the given index, for writing in place (OCCT's <c>const_cast</c> of <c>GetLink</c>).</summary>
    internal ref MeshEdge ChangeLink(int index) => ref _links.FindKey(index);

    /// <summary>The indices of the links of the domain.</summary>
    public NCollection_PackedMapOfInteger LinksOfDomain() => _linksOfDomain;

    /// <summary>Replaces the link of the given index; false if the new link already exists.</summary>
    public bool SubstituteLink(int index, in MeshEdge newLink)
    {
        PairOfIndex pair = new();
        MeshEdge link = GetLink(index);
        if (link.Movability() == DegreeOfFreedom.Deleted)
        {
            _links.Substitute(index, newLink, pair);
            return true;
        }

        if (IndexOf(newLink) != 0)
        {
            return false;
        }

        link.SetMovability(DegreeOfFreedom.Deleted);
        _links.Substitute(index, link, pair);
        CleanLink(index, link);

        int linkId = Math.Abs(index);
        LinksConnectedToCore(newLink.FirstNode()).Append(linkId);
        LinksConnectedToCore(newLink.LastNode()).Append(linkId);
        _links.Substitute(index, newLink, pair);

        return true;
    }

    /// <summary>
    /// Removes a free link (or any, with <paramref name="isForce"/>) to which no triangle is connected: it is marked
    /// deleted and its index is reused by the next <see cref="AddLink"/>.
    /// </summary>
    public void RemoveLink(int index, bool isForce = false)
    {
        ref MeshEdge link = ref _links.FindKey(index);
        if (link.Movability() == DegreeOfFreedom.Deleted || (!isForce && link.Movability() != DegreeOfFreedom.Free)
            || ElementsConnectedTo(index).Extent() != 0)
        {
            return;
        }

        CleanLink(index, link);
        link.SetMovability(DegreeOfFreedom.Deleted);

        _linksOfDomain.Remove(index);
        _delLinks.Enqueue(index);
    }

    /// <summary>
    /// The triangles connected to the link of the given index: a reference into the structure, which sees later
    /// changes (as OCCT's <c>const&amp;</c>).
    /// </summary>
    public ref readonly PairOfIndex ElementsConnectedTo(int linkIndex) => ref _links.FindFromIndex(linkIndex);

    // Elements

    /// <summary>The number of triangles (deleted ones included).</summary>
    public int ElementCount => _elements.Count;

    /// <summary>Adds a triangle; returns its index.</summary>
    public int AddElement(in Triangle element)
    {
        _elements.Add(element);
        int elementIndex = _elements.Count;
        _elementsOfDomain.Add(elementIndex);

        Triangle.Int3 e = element._edges;
        for (int i = 0; i < 3; ++i)
        {
            _links.FindFromIndex(e[i]).Append(elementIndex);
        }

        return elementIndex;
    }

    /// <summary>The triangle of the given index.</summary>
    public Triangle GetElement(int index) => _elements[index - 1];

    /// <summary>The triangle of the given index, for writing in place (valid until the next <see cref="AddElement"/>).</summary>
    internal ref Triangle ChangeElement(int index) => ref CollectionsMarshal.AsSpan(_elements)[index - 1];

    /// <summary>The indices of the triangles of the domain.</summary>
    public NCollection_PackedMapOfInteger ElementsOfDomain() => _elementsOfDomain;

    /// <summary>Replaces the triangle of the given index.</summary>
    public bool SubstituteElement(int index, in Triangle newElement)
    {
        Triangle element = GetElement(index);
        if (element.Movability() == DegreeOfFreedom.Deleted)
        {
            // OCCT: myElements(theIndex) writes at index theIndex of a vector from 0, one after the triangle theIndex
            // (from 1) that GetElement reads; kept (occt-issues.md).
            SetElementAt0(index, newElement);
            return true;
        }

        CleanElement(index, element);
        // Warning: here new element and old element should have different Hash code
        SetElementAt0(index, newElement); // OCCT: the same off-by-one write as above

        Triangle.Int3 e = newElement._edges;
        for (int i = 0; i < 3; ++i)
        {
            _links.FindFromIndex(e[i]).Append(index);
        }

        return true;
    }

    /// <summary>Removes the triangle of the given index: it is marked deleted and leaves the domain.</summary>
    public void RemoveElement(int index)
    {
        ref Triangle element = ref CollectionsMarshal.AsSpan(_elements)[index - 1];
        if (element.Movability() == DegreeOfFreedom.Deleted)
        {
            return;
        }

        CleanElement(index, element);
        element.SetMovability(DegreeOfFreedom.Deleted);
        _elementsOfDomain.Remove(index);
    }

    /// <summary>The indices of the three nodes of the triangle, in its orientation, into a span of three.</summary>
    public void ElementNodes(in Triangle element, Span<int> nodes)
    {
        Triangle.Int3 e = element._edges;
        Triangle.Bool3 o = element._orientations;

        MeshEdge link1 = GetLink(e[0]);
        if (o[0])
        {
            nodes[0] = link1.FirstNode();
            nodes[1] = link1.LastNode();
        }
        else
        {
            nodes[1] = link1.FirstNode();
            nodes[0] = link1.LastNode();
        }

        MeshEdge link2 = GetLink(e[2]);
        if (o[2])
        {
            nodes[2] = link2.FirstNode();
        }
        else
        {
            nodes[2] = link2.LastNode();
        }
    }

    /// <summary>
    /// Writes the links of the domain (or the nodes, if there is no link) as edges (vertices) of a compound in the plane
    /// Z = 0 to a BRep file; returns the file name, or an error message.
    /// </summary>
    public string Dump(string fileNameStr) => BRepMesh_Dump(this, fileNameStr);

    // Auxiliary API

    /// <summary>The tool holding the nodes (cell sizes and tolerances are set through it).</summary>
    public VertexTool Data() => _nodes;

    /// <summary>Removes every triangle of the domain, and the free links they leave.</summary>
    public void ClearDomain()
    {
        NCollection_PackedMapOfInteger freeEdges = new();
        NCollection_PackedMapOfInteger.Iterator elementIt = new(_elementsOfDomain);
        for (; elementIt.More(); elementIt.Next())
        {
            int elementId = elementIt.Key();
            ref Triangle element = ref CollectionsMarshal.AsSpan(_elements)[elementId - 1];

            Triangle.Int3 e = element._edges;

            for (int i = 0; i < 3; ++i)
            {
                freeEdges.Add(e[i]);
            }

            CleanElement(elementId, element);
            element.SetMovability(DegreeOfFreedom.Deleted);
        }

        _elementsOfDomain.Clear();

        NCollection_PackedMapOfInteger.Iterator edgeIt = new(freeEdges);
        for (; edgeIt.More(); edgeIt.Next())
        {
            RemoveLink(edgeIt.Key());
        }
    }

    /// <summary>Renumbers the links and nodes to remove the deleted ones.</summary>
    public void ClearDeleted()
    {
        ClearDeletedLinks();
        ClearDeletedNodes();
    }

    private LinkedListOfInteger LinksConnectedToCore(int index) => _nodeLinks.GetValue(index);

    private void ClearDeletedLinks()
    {
        Span<int> e = stackalloc int[3];
        Span<bool> o = stackalloc bool[3];
        int lastLiveItem = LinkCount;
        while (_delLinks.Count != 0)
        {
            while (lastLiveItem > 0)
            {
                if (GetLink(lastLiveItem).Movability() != DegreeOfFreedom.Deleted)
                {
                    break;
                }

                _links.RemoveLast();
                --lastLiveItem;
            }

            int delItem = _delLinks.Peek();
            _delLinks.Dequeue();

            if (delItem > lastLiveItem)
            {
                continue;
            }

            MeshEdge link = GetLink(lastLiveItem);
            // The pair is read from the removed node, as OCCT reads it after RemoveLast (its incremental allocator
            // keeps the memory).
            ref PairOfIndex pair = ref _links.FindFromIndex(lastLiveItem);

            _links.RemoveLast();
            _links.Substitute(delItem, link, pair);

            _linksOfDomain.Remove(lastLiveItem);
            _linksOfDomain.Add(delItem);
            --lastLiveItem;

            int lastLiveItemId = lastLiveItem + 1;
            LinkedListOfInteger.Iterator linkIt = new();
            // update link references
            for (int i = 0; i < 2; ++i)
            {
                int curNodeId = (i == 0) ? link.FirstNode() : link.LastNode();

                for (linkIt.Init(LinksConnectedToCore(curNodeId)); linkIt.More(); linkIt.Next())
                {
                    ref int linkId = ref linkIt.ChangeValue();
                    if (linkId == lastLiveItemId)
                    {
                        linkId = delItem;
                        break;
                    }
                }
            }

            // update elements references
            for (int j = 1, jn = pair.Extent(); j <= jn; ++j)
            {
                int elementId = pair.Index(j);
                Triangle element = GetElement(elementId);
                element.Edges(e, o);
                for (int i = 0; i < 3; ++i)
                {
                    if (e[i] == lastLiveItemId)
                    {
                        e[i] = delItem;
                        break;
                    }
                }

                // OCCT: myElements(anElementId) writes one after the triangle anElementId that GetElement read
                // (occt-issues.md).
                SetElementAt0(elementId, new Triangle(e, o, element.Movability()));
            }
        }
    }

    private void ClearDeletedNodes()
    {
        NCollection_List<int> delNodes = _nodes.GetListOfDelNodes();

        int lastLiveItem = NodeCount;
        while (!delNodes.IsEmpty())
        {
            while (lastLiveItem > 0)
            {
                if (GetNode(lastLiveItem).Movability() != DegreeOfFreedom.Deleted)
                {
                    break;
                }

                _nodes.RemoveLast();
                --lastLiveItem;
            }

            int delItem = delNodes.First();
            delNodes.RemoveFirst();

            if (delItem > lastLiveItem)
            {
                continue;
            }

            MeshVertex node = GetNode(lastLiveItem);
            LinkedListOfInteger linkList = LinksConnectedToCore(lastLiveItem);

            _nodes.RemoveLast();
            --lastLiveItem;

            _nodes.Substitute(delItem, node);
            _nodeLinks.GetValue(delItem).Assign(linkList);

            int lastLiveItemId = lastLiveItem + 1;
            LinkedListOfInteger.Iterator linkIt = new(linkList);
            for (; linkIt.More(); linkIt.Next())
            {
                int linkId = linkIt.Value();
                MeshEdge link = GetLink(linkId);
                PairOfIndex pair = _links.FindFromIndex(linkId);

                int v0 = link.FirstNode();
                int v1 = link.LastNode();
                if (v0 == lastLiveItemId)
                {
                    v0 = delItem;
                }
                else if (v1 == lastLiveItemId)
                {
                    v1 = delItem;
                }

                _links.Substitute(linkId, new MeshEdge(v0, v1, link.Movability()), pair);
            }
        }
    }

    // OCCT's myElements(theIndex) = ...: writes the triangle at index theIndex from 0. Past the last triangle OCCT
    // writes into the vector's unused capacity, which the next Append overwrites (or, at a block boundary, dereferences
    // a missing block); the port drops that write.
    private void SetElementAt0(int index, in Triangle element)
    {
        if (index < _elements.Count)
        {
            _elements[index] = element;
        }
    }

    private void CleanLink(int index, in MeshEdge link)
    {
        for (int i = 0; i < 2; ++i)
        {
            int nodeId = (i == 0) ? link.FirstNode() : link.LastNode();

            LinkedListOfInteger linkList = LinksConnectedToCore(nodeId);
            LinkedListOfInteger.Iterator linkIt = new(linkList);
            for (; linkIt.More(); linkIt.Next())
            {
                if (linkIt.Value() == index)
                {
                    linkList.Remove(linkIt);
                    break;
                }
            }
        }
    }

    private void CleanElement(int index, in Triangle element)
    {
        if (element.Movability() != DegreeOfFreedom.Free)
        {
            return;
        }

        Triangle.Int3 e = element._edges;
        for (int i = 0; i < 3; ++i)
        {
            RemoveElementIndex(index, ref _links.FindFromIndex(e[i]));
        }
    }

    private static void RemoveElementIndex(int index, ref PairOfIndex pair)
    {
        for (int i = 1, n = pair.Extent(); i <= n; ++i)
        {
            if (pair.Index(i) == index)
            {
                pair.RemoveIndex(i);
                return;
            }
        }
    }

    // OCCT's global BRepMesh_Dump (meant for the debugger); Dump calls it.
    private static string BRepMesh_Dump(DataStructureOfDelaun? meshData, string? fileNameStr)
    {
        if (meshData is null || fileNameStr is null)
        {
            return "Error: file name or mesh data is null";
        }

        Compound mesh = new();
        BrepBuilder builder = new();
        builder.MakeCompound(ref mesh);

        try
        {
            if (meshData.LinksOfDomain().IsEmpty())
            {
                int nodesNb = meshData.NodeCount;
                for (int i = 1; i <= nodesNb; ++i)
                {
                    Xy node = meshData.GetNode(i).Coord();
                    Point3D pnt = new(node.X(), node.Y(), 0.0);
                    builder.Add(mesh, new MakeVertex(pnt).Shape());
                }
            }
            else
            {
                NCollection_PackedMapOfInteger.Iterator linksIt = new(meshData.LinksOfDomain());
                Span<Point3D> pnt = stackalloc Point3D[2];
                for (; linksIt.More(); linksIt.Next())
                {
                    MeshEdge link = meshData.GetLink(linksIt.Key());
                    for (int i = 0; i < 2; ++i)
                    {
                        int nodeId = (i == 0) ? link.FirstNode() : link.LastNode();

                        Xy node = meshData.GetNode(nodeId).Coord();
                        pnt[i] = new Point3D(node.X(), node.Y(), 0.0);
                    }

                    if (pnt[0].SquareDistance(pnt[1]) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        continue;
                    }

                    builder.Add(mesh, new MakeEdge(pnt[0], pnt[1]).Shape());
                }
            }

            if (!BrepTools.Write(mesh, fileNameStr))
            {
                return "Error: write failed";
            }
        }
        catch (FailureException exception)
        {
            return exception.Message;
        }

        return fileNameStr;
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

        private Node? _first;
        private Node? _last;
        private int _length;

        /// <summary>The number of items.</summary>
        public int Extent() => _length;

        /// <summary>The number of items.</summary>
        public int Size() => _length;

        /// <summary>Whether the list is empty.</summary>
        public bool IsEmpty => _first is null;

        /// <summary>The first item; throws <see cref="NoSuchObjectException"/> on an empty list.</summary>
        public int First() => _first is not null ? _first.Value : throw new NoSuchObjectException("NCollection_List::First");

        /// <summary>The last item; throws <see cref="NoSuchObjectException"/> on an empty list.</summary>
        public int Last() => _last is not null ? _last.Value : throw new NoSuchObjectException("NCollection_List::Last");

        /// <summary>Appends an item.</summary>
        public void Append(int item)
        {
            Node node = new(item);
            if (_length == 0)
            {
                _first = node;
            }
            else
            {
                _last!.Next = node;
            }

            _last = node;
            _length++;
        }

        /// <summary>Removes every item.</summary>
        public void Clear()
        {
            _first = _last = null;
            _length = 0;
        }

        /// <summary>Replaces the items with a copy of the other list's (OCCT's assignment).</summary>
        public LinkedListOfInteger Assign(LinkedListOfInteger other)
        {
            if (ReferenceEquals(this, other))
            {
                return this;
            }

            Clear();
            for (Node? node = other._first; node is not null; node = node.Next)
            {
                Append(node.Value);
            }

            return this;
        }

        /// <summary>Removes the iterator's item; the iterator moves to the next one.</summary>
        public void Remove(Iterator iter)
        {
            Node current = iter._current!;
            Node? node = current.Next;
            if (iter._previous is null)
            {
                _first = current.Next;
                _length--;
                if (_length == 0)
                {
                    _last = null;
                }
            }
            else
            {
                iter._previous.Next = node;
                if (ReferenceEquals(_last, current))
                {
                    _last = iter._previous;
                }

                _length--;
            }

            // The removed node keeps its Next, as OCCT's freed node in incremental-allocator memory does.
            iter._current = node;
        }

        public IEnumerator<int> GetEnumerator()
        {
            for (Node? node = _first; node is not null; node = node.Next)
            {
                yield return node.Value;
            }
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        /// <summary>OCCT's list iterator: <c>for (It it(list); it.More(); it.Next()) it.Value();</c></summary>
        public sealed class Iterator
        {
            internal Node? _current;
            internal Node? _previous;

            public Iterator() { }

            public Iterator(LinkedListOfInteger list) => Init(list);

            public void Init(LinkedListOfInteger list)
            {
                _current = list._first;
                _previous = null;
            }

            public void Initialize(LinkedListOfInteger list) => Init(list);

            public bool More() => _current is not null;

            public void Next()
            {
                _previous = _current;
                _current = _current!.Next;
            }

            public int Value() => _current!.Value;

            public ref int ChangeValue() => ref _current!.Value;
        }
    }

    /// <summary>
    /// OCCT's <c>NCollection_IndexedDataMap&lt;BRepMesh_Edge, BRepMesh_PairOfIndex&gt;</c> (IMeshData::IDMapOfLink): links
    /// by index from 1 with OCCT's hash and equality, in nodes that stay put, so references to keys and values stay
    /// valid as in C++.
    /// </summary>
    private sealed class LinkMap
    {
        private sealed class Node(in MeshEdge theKey, in PairOfIndex theValue, int theIndex, Node? theNext)
        {
            public MeshEdge Key = theKey;
            public PairOfIndex Value = theValue;
            public readonly int Index = theIndex;
            public Node? Next = theNext;
        }

        private Node?[]? _data1;
        private readonly List<Node> _data2 = [];
        private long _nbBuckets;

        public LinkMap(int nbBuckets)
        {
            _nbBuckets = nbBuckets > 0 ? nbBuckets : 1;
        }

        public int Extent() => _data2.Count;

        private static long HashCode(in MeshEdge key, long upperBound) => key.OcctHash() % upperBound + 1;

        private void ReSize(long n)
        {
            long newBuck = Primes.NextPrimeForMap(n);
            if (newBuck <= _nbBuckets)
            {
                if (_data1 is null)
                {
                    newBuck = _nbBuckets;
                }
                else
                {
                    return;
                }
            }

            Node?[] newdata = new Node?[newBuck + 1];
            if (_data1 is not null)
            {
                for (long i = 0; i <= _nbBuckets; ++i)
                {
                    Node? p = _data1[i];
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

            _nbBuckets = newBuck;
            _data1 = newdata;
        }

        private Node? Lookup(in MeshEdge key, out long hash)
        {
            hash = 0;
            if (_data2.Count == 0)
            {
                return null;
            }

            hash = HashCode(key, _nbBuckets);
            for (Node? node = _data1![hash]; node is not null; node = node.Next)
            {
                if (node.Key.IsEqual(key))
                {
                    return node;
                }
            }

            return null;
        }

        public int Add(in MeshEdge key, in PairOfIndex item)
        {
            if (_data2.Count == 0 || _data2.Count > _nbBuckets)
            {
                ReSize(_data2.Count);
            }

            Node? node = this.Lookup(key, out long hash);
            if (node is not null)
            {
                return node.Index;
            }

            hash = HashCode(key, _nbBuckets);
            int newIndex = _data2.Count + 1;
            node = new Node(key, item, newIndex, _data1![hash]);
            _data1[hash] = node;
            _data2.Add(node);
            return newIndex;
        }

        public int FindIndex(in MeshEdge key) => this.Lookup(key, out _) is Node node ? node.Index : 0;

        private Node NodeOf(int index)
        {
            Standard_OutOfRange_Raise_if(index < 1 || index > _data2.Count, "NCollection_IndexedDataMap::FindKey");
            return _data2[index - 1];
        }

        public ref MeshEdge FindKey(int index) => ref NodeOf(index).Key;

        public ref PairOfIndex FindFromIndex(int index) => ref NodeOf(index).Value;

        public void Substitute(int index, in MeshEdge key1, in PairOfIndex item)
        {
            Standard_OutOfRange_Raise_if(index < 1 || index > _data2.Count,
                                         "NCollection_IndexedDataMap::Substitute : Index is out of range");

            // check if theKey1 is not already in the map
            Node? node = this.Lookup(key1, out long hash);
            if (node is not null)
            {
                if (node.Index != index)
                {
                    throw new DomainErrorException("NCollection_IndexedDataMap::Substitute : Attempt to substitute existing key");
                }

                node.Key = key1;
                node.Value = item;
                return;
            }

            // Find the node for the index I
            node = _data2[index - 1];

            // remove the old key
            long iK = HashCode(node.Key, _nbBuckets);
            Node? q = _data1![iK];
            if (ReferenceEquals(q, node))
            {
                _data1[iK] = node.Next;
            }
            else
            {
                while (!ReferenceEquals(q!.Next, node))
                {
                    q = q.Next;
                }

                q.Next = node.Next;
            }

            // update the node
            node.Key = key1;
            node.Value = item;
            node.Next = _data1[hash];
            _data1[hash] = node;
        }

        public void RemoveLast()
        {
            int lastIndex = _data2.Count;
            Standard_OutOfRange_Raise_if(lastIndex == 0, "NCollection_IndexedDataMap::RemoveLast");

            // Find the node for the last index and remove it
            Node p = _data2[lastIndex - 1];
            _data2.RemoveAt(lastIndex - 1);

            // remove the key
            long iK = HashCode(p.Key, _nbBuckets);
            Node? q = _data1![iK];
            if (ReferenceEquals(q, p))
            {
                _data1[iK] = p.Next;
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
