// Port of: src/FoundationClasses/TKMath/Poly/Poly_MakeLoops.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_MakeLoops.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2009-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using System.Globalization;
using static Kerf.Engine.Meshing.MakeLoops.LinkFlag;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Makes loops from a set of connected links, each a pair of node indices. A subclass chooses the way at a fork
/// (<see cref="MakeLoops3D"/>, <see cref="MakeLoops2D"/>), with a <see cref="Helper"/> giving the links
/// at each node.
/// </summary>
/// <remarks>
/// An algorithm object, a class as in OCCT. Allocator arguments are dropped, and OCCT's debug hook
/// (<see cref="Helper.OnAddLink"/>, called only in OCCT's debug builds) isn't called. The set of start links is OCCT's
/// packed integer map, ported with its iteration order, so loops are found in OCCT's order.
/// </remarks>
internal abstract class MakeLoops
{
    /// <summary>The orientation flags of a link.</summary>
    public enum LinkFlag
    {
        LF_None = 0,
        LF_Fwd = 1,      // forward orientation
        LF_Rev = 2,      // reversed orientation
        LF_Both = 3,     // both ways oriented
        LF_Reversed = 4, // means the link is reversed
    }

    /// <summary>
    /// A link: two node indices and orientation flags. Two links are equal (<c>==</c>, <see cref="Equals(Link)"/>) if
    /// they join the same nodes, in either order, whatever their flags: OCCT's own equality, which its maps use.
    /// </summary>
    /// <remarks>The fields are public, as in OCCT; a link is a small record filled by the caller.</remarks>
    public struct Link : IEquatable<Link>
    {
        public int node1, node2;
        public int flags;

        /// <summary>A null link (nodes 0, no flags).</summary>
        public Link()
        {
            node1 = 0;
            node2 = 0;
            flags = 0;
        }

        /// <summary>A forward link between two nodes.</summary>
        public Link(int theNode1, int theNode2)
        {
            node1 = theNode1;
            node2 = theNode2;
            flags = 1;
        }

        /// <summary>Flips the "reversed" flag.</summary>
        public void Reverse() => flags ^= (int)LF_Reversed;

        /// <summary>This link with the "reversed" flag flipped.</summary>
        public readonly Link Reversed()
        {
            Link copy = this;
            copy.Reverse();
            return copy;
        }

        public readonly bool IsReversed => (flags & (int)LF_Reversed) != 0;

        /// <summary>Makes this link null.</summary>
        public void Nullify() => node1 = node2 = 0;

        public readonly bool IsNull => node1 == 0 || node2 == 0;

        public readonly bool Equals(Link other)
            => (other.node1 == node1 && other.node2 == node2) || (other.node1 == node2 && other.node2 == node1);

        public override readonly bool Equals(object? obj) => obj is Link other && Equals(other);

        /// <summary>A hash of the two nodes, whatever their order.</summary>
        public override readonly int GetHashCode()
            => node1 < node2 ? HashCode.Combine(node1, node2) : HashCode.Combine(node2, node1);

        public static bool operator ==(in Link left, in Link right) => left.Equals(right);

        public static bool operator !=(in Link left, in Link right) => !left.Equals(right);

        public override readonly string ToString()
            => string.Create(CultureInfo.InvariantCulture, $"Poly_MakeLoops.Link({node1}, {node2}, flags {flags})");
    }

    /// <summary>The helper: gives the links at a node.</summary>
    public abstract class Helper
    {
        /// <summary>The links adjacent to a node.</summary>
        public abstract NCollection_List<Link> GetAdjacentLinks(int node);

        /// <summary>A hook OCCT calls from AddLink in its debug builds only; never called by the port.</summary>
        public virtual void OnAddLink(int num, in Link link) { }
    }

    /// <summary>
    /// A heap of integers, best used by adding all items first, then taking the top item and removing items until it
    /// is empty. The top is the first item in OCCT's packed map order.
    /// </summary>
    public sealed class HeapOfInteger
    {
        private readonly NCollection_PackedMapOfInteger _map;
        private readonly NCollection_PackedMapOfInteger.Iterator _iter = new();
        private bool _iterReady;

        public HeapOfInteger(int nbPreAllocated = 1)
        {
            _map = new NCollection_PackedMapOfInteger(nbPreAllocated);
            _iterReady = false;
        }

        public void Clear()
        {
            _map.Clear();
            _iterReady = false;
        }

        public void Add(int value)
        {
            _map.Add(value);
            _iterReady = false;
        }

        public int Top()
        {
            if (!_iterReady)
            {
                _iter.Initialize(_map);
                _iterReady = true;
            }

            return _iter.Key();
        }

        public bool Contains(int value) => _map.Contains(value);

        public void Remove(int value)
        {
            if (_iterReady && _iter.More() && _iter.Key() == value)
            {
                _iter.Next();
            }

            _map.Remove(value);
        }

        public bool IsEmpty()
        {
            if (!_iterReady)
            {
                _iter.Initialize(_map);
                _iterReady = true;
            }

            return !_iter.More();
        }
    }

    /// <summary>The result codes of <see cref="Perform"/>, combined.</summary>
    public enum ResultCode
    {
        RC_LoopsDone = 1,
        RC_HangingLinks = 2,
        RC_Failure = 4,
    }

    private Helper? _helper;
    private readonly IndexedSet<Link> _mapLink = new();
    private readonly NCollection_Sequence<NCollection_List<Link>> _loops = new();
    private readonly HeapOfInteger _startIndices = new(4000);
    private readonly NCollection_PackedMapOfInteger _hangIndices = new();

    /// <summary>The algorithm, with its helper (without one the result is probably wrong).</summary>
    protected MakeLoops(Helper? helper)
    {
        _helper = helper;
    }

    /// <summary>Resets the algorithm to its initial state, with a helper.</summary>
    public void Reset(Helper? helper)
    {
        _helper = helper;
        _mapLink.Clear();
        _loops.Clear();
        _startIndices.Clear();
    }

    /// <summary>Adds a link; its flags say which orientations are allowed (added to those of an equal link already there).</summary>
    public void AddLink(in Link link)
    {
        if (link.node1 == link.node2)
        {
            return;
        }

        int ind = _mapLink.Add(link) + 1;
        Link aLink = _mapLink[ind - 1];
        aLink.flags |= link.flags;
        _mapLink.Substitute(ind - 1, aLink);
    }

    /// <summary>Replaces a link with another (e.g. to change the order of its nodes); the link is added if not there.</summary>
    public void ReplaceLink(in Link link, in Link newLink)
    {
        if (newLink.node1 == newLink.node2)
        {
            return;
        }

        int ind = _mapLink.Add(link) + 1;
        if (ind > 0)
        {
            var aLink = new Link();
            // replace with a null link first (workaround exception)
            _mapLink.Substitute(ind - 1, aLink);
            aLink = newLink;
            // and now put there the final value of link
            _mapLink.Substitute(ind - 1, aLink);
        }
    }

    /// <summary>
    /// Sets the orientation flags of a link added earlier (<see cref="LinkFlag.LF_None"/> excludes it) and returns the old
    /// orientation.
    /// </summary>
    public LinkFlag SetLinkOrientation(in Link link, LinkFlag orient)
    {
        int ind = _mapLink.IndexOf(link) + 1;
        LinkFlag ori = LF_None;
        if (ind > 0)
        {
            Link aLink = _mapLink[ind - 1];
            ori = (LinkFlag)(aLink.flags & (int)LF_Both);
            aLink.flags = (int)orient;
            _mapLink.Substitute(ind - 1, aLink);
        }

        return ori;
    }

    /// <summary>The stored link equal to <paramref name="link"/>, or a null link.</summary>
    public Link FindLink(in Link link)
    {
        int ind = _mapLink.IndexOf(link) + 1;
        var aLink = new Link();
        if (ind > 0)
        {
            aLink = _mapLink[ind - 1];
        }

        return aLink;
    }

    /// <summary>Makes the loops; returns a combination of <see cref="ResultCode"/>s.</summary>
    public int Perform()
    {
        // prepare the set of start indices
        _startIndices.Clear();
        int i;
        for (i = 1; i <= _mapLink.Count; i++)
        {
            Link link = _mapLink[i - 1];
            if ((link.flags & (int)LF_Fwd) != 0)
            {
                _startIndices.Add(i);
            }

            if ((link.flags & (int)LF_Rev) != 0)
            {
                _startIndices.Add(-i);
            }
        }

        int result = 0;

        // two pass loop
        for (int passNum = 0; passNum < 2; passNum++)
        {
            _hangIndices.Clear();
            // main loop
            while (!_startIndices.IsEmpty())
            {
                int indexS = _startIndices.Top();

                var contour = new IndexedSet<int>();
                int startNumber = FindContour(indexS, contour);
                if (startNumber == 0)
                { // error
                    result |= (int)ResultCode.RC_Failure;
                    return result;
                }

                if (startNumber <= contour.Count)
                {
                    // there is a closed loop in the contour
                    AcceptContour(contour, startNumber);
                }

                if (startNumber > 1)
                {
                    // it is required to mark hanging edges
                    int node;
                    if (startNumber <= contour.Count)
                    {
                        // mark hanging edges starting from the first one till a bifurcation
                        node = GetFirstNode(indexS);
                    }
                    else
                    {
                        // open contour - mark from the end back till a bifurcation
                        indexS = contour[startNumber - 2];
                        node = GetLastNode(indexS);
                    }

                    MarkHangChain(node, indexS);
                }
            }

            if (passNum == 0)
            {
                // move hanging links to start indices to make the second pass
                var it = new NCollection_PackedMapOfInteger.Iterator(_hangIndices);
                for (; it.More(); it.Next())
                {
                    _startIndices.Add(it.Key());
                }
            }
        }

        if (_loops.Length() > 0)
        {
            result |= (int)ResultCode.RC_LoopsDone;
        }

        if (!_hangIndices.IsEmpty())
        {
            result |= (int)ResultCode.RC_HangingLinks;
        }

        return result;
    }

    /// <summary>The number of loops found.</summary>
    public int GetNbLoops() => _loops.Length();

    /// <summary>The loop at an index from 1: a list of oriented links.</summary>
    public NCollection_List<Link> GetLoop(int index) => _loops.Value(index);

    /// <summary>The number of hanging links found.</summary>
    public int GetNbHanging() => _hangIndices.Extent();

    /// <summary>Appends the hanging links, oriented, to <paramref name="links"/>.</summary>
    public void GetHangingLinks(NCollection_List<Link> links)
    {
        var it = new NCollection_PackedMapOfInteger.Iterator(_hangIndices);
        for (; it.More(); it.Next())
        {
            int indexS = it.Key();
            Link link = _mapLink[Math.Abs(indexS) - 1];
            if (indexS < 0)
            {
                link.Reverse();
            }

            links.Append(link);
        }
    }

    /// <summary>Chooses the leftmost way from <paramref name="node"/> among <paramref name="lstIndS"/> (signed link indices).</summary>
    protected abstract int ChooseLeftWay(int node, int segIndex, NCollection_List<int> lstIndS);

    /// <summary>The helper.</summary>
    protected Helper? GetHelper() => _helper;

    /// <summary>The link of a signed index, reversed if the index is negative.</summary>
    protected Link GetLink(int segIndex)
    {
        Link link = _mapLink[Math.Abs(segIndex) - 1];
        if (segIndex < 0)
        {
            link.Reverse();
        }

        return link;
    }

    private int FindContour(int indexS, IndexedSet<int> contour)
    {
        contour.Clear();
        int startIndex = 0;
        int aIndexS = indexS;
        var nodeLink = new Dictionary<int, int>();
        int lastNode = GetLastNode(aIndexS);

        for (;;)
        {
            contour.Add(aIndexS);
            nodeLink[GetFirstNode(aIndexS)] = aIndexS;

            int index = Math.Abs(aIndexS);

            // collect the list of links from this node able to participate
            // in this contour
            var lstIndS = new NCollection_List<int>();
            NCollection_List<Link> links = _helper!.GetAdjacentLinks(lastNode);
            var itLinks = new NCollection_List<Link>.Iterator(links);
            for (; itLinks.More(); itLinks.Next())
            {
                int ind = _mapLink.IndexOf(itLinks.Value()) + 1;
                if (ind == 0 || ind == index)
                {
                    continue;
                }

                // determine the orientation in which the link is to be taken
                int indS = ind;
                int node1 = GetFirstNode(ind);
                if (node1 != lastNode)
                {
                    indS = -indS;
                }

                if (CanLinkBeTaken(indS))
                {
                    lstIndS.Append(indS);
                }
            }

            if (lstIndS.IsEmpty())
            {
                // no more ways: open contour
                startIndex = contour.Count + 1;
                break;
            }

            int indexSNext = 0;
            if (lstIndS.First() == lstIndS.Last())
            {
                // only one possible way
                indexSNext = lstIndS.First();
            }
            else
            {
                // find the most left way
                indexSNext = ChooseLeftWay(lastNode, aIndexS, lstIndS);
            }

            aIndexS = indexSNext;

            if (aIndexS == 0)
            {
                // no more ways: open contour
                startIndex = contour.Count + 1;
                break;
            }

            if (contour.Contains(aIndexS))
            {
                // entering the loop second time, stop search
                startIndex = contour.IndexOf(aIndexS) + 1;
                break;
            }

            if (contour.Contains(-aIndexS))
            {
                // leaving the loop, stop search
                startIndex = contour.IndexOf(-aIndexS) + 1 + 1;
                break;
            }

            lastNode = GetLastNode(aIndexS);
            if (nodeLink.ContainsKey(lastNode))
            {
                // closing the loop, stop search
                contour.Add(aIndexS);
                startIndex = contour.IndexOf(nodeLink.GetValue(lastNode)) + 1;
                break;
            }
        }

        return startIndex;
    }

    private void AcceptContour(IndexedSet<int> contour, int startNumber)
    {
        // append a new loop to the result
        var loop = new NCollection_List<Link>();
        _loops.Append(loop);

        // build a loop, mark links as taken,
        // remove them from the set of start indices
        int i;
        for (i = startNumber; i <= contour.Count; i++)
        {
            int indexS = contour[i - 1]; // index with sign
            int index = Math.Abs(indexS);
            Link link = _mapLink[index - 1];
            Link orientedLink = link;
            if (indexS < 0)
            {
                orientedLink.Reverse();
            }

            loop.Append(orientedLink);
            // remove from start set
            _startIndices.Remove(indexS);
        }
    }

    private int GetFirstNode(int indexS)
    {
        int index = Math.Abs(indexS);
        Link link = _mapLink[index - 1];
        if (indexS > 0)
        {
            return link.node1;
        }

        return link.node2;
    }

    private int GetLastNode(int indexS)
    {
        int index = Math.Abs(indexS);
        Link link = _mapLink[index - 1];
        if (indexS > 0)
        {
            return link.node2;
        }

        return link.node1;
    }

    private void MarkHangChain(int node, int indexS)
    {
        int node1 = node;
        int aIndexS = indexS;
        int index = Math.Abs(aIndexS);
        bool isOut = (node1 == GetFirstNode(aIndexS));
        for (;;)
        {
            // check if the current link is hanging:
            // if it is outcoming from aNode1 then count the number of
            // other incoming links and vice-versa;
            // if the number is zero than it is hanging
            NCollection_List<Link> links = _helper!.GetAdjacentLinks(node1);
            int nEdges = 0;
            var itLinks = new NCollection_List<Link>.Iterator(links);
            for (; itLinks.More() && nEdges == 0; itLinks.Next())
            {
                Link l = itLinks.Value();
                int ind = _mapLink.IndexOf(l) + 1;
                if (ind == 0 || ind == index)
                {
                    continue;
                }

                if ((isOut && node1 == l.node1) || (!isOut && node1 == l.node2))
                {
                    ind = -ind;
                }

                if (CanLinkBeTaken(ind))
                {
                    nEdges++;
                }
            }

            if (nEdges > 0)
            {
                // leave this chain
                break;
            }

            // mark the current link as hanging
            _startIndices.Remove(aIndexS);
            _hangIndices.Add(aIndexS);

            // get other node of the link and the next link
            if (isOut)
            {
                node1 = GetLastNode(aIndexS);
            }
            else
            {
                node1 = GetFirstNode(aIndexS);
            }

            NCollection_List<Link> nextLinks = _helper.GetAdjacentLinks(node1);
            int nextIndexS = 0;
            for (itLinks.Initialize(nextLinks); itLinks.More(); itLinks.Next())
            {
                Link l = itLinks.Value();
                int ind = _mapLink.IndexOf(l) + 1;
                if (ind == 0 || ind == index)
                {
                    continue;
                }

                if ((isOut && node1 == l.node2) || (!isOut && node1 == l.node1))
                {
                    ind = -ind;
                }

                if (CanLinkBeTaken(ind))
                {
                    if (nextIndexS == 0)
                    {
                        nextIndexS = ind;
                    }
                    else
                    {
                        // more than 1 ways, stop the chain
                        nextIndexS = 0;
                        break;
                    }
                }
            }

            if (nextIndexS == 0)
            {
                break;
            }

            aIndexS = nextIndexS;
            index = Math.Abs(aIndexS);
        }
    }

    private bool CanLinkBeTaken(int indexS) => _startIndices.Contains(indexS);
}

/// <summary>Poly_MakeLoops in 3D: the leftmost way is measured around the surface normal at the node.</summary>
internal class MakeLoops3D : MakeLoops
{
    /// <summary>The helper: tangents of the links and normals at the nodes. Each method returns false if it can't.</summary>
    public new abstract class Helper : MakeLoops.Helper
    {
        /// <summary>The tangent at the first node of a link.</summary>
        public abstract bool GetFirstTangent(in Link link, out Direction3D dir);

        /// <summary>The tangent at the last node of a link.</summary>
        public abstract bool GetLastTangent(in Link link, out Direction3D dir);

        /// <summary>The normal to the surface at a node.</summary>
        public abstract bool GetNormal(int node, out Direction3D dir);
    }

    /// <summary>The algorithm, with its helper (without one the result is probably wrong).</summary>
    public MakeLoops3D(Helper? helper) : base(helper) { }

    protected override int ChooseLeftWay(int node, int segIndex, NCollection_List<int> lstIndS)
    {
        double angleMin = M_PI * 2;
        Helper helper = GetHelper()!;
        if (!helper.GetNormal(node, out Direction3D normal))
        {
            return lstIndS.First();
        }

        Link link = GetLink(segIndex);
        if (!helper.GetLastTangent(link, out Direction3D tgtRef))
        {
            return lstIndS.First();
        }

        // project tangent vector to the plane orthogonal to normal
        // to get the reference direction
        Xyz tgtRefXYZ = normal.XYZ().CrossCrossed(tgtRef.XYZ(), normal.XYZ());
        if (tgtRefXYZ.SquareModulus() < 1e-14)
        {
            // a problem with defining reference direction, take first way
            return lstIndS.First();
        }

        tgtRef = new Direction3D(tgtRefXYZ);

        // find the way with minimal angle to the reference direction
        // (the angle is in range ]-PI;PI])
        int resIndex = 0;
        var itI = new NCollection_List<int>.Iterator(lstIndS);
        for (; itI.More(); itI.Next())
        {
            int indS = itI.Value();

            link = GetLink(indS);
            if (!helper.GetFirstTangent(link, out Direction3D tgt))
            {
                continue;
            }

            Xyz tgtXYZ = normal.XYZ().CrossCrossed(tgt.XYZ(), normal.XYZ());
            if (tgtXYZ.SquareModulus() < 1e-14)
            {
                // skip a problem way
                continue;
            }

            tgt = new Direction3D(tgtXYZ);

            double angle = tgt.AngleWithRef(tgtRef, normal);
            if (angle < 1e-4 - M_PI)
            {
                angle = M_PI;
            }

            if (angle < angleMin)
            {
                angleMin = angle;
                resIndex = indS;
            }
        }

        return resIndex == 0 ? lstIndS.First() : resIndex;
    }

    /// <summary>The helper.</summary>
    protected new Helper? GetHelper() => (Helper?)base.GetHelper();
}

/// <summary>Poly_MakeLoops in 2D: the leftmost (or rightmost) way is measured in the plane.</summary>
internal class MakeLoops2D : MakeLoops
{
    /// <summary>The helper: tangents of the links. Each method returns false if it can't.</summary>
    public new abstract class Helper : MakeLoops.Helper
    {
        /// <summary>The tangent at the first node of a link.</summary>
        public abstract bool GetFirstTangent(in Link link, out Direction2D dir);

        /// <summary>The tangent at the last node of a link.</summary>
        public abstract bool GetLastTangent(in Link link, out Direction2D dir);
    }

    // this flag says that chooseLeftWay must choose the right way instead
    private readonly bool _rightWay;

    /// <summary>The algorithm, choosing the leftmost way, or the rightmost if <paramref name="leftWay"/> is false.</summary>
    public MakeLoops2D(bool leftWay, Helper? helper) : base(helper)
    {
        _rightWay = !leftWay;
    }

    protected override int ChooseLeftWay(int node, int segIndex, NCollection_List<int> lstIndS)
    {
        double angleMin = M_PI * 2;
        Helper helper = GetHelper()!;
        Link link = GetLink(segIndex);
        if (!helper.GetLastTangent(link, out Direction2D tgtRef))
        {
            // a problem with defining reference direction, take first way
            return lstIndS.First();
        }

        // find the way with minimal angle to the reference direction
        // (the angle is in range ]-PI;PI])
        int resIndex = 0;
        var itI = new NCollection_List<int>.Iterator(lstIndS);
        for (; itI.More(); itI.Next())
        {
            int indS = itI.Value();

            link = GetLink(indS);
            if (!helper.GetFirstTangent(link, out Direction2D tgt))
            {
                // skip a problem way
                continue;
            }

            double angle = tgt.AngleTo(tgtRef);
            if (_rightWay)
            {
                angle = -angle;
            }

            if (angle < 1e-4 - M_PI)
            {
                angle = M_PI;
            }

            if (angle < angleMin)
            {
                angleMin = angle;
                resIndex = indS;
            }
        }

        return resIndex == 0 ? lstIndS.First() : resIndex;
    }

    /// <summary>The helper.</summary>
    protected new Helper? GetHelper() => (Helper?)base.GetHelper();
}
