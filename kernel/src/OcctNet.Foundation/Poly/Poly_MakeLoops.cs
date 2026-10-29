// Port of: src/FoundationClasses/TKMath/Poly/Poly_MakeLoops.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_MakeLoops.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2009-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.Poly_MakeLoops.LinkFlag;

namespace Occt;

/// <summary>
/// Makes loops from a set of connected links, each a pair of node indices. A subclass chooses the way at a fork
/// (<see cref="Poly_MakeLoops3D"/>, <see cref="Poly_MakeLoops2D"/>), with a <see cref="Helper"/> giving the links
/// at each node.
/// </summary>
/// <remarks>
/// An algorithm object, a class as in OCCT. Allocator arguments are dropped, and OCCT's debug hook
/// (<see cref="Helper.OnAddLink"/>, called only in OCCT's debug builds) isn't called. The set of start links is OCCT's
/// packed integer map, ported with its iteration order, so loops are found in OCCT's order.
/// </remarks>
public abstract class Poly_MakeLoops
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
            Link aCopy = this;
            aCopy.Reverse();
            return aCopy;
        }

        public readonly bool IsReversed() => (flags & (int)LF_Reversed) != 0;

        /// <summary>Makes this link null.</summary>
        public void Nullify() => node1 = node2 = 0;

        public readonly bool IsNull() => node1 == 0 || node2 == 0;

        public readonly bool Equals(Link theOther)
            => (theOther.node1 == node1 && theOther.node2 == node2) || (theOther.node1 == node2 && theOther.node2 == node1);

        public override readonly bool Equals(object? obj) => obj is Link other && Equals(other);

        /// <summary>A hash of the two nodes, whatever their order.</summary>
        public override readonly int GetHashCode()
            => node1 < node2 ? HashCode.Combine(node1, node2) : HashCode.Combine(node2, node1);

        public static bool operator ==(in Link theLeft, in Link theRight) => theLeft.Equals(theRight);

        public static bool operator !=(in Link theLeft, in Link theRight) => !theLeft.Equals(theRight);

        public override readonly string ToString()
            => string.Create(CultureInfo.InvariantCulture, $"Poly_MakeLoops.Link({node1}, {node2}, flags {flags})");
    }

    /// <summary>The helper: gives the links at a node.</summary>
    public abstract class Helper
    {
        /// <summary>The links adjacent to a node.</summary>
        public abstract NCollection_List<Link> GetAdjacentLinks(int theNode);

        /// <summary>A hook OCCT calls from AddLink in its debug builds only; never called by the port.</summary>
        public virtual void OnAddLink(int theNum, in Link theLink) { }
    }

    /// <summary>
    /// A heap of integers, best used by adding all items first, then taking the top item and removing items until it
    /// is empty. The top is the first item in OCCT's packed map order.
    /// </summary>
    public sealed class HeapOfInteger
    {
        private readonly NCollection_PackedMapOfInteger myMap;
        private readonly NCollection_PackedMapOfInteger.Iterator myIter = new();
        private bool myIterReady;

        public HeapOfInteger(int theNbPreAllocated = 1)
        {
            myMap = new NCollection_PackedMapOfInteger(theNbPreAllocated);
            myIterReady = false;
        }

        public void Clear()
        {
            myMap.Clear();
            myIterReady = false;
        }

        public void Add(int theValue)
        {
            myMap.Add(theValue);
            myIterReady = false;
        }

        public int Top()
        {
            if (!myIterReady)
            {
                myIter.Initialize(myMap);
                myIterReady = true;
            }

            return myIter.Key();
        }

        public bool Contains(int theValue) => myMap.Contains(theValue);

        public void Remove(int theValue)
        {
            if (myIterReady && myIter.More() && myIter.Key() == theValue)
            {
                myIter.Next();
            }

            myMap.Remove(theValue);
        }

        public bool IsEmpty()
        {
            if (!myIterReady)
            {
                myIter.Initialize(myMap);
                myIterReady = true;
            }

            return !myIter.More();
        }
    }

    /// <summary>The result codes of <see cref="Perform"/>, combined.</summary>
    public enum ResultCode
    {
        RC_LoopsDone = 1,
        RC_HangingLinks = 2,
        RC_Failure = 4,
    }

    private Helper? myHelper;
    private readonly NCollection_IndexedMap<Link> myMapLink = new();
    private readonly NCollection_Sequence<NCollection_List<Link>> myLoops = new();
    private readonly HeapOfInteger myStartIndices = new(4000);
    private readonly NCollection_PackedMapOfInteger myHangIndices = new();

    /// <summary>The algorithm, with its helper (without one the result is probably wrong).</summary>
    protected Poly_MakeLoops(Helper? theHelper)
    {
        myHelper = theHelper;
    }

    /// <summary>Resets the algorithm to its initial state, with a helper.</summary>
    public void Reset(Helper? theHelper)
    {
        myHelper = theHelper;
        myMapLink.Clear();
        myLoops.Clear();
        myStartIndices.Clear();
    }

    /// <summary>Adds a link; its flags say which orientations are allowed (added to those of an equal link already there).</summary>
    public void AddLink(in Link theLink)
    {
        if (theLink.node1 == theLink.node2)
        {
            return;
        }

        int aInd = myMapLink.Add(theLink);
        Link aLink = myMapLink[aInd];
        aLink.flags |= theLink.flags;
        myMapLink.Substitute(aInd, aLink);
    }

    /// <summary>Replaces a link with another (e.g. to change the order of its nodes); the link is added if not there.</summary>
    public void ReplaceLink(in Link theLink, in Link theNewLink)
    {
        if (theNewLink.node1 == theNewLink.node2)
        {
            return;
        }

        int aInd = myMapLink.Add(theLink);
        if (aInd > 0)
        {
            var aLink = new Link();
            // replace with a null link first (workaround exception)
            myMapLink.Substitute(aInd, aLink);
            aLink = theNewLink;
            // and now put there the final value of link
            myMapLink.Substitute(aInd, aLink);
        }
    }

    /// <summary>
    /// Sets the orientation flags of a link added earlier (<see cref="LinkFlag.LF_None"/> excludes it) and returns the old
    /// orientation.
    /// </summary>
    public LinkFlag SetLinkOrientation(in Link theLink, LinkFlag theOrient)
    {
        int aInd = myMapLink.FindIndex(theLink);
        LinkFlag aOri = LF_None;
        if (aInd > 0)
        {
            Link aLink = myMapLink[aInd];
            aOri = (LinkFlag)(aLink.flags & (int)LF_Both);
            aLink.flags = (int)theOrient;
            myMapLink.Substitute(aInd, aLink);
        }

        return aOri;
    }

    /// <summary>The stored link equal to <paramref name="theLink"/>, or a null link.</summary>
    public Link FindLink(in Link theLink)
    {
        int aInd = myMapLink.FindIndex(theLink);
        var aLink = new Link();
        if (aInd > 0)
        {
            aLink = myMapLink[aInd];
        }

        return aLink;
    }

    /// <summary>Makes the loops; returns a combination of <see cref="ResultCode"/>s.</summary>
    public int Perform()
    {
        // prepare the set of start indices
        myStartIndices.Clear();
        int i;
        for (i = 1; i <= myMapLink.Extent(); i++)
        {
            Link aLink = myMapLink[i];
            if ((aLink.flags & (int)LF_Fwd) != 0)
            {
                myStartIndices.Add(i);
            }

            if ((aLink.flags & (int)LF_Rev) != 0)
            {
                myStartIndices.Add(-i);
            }
        }

        int aResult = 0;

        // two pass loop
        for (int aPassNum = 0; aPassNum < 2; aPassNum++)
        {
            myHangIndices.Clear();
            // main loop
            while (!myStartIndices.IsEmpty())
            {
                int aIndexS = myStartIndices.Top();

                var aContour = new NCollection_IndexedMap<int>();
                int aStartNumber = findContour(aIndexS, aContour);
                if (aStartNumber == 0)
                { // error
                    aResult |= (int)ResultCode.RC_Failure;
                    return aResult;
                }

                if (aStartNumber <= aContour.Extent())
                {
                    // there is a closed loop in the contour
                    acceptContour(aContour, aStartNumber);
                }

                if (aStartNumber > 1)
                {
                    // it is required to mark hanging edges
                    int aNode;
                    if (aStartNumber <= aContour.Extent())
                    {
                        // mark hanging edges starting from the first one till a bifurcation
                        aNode = getFirstNode(aIndexS);
                    }
                    else
                    {
                        // open contour - mark from the end back till a bifurcation
                        aIndexS = aContour[aStartNumber - 1];
                        aNode = getLastNode(aIndexS);
                    }

                    markHangChain(aNode, aIndexS);
                }
            }

            if (aPassNum == 0)
            {
                // move hanging links to start indices to make the second pass
                var it = new NCollection_PackedMapOfInteger.Iterator(myHangIndices);
                for (; it.More(); it.Next())
                {
                    myStartIndices.Add(it.Key());
                }
            }
        }

        if (myLoops.Length() > 0)
        {
            aResult |= (int)ResultCode.RC_LoopsDone;
        }

        if (!myHangIndices.IsEmpty())
        {
            aResult |= (int)ResultCode.RC_HangingLinks;
        }

        return aResult;
    }

    /// <summary>The number of loops found.</summary>
    public int GetNbLoops() => myLoops.Length();

    /// <summary>The loop at an index from 1: a list of oriented links.</summary>
    public NCollection_List<Link> GetLoop(int theIndex) => myLoops.Value(theIndex);

    /// <summary>The number of hanging links found.</summary>
    public int GetNbHanging() => myHangIndices.Extent();

    /// <summary>Appends the hanging links, oriented, to <paramref name="theLinks"/>.</summary>
    public void GetHangingLinks(NCollection_List<Link> theLinks)
    {
        var it = new NCollection_PackedMapOfInteger.Iterator(myHangIndices);
        for (; it.More(); it.Next())
        {
            int aIndexS = it.Key();
            Link aLink = myMapLink[Math.Abs(aIndexS)];
            if (aIndexS < 0)
            {
                aLink.Reverse();
            }

            theLinks.Append(aLink);
        }
    }

    /// <summary>Chooses the leftmost way from <paramref name="theNode"/> among <paramref name="theLstIndS"/> (signed link indices).</summary>
    protected abstract int chooseLeftWay(int theNode, int theSegIndex, NCollection_List<int> theLstIndS);

    /// <summary>The helper.</summary>
    protected Helper? getHelper() => myHelper;

    /// <summary>The link of a signed index, reversed if the index is negative.</summary>
    protected Link getLink(int theSegIndex)
    {
        Link aLink = myMapLink[Math.Abs(theSegIndex)];
        if (theSegIndex < 0)
        {
            aLink.Reverse();
        }

        return aLink;
    }

    private int findContour(int theIndexS, NCollection_IndexedMap<int> theContour)
    {
        theContour.Clear();
        int aStartIndex = 0;
        int aIndexS = theIndexS;
        var aNodeLink = new NCollection_DataMap<int, int>();
        int aLastNode = getLastNode(aIndexS);

        for (;;)
        {
            theContour.Add(aIndexS);
            aNodeLink.Bind(getFirstNode(aIndexS), aIndexS);

            int aIndex = Math.Abs(aIndexS);

            // collect the list of links from this node able to participate
            // in this contour
            var aLstIndS = new NCollection_List<int>();
            NCollection_List<Link> aLinks = myHelper!.GetAdjacentLinks(aLastNode);
            var itLinks = new NCollection_List<Link>.Iterator(aLinks);
            for (; itLinks.More(); itLinks.Next())
            {
                int aInd = myMapLink.FindIndex(itLinks.Value());
                if (aInd == 0 || aInd == aIndex)
                {
                    continue;
                }

                // determine the orientation in which the link is to be taken
                int aIndS = aInd;
                int aNode1 = getFirstNode(aInd);
                if (aNode1 != aLastNode)
                {
                    aIndS = -aIndS;
                }

                if (canLinkBeTaken(aIndS))
                {
                    aLstIndS.Append(aIndS);
                }
            }

            if (aLstIndS.IsEmpty())
            {
                // no more ways: open contour
                aStartIndex = theContour.Extent() + 1;
                break;
            }

            int aIndexSNext = 0;
            if (aLstIndS.First() == aLstIndS.Last())
            {
                // only one possible way
                aIndexSNext = aLstIndS.First();
            }
            else
            {
                // find the most left way
                aIndexSNext = chooseLeftWay(aLastNode, aIndexS, aLstIndS);
            }

            aIndexS = aIndexSNext;

            if (aIndexS == 0)
            {
                // no more ways: open contour
                aStartIndex = theContour.Extent() + 1;
                break;
            }

            if (theContour.Contains(aIndexS))
            {
                // entering the loop second time, stop search
                aStartIndex = theContour.FindIndex(aIndexS);
                break;
            }

            if (theContour.Contains(-aIndexS))
            {
                // leaving the loop, stop search
                aStartIndex = theContour.FindIndex(-aIndexS) + 1;
                break;
            }

            aLastNode = getLastNode(aIndexS);
            if (aNodeLink.IsBound(aLastNode))
            {
                // closing the loop, stop search
                theContour.Add(aIndexS);
                aStartIndex = theContour.FindIndex(aNodeLink.Find(aLastNode));
                break;
            }
        }

        return aStartIndex;
    }

    private void acceptContour(NCollection_IndexedMap<int> theContour, int theStartNumber)
    {
        // append a new loop to the result
        var aLoop = new NCollection_List<Link>();
        myLoops.Append(aLoop);

        // build a loop, mark links as taken,
        // remove them from the set of start indices
        int i;
        for (i = theStartNumber; i <= theContour.Extent(); i++)
        {
            int aIndexS = theContour[i]; // index with sign
            int aIndex = Math.Abs(aIndexS);
            Link aLink = myMapLink[aIndex];
            Link aOrientedLink = aLink;
            if (aIndexS < 0)
            {
                aOrientedLink.Reverse();
            }

            aLoop.Append(aOrientedLink);
            // remove from start set
            myStartIndices.Remove(aIndexS);
        }
    }

    private int getFirstNode(int theIndexS)
    {
        int aIndex = Math.Abs(theIndexS);
        Link aLink = myMapLink[aIndex];
        if (theIndexS > 0)
        {
            return aLink.node1;
        }

        return aLink.node2;
    }

    private int getLastNode(int theIndexS)
    {
        int aIndex = Math.Abs(theIndexS);
        Link aLink = myMapLink[aIndex];
        if (theIndexS > 0)
        {
            return aLink.node2;
        }

        return aLink.node1;
    }

    private void markHangChain(int theNode, int theIndexS)
    {
        int aNode1 = theNode;
        int aIndexS = theIndexS;
        int aIndex = Math.Abs(aIndexS);
        bool isOut = (aNode1 == getFirstNode(aIndexS));
        for (;;)
        {
            // check if the current link is hanging:
            // if it is outcoming from aNode1 then count the number of
            // other incoming links and vice-versa;
            // if the number is zero than it is hanging
            NCollection_List<Link> aLinks = myHelper!.GetAdjacentLinks(aNode1);
            int nEdges = 0;
            var itLinks = new NCollection_List<Link>.Iterator(aLinks);
            for (; itLinks.More() && nEdges == 0; itLinks.Next())
            {
                Link aL = itLinks.Value();
                int aInd = myMapLink.FindIndex(aL);
                if (aInd == 0 || aInd == aIndex)
                {
                    continue;
                }

                if ((isOut && aNode1 == aL.node1) || (!isOut && aNode1 == aL.node2))
                {
                    aInd = -aInd;
                }

                if (canLinkBeTaken(aInd))
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
            myStartIndices.Remove(aIndexS);
            myHangIndices.Add(aIndexS);

            // get other node of the link and the next link
            if (isOut)
            {
                aNode1 = getLastNode(aIndexS);
            }
            else
            {
                aNode1 = getFirstNode(aIndexS);
            }

            NCollection_List<Link> aNextLinks = myHelper.GetAdjacentLinks(aNode1);
            int aNextIndexS = 0;
            for (itLinks.Initialize(aNextLinks); itLinks.More(); itLinks.Next())
            {
                Link aL = itLinks.Value();
                int aInd = myMapLink.FindIndex(aL);
                if (aInd == 0 || aInd == aIndex)
                {
                    continue;
                }

                if ((isOut && aNode1 == aL.node2) || (!isOut && aNode1 == aL.node1))
                {
                    aInd = -aInd;
                }

                if (canLinkBeTaken(aInd))
                {
                    if (aNextIndexS == 0)
                    {
                        aNextIndexS = aInd;
                    }
                    else
                    {
                        // more than 1 ways, stop the chain
                        aNextIndexS = 0;
                        break;
                    }
                }
            }

            if (aNextIndexS == 0)
            {
                break;
            }

            aIndexS = aNextIndexS;
            aIndex = Math.Abs(aIndexS);
        }
    }

    private bool canLinkBeTaken(int theIndexS) => myStartIndices.Contains(theIndexS);
}

/// <summary>Poly_MakeLoops in 3D: the leftmost way is measured around the surface normal at the node.</summary>
public class Poly_MakeLoops3D : Poly_MakeLoops
{
    /// <summary>The helper: tangents of the links and normals at the nodes. Each method returns false if it can't.</summary>
    public new abstract class Helper : Poly_MakeLoops.Helper
    {
        /// <summary>The tangent at the first node of a link.</summary>
        public abstract bool GetFirstTangent(in Link theLink, out gp_Dir theDir);

        /// <summary>The tangent at the last node of a link.</summary>
        public abstract bool GetLastTangent(in Link theLink, out gp_Dir theDir);

        /// <summary>The normal to the surface at a node.</summary>
        public abstract bool GetNormal(int theNode, out gp_Dir theDir);
    }

    /// <summary>The algorithm, with its helper (without one the result is probably wrong).</summary>
    public Poly_MakeLoops3D(Helper? theHelper) : base(theHelper) { }

    protected override int chooseLeftWay(int theNode, int theSegIndex, NCollection_List<int> theLstIndS)
    {
        double aAngleMin = M_PI * 2;
        Helper aHelper = getHelper()!;
        if (!aHelper.GetNormal(theNode, out gp_Dir aNormal))
        {
            return theLstIndS.First();
        }

        Link aLink = getLink(theSegIndex);
        if (!aHelper.GetLastTangent(aLink, out gp_Dir aTgtRef))
        {
            return theLstIndS.First();
        }

        // project tangent vector to the plane orthogonal to normal
        // to get the reference direction
        gp_XYZ aTgtRefXYZ = aNormal.XYZ().CrossCrossed(aTgtRef.XYZ(), aNormal.XYZ());
        if (aTgtRefXYZ.SquareModulus() < 1e-14)
        {
            // a problem with defining reference direction, take first way
            return theLstIndS.First();
        }

        aTgtRef = new gp_Dir(aTgtRefXYZ);

        // find the way with minimal angle to the reference direction
        // (the angle is in range ]-PI;PI])
        int aResIndex = 0;
        var aItI = new NCollection_List<int>.Iterator(theLstIndS);
        for (; aItI.More(); aItI.Next())
        {
            int aIndS = aItI.Value();

            aLink = getLink(aIndS);
            if (!aHelper.GetFirstTangent(aLink, out gp_Dir aTgt))
            {
                continue;
            }

            gp_XYZ aTgtXYZ = aNormal.XYZ().CrossCrossed(aTgt.XYZ(), aNormal.XYZ());
            if (aTgtXYZ.SquareModulus() < 1e-14)
            {
                // skip a problem way
                continue;
            }

            aTgt = new gp_Dir(aTgtXYZ);

            double aAngle = aTgt.AngleWithRef(aTgtRef, aNormal);
            if (aAngle < 1e-4 - M_PI)
            {
                aAngle = M_PI;
            }

            if (aAngle < aAngleMin)
            {
                aAngleMin = aAngle;
                aResIndex = aIndS;
            }
        }

        return aResIndex == 0 ? theLstIndS.First() : aResIndex;
    }

    /// <summary>The helper.</summary>
    protected new Helper? getHelper() => (Helper?)base.getHelper();
}

/// <summary>Poly_MakeLoops in 2D: the leftmost (or rightmost) way is measured in the plane.</summary>
public class Poly_MakeLoops2D : Poly_MakeLoops
{
    /// <summary>The helper: tangents of the links. Each method returns false if it can't.</summary>
    public new abstract class Helper : Poly_MakeLoops.Helper
    {
        /// <summary>The tangent at the first node of a link.</summary>
        public abstract bool GetFirstTangent(in Link theLink, out gp_Dir2d theDir);

        /// <summary>The tangent at the last node of a link.</summary>
        public abstract bool GetLastTangent(in Link theLink, out gp_Dir2d theDir);
    }

    // this flag says that chooseLeftWay must choose the right way instead
    private readonly bool myRightWay;

    /// <summary>The algorithm, choosing the leftmost way, or the rightmost if <paramref name="theLeftWay"/> is false.</summary>
    public Poly_MakeLoops2D(bool theLeftWay, Helper? theHelper) : base(theHelper)
    {
        myRightWay = !theLeftWay;
    }

    protected override int chooseLeftWay(int theNode, int theSegIndex, NCollection_List<int> theLstIndS)
    {
        double aAngleMin = M_PI * 2;
        Helper aHelper = getHelper()!;
        Link aLink = getLink(theSegIndex);
        if (!aHelper.GetLastTangent(aLink, out gp_Dir2d aTgtRef))
        {
            // a problem with defining reference direction, take first way
            return theLstIndS.First();
        }

        // find the way with minimal angle to the reference direction
        // (the angle is in range ]-PI;PI])
        int aResIndex = 0;
        var aItI = new NCollection_List<int>.Iterator(theLstIndS);
        for (; aItI.More(); aItI.Next())
        {
            int aIndS = aItI.Value();

            aLink = getLink(aIndS);
            if (!aHelper.GetFirstTangent(aLink, out gp_Dir2d aTgt))
            {
                // skip a problem way
                continue;
            }

            double aAngle = aTgt.Angle(aTgtRef);
            if (myRightWay)
            {
                aAngle = -aAngle;
            }

            if (aAngle < 1e-4 - M_PI)
            {
                aAngle = M_PI;
            }

            if (aAngle < aAngleMin)
            {
                aAngleMin = aAngle;
                aResIndex = aIndS;
            }
        }

        return aResIndex == 0 ? theLstIndS.First() : aResIndex;
    }

    /// <summary>The helper.</summary>
    protected new Helper? getHelper() => (Helper?)base.getHelper();
}
