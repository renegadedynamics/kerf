// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Graph.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Graph.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>The graph of the bisecting locus: its arcs, nodes and basic elements, each indexed from 1.</summary>
/// <remarks>
/// Built by BRepMAT2d_BisectingLocus from MAT2d_Mat2d's result. A working object (decision 21): the queries are
/// public, the building members (Perform, FusionOfBasicElts, CompactArcs, CompactNodes, ChangeBasicElts) internal.
/// The maps are keyed by integers and only looked up (ChangeBasicElts iterates one to set each element's index from
/// its key, which doesn't depend on the order).
/// </remarks>
internal sealed class MedialGraph : Transient
{
    private Dictionary<int, MedialArc> _arcs = new();
    private Dictionary<int, MedialBasicElt> _basicElts = new();
    private readonly Dictionary<int, MedialNode> _nodes = new();
    private int _numberOfArcs;
    private int _numberOfNodes;
    private int _numberOfBasicElts;
    private int _numberOfInfiniteNodes;

    public MedialGraph()
    {
        _numberOfArcs = 0;
        _numberOfNodes = 0;
        _numberOfBasicElts = 0;
        _numberOfInfiniteNodes = 0;
    }

    /// <summary>
    /// Builds the graph from the result of MAT2d_Mat2d's CreateMat: SemiInfinite if some bisectors are infinite,
    /// TheRoots the root bisectors, NbBasicElts the number of basic elements, NbArcs the number of arcs (bisectors).
    /// </summary>
    internal void Perform(bool semiInfinite, MedialListOfBisector theRoots, int nbBasicElts, int nbArcs)
    {
        int nbRoots;
        MedialArc firstArc;
        MedialArc currentArc;
        MedialNode extremite;
        int indTabArcs = 1;
        int indTabNodes;
        int i;
        double distExt;
        int indExt;
        MedialArc previousArc;

        //------------------------
        // Construction of the graph.
        //------------------------
        if (semiInfinite)
        {
            nbRoots = theRoots.Number();
            _numberOfInfiniteNodes = nbRoots;
        }
        else
        {
            nbRoots = 1;
            _numberOfInfiniteNodes = 0;
        }

        _numberOfArcs = nbArcs;
        _numberOfBasicElts = nbBasicElts;
        _numberOfNodes = nbRoots + nbArcs;
        indTabNodes = _numberOfNodes;

        //---------------------------
        //... Creation of the BasicElts.
        //---------------------------
        for (i = 1; i <= nbBasicElts; i++)
        {
            _basicElts[i] = new MedialBasicElt(i);
            _basicElts.GetValue(i).SetGeomIndex(i);
        }

        //--------------------------------------------------------------------
        // ... Creation of the ARCS and the NODES: the trees of arcs from the root <Bisector>s.
        //--------------------------------------------------------------------
        if (semiInfinite)
        {
            // Several entry points at infinity.
            theRoots.First();

            while (theRoots.More())
            {
                currentArc = MakeArc(theRoots.Current(), _basicElts, _arcs, ref indTabArcs);
                extremite = new MedialNode(0, currentArc, global::Kerf.Engine.Foundation.Precision.Infinite());
                extremite.SetIndex(indTabNodes);
                currentArc.SetSecondNode(extremite);
                _nodes[indTabNodes] = extremite;
                theRoots.Next();
                indTabNodes--;
            }
        }
        else
        {
            // A single entry point: a first ARC and the root NODE.
            theRoots.First();
            currentArc = MakeArc(theRoots.Current(), _basicElts, _arcs, ref indTabArcs);
            distExt = theRoots.Current().FirstEdge().Distance();
            indExt = theRoots.Current().EndPoint();

            extremite = new MedialNode(indExt, currentArc, distExt);
            extremite.SetIndex(indTabNodes);
            currentArc.SetSecondNode(extremite);
            _nodes[indTabNodes] = extremite;
            indTabNodes--;

            // The ARCs from the root, with their neighbourhoods at the root node.
            firstArc = currentArc;
            previousArc = firstArc;
            theRoots.Next();

            while (theRoots.More())
            {
                currentArc = MakeArc(theRoots.Current(), _basicElts, _arcs, ref indTabArcs);
                currentArc.SetSecondNode(extremite);
                currentArc.SetNeighbour(MedialSide.Left, extremite, previousArc);
                previousArc.SetNeighbour(MedialSide.Right, extremite, currentArc);

                previousArc = currentArc;
                theRoots.Next();
            }
            firstArc.SetNeighbour(MedialSide.Left, extremite, currentArc);
            currentArc.SetNeighbour(MedialSide.Right, extremite, firstArc);
        }

        // The arcs of the root nodes are up to date; the other nodes.
        UpDateNodes(ref indTabNodes);
    }

    /// <summary>The arc of index Index.</summary>
    public MedialArc Arc(int index) => _arcs.GetValue(index);

    /// <summary>The basic element of index Index.</summary>
    public MedialBasicElt BasicElt(int index) => _basicElts.GetValue(index);

    /// <summary>The node of index Index.</summary>
    public MedialNode Node(int index) => _nodes.GetValue(index);

    public int NumberOfArcs() => _numberOfArcs;

    public int NumberOfNodes() => _numberOfNodes;

    public int NumberOfInfiniteNodes() => _numberOfInfiniteNodes;

    public int NumberOfBasicElts() => _numberOfBasicElts;

    /// <summary>
    /// Merges two basic elements: the end of Elt1 (IndexElt1) becomes the end of Elt2 (IndexElt2), Elt2 is replaced
    /// by Elt1 in the arcs and removed. MergeArc1 tells whether two arcs separating the same elements were merged
    /// (GeomIndexArc1 and 2 their geometric indices); MergeArc2 likewise for the start and end arcs of Elt1 on a
    /// closed line (GeomIndexArc3 and 4).
    /// </summary>
    internal void FusionOfBasicElts(int indexElt1, int indexElt2, ref bool mergeArc1, ref int iGeomArc1,
        ref int iGeomArc2, ref bool mergeArc2, ref int iGeomArc3, ref int iGeomArc4)
    {
        MedialBasicElt elt1 = _basicElts.GetValue(indexElt1);
        MedialBasicElt elt2 = _basicElts.GetValue(indexElt2);

        if (elt1 == elt2)
        {
            return;
        }

        int i;
        MedialZone zone2 = new(elt2);

        // The arcs of the zone of Elt2 separate Elt1 and something.
        for (i = 1; i <= zone2.NumberOfArcs(); i++)
        {
            if (zone2.ArcOnFrontier(i).FirstElement() == elt2)
            {
                _arcs.GetValue(zone2.ArcOnFrontier(i).Index()).SetFirstElement(elt1);
            }
            else
            {
                _arcs.GetValue(zone2.ArcOnFrontier(i).Index()).SetSecondElement(elt1);
            }
        }

        // The EndArc of Elt1 and the StartArc of Elt2 can separate the same basic elements: merge the two arcs.
        MedialArc ea1 = elt1.EndArc()!;
        MedialArc sa2 = elt2.StartArc()!;

        MedialBasicElt e1 = ea1.FirstElement();
        MedialBasicElt e2 = ea1.SecondElement();
        MedialBasicElt e3 = sa2.FirstElement();
        MedialBasicElt e4 = sa2.SecondElement();
        mergeArc1 = false;

        if ((e1 == e3 || e1 == e4) && (e2 == e3 || e2 == e4))
        {
            FusionOfArcs(_arcs.GetValue(ea1.Index()), _arcs.GetValue(sa2.Index()));
            mergeArc1 = true;
            iGeomArc1 = ea1.GeomIndex();
            iGeomArc2 = sa2.GeomIndex();
        }

        // The end of Elt1 becomes the end of Elt2.
        elt1.SetEndArc(elt2.EndArc());

        // The EndArc and the StartArc of Elt1 can separate the same basic elements: if the nodes of the arcs are
        // not on the contour, merge the arcs (a closed contour of a single BasicElt); else nothing (a closed
        // contour of two BasicElts).
        MedialArc sa1 = elt1.StartArc()!;
        ea1 = elt1.EndArc()!;

        if (ea1 != sa1)
        {
            e1 = ea1.FirstElement();
            e2 = ea1.SecondElement();
            e3 = sa1.FirstElement();
            e4 = sa1.SecondElement();

            bool onFig = ea1.FirstNode().OnBasicElt() || ea1.SecondNode().OnBasicElt()
                || sa1.FirstNode().OnBasicElt() || sa1.SecondNode().OnBasicElt();

            mergeArc2 = false;

            if ((e1 == e3 || e1 == e4) && (e2 == e3 || e2 == e4) && !onFig)
            {
                FusionOfArcs(_arcs.GetValue(ea1.Index()), _arcs.GetValue(sa1.Index()));
                mergeArc2 = true;
                iGeomArc3 = ea1.GeomIndex();
                iGeomArc4 = sa1.GeomIndex();
            }
        }

        // A basic element is removed.
        _basicElts.Remove(elt2.Index());
        _numberOfBasicElts--;
    }

    // Merges two arcs separating the same elements: Arc1 goes from the second node of Arc2 to its own second node.
    private void FusionOfArcs(MedialArc arc1, MedialArc arc2)
    {
        MedialNode oldNode1 = arc1.FirstNode();
        MedialNode oldNode2 = arc2.FirstNode();

        arc1.SetFirstNode(arc2.SecondNode());

        // The neighbourhoods around the new first node of Arc1.
        if (!arc2.SecondNode().Infinite())
        {
            MedialArc lNeighbour = arc2.Neighbour(arc2.SecondNode(), MedialSide.Left)!;
            MedialArc rNeighbour = arc2.Neighbour(arc2.SecondNode(), MedialSide.Right)!;

            arc1.SetFirstArc(MedialSide.Left, lNeighbour);
            arc1.SetFirstArc(MedialSide.Right, rNeighbour);
            _arcs.GetValue(lNeighbour.Index()).SetNeighbour(MedialSide.Right, arc2.SecondNode(), arc1);
            _arcs.GetValue(rNeighbour.Index()).SetNeighbour(MedialSide.Left, arc2.SecondNode(), arc1);
        }
        else
        {
            arc1.SetFirstArc(MedialSide.Left, null);
            arc1.SetFirstArc(MedialSide.Right, null);
        }

        // The first node of Arc1.
        arc1.FirstNode().SetLinkedArc(arc1);

        // Removal of Arc2 and the old nodes.
        if (_nodes.ContainsKey(oldNode1.Index()))
        {
            _nodes.Remove(oldNode1.Index());
            _numberOfNodes--;
        }
        if (_nodes.ContainsKey(oldNode2.Index()))
        {
            _nodes.Remove(oldNode2.Index());
            _numberOfNodes--;
        }

        MedialArc anArc2 = arc2;
        _arcs.Remove(arc2.Index());
        _numberOfArcs--;

        for (int i = 1; i <= 2; i++)
        {
            MedialBasicElt be;
            if (i == 1)
            {
                be = _basicElts.GetValue(anArc2.FirstElement().Index());
            }
            else
            {
                be = _basicElts.GetValue(anArc2.SecondElement().Index());
            }
            if (be.StartArc() == anArc2)
            {
                be.SetStartArc(arc1);
            }
            if (be.EndArc() == anArc2)
            {
                be.SetEndArc(arc1);
            }
        }
    }

    /// <summary>Shifts the arcs down to fill the holes in their indices.</summary>
    internal void CompactArcs()
    {
        int iFind = 0;
        int i = 1;
        bool yaDecalage = false;

        while (iFind < _numberOfArcs)
        {
            if (!_arcs.ContainsKey(i))
            {
                yaDecalage = true;
            }
            else
            {
                iFind++;
                if (yaDecalage)
                {
                    _arcs.GetValue(i).SetIndex(iFind);
                    _arcs[iFind] = _arcs.GetValue(i);
                    _arcs.Remove(i);
                }
            }
            i++;
        }
    }

    /// <summary>Shifts the nodes down to fill the holes in their indices.</summary>
    internal void CompactNodes()
    {
        int iFind = 0;
        int i = 1;
        bool yaDecalage = false;

        while (iFind < _numberOfNodes)
        {
            if (!_nodes.ContainsKey(i))
            {
                yaDecalage = true;
            }
            else
            {
                iFind++;
                if (yaDecalage)
                {
                    _nodes.GetValue(i).SetIndex(iFind);
                    _nodes[iFind] = _nodes.GetValue(i);
                    _nodes.Remove(i);
                }
            }
            i++;
        }
    }

    /// <summary>Replaces the basic elements by a copy of NewMap, each element taking its key as index.</summary>
    internal void ChangeBasicElts(Dictionary<int, MedialBasicElt> newMap)
    {
        _basicElts = new Dictionary<int, MedialBasicElt>(newMap,newMap.Comparer);
        foreach (KeyValuePair<int, MedialBasicElt> ite in _basicElts)
        {
            ite.Value.SetIndex(ite.Key);
        }
    }

    /// <summary>The basic element of index Index.</summary>
    public MedialBasicElt ChangeBasicElt(int index) => _basicElts.GetValue(index);

    // Binds the first node of each arc in the table of nodes and links it to the arc. The root nodes are bound in
    // Perform.
    private void UpDateNodes(ref int indTabNodes)
    {
        int i;
        MedialNode bout;

        for (i = 1; i <= _numberOfArcs; i++)
        {
            bout = _arcs.GetValue(i).FirstNode();
            _nodes[indTabNodes] = bout;
            bout.SetIndex(indTabNodes);
            indTabNodes--;
            bout.SetLinkedArc(_arcs.GetValue(i));
        }
    }

    // Creates the ARCs of the tree from aBisector.
    private static MedialArc MakeArc(MedialBisector aBisector, Dictionary<int, MedialBasicElt> theBasicElts,
        Dictionary<int, MedialArc> theArcs, ref int indTabArcs)
    {
        MedialArc currentArc;
        MedialArc prevArc;
        MedialArc? nextArc = null;
        MedialNode extremite;
        MedialListOfBisector bisectorList;
        double distExt;

        currentArc = new MedialArc(indTabArcs, aBisector.BisectorNumber(),
            theBasicElts.GetValue(aBisector.FirstEdge().EdgeNumber()),
            theBasicElts.GetValue(aBisector.SecondEdge().EdgeNumber()));
        distExt = aBisector.DistIssuePoint();
        if (distExt == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            distExt = 1.0;
        }
        extremite = new MedialNode(aBisector.IssuePoint(), currentArc, distExt);

        currentArc.SetFirstNode(extremite);
        bisectorList = aBisector.List();
        bisectorList.First();

        if (!bisectorList.More())
        {
            // An arc on the contour.
            theBasicElts.GetValue(aBisector.SecondEdge().EdgeNumber()).SetStartArc(currentArc);
            theBasicElts.GetValue(aBisector.FirstEdge().EdgeNumber()).SetEndArc(currentArc);
        }
        else
        {
            prevArc = currentArc;

            while (bisectorList.More())
            {
                nextArc = MakeArc(bisectorList.Current(), theBasicElts, theArcs, ref indTabArcs);
                nextArc.SetSecondNode(extremite);
                nextArc.SetNeighbour(MedialSide.Left, extremite, prevArc);
                prevArc.SetNeighbour(MedialSide.Right, extremite, nextArc);
                prevArc = nextArc;
                bisectorList.Next();
            }
            currentArc.SetNeighbour(MedialSide.Left, extremite, nextArc);
            nextArc!.SetNeighbour(MedialSide.Right, extremite, currentArc);
        }

        currentArc.SetIndex(indTabArcs);
        theArcs[indTabArcs] = currentArc;
        indTabArcs = indTabArcs + 1;

        return currentArc;
    }
}
