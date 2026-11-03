// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Graph.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Graph.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.MAT_Side;

namespace Occt;

/// <summary>The graph of the bisecting locus: its arcs, nodes and basic elements, each indexed from 1.</summary>
/// <remarks>
/// Built by BRepMAT2d_BisectingLocus from MAT2d_Mat2d's result. A working object (decision 21): the queries are
/// public, the building members (Perform, FusionOfBasicElts, CompactArcs, CompactNodes, ChangeBasicElts) internal.
/// The maps are keyed by integers and only looked up (ChangeBasicElts iterates one to set each element's index from
/// its key, which doesn't depend on the order).
/// </remarks>
public sealed class MAT_Graph : Transient
{
    private NCollection_DataMap<int, MAT_Arc> theArcs = new();
    private NCollection_DataMap<int, MAT_BasicElt> theBasicElts = new();
    private readonly NCollection_DataMap<int, MAT_Node> theNodes = new();
    private int numberOfArcs;
    private int numberOfNodes;
    private int numberOfBasicElts;
    private int numberOfInfiniteNodes;

    public MAT_Graph()
    {
        numberOfArcs = 0;
        numberOfNodes = 0;
        numberOfBasicElts = 0;
        numberOfInfiniteNodes = 0;
    }

    /// <summary>
    /// Builds the graph from the result of MAT2d_Mat2d's CreateMat: SemiInfinite if some bisectors are infinite,
    /// TheRoots the root bisectors, NbBasicElts the number of basic elements, NbArcs the number of arcs (bisectors).
    /// </summary>
    internal void Perform(bool SemiInfinite, MAT_ListOfBisector TheRoots, int NbBasicElts, int NbArcs)
    {
        int NbRoots;
        MAT_Arc FirstArc;
        MAT_Arc CurrentArc;
        MAT_Node Extremite;
        int IndTabArcs = 1;
        int IndTabNodes;
        int i;
        double DistExt;
        int IndExt;
        MAT_Arc PreviousArc;

        //------------------------
        // Construction of the graph.
        //------------------------
        if (SemiInfinite)
        {
            NbRoots = TheRoots.Number();
            numberOfInfiniteNodes = NbRoots;
        }
        else
        {
            NbRoots = 1;
            numberOfInfiniteNodes = 0;
        }

        numberOfArcs = NbArcs;
        numberOfBasicElts = NbBasicElts;
        numberOfNodes = NbRoots + NbArcs;
        IndTabNodes = numberOfNodes;

        //---------------------------
        //... Creation of the BasicElts.
        //---------------------------
        for (i = 1; i <= NbBasicElts; i++)
        {
            theBasicElts.Bind(i, new MAT_BasicElt(i));
            theBasicElts[i].SetGeomIndex(i);
        }

        //--------------------------------------------------------------------
        // ... Creation of the ARCS and the NODES: the trees of arcs from the root <Bisector>s.
        //--------------------------------------------------------------------
        if (SemiInfinite)
        {
            // Several entry points at infinity.
            TheRoots.First();

            while (TheRoots.More())
            {
                CurrentArc = MakeArc(TheRoots.Current(), theBasicElts, theArcs, ref IndTabArcs);
                Extremite = new MAT_Node(0, CurrentArc, global::Kerf.Engine.Foundation.Precision.Infinite());
                Extremite.SetIndex(IndTabNodes);
                CurrentArc.SetSecondNode(Extremite);
                theNodes.Bind(IndTabNodes, Extremite);
                TheRoots.Next();
                IndTabNodes--;
            }
        }
        else
        {
            // A single entry point: a first ARC and the root NODE.
            TheRoots.First();
            CurrentArc = MakeArc(TheRoots.Current(), theBasicElts, theArcs, ref IndTabArcs);
            DistExt = TheRoots.Current().FirstEdge().Distance();
            IndExt = TheRoots.Current().EndPoint();

            Extremite = new MAT_Node(IndExt, CurrentArc, DistExt);
            Extremite.SetIndex(IndTabNodes);
            CurrentArc.SetSecondNode(Extremite);
            theNodes.Bind(IndTabNodes, Extremite);
            IndTabNodes--;

            // The ARCs from the root, with their neighbourhoods at the root node.
            FirstArc = CurrentArc;
            PreviousArc = FirstArc;
            TheRoots.Next();

            while (TheRoots.More())
            {
                CurrentArc = MakeArc(TheRoots.Current(), theBasicElts, theArcs, ref IndTabArcs);
                CurrentArc.SetSecondNode(Extremite);
                CurrentArc.SetNeighbour(MAT_Left, Extremite, PreviousArc);
                PreviousArc.SetNeighbour(MAT_Right, Extremite, CurrentArc);

                PreviousArc = CurrentArc;
                TheRoots.Next();
            }
            FirstArc.SetNeighbour(MAT_Left, Extremite, CurrentArc);
            CurrentArc.SetNeighbour(MAT_Right, Extremite, FirstArc);
        }

        // The arcs of the root nodes are up to date; the other nodes.
        UpDateNodes(ref IndTabNodes);
    }

    /// <summary>The arc of index Index.</summary>
    public MAT_Arc Arc(int Index) => theArcs.Find(Index);

    /// <summary>The basic element of index Index.</summary>
    public MAT_BasicElt BasicElt(int Index) => theBasicElts.Find(Index);

    /// <summary>The node of index Index.</summary>
    public MAT_Node Node(int Index) => theNodes.Find(Index);

    public int NumberOfArcs() => numberOfArcs;

    public int NumberOfNodes() => numberOfNodes;

    public int NumberOfInfiniteNodes() => numberOfInfiniteNodes;

    public int NumberOfBasicElts() => numberOfBasicElts;

    /// <summary>
    /// Merges two basic elements: the end of Elt1 (IndexElt1) becomes the end of Elt2 (IndexElt2), Elt2 is replaced
    /// by Elt1 in the arcs and removed. MergeArc1 tells whether two arcs separating the same elements were merged
    /// (GeomIndexArc1 and 2 their geometric indices); MergeArc2 likewise for the start and end arcs of Elt1 on a
    /// closed line (GeomIndexArc3 and 4).
    /// </summary>
    internal void FusionOfBasicElts(int IndexElt1, int IndexElt2, ref bool MergeArc1, ref int IGeomArc1,
        ref int IGeomArc2, ref bool MergeArc2, ref int IGeomArc3, ref int IGeomArc4)
    {
        MAT_BasicElt Elt1 = theBasicElts[IndexElt1];
        MAT_BasicElt Elt2 = theBasicElts[IndexElt2];

        if (Elt1 == Elt2)
        {
            return;
        }

        int i;
        MAT_Zone Zone2 = new(Elt2);

        // The arcs of the zone of Elt2 separate Elt1 and something.
        for (i = 1; i <= Zone2.NumberOfArcs(); i++)
        {
            if (Zone2.ArcOnFrontier(i).FirstElement() == Elt2)
            {
                theArcs[Zone2.ArcOnFrontier(i).Index()].SetFirstElement(Elt1);
            }
            else
            {
                theArcs[Zone2.ArcOnFrontier(i).Index()].SetSecondElement(Elt1);
            }
        }

        // The EndArc of Elt1 and the StartArc of Elt2 can separate the same basic elements: merge the two arcs.
        MAT_Arc EA1 = Elt1.EndArc()!;
        MAT_Arc SA2 = Elt2.StartArc()!;

        MAT_BasicElt E1 = EA1.FirstElement();
        MAT_BasicElt E2 = EA1.SecondElement();
        MAT_BasicElt E3 = SA2.FirstElement();
        MAT_BasicElt E4 = SA2.SecondElement();
        MergeArc1 = false;

        if ((E1 == E3 || E1 == E4) && (E2 == E3 || E2 == E4))
        {
            FusionOfArcs(theArcs[EA1.Index()], theArcs[SA2.Index()]);
            MergeArc1 = true;
            IGeomArc1 = EA1.GeomIndex();
            IGeomArc2 = SA2.GeomIndex();
        }

        // The end of Elt1 becomes the end of Elt2.
        Elt1.SetEndArc(Elt2.EndArc());

        // The EndArc and the StartArc of Elt1 can separate the same basic elements: if the nodes of the arcs are
        // not on the contour, merge the arcs (a closed contour of a single BasicElt); else nothing (a closed
        // contour of two BasicElts).
        MAT_Arc SA1 = Elt1.StartArc()!;
        EA1 = Elt1.EndArc()!;

        if (EA1 != SA1)
        {
            E1 = EA1.FirstElement();
            E2 = EA1.SecondElement();
            E3 = SA1.FirstElement();
            E4 = SA1.SecondElement();

            bool OnFig = EA1.FirstNode().OnBasicElt() || EA1.SecondNode().OnBasicElt()
                || SA1.FirstNode().OnBasicElt() || SA1.SecondNode().OnBasicElt();

            MergeArc2 = false;

            if ((E1 == E3 || E1 == E4) && (E2 == E3 || E2 == E4) && !OnFig)
            {
                FusionOfArcs(theArcs[EA1.Index()], theArcs[SA1.Index()]);
                MergeArc2 = true;
                IGeomArc3 = EA1.GeomIndex();
                IGeomArc4 = SA1.GeomIndex();
            }
        }

        // A basic element is removed.
        theBasicElts.UnBind(Elt2.Index());
        numberOfBasicElts--;
    }

    // Merges two arcs separating the same elements: Arc1 goes from the second node of Arc2 to its own second node.
    private void FusionOfArcs(MAT_Arc Arc1, MAT_Arc Arc2)
    {
        MAT_Node OldNode1 = Arc1.FirstNode();
        MAT_Node OldNode2 = Arc2.FirstNode();

        Arc1.SetFirstNode(Arc2.SecondNode());

        // The neighbourhoods around the new first node of Arc1.
        if (!Arc2.SecondNode().Infinite())
        {
            MAT_Arc LNeighbour = Arc2.Neighbour(Arc2.SecondNode(), MAT_Left)!;
            MAT_Arc RNeighbour = Arc2.Neighbour(Arc2.SecondNode(), MAT_Right)!;

            Arc1.SetFirstArc(MAT_Left, LNeighbour);
            Arc1.SetFirstArc(MAT_Right, RNeighbour);
            theArcs[LNeighbour.Index()].SetNeighbour(MAT_Right, Arc2.SecondNode(), Arc1);
            theArcs[RNeighbour.Index()].SetNeighbour(MAT_Left, Arc2.SecondNode(), Arc1);
        }
        else
        {
            Arc1.SetFirstArc(MAT_Left, null);
            Arc1.SetFirstArc(MAT_Right, null);
        }

        // The first node of Arc1.
        Arc1.FirstNode().SetLinkedArc(Arc1);

        // Removal of Arc2 and the old nodes.
        if (theNodes.IsBound(OldNode1.Index()))
        {
            theNodes.UnBind(OldNode1.Index());
            numberOfNodes--;
        }
        if (theNodes.IsBound(OldNode2.Index()))
        {
            theNodes.UnBind(OldNode2.Index());
            numberOfNodes--;
        }

        MAT_Arc anArc2 = Arc2;
        theArcs.UnBind(Arc2.Index());
        numberOfArcs--;

        for (int i = 1; i <= 2; i++)
        {
            MAT_BasicElt BE;
            if (i == 1)
            {
                BE = theBasicElts[anArc2.FirstElement().Index()];
            }
            else
            {
                BE = theBasicElts[anArc2.SecondElement().Index()];
            }
            if (BE.StartArc() == anArc2)
            {
                BE.SetStartArc(Arc1);
            }
            if (BE.EndArc() == anArc2)
            {
                BE.SetEndArc(Arc1);
            }
        }
    }

    /// <summary>Shifts the arcs down to fill the holes in their indices.</summary>
    internal void CompactArcs()
    {
        int IFind = 0;
        int i = 1;
        bool YaDecalage = false;

        while (IFind < numberOfArcs)
        {
            if (!theArcs.IsBound(i))
            {
                YaDecalage = true;
            }
            else
            {
                IFind++;
                if (YaDecalage)
                {
                    theArcs[i].SetIndex(IFind);
                    theArcs.Bind(IFind, theArcs[i]);
                    theArcs.UnBind(i);
                }
            }
            i++;
        }
    }

    /// <summary>Shifts the nodes down to fill the holes in their indices.</summary>
    internal void CompactNodes()
    {
        int IFind = 0;
        int i = 1;
        bool YaDecalage = false;

        while (IFind < numberOfNodes)
        {
            if (!theNodes.IsBound(i))
            {
                YaDecalage = true;
            }
            else
            {
                IFind++;
                if (YaDecalage)
                {
                    theNodes[i].SetIndex(IFind);
                    theNodes.Bind(IFind, theNodes[i]);
                    theNodes.UnBind(i);
                }
            }
            i++;
        }
    }

    /// <summary>Replaces the basic elements by a copy of NewMap, each element taking its key as index.</summary>
    internal void ChangeBasicElts(NCollection_DataMap<int, MAT_BasicElt> NewMap)
    {
        theBasicElts = new NCollection_DataMap<int, MAT_BasicElt>(NewMap);
        foreach (KeyValuePair<int, MAT_BasicElt> Ite in theBasicElts)
        {
            Ite.Value.SetIndex(Ite.Key);
        }
    }

    /// <summary>The basic element of index Index.</summary>
    public MAT_BasicElt ChangeBasicElt(int Index) => theBasicElts[Index];

    // Binds the first node of each arc in the table of nodes and links it to the arc. The root nodes are bound in
    // Perform.
    private void UpDateNodes(ref int IndTabNodes)
    {
        int i;
        MAT_Node Bout;

        for (i = 1; i <= numberOfArcs; i++)
        {
            Bout = theArcs[i].FirstNode();
            theNodes.Bind(IndTabNodes, Bout);
            Bout.SetIndex(IndTabNodes);
            IndTabNodes--;
            Bout.SetLinkedArc(theArcs[i]);
        }
    }

    // Creates the ARCs of the tree from aBisector.
    private static MAT_Arc MakeArc(MAT_Bisector aBisector, NCollection_DataMap<int, MAT_BasicElt> TheBasicElts,
        NCollection_DataMap<int, MAT_Arc> TheArcs, ref int IndTabArcs)
    {
        MAT_Arc CurrentArc;
        MAT_Arc PrevArc;
        MAT_Arc? NextArc = null;
        MAT_Node Extremite;
        MAT_ListOfBisector BisectorList;
        double DistExt;

        CurrentArc = new MAT_Arc(IndTabArcs, aBisector.BisectorNumber(),
            TheBasicElts[aBisector.FirstEdge().EdgeNumber()],
            TheBasicElts[aBisector.SecondEdge().EdgeNumber()]);
        DistExt = aBisector.DistIssuePoint();
        if (DistExt == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            DistExt = 1.0;
        }
        Extremite = new MAT_Node(aBisector.IssuePoint(), CurrentArc, DistExt);

        CurrentArc.SetFirstNode(Extremite);
        BisectorList = aBisector.List();
        BisectorList.First();

        if (!BisectorList.More())
        {
            // An arc on the contour.
            TheBasicElts[aBisector.SecondEdge().EdgeNumber()].SetStartArc(CurrentArc);
            TheBasicElts[aBisector.FirstEdge().EdgeNumber()].SetEndArc(CurrentArc);
        }
        else
        {
            PrevArc = CurrentArc;

            while (BisectorList.More())
            {
                NextArc = MakeArc(BisectorList.Current(), TheBasicElts, TheArcs, ref IndTabArcs);
                NextArc.SetSecondNode(Extremite);
                NextArc.SetNeighbour(MAT_Left, Extremite, PrevArc);
                PrevArc.SetNeighbour(MAT_Right, Extremite, NextArc);
                PrevArc = NextArc;
                BisectorList.Next();
            }
            CurrentArc.SetNeighbour(MAT_Left, Extremite, NextArc);
            NextArc!.SetNeighbour(MAT_Right, Extremite, CurrentArc);
        }

        CurrentArc.SetIndex(IndTabArcs);
        TheArcs.Bind(IndTabArcs, CurrentArc);
        IndTabArcs = IndTabArcs + 1;

        return CurrentArc;
    }
}
