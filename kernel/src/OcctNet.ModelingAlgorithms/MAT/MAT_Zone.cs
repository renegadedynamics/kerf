// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Zone.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Zone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MAT_Side;

namespace Occt;

/// <summary>The zone of a basic element: the part of the plane nearer to it than to any other element, described
/// by the arcs on its frontier.</summary>
public sealed class MAT_Zone : Standard_Transient
{
    private readonly NCollection_Sequence<MAT_Arc> frontier = new();
    private bool limited;

    public MAT_Zone()
    {
        limited = true;
    }

    /// <summary>The zone of aBasicElt.</summary>
    public MAT_Zone(MAT_BasicElt aBasicElt)
    {
        Perform(aBasicElt);
    }

    /// <summary>Computes the frontier of the zone of aBasicElt.</summary>
    public void Perform(MAT_BasicElt aBasicElt)
    {
        MAT_Node NextNode, StartNode;
        MAT_Arc CurrentArc;

        limited = true;
        frontier.Clear();
        // If the end arc of elt is null, the elt is the extremity of the contour.
        if (aBasicElt.EndArc() is null)
        {
            return;
        }

        CurrentArc = aBasicElt.EndArc()!;
        frontier.Append(CurrentArc);
        // The node at which the arc is touched and to which the left side of the zone belongs.
        NextNode = NodeForTurn(CurrentArc, aBasicElt, MAT_Left);
        StartNode = CurrentArc.TheOtherNode(NextNode);

        // Exploration of the arcs of the frontier at the left of the zone.
        while (!NextNode.PendingNode() && (NextNode != StartNode))
        {
            CurrentArc = CurrentArc.Neighbour(NextNode, MAT_Left)!;
            frontier.Append(CurrentArc);
            NextNode = CurrentArc.TheOtherNode(NextNode);
        }

        // If the exploration stops at an infinite node, the zone is not limited: the right side is explored too.
        if (NextNode.Infinite())
        {
            limited = false;
            CurrentArc = aBasicElt.StartArc()!;
            frontier.Append(CurrentArc);
            NextNode = NodeForTurn(CurrentArc, aBasicElt, MAT_Right);

            while (!NextNode.Infinite())
            {
                CurrentArc = CurrentArc.Neighbour(NextNode, MAT_Right)!;
                frontier.Append(CurrentArc);
                NextNode = CurrentArc.TheOtherNode(NextNode);
            }
        }
    }

    /// <summary>The number of arcs on the frontier of the zone.</summary>
    public int NumberOfArcs() => frontier.Length();

    /// <summary>The arc of index Index on the frontier.</summary>
    public MAT_Arc ArcOnFrontier(int Index) => frontier.Value(Index);

    /// <summary>True if the zone has a frontier.</summary>
    public bool NoEmptyZone() => !frontier.IsEmpty();

    /// <summary>True if the zone is bounded.</summary>
    public bool Limited() => limited;

    private static MAT_Node NodeForTurn(MAT_Arc anArc, MAT_BasicElt aBE, MAT_Side aSide)
    {
        MAT_Arc? NeighbourArc;
        MAT_Node NodeSol;

        NodeSol = anArc.FirstNode();
        NeighbourArc = anArc.Neighbour(NodeSol, aSide);
        if (NeighbourArc is null)
        {
            NodeSol = anArc.SecondNode();
            NeighbourArc = anArc.Neighbour(NodeSol, aSide);
        }
        if (NeighbourArc is null)
        {
            return NodeSol;
        }
        if (NeighbourArc.FirstElement() == aBE)
        {
            return NodeSol;
        }
        else if (NeighbourArc.SecondElement() == aBE)
        {
            return NodeSol;
        }
        else
        {
            return anArc.TheOtherNode(NodeSol);
        }
    }
}
