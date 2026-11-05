// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Zone.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Zone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>The zone of a basic element: the part of the plane nearer to it than to any other element, described
/// by the arcs on its frontier.</summary>
internal sealed class MedialZone : Transient
{
    private readonly NCollection_Sequence<MedialArc> _frontier = new();
    private bool _limited;

    public MedialZone()
    {
        _limited = true;
    }

    /// <summary>The zone of aBasicElt.</summary>
    public MedialZone(MedialBasicElt aBasicElt)
    {
        Perform(aBasicElt);
    }

    /// <summary>Computes the frontier of the zone of aBasicElt.</summary>
    public void Perform(MedialBasicElt aBasicElt)
    {
        MedialNode nextNode, startNode;
        MedialArc currentArc;

        _limited = true;
        _frontier.Clear();
        // If the end arc of elt is null, the elt is the extremity of the contour.
        if (aBasicElt.EndArc() is null)
        {
            return;
        }

        currentArc = aBasicElt.EndArc()!;
        _frontier.Append(currentArc);
        // The node at which the arc is touched and to which the left side of the zone belongs.
        nextNode = NodeForTurn(currentArc, aBasicElt, MedialSide.Left);
        startNode = currentArc.TheOtherNode(nextNode);

        // Exploration of the arcs of the frontier at the left of the zone.
        while (!nextNode.PendingNode() && (nextNode != startNode))
        {
            currentArc = currentArc.Neighbour(nextNode, MedialSide.Left)!;
            _frontier.Append(currentArc);
            nextNode = currentArc.TheOtherNode(nextNode);
        }

        // If the exploration stops at an infinite node, the zone is not limited: the right side is explored too.
        if (nextNode.Infinite())
        {
            _limited = false;
            currentArc = aBasicElt.StartArc()!;
            _frontier.Append(currentArc);
            nextNode = NodeForTurn(currentArc, aBasicElt, MedialSide.Right);

            while (!nextNode.Infinite())
            {
                currentArc = currentArc.Neighbour(nextNode, MedialSide.Right)!;
                _frontier.Append(currentArc);
                nextNode = currentArc.TheOtherNode(nextNode);
            }
        }
    }

    /// <summary>The number of arcs on the frontier of the zone.</summary>
    public int NumberOfArcs() => _frontier.Length();

    /// <summary>The arc of index Index on the frontier.</summary>
    public MedialArc ArcOnFrontier(int index) => _frontier.Value(index);

    /// <summary>True if the zone has a frontier.</summary>
    public bool NoEmptyZone() => !_frontier.IsEmpty();

    /// <summary>True if the zone is bounded.</summary>
    public bool Limited() => _limited;

    private static MedialNode NodeForTurn(MedialArc anArc, MedialBasicElt aBE, MedialSide aSide)
    {
        MedialArc? neighbourArc;
        MedialNode nodeSol;

        nodeSol = anArc.FirstNode();
        neighbourArc = anArc.Neighbour(nodeSol, aSide);
        if (neighbourArc is null)
        {
            nodeSol = anArc.SecondNode();
            neighbourArc = anArc.Neighbour(nodeSol, aSide);
        }
        if (neighbourArc is null)
        {
            return nodeSol;
        }
        if (neighbourArc.FirstElement() == aBE)
        {
            return nodeSol;
        }
        else if (neighbourArc.SecondElement() == aBE)
        {
            return nodeSol;
        }
        else
        {
            return anArc.TheOtherNode(nodeSol);
        }
    }
}
