// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeBuilder.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Builds the parts of an edge in a given state from its vertex list: the list becomes a chain of area limits (one
/// per vertex or interference, circular on a periodic edge), each area between two limits gets its state and the
/// edge's, and the parts in the state asked for are iterated as new edges with their vertices.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. OCCT's file defines <c>No_Exception</c>, so its two <c>Standard_DomainError_Raise_if</c>
/// checks (an empty vertex list, no interferences) are compiled out; the port leaves them out too, so the hider sees
/// what native sees.
/// </remarks>
public sealed class HLRBRep_EdgeBuilder
{
    private TopAbs_State toBuild;
    private HLRBRep_AreaLimit? myLimits;
    private HLRBRep_AreaLimit? left;
    private HLRBRep_AreaLimit? right;
    private int current;

    /// <summary>
    /// Creates an EdgeBuilder algorithm. VList describes the edge and the interferences. AreaLimits are created from
    /// the vertices. Builds(IN) is automatically called.
    /// </summary>
    public HLRBRep_EdgeBuilder(HLRBRep_VertexList VList)
    {
        // at creation the EdgeBuilder explore the VertexList
        // and use it to build a list of "AreaLimit" on the edge.
        // An area is a part of the curve between
        // two consecutive vertices

        // Standard_DomainError_Raise_if(!VList.More(), ...): compiled out by No_Exception (see the remarks)

        HLRBRep_AreaLimit? last = null, cur;
        TopAbs_State before, after, ebefore, eafter;
        HLRAlgo_Intersection V;

        // loop on the Vertices
        for (; VList.More(); VList.Next())
        {
            before = after = ebefore = eafter = TopAbs_UNKNOWN;
            // compute the states
            if (VList.IsBoundary())
            {
                switch (VList.Orientation())
                {
                    case TopAbs_FORWARD:
                        ebefore = TopAbs_OUT;
                        eafter = TopAbs_IN;
                        break;

                    case TopAbs_REVERSED:
                        ebefore = TopAbs_IN;
                        eafter = TopAbs_OUT;
                        break;

                    case TopAbs_INTERNAL:
                        ebefore = TopAbs_IN;
                        eafter = TopAbs_IN;
                        break;

                    case TopAbs_EXTERNAL:
                        ebefore = TopAbs_OUT;
                        eafter = TopAbs_OUT;
                        break;
                }
            }

            if (VList.IsInterference())
            {
                switch (VList.Transition())
                {
                    case TopAbs_FORWARD:
                        before = TopAbs_OUT;
                        after = TopAbs_IN;
                        break;

                    case TopAbs_REVERSED:
                        before = TopAbs_IN;
                        after = TopAbs_OUT;
                        break;

                    case TopAbs_INTERNAL:
                        before = TopAbs_IN;
                        after = TopAbs_IN;
                        break;

                    case TopAbs_EXTERNAL:
                        before = TopAbs_OUT;
                        after = TopAbs_OUT;
                        break;
                }

                switch (VList.BoundaryTransition())
                {
                    case TopAbs_FORWARD:
                        after = TopAbs_ON;
                        break;

                    case TopAbs_REVERSED:
                        before = TopAbs_ON;
                        break;

                    case TopAbs_INTERNAL:
                        before = TopAbs_ON;
                        after = TopAbs_ON;
                        break;

                    case TopAbs_EXTERNAL:
                        break;
                }
            }

            // create the Limit and connect to list
            V = VList.Current();
            cur = new HLRBRep_AreaLimit(V, VList.IsBoundary(), VList.IsInterference(), before, after, ebefore, eafter);
            if (myLimits is null)
            {
                myLimits = cur;
                last = cur;
            }
            else
            {
                last!.Next(cur);
                cur.Previous(last);
                last = cur;
            }
        }

        // periodicity, make a circular list
        if (VList.IsPeriodic())
        {
            last!.Next(myLimits);
            myLimits!.Previous(last);
        }

        // process UNKNOWN areas
        TopAbs_State stat = TopAbs_UNKNOWN;
        TopAbs_State estat = TopAbs_UNKNOWN;

        cur = myLimits;
        while (cur is not null)
        {
            if (stat == TopAbs_UNKNOWN)
            {
                stat = cur.StateBefore();
                if (stat == TopAbs_UNKNOWN)
                {
                    stat = cur.StateAfter();
                }
            }
            if (estat == TopAbs_UNKNOWN)
            {
                estat = cur.EdgeBefore();
                if (estat == TopAbs_UNKNOWN)
                {
                    estat = cur.EdgeAfter();
                }
            }
            cur = cur.Next();
            // test for periodicicity
            if (cur == myLimits)
            {
                break;
            }
        }

        // error if no interferences
        // Standard_DomainError_Raise_if(stat == TopAbs_UNKNOWN, ...): compiled out by No_Exception (see the remarks)
        // if no boundary the edge covers the whole curve
        if (estat == TopAbs_UNKNOWN)
        {
            estat = TopAbs_IN;
        }

        // propagate states
        cur = myLimits;
        while (cur is not null)
        {
            if (cur.StateBefore() == TopAbs_UNKNOWN)
            {
                cur.StateBefore(stat);
            }
            else
            {
                stat = cur.StateAfter();
            }
            if (cur.StateAfter() == TopAbs_UNKNOWN)
            {
                cur.StateAfter(stat);
            }
            if (cur.EdgeBefore() == TopAbs_UNKNOWN)
            {
                cur.EdgeBefore(estat);
            }
            else
            {
                estat = cur.EdgeAfter();
            }
            if (cur.EdgeAfter() == TopAbs_UNKNOWN)
            {
                cur.EdgeAfter(estat);
            }

            cur = cur.Next();
            if (cur == myLimits)
            {
                break;
            }
        }

        // initialise with IN parts
        Builds(TopAbs_IN);
    }

    /// <summary>Initialize an iteration on the areas.</summary>
    public void InitAreas()
    {
        left = myLimits!.Previous();
        right = myLimits;
    }

    /// <summary>Set the current area to the next area.</summary>
    public void NextArea()
    {
        left = right;
        if (right is not null)
        {
            right = right.Next();
        }
    }

    /// <summary>Set the current area to the previous area.</summary>
    public void PreviousArea()
    {
        right = left;
        if (left is not null)
        {
            left = left.Previous();
        }
    }

    /// <summary>Returns True if there is a current area.</summary>
    public bool HasArea()
    {
        if (left is null)
        {
            if (right is null)
            {
                return false;
            }
        }
        if (right == myLimits)
        {
            return false;
        }
        return true;
    }

    /// <summary>Returns the state of the current area.</summary>
    public TopAbs_State AreaState()
    {
        TopAbs_State stat = TopAbs_UNKNOWN;
        if (left is not null)
        {
            stat = left.StateAfter();
        }
        if (right is not null)
        {
            stat = right.StateBefore();
        }
        return stat;
    }

    /// <summary>Returns the edge state of the current area.</summary>
    public TopAbs_State AreaEdgeState()
    {
        TopAbs_State stat = TopAbs_UNKNOWN;
        if (left is not null)
        {
            stat = left.EdgeAfter();
        }
        if (right is not null)
        {
            stat = right.EdgeBefore();
        }
        return stat;
    }

    /// <summary>Returns the AreaLimit beginning the current area: null when the area is infinite on the left.</summary>
    public HLRBRep_AreaLimit? LeftLimit() => left;

    /// <summary>Returns the AreaLimit ending the current area: null when the area is infinite on the right.</summary>
    public HLRBRep_AreaLimit? RightLimit() => right;

    /// <summary>
    /// Reinitialize the results iteration to the parts with State ToBuild. If this method is not called after
    /// construction the default is ToBuild = IN.
    /// </summary>
    public void Builds(TopAbs_State ToBuild)
    {
        toBuild = ToBuild;
        InitAreas();
        do
        {
            if ((AreaState() == toBuild) && (AreaEdgeState() == TopAbs_IN))
            {
                if (left is null)
                {
                    current = 2;
                }
                else
                {
                    current = 1;
                }
                return;
            }
            NextArea();
        } while (HasArea());
        current = 3;
    }

    /// <summary>Returns True if there are more new edges to build.</summary>
    public bool MoreEdges() => HasArea();

    /// <summary>Proceeds to the next edge to build. Skip all remaining vertices on the current edge.</summary>
    public void NextEdge()
    {
        // clean the current edge
        while (AreaState() == toBuild)
        {
            NextArea();
        }
        // go to the next edge
        while (HasArea())
        {
            if ((AreaState() == toBuild) && (AreaEdgeState() == TopAbs_IN))
            {
                if (left is null)
                {
                    current = 2;
                }
                else
                {
                    current = 1;
                }
                return;
            }
            NextArea();
        }
    }

    /// <summary>True if there are more vertices in the current new edge.</summary>
    public bool MoreVertices() => (current < 3);

    /// <summary>Proceeds to the next vertex of the current edge.</summary>
    public void NextVertex()
    {
        if (current == 1)
        {
            current = 2;
            if (right is null)
            {
                current = 3;
            }
        }
        else if (current == 2)
        {
            NextArea();
            if ((AreaState() == toBuild) && (AreaEdgeState() == TopAbs_IN))
            {
                current = 2;
            }
            else
            {
                current = 3;
            }
        }
        else
        {
            throw new Standard_NoSuchObject("EdgeBuilder::NextVertex : No current edge");
        }
    }

    /// <summary>Returns the current vertex of the current edge.</summary>
    public HLRAlgo_Intersection Current()
    {
        if (current == 1)
        {
            return left!.Vertex();
        }
        else if (current == 2)
        {
            return right!.Vertex();
        }
        else
        {
            throw new Standard_NoSuchObject("EdgeBuilder::Current : No current vertex");
        }
    }

    /// <summary>Returns True if the current vertex comes from the boundary of the edge.</summary>
    public bool IsBoundary()
    {
        if (current == 1)
        {
            return left!.IsBoundary();
        }
        else if (current == 2)
        {
            return right!.IsBoundary();
        }
        else
        {
            throw new Standard_NoSuchObject("EdgeBuilder::IsBoundary : No current vertex");
        }
    }

    /// <summary>Returns True if the current vertex was an interference.</summary>
    public bool IsInterference()
    {
        if (current == 1)
        {
            return left!.IsInterference();
        }
        else if (current == 2)
        {
            return right!.IsInterference();
        }
        else
        {
            throw new Standard_NoSuchObject("EdgeBuilder::IsInterference : No current vertex");
        }
    }

    /// <summary>Returns the new orientation of the current vertex.</summary>
    public TopAbs_Orientation Orientation()
    {
        if (current == 1)
        {
            if ((left!.StateBefore() == left.StateAfter()) && (left.EdgeBefore() == left.EdgeAfter()))
            {
                return TopAbs_INTERNAL;
            }
            else
            {
                return TopAbs_FORWARD;
            }
        }
        else if (current == 2)
        {
            if ((right!.StateBefore() == right.StateAfter()) && (right.EdgeBefore() == right.EdgeAfter()))
            {
                return TopAbs_INTERNAL;
            }
            else
            {
                return TopAbs_REVERSED;
            }
        }
        return TopAbs_EXTERNAL; // only for WNT.
    }

    /// <summary>Unlinks the area limits (OCCT's destructor breaks the circular list of handles).</summary>
    public void Destroy()
    {
        HLRBRep_AreaLimit? cur = myLimits;
        while (cur is not null)
        {
            HLRBRep_AreaLimit? n = cur.Next();
            cur.Clear();
            cur = n;
        }
        left = null;
        right = null;
        myLimits = null;
    }
}
