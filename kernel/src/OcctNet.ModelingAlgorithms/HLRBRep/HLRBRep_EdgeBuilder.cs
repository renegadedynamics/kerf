// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeBuilder.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

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
    private Containment toBuild;
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
        Containment before, after, ebefore, eafter;
        HLRAlgo_Intersection V;

        // loop on the Vertices
        for (; VList.More(); VList.Next())
        {
            before = after = ebefore = eafter = Unknown;
            // compute the states
            if (VList.IsBoundary())
            {
                switch (VList.Orientation())
                {
                    case Forward:
                        ebefore = Out;
                        eafter = In;
                        break;

                    case Reversed:
                        ebefore = In;
                        eafter = Out;
                        break;

                    case Internal:
                        ebefore = In;
                        eafter = In;
                        break;

                    case External:
                        ebefore = Out;
                        eafter = Out;
                        break;
                }
            }

            if (VList.IsInterference())
            {
                switch (VList.Transition())
                {
                    case Forward:
                        before = Out;
                        after = In;
                        break;

                    case Reversed:
                        before = In;
                        after = Out;
                        break;

                    case Internal:
                        before = In;
                        after = In;
                        break;

                    case External:
                        before = Out;
                        after = Out;
                        break;
                }

                switch (VList.BoundaryTransition())
                {
                    case Forward:
                        after = On;
                        break;

                    case Reversed:
                        before = On;
                        break;

                    case Internal:
                        before = On;
                        after = On;
                        break;

                    case External:
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
        Containment stat = Unknown;
        Containment estat = Unknown;

        cur = myLimits;
        while (cur is not null)
        {
            if (stat == Unknown)
            {
                stat = cur.StateBefore();
                if (stat == Unknown)
                {
                    stat = cur.StateAfter();
                }
            }
            if (estat == Unknown)
            {
                estat = cur.EdgeBefore();
                if (estat == Unknown)
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
        if (estat == Unknown)
        {
            estat = In;
        }

        // propagate states
        cur = myLimits;
        while (cur is not null)
        {
            if (cur.StateBefore() == Unknown)
            {
                cur.StateBefore(stat);
            }
            else
            {
                stat = cur.StateAfter();
            }
            if (cur.StateAfter() == Unknown)
            {
                cur.StateAfter(stat);
            }
            if (cur.EdgeBefore() == Unknown)
            {
                cur.EdgeBefore(estat);
            }
            else
            {
                estat = cur.EdgeAfter();
            }
            if (cur.EdgeAfter() == Unknown)
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
        Builds(In);
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
    public Containment AreaState()
    {
        Containment stat = Unknown;
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
    public Containment AreaEdgeState()
    {
        Containment stat = Unknown;
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
    public void Builds(Containment ToBuild)
    {
        toBuild = ToBuild;
        InitAreas();
        do
        {
            if ((AreaState() == toBuild) && (AreaEdgeState() == In))
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
            if ((AreaState() == toBuild) && (AreaEdgeState() == In))
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
            if ((AreaState() == toBuild) && (AreaEdgeState() == In))
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
            throw new NoSuchObjectException("EdgeBuilder::NextVertex : No current edge");
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
            throw new NoSuchObjectException("EdgeBuilder::Current : No current vertex");
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
            throw new NoSuchObjectException("EdgeBuilder::IsBoundary : No current vertex");
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
            throw new NoSuchObjectException("EdgeBuilder::IsInterference : No current vertex");
        }
    }

    /// <summary>Returns the new orientation of the current vertex.</summary>
    public Orientation Orientation()
    {
        if (current == 1)
        {
            if ((left!.StateBefore() == left.StateAfter()) && (left.EdgeBefore() == left.EdgeAfter()))
            {
                return Internal;
            }
            else
            {
                return Forward;
            }
        }
        else if (current == 2)
        {
            if ((right!.StateBefore() == right.StateAfter()) && (right.EdgeBefore() == right.EdgeAfter()))
            {
                return Internal;
            }
            else
            {
                return Reversed;
            }
        }
        return External; // only for WNT.
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
