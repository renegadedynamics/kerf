// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeBuilder.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.HiddenLines;

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
internal sealed class EdgeBuilder
{
    private Containment _toBuild;
    private AreaLimit? _limits;
    private AreaLimit? _left;
    private AreaLimit? _right;
    private int _current;

    /// <summary>
    /// Creates an EdgeBuilder algorithm. VList describes the edge and the interferences. AreaLimits are created from
    /// the vertices. Builds(IN) is automatically called.
    /// </summary>
    public EdgeBuilder(VertexList vList)
    {
        // at creation the EdgeBuilder explore the VertexList
        // and use it to build a list of "AreaLimit" on the edge.
        // An area is a part of the curve between
        // two consecutive vertices

        // Standard_DomainError_Raise_if(!VList.More(), ...): compiled out by No_Exception (see the remarks)

        AreaLimit? last = null, cur;
        Containment before, after, ebefore, eafter;
        AlgoIntersection v;

        // loop on the Vertices
        for (; vList.More(); vList.Next())
        {
            before = after = ebefore = eafter = Unknown;
            // compute the states
            if (vList.IsBoundary)
            {
                switch (vList.Orientation())
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

            if (vList.IsInterference)
            {
                switch (vList.Transition())
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

                switch (vList.BoundaryTransition())
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
            v = vList.Current();
            cur = new AreaLimit(v, vList.IsBoundary, vList.IsInterference, before, after, ebefore, eafter);
            if (_limits is null)
            {
                _limits = cur;
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
        if (vList.IsPeriodic)
        {
            last!.Next(_limits);
            _limits!.Previous(last);
        }

        // process UNKNOWN areas
        Containment stat = Unknown;
        Containment estat = Unknown;

        cur = _limits;
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
            if (cur == _limits)
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
        cur = _limits;
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
            if (cur == _limits)
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
        _left = _limits!.Previous();
        _right = _limits;
    }

    /// <summary>Set the current area to the next area.</summary>
    public void NextArea()
    {
        _left = _right;
        if (_right is not null)
        {
            _right = _right.Next();
        }
    }

    /// <summary>Set the current area to the previous area.</summary>
    public void PreviousArea()
    {
        _right = _left;
        if (_left is not null)
        {
            _left = _left.Previous();
        }
    }

    /// <summary>Returns True if there is a current area.</summary>
    public bool HasArea()
    {
        if (_left is null)
        {
            if (_right is null)
            {
                return false;
            }
        }
        if (_right == _limits)
        {
            return false;
        }
        return true;
    }

    /// <summary>Returns the state of the current area.</summary>
    public Containment AreaState()
    {
        Containment stat = Unknown;
        if (_left is not null)
        {
            stat = _left.StateAfter();
        }
        if (_right is not null)
        {
            stat = _right.StateBefore();
        }
        return stat;
    }

    /// <summary>Returns the edge state of the current area.</summary>
    public Containment AreaEdgeState()
    {
        Containment stat = Unknown;
        if (_left is not null)
        {
            stat = _left.EdgeAfter();
        }
        if (_right is not null)
        {
            stat = _right.EdgeBefore();
        }
        return stat;
    }

    /// <summary>Returns the AreaLimit beginning the current area: null when the area is infinite on the left.</summary>
    public AreaLimit? LeftLimit() => _left;

    /// <summary>Returns the AreaLimit ending the current area: null when the area is infinite on the right.</summary>
    public AreaLimit? RightLimit() => _right;

    /// <summary>
    /// Reinitialize the results iteration to the parts with State ToBuild. If this method is not called after
    /// construction the default is ToBuild = IN.
    /// </summary>
    public void Builds(Containment toBuild)
    {
        _toBuild = toBuild;
        InitAreas();
        do
        {
            if ((AreaState() == _toBuild) && (AreaEdgeState() == In))
            {
                if (_left is null)
                {
                    _current = 2;
                }
                else
                {
                    _current = 1;
                }
                return;
            }
            NextArea();
        } while (HasArea());
        _current = 3;
    }

    /// <summary>Returns True if there are more new edges to build.</summary>
    public bool MoreEdges() => HasArea();

    /// <summary>Proceeds to the next edge to build. Skip all remaining vertices on the current edge.</summary>
    public void NextEdge()
    {
        // clean the current edge
        while (AreaState() == _toBuild)
        {
            NextArea();
        }
        // go to the next edge
        while (HasArea())
        {
            if ((AreaState() == _toBuild) && (AreaEdgeState() == In))
            {
                if (_left is null)
                {
                    _current = 2;
                }
                else
                {
                    _current = 1;
                }
                return;
            }
            NextArea();
        }
    }

    /// <summary>True if there are more vertices in the current new edge.</summary>
    public bool MoreVertices() => (_current < 3);

    /// <summary>Proceeds to the next vertex of the current edge.</summary>
    public void NextVertex()
    {
        if (_current == 1)
        {
            _current = 2;
            if (_right is null)
            {
                _current = 3;
            }
        }
        else if (_current == 2)
        {
            NextArea();
            if ((AreaState() == _toBuild) && (AreaEdgeState() == In))
            {
                _current = 2;
            }
            else
            {
                _current = 3;
            }
        }
        else
        {
            throw new NoSuchObjectException("EdgeBuilder::NextVertex : No current edge");
        }
    }

    /// <summary>Returns the current vertex of the current edge.</summary>
    public AlgoIntersection Current()
    {
        if (_current == 1)
        {
            return _left!.Vertex();
        }
        else if (_current == 2)
        {
            return _right!.Vertex();
        }
        else
        {
            throw new NoSuchObjectException("EdgeBuilder::Current : No current vertex");
        }
    }

    /// <summary>Returns True if the current vertex comes from the boundary of the edge.</summary>
    public bool IsBoundary()
    {
        if (_current == 1)
        {
            return _left!.IsBoundary;
        }
        else if (_current == 2)
        {
            return _right!.IsBoundary;
        }
        else
        {
            throw new NoSuchObjectException("EdgeBuilder::IsBoundary : No current vertex");
        }
    }

    /// <summary>Returns True if the current vertex was an interference.</summary>
    public bool IsInterference()
    {
        if (_current == 1)
        {
            return _left!.IsInterference;
        }
        else if (_current == 2)
        {
            return _right!.IsInterference;
        }
        else
        {
            throw new NoSuchObjectException("EdgeBuilder::IsInterference : No current vertex");
        }
    }

    /// <summary>Returns the new orientation of the current vertex.</summary>
    public Orientation Orientation()
    {
        if (_current == 1)
        {
            if ((_left!.StateBefore() == _left.StateAfter()) && (_left.EdgeBefore() == _left.EdgeAfter()))
            {
                return Internal;
            }
            else
            {
                return Forward;
            }
        }
        else if (_current == 2)
        {
            if ((_right!.StateBefore() == _right.StateAfter()) && (_right.EdgeBefore() == _right.EdgeAfter()))
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
        AreaLimit? cur = _limits;
        while (cur is not null)
        {
            AreaLimit? n = cur.Next();
            cur.Clear();
            cur = n;
        }
        _left = null;
        _right = null;
        _limits = null;
    }
}
