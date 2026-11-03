// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Intervals.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Intervals.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Intervals.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Intersection.Position;

namespace Kerf.Engine.Intersection;

/// <summary>
/// A sorted sequence of disjoint intervals (<see cref="IntersectionInterval"/>), with the Boolean operations: intersection,
/// subtraction, union and exclusive union, with an interval or another sequence.
/// </summary>
/// <remarks>
/// OCCT's class is used by value but holds an NCollection_Sequence; the port makes it a class with OCCT's copy as the
/// copy constructor, and ported code copies where C++ copies (decision 1077).
/// </remarks>
public sealed class Intervals
{
    private readonly NCollection_Sequence<IntersectionInterval> _inter;

    /// <summary>No interval.</summary>
    public Intervals()
    {
        _inter = new NCollection_Sequence<IntersectionInterval>();
    }

    /// <summary>The single interval <paramref name="Int"/>.</summary>
    public Intervals(in IntersectionInterval Int)
    {
        _inter = new NCollection_Sequence<IntersectionInterval>();
        _inter.Append(Int);
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's copy constructor).</summary>
    public Intervals(Intervals other)
    {
        _inter = new NCollection_Sequence<IntersectionInterval>(other._inter);
    }

    /// <summary>OCCT's assignment: the intervals of <paramref name="other"/>.</summary>
    public Intervals Assign(Intervals other)
    {
        if (!ReferenceEquals(this, other))
        {
            _inter.Assign(other._inter);
        }
        return this;
    }

    public void Intersect(in IntersectionInterval tool)
    {
        Intervals inter = new(tool);
        Intersect(inter);
    }

    public void Intersect(Intervals tool)
    {
        Intervals xUni = new(this);
        xUni.XUnite(tool);
        Unite(tool);
        Subtract(xUni);
    }

    public void Subtract(in IntersectionInterval tool)
    {
        int index = 1;

        while (index <= _inter.Length())
        {
            switch (tool.Position(_inter[index]))
            {
                case Position.Before:
                    index = _inter.Length(); // sortir
                    break;

                case Position.JustBefore:
                    _inter[index].CutAtStart(tool.End(), tool.TolEnd()); // modifier le debut
                    index = _inter.Length(); // sortir
                    break;

                case Position.OverlappingAtStart:
                case Position.JustOverlappingAtStart:
                    _inter[index].SetStart(tool.End(), tool.TolEnd()); // garder la fin
                    index = _inter.Length(); // sortir
                    break;

                case Position.JustEnclosingAtEnd:
                case Position.Enclosing:
                case Position.Similar:
                case Position.JustEnclosingAtStart:
                    _inter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Position.Inside:
                    _inter.InsertAfter(index, _inter[index]);
                    _inter[index].SetEnd(tool.Start(), tool.TolStart()); // garder le debut
                    _inter[index + 1].SetStart(tool.End(), tool.TolEnd()); // garder la fin
                    index = _inter.Length(); // sortir
                    break;

                case Position.JustOverlappingAtEnd:
                case Position.OverlappingAtEnd:
                    _inter[index].SetEnd(tool.Start(), tool.TolStart()); // garder le debut
                    break; // continuer

                case Position.JustAfter:
                    _inter[index].CutAtEnd(tool.Start(), tool.TolStart()); // modifier la fin
                    break; // continuer

                case Position.After:
                    break; // continuer
            }
            index++;
        }
    }

    public void Subtract(Intervals tool)
    {
        int index;
        for (index = 1; index <= tool._inter.Length(); index++)
        {
            Subtract(tool._inter[index]);
        }
    }

    public void Unite(in IntersectionInterval tool)
    {
        bool inserted = false;
        IntersectionInterval tins = tool;
        int index = 1;

        while (index <= _inter.Length())
        {
            switch (tins.Position(_inter[index]))
            {
                case Position.Before:
                    inserted = true;
                    _inter.InsertBefore(index, tins); // inserer avant et
                    index = _inter.Length(); // sortir
                    break;

                case Position.JustBefore:
                case Position.OverlappingAtStart:
                    inserted = true;
                    _inter[index].SetStart(tins.Start(), tins.TolStart()); // changer le debut
                    index = _inter.Length(); // sortir
                    break;

                case Position.Similar:
                    tins.FuseAtStart(_inter[index].Start(), _inter[index].TolStart()); // modifier le debut
                    goto case Position.JustEnclosingAtEnd;
                case Position.JustEnclosingAtEnd:
                    tins.FuseAtEnd(_inter[index].End(), _inter[index].TolEnd()); // modifier la fin
                    goto case Position.Enclosing;
                case Position.Enclosing:
                    _inter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Position.JustOverlappingAtEnd:
                    tins.SetStart(_inter[index].Start(), _inter[index].TolStart()); // changer le debut
                    tins.FuseAtEnd(_inter[index].End(), _inter[index].TolEnd()); // modifier la fin
                    _inter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Position.JustOverlappingAtStart:
                    inserted = true;
                    _inter[index].FuseAtStart(tins.Start(), tins.TolStart()); // modifier le debut
                    index = _inter.Length(); // sortir
                    break;

                case Position.JustEnclosingAtStart:
                    tins.FuseAtStart(_inter[index].Start(), _inter[index].TolStart()); // modifier le debut
                    _inter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Position.Inside:
                    inserted = true;
                    index = _inter.Length(); // sortir
                    break;

                case Position.OverlappingAtEnd:
                case Position.JustAfter:
                    tins.SetStart(_inter[index].Start(), _inter[index].TolStart()); // changer le debut
                    _inter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Position.After:
                    break; // continuer
            }
            index++;
        }
        if (!inserted)
        {
            _inter.Append(tins);
        }
    }

    public void Unite(Intervals tool)
    {
        int index;
        for (index = 1; index <= tool._inter.Length(); index++)
        {
            Unite(tool._inter[index]);
        }
    }

    public void XUnite(in IntersectionInterval tool)
    {
        Intervals inter = new(tool);
        XUnite(inter);
    }

    public void XUnite(Intervals tool)
    {
        Intervals sub2 = new(tool);
        sub2.Subtract(this);
        Subtract(tool);
        Unite(sub2);
    }

    /// <summary>The number of intervals.</summary>
    public int IntervalCount => _inter.Length();

    /// <summary>The interval <paramref name="index"/>.</summary>
    public IntersectionInterval Value(int index) => _inter.Value(index);
}
