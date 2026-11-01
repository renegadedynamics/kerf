// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Intervals.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Intervals.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Intervals.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Intrv_Position;

namespace Occt;

/// <summary>
/// A sorted sequence of disjoint intervals (<see cref="Intrv_Interval"/>), with the Boolean operations: intersection,
/// subtraction, union and exclusive union, with an interval or another sequence.
/// </summary>
/// <remarks>
/// OCCT's class is used by value but holds an NCollection_Sequence; the port makes it a class with OCCT's copy as the
/// copy constructor, and ported code copies where C++ copies (decision 10103).
/// </remarks>
public sealed class Intrv_Intervals
{
    private readonly NCollection_Sequence<Intrv_Interval> myInter;

    /// <summary>No interval.</summary>
    public Intrv_Intervals()
    {
        myInter = new NCollection_Sequence<Intrv_Interval>();
    }

    /// <summary>The single interval <paramref name="Int"/>.</summary>
    public Intrv_Intervals(in Intrv_Interval Int)
    {
        myInter = new NCollection_Sequence<Intrv_Interval>();
        myInter.Append(Int);
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's copy constructor).</summary>
    public Intrv_Intervals(Intrv_Intervals theOther)
    {
        myInter = new NCollection_Sequence<Intrv_Interval>(theOther.myInter);
    }

    /// <summary>OCCT's assignment: the intervals of <paramref name="theOther"/>.</summary>
    public Intrv_Intervals Assign(Intrv_Intervals theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            myInter.Assign(theOther.myInter);
        }
        return this;
    }

    public void Intersect(in Intrv_Interval Tool)
    {
        Intrv_Intervals Inter = new(Tool);
        Intersect(Inter);
    }

    public void Intersect(Intrv_Intervals Tool)
    {
        Intrv_Intervals XUni = new(this);
        XUni.XUnite(Tool);
        Unite(Tool);
        Subtract(XUni);
    }

    public void Subtract(in Intrv_Interval Tool)
    {
        int index = 1;

        while (index <= myInter.Length())
        {
            switch (Tool.Position(myInter[index]))
            {
                case Intrv_Before:
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_JustBefore:
                    myInter[index].CutAtStart(Tool.End(), Tool.TolEnd()); // modifier le debut
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_OverlappingAtStart:
                case Intrv_JustOverlappingAtStart:
                    myInter[index].SetStart(Tool.End(), Tool.TolEnd()); // garder la fin
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_JustEnclosingAtEnd:
                case Intrv_Enclosing:
                case Intrv_Similar:
                case Intrv_JustEnclosingAtStart:
                    myInter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Intrv_Inside:
                    myInter.InsertAfter(index, myInter[index]);
                    myInter[index].SetEnd(Tool.Start(), Tool.TolStart()); // garder le debut
                    myInter[index + 1].SetStart(Tool.End(), Tool.TolEnd()); // garder la fin
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_JustOverlappingAtEnd:
                case Intrv_OverlappingAtEnd:
                    myInter[index].SetEnd(Tool.Start(), Tool.TolStart()); // garder le debut
                    break; // continuer

                case Intrv_JustAfter:
                    myInter[index].CutAtEnd(Tool.Start(), Tool.TolStart()); // modifier la fin
                    break; // continuer

                case Intrv_After:
                    break; // continuer
            }
            index++;
        }
    }

    public void Subtract(Intrv_Intervals Tool)
    {
        int index;
        for (index = 1; index <= Tool.myInter.Length(); index++)
        {
            Subtract(Tool.myInter[index]);
        }
    }

    public void Unite(in Intrv_Interval Tool)
    {
        bool Inserted = false;
        Intrv_Interval Tins = Tool;
        int index = 1;

        while (index <= myInter.Length())
        {
            switch (Tins.Position(myInter[index]))
            {
                case Intrv_Before:
                    Inserted = true;
                    myInter.InsertBefore(index, Tins); // inserer avant et
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_JustBefore:
                case Intrv_OverlappingAtStart:
                    Inserted = true;
                    myInter[index].SetStart(Tins.Start(), Tins.TolStart()); // changer le debut
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_Similar:
                    Tins.FuseAtStart(myInter[index].Start(), myInter[index].TolStart()); // modifier le debut
                    goto case Intrv_JustEnclosingAtEnd;
                case Intrv_JustEnclosingAtEnd:
                    Tins.FuseAtEnd(myInter[index].End(), myInter[index].TolEnd()); // modifier la fin
                    goto case Intrv_Enclosing;
                case Intrv_Enclosing:
                    myInter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Intrv_JustOverlappingAtEnd:
                    Tins.SetStart(myInter[index].Start(), myInter[index].TolStart()); // changer le debut
                    Tins.FuseAtEnd(myInter[index].End(), myInter[index].TolEnd()); // modifier la fin
                    myInter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Intrv_JustOverlappingAtStart:
                    Inserted = true;
                    myInter[index].FuseAtStart(Tins.Start(), Tins.TolStart()); // modifier le debut
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_JustEnclosingAtStart:
                    Tins.FuseAtStart(myInter[index].Start(), myInter[index].TolStart()); // modifier le debut
                    myInter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Intrv_Inside:
                    Inserted = true;
                    index = myInter.Length(); // sortir
                    break;

                case Intrv_OverlappingAtEnd:
                case Intrv_JustAfter:
                    Tins.SetStart(myInter[index].Start(), myInter[index].TolStart()); // changer le debut
                    myInter.Remove(index); // detruire et
                    index--; // continuer
                    break;

                case Intrv_After:
                    break; // continuer
            }
            index++;
        }
        if (!Inserted)
        {
            myInter.Append(Tins);
        }
    }

    public void Unite(Intrv_Intervals Tool)
    {
        int index;
        for (index = 1; index <= Tool.myInter.Length(); index++)
        {
            Unite(Tool.myInter[index]);
        }
    }

    public void XUnite(in Intrv_Interval Tool)
    {
        Intrv_Intervals Inter = new(Tool);
        XUnite(Inter);
    }

    public void XUnite(Intrv_Intervals Tool)
    {
        Intrv_Intervals Sub2 = new(Tool);
        Sub2.Subtract(this);
        Subtract(Tool);
        Unite(Sub2);
    }

    /// <summary>The number of intervals.</summary>
    public int NbIntervals() => myInter.Length();

    /// <summary>The interval <paramref name="Index"/>.</summary>
    public Intrv_Interval Value(int Index) => myInter.Value(Index);
}
