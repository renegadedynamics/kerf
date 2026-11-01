// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeStatus.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeStatus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The hidden-line status of an edge: its bounds and their tolerances, whether it is wholly visible or wholly
/// hidden, and otherwise its visible intervals.
/// </summary>
/// <remarks>
/// OCCT's class is used by value (in HLRBRep_EdgeData) but holds an Intrv_Intervals; the port makes it a class with
/// OCCT's copy as the copy constructor, and ported code copies where C++ copies (decision 1077).
/// </remarks>
public sealed class HLRAlgo_EdgeStatus
{
    private double myStart;
    private double myEnd;
    private float myTolStart;
    private float myTolEnd;
    private bool myAllHidden;
    private bool myAllVisible;
    private Intrv_Intervals myVisibles;

    public HLRAlgo_EdgeStatus()
    {
        myStart = 0.0;
        myEnd = 0.0;
        myTolStart = 0.0f;
        myTolEnd = 0.0f;
        myAllHidden = false;
        myAllVisible = false;
        myVisibles = new Intrv_Intervals();
    }

    /// <summary>An edge bounded by [Start, End] with the given tolerances, wholly visible.</summary>
    public HLRAlgo_EdgeStatus(double Start, float TolStart, double End, float TolEnd)
    {
        myStart = Start;
        myEnd = End;
        myTolStart = TolStart;
        myTolEnd = TolEnd;
        myAllHidden = false;
        myAllVisible = false;
        myVisibles = new Intrv_Intervals();
        ShowAll();
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's copy constructor).</summary>
    public HLRAlgo_EdgeStatus(HLRAlgo_EdgeStatus theOther)
    {
        myStart = theOther.myStart;
        myEnd = theOther.myEnd;
        myTolStart = theOther.myTolStart;
        myTolEnd = theOther.myTolEnd;
        myAllHidden = theOther.myAllHidden;
        myAllVisible = theOther.myAllVisible;
        myVisibles = new Intrv_Intervals(theOther.myVisibles);
    }

    /// <summary>OCCT's assignment: a copy of <paramref name="theOther"/>'s state.</summary>
    public HLRAlgo_EdgeStatus Assign(HLRAlgo_EdgeStatus theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            myStart = theOther.myStart;
            myEnd = theOther.myEnd;
            myTolStart = theOther.myTolStart;
            myTolEnd = theOther.myTolEnd;
            myAllHidden = theOther.myAllHidden;
            myAllVisible = theOther.myAllVisible;
            myVisibles = new Intrv_Intervals(theOther.myVisibles);
        }
        return this;
    }

    /// <summary>Initializes the edge to [Start, End] with the given tolerances, wholly visible.</summary>
    public void Initialize(double Start, float TolStart, double End, float TolEnd)
    {
        myStart = Start;
        myTolStart = TolStart;
        myEnd = End;
        myTolEnd = TolEnd;
        ShowAll();
    }

    public void Bounds(out double theStart, out float theTolStart, out double theEnd, out float theTolEnd)
    {
        theStart = myStart;
        theTolStart = myTolStart;
        theEnd = myEnd;
        theTolEnd = myTolEnd;
    }

    /// <summary>The number of visible parts: 0 if wholly hidden, 1 if wholly visible.</summary>
    public int NbVisiblePart()
    {
        if (AllHidden())
        {
            return 0;
        }
        else if (AllVisible())
        {
            return 1;
        }
        else
        {
            return myVisibles.NbIntervals();
        }
    }

    /// <summary>The visible part <paramref name="Index"/> (the bounds if wholly visible).</summary>
    public void VisiblePart(int Index, out double Start, out float TolStart, out double End, out float TolEnd)
    {
        if (AllVisible())
        {
            Bounds(out Start, out TolStart, out End, out TolEnd);
        }
        else
        {
            myVisibles.Value(Index).Bounds(out Start, out TolStart, out End, out TolEnd);
        }
    }

    /// <summary>
    /// Hides [Start, End] with the given tolerances: the interval is subtracted from the visible parts, unless the
    /// hidden part is on the face (<paramref name="OnFace"/>). <paramref name="OnBoundary"/> is not used.
    /// </summary>
    public void Hide(double Start, float TolStart, double End, float TolEnd, bool OnFace, bool OnBoundary)
    {
        if (!OnFace)
        {
            if (AllVisible())
            {
                myVisibles = new Intrv_Intervals(new Intrv_Interval(myStart, myTolStart, myEnd, myTolEnd));
                AllVisible(false);
            }
            myVisibles.Subtract(new Intrv_Interval(Start, TolStart, End, TolEnd));
            if (!AllHidden())
            {
                AllHidden(myVisibles.NbIntervals() == 0);
            }
        }
    }

    /// <summary>Hides the whole edge.</summary>
    public void HideAll()
    {
        AllVisible(false);
        AllHidden(true);
    }

    /// <summary>Shows the whole edge.</summary>
    public void ShowAll()
    {
        AllVisible(true);
        AllHidden(false);
    }

    public bool AllHidden() => myAllHidden;

    public void AllHidden(bool B) => myAllHidden = B;

    public bool AllVisible() => myAllVisible;

    public void AllVisible(bool B) => myAllVisible = B;
}
