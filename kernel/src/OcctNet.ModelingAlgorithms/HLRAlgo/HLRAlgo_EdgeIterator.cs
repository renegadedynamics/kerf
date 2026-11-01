// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeIterator.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeIterator.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An iterator on the visible or the hidden parts of an edge's status.</summary>
/// <remarks>A working object, as OCCT's; it keeps the statuses (OCCT's pointers) by reference.</remarks>
public sealed class HLRAlgo_EdgeIterator
{
    private int myNbVis;
    private int myNbHid;
    private HLRAlgo_EdgeStatus? EVis;
    private HLRAlgo_EdgeStatus? EHid;
    private int iVis;
    private int iHid;
    private double myHidStart;
    private double myHidEnd;
    private float myHidTolStart;
    private float myHidTolEnd;

    public HLRAlgo_EdgeIterator()
    {
        myNbVis = 0;
        myNbHid = 0;
        EVis = null;
        EHid = null;
        iVis = 0;
        iHid = 0;
        myHidStart = 0.0;
        myHidEnd = 0.0;
        myHidTolStart = 0.0f;
        myHidTolEnd = 0.0f;
    }

    /// <summary>Starts the iteration on the hidden parts of <paramref name="status"/>.</summary>
    public void InitHidden(HLRAlgo_EdgeStatus status)
    {
        EHid = status;
        iHid = 1;
        if (EHid.AllHidden())
        {
            EHid.Bounds(out myHidStart, out myHidTolStart, out myHidEnd, out myHidTolEnd);
            myNbHid = 0;
        }
        else
        {
            myNbHid = EHid.NbVisiblePart();
            EHid.Bounds(out myHidStart, out myHidTolStart, out _, out _);
            EHid.VisiblePart(iHid, out myHidEnd, out myHidTolEnd, out _, out _);
        }
        if (myHidStart + myHidTolStart >= myHidEnd - myHidTolEnd
            && myHidEnd + myHidTolEnd >= myHidStart - myHidTolStart)
        {
            NextHidden();
        }
    }

    public bool MoreHidden() => iHid <= myNbHid + 1;

    public void NextHidden()
    {
        if (iHid >= myNbHid + 1)
        {
            iHid++;
        }
        else
        {
            EHid!.VisiblePart(iHid, out _, out _, out myHidStart, out myHidTolStart);
            iHid++;
            if (iHid == myNbHid + 1)
            {
                EHid.Bounds(out _, out _, out myHidEnd, out myHidTolEnd);
                if (myHidStart + myHidTolStart >= myHidEnd - myHidTolEnd
                    && myHidEnd + myHidTolEnd >= myHidStart - myHidTolStart)
                {
                    iHid++;
                }
            }
            else
            {
                EHid.VisiblePart(iHid, out myHidEnd, out myHidTolEnd, out _, out _);
            }
        }
    }

    /// <summary>The bounds and tolerances of the current hidden interval.</summary>
    public void Hidden(out double Start, out float TolStart, out double End, out float TolEnd)
    {
        Start = myHidStart;
        TolStart = myHidTolStart;
        End = myHidEnd;
        TolEnd = myHidTolEnd;
    }

    /// <summary>Starts the iteration on the visible parts of <paramref name="status"/>.</summary>
    public void InitVisible(HLRAlgo_EdgeStatus status)
    {
        EVis = status;
        iVis = 1;
        myNbVis = EVis.NbVisiblePart();
    }

    public bool MoreVisible() => iVis <= myNbVis;

    public void NextVisible()
    {
        iVis++;
    }

    /// <summary>The bounds and tolerances of the current visible interval.</summary>
    public void Visible(out double Start, out float TolStart, out double End, out float TolEnd)
    {
        EVis!.VisiblePart(iVis, out Start, out TolStart, out End, out TolEnd);
    }
}
