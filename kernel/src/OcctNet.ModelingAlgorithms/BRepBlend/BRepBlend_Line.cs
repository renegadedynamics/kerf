// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Line.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Line.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// The line a blend's walking builds: its points (<see cref="Blend_Point"/>), its extremities on both supports and
/// the transitions on them.
/// </summary>
/// <remarks>
/// A working object (decision 21): the walkers fill it and the fillet builder reads it; its mutators are internal.
/// It is the line of <see cref="BRepBlend_AppSurf"/>'s approximation (<see cref="IAppBlend_Line{ThePoint}"/>).
/// </remarks>
public class BRepBlend_Line : Transient, IAppBlend_Line<Blend_Point>
{
    private readonly NCollection_Sequence<Blend_Point> seqpt = new();
    private IntSurf_TypeTrans tras1;
    private IntSurf_TypeTrans tras2;
    private BRepBlend_Extremity stp1 = new();
    private BRepBlend_Extremity stp2 = new();
    private BRepBlend_Extremity endp1 = new();
    private BRepBlend_Extremity endp2 = new();
    private bool hass1;
    private bool hass2;

    public BRepBlend_Line()
    {
        tras1 = IntSurf_Undecided;
        tras2 = IntSurf_Undecided;
        hass1 = false;
        hass2 = false;
    }

    internal void Clear()
    {
        seqpt.Clear();
        hass1 = false;
        hass2 = false;
        tras1 = IntSurf_Undecided;
        tras2 = IntSurf_Undecided;
    }

    internal void Append(in Blend_Point P) => seqpt.Append(P);

    internal void Prepend(in Blend_Point P) => seqpt.Prepend(P);

    internal void InsertBefore(int Index, in Blend_Point P) => seqpt.InsertBefore(Index, P);

    internal void Remove(int FromIndex, int ToIndex) => seqpt.Remove(FromIndex, ToIndex);

    internal void Set(IntSurf_TypeTrans TranS1, IntSurf_TypeTrans TranS2)
    {
        hass1 = true;
        hass2 = true;
        tras1 = TranS1;
        tras2 = TranS2;
    }

    internal void Set(IntSurf_TypeTrans Trans)
    {
        hass1 = true;
        hass2 = false;
        tras1 = Trans;
    }

    internal void SetStartPoints(in BRepBlend_Extremity StartPtOnS1, in BRepBlend_Extremity StartPtOnS2)
    {
        stp1 = StartPtOnS1;
        stp2 = StartPtOnS2;
    }

    internal void SetEndPoints(in BRepBlend_Extremity EndPtOnS1, in BRepBlend_Extremity EndPtOnS2)
    {
        endp1 = EndPtOnS1;
        endp2 = EndPtOnS2;
    }

    public int NbPoints() => seqpt.Length();

    public Blend_Point Point(int Index) => seqpt.Value(Index);

    /// <summary>The stored point at <paramref name="Index"/>, for the builders that edit it in place.</summary>
    internal ref Blend_Point ChangePoint(int Index) => ref seqpt.ChangeValue(Index);

    public IntSurf_TypeTrans TransitionOnS1()
    {
        if (!hass1)
        {
            throw new DomainErrorException();
        }

        return tras1;
    }

    public IntSurf_TypeTrans TransitionOnS2()
    {
        if (!hass2)
        {
            throw new DomainErrorException();
        }

        return tras2;
    }

    public BRepBlend_Extremity StartPointOnFirst() => stp1;

    public BRepBlend_Extremity StartPointOnSecond() => stp2;

    public BRepBlend_Extremity EndPointOnFirst() => endp1;

    public BRepBlend_Extremity EndPointOnSecond() => endp2;

    public IntSurf_TypeTrans TransitionOnS()
    {
        if (!hass1)
        {
            throw new DomainErrorException();
        }

        return tras1;
    }
}
