// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Line.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Line.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Walking;
using Occt;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The line a blend's walking builds: its points (<see cref="BlendPoint"/>), its extremities on both supports and
/// the transitions on them.
/// </summary>
/// <remarks>
/// A working object (decision 21): the walkers fill it and the fillet builder reads it; its mutators are internal.
/// It is the line of <see cref="AppSurf"/>'s approximation (<see cref="ILine{ThePoint}"/>).
/// </remarks>
internal class Line : Transient, ILine<BlendPoint>
{
    private readonly NCollection_Sequence<BlendPoint> _seqpt = new();
    private WalkingTypeTrans _tras1;
    private WalkingTypeTrans _tras2;
    private Extremity _stp1 = new();
    private Extremity _stp2 = new();
    private Extremity _endp1 = new();
    private Extremity _endp2 = new();
    private bool _hass1;
    private bool _hass2;

    public Line()
    {
        _tras1 = WalkingTypeTrans.Undecided;
        _tras2 = WalkingTypeTrans.Undecided;
        _hass1 = false;
        _hass2 = false;
    }

    internal void Clear()
    {
        _seqpt.Clear();
        _hass1 = false;
        _hass2 = false;
        _tras1 = WalkingTypeTrans.Undecided;
        _tras2 = WalkingTypeTrans.Undecided;
    }

    internal void Append(in BlendPoint p) => _seqpt.Append(p);

    internal void Prepend(in BlendPoint p) => _seqpt.Prepend(p);

    internal void InsertBefore(int index, in BlendPoint p) => _seqpt.InsertBefore(index, p);

    internal void Remove(int fromIndex, int toIndex) => _seqpt.Remove(fromIndex, toIndex);

    internal void Set(WalkingTypeTrans tranS1, WalkingTypeTrans tranS2)
    {
        _hass1 = true;
        _hass2 = true;
        _tras1 = tranS1;
        _tras2 = tranS2;
    }

    internal void Set(WalkingTypeTrans trans)
    {
        _hass1 = true;
        _hass2 = false;
        _tras1 = trans;
    }

    internal void SetStartPoints(in Extremity startPtOnS1, in Extremity startPtOnS2)
    {
        _stp1 = startPtOnS1;
        _stp2 = startPtOnS2;
    }

    internal void SetEndPoints(in Extremity endPtOnS1, in Extremity endPtOnS2)
    {
        _endp1 = endPtOnS1;
        _endp2 = endPtOnS2;
    }

    public int NbPoints() => _seqpt.Length();

    public BlendPoint Point(int index) => _seqpt.Value(index);

    /// <summary>The stored point at <paramref name="index"/>, for the builders that edit it in place.</summary>
    internal ref BlendPoint ChangePoint(int index) => ref _seqpt.ChangeValue(index);

    public WalkingTypeTrans TransitionOnS1()
    {
        if (!_hass1)
        {
            throw new DomainErrorException();
        }

        return _tras1;
    }

    public WalkingTypeTrans TransitionOnS2()
    {
        if (!_hass2)
        {
            throw new DomainErrorException();
        }

        return _tras2;
    }

    public Extremity StartPointOnFirst() => _stp1;

    public Extremity StartPointOnSecond() => _stp2;

    public Extremity EndPointOnFirst() => _endp1;

    public Extremity EndPointOnSecond() => _endp2;

    public WalkingTypeTrans TransitionOnS()
    {
        if (!_hass1)
        {
            throw new DomainErrorException();
        }

        return _tras1;
    }
}
