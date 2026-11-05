// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeStatus.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeStatus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection;

using Occt;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// The hidden-line status of an edge: its bounds and their tolerances, whether it is wholly visible or wholly
/// hidden, and otherwise its visible intervals.
/// </summary>
/// <remarks>
/// OCCT's class is used by value (in HLRBRep_EdgeData) but holds an Intrv_Intervals; the port makes it a class with
/// OCCT's copy as the copy constructor, and ported code copies where C++ copies (decision 1077).
/// </remarks>
internal sealed class EdgeStatus
{
    private double _start;
    private double _end;
    private float _tolStart;
    private float _tolEnd;
    private bool _allHidden;
    private bool _allVisible;
    private Intervals _visibles;

    public EdgeStatus()
    {
        _start = 0.0;
        _end = 0.0;
        _tolStart = 0.0f;
        _tolEnd = 0.0f;
        _allHidden = false;
        _allVisible = false;
        _visibles = new Intervals();
    }

    /// <summary>An edge bounded by [Start, End] with the given tolerances, wholly visible.</summary>
    public EdgeStatus(double start, float tolStart, double end, float tolEnd)
    {
        _start = start;
        _end = end;
        _tolStart = tolStart;
        _tolEnd = tolEnd;
        _allHidden = false;
        _allVisible = false;
        _visibles = new Intervals();
        ShowAll();
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's copy constructor).</summary>
    public EdgeStatus(EdgeStatus other)
    {
        _start = other._start;
        _end = other._end;
        _tolStart = other._tolStart;
        _tolEnd = other._tolEnd;
        _allHidden = other._allHidden;
        _allVisible = other._allVisible;
        _visibles = new Intervals(other._visibles);
    }

    /// <summary>OCCT's assignment: a copy of <paramref name="other"/>'s state.</summary>
    public EdgeStatus Assign(EdgeStatus other)
    {
        if (!ReferenceEquals(this, other))
        {
            _start = other._start;
            _end = other._end;
            _tolStart = other._tolStart;
            _tolEnd = other._tolEnd;
            _allHidden = other._allHidden;
            _allVisible = other._allVisible;
            _visibles = new Intervals(other._visibles);
        }
        return this;
    }

    /// <summary>Initializes the edge to [Start, End] with the given tolerances, wholly visible.</summary>
    public void Initialize(double start, float tolStart, double end, float tolEnd)
    {
        _start = start;
        _tolStart = tolStart;
        _end = end;
        _tolEnd = tolEnd;
        ShowAll();
    }

    public void Bounds(out double start, out float tolStart, out double end, out float tolEnd)
    {
        start = _start;
        tolStart = _tolStart;
        end = _end;
        tolEnd = _tolEnd;
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
            return _visibles.IntervalCount;
        }
    }

    /// <summary>The visible part <paramref name="index"/> (the bounds if wholly visible).</summary>
    public void VisiblePart(int index, out double start, out float tolStart, out double end, out float tolEnd)
    {
        if (AllVisible())
        {
            Bounds(out start, out tolStart, out end, out tolEnd);
        }
        else
        {
            _visibles.Value(index).Bounds(out start, out tolStart, out end, out tolEnd);
        }
    }

    /// <summary>
    /// Hides [Start, End] with the given tolerances: the interval is subtracted from the visible parts, unless the
    /// hidden part is on the face (<paramref name="onFace"/>). <paramref name="onBoundary"/> is not used.
    /// </summary>
    public void Hide(double start, float tolStart, double end, float tolEnd, bool onFace, bool onBoundary)
    {
        if (!onFace)
        {
            if (AllVisible())
            {
                _visibles = new Intervals(new IntersectionInterval(_start, _tolStart, _end, _tolEnd));
                AllVisible(false);
            }
            _visibles.Subtract(new IntersectionInterval(start, tolStart, end, tolEnd));
            if (!AllHidden())
            {
                AllHidden(_visibles.IntervalCount == 0);
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

    public bool AllHidden() => _allHidden;

    public void AllHidden(bool b) => _allHidden = b;

    public bool AllVisible() => _allVisible;

    public void AllVisible(bool b) => _allVisible = b;
}
