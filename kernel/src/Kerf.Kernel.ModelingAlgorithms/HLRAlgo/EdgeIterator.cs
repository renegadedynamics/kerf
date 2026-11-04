// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeIterator.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeIterator.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_EdgeIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>An iterator on the visible or the hidden parts of an edge's status.</summary>
/// <remarks>A working object, as OCCT's; it keeps the statuses (OCCT's pointers) by reference.</remarks>
internal sealed class EdgeIterator
{
    private int _nbVis;
    private int _nbHid;
    private EdgeStatus? _eVis;
    private EdgeStatus? _eHid;
    private int _iVis;
    private int _iHid;
    private double _hidStart;
    private double _hidEnd;
    private float _hidTolStart;
    private float _hidTolEnd;

    public EdgeIterator()
    {
        _nbVis = 0;
        _nbHid = 0;
        _eVis = null;
        _eHid = null;
        _iVis = 0;
        _iHid = 0;
        _hidStart = 0.0;
        _hidEnd = 0.0;
        _hidTolStart = 0.0f;
        _hidTolEnd = 0.0f;
    }

    /// <summary>Starts the iteration on the hidden parts of <paramref name="status"/>.</summary>
    public void InitHidden(EdgeStatus status)
    {
        _eHid = status;
        _iHid = 1;
        if (_eHid.AllHidden())
        {
            _eHid.Bounds(out _hidStart, out _hidTolStart, out _hidEnd, out _hidTolEnd);
            _nbHid = 0;
        }
        else
        {
            _nbHid = _eHid.NbVisiblePart();
            _eHid.Bounds(out _hidStart, out _hidTolStart, out _, out _);
            _eHid.VisiblePart(_iHid, out _hidEnd, out _hidTolEnd, out _, out _);
        }
        if (_hidStart + _hidTolStart >= _hidEnd - _hidTolEnd
            && _hidEnd + _hidTolEnd >= _hidStart - _hidTolStart)
        {
            NextHidden();
        }
    }

    public bool MoreHidden() => _iHid <= _nbHid + 1;

    public void NextHidden()
    {
        if (_iHid >= _nbHid + 1)
        {
            _iHid++;
        }
        else
        {
            _eHid!.VisiblePart(_iHid, out _, out _, out _hidStart, out _hidTolStart);
            _iHid++;
            if (_iHid == _nbHid + 1)
            {
                _eHid.Bounds(out _, out _, out _hidEnd, out _hidTolEnd);
                if (_hidStart + _hidTolStart >= _hidEnd - _hidTolEnd
                    && _hidEnd + _hidTolEnd >= _hidStart - _hidTolStart)
                {
                    _iHid++;
                }
            }
            else
            {
                _eHid.VisiblePart(_iHid, out _hidEnd, out _hidTolEnd, out _, out _);
            }
        }
    }

    /// <summary>The bounds and tolerances of the current hidden interval.</summary>
    public void Hidden(out double start, out float tolStart, out double end, out float tolEnd)
    {
        start = _hidStart;
        tolStart = _hidTolStart;
        end = _hidEnd;
        tolEnd = _hidTolEnd;
    }

    /// <summary>Starts the iteration on the visible parts of <paramref name="status"/>.</summary>
    public void InitVisible(EdgeStatus status)
    {
        _eVis = status;
        _iVis = 1;
        _nbVis = _eVis.NbVisiblePart();
    }

    public bool MoreVisible() => _iVis <= _nbVis;

    public void NextVisible()
    {
        _iVis++;
    }

    /// <summary>The bounds and tolerances of the current visible interval.</summary>
    public void Visible(out double start, out float tolStart, out double end, out float tolEnd)
    {
        _eVis!.VisiblePart(_iVis, out start, out tolStart, out end, out tolEnd);
    }
}
