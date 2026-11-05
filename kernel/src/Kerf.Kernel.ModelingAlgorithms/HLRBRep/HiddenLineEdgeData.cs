// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Topology.Brep;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The hidden-line data of an edge: its flags (selected, used, smooth, vertical, simple, outline vertices, cuts,
/// vertical ends, self-intersection done), hiding count, vertex indices, encoded min-max box, visibility status, and
/// projected curve.
/// </summary>
/// <remarks>
/// OCCT's value class, ported as a class (decisions 1084, 1092): it holds its own HLRAlgo_EdgeStatus and
/// HLRBRep_Curve, and HLRBRep_Data's arrays hold one instance per edge, which OCCT's code reaches by reference or
/// pointer. OCCT's copy assignment (<c>*ed = *e1</c> in <c>HLRBRep_Data::Write</c>) is <see cref="Assign"/>: a deep
/// copy of the status and a copy of the curve. A working object: its setters are public, as OCCT's.
/// </remarks>
internal sealed class HiddenLineEdgeData
{
    private const int EMaskSelected = 1;
    private const int EMaskUsed = 2;
    private const int EMaskRg1Line = 4;
    private const int EMaskVertical = 8;
    private const int EMaskSimple = 16;
    private const int EMaskOutLVSta = 32;
    private const int EMaskOutLVEnd = 64;
    private const int EMaskIntDone = 128;
    private const int EMaskCutAtSta = 256;
    private const int EMaskCutAtEnd = 512;
    private const int EMaskVerAtSta = 1024;
    private const int EMaskVerAtEnd = 2048;
    private const int EMaskRgNLine = 4096;

    private int _flags;
    private int _hideCount;
    private int _vSta;
    private int _vEnd;
    private EdgesBlock.MinMaxIndices _minMax;
    private readonly EdgeStatus _status = new();
    private HiddenLineCurve _geometry = new();
    private float _tolerance;

    public HiddenLineEdgeData()
    {
        _flags = 0;
        _hideCount = 0;
        Selected(true);
    }

    /// <summary>OCCT's copy assignment: the flags, indices and box copied, the status and the curve copied.</summary>
    public HiddenLineEdgeData Assign(HiddenLineEdgeData other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }
        _flags = other._flags;
        _hideCount = other._hideCount;
        _vSta = other._vSta;
        _vEnd = other._vEnd;
        _minMax = other._minMax;
        _status.Assign(other._status);
        _geometry = other._geometry.Copy();
        _tolerance = other._tolerance;
        return this;
    }

    /// <summary>Sets the edge, its vertices and their flags, and the bounds of its status.</summary>
    public void Set(bool rg1L, bool rgNL, in Edge eg, int v1, int v2, bool out1, bool out2, bool cut1, bool cut2,
                    double start, float tolStart, double end, float tolEnd)
    {
        Rg1Line(rg1L);
        RgNLine(rgNL);
        Used(false);
        ChangeGeometry().Curve(eg);
        _tolerance = (float)(Tool.Tolerance(eg));
        VSta(v1);
        VEnd(v2);
        OutLVSta(out1);
        OutLVEnd(out2);
        CutAtSta(cut1);
        CutAtEnd(cut2);
        Status().Initialize(start, (float)(ChangeGeometry().Curve().Resolution((double)tolStart)), end,
                            (float)(ChangeGeometry().Curve().Resolution((double)tolEnd)));
    }

    private bool Flag(int mask) => (_flags & mask) != 0;

    private void Flag(int mask, bool b)
    {
        if (b)
        {
            _flags |= mask;
        }
        else
        {
            _flags &= ~mask;
        }
    }

    public bool Selected() => Flag(EMaskSelected);

    public void Selected(bool b) => Flag(EMaskSelected, b);

    public bool Rg1Line() => Flag(EMaskRg1Line);

    public void Rg1Line(bool b) => Flag(EMaskRg1Line, b);

    public bool RgNLine() => Flag(EMaskRgNLine);

    public void RgNLine(bool b) => Flag(EMaskRgNLine, b);

    public bool Vertical() => Flag(EMaskVertical);

    public void Vertical(bool b) => Flag(EMaskVertical, b);

    public bool Simple() => Flag(EMaskSimple);

    public void Simple(bool b) => Flag(EMaskSimple, b);

    public bool OutLVSta() => Flag(EMaskOutLVSta);

    public void OutLVSta(bool b) => Flag(EMaskOutLVSta, b);

    public bool OutLVEnd() => Flag(EMaskOutLVEnd);

    public void OutLVEnd(bool b) => Flag(EMaskOutLVEnd, b);

    public bool CutAtSta() => Flag(EMaskCutAtSta);

    public void CutAtSta(bool b) => Flag(EMaskCutAtSta, b);

    public bool CutAtEnd() => Flag(EMaskCutAtEnd);

    public void CutAtEnd(bool b) => Flag(EMaskCutAtEnd, b);

    public bool VerAtSta() => Flag(EMaskVerAtSta);

    public void VerAtSta(bool b) => Flag(EMaskVerAtSta, b);

    public bool VerAtEnd() => Flag(EMaskVerAtEnd);

    public void VerAtEnd(bool b) => Flag(EMaskVerAtEnd, b);

    public bool AutoIntersectionDone() => Flag(EMaskIntDone);

    public void AutoIntersectionDone(bool b) => Flag(EMaskIntDone, b);

    public bool Used() => Flag(EMaskUsed);

    public void Used(bool b) => Flag(EMaskUsed, b);

    public int HideCount() => _hideCount;

    public void HideCount(int i) => _hideCount = i;

    public int VSta() => _vSta;

    public void VSta(int i) => _vSta = i;

    public int VEnd() => _vEnd;

    public void VEnd(int i) => _vEnd = i;

    public void UpdateMinMax(in EdgesBlock.MinMaxIndices totMinMax) => _minMax = totMinMax;

    /// <summary>The encoded min-max box (a reference, as OCCT's).</summary>
    public ref EdgesBlock.MinMaxIndices MinMax() => ref _minMax;

    /// <summary>The visibility status (the stored instance, as OCCT's reference).</summary>
    public EdgeStatus Status() => _status;

    public HiddenLineCurve ChangeGeometry() => _geometry;

    public HiddenLineCurve Geometry() => _geometry;

    /// <summary>The projected curve (OCCT's pointer to it, the curve type of the 2D intersections).</summary>
    public HiddenLineCurve Curve() => _geometry;

    public float Tolerance() => _tolerance;
}
