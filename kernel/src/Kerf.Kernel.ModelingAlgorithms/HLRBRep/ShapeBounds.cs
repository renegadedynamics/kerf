// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeBounds.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeBounds.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeBounds.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Topology;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The bounds of a shape loaded in the hidden-line data: its outliner and data, its number of isoparametric lines,
/// the ranges of its vertices, edges and faces in the data's arrays, and its encoded min-max box.
/// </summary>
/// <remarks>
/// OCCT's value class (held by value in HLRBRep_InternalAlgo's sequence); a class here, with OCCT's implicit copy as
/// <see cref="ShapeBounds(ShapeBounds)"/> (the handles shared). A working object.
/// </remarks>
internal sealed class ShapeBounds
{
    private OutLiner? _shape;
    private Transient? _shapeData;
    private int _nbIso;
    private int _vertStart;
    private int _vertEnd;
    private int _edgeStart;
    private int _edgeEnd;
    private int _faceStart;
    private int _faceEnd;
    private EdgesBlock.MinMaxIndices _minMax;

    public ShapeBounds()
    {
        _nbIso = 0;
        _vertStart = 0;
        _vertEnd = 0;
        _edgeStart = 0;
        _edgeEnd = 0;
        _faceStart = 0;
        _faceEnd = 0;
    }

    public ShapeBounds(OutLiner s, Transient? sData, int nbIso, int v1, int v2, int e1,
                               int e2, int f1, int f2)
    {
        _shape = s;
        _shapeData = sData;
        _nbIso = nbIso;
        _vertStart = v1;
        _vertEnd = v2;
        _edgeStart = e1;
        _edgeEnd = e2;
        _faceStart = f1;
        _faceEnd = f2;
    }

    public ShapeBounds(OutLiner s, int nbIso, int v1, int v2, int e1, int e2, int f1, int f2)
    {
        _shape = s;
        _nbIso = nbIso;
        _vertStart = v1;
        _vertEnd = v2;
        _edgeStart = e1;
        _edgeEnd = e2;
        _faceStart = f1;
        _faceEnd = f2;
    }

    /// <summary>OCCT's implicit copy (the handles shared).</summary>
    public ShapeBounds(ShapeBounds other)
    {
        _shape = other._shape;
        _shapeData = other._shapeData;
        _nbIso = other._nbIso;
        _vertStart = other._vertStart;
        _vertEnd = other._vertEnd;
        _edgeStart = other._edgeStart;
        _edgeEnd = other._edgeEnd;
        _faceStart = other._faceStart;
        _faceEnd = other._faceEnd;
        _minMax = other._minMax;
    }

    /// <summary>Shifts the ranges by NV vertices, NE edges and NF faces.</summary>
    public void Translate(int nv, int ne, int nf)
    {
        _vertStart += nv;
        _vertEnd += nv;
        _edgeStart += ne;
        _edgeEnd += ne;
        _faceStart += nf;
        _faceEnd += nf;
    }

    public void Shape(OutLiner? s) => _shape = s;

    public OutLiner? Shape() => _shape;

    public void ShapeData(Transient? sd) => _shapeData = sd;

    public Transient? ShapeData() => _shapeData;

    public void NbOfIso(int nbIso) => _nbIso = nbIso;

    public int OfIsoCount => _nbIso;

    /// <summary>The numbers of vertices, edges and faces.</summary>
    public void Sizes(out int nv, out int ne, out int nf)
    {
        nv = _vertEnd + 1 - _vertStart;
        ne = _edgeEnd + 1 - _edgeStart;
        nf = _faceEnd + 1 - _faceStart;
    }

    /// <summary>The ranges of the vertices, edges and faces.</summary>
    public void Bounds(out int v1, out int v2, out int e1, out int e2, out int f1, out int f2)
    {
        v1 = _vertStart;
        v2 = _vertEnd;
        e1 = _edgeStart;
        e2 = _edgeEnd;
        f1 = _faceStart;
        f2 = _faceEnd;
    }

    public void UpdateMinMax(in EdgesBlock.MinMaxIndices totMinMax) => _minMax = totMinMax;

    /// <summary>The encoded min-max box (a reference, as OCCT's).</summary>
    public ref EdgesBlock.MinMaxIndices MinMax() => ref _minMax;
}
