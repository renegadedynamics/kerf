// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceIterator.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceIterator.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_FaceIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Topology;

namespace Kerf.Engine.HiddenLines;

/// <summary>Iterates on the edges of the wires of a face's hidden-line data.</summary>
/// <remarks>A working object, as OCCT's.</remarks>
internal sealed class FaceIterator
{
    private int _iWire;
    private int _nbWires;
    private int _iEdge;
    private int _nbEdges;
    private WiresBlock? _wires;
    private EdgesBlock? _edges;

    public FaceIterator()
    {
    }

    /// <summary>Starts on the first edge of the first wire of fd.</summary>
    public void InitEdge(HiddenLineFaceData fd)
    {
        _iWire = 0;
        _wires = fd.Wires();
        _nbWires = _wires!.WireCount;

        _iEdge = 0;
        _nbEdges = 0;
        NextEdge();
    }

    public bool MoreEdge() => _iWire <= _nbWires;

    public void NextEdge()
    {
        _iEdge++;
        if (_iEdge > _nbEdges)
        {
            _iWire++;
            if (_iWire <= _nbWires)
            {
                _iEdge = 1;
                _edges = _wires!.Wire(_iWire);
                _nbEdges = _edges!.EdgeCount;
            }
        }
    }

    public bool BeginningOfWire() => _iEdge == 1;

    public bool EndOfWire() => _iEdge == _nbEdges;

    public void SkipWire()
    {
        _iEdge = _nbEdges;
        NextEdge();
    }

    public EdgesBlock Wire() => _wires!.Wire(_iWire)!;

    public int Edge() => _edges!.Edge(_iEdge);

    public Orientation Orientation() => _edges!.Orientation(_iEdge);

    public bool OutLine() => _edges!.OutLine(_iEdge);

    public bool Internal() => _edges!.Internal(_iEdge);

    public bool Double() => _edges!.Double(_iEdge);

    public bool IsoLine() => _edges!.IsoLine(_iEdge);
}
