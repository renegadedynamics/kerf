// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_WiresBlock.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_WiresBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// A block of edge blocks of the hidden-line data structure (the wires of a face), with the block's encoded min-max
/// box.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's. OCCT stores the blocks as transients and casts; the port types them.</remarks>
internal class WiresBlock : Transient
{
    private readonly NCollection_Array1<EdgesBlock?> _wires;
    private EdgesBlock.MinMaxIndices _minMax;

    /// <summary>A block of <paramref name="nbWires"/> wires.</summary>
    public WiresBlock(int nbWires)
    {
        _wires = new NCollection_Array1<EdgesBlock?>(1, nbWires);
    }

    public int WireCount => _wires.Upper();

    public void Set(int i, EdgesBlock? w) => _wires[i] = w;

    /// <summary>The wire <paramref name="i"/>, by reference.</summary>
    public ref EdgesBlock? Wire(int i) => ref _wires[i];

    public void UpdateMinMax(in EdgesBlock.MinMaxIndices minMaxes) => _minMax = minMaxes;

    /// <summary>The block's encoded min-max box, by reference.</summary>
    public ref EdgesBlock.MinMaxIndices MinMax() => ref _minMax;
}
