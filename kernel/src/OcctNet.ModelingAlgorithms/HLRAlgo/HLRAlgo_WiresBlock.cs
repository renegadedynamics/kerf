// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_WiresBlock.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_WiresBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A block of edge blocks of the hidden-line data structure (the wires of a face), with the block's encoded min-max
/// box.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's. OCCT stores the blocks as transients and casts; the port types them.</remarks>
public class HLRAlgo_WiresBlock : Standard_Transient
{
    private readonly NCollection_Array1<HLRAlgo_EdgesBlock?> myWires;
    private HLRAlgo_EdgesBlock.MinMaxIndices myMinMax;

    /// <summary>A block of <paramref name="NbWires"/> wires.</summary>
    public HLRAlgo_WiresBlock(int NbWires)
    {
        myWires = new NCollection_Array1<HLRAlgo_EdgesBlock?>(1, NbWires);
    }

    public int NbWires() => myWires.Upper();

    public void Set(int I, HLRAlgo_EdgesBlock? W) => myWires[I] = W;

    /// <summary>The wire <paramref name="I"/>, by reference.</summary>
    public ref HLRAlgo_EdgesBlock? Wire(int I) => ref myWires[I];

    public void UpdateMinMax(in HLRAlgo_EdgesBlock.MinMaxIndices theMinMaxes) => myMinMax = theMinMaxes;

    /// <summary>The block's encoded min-max box, by reference.</summary>
    public ref HLRAlgo_EdgesBlock.MinMaxIndices MinMax() => ref myMinMax;
}
