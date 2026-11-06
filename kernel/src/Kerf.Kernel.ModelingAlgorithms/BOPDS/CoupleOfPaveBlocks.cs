// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CoupleOfPaveBlocks.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Booleans.Data;

/// <summary>Two pave blocks with an index, the index of an interference and a tolerance.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class: it holds the two pave blocks by reference (handles in C++), so a copy shares
/// them as the C++ copy does. OCCT's deprecated <c>PaveBlocks(PB1, PB2)</c> is left out.
/// </remarks>
internal partial struct CoupleOfPaveBlocks
{
    private int _indexInterf;
    private int _index;
    private PaveBlock? _pb1;
    private PaveBlock? _pb2;
    private double _tolerance;

    public CoupleOfPaveBlocks()
    {
        _indexInterf = -1;
        _index = -1;
        _tolerance = 0;
    }

    public CoupleOfPaveBlocks(PaveBlock? pb1, PaveBlock? pb2)
    {
        _indexInterf = -1;
        _index = -1;
        _tolerance = 0;
        SetPaveBlocks(pb1, pb2);
    }

    internal void SetIndex(int index) => _index = index;

    public readonly int Index() => _index;

    internal void SetIndexInterf(int index) => _indexInterf = index;

    public readonly int IndexInterf() => _indexInterf;

    internal void SetPaveBlocks(PaveBlock? pb1, PaveBlock? pb2)
    {
        _pb1 = pb1;
        _pb2 = pb2;
    }

    internal void SetPaveBlock1(PaveBlock? pb) => _pb1 = pb;

    public readonly PaveBlock? PaveBlock1() => _pb1;

    internal void SetPaveBlock2(PaveBlock? pb) => _pb2 = pb;

    public readonly PaveBlock? PaveBlock2() => _pb2;

    internal void SetTolerance(double tol) => _tolerance = tol;

    public readonly double Tolerance() => _tolerance;
}
