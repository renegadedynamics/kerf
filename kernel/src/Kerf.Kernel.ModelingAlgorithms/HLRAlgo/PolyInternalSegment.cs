// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyInternalSegment.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>A segment of the internal triangulation of the polygonal hidden-line removal (to update the outlines).</summary>
/// <remarks>A plain C++ struct, ported with OCCT's public fields (plain data, changed in place in arrays).</remarks>
internal struct PolyInternalSegment : IEquatable<PolyInternalSegment>
{
    public int LstSg1;
    public int LstSg2;
    public int NxtSg1;
    public int NxtSg2;
    public int Conex1;
    public int Conex2;

    public readonly bool Equals(PolyInternalSegment other)
        => LstSg1 == other.LstSg1 && LstSg2 == other.LstSg2 && NxtSg1 == other.NxtSg1 && NxtSg2 == other.NxtSg2
           && Conex1 == other.Conex1 && Conex2 == other.Conex2;

    public override readonly bool Equals(object? obj) => obj is PolyInternalSegment other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(LstSg1, LstSg2, NxtSg1, NxtSg2, Conex1, Conex2);

    public static bool operator ==(in PolyInternalSegment left, in PolyInternalSegment right)
        => left.Equals(right);

    public static bool operator !=(in PolyInternalSegment left, in PolyInternalSegment right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"HLRAlgo_PolyInternalSegment({LstSg1}, {LstSg2}, {NxtSg1}, {NxtSg2}, {Conex1}, {Conex2})";
}
