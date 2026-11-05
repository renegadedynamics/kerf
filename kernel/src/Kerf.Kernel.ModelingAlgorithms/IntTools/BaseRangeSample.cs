// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BaseRangeSample.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BaseRangeSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>The depth of a range sample: the base of range index management.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class. OCCT derives <see cref="CurveRangeSample"/> from it; a C# struct
/// can't derive, so the curve sample holds its own depth with the same accessors (decision 628).
/// </remarks>
internal partial struct BaseRangeSample : IEquatable<BaseRangeSample>
{
    private int _depth;

    public BaseRangeSample()
    {
        _depth = 0;
    }

    public BaseRangeSample(int depth)
    {
        _depth = depth;
    }

    internal void SetDepth(int depth) => _depth = depth;

    public readonly int GetDepth() => _depth;

    // ---- Kerf additions

    public readonly BaseRangeSample WithDepth(int depth)
    {
        BaseRangeSample copy = this;
        copy.SetDepth(depth);
        return copy;
    }

    public readonly bool Equals(BaseRangeSample other) => _depth == other._depth;

    public override readonly bool Equals(object? obj) => obj is BaseRangeSample other && Equals(other);

    public override readonly int GetHashCode() => _depth;

    public static bool operator ==(BaseRangeSample left, BaseRangeSample right) => left.Equals(right);

    public static bool operator !=(BaseRangeSample left, BaseRangeSample right) => !left.Equals(right);

    public override readonly string ToString() => $"IntTools_BaseRangeSample(depth {_depth})";
}
