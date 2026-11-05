// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeSample.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The index and depth of a sample range of a curve: at depth d the curve's range is cut into NbSample^d equal parts,
/// and the index picks one.
/// </summary>
/// <remarks>
/// A struct, as OCCT's by-value class. OCCT's base class IntTools_BaseRangeSample (the depth) is folded in, since a
/// struct can't derive (decision 628); <see cref="BaseRangeSample"/> gives it. Equality is OCCT's
/// <c>IsEqual</c> (index and depth); the hash replaces OCCT's <c>std::hash</c> specialisation, so maps keyed by
/// samples need no comparer.
/// </remarks>
internal partial struct CurveRangeSample : IEquatable<CurveRangeSample>
{
    private int _depth;
    private int _index;

    public CurveRangeSample()
    {
        _depth = 0;
        _index = 0;
    }

    public CurveRangeSample(int index)
    {
        _depth = 0;
        _index = index;
    }

    // ---- IntTools_BaseRangeSample

    internal void SetDepth(int depth) => _depth = depth;

    public readonly int GetDepth() => _depth;

    /// <summary>The base-class part (the depth).</summary>
    public readonly BaseRangeSample BaseRangeSample() => new(_depth);

    // ---- IntTools_CurveRangeSample

    internal void SetRangeIndex(int index) => _index = index;

    public readonly int GetRangeIndex() => _index;

    public readonly bool IsEqual(in CurveRangeSample other) =>
        (_index == other._index) && (GetDepth() == other.GetDepth());

    /// <summary>The sample's range of [<paramref name="first"/>, <paramref name="last"/>] cut into NbSample^depth parts.</summary>
    public readonly ParameterRange GetRange(double first, double last, int nbSample)
    {
        double diffC = last - first;
        ParameterRange result = new();

        if (GetDepth() <= 0)
        {
            result.SetFirst(first);
            result.SetLast(last);
        }
        else
        {
            double tmp = Math.Pow(nbSample, GetDepth());
            double localdiffC = diffC / tmp;
            double firstC = first + _index * localdiffC;
            double lastC = firstC + localdiffC;
            result.SetFirst(firstC);
            result.SetLast(lastC);
        }
        return result;
    }

    /// <summary>The index of the first sub-range one level deeper.</summary>
    public readonly int GetRangeIndexDeeper(int nbSample) => _index * nbSample;

    // ---- Kerf additions

    public readonly CurveRangeSample WithDepth(int depth)
    {
        CurveRangeSample copy = this;
        copy.SetDepth(depth);
        return copy;
    }

    public readonly CurveRangeSample WithRangeIndex(int index)
    {
        CurveRangeSample copy = this;
        copy.SetRangeIndex(index);
        return copy;
    }

    public readonly bool Equals(CurveRangeSample other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is CurveRangeSample other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_depth, _index);

    public static bool operator ==(CurveRangeSample left, CurveRangeSample right) => left.IsEqual(right);

    public static bool operator !=(CurveRangeSample left, CurveRangeSample right) => !left.IsEqual(right);

    public override readonly string ToString() => $"IntTools_CurveRangeSample(index {_index}, depth {_depth})";
}
