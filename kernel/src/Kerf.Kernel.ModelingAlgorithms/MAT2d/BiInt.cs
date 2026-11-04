// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_BiInt.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_BiInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>A pair of integers (a key of MAT2d_Tool2d's and MAT2d_Circuit's maps).</summary>
/// <remarks>A value type (OCCT uses it by value): the setters are internal, equality compares both integers
/// exactly, as OCCT's IsEqual and std::hash.</remarks>
internal struct BiInt : IEquatable<BiInt>
{
    private int _i1;
    private int _i2;

    public BiInt(int i1, int i2)
    {
        _i1 = i1;
        _i2 = i2;
    }

    public readonly int FirstIndex() => _i1;

    public readonly int SecondIndex() => _i2;

    internal void FirstIndex(int i1) => _i1 = i1;

    internal void SecondIndex(int i2) => _i2 = i2;

    /// <summary>A copy with the first index I1.</summary>
    public readonly BiInt WithFirstIndex(int i1) => new(i1, _i2);

    /// <summary>A copy with the second index I2.</summary>
    public readonly BiInt WithSecondIndex(int i2) => new(_i1, i2);

    public readonly bool IsEqual(in BiInt b) => _i1 == b.FirstIndex() && _i2 == b.SecondIndex();

    public readonly bool Equals(BiInt other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is BiInt other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_i1, _i2);

    public static bool operator ==(BiInt left, BiInt right) => left.Equals(right);

    public static bool operator !=(BiInt left, BiInt right) => !left.Equals(right);

    public override readonly string ToString() => "MAT2d_BiInt(" + _i1 + ", " + _i2 + ")";
}
