// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_BiInt.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_BiInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A pair of integers (a key of MAT2d_Tool2d's and MAT2d_Circuit's maps).</summary>
/// <remarks>A value type (OCCT uses it by value): the setters are internal, equality compares both integers
/// exactly, as OCCT's IsEqual and std::hash.</remarks>
public struct MAT2d_BiInt : IEquatable<MAT2d_BiInt>
{
    private int i1;
    private int i2;

    public MAT2d_BiInt(int I1, int I2)
    {
        i1 = I1;
        i2 = I2;
    }

    public readonly int FirstIndex() => i1;

    public readonly int SecondIndex() => i2;

    internal void FirstIndex(int I1) => i1 = I1;

    internal void SecondIndex(int I2) => i2 = I2;

    /// <summary>A copy with the first index I1.</summary>
    public readonly MAT2d_BiInt WithFirstIndex(int I1) => new(I1, i2);

    /// <summary>A copy with the second index I2.</summary>
    public readonly MAT2d_BiInt WithSecondIndex(int I2) => new(i1, I2);

    public readonly bool IsEqual(in MAT2d_BiInt B) => i1 == B.FirstIndex() && i2 == B.SecondIndex();

    public readonly bool Equals(MAT2d_BiInt other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is MAT2d_BiInt other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(i1, i2);

    public static bool operator ==(MAT2d_BiInt left, MAT2d_BiInt right) => left.Equals(right);

    public static bool operator !=(MAT2d_BiInt left, MAT2d_BiInt right) => !left.Equals(right);

    public override readonly string ToString() => "MAT2d_BiInt(" + i1 + ", " + i2 + ")";
}
