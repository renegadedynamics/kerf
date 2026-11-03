// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyHidingData.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using System.Diagnostics.CodeAnalysis;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>A hiding triangle of the polygonal hidden-line removal: its index, its min-max indices and its plane.</summary>
/// <remarks>
/// A C++ value class, so a struct (kept in arrays and changed in place). The nested structs are plain data with
/// public fields; <see cref="Set"/> and the reference accessors are internal.
/// </remarks>
public partial struct PolyHidingData : IEquatable<PolyHidingData>
{
    /// <summary>The triangle's index and min-max indices (plain data).</summary>
    public struct TriangleIndices : IEquatable<TriangleIndices>
    {
        public int Index;
        public int Min;
        public int Max;

        public readonly bool Equals(TriangleIndices other) => Index == other.Index && Min == other.Min && Max == other.Max;

        public override readonly bool Equals(object? obj) => obj is TriangleIndices other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(Index, Min, Max);
    }

    /// <summary>The triangle's plane: Normal . P + D = 0 (plain data).</summary>
    public struct PlaneT : IEquatable<PlaneT>
    {
        public Xyz Normal;
        public double D;

        public PlaneT()
        {
            Normal = new Xyz();
            D = 0.0;
        }

        public readonly bool Equals(PlaneT other) => Normal.Equals(other.Normal) && D.Equals(other.D);

        public override readonly bool Equals(object? obj) => obj is PlaneT other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(Normal, D);
    }

    private TriangleIndices _indices;
    private PlaneT _plane;

    public PolyHidingData()
    {
        _indices = new TriangleIndices();
        _plane = new PlaneT();
    }

    internal void Set(int index, int minim, int maxim, double a, double b, double c, double d)
    {
        _indices.Index = index;
        _indices.Min = minim;
        _indices.Max = maxim;
        _plane.Normal = new Xyz(a, b, c);
        _plane.D = d;
    }

    [UnscopedRef]
    internal ref TriangleIndices Indices() => ref _indices;

    [UnscopedRef]
    internal ref PlaneT Plane() => ref _plane;

    public readonly TriangleIndices IndicesValue() => _indices;

    public readonly PlaneT PlaneValue() => _plane;

    public readonly bool Equals(PolyHidingData other)
        => _indices.Equals(other._indices) && _plane.Equals(other._plane);

    public override readonly bool Equals(object? obj) => obj is PolyHidingData other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_indices, _plane);

    public static bool operator ==(in PolyHidingData left, in PolyHidingData right)
        => left.Equals(right);

    public static bool operator !=(in PolyHidingData left, in PolyHidingData right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"HLRAlgo_PolyHidingData({_indices.Index}, {_indices.Min}, {_indices.Max}, {_plane.Normal}, {Constants.Format("D", _plane.D)})";
}
