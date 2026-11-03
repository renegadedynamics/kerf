// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyHidingData.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using System.Diagnostics.CodeAnalysis;

namespace Occt;

/// <summary>A hiding triangle of the polygonal hidden-line removal: its index, its min-max indices and its plane.</summary>
/// <remarks>
/// A C++ value class, so a struct (kept in arrays and changed in place). The nested structs are plain data with
/// public fields; <see cref="Set"/> and the reference accessors are internal.
/// </remarks>
public partial struct HLRAlgo_PolyHidingData : IEquatable<HLRAlgo_PolyHidingData>
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

    private TriangleIndices myIndices;
    private PlaneT myPlane;

    public HLRAlgo_PolyHidingData()
    {
        myIndices = new TriangleIndices();
        myPlane = new PlaneT();
    }

    internal void Set(int Index, int Minim, int Maxim, double A, double B, double C, double D)
    {
        myIndices.Index = Index;
        myIndices.Min = Minim;
        myIndices.Max = Maxim;
        myPlane.Normal = new Xyz(A, B, C);
        myPlane.D = D;
    }

    [UnscopedRef]
    internal ref TriangleIndices Indices() => ref myIndices;

    [UnscopedRef]
    internal ref PlaneT Plane() => ref myPlane;

    public readonly TriangleIndices IndicesValue() => myIndices;

    public readonly PlaneT PlaneValue() => myPlane;

    public readonly bool Equals(HLRAlgo_PolyHidingData other)
        => myIndices.Equals(other.myIndices) && myPlane.Equals(other.myPlane);

    public override readonly bool Equals(object? obj) => obj is HLRAlgo_PolyHidingData other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myIndices, myPlane);

    public static bool operator ==(in HLRAlgo_PolyHidingData theLeft, in HLRAlgo_PolyHidingData theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in HLRAlgo_PolyHidingData theLeft, in HLRAlgo_PolyHidingData theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"HLRAlgo_PolyHidingData({myIndices.Index}, {myIndices.Min}, {myIndices.Max}, {myPlane.Normal}, {Constants.Format("D", myPlane.D)})";
}
