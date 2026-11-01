// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Interval.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Interval.lxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Interval.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A range of parameters on an edge, with the type of connection of its faces there.</summary>
/// <remarks>
/// A value type (OCCT's class is used by value). OCCT's default constructor leaves the fields uninitialised;
/// <c>default</c> here is (0, 0, <see cref="ChFiDS_TypeOfConcavity.ChFiDS_Concave"/>).
/// </remarks>
public partial struct BRepOffset_Interval : IEquatable<BRepOffset_Interval>
{
    private double f;
    private double l;
    private ChFiDS_TypeOfConcavity type;

    /// <summary>The range [<paramref name="U1"/>, <paramref name="U2"/>] of type <paramref name="Type"/>.</summary>
    public BRepOffset_Interval(double U1, double U2, ChFiDS_TypeOfConcavity Type)
    {
        f = U1;
        l = U2;
        type = Type;
    }

    internal void First(double U) => f = U;

    internal void Last(double U) => l = U;

    internal void Type(ChFiDS_TypeOfConcavity T) => type = T;

    /// <summary>The first parameter.</summary>
    public readonly double First() => f;

    /// <summary>The last parameter.</summary>
    public readonly double Last() => l;

    /// <summary>The type of connection.</summary>
    public readonly ChFiDS_TypeOfConcavity Type() => type;

    /// <summary>A copy with the first parameter <paramref name="U"/>.</summary>
    public readonly BRepOffset_Interval WithFirst(double U) => new(U, l, type);

    /// <summary>A copy with the last parameter <paramref name="U"/>.</summary>
    public readonly BRepOffset_Interval WithLast(double U) => new(f, U, type);

    /// <summary>A copy with the type <paramref name="T"/>.</summary>
    public readonly BRepOffset_Interval WithType(ChFiDS_TypeOfConcavity T) => new(f, l, T);

    /// <inheritdoc/>
    public readonly bool Equals(BRepOffset_Interval other) => f.Equals(other.f) && l.Equals(other.l) && type == other.type;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is BRepOffset_Interval other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(f, l, type);

    public static bool operator ==(BRepOffset_Interval left, BRepOffset_Interval right) => left.Equals(right);

    public static bool operator !=(BRepOffset_Interval left, BRepOffset_Interval right) => !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString() => gp.Format("BRepOffset_Interval", f, l)[..^1] + ", " + type + ")";
}
