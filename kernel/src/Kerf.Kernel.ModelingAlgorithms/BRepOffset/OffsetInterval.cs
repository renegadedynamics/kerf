// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Interval.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Interval.lxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Interval.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Offsets;

/// <summary>A range of parameters on an edge, with the type of connection of its faces there.</summary>
/// <remarks>
/// A value type (OCCT's class is used by value). OCCT's default constructor leaves the fields uninitialised;
/// <c>default</c> here is (0, 0, <see cref="TypeOfConcavity.Concave"/>).
/// </remarks>
internal partial struct OffsetInterval : IEquatable<OffsetInterval>
{
    private double _f;
    private double _l;
    private TypeOfConcavity _type;

    /// <summary>The range [<paramref name="u1"/>, <paramref name="u2"/>] of type <paramref name="type"/>.</summary>
    public OffsetInterval(double u1, double u2, TypeOfConcavity type)
    {
        _f = u1;
        _l = u2;
        _type = type;
    }

    internal void First(double u) => _f = u;

    internal void Last(double u) => _l = u;

    internal void Type(TypeOfConcavity t) => _type = t;

    /// <summary>The first parameter.</summary>
    public readonly double First() => _f;

    /// <summary>The last parameter.</summary>
    public readonly double Last() => _l;

    /// <summary>The type of connection.</summary>
    public readonly TypeOfConcavity Type() => _type;

    /// <summary>A copy with the first parameter <paramref name="u"/>.</summary>
    public readonly OffsetInterval WithFirst(double u) => new(u, _l, _type);

    /// <summary>A copy with the last parameter <paramref name="u"/>.</summary>
    public readonly OffsetInterval WithLast(double u) => new(_f, u, _type);

    /// <summary>A copy with the type <paramref name="t"/>.</summary>
    public readonly OffsetInterval WithType(TypeOfConcavity t) => new(_f, _l, t);

    /// <inheritdoc/>
    public readonly bool Equals(OffsetInterval other) => _f.Equals(other._f) && _l.Equals(other._l) && _type == other._type;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is OffsetInterval other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(_f, _l, _type);

    public static bool operator ==(OffsetInterval left, OffsetInterval right) => left.Equals(right);

    public static bool operator !=(OffsetInterval left, OffsetInterval right) => !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString() => Constants.Format("BRepOffset_Interval", _f, _l)[..^1] + ", " + _type + ")";
}
