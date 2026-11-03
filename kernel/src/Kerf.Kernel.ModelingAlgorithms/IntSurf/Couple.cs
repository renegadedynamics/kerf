// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Couple.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Couple.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>A couple of integers.</summary>
/// <remarks><c>new()</c> and <c>default</c> are both (0, 0), as OCCT's default.</remarks>
public readonly partial struct Couple : IEquatable<Couple>
{
    private readonly int _firstInteger;
    private readonly int _secondInteger;

    public Couple()
    {
        _firstInteger = 0;
        _secondInteger = 0;
    }

    public Couple(int index1, int index2)
    {
        _firstInteger = index1;
        _secondInteger = index2;
    }

    /// <summary>The first element.</summary>
    public int First() => _firstInteger;

    /// <summary>The second element.</summary>
    public int Second() => _secondInteger;

    public bool Equals(Couple other) => _firstInteger == other._firstInteger && _secondInteger == other._secondInteger;

    public override bool Equals(object? obj) => obj is Couple other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_firstInteger, _secondInteger);

    public static bool operator ==(Couple left, Couple right) => left.Equals(right);

    public static bool operator !=(Couple left, Couple right) => !left.Equals(right);

    public override string ToString() => $"IntSurf_Couple({_firstInteger}, {_secondInteger})";
}
