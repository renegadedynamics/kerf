// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Couple.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Couple.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A couple of integers.</summary>
/// <remarks><c>new()</c> and <c>default</c> are both (0, 0), as OCCT's default.</remarks>
public readonly partial struct IntSurf_Couple : IEquatable<IntSurf_Couple>
{
    private readonly int firstInteger;
    private readonly int secondInteger;

    public IntSurf_Couple()
    {
        firstInteger = 0;
        secondInteger = 0;
    }

    public IntSurf_Couple(int Index1, int Index2)
    {
        firstInteger = Index1;
        secondInteger = Index2;
    }

    /// <summary>The first element.</summary>
    public int First() => firstInteger;

    /// <summary>The second element.</summary>
    public int Second() => secondInteger;

    public bool Equals(IntSurf_Couple other) => firstInteger == other.firstInteger && secondInteger == other.secondInteger;

    public override bool Equals(object? obj) => obj is IntSurf_Couple other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(firstInteger, secondInteger);

    public static bool operator ==(IntSurf_Couple theLeft, IntSurf_Couple theRight) => theLeft.Equals(theRight);

    public static bool operator !=(IntSurf_Couple theLeft, IntSurf_Couple theRight) => !theLeft.Equals(theRight);

    public override string ToString() => $"IntSurf_Couple({firstInteger}, {secondInteger})";
}
