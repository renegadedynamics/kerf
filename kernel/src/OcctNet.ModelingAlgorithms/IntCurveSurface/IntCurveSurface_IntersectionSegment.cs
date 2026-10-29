// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionSegment.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A segment of a curve lying on a surface, given by its two end points.</summary>
/// <remarks>
/// A value type; <c>new()</c> and <c>default</c> are two default points. <c>SetValues</c> is an internal mutator;
/// the constructor is its twin. OCCT's <c>Dump</c> is <see cref="ToString"/>. (IntCurveSurface_HInter never makes
/// segments: its AppendSegment is empty.)
/// </remarks>
public partial struct IntCurveSurface_IntersectionSegment : IEquatable<IntCurveSurface_IntersectionSegment>
{
    private IntCurveSurface_IntersectionPoint myP1;
    private IntCurveSurface_IntersectionPoint myP2;

    public IntCurveSurface_IntersectionSegment()
    {
        myP1 = new IntCurveSurface_IntersectionPoint();
        myP2 = new IntCurveSurface_IntersectionPoint();
    }

    public IntCurveSurface_IntersectionSegment(in IntCurveSurface_IntersectionPoint P1,
                                               in IntCurveSurface_IntersectionPoint P2)
    {
        myP1 = P1;
        myP2 = P2;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(in IntCurveSurface_IntersectionPoint P1, in IntCurveSurface_IntersectionPoint P2)
    {
        myP1 = P1;
        myP2 = P2;
    }

    // Queries

    public readonly void Values(out IntCurveSurface_IntersectionPoint P1, out IntCurveSurface_IntersectionPoint P2)
    {
        P1 = myP1;
        P2 = myP2;
    }

    public readonly void FirstPoint(out IntCurveSurface_IntersectionPoint P1) => P1 = myP1;

    public readonly void SecondPoint(out IntCurveSurface_IntersectionPoint P2) => P2 = myP2;

    public readonly IntCurveSurface_IntersectionPoint FirstPoint() => myP1;

    public readonly IntCurveSurface_IntersectionPoint SecondPoint() => myP2;

    public readonly bool Equals(IntCurveSurface_IntersectionSegment other) => myP1.Equals(other.myP1) && myP2.Equals(other.myP2);

    public override readonly bool Equals(object? obj) => obj is IntCurveSurface_IntersectionSegment other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myP1, myP2);

    public static bool operator ==(in IntCurveSurface_IntersectionSegment theLeft,
                                   in IntCurveSurface_IntersectionSegment theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntCurveSurface_IntersectionSegment theLeft,
                                   in IntCurveSurface_IntersectionSegment theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"IntCurveSurface_IntersectionSegment({myP1}, {myP2})";
}
