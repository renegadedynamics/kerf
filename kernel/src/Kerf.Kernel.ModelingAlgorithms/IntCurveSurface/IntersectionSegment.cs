// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionSegment.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>A segment of a curve lying on a surface, given by its two end points.</summary>
/// <remarks>
/// A value type; <c>new()</c> and <c>default</c> are two default points. <c>SetValues</c> is an internal mutator;
/// the constructor is its twin. OCCT's <c>Dump</c> is <see cref="ToString"/>. (IntCurveSurface_HInter never makes
/// segments: its AppendSegment is empty.)
/// </remarks>
internal partial struct IntersectionSegment : IEquatable<IntersectionSegment>
{
    private CurveSurfaceIntersectionPoint _p1;
    private CurveSurfaceIntersectionPoint _p2;

    public IntersectionSegment()
    {
        _p1 = new CurveSurfaceIntersectionPoint();
        _p2 = new CurveSurfaceIntersectionPoint();
    }

    public IntersectionSegment(in CurveSurfaceIntersectionPoint p1,
                                               in CurveSurfaceIntersectionPoint p2)
    {
        _p1 = p1;
        _p2 = p2;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(in CurveSurfaceIntersectionPoint p1, in CurveSurfaceIntersectionPoint p2)
    {
        _p1 = p1;
        _p2 = p2;
    }

    // Queries

    public readonly void Values(out CurveSurfaceIntersectionPoint p1, out CurveSurfaceIntersectionPoint p2)
    {
        p1 = _p1;
        p2 = _p2;
    }

    public readonly void FirstPoint(out CurveSurfaceIntersectionPoint p1) => p1 = _p1;

    public readonly void SecondPoint(out CurveSurfaceIntersectionPoint p2) => p2 = _p2;

    public readonly CurveSurfaceIntersectionPoint FirstPoint() => _p1;

    public readonly CurveSurfaceIntersectionPoint SecondPoint() => _p2;

    public readonly bool Equals(IntersectionSegment other) => _p1.Equals(other._p1) && _p2.Equals(other._p2);

    public override readonly bool Equals(object? obj) => obj is IntersectionSegment other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_p1, _p2);

    public static bool operator ==(in IntersectionSegment left,
                                   in IntersectionSegment right) => left.Equals(right);

    public static bool operator !=(in IntersectionSegment left,
                                   in IntersectionSegment right) => !left.Equals(right);

    public override readonly string ToString() => $"IntCurveSurface_IntersectionSegment({_p1}, {_p2})";
}
