// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionSegment.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionSegment.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>An intersection segment (a common part) between two 2D curves.</summary>
/// <remarks>
/// <c>new IntRes2d_IntersectionSegment()</c> is OCCT's default (an infinite segment, not opposite, with default
/// points); <c>default(IntRes2d_IntersectionSegment)</c> has zeroed points. The type has no mutators.
/// </remarks>
internal partial struct IntersectionSegment2D : IEquatable<IntersectionSegment2D>
{
    private bool _oppos;
    private bool _first;
    private bool _last;
    private IntersectionPoint2D _ptfirst;
    private IntersectionPoint2D _ptlast;

    /// <summary>Empty constructor.</summary>
    public IntersectionSegment2D()
    {
        _oppos = false;
        _first = false;
        _last = false;
        _ptfirst = new IntersectionPoint2D();
        _ptlast = new IntersectionPoint2D();
    }

    /// <summary>
    /// A segment bounded by two points. With <paramref name="reverseFlag"/> and <paramref name="oppos"/> both
    /// true, the points are swapped.
    /// </summary>
    public IntersectionSegment2D(in IntersectionPoint2D p1, in IntersectionPoint2D p2,
                                        bool oppos, bool reverseFlag)
    {
        _oppos = oppos;
        _first = true;
        _last = true;
        _ptfirst = p1;
        _ptlast = p2;
        if (reverseFlag)
        {
            if (oppos)
            {
                _ptfirst = p2;
                _ptlast = p1;
            }
        }
    }

    /// <summary>
    /// A semi-infinite segment: <paramref name="p"/> is its first point if <paramref name="first"/> is true,
    /// its last point otherwise (the other way round with <paramref name="reverseFlag"/> and
    /// <paramref name="oppos"/> both true).
    /// </summary>
    public IntersectionSegment2D(in IntersectionPoint2D p, bool first, bool oppos, bool reverseFlag)
    {
        _oppos = oppos;
        _ptfirst = new IntersectionPoint2D();
        _ptlast = new IntersectionPoint2D();
        if (reverseFlag && oppos)
        {
            if (first)
            {
                _first = false;
                _last = true;
                _ptlast = p;
            }
            else
            {
                _first = true;
                _last = false;
                _ptfirst = p;
            }
        }
        else
        {
            if (first)
            {
                _first = true;
                _last = false;
                _ptfirst = p;
            }
            else
            {
                _first = false;
                _last = true;
                _ptlast = p;
            }
        }
    }

    /// <summary>Creates an infinite segment of intersection.</summary>
    public IntersectionSegment2D(bool oppos)
    {
        _oppos = oppos;
        _first = false;
        _last = false;
        _ptfirst = new IntersectionPoint2D();
        _ptlast = new IntersectionPoint2D();
    }

    /// <summary>False if the intersection segment has the same orientation on both curves.</summary>
    public readonly bool IsOpposite => _oppos;

    /// <summary>
    /// True if the segment is limited by a first point: the lowest parameter on the first curve, and the lowest
    /// (resp. highest) parameter on the second curve if <see cref="IsOpposite"/> is false (resp. true).
    /// </summary>
    public readonly bool HasFirstPoint => _first;

    /// <summary>The first point of the segment.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly IntersectionPoint2D FirstPoint()
    {
        if (!_first)
        {
            throw new DomainErrorException();
        }

        return _ptfirst;
    }

    /// <summary>
    /// True if the segment is limited by a last point: the highest parameter on the first curve, and the highest
    /// (resp. lowest) parameter on the second curve if <see cref="IsOpposite"/> is false (resp. true).
    /// </summary>
    public readonly bool HasLastPoint => _last;

    /// <summary>The last point of the segment.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly IntersectionPoint2D LastPoint()
    {
        if (!_last)
        {
            throw new DomainErrorException();
        }

        return _ptlast;
    }

    public readonly bool Equals(IntersectionSegment2D other)
        => _oppos == other._oppos && _first == other._first && _last == other._last && _ptfirst.Equals(other._ptfirst)
           && _ptlast.Equals(other._ptlast);

    public override readonly bool Equals(object? obj) => obj is IntersectionSegment2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_oppos, _first, _last, _ptfirst, _ptlast);

    public static bool operator ==(in IntersectionSegment2D left, in IntersectionSegment2D right)
        => left.Equals(right);

    public static bool operator !=(in IntersectionSegment2D left, in IntersectionSegment2D right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"IntRes2d_IntersectionSegment(opposite: {_oppos}, first: {(_first ? _ptfirst.ToString() : "none")}, last: {(_last ? _ptlast.ToString() : "none")})";
}
