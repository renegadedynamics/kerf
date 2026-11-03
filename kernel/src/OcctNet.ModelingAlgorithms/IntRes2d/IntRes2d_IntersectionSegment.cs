// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionSegment.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionSegment.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>An intersection segment (a common part) between two 2D curves.</summary>
/// <remarks>
/// <c>new IntRes2d_IntersectionSegment()</c> is OCCT's default (an infinite segment, not opposite, with default
/// points); <c>default(IntRes2d_IntersectionSegment)</c> has zeroed points. The type has no mutators.
/// </remarks>
public partial struct IntRes2d_IntersectionSegment : IEquatable<IntRes2d_IntersectionSegment>
{
    private bool oppos;
    private bool first;
    private bool last;
    private IntRes2d_IntersectionPoint ptfirst;
    private IntRes2d_IntersectionPoint ptlast;

    /// <summary>Empty constructor.</summary>
    public IntRes2d_IntersectionSegment()
    {
        oppos = false;
        first = false;
        last = false;
        ptfirst = new IntRes2d_IntersectionPoint();
        ptlast = new IntRes2d_IntersectionPoint();
    }

    /// <summary>
    /// A segment bounded by two points. With <paramref name="ReverseFlag"/> and <paramref name="Oppos"/> both
    /// true, the points are swapped.
    /// </summary>
    public IntRes2d_IntersectionSegment(in IntRes2d_IntersectionPoint P1, in IntRes2d_IntersectionPoint P2,
                                        bool Oppos, bool ReverseFlag)
    {
        oppos = Oppos;
        first = true;
        last = true;
        ptfirst = P1;
        ptlast = P2;
        if (ReverseFlag)
        {
            if (Oppos)
            {
                ptfirst = P2;
                ptlast = P1;
            }
        }
    }

    /// <summary>
    /// A semi-infinite segment: <paramref name="P"/> is its first point if <paramref name="First"/> is true,
    /// its last point otherwise (the other way round with <paramref name="ReverseFlag"/> and
    /// <paramref name="Oppos"/> both true).
    /// </summary>
    public IntRes2d_IntersectionSegment(in IntRes2d_IntersectionPoint P, bool First, bool Oppos, bool ReverseFlag)
    {
        oppos = Oppos;
        ptfirst = new IntRes2d_IntersectionPoint();
        ptlast = new IntRes2d_IntersectionPoint();
        if (ReverseFlag && Oppos)
        {
            if (First)
            {
                first = false;
                last = true;
                ptlast = P;
            }
            else
            {
                first = true;
                last = false;
                ptfirst = P;
            }
        }
        else
        {
            if (First)
            {
                first = true;
                last = false;
                ptfirst = P;
            }
            else
            {
                first = false;
                last = true;
                ptlast = P;
            }
        }
    }

    /// <summary>Creates an infinite segment of intersection.</summary>
    public IntRes2d_IntersectionSegment(bool Oppos)
    {
        oppos = Oppos;
        first = false;
        last = false;
        ptfirst = new IntRes2d_IntersectionPoint();
        ptlast = new IntRes2d_IntersectionPoint();
    }

    /// <summary>False if the intersection segment has the same orientation on both curves.</summary>
    public readonly bool IsOpposite() => oppos;

    /// <summary>
    /// True if the segment is limited by a first point: the lowest parameter on the first curve, and the lowest
    /// (resp. highest) parameter on the second curve if <see cref="IsOpposite"/> is false (resp. true).
    /// </summary>
    public readonly bool HasFirstPoint() => first;

    /// <summary>The first point of the segment.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly IntRes2d_IntersectionPoint FirstPoint()
    {
        if (!first)
        {
            throw new DomainErrorException();
        }

        return ptfirst;
    }

    /// <summary>
    /// True if the segment is limited by a last point: the highest parameter on the first curve, and the highest
    /// (resp. lowest) parameter on the second curve if <see cref="IsOpposite"/> is false (resp. true).
    /// </summary>
    public readonly bool HasLastPoint() => last;

    /// <summary>The last point of the segment.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly IntRes2d_IntersectionPoint LastPoint()
    {
        if (!last)
        {
            throw new DomainErrorException();
        }

        return ptlast;
    }

    public readonly bool Equals(IntRes2d_IntersectionSegment other)
        => oppos == other.oppos && first == other.first && last == other.last && ptfirst.Equals(other.ptfirst)
           && ptlast.Equals(other.ptlast);

    public override readonly bool Equals(object? obj) => obj is IntRes2d_IntersectionSegment other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(oppos, first, last, ptfirst, ptlast);

    public static bool operator ==(in IntRes2d_IntersectionSegment theLeft, in IntRes2d_IntersectionSegment theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in IntRes2d_IntersectionSegment theLeft, in IntRes2d_IntersectionSegment theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntRes2d_IntersectionSegment(opposite: {oppos}, first: {(first ? ptfirst.ToString() : "none")}, last: {(last ? ptlast.ToString() : "none")})";
}
