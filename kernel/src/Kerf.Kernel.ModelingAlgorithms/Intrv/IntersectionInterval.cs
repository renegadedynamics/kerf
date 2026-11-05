// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Interval.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Interval.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Interval.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Position;

namespace Kerf.Engine.Intersection;

/// <summary>
/// An interval of parameters with a (single precision) tolerance on each bound, and its position relative to
/// another one:
/// <code>
///                   **-----------****             Other
///     ***-----*                                   IsBefore
///     ***------------*                            IsJustBefore
///     ***-----------------*                       IsOverlappingAtStart
///     ***--------------------------*              IsJustEnclosingAtEnd
///     ***-------------------------------------*   IsEnclosing
///                  ***----*                       IsJustOverlappingAtStart
///                  ***-------------*              IsSimilar
///                  ***------------------------*   IsJustEnclosingAtStart
///                         ***-*                   IsInside
///                         ***------*              IsJustOverlappingAtEnd
///                         ***-----------------*   IsOverlappingAtEnd
///                                  ***--------*   IsJustAfter
///                                       ***---*   IsAfter
/// </code>
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new()</c> is OCCT's empty constructor (the whole real line); <c>default</c> is
/// [0, 0] with null tolerances, so ported code writes <c>new()</c>. The mutators are internal. The tolerances are
/// <c>float</c> as OCCT's (<c>Standard_ShortReal</c>), and the arithmetic keeps OCCT's float/double promotions.
/// </remarks>
internal partial struct IntersectionInterval : IEquatable<IntersectionInterval>
{
    private double _start;
    private double _end;
    private float _tolStart;
    private float _tolEnd;

    /// <summary>The whole real line.</summary>
    public IntersectionInterval()
    {
        _start = RealFirst();
        _end = RealLast();
        _tolStart = (float)Epsilon(RealFirst());
        _tolEnd = (float)Epsilon(RealLast());
    }

    /// <summary>[Start, End] with the bounds' epsilons as tolerances.</summary>
    public IntersectionInterval(double start, double end)
    {
        _start = start;
        _end = end;
        _tolStart = (float)Epsilon(_start);
        _tolEnd = (float)Epsilon(_end);
    }

    /// <summary>[Start, End] with the given tolerances (at least the bounds' epsilons).</summary>
    public IntersectionInterval(double start, float tolStart, double end, float tolEnd)
    {
        _start = start;
        _end = end;
        _tolStart = tolStart;
        _tolEnd = tolEnd;
        float epsStart = (float)Epsilon(_start);
        float epsEnd = (float)Epsilon(_end);
        if (_tolStart < epsStart)
        {
            _tolStart = epsStart;
        }
        if (_tolEnd < epsEnd)
        {
            _tolEnd = epsEnd;
        }
    }

    public readonly double Start() => _start;

    public readonly double End() => _end;

    public readonly float TolStart() => _tolStart;

    public readonly float TolEnd() => _tolEnd;

    public readonly void Bounds(out double start, out float tolStart, out double end, out float tolEnd)
    {
        start = _start;
        tolStart = _tolStart;
        end = _end;
        tolEnd = _tolEnd;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetStart(double start, float tolStart)
    {
        _start = start;
        _tolStart = tolStart;
    }

    /// <summary>Fuses the start with another one: the union of the two tolerance zones (none on an infinite start).</summary>
    internal void FuseAtStart(double start, float tolStart)
    {
        if (_start != RealFirst())
        {
            double a = Min(_start - _tolStart, start - tolStart);
            double b = Min(_start + _tolStart, start + tolStart);
            _start = (a + b) / 2;
            _tolStart = (float)(b - a) / 2;
        }
    }

    /// <summary>Cuts the start by the end of a tool.</summary>
    internal void CutAtStart(double start, float tolStart)
    {
        if (_start != RealFirst())
        {
            double a = Max(_start - _tolStart, start - tolStart);
            double b = Max(_start + _tolStart, start + tolStart);
            _start = (a + b) / 2;
            _tolStart = (float)(b - a) / 2;
        }
    }

    internal void SetEnd(double end, float tolEnd)
    {
        _end = end;
        _tolEnd = tolEnd;
    }

    /// <summary>Fuses the end with another one.</summary>
    internal void FuseAtEnd(double end, float tolEnd)
    {
        if (_end != RealLast())
        {
            double a = Max(_end - _tolEnd, end - tolEnd);
            double b = Max(_end + _tolEnd, end + tolEnd);
            _end = (a + b) / 2;
            _tolEnd = (float)(b - a) / 2;
        }
    }

    /// <summary>Cuts the end by the start of a tool.</summary>
    internal void CutAtEnd(double end, float tolEnd)
    {
        if (_end != RealLast())
        {
            double a = Min(_end - _tolEnd, end - tolEnd);
            double b = Min(_end + _tolEnd, end + tolEnd);
            _end = (a + b) / 2;
            _tolEnd = (float)(b - a) / 2;
        }
    }

    // OCCT's file-scope inline AreFused (Intrv_Interval.lxx)
    private static bool AreFused(double c1, float t1, double c2, float t2) => t1 + t2 >= Math.Abs(c1 - c2);

    /// <summary>True if the start and the end are fused within their tolerances.</summary>
    public readonly bool IsProbablyEmpty => AreFused(_start, _tolStart, _end, _tolEnd);

    /// <summary>The position of this interval relative to <paramref name="other"/>.</summary>
    public readonly Position Position(in IntersectionInterval other)
    {
        double mySMin = _start - _tolStart;
        double mySMax = _start + _tolStart;
        double myEMin = _end - _tolEnd;
        double myEMax = _end + _tolEnd;
        double otSMin = other._start - other._tolStart;
        double otSMax = other._start + other._tolStart;
        double otEMin = other._end - other._tolEnd;
        double otEMax = other._end + other._tolEnd;
        Position p;
        if (mySMax < otSMin)
        {
            if (myEMax < otSMin)
            {
                p = global::Kerf.Engine.Intersection.Position.Before;
            }
            else if (otSMax >= myEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.JustBefore;
            }
            else if (myEMax < otEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.OverlappingAtStart;
            }
            else if (otEMax >= myEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.JustEnclosingAtEnd;
            }
            else
            {
                p = global::Kerf.Engine.Intersection.Position.Enclosing;
            }
        }
        else if (otSMax >= mySMin)
        {
            if (myEMax < otEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.JustOverlappingAtStart;
            }
            else if (otEMax >= myEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.Similar;
            }
            else
            {
                p = global::Kerf.Engine.Intersection.Position.JustEnclosingAtStart;
            }
        }
        else if (mySMax < otEMin)
        {
            if (myEMax < otEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.Inside;
            }
            else if (otEMax >= myEMin)
            {
                p = global::Kerf.Engine.Intersection.Position.JustOverlappingAtEnd;
            }
            else
            {
                p = global::Kerf.Engine.Intersection.Position.OverlappingAtEnd;
            }
        }
        else if (otEMax >= mySMin)
        {
            p = global::Kerf.Engine.Intersection.Position.JustAfter;
        }
        else
        {
            p = global::Kerf.Engine.Intersection.Position.After;
        }
        return p;
    }

    public readonly bool IsBefore(in IntersectionInterval other) => _tolEnd + other._tolStart < other._start - _end;

    public readonly bool IsAfter(in IntersectionInterval other) => _tolStart + other._tolEnd < _start - other._end;

    public readonly bool IsInside(in IntersectionInterval other)
        => _tolStart + other._tolStart < _start - other._start && _tolEnd + other._tolEnd < other._end - _end;

    public readonly bool IsEnclosing(in IntersectionInterval other)
        => _tolStart + other._tolStart < other._start - _start && _tolEnd + other._tolEnd < _end - other._end;

    public readonly bool IsJustEnclosingAtStart(in IntersectionInterval other)
        => AreFused(_start, _tolStart, other._start, other._tolStart)
           && _tolEnd + other._tolEnd < _end - other._end;

    public readonly bool IsJustEnclosingAtEnd(in IntersectionInterval other)
        => _tolStart + other._tolStart < other._start - _start
           && AreFused(other._end, other._tolEnd, _end, _tolEnd);

    public readonly bool IsJustBefore(in IntersectionInterval other)
        => AreFused(_end, _tolEnd, other._start, other._tolStart);

    public readonly bool IsJustAfter(in IntersectionInterval other)
        => AreFused(other._end, other._tolEnd, _start, _tolStart);

    public readonly bool IsOverlappingAtStart(in IntersectionInterval other)
        => _tolStart + other._tolStart < other._start - _start
           && _tolEnd + other._tolStart < _end - other._start
           && _tolEnd + other._tolEnd < other._end - _end;

    public readonly bool IsOverlappingAtEnd(in IntersectionInterval other)
        => _tolStart + other._tolStart < _start - other._start
           && _tolStart + other._tolEnd < other._end - _start
           && _tolEnd + other._tolEnd < _end - other._end;

    public readonly bool IsJustOverlappingAtStart(in IntersectionInterval other)
        => AreFused(_start, _tolStart, other._start, other._tolStart)
           && _tolEnd + other._tolEnd < other._end - _end;

    public readonly bool IsJustOverlappingAtEnd(in IntersectionInterval other)
        => _tolStart + other._tolStart < _start - other._start
           && AreFused(other._end, other._tolEnd, _end, _tolEnd);

    public readonly bool IsSimilar(in IntersectionInterval other)
    {
        bool b1, b2;
        b1 = AreFused(_start, _tolStart, other._start, other._tolStart);
        b2 = AreFused(_end, _tolEnd, other._end, other._tolEnd);
        return b1 && b2;
    }

    public readonly IntersectionInterval WithStart(double start, float tolStart)
    {
        IntersectionInterval copy = this;
        copy.SetStart(start, tolStart);
        return copy;
    }

    public readonly IntersectionInterval WithEnd(double end, float tolEnd)
    {
        IntersectionInterval copy = this;
        copy.SetEnd(end, tolEnd);
        return copy;
    }

    public readonly bool Equals(IntersectionInterval other)
        => _start.Equals(other._start) && _end.Equals(other._end) && _tolStart.Equals(other._tolStart)
           && _tolEnd.Equals(other._tolEnd);

    public override readonly bool Equals(object? obj) => obj is IntersectionInterval other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_start, _end, _tolStart, _tolEnd);

    public static bool operator ==(in IntersectionInterval left, in IntersectionInterval right) => left.Equals(right);

    public static bool operator !=(in IntersectionInterval left, in IntersectionInterval right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("Intrv_Interval", _start, _tolStart, _end, _tolEnd);
}
