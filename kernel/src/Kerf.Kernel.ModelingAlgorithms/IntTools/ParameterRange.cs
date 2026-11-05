// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Range.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Range.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>The 1D range [First, Last].</summary>
internal partial struct ParameterRange : IEquatable<ParameterRange>
{
    private double _first;
    private double _last;

    /// <summary>The range [0, 0].</summary>
    public ParameterRange()
    {
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>The range [<paramref name="aFirst"/>, <paramref name="aLast"/>].</summary>
    public ParameterRange(double aFirst, double aLast)
    {
        _first = aFirst;
        _last = aLast;
    }

    internal void SetFirst(double aFirst) => _first = aFirst;

    internal void SetLast(double aLast) => _last = aLast;

    public readonly double First() => _first;

    public readonly double Last() => _last;

    public readonly void Range(out double aFirst, out double aLast)
    {
        aFirst = _first;
        aLast = _last;
    }

    // ---- Kerf additions

    public readonly ParameterRange WithFirst(double aFirst)
    {
        ParameterRange copy = this;
        copy.SetFirst(aFirst);
        return copy;
    }

    public readonly ParameterRange WithLast(double aLast)
    {
        ParameterRange copy = this;
        copy.SetLast(aLast);
        return copy;
    }

    public readonly bool Equals(ParameterRange other) => _first.Equals(other._first) && _last.Equals(other._last);

    public override readonly bool Equals(object? obj) => obj is ParameterRange other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_first, _last);

    public static bool operator ==(ParameterRange left, ParameterRange right) => left.Equals(right);

    public static bool operator !=(ParameterRange left, ParameterRange right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("IntTools_Range", _first, _last);
}
