// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PointOnBis.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PointOnBis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// A point of a bisecting curve's polygon: the parameters of its projections on the two curves it separates, its
/// parameter on the bisector, its distance to the curves and the point.
/// </summary>
/// <remarks>
/// A value type (translation rules, section 3): the setters are internal; <c>default</c> is OCCT's default (zeros,
/// not infinite).
/// </remarks>
internal partial struct PointOnBis : IEquatable<PointOnBis>
{
    private double _param1;
    private double _param2;
    private double _paramBis;
    private double _distance;
    private bool _infinite;
    private Point2D _point;

    /// <summary>All zeros, not infinite.</summary>
    public PointOnBis()
    {
        _param1 = 0.0;
        _param2 = 0.0;
        _paramBis = 0.0;
        _distance = 0.0;
        _infinite = false;
        _point = new Point2D();
    }

    /// <summary>The point P of parameter ParamBis on the bisector, of parameters Param1 and Param2 on the curves.</summary>
    public PointOnBis(double param1, double param2, double paramBis, double distance, in Point2D p)
    {
        _param1 = param1;
        _param2 = param2;
        _paramBis = paramBis;
        _distance = distance;
        _point = p;
        _infinite = false;
    }

    internal void ParamOnC1(double param) => _param1 = param;

    internal void ParamOnC2(double param) => _param2 = param;

    internal void ParamOnBis(double param) => _paramBis = param;

    internal void Distance(double distance) => _distance = distance;

    internal void Point(in Point2D p) => _point = p;

    internal void IsInfinite(bool infinite) => _infinite = infinite;

    /// <summary>The parameter on the first curve.</summary>
    public readonly double ParamOnC1() => _param1;

    /// <summary>The parameter on the second curve.</summary>
    public readonly double ParamOnC2() => _param2;

    /// <summary>The parameter on the bisector.</summary>
    public readonly double ParamOnBis() => _paramBis;

    /// <summary>The distance to the curves.</summary>
    public readonly double Distance() => _distance;

    /// <summary>The point.</summary>
    public readonly Point2D Point() => _point;

    /// <summary>True if the point is at infinity.</summary>
    public readonly bool IsInfinite() => _infinite;

    /// <summary>A copy with the parameter on the first curve changed.</summary>
    public readonly PointOnBis WithParamOnC1(double param)
    {
        PointOnBis copy = this;
        copy.ParamOnC1(param);
        return copy;
    }

    /// <summary>A copy with the parameter on the second curve changed.</summary>
    public readonly PointOnBis WithParamOnC2(double param)
    {
        PointOnBis copy = this;
        copy.ParamOnC2(param);
        return copy;
    }

    /// <summary>A copy with the parameter on the bisector changed.</summary>
    public readonly PointOnBis WithParamOnBis(double param)
    {
        PointOnBis copy = this;
        copy.ParamOnBis(param);
        return copy;
    }

    /// <summary>A copy with the distance changed.</summary>
    public readonly PointOnBis WithDistance(double distance)
    {
        PointOnBis copy = this;
        copy.Distance(distance);
        return copy;
    }

    /// <summary>A copy with the point changed.</summary>
    public readonly PointOnBis WithPoint(in Point2D p)
    {
        PointOnBis copy = this;
        copy.Point(p);
        return copy;
    }

    /// <summary>A copy with the infinite flag changed.</summary>
    public readonly PointOnBis WithIsInfinite(bool infinite)
    {
        PointOnBis copy = this;
        copy.IsInfinite(infinite);
        return copy;
    }

    // OCCT's Dump (std::cout) is left out; ToString replaces it.

    /// <summary>Exact equality of every field.</summary>
    public readonly bool Equals(PointOnBis other) =>
        _param1.Equals(other._param1) && _param2.Equals(other._param2) && _paramBis.Equals(other._paramBis)
        && _distance.Equals(other._distance) && _infinite == other._infinite && _point.Equals(other._point);

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is PointOnBis other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(_param1, _param2, _paramBis, _distance, _infinite, _point);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(in PointOnBis left, in PointOnBis right) =>
        left.Equals(right);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(in PointOnBis left, in PointOnBis right) =>
        !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString() =>
        Constants.Format("Bisector_PointOnBis", _param1, _param2, _paramBis, _distance, _infinite ? 1.0 : 0.0, _point.X, _point.Y);
}
