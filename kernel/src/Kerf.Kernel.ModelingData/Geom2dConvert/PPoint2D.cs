// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_PPoint.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_PPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>A point on a curve: its parameter, its 2D coordinates and its first derivative (the tangent).</summary>
/// <remarks>
/// The parameterless constructor gives OCCT's empty point (parameter RealLast(), point and derivative zero);
/// <c>default</c> has parameter 0. OCCT's <c>operator==</c> and <c>operator!=</c>, which compare the parameters
/// within Precision::PConfusion(), are <see cref="IsEqual"/>; <c>==</c> and <c>!=</c> compare exactly, as for every
/// value type of the port.
/// </remarks>
internal partial struct PPoint2D : IEquatable<PPoint2D>
{
    private double _parameter; // Parameter value
    private Point2D _point; // Point location
    private Vector2D _d1; // derivatives by parameter (components of the tangent).

    /// <summary>The empty point: parameter RealLast(), point and derivative zero.</summary>
    public PPoint2D()
    {
        _parameter = RealLast();
        _point = new Point2D(0.0, 0.0);
        _d1 = new Vector2D(0.0, 0.0);
    }

    /// <summary>The point of parameter theParameter at thePoint, with derivative theD1.</summary>
    public PPoint2D(double parameter, in Xy point, in Xy d1)
    {
        _parameter = parameter;
        _point = new Point2D(point);
        _d1 = new Vector2D(d1);
    }

    /// <summary>The point of theAdaptor at theParameter, with its first derivative.</summary>
    public PPoint2D(double parameter, Curve2DAdaptorBase adaptor)
    {
        _parameter = parameter;
        adaptor.D1(parameter, out _point, out _d1);
    }

    /// <summary>The distance between the two points.</summary>
    public readonly double Dist(in PPoint2D oth) => _point.DistanceTo(oth._point);

    /// <summary>The parameter.</summary>
    public readonly double Parameter() => _parameter;

    /// <summary>The point.</summary>
    public readonly Xy Point() => _point.XY();

    /// <summary>The first derivative.</summary>
    public readonly Xy D1() => _d1.XY();

    internal void SetD1(in Xy d1) => _d1 = _d1.WithXY(d1);

    /// <summary>A copy with the derivative theD1.</summary>
    public readonly PPoint2D WithD1(in Xy d1)
    {
        PPoint2D copy = this;
        copy.SetD1(d1);
        return copy;
    }

    /// <summary>
    /// OCCT's <c>operator==</c>: whether the parameters are within Precision::PConfusion(). (OCCT's
    /// <c>operator!=</c> is its negation.)
    /// </summary>
    public readonly bool IsEqual(in PPoint2D other) =>
        Math.Abs(_parameter - other.Parameter()) <= global::Kerf.Engine.Foundation.Precision.PConfusion();

    // OCCT's operator!=, as it is written (the exact negation of IsEqual but for NaN).
    internal readonly bool IsNotEqual(in PPoint2D other) =>
        Math.Abs(_parameter - other.Parameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion();

    public readonly bool Equals(PPoint2D other) =>
        _parameter.Equals(other._parameter) && _point.Equals(other._point) && _d1.Equals(other._d1);

    public override readonly bool Equals(object? obj) => obj is PPoint2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_parameter, _point, _d1);

    public static bool operator ==(in PPoint2D left, in PPoint2D right) => left.Equals(right);

    public static bool operator !=(in PPoint2D left, in PPoint2D right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"Geom2dConvert_PPoint({_parameter.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, {_point}, {_d1})";
}
