// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>
/// An ellipse in 3D space, defined by its major and minor radii and positioned by a right-handed coordinate
/// system: the origin is the centre, the X direction the major axis and the main direction the normal to its
/// plane. It is parameterised by <c>P(U) = O + MajorRad * cos(U) * XDir + MinorRad * sin(U) * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new Ellipse3D()</c> is OCCT's indefinite ellipse (major radius <c>double.MaxValue</c>, minor radius
/// <c>2.2e-308</c>); <c>default(Ellipse3D)</c> is not valid.
/// </remarks>
public readonly partial struct Ellipse3D : IEquatable<Ellipse3D>
{
    private readonly Frame3D _pos;
    private readonly double _majorRadius;
    private readonly double _minorRadius;

    /// <summary>An indefinite ellipse, major radius <c>double.MaxValue</c> and minor radius next to zero, not a useful value until given real ones.</summary>
    public Ellipse3D()
    {
        _pos = new Frame3D();
        _majorRadius = RealLast();
        _minorRadius = RealSmall();
    }

    /// <exception cref="ArgumentException">
    /// <paramref name="minorRadius"/> is negative or greater than <paramref name="majorRadius"/>.
    /// </exception>
    public Ellipse3D(in Frame3D frame, double majorRadius, double minorRadius)
    {
        _pos = frame;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Refusal.ArgumentIf(minorRadius < 0.0 || majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips() - invalid construction parameters");
    }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Ellipse3D WithAxis(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(axis);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy moved so that its centre is <paramref name="point"/>; its directions stay.</summary>
    public readonly Ellipse3D WithLocation(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.WithLocation(point);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy with this major radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="majorRadius"/> is less than the minor radius.</exception>
    public readonly Ellipse3D WithMajorRadius(double majorRadius)
    {
        double newMajorRadius = _majorRadius;
        Refusal.ArgumentIf(
            majorRadius < _minorRadius, FailureKind.ConstructionError, "gp_Elips::SetMajorRadius() - major radius should be greater or equal to minor radius");
        newMajorRadius = majorRadius;
        return new Ellipse3D(_pos, newMajorRadius, _minorRadius);
    }

    /// <summary>A copy with this minor radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="minorRadius"/> is negative, or greater than the major radius.</exception>
    public readonly Ellipse3D WithMinorRadius(double minorRadius)
    {
        double newMinorRadius = _minorRadius;
        Refusal.ArgumentIf(minorRadius < 0.0 || _majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
        newMinorRadius = minorRadius;
        return new Ellipse3D(_pos, _majorRadius, newMinorRadius);
    }

    /// <summary>A copy positioned by <paramref name="frame"/>, keeping its size.</summary>
    public readonly Ellipse3D WithPosition(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = frame;
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    // Queries

    /// <summary>The area enclosed.</summary>
    public readonly double Area => M_PI * _majorRadius * _minorRadius;

    /// <summary>The main axis: the normal to the ellipse's plane through its centre.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="InvalidOperationException">The eccentricity is zero (a circle).</exception>
    public readonly Axis3D Directrix1
    {
        get
        {
            double e = Eccentricity;
            Refusal.StateIf(e <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Elips::Directrix1() - zero eccentricity");
            Xyz orig = _pos.XDirection.XYZ();
            orig.Multiply(_majorRadius / e);
            orig.Add(_pos.Location.XYZ());
            return new Axis3D(new Point3D(orig), _pos.YDirection);
        }
    }

    /// <summary>The second directrix, symmetric to the first about the minor axis.</summary>
    /// <exception cref="InvalidOperationException">The eccentricity is zero (a circle).</exception>
    public readonly Axis3D Directrix2
    {
        get
        {
            double e = Eccentricity;
            Refusal.StateIf(e <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Elips::Directrix2() - zero eccentricity");
            Xyz orig = _pos.XDirection.XYZ();
            orig.Multiply(-_majorRadius / e);
            orig.Add(_pos.Location.XYZ());
            return new Axis3D(new Point3D(orig), _pos.YDirection);
        }
    }

    /// <summary>sqrt(Major^2 - Minor^2) / Major, in [0, 1); 0 when the major radius is 0.</summary>
    public readonly double Eccentricity
    {
        get
        {
            if (_majorRadius == 0.0)
            {
                return 0.0;
            }
            else
            {
                return Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius) / _majorRadius;
            }
        }
    }

    /// <summary>The distance between the two foci.</summary>
    public readonly double Focal => 2.0 * Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public readonly Point3D Focus1
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            Point3D pp = _pos.Location;
            Direction3D dd = _pos.XDirection;
            return new Point3D(pp.X + c * dd.X, pp.Y + c * dd.Y, pp.Z + c * dd.Z);
        }
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public readonly Point3D Focus2
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            Point3D pp = _pos.Location;
            Direction3D dd = _pos.XDirection;
            return new Point3D(pp.X - c * dd.X, pp.Y - c * dd.Y, pp.Z - c * dd.Z);
        }
    }

    /// <summary>The centre of the ellipse.</summary>
    public readonly Point3D Location => _pos.Location;

    /// <summary>The longer semi-axis.</summary>
    public readonly double MajorRadius => _majorRadius;

    /// <summary>The shorter semi-axis.</summary>
    public readonly double MinorRadius => _minorRadius;

    /// <summary>The focal parameter, Minor^2 / Major; 0 when the major radius is 0.</summary>
    public readonly double Parameter
    {
        get
        {
            if (_majorRadius == 0.0)
            {
                return 0.0;
            }
            else
            {
                return (_minorRadius * _minorRadius) / _majorRadius;
            }
        }
    }

    /// <summary>The frame the curve lies in: its main direction is the plane's normal, and its X direction is parameter 0's.</summary>
    public readonly Frame3D Position => _pos;

    /// <summary>The major axis.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The minor axis.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Ellipse3D Mirrored(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(point);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Ellipse3D Mirrored(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Ellipse3D Mirrored(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(frame);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Ellipse3D Rotated(in Axis3D axis, double angle)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: both radii are multiplied by |factor|.</summary>
    public readonly Ellipse3D Scaled(in Point3D point, double factor)
    {
        Frame3D pos = _pos;
        double majorRadius = _majorRadius;
        double minorRadius = _minorRadius;
        majorRadius *= factor;
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= factor;
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        pos = pos.Scaled(point, factor);
        return new Ellipse3D(pos, majorRadius, minorRadius);
    }

    /// <summary>The image under <paramref name="transform"/>: both radii are multiplied by |ScaleFactor|.</summary>
    public readonly Ellipse3D Transformed(in Transform3D transform)
    {
        Frame3D pos = _pos;
        double majorRadius = _majorRadius;
        double minorRadius = _minorRadius;
        majorRadius *= transform.ScaleFactor;
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= transform.ScaleFactor;
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        pos = pos.Transformed(transform);
        return new Ellipse3D(pos, majorRadius, minorRadius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Ellipse3D Translated(in Vector3D vector)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(vector);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Ellipse3D Translated(in Point3D point1, in Point3D point2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Ellipse3D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Ellipse3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Ellipse3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Ellipse3D left, in Ellipse3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Ellipse3D left, in Ellipse3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => FormattableString.Invariant($"Ellipse3D({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
