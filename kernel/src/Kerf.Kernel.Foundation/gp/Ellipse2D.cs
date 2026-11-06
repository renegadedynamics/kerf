// Port of: src/FoundationClasses/TKMath/gp/gp_Elips2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>
/// An ellipse in 2D space, positioned by a coordinate system whose origin is the centre and whose X axis is
/// the major axis. It is parametrized from the X axis, counterclockwise when the system is direct.
/// </summary>
/// <remarks>
/// <c>new gp_Elips2d()</c> is OCCT's indefinite ellipse (major radius <see cref="Standard.RealLast"/>, minor
/// radius <see cref="Standard.RealSmall"/>); <c>default(gp_Elips2d)</c> is not valid.
/// </remarks>
public readonly partial struct Ellipse2D : IEquatable<Ellipse2D>
{
    private readonly Frame2D _pos;
    private readonly double _majorRadius;
    private readonly double _minorRadius;

    /// <summary>An indefinite ellipse, major radius <c>double.MaxValue</c> and minor radius next to zero, not a useful value until given real ones.</summary>
    public Ellipse2D()
    {
        _pos = new Frame2D();
        _majorRadius = RealLast();
        _minorRadius = RealSmall();
    }

    /// <summary>An ellipse whose major axis is <paramref name="majorAxis"/>, direct when <paramref name="sense"/> is true.</summary>
    /// <exception cref="ConstructionErrorException">The minor radius is negative or greater than the major radius.</exception>
    public Ellipse2D(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense = true)
    {
        _pos = new Frame2D(majorAxis, sense);
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Refusal.ArgumentIf(minorRadius < 0.0 || majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips2d() - invalid construction parameters");
    }

    /// <exception cref="ConstructionErrorException">The minor radius is negative or greater than the major radius.</exception>
    public Ellipse2D(in Frame2D frame, double majorRadius, double minorRadius)
    {
        _pos = frame;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Refusal.ArgumentIf(minorRadius < 0.0 || majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips2d() - invalid construction parameters");
    }

    // Copies

    /// <summary>A copy moved so that its centre is <paramref name="point"/>; its directions stay.</summary>
    public readonly Ellipse2D WithLocation(in Point2D point)
    {
        Frame2D pos = _pos;
        pos = pos.WithLocation(point);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy with this major radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="majorRadius"/> is less than the minor radius.</exception>
    public readonly Ellipse2D WithMajorRadius(double majorRadius)
    {
        double newMajorRadius = _majorRadius;
        Refusal.ArgumentIf(majorRadius < _minorRadius, FailureKind.ConstructionError, "gp_Elips2d::SetMajorRadius() - major radius should be greater or equal to minor radius");
        newMajorRadius = majorRadius;
        return new Ellipse2D(_pos, newMajorRadius, _minorRadius);
    }

    /// <summary>A copy with this minor radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="minorRadius"/> is negative, or greater than the major radius.</exception>
    public readonly Ellipse2D WithMinorRadius(double minorRadius)
    {
        double newMinorRadius = _minorRadius;
        Refusal.ArgumentIf(minorRadius < 0.0 || _majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips2d::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
        newMinorRadius = minorRadius;
        return new Ellipse2D(_pos, _majorRadius, newMinorRadius);
    }

    /// <summary>A copy positioned by <paramref name="frame"/>, which replaces the whole frame.</summary>
    public readonly Ellipse2D WithAxis(in Frame2D frame)
    {
        Frame2D pos = _pos;
        pos = pos.WithAxis(frame);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy whose location and X direction come from <paramref name="axis"/>; the Y direction is recomputed normal to it, keeping the handedness.</summary>
    public readonly Ellipse2D WithXAxis(in Axis2D axis)
    {
        Frame2D pos = _pos;
        pos = pos.WithXAxis(axis);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy whose location and Y direction come from <paramref name="axis"/>; the X direction is recomputed normal to it, keeping the handedness.</summary>
    public readonly Ellipse2D WithYAxis(in Axis2D axis)
    {
        Frame2D pos = _pos;
        pos = pos.WithYAxis(axis);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    // Queries

    /// <summary>The area enclosed.</summary>
    public readonly double Area => M_PI * _majorRadius * _minorRadius;

    /// <summary>
    /// The coefficients of the ellipse's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d, out double e, out double f)
    {
        double dMin = _minorRadius * _minorRadius;
        double dMaj = _majorRadius * _majorRadius;
        if (dMin <= Constants.Resolution() && dMaj <= Constants.Resolution())
        {
            a = b = c = d = e = f = 0.0;
        }
        else
        {
            Transform2D t = new Transform2D().WithTransformation(_pos.XAxis);
            double t11 = t.Value(1, 1);
            double t12 = t.Value(1, 2);
            double t13 = t.Value(1, 3);
            if (dMin <= Constants.Resolution())
            {
                a = t11 * t11;
                b = t12 * t12;
                c = t11 * t12;
                d = t11 * t13;
                e = t12 * t13;
                f = t13 * t13 - dMaj;
            }
            else
            {
                double t21 = t.Value(2, 1);
                double t22 = t.Value(2, 2);
                double t23 = t.Value(2, 3);
                a = (t11 * t11 / dMaj) + (t21 * t21 / dMin);
                b = (t12 * t12 / dMaj) + (t22 * t22 / dMin);
                c = (t11 * t12 / dMaj) + (t21 * t22 / dMin);
                d = (t11 * t13 / dMaj) + (t21 * t23 / dMin);
                e = (t12 * t13 / dMaj) + (t22 * t23 / dMin);
                f = (t13 * t13 / dMaj) + (t23 * t23 / dMin) - 1.0;
            }
        }
    }

    /// <summary>The directrix on the positive side of the major axis, at distance MajorRadius / e from the centre.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
    public readonly Axis2D Directrix1
    {
        get
        {
            double e = Eccentricity;
            Refusal.StateIf(e <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Elips2d::Directrix1() - zero eccentricity");
            Xy orig = _pos.XDirection.XY();
            orig.Multiply(_majorRadius / e);
            orig.Add(_pos.Location.XY());
            return new Axis2D(new Point2D(orig), _pos.YDirection);
        }
    }

    /// <summary>The directrix on the negative side of the major axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
    public readonly Axis2D Directrix2
    {
        get
        {
            double e = Eccentricity;
            Refusal.StateIf(e <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Elips2d::Directrix2() - zero eccentricity");
            Xy orig = _pos.XDirection.XY();
            orig.Multiply(-_majorRadius / e);
            orig.Add(_pos.Location.XY());
            return new Axis2D(new Point2D(orig), _pos.YDirection);
        }
    }

    /// <summary>The eccentricity, in [0, 1): 0 for a circle, and also when the major radius is 0.</summary>
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

    /// <summary>The distance between the foci.</summary>
    public readonly double Focal => 2.0 * Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the X axis.</summary>
    public readonly Point2D Focus1
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            Point2D pp = _pos.Location;
            Direction2D dd = _pos.XDirection;
            return new Point2D(pp.X + c * dd.X, pp.Y + c * dd.Y);
        }
    }

    /// <summary>The focus on the negative side of the X axis.</summary>
    public readonly Point2D Focus2
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            Point2D pp = _pos.Location;
            Direction2D dd = _pos.XDirection;
            return new Point2D(pp.X - c * dd.X, pp.Y - c * dd.Y);
        }
    }

    /// <summary>The centre.</summary>
    public readonly Point2D Location => _pos.Location;

    /// <summary>The longer semi-axis.</summary>
    public readonly double MajorRadius => _majorRadius;

    /// <summary>The shorter semi-axis.</summary>
    public readonly double MinorRadius => _minorRadius;

    /// <summary>The parameter (semi-latus rectum) of the ellipse: MinorRadius^2 / MajorRadius.</summary>
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

    /// <summary>The frame the curve is positioned by, whose X direction is parameter 0's and whose handedness is the sense it runs in.</summary>
    public readonly Frame2D Axis => _pos;

    /// <summary>The major axis, through the centre.</summary>
    public readonly Axis2D XAxis => _pos.XAxis;

    /// <summary>The minor axis, through the centre.</summary>
    public readonly Axis2D YAxis => _pos.YAxis;

    /// <summary>True when the ellipse's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection.Crossed(_pos.YDirection)) >= 0.0;

    /// <summary>The same curve, running the other way: the Y direction reverses, which flips the frame's handedness.</summary>
    public readonly Ellipse2D Reversed()
    {
        Frame2D pos = _pos;
        Direction2D temp = _pos.YDirection;
        temp = temp.Reversed();
        pos = pos.WithAxis(new Frame2D(_pos.Location, _pos.XDirection, temp));
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Ellipse2D Mirrored(in Point2D point)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(point);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The reflection in the line <paramref name="axis"/>.</summary>
    public readonly Ellipse2D Mirrored(in Axis2D axis)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Rotated about <paramref name="point"/> by <paramref name="angle"/> radians, anticlockwise.</summary>
    public readonly Ellipse2D Rotated(in Point2D point, double angle)
    {
        Frame2D pos = _pos;
        pos = pos.Rotated(point, angle);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: both radii are multiplied by |factor|.</summary>
    public readonly Ellipse2D Scaled(in Point2D point, double factor)
    {
        Frame2D pos = _pos;
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
        return new Ellipse2D(pos, majorRadius, minorRadius);
    }

    /// <summary>The image under <paramref name="transform"/>: both radii are multiplied by |ScaleFactor|.</summary>
    public readonly Ellipse2D Transformed(in Transform2D transform)
    {
        Frame2D pos = _pos;
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
        return new Ellipse2D(pos, majorRadius, minorRadius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Ellipse2D Translated(in Vector2D vector)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(vector);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Ellipse2D Translated(in Point2D point1, in Point2D point2)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Ellipse2D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Ellipse2D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Ellipse2D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Ellipse2D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Ellipse2D left, in Ellipse2D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Ellipse2D left, in Ellipse2D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Ellipse2D({_pos}, major {_majorRadius}, minor {_minorRadius})";
}
