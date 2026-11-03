// Port of: src/FoundationClasses/TKMath/gp/gp_Elips2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An ellipse in 2D space, positioned by a coordinate system whose origin is the centre and whose X axis is
/// the major axis. It is parametrized from the X axis, counterclockwise when the system is direct.
/// </summary>
/// <remarks>
/// <c>new gp_Elips2d()</c> is OCCT's indefinite ellipse (major radius <see cref="Standard.RealLast"/>, minor
/// radius <see cref="Standard.RealSmall"/>); <c>default(gp_Elips2d)</c> is not valid.
/// </remarks>
public partial struct Ellipse2D : IEquatable<Ellipse2D>
{
    private Frame2D _pos;
    private double _majorRadius;
    private double _minorRadius;

    public Ellipse2D()
    {
        _pos = new Frame2D();
        _majorRadius = RealLast();
        _minorRadius = RealSmall();
    }

    /// <summary>An ellipse whose major axis is <paramref name="majorAxis"/>, direct when <paramref name="isSense"/> is true.</summary>
    /// <exception cref="ConstructionErrorException">The minor radius is negative or greater than the major radius.</exception>
    public Ellipse2D(in Axis2D majorAxis, double majorRadius, double minorRadius, bool isSense = true)
    {
        _pos = new Frame2D(majorAxis, isSense);
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < minorRadius, "gp_Elips2d() - invalid construction parameters");
    }

    /// <exception cref="ConstructionErrorException">The minor radius is negative or greater than the major radius.</exception>
    public Ellipse2D(in Frame2D a, double majorRadius, double minorRadius)
    {
        _pos = a;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < minorRadius, "gp_Elips2d() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in Point2D p) => _pos.SetLocation(p);

    internal void SetMajorRadius(double majorRadius)
    {
        Standard_ConstructionError_Raise_if(majorRadius < _minorRadius,
                                            "gp_Elips2d::SetMajorRadius() - major radius should be greater or equal to minor radius");
        _majorRadius = majorRadius;
    }

    internal void SetMinorRadius(double minorRadius)
    {
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || _majorRadius < minorRadius,
                                            "gp_Elips2d::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
        _minorRadius = minorRadius;
    }

    internal void SetAxis(in Frame2D a) => _pos.SetAxis(a);

    internal void SetXAxis(in Axis2D a) => _pos.SetXAxis(a);

    internal void SetYAxis(in Axis2D a) => _pos.SetYAxis(a);

    internal void Reverse()
    {
        Direction2D temp = _pos.YDirection();
        temp = temp.Reversed();
        _pos = _pos.WithAxis(new Frame2D(_pos.Location(), _pos.XDirection(), temp));
    }

    internal void Mirror(in Point2D p) => _pos.Mirror(p);

    internal void Mirror(in Axis2D a) => _pos.Mirror(a);

    internal void Rotate(in Point2D p, double ang) => _pos.Rotate(p, ang);

    internal void Scale(in Point2D p, double s)
    {
        _majorRadius *= s;
        if (_majorRadius < 0)
        {
            _majorRadius = -_majorRadius;
        }

        _minorRadius *= s;
        if (_minorRadius < 0)
        {
            _minorRadius = -_minorRadius;
        }

        _pos = _pos.Scaled(p, s);
    }

    internal void Transform(in Transform2D t)
    {
        double tSca = t.ScaleFactor();
        if (tSca < 0.0)
        {
            tSca = -tSca;
        }

        _majorRadius *= tSca;
        _minorRadius *= tSca;
        _pos = _pos.Transformed(t);
    }

    internal void Translate(in Vector2D v) => _pos.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Ellipse2D WithLocation(in Point2D p)
    {
        Ellipse2D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Ellipse2D WithMajorRadius(double majorRadius)
    {
        Ellipse2D copy = this;
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    public readonly Ellipse2D WithMinorRadius(double minorRadius)
    {
        Ellipse2D copy = this;
        copy.SetMinorRadius(minorRadius);
        return copy;
    }

    public readonly Ellipse2D WithAxis(in Frame2D a)
    {
        Ellipse2D copy = this;
        copy.SetAxis(a);
        return copy;
    }

    public readonly Ellipse2D WithXAxis(in Axis2D a)
    {
        Ellipse2D copy = this;
        copy.SetXAxis(a);
        return copy;
    }

    public readonly Ellipse2D WithYAxis(in Axis2D a)
    {
        Ellipse2D copy = this;
        copy.SetYAxis(a);
        return copy;
    }

    // Queries

    public readonly double Area() => M_PI * _majorRadius * _minorRadius;

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
            Transform2D t = new();
            t = t.WithTransformation(_pos.XAxis());
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
    public readonly Axis2D Directrix1()
    {
        double e = Eccentricity();
        Standard_ConstructionError_Raise_if(e <= Constants.Resolution(), "gp_Elips2d::Directrix1() - zero eccentricity");
        Xy orig = _pos.XDirection().XY();
        orig.Multiply(_majorRadius / e);
        orig.Add(_pos.Location().XY());
        return new Axis2D(new Point2D(orig), _pos.YDirection());
    }

    /// <summary>The directrix on the negative side of the major axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
    public readonly Axis2D Directrix2()
    {
        double e = Eccentricity();
        Standard_ConstructionError_Raise_if(e <= Constants.Resolution(), "gp_Elips2d::Directrix2() - zero eccentricity");
        Xy orig = _pos.XDirection().XY();
        orig.Multiply(-_majorRadius / e);
        orig.Add(_pos.Location().XY());
        return new Axis2D(new Point2D(orig), _pos.YDirection());
    }

    public readonly double Eccentricity()
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

    /// <summary>The distance between the foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);

    public readonly Point2D Focus1()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
        Point2D pp = _pos.Location();
        Direction2D dd = _pos.XDirection();
        return new Point2D(pp.X() + c * dd.X(), pp.Y() + c * dd.Y());
    }

    public readonly Point2D Focus2()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
        Point2D pp = _pos.Location();
        Direction2D dd = _pos.XDirection();
        return new Point2D(pp.X() - c * dd.X(), pp.Y() - c * dd.Y());
    }

    public readonly Point2D Location() => _pos.Location();

    public readonly double MajorRadius() => _majorRadius;

    public readonly double MinorRadius() => _minorRadius;

    /// <summary>The parameter (semi-latus rectum) of the ellipse: MinorRadius^2 / MajorRadius.</summary>
    public readonly double Parameter()
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

    public readonly Frame2D Axis() => _pos;

    public readonly Axis2D XAxis() => _pos.XAxis();

    public readonly Axis2D YAxis() => _pos.YAxis();

    /// <summary>True when the ellipse's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;

    public readonly Ellipse2D Reversed()
    {
        Ellipse2D e = this;
        Direction2D temp = _pos.YDirection();
        temp = temp.Reversed();
        e._pos = e._pos.WithAxis(new Frame2D(_pos.Location(), _pos.XDirection(), temp));
        return e;
    }

    public readonly Ellipse2D Mirrored(in Point2D p)
    {
        Ellipse2D e = this;
        e._pos = e._pos.Mirrored(p);
        return e;
    }

    public readonly Ellipse2D Mirrored(in Axis2D a)
    {
        Ellipse2D e = this;
        e._pos = e._pos.Mirrored(a);
        return e;
    }

    public readonly Ellipse2D Rotated(in Point2D p, double ang)
    {
        Ellipse2D e = this;
        e._pos = e._pos.Rotated(p, ang);
        return e;
    }

    public readonly Ellipse2D Scaled(in Point2D p, double s)
    {
        Ellipse2D e = this;
        e._majorRadius *= s;
        if (e._majorRadius < 0)
        {
            e._majorRadius = -e._majorRadius;
        }

        e._minorRadius *= s;
        if (e._minorRadius < 0)
        {
            e._minorRadius = -e._minorRadius;
        }

        e._pos = e._pos.Scaled(p, s);
        return e;
    }

    public readonly Ellipse2D Transformed(in Transform2D t)
    {
        Ellipse2D e = this;
        e._majorRadius *= t.ScaleFactor();
        if (e._majorRadius < 0)
        {
            e._majorRadius = -e._majorRadius;
        }

        e._minorRadius *= t.ScaleFactor();
        if (e._minorRadius < 0)
        {
            e._minorRadius = -e._minorRadius;
        }

        e._pos = e._pos.Transformed(t);
        return e;
    }

    public readonly Ellipse2D Translated(in Vector2D v)
    {
        Ellipse2D e = this;
        e._pos = e._pos.Translated(v);
        return e;
    }

    public readonly Ellipse2D Translated(in Point2D p1, in Point2D p2)
    {
        Ellipse2D e = this;
        e._pos = e._pos.Translated(p1, p2);
        return e;
    }

    public readonly bool Equals(Ellipse2D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Ellipse2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Ellipse2D left, in Ellipse2D right) => left.Equals(right);

    public static bool operator !=(in Ellipse2D left, in Ellipse2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Elips2d({_pos}, major {_majorRadius}, minor {_minorRadius})";
}
