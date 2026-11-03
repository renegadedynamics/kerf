// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A branch of a hyperbola in 2D space, positioned by a coordinate system whose origin is the centre and
/// whose X axis is the major axis. The branch lies on the positive side of the X axis.
/// </summary>
/// <remarks>
/// <c>new gp_Hypr2d()</c> is OCCT's indefinite hyperbola (both radii <see cref="Standard.RealLast"/>);
/// <c>default(gp_Hypr2d)</c> is not valid.
/// </remarks>
public partial struct Hyperbola2D : IEquatable<Hyperbola2D>
{
    private Frame2D _pos;
    private double _majorRadius;
    private double _minorRadius;

    public Hyperbola2D()
    {
        _pos = new Frame2D();
        _majorRadius = RealLast();
        _minorRadius = RealLast();
    }

    /// <summary>A hyperbola whose major axis is <paramref name="majorAxis"/>, direct when <paramref name="isSense"/> is true.</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Hyperbola2D(in Axis2D majorAxis, double majorRadius, double minorRadius, bool isSense = true)
    {
        _pos = new Frame2D(majorAxis, isSense);
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < 0.0, "gp_Hypr2d() - invalid construction parameters");
    }

    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Hyperbola2D(in Frame2D a, double majorRadius, double minorRadius)
    {
        _pos = a;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < 0.0, "gp_Hypr2d() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in Point2D p) => _pos.SetLocation(p);

    internal void SetMajorRadius(double majorRadius)
    {
        Standard_ConstructionError_Raise_if(majorRadius < 0.0, "gp_Hypr2d::SetMajorRadius() - major radius should be greater or equal zero");
        _majorRadius = majorRadius;
    }

    internal void SetMinorRadius(double minorRadius)
    {
        Standard_ConstructionError_Raise_if(minorRadius < 0.0, "gp_Hypr2d::SetMinorRadius() - minor radius should be greater or equal zero");
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
        _majorRadius *= t.ScaleFactor();
        if (_majorRadius < 0)
        {
            _majorRadius = -_majorRadius;
        }

        _minorRadius *= t.ScaleFactor();
        if (_minorRadius < 0)
        {
            _minorRadius = -_minorRadius;
        }

        _pos = _pos.Transformed(t);
    }

    internal void Translate(in Vector2D v) => _pos.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Hyperbola2D WithLocation(in Point2D p)
    {
        Hyperbola2D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Hyperbola2D WithMajorRadius(double majorRadius)
    {
        Hyperbola2D copy = this;
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    public readonly Hyperbola2D WithMinorRadius(double minorRadius)
    {
        Hyperbola2D copy = this;
        copy.SetMinorRadius(minorRadius);
        return copy;
    }

    public readonly Hyperbola2D WithAxis(in Frame2D a)
    {
        Hyperbola2D copy = this;
        copy.SetAxis(a);
        return copy;
    }

    public readonly Hyperbola2D WithXAxis(in Axis2D a)
    {
        Hyperbola2D copy = this;
        copy.SetXAxis(a);
        return copy;
    }

    public readonly Hyperbola2D WithYAxis(in Axis2D a)
    {
        Hyperbola2D copy = this;
        copy.SetYAxis(a);
        return copy;
    }

    // Queries

    /// <summary>The first asymptote, meant to have direction X + (MinorRadius / MajorRadius) * Y.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly Axis2D Asymptote1()
    {
        Standard_ConstructionError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr2d::Asymptote1() - major radius is zero");
        Direction2D vdir = _pos.XDirection();
        // OCCT builds the direction from the Y direction alone (Y + (b/a) Y), where gp_Hypr adds the X direction,
        // so the result is along Y (docs/occt-issues.md); kept so results match OCCT
        Xy coord1 = _pos.YDirection().XY();
        Xy coord2 = coord1.Multiplied(_minorRadius / _majorRadius);
        coord1.Add(coord2);
        vdir = vdir.WithXY(coord1);
        return new Axis2D(_pos.Location(), vdir);
    }

    /// <summary>The second asymptote, meant to have direction X - (MinorRadius / MajorRadius) * Y.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly Axis2D Asymptote2()
    {
        Standard_ConstructionError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr2d::Asymptote2() - major radius is zero");
        Vector2D vdir = new(_pos.XDirection());
        // OCCT builds the direction from the Y direction alone (Y - (b/a) Y), where gp_Hypr adds the X direction,
        // so the result is along Y, or throws when the radii are equal (docs/occt-issues.md); kept so results match OCCT
        Xy coord1 = _pos.YDirection().XY();
        Xy coord2 = coord1.Multiplied(-_minorRadius / _majorRadius);
        coord1.Add(coord2);
        vdir = vdir.WithXY(coord1);
        return new Axis2D(_pos.Location(), new Direction2D(vdir));
    }

    /// <summary>
    /// The coefficients of the hyperbola's implicit equation
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
                a = (t11 * t11 / dMaj) - (t21 * t21 / dMin);
                b = (t12 * t12 / dMaj) - (t22 * t22 / dMin);
                c = (t11 * t12 / dMaj) - (t21 * t22 / dMin);
                d = (t11 * t13 / dMaj) - (t21 * t23 / dMin);
                e = (t12 * t13 / dMaj) - (t22 * t23 / dMin);
                f = (t13 * t13 / dMaj) - (t23 * t23 / dMin) - 1.0;
            }
        }
    }

    /// <summary>The conjugate branch on the positive side of the Y axis.</summary>
    public readonly Hyperbola2D ConjugateBranch1()
    {
        Direction2D v = _pos.YDirection();
        bool isSign = (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;
        return new Hyperbola2D(new Axis2D(_pos.Location(), v), _minorRadius, _majorRadius, isSign);
    }

    /// <summary>The conjugate branch on the negative side of the Y axis.</summary>
    public readonly Hyperbola2D ConjugateBranch2()
    {
        Direction2D v = _pos.YDirection().Reversed();
        bool isSign = (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;
        return new Hyperbola2D(new Axis2D(_pos.Location(), v), _minorRadius, _majorRadius, isSign);
    }

    /// <summary>The directrix on the positive side of the major axis, at distance MajorRadius / e from the centre.</summary>
    /// <exception cref="DomainErrorException">The major radius is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly Axis2D Directrix1()
    {
        double e = Eccentricity();
        Xy orig = _pos.XDirection().XY();
        orig.Multiply(_majorRadius / e);
        orig.Add(_pos.Location().XY());
        return new Axis2D(new Point2D(orig), _pos.YDirection());
    }

    /// <summary>The second directrix.</summary>
    /// <exception cref="DomainErrorException">The major radius is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly Axis2D Directrix2()
    {
        double e = Eccentricity();
        Xy orig = _pos.XDirection().XY();
        // OCCT places it at Focus1 + (Parameter / e) X, where gp_Hypr uses Location - (MajorRadius / e) X
        // (docs/occt-issues.md); kept so results match OCCT
        orig.Multiply(Parameter() / e);
        orig.Add(Focus1().XY());
        return new Axis2D(new Point2D(orig), _pos.YDirection());
    }

    /// <exception cref="DomainErrorException">The major radius is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly double Eccentricity()
    {
        Standard_DomainError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr2d::Eccentricity() - major radius is zero");
        return Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius) / _majorRadius;
    }

    /// <summary>The distance between the foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);

    public readonly Point2D Focus1()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        return new Point2D(_pos.Location().X() + c * _pos.XDirection().X(),
                            _pos.Location().Y() + c * _pos.XDirection().Y());
    }

    public readonly Point2D Focus2()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        return new Point2D(_pos.Location().X() - c * _pos.XDirection().X(),
                            _pos.Location().Y() - c * _pos.XDirection().Y());
    }

    public readonly Point2D Location() => _pos.Location();

    public readonly double MajorRadius() => _majorRadius;

    public readonly double MinorRadius() => _minorRadius;

    /// <summary>The branch on the negative side of the X axis.</summary>
    public readonly Hyperbola2D OtherBranch()
    {
        bool isSign = (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;
        return new Hyperbola2D(new Axis2D(_pos.Location(), _pos.XDirection().Reversed()), _majorRadius, _minorRadius, isSign);
    }

    /// <summary>The parameter (semi-latus rectum) of the hyperbola: MinorRadius^2 / MajorRadius.</summary>
    /// <exception cref="DomainErrorException">The major radius is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly double Parameter()
    {
        Standard_DomainError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr2d::Parameter() - major radius is zero");
        return (_minorRadius * _minorRadius) / _majorRadius;
    }

    public readonly Frame2D Axis() => _pos;

    public readonly Axis2D XAxis() => _pos.XAxis();

    public readonly Axis2D YAxis() => _pos.YAxis();

    /// <summary>True when the hyperbola's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;

    public readonly Hyperbola2D Reversed()
    {
        Hyperbola2D h = this;
        Direction2D temp = _pos.YDirection();
        temp = temp.Reversed();
        h._pos = h._pos.WithAxis(new Frame2D(_pos.Location(), _pos.XDirection(), temp));
        return h;
    }

    public readonly Hyperbola2D Mirrored(in Point2D p)
    {
        Hyperbola2D h = this;
        h._pos = h._pos.Mirrored(p);
        return h;
    }

    public readonly Hyperbola2D Mirrored(in Axis2D a)
    {
        Hyperbola2D h = this;
        h._pos = h._pos.Mirrored(a);
        return h;
    }

    public readonly Hyperbola2D Rotated(in Point2D p, double ang)
    {
        Hyperbola2D h = this;
        h._pos = h._pos.Rotated(p, ang);
        return h;
    }

    public readonly Hyperbola2D Scaled(in Point2D p, double s)
    {
        Hyperbola2D h = this;
        h._majorRadius *= s;
        if (h._majorRadius < 0)
        {
            h._majorRadius = -h._majorRadius;
        }

        h._minorRadius *= s;
        if (h._minorRadius < 0)
        {
            h._minorRadius = -h._minorRadius;
        }

        h._pos = h._pos.Scaled(p, s);
        return h;
    }

    public readonly Hyperbola2D Transformed(in Transform2D t)
    {
        Hyperbola2D h = this;
        h._majorRadius *= t.ScaleFactor();
        if (h._majorRadius < 0)
        {
            h._majorRadius = -h._majorRadius;
        }

        h._minorRadius *= t.ScaleFactor();
        if (h._minorRadius < 0)
        {
            h._minorRadius = -h._minorRadius;
        }

        h._pos = h._pos.Transformed(t);
        return h;
    }

    public readonly Hyperbola2D Translated(in Vector2D v)
    {
        Hyperbola2D h = this;
        h._pos = h._pos.Translated(v);
        return h;
    }

    public readonly Hyperbola2D Translated(in Point2D p1, in Point2D p2)
    {
        Hyperbola2D h = this;
        h._pos = h._pos.Translated(p1, p2);
        return h;
    }

    public readonly bool Equals(Hyperbola2D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Hyperbola2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Hyperbola2D left, in Hyperbola2D right) => left.Equals(right);

    public static bool operator !=(in Hyperbola2D left, in Hyperbola2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Hypr2d({_pos}, major {_majorRadius}, minor {_minorRadius})";
}
