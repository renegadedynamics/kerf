// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An ellipse in 3D space, defined by its major and minor radii and positioned by a right-handed coordinate
/// system: the origin is the centre, the X direction the major axis and the main direction the normal to its
/// plane. It is parameterised by <c>P(U) = O + MajorRad * cos(U) * XDir + MinorRad * sin(U) * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Elips()</c> is OCCT's indefinite ellipse (major radius <see cref="Standard.RealLast"/>, minor radius
/// <see cref="Standard.RealSmall"/>); <c>default(gp_Elips)</c> is not valid.
/// </remarks>
public partial struct Ellipse3D : IEquatable<Ellipse3D>
{
    private Frame3D _pos;
    private double _majorRadius;
    private double _minorRadius;

    public Ellipse3D()
    {
        _pos = new Frame3D();
        _majorRadius = RealLast();
        _minorRadius = RealSmall();
    }

    /// <exception cref="ConstructionErrorException">
    /// <paramref name="minorRadius"/> is negative or greater than <paramref name="majorRadius"/>.
    /// </exception>
    public Ellipse3D(in Frame3D a2, double majorRadius, double minorRadius)
    {
        _pos = a2;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < minorRadius,
                                            "gp_Elips() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1) => _pos.SetAxis(a1);

    internal void SetLocation(in Point3D p) => _pos.SetLocation(p);

    internal void SetMajorRadius(double majorRadius)
    {
        Standard_ConstructionError_Raise_if(
            majorRadius < _minorRadius,
            "gp_Elips::SetMajorRadius() - major radius should be greater or equal to minor radius");
        _majorRadius = majorRadius;
    }

    internal void SetMinorRadius(double minorRadius)
    {
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || _majorRadius < minorRadius,
                                            "gp_Elips::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
        _minorRadius = minorRadius;
    }

    internal void SetPosition(in Frame3D a2) => _pos = a2;

    internal void Mirror(in Point3D p) => _pos.Mirror(p);

    internal void Mirror(in Axis3D a1) => _pos.Mirror(a1);

    internal void Mirror(in Frame3D a2) => _pos.Mirror(a2);

    internal void Rotate(in Axis3D a1, double ang) => _pos.Rotate(a1, ang);

    internal void Scale(in Point3D p, double s)
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

    internal void Transform(in Transform3D t)
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

    internal void Translate(in Vector3D v) => _pos.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Ellipse3D WithAxis(in Axis3D a1)
    {
        Ellipse3D copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Ellipse3D WithLocation(in Point3D p)
    {
        Ellipse3D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Ellipse3D WithMajorRadius(double majorRadius)
    {
        Ellipse3D copy = this;
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    public readonly Ellipse3D WithMinorRadius(double minorRadius)
    {
        Ellipse3D copy = this;
        copy.SetMinorRadius(minorRadius);
        return copy;
    }

    public readonly Ellipse3D WithPosition(in Frame3D a2)
    {
        Ellipse3D copy = this;
        copy.SetPosition(a2);
        return copy;
    }

    // Queries

    public readonly double Area() => M_PI * _majorRadius * _minorRadius;

    /// <summary>The main axis: the normal to the ellipse's plane through its centre.</summary>
    public readonly Axis3D Axis() => _pos.Axis();

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
    public readonly Axis3D Directrix1()
    {
        double e = Eccentricity();
        Standard_ConstructionError_Raise_if(e <= Constants.Resolution(), "gp_Elips::Directrix1() - zero eccentricity");
        Xyz orig = _pos.XDirection().XYZ();
        orig.Multiply(_majorRadius / e);
        orig.Add(_pos.Location().XYZ());
        return new Axis3D(new Point3D(orig), _pos.YDirection());
    }

    /// <summary>The second directrix, symmetric to the first about the minor axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
    public readonly Axis3D Directrix2()
    {
        double e = Eccentricity();
        Standard_ConstructionError_Raise_if(e <= Constants.Resolution(), "gp_Elips::Directrix2() - zero eccentricity");
        Xyz orig = _pos.XDirection().XYZ();
        orig.Multiply(-_majorRadius / e);
        orig.Add(_pos.Location().XYZ());
        return new Axis3D(new Point3D(orig), _pos.YDirection());
    }

    /// <summary>sqrt(Major^2 - Minor^2) / Major, in [0, 1); 0 when the major radius is 0.</summary>
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

    /// <summary>The distance between the two foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public readonly Point3D Focus1()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
        Point3D pp = _pos.Location();
        Direction3D dd = _pos.XDirection();
        return new Point3D(pp.X() + c * dd.X(), pp.Y() + c * dd.Y(), pp.Z() + c * dd.Z());
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public readonly Point3D Focus2()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
        Point3D pp = _pos.Location();
        Direction3D dd = _pos.XDirection();
        return new Point3D(pp.X() - c * dd.X(), pp.Y() - c * dd.Y(), pp.Z() - c * dd.Z());
    }

    /// <summary>The centre of the ellipse.</summary>
    public readonly Point3D Location() => _pos.Location();

    public readonly double MajorRadius() => _majorRadius;

    public readonly double MinorRadius() => _minorRadius;

    /// <summary>The focal parameter, Minor^2 / Major; 0 when the major radius is 0.</summary>
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

    public readonly Frame3D Position() => _pos;

    /// <summary>The major axis.</summary>
    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    /// <summary>The minor axis.</summary>
    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Ellipse3D Mirrored(in Point3D p)
    {
        Ellipse3D e = this;
        e._pos = e._pos.Mirrored(p);
        return e;
    }

    public readonly Ellipse3D Mirrored(in Axis3D a1)
    {
        Ellipse3D e = this;
        e._pos = e._pos.Mirrored(a1);
        return e;
    }

    public readonly Ellipse3D Mirrored(in Frame3D a2)
    {
        Ellipse3D e = this;
        e._pos = e._pos.Mirrored(a2);
        return e;
    }

    public readonly Ellipse3D Rotated(in Axis3D a1, double ang)
    {
        Ellipse3D e = this;
        e._pos = e._pos.Rotated(a1, ang);
        return e;
    }

    public readonly Ellipse3D Scaled(in Point3D p, double s)
    {
        Ellipse3D e = this;
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

    public readonly Ellipse3D Transformed(in Transform3D t)
    {
        Ellipse3D e = this;
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

    public readonly Ellipse3D Translated(in Vector3D v)
    {
        Ellipse3D e = this;
        e._pos = e._pos.Translated(v);
        return e;
    }

    public readonly Ellipse3D Translated(in Point3D p1, in Point3D p2)
    {
        Ellipse3D e = this;
        e._pos = e._pos.Translated(p1, p2);
        return e;
    }

    public readonly bool Equals(Ellipse3D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Ellipse3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Ellipse3D left, in Ellipse3D right) => left.Equals(right);

    public static bool operator !=(in Ellipse3D left, in Ellipse3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Elips({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
