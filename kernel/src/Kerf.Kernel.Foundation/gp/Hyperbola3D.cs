// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A branch of a hyperbola in 3D space, defined by its major and minor radii and positioned by a right-handed
/// coordinate system: the origin is the centre, the X direction the major axis (the branch lies on its positive
/// side) and the main direction the normal to its plane. It is parameterised by
/// <c>P(U) = O + MajRad * cosh(U) * XDir + MinRad * sinh(U) * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Hypr()</c> is OCCT's indefinite hyperbola (major radius <see cref="Standard.RealLast"/>, minor radius
/// <see cref="Standard.RealFirst"/>); <c>default(gp_Hypr)</c> is not valid.
/// </remarks>
public partial struct Hyperbola3D : IEquatable<Hyperbola3D>
{
    private Frame3D _pos;
    private double _majorRadius;
    private double _minorRadius;

    public Hyperbola3D()
    {
        _pos = new Frame3D();
        _majorRadius = RealLast();
        _minorRadius = RealFirst(); // OCCT: a negative minor radius, which no Set method or constructor accepts
    }

    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Hyperbola3D(in Frame3D a2, double majorRadius, double minorRadius)
    {
        _pos = a2;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < 0.0,
                                            "gp_Hypr() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1) => _pos.SetAxis(a1);

    internal void SetLocation(in Point3D p) => _pos = new Frame3D(p, _pos.Direction(), _pos.XDirection());

    internal void SetMajorRadius(double majorRadius)
    {
        Standard_ConstructionError_Raise_if(
            majorRadius < 0.0,
            "gp_Hypr::SetMajorRadius() - major radius should be greater or equal zero");
        _majorRadius = majorRadius;
    }

    internal void SetMinorRadius(double minorRadius)
    {
        Standard_ConstructionError_Raise_if(
            minorRadius < 0.0,
            "gp_Hypr::SetMinorRadius() - minor radius should be greater or equal zero");
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

    public readonly Hyperbola3D WithAxis(in Axis3D a1)
    {
        Hyperbola3D copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Hyperbola3D WithLocation(in Point3D p)
    {
        Hyperbola3D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Hyperbola3D WithMajorRadius(double majorRadius)
    {
        Hyperbola3D copy = this;
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    public readonly Hyperbola3D WithMinorRadius(double minorRadius)
    {
        Hyperbola3D copy = this;
        copy.SetMinorRadius(minorRadius);
        return copy;
    }

    public readonly Hyperbola3D WithPosition(in Frame3D a2)
    {
        Hyperbola3D copy = this;
        copy.SetPosition(a2);
        return copy;
    }

    // Queries

    /// <summary>The first asymptote, of direction <c>XDir + (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is zero.</exception>
    public readonly Axis3D Asymptote1()
    {
        Standard_ConstructionError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr::Asymptote1() - major radius is zero");
        Vector3D v1 = new(_pos.YDirection());
        v1 = v1.Multiplied(_minorRadius / _majorRadius);
        Vector3D v = new(_pos.XDirection());
        v = v.Added(v1);
        return new Axis3D(_pos.Location(), new Direction3D(v));
    }

    /// <summary>The second asymptote, of direction <c>XDir - (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is zero.</exception>
    public readonly Axis3D Asymptote2()
    {
        // OCCT: the message names Asymptote1
        Standard_ConstructionError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr::Asymptote1() - major radius is zero");
        Vector3D v1 = new(_pos.YDirection());
        v1 = v1.Multiplied(-_minorRadius / _majorRadius);
        Vector3D v = new(_pos.XDirection());
        v = v.Added(v1);
        return new Axis3D(_pos.Location(), new Direction3D(v));
    }

    /// <summary>The main axis: the normal to the hyperbola's plane through its centre.</summary>
    public readonly Axis3D Axis() => _pos.Axis();

    /// <summary>The branch on the positive side of the Y axis: major and minor radii swapped.</summary>
    public readonly Hyperbola3D ConjugateBranch1()
        => new(new Frame3D(_pos.Location(), _pos.Direction(), _pos.YDirection()), _minorRadius, _majorRadius);

    /// <summary>The branch on the negative side of the Y axis: major and minor radii swapped.</summary>
    public readonly Hyperbola3D ConjugateBranch2()
    {
        Direction3D d = _pos.YDirection();
        d = d.Reversed();
        return new Hyperbola3D(new Frame3D(_pos.Location(), _pos.Direction(), d), _minorRadius, _majorRadius);
    }

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly Axis3D Directrix1()
    {
        double e = Eccentricity();
        Xyz orig = _pos.XDirection().XYZ();
        orig.Multiply(_majorRadius / e);
        orig.Add(_pos.Location().XYZ());
        return new Axis3D(new Point3D(orig), _pos.YDirection());
    }

    /// <summary>The second directrix, symmetric to the first about the minor axis.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly Axis3D Directrix2()
    {
        double e = Eccentricity();
        Xyz orig = _pos.XDirection().XYZ();
        orig.Multiply(-_majorRadius / e);
        orig.Add(_pos.Location().XYZ());
        return new Axis3D(new Point3D(orig), _pos.YDirection());
    }

    /// <summary>sqrt(Major^2 + Minor^2) / Major, greater than 1.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly double Eccentricity()
    {
        Standard_DomainError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr::Eccentricity() - major radius is zero");
        return Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius) / _majorRadius;
    }

    /// <summary>The distance between the two foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public readonly Point3D Focus1()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        Point3D pp = _pos.Location();
        Direction3D dd = _pos.XDirection();
        return new Point3D(pp.X() + c * dd.X(), pp.Y() + c * dd.Y(), pp.Z() + c * dd.Z());
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public readonly Point3D Focus2()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        Point3D pp = _pos.Location();
        Direction3D dd = _pos.XDirection();
        return new Point3D(pp.X() - c * dd.X(), pp.Y() - c * dd.Y(), pp.Z() - c * dd.Z());
    }

    /// <summary>The centre of the hyperbola.</summary>
    public readonly Point3D Location() => _pos.Location();

    public readonly double MajorRadius() => _majorRadius;

    public readonly double MinorRadius() => _minorRadius;

    /// <summary>The other branch, on the negative side of the major axis.</summary>
    public readonly Hyperbola3D OtherBranch()
    {
        Direction3D d = _pos.XDirection();
        d = d.Reversed();
        return new Hyperbola3D(new Frame3D(_pos.Location(), _pos.Direction(), d), _majorRadius, _minorRadius);
    }

    /// <summary>The focal parameter, Minor^2 / Major.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly double Parameter()
    {
        Standard_DomainError_Raise_if(_majorRadius <= Constants.Resolution(), "gp_Hypr::Parameter() - major radius is zero");
        return (_minorRadius * _minorRadius) / _majorRadius;
    }

    public readonly Frame3D Position() => _pos;

    /// <summary>The major axis.</summary>
    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    /// <summary>The minor axis.</summary>
    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Hyperbola3D Mirrored(in Point3D p)
    {
        Hyperbola3D h = this;
        h._pos = h._pos.Mirrored(p);
        return h;
    }

    public readonly Hyperbola3D Mirrored(in Axis3D a1)
    {
        Hyperbola3D h = this;
        h._pos = h._pos.Mirrored(a1);
        return h;
    }

    public readonly Hyperbola3D Mirrored(in Frame3D a2)
    {
        Hyperbola3D h = this;
        h._pos = h._pos.Mirrored(a2);
        return h;
    }

    public readonly Hyperbola3D Rotated(in Axis3D a1, double ang)
    {
        Hyperbola3D h = this;
        h._pos = h._pos.Rotated(a1, ang);
        return h;
    }

    public readonly Hyperbola3D Scaled(in Point3D p, double s)
    {
        Hyperbola3D h = this;
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

    public readonly Hyperbola3D Transformed(in Transform3D t)
    {
        Hyperbola3D h = this;
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

    public readonly Hyperbola3D Translated(in Vector3D v)
    {
        Hyperbola3D h = this;
        h._pos = h._pos.Translated(v);
        return h;
    }

    public readonly Hyperbola3D Translated(in Point3D p1, in Point3D p2)
    {
        Hyperbola3D h = this;
        h._pos = h._pos.Translated(p1, p2);
        return h;
    }

    public readonly bool Equals(Hyperbola3D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Hyperbola3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Hyperbola3D left, in Hyperbola3D right) => left.Equals(right);

    public static bool operator !=(in Hyperbola3D left, in Hyperbola3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Hypr({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
