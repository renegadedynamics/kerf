// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine;

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
public readonly partial struct Hyperbola3D : IEquatable<Hyperbola3D>
{
    private readonly Frame3D _pos;
    private readonly double _majorRadius;
    private readonly double _minorRadius;

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
        Refusal.ArgumentIf(minorRadius < 0.0 || majorRadius < 0.0, FailureKind.ConstructionError, "gp_Hypr() - invalid construction parameters");
    }

    // Copies

    public readonly Hyperbola3D WithAxis(in Axis3D a1)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(a1);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D WithLocation(in Point3D p)
    {
        Frame3D pos = _pos;
        pos = new Frame3D(p, pos.Direction, pos.XDirection);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D WithMajorRadius(double majorRadius)
    {
        double newMajorRadius = _majorRadius;
        Refusal.ArgumentIf(
            majorRadius < 0.0, FailureKind.ConstructionError, "gp_Hypr::SetMajorRadius() - major radius should be greater or equal zero");
        newMajorRadius = majorRadius;
        return new Hyperbola3D(_pos, newMajorRadius, _minorRadius);
    }

    public readonly Hyperbola3D WithMinorRadius(double minorRadius)
    {
        double newMinorRadius = _minorRadius;
        Refusal.ArgumentIf(
            minorRadius < 0.0, FailureKind.ConstructionError, "gp_Hypr::SetMinorRadius() - minor radius should be greater or equal zero");
        newMinorRadius = minorRadius;
        return new Hyperbola3D(_pos, _majorRadius, newMinorRadius);
    }

    public readonly Hyperbola3D WithPosition(in Frame3D a2)
    {
        Frame3D pos = _pos;
        pos = a2;
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    // Queries

    /// <summary>The first asymptote, of direction <c>XDir + (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is zero.</exception>
    public readonly Axis3D Asymptote1
    {
        get
        {
            Refusal.StateIf(_majorRadius <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Hypr::Asymptote1() - major radius is zero");
            Vector3D v1 = new(_pos.YDirection);
            v1 = v1.Multiplied(_minorRadius / _majorRadius);
            Vector3D v = new(_pos.XDirection);
            v = v.Added(v1);
            return new Axis3D(_pos.Location, new Direction3D(v));
        }
    }

    /// <summary>The second asymptote, of direction <c>XDir - (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is zero.</exception>
    public readonly Axis3D Asymptote2
    {
        get
        {
            // OCCT: the message names Asymptote1
            Refusal.StateIf(_majorRadius <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Hypr::Asymptote1() - major radius is zero");
            Vector3D v1 = new(_pos.YDirection);
            v1 = v1.Multiplied(-_minorRadius / _majorRadius);
            Vector3D v = new(_pos.XDirection);
            v = v.Added(v1);
            return new Axis3D(_pos.Location, new Direction3D(v));
        }
    }

    /// <summary>The main axis: the normal to the hyperbola's plane through its centre.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>The branch on the positive side of the Y axis: major and minor radii swapped.</summary>
    public readonly Hyperbola3D ConjugateBranch1()
        => new(new Frame3D(_pos.Location, _pos.Direction, _pos.YDirection), _minorRadius, _majorRadius);

    /// <summary>The branch on the negative side of the Y axis: major and minor radii swapped.</summary>
    public readonly Hyperbola3D ConjugateBranch2()
    {
        Direction3D d = _pos.YDirection;
        d = d.Reversed();
        return new Hyperbola3D(new Frame3D(_pos.Location, _pos.Direction, d), _minorRadius, _majorRadius);
    }

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly Axis3D Directrix1
    {
        get
        {
            double e = Eccentricity;
            Xyz orig = _pos.XDirection.XYZ();
            orig.Multiply(_majorRadius / e);
            orig.Add(_pos.Location.XYZ());
            return new Axis3D(new Point3D(orig), _pos.YDirection);
        }
    }

    /// <summary>The second directrix, symmetric to the first about the minor axis.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly Axis3D Directrix2
    {
        get
        {
            double e = Eccentricity;
            Xyz orig = _pos.XDirection.XYZ();
            orig.Multiply(-_majorRadius / e);
            orig.Add(_pos.Location.XYZ());
            return new Axis3D(new Point3D(orig), _pos.YDirection);
        }
    }

    /// <summary>sqrt(Major^2 + Minor^2) / Major, greater than 1.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly double Eccentricity
    {
        get
        {
            Refusal.StateIf(_majorRadius <= Constants.Resolution(), FailureKind.DomainError, "gp_Hypr::Eccentricity() - major radius is zero");
            return Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius) / _majorRadius;
        }
    }

    /// <summary>The distance between the two foci.</summary>
    public readonly double Focal => 2.0 * Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public readonly Point3D Focus1
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
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
            double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
            Point3D pp = _pos.Location;
            Direction3D dd = _pos.XDirection;
            return new Point3D(pp.X - c * dd.X, pp.Y - c * dd.Y, pp.Z - c * dd.Z);
        }
    }

    /// <summary>The centre of the hyperbola.</summary>
    public readonly Point3D Location => _pos.Location;

    public readonly double MajorRadius => _majorRadius;

    public readonly double MinorRadius => _minorRadius;

    /// <summary>The other branch, on the negative side of the major axis.</summary>
    public readonly Hyperbola3D OtherBranch()
    {
        Direction3D d = _pos.XDirection;
        d = d.Reversed();
        return new Hyperbola3D(new Frame3D(_pos.Location, _pos.Direction, d), _majorRadius, _minorRadius);
    }

    /// <summary>The focal parameter, Minor^2 / Major.</summary>
    /// <exception cref="DomainErrorException">The major radius is zero.</exception>
    public readonly double Parameter
    {
        get
        {
            Refusal.StateIf(_majorRadius <= Constants.Resolution(), FailureKind.DomainError, "gp_Hypr::Parameter() - major radius is zero");
            return (_minorRadius * _minorRadius) / _majorRadius;
        }
    }

    public readonly Frame3D Position => _pos;

    /// <summary>The major axis.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The minor axis.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    public readonly Hyperbola3D Mirrored(in Point3D p)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(p);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D Mirrored(in Axis3D a1)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(a1);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D Mirrored(in Frame3D a2)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(a2);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D Rotated(in Axis3D a1, double ang)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(a1, ang);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D Scaled(in Point3D p, double s)
    {
        Frame3D pos = _pos;
        double majorRadius = _majorRadius;
        double minorRadius = _minorRadius;
        majorRadius *= s;
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= s;
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        pos = pos.Scaled(p, s);
        return new Hyperbola3D(pos, majorRadius, minorRadius);
    }

    public readonly Hyperbola3D Transformed(in Transform3D t)
    {
        Frame3D pos = _pos;
        double majorRadius = _majorRadius;
        double minorRadius = _minorRadius;
        majorRadius *= t.ScaleFactor;
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= t.ScaleFactor;
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        pos = pos.Transformed(t);
        return new Hyperbola3D(pos, majorRadius, minorRadius);
    }

    public readonly Hyperbola3D Translated(in Vector3D v)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(v);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Hyperbola3D Translated(in Point3D p1, in Point3D p2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(p1, p2);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    public readonly bool Equals(Hyperbola3D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Hyperbola3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Hyperbola3D left, in Hyperbola3D right) => left.Equals(right);

    public static bool operator !=(in Hyperbola3D left, in Hyperbola3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Hypr({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
