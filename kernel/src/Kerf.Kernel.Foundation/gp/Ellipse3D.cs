// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
public readonly partial struct Ellipse3D : IEquatable<Ellipse3D>
{
    private readonly Frame3D _pos;
    private readonly double _majorRadius;
    private readonly double _minorRadius;

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
        Refusal.ArgumentIf(minorRadius < 0.0 || majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips() - invalid construction parameters");
    }

    // Copies

    public readonly Ellipse3D WithAxis(in Axis3D a1)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(a1);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D WithLocation(in Point3D p)
    {
        Frame3D pos = _pos;
        pos = pos.WithLocation(p);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D WithMajorRadius(double majorRadius)
    {
        double newMajorRadius = _majorRadius;
        Refusal.ArgumentIf(
            majorRadius < _minorRadius, FailureKind.ConstructionError, "gp_Elips::SetMajorRadius() - major radius should be greater or equal to minor radius");
        newMajorRadius = majorRadius;
        return new Ellipse3D(_pos, newMajorRadius, _minorRadius);
    }

    public readonly Ellipse3D WithMinorRadius(double minorRadius)
    {
        double newMinorRadius = _minorRadius;
        Refusal.ArgumentIf(minorRadius < 0.0 || _majorRadius < minorRadius, FailureKind.ConstructionError, "gp_Elips::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
        newMinorRadius = minorRadius;
        return new Ellipse3D(_pos, _majorRadius, newMinorRadius);
    }

    public readonly Ellipse3D WithPosition(in Frame3D a2)
    {
        Frame3D pos = _pos;
        pos = a2;
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    // Queries

    public readonly double Area => M_PI * _majorRadius * _minorRadius;

    /// <summary>The main axis: the normal to the ellipse's plane through its centre.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
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
    /// <exception cref="ConstructionErrorException">The eccentricity is zero (a circle).</exception>
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

    public readonly double MajorRadius => _majorRadius;

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

    public readonly Frame3D Position => _pos;

    /// <summary>The major axis.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The minor axis.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    public readonly Ellipse3D Mirrored(in Point3D p)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(p);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D Mirrored(in Axis3D a1)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(a1);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D Mirrored(in Frame3D a2)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(a2);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D Rotated(in Axis3D a1, double ang)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(a1, ang);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D Scaled(in Point3D p, double s)
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
        return new Ellipse3D(pos, majorRadius, minorRadius);
    }

    public readonly Ellipse3D Transformed(in Transform3D t)
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
        return new Ellipse3D(pos, majorRadius, minorRadius);
    }

    public readonly Ellipse3D Translated(in Vector3D v)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(v);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly Ellipse3D Translated(in Point3D p1, in Point3D p2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(p1, p2);
        return new Ellipse3D(pos, _majorRadius, _minorRadius);
    }

    public readonly bool Equals(Ellipse3D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Ellipse3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Ellipse3D left, in Ellipse3D right) => left.Equals(right);

    public static bool operator !=(in Ellipse3D left, in Ellipse3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Elips({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
