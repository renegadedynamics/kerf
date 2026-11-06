// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>
/// A branch of a hyperbola in 3D space, defined by its major and minor radii and positioned by a right-handed
/// coordinate system: the origin is the centre, the X direction the major axis (the branch lies on its positive
/// side) and the main direction the normal to its plane. It is parameterised by
/// <c>P(U) = O + MajRad * cosh(U) * XDir + MinRad * sinh(U) * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new Hyperbola3D()</c> is OCCT's indefinite hyperbola (major radius <c>double.MaxValue</c>, minor radius
/// the smallest positive number, where OCCT has a negative one that fails: #1341); <c>default(Hyperbola3D)</c> is not valid.
/// </remarks>
public readonly partial struct Hyperbola3D : IEquatable<Hyperbola3D>
{
    private readonly Frame3D _pos;
    private readonly double _majorRadius;
    private readonly double _minorRadius;

    /// <summary>An indefinite hyperbola, major radius <c>double.MaxValue</c> and minor radius the smallest positive number: a value nothing else can build, so give it real ones.</summary>
    public Hyperbola3D()
    {
        _pos = new Frame3D();
        _majorRadius = RealLast();
        _minorRadius = RealSmall(); // OCCT has a negative minor radius here, which later fails (#1341, decision 1789)
    }

    /// <exception cref="ArgumentException">A radius is negative.</exception>
    public Hyperbola3D(in Frame3D frame, double majorRadius, double minorRadius)
    {
        _pos = frame;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Refusal.ArgumentIf(!(minorRadius >= 0.0) || !(majorRadius >= 0.0), FailureKind.ConstructionError, "gp_Hypr() - invalid construction parameters");
    }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Hyperbola3D WithAxis(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(axis);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy moved so that its centre is <paramref name="point"/>; its directions stay.</summary>
    public readonly Hyperbola3D WithLocation(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = new Frame3D(point, pos.Direction, pos.XDirection);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>A copy with this major radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="majorRadius"/> is negative.</exception>
    public readonly Hyperbola3D WithMajorRadius(double majorRadius)
    {
        double newMajorRadius = _majorRadius;
        Refusal.ArgumentIf(
            majorRadius < 0.0, FailureKind.ConstructionError, "gp_Hypr::SetMajorRadius() - major radius should be greater or equal zero");
        newMajorRadius = majorRadius;
        return new Hyperbola3D(_pos, newMajorRadius, _minorRadius);
    }

    /// <summary>A copy with this minor radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="minorRadius"/> is negative.</exception>
    public readonly Hyperbola3D WithMinorRadius(double minorRadius)
    {
        double newMinorRadius = _minorRadius;
        Refusal.ArgumentIf(
            minorRadius < 0.0, FailureKind.ConstructionError, "gp_Hypr::SetMinorRadius() - minor radius should be greater or equal zero");
        newMinorRadius = minorRadius;
        return new Hyperbola3D(_pos, _majorRadius, newMinorRadius);
    }

    /// <summary>A copy positioned by <paramref name="frame"/>, keeping its size.</summary>
    public readonly Hyperbola3D WithPosition(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = frame;
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    // Queries

    /// <summary>The first asymptote, of direction <c>XDir + (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="InvalidOperationException">The major radius is zero.</exception>
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
    /// <exception cref="InvalidOperationException">The major radius is zero.</exception>
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
    /// <exception cref="InvalidOperationException">The major radius is zero.</exception>
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
    /// <exception cref="InvalidOperationException">The major radius is zero.</exception>
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
    /// <exception cref="InvalidOperationException">The major radius is zero.</exception>
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

    /// <summary>The distance from the centre to the apex of the branch, along the X axis.</summary>
    public readonly double MajorRadius => _majorRadius;

    /// <summary>The semi-axis along the Y axis, which sets the slope of the asymptotes.</summary>
    public readonly double MinorRadius => _minorRadius;

    /// <summary>The other branch, on the negative side of the major axis.</summary>
    public readonly Hyperbola3D OtherBranch()
    {
        Direction3D d = _pos.XDirection;
        d = d.Reversed();
        return new Hyperbola3D(new Frame3D(_pos.Location, _pos.Direction, d), _majorRadius, _minorRadius);
    }

    /// <summary>The focal parameter, Minor^2 / Major.</summary>
    /// <exception cref="InvalidOperationException">The major radius is zero.</exception>
    public readonly double Parameter
    {
        get
        {
            Refusal.StateIf(_majorRadius <= Constants.Resolution(), FailureKind.DomainError, "gp_Hypr::Parameter() - major radius is zero");
            return (_minorRadius * _minorRadius) / _majorRadius;
        }
    }

    /// <summary>The frame the curve lies in: its main direction is the plane's normal, and its X direction is parameter 0's.</summary>
    public readonly Frame3D Position => _pos;

    /// <summary>The major axis.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The minor axis.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Hyperbola3D Mirrored(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(point);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Hyperbola3D Mirrored(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Hyperbola3D Mirrored(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(frame);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Hyperbola3D Rotated(in Axis3D axis, double angle)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: both radii are multiplied by |factor|.</summary>
    public readonly Hyperbola3D Scaled(in Point3D point, double factor)
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
        return new Hyperbola3D(pos, majorRadius, minorRadius);
    }

    /// <summary>The image under <paramref name="transform"/>: both radii are multiplied by |ScaleFactor|.</summary>
    public readonly Hyperbola3D Transformed(in Transform3D transform)
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
        return new Hyperbola3D(pos, majorRadius, minorRadius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Hyperbola3D Translated(in Vector3D vector)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(vector);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Hyperbola3D Translated(in Point3D point1, in Point3D point2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Hyperbola3D(pos, _majorRadius, _minorRadius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Hyperbola3D other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Hyperbola3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Hyperbola3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Hyperbola3D left, in Hyperbola3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Hyperbola3D left, in Hyperbola3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => FormattableString.Invariant($"Hyperbola3D({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
