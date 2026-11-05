// Port of: src/FoundationClasses/TKMath/gp/gp_Parab2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Parab2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A parabola in 2D space, positioned by a coordinate system whose origin is the apex and whose X axis is the
/// axis of symmetry. In it the parabola is <c>Y^2 = (2 * P) * X</c>, where P = 2 * focal length.
/// </summary>
/// <remarks>
/// <c>new gp_Parab2d()</c> is OCCT's indefinite parabola (focal length <see cref="Standard.RealLast"/>);
/// <c>default(gp_Parab2d)</c> is not valid.
/// </remarks>
public readonly partial struct Parabola2D : IEquatable<Parabola2D>
{
    private readonly Frame2D _pos;
    private readonly double _focalLength;

    public Parabola2D()
    {
        _pos = new Frame2D();
        _focalLength = RealLast();
    }

    /// <summary>A parabola with the given axis of symmetry and focal length, direct when <paramref name="sense"/> is true.</summary>
    /// <exception cref="ConstructionErrorException">The focal length is negative.</exception>
    public Parabola2D(in Axis2D mirrorAxis, double focalLength, bool sense = true)
    {
        _pos = new Frame2D(mirrorAxis, sense);
        _focalLength = focalLength;
        Refusal.ArgumentIf(focalLength < 0.0, FailureKind.ConstructionError, "gp_Parab2d() - focal length should be >= 0");
    }

    /// <exception cref="ConstructionErrorException">The focal length is negative.</exception>
    public Parabola2D(in Frame2D axes, double focalLength)
    {
        _pos = axes;
        _focalLength = focalLength;
        Refusal.ArgumentIf(focalLength < 0.0, FailureKind.ConstructionError, "gp_Parab2d() - focal length should be >= 0");
    }

    /// <summary>
    /// The parabola with the given directrix and focus. When the focus is on the directrix, the axis of symmetry is
    /// the directrix turned by -PI/2 (or PI/2 when <paramref name="sense"/> is false) and the focal length is zero.
    /// </summary>
    public Parabola2D(in Axis2D directrix, in Point2D focus, bool sense = true)
    {
        Point2D dirLoc = directrix.Location;
        Direction2D dirVec = directrix.Direction;

        Vector2D fVec = new(dirLoc, focus);

        Point2D origin = new(dirLoc.XY() + dirVec.XY() * (fVec.Dot(new Vector2D(dirVec))));
        Point2D apex = new(0.5 * (origin.XY() + focus.XY()));

        _focalLength = 0.5 * origin.DistanceTo(focus);

        Direction2D xDir = (_focalLength > 0.0)
                             ? new Direction2D(focus.XY() - origin.XY())
                             : directrix.Rotated(dirLoc, sense ? -M_PI_2 : M_PI_2).Direction;

        _pos = new Frame2D(apex, xDir, dirVec);
    }

    // Copies

    public readonly Parabola2D WithFocal(double focal)
    {
        double focalLength = _focalLength;
        Refusal.ArgumentIf(focal < 0.0, FailureKind.ConstructionError, "gp_Parab2d::SetFocal() - focal length should be >= 0");
        focalLength = focal;
        return new Parabola2D(_pos, focalLength);
    }

    public readonly Parabola2D WithLocation(in Point2D p)
    {
        Frame2D pos = _pos;
        pos = pos.WithLocation(p);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D WithMirrorAxis(in Axis2D a)
    {
        Frame2D pos = _pos;
        pos = pos.WithXAxis(a);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D WithAxis(in Frame2D a)
    {
        Frame2D pos = _pos;
        pos = pos.WithAxis(a);
        return new Parabola2D(pos, _focalLength);
    }

    // Queries

    /// <summary>
    /// The coefficients of the parabola's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d, out double e, out double f)
    {
        double p = 2.0 * _focalLength;
        Transform2D t = new Transform2D().WithTransformation(_pos.XAxis);
        double t11 = t.Value(1, 1);
        double t12 = t.Value(1, 2);
        double t13 = t.Value(1, 3);
        double t21 = t.Value(2, 1);
        double t22 = t.Value(2, 2);
        double t23 = t.Value(2, 3);
        a = t21 * t21;
        b = t22 * t22;
        c = t21 * t22;
        d = (t21 * t23) - (p * t11);
        e = (t22 * t23) - (p * t12);
        f = (t23 * t23) - (2.0 * p * t13);
    }

    /// <summary>The directrix: normal to the axis of symmetry, at the focal length on the side opposite the focus.</summary>
    public readonly Axis2D Directrix
    {
        get
        {
            Point2D p = new(_pos.Location.X - _focalLength * _pos.XDirection.X,
                              _pos.Location.Y - _focalLength * _pos.XDirection.Y);
            Direction2D v = _pos.YDirection;
            return new Axis2D(p, v);
        }
    }

    /// <summary>The distance between the apex and the focus.</summary>
    public readonly double Focal => _focalLength;

    public readonly Point2D Focus
        => new(_pos.Location.X + _focalLength * _pos.XDirection.X,
               _pos.Location.Y + _focalLength * _pos.XDirection.Y);

    public readonly Point2D Location => _pos.Location;

    public readonly Axis2D MirrorAxis => _pos.XAxis;

    public readonly Frame2D Axis => _pos;

    /// <summary>The parameter: the distance between the focus and the directrix (2 * focal length).</summary>
    public readonly double Parameter => 2.0 * _focalLength;

    /// <summary>True when the parabola's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection.Crossed(_pos.YDirection)) >= 0.0;

    public readonly Parabola2D Reversed()
    {
        Frame2D pos = _pos;
        Direction2D temp = _pos.YDirection;
        temp = temp.Reversed();
        pos = pos.WithAxis(new Frame2D(_pos.Location, _pos.XDirection, temp));
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D Mirrored(in Point2D p)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(p);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D Mirrored(in Axis2D a)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(a);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D Rotated(in Point2D p, double ang)
    {
        Frame2D pos = _pos;
        pos = pos.Rotated(p, ang);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D Scaled(in Point2D p, double s)
    {
        Frame2D pos = _pos;
        double focalLength = _focalLength;
        focalLength *= s;
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos = pos.Scaled(p, s);
        return new Parabola2D(pos, focalLength);
    }

    public readonly Parabola2D Transformed(in Transform2D t)
    {
        Frame2D pos = _pos;
        double focalLength = _focalLength;
        focalLength *= t.ScaleFactor;
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos = pos.Transformed(t);
        return new Parabola2D(pos, focalLength);
    }

    public readonly Parabola2D Translated(in Vector2D v)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(v);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly Parabola2D Translated(in Point2D p1, in Point2D p2)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(p1, p2);
        return new Parabola2D(pos, _focalLength);
    }

    public readonly bool Equals(Parabola2D other) => _pos.Equals(other._pos) && _focalLength.Equals(other._focalLength);

    public override readonly bool Equals(object? obj) => obj is Parabola2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _focalLength);

    public static bool operator ==(in Parabola2D left, in Parabola2D right) => left.Equals(right);

    public static bool operator !=(in Parabola2D left, in Parabola2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Parab2d({_pos}, focal {_focalLength})";
}
