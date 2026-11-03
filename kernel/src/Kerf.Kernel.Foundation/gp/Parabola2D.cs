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
public partial struct Parabola2D : IEquatable<Parabola2D>
{
    private Frame2D _pos;
    private double _focalLength;

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
        Standard_ConstructionError_Raise_if(focalLength < 0.0, "gp_Parab2d() - focal length should be >= 0");
    }

    /// <exception cref="ConstructionErrorException">The focal length is negative.</exception>
    public Parabola2D(in Frame2D axes, double focalLength)
    {
        _pos = axes;
        _focalLength = focalLength;
        Standard_ConstructionError_Raise_if(focalLength < 0.0, "gp_Parab2d() - focal length should be >= 0");
    }

    /// <summary>
    /// The parabola with the given directrix and focus. When the focus is on the directrix, the axis of symmetry is
    /// the directrix turned by -PI/2 (or PI/2 when <paramref name="sense"/> is false) and the focal length is zero.
    /// </summary>
    public Parabola2D(in Axis2D directrix, in Point2D focus, bool sense = true)
    {
        Point2D dirLoc = directrix.Location();
        Direction2D dirVec = directrix.Direction();

        Vector2D fVec = new(dirLoc, focus);

        Point2D origin = new(dirLoc.XY() + dirVec.XY() * (fVec.Dot(new Vector2D(dirVec))));
        Point2D apex = new(0.5 * (origin.XY() + focus.XY()));

        _focalLength = 0.5 * origin.Distance(focus);

        Direction2D xDir = (_focalLength > 0.0)
                             ? new Direction2D(focus.XY() - origin.XY())
                             : directrix.Rotated(dirLoc, sense ? -M_PI_2 : M_PI_2).Direction();

        _pos = new Frame2D(apex, xDir, dirVec);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetFocal(double focal)
    {
        Standard_ConstructionError_Raise_if(focal < 0.0, "gp_Parab2d::SetFocal() - focal length should be >= 0");
        _focalLength = focal;
    }

    internal void SetLocation(in Point2D p) => _pos.SetLocation(p);

    internal void SetMirrorAxis(in Axis2D a) => _pos.SetXAxis(a);

    internal void SetAxis(in Frame2D a) => _pos.SetAxis(a);

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
        _focalLength *= s;
        if (_focalLength < 0)
        {
            _focalLength = -_focalLength;
        }

        _pos = _pos.Scaled(p, s);
    }

    internal void Transform(in Transform2D t)
    {
        _focalLength *= t.ScaleFactor();
        if (_focalLength < 0)
        {
            _focalLength = -_focalLength;
        }

        _pos = _pos.Transformed(t);
    }

    internal void Translate(in Vector2D v) => _pos.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Parabola2D WithFocal(double focal)
    {
        Parabola2D copy = this;
        copy.SetFocal(focal);
        return copy;
    }

    public readonly Parabola2D WithLocation(in Point2D p)
    {
        Parabola2D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Parabola2D WithMirrorAxis(in Axis2D a)
    {
        Parabola2D copy = this;
        copy.SetMirrorAxis(a);
        return copy;
    }

    public readonly Parabola2D WithAxis(in Frame2D a)
    {
        Parabola2D copy = this;
        copy.SetAxis(a);
        return copy;
    }

    // Queries

    /// <summary>
    /// The coefficients of the parabola's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d, out double e, out double f)
    {
        double p = 2.0 * _focalLength;
        Transform2D t = new();
        t = t.WithTransformation(_pos.XAxis());
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
    public readonly Axis2D Directrix()
    {
        Point2D p = new(_pos.Location().X() - _focalLength * _pos.XDirection().X(),
                          _pos.Location().Y() - _focalLength * _pos.XDirection().Y());
        Direction2D v = _pos.YDirection();
        return new Axis2D(p, v);
    }

    /// <summary>The distance between the apex and the focus.</summary>
    public readonly double Focal() => _focalLength;

    public readonly Point2D Focus()
        => new(_pos.Location().X() + _focalLength * _pos.XDirection().X(),
               _pos.Location().Y() + _focalLength * _pos.XDirection().Y());

    public readonly Point2D Location() => _pos.Location();

    public readonly Axis2D MirrorAxis() => _pos.XAxis();

    public readonly Frame2D Axis() => _pos;

    /// <summary>The parameter: the distance between the focus and the directrix (2 * focal length).</summary>
    public readonly double Parameter() => 2.0 * _focalLength;

    /// <summary>True when the parabola's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;

    public readonly Parabola2D Reversed()
    {
        Parabola2D p = this;
        Direction2D temp = _pos.YDirection();
        temp = temp.Reversed();
        p._pos = p._pos.WithAxis(new Frame2D(_pos.Location(), _pos.XDirection(), temp));
        return p;
    }

    public readonly Parabola2D Mirrored(in Point2D p)
    {
        Parabola2D prb = this;
        prb._pos = prb._pos.Mirrored(p);
        return prb;
    }

    public readonly Parabola2D Mirrored(in Axis2D a)
    {
        Parabola2D prb = this;
        prb._pos = prb._pos.Mirrored(a);
        return prb;
    }

    public readonly Parabola2D Rotated(in Point2D p, double ang)
    {
        Parabola2D prb = this;
        prb._pos = prb._pos.Rotated(p, ang);
        return prb;
    }

    public readonly Parabola2D Scaled(in Point2D p, double s)
    {
        Parabola2D prb = this;
        prb._focalLength *= s;
        if (prb._focalLength < 0)
        {
            prb._focalLength = -prb._focalLength;
        }

        prb._pos = prb._pos.Scaled(p, s);
        return prb;
    }

    public readonly Parabola2D Transformed(in Transform2D t)
    {
        Parabola2D prb = this;
        prb._focalLength *= t.ScaleFactor();
        if (prb._focalLength < 0)
        {
            prb._focalLength = -prb._focalLength;
        }

        prb._pos = prb._pos.Transformed(t);
        return prb;
    }

    public readonly Parabola2D Translated(in Vector2D v)
    {
        Parabola2D prb = this;
        prb._pos = prb._pos.Translated(v);
        return prb;
    }

    public readonly Parabola2D Translated(in Point2D p1, in Point2D p2)
    {
        Parabola2D prb = this;
        prb._pos = prb._pos.Translated(p1, p2);
        return prb;
    }

    public readonly bool Equals(Parabola2D other) => _pos.Equals(other._pos) && _focalLength.Equals(other._focalLength);

    public override readonly bool Equals(object? obj) => obj is Parabola2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _focalLength);

    public static bool operator ==(in Parabola2D left, in Parabola2D right) => left.Equals(right);

    public static bool operator !=(in Parabola2D left, in Parabola2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Parab2d({_pos}, focal {_focalLength})";
}
