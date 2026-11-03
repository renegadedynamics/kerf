// Port of: src/FoundationClasses/TKMath/gp/gp_Parab.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Parab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A parabola in 3D space, defined by its focal length and positioned by a right-handed coordinate system: the
/// origin is the apex, the X direction the axis of symmetry and the main direction the normal to its plane. It is
/// parameterised by <c>P(U) = O + U^2 / (4 * F) * XDir + U * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Parab()</c> is OCCT's indefinite parabola (focal length <see cref="Standard.RealLast"/>);
/// <c>default(gp_Parab)</c> is not valid.
/// </remarks>
public partial struct Parabola3D : IEquatable<Parabola3D>
{
    private Frame3D _pos;
    private double _focalLength;

    public Parabola3D()
    {
        _pos = new Frame3D();
        _focalLength = RealLast();
    }

    /// <exception cref="ConstructionErrorException"><paramref name="focal"/> is negative.</exception>
    public Parabola3D(in Frame3D a2, double focal)
    {
        _pos = a2;
        _focalLength = focal;
        Standard_ConstructionError_Raise_if(focal < 0.0, "gp_Parab() - focal length should be >= 0");
    }

    /// <summary>The parabola with directrix <paramref name="d"/> and focus <paramref name="f"/>.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="f"/> is on <paramref name="d"/>.</exception>
    public Parabola3D(in Axis3D d, in Point3D f)
    {
        Line3D droite = new(d);
        _focalLength = droite.Distance(f) / 2.0;
        Axis3D ax = droite.Normal(f).Position();
        Axis3D ay = droite.Position();
        Direction3D dd = ax.Direction();
        _pos = new Frame3D(new Point3D(f.X() - _focalLength * dd.X(),
                                    f.Y() - _focalLength * dd.Y(),
                                    f.Z() - _focalLength * dd.Z()),
                         ax.Direction().Crossed(ay.Direction()),
                         ax.Direction());
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1) => _pos.SetAxis(a1);

    internal void SetFocal(double focal)
    {
        Standard_ConstructionError_Raise_if(focal < 0.0, "gp_Parab::SetFocal() - focal length should be >= 0");
        _focalLength = focal;
    }

    internal void SetLocation(in Point3D p) => _pos.SetLocation(p);

    internal void SetPosition(in Frame3D a2) => _pos = a2;

    internal void Mirror(in Point3D p) => _pos.Mirror(p);

    internal void Mirror(in Axis3D a1) => _pos.Mirror(a1);

    internal void Mirror(in Frame3D a2) => _pos.Mirror(a2);

    internal void Rotate(in Axis3D a1, double ang) => _pos.Rotate(a1, ang);

    internal void Scale(in Point3D p, double s)
    {
        _focalLength *= s;
        if (_focalLength < 0)
        {
            _focalLength = -_focalLength;
        }

        _pos = _pos.Scaled(p, s);
    }

    internal void Transform(in Transform3D t)
    {
        _focalLength *= t.ScaleFactor();
        if (_focalLength < 0)
        {
            _focalLength = -_focalLength;
        }

        _pos = _pos.Transformed(t);
    }

    internal void Translate(in Vector3D v) => _pos.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Parabola3D WithAxis(in Axis3D a1)
    {
        Parabola3D copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Parabola3D WithFocal(double focal)
    {
        Parabola3D copy = this;
        copy.SetFocal(focal);
        return copy;
    }

    public readonly Parabola3D WithLocation(in Point3D p)
    {
        Parabola3D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Parabola3D WithPosition(in Frame3D a2)
    {
        Parabola3D copy = this;
        copy.SetPosition(a2);
        return copy;
    }

    // Queries

    /// <summary>The main axis: the normal to the parabola's plane through its apex.</summary>
    public readonly Axis3D Axis() => _pos.Axis();

    /// <summary>The directrix: parallel to the Y axis, at the focal length from the apex on the negative X side.</summary>
    public readonly Axis3D Directrix()
    {
        Point3D pp = _pos.Location();
        Direction3D dd = _pos.XDirection();
        Point3D p = new(pp.X() - _focalLength * dd.X(),
                        pp.Y() - _focalLength * dd.Y(),
                        pp.Z() - _focalLength * dd.Z());
        return new Axis3D(p, _pos.YDirection());
    }

    /// <summary>The distance between the apex and the focus.</summary>
    public readonly double Focal() => _focalLength;

    public readonly Point3D Focus()
    {
        Point3D pp = _pos.Location();
        Direction3D dd = _pos.XDirection();
        return new Point3D(pp.X() + _focalLength * dd.X(),
                          pp.Y() + _focalLength * dd.Y(),
                          pp.Z() + _focalLength * dd.Z());
    }

    /// <summary>The apex.</summary>
    public readonly Point3D Location() => _pos.Location();

    /// <summary>The distance between the focus and the directrix: twice the focal length.</summary>
    public readonly double Parameter() => 2.0 * _focalLength;

    public readonly Frame3D Position() => _pos;

    /// <summary>The axis of symmetry.</summary>
    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Parabola3D Mirrored(in Point3D p)
    {
        Parabola3D prb = this;
        prb._pos = prb._pos.Mirrored(p);
        return prb;
    }

    public readonly Parabola3D Mirrored(in Axis3D a1)
    {
        Parabola3D prb = this;
        prb._pos = prb._pos.Mirrored(a1);
        return prb;
    }

    public readonly Parabola3D Mirrored(in Frame3D a2)
    {
        Parabola3D prb = this;
        prb._pos = prb._pos.Mirrored(a2);
        return prb;
    }

    public readonly Parabola3D Rotated(in Axis3D a1, double ang)
    {
        Parabola3D prb = this;
        prb._pos = prb._pos.Rotated(a1, ang);
        return prb;
    }

    public readonly Parabola3D Scaled(in Point3D p, double s)
    {
        Parabola3D prb = this;
        prb._focalLength *= s;
        if (prb._focalLength < 0)
        {
            prb._focalLength = -prb._focalLength;
        }

        prb._pos = prb._pos.Scaled(p, s);
        return prb;
    }

    public readonly Parabola3D Transformed(in Transform3D t)
    {
        Parabola3D prb = this;
        prb._focalLength *= t.ScaleFactor();
        if (prb._focalLength < 0)
        {
            prb._focalLength = -prb._focalLength;
        }

        prb._pos = prb._pos.Transformed(t);
        return prb;
    }

    public readonly Parabola3D Translated(in Vector3D v)
    {
        Parabola3D prb = this;
        prb._pos = prb._pos.Translated(v);
        return prb;
    }

    public readonly Parabola3D Translated(in Point3D p1, in Point3D p2)
    {
        Parabola3D prb = this;
        prb._pos = prb._pos.Translated(p1, p2);
        return prb;
    }

    public readonly bool Equals(Parabola3D other) => _pos.Equals(other._pos) && _focalLength.Equals(other._focalLength);

    public override readonly bool Equals(object? obj) => obj is Parabola3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _focalLength);

    public static bool operator ==(in Parabola3D left, in Parabola3D right) => left.Equals(right);

    public static bool operator !=(in Parabola3D left, in Parabola3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Parab({_pos}, {_focalLength:R})");
}
