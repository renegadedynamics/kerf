// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using System.Diagnostics.CodeAnalysis;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A point in 3D space.</summary>
public partial struct Point3D : IEquatable<Point3D>
{
    private Xyz _coord;

    public Point3D(in Xyz coord) => _coord = coord;

    public Point3D(double xp, double yp, double zp) => _coord = new Xyz(xp, yp, zp);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int index, double xi) => _coord.SetCoord(index, xi);

    internal void SetCoord(double xp, double yp, double zp) => _coord.SetCoord(xp, yp, zp);

    internal void SetX(double x) => _coord.SetX(x);

    internal void SetY(double y) => _coord.SetY(y);

    internal void SetZ(double z) => _coord.SetZ(z);

    internal void SetXYZ(in Xyz coord) => _coord = coord;

    [UnscopedRef]
    internal ref Xyz ChangeCoord() => ref _coord;

    internal void BaryCenter(double alpha, in Point3D p, double beta)
    {
        _coord.SetLinearForm(alpha, _coord, beta, p._coord);
        _coord.Divide(alpha + beta);
    }

    internal void Mirror(in Point3D p)
    {
        _coord.Reverse();
        Xyz xyz = p._coord;
        xyz.Multiply(2.0);
        _coord.Add(xyz);
    }

    internal void Mirror(in Axis3D a1)
    {
        Transform3D t = new();
        t = t.WithMirror(a1);
        t.Transforms(ref _coord);
    }

    internal void Mirror(in Frame3D a2)
    {
        Transform3D t = new();
        t = t.WithMirror(a2);
        t.Transforms(ref _coord);
    }

    internal void Rotate(in Axis3D a1, double ang)
    {
        Transform3D t = new();
        t = t.WithRotation(a1, ang);
        t.Transforms(ref _coord);
    }

    internal void Scale(in Point3D p, double s)
    {
        Xyz xyz = p._coord;
        xyz.Multiply(1.0 - s);
        _coord.Multiply(s);
        _coord.Add(xyz);
    }

    internal void Transform(in Transform3D t)
    {
        if (t.Form() == Identity)
        {
        }
        else if (t.Form() == Translation)
        {
            _coord.Add(t.TranslationPart());
        }
        else if (t.Form() == global::Kerf.Geometry.TransformForm.Scale)
        {
            _coord.Multiply(t.ScaleFactor());
            _coord.Add(t.TranslationPart());
        }
        else if (t.Form() == PntMirror)
        {
            _coord.Reverse();
            _coord.Add(t.TranslationPart());
        }
        else
        {
            t.Transforms(ref _coord);
        }
    }

    internal void Translate(in Vector3D v) => _coord.Add(v.XYZ());

    internal void Translate(in Point3D p1, in Point3D p2)
    {
        _coord.Add(p2._coord);
        _coord.Subtract(p1._coord);
    }

    // Copies with a mutator applied

    public readonly Point3D WithCoord(int index, double xi)
    {
        Point3D copy = this;
        copy.SetCoord(index, xi);
        return copy;
    }

    public readonly Point3D WithCoord(double xp, double yp, double zp) => new(xp, yp, zp);

    public readonly Point3D WithX(double x) => new(x, _coord.Y(), _coord.Z());

    public readonly Point3D WithY(double y) => new(_coord.X(), y, _coord.Z());

    public readonly Point3D WithZ(double z) => new(_coord.X(), _coord.Y(), z);

    public readonly Point3D WithXYZ(in Xyz coord) => new(coord);

    /// <summary>The barycentre of this point with weight <paramref name="alpha"/> and <paramref name="p"/> with weight <paramref name="beta"/>.</summary>
    public readonly Point3D BaryCentered(double alpha, in Point3D p, double beta)
    {
        Point3D copy = this;
        copy.BaryCenter(alpha, p, beta);
        return copy;
    }

    // Queries

    public readonly double Coord(int index) => _coord.Coord(index);

    public readonly void Coord(out double xp, out double yp, out double zp) => _coord.Coord(out xp, out yp, out zp);

    public readonly double X() => _coord.X();

    public readonly double Y() => _coord.Y();

    public readonly double Z() => _coord.Z();

    public readonly Xyz XYZ() => _coord;

    public readonly Xyz Coord() => _coord;

    public readonly bool IsEqual(in Point3D other, double linearTolerance) => Distance(other) <= linearTolerance;

    public readonly double Distance(in Point3D other) => Math.Sqrt(SquareDistance(other));

    public readonly double SquareDistance(in Point3D other)
    {
        Xyz xyz = other._coord;
        double dx = _coord.X() - xyz.X();
        double dy = _coord.Y() - xyz.Y();
        double dz = _coord.Z() - xyz.Z();
        return dx * dx + dy * dy + dz * dz;
    }

    public readonly Point3D Mirrored(in Point3D p)
    {
        Point3D pres = this;
        pres.Mirror(p);
        return pres;
    }

    public readonly Point3D Mirrored(in Axis3D a1)
    {
        Point3D p = this;
        p.Mirror(a1);
        return p;
    }

    public readonly Point3D Mirrored(in Frame3D a2)
    {
        Point3D p = this;
        p.Mirror(a2);
        return p;
    }

    public readonly Point3D Rotated(in Axis3D a1, double ang)
    {
        Point3D p = this;
        p.Rotate(a1, ang);
        return p;
    }

    public readonly Point3D Scaled(in Point3D p, double s)
    {
        Point3D pres = this;
        pres.Scale(p, s);
        return pres;
    }

    public readonly Point3D Transformed(in Transform3D t)
    {
        Point3D p = this;
        p.Transform(t);
        return p;
    }

    public readonly Point3D Translated(in Vector3D v)
    {
        Point3D p = this;
        p._coord.Add(v.XYZ());
        return p;
    }

    public readonly Point3D Translated(in Point3D p1, in Point3D p2)
    {
        Point3D p = this;
        p.Translate(p1, p2);
        return p;
    }

    public readonly bool Equals(Point3D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Point3D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Point3D left, in Point3D right) => left.Equals(right);

    public static bool operator !=(in Point3D left, in Point3D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Pnt", _coord.X(), _coord.Y(), _coord.Z());
}
