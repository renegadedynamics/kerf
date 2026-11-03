// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using System.Diagnostics.CodeAnalysis;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A point in 2D space.</summary>
public partial struct Point2D : IEquatable<Point2D>
{
    private Xy _coord;

    public Point2D(in Xy coord) => _coord = coord;

    public Point2D(double xp, double yp) => _coord = new Xy(xp, yp);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int index, double xi) => _coord.SetCoord(index, xi);

    internal void SetCoord(double xp, double yp) => _coord.SetCoord(xp, yp);

    internal void SetX(double x) => _coord.SetX(x);

    internal void SetY(double y) => _coord.SetY(y);

    internal void SetXY(in Xy coord) => _coord = coord;

    [UnscopedRef]
    internal ref Xy ChangeCoord() => ref _coord;

    internal void Mirror(in Point2D p)
    {
        _coord.Reverse();
        Xy xy = p._coord;
        xy.Multiply(2.0);
        _coord.Add(xy);
    }

    internal void Mirror(in Axis2D a)
    {
        Transform2D t = new();
        t = t.WithMirror(a);
        t.Transforms(ref _coord);
    }

    internal void Rotate(in Point2D p, double ang)
    {
        Transform2D t = new();
        t = t.WithRotation(p, ang);
        t.Transforms(ref _coord);
    }

    internal void Scale(in Point2D p, double s)
    {
        Xy xy = p._coord;
        xy.Multiply(1.0 - s);
        _coord.Multiply(s);
        _coord.Add(xy);
    }

    internal void Transform(in Transform2D t)
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

    internal void Translate(in Vector2D v) => _coord.Add(v.XY());

    internal void Translate(in Point2D p1, in Point2D p2)
    {
        _coord.Add(p2._coord);
        _coord.Subtract(p1._coord);
    }

    // Copies with a mutator applied

    public readonly Point2D WithCoord(int index, double xi)
    {
        Point2D copy = this;
        copy.SetCoord(index, xi);
        return copy;
    }

    public readonly Point2D WithCoord(double xp, double yp) => new(xp, yp);

    public readonly Point2D WithX(double x) => new(x, _coord.Y());

    public readonly Point2D WithY(double y) => new(_coord.X(), y);

    public readonly Point2D WithXY(in Xy coord) => new(coord);

    // Queries

    public readonly double Coord(int index) => _coord.Coord(index);

    public readonly void Coord(out double xp, out double yp) => _coord.Coord(out xp, out yp);

    public readonly double X() => _coord.X();

    public readonly double Y() => _coord.Y();

    public readonly Xy XY() => _coord;

    public readonly Xy Coord() => _coord;

    public readonly bool IsEqual(in Point2D other, double linearTolerance) => Distance(other) <= linearTolerance;

    public readonly double Distance(in Point2D other)
    {
        Xy xy = other._coord;
        double x = _coord.X() - xy.X();
        double y = _coord.Y() - xy.Y();
        return Math.Sqrt(x * x + y * y);
    }

    public readonly double SquareDistance(in Point2D other)
    {
        Xy xy = other._coord;
        double x = _coord.X() - xy.X();
        double y = _coord.Y() - xy.Y();
        return (x * x + y * y);
    }

    public readonly Point2D Mirrored(in Point2D p)
    {
        Point2D pres = this;
        pres.Mirror(p);
        return pres;
    }

    public readonly Point2D Mirrored(in Axis2D a)
    {
        Point2D p = this;
        p.Mirror(a);
        return p;
    }

    public readonly Point2D Rotated(in Point2D p, double ang)
    {
        Point2D pres = this;
        pres.Rotate(p, ang);
        return pres;
    }

    public readonly Point2D Scaled(in Point2D p, double s)
    {
        Point2D pres = this;
        pres.Scale(p, s);
        return pres;
    }

    public readonly Point2D Transformed(in Transform2D t)
    {
        Point2D pres = this;
        pres.Transform(t);
        return pres;
    }

    public readonly Point2D Translated(in Vector2D v)
    {
        Point2D p = this;
        p._coord.Add(v.XY());
        return p;
    }

    public readonly Point2D Translated(in Point2D p1, in Point2D p2)
    {
        Point2D p = this;
        p.Translate(p1, p2);
        return p;
    }

    public readonly bool Equals(Point2D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Point2D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Point2D left, in Point2D right) => left.Equals(right);

    public static bool operator !=(in Point2D left, in Point2D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Pnt2d", _coord.X(), _coord.Y());
}
