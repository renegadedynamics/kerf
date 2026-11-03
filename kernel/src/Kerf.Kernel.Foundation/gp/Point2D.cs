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
public readonly partial struct Point2D : IEquatable<Point2D>
{
    private readonly Xy _coord;

    internal Point2D(in Xy coord) => _coord = coord;

    public Point2D(double xp, double yp) => _coord = new Xy(xp, yp);

    // Copies

    internal readonly Point2D WithCoord(int index, double xi)
    {
        Xy coord = _coord;
        coord.SetCoord(index, xi);
        return new Point2D(coord);
    }

    internal readonly Point2D WithCoord(double xp, double yp) => new(xp, yp);

    public readonly Point2D WithX(double x) => new(x, _coord.Y());

    public readonly Point2D WithY(double y) => new(_coord.X(), y);

    internal readonly Point2D WithXY(in Xy coord) => new(coord);

    // Queries

    internal readonly double Coord(int index) => _coord.Coord(index);

    internal readonly void Coord(out double xp, out double yp) => _coord.Coord(out xp, out yp);

    public readonly double X => _coord.X();

    public readonly double Y => _coord.Y();

    internal readonly Xy XY() => _coord;

    internal readonly Xy Coord() => _coord;

    public readonly bool IsEqual(in Point2D other, double linearTolerance) => DistanceTo(other) <= linearTolerance;

    public readonly double DistanceTo(in Point2D other)
    {
        Xy xy = other._coord;
        double x = _coord.X() - xy.X();
        double y = _coord.Y() - xy.Y();
        return Math.Sqrt(x * x + y * y);
    }

    public readonly double SquareDistanceTo(in Point2D other)
    {
        Xy xy = other._coord;
        double x = _coord.X() - xy.X();
        double y = _coord.Y() - xy.Y();
        return (x * x + y * y);
    }

    public readonly Point2D Mirrored(in Point2D p)
    {
        Xy coord = _coord;
        coord.Reverse();
        Xy xy = p._coord;
        xy.Multiply(2.0);
        coord.Add(xy);
        return new Point2D(coord);
    }

    public readonly Point2D Mirrored(in Axis2D a)
    {
        Xy coord = _coord;
        Transform2D t = new Transform2D().WithMirror(a);
        t.Transforms(ref coord);
        return new Point2D(coord);
    }

    public readonly Point2D Rotated(in Point2D p, double ang)
    {
        Xy coord = _coord;
        Transform2D t = new Transform2D().WithRotation(p, ang);
        t.Transforms(ref coord);
        return new Point2D(coord);
    }

    public readonly Point2D Scaled(in Point2D p, double s)
    {
        Xy coord = _coord;
        Xy xy = p._coord;
        xy.Multiply(1.0 - s);
        coord.Multiply(s);
        coord.Add(xy);
        return new Point2D(coord);
    }

    public readonly Point2D Transformed(in Transform2D t)
    {
        Xy coord = _coord;
        if (t.Form == Identity)
        {
        }
        else if (t.Form == Translation)
        {
            coord.Add(t.TranslationXy());
        }
        else if (t.Form == global::Kerf.Geometry.TransformForm.Scale)
        {
            coord.Multiply(t.ScaleFactor);
            coord.Add(t.TranslationXy());
        }
        else if (t.Form == PntMirror)
        {
            coord.Reverse();
            coord.Add(t.TranslationXy());
        }
        else
        {
            t.Transforms(ref coord);
        }
        return new Point2D(coord);
    }

    public readonly Point2D Translated(in Vector2D v)
    {
        Xy coord = _coord;
        coord.Add(v.XY());
        return new Point2D(coord);
    }

    public readonly Point2D Translated(in Point2D p1, in Point2D p2)
    {
        Xy coord = _coord;
        coord.Add(p2._coord);
        coord.Subtract(p1._coord);
        return new Point2D(coord);
    }

    public readonly bool Equals(Point2D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Point2D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Point2D left, in Point2D right) => left.Equals(right);

    public static bool operator !=(in Point2D left, in Point2D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Pnt2d", _coord.X(), _coord.Y());
}
