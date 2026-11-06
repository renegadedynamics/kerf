// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Diagnostics.CodeAnalysis;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A point in 2D space.</summary>
public readonly partial struct Point2D : IEquatable<Point2D>
{
    private readonly Xy _coord;

    internal Point2D(in Xy coord) => _coord = coord;

    public Point2D(double x, double y) => _coord = new Xy(x, y);

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

    public readonly Point2D Mirrored(in Point2D point)
    {
        Xy coord = _coord;
        coord.Reverse();
        Xy xy = point._coord;
        xy.Multiply(2.0);
        coord.Add(xy);
        return new Point2D(coord);
    }

    public readonly Point2D Mirrored(in Axis2D axis)
    {
        Xy coord = _coord;
        Transform2D t = new Transform2D().WithMirror(axis);
        t.Transforms(ref coord);
        return new Point2D(coord);
    }

    public readonly Point2D Rotated(in Point2D point, double angle)
    {
        Xy coord = _coord;
        Transform2D t = new Transform2D().WithRotation(point, angle);
        t.Transforms(ref coord);
        return new Point2D(coord);
    }

    public readonly Point2D Scaled(in Point2D point, double factor)
    {
        Xy coord = _coord;
        Xy xy = point._coord;
        xy.Multiply(1.0 - factor);
        coord.Multiply(factor);
        coord.Add(xy);
        return new Point2D(coord);
    }

    public readonly Point2D Transformed(in Transform2D transform)
    {
        Xy coord = _coord;
        if (transform.Form == Identity)
        {
        }
        else if (transform.Form == Translation)
        {
            coord.Add(transform.TranslationXy());
        }
        else if (transform.Form == global::Kerf.Geometry.TransformForm.Scale)
        {
            coord.Multiply(transform.ScaleFactor);
            coord.Add(transform.TranslationXy());
        }
        else if (transform.Form == PointMirror)
        {
            coord.Reverse();
            coord.Add(transform.TranslationXy());
        }
        else
        {
            transform.Transforms(ref coord);
        }
        return new Point2D(coord);
    }

    public readonly Point2D Translated(in Vector2D vector)
    {
        Xy coord = _coord;
        coord.Add(vector.XY());
        return new Point2D(coord);
    }

    public readonly Point2D Translated(in Point2D point1, in Point2D point2)
    {
        Xy coord = _coord;
        coord.Add(point2._coord);
        coord.Subtract(point1._coord);
        return new Point2D(coord);
    }

    public readonly bool Equals(Point2D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Point2D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Point2D left, in Point2D right) => left.Equals(right);

    public static bool operator !=(in Point2D left, in Point2D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("Point2D", _coord.X(), _coord.Y());
}
