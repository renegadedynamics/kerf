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
public readonly partial struct Point3D : IEquatable<Point3D>
{
    private readonly Xyz _coord;

    public Point3D(in Xyz coord) => _coord = coord;

    public Point3D(double xp, double yp, double zp) => _coord = new Xyz(xp, yp, zp);

    // Copies

    internal readonly Point3D WithCoord(int index, double xi)
    {
        Xyz coord = _coord;
        coord.SetCoord(index, xi);
        return new Point3D(coord);
    }

    internal readonly Point3D WithCoord(double xp, double yp, double zp) => new(xp, yp, zp);

    public readonly Point3D WithX(double x) => new(x, _coord.Y(), _coord.Z());

    public readonly Point3D WithY(double y) => new(_coord.X(), y, _coord.Z());

    public readonly Point3D WithZ(double z) => new(_coord.X(), _coord.Y(), z);

    internal readonly Point3D WithXYZ(in Xyz coord) => new(coord);

    /// <summary>The barycentre of this point with weight <paramref name="alpha"/> and <paramref name="p"/> with weight <paramref name="beta"/>.</summary>
    public readonly Point3D BaryCentered(double alpha, in Point3D p, double beta)
    {
        Xyz coord = _coord;
        coord.SetLinearForm(alpha, coord, beta, p._coord);
        coord.Divide(alpha + beta);
        return new Point3D(coord);
    }

    // Queries

    internal readonly double Coord(int index) => _coord.Coord(index);

    internal readonly void Coord(out double xp, out double yp, out double zp) => _coord.Coord(out xp, out yp, out zp);

    public readonly double X => _coord.X();

    public readonly double Y => _coord.Y();

    public readonly double Z => _coord.Z();

    internal readonly Xyz XYZ() => _coord;

    internal readonly Xyz Coord() => _coord;

    public readonly bool IsEqual(in Point3D other, double linearTolerance) => DistanceTo(other) <= linearTolerance;

    public readonly double DistanceTo(in Point3D other) => Math.Sqrt(SquareDistanceTo(other));

    public readonly double SquareDistanceTo(in Point3D other)
    {
        Xyz xyz = other._coord;
        double dx = _coord.X() - xyz.X();
        double dy = _coord.Y() - xyz.Y();
        double dz = _coord.Z() - xyz.Z();
        return dx * dx + dy * dy + dz * dz;
    }

    public readonly Point3D Mirrored(in Point3D p)
    {
        Xyz coord = _coord;
        coord.Reverse();
        Xyz xyz = p._coord;
        xyz.Multiply(2.0);
        coord.Add(xyz);
        return new Point3D(coord);
    }

    public readonly Point3D Mirrored(in Axis3D a1)
    {
        Xyz coord = _coord;
        Transform3D t = new Transform3D().WithMirror(a1);
        t.Transforms(ref coord);
        return new Point3D(coord);
    }

    public readonly Point3D Mirrored(in Frame3D a2)
    {
        Xyz coord = _coord;
        Transform3D t = new Transform3D().WithMirror(a2);
        t.Transforms(ref coord);
        return new Point3D(coord);
    }

    public readonly Point3D Rotated(in Axis3D a1, double ang)
    {
        Xyz coord = _coord;
        Transform3D t = new Transform3D().WithRotation(a1, ang);
        t.Transforms(ref coord);
        return new Point3D(coord);
    }

    public readonly Point3D Scaled(in Point3D p, double s)
    {
        Xyz coord = _coord;
        Xyz xyz = p._coord;
        xyz.Multiply(1.0 - s);
        coord.Multiply(s);
        coord.Add(xyz);
        return new Point3D(coord);
    }

    public readonly Point3D Transformed(in Transform3D t)
    {
        Xyz coord = _coord;
        if (t.Form == Identity)
        {
        }
        else if (t.Form == Translation)
        {
            coord.Add(t.TranslationXyz());
        }
        else if (t.Form == global::Kerf.Geometry.TransformForm.Scale)
        {
            coord.Multiply(t.ScaleFactor);
            coord.Add(t.TranslationXyz());
        }
        else if (t.Form == PntMirror)
        {
            coord.Reverse();
            coord.Add(t.TranslationXyz());
        }
        else
        {
            t.Transforms(ref coord);
        }
        return new Point3D(coord);
    }

    public readonly Point3D Translated(in Vector3D v)
    {
        Xyz coord = _coord;
        coord.Add(v.XYZ());
        return new Point3D(coord);
    }

    public readonly Point3D Translated(in Point3D p1, in Point3D p2)
    {
        Xyz coord = _coord;
        coord.Add(p2._coord);
        coord.Subtract(p1._coord);
        return new Point3D(coord);
    }

    public readonly bool Equals(Point3D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Point3D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Point3D left, in Point3D right) => left.Equals(right);

    public static bool operator !=(in Point3D left, in Point3D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Pnt", _coord.X(), _coord.Y(), _coord.Z());
}
