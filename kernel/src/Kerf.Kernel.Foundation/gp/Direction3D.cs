// Port of: src/FoundationClasses/TKMath/gp/gp_Dir.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Dir.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A unit vector in 3D space.</summary>
/// <remarks><c>new gp_Dir()</c> is (1, 0, 0), as in OCCT; <c>default(gp_Dir)</c> is all zeros and not a valid direction.</remarks>
public readonly partial struct Direction3D : IEquatable<Direction3D>
{
    /// <summary>The six directions along the axes of the global system.</summary>
    internal enum D
    {
        /// <summary>(1, 0, 0)</summary>
        X,
        /// <summary>(0, 1, 0)</summary>
        Y,
        /// <summary>(0, 0, 1)</summary>
        Z,
        /// <summary>(-1, 0, 0)</summary>
        NX,
        /// <summary>(0, -1, 0)</summary>
        NY,
        /// <summary>(0, 0, -1)</summary>
        NZ,
    }

    private readonly Xyz _coord;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    /// <remarks><paramref name="isUnit"/> tells it from the public constructor, which normalizes.</remarks>
    private Direction3D(in Xyz coord, bool isUnit)
    {
        _coord = coord;
    }

    /// <summary>The X direction, (1, 0, 0). <c>default(Direction3D)</c> is all zeros, and not a valid direction.</summary>
    public Direction3D() => _coord = new Xyz(1.0, 0.0, 0.0);

    internal Direction3D(D dir)
        => _coord = new Xyz(dir == D.X ? 1.0 : dir == D.NX ? -1.0 : 0.0,
                              dir == D.Y ? 1.0 : dir == D.NY ? -1.0 : 0.0,
                              dir == D.Z ? 1.0 : dir == D.NZ ? -1.0 : 0.0);

    /// <summary>The direction of <paramref name="vector"/>, normalised.</summary>
    /// <exception cref="ArgumentException">The vector has no length.</exception>
    public Direction3D(in Vector3D vector) : this(vector.XYZ()) { }

    internal Direction3D(in Xyz xyz) : this(xyz.X(), xyz.Y(), xyz.Z()) { }

    /// <summary>The direction of these components, normalised.</summary>
    /// <exception cref="ArgumentException">They have no length.</exception>
    public Direction3D(double x, double y, double z)
    {
        double sqMod = x * x + y * y + z * z;
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            _coord = new Xyz(x, y, z);
            return;
        }

        Refusal.ArgumentIf(sqMod <= Constants.Resolution() * Constants.Resolution(), FailureKind.ConstructionError, "gp_Dir::SetCoord() - input vector has zero norm");
        double d = Math.Sqrt(sqMod);
        _coord = new Xyz(x / d, y / d, z / d);
    }

    // Copies

    internal readonly Direction3D WithCoord(int index, double xi)
    {
        Refusal.RangeIf(index < 1 || index > 3, FailureKind.OutOfRange, "gp_Dir::SetCoord() - index is out of range [1, 3]");
        return index == 1 ? WithX(xi) : index == 2 ? WithY(xi) : WithZ(xi);
    }

    internal readonly Direction3D WithCoord(double xv, double yv, double zv) => new(xv, yv, zv);

    /// <summary>The direction with this X component replaced, then normalised, so the other components change too.</summary>
    /// <exception cref="ArgumentException">The result has no length.</exception>
    public readonly Direction3D WithX(double x) => new(x, _coord.Y(), _coord.Z());

    /// <summary>The direction with this Y component replaced, then normalised, so the other components change too.</summary>
    /// <exception cref="ArgumentException">The result has no length.</exception>
    public readonly Direction3D WithY(double y) => new(_coord.X(), y, _coord.Z());

    /// <summary>The direction with this Z component replaced, then normalised, so the other components change too.</summary>
    /// <exception cref="ArgumentException">The result has no length.</exception>
    public readonly Direction3D WithZ(double z) => new(_coord.X(), _coord.Y(), z);

    internal readonly Direction3D WithXYZ(in Xyz xyz) => new(xyz);

    // Queries

    internal readonly double Coord(int index) => _coord.Coord(index);

    internal readonly void Coord(out double xv, out double yv, out double zv) => _coord.Coord(out xv, out yv, out zv);

    /// <summary>The X component.</summary>
    public readonly double X => _coord.X();

    /// <summary>The Y component.</summary>
    public readonly double Y => _coord.Y();

    /// <summary>The Z component.</summary>
    public readonly double Z => _coord.Z();

    internal readonly Xyz XYZ() => _coord;

    /// <summary>True when the angle to <paramref name="other"/> is at most <paramref name="angularTolerance"/> radians.</summary>
    public readonly bool IsEqual(in Direction3D other, double angularTolerance) => AngleTo(other) <= angularTolerance;

    /// <summary>True when at right angles to <paramref name="other"/>, within <paramref name="angularTolerance"/> radians.</summary>
    public readonly bool IsNormal(in Direction3D other, double angularTolerance)
    {
        double ang = M_PI / 2.0 - AngleTo(other);
        if (ang < 0)
        {
            ang = -ang;
        }

        return ang <= angularTolerance;
    }

    /// <summary>True when pointing the opposite way to <paramref name="other"/>, within <paramref name="angularTolerance"/> radians.</summary>
    public readonly bool IsOpposite(in Direction3D other, double angularTolerance) => M_PI - AngleTo(other) <= angularTolerance;

    /// <summary>True when parallel to <paramref name="other"/>, in either sense, within <paramref name="angularTolerance"/> radians.</summary>
    public readonly bool IsParallel(in Direction3D other, double angularTolerance)
    {
        double ang = AngleTo(other);
        return ang <= angularTolerance || M_PI - ang <= angularTolerance;
    }

    /// <summary>The angle between them, in [0, PI] radians, with no sign.</summary>
    public readonly double AngleTo(in Direction3D other)
    {
        double cosinus = _coord.Dot(other._coord);
        if (cosinus > -0.70710678118655 && cosinus < 0.70710678118655)
        {
            return Math.Acos(cosinus);
        }
        else
        {
            double sinus = _coord.Crossed(other._coord).Modulus();
            if (cosinus < 0.0)
            {
                return M_PI - Math.Asin(sinus);
            }
            else
            {
                return Math.Asin(sinus);
            }
        }
    }

    /// <summary>The angle to <paramref name="other"/>, in [-PI, PI] radians: positive when this cross <paramref name="other"/> points to the same side as <paramref name="reference"/>.</summary>
    public readonly double AngleWithRef(in Direction3D other, in Direction3D reference)
    {
        double ang;
        Xyz xyz = _coord.Crossed(other._coord);
        double cosinus = _coord.Dot(other._coord);
        double sinus = xyz.Modulus();
        if (cosinus > -0.70710678118655 && cosinus < 0.70710678118655)
        {
            ang = Math.Acos(cosinus);
        }
        else
        {
            if (cosinus < 0.0)
            {
                ang = M_PI - Math.Asin(sinus);
            }
            else
            {
                ang = Math.Asin(sinus);
            }
        }

        if (xyz.Dot(reference._coord) >= 0.0)
        {
            return ang;
        }
        else
        {
            return -ang;
        }
    }

    /// <summary>The unit vector along this cross <paramref name="right"/>.</summary>
    /// <exception cref="ArgumentException">The two are parallel, so the cross product has no length.</exception>
    public readonly Direction3D Crossed(in Direction3D right)
    {
        Xyz coord = _coord;
        coord.Cross(right._coord);
        double sqMod = coord.SquareModulus();
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            return new Direction3D(coord, isUnit: true);
        }

        Refusal.ArgumentIf(sqMod <= Constants.Resolution() * Constants.Resolution(), FailureKind.ConstructionError, "gp_Dir::Cross() - result vector has zero norm");
        coord.Divide(Math.Sqrt(sqMod));
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The unit vector along this cross (<paramref name="v1"/> cross <paramref name="v2"/>).</summary>
    /// <exception cref="ArgumentException">The triple product has no length.</exception>
    public readonly Direction3D CrossCrossed(in Direction3D v1, in Direction3D v2)
    {
        Xyz coord = _coord;
        coord.CrossCross(v1._coord, v2._coord);
        double sqMod = coord.SquareModulus();
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            return new Direction3D(coord, isUnit: true);
        }

        Refusal.ArgumentIf(sqMod <= Constants.Resolution() * Constants.Resolution(), FailureKind.ConstructionError, "gp_Dir::CrossCross() - result vector has zero norm");
        coord.Divide(Math.Sqrt(sqMod));
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The dot product.</summary>
    public readonly double Dot(in Direction3D other) => _coord.Dot(other._coord);

    /// <summary>The scalar triple product: this dot (<paramref name="v1"/> cross <paramref name="v2"/>).</summary>
    public readonly double DotCross(in Direction3D v1, in Direction3D v2) => _coord.Dot(v1._coord.Crossed(v2._coord));

    /// <summary>The opposite direction.</summary>
    public readonly Direction3D Reversed()
    {
        Xyz coord = _coord;
        coord.Reverse();
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The half-turn about <paramref name="direction"/>: the part along it is kept and the part across it reversed.</summary>
    public readonly Direction3D Mirrored(in Direction3D direction)
    {
        Xyz coord = _coord;
        Xyz xyz = direction._coord;
        double a = xyz.X();
        double b = xyz.Y();
        double c = xyz.Z();
        double x = coord.X();
        double y = coord.Y();
        double z = coord.Z();
        double m1 = 2.0 * a * b;
        double m2 = 2.0 * a * c;
        double m3 = 2.0 * b * c;
        double xx = ((2.0 * a * a) - 1.0) * x + m1 * y + m2 * z;
        double yy = m1 * x + ((2.0 * b * b) - 1.0) * y + m3 * z;
        double zz = m2 * x + m3 * y + ((2.0 * c * c) - 1.0) * z;
        coord.SetCoord(xx, yy, zz);
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The half-turn about the direction of <paramref name="axis"/>; a direction has no position, so the axis's location doesn't matter.</summary>
    public readonly Direction3D Mirrored(in Axis3D axis)
    {
        Xyz coord = _coord;
        Xyz xyz = axis.Direction._coord;
        double a = xyz.X();
        double b = xyz.Y();
        double c = xyz.Z();
        double x = coord.X();
        double y = coord.Y();
        double z = coord.Z();
        double m1 = 2.0 * a * b;
        double m2 = 2.0 * a * c;
        double m3 = 2.0 * b * c;
        double xx = ((2.0 * a * a) - 1.0) * x + m1 * y + m2 * z;
        double yy = m1 * x + ((2.0 * b * b) - 1.0) * y + m3 * z;
        double zz = m2 * x + m3 * y + ((2.0 * c * c) - 1.0) * z;
        coord.SetCoord(xx, yy, zz);
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The reflection in a plane normal to the main direction of <paramref name="frame"/>; a direction has no position, so the frame's location doesn't matter.</summary>
    public readonly Direction3D Mirrored(in Frame3D frame)
    {
        Direction3D vz = frame.Direction;
        return Mirrored(vz).Reversed();
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction. Only the axis's direction matters: a direction has no position.</summary>
    public readonly Direction3D Rotated(in Axis3D axis, double angle)
    {
        Xyz coord = _coord;
        Transform3D t = new Transform3D().WithRotation(axis, angle);
        coord.Multiply(t.RotationPart);
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The direction <paramref name="transform"/> gives: translation has no effect, and scale only by its sign. The result stays a unit vector.</summary>
    public readonly Direction3D Transformed(in Transform3D transform)
    {
        Xyz coord = _coord;
        if (transform.Form == Identity || transform.Form == Translation)
        {
        }
        else if (transform.Form == PointMirror)
        {
            coord.Reverse();
        }
        else if (transform.Form == Scale)
        {
            if (transform.ScaleFactor < 0.0)
            {
                coord.Reverse();
            }
        }
        else
        {
            coord.Multiply(transform.RotationPart);
            double d = coord.Modulus();
            coord.Divide(d);
            if (transform.ScaleFactor < 0.0)
            {
                coord.Reverse();
            }
        }
        return new Direction3D(coord, isUnit: true);
    }

    /// <summary>The opposite direction.</summary>
    public static Direction3D operator -(in Direction3D v) => v.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Direction3D left, in Direction3D right) => left.Dot(right);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static Direction3D operator ^(in Direction3D left, in Direction3D right) => left.Crossed(right);

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Direction3D other) => _coord.Equals(other._coord);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Direction3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Direction3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => _coord.GetHashCode();

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Direction3D left, in Direction3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Direction3D left, in Direction3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => Constants.Format("Direction3D", _coord.X(), _coord.Y(), _coord.Z());
}
