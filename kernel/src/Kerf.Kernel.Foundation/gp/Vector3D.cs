// Port of: src/FoundationClasses/TKMath/gp/gp_Vec.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Vec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A vector in 3D space.</summary>
public readonly partial struct Vector3D : IEquatable<Vector3D>
{
    private readonly Xyz _coord;

    public Vector3D(in Direction3D v) => _coord = v.XYZ();

    internal Vector3D(in Xyz coord) => _coord = coord;

    public Vector3D(double xv, double yv, double zv) => _coord = new Xyz(xv, yv, zv);

    /// <summary>The vector from <paramref name="p1"/> to <paramref name="p2"/>.</summary>
    public Vector3D(in Point3D p1, in Point3D p2) => _coord = p2.XYZ().Subtracted(p1.XYZ());

    // Copies

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector3D LinearForm(double a1, in Vector3D v1, double a2, in Vector3D v2, double a3, in Vector3D v3, in Vector3D v4)
    {
        Xyz coord = default;
        coord.SetLinearForm(a1, v1._coord, a2, v2._coord, a3, v3._coord, v4._coord);
        return new Vector3D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector3D LinearForm(double a1, in Vector3D v1, double a2, in Vector3D v2, double a3, in Vector3D v3)
    {
        Xyz coord = default;
        coord.SetLinearForm(a1, v1._coord, a2, v2._coord, a3, v3._coord);
        return new Vector3D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector3D LinearForm(double a1, in Vector3D v1, double a2, in Vector3D v2, in Vector3D v3)
    {
        Xyz coord = default;
        coord.SetLinearForm(a1, v1._coord, a2, v2._coord, v3._coord);
        return new Vector3D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector3D LinearForm(double a1, in Vector3D v1, double a2, in Vector3D v2)
    {
        Xyz coord = default;
        coord.SetLinearForm(a1, v1._coord, a2, v2._coord);
        return new Vector3D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector3D LinearForm(double a1, in Vector3D v1, in Vector3D v2)
    {
        Xyz coord = default;
        coord.SetLinearForm(a1, v1._coord, v2._coord);
        return new Vector3D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector3D LinearForm(in Vector3D v1, in Vector3D v2)
    {
        Xyz coord = default;
        coord.SetLinearForm(v1._coord, v2._coord);
        return new Vector3D(coord);
    }

    internal readonly Vector3D WithCoord(int index, double xi)
    {
        Xyz coord = _coord;
        coord.SetCoord(index, xi);
        return new Vector3D(coord);
    }

    internal readonly Vector3D WithCoord(double xv, double yv, double zv) => new(xv, yv, zv);

    public readonly Vector3D WithX(double x) => new(x, _coord.Y(), _coord.Z());

    public readonly Vector3D WithY(double y) => new(_coord.X(), y, _coord.Z());

    public readonly Vector3D WithZ(double z) => new(_coord.X(), _coord.Y(), z);

    internal readonly Vector3D WithXYZ(in Xyz coord) => new(coord);

    // Queries

    internal readonly double Coord(int index) => _coord.Coord(index);

    internal readonly void Coord(out double xv, out double yv, out double zv)
    {
        xv = _coord.X();
        yv = _coord.Y();
        zv = _coord.Z();
    }

    public readonly double X => _coord.X();

    public readonly double Y => _coord.Y();

    public readonly double Z => _coord.Z();

    internal readonly Xyz XYZ() => _coord;

    public readonly bool IsEqual(in Vector3D other, double linearTolerance, double angularTolerance)
    {
        double magnitude = Magnitude;
        double otherMagnitude = other.Magnitude;
        if (magnitude <= linearTolerance || otherMagnitude <= linearTolerance)
        {
            double val = Math.Abs(magnitude - otherMagnitude);
            return val <= linearTolerance;
        }
        else
        {
            double val = Math.Abs(magnitude - otherMagnitude);
            return val <= linearTolerance && AngleTo(other) <= angularTolerance;
        }
    }

    public readonly bool IsNormal(in Vector3D other, double angularTolerance)
    {
        double ang = Math.Abs(M_PI_2 - AngleTo(other));
        return ang <= angularTolerance;
    }

    public readonly bool IsOpposite(in Vector3D other, double angularTolerance)
    {
        double ang = M_PI - AngleTo(other);
        return ang <= angularTolerance;
    }

    public readonly bool IsParallel(in Vector3D other, double angularTolerance)
    {
        double ang = AngleTo(other);
        return ang <= angularTolerance || M_PI - ang <= angularTolerance;
    }

    public readonly double AngleTo(in Vector3D other)
    {
        Refusal.StateIf(_coord.Modulus() <= Constants.Resolution() || other._coord.Modulus() <= Constants.Resolution(), FailureKind.NullVector, " ");
        return new Direction3D(_coord).AngleTo(new Direction3D(other._coord));
    }

    public readonly double AngleWithRef(in Vector3D other, in Vector3D vRef)
    {
        Refusal.StateIf(_coord.Modulus() <= Constants.Resolution()
                                            || vRef._coord.Modulus() <= Constants.Resolution()
                                            || other._coord.Modulus() <= Constants.Resolution(), FailureKind.NullVector, " ");
        return new Direction3D(_coord).AngleWithRef(new Direction3D(other._coord), new Direction3D(vRef._coord));
    }

    public readonly double Magnitude => _coord.Modulus();

    public readonly double SquareMagnitude => _coord.SquareModulus();

    public readonly Vector3D Added(in Vector3D other)
    {
        Xyz coord = _coord;
        coord.Add(other._coord);
        return new Vector3D(coord);
    }

    public readonly Vector3D Subtracted(in Vector3D right)
    {
        Xyz coord = _coord;
        coord.Subtract(right._coord);
        return new Vector3D(coord);
    }

    public readonly Vector3D Multiplied(double scalar)
    {
        Xyz coord = _coord;
        coord.Multiply(scalar);
        return new Vector3D(coord);
    }

    public readonly Vector3D Divided(double scalar)
    {
        Xyz coord = _coord;
        coord.Divide(scalar);
        return new Vector3D(coord);
    }

    public readonly Vector3D Crossed(in Vector3D right)
    {
        Xyz coord = _coord;
        coord.Cross(right._coord);
        return new Vector3D(coord);
    }

    public readonly double CrossMagnitude(in Vector3D right) => _coord.CrossMagnitude(right._coord);

    public readonly double CrossSquareMagnitude(in Vector3D right) => _coord.CrossSquareMagnitude(right._coord);

    public readonly Vector3D CrossCrossed(in Vector3D v1, in Vector3D v2)
    {
        Xyz coord = _coord;
        coord.CrossCross(v1._coord, v2._coord);
        return new Vector3D(coord);
    }

    public readonly double Dot(in Vector3D other) => _coord.Dot(other._coord);

    public readonly double DotCross(in Vector3D v1, in Vector3D v2) => _coord.DotCross(v1._coord, v2._coord);

    public readonly Vector3D Normalized()
    {
        double d = _coord.Modulus();
        Refusal.StateIf(d <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Vec::Normalized() - vector has zero norm");
        Xyz coord = _coord;
        coord.Divide(d);
        return new Vector3D(coord);
    }

    public readonly Vector3D Reversed()
    {
        Xyz coord = _coord;
        coord.Reverse();
        return new Vector3D(coord);
    }

    public readonly Vector3D Mirrored(in Vector3D vec)
    {
        Xyz coord = _coord;
        double magnitude = vec._coord.Modulus();
        if (magnitude > Constants.Resolution())
        {
            Xyz mirrorVecXYZ = vec._coord;
            double origX = coord.X();
            double origY = coord.Y();
            double origZ = coord.Z();

            double normDirX = mirrorVecXYZ.X() / magnitude;
            double normDirY = mirrorVecXYZ.Y() / magnitude;
            double normDirZ = mirrorVecXYZ.Z() / magnitude;

            double crossTermXY = 2.0 * normDirX * normDirY;
            double crossTermXZ = 2.0 * normDirX * normDirZ;
            double crossTermYZ = 2.0 * normDirY * normDirZ;
            double xxTerm = 2.0 * normDirX * normDirX - 1.0;
            double yyTerm = 2.0 * normDirY * normDirY - 1.0;
            double zzTerm = 2.0 * normDirZ * normDirZ - 1.0;

            coord.SetX(xxTerm * origX + crossTermXY * origY + crossTermXZ * origZ);
            coord.SetY(crossTermXY * origX + yyTerm * origY + crossTermYZ * origZ);
            coord.SetZ(crossTermXZ * origX + crossTermYZ * origY + zzTerm * origZ);
        }
        return new Vector3D(coord);
    }

    public readonly Vector3D Mirrored(in Axis3D axis)
    {
        Xyz coord = _coord;
        Xyz directionXYZ = axis.Direction.XYZ();
        double origX = coord.X();
        double origY = coord.Y();
        double origZ = coord.Z();
        double dirX = directionXYZ.X();
        double dirY = directionXYZ.Y();
        double dirZ = directionXYZ.Z();

        double crossTermXY = 2.0 * dirX * dirY;
        double crossTermXZ = 2.0 * dirX * dirZ;
        double crossTermYZ = 2.0 * dirY * dirZ;
        double xxTerm = 2.0 * dirX * dirX - 1.0;
        double yyTerm = 2.0 * dirY * dirY - 1.0;
        double zzTerm = 2.0 * dirZ * dirZ - 1.0;

        coord.SetX(xxTerm * origX + crossTermXY * origY + crossTermXZ * origZ);
        coord.SetY(crossTermXY * origX + yyTerm * origY + crossTermYZ * origZ);
        coord.SetZ(crossTermXZ * origX + crossTermYZ * origY + zzTerm * origZ);
        return new Vector3D(coord);
    }

    public readonly Vector3D Mirrored(in Frame3D axis)
    {
        Xyz coord = _coord;
        Xyz zDir = axis.Direction.XYZ();
        Xyz mirXYZ = zDir.Crossed(coord);

        if (mirXYZ.Modulus() <= Constants.Resolution())
        {
            coord.Reverse();
        }
        else
        {
            Xyz newZ = zDir;
            newZ.Cross(mirXYZ);
            return Mirrored(new Vector3D(newZ));
        }
        return new Vector3D(coord);
    }

    public readonly Vector3D Rotated(in Axis3D a1, double ang)
    {
        Xyz coord = _coord;
        Transform3D t = new Transform3D().WithRotation(a1, ang);
        coord.Multiply(t.VectorialPart);
        return new Vector3D(coord);
    }

    public readonly Vector3D Scaled(double s)
    {
        Xyz coord = _coord;
        coord.Multiply(s);
        return new Vector3D(coord);
    }

    public readonly Vector3D Transformed(in Transform3D transformation)
    {
        Xyz coord = _coord;
        if (transformation.Form == Identity || transformation.Form == Translation)
        {
        }
        else if (transformation.Form == PntMirror)
        {
            coord.Reverse();
        }
        else if (transformation.Form == global::Kerf.Geometry.TransformForm.Scale)
        {
            coord.Multiply(transformation.ScaleFactor);
        }
        else
        {
            coord.Multiply(transformation.VectorialPart);
        }
        return new Vector3D(coord);
    }

    public static Vector3D operator +(in Vector3D left, in Vector3D right) => left.Added(right);

    public static Vector3D operator -(in Vector3D left, in Vector3D right) => left.Subtracted(right);

    public static Vector3D operator -(in Vector3D v) => v.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Vector3D left, in Vector3D right) => left.Dot(right);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static Vector3D operator ^(in Vector3D left, in Vector3D right) => left.Crossed(right);

    public static Vector3D operator *(in Vector3D v, double scalar) => v.Multiplied(scalar);

    public static Vector3D operator *(double scalar, in Vector3D v) => v.Multiplied(scalar);

    public static Vector3D operator /(in Vector3D v, double scalar) => v.Divided(scalar);

    public readonly bool Equals(Vector3D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Vector3D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Vector3D left, in Vector3D right) => left.Equals(right);

    public static bool operator !=(in Vector3D left, in Vector3D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Vec", _coord.X(), _coord.Y(), _coord.Z());
}
