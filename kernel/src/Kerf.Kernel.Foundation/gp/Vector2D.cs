// Port of: src/FoundationClasses/TKMath/gp/gp_Vec2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Vec2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A vector in 2D space.</summary>
public readonly partial struct Vector2D : IEquatable<Vector2D>
{
    private readonly Xy _coord;

    public Vector2D(in Direction2D v) => _coord = v.XY();

    internal Vector2D(in Xy coord) => _coord = coord;

    public Vector2D(double xv, double yv) => _coord = new Xy(xv, yv);

    /// <summary>The vector from <paramref name="p1"/> to <paramref name="p2"/>.</summary>
    public Vector2D(in Point2D p1, in Point2D p2) => _coord = p2.XY().Subtracted(p1.XY());

    // Copies

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector2D LinearForm(double a1, in Vector2D v1, double a2, in Vector2D v2, in Vector2D v3)
    {
        Xy coord = default;
        coord.SetLinearForm(a1, v1._coord, a2, v2._coord, v3._coord);
        return new Vector2D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector2D LinearForm(double a1, in Vector2D v1, double a2, in Vector2D v2)
    {
        Xy coord = default;
        coord.SetLinearForm(a1, v1._coord, a2, v2._coord);
        return new Vector2D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector2D LinearForm(double a1, in Vector2D v1, in Vector2D v2)
    {
        Xy coord = default;
        coord.SetLinearForm(a1, v1._coord, v2._coord);
        return new Vector2D(coord);
    }

    /// <summary>The linear form of the arguments, as OCCT's <c>SetLinearForm</c> computes it.</summary>
    public static Vector2D LinearForm(in Vector2D v1, in Vector2D v2)
    {
        Xy coord = default;
        coord.SetLinearForm(v1._coord, v2._coord);
        return new Vector2D(coord);
    }

    internal readonly Vector2D WithCoord(int index, double xi)
    {
        Xy coord = _coord;
        coord.SetCoord(index, xi);
        return new Vector2D(coord);
    }

    internal readonly Vector2D WithCoord(double xv, double yv) => new(xv, yv);

    public readonly Vector2D WithX(double x) => new(x, _coord.Y());

    public readonly Vector2D WithY(double y) => new(_coord.X(), y);

    internal readonly Vector2D WithXY(in Xy coord) => new(coord);

    // Queries

    internal readonly double Coord(int index) => _coord.Coord(index);

    internal readonly void Coord(out double xv, out double yv) => _coord.Coord(out xv, out yv);

    public readonly double X => _coord.X();

    public readonly double Y => _coord.Y();

    internal readonly Xy XY() => _coord;

    /// <summary>True when the lengths differ by at most the linear tolerance and, if both are longer than it, the angle is at most the angular tolerance.</summary>
    public readonly bool IsEqual(in Vector2D other, double linearTolerance, double angularTolerance)
    {
        double norm = Magnitude;
        double otherNorm = other.Magnitude;
        double val = Math.Abs(norm - otherNorm);

        bool isEqualLength = (val <= linearTolerance);

        if (norm > linearTolerance && otherNorm > linearTolerance)
        {
            double ang = Math.Abs(AngleTo(other));
            return isEqualLength && (ang <= angularTolerance);
        }

        return isEqualLength;
    }

    public readonly bool IsNormal(in Vector2D other, double angularTolerance)
    {
        double ang = Math.Abs(M_PI_2 - Math.Abs(AngleTo(other)));
        return !(ang > angularTolerance);
    }

    public readonly bool IsOpposite(in Vector2D other, double angularTolerance)
    {
        double ang = Math.Abs(AngleTo(other));
        return M_PI - ang <= angularTolerance;
    }

    public readonly bool IsParallel(in Vector2D other, double angularTolerance)
    {
        double ang = Math.Abs(AngleTo(other));
        return ang <= angularTolerance || M_PI - ang <= angularTolerance;
    }

    /// <summary>The signed angle from this vector to <paramref name="other"/>, in ]-PI, PI].</summary>
    /// <exception cref="NullVectorException">Either vector's magnitude is at most <see cref="Constants.Resolution"/>.</exception>
    public readonly double AngleTo(in Vector2D other)
    {
        double norm = Magnitude;
        double otherNorm = other.Magnitude;
        if (norm <= Constants.Resolution() || otherNorm <= Constants.Resolution())
        {
            throw Refusal.State(new NullVectorException());
        }

        double d = norm * otherNorm;
        double cosinus = _coord.Dot(other._coord) / d;
        double sinus = _coord.Crossed(other._coord) / d;

        const double cos_45_DEG = 0.70710678118654752440; // M_SQRT1_2

        if (cosinus > -cos_45_DEG && cosinus < cos_45_DEG)
        {
            return (sinus > 0.0) ? Math.Acos(cosinus) : -Math.Acos(cosinus);
        }
        else
        {
            if (cosinus > 0.0)
            {
                return Math.Asin(sinus);
            }
            else
            {
                return (sinus > 0.0) ? M_PI - Math.Asin(sinus) : -M_PI - Math.Asin(sinus);
            }
        }
    }

    public readonly double Magnitude => _coord.Modulus();

    public readonly double SquareMagnitude => _coord.SquareModulus();

    public readonly Vector2D Added(in Vector2D other)
    {
        Xy coord = _coord;
        coord.Add(other._coord);
        return new Vector2D(coord);
    }

    /// <summary>The scalar cross product.</summary>
    public readonly double Crossed(in Vector2D right) => _coord.Crossed(right._coord);

    public readonly double CrossMagnitude(in Vector2D right) => _coord.CrossMagnitude(right._coord);

    public readonly double CrossSquareMagnitude(in Vector2D right) => _coord.CrossSquareMagnitude(right._coord);

    public readonly Vector2D Divided(double scalar)
    {
        Xy coord = _coord;
        coord.Divide(scalar);
        return new Vector2D(coord);
    }

    public readonly double Dot(in Vector2D other) => _coord.Dot(other._coord);

    /// <summary>The vector (Y, -X): this vector turned by -PI/2.</summary>
    public readonly Vector2D GetNormal() => new(this.Y, (-1) * this.X);

    public readonly Vector2D Multiplied(double scalar)
    {
        Xy coord = _coord;
        coord.Multiply(scalar);
        return new Vector2D(coord);
    }

    public readonly Vector2D Normalized()
    {
        double d = _coord.Modulus();
        Refusal.StateIf(d <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Vec2d::Normalized() - vector has zero norm");
        Xy coord = _coord;
        coord.Divide(d);
        return new Vector2D(coord);
    }

    public readonly Vector2D Reversed()
    {
        Xy coord = _coord;
        coord.Reverse();
        return new Vector2D(coord);
    }

    public readonly Vector2D Subtracted(in Vector2D right)
    {
        Xy coord = _coord;
        coord.Subtract(right._coord);
        return new Vector2D(coord);
    }

    public readonly Vector2D Mirrored(in Vector2D v)
    {
        Xy coord = _coord;
        double magnitude = v._coord.Modulus();
        if (magnitude > Constants.Resolution())
        {
            Xy mirrorVecXY = v._coord;
            double origX = coord.X();
            double origY = coord.Y();

            double normDirX = mirrorVecXY.X() / magnitude;
            double normDirY = mirrorVecXY.Y() / magnitude;

            double crossTerm = 2.0 * normDirX * normDirY;
            double xxTerm = 2.0 * normDirX * normDirX - 1.0;
            double yyTerm = 2.0 * normDirY * normDirY - 1.0;

            coord.SetX(xxTerm * origX + crossTerm * origY);
            coord.SetY(crossTerm * origX + yyTerm * origY);
        }
        return new Vector2D(coord);
    }

    public readonly Vector2D Mirrored(in Axis2D a1)
    {
        Xy coord = _coord;
        Xy directionXY = a1.Direction.XY();
        double origX = coord.X();
        double origY = coord.Y();
        double dirX = directionXY.X();
        double dirY = directionXY.Y();

        double crossTerm = 2.0 * dirX * dirY;
        double xxTerm = 2.0 * dirX * dirX - 1.0;
        double yyTerm = 2.0 * dirY * dirY - 1.0;

        coord.SetX(xxTerm * origX + crossTerm * origY);
        coord.SetY(crossTerm * origX + yyTerm * origY);
        return new Vector2D(coord);
    }

    public readonly Vector2D Rotated(double ang)
    {
        Xy coord = _coord;
        Transform2D t = new Transform2D().WithRotation(new Point2D(0.0, 0.0), ang);
        coord.Multiply(t.VectorialPart);
        return new Vector2D(coord);
    }

    public readonly Vector2D Scaled(double s)
    {
        Xy coord = _coord;
        coord.Multiply(s);
        return new Vector2D(coord);
    }

    public readonly Vector2D Transformed(in Transform2D t)
    {
        Xy coord = _coord;
        switch (t.Form)
        {
            case Identity:
            case Translation:
                break;
            case PntMirror:
                coord.Reverse();
                break;
            case global::Kerf.Geometry.TransformForm.Scale:
                coord.Multiply(t.ScaleFactor);
                break;
            default:
                coord.Multiply(t.VectorialPart);
                break;
        }
        return new Vector2D(coord);
    }

    public static Vector2D operator +(in Vector2D left, in Vector2D right) => left.Added(right);

    public static Vector2D operator -(in Vector2D left, in Vector2D right) => left.Subtracted(right);

    public static Vector2D operator -(in Vector2D v) => v.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Vector2D left, in Vector2D right) => left.Dot(right);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in Vector2D left, in Vector2D right) => left.Crossed(right);

    public static Vector2D operator *(in Vector2D v, double scalar) => v.Multiplied(scalar);

    public static Vector2D operator *(double scalar, in Vector2D v) => v.Multiplied(scalar);

    public static Vector2D operator /(in Vector2D v, double scalar) => v.Divided(scalar);

    public readonly bool Equals(Vector2D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Vector2D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Vector2D left, in Vector2D right) => left.Equals(right);

    public static bool operator !=(in Vector2D left, in Vector2D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Vec2d", _coord.X(), _coord.Y());
}
