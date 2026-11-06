// Port of: src/FoundationClasses/TKMath/gp/gp_Vec2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Vec2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A vector in 2D space.</summary>
public readonly partial struct Vector2D : IEquatable<Vector2D>
{
    private readonly Xy _coord;

    /// <summary>The unit vector along <paramref name="direction"/>.</summary>
    public Vector2D(in Direction2D direction) => _coord = direction.XY();

    internal Vector2D(in Xy coord) => _coord = coord;

    /// <summary>The vector with these components.</summary>
    public Vector2D(double x, double y) => _coord = new Xy(x, y);

    /// <summary>The vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public Vector2D(in Point2D point1, in Point2D point2) => _coord = point2.XY().Subtracted(point1.XY());

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

    /// <summary>A copy with its X component replaced.</summary>
    public readonly Vector2D WithX(double x) => new(x, _coord.Y());

    /// <summary>A copy with its Y component replaced.</summary>
    public readonly Vector2D WithY(double y) => new(_coord.X(), y);

    internal readonly Vector2D WithXY(in Xy coord) => new(coord);

    // Queries

    internal readonly double Coord(int index) => _coord.Coord(index);

    internal readonly void Coord(out double xv, out double yv) => _coord.Coord(out xv, out yv);

    /// <summary>The X component.</summary>
    public readonly double X => _coord.X();

    /// <summary>The Y component.</summary>
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

    /// <summary>True when at right angles to <paramref name="other"/>, within <paramref name="angularTolerance"/> radians.</summary>
    /// <exception cref="InvalidOperationException">Either vector is shorter than the resolution.</exception>
    public readonly bool IsNormal(in Vector2D other, double angularTolerance)
    {
        double ang = Math.Abs(M_PI_2 - Math.Abs(AngleTo(other)));
        return !(ang > angularTolerance);
    }

    /// <summary>True when pointing the opposite way to <paramref name="other"/>, within <paramref name="angularTolerance"/> radians.</summary>
    /// <exception cref="InvalidOperationException">Either vector is shorter than the resolution.</exception>
    public readonly bool IsOpposite(in Vector2D other, double angularTolerance)
    {
        double ang = Math.Abs(AngleTo(other));
        return M_PI - ang <= angularTolerance;
    }

    /// <summary>True when parallel to <paramref name="other"/>, in either sense, within <paramref name="angularTolerance"/> radians.</summary>
    /// <exception cref="InvalidOperationException">Either vector is shorter than the resolution.</exception>
    public readonly bool IsParallel(in Vector2D other, double angularTolerance)
    {
        double ang = Math.Abs(AngleTo(other));
        return ang <= angularTolerance || M_PI - ang <= angularTolerance;
    }

    /// <summary>The signed angle from this vector to <paramref name="other"/>, in ]-PI, PI].</summary>
    /// <exception cref="InvalidOperationException">Either vector's magnitude is at most the resolution.</exception>
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

    /// <summary>The length.</summary>
    public readonly double Magnitude => _coord.Modulus();

    /// <summary>The square of the length, without the square root: the cheaper one to compare.</summary>
    public readonly double SquareMagnitude => _coord.SquareModulus();

    /// <summary>The sum.</summary>
    public readonly Vector2D Added(in Vector2D other)
    {
        Xy coord = _coord;
        coord.Add(other._coord);
        return new Vector2D(coord);
    }

    /// <summary>The scalar cross product.</summary>
    public readonly double Crossed(in Vector2D right) => _coord.Crossed(right._coord);

    /// <summary>The magnitude of the cross product: |this| |<paramref name="right"/>| sin(angle).</summary>
    public readonly double CrossMagnitude(in Vector2D right) => _coord.CrossMagnitude(right._coord);

    /// <summary>The square of <see cref="CrossMagnitude"/>, without its square root.</summary>
    public readonly double CrossSquareMagnitude(in Vector2D right) => _coord.CrossSquareMagnitude(right._coord);

    /// <summary>Each component divided by <paramref name="scalar"/>, which isn't checked for zero.</summary>
    public readonly Vector2D Divided(double scalar)
    {
        Xy coord = _coord;
        coord.Divide(scalar);
        return new Vector2D(coord);
    }

    /// <summary>The dot product.</summary>
    public readonly double Dot(in Vector2D other) => _coord.Dot(other._coord);

    /// <summary>The vector (Y, -X): this vector turned by -PI/2.</summary>
    public readonly Vector2D GetNormal() => new(this.Y, (-1) * this.X);

    /// <summary>Each component multiplied by <paramref name="scalar"/>.</summary>
    public readonly Vector2D Multiplied(double scalar)
    {
        Xy coord = _coord;
        coord.Multiply(scalar);
        return new Vector2D(coord);
    }

    /// <summary>The unit vector with this direction.</summary>
    /// <exception cref="InvalidOperationException">The vector is no longer than the resolution.</exception>
    public readonly Vector2D Normalized()
    {
        double d = _coord.Modulus();
        Refusal.StateIf(d <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Vec2d::Normalized() - vector has zero norm");
        Xy coord = _coord;
        coord.Divide(d);
        return new Vector2D(coord);
    }

    /// <summary>The opposite.</summary>
    public readonly Vector2D Reversed()
    {
        Xy coord = _coord;
        coord.Reverse();
        return new Vector2D(coord);
    }

    /// <summary>This vector minus <paramref name="right"/>.</summary>
    public readonly Vector2D Subtracted(in Vector2D right)
    {
        Xy coord = _coord;
        coord.Subtract(right._coord);
        return new Vector2D(coord);
    }

    /// <summary>The reflection in the line along <paramref name="vector"/>: the part along it is kept and the part across it reversed. A zero-length mirror leaves it unchanged.</summary>
    public readonly Vector2D Mirrored(in Vector2D vector)
    {
        Xy coord = _coord;
        double magnitude = vector._coord.Modulus();
        if (magnitude > Constants.Resolution())
        {
            Xy mirrorVecXY = vector._coord;
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

    /// <summary>The reflection in the direction of <paramref name="axis"/>; a vector has no position, so the axis's location doesn't matter.</summary>
    public readonly Vector2D Mirrored(in Axis2D axis)
    {
        Xy coord = _coord;
        Xy directionXY = axis.Direction.XY();
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

    /// <summary>Rotated by <paramref name="angle"/> radians, anticlockwise.</summary>
    public readonly Vector2D Rotated(double angle)
    {
        Xy coord = _coord;
        Transform2D t = new Transform2D().WithRotation(new Point2D(0.0, 0.0), angle);
        coord.Multiply(t.VectorialPart);
        return new Vector2D(coord);
    }

    /// <summary>Each component multiplied by <paramref name="factor"/>.</summary>
    public readonly Vector2D Scaled(double factor)
    {
        Xy coord = _coord;
        coord.Multiply(factor);
        return new Vector2D(coord);
    }

    /// <summary>The image under <paramref name="transform"/>'s rotation and scale: translation has no effect on a vector.</summary>
    public readonly Vector2D Transformed(in Transform2D transform)
    {
        Xy coord = _coord;
        switch (transform.Form)
        {
            case Identity:
            case Translation:
                break;
            case PointMirror:
                coord.Reverse();
                break;
            case global::Kerf.Geometry.TransformForm.Scale:
                coord.Multiply(transform.ScaleFactor);
                break;
            default:
                coord.Multiply(transform.VectorialPart);
                break;
        }
        return new Vector2D(coord);
    }

    /// <summary>The sum.</summary>
    public static Vector2D operator +(in Vector2D left, in Vector2D right) => left.Added(right);

    /// <summary>The difference.</summary>
    public static Vector2D operator -(in Vector2D left, in Vector2D right) => left.Subtracted(right);

    /// <summary>The opposite.</summary>
    public static Vector2D operator -(in Vector2D v) => v.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Vector2D left, in Vector2D right) => left.Dot(right);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in Vector2D left, in Vector2D right) => left.Crossed(right);

    /// <summary>Each component multiplied by the scalar.</summary>
    public static Vector2D operator *(in Vector2D v, double scalar) => v.Multiplied(scalar);

    /// <summary>Each component multiplied by the scalar.</summary>
    public static Vector2D operator *(double scalar, in Vector2D v) => v.Multiplied(scalar);

    /// <summary>Each component divided by the scalar, which isn't checked for zero.</summary>
    public static Vector2D operator /(in Vector2D v, double scalar) => v.Divided(scalar);

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Vector2D other) => _coord.Equals(other._coord);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Vector2D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Vector2D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => _coord.GetHashCode();

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Vector2D left, in Vector2D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Vector2D left, in Vector2D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => Constants.Format("Vector2D", _coord.X(), _coord.Y());
}
