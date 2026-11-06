// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>
/// A general (affine) transformation in 2D space: <c>matrix * P + loc</c>, where the matrix need not be
/// orthogonal. Its form is gp_Other unless it was built from a <see cref="Transform2D"/>.
/// </summary>
/// <remarks><c>new gp_GTrsf2d()</c> is the identity, as in OCCT; <c>default(gp_GTrsf2d)</c> is not valid.</remarks>
public readonly partial struct GeneralTransform2D : IEquatable<GeneralTransform2D>
{
    private readonly Matrix2x2 _matrix;
    private readonly Xy _loc;
    private readonly TransformForm _shape;
    private readonly double _scale;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    private GeneralTransform2D(in Matrix2x2 matrix, in Xy loc, TransformForm shape, double scale)
    {
        _matrix = matrix;
        _loc = loc;
        _shape = shape;
        _scale = scale;
    }

    /// <summary>The fields, for the returning forms that take a whole value back.</summary>
    private void Deconstruct(out Matrix2x2 matrix, out Xy loc, out TransformForm shape, out double scale)
    {
        matrix = _matrix;
        loc = _loc;
        shape = _shape;
        scale = _scale;
    }

    /// <summary>The identity.</summary>
    public GeneralTransform2D()
    {
        _matrix = new Matrix2x2();
        _loc = new Xy(0.0, 0.0);
        _shape = Identity;
        _scale = 1.0;
        _matrix.SetScale(1.0);
    }

    /// <summary>The same transformation as <paramref name="transform"/>.</summary>
    public GeneralTransform2D(in Transform2D transform)
    {
        _shape = transform._shape;
        _matrix = transform._matrix;
        _loc = transform._loc;
        _scale = transform._scale;
    }

    /// <summary>The transformation <c>theM * P + theV</c>.</summary>
    internal GeneralTransform2D(in Matrix2x2 m, in Xy v)
    {
        _matrix = m;
        _loc = v;
        _shape = Other;
        _scale = 0.0;
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The affinity of ratio <paramref name="ratio"/> along the normal to the axis <paramref name="axis"/>.</summary>
    public static GeneralTransform2D Affinity(in Axis2D axis, double ratio) => new GeneralTransform2D().WithAffinity(axis, ratio);

    // Copies

    /// <summary>The affinity that multiplies distances from the axis by <paramref name="ratio"/>, not a uniform scale. It replaces this value rather than composing with it.</summary>
    public readonly GeneralTransform2D WithAffinity(in Axis2D A, double ratio)
    {
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        shape = Other;
        scale = 0.0;
        double a = A.Direction.X;
        double b = A.Direction.Y;
        matrix.SetValue(1, 1, (1.0 - ratio) * a * a + ratio);
        matrix.SetValue(2, 2, (1.0 - ratio) * b * b + ratio);
        matrix.SetValue(1, 2, (1.0 - ratio) * a * b);
        matrix.SetValue(2, 1, matrix.Value(1, 2));
        loc = A.Location.XY();
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(A.Location.XY());
        return new GeneralTransform2D(matrix, loc, shape, scale);
    }

    internal readonly GeneralTransform2D WithValue(int row, int col, double value)
    {
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        TransformForm shape = _shape;
        Refusal.RangeIf(row < 1 || row > 2 || col < 1 || col > 3, FailureKind.OutOfRange, " ");
        if (col == 3)
        {
            loc.SetCoord(row, value);
        }
        else
        {
            matrix.SetValue(row, col, value);
        }

        shape = Other;
        return new GeneralTransform2D(matrix, loc, shape, _scale);
    }

    internal readonly GeneralTransform2D WithTranslationPart(in Xy coord)
    {
        Xy loc = _loc;
        TransformForm shape = _shape;
        loc = coord;
        if (Form == Composite || Form == Other || Form == Translation)
        {
        }
        else if (Form == Identity)
        {
            shape = Translation;
        }
        else
        {
            shape = Composite;
        }
        return new GeneralTransform2D(_matrix, loc, shape, _scale);
    }

    /// <summary>The same transformation as <paramref name="transform"/>. It replaces this value rather than composing with it.</summary>
    public readonly GeneralTransform2D WithTransform(in Transform2D transform) => new(transform);

    internal readonly GeneralTransform2D WithVectorialPart(in Matrix2x2 matrix)
    {
        Matrix2x2 newMatrix = _matrix;
        TransformForm shape = _shape;
        double scale = _scale;
        newMatrix = matrix;
        shape = Other;
        scale = 0.0;
        return new GeneralTransform2D(newMatrix, _loc, shape, scale);
    }

    // Queries

    /// <summary>True when the 2 by 2 part's determinant is negative: the transformation mirrors.</summary>
    public readonly bool IsNegative => _matrix.Determinant() < 0.0;

    /// <summary>True when the 2 by 2 part is singular, so the transformation can't be inverted.</summary>
    public readonly bool IsSingular => _matrix.IsSingular();

    /// <summary><see cref="TransformForm.Other"/> unless this is really a <see cref="Transform2D"/>, as the kind of that.</summary>
    public readonly TransformForm Form => _shape;

    internal readonly Xy TranslationXy() => _loc;

    internal readonly Matrix2x2 VectorialPart => _matrix;

    /// <summary>The coefficient at a row (1 to 2) and column (1 to 3) of the 2x3 matrix.</summary>
    internal readonly double Value(int row, int col)
    {
        Refusal.RangeIf(row < 1 || row > 2 || col < 1 || col > 3, FailureKind.OutOfRange, " ");
        if (col == 3)
        {
            return _loc.Coord(row);
        }

        if (_shape == Other)
        {
            return _matrix.Value(row, col);
        }

        return _scale * _matrix.Value(row, col);
    }

    /// <summary>The coefficient at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    public readonly double this[int row, int col] => Value(row, col);

    /// <summary>The transformation that undoes this one. A singular 2 by 2 part is refused.</summary>
    public readonly GeneralTransform2D Inverted()
    {
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (shape == Other)
        {
            matrix.Invert();
            loc.Multiply(matrix);
            loc.Reverse();
        }
        else
        {
            Transform2D t = ToTransform();
            t = t.Inverted();
            (matrix, loc, shape, scale) = new GeneralTransform2D(matrix, loc, shape, scale).WithTransform(t);
        }
        return new GeneralTransform2D(matrix, loc, shape, scale);
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="transform"/> first.</summary>
    public readonly GeneralTransform2D Multiplied(in GeneralTransform2D transform)
    {
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (Form == Other || transform.Form == Other)
        {
            shape = Other;
            loc.Add(transform._loc.Multiplied(matrix));
            matrix.Multiply(transform._matrix);
        }
        else
        {
            Transform2D t1 = ToTransform();
            Transform2D t2 = transform.ToTransform();
            t1 = t1.Multiplied(t2);
            matrix = t1._matrix;
            loc = t1._loc;
            scale = t1._scale;
            shape = t1._shape;
        }
        return new GeneralTransform2D(matrix, loc, shape, scale);
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly GeneralTransform2D PreMultiplied(in GeneralTransform2D transform)
    {
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (Form == Other || transform.Form == Other)
        {
            shape = Other;
            loc.Multiply(transform._matrix);
            loc.Add(transform._loc);
            matrix.PreMultiply(transform._matrix);
        }
        else
        {
            Transform2D t1 = ToTransform();
            Transform2D t2 = transform.ToTransform();
            t1 = t1.PreMultiplied(t2);
            matrix = t1._matrix;
            loc = t1._loc;
            scale = t1._scale;
            shape = t1._shape;
        }
        return new GeneralTransform2D(matrix, loc, shape, scale);
    }

    /// <summary>This transformation composed with itself <paramref name="n"/> times: the identity for 0, and the inverse's powers for a negative <paramref name="n"/>.</summary>
    public readonly GeneralTransform2D Powered(int n)
    {
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (n == 0)
        {
            scale = 1.0;
            shape = Identity;
            matrix.SetIdentity();
            loc = new Xy(0.0, 0.0);
        }
        else if (n == 1)
        {
        }
        else if (n == -1)
        {
            (matrix, loc, shape, scale) = new GeneralTransform2D(matrix, loc, shape, scale).Inverted();
        }
        else
        {
            if (n < 0)
            {
                (matrix, loc, shape, scale) = new GeneralTransform2D(matrix, loc, shape, scale).Inverted();
            }

            if (shape == Other)
            {
                int npower = Math.Abs(n) - 1;
                Xy temploc = loc;
                Matrix2x2 tempmatrix = matrix;
                for (;;)
                {
                    if (IsOdd(npower))
                    {
                        loc.Add(temploc.Multiplied(matrix));
                        matrix.Multiply(tempmatrix);
                    }

                    if (npower == 1)
                    {
                        break;
                    }

                    temploc.Add(temploc.Multiplied(tempmatrix));
                    tempmatrix.Multiply(tempmatrix);
                    npower = npower / 2;
                }
            }
            else
            {
                Transform2D t = ToTransform();
                t = t.Powered(n);
                (matrix, loc, shape, scale) = new GeneralTransform2D(matrix, loc, shape, scale).WithTransform(t);
            }
        }
        return new GeneralTransform2D(matrix, loc, shape, scale);
    }

    internal readonly void Transforms(ref Xy coord)
    {
        coord.Multiply(_matrix);
        if (!(_shape == Other) && !(_scale == 1.0))
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    internal readonly Xy Transformed(in Xy coord)
    {
        Xy newCoord = coord;
        Transforms(ref newCoord);
        return newCoord;
    }

    internal readonly void Transforms(ref double x, ref double y)
    {
        Xy doublet = new(x, y);
        doublet.Multiply(_matrix);
        if (!(_shape == Other) && !(_scale == 1.0))
        {
            doublet.Multiply(_scale);
        }

        doublet.Add(_loc);
        doublet.Coord(out x, out y);
    }

    /// <summary>This transformation as a <see cref="Transform2D"/>.</summary>
    /// <exception cref="ConstructionErrorException">The form is gp_Other, or the matrix is not orthogonal.</exception>
    public readonly Transform2D ToTransform()
    {
        // Test of orthogonality
        double tolerance = global::Kerf.Engine.Foundation.Precision.Angular();
        double tolerance2 = 2.0 * tolerance;

        if (Form == Other)
        {
            throw Refusal.State(new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(0)"));
        }

        double value = (_matrix.Value(1, 1) * _matrix.Value(1, 1) + _matrix.Value(2, 1) * _matrix.Value(2, 1));
        if (Math.Abs(value - 1.0) > tolerance2)
        {
            throw Refusal.State(new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(1)"));
        }

        value = (_matrix.Value(1, 2) * _matrix.Value(1, 2) + _matrix.Value(2, 2) * _matrix.Value(2, 2));
        if (Math.Abs(value - 1.0) > tolerance2)
        {
            throw Refusal.State(new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(2)"));
        }

        value = (_matrix.Value(1, 1) * _matrix.Value(1, 2) + _matrix.Value(2, 1) * _matrix.Value(2, 2));
        if (Math.Abs(value) > tolerance)
        {
            throw Refusal.State(new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(3)"));
        }

        return new Transform2D(_scale, _shape, _matrix, _loc);
    }

    /// <summary>The composition: <paramref name="right"/> is applied first, then <paramref name="left"/>.</summary>
    public static GeneralTransform2D operator *(in GeneralTransform2D left, in GeneralTransform2D right) => left.Multiplied(right);

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(GeneralTransform2D other)
        => _matrix.Equals(other._matrix) && _loc.Equals(other._loc) && _shape == other._shape && _scale.Equals(other._scale);

    /// <summary>True when <paramref name="obj"/> is a <see cref="GeneralTransform2D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is GeneralTransform2D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_matrix, _loc, _shape, _scale);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in GeneralTransform2D left, in GeneralTransform2D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in GeneralTransform2D left, in GeneralTransform2D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"GeneralTransform2D({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
