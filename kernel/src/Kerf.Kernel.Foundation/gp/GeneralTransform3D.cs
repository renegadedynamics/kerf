// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>
/// A general transformation in 3D space: an affine map <c>P' = matrix * P + loc</c> whose vectorial part may be
/// any 3x3 matrix (affinities, non-uniform scales, ...). When <see cref="Form"/> is not <c>gp_Other</c>, the
/// transformation is a <see cref="Transform3D"/> and its scale factor is kept apart from the matrix.
/// </summary>
/// <remarks><c>new gp_GTrsf()</c> is the identity, as in OCCT; <c>default(gp_GTrsf)</c> is not valid.</remarks>
public readonly partial struct GeneralTransform3D : IEquatable<GeneralTransform3D>
{
    private readonly Matrix3x3 _matrix;
    private readonly Xyz _loc;
    private readonly TransformForm _shape;
    private readonly double _scale;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    private GeneralTransform3D(in Matrix3x3 matrix, in Xyz loc, TransformForm shape, double scale)
    {
        _matrix = matrix;
        _loc = loc;
        _shape = shape;
        _scale = scale;
    }

    /// <summary>The fields, for the returning forms that take a whole value back (unit 10G).</summary>
    private void Deconstruct(out Matrix3x3 matrix, out Xyz loc, out TransformForm shape, out double scale)
    {
        matrix = _matrix;
        loc = _loc;
        shape = _shape;
        scale = _scale;
    }

    public GeneralTransform3D()
    {
        _loc = new Xyz(0.0, 0.0, 0.0);
        _shape = Identity;
        _scale = 1.0;
        _matrix = new Matrix3x3();
        _matrix = _matrix.WithScale(1.0);
    }

    public GeneralTransform3D(in Transform3D t)
    {
        _shape = t.Form;
        _matrix = t._matrix;
        _loc = t.TranslationXyz();
        _scale = t.ScaleFactor;
    }

    /// <summary>The transformation with vectorial part <paramref name="m"/> and translation part <paramref name="v"/>.</summary>
    internal GeneralTransform3D(in Matrix3x3 m, in Xyz v)
    {
        _matrix = m;
        _loc = v;
        _shape = Other;
        _scale = 0.0;
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The affinity of ratio <paramref name="ratio"/> about an axis.</summary>
    public static GeneralTransform3D Affinity(in Axis3D a1, double ratio) => new GeneralTransform3D().WithAffinity(a1, ratio);

    /// <summary>The affinity of ratio <paramref name="ratio"/> about a plane.</summary>
    public static GeneralTransform3D Affinity(in Frame3D a2, double ratio) => new GeneralTransform3D().WithAffinity(a2, ratio);

    /// <summary>The transformation from a 4x4 matrix given row by row (16 values; the last row is ignored).</summary>
    public static GeneralTransform3D Mat4(ReadOnlySpan<double> mat) => new GeneralTransform3D().WithMat4(mat);

    // Copies

    public readonly GeneralTransform3D WithAffinity(in Axis3D a1, double ratio)
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        shape = Other;
        scale = 0.0;
        matrix = matrix.WithDot(a1.Direction.XYZ());
        matrix = matrix.Multiplied(1.0 - ratio);
        matrix = matrix.WithDiagonal(matrix.Value(1, 1) + ratio,
                           matrix.Value(2, 2) + ratio,
                           matrix.Value(3, 3) + ratio);
        loc = a1.Location.XYZ();
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(a1.Location.XYZ());
        return new GeneralTransform3D(matrix, loc, shape, scale);
    }

    public readonly GeneralTransform3D WithAffinity(in Frame3D a2, double ratio)
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        shape = Other;
        scale = 0.0;
        matrix = matrix.WithDot(a2.Direction.XYZ());
        matrix = matrix.Multiplied(ratio - 1.0);
        loc = a2.Location.XYZ();
        loc.Reverse();
        loc.Multiply(matrix);
        matrix = matrix.WithDiagonal(matrix.Value(1, 1) + 1.0, matrix.Value(2, 2) + 1.0, matrix.Value(3, 3) + 1.0);
        return new GeneralTransform3D(matrix, loc, shape, scale);
    }

    /// <summary>A copy with one coefficient of the 3x4 matrix replaced (row 1 to 3, column 1 to 4).</summary>
    internal readonly GeneralTransform3D WithValue(int row, int col, double value)
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 4, " ");
        if (col == 4)
        {
            loc.SetCoord(row, value);
            if (shape == Identity)
            {
                shape = Translation;
            }

            return new GeneralTransform3D(matrix, loc, shape, scale);
        }
        else
        {
            if (!(shape == Other) && !(scale == 1.0))
            {
                matrix = matrix.Multiplied(scale);
            }

            matrix = matrix.WithValue(row, col, value);
            shape = Other;
            scale = 0.0;
            return new GeneralTransform3D(matrix, loc, shape, scale);
        }
    }

    public readonly GeneralTransform3D WithVectorialPart(in Matrix3x3 matrix)
    {
        Matrix3x3 newMatrix = _matrix;
        TransformForm shape = _shape;
        double scale = _scale;
        newMatrix = matrix;
        shape = Other;
        scale = 0.0;
        return new GeneralTransform3D(newMatrix, _loc, shape, scale);
    }

    internal readonly GeneralTransform3D WithTranslationPart(in Xyz coord)
    {
        Xyz loc = _loc;
        TransformForm shape = _shape;
        loc = coord;
        if (Form == CompoundTrsf || Form == Other || Form == Translation)
        {
        }
        else if (Form == Identity)
        {
            shape = Translation;
        }
        else
        {
            shape = CompoundTrsf;
        }
        return new GeneralTransform3D(_matrix, loc, shape, _scale);
    }

    public readonly GeneralTransform3D WithTrsf(in Transform3D t) => new(t);

    /// <summary>A copy whose form is recomputed from its matrix (OCCT's <c>SetForm</c>).</summary>
    internal readonly GeneralTransform3D WithForm()
    {
        TransformForm shape = _shape;
        double tol = 1.0e-12; // Precision::Angular();
        //
        // don t trust the initial values !
        //
        Matrix3x3 m = _matrix;
        double s = m.Determinant;

        if (Math.Abs(s) < Constants.Resolution())
        {
            throw new ConstructionErrorException("gp_GTrsf::SetForm, null determinant");
        }

        if (s > 0)
        {
            s = Math.Pow(s, 1.0 / 3.0);
        }
        else
        {
            s = -Math.Pow(-s, 1.0 / 3.0);
        }

        m = m.Divided(s);

        // check if the matrix is an uniform matrix
        // the transposition should be the invert.
        Matrix3x3 tm = m;
        tm = tm.Transposed();
        tm = tm.Multiplied(m);
        Matrix3x3 identity = new Matrix3x3().WithIdentity();
        tm = tm.Subtracted(identity);
        if (shape == Other)
        {
            shape = CompoundTrsf;
        }

        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                if (Math.Abs(tm.Value(i, j)) > tol)
                {
                    shape = Other;
                    return new GeneralTransform3D(_matrix, _loc, shape, _scale);
                }
            }
        }
        return new GeneralTransform3D(_matrix, _loc, shape, _scale);
    }

    public readonly GeneralTransform3D WithMat4(ReadOnlySpan<double> mat)
    {
        Matrix3x3 newMatrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (mat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(mat));
        }

        shape = Other;
        scale = 0.0;
        newMatrix = newMatrix.WithValue(1, 1, mat[0]);
        newMatrix = newMatrix.WithValue(1, 2, mat[1]);
        newMatrix = newMatrix.WithValue(1, 3, mat[2]);
        newMatrix = newMatrix.WithValue(2, 1, mat[4]);
        newMatrix = newMatrix.WithValue(2, 2, mat[5]);
        newMatrix = newMatrix.WithValue(2, 3, mat[6]);
        newMatrix = newMatrix.WithValue(3, 1, mat[8]);
        newMatrix = newMatrix.WithValue(3, 2, mat[9]);
        newMatrix = newMatrix.WithValue(3, 3, mat[10]);
        loc.SetCoord(mat[3], mat[7], mat[11]);
        return new GeneralTransform3D(newMatrix, loc, shape, scale);
    }

    // Queries

    /// <summary>True when the determinant of the vectorial part is negative.</summary>
    public readonly bool IsNegative => _matrix.Determinant < 0.0;

    /// <summary>True when the vectorial part is singular, so the transformation cannot be inverted.</summary>
    public readonly bool IsSingular => _matrix.IsSingular;

    public readonly TransformForm Form => _shape;

    internal readonly Xyz TranslationXyz() => _loc;

    /// <summary>The 3x3 vectorial part, without the scale factor of a non-<c>gp_Other</c> form.</summary>
    public readonly Matrix3x3 VectorialPart => _matrix;

    /// <summary>The coefficient at a row (1 to 3) and column (1 to 4) of the 3x4 matrix, scale included.</summary>
    internal readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 4, " ");
        if (col == 4)
        {
            return _loc.Coord(row);
        }

        if (_shape == Other)
        {
            return _matrix.Value(row, col);
        }

        return _scale * _matrix.Value(row, col);
    }

    public readonly double this[int row, int col] => Value(row, col);

    public readonly GeneralTransform3D Inverted()
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (shape == Other)
        {
            matrix = matrix.Inverted();
            loc.Multiply(matrix);
            loc.Reverse();
        }
        else
        {
            Transform3D t = ToTransform();
            t = t.Inverted();
            (matrix, loc, shape, scale) = new GeneralTransform3D(matrix, loc, shape, scale).WithTrsf(t);
        }
        return new GeneralTransform3D(matrix, loc, shape, scale);
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly GeneralTransform3D Multiplied(in GeneralTransform3D t)
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (Form == Other || t.Form == Other)
        {
            shape = Other;
            loc.Add(t._loc.Multiplied(matrix));
            matrix = matrix.Multiplied(t._matrix);
        }
        else
        {
            Transform3D t1 = ToTransform();
            Transform3D t2 = t.ToTransform();
            t1 = t1.Multiplied(t2);
            matrix = t1._matrix;
            loc = t1._loc;
            scale = t1._scale;
            shape = t1._shape;
        }
        return new GeneralTransform3D(matrix, loc, shape, scale);
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly GeneralTransform3D PreMultiplied(in GeneralTransform3D t)
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (Form == Other || t.Form == Other)
        {
            shape = Other;
            loc.Multiply(t._matrix);
            loc.Add(t._loc);
            matrix = matrix.PreMultiplied(t._matrix);
        }
        else
        {
            Transform3D t1 = ToTransform();
            Transform3D t2 = t.ToTransform();
            t1 = t1.PreMultiplied(t2);
            matrix = t1._matrix;
            loc = t1._loc;
            scale = t1._scale;
            shape = t1._shape;
        }
        return new GeneralTransform3D(matrix, loc, shape, scale);
    }

    public readonly GeneralTransform3D Powered(int n)
    {
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        TransformForm shape = _shape;
        double scale = _scale;
        if (n == 0)
        {
            scale = 1.0;
            shape = Identity;
            matrix = matrix.WithIdentity();
            loc = new Xyz(0.0, 0.0, 0.0);
        }
        else if (n == 1)
        {
        }
        else if (n == -1)
        {
            (matrix, loc, shape, scale) = new GeneralTransform3D(matrix, loc, shape, scale).Inverted();
        }
        else
        {
            if (shape == Other)
            {
                if (n < 0)
                {
                    (matrix, loc, shape, scale) = new GeneralTransform3D(matrix, loc, shape, scale).Inverted();
                }

                int npower = Math.Abs(n) - 1;
                Xyz temploc = loc;
                Matrix3x3 tempmatrix = matrix;
                for (;;)
                {
                    if (IsOdd(npower))
                    {
                        loc.Add(temploc.Multiplied(matrix));
                        matrix = matrix.Multiplied(tempmatrix);
                    }

                    if (npower == 1)
                    {
                        break;
                    }

                    temploc.Add(temploc.Multiplied(tempmatrix));
                    tempmatrix = tempmatrix.Multiplied(tempmatrix);
                    npower = npower / 2;
                }
            }
            else
            {
                Transform3D t = ToTransform();
                t = t.Powered(n);
                (matrix, loc, shape, scale) = new GeneralTransform3D(matrix, loc, shape, scale).WithTrsf(t);
            }
        }
        return new GeneralTransform3D(matrix, loc, shape, scale);
    }

    internal readonly void Transforms(ref Xyz coord)
    {
        coord.Multiply(_matrix);
        if (!(_shape == Other) && !(_scale == 1.0))
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    internal readonly void Transforms(ref double x, ref double y, ref double z)
    {
        Xyz triplet = new(x, y, z);
        triplet.Multiply(_matrix);
        if (!(_shape == Other) && !(_scale == 1.0))
        {
            triplet.Multiply(_scale);
        }

        triplet.Add(_loc);
        triplet.Coord(out x, out y, out z);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref Xyz)"/>.</summary>
    internal readonly Xyz Transformed(in Xyz coord)
    {
        Xyz aCoord = coord;
        Transforms(ref aCoord);
        return aCoord;
    }

    /// <summary>This transformation as a <see cref="Transform3D"/>.</summary>
    /// <exception cref="ConstructionErrorException">The form is <c>gp_Other</c>.</exception>
    public readonly Transform3D ToTransform()
    {
        if (Form == Other)
        {
            throw new ConstructionErrorException("gp_GTrsf::Trsf() - non-orthogonal GTrsf");
        }

        return new Transform3D(_scale, _shape, _matrix, _loc);
    }

    /// <summary>The 4x4 matrix, row by row, as OCCT's <c>GetMat4</c>.</summary>
    public readonly void GetMat4(Span<double> mat)
    {
        if (mat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(mat));
        }

        if (_shape == Identity)
        {
            mat[..16].Clear();
            mat[0] = mat[5] = mat[10] = mat[15] = 1.0;
            return;
        }

        for (int row = 1; row <= 3; row++)
        {
            for (int col = 1; col <= 4; col++)
            {
                mat[(row - 1) * 4 + (col - 1)] = Value(row, col);
            }
        }

        mat[12] = 0.0;
        mat[13] = 0.0;
        mat[14] = 0.0;
        mat[15] = 1.0;
    }

    public static GeneralTransform3D operator *(in GeneralTransform3D left, in GeneralTransform3D right) => left.Multiplied(right);

    public readonly bool Equals(GeneralTransform3D other)
        => _matrix.Equals(other._matrix) && _loc.Equals(other._loc) && _shape == other._shape && _scale.Equals(other._scale);

    public override readonly bool Equals(object? obj) => obj is GeneralTransform3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_matrix, _loc, _shape, _scale);

    public static bool operator ==(in GeneralTransform3D left, in GeneralTransform3D right) => left.Equals(right);

    public static bool operator !=(in GeneralTransform3D left, in GeneralTransform3D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_GTrsf({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
