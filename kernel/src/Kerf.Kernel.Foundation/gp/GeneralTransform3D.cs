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
public partial struct GeneralTransform3D : IEquatable<GeneralTransform3D>
{
    private Matrix3x3 _matrix;
    private Xyz _loc;
    private TransformForm _shape;
    private double _scale;

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
        _shape = t.Form();
        _matrix = t._matrix;
        _loc = t.TranslationPart();
        _scale = t.ScaleFactor();
    }

    /// <summary>The transformation with vectorial part <paramref name="m"/> and translation part <paramref name="v"/>.</summary>
    public GeneralTransform3D(in Matrix3x3 m, in Xyz v)
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

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAffinity(in Axis3D a1, double ratio)
    {
        _shape = Other;
        _scale = 0.0;
        _matrix = _matrix.WithDot(a1.Direction().XYZ());
        _matrix = _matrix.Multiplied(1.0 - ratio);
        _matrix = _matrix.WithDiagonal(_matrix.Value(1, 1) + ratio,
                           _matrix.Value(2, 2) + ratio,
                           _matrix.Value(3, 3) + ratio);
        _loc = a1.Location().XYZ();
        _loc.Reverse();
        _loc.Multiply(_matrix);
        _loc.Add(a1.Location().XYZ());
    }

    internal void SetAffinity(in Frame3D a2, double ratio)
    {
        _shape = Other;
        _scale = 0.0;
        _matrix = _matrix.WithDot(a2.Direction().XYZ());
        _matrix = _matrix.Multiplied(ratio - 1.0);
        _loc = a2.Location().XYZ();
        _loc.Reverse();
        _loc.Multiply(_matrix);
        _matrix = _matrix.WithDiagonal(_matrix.Value(1, 1) + 1.0, _matrix.Value(2, 2) + 1.0, _matrix.Value(3, 3) + 1.0);
    }

    internal void SetValue(int row, int col, double value)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 4, " ");
        if (col == 4)
        {
            _loc.SetCoord(row, value);
            if (_shape == Identity)
            {
                _shape = Translation;
            }

            return;
        }
        else
        {
            if (!(_shape == Other) && !(_scale == 1.0))
            {
                _matrix = _matrix.Multiplied(_scale);
            }

            _matrix = _matrix.WithValue(row, col, value);
            _shape = Other;
            _scale = 0.0;
            return;
        }
    }

    internal void SetVectorialPart(in Matrix3x3 matrix)
    {
        _matrix = matrix;
        _shape = Other;
        _scale = 0.0;
    }

    internal void SetTranslationPart(in Xyz coord)
    {
        _loc = coord;
        if (Form() == CompoundTrsf || Form() == Other || Form() == Translation)
        {
        }
        else if (Form() == Identity)
        {
            _shape = Translation;
        }
        else
        {
            _shape = CompoundTrsf;
        }
    }

    internal void SetTrsf(in Transform3D t)
    {
        _shape = t._shape;
        _matrix = t._matrix;
        _loc = t._loc;
        _scale = t._scale;
    }

    /// <summary>Recomputes the form from the matrix: <c>gp_Other</c> unless the matrix is a uniform scale of an orthogonal matrix.</summary>
    internal void SetForm()
    {
        double tol = 1.0e-12; // Precision::Angular();
        //
        // don t trust the initial values !
        //
        Matrix3x3 m = _matrix;
        double s = m.Determinant();

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
        Matrix3x3 identity = new();
        identity = identity.WithIdentity();
        tm = tm.Subtracted(identity);
        if (_shape == Other)
        {
            _shape = CompoundTrsf;
        }

        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                if (Math.Abs(tm.Value(i, j)) > tol)
                {
                    _shape = Other;
                    return;
                }
            }
        }
    }

    internal void Invert()
    {
        if (_shape == Other)
        {
            _matrix = _matrix.Inverted();
            _loc.Multiply(_matrix);
            _loc.Reverse();
        }
        else
        {
            Transform3D t = Trsf();
            t = t.Inverted();
            SetTrsf(t);
        }
    }

    /// <summary>this = this * T, which applies T first.</summary>
    internal void Multiply(in GeneralTransform3D t)
    {
        if (Form() == Other || t.Form() == Other)
        {
            _shape = Other;
            _loc.Add(t._loc.Multiplied(_matrix));
            _matrix = _matrix.Multiplied(t._matrix);
        }
        else
        {
            Transform3D t1 = Trsf();
            Transform3D t2 = t.Trsf();
            t1 = t1.Multiplied(t2);
            _matrix = t1._matrix;
            _loc = t1._loc;
            _scale = t1._scale;
            _shape = t1._shape;
        }
    }

    /// <summary>this = T * this, which applies this transformation first.</summary>
    internal void PreMultiply(in GeneralTransform3D t)
    {
        if (Form() == Other || t.Form() == Other)
        {
            _shape = Other;
            _loc.Multiply(t._matrix);
            _loc.Add(t._loc);
            _matrix = _matrix.PreMultiplied(t._matrix);
        }
        else
        {
            Transform3D t1 = Trsf();
            Transform3D t2 = t.Trsf();
            t1 = t1.PreMultiplied(t2);
            _matrix = t1._matrix;
            _loc = t1._loc;
            _scale = t1._scale;
            _shape = t1._shape;
        }
    }

    internal void Power(int n)
    {
        if (n == 0)
        {
            _scale = 1.0;
            _shape = Identity;
            _matrix = _matrix.WithIdentity();
            _loc = new Xyz(0.0, 0.0, 0.0);
        }
        else if (n == 1)
        {
        }
        else if (n == -1)
        {
            Invert();
        }
        else
        {
            if (_shape == Other)
            {
                // OCCT: a negative N other than -1 is treated as |N|; the inverse is never taken.
                int npower = Math.Abs(n) - 1;
                Xyz temploc = _loc;
                Matrix3x3 tempmatrix = _matrix;
                for (;;)
                {
                    if (IsOdd(npower))
                    {
                        _loc.Add(temploc.Multiplied(_matrix));
                        _matrix = _matrix.Multiplied(tempmatrix);
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
                Transform3D t = Trsf();
                t = t.Powered(n);
                SetTrsf(t);
            }
        }
    }

    /// <summary>Sets the transformation from a 4x4 matrix given row by row (OCCT's <c>SetMat4</c> on an <c>NCollection_Mat4</c>).</summary>
    internal void SetMat4(ReadOnlySpan<double> mat)
    {
        if (mat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(mat));
        }

        _shape = Other;
        _scale = 0.0;
        _matrix = _matrix.WithValue(1, 1, mat[0]);
        _matrix = _matrix.WithValue(1, 2, mat[1]);
        _matrix = _matrix.WithValue(1, 3, mat[2]);
        _matrix = _matrix.WithValue(2, 1, mat[4]);
        _matrix = _matrix.WithValue(2, 2, mat[5]);
        _matrix = _matrix.WithValue(2, 3, mat[6]);
        _matrix = _matrix.WithValue(3, 1, mat[8]);
        _matrix = _matrix.WithValue(3, 2, mat[9]);
        _matrix = _matrix.WithValue(3, 3, mat[10]);
        _loc.SetCoord(mat[3], mat[7], mat[11]);
    }

    // Copies with a mutator applied

    public readonly GeneralTransform3D WithAffinity(in Axis3D a1, double ratio)
    {
        GeneralTransform3D copy = this;
        copy.SetAffinity(a1, ratio);
        return copy;
    }

    public readonly GeneralTransform3D WithAffinity(in Frame3D a2, double ratio)
    {
        GeneralTransform3D copy = this;
        copy.SetAffinity(a2, ratio);
        return copy;
    }

    /// <summary>A copy with one coefficient of the 3x4 matrix replaced (row 1 to 3, column 1 to 4).</summary>
    public readonly GeneralTransform3D WithValue(int row, int col, double value)
    {
        GeneralTransform3D copy = this;
        copy.SetValue(row, col, value);
        return copy;
    }

    public readonly GeneralTransform3D WithVectorialPart(in Matrix3x3 matrix)
    {
        GeneralTransform3D copy = this;
        copy.SetVectorialPart(matrix);
        return copy;
    }

    public readonly GeneralTransform3D WithTranslationPart(in Xyz coord)
    {
        GeneralTransform3D copy = this;
        copy.SetTranslationPart(coord);
        return copy;
    }

    public readonly GeneralTransform3D WithTrsf(in Transform3D t) => new(t);

    /// <summary>A copy whose form is recomputed from its matrix (OCCT's <c>SetForm</c>).</summary>
    public readonly GeneralTransform3D WithForm()
    {
        GeneralTransform3D copy = this;
        copy.SetForm();
        return copy;
    }

    public readonly GeneralTransform3D WithMat4(ReadOnlySpan<double> mat)
    {
        GeneralTransform3D copy = this;
        copy.SetMat4(mat);
        return copy;
    }

    // Queries

    /// <summary>True when the determinant of the vectorial part is negative.</summary>
    public readonly bool IsNegative => _matrix.Determinant() < 0.0;

    /// <summary>True when the vectorial part is singular, so the transformation cannot be inverted.</summary>
    public readonly bool IsSingular => _matrix.IsSingular();

    public readonly TransformForm Form() => _shape;

    public readonly Xyz TranslationPart() => _loc;

    /// <summary>The 3x3 vectorial part, without the scale factor of a non-<c>gp_Other</c> form.</summary>
    public readonly Matrix3x3 VectorialPart() => _matrix;

    /// <summary>The coefficient at a row (1 to 3) and column (1 to 4) of the 3x4 matrix, scale included.</summary>
    public readonly double Value(int row, int col)
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
        GeneralTransform3D t = this;
        t.Invert();
        return t;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly GeneralTransform3D Multiplied(in GeneralTransform3D t)
    {
        GeneralTransform3D tres = this;
        tres.Multiply(t);
        return tres;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly GeneralTransform3D PreMultiplied(in GeneralTransform3D t)
    {
        GeneralTransform3D tres = this;
        tres.PreMultiply(t);
        return tres;
    }

    public readonly GeneralTransform3D Powered(int n)
    {
        GeneralTransform3D t = this;
        t.Power(n);
        return t;
    }

    public readonly void Transforms(ref Xyz coord)
    {
        coord.Multiply(_matrix);
        if (!(_shape == Other) && !(_scale == 1.0))
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    public readonly void Transforms(ref double x, ref double y, ref double z)
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
    public readonly Xyz Transformed(in Xyz coord)
    {
        Xyz aCoord = coord;
        Transforms(ref aCoord);
        return aCoord;
    }

    /// <summary>This transformation as a <see cref="Transform3D"/>.</summary>
    /// <exception cref="ConstructionErrorException">The form is <c>gp_Other</c>.</exception>
    public readonly Transform3D Trsf()
    {
        if (Form() == Other)
        {
            throw new ConstructionErrorException("gp_GTrsf::Trsf() - non-orthogonal GTrsf");
        }

        Transform3D t = new();
        t._shape = _shape;
        t._scale = _scale;
        t._matrix = _matrix;
        t._loc = _loc;
        return t;
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
