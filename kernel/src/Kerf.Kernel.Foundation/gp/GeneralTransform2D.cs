// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>
/// A general (affine) transformation in 2D space: <c>matrix * P + loc</c>, where the matrix need not be
/// orthogonal. Its form is gp_Other unless it was built from a <see cref="Transform2D"/>.
/// </summary>
/// <remarks><c>new gp_GTrsf2d()</c> is the identity, as in OCCT; <c>default(gp_GTrsf2d)</c> is not valid.</remarks>
public partial struct GeneralTransform2D : IEquatable<GeneralTransform2D>
{
    private Matrix2x2 _matrix;
    private Xy _loc;
    private TransformForm _shape;
    private double _scale;

    public GeneralTransform2D()
    {
        _matrix = new Matrix2x2();
        _loc = new Xy(0.0, 0.0);
        _shape = Identity;
        _scale = 1.0;
        _matrix.SetScale(1.0);
    }

    public GeneralTransform2D(in Transform2D t)
    {
        _shape = t._shape;
        _matrix = t._matrix;
        _loc = t._loc;
        _scale = t._scale;
    }

    /// <summary>The transformation <c>theM * P + theV</c>.</summary>
    public GeneralTransform2D(in Matrix2x2 m, in Xy v)
    {
        _matrix = m;
        _loc = v;
        _shape = Other;
        _scale = 0.0;
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The affinity of ratio <paramref name="ratio"/> along the normal to the axis <paramref name="a"/>.</summary>
    public static GeneralTransform2D Affinity(in Axis2D a, double ratio) => new GeneralTransform2D().WithAffinity(a, ratio);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAffinity(in Axis2D A, double ratio)
    {
        _shape = Other;
        _scale = 0.0;
        double a = A.Direction().X();
        double b = A.Direction().Y();
        _matrix.SetValue(1, 1, (1.0 - ratio) * a * a + ratio);
        _matrix.SetValue(2, 2, (1.0 - ratio) * b * b + ratio);
        _matrix.SetValue(1, 2, (1.0 - ratio) * a * b);
        _matrix.SetValue(2, 1, _matrix.Value(1, 2));
        _loc = A.Location().XY();
        _loc.Reverse();
        _loc.Multiply(_matrix);
        _loc.Add(A.Location().XY());
    }

    internal void SetValue(int row, int col, double value)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2 || col < 1 || col > 3, " ");
        if (col == 3)
        {
            _loc.SetCoord(row, value);
        }
        else
        {
            _matrix.SetValue(row, col, value);
        }

        _shape = Other;
    }

    internal void SetTranslationPart(in Xy coord)
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

    internal void SetTrsf2d(in Transform2D t)
    {
        _shape = t._shape;
        _matrix = t._matrix;
        _loc = t._loc;
        _scale = t._scale;
    }

    internal void SetVectorialPart(in Matrix2x2 matrix)
    {
        _matrix = matrix;
        _shape = Other;
        _scale = 0.0;
    }

    internal void Invert()
    {
        if (_shape == Other)
        {
            _matrix.Invert();
            _loc.Multiply(_matrix);
            _loc.Reverse();
        }
        else
        {
            Transform2D t = Trsf2d();
            t = t.Inverted();
            SetTrsf2d(t);
        }
    }

    internal void Multiply(in GeneralTransform2D t)
    {
        if (Form() == Other || t.Form() == Other)
        {
            _shape = Other;
            _loc.Add(t._loc.Multiplied(_matrix));
            _matrix.Multiply(t._matrix);
        }
        else
        {
            Transform2D t1 = Trsf2d();
            Transform2D t2 = t.Trsf2d();
            t1 = t1.Multiplied(t2);
            _matrix = t1._matrix;
            _loc = t1._loc;
            _scale = t1._scale;
            _shape = t1._shape;
        }
    }

    internal void PreMultiply(in GeneralTransform2D t)
    {
        if (Form() == Other || t.Form() == Other)
        {
            _shape = Other;
            _loc.Multiply(t._matrix);
            _loc.Add(t._loc);
            _matrix.PreMultiply(t._matrix);
        }
        else
        {
            Transform2D t1 = Trsf2d();
            Transform2D t2 = t.Trsf2d();
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
            _matrix.SetIdentity();
            _loc = new Xy(0.0, 0.0);
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
            if (n < 0)
            {
                Invert();
            }

            if (_shape == Other)
            {
                int npower = Math.Abs(n) - 1;
                Xy temploc = _loc;
                Matrix2x2 tempmatrix = _matrix;
                for (;;)
                {
                    if (IsOdd(npower))
                    {
                        _loc.Add(temploc.Multiplied(_matrix));
                        _matrix.Multiply(tempmatrix);
                    }

                    if (npower == 1)
                    {
                        break;
                    }

                    temploc.Add(temploc.Multiplied(tempmatrix));
                    // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
                    tempmatrix.Multiply(tempmatrix);
                    npower = npower / 2;
                }
            }
            else
            {
                Transform2D t = Trsf2d();
                t = t.Powered(n);
                SetTrsf2d(t);
            }
        }
    }

    // Copies with a mutator applied

    public readonly GeneralTransform2D WithAffinity(in Axis2D a, double ratio)
    {
        GeneralTransform2D copy = this;
        copy.SetAffinity(a, ratio);
        return copy;
    }

    public readonly GeneralTransform2D WithValue(int row, int col, double value)
    {
        GeneralTransform2D copy = this;
        copy.SetValue(row, col, value);
        return copy;
    }

    public readonly GeneralTransform2D WithTranslationPart(in Xy coord)
    {
        GeneralTransform2D copy = this;
        copy.SetTranslationPart(coord);
        return copy;
    }

    public readonly GeneralTransform2D WithTrsf2d(in Transform2D t) => new(t);

    public readonly GeneralTransform2D WithVectorialPart(in Matrix2x2 matrix)
    {
        GeneralTransform2D copy = this;
        copy.SetVectorialPart(matrix);
        return copy;
    }

    // Queries

    public readonly bool IsNegative => _matrix.Determinant() < 0.0;

    public readonly bool IsSingular => _matrix.IsSingular();

    public readonly TransformForm Form() => _shape;

    public readonly Xy TranslationPart() => _loc;

    public readonly Matrix2x2 VectorialPart() => _matrix;

    /// <summary>The coefficient at a row (1 to 2) and column (1 to 3) of the 2x3 matrix.</summary>
    public readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2 || col < 1 || col > 3, " ");
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

    public readonly GeneralTransform2D Inverted()
    {
        GeneralTransform2D t = this;
        t.Invert();
        return t;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly GeneralTransform2D Multiplied(in GeneralTransform2D t)
    {
        GeneralTransform2D tres = this;
        tres.Multiply(t);
        return tres;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly GeneralTransform2D PreMultiplied(in GeneralTransform2D t)
    {
        GeneralTransform2D tres = this;
        tres.PreMultiply(t);
        return tres;
    }

    public readonly GeneralTransform2D Powered(int n)
    {
        GeneralTransform2D t = this;
        t.Power(n);
        return t;
    }

    public readonly void Transforms(ref Xy coord)
    {
        coord.Multiply(_matrix);
        if (!(_shape == Other) && !(_scale == 1.0))
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    public readonly Xy Transformed(in Xy coord)
    {
        Xy newCoord = coord;
        Transforms(ref newCoord);
        return newCoord;
    }

    public readonly void Transforms(ref double x, ref double y)
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
    public readonly Transform2D Trsf2d()
    {
        // Test of orthogonality
        double tolerance = global::Kerf.Engine.Foundation.Precision.Angular();
        double tolerance2 = 2.0 * tolerance;

        if (Form() == Other)
        {
            throw new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(0)");
        }

        double value = (_matrix.Value(1, 1) * _matrix.Value(1, 1) + _matrix.Value(2, 1) * _matrix.Value(2, 1));
        if (Math.Abs(value - 1.0) > tolerance2)
        {
            throw new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(1)");
        }

        value = (_matrix.Value(1, 2) * _matrix.Value(1, 2) + _matrix.Value(2, 2) * _matrix.Value(2, 2));
        if (Math.Abs(value - 1.0) > tolerance2)
        {
            throw new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(2)");
        }

        value = (_matrix.Value(1, 1) * _matrix.Value(1, 2) + _matrix.Value(2, 1) * _matrix.Value(2, 2));
        if (Math.Abs(value) > tolerance)
        {
            throw new ConstructionErrorException("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(3)");
        }

        Transform2D transformation = new();
        transformation._matrix = _matrix;
        transformation._shape = _shape;
        transformation._scale = _scale;
        transformation._loc = _loc;
        return transformation;
    }

    public static GeneralTransform2D operator *(in GeneralTransform2D left, in GeneralTransform2D right) => left.Multiplied(right);

    public readonly bool Equals(GeneralTransform2D other)
        => _matrix.Equals(other._matrix) && _loc.Equals(other._loc) && _shape == other._shape && _scale.Equals(other._scale);

    public override readonly bool Equals(object? obj) => obj is GeneralTransform2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_matrix, _loc, _shape, _scale);

    public static bool operator ==(in GeneralTransform2D left, in GeneralTransform2D right) => left.Equals(right);

    public static bool operator !=(in GeneralTransform2D left, in GeneralTransform2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_GTrsf2d({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
