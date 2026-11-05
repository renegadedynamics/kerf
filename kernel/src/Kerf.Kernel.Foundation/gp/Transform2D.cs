// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>
/// A non-persistent transformation in 2D space: a rigid motion, mirror or uniform scale, or a combination.
/// It maps P to <c>scale * matrix * P + loc</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Trsf2d()</c> is the identity, as in OCCT; <c>default(gp_Trsf2d)</c> is not valid. Build one with the
/// static factories (<see cref="Rotation(in Point2D, double)"/>, <see cref="Translation(in Vector2D)"/>, ...) or the
/// <c>With</c> methods, which mirror OCCT's <c>Set</c> methods.
/// </remarks>
public readonly partial struct Transform2D : IEquatable<Transform2D>
{
    internal readonly double _scale;
    internal readonly TransformForm _shape;
    internal readonly Matrix2x2 _matrix;
    internal readonly Xy _loc;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    internal Transform2D(double scale, TransformForm shape, in Matrix2x2 matrix, in Xy loc)
    {
        _scale = scale;
        _shape = shape;
        _matrix = matrix;
        _loc = loc;
    }

    /// <summary>The fields, for the returning forms that take a whole value back (unit 10G).</summary>
    private void Deconstruct(out double scale, out TransformForm shape, out Matrix2x2 matrix, out Xy loc)
    {
        scale = _scale;
        shape = _shape;
        matrix = _matrix;
        loc = _loc;
    }

    public Transform2D()
    {
        _scale = 1.0;
        _shape = TransformForm.Identity;
        _matrix = new Matrix2x2();
        _loc = new Xy(0.0, 0.0);
        _matrix.SetIdentity();
    }

    /// <summary>The 2D part of a 3D transformation: its translation and the upper-left 2x2 of its matrix.</summary>
    public Transform2D(in Transform3D t)
    {
        _scale = t.ScaleFactor;
        _shape = t.Form;
        _matrix = new Matrix2x2();
        _loc = new Xy(t.TranslationXyz().X(), t.TranslationXyz().Y());
        Matrix3x3 m = t.RotationPart;
        _matrix.ChangeValue(1, 1) = m[1, 1];
        _matrix.ChangeValue(1, 2) = m[1, 2];
        _matrix.ChangeValue(2, 1) = m[2, 1];
        _matrix.ChangeValue(2, 2) = m[2, 2];
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The central symmetry about a point.</summary>
    public static Transform2D Mirror(in Point2D p) => new Transform2D().WithMirror(p);

    /// <summary>The reflection in an axis.</summary>
    public static Transform2D Mirror(in Axis2D a) => new Transform2D().WithMirror(a);

    public static Transform2D Rotation(in Point2D p, double ang) => new Transform2D().WithRotation(p, ang);

    public static Transform2D Scale(in Point2D p, double s) => new Transform2D().WithScale(p, s);

    public static Transform2D Transformation(in Axis2D fromSystem1, in Axis2D toSystem2)
        => new Transform2D().WithTransformation(fromSystem1, toSystem2);

    public static Transform2D Transformation(in Axis2D toSystem) => new Transform2D().WithTransformation(toSystem);

    public static Transform2D Translation(in Vector2D v) => new Transform2D().WithTranslation(v);

    public static Transform2D Translation(in Point2D p1, in Point2D p2) => new Transform2D().WithTranslation(p1, p2);

    public static Transform2D Values(double a11, double a12, double a13, double a21, double a22, double a23)
        => new Transform2D().WithValues(a11, a12, a13, a21, a22, a23);

    // Copies

    public readonly Transform2D WithMirror(in Point2D p)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = PntMirror;
        scale = -1.0;
        matrix.SetIdentity();
        loc = p.XY();
        loc.Multiply(2.0);
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithMirror(in Axis2D a)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = Ax1Mirror;
        scale = -1.0;
        Direction2D v = a.Direction;
        Point2D p = a.Location;
        double vx = v.X;
        double vy = v.Y;
        double x0 = p.X;
        double y0 = p.Y;
        matrix.SetCol(1, new Xy(1.0 - 2.0 * vx * vx, -2.0 * vx * vy));
        matrix.SetCol(2, new Xy(-2.0 * vx * vy, 1.0 - 2.0 * vy * vy));

        loc.SetCoord(-2.0 * ((vx * vx - 1.0) * x0 + (vx * vy * y0)),
                     -2.0 * ((vx * vy * x0) + (vy * vy - 1.0) * y0));
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithRotation(in Point2D p, double ang)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Rotation;
        scale = 1.0;
        loc = p.XY();
        loc.Reverse();
        matrix.SetRotation(ang);
        loc.Multiply(matrix);
        loc.Add(p.XY());
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithScale(in Point2D p, double s)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Scale;
        scale = s;
        matrix.SetIdentity();
        loc = p.XY();
        loc.Multiply(1.0 - s);
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithTransformation(in Axis2D fromA1, in Axis2D toA2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 newMatrix = _matrix;
        Xy loc = _loc;
        shape = CompoundTrsf;
        scale = 1.0;
        // matrix from XOY to ToA2
        Xy v1 = toA2.Direction.XY();
        Xy v2 = new(-v1.Y(), v1.X());
        newMatrix.SetCol(1, v1);
        newMatrix.SetCol(2, v2);
        loc = toA2.Location.XY();
        newMatrix.Transpose();
        loc.Multiply(newMatrix);
        loc.Reverse();
        // matrix FromA1 to XOY
        Xy v3 = fromA1.Direction.XY();
        Xy v4 = new(-v3.Y(), v3.X());
        Matrix2x2 ma1 = new(v3, v4);
        Xy ma1loc = fromA1.Location.XY();
        // matrix * MA1 => FromA1 ToA2
        ma1loc.Multiply(newMatrix);
        loc.Add(ma1loc);
        newMatrix.Multiply(ma1);
        return new Transform2D(scale, shape, newMatrix, loc);
    }

    public readonly Transform2D WithTransformation(in Axis2D a)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = CompoundTrsf;
        scale = 1.0;
        Xy v1 = a.Direction.XY();
        Xy v2 = new(-v1.Y(), v1.X());
        matrix.SetCol(1, v1);
        matrix.SetCol(2, v2);
        loc = a.Location.XY();
        matrix.Transpose();
        loc.Multiply(matrix);
        loc.Reverse();
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithTranslation(in Vector2D v)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Translation;
        scale = 1.0;
        matrix.SetIdentity();
        loc = v.XY();
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithTranslation(in Point2D p1, in Point2D p2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Translation;
        scale = 1.0;
        matrix.SetIdentity();
        loc = (p2.XY()).Subtracted(p1.XY());
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D WithTranslationPart(in Vector2D v)
    {
        TransformForm shape = _shape;
        Xy loc = _loc;
        loc = v.XY();
        if (Math.Abs(loc.X()) <= Constants.Resolution() && Math.Abs(loc.Y()) <= Constants.Resolution())
        {
            if (shape == TransformForm.Identity || shape == PntMirror || shape == global::Kerf.Geometry.TransformForm.Scale || shape == global::Kerf.Geometry.TransformForm.Rotation
                || shape == Ax1Mirror)
            {
            }
            else if (shape == global::Kerf.Geometry.TransformForm.Translation)
            {
                shape = TransformForm.Identity;
            }
            else
            {
                shape = CompoundTrsf;
            }
        }
        else
        {
            if (shape == global::Kerf.Geometry.TransformForm.Translation || shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror)
            {
            }
            else if (shape == TransformForm.Identity)
            {
                shape = global::Kerf.Geometry.TransformForm.Translation;
            }
            else
            {
                shape = CompoundTrsf;
            }
        }
        return new Transform2D(_scale, shape, _matrix, loc);
    }

    public readonly Transform2D WithScaleFactor(double s)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        if (s == 1.0)
        {
            double x = _loc.X();
            if (x < 0)
            {
                x = -x;
            }

            double y = _loc.Y();
            if (y < 0)
            {
                y = -y;
            }

            if (x <= Constants.Resolution() && y <= Constants.Resolution())
            {
                if (shape == TransformForm.Identity || shape == global::Kerf.Geometry.TransformForm.Rotation)
                {
                }
                else if (shape == global::Kerf.Geometry.TransformForm.Scale)
                {
                    shape = TransformForm.Identity;
                }
                else if (shape == PntMirror)
                {
                    shape = global::Kerf.Geometry.TransformForm.Translation;
                }
                else
                {
                    shape = CompoundTrsf;
                }
            }
            else
            {
                if (shape == TransformForm.Identity || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == global::Kerf.Geometry.TransformForm.Scale)
                {
                }
                else if (shape == PntMirror)
                {
                    shape = global::Kerf.Geometry.TransformForm.Translation;
                }
                else
                {
                    shape = CompoundTrsf;
                }
            }
        }
        else if (s == -1)
        {
            if (shape == PntMirror || shape == Ax1Mirror)
            {
            }
            else if (shape == TransformForm.Identity || shape == global::Kerf.Geometry.TransformForm.Scale)
            {
                shape = PntMirror;
            }
            else
            {
                shape = CompoundTrsf;
            }
        }
        else
        {
            if (shape == global::Kerf.Geometry.TransformForm.Scale)
            {
            }
            else if (shape == TransformForm.Identity || shape == global::Kerf.Geometry.TransformForm.Translation || shape == PntMirror)
            {
                shape = global::Kerf.Geometry.TransformForm.Scale;
            }
            else
            {
                shape = CompoundTrsf;
            }
        }

        scale = s;
        return new Transform2D(scale, shape, _matrix, _loc);
    }

    public readonly Transform2D WithValues(double a11, double a12, double a13, double a21, double a22, double a23)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        Xy col1 = new(a11, a21);
        Xy col2 = new(a12, a22);
        Xy col3 = new(a13, a23);
        // compute the determinant
        Matrix2x2 m = new(col1, col2);
        double s = m.Determinant();
        Refusal.ArgumentIf(Math.Abs(s) < Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf2d::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Sqrt(s);
        }
        else
        {
            s = Math.Sqrt(-s);
        }

        m.Divide(s);

        scale = s;
        shape = CompoundTrsf;

        matrix = m;
        try
        {
            matrix = Orthogonalized(matrix);
        }
        catch (FailureException failure)
        {
            throw Refusal.Argument(failure);
        }

        loc = col3;
        return new Transform2D(scale, shape, matrix, loc);
    }

    /// <summary>The matrix made orthogonal again (OCCT's <c>Orthogonalize</c>, which changed the transformation in place).</summary>
    private static Matrix2x2 Orthogonalized(Matrix2x2 tm)
    {
        Xy v1 = tm.Column(1);
        Xy v2 = tm.Column(2);

        v1.Normalize();

        v2 -= v1 * (v2.Dot(v1));
        v2.Normalize();

        tm.SetCols(v1, v2);

        v1 = tm.Row(1);
        v2 = tm.Row(2);

        v1.Normalize();

        v2 -= v1 * (v2.Dot(v1));
        v2.Normalize();

        tm.SetRows(v1, v2);
        return tm;
    }

    // Queries

    /// <summary>True when the determinant of the matrix is negative (the transformation mirrors).</summary>
    public readonly bool IsNegative => (_matrix.Determinant() < 0.0);

    public readonly TransformForm Form => _shape;

    public readonly double ScaleFactor => _scale;

    internal readonly Xy TranslationXy() => _loc;

    /// <summary>The matrix multiplied by the scale factor.</summary>
    internal readonly Matrix2x2 VectorialPart
    {
        get
        {
            if (_scale == 1.0)
            {
                return _matrix;
            }

            Matrix2x2 m = _matrix;
            if (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror)
            {
                m.SetDiagonal(_matrix.Value(1, 1) * _scale, _matrix.Value(2, 2) * _scale);
            }
            else
            {
                m.Multiply(_scale);
            }

            return m;
        }
    }

    /// <summary>The matrix without the scale factor.</summary>
    internal readonly Matrix2x2 RotationPart => _matrix;

    /// <summary>The angle of the rotation part, in ]-PI, PI].</summary>
    public readonly double RotationAngle => Math.Atan2(_matrix.Value(2, 1), _matrix.Value(1, 1));

    /// <summary>The coefficient at a row (1 to 2) and column (1 to 3) of the 2x3 matrix, scale included.</summary>
    internal readonly double Value(int row, int col)
    {
        Refusal.RangeIf(row < 1 || row > 2 || col < 1 || col > 3, FailureKind.OutOfRange, " ");
        if (col < 3)
        {
            return _scale * _matrix.Value(row, col);
        }
        else
        {
            return _loc.Coord(row);
        }
    }

    public readonly Transform2D Inverted()
    {
        double newScale = _scale;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        //                                    -1
        //  X' = scale * R * X + T  =>  X = (R  / scale)  * ( X' - T)
        if (_shape == TransformForm.Identity)
        {
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation || _shape == PntMirror)
        {
            loc.Reverse();
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            Refusal.StateIf(Math.Abs(newScale) <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf2d::Invert() - transformation has zero scale");
            newScale = 1.0 / newScale;
            loc.Multiply(-newScale);
        }
        else
        {
            Refusal.StateIf(Math.Abs(newScale) <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf2d::Invert() - transformation has zero scale");
            newScale = 1.0 / newScale;
            matrix.Transpose();
            loc.Multiply(matrix);
            loc.Multiply(-newScale);
        }
        return new Transform2D(newScale, _shape, matrix, loc);
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly Transform2D Multiplied(in Transform2D t)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy newLoc = _loc;
        if (t._shape == TransformForm.Identity)
        {
        }
        else if (shape == TransformForm.Identity)
        {
            shape = t._shape;
            scale = t._scale;
            newLoc = t._loc;
            matrix = t._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Rotation && t._shape == global::Kerf.Geometry.TransformForm.Rotation)
        {
            if (t._loc.X() != 0.0 || t._loc.Y() != 0.0)
            {
                newLoc.Add(t._loc.Multiplied(matrix));
            }

            matrix.Multiply(t._matrix);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            newLoc.Add(t._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Scale && t._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            newLoc.Add(t._loc.Multiplied(scale));
            scale = scale * t._scale;
        }
        else if (shape == PntMirror && t._shape == PntMirror)
        {
            scale = 1.0;
            shape = global::Kerf.Geometry.TransformForm.Translation;
            newLoc.Add(t._loc.Reversed());
        }
        else if (shape == Ax1Mirror && t._shape == Ax1Mirror)
        {
            shape = global::Kerf.Geometry.TransformForm.Rotation;
            Xy tloc = t._loc;
            tloc.Multiply(matrix);
            tloc.Multiply(scale);
            scale = scale * t._scale;
            newLoc.Add(tloc);
            matrix.Multiply(t._matrix);
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xy tloc = t._loc;
            tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                tloc.Multiply(scale);
            }

            newLoc.Add(tloc);
        }
        else if ((shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xy tloc = t._loc;
            tloc.Multiply(scale);
            newLoc.Add(tloc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror))
        {
            shape = CompoundTrsf;
            scale = t._scale;
            newLoc.Add(t._loc);
            matrix = t._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            shape = t._shape;
            newLoc.Add(t._loc);
            scale = t._scale;
        }
        else if ((shape == PntMirror || shape == global::Kerf.Geometry.TransformForm.Scale)
                 && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(scale);
            newLoc.Add(tloc);
            scale = scale * t._scale;
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(matrix);
            if (scale == 1.0)
            {
                scale = t._scale;
            }
            else
            {
                tloc.Multiply(scale);
                scale = scale * t._scale;
            }

            newLoc.Add(tloc);
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror)
                 && (shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror))
        {
            shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(scale);
            scale = scale * t._scale;
            newLoc.Add(tloc);
            matrix = t._matrix;
        }
        else
        {
            shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                tloc.Multiply(scale);
                scale = scale * t._scale;
            }
            else
            {
                scale = t._scale;
            }

            newLoc.Add(tloc);
            matrix.Multiply(t._matrix);
        }
        return new Transform2D(scale, shape, matrix, newLoc);
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly Transform2D PreMultiplied(in Transform2D t)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        if (t._shape == TransformForm.Identity)
        {
        }
        else if (shape == TransformForm.Identity)
        {
            shape = t._shape;
            scale = t._scale;
            loc = t._loc;
            matrix = t._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Rotation && t._shape == global::Kerf.Geometry.TransformForm.Rotation)
        {
            loc.Multiply(t._matrix);
            loc.Add(t._loc);
            matrix.PreMultiply(t._matrix);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(t._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Scale && t._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            loc.Multiply(t._scale);
            loc.Add(t._loc);
            scale = scale * t._scale;
        }
        else if (shape == PntMirror && t._shape == PntMirror)
        {
            scale = 1.0;
            shape = global::Kerf.Geometry.TransformForm.Translation;
            loc.Reverse();
            loc.Add(t._loc);
        }
        else if (shape == Ax1Mirror && t._shape == Ax1Mirror)
        {
            shape = global::Kerf.Geometry.TransformForm.Rotation;
            loc.Multiply(t._matrix);
            loc.Multiply(t._scale);
            scale = scale * t._scale;
            loc.Add(t._loc);
            matrix.PreMultiply(t._matrix);
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(t._loc);
        }
        else if ((shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(t._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror))
        {
            shape = CompoundTrsf;
            matrix = t._matrix;
            if (t._scale == 1.0)
            {
                loc.Multiply(t._matrix);
            }
            else
            {
                scale = t._scale;
                loc.Multiply(matrix);
                loc.Multiply(scale);
            }

            loc.Add(t._loc);
        }
        else if ((t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror) && shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Multiply(t._scale);
            loc.Add(t._loc);
            scale = t._scale;
            shape = t._shape;
        }
        else if ((shape == PntMirror || shape == global::Kerf.Geometry.TransformForm.Scale)
                 && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            shape = CompoundTrsf;
            loc.Multiply(t._scale);
            loc.Add(t._loc);
            scale = scale * t._scale;
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            shape = CompoundTrsf;
            loc.Multiply(t._scale);
            loc.Add(t._loc);
            scale = scale * t._scale;
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror)
                 && (shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror))
        {
            shape = CompoundTrsf;
            matrix = t._matrix;
            if (t._scale == 1.0)
            {
                loc.Multiply(t._matrix);
            }
            else
            {
                loc.Multiply(matrix);
                loc.Multiply(t._scale);
                scale = t._scale * scale;
            }

            loc.Add(t._loc);
        }
        else
        {
            shape = CompoundTrsf;
            loc.Multiply(t._matrix);
            if (t._scale != 1.0)
            {
                loc.Multiply(t._scale);
                scale = scale * t._scale;
            }

            loc.Add(t._loc);
            matrix.PreMultiply(t._matrix);
        }
        return new Transform2D(scale, shape, matrix, loc);
    }

    public readonly Transform2D Powered(int n)
    {
        double newScale = _scale;
        TransformForm shape = _shape;
        Matrix2x2 matrix = _matrix;
        Xy loc = _loc;
        if (shape == TransformForm.Identity)
        {
        }
        else
        {
            if (n == 0)
            {
                newScale = 1.0;
                shape = TransformForm.Identity;
                matrix.SetIdentity();
                loc = new Xy(0.0, 0.0);
            }
            else if (n == 1)
            {
            }
            else if (n == -1)
            {
                (newScale, shape, matrix, loc) = new Transform2D(newScale, shape, matrix, loc).Inverted();
            }
            else
            {
                if (n < 0)
                {
                    (newScale, shape, matrix, loc) = new Transform2D(newScale, shape, matrix, loc).Inverted();
                }

                if (shape == global::Kerf.Geometry.TransformForm.Translation)
                {
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Xy temploc = loc;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            loc.Add(temploc);
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        temploc.Add(temploc);
                        npower = npower / 2;
                    }
                }
                else if (shape == global::Kerf.Geometry.TransformForm.Scale)
                {
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Xy temploc = loc;
                    double tempscale = newScale;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            loc.Add(temploc.Multiplied(newScale));
                            newScale = newScale * tempscale;
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        temploc.Add(temploc.Multiplied(tempscale));
                        tempscale = tempscale * tempscale;
                        npower = npower / 2;
                    }
                }
                else if (shape == global::Kerf.Geometry.TransformForm.Rotation)
                {
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Matrix2x2 tempmatrix = matrix;
                    if (loc.X() == 0.0 && loc.Y() == 0.0)
                    {
                        for (;;)
                        {
                            if (IsOdd(npower))
                            {
                                matrix.Multiply(tempmatrix);
                            }

                            if (npower == 1)
                            {
                                break;
                            }

                            tempmatrix.Multiply(tempmatrix);
                            npower = npower / 2;
                        }
                    }
                    else
                    {
                        Xy temploc = loc;
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
                }
                else if (shape == PntMirror || shape == Ax1Mirror)
                {
                    if (IsEven(n))
                    {
                        shape = TransformForm.Identity;
                        newScale = 1.0;
                        matrix.SetIdentity();
                        loc.SetCoord(0.0, 0.0);
                    }
                }
                else
                {
                    shape = CompoundTrsf;
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Xy temploc = loc;
                    double tempscale = newScale;
                    Matrix2x2 tempmatrix = matrix;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            loc.Add((temploc.Multiplied(matrix)).Multiplied(newScale));
                            newScale = newScale * tempscale;
                            matrix.Multiply(tempmatrix);
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        // the square's translation takes the scale before it is squared (decision 1485)
                        temploc.Add((temploc.Multiplied(tempmatrix)).Multiplied(tempscale));
                        tempscale = tempscale * tempscale;
                        tempmatrix.Multiply(tempmatrix);
                        npower = npower / 2;
                    }
                }
            }
        }
        return new Transform2D(newScale, shape, matrix, loc);
    }

    internal readonly void Transforms(ref double x, ref double y)
    {
        Xy doublet = new(x, y);
        doublet.Multiply(_matrix);
        if (_scale != 1.0)
        {
            doublet.Multiply(_scale);
        }

        doublet.Add(_loc);
        doublet.Coord(out x, out y);
    }

    internal readonly void Transforms(ref Xy coord)
    {
        coord.Multiply(_matrix);
        if (_scale != 1.0)
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref Xy)"/>.</summary>
    internal readonly Xy Transformed(in Xy coord)
    {
        Xy aCoord = coord;
        Transforms(ref aCoord);
        return aCoord;
    }

    public static Transform2D operator *(in Transform2D left, in Transform2D right) => left.Multiplied(right);

    public readonly bool Equals(Transform2D other)
        => _scale.Equals(other._scale) && _shape == other._shape && _matrix.Equals(other._matrix) && _loc.Equals(other._loc);

    public override readonly bool Equals(object? obj) => obj is Transform2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_scale, _shape, _matrix, _loc);

    public static bool operator ==(in Transform2D left, in Transform2D right) => left.Equals(right);

    public static bool operator !=(in Transform2D left, in Transform2D right) => !left.Equals(right);

    public override readonly string ToString() => $"Transform2D({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
