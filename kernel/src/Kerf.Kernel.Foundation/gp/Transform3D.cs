// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>
/// A non-persistent transformation in 3D space: a rigid motion, mirror or uniform scale, or a combination.
/// It maps P to <c>scale * matrix * P + loc</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Trsf()</c> is the identity, as in OCCT; <c>default(gp_Trsf)</c> is not valid. Build one with the
/// static factories (<see cref="Rotation(in Axis3D, double)"/>, <see cref="Translation(in Vector3D)"/>, ...) or the
/// <c>With</c> methods, which mirror OCCT's <c>Set</c> methods.
/// </remarks>
public readonly partial struct Transform3D : IEquatable<Transform3D>
{
    internal readonly double _scale;
    internal readonly TransformForm _shape;
    internal readonly Matrix3x3 _matrix;
    internal readonly Xyz _loc;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    internal Transform3D(double scale, TransformForm shape, in Matrix3x3 matrix, in Xyz loc)
    {
        _scale = scale;
        _shape = shape;
        _matrix = matrix;
        _loc = loc;
    }

    /// <summary>The fields, for the returning forms that take a whole value back (unit 10G).</summary>
    private void Deconstruct(out double scale, out TransformForm shape, out Matrix3x3 matrix, out Xyz loc)
    {
        scale = _scale;
        shape = _shape;
        matrix = _matrix;
        loc = _loc;
    }

    public Transform3D()
    {
        _scale = 1.0;
        _shape = TransformForm.Identity;
        _matrix = new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        _loc = new Xyz(0.0, 0.0, 0.0);
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The central symmetry about a point.</summary>
    public static Transform3D Mirror(in Point3D p) => new Transform3D().WithMirror(p);

    /// <summary>The rotation by 180 degrees about an axis.</summary>
    public static Transform3D Mirror(in Axis3D a1) => new Transform3D().WithMirror(a1);

    /// <summary>The reflection in a plane.</summary>
    public static Transform3D Mirror(in Frame3D a2) => new Transform3D().WithMirror(a2);

    public static Transform3D Rotation(in Axis3D a1, double ang) => new Transform3D().WithRotation(a1, ang);

    public static Transform3D Rotation(in Quaternion3 r) => new Transform3D().WithRotation(r);

    public static Transform3D Scale(in Point3D p, double s) => new Transform3D().WithScale(p, s);

    public static Transform3D Displacement(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2)
        => new Transform3D().WithDisplacement(fromSystem1, toSystem2);

    public static Transform3D Transformation(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2)
        => new Transform3D().WithTransformation(fromSystem1, toSystem2);

    public static Transform3D Transformation(in CoordinateSystem toSystem) => new Transform3D().WithTransformation(toSystem);

    public static Transform3D Transformation(in Quaternion3 r, in Vector3D t) => new Transform3D().WithTransformation(r, t);

    public static Transform3D Translation(in Vector3D v) => new Transform3D().WithTranslation(v);

    public static Transform3D Translation(in Point3D p1, in Point3D p2) => new Transform3D().WithTranslation(p1, p2);

    public static Transform3D Values(double a11, double a12, double a13, double a14,
                                 double a21, double a22, double a23, double a24,
                                 double a31, double a32, double a33, double a34)
        => new Transform3D().WithValues(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);

    // Copies

    public readonly Transform3D WithMirror(in Point3D p)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = PntMirror;
        scale = -1.0;
        loc = p.XYZ();
        matrix = matrix.WithIdentity();
        loc.Multiply(2.0);
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithMirror(in Axis3D a1)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = Ax1Mirror;
        scale = 1;
        loc = a1.Location.XYZ();
        matrix = matrix.WithDot(a1.Direction.XYZ());
        matrix = matrix.Multiplied(-2);
        matrix = matrix.WithDiagonal(matrix.Value(1, 1) + 1, matrix.Value(2, 2) + 1, matrix.Value(3, 3) + 1);
        loc.Multiply(matrix);
        loc.Add(a1.Location.XYZ());
        matrix = matrix.Multiplied(-1);
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithMirror(in Frame3D a2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = Ax2Mirror;
        scale = -1;
        loc = a2.Location.XYZ();
        matrix = matrix.WithDot(a2.Direction.XYZ());
        matrix = matrix.Multiplied(2);
        matrix = matrix.WithDiagonal(matrix.Value(1, 1) - 1, matrix.Value(2, 2) - 1, matrix.Value(3, 3) - 1);
        loc.Multiply(matrix);
        loc.Add(a2.Location.XYZ());
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithRotation(in Axis3D a1, double ang)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Rotation;
        scale = 1.0;
        loc = a1.Location.XYZ();
        matrix = matrix.WithRotation(a1.Direction.XYZ(), ang);
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(a1.Location.XYZ());
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithRotation(in Quaternion3 r)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Rotation;
        scale = 1.0;
        loc.SetCoord(0.0, 0.0, 0.0);
        matrix = r.GetMatrix();
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithRotationPart(in Quaternion3 r)
    {
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        bool hasRotation = !r.IsEqual(new Quaternion3());
        if (hasRotation)
        {
            matrix = r.GetMatrix();
        }
        else
        {
            matrix = matrix.WithIdentity();
        }

        switch (shape)
        {
            case TransformForm.Identity:
                if (hasRotation)
                {
                    shape = global::Kerf.Geometry.TransformForm.Rotation;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Rotation:
                if (!hasRotation)
                {
                    shape = TransformForm.Identity;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Translation:
            case PntMirror:
            case Ax1Mirror:
            case Ax2Mirror:
            case global::Kerf.Geometry.TransformForm.Scale:
            case CompoundTrsf:
            case Other:
                if (hasRotation)
                {
                    shape = CompoundTrsf;
                }

                break;
        }
        return new Transform3D(_scale, shape, matrix, _loc);
    }

    public readonly Transform3D WithScale(in Point3D p, double s)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Scale;
        scale = s;
        loc = p.XYZ();
        Standard_ConstructionError_Raise_if(Math.Abs(scale) <= Constants.Resolution(), "gp_Trsf::SetScaleFactor");
        matrix = matrix.WithIdentity();
        loc.Multiply(1 - s);
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithDisplacement(in CoordinateSystem fromA1, in CoordinateSystem toA2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 newMatrix = _matrix;
        Xyz loc = _loc;
        shape = CompoundTrsf;
        scale = 1.0;
        // matrix from ToA2 to XOY
        newMatrix = newMatrix.WithCol(1, toA2.XDirection.XYZ());
        newMatrix = newMatrix.WithCol(2, toA2.YDirection.XYZ());
        newMatrix = newMatrix.WithCol(3, toA2.Direction.XYZ());
        loc = toA2.Location.XYZ();

        // matrix XOY to FromA1
        Xyz xDir = fromA1.XDirection.XYZ();
        Xyz yDir = fromA1.YDirection.XYZ();
        Xyz zDir = fromA1.Direction.XYZ();

        Matrix3x3 ma1 = new(xDir, yDir, zDir);
        ma1 = ma1.Transposed();
        Xyz ma1loc = fromA1.Location.XYZ();
        ma1loc.Multiply(ma1);
        ma1loc.Reverse();

        // matrix * MA1
        ma1loc.Multiply(newMatrix);
        loc.Add(ma1loc);
        newMatrix = newMatrix.Multiplied(ma1);
        return new Transform3D(scale, shape, newMatrix, loc);
    }

    public readonly Transform3D WithTransformation(in CoordinateSystem fromA1, in CoordinateSystem toA2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 newMatrix = _matrix;
        Xyz loc = _loc;
        shape = CompoundTrsf;
        scale = 1.0;
        // matrix from XOY to ToA2
        newMatrix = newMatrix.WithRows(toA2.XDirection.XYZ(), toA2.YDirection.XYZ(), toA2.Direction.XYZ());
        loc = toA2.Location.XYZ();
        loc.Multiply(newMatrix);
        loc.Reverse();

        // matrix FromA1 to XOY
        Xyz xDir = fromA1.XDirection.XYZ();
        Xyz yDir = fromA1.YDirection.XYZ();
        Xyz zDir = fromA1.Direction.XYZ();

        Matrix3x3 ma1 = new(xDir, yDir, zDir);
        Xyz ma1loc = fromA1.Location.XYZ();

        // matrix * MA1 => FromA1 ToA2
        ma1loc.Multiply(newMatrix);
        loc.Add(ma1loc);
        newMatrix = newMatrix.Multiplied(ma1);
        return new Transform3D(scale, shape, newMatrix, loc);
    }

    public readonly Transform3D WithTransformation(in CoordinateSystem a3)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = CompoundTrsf;
        scale = 1.0;
        matrix = matrix.WithRows(a3.XDirection.XYZ(), a3.YDirection.XYZ(), a3.Direction.XYZ());
        loc = a3.Location.XYZ();
        loc.Multiply(matrix);
        loc.Reverse();
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithTransformation(in Quaternion3 r, in Vector3D t)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = CompoundTrsf;
        scale = 1.0;
        loc = t.XYZ();
        matrix = r.GetMatrix();
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithTranslation(in Vector3D v)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Translation;
        scale = 1.0;
        matrix = matrix.WithIdentity();
        loc = v.XYZ();
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithTranslation(in Point3D p1, in Point3D p2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Translation;
        scale = 1.0;
        matrix = matrix.WithIdentity();
        loc = p2.XYZ().Subtracted(p1.XYZ());
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D WithTranslationPart(in Vector3D v)
    {
        TransformForm shape = _shape;
        Xyz loc = _loc;
        loc = v.XYZ();
        bool locnull = loc.SquareModulus() < Constants.Resolution();

        switch (shape)
        {
            case TransformForm.Identity:
                if (!locnull)
                {
                    shape = global::Kerf.Geometry.TransformForm.Translation;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Translation:
                if (locnull)
                {
                    shape = TransformForm.Identity;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Rotation:
            case PntMirror:
            case Ax1Mirror:
            case Ax2Mirror:
            case global::Kerf.Geometry.TransformForm.Scale:
            case CompoundTrsf:
            case Other:
                if (!locnull)
                {
                    shape = CompoundTrsf;
                }

                break;
        }
        return new Transform3D(_scale, shape, _matrix, loc);
    }

    public readonly Transform3D WithScaleFactor(double s)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Standard_ConstructionError_Raise_if(Math.Abs(s) <= Constants.Resolution(), "gp_Trsf::SetScaleFactor");
        scale = s;

        bool unit = Math.Abs(scale - 1.0) <= Constants.Resolution();
        bool munit = Math.Abs(scale + 1.0) <= Constants.Resolution();

        switch (shape)
        {
            case TransformForm.Identity:
            case global::Kerf.Geometry.TransformForm.Translation:
                if (!unit)
                {
                    shape = global::Kerf.Geometry.TransformForm.Scale;
                }

                if (munit)
                {
                    shape = PntMirror;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Rotation:
                if (!unit)
                {
                    shape = CompoundTrsf;
                }

                break;
            case PntMirror:
            case Ax1Mirror:
            case Ax2Mirror:
                if (!munit)
                {
                    shape = global::Kerf.Geometry.TransformForm.Scale;
                }

                if (unit)
                {
                    shape = TransformForm.Identity;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Scale:
                if (unit)
                {
                    shape = TransformForm.Identity;
                }

                if (munit)
                {
                    shape = PntMirror;
                }

                break;
            case CompoundTrsf:
                break;
            case Other:
                break;
        }
        return new Transform3D(scale, shape, _matrix, _loc);
    }

    internal readonly Transform3D WithForm(TransformForm p)
    {
        TransformForm shape = _shape;
        shape = p;
        return new Transform3D(_scale, shape, _matrix, _loc);
    }

    public readonly Transform3D WithValues(double a11, double a12, double a13, double a14,
                                       double a21, double a22, double a23, double a24,
                                       double a31, double a32, double a33, double a34)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        Xyz col1 = new(a11, a21, a31);
        Xyz col2 = new(a12, a22, a32);
        Xyz col3 = new(a13, a23, a33);
        Xyz col4 = new(a14, a24, a34);
        // compute the determinant
        Matrix3x3 m = new(col1, col2, col3);
        double s = m.Determinant;
        Standard_ConstructionError_Raise_if(Math.Abs(s) < Constants.Resolution(), "gp_Trsf::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Pow(s, 1.0 / 3.0);
        }
        else
        {
            s = -Math.Pow(-s, 1.0 / 3.0);
        }

        m = m.Divided(s);

        return new Transform3D(s, CompoundTrsf, Orthogonalized(m), col4);
    }

    /// <summary>The matrix made orthogonal again (OCCT's <c>Orthogonalize</c>, which changed the transformation in place).</summary>
    private static Matrix3x3 Orthogonalized(Matrix3x3 tm)
    {
        Xyz v1 = tm.Column(1);
        Xyz v2 = tm.Column(2);
        Xyz v3 = tm.Column(3);

        v1.Normalize();

        v2 -= v1 * v2.Dot(v1);
        v2.Normalize();

        v3 -= v1 * v3.Dot(v1) + v2 * v3.Dot(v2);
        v3.Normalize();

        tm = tm.WithCols(v1, v2, v3);

        v1 = tm.Row(1);
        v2 = tm.Row(2);
        v3 = tm.Row(3);

        v1.Normalize();

        v2 -= v1 * v2.Dot(v1);
        v2.Normalize();

        v3 -= v1 * v3.Dot(v1) + v2 * v3.Dot(v2);
        v3.Normalize();

        return tm.WithRows(v1, v2, v3);
    }

    // Queries

    /// <summary>True when the determinant of the vectorial part is negative (the transformation mirrors).</summary>
    public readonly bool IsNegative => _scale < 0.0;

    public readonly TransformForm Form => _shape;

    public readonly double ScaleFactor => _scale;

    public readonly Xyz TranslationXyz() => _loc;

    /// <summary>The rotation as an axis and an angle. Always true, as in OCCT.</summary>
    internal readonly bool GetRotation(out Xyz axis, out double angle)
    {
        Quaternion3 q = GetRotation();
        q.GetVectorAndAngle(out Vector3D vec, out angle);
        axis = vec.XYZ();
        return true;
    }

    internal readonly Quaternion3 GetRotation() => new(_matrix);

    /// <summary>The matrix multiplied by the scale factor.</summary>
    public readonly Matrix3x3 VectorialPart
    {
        get
        {
            if (_scale == 1.0)
            {
                return _matrix;
            }

            Matrix3x3 m = _matrix;
            if (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror)
            {
                m = m.WithDiagonal(_scale * m._m00, _scale * m._m11, _scale * m._m22);
            }
            else
            {
                m = m.Multiplied(_scale);
            }

            return m;
        }
    }

    /// <summary>The matrix without the scale factor.</summary>
    public readonly Matrix3x3 RotationPart => _matrix;

    /// <summary>The coefficient at a row (1 to 3) and column (1 to 4) of the 3x4 matrix, scale included.</summary>
    internal readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 4, " ");
        if (col < 4)
        {
            return _scale * _matrix.Value(row, col);
        }
        else
        {
            return _loc.Coord(row);
        }
    }

    public readonly Transform3D Inverted()
    {
        double newScale = _scale;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        // -1
        // X' = scale * R * X + T  =>  X = (R / scale) * (X' - T)
        if (_shape == TransformForm.Identity)
        {
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation || _shape == PntMirror)
        {
            loc.Reverse();
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            Standard_ConstructionError_Raise_if(Math.Abs(newScale) <= Constants.Resolution(), "gp_Trsf::Invert() - transformation has zero scale");
            newScale = 1.0 / newScale;
            loc.Multiply(-newScale);
        }
        else
        {
            Standard_ConstructionError_Raise_if(Math.Abs(newScale) <= Constants.Resolution(), "gp_Trsf::Invert() - transformation has zero scale");
            newScale = 1.0 / newScale;
            matrix = matrix.Transposed();
            loc.Multiply(matrix);
            loc.Multiply(-newScale);
        }
        return new Transform3D(newScale, _shape, matrix, loc);
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly Transform3D Multiplied(in Transform3D t)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
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
            if (t._loc.X() != 0.0 || t._loc.Y() != 0.0 || t._loc.Z() != 0.0)
            {
                loc.Add(t._loc.Multiplied(matrix));
            }

            matrix = matrix.Multiplied(t._matrix);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(t._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Scale && t._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            loc.Add(t._loc.Multiplied(scale));
            scale = scale * t._scale;
        }
        else if (shape == PntMirror && t._shape == PntMirror)
        {
            scale = 1.0;
            shape = global::Kerf.Geometry.TransformForm.Translation;
            loc.Add(t._loc.Reversed());
        }
        else if (shape == Ax1Mirror && t._shape == Ax1Mirror)
        {
            shape = global::Kerf.Geometry.TransformForm.Rotation;
            loc.Add(t._loc.Multiplied(matrix));
            matrix = matrix.Multiplied(t._matrix);
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror || shape == Ax2Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xyz tloc = t._loc;
            tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                tloc.Multiply(scale);
            }

            loc.Add(tloc);
        }
        else if ((shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xyz tloc = t._loc;
            tloc.Multiply(scale);
            loc.Add(tloc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror))
        {
            shape = CompoundTrsf;
            scale = t._scale;
            loc.Add(t._loc);
            matrix = t._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            shape = t._shape;
            loc.Add(t._loc);
            scale = t._scale;
        }
        else if ((shape == PntMirror || shape == global::Kerf.Geometry.TransformForm.Scale) && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            shape = CompoundTrsf;
            Xyz tloc = t._loc;
            tloc.Multiply(scale);
            loc.Add(tloc);
            scale = scale * t._scale;
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror || shape == Ax2Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            shape = CompoundTrsf;
            Xyz tloc = t._loc;
            if (scale == 1.0)
            {
                scale = t._scale;
                tloc.Multiply(matrix);
            }
            else
            {
                tloc.Multiply(matrix);
                tloc.Multiply(scale);
                scale = scale * t._scale;
            }

            loc.Add(tloc);
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror)
                 && (shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror))
        {
            shape = CompoundTrsf;
            Xyz tloc = t._loc;
            tloc.Multiply(scale);
            loc.Add(tloc);
            scale = scale * t._scale;
            matrix = t._matrix;
        }
        else
        {
            shape = CompoundTrsf;
            Xyz tloc = t._loc;
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

            loc.Add(tloc);
            matrix = matrix.Multiplied(t._matrix);
        }
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly Transform3D PreMultiplied(in Transform3D t)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
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
            matrix = matrix.PreMultiplied(t._matrix);
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
            loc.Add(t._loc);
            matrix = matrix.PreMultiplied(t._matrix);
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror || shape == Ax2Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(t._loc);
        }
        else if ((shape == global::Kerf.Geometry.TransformForm.Scale || shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(t._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror))
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
        else if ((shape == PntMirror || shape == global::Kerf.Geometry.TransformForm.Scale) && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            shape = CompoundTrsf;
            loc.Multiply(t._scale);
            loc.Add(t._loc);
            scale = scale * t._scale;
        }
        else if ((shape == CompoundTrsf || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == Ax1Mirror || shape == Ax2Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            shape = CompoundTrsf;
            loc.Multiply(t._scale);
            loc.Add(t._loc);
            scale = scale * t._scale;
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror)
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
            matrix = matrix.PreMultiplied(t._matrix);
        }
        return new Transform3D(scale, shape, matrix, loc);
    }

    public readonly Transform3D Powered(int n)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        if (shape == TransformForm.Identity)
        {
        }
        else
        {
            if (n == 0)
            {
                scale = 1.0;
                shape = TransformForm.Identity;
                matrix = matrix.WithIdentity();
                loc = new Xyz(0.0, 0.0, 0.0);
            }
            else if (n == 1)
            {
            }
            else if (n == -1)
            {
                (scale, shape, matrix, loc) = new Transform3D(scale, shape, matrix, loc).Inverted();
            }
            else
            {
                if (n < 0)
                {
                    (scale, shape, matrix, loc) = new Transform3D(scale, shape, matrix, loc).Inverted();
                }

                if (shape == global::Kerf.Geometry.TransformForm.Translation)
                {
                    int npower = Math.Abs(n) - 1;
                    Xyz temploc = loc;
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
                    int npower = Math.Abs(n) - 1;
                    Xyz temploc = loc;
                    double tempscale = scale;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            loc.Add(temploc.Multiplied(scale));
                            scale = scale * tempscale;
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
                    int npower = Math.Abs(n) - 1;
                    Matrix3x3 tempmatrix = matrix;
                    if (loc.X() == 0.0 && loc.Y() == 0.0 && loc.Z() == 0.0)
                    {
                        for (;;)
                        {
                            if (IsOdd(npower))
                            {
                                matrix = matrix.Multiplied(tempmatrix);
                            }

                            if (npower == 1)
                            {
                                break;
                            }

                            tempmatrix = tempmatrix.Multiplied(tempmatrix);
                            npower = npower / 2;
                        }
                    }
                    else
                    {
                        Xyz temploc = loc;
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
                }
                else if (shape == PntMirror || shape == Ax1Mirror || shape == Ax2Mirror)
                {
                    if (IsEven(n))
                    {
                        shape = TransformForm.Identity;
                        scale = 1.0;
                        matrix = matrix.WithIdentity();
                        loc.SetX(0);
                        loc.SetY(0);
                        loc.SetZ(0);
                    }
                }
                else
                {
                    shape = CompoundTrsf;
                    int npower = Math.Abs(n) - 1;
                    Xyz temploc = loc;
                    double tempscale = scale;
                    Matrix3x3 tempmatrix = matrix;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            loc.Add(temploc.Multiplied(matrix).Multiplied(scale));
                            scale = scale * tempscale;
                            matrix = matrix.Multiplied(tempmatrix);
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        tempscale = tempscale * tempscale;
                        temploc.Add(temploc.Multiplied(tempmatrix).Multiplied(tempscale));
                        tempmatrix = tempmatrix.Multiplied(tempmatrix);
                        npower = npower / 2;
                    }
                }
            }
        }
        return new Transform3D(scale, shape, matrix, loc);
    }

    internal readonly void Transforms(ref double x, ref double y, ref double z)
    {
        Xyz triplet = new(x, y, z);
        triplet.Multiply(_matrix);
        if (_scale != 1.0)
        {
            triplet.Multiply(_scale);
        }

        triplet.Add(_loc);
        x = triplet.X();
        y = triplet.Y();
        z = triplet.Z();
    }

    internal readonly void Transforms(ref Xyz coord)
    {
        coord.Multiply(_matrix);
        if (_scale != 1.0)
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref Xyz)"/>.</summary>
    public readonly Xyz Transformed(in Xyz coord)
    {
        Xyz aCoord = coord;
        Transforms(ref aCoord);
        return aCoord;
    }

    /// <summary>The 4x4 matrix, row by row, as OCCT's <c>GetMat4</c>.</summary>
    internal readonly void GetMat4(Span<double> mat)
    {
        if (mat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(mat));
        }

        if (_shape == TransformForm.Identity)
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

    public static Transform3D operator *(in Transform3D left, in Transform3D right) => left.Multiplied(right);

    public readonly bool Equals(Transform3D other)
        => _scale.Equals(other._scale) && _shape == other._shape && _matrix.Equals(other._matrix) && _loc.Equals(other._loc);

    public override readonly bool Equals(object? obj) => obj is Transform3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_scale, _shape, _matrix, _loc);

    public static bool operator ==(in Transform3D left, in Transform3D right) => left.Equals(right);

    public static bool operator !=(in Transform3D left, in Transform3D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Trsf({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
