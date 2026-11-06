// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>
/// A non-persistent transformation in 3D space: a rigid motion, mirror or uniform scale, or a combination.
/// It maps P to <c>scale * matrix * P + loc</c>.
/// </summary>
/// <remarks>
/// <c>new Transform3D()</c> is the identity, as in OCCT; <c>default(Transform3D)</c> is not valid. Build one with the
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

    /// <summary>The fields, for the returning forms that take a whole value back.</summary>
    private void Deconstruct(out double scale, out TransformForm shape, out Matrix3x3 matrix, out Xyz loc)
    {
        scale = _scale;
        shape = _shape;
        matrix = _matrix;
        loc = _loc;
    }

    /// <summary>The identity.</summary>
    public Transform3D()
    {
        _scale = 1.0;
        _shape = TransformForm.Identity;
        _matrix = new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        _loc = new Xyz(0.0, 0.0, 0.0);
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The central symmetry about a point.</summary>
    public static Transform3D Mirror(in Point3D point) => new Transform3D().WithMirror(point);

    /// <summary>The rotation by 180 degrees about an axis.</summary>
    public static Transform3D Mirror(in Axis3D axis) => new Transform3D().WithMirror(axis);

    /// <summary>The reflection in a plane.</summary>
    public static Transform3D Mirror(in Frame3D frame) => new Transform3D().WithMirror(frame);

    /// <summary>The rotation about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public static Transform3D Rotation(in Axis3D axis, double angle) => new Transform3D().WithRotation(axis, angle);

    internal static Transform3D Rotation(in Quaternion3 r) => new Transform3D().WithRotation(r);

    /// <summary>The uniform scale about <paramref name="point"/>.</summary>
    /// <exception cref="ArgumentException"><paramref name="factor"/> is within the resolution of zero.</exception>
    public static Transform3D Scale(in Point3D point, double factor) => new Transform3D().WithScale(point, factor);

    /// <summary>The rigid motion that carries <paramref name="from"/> on to <paramref name="to"/>: it moves geometry. <see cref="Transformation(in CoordinateSystem, in CoordinateSystem)"/> re-expresses coordinates instead.</summary>
    public static Transform3D Displacement(in CoordinateSystem from, in CoordinateSystem to)
        => new Transform3D().WithDisplacement(from, to);

    /// <summary>What re-expresses coordinates given in <paramref name="from"/> as coordinates in <paramref name="to"/>: geometry stays where it is. <see cref="Displacement"/> moves it instead.</summary>
    public static Transform3D Transformation(in CoordinateSystem from, in CoordinateSystem to)
        => new Transform3D().WithTransformation(from, to);

    /// <summary>What re-expresses absolute coordinates as coordinates in <paramref name="toSystem"/>.</summary>
    public static Transform3D Transformation(in CoordinateSystem toSystem) => new Transform3D().WithTransformation(toSystem);

    internal static Transform3D Transformation(in Quaternion3 r, in Vector3D t) => new Transform3D().WithTransformation(r, t);

    /// <summary>The translation by <paramref name="vector"/>.</summary>
    public static Transform3D Translation(in Vector3D vector) => new Transform3D().WithTranslation(vector);

    /// <summary>The translation by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public static Transform3D Translation(in Point3D point1, in Point3D point2) => new Transform3D().WithTranslation(point1, point2);

    /// <summary>The transformation from a 3 by 4 matrix given row by row, the last column being the translation: as <see cref="WithValues"/>.</summary>
    /// <exception cref="ArgumentException">The 3 by 3 part is singular or not a similarity.</exception>
    public static Transform3D Values(double a11, double a12, double a13, double a14,
                                 double a21, double a22, double a23, double a24,
                                 double a31, double a32, double a33, double a34)
        => new Transform3D().WithValues(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);

    // Copies

    /// <summary>The central symmetry through <paramref name="point"/>. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithMirror(in Point3D point)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = PointMirror;
        scale = -1.0;
        loc = point.XYZ();
        matrix = matrix.WithIdentity();
        loc.Multiply(2.0);
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>The half-turn about <paramref name="axis"/>, with a scale factor of 1. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithMirror(in Axis3D axis)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = AxisMirror;
        scale = 1;
        loc = axis.Location.XYZ();
        matrix = matrix.WithDot(axis.Direction.XYZ());
        matrix = matrix.Multiplied(-2);
        matrix = matrix.WithDiagonal(matrix.Value(1, 1) + 1, matrix.Value(2, 2) + 1, matrix.Value(3, 3) + 1);
        loc.Multiply(matrix);
        loc.Add(axis.Location.XYZ());
        matrix = matrix.Multiplied(-1);
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>, with a scale factor of -1. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithMirror(in Frame3D frame)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = PlaneMirror;
        scale = -1;
        loc = frame.Location.XYZ();
        matrix = matrix.WithDot(frame.Direction.XYZ());
        matrix = matrix.Multiplied(2);
        matrix = matrix.WithDiagonal(matrix.Value(1, 1) - 1, matrix.Value(2, 2) - 1, matrix.Value(3, 3) - 1);
        loc.Multiply(matrix);
        loc.Add(frame.Location.XYZ());
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>The rotation about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithRotation(in Axis3D axis, double angle)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Rotation;
        scale = 1.0;
        loc = axis.Location.XYZ();
        matrix = matrix.WithRotation(axis.Direction.XYZ(), angle);
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(axis.Location.XYZ());
        return new Transform3D(scale, shape, matrix, loc);
    }

    internal readonly Transform3D WithRotation(in Quaternion3 r)
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

    internal readonly Transform3D WithRotationPart(in Quaternion3 r)
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
            case PointMirror:
            case AxisMirror:
            case PlaneMirror:
            case global::Kerf.Geometry.TransformForm.Scale:
            case Composite:
            case Other:
                if (hasRotation)
                {
                    shape = Composite;
                }

                break;
        }
        return new Transform3D(_scale, shape, matrix, _loc);
    }

    /// <summary>The uniform scale about <paramref name="point"/>. It replaces this value rather than composing with it.</summary>
    /// <exception cref="ArgumentException"><paramref name="factor"/> is within the resolution of zero.</exception>
    public readonly Transform3D WithScale(in Point3D point, double factor)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Scale;
        scale = factor;
        loc = point.XYZ();
        Refusal.ArgumentIf(Math.Abs(scale) <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf::SetScaleFactor");
        matrix = matrix.WithIdentity();
        loc.Multiply(1 - factor);
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>The rigid motion that carries <paramref name="from"/> on to <paramref name="to"/>. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithDisplacement(in CoordinateSystem from, in CoordinateSystem to)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 newMatrix = _matrix;
        Xyz loc = _loc;
        shape = Composite;
        scale = 1.0;
        // matrix from ToA2 to XOY
        newMatrix = newMatrix.WithCol(1, to.XDirection.XYZ());
        newMatrix = newMatrix.WithCol(2, to.YDirection.XYZ());
        newMatrix = newMatrix.WithCol(3, to.Direction.XYZ());
        loc = to.Location.XYZ();

        // matrix XOY to FromA1
        Xyz xDir = from.XDirection.XYZ();
        Xyz yDir = from.YDirection.XYZ();
        Xyz zDir = from.Direction.XYZ();

        Matrix3x3 ma1 = new(xDir, yDir, zDir);
        ma1 = ma1.Transposed();
        Xyz ma1loc = from.Location.XYZ();
        ma1loc.Multiply(ma1);
        ma1loc.Reverse();

        // matrix * MA1
        ma1loc.Multiply(newMatrix);
        loc.Add(ma1loc);
        newMatrix = newMatrix.Multiplied(ma1);
        return new Transform3D(scale, shape, newMatrix, loc);
    }

    /// <summary>What re-expresses coordinates given in <paramref name="from"/> as coordinates in <paramref name="to"/>. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithTransformation(in CoordinateSystem from, in CoordinateSystem to)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 newMatrix = _matrix;
        Xyz loc = _loc;
        shape = Composite;
        scale = 1.0;
        // matrix from XOY to ToA2
        newMatrix = newMatrix.WithRows(to.XDirection.XYZ(), to.YDirection.XYZ(), to.Direction.XYZ());
        loc = to.Location.XYZ();
        loc.Multiply(newMatrix);
        loc.Reverse();

        // matrix FromA1 to XOY
        Xyz xDir = from.XDirection.XYZ();
        Xyz yDir = from.YDirection.XYZ();
        Xyz zDir = from.Direction.XYZ();

        Matrix3x3 ma1 = new(xDir, yDir, zDir);
        Xyz ma1loc = from.Location.XYZ();

        // matrix * MA1 => FromA1 ToA2
        ma1loc.Multiply(newMatrix);
        loc.Add(ma1loc);
        newMatrix = newMatrix.Multiplied(ma1);
        return new Transform3D(scale, shape, newMatrix, loc);
    }

    /// <summary>What re-expresses absolute coordinates as coordinates in <paramref name="position"/>. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithTransformation(in CoordinateSystem position)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = Composite;
        scale = 1.0;
        matrix = matrix.WithRows(position.XDirection.XYZ(), position.YDirection.XYZ(), position.Direction.XYZ());
        loc = position.Location.XYZ();
        loc.Multiply(matrix);
        loc.Reverse();
        return new Transform3D(scale, shape, matrix, loc);
    }

    internal readonly Transform3D WithTransformation(in Quaternion3 r, in Vector3D t)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = Composite;
        scale = 1.0;
        loc = t.XYZ();
        matrix = r.GetMatrix();
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>The translation by <paramref name="vector"/>. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithTranslation(in Vector3D vector)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Translation;
        scale = 1.0;
        matrix = matrix.WithIdentity();
        loc = vector.XYZ();
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>The translation by the vector from <paramref name="point1"/> to <paramref name="point2"/>. It replaces this value rather than composing with it.</summary>
    public readonly Transform3D WithTranslation(in Point3D point1, in Point3D point2)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        shape = global::Kerf.Geometry.TransformForm.Translation;
        scale = 1.0;
        matrix = matrix.WithIdentity();
        loc = point2.XYZ().Subtracted(point1.XYZ());
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>A copy with this translation part, keeping the rotation and scale. <see cref="Form"/> changes to match.</summary>
    public readonly Transform3D WithTranslationPart(in Vector3D vector)
    {
        TransformForm shape = _shape;
        Xyz loc = _loc;
        loc = vector.XYZ();
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
            case PointMirror:
            case AxisMirror:
            case PlaneMirror:
            case global::Kerf.Geometry.TransformForm.Scale:
            case Composite:
            case Other:
                if (!locnull)
                {
                    shape = Composite;
                }

                break;
        }
        return new Transform3D(_scale, shape, _matrix, loc);
    }

    /// <summary>A copy with this scale factor, keeping the rest. <see cref="Form"/> changes to match.</summary>
    /// <exception cref="ArgumentException"><paramref name="factor"/> is within the resolution of zero.</exception>
    public readonly Transform3D WithScaleFactor(double factor)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Refusal.ArgumentIf(Math.Abs(factor) <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf::SetScaleFactor");
        scale = factor;

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
                    shape = PointMirror;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Rotation:
                if (!unit)
                {
                    shape = Composite;
                }

                break;
            case PointMirror:
            case AxisMirror:
            case PlaneMirror:
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
                    shape = PointMirror;
                }

                break;
            case Composite:
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

    /// <summary>The transformation from a 3 by 4 matrix given row by row, the last column being the translation. The scale is the cube root of the 3 by 3 part's determinant, and that part is orthogonalised after dividing by it. It replaces this value rather than composing with it.</summary>
    /// <exception cref="ArgumentException">The 3 by 3 part is singular or not a similarity.</exception>
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
        Refusal.ArgumentIf(Math.Abs(s) < Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Pow(s, 1.0 / 3.0);
        }
        else
        {
            s = -Math.Pow(-s, 1.0 / 3.0);
        }

        m = m.Divided(s);

        try
        {
            return new Transform3D(s, Composite, Orthogonalized(m), col4);
        }
        catch (FailureException failure)
        {
            throw Refusal.Argument(failure);
        }
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

    /// <summary>The kind of transformation this is, which some algorithms use as a shortcut.</summary>
    public readonly TransformForm Form => _shape;

    /// <summary>The uniform scale: 1 for a rigid motion, and negative for a point or plane mirror.</summary>
    public readonly double ScaleFactor => _scale;

    internal readonly Xyz TranslationXyz() => _loc;

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
            if (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PointMirror)
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
        Refusal.RangeIf(row < 1 || row > 3 || col < 1 || col > 4, FailureKind.OutOfRange, " ");
        if (col < 4)
        {
            return _scale * _matrix.Value(row, col);
        }
        else
        {
            return _loc.Coord(row);
        }
    }

    /// <summary>The transformation that undoes this one.</summary>
    /// <exception cref="InvalidOperationException">The scale factor is within the resolution of zero.</exception>
    public readonly Transform3D Inverted()
    {
        double newScale = _scale;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        // -1
        // X' = scale * R * X + T  =>  X = (R / scale) * (X' - T)
        // A mirror through a point is its own inverse (OCCT 3270746444, after 8.0.1; decision 20204)
        if (_shape == TransformForm.Identity || _shape == PointMirror)
        {
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Reverse();
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            Refusal.StateIf(Math.Abs(newScale) <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf::Invert() - transformation has zero scale");
            newScale = 1.0 / newScale;
            loc.Multiply(-newScale);
        }
        else
        {
            Refusal.StateIf(Math.Abs(newScale) <= Constants.Resolution(), FailureKind.ConstructionError, "gp_Trsf::Invert() - transformation has zero scale");
            newScale = 1.0 / newScale;
            matrix = matrix.Transposed();
            loc.Multiply(matrix);
            loc.Multiply(-newScale);
        }
        return new Transform3D(newScale, _shape, matrix, loc);
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="transform"/> first.</summary>
    public readonly Transform3D Multiplied(in Transform3D transform)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        if (transform._shape == TransformForm.Identity)
        {
        }
        else if (shape == TransformForm.Identity)
        {
            shape = transform._shape;
            scale = transform._scale;
            loc = transform._loc;
            matrix = transform._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Rotation && transform._shape == global::Kerf.Geometry.TransformForm.Rotation)
        {
            if (transform._loc.X() != 0.0 || transform._loc.Y() != 0.0 || transform._loc.Z() != 0.0)
            {
                loc.Add(transform._loc.Multiplied(matrix));
            }

            matrix = matrix.Multiplied(transform._matrix);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && transform._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(transform._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Scale && transform._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            loc.Add(transform._loc.Multiplied(scale));
            scale = scale * transform._scale;
        }
        else if (shape == PointMirror && transform._shape == PointMirror)
        {
            scale = 1.0;
            shape = global::Kerf.Geometry.TransformForm.Translation;
            loc.Add(transform._loc.Reversed());
        }
        else if (shape == AxisMirror && transform._shape == AxisMirror)
        {
            shape = global::Kerf.Geometry.TransformForm.Rotation;
            loc.Add(transform._loc.Multiplied(matrix));
            matrix = matrix.Multiplied(transform._matrix);
        }
        else if ((shape == Composite || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == AxisMirror || shape == PlaneMirror)
                 && transform._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xyz tloc = transform._loc;
            tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                tloc.Multiply(scale);
            }

            loc.Add(tloc);
        }
        else if ((shape == global::Kerf.Geometry.TransformForm.Scale || shape == PointMirror) && transform._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xyz tloc = transform._loc;
            tloc.Multiply(scale);
            loc.Add(tloc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation
                 && (transform._shape == Composite || transform._shape == global::Kerf.Geometry.TransformForm.Rotation || transform._shape == AxisMirror || transform._shape == PlaneMirror))
        {
            shape = Composite;
            scale = transform._scale;
            loc.Add(transform._loc);
            matrix = transform._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && (transform._shape == global::Kerf.Geometry.TransformForm.Scale || transform._shape == PointMirror))
        {
            shape = transform._shape;
            loc.Add(transform._loc);
            scale = transform._scale;
        }
        else if ((shape == PointMirror || shape == global::Kerf.Geometry.TransformForm.Scale) && (transform._shape == PointMirror || transform._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            shape = Composite;
            Xyz tloc = transform._loc;
            tloc.Multiply(scale);
            loc.Add(tloc);
            scale = scale * transform._scale;
        }
        else if ((shape == Composite || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == AxisMirror || shape == PlaneMirror)
                 && (transform._shape == global::Kerf.Geometry.TransformForm.Scale || transform._shape == PointMirror))
        {
            shape = Composite;
            Xyz tloc = transform._loc;
            if (scale == 1.0)
            {
                scale = transform._scale;
                tloc.Multiply(matrix);
            }
            else
            {
                tloc.Multiply(matrix);
                tloc.Multiply(scale);
                scale = scale * transform._scale;
            }

            loc.Add(tloc);
        }
        else if ((transform._shape == Composite || transform._shape == global::Kerf.Geometry.TransformForm.Rotation || transform._shape == AxisMirror || transform._shape == PlaneMirror)
                 && (shape == global::Kerf.Geometry.TransformForm.Scale || shape == PointMirror))
        {
            shape = Composite;
            Xyz tloc = transform._loc;
            tloc.Multiply(scale);
            loc.Add(tloc);
            scale = scale * transform._scale;
            matrix = transform._matrix;
        }
        else
        {
            shape = Composite;
            Xyz tloc = transform._loc;
            tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                tloc.Multiply(scale);
                scale = scale * transform._scale;
            }
            else
            {
                scale = transform._scale;
            }

            loc.Add(tloc);
            matrix = matrix.Multiplied(transform._matrix);
        }
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly Transform3D PreMultiplied(in Transform3D transform)
    {
        double scale = _scale;
        TransformForm shape = _shape;
        Matrix3x3 matrix = _matrix;
        Xyz loc = _loc;
        if (transform._shape == TransformForm.Identity)
        {
        }
        else if (shape == TransformForm.Identity)
        {
            shape = transform._shape;
            scale = transform._scale;
            loc = transform._loc;
            matrix = transform._matrix;
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Rotation && transform._shape == global::Kerf.Geometry.TransformForm.Rotation)
        {
            loc.Multiply(transform._matrix);
            loc.Add(transform._loc);
            matrix = matrix.PreMultiplied(transform._matrix);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation && transform._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(transform._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Scale && transform._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            loc.Multiply(transform._scale);
            loc.Add(transform._loc);
            scale = scale * transform._scale;
        }
        else if (shape == PointMirror && transform._shape == PointMirror)
        {
            scale = 1.0;
            shape = global::Kerf.Geometry.TransformForm.Translation;
            loc.Reverse();
            loc.Add(transform._loc);
        }
        else if (shape == AxisMirror && transform._shape == AxisMirror)
        {
            shape = global::Kerf.Geometry.TransformForm.Rotation;
            loc.Multiply(transform._matrix);
            loc.Add(transform._loc);
            matrix = matrix.PreMultiplied(transform._matrix);
        }
        else if ((shape == Composite || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == AxisMirror || shape == PlaneMirror)
                 && transform._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(transform._loc);
        }
        else if ((shape == global::Kerf.Geometry.TransformForm.Scale || shape == PointMirror) && transform._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Add(transform._loc);
        }
        else if (shape == global::Kerf.Geometry.TransformForm.Translation
                 && (transform._shape == Composite || transform._shape == global::Kerf.Geometry.TransformForm.Rotation || transform._shape == AxisMirror || transform._shape == PlaneMirror))
        {
            shape = Composite;
            matrix = transform._matrix;
            if (transform._scale == 1.0)
            {
                loc.Multiply(transform._matrix);
            }
            else
            {
                scale = transform._scale;
                loc.Multiply(matrix);
                loc.Multiply(scale);
            }

            loc.Add(transform._loc);
        }
        else if ((transform._shape == global::Kerf.Geometry.TransformForm.Scale || transform._shape == PointMirror) && shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            loc.Multiply(transform._scale);
            loc.Add(transform._loc);
            scale = transform._scale;
            shape = transform._shape;
        }
        else if ((shape == PointMirror || shape == global::Kerf.Geometry.TransformForm.Scale) && (transform._shape == PointMirror || transform._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            shape = Composite;
            loc.Multiply(transform._scale);
            loc.Add(transform._loc);
            scale = scale * transform._scale;
        }
        else if ((shape == Composite || shape == global::Kerf.Geometry.TransformForm.Rotation || shape == AxisMirror || shape == PlaneMirror)
                 && (transform._shape == global::Kerf.Geometry.TransformForm.Scale || transform._shape == PointMirror))
        {
            shape = Composite;
            loc.Multiply(transform._scale);
            loc.Add(transform._loc);
            scale = scale * transform._scale;
        }
        else if ((transform._shape == Composite || transform._shape == global::Kerf.Geometry.TransformForm.Rotation || transform._shape == AxisMirror || transform._shape == PlaneMirror)
                 && (shape == global::Kerf.Geometry.TransformForm.Scale || shape == PointMirror))
        {
            shape = Composite;
            matrix = transform._matrix;
            if (transform._scale == 1.0)
            {
                loc.Multiply(transform._matrix);
            }
            else
            {
                loc.Multiply(matrix);
                loc.Multiply(transform._scale);
                scale = transform._scale * scale;
            }

            loc.Add(transform._loc);
        }
        else
        {
            shape = Composite;
            loc.Multiply(transform._matrix);
            if (transform._scale != 1.0)
            {
                loc.Multiply(transform._scale);
                scale = scale * transform._scale;
            }

            loc.Add(transform._loc);
            matrix = matrix.PreMultiplied(transform._matrix);
        }
        return new Transform3D(scale, shape, matrix, loc);
    }

    /// <summary>This transformation composed with itself <paramref name="n"/> times: the identity for 0, and the inverse's powers for a negative <paramref name="n"/>.</summary>
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
                else if (shape == PointMirror || shape == AxisMirror || shape == PlaneMirror)
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
                    shape = Composite;
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

                        // the square's translation takes the scale before it is squared (decision 1485)
                        temploc.Add(temploc.Multiplied(tempmatrix).Multiplied(tempscale));
                        tempscale = tempscale * tempscale;
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
    internal readonly Xyz Transformed(in Xyz coord)
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

    /// <summary>The composition: <paramref name="right"/> is applied first, then <paramref name="left"/>.</summary>
    public static Transform3D operator *(in Transform3D left, in Transform3D right) => left.Multiplied(right);

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Transform3D other)
        => _scale.Equals(other._scale) && _shape == other._shape && _matrix.Equals(other._matrix) && _loc.Equals(other._loc);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Transform3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Transform3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_scale, _shape, _matrix, _loc);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Transform3D left, in Transform3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Transform3D left, in Transform3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Transform3D({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
