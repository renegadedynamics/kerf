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
public partial struct Transform3D : IEquatable<Transform3D>
{
    internal double _scale;
    internal TransformForm _shape;
    internal Matrix3x3 _matrix;
    internal Xyz _loc;

    public Transform3D()
    {
        _scale = 1.0;
        _shape = Identity;
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

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetMirror(in Point3D p)
    {
        _shape = PntMirror;
        _scale = -1.0;
        _loc = p.XYZ();
        _matrix.SetIdentity();
        _loc.Multiply(2.0);
    }

    internal void SetMirror(in Axis3D a1)
    {
        _shape = Ax1Mirror;
        _scale = 1;
        _loc = a1.Location().XYZ();
        _matrix.SetDot(a1.Direction().XYZ());
        _matrix.Multiply(-2);
        _matrix.SetDiagonal(_matrix.Value(1, 1) + 1, _matrix.Value(2, 2) + 1, _matrix.Value(3, 3) + 1);
        _loc.Multiply(_matrix);
        _loc.Add(a1.Location().XYZ());
        _matrix.Multiply(-1);
    }

    internal void SetMirror(in Frame3D a2)
    {
        _shape = Ax2Mirror;
        _scale = -1;
        _loc = a2.Location().XYZ();
        _matrix.SetDot(a2.Direction().XYZ());
        _matrix.Multiply(2);
        _matrix.SetDiagonal(_matrix.Value(1, 1) - 1, _matrix.Value(2, 2) - 1, _matrix.Value(3, 3) - 1);
        _loc.Multiply(_matrix);
        _loc.Add(a2.Location().XYZ());
    }

    internal void SetRotation(in Axis3D a1, double ang)
    {
        _shape = global::Kerf.Geometry.TransformForm.Rotation;
        _scale = 1.0;
        _loc = a1.Location().XYZ();
        _matrix.SetRotation(a1.Direction().XYZ(), ang);
        _loc.Reverse();
        _loc.Multiply(_matrix);
        _loc.Add(a1.Location().XYZ());
    }

    internal void SetRotation(in Quaternion3 r)
    {
        _shape = global::Kerf.Geometry.TransformForm.Rotation;
        _scale = 1.0;
        _loc.SetCoord(0.0, 0.0, 0.0);
        _matrix = r.GetMatrix();
    }

    internal void SetRotationPart(in Quaternion3 r)
    {
        bool hasRotation = !r.IsEqual(new Quaternion3());
        if (hasRotation)
        {
            _matrix = r.GetMatrix();
        }
        else
        {
            _matrix.SetIdentity();
        }

        switch (_shape)
        {
            case Identity:
                if (hasRotation)
                {
                    _shape = global::Kerf.Geometry.TransformForm.Rotation;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Rotation:
                if (!hasRotation)
                {
                    _shape = Identity;
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
                    _shape = CompoundTrsf;
                }

                break;
        }
    }

    internal void SetScale(in Point3D p, double s)
    {
        _shape = global::Kerf.Geometry.TransformForm.Scale;
        _scale = s;
        _loc = p.XYZ();
        Standard_ConstructionError_Raise_if(Math.Abs(_scale) <= Constants.Resolution(), "gp_Trsf::SetScaleFactor");
        _matrix.SetIdentity();
        _loc.Multiply(1 - s);
    }

    internal void SetTransformation(in CoordinateSystem fromA1, in CoordinateSystem toA2)
    {
        _shape = CompoundTrsf;
        _scale = 1.0;
        // matrix from XOY to ToA2
        _matrix.SetRows(toA2.XDirection().XYZ(), toA2.YDirection().XYZ(), toA2.Direction().XYZ());
        _loc = toA2.Location().XYZ();
        _loc.Multiply(_matrix);
        _loc.Reverse();

        // matrix FromA1 to XOY
        Xyz xDir = fromA1.XDirection().XYZ();
        Xyz yDir = fromA1.YDirection().XYZ();
        Xyz zDir = fromA1.Direction().XYZ();

        Matrix3x3 ma1 = new(xDir, yDir, zDir);
        Xyz ma1loc = fromA1.Location().XYZ();

        // matrix * MA1 => FromA1 ToA2
        ma1loc.Multiply(_matrix);
        _loc.Add(ma1loc);
        _matrix.Multiply(ma1);
    }

    internal void SetTransformation(in CoordinateSystem a3)
    {
        _shape = CompoundTrsf;
        _scale = 1.0;
        _matrix.SetRows(a3.XDirection().XYZ(), a3.YDirection().XYZ(), a3.Direction().XYZ());
        _loc = a3.Location().XYZ();
        _loc.Multiply(_matrix);
        _loc.Reverse();
    }

    internal void SetTransformation(in Quaternion3 r, in Vector3D t)
    {
        _shape = CompoundTrsf;
        _scale = 1.0;
        _loc = t.XYZ();
        _matrix = r.GetMatrix();
    }

    internal void SetDisplacement(in CoordinateSystem fromA1, in CoordinateSystem toA2)
    {
        _shape = CompoundTrsf;
        _scale = 1.0;
        // matrix from ToA2 to XOY
        _matrix.SetCol(1, toA2.XDirection().XYZ());
        _matrix.SetCol(2, toA2.YDirection().XYZ());
        _matrix.SetCol(3, toA2.Direction().XYZ());
        _loc = toA2.Location().XYZ();

        // matrix XOY to FromA1
        Xyz xDir = fromA1.XDirection().XYZ();
        Xyz yDir = fromA1.YDirection().XYZ();
        Xyz zDir = fromA1.Direction().XYZ();

        Matrix3x3 ma1 = new(xDir, yDir, zDir);
        ma1.Transpose();
        Xyz ma1loc = fromA1.Location().XYZ();
        ma1loc.Multiply(ma1);
        ma1loc.Reverse();

        // matrix * MA1
        ma1loc.Multiply(_matrix);
        _loc.Add(ma1loc);
        _matrix.Multiply(ma1);
    }

    internal void SetTranslation(in Vector3D v)
    {
        _shape = global::Kerf.Geometry.TransformForm.Translation;
        _scale = 1.0;
        _matrix.SetIdentity();
        _loc = v.XYZ();
    }

    internal void SetTranslation(in Point3D p1, in Point3D p2)
    {
        _shape = global::Kerf.Geometry.TransformForm.Translation;
        _scale = 1.0;
        _matrix.SetIdentity();
        _loc = p2.XYZ().Subtracted(p1.XYZ());
    }

    internal void SetTranslationPart(in Vector3D v)
    {
        _loc = v.XYZ();
        bool locnull = _loc.SquareModulus() < Constants.Resolution();

        switch (_shape)
        {
            case Identity:
                if (!locnull)
                {
                    _shape = global::Kerf.Geometry.TransformForm.Translation;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Translation:
                if (locnull)
                {
                    _shape = Identity;
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
                    _shape = CompoundTrsf;
                }

                break;
        }
    }

    internal void SetScaleFactor(double s)
    {
        Standard_ConstructionError_Raise_if(Math.Abs(s) <= Constants.Resolution(), "gp_Trsf::SetScaleFactor");
        _scale = s;

        bool unit = Math.Abs(_scale - 1.0) <= Constants.Resolution();
        bool munit = Math.Abs(_scale + 1.0) <= Constants.Resolution();

        switch (_shape)
        {
            case Identity:
            case global::Kerf.Geometry.TransformForm.Translation:
                if (!unit)
                {
                    _shape = global::Kerf.Geometry.TransformForm.Scale;
                }

                if (munit)
                {
                    _shape = PntMirror;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Rotation:
                if (!unit)
                {
                    _shape = CompoundTrsf;
                }

                break;
            case PntMirror:
            case Ax1Mirror:
            case Ax2Mirror:
                if (!munit)
                {
                    _shape = global::Kerf.Geometry.TransformForm.Scale;
                }

                if (unit)
                {
                    _shape = Identity;
                }

                break;
            case global::Kerf.Geometry.TransformForm.Scale:
                if (unit)
                {
                    _shape = Identity;
                }

                if (munit)
                {
                    _shape = PntMirror;
                }

                break;
            case CompoundTrsf:
                break;
            case Other:
                break;
        }
    }

    internal void SetForm(TransformForm p) => _shape = p;

    internal void SetValues(double a11, double a12, double a13, double a14,
                            double a21, double a22, double a23, double a24,
                            double a31, double a32, double a33, double a34)
    {
        Xyz col1 = new(a11, a21, a31);
        Xyz col2 = new(a12, a22, a32);
        Xyz col3 = new(a13, a23, a33);
        Xyz col4 = new(a14, a24, a34);
        // compute the determinant
        Matrix3x3 m = new(col1, col2, col3);
        double s = m.Determinant();
        Standard_ConstructionError_Raise_if(Math.Abs(s) < Constants.Resolution(), "gp_Trsf::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Pow(s, 1.0 / 3.0);
        }
        else
        {
            s = -Math.Pow(-s, 1.0 / 3.0);
        }

        m.Divide(s);

        _scale = s;
        _shape = CompoundTrsf;

        _matrix = m;
        Orthogonalize();

        _loc = col4;
    }

    internal void Invert()
    {
        // -1
        // X' = scale * R * X + T  =>  X = (R / scale) * (X' - T)
        if (_shape == Identity)
        {
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation || _shape == PntMirror)
        {
            _loc.Reverse();
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            Standard_ConstructionError_Raise_if(Math.Abs(_scale) <= Constants.Resolution(), "gp_Trsf::Invert() - transformation has zero scale");
            _scale = 1.0 / _scale;
            _loc.Multiply(-_scale);
        }
        else
        {
            Standard_ConstructionError_Raise_if(Math.Abs(_scale) <= Constants.Resolution(), "gp_Trsf::Invert() - transformation has zero scale");
            _scale = 1.0 / _scale;
            _matrix.Transpose();
            _loc.Multiply(_matrix);
            _loc.Multiply(-_scale);
        }
    }

    internal void Multiply(in Transform3D t)
    {
        if (t._shape == Identity)
        {
        }
        else if (_shape == Identity)
        {
            _shape = t._shape;
            _scale = t._scale;
            _loc = t._loc;
            _matrix = t._matrix;
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Rotation && t._shape == global::Kerf.Geometry.TransformForm.Rotation)
        {
            if (t._loc.X() != 0.0 || t._loc.Y() != 0.0 || t._loc.Z() != 0.0)
            {
                _loc.Add(t._loc.Multiplied(_matrix));
            }

            _matrix.Multiply(t._matrix);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Add(t._loc);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale && t._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            _loc.Add(t._loc.Multiplied(_scale));
            _scale = _scale * t._scale;
        }
        else if (_shape == PntMirror && t._shape == PntMirror)
        {
            _scale = 1.0;
            _shape = global::Kerf.Geometry.TransformForm.Translation;
            _loc.Add(t._loc.Reversed());
        }
        else if (_shape == Ax1Mirror && t._shape == Ax1Mirror)
        {
            _shape = global::Kerf.Geometry.TransformForm.Rotation;
            _loc.Add(t._loc.Multiplied(_matrix));
            _matrix.Multiply(t._matrix);
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror || _shape == Ax2Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xyz tloc = t._loc;
            tloc.Multiply(_matrix);
            if (_scale != 1.0)
            {
                tloc.Multiply(_scale);
            }

            _loc.Add(tloc);
        }
        else if ((_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xyz tloc = t._loc;
            tloc.Multiply(_scale);
            _loc.Add(tloc);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror))
        {
            _shape = CompoundTrsf;
            _scale = t._scale;
            _loc.Add(t._loc);
            _matrix = t._matrix;
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            _shape = t._shape;
            _loc.Add(t._loc);
            _scale = t._scale;
        }
        else if ((_shape == PntMirror || _shape == global::Kerf.Geometry.TransformForm.Scale) && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            _shape = CompoundTrsf;
            Xyz tloc = t._loc;
            tloc.Multiply(_scale);
            _loc.Add(tloc);
            _scale = _scale * t._scale;
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror || _shape == Ax2Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            _shape = CompoundTrsf;
            Xyz tloc = t._loc;
            if (_scale == 1.0)
            {
                _scale = t._scale;
                tloc.Multiply(_matrix);
            }
            else
            {
                tloc.Multiply(_matrix);
                tloc.Multiply(_scale);
                _scale = _scale * t._scale;
            }

            _loc.Add(tloc);
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror)
                 && (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror))
        {
            _shape = CompoundTrsf;
            Xyz tloc = t._loc;
            tloc.Multiply(_scale);
            _loc.Add(tloc);
            _scale = _scale * t._scale;
            _matrix = t._matrix;
        }
        else
        {
            _shape = CompoundTrsf;
            Xyz tloc = t._loc;
            tloc.Multiply(_matrix);
            if (_scale != 1.0)
            {
                tloc.Multiply(_scale);
                _scale = _scale * t._scale;
            }
            else
            {
                _scale = t._scale;
            }

            _loc.Add(tloc);
            _matrix.Multiply(t._matrix);
        }
    }

    internal void PreMultiply(in Transform3D t)
    {
        if (t._shape == Identity)
        {
        }
        else if (_shape == Identity)
        {
            _shape = t._shape;
            _scale = t._scale;
            _loc = t._loc;
            _matrix = t._matrix;
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Rotation && t._shape == global::Kerf.Geometry.TransformForm.Rotation)
        {
            _loc.Multiply(t._matrix);
            _loc.Add(t._loc);
            _matrix.PreMultiply(t._matrix);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Add(t._loc);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale && t._shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            _loc.Multiply(t._scale);
            _loc.Add(t._loc);
            _scale = _scale * t._scale;
        }
        else if (_shape == PntMirror && t._shape == PntMirror)
        {
            _scale = 1.0;
            _shape = global::Kerf.Geometry.TransformForm.Translation;
            _loc.Reverse();
            _loc.Add(t._loc);
        }
        else if (_shape == Ax1Mirror && t._shape == Ax1Mirror)
        {
            _shape = global::Kerf.Geometry.TransformForm.Rotation;
            _loc.Multiply(t._matrix);
            _loc.Add(t._loc);
            _matrix.PreMultiply(t._matrix);
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror || _shape == Ax2Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Add(t._loc);
        }
        else if ((_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Add(t._loc);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror))
        {
            _shape = CompoundTrsf;
            _matrix = t._matrix;
            if (t._scale == 1.0)
            {
                _loc.Multiply(t._matrix);
            }
            else
            {
                _scale = t._scale;
                _loc.Multiply(_matrix);
                _loc.Multiply(_scale);
            }

            _loc.Add(t._loc);
        }
        else if ((t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror) && _shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Multiply(t._scale);
            _loc.Add(t._loc);
            _scale = t._scale;
            _shape = t._shape;
        }
        else if ((_shape == PntMirror || _shape == global::Kerf.Geometry.TransformForm.Scale) && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            _shape = CompoundTrsf;
            _loc.Multiply(t._scale);
            _loc.Add(t._loc);
            _scale = _scale * t._scale;
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror || _shape == Ax2Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            _shape = CompoundTrsf;
            _loc.Multiply(t._scale);
            _loc.Add(t._loc);
            _scale = _scale * t._scale;
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror || t._shape == Ax2Mirror)
                 && (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror))
        {
            _shape = CompoundTrsf;
            _matrix = t._matrix;
            if (t._scale == 1.0)
            {
                _loc.Multiply(t._matrix);
            }
            else
            {
                _loc.Multiply(_matrix);
                _loc.Multiply(t._scale);
                _scale = t._scale * _scale;
            }

            _loc.Add(t._loc);
        }
        else
        {
            _shape = CompoundTrsf;
            _loc.Multiply(t._matrix);
            if (t._scale != 1.0)
            {
                _loc.Multiply(t._scale);
                _scale = _scale * t._scale;
            }

            _loc.Add(t._loc);
            _matrix.PreMultiply(t._matrix);
        }
    }

    internal void Power(int n)
    {
        if (_shape == Identity)
        {
        }
        else
        {
            if (n == 0)
            {
                _scale = 1.0;
                _shape = Identity;
                _matrix.SetIdentity();
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
                if (n < 0)
                {
                    Invert();
                }

                if (_shape == global::Kerf.Geometry.TransformForm.Translation)
                {
                    int npower = Math.Abs(n) - 1;
                    Xyz temploc = _loc;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            _loc.Add(temploc);
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        temploc.Add(temploc);
                        npower = npower / 2;
                    }
                }
                else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
                {
                    int npower = Math.Abs(n) - 1;
                    Xyz temploc = _loc;
                    double tempscale = _scale;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            _loc.Add(temploc.Multiplied(_scale));
                            _scale = _scale * tempscale;
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
                else if (_shape == global::Kerf.Geometry.TransformForm.Rotation)
                {
                    int npower = Math.Abs(n) - 1;
                    Matrix3x3 tempmatrix = _matrix;
                    if (_loc.X() == 0.0 && _loc.Y() == 0.0 && _loc.Z() == 0.0)
                    {
                        for (;;)
                        {
                            if (IsOdd(npower))
                            {
                                _matrix.Multiply(tempmatrix);
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
                        Xyz temploc = _loc;
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
                            tempmatrix.Multiply(tempmatrix);
                            npower = npower / 2;
                        }
                    }
                }
                else if (_shape == PntMirror || _shape == Ax1Mirror || _shape == Ax2Mirror)
                {
                    if (IsEven(n))
                    {
                        _shape = Identity;
                        _scale = 1.0;
                        _matrix.SetIdentity();
                        _loc.SetX(0);
                        _loc.SetY(0);
                        _loc.SetZ(0);
                    }
                }
                else
                {
                    _shape = CompoundTrsf;
                    int npower = Math.Abs(n) - 1;
                    Xyz temploc = _loc;
                    double tempscale = _scale;
                    Matrix3x3 tempmatrix = _matrix;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            _loc.Add(temploc.Multiplied(_matrix).Multiplied(_scale));
                            _scale = _scale * tempscale;
                            _matrix.Multiply(tempmatrix);
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        tempscale = tempscale * tempscale;
                        temploc.Add(temploc.Multiplied(tempmatrix).Multiplied(tempscale));
                        tempmatrix.Multiply(tempmatrix);
                        npower = npower / 2;
                    }
                }
            }
        }
    }

    /// <summary>Makes the matrix orthogonal (Gram-Schmidt on columns, then rows).</summary>
    internal void Orthogonalize()
    {
        Matrix3x3 tm = _matrix;

        Xyz v1 = tm.Column(1);
        Xyz v2 = tm.Column(2);
        Xyz v3 = tm.Column(3);

        v1.Normalize();

        v2 -= v1 * v2.Dot(v1);
        v2.Normalize();

        v3 -= v1 * v3.Dot(v1) + v2 * v3.Dot(v2);
        v3.Normalize();

        tm.SetCols(v1, v2, v3);

        v1 = tm.Row(1);
        v2 = tm.Row(2);
        v3 = tm.Row(3);

        v1.Normalize();

        v2 -= v1 * v2.Dot(v1);
        v2.Normalize();

        v3 -= v1 * v3.Dot(v1) + v2 * v3.Dot(v2);
        v3.Normalize();

        tm.SetRows(v1, v2, v3);

        _matrix = tm;
    }

    // Copies with a mutator applied

    public readonly Transform3D WithMirror(in Point3D p)
    {
        Transform3D copy = this;
        copy.SetMirror(p);
        return copy;
    }

    public readonly Transform3D WithMirror(in Axis3D a1)
    {
        Transform3D copy = this;
        copy.SetMirror(a1);
        return copy;
    }

    public readonly Transform3D WithMirror(in Frame3D a2)
    {
        Transform3D copy = this;
        copy.SetMirror(a2);
        return copy;
    }

    public readonly Transform3D WithRotation(in Axis3D a1, double ang)
    {
        Transform3D copy = this;
        copy.SetRotation(a1, ang);
        return copy;
    }

    public readonly Transform3D WithRotation(in Quaternion3 r)
    {
        Transform3D copy = this;
        copy.SetRotation(r);
        return copy;
    }

    public readonly Transform3D WithRotationPart(in Quaternion3 r)
    {
        Transform3D copy = this;
        copy.SetRotationPart(r);
        return copy;
    }

    public readonly Transform3D WithScale(in Point3D p, double s)
    {
        Transform3D copy = this;
        copy.SetScale(p, s);
        return copy;
    }

    public readonly Transform3D WithDisplacement(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2)
    {
        Transform3D copy = this;
        copy.SetDisplacement(fromSystem1, toSystem2);
        return copy;
    }

    public readonly Transform3D WithTransformation(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2)
    {
        Transform3D copy = this;
        copy.SetTransformation(fromSystem1, toSystem2);
        return copy;
    }

    public readonly Transform3D WithTransformation(in CoordinateSystem toSystem)
    {
        Transform3D copy = this;
        copy.SetTransformation(toSystem);
        return copy;
    }

    public readonly Transform3D WithTransformation(in Quaternion3 r, in Vector3D t)
    {
        Transform3D copy = this;
        copy.SetTransformation(r, t);
        return copy;
    }

    public readonly Transform3D WithTranslation(in Vector3D v)
    {
        Transform3D copy = this;
        copy.SetTranslation(v);
        return copy;
    }

    public readonly Transform3D WithTranslation(in Point3D p1, in Point3D p2)
    {
        Transform3D copy = this;
        copy.SetTranslation(p1, p2);
        return copy;
    }

    public readonly Transform3D WithTranslationPart(in Vector3D v)
    {
        Transform3D copy = this;
        copy.SetTranslationPart(v);
        return copy;
    }

    public readonly Transform3D WithScaleFactor(double s)
    {
        Transform3D copy = this;
        copy.SetScaleFactor(s);
        return copy;
    }

    public readonly Transform3D WithForm(TransformForm p)
    {
        Transform3D copy = this;
        copy.SetForm(p);
        return copy;
    }

    public readonly Transform3D WithValues(double a11, double a12, double a13, double a14,
                                       double a21, double a22, double a23, double a24,
                                       double a31, double a32, double a33, double a34)
    {
        Transform3D copy = this;
        copy.SetValues(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);
        return copy;
    }

    // Queries

    /// <summary>True when the determinant of the vectorial part is negative (the transformation mirrors).</summary>
    public readonly bool IsNegative => _scale < 0.0;

    public readonly TransformForm Form() => _shape;

    public readonly double ScaleFactor() => _scale;

    public readonly Xyz TranslationPart() => _loc;

    /// <summary>The rotation as an axis and an angle. Always true, as in OCCT.</summary>
    public readonly bool GetRotation(out Xyz axis, out double angle)
    {
        Quaternion3 q = GetRotation();
        q.GetVectorAndAngle(out Vector3D vec, out angle);
        axis = vec.XYZ();
        return true;
    }

    public readonly Quaternion3 GetRotation() => new(_matrix);

    /// <summary>The matrix multiplied by the scale factor.</summary>
    public readonly Matrix3x3 VectorialPart()
    {
        if (_scale == 1.0)
        {
            return _matrix;
        }

        Matrix3x3 m = _matrix;
        if (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror)
        {
            m.SetDiagonal(_scale * m._m00, _scale * m._m11, _scale * m._m22);
        }
        else
        {
            m.Multiply(_scale);
        }

        return m;
    }

    /// <summary>The matrix without the scale factor.</summary>
    public readonly Matrix3x3 HVectorialPart() => _matrix;

    /// <summary>The coefficient at a row (1 to 3) and column (1 to 4) of the 3x4 matrix, scale included.</summary>
    public readonly double Value(int row, int col)
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
        Transform3D t = this;
        t.Invert();
        return t;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly Transform3D Multiplied(in Transform3D t)
    {
        Transform3D tresult = this;
        tresult.Multiply(t);
        return tresult;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly Transform3D PreMultiplied(in Transform3D t)
    {
        Transform3D tresult = this;
        tresult.PreMultiply(t);
        return tresult;
    }

    public readonly Transform3D Powered(int n)
    {
        Transform3D t = this;
        t.Power(n);
        return t;
    }

    public readonly void Transforms(ref double x, ref double y, ref double z)
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

    public readonly void Transforms(ref Xyz coord)
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

    public static Transform3D operator *(in Transform3D left, in Transform3D right) => left.Multiplied(right);

    public readonly bool Equals(Transform3D other)
        => _scale.Equals(other._scale) && _shape == other._shape && _matrix.Equals(other._matrix) && _loc.Equals(other._loc);

    public override readonly bool Equals(object? obj) => obj is Transform3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_scale, _shape, _matrix, _loc);

    public static bool operator ==(in Transform3D left, in Transform3D right) => left.Equals(right);

    public static bool operator !=(in Transform3D left, in Transform3D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Trsf({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
