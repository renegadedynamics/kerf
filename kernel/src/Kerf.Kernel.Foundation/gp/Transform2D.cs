// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
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
public partial struct Transform2D : IEquatable<Transform2D>
{
    internal double _scale;
    internal TransformForm _shape;
    internal Matrix2x2 _matrix;
    internal Xy _loc;

    public Transform2D()
    {
        _scale = 1.0;
        _shape = Identity;
        _matrix = new Matrix2x2();
        _loc = new Xy(0.0, 0.0);
        _matrix.SetIdentity();
    }

    /// <summary>The 2D part of a 3D transformation: its translation and the upper-left 2x2 of its matrix.</summary>
    public Transform2D(in Transform3D t)
    {
        _scale = t.ScaleFactor();
        _shape = t.Form();
        _matrix = new Matrix2x2();
        _loc = new Xy(t.TranslationPart().X(), t.TranslationPart().Y());
        Matrix3x3 m = t.HVectorialPart();
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

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetMirror(in Point2D p)
    {
        _shape = PntMirror;
        _scale = -1.0;
        _matrix.SetIdentity();
        _loc = p.XY();
        _loc.Multiply(2.0);
    }

    internal void SetMirror(in Axis2D a)
    {
        _shape = Ax1Mirror;
        _scale = -1.0;
        Direction2D v = a.Direction();
        Point2D p = a.Location();
        double vx = v.X();
        double vy = v.Y();
        double x0 = p.X();
        double y0 = p.Y();
        _matrix.SetCol(1, new Xy(1.0 - 2.0 * vx * vx, -2.0 * vx * vy));
        _matrix.SetCol(2, new Xy(-2.0 * vx * vy, 1.0 - 2.0 * vy * vy));

        _loc.SetCoord(-2.0 * ((vx * vx - 1.0) * x0 + (vx * vy * y0)),
                     -2.0 * ((vx * vy * x0) + (vy * vy - 1.0) * y0));
    }

    internal void SetRotation(in Point2D p, double ang)
    {
        _shape = global::Kerf.Geometry.TransformForm.Rotation;
        _scale = 1.0;
        _loc = p.XY();
        _loc.Reverse();
        _matrix.SetRotation(ang);
        _loc.Multiply(_matrix);
        _loc.Add(p.XY());
    }

    internal void SetScale(in Point2D p, double s)
    {
        _shape = global::Kerf.Geometry.TransformForm.Scale;
        _scale = s;
        _matrix.SetIdentity();
        _loc = p.XY();
        _loc.Multiply(1.0 - s);
    }

    internal void SetTransformation(in Axis2D fromA1, in Axis2D toA2)
    {
        _shape = CompoundTrsf;
        _scale = 1.0;
        // matrix from XOY to ToA2
        Xy v1 = toA2.Direction().XY();
        Xy v2 = new(-v1.Y(), v1.X());
        _matrix.SetCol(1, v1);
        _matrix.SetCol(2, v2);
        _loc = toA2.Location().XY();
        _matrix.Transpose();
        _loc.Multiply(_matrix);
        _loc.Reverse();
        // matrix FromA1 to XOY
        Xy v3 = fromA1.Direction().XY();
        Xy v4 = new(-v3.Y(), v3.X());
        Matrix2x2 ma1 = new(v3, v4);
        Xy ma1loc = fromA1.Location().XY();
        // matrix * MA1 => FromA1 ToA2
        ma1loc.Multiply(_matrix);
        _loc.Add(ma1loc);
        _matrix.Multiply(ma1);
    }

    internal void SetTransformation(in Axis2D a)
    {
        _shape = CompoundTrsf;
        _scale = 1.0;
        Xy v1 = a.Direction().XY();
        Xy v2 = new(-v1.Y(), v1.X());
        _matrix.SetCol(1, v1);
        _matrix.SetCol(2, v2);
        _loc = a.Location().XY();
        _matrix.Transpose();
        _loc.Multiply(_matrix);
        _loc.Reverse();
    }

    internal void SetTranslation(in Vector2D v)
    {
        _shape = global::Kerf.Geometry.TransformForm.Translation;
        _scale = 1.0;
        _matrix.SetIdentity();
        _loc = v.XY();
    }

    internal void SetTranslation(in Point2D p1, in Point2D p2)
    {
        _shape = global::Kerf.Geometry.TransformForm.Translation;
        _scale = 1.0;
        _matrix.SetIdentity();
        _loc = (p2.XY()).Subtracted(p1.XY());
    }

    internal void SetTranslationPart(in Vector2D v)
    {
        _loc = v.XY();
        if (Math.Abs(_loc.X()) <= Constants.Resolution() && Math.Abs(_loc.Y()) <= Constants.Resolution())
        {
            if (_shape == Identity || _shape == PntMirror || _shape == global::Kerf.Geometry.TransformForm.Scale || _shape == global::Kerf.Geometry.TransformForm.Rotation
                || _shape == Ax1Mirror)
            {
            }
            else if (_shape == global::Kerf.Geometry.TransformForm.Translation)
            {
                _shape = Identity;
            }
            else
            {
                _shape = CompoundTrsf;
            }
        }
        else
        {
            if (_shape == global::Kerf.Geometry.TransformForm.Translation || _shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror)
            {
            }
            else if (_shape == Identity)
            {
                _shape = global::Kerf.Geometry.TransformForm.Translation;
            }
            else
            {
                _shape = CompoundTrsf;
            }
        }
    }

    internal void SetScaleFactor(double s)
    {
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
                if (_shape == Identity || _shape == global::Kerf.Geometry.TransformForm.Rotation)
                {
                }
                else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
                {
                    _shape = Identity;
                }
                else if (_shape == PntMirror)
                {
                    _shape = global::Kerf.Geometry.TransformForm.Translation;
                }
                else
                {
                    _shape = CompoundTrsf;
                }
            }
            else
            {
                if (_shape == Identity || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == global::Kerf.Geometry.TransformForm.Scale)
                {
                }
                else if (_shape == PntMirror)
                {
                    _shape = global::Kerf.Geometry.TransformForm.Translation;
                }
                else
                {
                    _shape = CompoundTrsf;
                }
            }
        }
        else if (s == -1)
        {
            if (_shape == PntMirror || _shape == Ax1Mirror)
            {
            }
            else if (_shape == Identity || _shape == global::Kerf.Geometry.TransformForm.Scale)
            {
                _shape = PntMirror;
            }
            else
            {
                _shape = CompoundTrsf;
            }
        }
        else
        {
            if (_shape == global::Kerf.Geometry.TransformForm.Scale)
            {
            }
            else if (_shape == Identity || _shape == global::Kerf.Geometry.TransformForm.Translation || _shape == PntMirror)
            {
                _shape = global::Kerf.Geometry.TransformForm.Scale;
            }
            else
            {
                _shape = CompoundTrsf;
            }
        }

        _scale = s;
    }

    internal void SetValues(double a11, double a12, double a13, double a21, double a22, double a23)
    {
        Xy col1 = new(a11, a21);
        Xy col2 = new(a12, a22);
        Xy col3 = new(a13, a23);
        // compute the determinant
        Matrix2x2 m = new(col1, col2);
        double s = m.Determinant();
        Standard_ConstructionError_Raise_if(Math.Abs(s) < Constants.Resolution(), "gp_Trsf2d::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Sqrt(s);
        }
        else
        {
            s = Math.Sqrt(-s);
        }

        m.Divide(s);

        _scale = s;
        _shape = CompoundTrsf;

        _matrix = m;
        Orthogonalize();

        _loc = col3;
    }

    internal void Invert()
    {
        //                                    -1
        //  X' = scale * R * X + T  =>  X = (R  / scale)  * ( X' - T)
        if (_shape == Identity)
        {
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation || _shape == PntMirror)
        {
            _loc.Reverse();
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Scale)
        {
            Standard_ConstructionError_Raise_if(Math.Abs(_scale) <= Constants.Resolution(), "gp_Trsf2d::Invert() - transformation has zero scale");
            _scale = 1.0 / _scale;
            _loc.Multiply(-_scale);
        }
        else
        {
            Standard_ConstructionError_Raise_if(Math.Abs(_scale) <= Constants.Resolution(), "gp_Trsf2d::Invert() - transformation has zero scale");
            _scale = 1.0 / _scale;
            _matrix.Transpose();
            _loc.Multiply(_matrix);
            _loc.Multiply(-_scale);
        }
    }

    internal void Multiply(in Transform2D t)
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
            // OCCT tests this loc, where gp_Trsf tests T.loc: when this rotation has no translation, T's is
            // dropped (docs/occt-issues.md); kept so results match OCCT
            if (_loc.X() != 0.0 || _loc.Y() != 0.0)
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
            Xy tloc = t._loc;
            tloc.Multiply(_matrix);
            tloc.Multiply(_scale);
            _scale = _scale * t._scale;
            _loc.Add(tloc);
            _matrix.Multiply(t._matrix);
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xy tloc = t._loc;
            tloc.Multiply(_matrix);
            if (_scale != 1.0)
            {
                tloc.Multiply(_scale);
            }

            _loc.Add(tloc);
        }
        else if ((_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            Xy tloc = t._loc;
            tloc.Multiply(_scale);
            _loc.Add(tloc);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror))
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
        else if ((_shape == PntMirror || _shape == global::Kerf.Geometry.TransformForm.Scale)
                 && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            _shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(_scale);
            _loc.Add(tloc);
            _scale = _scale * t._scale;
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            _shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(_matrix);
            if (_scale == 1.0)
            {
                _scale = t._scale;
            }
            else
            {
                tloc.Multiply(_scale);
                _scale = _scale * t._scale;
            }

            _loc.Add(tloc);
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror)
                 && (_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror))
        {
            _shape = CompoundTrsf;
            Xy tloc = t._loc;
            tloc.Multiply(_scale);
            _scale = _scale * t._scale;
            _loc.Add(tloc);
            _matrix = t._matrix;
        }
        else
        {
            _shape = CompoundTrsf;
            Xy tloc = t._loc;
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

    internal void PreMultiply(in Transform2D t)
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
            _loc.Multiply(t._scale);
            _scale = _scale * t._scale;
            _loc.Add(t._loc);
            _matrix.PreMultiply(t._matrix);
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror)
                 && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Add(t._loc);
        }
        else if ((_shape == global::Kerf.Geometry.TransformForm.Scale || _shape == PntMirror) && t._shape == global::Kerf.Geometry.TransformForm.Translation)
        {
            _loc.Add(t._loc);
        }
        else if (_shape == global::Kerf.Geometry.TransformForm.Translation
                 && (t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror))
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
        else if ((_shape == PntMirror || _shape == global::Kerf.Geometry.TransformForm.Scale)
                 && (t._shape == PntMirror || t._shape == global::Kerf.Geometry.TransformForm.Scale))
        {
            _shape = CompoundTrsf;
            _loc.Multiply(t._scale);
            _loc.Add(t._loc);
            _scale = _scale * t._scale;
        }
        else if ((_shape == CompoundTrsf || _shape == global::Kerf.Geometry.TransformForm.Rotation || _shape == Ax1Mirror)
                 && (t._shape == global::Kerf.Geometry.TransformForm.Scale || t._shape == PntMirror))
        {
            _shape = CompoundTrsf;
            _loc.Multiply(t._scale);
            _loc.Add(t._loc);
            _scale = _scale * t._scale;
        }
        else if ((t._shape == CompoundTrsf || t._shape == global::Kerf.Geometry.TransformForm.Rotation || t._shape == Ax1Mirror)
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

                if (_shape == global::Kerf.Geometry.TransformForm.Translation)
                {
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Xy temploc = _loc;
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
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Xy temploc = _loc;
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
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    Matrix2x2 tempmatrix = _matrix;
                    if (_loc.X() == 0.0 && _loc.Y() == 0.0)
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

                            // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
                            tempmatrix.Multiply(tempmatrix);
                            npower = npower / 2;
                        }
                    }
                    else
                    {
                        Xy temploc = _loc;
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
                }
                else if (_shape == PntMirror || _shape == Ax1Mirror)
                {
                    if (IsEven(n))
                    {
                        _shape = Identity;
                        _scale = 1.0;
                        _matrix.SetIdentity();
                        _loc.SetCoord(0.0, 0.0);
                    }
                }
                else
                {
                    _shape = CompoundTrsf;
                    int npower = n;
                    if (npower < 0)
                    {
                        npower = -npower;
                    }

                    npower--;
                    // OCCT folds the scale into the diagonal only, then scales again in the loop
                    // (docs/occt-issues.md); kept so results match OCCT
                    _matrix.SetDiagonal(_scale * _matrix.Value(1, 1), _scale * _matrix.Value(2, 2));
                    Xy temploc = _loc;
                    double tempscale = _scale;
                    Matrix2x2 tempmatrix = _matrix;
                    for (;;)
                    {
                        if (IsOdd(npower))
                        {
                            _loc.Add((temploc.Multiplied(_matrix)).Multiplied(_scale));
                            _scale = _scale * tempscale;
                            _matrix.Multiply(tempmatrix);
                        }

                        if (npower == 1)
                        {
                            break;
                        }

                        tempscale = tempscale * tempscale;
                        temploc.Add((temploc.Multiplied(tempmatrix)).Multiplied(tempscale));
                        // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
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
        Matrix2x2 tm = _matrix;

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

        _matrix = tm;
    }

    // Copies with a mutator applied

    public readonly Transform2D WithMirror(in Point2D p)
    {
        Transform2D copy = this;
        copy.SetMirror(p);
        return copy;
    }

    public readonly Transform2D WithMirror(in Axis2D a)
    {
        Transform2D copy = this;
        copy.SetMirror(a);
        return copy;
    }

    public readonly Transform2D WithRotation(in Point2D p, double ang)
    {
        Transform2D copy = this;
        copy.SetRotation(p, ang);
        return copy;
    }

    public readonly Transform2D WithScale(in Point2D p, double s)
    {
        Transform2D copy = this;
        copy.SetScale(p, s);
        return copy;
    }

    public readonly Transform2D WithTransformation(in Axis2D fromSystem1, in Axis2D toSystem2)
    {
        Transform2D copy = this;
        copy.SetTransformation(fromSystem1, toSystem2);
        return copy;
    }

    public readonly Transform2D WithTransformation(in Axis2D toSystem)
    {
        Transform2D copy = this;
        copy.SetTransformation(toSystem);
        return copy;
    }

    public readonly Transform2D WithTranslation(in Vector2D v)
    {
        Transform2D copy = this;
        copy.SetTranslation(v);
        return copy;
    }

    public readonly Transform2D WithTranslation(in Point2D p1, in Point2D p2)
    {
        Transform2D copy = this;
        copy.SetTranslation(p1, p2);
        return copy;
    }

    public readonly Transform2D WithTranslationPart(in Vector2D v)
    {
        Transform2D copy = this;
        copy.SetTranslationPart(v);
        return copy;
    }

    public readonly Transform2D WithScaleFactor(double s)
    {
        Transform2D copy = this;
        copy.SetScaleFactor(s);
        return copy;
    }

    public readonly Transform2D WithValues(double a11, double a12, double a13, double a21, double a22, double a23)
    {
        Transform2D copy = this;
        copy.SetValues(a11, a12, a13, a21, a22, a23);
        return copy;
    }

    // Queries

    /// <summary>True when the determinant of the matrix is negative (the transformation mirrors).</summary>
    public readonly bool IsNegative => (_matrix.Determinant() < 0.0);

    public readonly TransformForm Form() => _shape;

    public readonly double ScaleFactor() => _scale;

    public readonly Xy TranslationPart() => _loc;

    /// <summary>The matrix multiplied by the scale factor.</summary>
    public readonly Matrix2x2 VectorialPart()
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

    /// <summary>The matrix without the scale factor.</summary>
    public readonly Matrix2x2 HVectorialPart() => _matrix;

    /// <summary>The angle of the rotation part, in ]-PI, PI].</summary>
    public readonly double RotationPart() => Math.Atan2(_matrix.Value(2, 1), _matrix.Value(1, 1));

    /// <summary>The coefficient at a row (1 to 2) and column (1 to 3) of the 2x3 matrix, scale included.</summary>
    public readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2 || col < 1 || col > 3, " ");
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
        Transform2D t = this;
        t.Invert();
        return t;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="t"/> first.</summary>
    public readonly Transform2D Multiplied(in Transform2D t)
    {
        Transform2D tresult = this;
        tresult.Multiply(t);
        return tresult;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly Transform2D PreMultiplied(in Transform2D t)
    {
        Transform2D tresult = this;
        tresult.PreMultiply(t);
        return tresult;
    }

    public readonly Transform2D Powered(int n)
    {
        Transform2D t = this;
        t.Power(n);
        return t;
    }

    public readonly void Transforms(ref double x, ref double y)
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

    public readonly void Transforms(ref Xy coord)
    {
        coord.Multiply(_matrix);
        if (_scale != 1.0)
        {
            coord.Multiply(_scale);
        }

        coord.Add(_loc);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref Xy)"/>.</summary>
    public readonly Xy Transformed(in Xy coord)
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

    public override readonly string ToString() => $"gp_Trsf2d({_shape}, scale {_scale}, {_matrix}, {_loc})";
}
