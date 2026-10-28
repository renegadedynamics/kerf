// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// A non-persistent transformation in 2D space: a rigid motion, mirror or uniform scale, or a combination.
/// It maps P to <c>scale * matrix * P + loc</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Trsf2d()</c> is the identity, as in OCCT; <c>default(gp_Trsf2d)</c> is not valid. Build one with the
/// static factories (<see cref="Rotation(in gp_Pnt2d, double)"/>, <see cref="Translation(in gp_Vec2d)"/>, ...) or the
/// <c>With</c> methods, which mirror OCCT's <c>Set</c> methods.
/// </remarks>
public partial struct gp_Trsf2d : IEquatable<gp_Trsf2d>
{
    internal double scale;
    internal gp_TrsfForm shape;
    internal gp_Mat2d matrix;
    internal gp_XY loc;

    public gp_Trsf2d()
    {
        scale = 1.0;
        shape = gp_Identity;
        matrix = new gp_Mat2d();
        loc = new gp_XY(0.0, 0.0);
        matrix.SetIdentity();
    }

    /// <summary>The 2D part of a 3D transformation: its translation and the upper-left 2x2 of its matrix.</summary>
    public gp_Trsf2d(in gp_Trsf theT)
    {
        scale = theT.ScaleFactor();
        shape = theT.Form();
        matrix = new gp_Mat2d();
        loc = new gp_XY(theT.TranslationPart().X(), theT.TranslationPart().Y());
        gp_Mat M = theT.HVectorialPart();
        matrix.ChangeValue(1, 1) = M[1, 1];
        matrix.ChangeValue(1, 2) = M[1, 2];
        matrix.ChangeValue(2, 1) = M[2, 1];
        matrix.ChangeValue(2, 2) = M[2, 2];
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The central symmetry about a point.</summary>
    public static gp_Trsf2d Mirror(in gp_Pnt2d theP) => new gp_Trsf2d().WithMirror(theP);

    /// <summary>The reflection in an axis.</summary>
    public static gp_Trsf2d Mirror(in gp_Ax2d theA) => new gp_Trsf2d().WithMirror(theA);

    public static gp_Trsf2d Rotation(in gp_Pnt2d theP, double theAng) => new gp_Trsf2d().WithRotation(theP, theAng);

    public static gp_Trsf2d Scale(in gp_Pnt2d theP, double theS) => new gp_Trsf2d().WithScale(theP, theS);

    public static gp_Trsf2d Transformation(in gp_Ax2d theFromSystem1, in gp_Ax2d theToSystem2)
        => new gp_Trsf2d().WithTransformation(theFromSystem1, theToSystem2);

    public static gp_Trsf2d Transformation(in gp_Ax2d theToSystem) => new gp_Trsf2d().WithTransformation(theToSystem);

    public static gp_Trsf2d Translation(in gp_Vec2d theV) => new gp_Trsf2d().WithTranslation(theV);

    public static gp_Trsf2d Translation(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => new gp_Trsf2d().WithTranslation(theP1, theP2);

    public static gp_Trsf2d Values(double a11, double a12, double a13, double a21, double a22, double a23)
        => new gp_Trsf2d().WithValues(a11, a12, a13, a21, a22, a23);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetMirror(in gp_Pnt2d theP)
    {
        shape = gp_PntMirror;
        scale = -1.0;
        matrix.SetIdentity();
        loc = theP.XY();
        loc.Multiply(2.0);
    }

    internal void SetMirror(in gp_Ax2d A)
    {
        shape = gp_Ax1Mirror;
        scale = -1.0;
        gp_Dir2d V = A.Direction();
        gp_Pnt2d P = A.Location();
        double VX = V.X();
        double VY = V.Y();
        double X0 = P.X();
        double Y0 = P.Y();
        matrix.SetCol(1, new gp_XY(1.0 - 2.0 * VX * VX, -2.0 * VX * VY));
        matrix.SetCol(2, new gp_XY(-2.0 * VX * VY, 1.0 - 2.0 * VY * VY));

        loc.SetCoord(-2.0 * ((VX * VX - 1.0) * X0 + (VX * VY * Y0)),
                     -2.0 * ((VX * VY * X0) + (VY * VY - 1.0) * Y0));
    }

    internal void SetRotation(in gp_Pnt2d theP, double theAng)
    {
        shape = gp_Rotation;
        scale = 1.0;
        loc = theP.XY();
        loc.Reverse();
        matrix.SetRotation(theAng);
        loc.Multiply(matrix);
        loc.Add(theP.XY());
    }

    internal void SetScale(in gp_Pnt2d theP, double theS)
    {
        shape = gp_Scale;
        scale = theS;
        matrix.SetIdentity();
        loc = theP.XY();
        loc.Multiply(1.0 - theS);
    }

    internal void SetTransformation(in gp_Ax2d FromA1, in gp_Ax2d ToA2)
    {
        shape = gp_CompoundTrsf;
        scale = 1.0;
        // matrix from XOY to ToA2
        gp_XY V1 = ToA2.Direction().XY();
        gp_XY V2 = new(-V1.Y(), V1.X());
        matrix.SetCol(1, V1);
        matrix.SetCol(2, V2);
        loc = ToA2.Location().XY();
        matrix.Transpose();
        loc.Multiply(matrix);
        loc.Reverse();
        // matrix FromA1 to XOY
        gp_XY V3 = FromA1.Direction().XY();
        gp_XY V4 = new(-V3.Y(), V3.X());
        gp_Mat2d MA1 = new(V3, V4);
        gp_XY MA1loc = FromA1.Location().XY();
        // matrix * MA1 => FromA1 ToA2
        MA1loc.Multiply(matrix);
        loc.Add(MA1loc);
        matrix.Multiply(MA1);
    }

    internal void SetTransformation(in gp_Ax2d A)
    {
        shape = gp_CompoundTrsf;
        scale = 1.0;
        gp_XY V1 = A.Direction().XY();
        gp_XY V2 = new(-V1.Y(), V1.X());
        matrix.SetCol(1, V1);
        matrix.SetCol(2, V2);
        loc = A.Location().XY();
        matrix.Transpose();
        loc.Multiply(matrix);
        loc.Reverse();
    }

    internal void SetTranslation(in gp_Vec2d theV)
    {
        shape = gp_Translation;
        scale = 1.0;
        matrix.SetIdentity();
        loc = theV.XY();
    }

    internal void SetTranslation(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        shape = gp_Translation;
        scale = 1.0;
        matrix.SetIdentity();
        loc = (theP2.XY()).Subtracted(theP1.XY());
    }

    internal void SetTranslationPart(in gp_Vec2d V)
    {
        loc = V.XY();
        if (Math.Abs(loc.X()) <= gp.Resolution() && Math.Abs(loc.Y()) <= gp.Resolution())
        {
            if (shape == gp_Identity || shape == gp_PntMirror || shape == gp_Scale || shape == gp_Rotation
                || shape == gp_Ax1Mirror)
            {
            }
            else if (shape == gp_Translation)
            {
                shape = gp_Identity;
            }
            else
            {
                shape = gp_CompoundTrsf;
            }
        }
        else
        {
            if (shape == gp_Translation || shape == gp_Scale || shape == gp_PntMirror)
            {
            }
            else if (shape == gp_Identity)
            {
                shape = gp_Translation;
            }
            else
            {
                shape = gp_CompoundTrsf;
            }
        }
    }

    internal void SetScaleFactor(double S)
    {
        if (S == 1.0)
        {
            double X = loc.X();
            if (X < 0)
            {
                X = -X;
            }

            double Y = loc.Y();
            if (Y < 0)
            {
                Y = -Y;
            }

            if (X <= gp.Resolution() && Y <= gp.Resolution())
            {
                if (shape == gp_Identity || shape == gp_Rotation)
                {
                }
                else if (shape == gp_Scale)
                {
                    shape = gp_Identity;
                }
                else if (shape == gp_PntMirror)
                {
                    shape = gp_Translation;
                }
                else
                {
                    shape = gp_CompoundTrsf;
                }
            }
            else
            {
                if (shape == gp_Identity || shape == gp_Rotation || shape == gp_Scale)
                {
                }
                else if (shape == gp_PntMirror)
                {
                    shape = gp_Translation;
                }
                else
                {
                    shape = gp_CompoundTrsf;
                }
            }
        }
        else if (S == -1)
        {
            if (shape == gp_PntMirror || shape == gp_Ax1Mirror)
            {
            }
            else if (shape == gp_Identity || shape == gp_Scale)
            {
                shape = gp_PntMirror;
            }
            else
            {
                shape = gp_CompoundTrsf;
            }
        }
        else
        {
            if (shape == gp_Scale)
            {
            }
            else if (shape == gp_Identity || shape == gp_Translation || shape == gp_PntMirror)
            {
                shape = gp_Scale;
            }
            else
            {
                shape = gp_CompoundTrsf;
            }
        }

        scale = S;
    }

    internal void SetValues(double a11, double a12, double a13, double a21, double a22, double a23)
    {
        gp_XY col1 = new(a11, a21);
        gp_XY col2 = new(a12, a22);
        gp_XY col3 = new(a13, a23);
        // compute the determinant
        gp_Mat2d M = new(col1, col2);
        double s = M.Determinant();
        Standard_ConstructionError_Raise_if(Math.Abs(s) < gp.Resolution(), "gp_Trsf2d::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Sqrt(s);
        }
        else
        {
            s = Math.Sqrt(-s);
        }

        M.Divide(s);

        scale = s;
        shape = gp_CompoundTrsf;

        matrix = M;
        Orthogonalize();

        loc = col3;
    }

    internal void Invert()
    {
        //                                    -1
        //  X' = scale * R * X + T  =>  X = (R  / scale)  * ( X' - T)
        if (shape == gp_Identity)
        {
        }
        else if (shape == gp_Translation || shape == gp_PntMirror)
        {
            loc.Reverse();
        }
        else if (shape == gp_Scale)
        {
            Standard_ConstructionError_Raise_if(Math.Abs(scale) <= gp.Resolution(), "gp_Trsf2d::Invert() - transformation has zero scale");
            scale = 1.0 / scale;
            loc.Multiply(-scale);
        }
        else
        {
            Standard_ConstructionError_Raise_if(Math.Abs(scale) <= gp.Resolution(), "gp_Trsf2d::Invert() - transformation has zero scale");
            scale = 1.0 / scale;
            matrix.Transpose();
            loc.Multiply(matrix);
            loc.Multiply(-scale);
        }
    }

    internal void Multiply(in gp_Trsf2d T)
    {
        if (T.shape == gp_Identity)
        {
        }
        else if (shape == gp_Identity)
        {
            shape = T.shape;
            scale = T.scale;
            loc = T.loc;
            matrix = T.matrix;
        }
        else if (shape == gp_Rotation && T.shape == gp_Rotation)
        {
            // OCCT tests this loc, where gp_Trsf tests T.loc: when this rotation has no translation, T's is
            // dropped (docs/occt-issues.md); kept so results match OCCT
            if (loc.X() != 0.0 || loc.Y() != 0.0)
            {
                loc.Add(T.loc.Multiplied(matrix));
            }

            matrix.Multiply(T.matrix);
        }
        else if (shape == gp_Translation && T.shape == gp_Translation)
        {
            loc.Add(T.loc);
        }
        else if (shape == gp_Scale && T.shape == gp_Scale)
        {
            loc.Add(T.loc.Multiplied(scale));
            scale = scale * T.scale;
        }
        else if (shape == gp_PntMirror && T.shape == gp_PntMirror)
        {
            scale = 1.0;
            shape = gp_Translation;
            loc.Add(T.loc.Reversed());
        }
        else if (shape == gp_Ax1Mirror && T.shape == gp_Ax1Mirror)
        {
            shape = gp_Rotation;
            gp_XY Tloc = T.loc;
            Tloc.Multiply(matrix);
            Tloc.Multiply(scale);
            scale = scale * T.scale;
            loc.Add(Tloc);
            matrix.Multiply(T.matrix);
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror)
                 && T.shape == gp_Translation)
        {
            gp_XY Tloc = T.loc;
            Tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                Tloc.Multiply(scale);
            }

            loc.Add(Tloc);
        }
        else if ((shape == gp_Scale || shape == gp_PntMirror) && T.shape == gp_Translation)
        {
            gp_XY Tloc = T.loc;
            Tloc.Multiply(scale);
            loc.Add(Tloc);
        }
        else if (shape == gp_Translation
                 && (T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror))
        {
            shape = gp_CompoundTrsf;
            scale = T.scale;
            loc.Add(T.loc);
            matrix = T.matrix;
        }
        else if (shape == gp_Translation && (T.shape == gp_Scale || T.shape == gp_PntMirror))
        {
            shape = T.shape;
            loc.Add(T.loc);
            scale = T.scale;
        }
        else if ((shape == gp_PntMirror || shape == gp_Scale)
                 && (T.shape == gp_PntMirror || T.shape == gp_Scale))
        {
            shape = gp_CompoundTrsf;
            gp_XY Tloc = T.loc;
            Tloc.Multiply(scale);
            loc.Add(Tloc);
            scale = scale * T.scale;
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror)
                 && (T.shape == gp_Scale || T.shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            gp_XY Tloc = T.loc;
            Tloc.Multiply(matrix);
            if (scale == 1.0)
            {
                scale = T.scale;
            }
            else
            {
                Tloc.Multiply(scale);
                scale = scale * T.scale;
            }

            loc.Add(Tloc);
        }
        else if ((T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror)
                 && (shape == gp_Scale || shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            gp_XY Tloc = T.loc;
            Tloc.Multiply(scale);
            scale = scale * T.scale;
            loc.Add(Tloc);
            matrix = T.matrix;
        }
        else
        {
            shape = gp_CompoundTrsf;
            gp_XY Tloc = T.loc;
            Tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                Tloc.Multiply(scale);
                scale = scale * T.scale;
            }
            else
            {
                scale = T.scale;
            }

            loc.Add(Tloc);
            matrix.Multiply(T.matrix);
        }
    }

    internal void PreMultiply(in gp_Trsf2d T)
    {
        if (T.shape == gp_Identity)
        {
        }
        else if (shape == gp_Identity)
        {
            shape = T.shape;
            scale = T.scale;
            loc = T.loc;
            matrix = T.matrix;
        }
        else if (shape == gp_Rotation && T.shape == gp_Rotation)
        {
            loc.Multiply(T.matrix);
            loc.Add(T.loc);
            matrix.PreMultiply(T.matrix);
        }
        else if (shape == gp_Translation && T.shape == gp_Translation)
        {
            loc.Add(T.loc);
        }
        else if (shape == gp_Scale && T.shape == gp_Scale)
        {
            loc.Multiply(T.scale);
            loc.Add(T.loc);
            scale = scale * T.scale;
        }
        else if (shape == gp_PntMirror && T.shape == gp_PntMirror)
        {
            scale = 1.0;
            shape = gp_Translation;
            loc.Reverse();
            loc.Add(T.loc);
        }
        else if (shape == gp_Ax1Mirror && T.shape == gp_Ax1Mirror)
        {
            shape = gp_Rotation;
            loc.Multiply(T.matrix);
            loc.Multiply(T.scale);
            scale = scale * T.scale;
            loc.Add(T.loc);
            matrix.PreMultiply(T.matrix);
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror)
                 && T.shape == gp_Translation)
        {
            loc.Add(T.loc);
        }
        else if ((shape == gp_Scale || shape == gp_PntMirror) && T.shape == gp_Translation)
        {
            loc.Add(T.loc);
        }
        else if (shape == gp_Translation
                 && (T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror))
        {
            shape = gp_CompoundTrsf;
            matrix = T.matrix;
            if (T.scale == 1.0)
            {
                loc.Multiply(T.matrix);
            }
            else
            {
                scale = T.scale;
                loc.Multiply(matrix);
                loc.Multiply(scale);
            }

            loc.Add(T.loc);
        }
        else if ((T.shape == gp_Scale || T.shape == gp_PntMirror) && shape == gp_Translation)
        {
            loc.Multiply(T.scale);
            loc.Add(T.loc);
            scale = T.scale;
            shape = T.shape;
        }
        else if ((shape == gp_PntMirror || shape == gp_Scale)
                 && (T.shape == gp_PntMirror || T.shape == gp_Scale))
        {
            shape = gp_CompoundTrsf;
            loc.Multiply(T.scale);
            loc.Add(T.loc);
            scale = scale * T.scale;
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror)
                 && (T.shape == gp_Scale || T.shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            loc.Multiply(T.scale);
            loc.Add(T.loc);
            scale = scale * T.scale;
        }
        else if ((T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror)
                 && (shape == gp_Scale || shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            matrix = T.matrix;
            if (T.scale == 1.0)
            {
                loc.Multiply(T.matrix);
            }
            else
            {
                loc.Multiply(matrix);
                loc.Multiply(T.scale);
                scale = T.scale * scale;
            }

            loc.Add(T.loc);
        }
        else
        {
            shape = gp_CompoundTrsf;
            loc.Multiply(T.matrix);
            if (T.scale != 1.0)
            {
                loc.Multiply(T.scale);
                scale = scale * T.scale;
            }

            loc.Add(T.loc);
            matrix.PreMultiply(T.matrix);
        }
    }

    internal void Power(int N)
    {
        if (shape == gp_Identity)
        {
        }
        else
        {
            if (N == 0)
            {
                scale = 1.0;
                shape = gp_Identity;
                matrix.SetIdentity();
                loc = new gp_XY(0.0, 0.0);
            }
            else if (N == 1)
            {
            }
            else if (N == -1)
            {
                Invert();
            }
            else
            {
                if (N < 0)
                {
                    Invert();
                }

                if (shape == gp_Translation)
                {
                    int Npower = N;
                    if (Npower < 0)
                    {
                        Npower = -Npower;
                    }

                    Npower--;
                    gp_XY Temploc = loc;
                    for (;;)
                    {
                        if (IsOdd(Npower))
                        {
                            loc.Add(Temploc);
                        }

                        if (Npower == 1)
                        {
                            break;
                        }

                        Temploc.Add(Temploc);
                        Npower = Npower / 2;
                    }
                }
                else if (shape == gp_Scale)
                {
                    int Npower = N;
                    if (Npower < 0)
                    {
                        Npower = -Npower;
                    }

                    Npower--;
                    gp_XY Temploc = loc;
                    double Tempscale = scale;
                    for (;;)
                    {
                        if (IsOdd(Npower))
                        {
                            loc.Add(Temploc.Multiplied(scale));
                            scale = scale * Tempscale;
                        }

                        if (Npower == 1)
                        {
                            break;
                        }

                        Temploc.Add(Temploc.Multiplied(Tempscale));
                        Tempscale = Tempscale * Tempscale;
                        Npower = Npower / 2;
                    }
                }
                else if (shape == gp_Rotation)
                {
                    int Npower = N;
                    if (Npower < 0)
                    {
                        Npower = -Npower;
                    }

                    Npower--;
                    gp_Mat2d Tempmatrix = matrix;
                    if (loc.X() == 0.0 && loc.Y() == 0.0)
                    {
                        for (;;)
                        {
                            if (IsOdd(Npower))
                            {
                                matrix.Multiply(Tempmatrix);
                            }

                            if (Npower == 1)
                            {
                                break;
                            }

                            // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
                            Tempmatrix.Multiply(Tempmatrix);
                            Npower = Npower / 2;
                        }
                    }
                    else
                    {
                        gp_XY Temploc = loc;
                        for (;;)
                        {
                            if (IsOdd(Npower))
                            {
                                loc.Add(Temploc.Multiplied(matrix));
                                matrix.Multiply(Tempmatrix);
                            }

                            if (Npower == 1)
                            {
                                break;
                            }

                            Temploc.Add(Temploc.Multiplied(Tempmatrix));
                            // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
                            Tempmatrix.Multiply(Tempmatrix);
                            Npower = Npower / 2;
                        }
                    }
                }
                else if (shape == gp_PntMirror || shape == gp_Ax1Mirror)
                {
                    if (IsEven(N))
                    {
                        shape = gp_Identity;
                        scale = 1.0;
                        matrix.SetIdentity();
                        loc.SetCoord(0.0, 0.0);
                    }
                }
                else
                {
                    shape = gp_CompoundTrsf;
                    int Npower = N;
                    if (Npower < 0)
                    {
                        Npower = -Npower;
                    }

                    Npower--;
                    // OCCT folds the scale into the diagonal only, then scales again in the loop
                    // (docs/occt-issues.md); kept so results match OCCT
                    matrix.SetDiagonal(scale * matrix.Value(1, 1), scale * matrix.Value(2, 2));
                    gp_XY Temploc = loc;
                    double Tempscale = scale;
                    gp_Mat2d Tempmatrix = matrix;
                    for (;;)
                    {
                        if (IsOdd(Npower))
                        {
                            loc.Add((Temploc.Multiplied(matrix)).Multiplied(scale));
                            scale = scale * Tempscale;
                            matrix.Multiply(Tempmatrix);
                        }

                        if (Npower == 1)
                        {
                            break;
                        }

                        Tempscale = Tempscale * Tempscale;
                        Temploc.Add((Temploc.Multiplied(Tempmatrix)).Multiplied(Tempscale));
                        // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
                        Tempmatrix.Multiply(Tempmatrix);
                        Npower = Npower / 2;
                    }
                }
            }
        }
    }

    /// <summary>Makes the matrix orthogonal (Gram-Schmidt on columns, then rows).</summary>
    internal void Orthogonalize()
    {
        gp_Mat2d aTM = matrix;

        gp_XY aV1 = aTM.Column(1);
        gp_XY aV2 = aTM.Column(2);

        aV1.Normalize();

        aV2 -= aV1 * (aV2.Dot(aV1));
        aV2.Normalize();

        aTM.SetCols(aV1, aV2);

        aV1 = aTM.Row(1);
        aV2 = aTM.Row(2);

        aV1.Normalize();

        aV2 -= aV1 * (aV2.Dot(aV1));
        aV2.Normalize();

        aTM.SetRows(aV1, aV2);

        matrix = aTM;
    }

    // Copies with a mutator applied

    public readonly gp_Trsf2d WithMirror(in gp_Pnt2d theP)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetMirror(theP);
        return aCopy;
    }

    public readonly gp_Trsf2d WithMirror(in gp_Ax2d theA)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetMirror(theA);
        return aCopy;
    }

    public readonly gp_Trsf2d WithRotation(in gp_Pnt2d theP, double theAng)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetRotation(theP, theAng);
        return aCopy;
    }

    public readonly gp_Trsf2d WithScale(in gp_Pnt2d theP, double theS)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetScale(theP, theS);
        return aCopy;
    }

    public readonly gp_Trsf2d WithTransformation(in gp_Ax2d theFromSystem1, in gp_Ax2d theToSystem2)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetTransformation(theFromSystem1, theToSystem2);
        return aCopy;
    }

    public readonly gp_Trsf2d WithTransformation(in gp_Ax2d theToSystem)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetTransformation(theToSystem);
        return aCopy;
    }

    public readonly gp_Trsf2d WithTranslation(in gp_Vec2d theV)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetTranslation(theV);
        return aCopy;
    }

    public readonly gp_Trsf2d WithTranslation(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetTranslation(theP1, theP2);
        return aCopy;
    }

    public readonly gp_Trsf2d WithTranslationPart(in gp_Vec2d theV)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetTranslationPart(theV);
        return aCopy;
    }

    public readonly gp_Trsf2d WithScaleFactor(double theS)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetScaleFactor(theS);
        return aCopy;
    }

    public readonly gp_Trsf2d WithValues(double a11, double a12, double a13, double a21, double a22, double a23)
    {
        gp_Trsf2d aCopy = this;
        aCopy.SetValues(a11, a12, a13, a21, a22, a23);
        return aCopy;
    }

    // Queries

    /// <summary>True when the determinant of the matrix is negative (the transformation mirrors).</summary>
    public readonly bool IsNegative() => (matrix.Determinant() < 0.0);

    public readonly gp_TrsfForm Form() => shape;

    public readonly double ScaleFactor() => scale;

    public readonly gp_XY TranslationPart() => loc;

    /// <summary>The matrix multiplied by the scale factor.</summary>
    public readonly gp_Mat2d VectorialPart()
    {
        if (scale == 1.0)
        {
            return matrix;
        }

        gp_Mat2d M = matrix;
        if (shape == gp_Scale || shape == gp_PntMirror)
        {
            M.SetDiagonal(matrix.Value(1, 1) * scale, matrix.Value(2, 2) * scale);
        }
        else
        {
            M.Multiply(scale);
        }

        return M;
    }

    /// <summary>The matrix without the scale factor.</summary>
    public readonly gp_Mat2d HVectorialPart() => matrix;

    /// <summary>The angle of the rotation part, in ]-PI, PI].</summary>
    public readonly double RotationPart() => Math.Atan2(matrix.Value(2, 1), matrix.Value(1, 1));

    /// <summary>The coefficient at a row (1 to 2) and column (1 to 3) of the 2x3 matrix, scale included.</summary>
    public readonly double Value(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2 || theCol < 1 || theCol > 3, " ");
        if (theCol < 3)
        {
            return scale * matrix.Value(theRow, theCol);
        }
        else
        {
            return loc.Coord(theRow);
        }
    }

    public readonly gp_Trsf2d Inverted()
    {
        gp_Trsf2d aT = this;
        aT.Invert();
        return aT;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="theT"/> first.</summary>
    public readonly gp_Trsf2d Multiplied(in gp_Trsf2d theT)
    {
        gp_Trsf2d aTresult = this;
        aTresult.Multiply(theT);
        return aTresult;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly gp_Trsf2d PreMultiplied(in gp_Trsf2d theT)
    {
        gp_Trsf2d aTresult = this;
        aTresult.PreMultiply(theT);
        return aTresult;
    }

    public readonly gp_Trsf2d Powered(int theN)
    {
        gp_Trsf2d aT = this;
        aT.Power(theN);
        return aT;
    }

    public readonly void Transforms(ref double theX, ref double theY)
    {
        gp_XY aDoublet = new(theX, theY);
        aDoublet.Multiply(matrix);
        if (scale != 1.0)
        {
            aDoublet.Multiply(scale);
        }

        aDoublet.Add(loc);
        aDoublet.Coord(out theX, out theY);
    }

    public readonly void Transforms(ref gp_XY theCoord)
    {
        theCoord.Multiply(matrix);
        if (scale != 1.0)
        {
            theCoord.Multiply(scale);
        }

        theCoord.Add(loc);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref gp_XY)"/>.</summary>
    public readonly gp_XY Transformed(in gp_XY theCoord)
    {
        gp_XY aCoord = theCoord;
        Transforms(ref aCoord);
        return aCoord;
    }

    public static gp_Trsf2d operator *(in gp_Trsf2d theLeft, in gp_Trsf2d theRight) => theLeft.Multiplied(theRight);

    public readonly bool Equals(gp_Trsf2d other)
        => scale.Equals(other.scale) && shape == other.shape && matrix.Equals(other.matrix) && loc.Equals(other.loc);

    public override readonly bool Equals(object? obj) => obj is gp_Trsf2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(scale, shape, matrix, loc);

    public static bool operator ==(in gp_Trsf2d theLeft, in gp_Trsf2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Trsf2d theLeft, in gp_Trsf2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Trsf2d({shape}, scale {scale}, {matrix}, {loc})";
}
