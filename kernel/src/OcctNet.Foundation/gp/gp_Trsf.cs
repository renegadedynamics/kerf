// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// A non-persistent transformation in 3D space: a rigid motion, mirror or uniform scale, or a combination.
/// It maps P to <c>scale * matrix * P + loc</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Trsf()</c> is the identity, as in OCCT; <c>default(gp_Trsf)</c> is not valid. Build one with the
/// static factories (<see cref="Rotation(in gp_Ax1, double)"/>, <see cref="Translation(in gp_Vec)"/>, ...) or the
/// <c>With</c> methods, which mirror OCCT's <c>Set</c> methods.
/// </remarks>
public partial struct gp_Trsf : IEquatable<gp_Trsf>
{
    internal double scale;
    internal gp_TrsfForm shape;
    internal gp_Mat matrix;
    internal gp_XYZ loc;

    public gp_Trsf()
    {
        scale = 1.0;
        shape = gp_Identity;
        matrix = new gp_Mat(1, 0, 0, 0, 1, 0, 0, 0, 1);
        loc = new gp_XYZ(0.0, 0.0, 0.0);
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The central symmetry about a point.</summary>
    public static gp_Trsf Mirror(in gp_Pnt theP) => new gp_Trsf().WithMirror(theP);

    /// <summary>The rotation by 180 degrees about an axis.</summary>
    public static gp_Trsf Mirror(in gp_Ax1 theA1) => new gp_Trsf().WithMirror(theA1);

    /// <summary>The reflection in a plane.</summary>
    public static gp_Trsf Mirror(in gp_Ax2 theA2) => new gp_Trsf().WithMirror(theA2);

    public static gp_Trsf Rotation(in gp_Ax1 theA1, double theAng) => new gp_Trsf().WithRotation(theA1, theAng);

    public static gp_Trsf Rotation(in gp_Quaternion theR) => new gp_Trsf().WithRotation(theR);

    public static gp_Trsf Scale(in gp_Pnt theP, double theS) => new gp_Trsf().WithScale(theP, theS);

    public static gp_Trsf Displacement(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2)
        => new gp_Trsf().WithDisplacement(theFromSystem1, theToSystem2);

    public static gp_Trsf Transformation(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2)
        => new gp_Trsf().WithTransformation(theFromSystem1, theToSystem2);

    public static gp_Trsf Transformation(in gp_Ax3 theToSystem) => new gp_Trsf().WithTransformation(theToSystem);

    public static gp_Trsf Transformation(in gp_Quaternion R, in gp_Vec theT) => new gp_Trsf().WithTransformation(R, theT);

    public static gp_Trsf Translation(in gp_Vec theV) => new gp_Trsf().WithTranslation(theV);

    public static gp_Trsf Translation(in gp_Pnt theP1, in gp_Pnt theP2) => new gp_Trsf().WithTranslation(theP1, theP2);

    public static gp_Trsf Values(double a11, double a12, double a13, double a14,
                                 double a21, double a22, double a23, double a24,
                                 double a31, double a32, double a33, double a34)
        => new gp_Trsf().WithValues(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetMirror(in gp_Pnt theP)
    {
        shape = gp_PntMirror;
        scale = -1.0;
        loc = theP.XYZ();
        matrix.SetIdentity();
        loc.Multiply(2.0);
    }

    internal void SetMirror(in gp_Ax1 A1)
    {
        shape = gp_Ax1Mirror;
        scale = 1;
        loc = A1.Location().XYZ();
        matrix.SetDot(A1.Direction().XYZ());
        matrix.Multiply(-2);
        matrix.SetDiagonal(matrix.Value(1, 1) + 1, matrix.Value(2, 2) + 1, matrix.Value(3, 3) + 1);
        loc.Multiply(matrix);
        loc.Add(A1.Location().XYZ());
        matrix.Multiply(-1);
    }

    internal void SetMirror(in gp_Ax2 A2)
    {
        shape = gp_Ax2Mirror;
        scale = -1;
        loc = A2.Location().XYZ();
        matrix.SetDot(A2.Direction().XYZ());
        matrix.Multiply(2);
        matrix.SetDiagonal(matrix.Value(1, 1) - 1, matrix.Value(2, 2) - 1, matrix.Value(3, 3) - 1);
        loc.Multiply(matrix);
        loc.Add(A2.Location().XYZ());
    }

    internal void SetRotation(in gp_Ax1 A1, double Ang)
    {
        shape = gp_Rotation;
        scale = 1.0;
        loc = A1.Location().XYZ();
        matrix.SetRotation(A1.Direction().XYZ(), Ang);
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(A1.Location().XYZ());
    }

    internal void SetRotation(in gp_Quaternion R)
    {
        shape = gp_Rotation;
        scale = 1.0;
        loc.SetCoord(0.0, 0.0, 0.0);
        matrix = R.GetMatrix();
    }

    internal void SetRotationPart(in gp_Quaternion theR)
    {
        bool hasRotation = !theR.IsEqual(new gp_Quaternion());
        if (hasRotation)
        {
            matrix = theR.GetMatrix();
        }
        else
        {
            matrix.SetIdentity();
        }

        switch (shape)
        {
            case gp_Identity:
                if (hasRotation)
                {
                    shape = gp_Rotation;
                }

                break;
            case gp_Rotation:
                if (!hasRotation)
                {
                    shape = gp_Identity;
                }

                break;
            case gp_Translation:
            case gp_PntMirror:
            case gp_Ax1Mirror:
            case gp_Ax2Mirror:
            case gp_Scale:
            case gp_CompoundTrsf:
            case gp_Other:
                if (hasRotation)
                {
                    shape = gp_CompoundTrsf;
                }

                break;
        }
    }

    internal void SetScale(in gp_Pnt P, double S)
    {
        shape = gp_Scale;
        scale = S;
        loc = P.XYZ();
        Standard_ConstructionError_Raise_if(Math.Abs(scale) <= gp.Resolution(), "gp_Trsf::SetScaleFactor");
        matrix.SetIdentity();
        loc.Multiply(1 - S);
    }

    internal void SetTransformation(in gp_Ax3 FromA1, in gp_Ax3 ToA2)
    {
        shape = gp_CompoundTrsf;
        scale = 1.0;
        // matrix from XOY to ToA2
        matrix.SetRows(ToA2.XDirection().XYZ(), ToA2.YDirection().XYZ(), ToA2.Direction().XYZ());
        loc = ToA2.Location().XYZ();
        loc.Multiply(matrix);
        loc.Reverse();

        // matrix FromA1 to XOY
        gp_XYZ xDir = FromA1.XDirection().XYZ();
        gp_XYZ yDir = FromA1.YDirection().XYZ();
        gp_XYZ zDir = FromA1.Direction().XYZ();

        gp_Mat MA1 = new(xDir, yDir, zDir);
        gp_XYZ MA1loc = FromA1.Location().XYZ();

        // matrix * MA1 => FromA1 ToA2
        MA1loc.Multiply(matrix);
        loc.Add(MA1loc);
        matrix.Multiply(MA1);
    }

    internal void SetTransformation(in gp_Ax3 A3)
    {
        shape = gp_CompoundTrsf;
        scale = 1.0;
        matrix.SetRows(A3.XDirection().XYZ(), A3.YDirection().XYZ(), A3.Direction().XYZ());
        loc = A3.Location().XYZ();
        loc.Multiply(matrix);
        loc.Reverse();
    }

    internal void SetTransformation(in gp_Quaternion R, in gp_Vec T)
    {
        shape = gp_CompoundTrsf;
        scale = 1.0;
        loc = T.XYZ();
        matrix = R.GetMatrix();
    }

    internal void SetDisplacement(in gp_Ax3 FromA1, in gp_Ax3 ToA2)
    {
        shape = gp_CompoundTrsf;
        scale = 1.0;
        // matrix from ToA2 to XOY
        matrix.SetCol(1, ToA2.XDirection().XYZ());
        matrix.SetCol(2, ToA2.YDirection().XYZ());
        matrix.SetCol(3, ToA2.Direction().XYZ());
        loc = ToA2.Location().XYZ();

        // matrix XOY to FromA1
        gp_XYZ xDir = FromA1.XDirection().XYZ();
        gp_XYZ yDir = FromA1.YDirection().XYZ();
        gp_XYZ zDir = FromA1.Direction().XYZ();

        gp_Mat MA1 = new(xDir, yDir, zDir);
        MA1.Transpose();
        gp_XYZ MA1loc = FromA1.Location().XYZ();
        MA1loc.Multiply(MA1);
        MA1loc.Reverse();

        // matrix * MA1
        MA1loc.Multiply(matrix);
        loc.Add(MA1loc);
        matrix.Multiply(MA1);
    }

    internal void SetTranslation(in gp_Vec theV)
    {
        shape = gp_Translation;
        scale = 1.0;
        matrix.SetIdentity();
        loc = theV.XYZ();
    }

    internal void SetTranslation(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        shape = gp_Translation;
        scale = 1.0;
        matrix.SetIdentity();
        loc = theP2.XYZ().Subtracted(theP1.XYZ());
    }

    internal void SetTranslationPart(in gp_Vec V)
    {
        loc = V.XYZ();
        bool locnull = loc.SquareModulus() < gp.Resolution();

        switch (shape)
        {
            case gp_Identity:
                if (!locnull)
                {
                    shape = gp_Translation;
                }

                break;
            case gp_Translation:
                if (locnull)
                {
                    shape = gp_Identity;
                }

                break;
            case gp_Rotation:
            case gp_PntMirror:
            case gp_Ax1Mirror:
            case gp_Ax2Mirror:
            case gp_Scale:
            case gp_CompoundTrsf:
            case gp_Other:
                if (!locnull)
                {
                    shape = gp_CompoundTrsf;
                }

                break;
        }
    }

    internal void SetScaleFactor(double S)
    {
        Standard_ConstructionError_Raise_if(Math.Abs(S) <= gp.Resolution(), "gp_Trsf::SetScaleFactor");
        scale = S;

        bool unit = Math.Abs(scale - 1.0) <= gp.Resolution();
        bool munit = Math.Abs(scale + 1.0) <= gp.Resolution();

        switch (shape)
        {
            case gp_Identity:
            case gp_Translation:
                if (!unit)
                {
                    shape = gp_Scale;
                }

                if (munit)
                {
                    shape = gp_PntMirror;
                }

                break;
            case gp_Rotation:
                if (!unit)
                {
                    shape = gp_CompoundTrsf;
                }

                break;
            case gp_PntMirror:
            case gp_Ax1Mirror:
            case gp_Ax2Mirror:
                if (!munit)
                {
                    shape = gp_Scale;
                }

                if (unit)
                {
                    shape = gp_Identity;
                }

                break;
            case gp_Scale:
                if (unit)
                {
                    shape = gp_Identity;
                }

                if (munit)
                {
                    shape = gp_PntMirror;
                }

                break;
            case gp_CompoundTrsf:
                break;
            case gp_Other:
                break;
        }
    }

    internal void SetForm(gp_TrsfForm theP) => shape = theP;

    internal void SetValues(double a11, double a12, double a13, double a14,
                            double a21, double a22, double a23, double a24,
                            double a31, double a32, double a33, double a34)
    {
        gp_XYZ col1 = new(a11, a21, a31);
        gp_XYZ col2 = new(a12, a22, a32);
        gp_XYZ col3 = new(a13, a23, a33);
        gp_XYZ col4 = new(a14, a24, a34);
        // compute the determinant
        gp_Mat M = new(col1, col2, col3);
        double s = M.Determinant();
        Standard_ConstructionError_Raise_if(Math.Abs(s) < gp.Resolution(), "gp_Trsf::SetValues, null determinant");
        if (s > 0)
        {
            s = Math.Pow(s, 1.0 / 3.0);
        }
        else
        {
            s = -Math.Pow(-s, 1.0 / 3.0);
        }

        M.Divide(s);

        scale = s;
        shape = gp_CompoundTrsf;

        matrix = M;
        Orthogonalize();

        loc = col4;
    }

    internal void Invert()
    {
        // -1
        // X' = scale * R * X + T  =>  X = (R / scale) * (X' - T)
        if (shape == gp_Identity)
        {
        }
        else if (shape == gp_Translation || shape == gp_PntMirror)
        {
            loc.Reverse();
        }
        else if (shape == gp_Scale)
        {
            Standard_ConstructionError_Raise_if(Math.Abs(scale) <= gp.Resolution(), "gp_Trsf::Invert() - transformation has zero scale");
            scale = 1.0 / scale;
            loc.Multiply(-scale);
        }
        else
        {
            Standard_ConstructionError_Raise_if(Math.Abs(scale) <= gp.Resolution(), "gp_Trsf::Invert() - transformation has zero scale");
            scale = 1.0 / scale;
            matrix.Transpose();
            loc.Multiply(matrix);
            loc.Multiply(-scale);
        }
    }

    internal void Multiply(in gp_Trsf T)
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
            if (T.loc.X() != 0.0 || T.loc.Y() != 0.0 || T.loc.Z() != 0.0)
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
            loc.Add(T.loc.Multiplied(matrix));
            matrix.Multiply(T.matrix);
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
                 && T.shape == gp_Translation)
        {
            gp_XYZ Tloc = T.loc;
            Tloc.Multiply(matrix);
            if (scale != 1.0)
            {
                Tloc.Multiply(scale);
            }

            loc.Add(Tloc);
        }
        else if ((shape == gp_Scale || shape == gp_PntMirror) && T.shape == gp_Translation)
        {
            gp_XYZ Tloc = T.loc;
            Tloc.Multiply(scale);
            loc.Add(Tloc);
        }
        else if (shape == gp_Translation
                 && (T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror))
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
        else if ((shape == gp_PntMirror || shape == gp_Scale) && (T.shape == gp_PntMirror || T.shape == gp_Scale))
        {
            shape = gp_CompoundTrsf;
            gp_XYZ Tloc = T.loc;
            Tloc.Multiply(scale);
            loc.Add(Tloc);
            scale = scale * T.scale;
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
                 && (T.shape == gp_Scale || T.shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            gp_XYZ Tloc = T.loc;
            if (scale == 1.0)
            {
                scale = T.scale;
                Tloc.Multiply(matrix);
            }
            else
            {
                Tloc.Multiply(matrix);
                Tloc.Multiply(scale);
                scale = scale * T.scale;
            }

            loc.Add(Tloc);
        }
        else if ((T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror)
                 && (shape == gp_Scale || shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            gp_XYZ Tloc = T.loc;
            Tloc.Multiply(scale);
            loc.Add(Tloc);
            scale = scale * T.scale;
            matrix = T.matrix;
        }
        else
        {
            shape = gp_CompoundTrsf;
            gp_XYZ Tloc = T.loc;
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

    internal void PreMultiply(in gp_Trsf T)
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
            loc.Add(T.loc);
            matrix.PreMultiply(T.matrix);
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
                 && T.shape == gp_Translation)
        {
            loc.Add(T.loc);
        }
        else if ((shape == gp_Scale || shape == gp_PntMirror) && T.shape == gp_Translation)
        {
            loc.Add(T.loc);
        }
        else if (shape == gp_Translation
                 && (T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror))
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
        else if ((shape == gp_PntMirror || shape == gp_Scale) && (T.shape == gp_PntMirror || T.shape == gp_Scale))
        {
            shape = gp_CompoundTrsf;
            loc.Multiply(T.scale);
            loc.Add(T.loc);
            scale = scale * T.scale;
        }
        else if ((shape == gp_CompoundTrsf || shape == gp_Rotation || shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
                 && (T.shape == gp_Scale || T.shape == gp_PntMirror))
        {
            shape = gp_CompoundTrsf;
            loc.Multiply(T.scale);
            loc.Add(T.loc);
            scale = scale * T.scale;
        }
        else if ((T.shape == gp_CompoundTrsf || T.shape == gp_Rotation || T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror)
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
                loc = new gp_XYZ(0.0, 0.0, 0.0);
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
                    int Npower = Math.Abs(N) - 1;
                    gp_XYZ Temploc = loc;
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
                    int Npower = Math.Abs(N) - 1;
                    gp_XYZ Temploc = loc;
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
                    int Npower = Math.Abs(N) - 1;
                    gp_Mat Tempmatrix = matrix;
                    if (loc.X() == 0.0 && loc.Y() == 0.0 && loc.Z() == 0.0)
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

                            Tempmatrix.Multiply(Tempmatrix);
                            Npower = Npower / 2;
                        }
                    }
                    else
                    {
                        gp_XYZ Temploc = loc;
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
                            Tempmatrix.Multiply(Tempmatrix);
                            Npower = Npower / 2;
                        }
                    }
                }
                else if (shape == gp_PntMirror || shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
                {
                    if (IsEven(N))
                    {
                        shape = gp_Identity;
                        scale = 1.0;
                        matrix.SetIdentity();
                        loc.SetX(0);
                        loc.SetY(0);
                        loc.SetZ(0);
                    }
                }
                else
                {
                    shape = gp_CompoundTrsf;
                    int Npower = Math.Abs(N) - 1;
                    gp_XYZ Temploc = loc;
                    double Tempscale = scale;
                    gp_Mat Tempmatrix = matrix;
                    for (;;)
                    {
                        if (IsOdd(Npower))
                        {
                            loc.Add(Temploc.Multiplied(matrix).Multiplied(scale));
                            scale = scale * Tempscale;
                            matrix.Multiply(Tempmatrix);
                        }

                        if (Npower == 1)
                        {
                            break;
                        }

                        Tempscale = Tempscale * Tempscale;
                        Temploc.Add(Temploc.Multiplied(Tempmatrix).Multiplied(Tempscale));
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
        gp_Mat aTM = matrix;

        gp_XYZ aV1 = aTM.Column(1);
        gp_XYZ aV2 = aTM.Column(2);
        gp_XYZ aV3 = aTM.Column(3);

        aV1.Normalize();

        aV2 -= aV1 * aV2.Dot(aV1);
        aV2.Normalize();

        aV3 -= aV1 * aV3.Dot(aV1) + aV2 * aV3.Dot(aV2);
        aV3.Normalize();

        aTM.SetCols(aV1, aV2, aV3);

        aV1 = aTM.Row(1);
        aV2 = aTM.Row(2);
        aV3 = aTM.Row(3);

        aV1.Normalize();

        aV2 -= aV1 * aV2.Dot(aV1);
        aV2.Normalize();

        aV3 -= aV1 * aV3.Dot(aV1) + aV2 * aV3.Dot(aV2);
        aV3.Normalize();

        aTM.SetRows(aV1, aV2, aV3);

        matrix = aTM;
    }

    // Copies with a mutator applied

    public readonly gp_Trsf WithMirror(in gp_Pnt theP)
    {
        gp_Trsf aCopy = this;
        aCopy.SetMirror(theP);
        return aCopy;
    }

    public readonly gp_Trsf WithMirror(in gp_Ax1 theA1)
    {
        gp_Trsf aCopy = this;
        aCopy.SetMirror(theA1);
        return aCopy;
    }

    public readonly gp_Trsf WithMirror(in gp_Ax2 theA2)
    {
        gp_Trsf aCopy = this;
        aCopy.SetMirror(theA2);
        return aCopy;
    }

    public readonly gp_Trsf WithRotation(in gp_Ax1 theA1, double theAng)
    {
        gp_Trsf aCopy = this;
        aCopy.SetRotation(theA1, theAng);
        return aCopy;
    }

    public readonly gp_Trsf WithRotation(in gp_Quaternion theR)
    {
        gp_Trsf aCopy = this;
        aCopy.SetRotation(theR);
        return aCopy;
    }

    public readonly gp_Trsf WithRotationPart(in gp_Quaternion theR)
    {
        gp_Trsf aCopy = this;
        aCopy.SetRotationPart(theR);
        return aCopy;
    }

    public readonly gp_Trsf WithScale(in gp_Pnt theP, double theS)
    {
        gp_Trsf aCopy = this;
        aCopy.SetScale(theP, theS);
        return aCopy;
    }

    public readonly gp_Trsf WithDisplacement(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2)
    {
        gp_Trsf aCopy = this;
        aCopy.SetDisplacement(theFromSystem1, theToSystem2);
        return aCopy;
    }

    public readonly gp_Trsf WithTransformation(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2)
    {
        gp_Trsf aCopy = this;
        aCopy.SetTransformation(theFromSystem1, theToSystem2);
        return aCopy;
    }

    public readonly gp_Trsf WithTransformation(in gp_Ax3 theToSystem)
    {
        gp_Trsf aCopy = this;
        aCopy.SetTransformation(theToSystem);
        return aCopy;
    }

    public readonly gp_Trsf WithTransformation(in gp_Quaternion R, in gp_Vec theT)
    {
        gp_Trsf aCopy = this;
        aCopy.SetTransformation(R, theT);
        return aCopy;
    }

    public readonly gp_Trsf WithTranslation(in gp_Vec theV)
    {
        gp_Trsf aCopy = this;
        aCopy.SetTranslation(theV);
        return aCopy;
    }

    public readonly gp_Trsf WithTranslation(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Trsf aCopy = this;
        aCopy.SetTranslation(theP1, theP2);
        return aCopy;
    }

    public readonly gp_Trsf WithTranslationPart(in gp_Vec theV)
    {
        gp_Trsf aCopy = this;
        aCopy.SetTranslationPart(theV);
        return aCopy;
    }

    public readonly gp_Trsf WithScaleFactor(double theS)
    {
        gp_Trsf aCopy = this;
        aCopy.SetScaleFactor(theS);
        return aCopy;
    }

    public readonly gp_Trsf WithForm(gp_TrsfForm theP)
    {
        gp_Trsf aCopy = this;
        aCopy.SetForm(theP);
        return aCopy;
    }

    public readonly gp_Trsf WithValues(double a11, double a12, double a13, double a14,
                                       double a21, double a22, double a23, double a24,
                                       double a31, double a32, double a33, double a34)
    {
        gp_Trsf aCopy = this;
        aCopy.SetValues(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);
        return aCopy;
    }

    // Queries

    /// <summary>True when the determinant of the vectorial part is negative (the transformation mirrors).</summary>
    public readonly bool IsNegative() => scale < 0.0;

    public readonly gp_TrsfForm Form() => shape;

    public readonly double ScaleFactor() => scale;

    public readonly gp_XYZ TranslationPart() => loc;

    /// <summary>The rotation as an axis and an angle. Always true, as in OCCT.</summary>
    public readonly bool GetRotation(out gp_XYZ theAxis, out double theAngle)
    {
        gp_Quaternion Q = GetRotation();
        Q.GetVectorAndAngle(out gp_Vec aVec, out theAngle);
        theAxis = aVec.XYZ();
        return true;
    }

    public readonly gp_Quaternion GetRotation() => new(matrix);

    /// <summary>The matrix multiplied by the scale factor.</summary>
    public readonly gp_Mat VectorialPart()
    {
        if (scale == 1.0)
        {
            return matrix;
        }

        gp_Mat M = matrix;
        if (shape == gp_Scale || shape == gp_PntMirror)
        {
            M.SetDiagonal(scale * M.m00, scale * M.m11, scale * M.m22);
        }
        else
        {
            M.Multiply(scale);
        }

        return M;
    }

    /// <summary>The matrix without the scale factor.</summary>
    public readonly gp_Mat HVectorialPart() => matrix;

    /// <summary>The coefficient at a row (1 to 3) and column (1 to 4) of the 3x4 matrix, scale included.</summary>
    public readonly double Value(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3 || theCol < 1 || theCol > 4, " ");
        if (theCol < 4)
        {
            return scale * matrix.Value(theRow, theCol);
        }
        else
        {
            return loc.Coord(theRow);
        }
    }

    public readonly gp_Trsf Inverted()
    {
        gp_Trsf aT = this;
        aT.Invert();
        return aT;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="theT"/> first.</summary>
    public readonly gp_Trsf Multiplied(in gp_Trsf theT)
    {
        gp_Trsf aTresult = this;
        aTresult.Multiply(theT);
        return aTresult;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly gp_Trsf PreMultiplied(in gp_Trsf theT)
    {
        gp_Trsf aTresult = this;
        aTresult.PreMultiply(theT);
        return aTresult;
    }

    public readonly gp_Trsf Powered(int theN)
    {
        gp_Trsf aT = this;
        aT.Power(theN);
        return aT;
    }

    public readonly void Transforms(ref double theX, ref double theY, ref double theZ)
    {
        gp_XYZ aTriplet = new(theX, theY, theZ);
        aTriplet.Multiply(matrix);
        if (scale != 1.0)
        {
            aTriplet.Multiply(scale);
        }

        aTriplet.Add(loc);
        theX = aTriplet.X();
        theY = aTriplet.Y();
        theZ = aTriplet.Z();
    }

    public readonly void Transforms(ref gp_XYZ theCoord)
    {
        theCoord.Multiply(matrix);
        if (scale != 1.0)
        {
            theCoord.Multiply(scale);
        }

        theCoord.Add(loc);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref gp_XYZ)"/>.</summary>
    public readonly gp_XYZ Transformed(in gp_XYZ theCoord)
    {
        gp_XYZ aCoord = theCoord;
        Transforms(ref aCoord);
        return aCoord;
    }

    /// <summary>The 4x4 matrix, row by row, as OCCT's <c>GetMat4</c>.</summary>
    public readonly void GetMat4(Span<double> theMat)
    {
        if (theMat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(theMat));
        }

        if (shape == gp_Identity)
        {
            theMat[..16].Clear();
            theMat[0] = theMat[5] = theMat[10] = theMat[15] = 1.0;
            return;
        }

        for (int aRow = 1; aRow <= 3; aRow++)
        {
            for (int aCol = 1; aCol <= 4; aCol++)
            {
                theMat[(aRow - 1) * 4 + (aCol - 1)] = Value(aRow, aCol);
            }
        }

        theMat[12] = 0.0;
        theMat[13] = 0.0;
        theMat[14] = 0.0;
        theMat[15] = 1.0;
    }

    public static gp_Trsf operator *(in gp_Trsf theLeft, in gp_Trsf theRight) => theLeft.Multiplied(theRight);

    public readonly bool Equals(gp_Trsf other)
        => scale.Equals(other.scale) && shape == other.shape && matrix.Equals(other.matrix) && loc.Equals(other.loc);

    public override readonly bool Equals(object? obj) => obj is gp_Trsf other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(scale, shape, matrix, loc);

    public static bool operator ==(in gp_Trsf theLeft, in gp_Trsf theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Trsf theLeft, in gp_Trsf theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Trsf({shape}, scale {scale}, {matrix}, {loc})";
}
