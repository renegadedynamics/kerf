// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Geom2dGcc_Type1;

namespace Occt;

/// <summary>
/// The function whose roots are the parameters (u1, u2, u3) of the tangency points of a circle tangent to three
/// curves (circles and lines among them).
/// </summary>
/// <remarks>
/// OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 5971). OCCT declares three
/// more constructors (a curve and points) without defining them; they are left out.
/// </remarks>
public sealed class Geom2dGcc_FunctionTanCuCuCu : math_FunctionSetWithDerivatives
{
    private readonly Geom2dAdaptor_Curve Curv1 = new();
    private readonly Geom2dAdaptor_Curve Curv2 = new();
    private readonly Geom2dAdaptor_Curve Curv3 = new();
    private readonly gp_Circ2d Circ1 = new();
    private readonly gp_Circ2d Circ2 = new();
    private readonly gp_Lin2d Lin1 = new();
    private readonly gp_Lin2d Lin2 = new();
    private readonly Geom2dGcc_Type1 TheType;

    public void InitDerivative(math_Vector X,
                               out gp_Pnt2d Point1,
                               out gp_Pnt2d Point2,
                               out gp_Pnt2d Point3,
                               out gp_Vec2d Tan1,
                               out gp_Vec2d Tan2,
                               out gp_Vec2d Tan3,
                               out gp_Vec2d D21,
                               out gp_Vec2d D22,
                               out gp_Vec2d D23)
    {
        switch (TheType)
        {
            case Geom2dGcc_CiCuCu:
            {
                ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CiCiCu:
            {
                ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                ElCLib.D2(X[2], Circ2, out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CiLiCu:
            {
                ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                ElCLib.D1(X[2], Lin2, out Point2, out Tan2);
                D22 = new gp_Vec2d(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_LiCuCu:
            {
                ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                D21 = new gp_Vec2d(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_LiLiCu:
            {
                ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                D21 = new gp_Vec2d(0.0, 0.0);
                ElCLib.D1(X[2], Lin2, out Point2, out Tan2);
                D22 = new gp_Vec2d(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CuCuCu:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            default:
            {
                throw new Standard_ConstructionError();
            }
        }
    }

    public Geom2dGcc_FunctionTanCuCuCu(Geom2dAdaptor_Curve C1, Geom2dAdaptor_Curve C2, Geom2dAdaptor_Curve C3)
    {
        Curv1   = C1;
        Curv2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_CuCuCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in gp_Circ2d C1, Geom2dAdaptor_Curve C2, Geom2dAdaptor_Curve C3)
    {
        Circ1   = C1;
        Curv2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_CiCuCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in gp_Circ2d C1, in gp_Circ2d C2, Geom2dAdaptor_Curve C3)
    {
        Circ1   = C1;
        Circ2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_CiCiCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in gp_Circ2d C1, in gp_Lin2d L2, Geom2dAdaptor_Curve C3)
    {
        Circ1   = C1;
        Lin2    = L2;
        Curv3   = C3;
        TheType = Geom2dGcc_CiLiCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in gp_Lin2d L1, in gp_Lin2d L2, Geom2dAdaptor_Curve C3)
    {
        Lin1    = L1;
        Lin2    = L2;
        Curv3   = C3;
        TheType = Geom2dGcc_LiLiCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in gp_Lin2d L1, Geom2dAdaptor_Curve C2, Geom2dAdaptor_Curve C3)
    {
        Lin1    = L1;
        Curv2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_LiCuCu;
    }

    //==========================================================================

    public override int NbVariables()
    {
        return 3;
    }

    public override int NbEquations()
    {
        return 3;
    }

    public override bool Value(math_Vector X, math_Vector Fval)
    {
        InitDerivative(X, out gp_Pnt2d Point1, out gp_Pnt2d Point2, out gp_Pnt2d Point3, out gp_Vec2d Tan1,
                       out gp_Vec2d Tan2, out gp_Vec2d Tan3, out _, out _, out _);
        // pipj (normes) et PiPj (non Normes).
        gp_XY  P1P2    = new gp_Vec2d(Point1, Point2).XY();
        gp_XY  P2P3    = new gp_Vec2d(Point2, Point3).XY();
        gp_XY  P3P1    = new gp_Vec2d(Point3, Point1).XY();
        double NorP1P2 = P1P2.Modulus();
        double NorP2P3 = P2P3.Modulus();
        double NorP3P1 = P3P1.Modulus();
        gp_XY  p1p2, p2p3, p3p1;
        if (NorP1P2 >= gp.Resolution())
        {
            p1p2 = P1P2 / NorP1P2;
        }
        else
        {
            p1p2 = new gp_XY(0.0, 0.0);
        }
        if (NorP2P3 >= gp.Resolution())
        {
            p2p3 = P2P3 / NorP2P3;
        }
        else
        {
            p2p3 = new gp_XY(0.0, 0.0);
        }
        if (NorP3P1 >= gp.Resolution())
        {
            p3p1 = P3P1 / NorP3P1;
        }
        else
        {
            p3p1 = new gp_XY(0.0, 0.0);
        }
        // derivees premieres non normees Deriv1ui.
        gp_XY Deriv1u1 = Tan1.XY();
        gp_XY Deriv1u2 = Tan2.XY();
        gp_XY Deriv1u3 = Tan3.XY();
        // normales aux courbes.
        double nnor1 = Deriv1u1.Modulus();
        double nnor2 = Deriv1u2.Modulus();
        double nnor3 = Deriv1u3.Modulus();
        gp_XY  Nor1  = new(-Deriv1u1.Y(), Deriv1u1.X());
        gp_XY  Nor2  = new(-Deriv1u2.Y(), Deriv1u2.X());
        gp_XY  Nor3  = new(-Deriv1u3.Y(), Deriv1u3.X());
        gp_XY  nor1, nor2, nor3;
        if (nnor1 >= gp.Resolution())
        {
            nor1 = Nor1 / nnor1;
        }
        else
        {
            nor1 = new gp_XY(0.0, 0.0);
        }
        if (nnor2 >= gp.Resolution())
        {
            nor2 = Nor2 / nnor2;
        }
        else
        {
            nor2 = new gp_XY(0.0, 0.0);
        }
        if (nnor3 >= gp.Resolution())
        {
            nor3 = Nor3 / nnor3;
        }
        else
        {
            nor3 = new gp_XY(0.0, 0.0);
        }
        // determination des signes pour les produits scalaires.
        double   signe1  = 1.0;
        double   signe2  = 1.0;
        double   signe3  = 1.0;
        gp_Pnt2d Pcenter = new((Point1.XY() + Point2.XY() + Point3.XY()) / 3.0);
        gp_XY    fic1    = Pcenter.XY() - Point1.XY();
        gp_XY    fic2    = Pcenter.XY() - Point2.XY();
        gp_XY    fic3    = Pcenter.XY() - Point3.XY();
        double   pscal11 = nor1.Dot(fic1);
        double   pscal22 = nor2.Dot(fic2);
        double   pscal33 = nor3.Dot(fic3);
        if (pscal11 <= 0.0)
        {
            signe1 = -1;
        }
        if (pscal22 <= 0.0)
        {
            signe2 = -1;
        }
        if (pscal33 <= 0.0)
        {
            signe3 = -1;
        }
        // Fonctions Fui.
        // ==============
        Fval[1] = signe1 * nor1.Dot(p1p2) + signe2 * nor2.Dot(p1p2);
        Fval[2] = signe2 * nor2.Dot(p2p3) + signe3 * nor3.Dot(p2p3);
        Fval[3] = signe3 * nor3.Dot(p3p1) + signe1 * nor1.Dot(p3p1);
        return true;
    }

    // OCCT: computes nothing and leaves the derivatives as they are (occt-issues.md).
    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        return true;
    }

    // OCCT: computes nothing and leaves the values and derivatives as they are. Circ2d3TanIter solves with
    // math_FunctionSetRoot, which calls Values: its search starts from whatever F and DF hold, uninitialised natively
    // and zeros in the port, which therefore stops at once at the starting point (occt-issues.md, decision 5976).
    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        return true;
    }
}
