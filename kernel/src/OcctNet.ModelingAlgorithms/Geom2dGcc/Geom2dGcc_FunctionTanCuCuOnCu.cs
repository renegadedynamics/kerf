// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuOnCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuOnCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Geom2dGcc_Type2;

namespace Occt;

/// <summary>
/// The function whose roots are the parameters (u1, u2, u3) and the radius of a circle tangent to two curves (or a
/// circle, a line or a point and a curve) whose centre lies on a third curve (a line, a circle or a curve).
/// </summary>
/// <remarks>OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class Geom2dGcc_FunctionTanCuCuOnCu : math_FunctionSetWithDerivatives
{
    private readonly Geom2dAdaptor_Curve Curv1 = new();
    private readonly Geom2dAdaptor_Curve Curv2 = new();
    private readonly gp_Circ2d Circ1 = new();
    private readonly gp_Lin2d Lin1 = new();
    private readonly gp_Pnt2d Pnt2 = new();
    private readonly gp_Circ2d Circon = new();
    private readonly gp_Lin2d Linon = new();
    private readonly Geom2dAdaptor_Curve Curvon = new();
    private readonly double FirstRad;
    private readonly Geom2dGcc_Type2 TheType;

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
            case Geom2dGcc_CuCuOnCu:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curvon, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CiCuOnCu:
            {
                ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curvon, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_LiCuOnCu:
            {
                ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                D21 = new gp_Vec2d(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curvon, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CuPtOnCu:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curvon, X[3], out Point3, out Tan3, out D23);
                Point2 = Pnt2;
                Tan2   = new gp_Vec2d(0.0, 0.0);
                D22    = new gp_Vec2d(0.0, 0.0);
            }
            break;
            case Geom2dGcc_CuCuOnCi:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                ElCLib.D2(X[3], Circon, out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CiCuOnCi:
            {
                ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                ElCLib.D2(X[3], Circon, out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_LiCuOnCi:
            {
                ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                D21 = new gp_Vec2d(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                ElCLib.D2(X[3], Circon, out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CuPtOnCi:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Point2 = Pnt2;
                Tan2   = new gp_Vec2d(0.0, 0.0);
                D22    = new gp_Vec2d(0.0, 0.0);
                ElCLib.D2(X[3], Circon, out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CuCuOnLi:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                ElCLib.D1(X[3], Linon, out Point3, out Tan3);
                D23 = new gp_Vec2d(0.0, 0.0);
            }
            break;
            case Geom2dGcc_CiCuOnLi:
            {
                ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                ElCLib.D1(X[3], Linon, out Point3, out Tan3);
                D23 = new gp_Vec2d(0.0, 0.0);
            }
            break;
            case Geom2dGcc_LiCuOnLi:
            {
                ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                D21 = new gp_Vec2d(0.0, 0.0);
                ElCLib.D1(X[3], Linon, out Point3, out Tan3);
                D23 = new gp_Vec2d(0.0, 0.0);
            }
            break;
            case Geom2dGcc_CuPtOnLi:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Point2 = Pnt2;
                Tan2   = new gp_Vec2d(0.0, 0.0);
                D22    = new gp_Vec2d(0.0, 0.0);
                ElCLib.D1(X[3], Linon, out Point3, out Tan3);
                D23 = new gp_Vec2d(0.0, 0.0);
            }
            break;
            default:
            {
                throw new Standard_ConstructionError();
            }
        }
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(Geom2dAdaptor_Curve C1, Geom2dAdaptor_Curve C2, Geom2dAdaptor_Curve C3, double Rad)
    {
        Curv1 = C1;
        Curv2 = C2;
        Curvon = C3;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuCuOnCu;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(in gp_Circ2d C1, Geom2dAdaptor_Curve C2, Geom2dAdaptor_Curve C3, double Rad)
    {
        Circ1 = C1;
        Curv2 = C2;
        Curvon = C3;
        FirstRad = Rad;
        TheType = Geom2dGcc_CiCuOnCu;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(in gp_Lin2d L1, Geom2dAdaptor_Curve C2, Geom2dAdaptor_Curve C3, double Rad)
    {
        Lin1 = L1;
        Curv2 = C2;
        Curvon = C3;
        FirstRad = Rad;
        TheType = Geom2dGcc_LiCuOnCu;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(Geom2dAdaptor_Curve C1, in gp_Pnt2d P2, Geom2dAdaptor_Curve C3, double Rad)
    {
        Curv1 = C1;
        Pnt2 = P2;
        Curvon = C3;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuPtOnCu;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(Geom2dAdaptor_Curve C1, Geom2dAdaptor_Curve C2, in gp_Lin2d OnLi, double Rad)
    {
        Curv1 = C1;
        Curv2 = C2;
        Linon = OnLi;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuCuOnLi;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(in gp_Circ2d C1, Geom2dAdaptor_Curve C2, in gp_Lin2d OnLi, double Rad)
    {
        Circ1 = C1;
        Curv2 = C2;
        Linon = OnLi;
        FirstRad = Rad;
        TheType = Geom2dGcc_CiCuOnLi;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(in gp_Lin2d L1, Geom2dAdaptor_Curve C2, in gp_Lin2d OnLi, double Rad)
    {
        Lin1 = L1;
        Curv2 = C2;
        Linon = OnLi;
        FirstRad = Rad;
        TheType = Geom2dGcc_LiCuOnLi;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(Geom2dAdaptor_Curve C1, in gp_Pnt2d P2, in gp_Lin2d OnLi, double Rad)
    {
        Curv1 = C1;
        Pnt2 = P2;
        Linon = OnLi;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuPtOnLi;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(Geom2dAdaptor_Curve C1, Geom2dAdaptor_Curve C2, in gp_Circ2d OnCi, double Rad)
    {
        Curv1 = C1;
        Curv2 = C2;
        Circon = OnCi;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuCuOnCi;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(in gp_Circ2d C1, Geom2dAdaptor_Curve C2, in gp_Circ2d OnCi, double Rad)
    {
        Circ1 = C1;
        Curv2 = C2;
        Circon = OnCi;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuCuOnCi; // OCCT: should be Geom2dGcc_CiCuOnCi; InitDerivative then evaluates the unset Curv1
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(in gp_Lin2d L1, Geom2dAdaptor_Curve C2, in gp_Circ2d OnCi, double Rad)
    {
        Lin1 = L1;
        Curv2 = C2;
        Circon = OnCi;
        FirstRad = Rad;
        TheType = Geom2dGcc_LiCuOnCi;
    }

    public Geom2dGcc_FunctionTanCuCuOnCu(Geom2dAdaptor_Curve C1, in gp_Pnt2d P2, in gp_Circ2d OnCi, double Rad)
    {
        Curv1 = C1;
        Pnt2 = P2;
        Circon = OnCi;
        FirstRad = Rad;
        TheType = Geom2dGcc_CuPtOnCi;
    }

    public override int NbVariables()
    {
        return 4;
    }

    public override int NbEquations()
    {
        return 4;
    }

    public override bool Value(math_Vector X, math_Vector Fval)
    {
        InitDerivative(X, out gp_Pnt2d Point1, out gp_Pnt2d Point2, out gp_Pnt2d Point3, out gp_Vec2d Tan1, out gp_Vec2d Tan2,
                       out gp_Vec2d Tan3, out gp_Vec2d D21, out gp_Vec2d D22, out gp_Vec2d D23);
        // pipj (normes) et PiPj (non Normes).
        gp_Vec2d P1P2 = new(Point1, Point2);
        gp_Vec2d P2P3 = new(Point2, Point3);
        gp_Vec2d P3P1 = new(Point3, Point1);
        gp_Vec2d p1p2, p2p3, p3p1;
        //  if (FirstRad < 1.0) {FirstRad = 1.0; }
        p1p2 = P1P2 / FirstRad;
        p2p3 = P2P3 / FirstRad;
        p3p1 = P3P1 / FirstRad;
        // norme des Tani.
        double nnor1 = Tan1.Magnitude();
        double nnor2 = Tan2.Magnitude();
        // Fonctions Fui.
        // ==============
        Fval[1] = (P3P1.Dot(P3P1) - X[4] * X[4]) / (FirstRad * FirstRad);
        Fval[2] = (P2P3.Dot(P2P3) - X[4] * X[4]) / (FirstRad * FirstRad);
        Fval[3] = P3P1.Dot(Tan1) / (nnor1 * FirstRad);
        Fval[4] = P2P3.Dot(Tan2) / (nnor2 * FirstRad);
        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix Deriv)
    {
        InitDerivative(X, out gp_Pnt2d Point1, out gp_Pnt2d Point2, out gp_Pnt2d Point3, out gp_Vec2d Tan1, out gp_Vec2d Tan2,
                       out gp_Vec2d Tan3, out gp_Vec2d D21, out gp_Vec2d D22, out gp_Vec2d D23);
        // pipj (normes) et PiPj (non Normes).
        gp_Vec2d P1P2 = new(Point1, Point2);
        gp_Vec2d P2P3 = new(Point2, Point3);
        gp_Vec2d P3P1 = new(Point3, Point1);
        gp_Vec2d p1p2, p2p3, p3p1;
        //  if (FirstRad < 1.0) {FirstRad = 1.0; }
        p1p2 = P1P2 / FirstRad;
        p2p3 = P2P3 / FirstRad;
        p3p1 = P3P1 / FirstRad;
        // normales au courbes normees Nori et non nromees nori et norme des nori.
        double nnor1 = Tan1.Magnitude();
        double nnor2 = Tan2.Magnitude();
        // Derivees dFui/uj  1 <= ui <= 3 , 1 <= uj <= 3
        // =============================================
        Deriv[1, 1] = 2.0 * Tan1.Dot(P3P1) / (FirstRad * FirstRad);
        Deriv[1, 2] = 0.0;
        Deriv[1, 3] = -2.0 * Tan3.Dot(P3P1) / (FirstRad * FirstRad);
        Deriv[1, 4] = -2.0 * X[4] / (FirstRad * FirstRad);

        Deriv[2, 1] = 0.0;
        Deriv[2, 2] = -2.0 * Tan2.Dot(P2P3) / (FirstRad * FirstRad);
        Deriv[2, 3] = 2.0 * Tan3.Dot(P2P3) / (FirstRad * FirstRad);
        Deriv[2, 4] = -2.0 * X[4] / (FirstRad * FirstRad);

        Deriv[3, 1] = (P3P1.Dot(D21) + Tan1.Dot(Tan1)) / (FirstRad * nnor1)
                      - (P3P1.Dot(Tan1) * D21.Dot(Tan1)) / (FirstRad * nnor1 * nnor1 * nnor1);
        Deriv[3, 2] = 0.0;
        Deriv[3, 3] = -(Tan3.Dot(Tan1)) / (FirstRad * nnor1);
        Deriv[3, 4] = 0.0;

        Deriv[4, 1] = 0.0;
        Deriv[4, 2] = (P2P3.Dot(D22) - Tan2.Dot(Tan2)) / (FirstRad * nnor2)
                      - P2P3.Dot(Tan2) * Tan2.Dot(D22) / (FirstRad * nnor2 * nnor2 * nnor2);
        Deriv[4, 3] = Tan3.Dot(Tan2) / (FirstRad * nnor1);
        Deriv[4, 4] = 0.0;
        return true;
    }

    public override bool Values(math_Vector X, math_Vector Fval, math_Matrix Deriv)
    {
        InitDerivative(X, out gp_Pnt2d Point1, out gp_Pnt2d Point2, out gp_Pnt2d Point3, out gp_Vec2d Tan1, out gp_Vec2d Tan2,
                       out gp_Vec2d Tan3, out gp_Vec2d D21, out gp_Vec2d D22, out gp_Vec2d D23);
        // pipj (normes) et PiPj (non Normes).
        gp_Vec2d P1P2 = new(Point1, Point2);
        gp_Vec2d P2P3 = new(Point2, Point3);
        gp_Vec2d P3P1 = new(Point3, Point1);
        gp_Vec2d p1p2, p2p3, p3p1;
        //  if (FirstRad < 1.0) {FirstRad = 1.0; }
        p1p2 = P1P2 / FirstRad;
        p2p3 = P2P3 / FirstRad;
        p3p1 = P3P1 / FirstRad;
        // normales au courbes normees Nori et non nromees nori et norme des nori.
        double nnor1 = Tan1.Magnitude();
        double nnor2 = Tan2.Magnitude();
        // Fonctions Fui.
        // ==============
        Fval[1] = (P3P1.Dot(P3P1) - X[4] * X[4]) / (FirstRad * FirstRad);
        Fval[2] = (P2P3.Dot(P2P3) - X[4] * X[4]) / (FirstRad * FirstRad);
        Fval[3] = P3P1.Dot(Tan1) / (nnor1 * FirstRad);
        Fval[4] = P2P3.Dot(Tan2) / (nnor2 * FirstRad);
        // Derivees dFui/uj  1 <= ui <= 3 , 1 <= uj <= 3
        // =============================================
        Deriv[1, 1] = 2.0 * Tan1.Dot(P3P1) / (FirstRad * FirstRad);
        Deriv[1, 2] = 0.0;
        Deriv[1, 3] = -2.0 * Tan3.Dot(P3P1) / (FirstRad * FirstRad);
        Deriv[1, 4] = -2.0 * X[4] / (FirstRad * FirstRad);

        Deriv[2, 1] = 0.0;
        Deriv[2, 2] = -2.0 * Tan2.Dot(P2P3) / (FirstRad * FirstRad);
        Deriv[2, 3] = 2.0 * Tan3.Dot(P2P3) / (FirstRad * FirstRad);
        Deriv[2, 4] = -2.0 * X[4] / (FirstRad * FirstRad);

        Deriv[3, 1] = (P3P1.Dot(D21) + Tan1.Dot(Tan1)) / (FirstRad * nnor1)
                      - (P3P1.Dot(Tan1) * D21.Dot(Tan1)) / (FirstRad * nnor1 * nnor1 * nnor1);
        Deriv[3, 2] = 0.0;
        Deriv[3, 3] = -(Tan3.Dot(Tan1)) / (FirstRad * nnor1);
        Deriv[3, 4] = 0.0;

        Deriv[4, 1] = 0.0;
        Deriv[4, 2] = (P2P3.Dot(D22) - Tan2.Dot(Tan2)) / (FirstRad * nnor2)
                      - P2P3.Dot(Tan2) * Tan2.Dot(D22) / (FirstRad * nnor2 * nnor2 * nnor2);
        Deriv[4, 3] = Tan3.Dot(Tan2) / (FirstRad * nnor1);
        Deriv[4, 4] = 0.0;
        return true;
    }
}
