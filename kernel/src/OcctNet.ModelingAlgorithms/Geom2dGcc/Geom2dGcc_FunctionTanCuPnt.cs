// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuPnt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuPnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function whose roots are the parameters where the curve's tangent passes through a point (the lines through a
/// point tangent to a curve).
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class Geom2dGcc_FunctionTanCuPnt : math_FunctionWithDerivative
{
    private readonly Geom2dAdaptor_Curve TheCurv;
    private readonly gp_Pnt2d ThePoint;

    //=========================================================================
    //  soit P1 le point sur la courbe Geom2dAdaptor_Curve d abscisse u.      +
    //  soit C  le point ThePoint.                                            +
    //  Nous cherchons donc les zeros de la fonction suivante:                +
    //                                                                        +
    //                 -->   -->                                              +
    //                 CP1 /\ T                                               +
    //             ---------------  =  F(u)                                   +
    //             ||CP1|| * ||T||                                            +
    //                                                                        +
    //  La derivee de cette fonction est :                                    +
    //            CP1 /\ N        (T.N)*((CP1/\T).((CP1/\T))                  +
    //     f(u) = --------  -  --------------------------------               +
    //               N.N            N*N*N*CP1*CP1*CP1                         +
    //=========================================================================
    public Geom2dGcc_FunctionTanCuPnt(Geom2dAdaptor_Curve C, in gp_Pnt2d Point)
    {
        TheCurv  = C;
        ThePoint = Point;
    }

    public override bool Value(double X, out double Fval)
    {
        Geom2dGcc_CurveTool.D1(TheCurv, X, out gp_Pnt2d Point, out gp_Vec2d Vect);
        double   NormeD1      = Vect.Magnitude();
        gp_Vec2d TheDirection = new(ThePoint, Point);
        double   NormeDir     = TheDirection.Magnitude();
        Fval                  = TheDirection.Crossed(Vect) / (NormeD1 * NormeDir);
        return true;
    }

    public override bool Derivative(double X, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(TheCurv, X, out gp_Pnt2d Point, out gp_Vec2d Vec1, out gp_Vec2d Vec2);
        gp_Vec2d TheDirection = new(new gp_Pnt2d(ThePoint.XY()), new gp_Pnt2d(Point.XY()));
        double   NormeD1      = Vec1.Magnitude();
        double   NormeDir     = TheDirection.Magnitude();
        Deriv =
          TheDirection.Crossed(Vec2) / (NormeD1 * NormeDir)
          - (TheDirection.Crossed(Vec1) / (NormeD1 * NormeDir))
              * (Vec1.Dot(Vec2) / (NormeD1 * NormeD1) + Vec1.Dot(TheDirection) / (NormeDir * NormeDir));
        return true;
    }

    public override bool Values(double X, out double Fval, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(TheCurv, X, out gp_Pnt2d Point, out gp_Vec2d Vec1, out gp_Vec2d Vec2);
        gp_Vec2d TheDirection = new(new gp_Pnt2d(ThePoint.XY()), new gp_Pnt2d(Point.XY()));
        double   NormeD1      = Vec1.Magnitude();
        double   NormeDir     = TheDirection.Magnitude();
        Fval                  = TheDirection.Crossed(Vec1) / (NormeD1 * NormeDir);
        Deriv =
          TheDirection.Crossed(Vec2) / (NormeD1 * NormeDir)
          - (TheDirection.Crossed(Vec1) / (NormeD1 * NormeDir))
              * (Vec1.Dot(Vec2) / (NormeD1 * NormeD1) + Vec1.Dot(TheDirection) / (NormeDir * NormeDir));
        return true;
    }
}
