// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuPnt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuPnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function whose roots are the parameters where the curve's tangent passes through a point (the lines through a
/// point tangent to a curve).
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class Geom2dGcc_FunctionTanCuPnt : FunctionWithDerivative
{
    private readonly CurveAdaptor2D TheCurv;
    private readonly Point2D ThePoint;

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
    public Geom2dGcc_FunctionTanCuPnt(CurveAdaptor2D C, in Point2D Point)
    {
        TheCurv  = C;
        ThePoint = Point;
    }

    public override bool Value(double X, out double Fval)
    {
        Geom2dGcc_CurveTool.D1(TheCurv, X, out Point2D Point, out Vector2D Vect);
        double   NormeD1      = Vect.Magnitude();
        Vector2D TheDirection = new(ThePoint, Point);
        double   NormeDir     = TheDirection.Magnitude();
        Fval                  = TheDirection.Crossed(Vect) / (NormeD1 * NormeDir);
        return true;
    }

    public override bool Derivative(double X, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(TheCurv, X, out Point2D Point, out Vector2D Vec1, out Vector2D Vec2);
        Vector2D TheDirection = new(new Point2D(ThePoint.XY()), new Point2D(Point.XY()));
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
        Geom2dGcc_CurveTool.D2(TheCurv, X, out Point2D Point, out Vector2D Vec1, out Vector2D Vec2);
        Vector2D TheDirection = new(new Point2D(ThePoint.XY()), new Point2D(Point.XY()));
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
