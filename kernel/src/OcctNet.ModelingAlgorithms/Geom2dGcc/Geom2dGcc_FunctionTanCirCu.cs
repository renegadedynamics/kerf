// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCirCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCirCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function whose roots are the parameters on a curve of the points whose tangent touches a circle (the lines
/// tangent to a circle and a curve), normalised by an average square distance between them.
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class Geom2dGcc_FunctionTanCirCu : FunctionWithDerivative
{
    private readonly Circle2D TheCirc;
    private readonly CurveAdaptor2D Curve;
    private readonly double myWeight;

    //=========================================================================
    //  soit P1 le point sur la courbe Geom2dAdaptor_Curve d abscisse u.      +
    //  soit C  le centre du cercle TheCirc.                                  +
    //  Nous recherchons un point P2 appartenant au cercle tel que :          +
    //           --->   -->                                                   +
    //        *  P1P2 . CP2 = 0                                               +
    //                                                                        +
    //        *    -->  2    2                                                +
    //           ||CP2||  = R                                                 +
    //  Nous cherchons donc les zeros de la fonction suivante:                +
    //                         -->  --> 2                                     +
    //             -->  2    ( CP1 . T )      2                               +
    //           ||CP1||  -  -----------  -  R   =  F(u)                      +
    //                          --> 2                                         +
    //                         ||T||                                          +
    //                                                                        +
    //  La derivee de cette fonction est :                                    +
    //                                                                        +
    //             2*(CP1.T)(CP1.N)     2*(CP1.T)*(CP1.T)*T.N                 +
    //  f(u) =  -  ----------------  +  ---------------------                 +
    //                  T.T                  (T.T)*(T.T)                      +
    //=========================================================================
    //                                                                        +
    // skv: Small addition: The function and the derivative are normalized    +
    //                      by an average square distance between the circle  +
    //                      and the curve.                                    +
    //=========================================================================
    public Geom2dGcc_FunctionTanCirCu(in Circle2D Circ, CurveAdaptor2D Curv)
    {
        Curve   = Curv;
        TheCirc = Circ;

        int    aNbSamp = Geom2dGcc_CurveTool.NbSamples(Curve);
        double aFirst  = Geom2dGcc_CurveTool.FirstParameter(Curve);
        double aLast   = Geom2dGcc_CurveTool.LastParameter(Curve);
        double aStep   = (aLast - aFirst) / aNbSamp;
        double anX     = aFirst + aStep / 2.0;
        Xy  aLoc    = new(0.0, 0.0);

        while (anX <= aLast)
        {
            aLoc += (Geom2dGcc_CurveTool.Value(Curve, anX)).XY();
            anX += aStep;
        }
        myWeight = Max((aLoc - TheCirc.Location().XY()).SquareModulus(), TheCirc.Radius());
    }

    public override bool Value(double X, out double Fval)
    {
        Geom2dGcc_CurveTool.D1(Curve, X, out Point2D Point, out Vector2D Vect1);
        double   NormeD1      = Vect1.Magnitude();
        Vector2D TheDirection = new(TheCirc.Location(), Point);
        double   squaredir    = TheDirection.Dot(TheDirection);
        double   R            = TheCirc.Radius();
        Fval =
          squaredir - R * R - (TheDirection.Dot(Vect1)) * (TheDirection.Dot(Vect1)) / (NormeD1 * NormeD1);
        Fval /= myWeight;
        return true;
    }

    public override bool Derivative(double X, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(Curve, X, out Point2D Point, out Vector2D Vect1, out Vector2D Vect2);
        double   NormeD1      = Vect1.SquareMagnitude();
        Vector2D TheDirection = new(TheCirc.Location(), Point);
        double   cp1dott      = TheDirection.Dot(Vect1);
        Deriv =
          -2.0 * (cp1dott / NormeD1) * ((TheDirection.Dot(Vect2)) - cp1dott * Vect1.Dot(Vect2) / NormeD1);
        Deriv /= myWeight;
        return true;
    }

    public override bool Values(double X, out double Fval, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(Curve, X, out Point2D Point, out Vector2D Vect1, out Vector2D Vect2);
        double   NormeD1      = Vect1.SquareMagnitude();
        Vector2D TheDirection = new(TheCirc.Location(), Point);
        double   squaredir    = TheDirection.SquareMagnitude();
        double   cp1dott      = TheDirection.Dot(Vect1);
        double   R            = TheCirc.Radius();

        Fval = squaredir - R * R - cp1dott * cp1dott / NormeD1;
        Fval /= myWeight;

        Deriv =
          -2.0 * (cp1dott / NormeD1) * ((TheDirection.Dot(Vect2)) - cp1dott * Vect1.Dot(Vect2) / NormeD1);
        Deriv /= myWeight;
        return true;
    }
}
