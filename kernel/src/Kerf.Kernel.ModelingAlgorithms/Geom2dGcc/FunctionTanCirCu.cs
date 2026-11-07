// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCirCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCirCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// The function whose roots are the parameters on a curve of the points whose tangent touches a circle (the lines
/// tangent to a circle and a curve), normalised by an average square distance between them.
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 797).</remarks>
internal sealed class FunctionTanCirCu : FunctionWithDerivative
{
    private readonly Circle2D _theCirc;
    private readonly CurveAdaptor2D _curve;
    private readonly double _weight;

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
    public FunctionTanCirCu(in Circle2D circ, CurveAdaptor2D curv)
    {
        _curve   = curv;
        _theCirc = circ;

        int    nbSamp = ConstraintCurveTool.NbSamples(_curve);
        double first  = ConstraintCurveTool.FirstParameter(_curve);
        double last   = ConstraintCurveTool.LastParameter(_curve);
        double step   = (last - first) / nbSamp;
        double x     = first + step / 2.0;
        Xy  loc    = new(0.0, 0.0);

        // Kerf: a curve of no length steps by 0 (decision 22501)
        for (int aStep = 0; x <= last; StepLimit.Next(ref aStep))
        {
            loc += (ConstraintCurveTool.Value(_curve, x)).XY();
            x += step;
        }
        _weight = Max((loc - _theCirc.Location.XY()).SquareModulus(), _theCirc.Radius);
    }

    public override bool Value(double x, out double fval)
    {
        ConstraintCurveTool.D1(_curve, x, out Point2D point, out Vector2D vect1);
        double   normeD1      = vect1.Magnitude;
        Vector2D theDirection = new(_theCirc.Location, point);
        double   squaredir    = theDirection.Dot(theDirection);
        double   r            = _theCirc.Radius;
        fval =
          squaredir - r * r - (theDirection.Dot(vect1)) * (theDirection.Dot(vect1)) / (normeD1 * normeD1);
        fval /= _weight;
        return true;
    }

    public override bool Derivative(double x, out double deriv)
    {
        ConstraintCurveTool.D2(_curve, x, out Point2D point, out Vector2D vect1, out Vector2D vect2);
        double   normeD1      = vect1.SquareMagnitude;
        Vector2D theDirection = new(_theCirc.Location, point);
        double   cp1dott      = theDirection.Dot(vect1);
        deriv =
          -2.0 * (cp1dott / normeD1) * ((theDirection.Dot(vect2)) - cp1dott * vect1.Dot(vect2) / normeD1);
        deriv /= _weight;
        return true;
    }

    public override bool Values(double x, out double fval, out double deriv)
    {
        ConstraintCurveTool.D2(_curve, x, out Point2D point, out Vector2D vect1, out Vector2D vect2);
        double   normeD1      = vect1.SquareMagnitude;
        Vector2D theDirection = new(_theCirc.Location, point);
        double   squaredir    = theDirection.SquareMagnitude;
        double   cp1dott      = theDirection.Dot(vect1);
        double   r            = _theCirc.Radius;

        fval = squaredir - r * r - cp1dott * cp1dott / normeD1;
        fval /= _weight;

        deriv =
          -2.0 * (cp1dott / normeD1) * ((theDirection.Dot(vect2)) - cp1dott * vect1.Dot(vect2) / normeD1);
        deriv /= _weight;
        return true;
    }
}
