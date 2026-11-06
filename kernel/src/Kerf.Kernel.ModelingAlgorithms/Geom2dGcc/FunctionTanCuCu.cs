// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// The function whose roots are the pairs of parameters (u1, u2) of the lines tangent to two curves (or a circle and a
/// curve): the line through the two points is tangent to the first curve, and the two tangents are parallel.
/// </summary>
/// <remarks>OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 797).</remarks>
internal sealed class FunctionTanCuCu : FunctionSetWithDerivatives
{
    private readonly CurveAdaptor2D _theCurve1 = new();
    private readonly CurveAdaptor2D _theCurve2 = new();
    private readonly Circle2D _theCirc1 = new();
    private readonly Type3 _theType;

    public void InitDerivative(math_Vector x,
                               out Point2D point1,
                               out Point2D point2,
                               out Vector2D tan1,
                               out Vector2D tan2,
                               out Vector2D d21,
                               out Vector2D d22)
    {
        switch (_theType)
        {
            case Type3.CuCu:
            {
                ConstraintCurveTool.D2(_theCurve1, x[1], out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_theCurve2, x[2], out point2, out tan2, out d22);
            }
            break;
            case Type3.CiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _theCirc1, out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_theCurve2, x[2], out point2, out tan2, out d22);
            }
            break;
            default:
            {
                // The caller's default-constructed values stay (OCCT writes nothing here; unreachable).
                point1 = new Point2D();
                point2 = new Point2D();
                tan1   = new Vector2D();
                tan2   = new Vector2D();
                d21    = new Vector2D();
                d22    = new Vector2D();
            }
            break;
        }
    }

    public FunctionTanCuCu(CurveAdaptor2D c1, CurveAdaptor2D c2)
    {
        _theCurve1 = c1;
        _theCurve2 = c2;
        _theType   = Type3.CuCu;
    }

    public FunctionTanCuCu(in Circle2D c1, CurveAdaptor2D c2)
    {
        _theCirc1  = c1;
        _theCurve2 = c2;
        _theType   = Type3.CiCu;
    }

    //=========================================================================
    //  soit P1 le point sur la courbe TheCurve1 d abscisse u1.               +
    //  soit P2 le point sur la courbe TheCurve2 d abscisse u2.               +
    //  soit T1 la tangente a la courbe TheCurve1 en P1.                      +
    //  soit T2 la tangente a la courbe TheCurve2 en P2.                      +
    //  Nous voulons P1 et P2 tels que :                                      +
    //           --->    -->                                                  +
    //        *  P1P2 /\ T1 = 0                                               +
    //                                                                        +
    //           -->   -->                                                    +
    //        *  T1 /\ T2 = 0                                                 +
    //                                                                        +
    //  Nous cherchons donc les zeros des fonctions suivantes:                +
    //             --->    -->                                                +
    //        *    P1P2 /\ T1                                                 +
    //          --------------- = F1(u)                                       +
    //            --->     -->                                                +
    //          ||P1P2||*||T1||                                               +
    //                                                                        +
    //              -->   -->                                                 +
    //        *     T1 /\ T2                                                  +
    //          --------------- = F2(u)                                       +
    //             -->    -->                                                 +
    //           ||T2||*||T1||                                                +
    //=========================================================================

    public override int NbVariables()
    {
        return 2;
    }

    public override int NbEquations()
    {
        return 2;
    }

    public override bool Value(math_Vector x, math_Vector fval)
    {
        InitDerivative(x, out Point2D point1, out Point2D point2, out Vector2D vect11, out Vector2D vect21,
                       out _, out _);
        double   normeD11     = vect11.Magnitude;
        double   normeD21     = vect21.Magnitude;
        Vector2D theDirection = new(point1, point2);
        double   squaredir    = theDirection.Dot(theDirection);
        fval[1]               = theDirection.Crossed(vect11) / (normeD11 * squaredir);
        fval[2]               = vect11.Crossed(vect21) / (normeD11 * normeD21);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix deriv)
    {
        InitDerivative(x, out Point2D point1, out Point2D point2, out Vector2D vect11, out Vector2D vect21,
                       out Vector2D vect12, out Vector2D vect22);
        double   normeD11     = vect11.Magnitude;
        double   normeD21     = vect21.Magnitude;
        Vector2D theDirection = new(point1, point2);
        double   squaredir    = theDirection.Dot(theDirection);
        deriv[1, 1]           = theDirection.Crossed(vect12) / (normeD11 * squaredir)
                      + (theDirection.Crossed(vect11) * normeD11 * normeD11 * vect11.Dot(theDirection))
                          / (normeD11 * normeD11 * normeD11 * squaredir * squaredir * squaredir);
        deriv[1, 2] = vect21.Crossed(vect11) / (normeD11 * squaredir)
                      - (theDirection.Crossed(vect11) * normeD11 * normeD11 * vect21.Dot(theDirection))
                          / (normeD11 * normeD11 * normeD11 * squaredir * squaredir * squaredir);
        deriv[2, 1] = (vect12.Crossed(vect21)) / (normeD11 * normeD21)
                      - (vect11.Crossed(vect21)) * (vect12.Dot(vect11)) * normeD21 * normeD21
                          / (normeD11 * normeD11 * normeD11 * normeD21 * normeD21 * normeD21);
        deriv[2, 2] = (vect11.Crossed(vect22)) / (normeD11 * normeD21)
                      - (vect11.Crossed(vect21)) * (vect22.Dot(vect21)) * normeD11 * normeD11
                          / (normeD11 * normeD11 * normeD11 * normeD21 * normeD21 * normeD21);
        return true;
    }

    public override bool Values(math_Vector x, math_Vector fval, Matrix deriv)
    {
        InitDerivative(x, out Point2D point1, out Point2D point2, out Vector2D vect11, out Vector2D vect21,
                       out Vector2D vect12, out Vector2D vect22);
        double   normeD11     = vect11.Magnitude;
        double   normeD21     = vect21.Magnitude;
        Vector2D theDirection = new(point1, point2);
        double   squaredir    = theDirection.Dot(theDirection);
        fval[1]               = theDirection.Crossed(vect11) / (normeD11 * squaredir);
        fval[2]               = vect11.Crossed(vect21) / (normeD11 * normeD21);
        deriv[1, 1]           = theDirection.Crossed(vect12) / (normeD11 * squaredir)
                      + (theDirection.Crossed(vect11) * normeD11 * normeD11 * vect11.Dot(theDirection))
                          / (normeD11 * normeD11 * normeD11 * squaredir * squaredir * squaredir);
        deriv[1, 2] = vect21.Crossed(vect11) / (normeD11 * squaredir)
                      - (theDirection.Crossed(vect11) * normeD11 * normeD11 * vect21.Dot(theDirection))
                          / (normeD11 * normeD11 * normeD11 * squaredir * squaredir * squaredir);
        deriv[2, 1] = (vect12.Crossed(vect21)) / (normeD11 * normeD21)
                      - (vect11.Crossed(vect21)) * (vect12.Dot(vect11)) * normeD21 * normeD21
                          / (normeD11 * normeD11 * normeD11 * normeD21 * normeD21 * normeD21);
        deriv[2, 2] = (vect11.Crossed(vect22)) / (normeD11 * normeD21)
                      - (vect11.Crossed(vect21)) * (vect22.Dot(vect21)) * normeD11 * normeD11
                          / (normeD11 * normeD11 * normeD11 * normeD21 * normeD21 * normeD21);
        return true;
    }
}
