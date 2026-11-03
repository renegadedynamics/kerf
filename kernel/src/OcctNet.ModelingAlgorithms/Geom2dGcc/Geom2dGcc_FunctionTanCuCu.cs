// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.Geom2dGcc_Type3;

namespace Occt;

/// <summary>
/// The function whose roots are the pairs of parameters (u1, u2) of the lines tangent to two curves (or a circle and a
/// curve): the line through the two points is tangent to the first curve, and the two tangents are parallel.
/// </summary>
/// <remarks>OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class Geom2dGcc_FunctionTanCuCu : FunctionSetWithDerivatives
{
    private readonly CurveAdaptor2D TheCurve1 = new();
    private readonly CurveAdaptor2D TheCurve2 = new();
    private readonly Circle2D TheCirc1 = new();
    private readonly Geom2dGcc_Type3 TheType;

    public void InitDerivative(math_Vector X,
                               out Point2D Point1,
                               out Point2D Point2,
                               out Vector2D Tan1,
                               out Vector2D Tan2,
                               out Vector2D D21,
                               out Vector2D D22)
    {
        switch (TheType)
        {
            case Geom2dGcc_CuCu:
            {
                Geom2dGcc_CurveTool.D2(TheCurve1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(TheCurve2, X[2], out Point2, out Tan2, out D22);
            }
            break;
            case Geom2dGcc_CiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(X[1], TheCirc1, out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(TheCurve2, X[2], out Point2, out Tan2, out D22);
            }
            break;
            default:
            {
                // The caller's default-constructed values stay (OCCT writes nothing here; unreachable).
                Point1 = new Point2D();
                Point2 = new Point2D();
                Tan1   = new Vector2D();
                Tan2   = new Vector2D();
                D21    = new Vector2D();
                D22    = new Vector2D();
            }
            break;
        }
    }

    public Geom2dGcc_FunctionTanCuCu(CurveAdaptor2D C1, CurveAdaptor2D C2)
    {
        TheCurve1 = C1;
        TheCurve2 = C2;
        TheType   = Geom2dGcc_CuCu;
    }

    public Geom2dGcc_FunctionTanCuCu(in Circle2D C1, CurveAdaptor2D C2)
    {
        TheCirc1  = C1;
        TheCurve2 = C2;
        TheType   = Geom2dGcc_CiCu;
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

    public override bool Value(math_Vector X, math_Vector Fval)
    {
        InitDerivative(X, out Point2D Point1, out Point2D Point2, out Vector2D Vect11, out Vector2D Vect21,
                       out _, out _);
        double   NormeD11     = Vect11.Magnitude();
        double   NormeD21     = Vect21.Magnitude();
        Vector2D TheDirection = new(Point1, Point2);
        double   squaredir    = TheDirection.Dot(TheDirection);
        Fval[1]               = TheDirection.Crossed(Vect11) / (NormeD11 * squaredir);
        Fval[2]               = Vect11.Crossed(Vect21) / (NormeD11 * NormeD21);
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix Deriv)
    {
        InitDerivative(X, out Point2D Point1, out Point2D Point2, out Vector2D Vect11, out Vector2D Vect21,
                       out Vector2D Vect12, out Vector2D Vect22);
        double   NormeD11     = Vect11.Magnitude();
        double   NormeD21     = Vect21.Magnitude();
        Vector2D TheDirection = new(Point1, Point2);
        double   squaredir    = TheDirection.Dot(TheDirection);
        Deriv[1, 1]           = TheDirection.Crossed(Vect12) / (NormeD11 * squaredir)
                      + (TheDirection.Crossed(Vect11) * NormeD11 * NormeD11 * Vect11.Dot(TheDirection))
                          / (NormeD11 * NormeD11 * NormeD11 * squaredir * squaredir * squaredir);
        Deriv[1, 2] = Vect21.Crossed(Vect11) / (NormeD11 * squaredir)
                      - (TheDirection.Crossed(Vect11) * NormeD11 * NormeD11 * Vect21.Dot(TheDirection))
                          / (NormeD11 * NormeD11 * NormeD11 * squaredir * squaredir * squaredir);
        Deriv[2, 1] = (Vect12.Crossed(Vect21)) / (NormeD11 * NormeD21)
                      - (Vect11.Crossed(Vect21)) * (Vect12.Dot(Vect11)) * NormeD21 * NormeD21
                          / (NormeD11 * NormeD11 * NormeD11 * NormeD21 * NormeD21 * NormeD21);
        Deriv[2, 2] = (Vect11.Crossed(Vect22)) / (NormeD11 * NormeD21)
                      - (Vect11.Crossed(Vect21)) * (Vect22.Dot(Vect21)) * NormeD11 * NormeD11
                          / (NormeD11 * NormeD11 * NormeD11 * NormeD21 * NormeD21 * NormeD21);
        return true;
    }

    public override bool Values(math_Vector X, math_Vector Fval, Matrix Deriv)
    {
        InitDerivative(X, out Point2D Point1, out Point2D Point2, out Vector2D Vect11, out Vector2D Vect21,
                       out Vector2D Vect12, out Vector2D Vect22);
        double   NormeD11     = Vect11.Magnitude();
        double   NormeD21     = Vect21.Magnitude();
        Vector2D TheDirection = new(Point1, Point2);
        double   squaredir    = TheDirection.Dot(TheDirection);
        Fval[1]               = TheDirection.Crossed(Vect11) / (NormeD11 * squaredir);
        Fval[2]               = Vect11.Crossed(Vect21) / (NormeD11 * NormeD21);
        Deriv[1, 1]           = TheDirection.Crossed(Vect12) / (NormeD11 * squaredir)
                      + (TheDirection.Crossed(Vect11) * NormeD11 * NormeD11 * Vect11.Dot(TheDirection))
                          / (NormeD11 * NormeD11 * NormeD11 * squaredir * squaredir * squaredir);
        Deriv[1, 2] = Vect21.Crossed(Vect11) / (NormeD11 * squaredir)
                      - (TheDirection.Crossed(Vect11) * NormeD11 * NormeD11 * Vect21.Dot(TheDirection))
                          / (NormeD11 * NormeD11 * NormeD11 * squaredir * squaredir * squaredir);
        Deriv[2, 1] = (Vect12.Crossed(Vect21)) / (NormeD11 * NormeD21)
                      - (Vect11.Crossed(Vect21)) * (Vect12.Dot(Vect11)) * NormeD21 * NormeD21
                          / (NormeD11 * NormeD11 * NormeD11 * NormeD21 * NormeD21 * NormeD21);
        Deriv[2, 2] = (Vect11.Crossed(Vect22)) / (NormeD11 * NormeD21)
                      - (Vect11.Crossed(Vect21)) * (Vect22.Dot(Vect21)) * NormeD11 * NormeD11
                          / (NormeD11 * NormeD11 * NormeD11 * NormeD21 * NormeD21 * NormeD21);
        return true;
    }
}
