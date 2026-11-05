// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuPnt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuPnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// The function whose roots are the parameters where the curve's tangent passes through a point (the lines through a
/// point tangent to a curve).
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 797).</remarks>
internal sealed class FunctionTanCuPnt : FunctionWithDerivative
{
    private readonly CurveAdaptor2D _theCurv;
    private readonly Point2D _thePoint;

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
    public FunctionTanCuPnt(CurveAdaptor2D c, in Point2D point)
    {
        _theCurv  = c;
        _thePoint = point;
    }

    public override bool Value(double x, out double fval)
    {
        ConstraintCurveTool.D1(_theCurv, x, out Point2D point, out Vector2D vect);
        double   normeD1      = vect.Magnitude;
        Vector2D theDirection = new(_thePoint, point);
        double   normeDir     = theDirection.Magnitude;
        fval                  = theDirection.Crossed(vect) / (normeD1 * normeDir);
        return true;
    }

    public override bool Derivative(double x, out double deriv)
    {
        ConstraintCurveTool.D2(_theCurv, x, out Point2D point, out Vector2D vec1, out Vector2D vec2);
        Vector2D theDirection = new(new Point2D(_thePoint.XY()), new Point2D(point.XY()));
        double   normeD1      = vec1.Magnitude;
        double   normeDir     = theDirection.Magnitude;
        deriv =
          theDirection.Crossed(vec2) / (normeD1 * normeDir)
          - (theDirection.Crossed(vec1) / (normeD1 * normeDir))
              * (vec1.Dot(vec2) / (normeD1 * normeD1) + vec1.Dot(theDirection) / (normeDir * normeDir));
        return true;
    }

    public override bool Values(double x, out double fval, out double deriv)
    {
        ConstraintCurveTool.D2(_theCurv, x, out Point2D point, out Vector2D vec1, out Vector2D vec2);
        Vector2D theDirection = new(new Point2D(_thePoint.XY()), new Point2D(point.XY()));
        double   normeD1      = vec1.Magnitude;
        double   normeDir     = theDirection.Magnitude;
        fval                  = theDirection.Crossed(vec1) / (normeD1 * normeDir);
        deriv =
          theDirection.Crossed(vec2) / (normeD1 * normeDir)
          - (theDirection.Crossed(vec1) / (normeD1 * normeDir))
              * (vec1.Dot(vec2) / (normeD1 * normeD1) + vec1.Dot(theDirection) / (normeDir * normeDir));
        return true;
    }
}
