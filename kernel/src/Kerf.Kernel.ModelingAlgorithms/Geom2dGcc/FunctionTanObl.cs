// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanObl.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanObl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// The function whose roots are the parameters where the curve's tangent is parallel to a direction (the lines
/// tangent to a curve at a given angle to a line).
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class FunctionTanObl : FunctionWithDerivative
{
    private readonly CurveAdaptor2D _theCurv;
    private readonly Direction2D _theDirection;

    public FunctionTanObl(CurveAdaptor2D c, in Direction2D dir)
    {
        _theCurv      = c;
        _theDirection = dir;
    }

    public override bool Value(double x, out double fval)
    {
        ConstraintCurveTool.D1(_theCurv, x, out _, out Vector2D vect);
        double normeD1 = vect.Magnitude();
        fval           = _theDirection.XY().Crossed(vect.XY()) / normeD1;
        return true;
    }

    public override bool Derivative(double x, out double deriv)
    {
        ConstraintCurveTool.D2(_theCurv, x, out _, out Vector2D vec1, out Vector2D vec2);
        double normeD1 = vec1.Magnitude();
        deriv          = _theDirection.XY().Crossed(vec2.XY()) / normeD1
                - vec1.XY().Dot(vec2.XY()) * _theDirection.XY().Crossed(vec1.XY()) / normeD1;
        return true;
    }

    public override bool Values(double x, out double fval, out double deriv)
    {
        ConstraintCurveTool.D2(_theCurv, x, out _, out Vector2D vec1, out Vector2D vec2);
        double normeD1 = vec1.Magnitude();
        fval           = _theDirection.XY().Crossed(vec1.XY()) / normeD1;
        deriv          = _theDirection.XY().Crossed(vec2.XY()) / normeD1
                - vec1.XY().Dot(vec2.XY()) * _theDirection.XY().Crossed(vec1.XY()) / normeD1;
        return true;
    }
}
