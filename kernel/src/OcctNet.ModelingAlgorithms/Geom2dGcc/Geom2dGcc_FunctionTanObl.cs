// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanObl.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanObl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function whose roots are the parameters where the curve's tangent is parallel to a direction (the lines
/// tangent to a curve at a given angle to a line).
/// </summary>
/// <remarks>OCCT copies the adaptor; the port shares the solver's, which nothing writes (decision 5971).</remarks>
public sealed class Geom2dGcc_FunctionTanObl : math_FunctionWithDerivative
{
    private readonly Geom2dAdaptor_Curve TheCurv;
    private readonly gp_Dir2d TheDirection;

    public Geom2dGcc_FunctionTanObl(Geom2dAdaptor_Curve C, in gp_Dir2d Dir)
    {
        TheCurv      = C;
        TheDirection = Dir;
    }

    public override bool Value(double X, out double Fval)
    {
        Geom2dGcc_CurveTool.D1(TheCurv, X, out _, out gp_Vec2d Vect);
        double NormeD1 = Vect.Magnitude();
        Fval           = TheDirection.XY().Crossed(Vect.XY()) / NormeD1;
        return true;
    }

    public override bool Derivative(double X, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(TheCurv, X, out _, out gp_Vec2d Vec1, out gp_Vec2d Vec2);
        double NormeD1 = Vec1.Magnitude();
        Deriv          = TheDirection.XY().Crossed(Vec2.XY()) / NormeD1
                - Vec1.XY().Dot(Vec2.XY()) * TheDirection.XY().Crossed(Vec1.XY()) / NormeD1;
        return true;
    }

    public override bool Values(double X, out double Fval, out double Deriv)
    {
        Geom2dGcc_CurveTool.D2(TheCurv, X, out _, out gp_Vec2d Vec1, out gp_Vec2d Vec2);
        double NormeD1 = Vec1.Magnitude();
        Fval           = TheDirection.XY().Crossed(Vec1.XY()) / NormeD1;
        Deriv          = TheDirection.XY().Crossed(Vec2.XY()) / NormeD1
                - Vec1.XY().Dot(Vec2.XY()) * TheDirection.XY().Crossed(Vec1.XY()) / NormeD1;
        return true;
    }
}
