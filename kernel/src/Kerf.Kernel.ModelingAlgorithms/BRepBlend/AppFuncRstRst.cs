// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRstRst.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRstRst.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>A restriction/restriction blend function as a sweep function for the approximation (<see cref="AppFuncRoot"/>).</summary>
internal class AppFuncRstRst : AppFuncRoot
{
    public AppFuncRstRst(Line line, RstRstFunction func, double tol3d, double tol2d)
        : base(line, func, tol3d, tol2d)
    {
    }

    public override void Point(AppFunction func, double param, math_Vector sol, ref BlendPoint pnt)
    {
        pnt.SetValue(func.Pnt1(), func.Pnt2(), param, sol[1], sol[2]);
    }

    public override void Vec(math_Vector sol, in BlendPoint pnt)
    {
        sol[1] = pnt.ParameterOnC1();
        sol[2] = pnt.ParameterOnC2();
    }
}
