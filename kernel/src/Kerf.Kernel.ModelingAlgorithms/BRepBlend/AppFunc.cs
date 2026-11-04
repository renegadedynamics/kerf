// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFunc.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>A surface/surface blend function as a sweep function for the approximation (<see cref="AppFuncRoot"/>).</summary>
internal class AppFunc : AppFuncRoot
{
    public AppFunc(Line line, BlendFunction func, double tol3d, double tol2d)
        : base(line, func, tol3d, tol2d)
    {
    }

    public override void Point(AppFunction func, double param, math_Vector sol, ref BlendPoint pnt)
    {
        pnt.SetValue(func.Pnt1(), func.Pnt2(), param, sol[1], sol[2], sol[3], sol[4]);
    }

    public override void Vec(math_Vector sol, in BlendPoint pnt)
    {
        pnt.ParametersOnS1(out sol[1], out sol[2]);
        pnt.ParametersOnS2(out sol[3], out sol[4]);
    }
}
