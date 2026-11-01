// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRstRst.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRstRst.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A restriction/restriction blend function as a sweep function for the approximation (<see cref="BRepBlend_AppFuncRoot"/>).</summary>
public class BRepBlend_AppFuncRstRst : BRepBlend_AppFuncRoot
{
    public BRepBlend_AppFuncRstRst(BRepBlend_Line Line, Blend_RstRstFunction Func, double Tol3d, double Tol2d)
        : base(Line, Func, Tol3d, Tol2d)
    {
    }

    public override void Point(Blend_AppFunction Func, double Param, math_Vector theSol, ref Blend_Point Pnt)
    {
        Pnt.SetValue(Func.Pnt1(), Func.Pnt2(), Param, theSol[1], theSol[2]);
    }

    public override void Vec(math_Vector theSol, in Blend_Point Pnt)
    {
        theSol[1] = Pnt.ParameterOnC1();
        theSol[2] = Pnt.ParameterOnC2();
    }
}
