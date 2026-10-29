// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Gradient_BFGSOfMyGradientOfCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Gradient_BFGSOfMyGradientOfCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the BFGS minimisation over <see cref="AppDef_MultiLine"/>: <see cref="AppParCurves_Gradient_BFGS{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class AppDef_Gradient_BFGSOfMyGradientOfCompute : AppParCurves_Gradient_BFGS<AppDef_MultiLine, AppDef_MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public AppDef_Gradient_BFGSOfMyGradientOfCompute(math_MultipleVarFunctionWithGradient F,
                                                     math_Vector StartingPoint, double Tolerance3d,
                                                     double Tolerance2d, double Eps, int NbIterations = 200)
        : base(F, StartingPoint, Tolerance3d, Tolerance2d, Eps, NbIterations) { }
}
