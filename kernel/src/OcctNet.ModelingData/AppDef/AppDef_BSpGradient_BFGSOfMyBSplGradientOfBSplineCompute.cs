// AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute.hxx and _0.cxx are listed as ported by AppParCurves_BSpGradient_BFGS.cs, whose body is AppDef's copy.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the BFGS minimisation of the B-spline gradient over <see cref="AppDef_MultiLine"/>: <see cref="AppParCurves_BSpGradient_BFGS{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 1602).
/// </remarks>
public sealed class AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute : AppParCurves_BSpGradient_BFGS<AppDef_MultiLine, AppDef_MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute(math_MultipleVarFunctionWithGradient F,
                                                                   math_Vector StartingPoint, double Tolerance3d,
                                                                   double Tolerance2d, double Eps,
                                                                   int NbIterations = 200)
        : base(F, StartingPoint, Tolerance3d, Tolerance2d, Eps, NbIterations) { }
}
