// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyGradientbisOfBSplineCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyGradientbisOfBSplineCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the Bezier approximation with parameter optimisation over <see cref="AppDef_MultiLine"/>: <see cref="AppParCurves_Gradient{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 1602).
/// </remarks>
public sealed class AppDef_MyGradientbisOfBSplineCompute : AppParCurves_Gradient<AppDef_MultiLine, AppDef_MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public AppDef_MyGradientbisOfBSplineCompute(AppDef_MultiLine SSP, int FirstPoint, int LastPoint,
                                                NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                math_Vector Parameters, int Deg, double Tol3d, double Tol2d,
                                                int NbIterations = 200)
        : base(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Deg, Tol3d, Tol2d, NbIterations) { }
}
