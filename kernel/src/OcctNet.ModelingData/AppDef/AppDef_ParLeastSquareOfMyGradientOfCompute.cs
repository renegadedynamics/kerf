// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ParLeastSquareOfMyGradientOfCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ParLeastSquareOfMyGradientOfCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the least-squares approximation over <see cref="AppDef_MultiLine"/>: <see cref="AppParCurves_LeastSquare{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class AppDef_ParLeastSquareOfMyGradientOfCompute : AppParCurves_LeastSquare<AppDef_MultiLine, AppDef_MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public AppDef_ParLeastSquareOfMyGradientOfCompute(AppDef_MultiLine SSP, int FirstPoint, int LastPoint,
                                                      AppParCurves_Constraint FirstCons,
                                                      AppParCurves_Constraint LastCons, math_Vector Parameters,
                                                      int NbPol)
        : base(SSP, FirstPoint, LastPoint, FirstCons, LastCons, Parameters, NbPol) { }

    /// <summary>As the base constructor.</summary>
    public AppDef_ParLeastSquareOfMyGradientOfCompute(AppDef_MultiLine SSP, int FirstPoint, int LastPoint,
                                                      AppParCurves_Constraint FirstCons,
                                                      AppParCurves_Constraint LastCons, int NbPol)
        : base(SSP, FirstPoint, LastPoint, FirstCons, LastCons, NbPol) { }

    /// <summary>As the base constructor.</summary>
    public AppDef_ParLeastSquareOfMyGradientOfCompute(AppDef_MultiLine SSP, NCollection_Array1<double> Knots,
                                                      NCollection_Array1<int> Mults, int FirstPoint, int LastPoint,
                                                      AppParCurves_Constraint FirstCons,
                                                      AppParCurves_Constraint LastCons, math_Vector Parameters,
                                                      int NbPol)
        : base(SSP, Knots, Mults, FirstPoint, LastPoint, FirstCons, LastCons, Parameters, NbPol) { }

    /// <summary>As the base constructor.</summary>
    public AppDef_ParLeastSquareOfMyGradientOfCompute(AppDef_MultiLine SSP, NCollection_Array1<double> Knots,
                                                      NCollection_Array1<int> Mults, int FirstPoint, int LastPoint,
                                                      AppParCurves_Constraint FirstCons,
                                                      AppParCurves_Constraint LastCons, int NbPol)
        : base(SSP, Knots, Mults, FirstPoint, LastPoint, FirstCons, LastCons, NbPol) { }
}
