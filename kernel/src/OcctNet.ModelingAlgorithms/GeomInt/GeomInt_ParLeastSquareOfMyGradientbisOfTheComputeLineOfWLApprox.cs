// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the least-squares approximation over the <see cref="ApproxInt_MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="AppParCurves_LeastSquare{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox : AppParCurves_LeastSquare<ApproxInt_MultiLine<IntPatch_WLine>, ApproxInt_MultiLineTool<IntPatch_WLine>>
{
    /// <summary>As the base constructor.</summary>
    public GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(ApproxInt_MultiLine<IntPatch_WLine> SSP, int FirstPoint, int LastPoint,
                                                                AppParCurves_Constraint FirstCons,
                                                                AppParCurves_Constraint LastCons,
                                                                math_Vector Parameters, int NbPol)
        : base(SSP, FirstPoint, LastPoint, FirstCons, LastCons, Parameters, NbPol) { }

    /// <summary>As the base constructor.</summary>
    public GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(ApproxInt_MultiLine<IntPatch_WLine> SSP, int FirstPoint, int LastPoint,
                                                                AppParCurves_Constraint FirstCons,
                                                                AppParCurves_Constraint LastCons, int NbPol)
        : base(SSP, FirstPoint, LastPoint, FirstCons, LastCons, NbPol) { }

    /// <summary>As the base constructor.</summary>
    public GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(ApproxInt_MultiLine<IntPatch_WLine> SSP,
                                                                NCollection_Array1<double> Knots,
                                                                NCollection_Array1<int> Mults, int FirstPoint,
                                                                int LastPoint, AppParCurves_Constraint FirstCons,
                                                                AppParCurves_Constraint LastCons,
                                                                math_Vector Parameters, int NbPol)
        : base(SSP, Knots, Mults, FirstPoint, LastPoint, FirstCons, LastCons, Parameters, NbPol) { }

    /// <summary>As the base constructor.</summary>
    public GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(ApproxInt_MultiLine<IntPatch_WLine> SSP,
                                                                NCollection_Array1<double> Knots,
                                                                NCollection_Array1<int> Mults, int FirstPoint,
                                                                int LastPoint, AppParCurves_Constraint FirstCons,
                                                                AppParCurves_Constraint LastCons, int NbPol)
        : base(SSP, Knots, Mults, FirstPoint, LastPoint, FirstCons, LastCons, NbPol) { }
}
