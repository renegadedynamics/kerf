// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_MyBSplGradientOfTheComputeLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the B-spline approximation with parameter optimisation over the <see cref="ApproxInt_MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="AppParCurves_BSpGradient{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class BRepApprox_MyBSplGradientOfTheComputeLineOfApprox : AppParCurves_BSpGradient<ApproxInt_MultiLine<BRepApprox_ApproxLine>, ApproxInt_MultiLineTool<BRepApprox_ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public BRepApprox_MyBSplGradientOfTheComputeLineOfApprox(ApproxInt_MultiLine<BRepApprox_ApproxLine> SSP, int FirstPoint, int LastPoint,
                                                 NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                 math_Vector Parameters, NCollection_Array1<double> Knots,
                                                 NCollection_Array1<int> Mults, int Deg, double Tol3d, double Tol2d,
                                                 int NbIterations = 1)
        : base(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Knots, Mults, Deg, Tol3d, Tol2d, NbIterations) { }

    /// <summary>As the base constructor.</summary>
    public BRepApprox_MyBSplGradientOfTheComputeLineOfApprox(ApproxInt_MultiLine<BRepApprox_ApproxLine> SSP, int FirstPoint, int LastPoint,
                                                 NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                 math_Vector Parameters, NCollection_Array1<double> Knots,
                                                 NCollection_Array1<int> Mults, int Deg, double Tol3d, double Tol2d,
                                                 int NbIterations, double lambda1, double lambda2)
        : base(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Knots, Mults, Deg, Tol3d, Tol2d, NbIterations, lambda1, lambda2) { }
}
