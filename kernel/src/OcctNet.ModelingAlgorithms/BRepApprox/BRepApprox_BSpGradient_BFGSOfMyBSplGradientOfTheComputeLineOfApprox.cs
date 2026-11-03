// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_BSpGradient_BFGSOfMyBSplGradientOfTheComputeLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_BSpGradient_BFGSOfMyBSplGradientOfTheComputeLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>OCCT's instantiation of the BFGS minimisation of the B-spline gradient over the <see cref="ApproxInt_MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="BSpGradientBfgs{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class BRepApprox_BSpGradient_BFGSOfMyBSplGradientOfTheComputeLineOfApprox : BSpGradientBfgs<ApproxInt_MultiLine<BRepApprox_ApproxLine>, ApproxInt_MultiLineTool<BRepApprox_ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public BRepApprox_BSpGradient_BFGSOfMyBSplGradientOfTheComputeLineOfApprox(MultipleVarFunctionWithGradient F,
                                                                   math_Vector StartingPoint, double Tolerance3d,
                                                                   double Tolerance2d, double Eps,
                                                                   int NbIterations = 200)
        : base(F, StartingPoint, Tolerance3d, Tolerance2d, Eps, NbIterations) { }
}
