// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_Gradient_BFGSOfMyGradientbisOfTheComputeLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_Gradient_BFGSOfMyGradientbisOfTheComputeLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>OCCT's instantiation of the BFGS minimisation over the <see cref="MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="GradientBfgs{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
internal sealed class GradientBfgsOfMyGradientbisOfTheComputeLineOfApprox : GradientBfgs<global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine>, MultiLineTool<ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public GradientBfgsOfMyGradientbisOfTheComputeLineOfApprox(MultipleVarFunctionWithGradient f,
                                                               math_Vector startingPoint, double tolerance3d,
                                                               double tolerance2d, double eps, int nbIterations = 200)
        : base(f, startingPoint, tolerance3d, tolerance2d, eps, nbIterations) { }
}
