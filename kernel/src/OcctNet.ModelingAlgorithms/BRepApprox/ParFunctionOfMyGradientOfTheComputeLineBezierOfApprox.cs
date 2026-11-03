// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ParFunctionOfMyGradientOfTheComputeLineBezierOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ParFunctionOfMyGradientOfTheComputeLineBezierOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Occt;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>OCCT's instantiation of the function minimised by the gradient over the <see cref="MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="Function{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class ParFunctionOfMyGradientOfTheComputeLineBezierOfApprox : Function<global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine>, MultiLineTool<ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public ParFunctionOfMyGradientOfTheComputeLineBezierOfApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine> ssp, int firstPoint, int lastPoint,
                                                   NCollection_HArray1<ConstraintCouple> theConstraints,
                                                   math_Vector parameters, int deg)
        : base(ssp, firstPoint, lastPoint, theConstraints, parameters, deg) { }
}
