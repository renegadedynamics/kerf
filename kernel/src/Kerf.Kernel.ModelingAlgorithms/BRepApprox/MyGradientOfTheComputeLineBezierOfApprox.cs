// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_MyGradientOfTheComputeLineBezierOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_MyGradientOfTheComputeLineBezierOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>OCCT's instantiation of the Bezier approximation with parameter optimisation over the <see cref="MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="Gradient{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
internal sealed class MyGradientOfTheComputeLineBezierOfApprox : Gradient<global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine>, MultiLineTool<ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public MyGradientOfTheComputeLineBezierOfApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine> ssp, int firstPoint, int lastPoint,
                                      NCollection_HArray1<ConstraintCouple> theConstraints,
                                      math_Vector parameters, int deg, double tol3d, double tol2d,
                                      int nbIterations = 200)
        : base(ssp, firstPoint, lastPoint, theConstraints, parameters, deg, tol3d, tol2d, nbIterations) { }
}
