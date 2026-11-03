// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_Gradient_BFGSOfMyGradientOfTheComputeLineBezierOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_Gradient_BFGSOfMyGradientOfTheComputeLineBezierOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>OCCT's instantiation of the BFGS minimisation over the <see cref="ApproxInt_MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="AppParCurves_Gradient_BFGS{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class GeomInt_Gradient_BFGSOfMyGradientOfTheComputeLineBezierOfWLApprox : AppParCurves_Gradient_BFGS<ApproxInt_MultiLine<IntPatch_WLine>, ApproxInt_MultiLineTool<IntPatch_WLine>>
{
    /// <summary>As the base constructor.</summary>
    public GeomInt_Gradient_BFGSOfMyGradientOfTheComputeLineBezierOfWLApprox(MultipleVarFunctionWithGradient F,
                                                     math_Vector StartingPoint, double Tolerance3d,
                                                     double Tolerance2d, double Eps, int NbIterations = 200)
        : base(F, StartingPoint, Tolerance3d, Tolerance2d, Eps, NbIterations) { }
}
