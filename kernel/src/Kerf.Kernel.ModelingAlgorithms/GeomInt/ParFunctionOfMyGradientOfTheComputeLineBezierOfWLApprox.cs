// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParFunctionOfMyGradientOfTheComputeLineBezierOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParFunctionOfMyGradientOfTheComputeLineBezierOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Patches;
using Occt;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>OCCT's instantiation of the function minimised by the gradient over the <see cref="MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="Function{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
internal sealed class ParFunctionOfMyGradientOfTheComputeLineBezierOfWLApprox : Function<global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine>, MultiLineTool<WLine>>
{
    /// <summary>As the base constructor.</summary>
    public ParFunctionOfMyGradientOfTheComputeLineBezierOfWLApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine> ssp, int firstPoint, int lastPoint,
                                                   NCollection_HArray1<ConstraintCouple> theConstraints,
                                                   math_Vector parameters, int deg)
        : base(ssp, firstPoint, lastPoint, theConstraints, parameters, deg) { }
}
