// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ParFunctionOfMyGradientbisOfBSplineCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ParFunctionOfMyGradientbisOfBSplineCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Approximation;

/// <summary>OCCT's instantiation of the function minimised by the gradient over <see cref="MultiLine"/>: <see cref="Function{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
internal sealed class ParFunctionOfMyGradientbisOfBSplineCompute : Function<MultiLine, MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public ParFunctionOfMyGradientbisOfBSplineCompute(MultiLine ssp, int firstPoint, int lastPoint,
                                                             NCollection_HArray1<ConstraintCouple> theConstraints,
                                                             math_Vector parameters, int deg)
        : base(ssp, firstPoint, lastPoint, theConstraints, parameters, deg) { }
}
