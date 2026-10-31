// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ResConstraintOfMyGradientbisOfTheComputeLineOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ResConstraintOfMyGradientbisOfTheComputeLineOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's instantiation of the constraint resolution over the <see cref="ApproxInt_MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="AppParCurves_ResolConstraint{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class GeomInt_ResConstraintOfMyGradientbisOfTheComputeLineOfWLApprox : AppParCurves_ResolConstraint<ApproxInt_MultiLine<IntPatch_WLine>, ApproxInt_MultiLineTool<IntPatch_WLine>>
{
    /// <summary>As the base constructor.</summary>
    public GeomInt_ResConstraintOfMyGradientbisOfTheComputeLineOfWLApprox(ApproxInt_MultiLine<IntPatch_WLine> SSP, AppParCurves_MultiCurve SCurv,
                                                               int FirstPoint, int LastPoint,
                                                               NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                               math_Matrix Bern, math_Matrix DerivativeBern,
                                                               double Tolerance = 1.0e-10)
        : base(SSP, SCurv, FirstPoint, LastPoint, TheConstraints, Bern, DerivativeBern, Tolerance) { }
}
