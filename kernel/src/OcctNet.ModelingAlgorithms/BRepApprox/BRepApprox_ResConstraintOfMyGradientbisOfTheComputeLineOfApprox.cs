// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ResConstraintOfMyGradientbisOfTheComputeLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ResConstraintOfMyGradientbisOfTheComputeLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>OCCT's instantiation of the constraint resolution over the <see cref="ApproxInt_MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="AppParCurves_ResolConstraint{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class BRepApprox_ResConstraintOfMyGradientbisOfTheComputeLineOfApprox : AppParCurves_ResolConstraint<ApproxInt_MultiLine<BRepApprox_ApproxLine>, ApproxInt_MultiLineTool<BRepApprox_ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public BRepApprox_ResConstraintOfMyGradientbisOfTheComputeLineOfApprox(ApproxInt_MultiLine<BRepApprox_ApproxLine> SSP, AppParCurves_MultiCurve SCurv,
                                                               int FirstPoint, int LastPoint,
                                                               NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                               Matrix Bern, Matrix DerivativeBern,
                                                               double Tolerance = 1.0e-10)
        : base(SSP, SCurv, FirstPoint, LastPoint, TheConstraints, Bern, DerivativeBern, Tolerance) { }
}
