// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ResConstraintOfTheGradient.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ResConstraintOfTheGradient_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>OCCT's instantiation of the constraint resolution over <see cref="AppDef_MultiLine"/>: <see cref="AppParCurves_ResolConstraint{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class AppDef_ResConstraintOfTheGradient : AppParCurves_ResolConstraint<AppDef_MultiLine, AppDef_MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public AppDef_ResConstraintOfTheGradient(AppDef_MultiLine SSP, AppParCurves_MultiCurve SCurv, int FirstPoint,
                                             int LastPoint,
                                             NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                             Matrix Bern, Matrix DerivativeBern, double Tolerance = 1.0e-10)
        : base(SSP, SCurv, FirstPoint, LastPoint, TheConstraints, Bern, DerivativeBern, Tolerance) { }
}
