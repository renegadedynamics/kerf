// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyBSplGradientOfBSplineCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>OCCT's instantiation of the B-spline approximation with parameter optimisation over <see cref="MultiLine"/>: <see cref="BSpGradient{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class MyBSplGradientOfBSplineCompute : BSpGradient<MultiLine, MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public MyBSplGradientOfBSplineCompute(MultiLine ssp, int firstPoint, int lastPoint,
                                                 NCollection_HArray1<ConstraintCouple> theConstraints,
                                                 math_Vector parameters, NCollection_Array1<double> knots,
                                                 NCollection_Array1<int> mults, int deg, double tol3d, double tol2d,
                                                 int nbIterations = 1)
        : base(ssp, firstPoint, lastPoint, theConstraints, parameters, knots, mults, deg, tol3d, tol2d, nbIterations) { }

    /// <summary>As the base constructor.</summary>
    public MyBSplGradientOfBSplineCompute(MultiLine ssp, int firstPoint, int lastPoint,
                                                 NCollection_HArray1<ConstraintCouple> theConstraints,
                                                 math_Vector parameters, NCollection_Array1<double> knots,
                                                 NCollection_Array1<int> mults, int deg, double tol3d, double tol2d,
                                                 int nbIterations, double lambda1, double lambda2)
        : base(ssp, firstPoint, lastPoint, theConstraints, parameters, knots, mults, deg, tol3d, tol2d, nbIterations, lambda1, lambda2) { }
}
