// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The BFGS minimisation of <see cref="AppParCurves_BSpFunction{MultiLine, ToolLine}"/> used by
/// <see cref="AppParCurves_BSpGradient{MultiLine, ToolLine}"/>: it also stops when the approximation errors are within
/// the tolerances.
/// </summary>
/// <remarks>
/// OCCT has no <c>.gxx</c> for this class: its CDL generator wrote one copy per instantiation (AppDef_BSpGradient_BFGSOf...,
/// GeomInt_..., BRepApprox_...), all identical but for the function type. The port is the generic form of those
/// copies (decision 142), with AppDef's as the source. The minimisation runs in the constructor.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class AppParCurves_BSpGradient_BFGS<MultiLine, ToolLine> : Bfgs
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    private readonly double myTol3d;
    private readonly double myTol2d;

    /// <summary>Minimises <paramref name="F"/> from <paramref name="StartingPoint"/>.</summary>
    public AppParCurves_BSpGradient_BFGS(MultipleVarFunctionWithGradient F, math_Vector StartingPoint, double Tolerance3d,
                                      double Tolerance2d, double Eps, int NbIterations = 200)
        : base(F.VariableCount, Eps, NbIterations, Eps)
    {
        myTol3d = Tolerance3d;
        myTol2d = Tolerance2d;
        Perform(F, StartingPoint);
    }

    /// <summary>
    /// True when the minimum no longer decreases, or when the errors of the function (an
    /// <see cref="AppParCurves_BSpFunction{MultiLine, ToolLine}"/>) are within the tolerances.
    /// </summary>
    public override bool IsSolutionReached(MultipleVarFunctionWithGradient F)
    {
        AppParCurves_BSpFunction<MultiLine, ToolLine> F1 = (AppParCurves_BSpFunction<MultiLine, ToolLine>)F;
        bool Result, Result2;
        Result = (2.0 * Math.Abs(_theMinimum - _previousMinimum)
                  <= 1.0e-10 * (Math.Abs(_theMinimum) + Math.Abs(_previousMinimum)) + 1.0e-12);
        double MErr3d = F1.MaxError3d();
        double MErr2d = F1.MaxError2d();
        Result2 = ((MErr3d <= myTol3d) && (MErr2d <= myTol2d));
        return (Result || Result2);
    }
}
