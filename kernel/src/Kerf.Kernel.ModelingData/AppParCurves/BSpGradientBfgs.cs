// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_BSpGradient_BFGSOfMyBSplGradientOfBSplineCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The BFGS minimisation of <see cref="BSpFunction{MultiLine, ToolLine}"/> used by
/// <see cref="BSpGradient{MultiLine, ToolLine}"/>: it also stops when the approximation errors are within
/// the tolerances.
/// </summary>
/// <remarks>
/// OCCT has no <c>.gxx</c> for this class: its CDL generator wrote one copy per instantiation (AppDef_BSpGradient_BFGSOf...,
/// GeomInt_..., BRepApprox_...), all identical but for the function type. The port is the generic form of those
/// copies (decision 142), with AppDef's as the source. The minimisation runs in the constructor.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class BSpGradientBfgs<MultiLine, ToolLine> : Bfgs
    where ToolLine : struct, ILineTool<MultiLine>
{
    private readonly double _tol3d;
    private readonly double _tol2d;

    /// <summary>Minimises <paramref name="f"/> from <paramref name="startingPoint"/>.</summary>
    public BSpGradientBfgs(MultipleVarFunctionWithGradient f, math_Vector startingPoint, double tolerance3d,
                                      double tolerance2d, double eps, int nbIterations = 200)
        : base(f.VariableCount, eps, nbIterations, eps)
    {
        _tol3d = tolerance3d;
        _tol2d = tolerance2d;
        Perform(f, startingPoint);
    }

    /// <summary>
    /// True when the minimum no longer decreases, or when the errors of the function (an
    /// <see cref="BSpFunction{MultiLine, ToolLine}"/>) are within the tolerances.
    /// </summary>
    public override bool IsSolutionReached(MultipleVarFunctionWithGradient f)
    {
        BSpFunction<MultiLine, ToolLine> f1 = (BSpFunction<MultiLine, ToolLine>)f;
        bool result, result2;
        result = (2.0 * Math.Abs(_theMinimum - _previousMinimum)
                  <= 1.0e-10 * (Math.Abs(_theMinimum) + Math.Abs(_previousMinimum)) + 1.0e-12);
        double mErr3d = f1.MaxError3d();
        double mErr2d = f1.MaxError2d();
        result2 = ((mErr3d <= _tol3d) && (mErr2d <= _tol2d));
        return (result || result2);
    }
}
