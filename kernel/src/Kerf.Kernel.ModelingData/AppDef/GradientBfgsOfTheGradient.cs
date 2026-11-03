// AppDef_Gradient_BFGSOfTheGradient.hxx and _0.cxx are listed as ported by AppParCurves_Gradient_BFGS.cs, whose body is AppDef's copy.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>OCCT's instantiation of the BFGS minimisation over <see cref="MultiLine"/>: <see cref="GradientBfgs{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class GradientBfgsOfTheGradient : GradientBfgs<MultiLine, MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public GradientBfgsOfTheGradient(MultipleVarFunctionWithGradient f, math_Vector startingPoint,
                                             double tolerance3d, double tolerance2d, double eps,
                                             int nbIterations = 200)
        : base(f, startingPoint, tolerance3d, tolerance2d, eps, nbIterations) { }
}
