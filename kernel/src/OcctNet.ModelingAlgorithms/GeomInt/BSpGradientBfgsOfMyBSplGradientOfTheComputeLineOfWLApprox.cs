// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_BSpGradient_BFGSOfMyBSplGradientOfTheComputeLineOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_BSpGradient_BFGSOfMyBSplGradientOfTheComputeLineOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>OCCT's instantiation of the BFGS minimisation of the B-spline gradient over the <see cref="MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="BSpGradientBfgs{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class BSpGradientBfgsOfMyBSplGradientOfTheComputeLineOfWLApprox : BSpGradientBfgs<global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine>, MultiLineTool<WLine>>
{
    /// <summary>As the base constructor.</summary>
    public BSpGradientBfgsOfMyBSplGradientOfTheComputeLineOfWLApprox(MultipleVarFunctionWithGradient f,
                                                                   math_Vector startingPoint, double tolerance3d,
                                                                   double tolerance2d, double eps,
                                                                   int nbIterations = 200)
        : base(f, startingPoint, tolerance3d, tolerance2d, eps, nbIterations) { }
}
