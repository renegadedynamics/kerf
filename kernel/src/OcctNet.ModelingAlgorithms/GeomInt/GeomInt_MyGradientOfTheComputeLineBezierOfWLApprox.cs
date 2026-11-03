// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_MyGradientOfTheComputeLineBezierOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_MyGradientOfTheComputeLineBezierOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;

namespace Occt;

/// <summary>OCCT's instantiation of the Bezier approximation with parameter optimisation over the <see cref="ApproxInt_MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="Gradient{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class GeomInt_MyGradientOfTheComputeLineBezierOfWLApprox : Gradient<ApproxInt_MultiLine<IntPatch_WLine>, ApproxInt_MultiLineTool<IntPatch_WLine>>
{
    /// <summary>As the base constructor.</summary>
    public GeomInt_MyGradientOfTheComputeLineBezierOfWLApprox(ApproxInt_MultiLine<IntPatch_WLine> SSP, int FirstPoint, int LastPoint,
                                      NCollection_HArray1<ConstraintCouple> TheConstraints,
                                      math_Vector Parameters, int Deg, double Tol3d, double Tol2d,
                                      int NbIterations = 200)
        : base(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Deg, Tol3d, Tol2d, NbIterations) { }
}
