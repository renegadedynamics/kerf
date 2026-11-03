// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Occt;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>OCCT's instantiation of the least-squares approximation over the <see cref="MultiLine{TheLine}"/> of BRepApprox_ApproxLine: <see cref="LeastSquare{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over BRepApprox_ApproxLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
public sealed class BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox : LeastSquare<global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine>, MultiLineTool<ApproxLine>>
{
    /// <summary>As the base constructor.</summary>
    public BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine> ssp, int firstPoint,
                                                                    int lastPoint, Constraint firstCons,
                                                                    Constraint lastCons,
                                                                    math_Vector parameters, int nbPol)
        : base(ssp, firstPoint, lastPoint, firstCons, lastCons, parameters, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine> ssp, int firstPoint,
                                                                    int lastPoint, Constraint firstCons,
                                                                    Constraint lastCons, int nbPol)
        : base(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine> ssp,
                                                                    NCollection_Array1<double> knots,
                                                                    NCollection_Array1<int> mults, int firstPoint,
                                                                    int lastPoint, Constraint firstCons,
                                                                    Constraint lastCons,
                                                                    math_Vector parameters, int nbPol)
        : base(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, parameters, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public BSpParLeastSquareOfMyBSplGradientOfTheComputeLineOfApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<ApproxLine> ssp,
                                                                    NCollection_Array1<double> knots,
                                                                    NCollection_Array1<int> mults, int firstPoint,
                                                                    int lastPoint, Constraint firstCons,
                                                                    Constraint lastCons, int nbPol)
        : base(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, nbPol) { }
}
