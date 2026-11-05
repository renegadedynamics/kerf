// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Patches;
using Occt;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>OCCT's instantiation of the least-squares approximation over the <see cref="MultiLine{TheLine}"/> of IntPatch_WLine: <see cref="LeastSquare{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with ApproxInt_MultiLineTool over IntPatch_WLine; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
internal sealed class ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox : LeastSquare<global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine>, MultiLineTool<WLine>>
{
    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine> ssp, int firstPoint, int lastPoint,
                                                                Constraint firstCons,
                                                                Constraint lastCons,
                                                                math_Vector parameters, int nbPol)
        : base(ssp, firstPoint, lastPoint, firstCons, lastCons, parameters, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine> ssp, int firstPoint, int lastPoint,
                                                                Constraint firstCons,
                                                                Constraint lastCons, int nbPol)
        : base(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine> ssp,
                                                                NCollection_Array1<double> knots,
                                                                NCollection_Array1<int> mults, int firstPoint,
                                                                int lastPoint, Constraint firstCons,
                                                                Constraint lastCons,
                                                                math_Vector parameters, int nbPol)
        : base(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, parameters, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfTheComputeLineOfWLApprox(global::Kerf.Engine.Intersection.Approximation.MultiLine<WLine> ssp,
                                                                NCollection_Array1<double> knots,
                                                                NCollection_Array1<int> mults, int firstPoint,
                                                                int lastPoint, Constraint firstCons,
                                                                Constraint lastCons, int nbPol)
        : base(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, nbPol) { }
}
