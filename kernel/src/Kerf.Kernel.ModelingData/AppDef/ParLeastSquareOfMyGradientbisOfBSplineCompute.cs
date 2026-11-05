// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ParLeastSquareOfMyGradientbisOfBSplineCompute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_ParLeastSquareOfMyGradientbisOfBSplineCompute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Approximation;

/// <summary>OCCT's instantiation of the least-squares approximation over <see cref="MultiLine"/>: <see cref="LeastSquare{MultiLine, ToolLine}"/>.</summary>
/// <remarks>
/// OCCT generates this class from AppParCurves' generic code with AppDef_MyLineTool; the port derives it from the
/// generic class, which the generic algorithms use directly (decisions 142 and 202).
/// </remarks>
internal sealed class ParLeastSquareOfMyGradientbisOfBSplineCompute : LeastSquare<MultiLine, MyLineTool>
{
    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfBSplineCompute(MultiLine ssp, int firstPoint, int lastPoint,
                                                                Constraint firstCons,
                                                                Constraint lastCons,
                                                                math_Vector parameters, int nbPol)
        : base(ssp, firstPoint, lastPoint, firstCons, lastCons, parameters, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfBSplineCompute(MultiLine ssp, int firstPoint, int lastPoint,
                                                                Constraint firstCons,
                                                                Constraint lastCons, int nbPol)
        : base(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfBSplineCompute(MultiLine ssp,
                                                                NCollection_Array1<double> knots,
                                                                NCollection_Array1<int> mults, int firstPoint,
                                                                int lastPoint, Constraint firstCons,
                                                                Constraint lastCons,
                                                                math_Vector parameters, int nbPol)
        : base(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, parameters, nbPol) { }

    /// <summary>As the base constructor.</summary>
    public ParLeastSquareOfMyGradientbisOfBSplineCompute(MultiLine ssp,
                                                                NCollection_Array1<double> knots,
                                                                NCollection_Array1<int> mults, int firstPoint,
                                                                int lastPoint, Constraint firstCons,
                                                                Constraint lastCons, int nbPol)
        : base(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, nbPol) { }
}
