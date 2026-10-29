// Port of: src/ModelingData/TKGeomBase/Approx/Approx_FitAndDivide2d.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_FitAndDivide2d_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>
/// OCCT's instantiation of Approx_ComputeCLine.gxx over AppCont_Function (used for 2D lines): approximates a
/// continuous MultiLine by Bezier MultiCurves until the tolerances are reached.
/// </summary>
public sealed class Approx_FitAndDivide2d : Approx_ComputeCLine
{
    /// <summary>
    /// Approximates Line until the tolerances are reached, from degreemin to degreemax, cutting the range if
    /// cutting is true.
    /// </summary>
    public Approx_FitAndDivide2d(AppCont_Function Line,
                                 int degreemin = 3,
                                 int degreemax = 8,
                                 double Tolerance3d = 1.0e-5,
                                 double Tolerance2d = 1.0e-5,
                                 bool cutting = false,
                                 AppParCurves_Constraint FirstC = AppParCurves_TangencyPoint,
                                 AppParCurves_Constraint LastC = AppParCurves_TangencyPoint)
        : base(Line, degreemin, degreemax, Tolerance3d, Tolerance2d, cutting, FirstC, LastC)
    {
    }

    /// <summary>Initializes the fields of the algorithm; <see cref="Approx_ComputeCLine.Perform"/> runs it.</summary>
    public Approx_FitAndDivide2d(int degreemin = 3,
                                 int degreemax = 8,
                                 double Tolerance3d = 1.0e-05,
                                 double Tolerance2d = 1.0e-05,
                                 bool cutting = false,
                                 AppParCurves_Constraint FirstC = AppParCurves_TangencyPoint,
                                 AppParCurves_Constraint LastC = AppParCurves_TangencyPoint)
        : base(degreemin, degreemax, Tolerance3d, Tolerance2d, cutting, FirstC, LastC)
    {
    }
}
