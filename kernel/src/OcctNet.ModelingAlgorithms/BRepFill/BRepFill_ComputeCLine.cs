// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ComputeCLine.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ComputeCLine_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using static Kerf.Engine.Approximation.Constraint;

namespace Occt;

/// <summary>
/// OCCT's instantiation of Approx_ComputeCLine.gxx over <see cref="BRepFill_MultiLine"/>: approximates the bisector
/// line of two faces by Bezier MultiCurves until the tolerances are reached.
/// </summary>
/// <remarks>
/// BRepFill_MultiLine is an AppCont_Function, so this is a third sealed subclass of the generic body, as
/// Approx_FitAndDivide and Approx_FitAndDivide2d; <see cref="ComputeCLine.Perform"/> takes the line.
/// </remarks>
public sealed class BRepFill_ComputeCLine : ComputeCLine
{
    /// <summary>
    /// Approximates Line until the tolerances are reached, from degreemin to degreemax, cutting the range if
    /// cutting is true.
    /// </summary>
    public BRepFill_ComputeCLine(BRepFill_MultiLine Line,
                                 int degreemin = 3,
                                 int degreemax = 8,
                                 double Tolerance3d = 1.0e-5,
                                 double Tolerance2d = 1.0e-5,
                                 bool cutting = false,
                                 Constraint FirstC = TangencyPoint,
                                 Constraint LastC = TangencyPoint)
        : base(Line, degreemin, degreemax, Tolerance3d, Tolerance2d, cutting, FirstC, LastC)
    {
    }

    /// <summary>Initializes the fields of the algorithm; <see cref="ComputeCLine.Perform"/> runs it.</summary>
    public BRepFill_ComputeCLine(int degreemin = 3,
                                 int degreemax = 8,
                                 double Tolerance3d = 1.0e-05,
                                 double Tolerance2d = 1.0e-05,
                                 bool cutting = false,
                                 Constraint FirstC = TangencyPoint,
                                 Constraint LastC = TangencyPoint)
        : base(degreemin, degreemax, Tolerance3d, Tolerance2d, cutting, FirstC, LastC)
    {
    }
}
