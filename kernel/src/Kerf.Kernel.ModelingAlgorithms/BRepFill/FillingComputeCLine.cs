// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ComputeCLine.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ComputeCLine_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Occt;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// OCCT's instantiation of Approx_ComputeCLine.gxx over <see cref="FillingMultiLine"/>: approximates the bisector
/// line of two faces by Bezier MultiCurves until the tolerances are reached.
/// </summary>
/// <remarks>
/// BRepFill_MultiLine is an AppCont_Function, so this is a third sealed subclass of the generic body, as
/// Approx_FitAndDivide and Approx_FitAndDivide2d; <see cref="ComputeCLine.Perform"/> takes the line.
/// </remarks>
internal sealed class FillingComputeCLine : ComputeCLine
{
    /// <summary>
    /// Approximates Line until the tolerances are reached, from degreemin to degreemax, cutting the range if
    /// cutting is true.
    /// </summary>
    public FillingComputeCLine(FillingMultiLine line,
                                 int degreemin = 3,
                                 int degreemax = 8,
                                 double tolerance3d = 1.0e-5,
                                 double tolerance2d = 1.0e-5,
                                 bool cutting = false,
                                 Constraint firstC = TangencyPoint,
                                 Constraint lastC = TangencyPoint)
        : base(line, degreemin, degreemax, tolerance3d, tolerance2d, cutting, firstC, lastC)
    {
    }

    /// <summary>Initializes the fields of the algorithm; <see cref="ComputeCLine.Perform"/> runs it.</summary>
    public FillingComputeCLine(int degreemin = 3,
                                 int degreemax = 8,
                                 double tolerance3d = 1.0e-05,
                                 double tolerance2d = 1.0e-05,
                                 bool cutting = false,
                                 Constraint firstC = TangencyPoint,
                                 Constraint lastC = TangencyPoint)
        : base(degreemin, degreemax, tolerance3d, tolerance2d, cutting, firstC, lastC)
    {
    }
}
