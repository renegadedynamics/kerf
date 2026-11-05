// Port of: src/ModelingData/TKGeomBase/Approx/Approx_FitAndDivide.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_FitAndDivide_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>
/// OCCT's instantiation of Approx_ComputeCLine.gxx over AppCont_Function (used for 3D lines): approximates a
/// continuous MultiLine by Bezier MultiCurves until the tolerances are reached.
/// </summary>
internal sealed class FitAndDivide : ComputeCLine
{
    /// <summary>
    /// Approximates Line until the tolerances are reached, from degreemin to degreemax, cutting the range if
    /// cutting is true.
    /// </summary>
    public FitAndDivide(Function line,
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
    public FitAndDivide(int degreemin = 3,
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
