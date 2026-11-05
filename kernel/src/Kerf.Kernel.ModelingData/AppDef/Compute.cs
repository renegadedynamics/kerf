// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Compute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Compute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Geometry.ParametrizationType;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates an <see cref="MultiLine"/> by a sequence of Bezier MultiCurves: OCCT's instantiation of
/// <see cref="ComputeLine{MultiLine, LineTool}"/> over AppDef_MyLineTool.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 142 and 202); the members are the base class's.</remarks>
internal sealed class Compute : ComputeLine<MultiLine, MyLineTool>
{
    /// <summary>
    /// Approximates <paramref name="line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, cutting the line if <paramref name="cutting"/>.
    /// With <paramref name="squares"/>, without iterations.
    /// </summary>
    public Compute(MultiLine line, int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-3,
                          double tolerance2d = 1.0e-6, int nbIterations = 5, bool cutting = true,
                          global::Kerf.Geometry.ParametrizationType parametrization = ChordLength, bool squares = false)
        : base(line, degreemin, degreemax, tolerance3d, tolerance2d, nbIterations, cutting, parametrization, squares) { }

    /// <summary>As the constructor above, starting from the parameters <paramref name="parameters"/>.</summary>
    public Compute(MultiLine line, math_Vector parameters, int degreemin = 4, int degreemax = 8,
                          double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06, int nbIterations = 5,
                          bool cutting = true, bool squares = false)
        : base(line, parameters, degreemin, degreemax, tolerance3d, tolerance2d, nbIterations, cutting, squares) { }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="parameters"/>.</summary>
    public Compute(math_Vector parameters, int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03,
                          double tolerance2d = 1.0e-06, int nbIterations = 5, bool cutting = true, bool squares = false)
        : base(parameters, degreemin, degreemax, tolerance3d, tolerance2d, nbIterations, cutting, squares) { }

    /// <summary>Initialises the fields of the algorithm; <c>Perform</c> runs it.</summary>
    public Compute(int degreemin = 4, int degreemax = 8, double tolerance3d = 1.0e-03, double tolerance2d = 1.0e-06,
                          int nbIterations = 5, bool cutting = true,
                          global::Kerf.Geometry.ParametrizationType parametrization = ChordLength, bool squares = false)
        : base(degreemin, degreemax, tolerance3d, tolerance2d, nbIterations, cutting, parametrization, squares) { }
}
