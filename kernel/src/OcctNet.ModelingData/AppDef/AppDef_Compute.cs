// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Compute.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Compute_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Approx_ParametrizationType;

namespace Occt;

/// <summary>
/// Approximates an <see cref="AppDef_MultiLine"/> by a sequence of Bezier MultiCurves: OCCT's instantiation of
/// <see cref="Approx_ComputeLine{MultiLine, LineTool}"/> over AppDef_MyLineTool.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 142 and 1602); the members are the base class's.</remarks>
public sealed class AppDef_Compute : Approx_ComputeLine<AppDef_MultiLine, AppDef_MyLineTool>
{
    /// <summary>
    /// Approximates <paramref name="Line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, cutting the line if <paramref name="cutting"/>.
    /// With <paramref name="Squares"/>, without iterations.
    /// </summary>
    public AppDef_Compute(AppDef_MultiLine Line, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-3,
                          double Tolerance2d = 1.0e-6, int NbIterations = 5, bool cutting = true,
                          Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
        : base(Line, degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, parametrization, Squares) { }

    /// <summary>As the constructor above, starting from the parameters <paramref name="Parameters"/>.</summary>
    public AppDef_Compute(AppDef_MultiLine Line, math_Vector Parameters, int degreemin = 4, int degreemax = 8,
                          double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06, int NbIterations = 5,
                          bool cutting = true, bool Squares = false)
        : base(Line, Parameters, degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, Squares) { }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="Parameters"/>.</summary>
    public AppDef_Compute(math_Vector Parameters, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03,
                          double Tolerance2d = 1.0e-06, int NbIterations = 5, bool cutting = true, bool Squares = false)
        : base(Parameters, degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, Squares) { }

    /// <summary>Initialises the fields of the algorithm; <c>Perform</c> runs it.</summary>
    public AppDef_Compute(int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06,
                          int NbIterations = 5, bool cutting = true,
                          Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
        : base(degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, parametrization, Squares) { }
}
