// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheComputeLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheComputeLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using static Kerf.Engine.Approximation.ParametrizationType;

namespace Occt;

/// <summary>
/// Approximates an the <see cref="ApproxInt_MultiLine{TheLine}"/> of BRepApprox_ApproxLine by one MultiBSpCurve:
/// OCCT's instantiation of <see cref="BSplComputeLine{MultiLine, LineTool}"/> over ApproxInt_MultiLineTool and BRepApprox_ApproxLine.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 142 and 202); the members are the base class's.</remarks>
public sealed class BRepApprox_TheComputeLineOfApprox : BSplComputeLine<ApproxInt_MultiLine<BRepApprox_ApproxLine>, ApproxInt_MultiLineTool<BRepApprox_ApproxLine>>
{
    /// <summary>
    /// Approximates <paramref name="Line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, adding knots if <paramref name="cutting"/>.
    /// With <paramref name="Squares"/>, without iterations. The multiplicities of the interior knots are set by
    /// default.
    /// </summary>
    public BRepApprox_TheComputeLineOfApprox(ApproxInt_MultiLine<BRepApprox_ApproxLine> Line, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-3,
                                 double Tolerance2d = 1.0e-6, int NbIterations = 5, bool cutting = true,
                                 ParametrizationType parametrization = ChordLength, bool Squares = false)
        : base(Line, degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, parametrization, Squares) { }

    /// <summary>As the constructor above, starting from the parameters <paramref name="Parameters"/>.</summary>
    public BRepApprox_TheComputeLineOfApprox(ApproxInt_MultiLine<BRepApprox_ApproxLine> Line, math_Vector Parameters, int degreemin = 4, int degreemax = 8,
                                 double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06, int NbIterations = 5,
                                 bool cutting = true, bool Squares = false)
        : base(Line, Parameters, degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, Squares) { }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="Parameters"/>.</summary>
    public BRepApprox_TheComputeLineOfApprox(math_Vector Parameters, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03,
                                 double Tolerance2d = 1.0e-06, int NbIterations = 5, bool cutting = true,
                                 bool Squares = false)
        : base(Parameters, degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, Squares) { }

    /// <summary>Initialises the fields of the algorithm; <c>Perform</c> runs it.</summary>
    public BRepApprox_TheComputeLineOfApprox(int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03,
                                 double Tolerance2d = 1.0e-06, int NbIterations = 5, bool cutting = true,
                                 ParametrizationType parametrization = ChordLength, bool Squares = false)
        : base(degreemin, degreemax, Tolerance3d, Tolerance2d, NbIterations, cutting, parametrization, Squares) { }
}
