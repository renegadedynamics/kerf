// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Newton.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Newton.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>The optimisation algorithm of the fair curves: math_NewtonMinimum with a spatial convergence test.</summary>
public class Newton : NewtonMinimum
{
    private readonly double _spTol;

    /// <summary>
    /// Prepares the algorithm (no computation). Iterations stop if (!WithSingularity) and the hessian is not definite
    /// positive (its smallest eigenvalue below <paramref name="convexity"/>), or when <see cref="IsConverged"/> is
    /// true for 2 successive iterations.
    /// </summary>
    public Newton(MultipleVarFunctionWithHessian function,
                            double spatialTolerance = 1.0e-7,
                            double criteriumTolerance = 1.0e-7,
                            int nbIterations = 40,
                            double convexity = 1.0e-6,
                            bool withSingularity = true)
        : base(function, criteriumTolerance, nbIterations, convexity, withSingularity)
    {
        _spTol = spatialTolerance;
    }

    /// <summary>
    /// Converged if the step is too small (|Xi+1 - Xi| &lt;= SpatialTolerance / 100), or if the criterion progresses
    /// little with a reasonable step (|Xi+1 - Xi| &lt;= SpatialTolerance and |F(Xi+1) - F(Xi)| &lt;=
    /// CriteriumTolerance * |F(Xi)|), which detects infinite slidings (a criterion varying too slowly).
    /// </summary>
    public override bool IsConverged
    {
        get
        {
            double n = _theStep.Norm();
            return (n <= 0.01 * _spTol)
                   || (n <= _spTol
                       && Math.Abs(_theMinimum - _previousMinimum) <= _xTol * Math.Abs(_previousMinimum));
        }
    }
}
