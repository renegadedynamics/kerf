// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Newton.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Newton.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The optimisation algorithm of the fair curves: math_NewtonMinimum with a spatial convergence test.</summary>
public class FairCurve_Newton : math_NewtonMinimum
{
    private readonly double mySpTol;

    /// <summary>
    /// Prepares the algorithm (no computation). Iterations stop if (!WithSingularity) and the hessian is not definite
    /// positive (its smallest eigenvalue below <paramref name="theConvexity"/>), or when <see cref="IsConverged"/> is
    /// true for 2 successive iterations.
    /// </summary>
    public FairCurve_Newton(math_MultipleVarFunctionWithHessian theFunction,
                            double theSpatialTolerance = 1.0e-7,
                            double theCriteriumTolerance = 1.0e-7,
                            int theNbIterations = 40,
                            double theConvexity = 1.0e-6,
                            bool theWithSingularity = true)
        : base(theFunction, theCriteriumTolerance, theNbIterations, theConvexity, theWithSingularity)
    {
        mySpTol = theSpatialTolerance;
    }

    /// <summary>
    /// Converged if the step is too small (|Xi+1 - Xi| &lt;= SpatialTolerance / 100), or if the criterion progresses
    /// little with a reasonable step (|Xi+1 - Xi| &lt;= SpatialTolerance and |F(Xi+1) - F(Xi)| &lt;=
    /// CriteriumTolerance * |F(Xi)|), which detects infinite slidings (a criterion varying too slowly).
    /// </summary>
    public override bool IsConverged()
    {
        double N = TheStep.Norm();
        return (N <= 0.01 * mySpTol)
               || (N <= mySpTol
                   && Math.Abs(TheMinimum - PreviousMinimum) <= XTol * Math.Abs(PreviousMinimum));
    }
}
