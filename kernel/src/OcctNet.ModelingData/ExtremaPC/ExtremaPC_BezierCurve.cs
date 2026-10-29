// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BezierCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Point-Bezier curve extrema: a uniform grid of max(24, 3 (degree + 1)) samples, built once at construction
/// through GeomGridEval_BezierCurve, then Newton refinement (<see cref="ExtremaPC_GridEvaluator"/>).
/// </summary>
/// <remarks>The domain is fixed at construction. <c>Perform</c> returns the evaluator's own reusable result,
/// which the next call overwrites.</remarks>
public sealed class ExtremaPC_BezierCurve : ExtremaPC.ICurveEvaluator
{
    private readonly Geom_BezierCurve myCurve;            // Bezier curve
    private readonly GeomAdaptor_Curve myAdaptor;         // Curve adaptor
    private readonly MathUtils.Domain1D myDomain;         // Parameter domain (fixed)
    private readonly int myNbSamples;                     // Number of samples

    // Grid evaluator with cached state (grid, result, temporary vectors)
    private readonly ExtremaPC_GridEvaluator myEvaluator = new();

    /// <summary>Constructor with Bezier curve (uses its natural domain).</summary>
    public ExtremaPC_BezierCurve(Geom_BezierCurve theCurve)
    {
        myCurve = theCurve;
        myAdaptor = new GeomAdaptor_Curve(theCurve);
        myDomain = new MathUtils.Domain1D(theCurve.FirstParameter(), theCurve.LastParameter());
        myNbSamples = Max(ExtremaPC.THE_BEZIER_MIN_SAMPLES,
                          ExtremaPC.THE_BEZIER_DEGREE_MULTIPLIER * (theCurve.Degree() + 1));
        buildGrid();
    }

    /// <summary>Constructor with Bezier curve and parameter domain (fixed for all queries).</summary>
    public ExtremaPC_BezierCurve(Geom_BezierCurve theCurve, in MathUtils.Domain1D theDomain)
    {
        myCurve = theCurve;
        myAdaptor = new GeomAdaptor_Curve(theCurve);
        myDomain = theDomain;
        myNbSamples = Max(ExtremaPC.THE_BEZIER_MIN_SAMPLES,
                          ExtremaPC.THE_BEZIER_DEGREE_MULTIPLIER * (theCurve.Degree() + 1));
        buildGrid();
    }

    /// <summary>Evaluates point on curve at parameter.</summary>
    public gp_Pnt Value(double theU)
    {
        return myCurve.Value(theU);
    }

    /// <summary>Returns true: Bezier curves are always bounded.</summary>
    public bool IsBounded() => true;

    /// <summary>Returns the domain.</summary>
    public MathUtils.Domain1D Domain() => myDomain;

    /// <summary>Computes the interior extrema between point P and the curve.</summary>
    public ExtremaPC.Result Perform(in gp_Pnt theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        if (myCurve is null)
        {
            myEvaluator.Result().Clear();
            myEvaluator.Result().Status = ExtremaPC.Status.NotDone;
            return myEvaluator.Result();
        }

        return myEvaluator.Perform(myAdaptor, theP, myDomain, theTol, theMode);
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in gp_Pnt theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        // Get interior extrema (populates myEvaluator's result)
        _ = myEvaluator.Perform(myAdaptor, theP, myDomain, theTol, theMode);

        // Add endpoints to the result
        ExtremaPC.Result aResult = myEvaluator.Result();
        if (aResult.Status == ExtremaPC.Status.OK || aResult.Status == ExtremaPC.Status.NoSolution)
        {
            ExtremaPC.AddEndpointExtrema(aResult, theP, myDomain, this, theTol, theMode);

            // Update status if we found any extrema (including endpoints)
            if (!aResult.Extrema.IsEmpty())
            {
                aResult.Status = ExtremaPC.Status.OK;
            }
        }

        return aResult;
    }

    /// <summary>Returns the Bezier curve.</summary>
    public Geom_BezierCurve Curve() => myCurve;

    private void buildGrid()
    {
        if (myCurve is null)
        {
            return;
        }

        math_Vector aParams =
          ExtremaPC_GridEvaluator.BuildUniformParams(myDomain.Min, myDomain.Max, myNbSamples);

        GeomGridEval_BezierCurve aGridEval = new(myCurve);
        myEvaluator.BuildGrid(aGridEval, aParams);
    }
}
