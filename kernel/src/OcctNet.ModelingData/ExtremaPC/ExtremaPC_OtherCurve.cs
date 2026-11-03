// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_OtherCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-curve extrema for any other curve: a uniform grid of 64 samples (THE_OTHER_CURVE_NB_SAMPLES) through GeomGridEval_OtherCurve on the caller's
/// adaptor, then Newton refinement (<see cref="ExtremaPC_GridEvaluator"/>).
/// </summary>
/// <remarks>Keeps the caller's adaptor, as OCCT keeps a pointer to it (decision 220). The domain is fixed at
/// construction.</remarks>
public sealed class ExtremaPC_OtherCurve : ExtremaPC.ICurveEvaluator
{
    private readonly Adaptor3d_Curve myCurve;             // Curve adaptor (not owned)
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D myDomain;         // Parameter domain (fixed)

    private readonly ExtremaPC_GridEvaluator myEvaluator = new();

    /// <summary>Constructor with curve adaptor (uses its natural domain).</summary>
    public ExtremaPC_OtherCurve(Adaptor3d_Curve theCurve)
    {
        myCurve = theCurve;
        myDomain = new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(theCurve.FirstParameter(), theCurve.LastParameter());
        buildGrid();
    }

    /// <summary>Constructor with curve adaptor and parameter domain (fixed for all queries).</summary>
    public ExtremaPC_OtherCurve(Adaptor3d_Curve theCurve, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myCurve = theCurve;
        myDomain = theDomain;
        buildGrid();
    }

    /// <summary>Evaluates point on curve at parameter.</summary>
    public Point3D Value(double theU)
    {
        return myCurve.Value(theU);
    }

    /// <summary>Returns true: the curve is treated as bounded.</summary>
    public bool IsBounded() => true;

    /// <summary>Returns the domain.</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => myDomain;

    /// <summary>Computes the interior extrema between point P and the curve.</summary>
    public ExtremaPC.Result Perform(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        if (myCurve is null)
        {
            myEvaluator.Result().Clear();
            myEvaluator.Result().Status = ExtremaPC.Status.NotDone;
            return myEvaluator.Result();
        }

        return myEvaluator.Perform(myCurve, theP, myDomain, theTol, theMode);
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        // Get interior extrema (populates myEvaluator's result)
        _ = myEvaluator.Perform(myCurve, theP, myDomain, theTol, theMode);

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

    private void buildGrid()
    {
        if (myCurve is null)
        {
            return;
        }

        // Use higher number of samples for general curves
        math_Vector aParams =
          ExtremaPC_GridEvaluator.BuildUniformParams(myDomain.Min,
                                                     myDomain.Max,
                                                     ExtremaPC.THE_OTHER_CURVE_NB_SAMPLES);

        GeomGridEval_OtherCurve aGridEval = new(myCurve);
        myEvaluator.BuildGrid(aGridEval, aParams);
    }
}
