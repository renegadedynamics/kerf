// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-B-spline curve extrema: a knot-aware grid (2 (degree + 1) samples per knot span, the knots themselves
/// included), built once at construction through GeomGridEval_BSplineCurve, then Newton refinement
/// (<see cref="ExtremaPC_GridEvaluator"/>).
/// </summary>
/// <remarks>The domain is fixed at construction. The grid parameters are ascending, as GeomGridEval requires.
/// <c>Perform</c> returns the evaluator's own reusable result, which the next call overwrites.</remarks>
public sealed class ExtremaPC_BSplineCurve : ExtremaPC.ICurveEvaluator
{
    private readonly Geom_BSplineCurve myCurve;           // BSpline curve
    private readonly GeomAdaptor_Curve myAdaptor;         // Curve adaptor
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D myDomain;         // Parameter domain (fixed)

    // Grid evaluator with cached state (grid, result, temporary vectors)
    private readonly ExtremaPC_GridEvaluator myEvaluator = new();

    /// <summary>Constructor with B-spline curve (uses its natural domain).</summary>
    public ExtremaPC_BSplineCurve(Geom_BSplineCurve theCurve)
    {
        myCurve = theCurve;
        myAdaptor = new GeomAdaptor_Curve(theCurve);
        myDomain = new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(theCurve.FirstParameter(), theCurve.LastParameter());
        buildGrid();
    }

    /// <summary>Constructor with B-spline curve and parameter domain (fixed for all queries).</summary>
    public ExtremaPC_BSplineCurve(Geom_BSplineCurve theCurve, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myCurve = theCurve;
        myAdaptor = new GeomAdaptor_Curve(theCurve);
        myDomain = theDomain;
        buildGrid();
    }

    /// <summary>Evaluates point on curve at parameter.</summary>
    public Point3D Value(double theU)
    {
        return myCurve.Value(theU);
    }

    /// <summary>Returns true: B-spline curves are always bounded.</summary>
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

        return myEvaluator.Perform(myAdaptor, theP, myDomain, theTol, theMode);
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
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

    /// <summary>Returns the B-spline curve.</summary>
    public Geom_BSplineCurve Curve() => myCurve;

    private math_Vector buildKnotAwareParams()
    {
        if (myCurve is null)
        {
            // Fallback to uniform sampling
            return ExtremaPC_GridEvaluator.BuildUniformParams(myDomain.Min,
                                                              myDomain.Max,
                                                              ExtremaPC.THE_BSPLINE_FALLBACK_SAMPLES);
        }

        double theUMin = myDomain.Min;
        double theUMax = myDomain.Max;

        int aDegree = myCurve.Degree();
        NCollection_Array1<double> aKnots = myCurve.InternalKnots();

        // Use multiplier*(degree+1) samples per span for accurate extrema detection.
        int aSamplesPerSpan = ExtremaPC.THE_BSPLINE_SPAN_MULTIPLIER * (aDegree + 1);

        // Single-pass algorithm using dynamic vector
        NCollection_DynamicArray<double> aParams = new();
        aParams.Append(theUMin);

        for (int i = aKnots.Lower(); i < aKnots.Upper(); ++i)
        {
            double aKnotLo = aKnots.Value(i);
            double aKnotHi = aKnots.Value(i + 1);
            double aSpanLo = Max(aKnotLo, theUMin);
            double aSpanHi = Min(aKnotHi, theUMax);
            if (aSpanHi <= aSpanLo)
            {
                continue;
            }

            double aStep = (aSpanHi - aSpanLo) / aSamplesPerSpan;
            for (int j = 1; j < aSamplesPerSpan; ++j)
            {
                double aU = aSpanLo + j * aStep;
                if (aU > theUMin && aU < theUMax)
                {
                    aParams.Append(aU);
                }
            }
            if (aKnotHi > theUMin && aKnotHi < theUMax)
            {
                aParams.Append(aKnotHi);
            }
        }
        aParams.Append(theUMax);

        // Convert to math_Vector (required by BuildGrid interface)
        math_Vector aResult = new(1, aParams.Length());
        for (int i = 0; i < aParams.Length(); ++i)
        {
            aResult[i + 1] = aParams.Value(i);
        }

        return aResult;
    }

    private void buildGrid()
    {
        if (myCurve is null)
        {
            return;
        }

        // Build knot-aware parameter grid
        math_Vector aParams = buildKnotAwareParams();

        // Build grid using the evaluator
        GeomGridEval_BSplineCurve aGridEval = new(myCurve);
        myEvaluator.BuildGrid(aGridEval, aParams);
    }
}
