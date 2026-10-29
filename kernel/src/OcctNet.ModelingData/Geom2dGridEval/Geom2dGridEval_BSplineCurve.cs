// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_BSplineCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_BSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D B-spline curve grid points: parameters are located span by span, and the
/// span's BSplCLib_Cache is built once for a run of at least four consecutive parameters in the same span (fewer
/// are evaluated directly by BSplCLib).
/// </summary>
/// <remarks>
/// The runs are counted over consecutive parameters, so ascending parameters make the most of the cache; any
/// order gives correct values.
/// </remarks>
public sealed class Geom2dGridEval_BSplineCurve : Geom2dGridEval.ICurveEvaluator
{
    // Threshold for using cache vs direct evaluation.
    private const int THE_CACHE_THRESHOLD = 4;

    // Helper structure holding extracted 2D B-spline curve data.
    private readonly struct CurveData
    {
        public readonly NCollection_Array1<double> FlatKnots;
        public readonly NCollection_Array1<gp_Pnt2d> Poles;
        public readonly NCollection_Array1<double>? Weights;
        public readonly NCollection_Array1<double> Knots;
        public readonly NCollection_Array1<int> Mults;
        public readonly int Degree;
        public readonly bool IsPeriodic;

        public CurveData(Geom2d_BSplineCurve theGeom)
        {
            FlatKnots = theGeom.InternalKnotSequence();
            Poles = theGeom.InternalPoles();
            Weights = theGeom.InternalWeights();
            Knots = theGeom.InternalKnots();
            Mults = theGeom.InternalMultiplicities();
            Degree = theGeom.Degree();
            IsPeriodic = theGeom.IsPeriodic();
        }
    }

    // The two functors of evaluateGridCached: (cache, localParam, result) and (data, param, spanIdx, result).
    private interface IEvalFunctor<TResult>
    {
        static abstract void CacheEval(BSplCLib_Cache theCache, double theLocalParam, ref TResult theResult);

        static abstract void DirectEval(in CurveData theData, double theParam, int theSpanIdx, ref TResult theResult);
    }

    private readonly Geom2d_BSplineCurve? myGeom;

    /// <summary>Constructor with the B-spline curve to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_BSplineCurve(Geom2d_BSplineCurve? theCurve)
    {
        myGeom = theCurve;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_BSplineCurve? Geometry() => myGeom;

    // Extract curve data from geometry. Returns false if data is invalid.
    private static bool extractCurveData(Geom2d_BSplineCurve? theGeom, out CurveData theData)
    {
        if (theGeom is null)
        {
            theData = default;
            return false;
        }

        theData = new CurveData(theGeom);
        return true;
    }

    // Locate parameter and compute span index.
    private static void locateSpan(double theParam,
                                   NCollection_Array1<double> theFlatKnots,
                                   int theDegree,
                                   bool theIsPeriodic,
                                   out double theAdjustedParam,
                                   ref int theSpanIdx)
    {
        BSplCLib.LocateParameter(theDegree,
                                 theFlatKnots,
                                 BSplCLib.NoMults(),
                                 theParam,
                                 theIsPeriodic,
                                 ref theSpanIdx,
                                 out theAdjustedParam);
    }

    // Count consecutive parameters in the same span (for cache threshold decision).
    private static int countSpanSize(NCollection_Array1<double> theParams,
                                     NCollection_Array1<double> theFlatKnots,
                                     int theDegree,
                                     bool theIsPeriodic,
                                     int theStartIdx,
                                     int theTargetSpan)
    {
        int aLower = theParams.Lower();
        int aNb = theParams.Length();
        int aCount = 1;

        for (int i = theStartIdx + 1; i < aNb; ++i)
        {
            double aParam = theParams.Value(aLower + i);
            int aSpan = 0;
            BSplCLib.LocateParameter(theDegree,
                                     theFlatKnots,
                                     BSplCLib.NoMults(),
                                     aParam,
                                     theIsPeriodic,
                                     ref aSpan,
                                     out double aAdjusted);
            _ = aAdjusted;
            if (aSpan == theTargetSpan)
            {
                ++aCount;
            }
            else
            {
                break;
            }
        }
        return aCount;
    }

    // Compute span local parameter coefficients (start and length).
    // Note: Curve cache uses [0,1] local parameter range (unlike surface which uses [-1,1]).
    private static void computeSpanCoeffs(NCollection_Array1<double> theFlatKnots,
                                          int theSpan,
                                          out double theStart,
                                          out double theLen)
    {
        theStart = theFlatKnots.Value(theSpan);
        theLen = theFlatKnots.Value(theSpan + 1) - theStart;
    }

    // Template helper for cached grid evaluation.
    private static NCollection_Array1<TResult> evaluateGridCached<TResult, TFunctor>(in CurveData theData,
                                                                                     NCollection_Array1<double> theParams)
        where TFunctor : struct, IEvalFunctor<TResult>
    {
        int aNbParams = theParams.Length();
        int aLow = theParams.Lower();

        NCollection_Array1<TResult> aResults = new(1, aNbParams);
        BSplCLib_Cache aCache = new(theData.Degree, theData.IsPeriodic, theData.FlatKnots, theData.Poles, theData.Weights);

        int aPrevSpan = -1;
        int aSpanSize = 0;
        double aSpanStart = 0.0;
        double aSpanLen = 1.0;
        bool aUseCache = false;

        for (int i = 0; i < aNbParams; ++i)
        {
            double aParam = theParams.Value(aLow + i);
            int aSpan = 0;
            locateSpan(aParam, theData.FlatKnots, theData.Degree, theData.IsPeriodic, out double aAdjParam, ref aSpan);

            // Update span info when span changes
            if (aSpan != aPrevSpan)
            {
                aPrevSpan = aSpan;
                aSpanSize = countSpanSize(theParams, theData.FlatKnots, theData.Degree, theData.IsPeriodic, i, aSpan);
                computeSpanCoeffs(theData.FlatKnots, aSpan, out aSpanStart, out aSpanLen);

                aUseCache = (aSpanSize >= THE_CACHE_THRESHOLD);
                if (aUseCache)
                {
                    aCache.BuildCache(aAdjParam, theData.FlatKnots, theData.Poles, theData.Weights);
                }
            }

            ref TResult aResult = ref aResults.ChangeValue(i + 1);
            if (aUseCache)
            {
                // Curve cache uses [0,1] local parameter range
                double aLocalParam = (aSpanLen > 0.0) ? (aAdjParam - aSpanStart) / aSpanLen : 0.0;
                TFunctor.CacheEval(aCache, aLocalParam, ref aResult);
            }
            else
            {
                TFunctor.DirectEval(theData, aAdjParam, aSpan, ref aResult);
            }
        }

        return aResults;
    }

    private struct EvalD0 : IEvalFunctor<gp_Pnt2d>
    {
        public static void CacheEval(BSplCLib_Cache theCache, double theLocalParam, ref gp_Pnt2d theResult)
        {
            theCache.D0Local(theLocalParam, out theResult);
        }

        public static void DirectEval(in CurveData theData, double theParam, int theSpanIdx, ref gp_Pnt2d theResult)
        {
            BSplCLib.D0(theParam,
                        theSpanIdx,
                        theData.Degree,
                        theData.IsPeriodic,
                        theData.Poles,
                        theData.Weights,
                        theData.FlatKnots,
                        null,
                        out theResult);
        }
    }

    private struct EvalD1 : IEvalFunctor<Geom2d_Curve.ResD1>
    {
        public static void CacheEval(BSplCLib_Cache theCache, double theLocalParam, ref Geom2d_Curve.ResD1 theResult)
        {
            theCache.D1Local(theLocalParam, out theResult.Point, out theResult.D1);
        }

        public static void DirectEval(in CurveData theData, double theParam, int theSpanIdx,
                                      ref Geom2d_Curve.ResD1 theResult)
        {
            BSplCLib.D1(theParam,
                        theSpanIdx,
                        theData.Degree,
                        theData.IsPeriodic,
                        theData.Poles,
                        theData.Weights,
                        theData.FlatKnots,
                        null,
                        out theResult.Point,
                        out theResult.D1);
        }
    }

    private struct EvalD2 : IEvalFunctor<Geom2d_Curve.ResD2>
    {
        public static void CacheEval(BSplCLib_Cache theCache, double theLocalParam, ref Geom2d_Curve.ResD2 theResult)
        {
            theCache.D2Local(theLocalParam, out theResult.Point, out theResult.D1, out theResult.D2);
        }

        public static void DirectEval(in CurveData theData, double theParam, int theSpanIdx,
                                      ref Geom2d_Curve.ResD2 theResult)
        {
            BSplCLib.D2(theParam,
                        theSpanIdx,
                        theData.Degree,
                        theData.IsPeriodic,
                        theData.Poles,
                        theData.Weights,
                        theData.FlatKnots,
                        null,
                        out theResult.Point,
                        out theResult.D1,
                        out theResult.D2);
        }
    }

    private struct EvalD3 : IEvalFunctor<Geom2d_Curve.ResD3>
    {
        public static void CacheEval(BSplCLib_Cache theCache, double theLocalParam, ref Geom2d_Curve.ResD3 theResult)
        {
            theCache.D3Local(theLocalParam, out theResult.Point, out theResult.D1, out theResult.D2, out theResult.D3);
        }

        public static void DirectEval(in CurveData theData, double theParam, int theSpanIdx,
                                      ref Geom2d_Curve.ResD3 theResult)
        {
            BSplCLib.D3(theParam,
                        theSpanIdx,
                        theData.Degree,
                        theData.IsPeriodic,
                        theData.Poles,
                        theData.Weights,
                        theData.FlatKnots,
                        null,
                        out theResult.Point,
                        out theResult.D1,
                        out theResult.D2,
                        out theResult.D3);
        }
    }

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt2d> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !extractCurveData(myGeom, out CurveData aData))
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        return evaluateGridCached<gp_Pnt2d, EvalD0>(aData, theParams);
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !extractCurveData(myGeom, out CurveData aData))
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        return evaluateGridCached<Geom2d_Curve.ResD1, EvalD1>(aData, theParams);
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !extractCurveData(myGeom, out CurveData aData))
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        return evaluateGridCached<Geom2d_Curve.ResD2, EvalD2>(aData, theParams);
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !extractCurveData(myGeom, out CurveData aData))
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        return evaluateGridCached<Geom2d_Curve.ResD3, EvalD3>(aData, theParams);
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points, directly with BSplCLib (zero above the degree).
    /// </summary>
    /// <remarks>
    /// As in OCCT, the values are wrong (or, where OCCT reads outside the knot array, an
    /// <see cref="IndexOutOfRangeException"/> is thrown) wherever the span's flat-knot index is also a valid knot
    /// index, and zero above the degree for rational curves too (occt-issues.md).
    /// </remarks>
    public NCollection_Array1<gp_Vec2d> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (theParams.IsEmpty() || theN < 1 || !extractCurveData(myGeom, out CurveData aData))
        {
            return new NCollection_Array1<gp_Vec2d>();
        }

        int aNbParams = theParams.Length();
        int aLow = theParams.Lower();

        // Derivatives beyond degree are zero
        // OCCT: not for a rational curve, whose derivatives above the degree are generally not zero
        if (theN > aData.Degree)
        {
            NCollection_Array1<gp_Vec2d> aZeroResult = new(1, aNbParams);
            gp_Vec2d aZero = new(0.0, 0.0);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aZeroResult.SetValue(i, aZero);
            }
            return aZeroResult;
        }

        NCollection_Array1<gp_Vec2d> aResult = new(1, aNbParams);

        // DN uses direct evaluation (no cache available)
        for (int i = 0; i < aNbParams; ++i)
        {
            double aParam = theParams.Value(aLow + i);
            int aSpan = 0;
            locateSpan(aParam, aData.FlatKnots, aData.Degree, aData.IsPeriodic, out double aAdjParam, ref aSpan);

            // OCCT: aSpan indexes the flat knots, but BSplCLib::DN takes it as an index into the knots given with
            // their multiplicities. When it is a valid knot index, DN evaluates that span's polynomial at the
            // parameter (wrong values); building the span's knots may then read past the knots (garbage in OCCT,
            // IndexOutOfRangeException here). See occt-issues.md.
            BSplCLib.DN(aAdjParam,
                        theN,
                        aSpan,
                        aData.Degree,
                        aData.IsPeriodic,
                        aData.Poles,
                        aData.Weights,
                        aData.Knots,
                        aData.Mults,
                        out gp_Vec2d aDN);
            aResult.SetValue(i + 1, aDN);
        }

        return aResult;
    }
}
