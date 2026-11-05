// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BSplineCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for B-spline curve grid points: parameters are located span by span, and the
/// span's BSplCLib_Cache is built once for a run of at least two consecutive parameters in the same span (fewer
/// are evaluated directly by BSplCLib).
/// </summary>
/// <remarks>
/// As in OCCT, the parameters of a non-periodic curve must be ascending: a parameter below the current span's
/// end is evaluated on the current span without being located again.
/// </remarks>
internal sealed class BSplineCurveGrid : GeomGridEvalGrid.ICurveEvaluator
{
    // Threshold for using cache vs direct evaluation.
    private const int THE_CACHE_THRESHOLD = 2;

    // Helper structure holding extracted B-spline curve data.
    private readonly struct CurveData
    {
        public readonly NCollection_Array1<double> FlatKnots;
        public readonly NCollection_Array1<Point3D> Poles;
        public readonly NCollection_Array1<double>? Weights;
        public readonly NCollection_Array1<double> Knots;
        public readonly NCollection_Array1<int> Mults;
        public readonly int Degree;
        public readonly bool IsPeriodic;

        public CurveData(BSplineCurve3D geom)
        {
            FlatKnots = geom.InternalKnotSequence();
            Poles = geom.InternalPoles();
            Weights = geom.InternalWeights();
            Knots = geom.InternalKnots();
            Mults = geom.InternalMultiplicities();
            Degree = geom.Degree;
            IsPeriodic = geom.IsPeriodic;
        }
    }

    // The two functors of evaluateGridCached: (cache, localParam, result) and (data, param, spanIdx, result).
    private interface IEvalFunctor<TResult>
    {
        static abstract void CacheEval(Cache cache, double localParam, ref TResult result);

        static abstract void DirectEval(in CurveData data, double param, int spanIdx, ref TResult result);
    }

    private readonly BSplineCurve3D? _geom;

    /// <summary>Constructor with the B-spline curve to evaluate (null gives empty results).</summary>
    public BSplineCurveGrid(BSplineCurve3D? curve)
    {
        _geom = curve;
    }

    /// <summary>Returns the geometry.</summary>
    public BSplineCurve3D? Geometry() => _geom;

    // Extract curve data from geometry. Returns false if data is invalid.
    private static bool ExtractCurveData(BSplineCurve3D? geom, out CurveData data)
    {
        if (geom is null)
        {
            data = default;
            return false;
        }

        data = new CurveData(geom);
        return true;
    }

    // Locate parameter and compute span index.
    private static void LocateSpan(double param,
                                   NCollection_Array1<double> flatKnots,
                                   int degree,
                                   bool isPeriodic,
                                   out double adjustedParam,
                                   ref int spanIdx)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree,
                                 flatKnots,
                                 global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                                 param,
                                 isPeriodic,
                                 ref spanIdx,
                                 out adjustedParam);
    }

    // Count consecutive parameters in the same span (for cache threshold decision).
    // For non-periodic curves, uses a fast knot-boundary comparison instead of
    // calling BSplCLib::LocateParameter for every subsequent parameter.
    private static int CountSpanSize(NCollection_Array1<double> theParams,
                                     NCollection_Array1<double> flatKnots,
                                     int degree,
                                     bool isPeriodic,
                                     int startIdx,
                                     int targetSpan)
    {
        int lower = theParams.Lower();
        int nb = theParams.Length();
        int count = 1;

        if (!isPeriodic)
        {
            // Fast path: params are sorted (ascending from buildKnotAwareParams),
            // so just compare against the next knot boundary.
            if (targetSpan + 1 > flatKnots.Upper())
            {
                return count;
            }
            double nextKnot = flatKnots.Value(targetSpan + 1);
            for (int i = startIdx + 1; i < nb; ++i)
            {
                if (theParams.Value(lower + i) >= nextKnot)
                {
                    break;
                }
                ++count;
            }
            return count;
        }

        // Periodic fallback: span assignment requires full LocateParameter.
        for (int i = startIdx + 1; i < nb; ++i)
        {
            double param = theParams.Value(lower + i);
            int span = 0;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree,
                                     flatKnots,
                                     global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                                     param,
                                     isPeriodic,
                                     ref span,
                                     out double adjusted);
            _ = adjusted;
            if (span == targetSpan)
            {
                ++count;
            }
            else
            {
                break;
            }
        }
        return count;
    }

    // Compute span local parameter coefficients (start and length).
    // Note: Curve cache uses [0,1] local parameter range (unlike surface which uses [-1,1]).
    private static void ComputeSpanCoeffs(NCollection_Array1<double> flatKnots,
                                          int span,
                                          out double start,
                                          out double len)
    {
        start = flatKnots.Value(span);
        len = flatKnots.Value(span + 1) - start;
    }

    // Template helper for cached grid evaluation.
    private static NCollection_Array1<TResult> EvaluateGridCached<TResult, TFunctor>(in CurveData data,
                                                                                     NCollection_Array1<double> theParams)
        where TFunctor : struct, IEvalFunctor<TResult>
    {
        int nbParams = theParams.Length();
        int low = theParams.Lower();

        NCollection_Array1<TResult> results = new(1, nbParams);
        Cache cache = new(data.Degree, data.IsPeriodic, data.FlatKnots, data.Poles, data.Weights);

        int prevSpan = -1;
        int spanSize = 0;
        double spanStart = 0.0;
        double spanLen = 1.0;
        bool useCache = false;
        double spanEnd = 0.0; // upper flat-knot boundary of current span

        for (int i = 0; i < nbParams; ++i)
        {
            double param = theParams.Value(low + i);
            double adjParam;
            int span = 0;

            // Precondition: theParams is sorted ascending (guaranteed by buildKnotAwareParams).
            // For non-periodic curves, skip locateSpan when the parameter is still within
            // the current span. This avoids BSplCLib::Hunt binary search for the common case.
            // locateSpan is called only on span transitions, preserving exact semantics.
            if (!data.IsPeriodic && prevSpan >= 0 && param < spanEnd)
            {
                adjParam = param;
                span = prevSpan;
            }
            else
            {
                LocateSpan(param, data.FlatKnots, data.Degree, data.IsPeriodic, out adjParam, ref span);
            }

            // Update span info when span changes
            if (span != prevSpan)
            {
                prevSpan = span;
                spanEnd = data.FlatKnots.Value(span + 1);
                spanSize = CountSpanSize(theParams, data.FlatKnots, data.Degree, data.IsPeriodic, i, span);
                ComputeSpanCoeffs(data.FlatKnots, span, out spanStart, out spanLen);

                useCache = (spanSize >= THE_CACHE_THRESHOLD);
                if (useCache)
                {
                    cache.BuildCache(adjParam, data.FlatKnots, data.Poles, data.Weights);
                }
            }

            ref TResult result = ref results.ChangeValue(i + 1);
            if (useCache)
            {
                // Curve cache uses [0,1] local parameter range
                double localParam = (spanLen > 0.0) ? (adjParam - spanStart) / spanLen : 0.0;
                TFunctor.CacheEval(cache, localParam, ref result);
            }
            else
            {
                TFunctor.DirectEval(data, adjParam, span, ref result);
            }
        }

        return results;
    }

    private struct EvalD0 : IEvalFunctor<Point3D>
    {
        public static void CacheEval(Cache cache, double localParam, ref Point3D result)
        {
            cache.D0Local(localParam, out result);
        }

        public static void DirectEval(in CurveData data, double param, int spanIdx, ref Point3D result)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(param,
                        spanIdx,
                        data.Degree,
                        data.IsPeriodic,
                        data.Poles,
                        data.Weights,
                        data.FlatKnots,
                        null,
                        out result);
        }
    }

    private struct EvalD1 : IEvalFunctor<Curve3D.ResD1>
    {
        public static void CacheEval(Cache cache, double localParam, ref Curve3D.ResD1 result)
        {
            cache.D1Local(localParam, out result.Point, out result.D1);
        }

        public static void DirectEval(in CurveData data, double param, int spanIdx,
                                      ref Curve3D.ResD1 result)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(param,
                        spanIdx,
                        data.Degree,
                        data.IsPeriodic,
                        data.Poles,
                        data.Weights,
                        data.FlatKnots,
                        null,
                        out result.Point,
                        out result.D1);
        }
    }

    private struct EvalD2 : IEvalFunctor<Curve3D.ResD2>
    {
        public static void CacheEval(Cache cache, double localParam, ref Curve3D.ResD2 result)
        {
            cache.D2Local(localParam, out result.Point, out result.D1, out result.D2);
        }

        public static void DirectEval(in CurveData data, double param, int spanIdx,
                                      ref Curve3D.ResD2 result)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(param,
                        spanIdx,
                        data.Degree,
                        data.IsPeriodic,
                        data.Poles,
                        data.Weights,
                        data.FlatKnots,
                        null,
                        out result.Point,
                        out result.D1,
                        out result.D2);
        }
    }

    private struct EvalD3 : IEvalFunctor<Curve3D.ResD3>
    {
        public static void CacheEval(Cache cache, double localParam, ref Curve3D.ResD3 result)
        {
            cache.D3Local(localParam, out result.Point, out result.D1, out result.D2, out result.D3);
        }

        public static void DirectEval(in CurveData data, double param, int spanIdx,
                                      ref Curve3D.ResD3 result)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(param,
                        spanIdx,
                        data.Degree,
                        data.IsPeriodic,
                        data.Poles,
                        data.Weights,
                        data.FlatKnots,
                        null,
                        out result.Point,
                        out result.D1,
                        out result.D2,
                        out result.D3);
        }
    }

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !ExtractCurveData(_geom, out CurveData data))
        {
            return new NCollection_Array1<Point3D>();
        }

        return EvaluateGridCached<Point3D, EvalD0>(data, theParams);
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve3D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !ExtractCurveData(_geom, out CurveData data))
        {
            return new NCollection_Array1<Curve3D.ResD1>();
        }

        return EvaluateGridCached<Curve3D.ResD1, EvalD1>(data, theParams);
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve3D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !ExtractCurveData(_geom, out CurveData data))
        {
            return new NCollection_Array1<Curve3D.ResD2>();
        }

        return EvaluateGridCached<Curve3D.ResD2, EvalD2>(data, theParams);
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Curve3D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty() || !ExtractCurveData(_geom, out CurveData data))
        {
            return new NCollection_Array1<Curve3D.ResD3>();
        }

        return EvaluateGridCached<Curve3D.ResD3, EvalD3>(data, theParams);
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points, directly with BSplCLib (zero above the degree).
    /// </summary>
    /// <remarks>
    /// As in OCCT, the values are wrong (or, where OCCT reads outside the knot array, an
    /// <see cref="IndexOutOfRangeException"/> is thrown) wherever the span's flat-knot index is also a valid knot
    /// index, and zero above the degree for rational curves too (occt-issues.md).
    /// </remarks>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (theParams.IsEmpty() || n < 1 || !ExtractCurveData(_geom, out CurveData data))
        {
            return new NCollection_Array1<Vector3D>();
        }

        int nbParams = theParams.Length();
        int low = theParams.Lower();

        // Derivatives beyond degree are zero
        // OCCT: not for a rational curve, whose derivatives above the degree are generally not zero
        if (n > data.Degree)
        {
            NCollection_Array1<Vector3D> zeroResult = new(1, nbParams);
            Vector3D zero = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= nbParams; ++i)
            {
                zeroResult.SetValue(i, zero);
            }
            return zeroResult;
        }

        NCollection_Array1<Vector3D> result = new(1, nbParams);

        // DN uses direct evaluation (no cache available)
        for (int i = 0; i < nbParams; ++i)
        {
            double param = theParams.Value(low + i);
            int span = 0;
            LocateSpan(param, data.FlatKnots, data.Degree, data.IsPeriodic, out double adjParam, ref span);

            // OCCT: aSpan indexes the flat knots, but BSplCLib::DN takes it as an index into the knots given with
            // their multiplicities. When it is a valid knot index, DN evaluates that span's polynomial at the
            // parameter (wrong values); building the span's knots may then read past the knots (garbage in OCCT,
            // IndexOutOfRangeException here). See occt-issues.md.
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(adjParam,
                        n,
                        span,
                        data.Degree,
                        data.IsPeriodic,
                        data.Poles,
                        data.Weights,
                        data.Knots,
                        data.Mults,
                        out Vector3D dn);
            result.SetValue(i + 1, dn);
        }

        return result;
    }
}
