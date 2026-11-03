// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BSplineSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for B-spline surface grid points: U and V parameters are located span by span, and
/// the BSplSLib_Cache of a (U span, V span) patch is built once when the patch holds at least two grid points
/// (fewer are evaluated directly by BSplSLib). D3 and DN are evaluated directly.
/// </summary>
/// <remarks>
/// As in OCCT, the parameters of a non-periodic direction must be ascending: a parameter below the current span's
/// end is evaluated on the current span without being located again.
/// </remarks>
internal sealed class BSplineSurfaceGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    // Threshold for using cache vs direct evaluation.
    // For small spans (few points), direct BSplSLib evaluation is faster than building cache.
    private const int THE_CACHE_THRESHOLD = 2;

    // Helper structure holding extracted B-spline surface data.
    private readonly struct SurfaceData
    {
        public readonly NCollection_Array1<double> UFlatKnots;
        public readonly NCollection_Array1<double> VFlatKnots;
        public readonly NCollection_Array2<Point3D> Poles;
        public readonly NCollection_Array2<double>? Weights;
        public readonly int UDegree;
        public readonly int VDegree;
        public readonly bool IsUPeriodic;
        public readonly bool IsVPeriodic;
        public readonly bool IsURational;
        public readonly bool IsVRational;

        public SurfaceData(BSplineSurface geom)
        {
            UFlatKnots = geom.InternalUKnotSequence();
            VFlatKnots = geom.InternalVKnotSequence();
            Poles = geom.InternalPoles();
            Weights = geom.InternalWeights();
            UDegree = geom.UDegree;
            VDegree = geom.VDegree;
            IsUPeriodic = geom.IsUPeriodic;
            IsVPeriodic = geom.IsVPeriodic;
            IsURational = geom.IsURational;
            IsVRational = geom.IsVRational;
        }
    }

    // The functors of evaluateGridCached: (cache, localU, localV, result) and
    // (data, adjU, adjV, uSpan, vSpan, result); evaluateGridDirect uses the second only.
    private interface IEvalFunctor<TResult>
    {
        static abstract void CacheEval(SurfaceCache cache, double localU, double localV, ref TResult result);

        static abstract void DirectEval(in SurfaceData data,
                                        double u,
                                        double v,
                                        int uSpan,
                                        int vSpan,
                                        int nu,
                                        int nv,
                                        ref TResult result);
    }

    private readonly BSplineSurface? _geom;

    /// <summary>Constructor with the B-spline surface to evaluate (null gives empty results).</summary>
    public BSplineSurfaceGrid(BSplineSurface? surface)
    {
        _geom = surface;
    }

    /// <summary>Returns the geometry.</summary>
    public BSplineSurface? Geometry() => _geom;

    // Extract surface data from geometry. Returns false if data is invalid.
    private static bool ExtractSurfaceData(BSplineSurface? geom, out SurfaceData data)
    {
        if (geom is null)
        {
            data = default;
            return false;
        }

        data = new SurfaceData(geom);
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
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, flatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), param, isPeriodic,
                                 ref spanIdx, out adjustedParam);
    }

    // Count consecutive parameters in the same span (for cache threshold decision).
    // For non-periodic cases, uses a fast knot-boundary comparison instead of
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
            // Fast path: params are sorted (ascending from grid construction),
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
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, flatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), param, isPeriodic, ref span,
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

    // Compute span local parameter coefficients (mid and half-length).
    private static void ComputeSpanCoeffs(NCollection_Array1<double> flatKnots,
                                          int span,
                                          out double mid,
                                          out double halfLen)
    {
        double start = flatKnots.Value(span);
        halfLen = 0.5 * (flatKnots.Value(span + 1) - start);
        mid = start + halfLen;
    }

    // Template helper for cached grid evaluation (D0, D1, D2).
    private static NCollection_Array2<TResult> EvaluateGridCached<TResult, TFunctor>(in SurfaceData data,
                                                                                     NCollection_Array1<double> uParams,
                                                                                     NCollection_Array1<double> vParams)
        where TFunctor : struct, IEvalFunctor<TResult>
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        int lowU = uParams.Lower();
        int lowV = vParams.Lower();

        NCollection_Array2<TResult> grid = new(1, nbU, 1, nbV);
        SurfaceCache cache = new(data.UDegree,
                                    data.IsUPeriodic,
                                    data.UFlatKnots,
                                    data.VDegree,
                                    data.IsVPeriodic,
                                    data.VFlatKnots,
                                    data.Weights);

        int prevUSpan = -1;
        int uSpanSize = 0;
        double uSpanMid = 0.0;
        double uSpanHalfLen = 1.0;
        double uSpanEnd = 0.0; // upper flat-knot boundary of current U span

        for (int ui = 0; ui < nbU; ++ui)
        {
            double u = uParams.Value(lowU + ui);
            double adjU = u;
            int uSpan = 0;

            // Skip locateSpan when parameter is still within current U span.
            if (!data.IsUPeriodic && prevUSpan >= 0 && u < uSpanEnd)
            {
                adjU = u;
                uSpan = prevUSpan;
            }
            else
            {
                LocateSpan(u, data.UFlatKnots, data.UDegree, data.IsUPeriodic, out adjU, ref uSpan);
            }

            if (uSpan != prevUSpan)
            {
                prevUSpan = uSpan;
                uSpanEnd = data.UFlatKnots.Value(uSpan + 1);
                uSpanSize = CountSpanSize(uParams, data.UFlatKnots, data.UDegree, data.IsUPeriodic, ui,
                                           uSpan);
                ComputeSpanCoeffs(data.UFlatKnots, uSpan, out uSpanMid, out uSpanHalfLen);
            }

            int prevVSpan = -1;
            int vSpanSize = 0;
            double vSpanMid = 0.0;
            double vSpanHalfLen = 1.0;
            bool useCache = false;
            double vSpanEnd = 0.0; // upper flat-knot boundary of current V span

            for (int vi = 0; vi < nbV; ++vi)
            {
                double v = vParams.Value(lowV + vi);
                double adjV = v;
                int vSpan = 0;

                // Skip locateSpan when parameter is still within current V span.
                if (!data.IsVPeriodic && prevVSpan >= 0 && v < vSpanEnd)
                {
                    adjV = v;
                    vSpan = prevVSpan;
                }
                else
                {
                    LocateSpan(v, data.VFlatKnots, data.VDegree, data.IsVPeriodic, out adjV, ref vSpan);
                }

                if (vSpan != prevVSpan)
                {
                    prevVSpan = vSpan;
                    vSpanEnd = data.VFlatKnots.Value(vSpan + 1);
                    vSpanSize = CountSpanSize(vParams, data.VFlatKnots, data.VDegree, data.IsVPeriodic,
                                               vi, vSpan);
                    ComputeSpanCoeffs(data.VFlatKnots, vSpan, out vSpanMid, out vSpanHalfLen);

                    useCache = (uSpanSize * vSpanSize >= THE_CACHE_THRESHOLD);
                    if (useCache)
                    {
                        cache.BuildCache(adjU, adjV, data.UFlatKnots, data.VFlatKnots, data.Poles,
                                          data.Weights);
                    }
                }

                ref TResult result = ref grid.ChangeValue(ui + 1, vi + 1);
                if (useCache)
                {
                    double localU = (adjU - uSpanMid) / uSpanHalfLen;
                    double localV = (adjV - vSpanMid) / vSpanHalfLen;
                    TFunctor.CacheEval(cache, localU, localV, ref result);
                }
                else
                {
                    TFunctor.DirectEval(data, adjU, adjV, uSpan, vSpan, 0, 0, ref result);
                }
            }
        }

        return grid;
    }

    // Template helper for direct-only grid evaluation (D3, DN).
    private static NCollection_Array2<TResult> EvaluateGridDirect<TResult, TFunctor>(in SurfaceData data,
                                                                                     NCollection_Array1<double> uParams,
                                                                                     NCollection_Array1<double> vParams,
                                                                                     int nu,
                                                                                     int nv)
        where TFunctor : struct, IEvalFunctor<TResult>
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        int lowU = uParams.Lower();
        int lowV = vParams.Lower();

        NCollection_Array2<TResult> grid = new(1, nbU, 1, nbV);

        int prevUSpan = -1;
        double uSpanEnd = 0.0;

        for (int ui = 0; ui < nbU; ++ui)
        {
            double u = uParams.Value(lowU + ui);
            double adjU = u;
            int uSpan = 0;

            if (!data.IsUPeriodic && prevUSpan >= 0 && u < uSpanEnd)
            {
                adjU = u;
                uSpan = prevUSpan;
            }
            else
            {
                LocateSpan(u, data.UFlatKnots, data.UDegree, data.IsUPeriodic, out adjU, ref uSpan);
            }
            if (uSpan != prevUSpan)
            {
                prevUSpan = uSpan;
                uSpanEnd = data.UFlatKnots.Value(uSpan + 1);
            }

            int prevVSpan = -1;
            double vSpanEnd = 0.0;

            for (int vi = 0; vi < nbV; ++vi)
            {
                double v = vParams.Value(lowV + vi);
                double adjV = v;
                int vSpan = 0;

                if (!data.IsVPeriodic && prevVSpan >= 0 && v < vSpanEnd)
                {
                    adjV = v;
                    vSpan = prevVSpan;
                }
                else
                {
                    LocateSpan(v, data.VFlatKnots, data.VDegree, data.IsVPeriodic, out adjV, ref vSpan);
                }
                if (vSpan != prevVSpan)
                {
                    prevVSpan = vSpan;
                    vSpanEnd = data.VFlatKnots.Value(vSpan + 1);
                }

                TFunctor.DirectEval(data, adjU, adjV, uSpan, vSpan, nu, nv,
                                    ref grid.ChangeValue(ui + 1, vi + 1));
            }
        }

        return grid;
    }

    // D0: cache evaluation and direct evaluation.
    private struct EvalD0 : IEvalFunctor<Point3D>
    {
        public static void CacheEval(SurfaceCache cache, double localU, double localV, ref Point3D result)
        {
            cache.D0Local(localU, localV, out result);
        }

        public static void DirectEval(in SurfaceData data,
                                      double u,
                                      double v,
                                      int uSpan,
                                      int vSpan,
                                      int nu,
                                      int nv,
                                      ref Point3D result)
        {
            SurfaceBSplSLib.D0(u,
                        v,
                        uSpan,
                        vSpan,
                        data.Poles,
                        data.Weights,
                        data.UFlatKnots,
                        data.VFlatKnots,
                        null,
                        null,
                        data.UDegree,
                        data.VDegree,
                        data.IsURational,
                        data.IsVRational,
                        data.IsUPeriodic,
                        data.IsVPeriodic,
                        out result);
        }
    }

    // D1: cache evaluation and direct evaluation.
    private struct EvalD1 : IEvalFunctor<Surface.ResD1>
    {
        public static void CacheEval(SurfaceCache cache,
                                     double localU,
                                     double localV,
                                     ref Surface.ResD1 result)
        {
            cache.D1Local(localU, localV, out result.Point, out result.D1U, out result.D1V);
        }

        public static void DirectEval(in SurfaceData data,
                                      double u,
                                      double v,
                                      int uSpan,
                                      int vSpan,
                                      int nu,
                                      int nv,
                                      ref Surface.ResD1 result)
        {
            SurfaceBSplSLib.D1(u,
                        v,
                        uSpan,
                        vSpan,
                        data.Poles,
                        data.Weights,
                        data.UFlatKnots,
                        data.VFlatKnots,
                        null,
                        null,
                        data.UDegree,
                        data.VDegree,
                        data.IsURational,
                        data.IsVRational,
                        data.IsUPeriodic,
                        data.IsVPeriodic,
                        out result.Point,
                        out result.D1U,
                        out result.D1V);
        }
    }

    // D2: cache evaluation and direct evaluation.
    private struct EvalD2 : IEvalFunctor<Surface.ResD2>
    {
        public static void CacheEval(SurfaceCache cache,
                                     double localU,
                                     double localV,
                                     ref Surface.ResD2 result)
        {
            cache.D2Local(localU,
                             localV,
                             out result.Point,
                             out result.D1U,
                             out result.D1V,
                             out result.D2U,
                             out result.D2V,
                             out result.D2UV);
        }

        public static void DirectEval(in SurfaceData data,
                                      double u,
                                      double v,
                                      int uSpan,
                                      int vSpan,
                                      int nu,
                                      int nv,
                                      ref Surface.ResD2 result)
        {
            SurfaceBSplSLib.D2(u,
                        v,
                        uSpan,
                        vSpan,
                        data.Poles,
                        data.Weights,
                        data.UFlatKnots,
                        data.VFlatKnots,
                        null,
                        null,
                        data.UDegree,
                        data.VDegree,
                        data.IsURational,
                        data.IsVRational,
                        data.IsUPeriodic,
                        data.IsVPeriodic,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV);
        }
    }

    // D3: direct evaluation only.
    private struct EvalD3 : IEvalFunctor<Surface.ResD3>
    {
        public static void CacheEval(SurfaceCache cache,
                                     double localU,
                                     double localV,
                                     ref Surface.ResD3 result) =>
            throw new ProgramErrorException("GeomGridEval_BSplineSurface: D3 has no cache evaluation");

        public static void DirectEval(in SurfaceData data,
                                      double u,
                                      double v,
                                      int uSpan,
                                      int vSpan,
                                      int nu,
                                      int nv,
                                      ref Surface.ResD3 result)
        {
            SurfaceBSplSLib.D3(u,
                        v,
                        uSpan,
                        vSpan,
                        data.Poles,
                        data.Weights,
                        data.UFlatKnots,
                        data.VFlatKnots,
                        null,
                        null,
                        data.UDegree,
                        data.VDegree,
                        data.IsURational,
                        data.IsVRational,
                        data.IsUPeriodic,
                        data.IsVPeriodic,
                        out result.Point,
                        out Vector3D d1U,
                        out Vector3D d1V,
                        out Vector3D d2U,
                        out Vector3D d2V,
                        out Vector3D d2UV,
                        out result.D3U,
                        out result.D3V,
                        out result.D3UUV,
                        out result.D3UVV);
            result.D1U = d1U;
            result.D1V = d1V;
            result.D2U = d2U;
            result.D2V = d2V;
            result.D2UV = d2UV;
        }
    }

    // DN: direct evaluation only (OCCT's lambda captures theNU and theNV; the port passes them).
    private struct EvalDN : IEvalFunctor<Vector3D>
    {
        public static void CacheEval(SurfaceCache cache, double localU, double localV, ref Vector3D result) =>
            throw new ProgramErrorException("GeomGridEval_BSplineSurface: DN has no cache evaluation");

        public static void DirectEval(in SurfaceData data,
                                      double u,
                                      double v,
                                      int uSpan,
                                      int vSpan,
                                      int nu,
                                      int nv,
                                      ref Vector3D result)
        {
            SurfaceBSplSLib.DN(u,
                        v,
                        nu,
                        nv,
                        uSpan,
                        vSpan,
                        data.Poles,
                        data.Weights,
                        data.UFlatKnots,
                        data.VFlatKnots,
                        null,
                        null,
                        data.UDegree,
                        data.VDegree,
                        data.IsURational,
                        data.IsVRational,
                        data.IsUPeriodic,
                        data.IsVPeriodic,
                        out result);
        }
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        if (uParams.IsEmpty() || vParams.IsEmpty() || !ExtractSurfaceData(_geom, out SurfaceData data))
        {
            return new NCollection_Array2<Point3D>();
        }

        return EvaluateGridCached<Point3D, EvalD0>(data, uParams, vParams);
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (uParams.IsEmpty() || vParams.IsEmpty() || !ExtractSurfaceData(_geom, out SurfaceData data))
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        return EvaluateGridCached<Surface.ResD1, EvalD1>(data, uParams, vParams);
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (uParams.IsEmpty() || vParams.IsEmpty() || !ExtractSurfaceData(_geom, out SurfaceData data))
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        return EvaluateGridCached<Surface.ResD2, EvalD2>(data, uParams, vParams);
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order (direct evaluation).</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (uParams.IsEmpty() || vParams.IsEmpty() || !ExtractSurfaceData(_geom, out SurfaceData data))
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        return EvaluateGridDirect<Surface.ResD3, EvalD3>(data, uParams, vParams, 0, 0);
    }

    /// <summary>
    /// Evaluates the partial derivative of orders (NU, NV) on the grid (direct evaluation); zero above a
    /// degree.
    /// </summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        if (uParams.IsEmpty() || vParams.IsEmpty() || nu < 0 || nv < 0 || (nu + nv) < 1
            || !ExtractSurfaceData(_geom, out SurfaceData data))
        {
            return new NCollection_Array2<Vector3D>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        // Derivatives beyond degree are zero
        // OCCT: not for a rational surface, whose derivatives above the degree are in general not zero
        // (BSplSLib::DN and Geom_BSplineSurface::DN return them); the grid returns zero (occt-issues.md).
        if (nu > data.UDegree || nv > data.VDegree)
        {
            NCollection_Array2<Vector3D> zeroGrid = new(1, nbU, 1, nbV);
            Vector3D zero = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= nbU; ++i)
            {
                for (int j = 1; j <= nbV; ++j)
                {
                    zeroGrid.SetValue(i, j, zero);
                }
            }
            return zeroGrid;
        }

        return EvaluateGridDirect<Vector3D, EvalDN>(data, uParams, vParams, nu, nv);
    }
}
