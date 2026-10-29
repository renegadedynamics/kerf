// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BSplineSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for B-spline surface grid points: U and V parameters are located span by span, and
/// the BSplSLib_Cache of a (U span, V span) patch is built once when the patch holds at least two grid points
/// (fewer are evaluated directly by BSplSLib). D3 and DN are evaluated directly.
/// </summary>
/// <remarks>
/// As in OCCT, the parameters of a non-periodic direction must be ascending: a parameter below the current span's
/// end is evaluated on the current span without being located again.
/// </remarks>
public sealed class GeomGridEval_BSplineSurface : GeomGridEval.ISurfaceEvaluator
{
    // Threshold for using cache vs direct evaluation.
    // For small spans (few points), direct BSplSLib evaluation is faster than building cache.
    private const int THE_CACHE_THRESHOLD = 2;

    // Helper structure holding extracted B-spline surface data.
    private readonly struct SurfaceData
    {
        public readonly NCollection_Array1<double> UFlatKnots;
        public readonly NCollection_Array1<double> VFlatKnots;
        public readonly NCollection_Array2<gp_Pnt> Poles;
        public readonly NCollection_Array2<double>? Weights;
        public readonly int UDegree;
        public readonly int VDegree;
        public readonly bool IsUPeriodic;
        public readonly bool IsVPeriodic;
        public readonly bool IsURational;
        public readonly bool IsVRational;

        public SurfaceData(Geom_BSplineSurface theGeom)
        {
            UFlatKnots = theGeom.InternalUKnotSequence();
            VFlatKnots = theGeom.InternalVKnotSequence();
            Poles = theGeom.InternalPoles();
            Weights = theGeom.InternalWeights();
            UDegree = theGeom.UDegree();
            VDegree = theGeom.VDegree();
            IsUPeriodic = theGeom.IsUPeriodic();
            IsVPeriodic = theGeom.IsVPeriodic();
            IsURational = theGeom.IsURational();
            IsVRational = theGeom.IsVRational();
        }
    }

    // The functors of evaluateGridCached: (cache, localU, localV, result) and
    // (data, adjU, adjV, uSpan, vSpan, result); evaluateGridDirect uses the second only.
    private interface IEvalFunctor<TResult>
    {
        static abstract void CacheEval(BSplSLib_Cache theCache, double theLocalU, double theLocalV, ref TResult theResult);

        static abstract void DirectEval(in SurfaceData theData,
                                        double theU,
                                        double theV,
                                        int theUSpan,
                                        int theVSpan,
                                        int theNU,
                                        int theNV,
                                        ref TResult theResult);
    }

    private readonly Geom_BSplineSurface? myGeom;

    /// <summary>Constructor with the B-spline surface to evaluate (null gives empty results).</summary>
    public GeomGridEval_BSplineSurface(Geom_BSplineSurface? theSurface)
    {
        myGeom = theSurface;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_BSplineSurface? Geometry() => myGeom;

    // Extract surface data from geometry. Returns false if data is invalid.
    private static bool extractSurfaceData(Geom_BSplineSurface? theGeom, out SurfaceData theData)
    {
        if (theGeom is null)
        {
            theData = default;
            return false;
        }

        theData = new SurfaceData(theGeom);
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
        BSplCLib.LocateParameter(theDegree, theFlatKnots, BSplCLib.NoMults(), theParam, theIsPeriodic,
                                 ref theSpanIdx, out theAdjustedParam);
    }

    // Count consecutive parameters in the same span (for cache threshold decision).
    // For non-periodic cases, uses a fast knot-boundary comparison instead of
    // calling BSplCLib::LocateParameter for every subsequent parameter.
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

        if (!theIsPeriodic)
        {
            // Fast path: params are sorted (ascending from grid construction),
            // so just compare against the next knot boundary.
            if (theTargetSpan + 1 > theFlatKnots.Upper())
            {
                return aCount;
            }
            double aNextKnot = theFlatKnots.Value(theTargetSpan + 1);
            for (int i = theStartIdx + 1; i < aNb; ++i)
            {
                if (theParams.Value(aLower + i) >= aNextKnot)
                {
                    break;
                }
                ++aCount;
            }
            return aCount;
        }

        // Periodic fallback: span assignment requires full LocateParameter.
        for (int i = theStartIdx + 1; i < aNb; ++i)
        {
            double aParam = theParams.Value(aLower + i);
            int aSpan = 0;
            BSplCLib.LocateParameter(theDegree, theFlatKnots, BSplCLib.NoMults(), aParam, theIsPeriodic, ref aSpan,
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

    // Compute span local parameter coefficients (mid and half-length).
    private static void computeSpanCoeffs(NCollection_Array1<double> theFlatKnots,
                                          int theSpan,
                                          out double theMid,
                                          out double theHalfLen)
    {
        double aStart = theFlatKnots.Value(theSpan);
        theHalfLen = 0.5 * (theFlatKnots.Value(theSpan + 1) - aStart);
        theMid = aStart + theHalfLen;
    }

    // Template helper for cached grid evaluation (D0, D1, D2).
    private static NCollection_Array2<TResult> evaluateGridCached<TResult, TFunctor>(in SurfaceData theData,
                                                                                     NCollection_Array1<double> theUParams,
                                                                                     NCollection_Array1<double> theVParams)
        where TFunctor : struct, IEvalFunctor<TResult>
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        int aLowU = theUParams.Lower();
        int aLowV = theVParams.Lower();

        NCollection_Array2<TResult> aGrid = new(1, aNbU, 1, aNbV);
        BSplSLib_Cache aCache = new(theData.UDegree,
                                    theData.IsUPeriodic,
                                    theData.UFlatKnots,
                                    theData.VDegree,
                                    theData.IsVPeriodic,
                                    theData.VFlatKnots,
                                    theData.Weights);

        int aPrevUSpan = -1;
        int aUSpanSize = 0;
        double aUSpanMid = 0.0;
        double aUSpanHalfLen = 1.0;
        double aUSpanEnd = 0.0; // upper flat-knot boundary of current U span

        for (int ui = 0; ui < aNbU; ++ui)
        {
            double aU = theUParams.Value(aLowU + ui);
            double aAdjU = aU;
            int aUSpan = 0;

            // Skip locateSpan when parameter is still within current U span.
            if (!theData.IsUPeriodic && aPrevUSpan >= 0 && aU < aUSpanEnd)
            {
                aAdjU = aU;
                aUSpan = aPrevUSpan;
            }
            else
            {
                locateSpan(aU, theData.UFlatKnots, theData.UDegree, theData.IsUPeriodic, out aAdjU, ref aUSpan);
            }

            if (aUSpan != aPrevUSpan)
            {
                aPrevUSpan = aUSpan;
                aUSpanEnd = theData.UFlatKnots.Value(aUSpan + 1);
                aUSpanSize = countSpanSize(theUParams, theData.UFlatKnots, theData.UDegree, theData.IsUPeriodic, ui,
                                           aUSpan);
                computeSpanCoeffs(theData.UFlatKnots, aUSpan, out aUSpanMid, out aUSpanHalfLen);
            }

            int aPrevVSpan = -1;
            int aVSpanSize = 0;
            double aVSpanMid = 0.0;
            double aVSpanHalfLen = 1.0;
            bool aUseCache = false;
            double aVSpanEnd = 0.0; // upper flat-knot boundary of current V span

            for (int vi = 0; vi < aNbV; ++vi)
            {
                double aV = theVParams.Value(aLowV + vi);
                double aAdjV = aV;
                int aVSpan = 0;

                // Skip locateSpan when parameter is still within current V span.
                if (!theData.IsVPeriodic && aPrevVSpan >= 0 && aV < aVSpanEnd)
                {
                    aAdjV = aV;
                    aVSpan = aPrevVSpan;
                }
                else
                {
                    locateSpan(aV, theData.VFlatKnots, theData.VDegree, theData.IsVPeriodic, out aAdjV, ref aVSpan);
                }

                if (aVSpan != aPrevVSpan)
                {
                    aPrevVSpan = aVSpan;
                    aVSpanEnd = theData.VFlatKnots.Value(aVSpan + 1);
                    aVSpanSize = countSpanSize(theVParams, theData.VFlatKnots, theData.VDegree, theData.IsVPeriodic,
                                               vi, aVSpan);
                    computeSpanCoeffs(theData.VFlatKnots, aVSpan, out aVSpanMid, out aVSpanHalfLen);

                    aUseCache = (aUSpanSize * aVSpanSize >= THE_CACHE_THRESHOLD);
                    if (aUseCache)
                    {
                        aCache.BuildCache(aAdjU, aAdjV, theData.UFlatKnots, theData.VFlatKnots, theData.Poles,
                                          theData.Weights);
                    }
                }

                ref TResult aResult = ref aGrid.ChangeValue(ui + 1, vi + 1);
                if (aUseCache)
                {
                    double aLocalU = (aAdjU - aUSpanMid) / aUSpanHalfLen;
                    double aLocalV = (aAdjV - aVSpanMid) / aVSpanHalfLen;
                    TFunctor.CacheEval(aCache, aLocalU, aLocalV, ref aResult);
                }
                else
                {
                    TFunctor.DirectEval(theData, aAdjU, aAdjV, aUSpan, aVSpan, 0, 0, ref aResult);
                }
            }
        }

        return aGrid;
    }

    // Template helper for direct-only grid evaluation (D3, DN).
    private static NCollection_Array2<TResult> evaluateGridDirect<TResult, TFunctor>(in SurfaceData theData,
                                                                                     NCollection_Array1<double> theUParams,
                                                                                     NCollection_Array1<double> theVParams,
                                                                                     int theNU,
                                                                                     int theNV)
        where TFunctor : struct, IEvalFunctor<TResult>
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        int aLowU = theUParams.Lower();
        int aLowV = theVParams.Lower();

        NCollection_Array2<TResult> aGrid = new(1, aNbU, 1, aNbV);

        int aPrevUSpan = -1;
        double aUSpanEnd = 0.0;

        for (int ui = 0; ui < aNbU; ++ui)
        {
            double aU = theUParams.Value(aLowU + ui);
            double aAdjU = aU;
            int aUSpan = 0;

            if (!theData.IsUPeriodic && aPrevUSpan >= 0 && aU < aUSpanEnd)
            {
                aAdjU = aU;
                aUSpan = aPrevUSpan;
            }
            else
            {
                locateSpan(aU, theData.UFlatKnots, theData.UDegree, theData.IsUPeriodic, out aAdjU, ref aUSpan);
            }
            if (aUSpan != aPrevUSpan)
            {
                aPrevUSpan = aUSpan;
                aUSpanEnd = theData.UFlatKnots.Value(aUSpan + 1);
            }

            int aPrevVSpan = -1;
            double aVSpanEnd = 0.0;

            for (int vi = 0; vi < aNbV; ++vi)
            {
                double aV = theVParams.Value(aLowV + vi);
                double aAdjV = aV;
                int aVSpan = 0;

                if (!theData.IsVPeriodic && aPrevVSpan >= 0 && aV < aVSpanEnd)
                {
                    aAdjV = aV;
                    aVSpan = aPrevVSpan;
                }
                else
                {
                    locateSpan(aV, theData.VFlatKnots, theData.VDegree, theData.IsVPeriodic, out aAdjV, ref aVSpan);
                }
                if (aVSpan != aPrevVSpan)
                {
                    aPrevVSpan = aVSpan;
                    aVSpanEnd = theData.VFlatKnots.Value(aVSpan + 1);
                }

                TFunctor.DirectEval(theData, aAdjU, aAdjV, aUSpan, aVSpan, theNU, theNV,
                                    ref aGrid.ChangeValue(ui + 1, vi + 1));
            }
        }

        return aGrid;
    }

    // D0: cache evaluation and direct evaluation.
    private struct EvalD0 : IEvalFunctor<gp_Pnt>
    {
        public static void CacheEval(BSplSLib_Cache theCache, double theLocalU, double theLocalV, ref gp_Pnt theResult)
        {
            theCache.D0Local(theLocalU, theLocalV, out theResult);
        }

        public static void DirectEval(in SurfaceData theData,
                                      double theU,
                                      double theV,
                                      int theUSpan,
                                      int theVSpan,
                                      int theNU,
                                      int theNV,
                                      ref gp_Pnt theResult)
        {
            BSplSLib.D0(theU,
                        theV,
                        theUSpan,
                        theVSpan,
                        theData.Poles,
                        theData.Weights,
                        theData.UFlatKnots,
                        theData.VFlatKnots,
                        null,
                        null,
                        theData.UDegree,
                        theData.VDegree,
                        theData.IsURational,
                        theData.IsVRational,
                        theData.IsUPeriodic,
                        theData.IsVPeriodic,
                        out theResult);
        }
    }

    // D1: cache evaluation and direct evaluation.
    private struct EvalD1 : IEvalFunctor<Geom_Surface.ResD1>
    {
        public static void CacheEval(BSplSLib_Cache theCache,
                                     double theLocalU,
                                     double theLocalV,
                                     ref Geom_Surface.ResD1 theResult)
        {
            theCache.D1Local(theLocalU, theLocalV, out theResult.Point, out theResult.D1U, out theResult.D1V);
        }

        public static void DirectEval(in SurfaceData theData,
                                      double theU,
                                      double theV,
                                      int theUSpan,
                                      int theVSpan,
                                      int theNU,
                                      int theNV,
                                      ref Geom_Surface.ResD1 theResult)
        {
            BSplSLib.D1(theU,
                        theV,
                        theUSpan,
                        theVSpan,
                        theData.Poles,
                        theData.Weights,
                        theData.UFlatKnots,
                        theData.VFlatKnots,
                        null,
                        null,
                        theData.UDegree,
                        theData.VDegree,
                        theData.IsURational,
                        theData.IsVRational,
                        theData.IsUPeriodic,
                        theData.IsVPeriodic,
                        out theResult.Point,
                        out theResult.D1U,
                        out theResult.D1V);
        }
    }

    // D2: cache evaluation and direct evaluation.
    private struct EvalD2 : IEvalFunctor<Geom_Surface.ResD2>
    {
        public static void CacheEval(BSplSLib_Cache theCache,
                                     double theLocalU,
                                     double theLocalV,
                                     ref Geom_Surface.ResD2 theResult)
        {
            theCache.D2Local(theLocalU,
                             theLocalV,
                             out theResult.Point,
                             out theResult.D1U,
                             out theResult.D1V,
                             out theResult.D2U,
                             out theResult.D2V,
                             out theResult.D2UV);
        }

        public static void DirectEval(in SurfaceData theData,
                                      double theU,
                                      double theV,
                                      int theUSpan,
                                      int theVSpan,
                                      int theNU,
                                      int theNV,
                                      ref Geom_Surface.ResD2 theResult)
        {
            BSplSLib.D2(theU,
                        theV,
                        theUSpan,
                        theVSpan,
                        theData.Poles,
                        theData.Weights,
                        theData.UFlatKnots,
                        theData.VFlatKnots,
                        null,
                        null,
                        theData.UDegree,
                        theData.VDegree,
                        theData.IsURational,
                        theData.IsVRational,
                        theData.IsUPeriodic,
                        theData.IsVPeriodic,
                        out theResult.Point,
                        out theResult.D1U,
                        out theResult.D1V,
                        out theResult.D2U,
                        out theResult.D2V,
                        out theResult.D2UV);
        }
    }

    // D3: direct evaluation only.
    private struct EvalD3 : IEvalFunctor<Geom_Surface.ResD3>
    {
        public static void CacheEval(BSplSLib_Cache theCache,
                                     double theLocalU,
                                     double theLocalV,
                                     ref Geom_Surface.ResD3 theResult) =>
            throw new Standard_ProgramError("GeomGridEval_BSplineSurface: D3 has no cache evaluation");

        public static void DirectEval(in SurfaceData theData,
                                      double theU,
                                      double theV,
                                      int theUSpan,
                                      int theVSpan,
                                      int theNU,
                                      int theNV,
                                      ref Geom_Surface.ResD3 theResult)
        {
            BSplSLib.D3(theU,
                        theV,
                        theUSpan,
                        theVSpan,
                        theData.Poles,
                        theData.Weights,
                        theData.UFlatKnots,
                        theData.VFlatKnots,
                        null,
                        null,
                        theData.UDegree,
                        theData.VDegree,
                        theData.IsURational,
                        theData.IsVRational,
                        theData.IsUPeriodic,
                        theData.IsVPeriodic,
                        out theResult.Point,
                        out gp_Vec aD1U,
                        out gp_Vec aD1V,
                        out gp_Vec aD2U,
                        out gp_Vec aD2V,
                        out gp_Vec aD2UV,
                        out theResult.D3U,
                        out theResult.D3V,
                        out theResult.D3UUV,
                        out theResult.D3UVV);
            theResult.D1U = aD1U;
            theResult.D1V = aD1V;
            theResult.D2U = aD2U;
            theResult.D2V = aD2V;
            theResult.D2UV = aD2UV;
        }
    }

    // DN: direct evaluation only (OCCT's lambda captures theNU and theNV; the port passes them).
    private struct EvalDN : IEvalFunctor<gp_Vec>
    {
        public static void CacheEval(BSplSLib_Cache theCache, double theLocalU, double theLocalV, ref gp_Vec theResult) =>
            throw new Standard_ProgramError("GeomGridEval_BSplineSurface: DN has no cache evaluation");

        public static void DirectEval(in SurfaceData theData,
                                      double theU,
                                      double theV,
                                      int theUSpan,
                                      int theVSpan,
                                      int theNU,
                                      int theNV,
                                      ref gp_Vec theResult)
        {
            BSplSLib.DN(theU,
                        theV,
                        theNU,
                        theNV,
                        theUSpan,
                        theVSpan,
                        theData.Poles,
                        theData.Weights,
                        theData.UFlatKnots,
                        theData.VFlatKnots,
                        null,
                        null,
                        theData.UDegree,
                        theData.VDegree,
                        theData.IsURational,
                        theData.IsVRational,
                        theData.IsUPeriodic,
                        theData.IsVPeriodic,
                        out theResult);
        }
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        if (theUParams.IsEmpty() || theVParams.IsEmpty() || !extractSurfaceData(myGeom, out SurfaceData aData))
        {
            return new NCollection_Array2<gp_Pnt>();
        }

        return evaluateGridCached<gp_Pnt, EvalD0>(aData, theUParams, theVParams);
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (theUParams.IsEmpty() || theVParams.IsEmpty() || !extractSurfaceData(myGeom, out SurfaceData aData))
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        return evaluateGridCached<Geom_Surface.ResD1, EvalD1>(aData, theUParams, theVParams);
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (theUParams.IsEmpty() || theVParams.IsEmpty() || !extractSurfaceData(myGeom, out SurfaceData aData))
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        return evaluateGridCached<Geom_Surface.ResD2, EvalD2>(aData, theUParams, theVParams);
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order (direct evaluation).</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (theUParams.IsEmpty() || theVParams.IsEmpty() || !extractSurfaceData(myGeom, out SurfaceData aData))
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        return evaluateGridDirect<Geom_Surface.ResD3, EvalD3>(aData, theUParams, theVParams, 0, 0);
    }

    /// <summary>
    /// Evaluates the partial derivative of orders (NU, NV) on the grid (direct evaluation); zero above a
    /// degree.
    /// </summary>
    public NCollection_Array2<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        if (theUParams.IsEmpty() || theVParams.IsEmpty() || theNU < 0 || theNV < 0 || (theNU + theNV) < 1
            || !extractSurfaceData(myGeom, out SurfaceData aData))
        {
            return new NCollection_Array2<gp_Vec>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        // Derivatives beyond degree are zero
        // OCCT: not for a rational surface, whose derivatives above the degree are in general not zero
        // (BSplSLib::DN and Geom_BSplineSurface::DN return them); the grid returns zero (occt-issues.md).
        if (theNU > aData.UDegree || theNV > aData.VDegree)
        {
            NCollection_Array2<gp_Vec> aZeroGrid = new(1, aNbU, 1, aNbV);
            gp_Vec aZero = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= aNbU; ++i)
            {
                for (int j = 1; j <= aNbV; ++j)
                {
                    aZeroGrid.SetValue(i, j, aZero);
                }
            }
            return aZeroGrid;
        }

        return evaluateGridDirect<gp_Vec, EvalDN>(aData, theUParams, theVParams, theNU, theNV);
    }
}
