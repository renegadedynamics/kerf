// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_GridEvaluator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Grid-based point-curve extrema computation with cached state for repeated queries. Used by the B-spline,
/// Bezier, offset and other curve evaluators.
/// </summary>
/// <remarks>
/// Algorithm: build a grid of (parameter, point, D1) through GeomGridEval; scan it for candidate intervals
/// (sign changes of F(u), near-zero F, 3-point distance extrema); refine each candidate by Newton's method. The
/// temporary collections are fields reused through <c>Clear()</c>, as OCCT's mutable members.
/// </remarks>
public sealed class ExtremaPC_GridEvaluator
{
    /// <summary>Cached grid point with pre-computed data.</summary>
    public struct GridPoint
    {
        /// <summary>Parameter value.</summary>
        public double Param;
        /// <summary>Curve point C(u).</summary>
        public gp_Pnt Point;
        /// <summary>First derivative C'(u).</summary>
        public gp_Vec D1;
    }

    /// <summary>Type of candidate extremum detected during grid scan.</summary>
    public enum CandidateType
    {
        /// <summary>F(u) changes sign between grid points.</summary>
        SignChange,
        /// <summary>F(u) is very small at grid point.</summary>
        NearZero
    }

    /// <summary>Candidate interval for Newton refinement.</summary>
    public struct Candidate
    {
        /// <summary>Type of candidate.</summary>
        public CandidateType Type;
        /// <summary>Lower grid index.</summary>
        public int IdxLo;
        /// <summary>Upper grid index (same as IdxLo for NearZero).</summary>
        public int IdxHi;
        /// <summary>Starting point for Newton.</summary>
        public double StartU;
    }

    private NCollection_Array1<GridPoint> myGrid = new(); // Cached grid

    // Cached temporaries (reused via Clear())
    private readonly ExtremaPC.Result myResult = new();                            // Reusable result
    private readonly NCollection_DynamicArray<Candidate> myCandidates = new();     // Candidates from grid scan
    private readonly NCollection_DynamicArray<double> myFoundRoots = new();        // Found roots for dedup
    private readonly NCollection_DynamicArray<(int first, double second)> mySortedIndices = new(); // Sorted candidate indices
    private NCollection_Array1<bool> myProcessed = new();                          // Processed flags for grid scan

    public ExtremaPC_GridEvaluator()
    {
    }

    /// <summary>
    /// Builds the grid from GeomGridEval D1 results at the given parameters (any bounds; ascending for B-spline
    /// evaluators, as GeomGridEval requires).
    /// </summary>
    public void BuildGrid<GridEval>(GridEval theEval, math_Vector theParams)
        where GridEval : GeomGridEval.ICurveEvaluator
    {
        // Use Array1() accessor to pass to GeomGridEval which expects NCollection_Array1
        NCollection_Array1<Geom_Curve.ResD1> aD1Grid = theEval.EvaluateGridD1(theParams.Array1());

        int aNbParams = theParams.Length();

        // Resize grid if needed
        if (myGrid.Length() != aNbParams)
        {
            myGrid = new NCollection_Array1<GridPoint>(0, aNbParams - 1);
        }

        for (int i = 0; i < aNbParams; ++i)
        {
            int aD1Idx = aD1Grid.Lower() + i;

            myGrid[i].Param = theParams[theParams.Lower() + i];
            myGrid[i].Point = aD1Grid.Value(aD1Idx).Point;
            myGrid[i].D1 = aD1Grid.Value(aD1Idx).D1;
        }
    }

    /// <summary>Returns the cached grid (shared: don't modify).</summary>
    public NCollection_Array1<GridPoint> Grid() => myGrid;

    /// <summary>Returns the reusable result, for post-processing.</summary>
    public ExtremaPC.Result Result() => myResult;

    /// <summary>Extrema computation on the cached grid (interior only).</summary>
    public ExtremaPC.Result Perform(Adaptor3d_Curve theCurve,
                                    in gp_Pnt theP,
                                    in MathUtils.Domain1D theDomain,
                                    double theTol,
                                    ExtremaPC.SearchMode theMode)
    {
        myResult.Clear();
        scanGrid(theP, theTol, theMode);
        refineCandidates(theCurve, theP, theDomain, theTol, theMode);

        if (!myResult.Extrema.IsEmpty())
        {
            myResult.Status = ExtremaPC.Status.OK;
        }
        return myResult;
    }

    /// <summary>Builds a uniform parameter grid (1-based).</summary>
    public static math_Vector BuildUniformParams(double theUMin, double theUMax, int theNbSamples)
    {
        math_Vector aParams = new(1, theNbSamples);
        double aStep = (theUMax - theUMin) / (theNbSamples - 1);

        for (int i = 1; i <= theNbSamples; ++i)
        {
            aParams[i] = theUMin + (i - 1) * aStep;
        }
        aParams[theNbSamples] = theUMax; // Ensure exact endpoint

        return aParams;
    }

    // Scans the grid to find candidate intervals for extrema.
    private void scanGrid(in gp_Pnt theP, double theTol, ExtremaPC.SearchMode theMode)
    {
        myCandidates.Clear();
        int aNbGrid = myGrid.Length();

        if (aNbGrid < 2)
        {
            return;
        }

        // Resize processed array if needed
        if (myProcessed.Length() != aNbGrid)
        {
            myProcessed = new NCollection_Array1<bool>(0, aNbGrid - 1);
        }
        myProcessed.Init(false);

        double aPrevF = 0.0;
        double aPrevDist = 0.0;
        bool aPrevValid = false;

        for (int i = 0; i < aNbGrid; ++i)
        {
            GridPoint aGP = myGrid[i];

            // Compute distance function value: F(u) = (C(u) - P) . C'(u)
            gp_Vec aVec = new(theP, aGP.Point);
            double aF = aVec.Dot(aGP.D1);
            double aDist = aVec.SquareMagnitude();

            // Check for sign change with previous point
            if (aPrevValid && aPrevF * aF < 0.0 && !myProcessed[i - 1])
            {
                Candidate aCand;
                aCand.Type = CandidateType.SignChange;
                aCand.IdxLo = i - 1;
                aCand.IdxHi = i;
                // Use linear interpolation for better starting point (secant method)
                double aFLo = aPrevF;
                double aFHi = aF;
                double aULo = myGrid[i - 1].Param;
                double aUHi = aGP.Param;
                aCand.StartU = aULo - aFLo * (aUHi - aULo) / (aFHi - aFLo);
                myCandidates.Append(aCand);
                myProcessed[i - 1] = true;
                myProcessed[i] = true;
            }

            // Check for near-zero F (direct hit on extremum)
            if (Math.Abs(aF) < theTol * ExtremaPC.THE_NEAR_ZERO_F_FACTOR && !myProcessed[i])
            {
                Candidate aCand;
                aCand.Type = CandidateType.NearZero;
                aCand.IdxLo = i;
                aCand.IdxHi = i;
                aCand.StartU = aGP.Param;
                myCandidates.Append(aCand);
                myProcessed[i] = true;
            }

            // Check for local extremum by distance comparison (3-point test)
            if (i > 0 && i < aNbGrid - 1 && !myProcessed[i])
            {
                double aNextDist = theP.SquareDistance(myGrid[i + 1].Point);

                // Local minimum: distance decreases then increases
                bool aIsLocalMin = (aDist <= aPrevDist && aDist <= aNextDist);
                // Local maximum: distance increases then decreases
                bool aIsLocalMax = (aDist >= aPrevDist && aDist >= aNextDist);

                if ((theMode == ExtremaPC.SearchMode.Min || theMode == ExtremaPC.SearchMode.MinMax)
                    && aIsLocalMin)
                {
                    Candidate aCand;
                    aCand.Type = CandidateType.NearZero;
                    aCand.IdxLo = i;
                    aCand.IdxHi = i;
                    aCand.StartU = aGP.Param;
                    myCandidates.Append(aCand);
                    myProcessed[i] = true;
                }
                else if ((theMode == ExtremaPC.SearchMode.Max || theMode == ExtremaPC.SearchMode.MinMax)
                         && aIsLocalMax && !aIsLocalMin)
                {
                    Candidate aCand;
                    aCand.Type = CandidateType.NearZero;
                    aCand.IdxLo = i;
                    aCand.IdxHi = i;
                    aCand.StartU = aGP.Param;
                    myCandidates.Append(aCand);
                    myProcessed[i] = true;
                }
            }

            aPrevF = aF;
            aPrevDist = aDist;
            aPrevValid = true;
        }
    }

    // Refines candidates using Newton's method.
    private void refineCandidates(Adaptor3d_Curve theCurve,
                                  in gp_Pnt theP,
                                  in MathUtils.Domain1D theDomain,
                                  double theTol,
                                  ExtremaPC.SearchMode theMode)
    {
        myResult.Status = ExtremaPC.Status.OK;
        myFoundRoots.Clear();
        mySortedIndices.Clear();

        ExtremaPC_DistanceFunction aFunc = new(theCurve, theP);

        // Newton configuration
        MathUtils.Config aConfig = new();
        aConfig.XTolerance = theTol * ExtremaPC.THE_NEWTON_XTOL_FACTOR;
        aConfig.FTolerance = theTol * ExtremaPC.THE_NEWTON_FTOL_FACTOR;
        aConfig.MaxIterations = ExtremaPC.THE_MAX_NEWTON_ITERATIONS;

        // Build sorted indices by estimated distance
        for (int c = 0; c < myCandidates.Length(); ++c)
        {
            Candidate aCand = myCandidates.Value(c);
            double anEstDist = theP.SquareDistance(myGrid[aCand.IdxLo].Point);
            mySortedIndices.Append((c, anEstDist));
        }

        // Sort by estimated distance for Min mode (ascending), Max mode (descending)
        if (theMode == ExtremaPC.SearchMode.Min)
        {
            sortIndices(static (a, b) => a.second < b.second ? -1 : (b.second < a.second ? 1 : 0));
        }
        else if (theMode == ExtremaPC.SearchMode.Max)
        {
            sortIndices(static (a, b) => a.second > b.second ? -1 : (b.second > a.second ? 1 : 0));
        }

        // Best distance found so far (for early termination)
        double aBestSqDist = (theMode == ExtremaPC.SearchMode.Min)
                               ? double.MaxValue
                               : -double.MaxValue;

        for (int s = 0; s < mySortedIndices.Length(); ++s)
        {
            int c = mySortedIndices.Value(s).first;
            double anEstDist = mySortedIndices.Value(s).second;
            Candidate aCand = myCandidates.Value(c);

            // Early termination: skip candidates that are clearly worse than the best found.
            // For Min mode: skip if estimated distance > best * (2.0 - threshold), i.e., ~1.1x best.
            // For Max mode: skip if estimated distance < best * threshold, i.e., ~0.9x best.
            const double aMinSkipThreshold = 2.0 - ExtremaPC.THE_MAX_SKIP_THRESHOLD;
            if (theMode == ExtremaPC.SearchMode.Min && anEstDist > aBestSqDist * aMinSkipThreshold)
            {
                break;
            }
            if (theMode == ExtremaPC.SearchMode.Max
                && anEstDist < aBestSqDist * ExtremaPC.THE_MAX_SKIP_THRESHOLD)
            {
                break;
            }

            // Skip if too close to already found root
            bool aSkip = false;
            for (int r = 0; r < myFoundRoots.Length(); ++r)
            {
                if (Math.Abs(aCand.StartU - myFoundRoots.Value(r)) < theTol)
                {
                    aSkip = true;
                    break;
                }
            }
            if (aSkip)
            {
                continue;
            }

            // Determine Newton bounds
            double aULo, aUHi;
            if (aCand.Type == CandidateType.SignChange)
            {
                aULo = myGrid[aCand.IdxLo].Param;
                aUHi = myGrid[aCand.IdxHi].Param;
            }
            else
            {
                double aExpand = (theDomain.Max - theDomain.Min) * ExtremaPC.THE_INTERVAL_EXPAND_RATIO;
                aULo = Max(theDomain.Min, aCand.StartU - aExpand);
                aUHi = Min(theDomain.Max, aCand.StartU + aExpand);
            }

            // Try Newton refinement
            MathUtils.ScalarResult aNewtonRes =
              MathRoot.NewtonBounded(aFunc, aCand.StartU, aULo, aUHi, aConfig);

            double aRootU = 0.0;
            bool aConverged = false;

            if (aNewtonRes.IsDone())
            {
                aRootU = Max(theDomain.Min, Min(theDomain.Max, aNewtonRes.Root!.Value));
                aConverged = true;
            }
            else
            {
                // Try iterative grid refinement as fallback
                double aBestU = aCand.StartU;
                double aBestDist = double.MaxValue;
                double aRefUMin = aULo;
                double aRefUMax = aUHi;

                for (int aPass = 0; aPass < ExtremaPC.THE_REFINEMENT_NB_PASSES; ++aPass)
                {
                    const int aNbSamples = ExtremaPC.THE_REFINEMENT_NB_SAMPLES;
                    double aStep0 = (aRefUMax - aRefUMin) / (aNbSamples - 1);

                    for (int i = 0; i < aNbSamples; ++i)
                    {
                        double aU = aRefUMin + i * aStep0;
                        gp_Pnt aPt0 = theCurve.Value(aU);
                        double aDist = theP.SquareDistance(aPt0);

                        if (aDist < aBestDist)
                        {
                            aBestDist = aDist;
                            aBestU = aU;
                        }
                    }

                    // Narrow range
                    double aRangeHalf = (aRefUMax - aRefUMin) * ExtremaPC.THE_RANGE_NARROWING_FACTOR * 0.5;
                    aRefUMin = Max(theDomain.Min, aBestU - aRangeHalf);
                    aRefUMax = Min(theDomain.Max, aBestU + aRangeHalf);

                    // Try Newton with refined point
                    MathUtils.ScalarResult aRetryRes =
                      MathRoot.NewtonBounded(aFunc, aBestU, aRefUMin, aRefUMax, aConfig);
                    if (aRetryRes.IsDone())
                    {
                        aRootU = Max(theDomain.Min, Min(theDomain.Max, aRetryRes.Root!.Value));
                        aConverged = true;
                        break;
                    }
                }

                // Use best grid point as fallback
                if (!aConverged)
                {
                    theCurve.D1(aBestU, out gp_Pnt aPt0, out gp_Vec aD1);
                    gp_Vec aVec = new(theP, aPt0);
                    double aF = aVec.Dot(aD1);

                    if (Math.Abs(aF) < theTol * ExtremaPC.THE_FALLBACK_F_FACTOR)
                    {
                        aRootU = aBestU;
                        aConverged = true;
                    }
                }
            }

            if (!aConverged)
                continue;

            // Check for duplicate
            bool aDuplicate = false;
            for (int r = 0; r < myFoundRoots.Length(); ++r)
            {
                if (Math.Abs(aRootU - myFoundRoots.Value(r)) < theTol)
                {
                    aDuplicate = true;
                    break;
                }
            }
            if (aDuplicate)
                continue;

            gp_Pnt aPt = theCurve.Value(aRootU);
            double aSqDist = theP.SquareDistance(aPt);

            // Classify as min/max using neighbor sampling
            double aStep = (theDomain.Max - theDomain.Min) * ExtremaPC.THE_REFINEMENT_STEP_RATIO;
            double aDistPlus =
              theP.SquareDistance(theCurve.Value(Min(theDomain.Max, aRootU + aStep)));
            double aDistMinus =
              theP.SquareDistance(theCurve.Value(Max(theDomain.Min, aRootU - aStep)));
            bool aIsMin = (aSqDist <= aDistPlus) && (aSqDist <= aDistMinus);

            // Filter by mode
            bool aKeep = false;
            if (theMode == ExtremaPC.SearchMode.MinMax)
            {
                aKeep = true;
            }
            else if (theMode == ExtremaPC.SearchMode.Min && aIsMin)
            {
                aKeep = true;
            }
            else if (theMode == ExtremaPC.SearchMode.Max && !aIsMin)
            {
                aKeep = true;
            }

            if (aKeep)
            {
                ExtremaPC.ExtremumResult anExt = new();
                anExt.Parameter = aRootU;
                anExt.Point = aPt;
                anExt.SquareDistance = aSqDist;
                anExt.IsMinimum = aIsMin;
                myResult.Extrema.Append(anExt);

                myFoundRoots.Append(aRootU);

                // Update best distance for early termination
                if (theMode == ExtremaPC.SearchMode.Min && aSqDist < aBestSqDist)
                {
                    aBestSqDist = aSqDist;
                }
                else if (theMode == ExtremaPC.SearchMode.Max && aSqDist > aBestSqDist)
                {
                    aBestSqDist = aSqDist;
                }
            }
        }

        if (myResult.Extrema.IsEmpty() && myCandidates.IsEmpty())
        {
            myResult.Status = ExtremaPC.Status.NoSolution;
        }
    }

    // std::sort of mySortedIndices (decision 59: an unstable sort, as std::sort)
    private void sortIndices(Comparison<(int first, double second)> theCompare)
    {
        int aNb = mySortedIndices.Length();
        (int, double)[] aTmp = new (int, double)[aNb];
        for (int i = 0; i < aNb; ++i)
        {
            aTmp[i] = mySortedIndices.Value(i);
        }
        aTmp.AsSpan().Sort(theCompare);
        for (int i = 0; i < aNb; ++i)
        {
            mySortedIndices[i] = aTmp[i];
        }
    }
}
