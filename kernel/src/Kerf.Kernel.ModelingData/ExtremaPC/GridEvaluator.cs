// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_GridEvaluator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Grid-based point-curve extrema computation with cached state for repeated queries. Used by the B-spline,
/// Bezier, offset and other curve evaluators.
/// </summary>
/// <remarks>
/// Algorithm: build a grid of (parameter, point, D1) through GeomGridEval; scan it for candidate intervals
/// (sign changes of F(u), near-zero F, 3-point distance extrema); refine each candidate by Newton's method. The
/// temporary collections are fields reused through <c>Clear()</c>, as OCCT's mutable members.
/// </remarks>
public sealed class GridEvaluator
{
    /// <summary>Cached grid point with pre-computed data.</summary>
    public struct GridPoint
    {
        /// <summary>Parameter value.</summary>
        public double Param;
        /// <summary>Curve point C(u).</summary>
        public Point3D Point;
        /// <summary>First derivative C'(u).</summary>
        public Vector3D D1;
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

    private NCollection_Array1<GridPoint> _grid = new(); // Cached grid

    // Cached temporaries (reused via Clear())
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _result = new();                            // Reusable result
    private readonly NCollection_DynamicArray<Candidate> _candidates = new();     // Candidates from grid scan
    private readonly NCollection_DynamicArray<double> _foundRoots = new();        // Found roots for dedup
    private readonly NCollection_DynamicArray<(int first, double second)> _sortedIndices = new(); // Sorted candidate indices
    private NCollection_Array1<bool> _processed = new();                          // Processed flags for grid scan

    public GridEvaluator()
    {
    }

    /// <summary>
    /// Builds the grid from GeomGridEval D1 results at the given parameters (any bounds; ascending for B-spline
    /// evaluators, as GeomGridEval requires).
    /// </summary>
    public void BuildGrid<TGridEval>(TGridEval eval, math_Vector theParams)
        where TGridEval : GeomGridEvalGrid.ICurveEvaluator
    {
        // Use Array1() accessor to pass to GeomGridEval which expects NCollection_Array1
        NCollection_Array1<Curve3D.ResD1> d1Grid = eval.EvaluateGridD1(theParams.Array1());

        int nbParams = theParams.Length();

        // Resize grid if needed
        if (_grid.Length() != nbParams)
        {
            _grid = new NCollection_Array1<GridPoint>(0, nbParams - 1);
        }

        for (int i = 0; i < nbParams; ++i)
        {
            int d1Idx = d1Grid.Lower() + i;

            _grid[i].Param = theParams[theParams.Lower() + i];
            _grid[i].Point = d1Grid.Value(d1Idx).Point;
            _grid[i].D1 = d1Grid.Value(d1Idx).D1;
        }
    }

    /// <summary>Returns the cached grid (shared: don't modify).</summary>
    public NCollection_Array1<GridPoint> Grid() => _grid;

    /// <summary>Returns the reusable result, for post-processing.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Result() => _result;

    /// <summary>Extrema computation on the cached grid (interior only).</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(CurveAdaptorBase curve,
                                    in Point3D p,
                                    in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain,
                                    double tol,
                                    global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        _result.Clear();
        ScanGrid(p, tol, mode);
        RefineCandidates(curve, p, domain, tol, mode);

        if (!_result._extrema.IsEmpty())
        {
            _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
        }
        return _result;
    }

    /// <summary>Builds a uniform parameter grid (1-based).</summary>
    public static math_Vector BuildUniformParams(double uMin, double uMax, int nbSamples)
    {
        math_Vector aParams = new(1, nbSamples);
        double step = (uMax - uMin) / (nbSamples - 1);

        for (int i = 1; i <= nbSamples; ++i)
        {
            aParams[i] = uMin + (i - 1) * step;
        }
        aParams[nbSamples] = uMax; // Ensure exact endpoint

        return aParams;
    }

    // Scans the grid to find candidate intervals for extrema.
    private void ScanGrid(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        _candidates.Clear();
        int nbGrid = _grid.Length();

        if (nbGrid < 2)
        {
            return;
        }

        // Resize processed array if needed
        if (_processed.Length() != nbGrid)
        {
            _processed = new NCollection_Array1<bool>(0, nbGrid - 1);
        }
        _processed.Init(false);

        double prevF = 0.0;
        double prevDist = 0.0;
        bool prevValid = false;

        for (int i = 0; i < nbGrid; ++i)
        {
            GridPoint gp = _grid[i];

            // Compute distance function value: F(u) = (C(u) - P) . C'(u)
            Vector3D vec = new(p, gp.Point);
            double f = vec.Dot(gp.D1);
            double dist = vec.SquareMagnitude();

            // Check for sign change with previous point
            if (prevValid && prevF * f < 0.0 && !_processed[i - 1])
            {
                Candidate cand;
                cand.Type = CandidateType.SignChange;
                cand.IdxLo = i - 1;
                cand.IdxHi = i;
                // Use linear interpolation for better starting point (secant method)
                double fLo = prevF;
                double fHi = f;
                double uLo = _grid[i - 1].Param;
                double uHi = gp.Param;
                cand.StartU = uLo - fLo * (uHi - uLo) / (fHi - fLo);
                _candidates.Append(cand);
                _processed[i - 1] = true;
                _processed[i] = true;
            }

            // Check for near-zero F (direct hit on extremum)
            if (Math.Abs(f) < tol * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_NEAR_ZERO_F_FACTOR && !_processed[i])
            {
                Candidate cand;
                cand.Type = CandidateType.NearZero;
                cand.IdxLo = i;
                cand.IdxHi = i;
                cand.StartU = gp.Param;
                _candidates.Append(cand);
                _processed[i] = true;
            }

            // Check for local extremum by distance comparison (3-point test)
            if (i > 0 && i < nbGrid - 1 && !_processed[i])
            {
                double nextDist = p.SquareDistance(_grid[i + 1].Point);

                // Local minimum: distance decreases then increases
                bool isLocalMin = (dist <= prevDist && dist <= nextDist);
                // Local maximum: distance increases then decreases
                bool isLocalMax = (dist >= prevDist && dist >= nextDist);

                if ((mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min || mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
                    && isLocalMin)
                {
                    Candidate cand;
                    cand.Type = CandidateType.NearZero;
                    cand.IdxLo = i;
                    cand.IdxHi = i;
                    cand.StartU = gp.Param;
                    _candidates.Append(cand);
                    _processed[i] = true;
                }
                else if ((mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max || mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
                         && isLocalMax && !isLocalMin)
                {
                    Candidate cand;
                    cand.Type = CandidateType.NearZero;
                    cand.IdxLo = i;
                    cand.IdxHi = i;
                    cand.StartU = gp.Param;
                    _candidates.Append(cand);
                    _processed[i] = true;
                }
            }

            prevF = f;
            prevDist = dist;
            prevValid = true;
        }
    }

    // Refines candidates using Newton's method.
    private void RefineCandidates(CurveAdaptorBase curve,
                                  in Point3D p,
                                  in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain,
                                  double tol,
                                  global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
        _foundRoots.Clear();
        _sortedIndices.Clear();

        DistanceFunction func = new(curve, p);

        // Newton configuration
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config config = new();
        config.XTolerance = tol * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_NEWTON_XTOL_FACTOR;
        config.FTolerance = tol * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_NEWTON_FTOL_FACTOR;
        config.MaxIterations = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_MAX_NEWTON_ITERATIONS;

        // Build sorted indices by estimated distance
        for (int c = 0; c < _candidates.Length(); ++c)
        {
            Candidate cand = _candidates.Value(c);
            double estDist = p.SquareDistance(_grid[cand.IdxLo].Point);
            _sortedIndices.Append((c, estDist));
        }

        // Sort by estimated distance for Min mode (ascending), Max mode (descending)
        if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min)
        {
            SortIndices(static (a, b) => a.second < b.second ? -1 : (b.second < a.second ? 1 : 0));
        }
        else if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max)
        {
            SortIndices(static (a, b) => a.second > b.second ? -1 : (b.second > a.second ? 1 : 0));
        }

        // Best distance found so far (for early termination)
        double bestSqDist = (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min)
                               ? double.MaxValue
                               : -double.MaxValue;

        for (int s = 0; s < _sortedIndices.Length(); ++s)
        {
            int c = _sortedIndices.Value(s).first;
            double estDist = _sortedIndices.Value(s).second;
            Candidate cand = _candidates.Value(c);

            // Early termination: skip candidates that are clearly worse than the best found.
            // For Min mode: skip if estimated distance > best * (2.0 - threshold), i.e., ~1.1x best.
            // For Max mode: skip if estimated distance < best * threshold, i.e., ~0.9x best.
            const double minSkipThreshold = 2.0 - global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_MAX_SKIP_THRESHOLD;
            if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min && estDist > bestSqDist * minSkipThreshold)
            {
                break;
            }
            if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max
                && estDist < bestSqDist * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_MAX_SKIP_THRESHOLD)
            {
                break;
            }

            // Skip if too close to already found root
            bool skip = false;
            for (int r = 0; r < _foundRoots.Length(); ++r)
            {
                if (Math.Abs(cand.StartU - _foundRoots.Value(r)) < tol)
                {
                    skip = true;
                    break;
                }
            }
            if (skip)
            {
                continue;
            }

            // Determine Newton bounds
            double uLo, uHi;
            if (cand.Type == CandidateType.SignChange)
            {
                uLo = _grid[cand.IdxLo].Param;
                uHi = _grid[cand.IdxHi].Param;
            }
            else
            {
                double expand = (domain.Max - domain.Min) * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_INTERVAL_EXPAND_RATIO;
                uLo = Max(domain.Min, cand.StartU - expand);
                uHi = Min(domain.Max, cand.StartU + expand);
            }

            // Try Newton refinement
            global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult newtonRes =
              global::Kerf.Engine.Numerics.Roots.MathRoot.NewtonBounded(func, cand.StartU, uLo, uHi, config);

            double rootU = 0.0;
            bool converged = false;

            if (newtonRes.IsDone)
            {
                rootU = Max(domain.Min, Min(domain.Max, newtonRes.Root!.Value));
                converged = true;
            }
            else
            {
                // Try iterative grid refinement as fallback
                double bestU = cand.StartU;
                double bestDist = double.MaxValue;
                double refUMin = uLo;
                double refUMax = uHi;

                for (int pass = 0; pass < global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_REFINEMENT_NB_PASSES; ++pass)
                {
                    const int nbSamples = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_REFINEMENT_NB_SAMPLES;
                    double step0 = (refUMax - refUMin) / (nbSamples - 1);

                    for (int i = 0; i < nbSamples; ++i)
                    {
                        double u = refUMin + i * step0;
                        Point3D pt0 = curve.Value(u);
                        double dist = p.SquareDistance(pt0);

                        if (dist < bestDist)
                        {
                            bestDist = dist;
                            bestU = u;
                        }
                    }

                    // Narrow range
                    double rangeHalf = (refUMax - refUMin) * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_RANGE_NARROWING_FACTOR * 0.5;
                    refUMin = Max(domain.Min, bestU - rangeHalf);
                    refUMax = Min(domain.Max, bestU + rangeHalf);

                    // Try Newton with refined point
                    global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult retryRes =
                      global::Kerf.Engine.Numerics.Roots.MathRoot.NewtonBounded(func, bestU, refUMin, refUMax, config);
                    if (retryRes.IsDone)
                    {
                        rootU = Max(domain.Min, Min(domain.Max, retryRes.Root!.Value));
                        converged = true;
                        break;
                    }
                }

                // Use best grid point as fallback
                if (!converged)
                {
                    curve.D1(bestU, out Point3D pt0, out Vector3D d1);
                    Vector3D vec = new(p, pt0);
                    double f = vec.Dot(d1);

                    if (Math.Abs(f) < tol * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_FALLBACK_F_FACTOR)
                    {
                        rootU = bestU;
                        converged = true;
                    }
                }
            }

            if (!converged)
                continue;

            // Check for duplicate
            bool duplicate = false;
            for (int r = 0; r < _foundRoots.Length(); ++r)
            {
                if (Math.Abs(rootU - _foundRoots.Value(r)) < tol)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate)
                continue;

            Point3D pt = curve.Value(rootU);
            double sqDist = p.SquareDistance(pt);

            // Classify as min/max using neighbor sampling
            double step = (domain.Max - domain.Min) * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_REFINEMENT_STEP_RATIO;
            double distPlus =
              p.SquareDistance(curve.Value(Min(domain.Max, rootU + step)));
            double distMinus =
              p.SquareDistance(curve.Value(Max(domain.Min, rootU - step)));
            bool isMin = (sqDist <= distPlus) && (sqDist <= distMinus);

            // Filter by mode
            bool keep = false;
            if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
            {
                keep = true;
            }
            else if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min && isMin)
            {
                keep = true;
            }
            else if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max && !isMin)
            {
                keep = true;
            }

            if (keep)
            {
                global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult ext = new();
                ext.Parameter = rootU;
                ext.Point = pt;
                ext.SquareDistance = sqDist;
                ext.IsMinimum = isMin;
                _result._extrema.Append(ext);

                _foundRoots.Append(rootU);

                // Update best distance for early termination
                if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min && sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                }
                else if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max && sqDist > bestSqDist)
                {
                    bestSqDist = sqDist;
                }
            }
        }

        if (_result._extrema.IsEmpty() && _candidates.IsEmpty())
        {
            _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NoSolution;
        }
    }

    // std::sort of mySortedIndices (decision 59: an unstable sort, as std::sort)
    private void SortIndices(Comparison<(int first, double second)> compare)
    {
        int nb = _sortedIndices.Length();
        (int, double)[] tmp = new (int, double)[nb];
        for (int i = 0; i < nb; ++i)
        {
            tmp[i] = _sortedIndices.Value(i);
        }
        tmp.AsSpan().Sort(compare);
        for (int i = 0; i < nb; ++i)
        {
            _sortedIndices[i] = tmp[i];
        }
    }
}
