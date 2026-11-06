// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics.Utilities;

using Kerf.Geometry;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Common types and utilities for point-curve extrema computation (OCCT 8's ExtremaPC package): the precision
/// constants, the result types, the configuration and the endpoint handling shared by the evaluators.
/// </summary>
/// <remarks>
/// OCCT's alias <c>ExtremaPC::Domain1D</c> is <see cref="MathUtils.Domain1D"/>: C# can't export a type alias.
/// </remarks>
internal static partial class ExtremaPC
{
    //==============================================================================================
    // Precision constants
    //==============================================================================================

    /// <summary>Default tolerance for root finding and distance comparison (Precision::Confusion()).</summary>
    public const double THE_DEFAULT_TOLERANCE = 1.0e-7;

    /// <summary>Tolerance for parameter domain comparison (Precision::PConfusion()).</summary>
    public const double THE_PARAM_TOLERANCE = 1.0e-9;

    /// <summary>Ratio of parameter range for neighbor point sampling.</summary>
    public const double THE_NEIGHBOR_STEP_RATIO = 0.01;

    /// <summary>Ratio of parameter range for search interval expansion.</summary>
    public const double THE_INTERVAL_EXPAND_RATIO = 0.05;

    /// <summary>Ratio of parameter range for refinement step.</summary>
    public const double THE_REFINEMENT_STEP_RATIO = 0.001;

    /// <summary>Multiplier for X (parameter) tolerance in Newton refinement.</summary>
    public const double THE_NEWTON_XTOL_FACTOR = 0.01;

    /// <summary>Multiplier for F (function) tolerance in Newton refinement.</summary>
    public const double THE_NEWTON_FTOL_FACTOR = 0.001;

    /// <summary>Default search radius for hint-based search (fraction of parameter range).</summary>
    public const double THE_HINT_SEARCH_RADIUS = 0.1;

    /// <summary>Factor for narrowing parameter range during refinement iterations.</summary>
    public const double THE_RANGE_NARROWING_FACTOR = 0.25;

    /// <summary>Threshold for skipping max candidates that are clearly worse.</summary>
    public const double THE_MAX_SKIP_THRESHOLD = 0.9;

    /// <summary>Multiplier for near-zero F detection during grid scan.</summary>
    public const double THE_NEAR_ZERO_F_FACTOR = 10.0;

    /// <summary>Multiplier for fallback F tolerance when Newton fails.</summary>
    public const double THE_FALLBACK_F_FACTOR = 100.0;

    /// <summary>Maximum number of Newton iterations for refinement.</summary>
    public const int THE_MAX_NEWTON_ITERATIONS = 20;

    /// <summary>Number of samples for iterative grid refinement fallback.</summary>
    public const int THE_REFINEMENT_NB_SAMPLES = 20;

    /// <summary>Number of refinement passes when Newton fails.</summary>
    public const int THE_REFINEMENT_NB_PASSES = 3;

    /// <summary>Minimum number of samples for Bezier curves.</summary>
    public const int THE_BEZIER_MIN_SAMPLES = 24;

    /// <summary>Multiplier for degree to compute Bezier samples: samples = max(min, multiplier * (degree + 1)).</summary>
    public const int THE_BEZIER_DEGREE_MULTIPLIER = 3;

    /// <summary>Default number of samples for general (other) curves.</summary>
    public const int THE_OTHER_CURVE_NB_SAMPLES = 64;

    /// <summary>Fallback number of samples for BSpline curves when curve is null.</summary>
    public const int THE_BSPLINE_FALLBACK_SAMPLES = 32;

    /// <summary>Multiplier for BSpline curve samples per knot span: samples = multiplier * (degree + 1).</summary>
    public const int THE_BSPLINE_SPAN_MULTIPLIER = 2;

    /// <summary>Status of extrema computation.</summary>
    public enum Status
    {
        /// <summary>Computation succeeded, finite number of extrema found.</summary>
        OK,
        /// <summary>Computation not performed.</summary>
        NotDone,
        /// <summary>Infinite solutions exist (e.g., point at circle center).</summary>
        InfiniteSolutions,
        /// <summary>No extrema found in the given parameter range.</summary>
        NoSolution,
        /// <summary>Numerical issues during computation.</summary>
        NumericalError
    }

    /// <summary>Search mode for extrema computation: which extrema to find.</summary>
    public enum SearchMode
    {
        /// <summary>Find all extrema (both minima and maxima) - default.</summary>
        MinMax,
        /// <summary>Find only minimum distance.</summary>
        Min,
        /// <summary>Find only maximum distance.</summary>
        Max
    }

    /// <summary>Result of a single extremum computation.</summary>
    /// <remarks>A C++ aggregate with public fields (decision 54); <c>new()</c> has OCCT's defaults (IsMinimum true),
    /// <c>default</c> has IsMinimum false.</remarks>
    public struct ExtremumResult
    {
        /// <summary>Parameter value on curve.</summary>
        public double Parameter = 0.0;
        /// <summary>Point on curve at parameter.</summary>
        public Point3D Point;
        /// <summary>Square of the distance from query point to curve point.</summary>
        public double SquareDistance = 0.0;
        /// <summary>True if this is a local minimum, false if maximum.</summary>
        public bool IsMinimum = true;

        public ExtremumResult()
        {
        }
    }

    /// <summary>
    /// Result of extrema computation containing all found extrema.
    /// </summary>
    /// <remarks>
    /// Non-copyable in OCCT: the evaluators return a reference to their own reusable result, which the next
    /// <c>Perform</c> overwrites. Its state is changed only inside Kerf (internal setters and the internal
    /// <see cref="_extrema"/> collection); callers read it through <see cref="ExtCount"/> and the indexer.
    /// </remarks>
    public sealed class Result
    {
        /// <summary>Computation status.</summary>
        public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status Status { get; internal set; } = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NotDone;

        /// <summary>Collection of found extrema (0-based).</summary>
        internal readonly List<ExtremumResult> _extrema = new(8);

        /// <summary>For infinite solutions, the constant squared distance.</summary>
        public double InfiniteSquareDistance { get; internal set; } = 0.0;

        /// <summary>Returns true if computation succeeded with finite number of extrema.</summary>
        public bool IsDone => Status == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;

        /// <summary>Returns true if there are infinite solutions.</summary>
        public bool IsInfinite => Status == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.InfiniteSolutions;

        /// <summary>Returns number of extrema found (0 if infinite or failed).</summary>
        public int ExtCount => _extrema.Count;

        /// <summary>Access extremum by 0-based index.</summary>
        public ExtremumResult this[int index] => _extrema[index];

        /// <summary>Returns the squared distance of the closest extremum, or infinity if no extrema found.</summary>
        public double MinSquareDistance()
        {
            if (_extrema.Count == 0)
            {
                return double.PositiveInfinity;
            }
            double minSqDist = _extrema[0].SquareDistance;
            for (int i = 1; i < _extrema.Count; ++i)
            {
                if (_extrema[i].SquareDistance < minSqDist)
                {
                    minSqDist = _extrema[i].SquareDistance;
                }
            }
            return minSqDist;
        }

        /// <summary>Returns the index of the closest extremum (0-based), or -1 if no extrema found.</summary>
        public int MinIndex()
        {
            if (_extrema.Count == 0)
            {
                return -1;
            }
            int minIdx = 0;
            double minSqDist = _extrema[0].SquareDistance;
            for (int i = 1; i < _extrema.Count; ++i)
            {
                if (_extrema[i].SquareDistance < minSqDist)
                {
                    minSqDist = _extrema[i].SquareDistance;
                    minIdx = i;
                }
            }
            return minIdx;
        }

        /// <summary>Returns the squared distance of the farthest extremum, or 0 if no extrema found.</summary>
        public double MaxSquareDistance()
        {
            if (_extrema.Count == 0)
            {
                return 0.0;
            }
            double maxSqDist = _extrema[0].SquareDistance;
            for (int i = 1; i < _extrema.Count; ++i)
            {
                if (_extrema[i].SquareDistance > maxSqDist)
                {
                    maxSqDist = _extrema[i].SquareDistance;
                }
            }
            return maxSqDist;
        }

        /// <summary>Returns the index of the farthest extremum (0-based), or -1 if no extrema found.</summary>
        public int MaxIndex()
        {
            if (_extrema.Count == 0)
            {
                return -1;
            }
            int maxIdx = 0;
            double maxSqDist = _extrema[0].SquareDistance;
            for (int i = 1; i < _extrema.Count; ++i)
            {
                if (_extrema[i].SquareDistance > maxSqDist)
                {
                    maxSqDist = _extrema[i].SquareDistance;
                    maxIdx = i;
                }
            }
            return maxIdx;
        }

        /// <summary>Clears the result for reuse.</summary>
        internal void Clear()
        {
            Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NotDone;
            _extrema.Clear();
            InfiniteSquareDistance = 0.0;
        }
    }

    /// <summary>Configuration for extrema computation.</summary>
    /// <remarks>A class with public fields (decision 53); a C++ copy is <c>new(theConfig)</c>.</remarks>
    public sealed class Config
    {
        /// <summary>Tolerance for root finding.</summary>
        public double Tolerance = THE_DEFAULT_TOLERANCE;
        /// <summary>Parameter domain (null = use natural/unbounded).</summary>
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? Domain;
        /// <summary>Number of samples for numerical methods.</summary>
        public int NbSamples = 32;
        /// <summary>Search mode (MinMax, Min, or Max).</summary>
        public SearchMode Mode = SearchMode.MinMax;
        /// <summary>Include endpoints as potential extrema.</summary>
        public bool IncludeEndpoints = true;

        public Config()
        {
        }

        public Config(Config other)
        {
            Tolerance = other.Tolerance;
            Domain = other.Domain;
            NbSamples = other.NbSamples;
            Mode = other.Mode;
            IncludeEndpoints = other.IncludeEndpoints;
        }
    }

    /// <summary>
    /// The member set shared by the point-curve evaluators (an addition: OCCT's templates and its
    /// <c>std::variant</c> rely on duck typing).
    /// </summary>
    public interface ICurveEvaluator
    {
        /// <summary>Evaluates the point on the curve at a parameter.</summary>
        Point3D Value(double u);

        /// <summary>Interior extrema between a point and the curve.</summary>
        Result Perform(in Point3D p, double tol, SearchMode mode = SearchMode.MinMax);

        /// <summary>Interior and endpoint extrema between a point and the curve.</summary>
        Result PerformWithEndpoints(in Point3D p, double tol, SearchMode mode = SearchMode.MinMax);
    }

    /// <summary>
    /// Adds the curve endpoints to a result when the domain is finite, the endpoint is a true local extremum
    /// (checked against a neighbouring point) and it doesn't duplicate an existing extremum.
    /// </summary>
    internal static void AddEndpointExtrema<CurveEvaluator>(Result result,
                                                            in Point3D p,
                                                            in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain,
                                                            CurveEvaluator eval,
                                                            double tol,
                                                            SearchMode mode)
        where CurveEvaluator : ICurveEvaluator
    {
        // Check for infinite bounds
        if (!domain.IsFinite())
        {
            return;
        }

        double theUMin = domain.Min;
        double theUMax = domain.Max;

        // Evaluate endpoints
        Point3D ptMin = eval.Value(theUMin);
        Point3D ptMax = eval.Value(theUMax);

        double sqDistMin = p.SquareDistanceTo(ptMin);
        double sqDistMax = p.SquareDistanceTo(ptMax);

        // Sample step for neighbor point evaluation
        double step = (theUMax - theUMin) * THE_NEIGHBOR_STEP_RATIO;
        if (step < tol)
        {
            step = tol;
        }

        // Check if UMin is a local extremum by comparing to neighbor
        Point3D neighborMin = eval.Value(theUMin + step);
        double neighborDistMin = p.SquareDistanceTo(neighborMin);
        bool isMinAtUMin = (sqDistMin <= neighborDistMin);
        bool isMaxAtUMin = (sqDistMin >= neighborDistMin);

        // Check if UMax is a local extremum by comparing to neighbor
        Point3D neighborMax = eval.Value(theUMax - step);
        double neighborDistMax = p.SquareDistanceTo(neighborMax);
        bool isMinAtUMax = (sqDistMax <= neighborDistMax);
        bool isMaxAtUMax = (sqDistMax >= neighborDistMax);

        // Also check if endpoints themselves are duplicates (for periodic curves)
        bool endpointsAreSame = ptMin.SquareDistanceTo(ptMax) < tol * tol;

        // For periodic/closed curves, there are no true endpoints to consider
        if (endpointsAreSame)
        {
            return;
        }

        // Add endpoints only if they are true local extrema matching the search mode
        if (mode == SearchMode.Min || mode == SearchMode.MinMax)
        {
            // Add UMin if it's a local minimum
            if (isMinAtUMin && !IsDuplicate(result, tol, theUMin, ptMin))
            {
                ExtremumResult ext = new();
                ext.Parameter = theUMin;
                ext.Point = ptMin;
                ext.SquareDistance = sqDistMin;
                ext.IsMinimum = true;
                result._extrema.Add(ext);
            }
            // Add UMax if it's a local minimum (and not same point as UMin)
            if (isMinAtUMax && !endpointsAreSame && !IsDuplicate(result, tol, theUMax, ptMax))
            {
                ExtremumResult ext = new();
                ext.Parameter = theUMax;
                ext.Point = ptMax;
                ext.SquareDistance = sqDistMax;
                ext.IsMinimum = true;
                result._extrema.Add(ext);
            }
        }

        if (mode == SearchMode.Max || mode == SearchMode.MinMax)
        {
            // Add UMin if it's a local maximum
            if (isMaxAtUMin && !isMinAtUMin && !IsDuplicate(result, tol, theUMin, ptMin))
            {
                ExtremumResult ext = new();
                ext.Parameter = theUMin;
                ext.Point = ptMin;
                ext.SquareDistance = sqDistMin;
                ext.IsMinimum = false;
                result._extrema.Add(ext);
            }
            // Add UMax if it's a local maximum (and not same point as UMin)
            if (isMaxAtUMax && !isMinAtUMax && !endpointsAreSame && !IsDuplicate(result, tol, theUMax, ptMax))
            {
                ExtremumResult ext = new();
                ext.Parameter = theUMax;
                ext.Point = ptMax;
                ext.SquareDistance = sqDistMax;
                ext.IsMinimum = false;
                result._extrema.Add(ext);
            }
        }
    }

    // AddEndpointExtrema's lambda: whether a parameter or point already exists in the result
    private static bool IsDuplicate(Result result, double tol, double u, in Point3D pt)
    {
        for (int i = 0; i < result._extrema.Count; ++i)
        {
            // Check parameter proximity
            if (Math.Abs(result._extrema[i].Parameter - u) < tol)
            {
                return true;
            }
            // Check point proximity (handles periodic curves where different params map to same point)
            double sqDist = result._extrema[i].Point.SquareDistanceTo(pt);
            if (sqDist < tol * tol)
            {
                return true;
            }
        }
        return false;
    }
}
