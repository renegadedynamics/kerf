// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_Class2d.hxx
// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_Class2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry;

/// <summary>
/// A sequence for the points of a wire on their way to a <see cref="Class2d"/>, which copies them: one kept by each thread, so
/// each classifier built doesn't grow its own from nothing. Taken out while in use, so a nested use makes its own.
/// </summary>
internal static class Class2dPoints
{
    private const int LargestKept = 16384;

    [ThreadStatic]
    private static NCollection_Sequence<Point2D>? t_spare;

    public static NCollection_Sequence<Point2D> Rent()
    {
        NCollection_Sequence<Point2D> points = t_spare ?? new NCollection_Sequence<Point2D>();
        t_spare = null;
        points.Clear();
        return points;
    }

    public static void Return(NCollection_Sequence<Point2D> points)
    {
        if (points.Length() <= LargestKept)
        {
            t_spare = points;
        }
    }
}

/// <summary>
/// Low-level algorithm for 2D point-in-polygon classification, by ray casting: a horizontal ray from the test
/// point is extended to infinity and the number of polygon edge crossings determines the classification.
/// The polygon is normalized internally to the [0,1] x [0,1] domain for numerical stability.
/// </summary>
/// <remarks>
/// OCCT's class is movable but not copyable, so it is a class here. The default constructor makes an empty
/// classifier, for which every point is <see cref="Result.Result_Uncertain"/>.
/// </remarks>
internal sealed class Class2d
{
    /// <summary>Classification result for point-in-polygon tests.</summary>
    public enum Result
    {
        /// <summary>Point is strictly inside the polygon.</summary>
        Result_Inside = 1,
        /// <summary>Point is strictly outside the polygon.</summary>
        Result_Outside = -1,
        /// <summary>Point is on the boundary or classification is uncertain.</summary>
        Result_Uncertain = 0,
    }

    private readonly NCollection_Array1<double> _pnts2dX = new(); // X coordinates (normalized)
    private readonly NCollection_Array1<double> _pnts2dY = new(); // Y coordinates (normalized)
    private double                              _tolU;            // Tolerance in U direction (normalized)
    private double                              _tolV;            // Tolerance in V direction (normalized)
    private int                                 _pointsCount;     // Number of polygon vertices
    private double                              _uMin;            // Original minimum U bound
    private double                              _vMin;            // Original minimum V bound
    private double                              _uMax;            // Original maximum U bound
    private double                              _vMax;            // Original maximum V bound

    /// <summary>Creates an empty classifier.</summary>
    public Class2d() { }

    /// <summary>
    /// Constructs a 2D classifier from the polygon's vertices (at least 3; the polygon is closed automatically),
    /// the tolerances in U and V for boundary detection, and the bounds of the polygon's domain.
    /// </summary>
    public Class2d(NCollection_Array1<Point2D> pnts2d, double tolU, double tolV,
                         double uMin, double vMin, double uMax, double vMax)
    {
        Init(pnts2d.Lower(), pnts2d.Length(), i => pnts2d.Value(i), tolU, tolV, uMin, vMin, uMax, vMax);
    }

    /// <summary>Constructs a 2D classifier from a sequence of polygon vertices (see the array constructor).</summary>
    public Class2d(NCollection_Sequence<Point2D> pnts2d, double tolU, double tolV,
                         double uMin, double vMin, double uMax, double vMax)
    {
        Init(NCollection_Sequence<Point2D>.Lower(), pnts2d.Length(), i => pnts2d.Value(i), tolU, tolV, uMin, vMin, uMax, vMax);
    }

    /// <summary>Constructs a 2D classifier from a vector of polygon vertices (see the array constructor).</summary>
    public Class2d(List<Point2D> pnts2d, double tolU, double tolV,
                         double uMin, double vMin, double uMax, double vMax)
    {
        Init(0, pnts2d.Count, i => pnts2d[i], tolU, tolV, uMin, vMin, uMax, vMax);
    }

    // Transforms a coordinate from original space to normalized [0,1] space,
    // or returns the original value if the range is too small.
    private static double TransformToNormalized(double u, double uMin, double uRange)
    {
        const double THE_MIN_RANGE = 1e-10;
        if (uRange > THE_MIN_RANGE)
        {
            return (u - uMin) / uRange;
        }
        return u;
    }

    // Note: OCCT's template over the container type: the container is given by its lower bound, length and accessor.
    private void Init(int aLower, int aLength, Func<int, Point2D> pnts2d,
                      double tolU, double tolV, double uMin, double vMin, double uMax, double vMax)
    {
        _uMin = uMin;
        _vMin = vMin;
        _uMax = uMax;
        _vMax = vMax;

        // Validate input parameters.
        if (uMax <= uMin || vMax <= vMin || aLength < 3)
        {
            _pointsCount = 0;
            return;
        }

        _pointsCount = aLength;
        _tolU        = tolU;
        _tolV        = tolV;

        // Allocate arrays with one extra element for closing the polygon.
        _pnts2dX.Resize(0, _pointsCount, false);
        _pnts2dY.Resize(0, _pointsCount, false);

        double du = uMax - uMin;
        double dv = vMax - vMin;

        // Transform points to normalized coordinates.
        for (int i = 0; i < _pointsCount; ++i)
        {
            Point2D p2D = pnts2d(i + aLower);
            _pnts2dX[i]  = TransformToNormalized(p2D.X, uMin, du);
            _pnts2dY[i]  = TransformToNormalized(p2D.Y, vMin, dv);
        }

        // Close the polygon by copying first point to last position.
        _pnts2dX.ChangeLast() = _pnts2dX.First();
        _pnts2dY.ChangeLast() = _pnts2dY.First();

        // Normalize tolerances.
        const double THE_MIN_RANGE = 1e-10;
        if (du > THE_MIN_RANGE)
        {
            _tolU /= du;
        }
        if (dv > THE_MIN_RANGE)
        {
            _tolV /= dv;
        }
    }

    /// <summary>Classifies a point relative to the polygon, using the tolerances given at construction.</summary>
    public Result SiDans(in Point2D point)
    {
        if (_pointsCount == 0)
        {
            return Result.Result_Uncertain;
        }

        double x = point.X;
        double y = point.Y;

        // Compute tolerance in original coordinate space.
        double tolU = _tolU * (_uMax - _uMin);
        double tolV = _tolV * (_vMax - _vMin);

        // Quick rejection test for points clearly outside the bounding box.
        if (x < (_uMin - tolU) || x > (_uMax + tolU) || y < (_vMin - tolV)
            || y > (_vMax + tolV))
        {
            return Result.Result_Outside;
        }

        // Transform to normalized coordinates.
        x = TransformToNormalized(x, _uMin, _uMax - _uMin);
        y = TransformToNormalized(y, _vMin, _vMax - _vMin);

        // Perform classification with ON detection.
        Result result = InternalSiDansOuOn(x, y);
        if (result == Result.Result_Uncertain)
        {
            return Result.Result_Uncertain; // ON boundary
        }

        // Check corner points with tolerance for boundary detection.
        if (_tolU > 0.0 || _tolV > 0.0)
        {
            bool isInside = (result == Result.Result_Inside);
            if (isInside != InternalSiDans(x - _tolU, y - _tolV)
                || isInside != InternalSiDans(x + _tolU, y - _tolV)
                || isInside != InternalSiDans(x - _tolU, y + _tolV)
                || isInside != InternalSiDans(x + _tolU, y + _tolV))
            {
                return Result.Result_Uncertain; // Near boundary
            }
        }

        return result;
    }

    /// <summary>Classifies a point relative to the polygon, with <paramref name="tol"/> as the ON tolerance.</summary>
    public Result SiDans_OnMode(in Point2D point, double tol)
    {
        if (_pointsCount == 0)
        {
            return Result.Result_Uncertain;
        }

        double x = point.X;
        double y = point.Y;

        // Quick rejection test.
        if (x < (_uMin - tol) || x > (_uMax + tol) || y < (_vMin - tol)
            || y > (_vMax + tol))
        {
            return Result.Result_Outside;
        }

        // Transform to normalized coordinates.
        x = TransformToNormalized(x, _uMin, _uMax - _uMin);
        y = TransformToNormalized(y, _vMin, _vMax - _vMin);

        // Perform classification with ON detection.
        Result result = InternalSiDansOuOn(x, y);

        // Check corner points with tolerance.
        if (tol > 0.0)
        {
            bool isInside = (result == Result.Result_Inside);
            if (isInside != InternalSiDans(x - tol, y - tol)
                || isInside != InternalSiDans(x + tol, y - tol)
                || isInside != InternalSiDans(x - tol, y + tol)
                || isInside != InternalSiDans(x + tol, y + tol))
            {
                return Result.Result_Uncertain;
            }
        }

        return result;
    }

    // Ray-casting classification in normalized coordinates: true if inside.
    private bool InternalSiDans(double px, double py)
    {
        // Count edge crossings with a horizontal ray from (Px, Py) to +infinity.
        int nbCrossings = 0;

        double prevDx          = _pnts2dX.Value(0) - px;
        double prevDy          = _pnts2dY.Value(0) - py;
        bool   prevYIsNegative = (prevDy < 0.0);

        for (int nextIdx = 1; nextIdx <= _pointsCount; ++nextIdx)
        {
            double currDx          = _pnts2dX.Value(nextIdx) - px;
            double currDy          = _pnts2dY.Value(nextIdx) - py;
            bool   currYIsNegative = (currDy < 0.0);

            // Check for edge crossing when Y changes sign.
            if (currYIsNegative != prevYIsNegative)
            {
                if (prevDx > 0.0 && currDx > 0.0)
                {
                    // Both endpoints are to the right of the test point.
                    ++nbCrossings;
                }
                else if (prevDx > 0.0 || currDx > 0.0)
                {
                    // Compute X intersection with horizontal line Y = 0.
                    double xIntersect = prevDx - prevDy * (currDx - prevDx) / (currDy - prevDy);
                    if (xIntersect > 0.0)
                    {
                        ++nbCrossings;
                    }
                }
                prevYIsNegative = currYIsNegative;
            }

            prevDx = currDx;
            prevDy = currDy;
        }

        // Odd number of crossings means inside.
        return (nbCrossings & 1) != 0;
    }

    // Ray-casting classification with ON detection, in normalized coordinates.
    private Result InternalSiDansOuOn(double px, double py)
    {
        int nbCrossings = 0;

        double prevDx          = _pnts2dX.Value(0) - px;
        double prevDy          = _pnts2dY.Value(0) - py;
        bool   prevYIsNegative = (prevDy < 0.0);

        for (int nextIdx = 1; nextIdx <= _pointsCount; ++nextIdx)
        {
            int    prevIdx = nextIdx - 1;
            double currDx  = _pnts2dX.Value(nextIdx) - px;
            double currDy  = _pnts2dY.Value(nextIdx) - py;

            // Check if point is very close to current vertex.
            if (currDx < _tolU && currDx > -_tolU && currDy < _tolV && currDy > -_tolV)
            {
                return Result.Result_Uncertain; // ON boundary (at vertex)
            }

            // Check if point is ON the edge by computing Y at the test point's X.
            // Skip interpolation for nearly vertical edges to avoid division instability.
            // For vertical edges, the ON detection is handled by the tolerance check above.
            double edgeDx = _pnts2dX.Value(nextIdx) - _pnts2dX.Value(prevIdx);
            if ((_pnts2dX.Value(prevIdx) - px) * currDx < 0.0
                && Math.Abs(edgeDx) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                double interpY =
                  _pnts2dY.Value(nextIdx)
                  - (_pnts2dY.Value(nextIdx) - _pnts2dY.Value(prevIdx)) / edgeDx * currDx;
                double deltaY = interpY - py;
                if (deltaY >= -_tolV && deltaY <= _tolV)
                {
                    return Result.Result_Uncertain; // ON boundary (on edge)
                }
            }

            bool currYIsNegative = (currDy < 0.0);
            if (currYIsNegative != prevYIsNegative)
            {
                if (prevDx > 0.0 && currDx > 0.0)
                {
                    ++nbCrossings;
                }
                else if (prevDx > 0.0 || currDx > 0.0)
                {
                    double xIntersect = prevDx - prevDy * (currDx - prevDx) / (currDy - prevDy);
                    if (xIntersect > 0.0)
                    {
                        ++nbCrossings;
                    }
                }
                prevYIsNegative = currYIsNegative;
            }

            prevDx = currDx;
            prevDy = currDy;
        }

        // Odd number of crossings means inside.
        return ((nbCrossings & 1) != 0) ? Result.Result_Inside : Result.Result_Outside;
    }
}
