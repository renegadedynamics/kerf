// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_Class2d.hxx
// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_Class2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Low-level algorithm for 2D point-in-polygon classification, by ray casting: a horizontal ray from the test
/// point is extended to infinity and the number of polygon edge crossings determines the classification.
/// The polygon is normalized internally to the [0,1] x [0,1] domain for numerical stability.
/// </summary>
/// <remarks>
/// OCCT's class is movable but not copyable, so it is a class here. The default constructor makes an empty
/// classifier, for which every point is <see cref="Result.Result_Uncertain"/>.
/// </remarks>
public sealed class CSLib_Class2d
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

    private readonly NCollection_Array1<double> myPnts2dX = new(); // X coordinates (normalized)
    private readonly NCollection_Array1<double> myPnts2dY = new(); // Y coordinates (normalized)
    private double                              myTolU;            // Tolerance in U direction (normalized)
    private double                              myTolV;            // Tolerance in V direction (normalized)
    private int                                 myPointsCount;     // Number of polygon vertices
    private double                              myUMin;            // Original minimum U bound
    private double                              myVMin;            // Original minimum V bound
    private double                              myUMax;            // Original maximum U bound
    private double                              myVMax;            // Original maximum V bound

    /// <summary>Creates an empty classifier.</summary>
    public CSLib_Class2d() { }

    /// <summary>
    /// Constructs a 2D classifier from the polygon's vertices (at least 3; the polygon is closed automatically),
    /// the tolerances in U and V for boundary detection, and the bounds of the polygon's domain.
    /// </summary>
    public CSLib_Class2d(NCollection_Array1<gp_Pnt2d> thePnts2d, double theTolU, double theTolV,
                         double theUMin, double theVMin, double theUMax, double theVMax)
    {
        init(thePnts2d.Lower(), thePnts2d.Length(), i => thePnts2d.Value(i), theTolU, theTolV, theUMin, theVMin, theUMax, theVMax);
    }

    /// <summary>Constructs a 2D classifier from a sequence of polygon vertices (see the array constructor).</summary>
    public CSLib_Class2d(NCollection_Sequence<gp_Pnt2d> thePnts2d, double theTolU, double theTolV,
                         double theUMin, double theVMin, double theUMax, double theVMax)
    {
        init(NCollection_Sequence<gp_Pnt2d>.Lower(), thePnts2d.Length(), i => thePnts2d.Value(i), theTolU, theTolV, theUMin, theVMin, theUMax, theVMax);
    }

    /// <summary>Constructs a 2D classifier from a vector of polygon vertices (see the array constructor).</summary>
    public CSLib_Class2d(NCollection_DynamicArray<gp_Pnt2d> thePnts2d, double theTolU, double theTolV,
                         double theUMin, double theVMin, double theUMax, double theVMax)
    {
        init(thePnts2d.Lower(), thePnts2d.Length(), i => thePnts2d.Value(i), theTolU, theTolV, theUMin, theVMin, theUMax, theVMax);
    }

    // Transforms a coordinate from original space to normalized [0,1] space,
    // or returns the original value if the range is too small.
    private static double transformToNormalized(double theU, double theUMin, double theURange)
    {
        const double THE_MIN_RANGE = 1e-10;
        if (theURange > THE_MIN_RANGE)
        {
            return (theU - theUMin) / theURange;
        }
        return theU;
    }

    // Note: OCCT's template over the container type: the container is given by its lower bound, length and accessor.
    private void init(int aLower, int aLength, Func<int, gp_Pnt2d> thePnts2d,
                      double theTolU, double theTolV, double theUMin, double theVMin, double theUMax, double theVMax)
    {
        myUMin = theUMin;
        myVMin = theVMin;
        myUMax = theUMax;
        myVMax = theVMax;

        // Validate input parameters.
        if (theUMax <= theUMin || theVMax <= theVMin || aLength < 3)
        {
            myPointsCount = 0;
            return;
        }

        myPointsCount = aLength;
        myTolU        = theTolU;
        myTolV        = theTolV;

        // Allocate arrays with one extra element for closing the polygon.
        myPnts2dX.Resize(0, myPointsCount, false);
        myPnts2dY.Resize(0, myPointsCount, false);

        double aDu = theUMax - theUMin;
        double aDv = theVMax - theVMin;

        // Transform points to normalized coordinates.
        for (int i = 0; i < myPointsCount; ++i)
        {
            gp_Pnt2d aP2D = thePnts2d(i + aLower);
            myPnts2dX[i]  = transformToNormalized(aP2D.X(), theUMin, aDu);
            myPnts2dY[i]  = transformToNormalized(aP2D.Y(), theVMin, aDv);
        }

        // Close the polygon by copying first point to last position.
        myPnts2dX.ChangeLast() = myPnts2dX.First();
        myPnts2dY.ChangeLast() = myPnts2dY.First();

        // Normalize tolerances.
        const double THE_MIN_RANGE = 1e-10;
        if (aDu > THE_MIN_RANGE)
        {
            myTolU /= aDu;
        }
        if (aDv > THE_MIN_RANGE)
        {
            myTolV /= aDv;
        }
    }

    /// <summary>Classifies a point relative to the polygon, using the tolerances given at construction.</summary>
    public Result SiDans(in gp_Pnt2d thePoint)
    {
        if (myPointsCount == 0)
        {
            return Result.Result_Uncertain;
        }

        double aX = thePoint.X();
        double aY = thePoint.Y();

        // Compute tolerance in original coordinate space.
        double aTolU = myTolU * (myUMax - myUMin);
        double aTolV = myTolV * (myVMax - myVMin);

        // Quick rejection test for points clearly outside the bounding box.
        if (aX < (myUMin - aTolU) || aX > (myUMax + aTolU) || aY < (myVMin - aTolV)
            || aY > (myVMax + aTolV))
        {
            return Result.Result_Outside;
        }

        // Transform to normalized coordinates.
        aX = transformToNormalized(aX, myUMin, myUMax - myUMin);
        aY = transformToNormalized(aY, myVMin, myVMax - myVMin);

        // Perform classification with ON detection.
        Result aResult = internalSiDansOuOn(aX, aY);
        if (aResult == Result.Result_Uncertain)
        {
            return Result.Result_Uncertain; // ON boundary
        }

        // Check corner points with tolerance for boundary detection.
        if (myTolU > 0.0 || myTolV > 0.0)
        {
            bool isInside = (aResult == Result.Result_Inside);
            if (isInside != internalSiDans(aX - myTolU, aY - myTolV)
                || isInside != internalSiDans(aX + myTolU, aY - myTolV)
                || isInside != internalSiDans(aX - myTolU, aY + myTolV)
                || isInside != internalSiDans(aX + myTolU, aY + myTolV))
            {
                return Result.Result_Uncertain; // Near boundary
            }
        }

        return aResult;
    }

    /// <summary>Classifies a point relative to the polygon, with <paramref name="theTol"/> as the ON tolerance.</summary>
    public Result SiDans_OnMode(in gp_Pnt2d thePoint, double theTol)
    {
        if (myPointsCount == 0)
        {
            return Result.Result_Uncertain;
        }

        double aX = thePoint.X();
        double aY = thePoint.Y();

        // Quick rejection test.
        if (aX < (myUMin - theTol) || aX > (myUMax + theTol) || aY < (myVMin - theTol)
            || aY > (myVMax + theTol))
        {
            return Result.Result_Outside;
        }

        // Transform to normalized coordinates.
        aX = transformToNormalized(aX, myUMin, myUMax - myUMin);
        aY = transformToNormalized(aY, myVMin, myVMax - myVMin);

        // Perform classification with ON detection.
        Result aResult = internalSiDansOuOn(aX, aY);

        // Check corner points with tolerance.
        if (theTol > 0.0)
        {
            bool isInside = (aResult == Result.Result_Inside);
            if (isInside != internalSiDans(aX - theTol, aY - theTol)
                || isInside != internalSiDans(aX + theTol, aY - theTol)
                || isInside != internalSiDans(aX - theTol, aY + theTol)
                || isInside != internalSiDans(aX + theTol, aY + theTol))
            {
                return Result.Result_Uncertain;
            }
        }

        return aResult;
    }

    // Ray-casting classification in normalized coordinates: true if inside.
    private bool internalSiDans(double thePx, double thePy)
    {
        // Count edge crossings with a horizontal ray from (Px, Py) to +infinity.
        int aNbCrossings = 0;

        double aPrevDx          = myPnts2dX.Value(0) - thePx;
        double aPrevDy          = myPnts2dY.Value(0) - thePy;
        bool   aPrevYIsNegative = (aPrevDy < 0.0);

        for (int aNextIdx = 1; aNextIdx <= myPointsCount; ++aNextIdx)
        {
            double aCurrDx          = myPnts2dX.Value(aNextIdx) - thePx;
            double aCurrDy          = myPnts2dY.Value(aNextIdx) - thePy;
            bool   aCurrYIsNegative = (aCurrDy < 0.0);

            // Check for edge crossing when Y changes sign.
            if (aCurrYIsNegative != aPrevYIsNegative)
            {
                if (aPrevDx > 0.0 && aCurrDx > 0.0)
                {
                    // Both endpoints are to the right of the test point.
                    ++aNbCrossings;
                }
                else if (aPrevDx > 0.0 || aCurrDx > 0.0)
                {
                    // Compute X intersection with horizontal line Y = 0.
                    double aXIntersect = aPrevDx - aPrevDy * (aCurrDx - aPrevDx) / (aCurrDy - aPrevDy);
                    if (aXIntersect > 0.0)
                    {
                        ++aNbCrossings;
                    }
                }
                aPrevYIsNegative = aCurrYIsNegative;
            }

            aPrevDx = aCurrDx;
            aPrevDy = aCurrDy;
        }

        // Odd number of crossings means inside.
        return (aNbCrossings & 1) != 0;
    }

    // Ray-casting classification with ON detection, in normalized coordinates.
    private Result internalSiDansOuOn(double thePx, double thePy)
    {
        int aNbCrossings = 0;

        double aPrevDx          = myPnts2dX.Value(0) - thePx;
        double aPrevDy          = myPnts2dY.Value(0) - thePy;
        bool   aPrevYIsNegative = (aPrevDy < 0.0);

        for (int aNextIdx = 1; aNextIdx <= myPointsCount; ++aNextIdx)
        {
            int    aPrevIdx = aNextIdx - 1;
            double aCurrDx  = myPnts2dX.Value(aNextIdx) - thePx;
            double aCurrDy  = myPnts2dY.Value(aNextIdx) - thePy;

            // Check if point is very close to current vertex.
            if (aCurrDx < myTolU && aCurrDx > -myTolU && aCurrDy < myTolV && aCurrDy > -myTolV)
            {
                return Result.Result_Uncertain; // ON boundary (at vertex)
            }

            // Check if point is ON the edge by computing Y at the test point's X.
            // Skip interpolation for nearly vertical edges to avoid division instability.
            // For vertical edges, the ON detection is handled by the tolerance check above.
            double aEdgeDx = myPnts2dX.Value(aNextIdx) - myPnts2dX.Value(aPrevIdx);
            if ((myPnts2dX.Value(aPrevIdx) - thePx) * aCurrDx < 0.0
                && Math.Abs(aEdgeDx) > Precision.PConfusion())
            {
                double aInterpY =
                  myPnts2dY.Value(aNextIdx)
                  - (myPnts2dY.Value(aNextIdx) - myPnts2dY.Value(aPrevIdx)) / aEdgeDx * aCurrDx;
                double aDeltaY = aInterpY - thePy;
                if (aDeltaY >= -myTolV && aDeltaY <= myTolV)
                {
                    return Result.Result_Uncertain; // ON boundary (on edge)
                }
            }

            bool aCurrYIsNegative = (aCurrDy < 0.0);
            if (aCurrYIsNegative != aPrevYIsNegative)
            {
                if (aPrevDx > 0.0 && aCurrDx > 0.0)
                {
                    ++aNbCrossings;
                }
                else if (aPrevDx > 0.0 || aCurrDx > 0.0)
                {
                    double aXIntersect = aPrevDx - aPrevDy * (aCurrDx - aPrevDx) / (aCurrDy - aPrevDy);
                    if (aXIntersect > 0.0)
                    {
                        ++aNbCrossings;
                    }
                }
                aPrevYIsNegative = aCurrYIsNegative;
            }

            aPrevDx = aCurrDx;
            aPrevDy = aCurrDy;
        }

        // Odd number of crossings means inside.
        return ((aNbCrossings & 1) != 0) ? Result.Result_Inside : Result.Result_Outside;
    }
}
