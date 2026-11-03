// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Circle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-circle extrema: the point is projected onto the circle plane and the angle of the projection gives the
/// closest point (minimum) and the opposite one (maximum).
/// </summary>
/// <remarks>When the point projects onto the centre, every point of the circle is at the same distance:
/// Status.InfiniteSolutions with InfiniteSquareDistance = R² + h². The domain is fixed at construction; a full
/// period means the whole circle.</remarks>
public sealed class ExtremaPC_Circle : ExtremaPC.ICurveEvaluator
{
    private readonly Circle3D myCircle;                      // Circle geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? myDomain;          // Parameter domain (null for full circle)
    private readonly ExtremaPC.Result myResult = new();     // Reusable result storage

    /// <summary>Constructor with circle geometry (full circle).</summary>
    public ExtremaPC_Circle(in Circle3D theCircle)
    {
        myCircle = theCircle;
        myDomain = null;
    }

    /// <summary>Constructor with circle geometry and parameter domain in radians (fixed for all queries).</summary>
    public ExtremaPC_Circle(in Circle3D theCircle, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myCircle = theCircle;
        myDomain = theDomain.IsFullPeriod(2.0 * M_PI) ? null : theDomain;
    }

    /// <summary>Evaluates point on circle at parameter (radians).</summary>
    public Point3D Value(double theU) => global::Kerf.Engine.Geometry.ElCLib.Value(theU, myCircle);

    /// <summary>Returns true if domain is bounded (partial arc).</summary>
    public bool IsBounded() => myDomain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => myDomain!.Value;

    /// <summary>Computes the extrema between point P and the circle.</summary>
    public ExtremaPC.Result Perform(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        myResult.Clear();

        // Step 1: Project point P onto the circle plane
        Point3D aCenter = myCircle.Location();
        Direction3D aAxis = myCircle.Axis().Direction();
        Vector3D aToP = new(aCenter, theP);
        double aHeight = aToP.Dot(new Vector3D(aAxis));
        Vector3D aTrsl = new Vector3D(aAxis) * (-aHeight);
        Point3D aPp = theP.Translated(aTrsl);

        // Step 2: Check for degenerate case - point projects to center
        Vector3D aOPp = new(aCenter, aPp);
        double aOPpMag = aOPp.Magnitude();

        if (aOPpMag < theTol)
        {
            // Point is on the circle axis - all points on circle are equidistant
            myResult.Status = ExtremaPC.Status.InfiniteSolutions;
            double aRadius = myCircle.Radius();
            myResult.InfiniteSquareDistance = aRadius * aRadius + aHeight * aHeight;
            return myResult;
        }

        // Step 3: Compute the angle of the closest point
        // Us1 corresponds to minimum distance (closest point)
        double aUs1 = myCircle.XAxis().Direction().AngleWithRef(new Direction3D(aOPp), aAxis);

        // Handle angle boundaries
        double aAngTol = global::Kerf.Engine.Foundation.Precision.Angular();
        if (aUs1 + M_PI < aAngTol)
        {
            aUs1 = -M_PI;
        }
        else if (aUs1 - M_PI > -aAngTol)
        {
            aUs1 = M_PI;
        }

        // Us2 = Us1 + PI corresponds to maximum distance (farthest point)
        double aUs2 = aUs1 + M_PI;

        // Step 4: For bounded case, adjust for periodicity
        double aTolU = global::Kerf.Engine.Foundation.Precision.Angular();
        if (myDomain.HasValue)
        {
            double theUMin = myDomain.Value.Min;

            double aRadius = myCircle.Radius();
            if (aRadius > Constants.Resolution())
            {
                aTolU = theTol / aRadius;
            }

            // Adjust angles to be within [theUMin, theUMin + 2*PI]
            double aUinf = theUMin;
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(theUMin, theUMin + 2.0 * M_PI, aTolU, ref aUinf, ref aUs1);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(theUMin, theUMin + 2.0 * M_PI, aTolU, ref aUinf, ref aUs2);

            // Handle boundary tolerance
            if (Math.Abs(aUs1 - 2.0 * M_PI - theUMin) < aTolU)
            {
                aUs1 = theUMin;
            }
            if (Math.Abs(aUs2 - 2.0 * M_PI - theUMin) < aTolU)
            {
                aUs2 = theUMin;
            }
        }

        // Step 5: Add extrema (with bounds check if domain specified)
        // Skip based on search mode: i=0 is minimum, i=1 is maximum
        int aStart = (theMode == ExtremaPC.SearchMode.Max) ? 1 : 0;
        int aEnd = (theMode == ExtremaPC.SearchMode.Min) ? 1 : 2;

        Span<double> aSolutions = [aUs1, aUs2];

        for (int i = aStart; i < aEnd; ++i)
        {
            double aU = aSolutions[i];

            // Check bounds only if domain is specified
            if (myDomain.HasValue)
            {
                if (aU < myDomain.Value.Min - aTolU || aU > myDomain.Value.Max + aTolU)
                    continue;
            }

            Point3D aCurvePt = global::Kerf.Engine.Geometry.ElCLib.Value(aU, myCircle);

            ExtremaPC.ExtremumResult anExt = new();
            anExt.Parameter = aU;
            anExt.Point = aCurvePt;
            anExt.SquareDistance = theP.SquareDistance(aCurvePt);
            anExt.IsMinimum = (i == 0); // First solution is minimum, second is maximum

            myResult.Extrema.Append(anExt);
        }

        myResult.Status = ExtremaPC.Status.OK;
        return myResult;
    }

    /// <summary>Computes the extrema between point P and the circle arc including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        _ = Perform(theP, theTol, theMode);

        // Add endpoints if interior computation succeeded and domain is bounded
        if (myResult.Status == ExtremaPC.Status.OK && myDomain.HasValue)
        {
            ExtremaPC.AddEndpointExtrema(myResult, theP, myDomain.Value, this, theTol, theMode);
        }

        return myResult;
    }

    /// <summary>Returns the circle geometry.</summary>
    public Circle3D Circle() => myCircle;
}
