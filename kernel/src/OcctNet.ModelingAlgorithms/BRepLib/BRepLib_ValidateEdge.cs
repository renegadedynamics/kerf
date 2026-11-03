// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ValidateEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ValidateEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes the max distance between a 3D curve and a curve on a surface, approximately at a finite number of
/// points (the default) or exactly through <see cref="GeomLib_CheckCurveOnSurface"/> (for SameParameter edges).
/// </summary>
/// <remarks>A working object; it keeps the caller's adaptors.</remarks>
public class BRepLib_ValidateEdge
{
    private readonly Adaptor3d_Curve myReferenceCurve;
    private readonly Adaptor3d_CurveOnSurface myOtherCurve;
    private readonly bool mySameParameter;
    private int myControlPointsNumber;
    private double myToleranceForChecking;
    private double myCalculatedDistance;
    private bool myExitIfToleranceExceeded;
    private bool myIsDone;
    private bool myIsExactMethod;
    private bool myIsMultiThread;

    public BRepLib_ValidateEdge(Adaptor3d_Curve theReferenceCurve,
                                Adaptor3d_CurveOnSurface theOtherCurve,
                                bool theSameParameter)
    {
        myReferenceCurve = theReferenceCurve;
        myOtherCurve = theOtherCurve;
        mySameParameter = theSameParameter;
        myControlPointsNumber = 22;
        myToleranceForChecking = 0;
        myCalculatedDistance = 0;
        myExitIfToleranceExceeded = false;
        myIsDone = false;
        myIsExactMethod = false;
        myIsMultiThread = false;
    }

    /// <summary>
    /// Selects the exact method (BRepLib_CheckCurveOnSurface's, slower) instead of a finite number of points. The
    /// exact method is used only for SameParameter edges.
    /// </summary>
    public void SetExactMethod(bool theIsExact) => myIsExactMethod = theIsExact;

    public bool IsExactMethod() => myIsExactMethod;

    public void SetParallel(bool theIsMultiThread) => myIsMultiThread = theIsMultiThread;

    public bool IsParallel() => myIsMultiThread;

    /// <summary>Sets the number of control points (22 by default).</summary>
    public void SetControlPointsNumber(int theControlPointsNumber) => myControlPointsNumber = theControlPointsNumber;

    /// <summary>
    /// Stops <see cref="Process"/> as soon as a distance greater than the tolerance (with a margin) is found. No effect
    /// with the exact method.
    /// </summary>
    public void SetExitIfToleranceExceeded(double theToleranceForChecking)
    {
        myExitIfToleranceExceeded = true;
        myToleranceForChecking = correctTolerance(theToleranceForChecking);
    }

    /// <summary>Computes the max distance between the curves.</summary>
    public void Process()
    {
        if (myIsExactMethod && mySameParameter)
        {
            processExact();
        }
        else
        {
            processApprox();
        }
    }

    /// <summary>True if the distance has been found at all points.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>True if the computed distance is less than the tolerance (with a margin).</summary>
    public bool CheckTolerance(double theToleranceToCheck)
        => correctTolerance(theToleranceToCheck) > myCalculatedDistance;

    /// <summary>The max distance, increased by a relative 1e-5.</summary>
    public double GetMaxDistance()
    {
        double aCorrectedTolerance = myCalculatedDistance * 1.00001;
        return aCorrectedTolerance;
    }

    /// <summary>Increases <paramref name="theToleranceToUpdate"/> to the max distance (increased by 1e-5) if smaller.</summary>
    public void UpdateTolerance(ref double theToleranceToUpdate)
    {
        double aCorrectedTolerance = myCalculatedDistance * 1.00001;
        if (aCorrectedTolerance > theToleranceToUpdate)
        {
            theToleranceToUpdate = aCorrectedTolerance;
        }
    }

    /// <summary>Adds the precision of the curves' numbers to the tolerance.</summary>
    private double correctTolerance(double theTolerance)
    {
        Adaptor3d_Surface aSurface = myOtherCurve.GetSurface()!;
        double aCurvePrecision = BRepCheck.PrecCurve(myReferenceCurve);
        double aSurfacePrecision = BRepCheck.PrecSurface(aSurface);
        double aToleranceDelta = (aCurvePrecision > aSurfacePrecision) ? aCurvePrecision : aSurfacePrecision;
        double aCorrectedTolerance = theTolerance + aToleranceDelta;
        return aCorrectedTolerance;
    }

    /// <summary>The distance at a finite number of points.</summary>
    private void processApprox()
    {
        myIsDone = true;
        double aSquareToleranceForChecking = myToleranceForChecking * myToleranceForChecking;
        double aReferenceFirstParam = myReferenceCurve.FirstParameter();
        double aReferenceLastParam = myReferenceCurve.LastParameter();
        double anOtherFirstParam = myOtherCurve.FirstParameter();
        double anOtherLastParam = myOtherCurve.LastParameter();
        double aMaxSquareDistance = 0.0;

        int aControlPointsNumber = (myControlPointsNumber < 1) ? 1 : myControlPointsNumber;
        bool anIsProjection =
            (!mySameParameter
             || Math.Abs(anOtherFirstParam - aReferenceFirstParam) > global::Kerf.Engine.Foundation.Precision.PConfusion()
             || Math.Abs(anOtherLastParam - aReferenceLastParam) > global::Kerf.Engine.Foundation.Precision.PConfusion());

        if (!anIsProjection)
        {
            for (int i = 0; i <= aControlPointsNumber; ++i)
            {
                double aControlPointParam =
                    ((aControlPointsNumber - i) * aReferenceFirstParam + i * aReferenceLastParam) / aControlPointsNumber;
                Point3D aReferencePoint = myReferenceCurve.Value(aControlPointParam);
                Point3D anOtherPoint = myOtherCurve.Value(aControlPointParam);
                double aSquareDistance = aReferencePoint.SquareDistance(anOtherPoint);
                if (aSquareDistance > aMaxSquareDistance)
                {
                    aMaxSquareDistance = aSquareDistance;
                }

                // Stop process for best performance
                if (myExitIfToleranceExceeded && aMaxSquareDistance > aSquareToleranceForChecking)
                {
                    myCalculatedDistance = Math.Sqrt(aMaxSquareDistance);
                    return;
                }
            }
        }
        else
        {
            Point3D aReferencePoint = myReferenceCurve.Value(aReferenceFirstParam);
            Point3D anOtherPoint = myOtherCurve.Value(anOtherFirstParam);
            double aSquareDistance = aReferencePoint.SquareDistance(anOtherPoint);
            if (aSquareDistance > aMaxSquareDistance)
            {
                aMaxSquareDistance = aSquareDistance;
            }

            if (myExitIfToleranceExceeded && aMaxSquareDistance > aSquareToleranceForChecking)
            {
                myCalculatedDistance = Math.Sqrt(aMaxSquareDistance);
                return;
            }

            aReferencePoint = myReferenceCurve.Value(aReferenceLastParam);
            anOtherPoint = myOtherCurve.Value(anOtherLastParam);
            aSquareDistance = aReferencePoint.SquareDistance(anOtherPoint);
            if (aSquareDistance > aMaxSquareDistance)
            {
                aMaxSquareDistance = aSquareDistance;
            }

            if (myExitIfToleranceExceeded && aMaxSquareDistance > aSquareToleranceForChecking)
            {
                myCalculatedDistance = Math.Sqrt(aMaxSquareDistance);
                return;
            }

            var aReferenceExtrema = new Extrema_LocateExtPC();
            var anOtherExtrema = new Extrema_LocateExtPC();
            aReferenceExtrema.Initialize(myReferenceCurve,
                                         aReferenceFirstParam,
                                         aReferenceLastParam,
                                         myReferenceCurve.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
            anOtherExtrema.Initialize(myOtherCurve,
                                      anOtherFirstParam,
                                      anOtherLastParam,
                                      myOtherCurve.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
            for (int i = 1; i < aControlPointsNumber; i++)
            {
                double aReferenceParam =
                    ((aControlPointsNumber - i) * aReferenceFirstParam + i * aReferenceLastParam) / aControlPointsNumber;
                Point3D aReferenceExtremaPoint = myReferenceCurve.Value(aReferenceParam);
                double anOtherParam =
                    ((aControlPointsNumber - i) * anOtherFirstParam + i * anOtherLastParam) / aControlPointsNumber;
                Point3D anOtherExtremaPoint = myOtherCurve.Value(anOtherParam);

                aReferenceExtrema.Perform(anOtherExtremaPoint, aReferenceParam);
                if (aReferenceExtrema.IsDone())
                {
                    if (aReferenceExtrema.SquareDistance() > aMaxSquareDistance)
                    {
                        aMaxSquareDistance = aReferenceExtrema.SquareDistance();
                    }

                    if (myExitIfToleranceExceeded && aMaxSquareDistance > aSquareToleranceForChecking)
                    {
                        myCalculatedDistance = Math.Sqrt(aMaxSquareDistance);
                        return;
                    }
                }
                else
                {
                    myIsDone = false;
                    // Stop process for best performance
                    return;
                }

                anOtherExtrema.Perform(aReferenceExtremaPoint, anOtherParam);
                if (anOtherExtrema.IsDone())
                {
                    if (anOtherExtrema.SquareDistance() > aMaxSquareDistance)
                    {
                        aMaxSquareDistance = anOtherExtrema.SquareDistance();
                    }

                    if (myExitIfToleranceExceeded && aMaxSquareDistance > aSquareToleranceForChecking)
                    {
                        myCalculatedDistance = Math.Sqrt(aMaxSquareDistance);
                        return;
                    }
                }
                else
                {
                    myIsDone = false;
                    // Stop process for best performance
                    return;
                }
            }
        }

        myCalculatedDistance = Math.Sqrt(aMaxSquareDistance);
    }

    /// <summary>The exact distance, by GeomLib_CheckCurveOnSurface.</summary>
    private void processExact()
    {
        var aCheckCurveOnSurface = new GeomLib_CheckCurveOnSurface(myReferenceCurve);
        aCheckCurveOnSurface.SetParallel(myIsMultiThread);
        aCheckCurveOnSurface.Perform(myOtherCurve);
        myIsDone = aCheckCurveOnSurface.IsDone();
        if (myIsDone)
        {
            myCalculatedDistance = aCheckCurveOnSurface.MaxDistance();
        }
    }
}
