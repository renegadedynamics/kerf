// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelix.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds a helix about an arbitrary axis, approximated by B-spline curves, one per full turn and one for the rest.
/// </summary>
public sealed class HelixGeom_BuilderHelix : HelixGeom_BuilderHelixGen
{
    private Frame3D myPosition = new();

    /// <summary>A builder with the default parameters, about the global Z axis.</summary>
    public HelixGeom_BuilderHelix()
    {
    }

    /// <summary>Sets the coordinate system of the helix (its Z axis is the helix's axis).</summary>
    public void SetPosition(in Frame3D aAx2)
    {
        myPosition = aAx2;
    }

    /// <summary>The coordinate system of the helix.</summary>
    public Frame3D Position() => myPosition;

    /// <summary>
    /// Builds the curves. <see cref="HelixGeom_BuilderApproxCurve.ErrorStatus"/> is 2 if an approximation fails.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The helix parameters are invalid.</exception>
    public override void Perform()
    {
        myErrorStatus = 0;
        myWarningStatus = 0;

        // Initialize variables for helix segmentation
        int iErr, aN, i, aNbC;
        double aTwoPI, dT, aT1x, aT2x, aTR;
        HelixGeom_BuilderHelixCoil aBHC = new();

        // Initialize result containers
        myCurves.Clear();
        myTolReached = -1.0;
        aTwoPI = 2.0 * M_PI;
        aBHC.SetTolerance(myTolerance);
        aBHC.SetApproxParameters(myCont, myMaxDegree, myMaxSeg);

        // Determine number of full turns for segmentation
        dT = myT2 - myT1;
        aN = (int)(dT / aTwoPI);
        if (aN == 0)
        {
            aBHC.SetCurveParameters(myT1, myT2, myPitch, myRStart, myTaperAngle, myIsClockWise);
            aBHC.Perform();
            iErr = aBHC.ErrorStatus();
            if (iErr != 0)
            {
                myErrorStatus = 2;
                return;
            }
            NCollection_Sequence<Geom_Curve> aSC = aBHC.InternalCurves();
            Geom_Curve aC = aSC[1];
            myCurves.Append(aC);
            myTolReached = aBHC.ToleranceReached();
        }
        else
        {
            // Case: helix spans multiple full turns - process in segments
            bool bIsCylindrical;
            double aTolAngle;

            aTolAngle = 1.0e-4;
            bIsCylindrical = Math.Abs(myTaperAngle) < aTolAngle;
            aT1x = myT1;
            aT2x = myT1 + aTwoPI;
            for (i = 1; i <= aN; ++i)
            {
                if (i > 1 && bIsCylindrical)
                {
                    // Optimization: for cylindrical helixes, reuse first coil with translation
                    Geom_Curve aCi;
                    Point3D aP1, aPi = new();

                    Geom_Curve aC1 = myCurves[1];
                    aC1.D0(aC1.FirstParameter(), out aP1);
                    aPi.SetCoord(aP1.X(), aP1.Y(), aP1.Z() + (i - 1) * myPitch);
                    aCi = (Geom_Curve)aC1.Translated(aP1, aPi);

                    myCurves.Append(aCi);
                    aT1x = aT2x;
                    aT2x = aT1x + aTwoPI;
                    // Skip to next iteration for optimization
                    continue;
                }

                aBHC.SetCurveParameters(aT1x, aT2x, myPitch, myRStart, myTaperAngle, myIsClockWise);
                // Perform approximation for this segment
                aBHC.Perform();
                iErr = aBHC.ErrorStatus();
                if (iErr != 0)
                {
                    myErrorStatus = 2;
                    return;
                }
                // Extract approximated curves from builder
                NCollection_Sequence<Geom_Curve> aSC = aBHC.InternalCurves();
                Geom_Curve aC = aSC[1];
                myCurves.Append(aC);
                aTR = aBHC.ToleranceReached();
                if (aTR > myTolReached)
                {
                    myTolReached = aTR;
                }
                // Move to next segment parameters
                aT1x = aT2x;
                aT2x = aT1x + aTwoPI;
            } // for (i=1; i<=aN; ++i) {
            // Handle remaining partial turn if any
            aT2x = myT2;
            double eps = 1.0e-7 * aTwoPI;
            if (Math.Abs(aT2x - aT1x) > eps)
            {
                aBHC.SetCurveParameters(aT1x, aT2x, myPitch, myRStart, myTaperAngle, myIsClockWise);
                aBHC.Perform();
                iErr = aBHC.ErrorStatus();
                if (iErr != 0)
                {
                    myErrorStatus = 2;
                    return;
                }
                // Extract curves from the final partial segment
                NCollection_Sequence<Geom_Curve> aSC = aBHC.InternalCurves();
                Geom_Curve aC = aSC[1];
                myCurves.Append(aC);
                aTR = aBHC.ToleranceReached();
                if (aTR > myTolReached)
                {
                    myTolReached = aTR;
                }
            }
        }
        // Apply coordinate system transformation to all curves
        // Transformation
        Transform3D aTrsf = new();
        CoordinateSystem aAx3 = new(), aAx3x = new(myPosition);
        // Set transformation from standard coordinate system to helix position
        aTrsf.SetDisplacement(aAx3, aAx3x);
        // Apply transformation to all generated curves (new and unfrozen: the coil builder's own, or translated copies)
        aNbC = myCurves.Length();
        for (i = 1; i <= aNbC; ++i)
        {
            Geom_Curve aC = myCurves[i];
            aC.Transform(aTrsf);
        }
    }
}
