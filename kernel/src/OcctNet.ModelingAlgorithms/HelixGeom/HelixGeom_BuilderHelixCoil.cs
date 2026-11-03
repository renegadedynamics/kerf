// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixCoil.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixCoil.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Occt;

/// <summary>Builds one helix coil about the Z axis, approximated by a B-spline curve.</summary>
public sealed class HelixGeom_BuilderHelixCoil : HelixGeom_BuilderHelixGen
{
    /// <summary>A builder with the default parameters.</summary>
    public HelixGeom_BuilderHelixCoil()
    {
        myT1 = 0.0;
        myT2 = 2.0 * M_PI;
        myPitch = 1.0;
        myRStart = 1.0;
        myTaperAngle = 0.0;
        myIsClockWise = true;
    }

    /// <summary>
    /// Approximates the coil. <see cref="HelixGeom_BuilderApproxCurve.ErrorStatus"/> is 2 if the approximation fails.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The helix parameters are invalid (see <see cref="HelixGeom_HelixCurve.Load(double, double, double, double, double, bool)"/>).</exception>
    public override void Perform()
    {
        myErrorStatus = 0;
        myWarningStatus = 0;
        // Initialize variables for curve approximation
        int iErr;
        HelixGeom_HelixCurve aAdaptor = new();
        HelixGeom_HelixCurve aHAdaptor;
        BSplineCurve3D? aBC = null;
        // Clear previous results and setup helix adaptor
        myCurves.Clear();
        // Load helix parameters into the adaptor
        aAdaptor.Load(myT1, myT2, myPitch, myRStart, myTaperAngle, myIsClockWise);
        aHAdaptor = new HelixGeom_HelixCurve(aAdaptor);
        // Perform B-spline approximation of the helix curve
        iErr = HelixGeom_Tools.ApprCurve3D(aHAdaptor,
                                           myTolerance,
                                           myCont,
                                           myMaxSeg,
                                           myMaxDegree,
                                           ref aBC,
                                           ref myTolReached);
        if (iErr != 0)
        {
            myErrorStatus = 2;
        }
        else
        {
            myCurves.Append(aBC!);
        }
    }
}
