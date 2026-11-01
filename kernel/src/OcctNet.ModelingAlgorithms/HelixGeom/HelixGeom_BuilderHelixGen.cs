// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixGen.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixGen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The root of the helix builders: the helix's parameters.</summary>
public abstract class HelixGeom_BuilderHelixGen : HelixGeom_BuilderApproxCurve
{
    protected double myT1;
    protected double myT2;
    protected double myPitch;
    protected double myRStart;
    protected double myTaperAngle;
    protected bool myIsClockWise;

    /// <summary>Sets the default parameters: one turn from 0, pitch 1, radius 1, no taper, clockwise.</summary>
    protected HelixGeom_BuilderHelixGen()
    {
        myT1 = 0.0;
        myT2 = 2.0 * M_PI;
        myPitch = 1.0;
        myRStart = 1.0;
        myTaperAngle = 0.0;
        myIsClockWise = true;
    }

    /// <summary>
    /// Sets the parameters of the helix: the parameter range (angles in radians), the pitch, the radius at the start,
    /// the taper angle and the direction. They are checked when the curve is built.
    /// </summary>
    public void SetCurveParameters(double aT1, double aT2, double aPitch, double aRStart, double aTaperAngle,
                                   bool aIsCW)
    {
        // Input validation would typically be done in derived classes or Load methods
        // Base class just stores the values
        myT1 = aT1;
        myT2 = aT2;
        myPitch = aPitch;
        myRStart = aRStart;
        myTaperAngle = aTaperAngle;
        myIsClockWise = aIsCW;
    }

    /// <summary>The parameters of the helix.</summary>
    public void CurveParameters(out double aT1, out double aT2, out double aPitch, out double aRStart,
                                out double aTaperAngle, out bool aIsClockWise)
    {
        aT1 = myT1;
        aT2 = myT2;
        aPitch = myPitch;
        aRStart = myRStart;
        aTaperAngle = myTaperAngle;
        aIsClockWise = myIsClockWise;
    }
}
