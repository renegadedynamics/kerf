// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixGen.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixGen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Helix.Geometry;

/// <summary>The root of the helix builders: the helix's parameters.</summary>
public abstract class BuilderHelixGen : BuilderApproxCurve
{
    protected double _t1;
    protected double _t2;
    protected double _pitch;
    protected double _rStart;
    protected double _taperAngle;
    protected bool _isClockWise;

    /// <summary>Sets the default parameters: one turn from 0, pitch 1, radius 1, no taper, clockwise.</summary>
    protected BuilderHelixGen()
    {
        _t1 = 0.0;
        _t2 = 2.0 * M_PI;
        _pitch = 1.0;
        _rStart = 1.0;
        _taperAngle = 0.0;
        _isClockWise = true;
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
        _t1 = aT1;
        _t2 = aT2;
        _pitch = aPitch;
        _rStart = aRStart;
        _taperAngle = aTaperAngle;
        _isClockWise = aIsCW;
    }

    /// <summary>The parameters of the helix.</summary>
    public void CurveParameters(out double aT1, out double aT2, out double aPitch, out double aRStart,
                                out double aTaperAngle, out bool aIsClockWise)
    {
        aT1 = _t1;
        aT2 = _t2;
        aPitch = _pitch;
        aRStart = _rStart;
        aTaperAngle = _taperAngle;
        aIsClockWise = _isClockWise;
    }
}
