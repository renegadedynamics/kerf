// Port of: src/ModelingData/TKG3d/Geom/Geom_BoundedCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BoundedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A curve limited by two finite parameters, the first giving its start point and the last its end point
/// (Bezier and B-spline curves, trimmed curves). Its length is finite.
/// </summary>
public abstract class Geom_BoundedCurve : Geom_Curve
{
    /// <summary>The end point of the curve.</summary>
    public abstract gp_Pnt EndPoint();

    /// <summary>The start point of the curve.</summary>
    public abstract gp_Pnt StartPoint();
}
