// Port of: src/ModelingData/TKG3d/Geom/Geom_BoundedCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BoundedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// A curve limited by two finite parameters, the first giving its start point and the last its end point
/// (Bezier and B-spline curves, trimmed curves). Its length is finite.
/// </summary>
public abstract class BoundedCurve3D : Curve3D
{
    /// <summary>Only the kernel derives from it.</summary>
    private protected BoundedCurve3D()
    {
    }

    /// <summary>The end point of the curve.</summary>
    public abstract Point3D EndPoint { get; }

    /// <summary>The start point of the curve.</summary>
    public abstract Point3D StartPoint { get; }
}
