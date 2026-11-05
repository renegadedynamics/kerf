// Port of: src/ModelingData/TKG3d/Geom/Geom_BoundedSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BoundedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A surface defined on a rectangle of its parametric space, between finite first and last parameters in u and
/// v, and limited by its four boundary iso-curves (Bezier and B-spline surfaces, rectangular trimmed surfaces).
/// </summary>
public abstract class BoundedSurface : Surface
{
    /// <summary>Only the kernel derives from it (unit 10L1).</summary>
    private protected BoundedSurface()
    {
    }

}
