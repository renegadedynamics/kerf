// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_AddSurface.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_AddSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// Adds the bounding box of a surface to a box (through <see cref="SurfaceBounds"/>). The box is enlarged by
/// the tolerance; an infinite surface opens it.
/// </summary>
internal static class AddSurface
{
    /// <summary>Adds the bounding box of the surface S, over its own bounds, enlarged by Tol, to B.</summary>
    public static void Add(SurfaceAdaptorBase s, double tol, ref Box b)
    {
        Add(s, s.FirstUParameter(), s.LastUParameter(), s.FirstVParameter(), s.LastVParameter(), tol, ref b);
    }

    /// <summary>Adds the bounding box of the patch [UMin, UMax] x [VMin, VMax] of S, enlarged by Tol, to B.</summary>
    public static void Add(SurfaceAdaptorBase s, double uMin, double uMax, double vMin, double vMax, double tol, ref Box b)
    {
        new SurfaceBounds(s).Add(uMin, uMax, vMin, vMax, tol, ref b);
    }

    /// <summary>Adds the precise bounding box of the surface S to B (numerical search of the extrema; slower).</summary>
    public static void AddOptimal(SurfaceAdaptorBase s, double tol, ref Box b)
    {
        AddOptimal(s, s.FirstUParameter(), s.LastUParameter(), s.FirstVParameter(), s.LastVParameter(), tol, ref b);
    }

    /// <summary>Adds the precise bounding box of the patch [UMin, UMax] x [VMin, VMax] of S to B.</summary>
    public static void AddOptimal(SurfaceAdaptorBase s, double uMin, double uMax, double vMin, double vMax, double tol, ref Box b)
    {
        new SurfaceBounds(s).AddOptimal(uMin, uMax, vMin, vMax, tol, ref b);
    }
}
