// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Healing.Build;

/// <summary>Tools for building shapes in Shape Healing.</summary>
public static class ShapeBuildBuilder
{
    private static PlaneSurface? s_xoy;

    /// <summary>The plane XOY (Z = 0): one object, frozen, shared by every call.</summary>
    public static PlaneSurface PlaneXOY()
    {
        if (s_xoy is null)
        {
            var plane = new PlaneSurface(0, 0, 1, 0);
            plane.Freeze();
            Interlocked.CompareExchange(ref s_xoy, plane, null);
        }

        return s_xoy;
    }
}
