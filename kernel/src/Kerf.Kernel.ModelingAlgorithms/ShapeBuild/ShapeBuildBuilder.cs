// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Healing.Build;

/// <summary>Tools for building shapes in Shape Healing.</summary>
internal static class ShapeBuildBuilder
{
    private static readonly PlaneSurface Xoy = MakeXoy();

    private static PlaneSurface MakeXoy()
    {
        var plane = new PlaneSurface(0, 0, 1, 0);
        plane.Freeze();
        return plane;
    }

    /// <summary>The plane XOY (Z = 0): one object, frozen, shared by every call.</summary>
    public static PlaneSurface PlaneXOY() => Xoy;
}
