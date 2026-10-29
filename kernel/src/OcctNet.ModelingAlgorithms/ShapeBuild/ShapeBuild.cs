// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Tools for building shapes in Shape Healing.</summary>
public static class ShapeBuild
{
    private static Geom_Plane? xoy;

    /// <summary>The plane XOY (Z = 0): one object, frozen, shared by every call.</summary>
    public static Geom_Plane PlaneXOY()
    {
        if (xoy is null)
        {
            var aPlane = new Geom_Plane(0, 0, 1, 0);
            aPlane.Freeze();
            Interlocked.CompareExchange(ref xoy, aPlane, null);
        }

        return xoy;
    }
}
