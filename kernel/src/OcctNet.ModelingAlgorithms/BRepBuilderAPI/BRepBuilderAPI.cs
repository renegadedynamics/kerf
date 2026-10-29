// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The BRepBuilderAPI package: makers of vertices, edges, wires, faces, shells and solids (over BRepLib's), and the
/// shape modifications (transformation, general transformation, copy, NURBS conversion).
/// </summary>
/// <remarks>
/// The default plane and precision are BRepLib's process-wide statics (decision 415): callers read them; only ported
/// code sets them, through the internal setters.
/// </remarks>
public static class BRepBuilderAPI
{
    /// <summary>Sets the default plane of the 2D edges (internal: BRepLib's process-wide static).</summary>
    internal static void Plane(Geom_Plane P)
    {
        BRepLib.Plane(P);
    }

    /// <summary>The default plane of the 2D edges (frozen).</summary>
    public static Geom_Plane Plane()
    {
        return BRepLib.Plane();
    }

    /// <summary>Sets the default precision (internal: BRepLib's process-wide static).</summary>
    internal static void Precision(double P)
    {
        BRepLib.Precision(P);
    }

    /// <summary>The default precision.</summary>
    public static double Precision()
    {
        return BRepLib.Precision();
    }
}
