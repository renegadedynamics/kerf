// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// The BRepBuilderAPI package: makers of vertices, edges, wires, faces, shells and solids (over BRepLib's), and the
/// shape modifications (transformation, general transformation, copy, NURBS conversion).
/// </summary>
/// <remarks>
/// The default plane and precision are BRepLib's settings of the operation context (process-wide in OCCT; decision 415
/// and unit 10I): callers read them; only ported code sets them, through the internal setters.
/// </remarks>
public static class BRepBuilderApi
{
    /// <summary>Sets the default plane of the 2D edges (internal: BRepLib's setting, for the rest of the flow).</summary>
    internal static void Plane(PlaneSurface p)
    {
        BrepLibrary.Plane(p);
    }

    /// <summary>The default plane of the 2D edges (frozen).</summary>
    public static PlaneSurface Plane()
    {
        return BrepLibrary.Plane();
    }

    /// <summary>Sets the default precision (internal: BRepLib's setting, for the rest of the flow).</summary>
    internal static void Precision(double p)
    {
        BrepLibrary.Precision(p);
    }

    /// <summary>The default precision.</summary>
    public static double Precision()
    {
        return BrepLibrary.Precision();
    }
}
