// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PROJECT.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PROJECT.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The legacy Booleans' projection functions (<c>FUN_tool_*</c>).</summary>
/// <remarks>
/// OCCT's free functions, as a static class named after the file. Only <c>FUN_tool_bounds</c> (for an edge) is ported,
/// for the face maker of the 2D offset (plan 2.11); the projections of points on curves and surfaces are left out.
/// </remarks>
public static class TopOpeBRepTool_PROJECT
{
    /// <summary>The parameter range of the edge's curve.</summary>
    public static void FUN_tool_bounds(in TopoDS_Edge E, out double f, out double l)
    {
        BRepAdaptor_Curve BAC = new(E);
        f = BAC.FirstParameter();
        l = BAC.LastParameter();
    }

    // FUN_tool_getindex, FUN_tool_projPonC, FUN_tool_projPonC2D, FUN_tool_projPonS, FUN_tool_projPonE,
    // FUN_tool_projPonboundedF, FUN_tool_projPonF: not ported (plan 2.11).
}
